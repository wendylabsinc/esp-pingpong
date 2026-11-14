/* One board's signal chain for sim/sim.py: the firmware's own sweep detector
 * (chirp.h) and tap trigger (tap.h), driven the way firmware/main/main.c
 * drives them, with samples in place of a microphone. sim.py builds this as a
 * shared library. Keep it in step with mic_task, chirp_task and net_task.
 */
#include <stdlib.h>

#include "../firmware/main/chirp.h"
#include "../firmware/main/tap.h"

#define HIST 16384 /* as in main.c */
#define MAX_TAPS 2 /* the firmware's tap queue */

typedef struct {
    int64_t first;
    int16_t samples[TAP_SAMPLES];
} tap_t;

typedef struct {
    chirp_t c;
    tap_trig_t trig;
    int rate, ratio, min, block;
    int64_t written, base, own_base;
    tap_t queue[MAX_TAPS], tap;
    int queued, have_tap;
    int64_t tap_due;
    int16_t hist[HIST];
} board_t;

/* `start` is the sample index of the first sample fed: a board that has been
 * running for a while. Returns NULL if the sweep does not fit the detector. */
board_t *board_new(int rate, int f0, int f1, int ms, float detect_db, int ratio, int min, int64_t start)
{
    board_t *b = calloc(1, sizeof *b);
    if (!b || !chirp_init(&b->c, rate, f0, f1, ms, detect_db)) {
        free(b);
        return NULL;
    }
    b->rate = rate, b->ratio = ratio, b->min = min, b->block = rate / 50;
    b->written = start, b->base = -1, b->own_base = -1;
    b->trig = (tap_trig_t){.pending = -1, .holdoff = start + rate};
    return b;
}

int board_edges(int f0, int f1, int ms, uint16_t *half_us, int max)
{
    return chirp_edges(f0, f1, ms, half_us, max);
}

/* net_task: a sweep sets off the tap trigger too; once the detector names it,
 * it is not a tap. */
static void note_hit(board_t *b, const chirp_hit_t *hit)
{
    int64_t t_ns = (int64_t)(hit->at * 1e9 / b->rate), at = t_ns / 1000 * b->rate / 1000000;
    if (b->have_tap && at > b->tap.first - b->c.len && at < b->tap.first + TAP_SAMPLES) {
        b->have_tap = 0;
    }
}

/* Takes the next `n` samples. Writes up to `max_hits` sweeps heard as (sample
 * index, dB, 1 if our own) to `hits` and returns how many. If a tap is ready
 * to report, `*tap_first` is its first sample index and `tap_samples` its
 * TAP_SAMPLES samples; else `*tap_first` is -1. */
int board_feed(board_t *b, const int16_t *s, int n, double *hits, int max_hits, int64_t *tap_first,
               int16_t *tap_samples)
{
    static int16_t block[CHIRP_N];
    chirp_hit_t got[4];
    int out = 0, found = 0;
    *tap_first = -1;

    /* mic_task */
    for (int k = 0; k < n; k++, b->written++) {
        b->hist[b->written % HIST] = s[k];
        int64_t first = tap_feed(&b->trig, s[k], b->written, b->rate, b->ratio, b->min);
        if (first >= 0 && b->queued < MAX_TAPS) {
            tap_t *t = &b->queue[b->queued++];
            t->first = first;
            for (int i = 0; i < TAP_SAMPLES; i++) {
                t->samples[i] = b->hist[(first + i) % HIST];
            }
        }
    }

    /* net_task takes a tap up before the sweeps found below reach it */
    if (!b->have_tap && b->queued) {
        b->tap = b->queue[0];
        b->queue[0] = b->queue[1];
        b->queued--;
        b->have_tap = 1;
        b->tap_due = b->written + b->rate * 3 / 10; /* TAP_HOLD_US */
    }

    /* chirp_task */
    if (b->base < 0 || b->written - b->base > HIST - 2 * b->block) {
        b->base = b->written;
        b->own_base = -1;
    } else {
        if (b->own_base >= 0 && b->written - b->own_base >= CHIRP_N) {
            for (int i = 0; i < CHIRP_N; i++) {
                block[i] = b->hist[(b->own_base + i) % HIST];
            }
            found = chirp_own(&b->c, block, b->own_base, got);
            b->own_base = -1;
        }
        for (;; b->base += chirp_hop(&b->c)) {
            for (int i = 0; i < found; i++) {
                note_hit(b, &got[i]);
                if (out < max_hits) {
                    hits[3 * out] = got[i].at;
                    hits[3 * out + 1] = got[i].snr_db;
                    hits[3 * out + 2] = got[i].mine;
                    out++;
                }
            }
            if (b->written - b->base < CHIRP_N) {
                break;
            }
            for (int i = 0; i < CHIRP_N; i++) {
                block[i] = b->hist[(b->base + i) % HIST];
            }
            found = chirp_scan(&b->c, block, b->base, got, 4);
        }
    }

    if (b->have_tap && b->written >= b->tap_due) {
        *tap_first = b->tap.first;
        memcpy(tap_samples, b->tap.samples, sizeof b->tap.samples);
        b->have_tap = 0;
    }
    return out;
}

/* The board wants to play a sweep `delay` samples from now. Returns the sample
 * index it will start at, or -1 if someone else's sweep is still ringing. */
int64_t board_play(board_t *b, int delay)
{
    int64_t at = b->written + delay;
    if (at < b->c.quiet_until + b->rate / 10) {
        return -1;
    }
    chirp_played(&b->c, at);
    b->own_base = at - b->rate / 80;
    return at;
}
