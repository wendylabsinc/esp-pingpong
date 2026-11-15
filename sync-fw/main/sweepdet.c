/* See sweepdet.h. */
#include "sweepdet.h"

#include <math.h>
#include <string.h>

#define F0 2000.0
#define F1 6000.0
#define DUR 0.064
#define OVER 80.0f   /* a sweep: the envelope's power this many times its usual level (9 times in size) */
#define SETTLE 160   /* the strongest point counts once nothing stronger came for 20 ms */
#define HOLD 640     /* then nothing new for 80 ms from it: the sweep's own skirts and echoes */
#define RING 512

static int16_t s_tc[SD_TAPS], s_ts[SD_TAPS]; /* the sweep's cosine and sine, +-127 */
static int16_t s_hist[2 * SD_TAPS];          /* the last SD_TAPS samples, written twice so they lie in a row */
static int s_w;
static int16_t s_prev;
static float s_ci[RING], s_cq[RING];
static float s_floor;
static int s_warm;
static enum { IDLE, TRACK, HELD } s_state;
static int64_t s_peak_m, s_until;
static float s_peak;

void sd_init(void)
{
    for (int k = 0; k < SD_TAPS; k++) {
        double t = k / 16000.0, ph = 2.0 * M_PI * (F0 * t + 0.5 * (F1 - F0) / DUR * t * t);
        s_tc[k] = (int16_t)lrint(127.0 * cos(ph));
        s_ts[k] = (int16_t)lrint(127.0 * sin(ph));
    }
    memset(s_hist, 0, sizeof s_hist);
    s_w = 0;
    s_prev = 0;
    s_floor = 0;
    s_warm = 0;
    s_state = IDLE;
}

int sd_feed(const int16_t *x, int n, int64_t n0, sd_hit_t *out)
{
    int ready = 0;
    for (int i = 0; i + 1 < n; i += 2) {
        /* (1 2 1) / 4 around every second sample, as a round's recording is halved */
        int32_t y = ((int32_t)s_prev + 2 * (int32_t)x[i] + (int32_t)x[i + 1]) / 4;
        s_prev = x[i + 1];
        int64_t j = (n0 + i) / 2; /* this sample's number at 16 kHz */
        s_hist[s_w] = s_hist[s_w + SD_TAPS] = (int16_t)y;
        s_w = (s_w + 1) % SD_TAPS;
        if (!(j & 1) || j < SD_TAPS) {
            continue;
        }
        const int16_t *w = &s_hist[s_w]; /* samples j - 1023 .. j, oldest first */
        /* Two halves each: 512 products of a sample and +-127 stay inside 32 bits. */
        int32_t i0 = 0, q0 = 0, i1 = 0, q1 = 0;
        for (int k = 0; k < SD_TAPS / 2; k++) {
            i0 += w[k] * s_tc[k];
            q0 += w[k] * s_ts[k];
        }
        for (int k = SD_TAPS / 2; k < SD_TAPS; k++) {
            i1 += w[k] * s_tc[k];
            q1 += w[k] * s_ts[k];
        }
        float ci = (float)i0 + (float)i1, cq = -((float)q0 + (float)q1);
        float p = ci * ci + cq * cq;
        int64_t m = (j - (SD_TAPS - 1)) / 2;
        s_ci[m & (RING - 1)] = ci;
        s_cq[m & (RING - 1)] = cq;
        if (s_state == IDLE) {
            if (s_warm < 400) { /* the first 50 ms: a plain mean */
                s_floor += (p - s_floor) / (float)(++s_warm);
            } else if (p > OVER * s_floor) {
                s_state = TRACK;
                s_peak = p;
                s_peak_m = m;
                s_until = m + SETTLE;
            } else if (p < 8.0f * s_floor) {
                s_floor += 0.0005f * (p - s_floor); /* the usual level, over a quarter second */
            } else {
                s_floor *= 1.0005f; /* a room that got louder for good is followed too */
            }
        } else if (s_state == TRACK) {
            if (p > s_peak) {
                s_peak = p;
                s_peak_m = m;
                s_until = m + SETTLE;
            } else if (m >= s_until) {
                s_state = HELD;
                s_until = s_peak_m + HOLD;
                int64_t first = s_peak_m - SD_PRE;
                if (first >= 0 && !ready) {
                    float g = 30000.0f / sqrtf(s_peak);
                    for (int k = 0; k < SD_LEN; k++) {
                        out->iq[2 * k] = (int16_t)lrintf(g * s_ci[(first + k) & (RING - 1)]);
                        out->iq[2 * k + 1] = (int16_t)lrintf(g * s_cq[(first + k) & (RING - 1)]);
                    }
                    float fl = g * sqrtf(s_floor);
                    out->iq[2 * SD_LEN] = (int16_t)(fl < 1.0f ? 1 : fl > 32767.0f ? 32767 : lrintf(fl));
                    out->iq[2 * SD_LEN + 1] = 0;
                    out->first = first;
                    ready = 1;
                }
            }
        } else if (m >= s_until) {
            s_state = IDLE;
        }
    }
    return ready;
}
