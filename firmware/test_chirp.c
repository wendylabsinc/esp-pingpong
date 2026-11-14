/* Self-check for main/chirp.h, run on a desk:
 *
 *     clang -O2 -o /tmp/test_chirp firmware/test_chirp.c -lm && /tmp/test_chirp
 *
 * Plays the square-wave sweep the buzzer would play into a simulated noisy
 * microphone and expects the detector to find it to a fraction of a sample.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "main/chirp.h"

#define RATE 16000
#define F0 2000
#define F1 6000
#define MS 50
#define TOTAL (RATE * 4)

static uint16_t half_us[1024];
static int halves;
static double mic[TOTAL];

static double noise(void)
{
    double sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += rand() / (double)RAND_MAX;
    }
    return sum - 6; /* near enough Gaussian, sigma 1 */
}

/* Level of the square wave `us` microseconds after it starts. */
static double square(double us)
{
    double edge = 0;
    for (int k = 0; k < halves; k++) {
        edge += half_us[k];
        if (us < edge) {
            return us < 0 ? 0 : (k & 1 ? -1 : 1);
        }
    }
    return 0;
}

/* Adds a sweep starting at sample `at`, averaged over each sample period the
 * way a microphone's own filter would. */
static void play(double at, double amplitude)
{
    for (int n = (int)at - 2; n < at + RATE * MS / 1000 + 2; n++) {
        double sum = 0;
        for (int sub = 0; sub < 16; sub++) {
            sum += square((n + sub / 16.0 - at) * 1e6 / RATE);
        }
        mic[n] += amplitude * sum / 16;
    }
}

int main(void)
{
    static chirp_t c;
    static int16_t stream[TOTAL];
    chirp_hit_t hits[8], all[16];
    int found = 0, played = 0, own_found = 0;

    halves = chirp_edges(F0, F1, MS, half_us, 1024);
    assert(halves == (F0 + F1) * MS / 1000);
    assert(chirp_init(&c, RATE, F0, F1, MS, 13));
    assert(!chirp_init(&c, RATE, F0, F1, 200, 13) && chirp_init(&c, RATE, F0, F1, MS, 13));

    /* Our own sweep, loud enough to clip, with a fainter ghost before and after
     * it standing in for the distortion a real buzzer adds; a neighbour's, far
     * below the noise in any one sample and followed by a louder echo; and a
     * third in the noise. */
    double own = 9000.25, far = 30000.6, faint = 50000.4;
    play(own, 60000);
    play(own - 500, 1500);
    play(own + 600, 1500);
    play(far, 25);
    play(far + 40.3, 40);
    play(faint, 12);
    srand(1);
    for (int n = 0; n < TOTAL; n++) {
        double v = mic[n] + 30 * noise() + 300; /* offset: the detector must not care */
        stream[n] = v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)lrint(v);
    }
    for (int64_t base = 0; base + CHIRP_N <= TOTAL; base += chirp_hop(&c)) {
        if (!played && base + CHIRP_N > 9003) {
            chirp_played(&c, 9003); /* the firmware knows when it played, to a few ms */
            played = 1;
        }
        if (played && !own_found && base > 9003 + CHIRP_N) { /* once the samples are in */
            own_found = chirp_own(&c, stream + 9003 - 200, 9003 - 200, &all[found]);
            assert(own_found);
            found++;
        }
        int n = chirp_scan(&c, stream + base, base, hits, 8);
        for (int i = 0; i < n && found < 16; i++) {
            all[found++] = hits[i];
        }
    }
    for (int i = 0; i < found; i++) {
        printf("sweep at %.2f  %.1f dB%s\n", all[i].at, all[i].snr_db, all[i].mine ? "  (own)" : "");
    }
    assert(found == 3);
    /* The detector's fixed lead or lag is the same for every sweep and cancels
     * in ranging, so check spacing, not absolute position. */
    double bias = all[0].at - own;
    printf("fixed offset %.2f samples; errors %.2f and %.2f samples\n", bias,
           all[1].at - far - bias, all[2].at - faint - bias);
    assert(all[0].mine && !all[1].mine && fabs(bias) < 2);
    assert(fabs(all[1].at - far - bias) < 0.5);   /* the direct sound, not its louder echo; 1 cm */
    assert(fabs(all[2].at - faint - bias) < 1.0); /* barely above the noise: 2 cm */
    puts("ok");
    return 0;
}
