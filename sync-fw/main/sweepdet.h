/*
 * Finds the ranging sweep (2 to 6 kHz in 64 ms) in the microphone's samples as
 * they come in, so a board can tell the Anchor "a sweep reached me then"
 * without sending the audio. Plain C with no ESP calls: the same file is built
 * on a laptop for its test (anchor/test_sweepdet.py).
 *
 * The samples are halved to 16 kHz and matched against the sweep 8000 times a
 * second: c[m] is how well a sweep starting at 32 kHz sample 4 m fits, as a
 * complex number whose size is the envelope the Anchor times arrivals on
 * (anchor/ranging.py does the same sum over a whole recording). When the
 * envelope stands far out of its usual level the strongest point of the next
 * stretch is taken, and SD_LEN values around it are handed over: enough before
 * it for a direct sound weaker than its echo, and what follows the peak.
 */
#pragma once
#include <stdint.h>

#define SD_TAPS 1024 /* the sweep at 16 kHz */
#define SD_PRE 112   /* values kept before the strongest: 14 ms */
#define SD_POST 48   /* and from it on: 6 ms */
#define SD_LEN (SD_PRE + SD_POST)

typedef struct {
    int64_t first; /* m of the first value: it stands for a sweep starting at 32 kHz sample 4 * first */
    /* SD_LEN pairs (real, imaginary), scaled so the strongest is 30000 long; then one more pair:
     * the envelope's usual level in the same scale (at least 1), and 0. */
    int16_t iq[2 * SD_LEN + 2];
} sd_hit_t;

void sd_init(void);
/* x: n samples at 32 kHz, n even, the first being sample n0 (even) of an unbroken count.
 * Returns 1 when *out was filled: at most once per call for blocks under 80 ms. */
int sd_feed(const int16_t *x, int n, int64_t n0, sd_hit_t *out);
