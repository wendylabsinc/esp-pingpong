/* Chirp maths: the sweep a board plays, and finding sweeps in the microphone
 * stream. Free of ESP-IDF so that firmware/test_chirp.c can run it on a desk.
 *
 * The detector slides the sweep over the stream by FFT, one block at a time,
 * and reports where the two line up. Only the sweep's own band takes part, so
 * hum, rumble and the microphone's offset do not reach it.
 */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

/* ponytail: one FFT size. The sweep must fit in under half of it, so 64 ms at
 * 16 kHz; raise CHIRP_N to 4096 for longer sweeps or faster sampling. */
#define CHIRP_N 2048
#define CHIRP_LOOK 128 /* samples after a first rise in which the true peak must lie */

/* The FFT is the firmware's one hot loop; the rest of the build optimises for size. */
#if defined(__GNUC__) && !defined(__clang__)
#define CHIRP_FAST __attribute__((optimize("O2")))
#else
#define CHIRP_FAST
#endif

typedef struct {
    float re, im;
} cpx_t;

typedef struct {
    double at;    /* sample index where the sweep starts, with its fraction */
    float snr_db; /* peak over the block's background */
    int mine;
} chirp_hit_t;

typedef struct {
    int rate, len, lo, hi, hold;
    float threshold;           /* peak must exceed the background by this factor */
    int64_t quiet_until;       /* nothing is reported before this sample index */
    int64_t mask_from, mask_to; /* around our own sweep: chirp_own() reports that one */
    cpx_t w[CHIRP_N / 2];      /* FFT twiddles */
    cpx_t ref[CHIRP_N / 2];    /* the sweep's spectrum, conjugated */
    cpx_t x[CHIRP_N];          /* work */
} chirp_t;

static double chirp_phase(int f0, int f1, int ms, double t)
{
    return 2 * M_PI * (f0 * t + (f1 - f0) * t * t / (2 * ms * 1e-3));
}

/* Half-periods, in microseconds, of the square wave that plays the sweep.
 * Returns how many; none if `max` is too few. */
static int chirp_edges(int f0, int f1, int ms, uint16_t *half_us, int max)
{
    double k_r = (f1 - f0) / (ms * 1e-3), prev = 0;
    int n = (int)((f0 + f1) * (ms * 1e-3)); /* half-periods in the sweep */
    if (n > max) {
        return 0;
    }
    for (int k = 1; k <= n; k++) { /* the k-th edge is where the phase reaches k * pi */
        double t = k_r ? (-f0 + sqrt((double)f0 * f0 + k_r * k)) / k_r : k / (2.0 * f0);
        double edge = floor(t * 1e6 + 0.5); /* rounded from the start, so errors do not add up */
        half_us[k - 1] = (uint16_t)(edge - prev);
        prev = edge;
    }
    return n;
}

CHIRP_FAST static void chirp_fft(cpx_t *x, const cpx_t *w, int inverse)
{
    for (int i = 1, j = 0; i < CHIRP_N; i++) {
        int bit = CHIRP_N >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            cpx_t t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }
    for (int len = 2; len <= CHIRP_N; len <<= 1) {
        int half = len >> 1, step = CHIRP_N / len;
        for (int i = 0; i < CHIRP_N; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = w[k * step].re, wi = inverse ? -w[k * step].im : w[k * step].im;
                cpx_t a = x[i + k], b = x[i + k + half];
                float tr = b.re * wr - b.im * wi, ti = b.re * wi + b.im * wr;
                x[i + k] = (cpx_t){a.re + tr, a.im + ti};
                x[i + k + half] = (cpx_t){a.re - tr, a.im - ti};
            }
        }
    }
}

/* Samples to advance between blocks. */
static int chirp_hop(const chirp_t *c)
{
    return CHIRP_N - c->len + 1 - CHIRP_LOOK;
}

/* Returns 0 if the sweep does not fit the detector. */
static int chirp_init(chirp_t *c, int rate, int f0, int f1, int ms, float threshold_db)
{
    memset(c, 0, sizeof *c);
    c->rate = rate;
    c->len = rate * ms / 1000;
    c->lo = (int)((long long)f0 * CHIRP_N / rate) - 8;
    c->hi = (int)((long long)f1 * CHIRP_N / rate) + 8;
    c->threshold = powf(10, threshold_db / 10);
    c->hold = c->len + rate / 5; /* ponytail: deaf for the sweep plus 200 ms of echoes after each one */
    if (c->len < 64 || c->len > CHIRP_N / 2 || c->lo < 1 || c->hi >= CHIRP_N / 2 || f0 >= f1) {
        return 0;
    }
    for (int k = 0; k < CHIRP_N / 2; k++) {
        c->w[k] = (cpx_t){(float)cos(2 * M_PI * k / CHIRP_N), (float)-sin(2 * M_PI * k / CHIRP_N)};
    }
    /* Tapering the reference trades 2 dB of reach for side lobes too low to be
     * mistaken for an early arrival. */
    for (int n = 0; n < c->len; n++) {
        double taper = 0.5 - 0.5 * cos(2 * M_PI * (n + 0.5) / c->len);
        c->x[n].re = (float)(taper * sin(chirp_phase(f0, f1, ms, (double)n / rate)));
    }
    chirp_fft(c->x, c->w, 0);
    for (int k = 0; k < CHIRP_N / 2; k++) {
        c->ref[k] = (cpx_t){c->x[k].re, -c->x[k].im};
    }
    return 1;
}

static float chirp_power(const chirp_t *c, int lag)
{
    return c->x[lag].re;
}

static chirp_hit_t chirp_hit(const chirp_t *c, int64_t base, int peak, float background, int mine)
{
    /* Parabola through the peak of the envelope: a fraction of a sample. */
    float lo = sqrtf(chirp_power(c, peak - 1)), mid = sqrtf(chirp_power(c, peak)),
          hi = sqrtf(chirp_power(c, peak + 1)), bend = lo - 2 * mid + hi;
    double fine = bend < 0 ? 0.5 * (lo - hi) / bend : 0;
    return (chirp_hit_t){base + peak + fine, 10 * log10f(chirp_power(c, peak) / background), mine};
}

/* Slides the sweep over CHIRP_N samples. Leaves the match at each lag in
 * chirp_power() and returns the background level of the block, leaving out
 * any sweep in it. Echoes still ringing raise the background, which is what
 * keeps them from being reported. */
CHIRP_FAST static float chirp_correlate(chirp_t *c, const int16_t *s)
{
    int valid = CHIRP_N - c->len + 1, count = 0;
    for (int n = 0; n < CHIRP_N; n++) {
        c->x[n] = (cpx_t){s[n], 0};
    }
    chirp_fft(c->x, c->w, 0);
    /* Keeping only the positive frequencies of the band gives the envelope
     * directly, whatever phase the sweep arrives with. */
    for (int k = 0; k < CHIRP_N; k++) {
        if (k < c->lo || k > c->hi) {
            c->x[k] = (cpx_t){0, 0};
        } else {
            cpx_t a = c->x[k], b = c->ref[k];
            c->x[k] = (cpx_t){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
        }
    }
    chirp_fft(c->x, c->w, 1);
    float sum = 0, kept = 0;
    for (int k = 0; k < valid; k++) {
        c->x[k].re = c->x[k].re * c->x[k].re + c->x[k].im * c->x[k].im;
        sum += c->x[k].re;
    }
    for (int k = 0; k < valid; k++) {
        if (chirp_power(c, k) < 4 * sum / valid) {
            kept += chirp_power(c, k);
            count++;
        }
    }
    return count && kept > 0 ? kept / count : 1;
}

/* Call when we start playing, with the sample index of that moment. Our own
 * sweep is so loud at our own microphone that its distortion alone looks like
 * sweeps starting early and late, so the stream scan stays out of its way. */
static void chirp_played(chirp_t *c, int64_t at)
{
    c->mask_from = at - c->len;
    c->mask_to = at + c->len + c->hold;
}

/* Finds our own sweep in CHIRP_N samples that start shortly before we played:
 * nothing else is as loud as a buzzer a few centimetres from the microphone,
 * so it is simply the best match in the block. Returns 0 if it is not there. */
static int chirp_own(chirp_t *c, const int16_t *s, int64_t base, chirp_hit_t *hit)
{
    float background = chirp_correlate(c, s);
    int valid = CHIRP_N - c->len + 1, peak = 1;
    for (int k = 1; k < valid - 1; k++) {
        if (chirp_power(c, k) > chirp_power(c, peak)) {
            peak = k;
        }
    }
    if (chirp_power(c, peak) < c->threshold * background) {
        return 0; /* played, not heard: the buzzer is not connected */
    }
    *hit = chirp_hit(c, base, peak, background, 1);
    return 1;
}

/* Looks for other boards' sweeps starting in samples [base, base + chirp_hop)
 * of the stream; `s` holds CHIRP_N samples from `base`. Returns how many. */
static int chirp_scan(chirp_t *c, const int16_t *s, int64_t base, chirp_hit_t *hits, int max_hits)
{
    int valid = CHIRP_N - c->len + 1, hop = chirp_hop(c), found = 0;
    float background = chirp_correlate(c, s);
    for (int k = 1; k < hop && found < max_hits; k++) {
        int64_t at = base + k;
        if (at < c->quiet_until || (at >= c->mask_from && at < c->mask_to) ||
            chirp_power(c, k) < c->threshold * background) {
            continue;
        }
        int loudest = k, peak;
        for (int j = k; j < k + CHIRP_LOOK && j < valid - 1; j++) {
            if (chirp_power(c, j) > chirp_power(c, loudest)) {
                loudest = j;
            }
        }
        /* The direct sound arrives first, but an echo can be louder: take the
         * first peak within 6 dB of the loudest nearby. */
        for (peak = k; peak < loudest; peak++) {
            if (chirp_power(c, peak) >= chirp_power(c, loudest) / 4 &&
                chirp_power(c, peak) >= chirp_power(c, peak + 1)) {
                break;
            }
        }
        hits[found++] = chirp_hit(c, base, peak, background, 0);
        c->quiet_until = base + peak + c->hold;
        k = peak;
    }
    return found;
}
