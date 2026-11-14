/* Tap trigger: spots a sharp rise in the microphone stream. Free of ESP-IDF so
 * that sim/board.c can run it on a desk, as chirp.h is.
 */
#pragma once
#include <stdint.h>
#include <stdlib.h>

#define TAP_SAMPLES 512 /* sent per tap */
#define TAP_PRE 128     /* of which before the trigger, so the box sees the background */

typedef struct {
    int64_t pending;   /* sample index that set the trigger off, or -1 */
    int64_t holdoff;   /* nothing triggers before this sample index */
    int32_t dc, floor; /* running mean and mean deviation, both times 256 */
} tap_trig_t;

/* Takes sample `x` at `index`. Returns the index of the first of TAP_SAMPLES
 * samples once a tap has been captured whole, else -1. A sample must exceed
 * the background by `ratio`, and `min` at least, to count as a tap. */
static int64_t tap_feed(tap_trig_t *t, int16_t x, int64_t index, int rate, int ratio, int min)
{
    t->dc += ((int32_t)x * 256 - t->dc) >> 8;
    int32_t level = abs(x - (t->dc >> 8));
    if (t->pending < 0) {
        int32_t threshold = (t->floor >> 8) * ratio;
        if (threshold < min) {
            threshold = min;
        }
        if (index >= t->holdoff && level > threshold) {
            t->pending = index;
        } else {
            t->floor += (level * 256 - t->floor) >> 10;
        }
    } else if (index == t->pending + (TAP_SAMPLES - TAP_PRE) - 1) {
        int64_t first = t->pending - TAP_PRE;
        t->holdoff = index + rate / 2; /* let the ring die away before listening again */
        t->pending = -1;
        return first;
    }
    return -1;
}
