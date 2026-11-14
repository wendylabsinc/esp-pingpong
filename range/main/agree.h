/* The distance that recent rounds agree on. Free of ESP-IDF so that
 * ../test_agree.c can run it on a desk.
 *
 * A round that timed a false detection or an echo lands anywhere from 0 to
 * 20 m; rounds that timed the real sweeps land together. A median follows the
 * wrong ones as soon as they are half of the rounds, so this looks for the
 * largest group instead.
 */
#pragma once
#include <stdlib.h>
#include <string.h>

#define AGREE_KEEP 15 /* rounds remembered */

static int agree_cmp(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

/* Finds the stretch `width_mm` wide that holds the most of mm[0..n), n from 1
 * to AGREE_KEEP, and returns its median. `*agree` is how many rounds are in it
 * and `*apart_mm` the distance between the lowest and highest of them. */
static int agree_distance(const int *mm, int n, int width_mm, int *agree, int *apart_mm)
{
    int s[AGREE_KEEP], first = 0, count = 0;
    memcpy(s, mm, n * sizeof s[0]);
    qsort(s, n, sizeof s[0], agree_cmp);
    for (int i = 0, j = 0; i < n; i++) {
        while (j < n && s[j] - s[i] <= width_mm) {
            j++;
        }
        if (j - i > count) {
            first = i;
            count = j - i;
        }
    }
    *agree = count;
    *apart_mm = s[first + count - 1] - s[first];
    return s[first + count / 2];
}
