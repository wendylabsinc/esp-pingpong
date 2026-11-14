/* The distance that recent rounds agree on. Free of ESP-IDF so that
 * ../test_agree.c can run it on a desk.
 *
 * A round that timed a false detection lands anywhere from 0 to 20 m; rounds
 * that timed the real sweeps land together. A median follows the wrong ones as
 * soon as they are half of the rounds, so this looks for a group instead.
 *
 * Rounds that timed an echo land together too, and can outnumber the real
 * ones: at a taped 1280 mm on 2026-11-14, of 84 rounds 8 read about 1270 mm,
 * 9 about 2480, 14 about 2670 and 4 about 4000 (one board, the other, both
 * hearing a reflection first). But an echo only ever arrives later than the
 * direct sound, so the real distance is the SHORTEST one that enough rounds
 * agree on, not the one that most do.
 */
#pragma once
#include <stdlib.h>
#include <string.h>

#define AGREE_KEEP 40 /* rounds remembered: enough that the direct sound, heard in one round of ten, gathers a group */

static int agree_cmp(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

/* Finds the shortest stretch `width_mm` wide that holds at least `least` of
 * mm[0..n), n from 1 to AGREE_KEEP, and returns its median; if none holds that
 * many, the stretch that holds the most. `*agree` is how many rounds are in it
 * and `*apart_mm` the distance between the lowest and highest of them. */
static int agree_distance(const int *mm, int n, int width_mm, int least, int *agree, int *apart_mm)
{
    int s[AGREE_KEEP], first = 0, count = 0;
    memcpy(s, mm, n * sizeof s[0]);
    qsort(s, n, sizeof s[0], agree_cmp);
    for (int i = 0, j = 0; i < n; i++) {
        while (j < n && s[j] - s[i] <= width_mm) {
            j++;
        }
        if (count >= least && s[i] - s[first] > width_mm) {
            break; /* past the shortest group that has enough */
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
