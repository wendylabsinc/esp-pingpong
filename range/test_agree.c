/* Self-check for main/agree.h, run on a desk:
 *
 *     clang -O2 -o /tmp/test_agree range/test_agree.c && /tmp/test_agree
 *
 * Replays the 22 rounds two boards printed at a taped 1270 mm on 2026-11-14,
 * in order, and expects the answer to stay on the real distance once three
 * rounds agree.
 */
#include <assert.h>
#include <stdio.h>

#include "main/agree.h"

int main(void)
{
    static const int rounds[] = {5848,  1614, 1287, 8239, 17387, 14003, 1331,  1254, 1263, 1264, 13547,
                                 13826, 1268, 1479, 1248, 7174,  1797,  14780, 1318, 1269, 2310, 1292};
    int ring[AGREE_KEEP], agree, apart, settled = 0;

    for (int i = 0; i < (int)(sizeof rounds / sizeof rounds[0]); i++) {
        ring[i % AGREE_KEEP] = rounds[i];
        int mm = agree_distance(ring, i < AGREE_KEEP ? i + 1 : AGREE_KEEP, 100, 3, &agree, &apart);
        assert(apart <= 100);
        if (agree >= 3) {
            settled++;
            assert(mm >= 1248 && mm <= 1331);
        }
    }
    assert(settled >= 14); /* it settles at the 8th round and stays settled */
    assert(agree >= 7);

    int one = 500, two[] = {900, 100, 110, 905, 120};
    assert(agree_distance(&one, 1, 100, 3, &agree, &apart) == 500 && agree == 1 && apart == 0);
    assert(agree_distance(two, 5, 100, 3, &agree, &apart) == 110 && agree == 3 && apart == 20);

    /* Echoes read long and can outnumber the direct sound: the shortest group
     * of three wins over a bigger group further out, a pair does not. */
    int echo[] = {2663, 1277, 2645, 2691, 1235, 16627, 2682, 1268, 2703, 4018, 2665, 1104};
    assert(agree_distance(echo, 12, 100, 3, &agree, &apart) == 1268 && agree == 3 && apart == 42);
    assert(agree_distance(echo, 7, 100, 3, &agree, &apart) == 2682 && agree == 4);
    int stray[] = {1160, 1250, 1255, 1268, 1277, 1279, 1340, 2663, 2665}; /* the fullest stretch of the shortest group */
    assert(agree_distance(stray, 9, 100, 3, &agree, &apart) == 1277 && agree == 6 && apart == 90);
    printf("ok\n");
    return 0;
}
