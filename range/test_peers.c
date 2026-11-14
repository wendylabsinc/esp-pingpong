/* Self-check for main/peers.h, run on a desk:
 *
 *     clang -O2 -o /tmp/test_peers range/test_peers.c && /tmp/test_peers
 */
#include <assert.h>
#include <stdio.h>

#include "main/peers.h"

int main(void)
{
    static peer_t peers[PEERS_MAX];
    const uint8_t me[6] = {0, 0, 0, 0, 0, 5}, low[6] = {0, 0, 0, 0, 0, 2}, b[6] = {0, 0, 0, 0, 0, 7},
                  c[6] = {0, 0, 0, 0, 1, 0};
    bool is_new;
    int turn = 0;

    assert(!peer_next(peers, me, &turn) && peer_count(peers) == 0);
    assert(peer_seen(peers, low, 100, &is_new) && is_new);
    assert(!peer_next(peers, me, &turn)); /* the lower board starts the rounds with us */
    peer_t *pb = peer_seen(peers, b, 200, &is_new);
    assert(pb && is_new && pb->last_mm == INT_MIN);
    assert(peer_seen(peers, b, 300, &is_new) == pb && !is_new && pb->seen_us == 300);
    assert(peer_next(peers, me, &turn) == pb && peer_next(peers, me, &turn) == pb); /* the only one */
    peer_t *pc = peer_seen(peers, c, 400, &is_new);
    assert(peer_count(peers) == 3 && peer_find(peers, c) == pc);
    for (int i = 0; i < 6; i++) { /* the two higher boards take turns */
        assert(peer_next(peers, me, &turn) == (i % 2 ? pb : pc));
    }
    pc->used = false; /* lost */
    assert(!peer_find(peers, c) && peer_next(peers, me, &turn) == pb);

    for (uint8_t i = 0; i < PEERS_MAX; i++) { /* a full table takes no more, and still knows its own */
        uint8_t mac[6] = {9, 0, 0, 0, 0, i};
        peer_seen(peers, mac, 500, &is_new);
    }
    const uint8_t extra[6] = {9, 9, 9, 9, 9, 9};
    assert(peer_count(peers) == PEERS_MAX && !peer_seen(peers, extra, 600, &is_new) && is_new);
    assert(peer_seen(peers, b, 700, &is_new) == pb && !is_new);
    printf("ok\n");
    return 0;
}
