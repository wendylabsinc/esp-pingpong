/* The boards in radio contact, and which of them to range next. Free of
 * ESP-IDF so that ../test_peers.c can run it on a desk.
 *
 * Of any two boards the one with the lower MAC starts their rounds, so every
 * pair is ranged by exactly one of them.
 */
#pragma once
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "agree.h"

#define PEERS_MAX 8 /* ponytail: a fixed table; a ninth board is ignored until one leaves */

typedef struct {
    bool used;
    uint8_t mac[6];
    int64_t seen_us;    /* when its last frame came */
    int mm[AGREE_KEEP]; /* the last good rounds with it */
    unsigned good;
    int last_mm; /* the distance those agree on; INT_MIN: not settled yet */
} peer_t;

static peer_t *peer_find(peer_t *peers, const uint8_t *mac)
{
    for (peer_t *p = peers; p < peers + PEERS_MAX; p++) {
        if (p->used && !memcmp(p->mac, mac, 6)) {
            return p;
        }
    }
    return NULL;
}

/* A frame came from `mac`: its entry, made if need be. NULL: the table is full. */
static peer_t *peer_seen(peer_t *peers, const uint8_t *mac, int64_t now_us, bool *is_new)
{
    peer_t *p = peer_find(peers, mac);
    *is_new = !p;
    for (peer_t *q = peers; !p && q < peers + PEERS_MAX; q++) {
        if (!q->used) {
            p = q;
            memset(p, 0, sizeof *p);
            p->used = true;
            memcpy(p->mac, mac, 6);
            p->last_mm = INT_MIN;
        }
    }
    if (p) {
        p->seen_us = now_us;
    }
    return p;
}

/* The next board whose rounds we start: those with a higher MAC than `me`, in
 * turn. `*turn` is where the last one was. NULL: there is none. */
static peer_t *peer_next(peer_t *peers, const uint8_t *me, int *turn)
{
    for (int i = 1; i <= PEERS_MAX; i++) {
        int at = (*turn + i) % PEERS_MAX;
        if (peers[at].used && memcmp(me, peers[at].mac, 6) < 0) {
            *turn = at;
            return &peers[at];
        }
    }
    return NULL;
}

static int peer_count(const peer_t *peers)
{
    int n = 0;
    for (int i = 0; i < PEERS_MAX; i++) {
        n += peers[i].used;
    }
    return n;
}
