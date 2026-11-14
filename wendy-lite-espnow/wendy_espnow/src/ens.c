#include "ens.h"

#include <string.h>

#define ACK_DELAY_US 10000
#define PROBE_US 200000
#define MAX_BACKOFF 4

void ens_init(struct ens *e, uint32_t conn, bool opener, uint8_t *tx, size_t tx_cap, uint8_t *rx,
              size_t rx_cap, uint16_t mss, uint32_t max_inflight, int64_t rto_us, int64_t now)
{
    memset(e, 0, sizeof *e);
    e->conn = conn;
    e->opener = opener;
    e->tx = tx;
    e->tx_cap = tx_cap;
    e->rx = rx;
    e->rx_cap = rx_cap;
    e->mss = mss;
    e->max_inflight = max_inflight;
    e->rto_us = rto_us;
    e->peer_wnd = mss;
    e->last_rx_us = e->last_progress_us = now;
    e->ack_now = opener; /* announce the connection even before any data */
}

static uint16_t rx_window(const struct ens *e)
{
    size_t room = e->rx_cap - e->rx_len;
    return room > 65535 ? 65535 : (uint16_t)room;
}

size_t ens_write(struct ens *e, const void *data, size_t size)
{
    size_t room = e->tx_cap - e->tx_len, n = size < room ? size : room;
    size_t at = (e->tx_head + e->tx_len) % e->tx_cap, first = e->tx_cap - at;
    if (first > n) first = n;
    memcpy(e->tx + at, data, first);
    memcpy(e->tx, (const uint8_t *)data + first, n - first);
    e->tx_len += n;
    return n;
}

size_t ens_read(struct ens *e, void *data, size_t size)
{
    size_t n = size < e->rx_len ? size : e->rx_len, first = e->rx_cap - e->rx_head;
    bool was_tight = e->rx_cap - e->rx_len < e->mss;
    if (first > n) first = n;
    memcpy(data, e->rx + e->rx_head, first);
    memcpy((uint8_t *)data + first, e->rx, n - first);
    e->rx_head = (e->rx_head + n) % e->rx_cap;
    e->rx_len -= n;
    /* reopening a closed window must be announced, or the peer keeps waiting */
    if (n && was_tight && e->rx_cap - e->rx_len >= e->mss) e->ack_now = true;
    return n;
}

void ens_input(struct ens *e, const struct ens_segment *seg, int64_t now)
{
    if (seg->conn != e->conn) return;
    e->stat_rx_segments++;
    e->peer_seen = true;
    e->last_rx_us = now;
    if (seg->flags & ENS_RST) {
        e->rst_rcvd = true;
        return;
    }
    /* acknowledgement: everything before seg->ack has arrived */
    uint32_t acked = seg->ack - e->snd_una;
    if (acked && acked <= e->tx_len) {
        e->tx_head = (e->tx_head + acked) % e->tx_cap;
        e->tx_len -= acked;
        e->snd_una += acked;
        if ((int32_t)(e->snd_nxt - e->snd_una) < 0) e->snd_nxt = e->snd_una;
        e->backoff = 0;
        e->last_progress_us = now;
        e->rto_at = now + e->rto_us;
        e->dup_acks = 0;
        e->recovering = false;
    } else if (!acked && (seg->flags & ENS_ACKONLY) && e->snd_nxt != e->snd_una && !e->recovering &&
               ++e->dup_acks >= 2) {
        /* The peer keeps acknowledging the same byte while more is in flight:
         * a segment was lost. Go back now instead of waiting for the timer. */
        e->snd_nxt = e->snd_una;
        e->recovering = true;
        e->stat_retx++;
    }
    e->peer_wnd = seg->wnd;
    if (seg->flags & ENS_ACKONLY) return;
    if (!seg->len) return;
    if (seg->seq == e->rcv_nxt) {
        size_t room = e->rx_cap - e->rx_len, n = seg->len < room ? seg->len : room;
        size_t at = (e->rx_head + e->rx_len) % e->rx_cap, first = e->rx_cap - at;
        if (first > n) first = n;
        memcpy(e->rx + at, seg->data, first);
        memcpy(e->rx, seg->data + first, n - first);
        e->rx_len += n;
        e->rcv_nxt += (uint32_t)n;
        if (n < seg->len) e->stat_rx_dropped++;
        /* acknowledge every second segment at once, a lone one after a short delay */
        if (++e->unacked_segments >= 2 || n < seg->len)
            e->ack_now = true;
        else if (!e->ack_at)
            e->ack_at = now + ACK_DELAY_US;
    } else {
        e->stat_rx_dropped++;
        e->ack_now = true; /* tell the sender where we are */
    }
}

void ens_reset(struct ens *e) { e->rst_send = true; }

static void fill(const struct ens *e, struct ens_segment *out, uint16_t flags)
{
    out->conn = e->conn;
    out->seq = e->snd_nxt;
    out->ack = e->rcv_nxt;
    out->wnd = rx_window(e);
    out->flags = (uint16_t)(flags | (e->opener && !e->peer_seen ? ENS_SYN : 0));
    out->data = NULL;
    out->len = 0;
}

int ens_poll(struct ens *e, int64_t now, struct ens_segment *out, uint8_t *scratch)
{
    if (e->rst_send) {
        e->rst_send = false;
        fill(e, out, ENS_RST | ENS_ACKONLY);
        return 1;
    }
    if (e->rst_rcvd) return 0;
    uint32_t inflight = e->snd_nxt - e->snd_una;
    if (inflight && now >= e->rto_at) { /* nothing acknowledged in time: go back */
        e->snd_nxt = e->snd_una;
        inflight = 0;
        e->stat_retx++;
        if (e->backoff < MAX_BACKOFF) e->backoff++;
    }
    size_t unsent = e->tx_len - inflight;
    uint32_t window = e->max_inflight;
    if (e->peer_wnd < window) window = e->peer_wnd;
    if (unsent && inflight < window) {
        size_t n = unsent < e->mss ? unsent : e->mss;
        if (n > window - inflight) n = window - inflight;
        size_t at = (e->tx_head + inflight) % e->tx_cap, first = e->tx_cap - at;
        if (first > n) first = n;
        memcpy(scratch, e->tx + at, first);
        memcpy(scratch + first, e->tx, n - first);
        fill(e, out, 0);
        out->data = scratch;
        out->len = (uint16_t)n;
        if (!inflight) e->rto_at = now + (e->rto_us << e->backoff);
        e->snd_nxt += (uint32_t)n;
        e->stat_tx_segments++;
        e->ack_now = false;
        e->ack_at = 0;
        e->unacked_segments = 0;
        return 1;
    }
    bool probe = false;
    if (unsent && !inflight && e->peer_wnd == 0 && now >= e->probe_at) { /* closed window */
        e->probe_at = now + PROBE_US;
        probe = true;
    }
    bool hello = e->opener && !e->peer_seen && now >= e->probe_at;
    if (hello) e->probe_at = now + PROBE_US;
    if (e->ack_now || (e->ack_at && now >= e->ack_at) || probe || hello) {
        e->ack_now = false;
        e->ack_at = 0;
        e->unacked_segments = 0;
        fill(e, out, ENS_ACKONLY);
        return 1;
    }
    return 0;
}
