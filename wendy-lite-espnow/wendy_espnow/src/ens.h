/* ens: a small reliable byte stream over lossy datagrams (ESP-NOW).
 *
 * Portable, no allocation, no OS calls: the owner feeds received segments to
 * ens_input(), asks ens_poll() what to send next, and moves bytes with
 * ens_write()/ens_read(). Cumulative acknowledgement, go-back-N on a
 * retransmit timer, receiver window, delayed acknowledgements. One side opens
 * (SYN on every segment until the peer answers); either side resets (RST).
 *
 * The owner maps segments to its link frames: see wendy_espnow.c (node) and
 * anchor/enstream.py (Anchor); both are tested against this file. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { ENS_SYN = 1, ENS_FIN = 2, ENS_RST = 4, ENS_ACKONLY = 8 };

struct ens_segment {
    uint32_t conn, seq, ack;
    uint16_t wnd, flags;
    const uint8_t *data; /* ACKONLY segments carry no stream bytes (len ignored) */
    uint16_t len;
};

struct ens {
    uint32_t conn;
    uint8_t *tx, *rx;
    size_t tx_cap, tx_head, tx_len; /* tx holds [snd_una, snd_una + tx_len) */
    size_t rx_cap, rx_head, rx_len;
    uint32_t snd_una, snd_nxt, rcv_nxt;
    uint16_t mss, peer_wnd;
    uint32_t max_inflight;
    int64_t rto_us, rto_at, ack_at, probe_at, last_rx_us, last_progress_us;
    unsigned backoff, unacked_segments, dup_acks;
    bool opener, peer_seen, ack_now, rst_rcvd, rst_send, recovering;
    uint32_t stat_retx, stat_tx_segments, stat_rx_segments, stat_rx_dropped;
};

void ens_init(struct ens *e, uint32_t conn, bool opener, uint8_t *tx, size_t tx_cap, uint8_t *rx,
              size_t rx_cap, uint16_t mss, uint32_t max_inflight, int64_t rto_us, int64_t now);
/* bytes accepted (0 when the send buffer is full) */
size_t ens_write(struct ens *e, const void *data, size_t size);
/* bytes delivered (0 when nothing is waiting) */
size_t ens_read(struct ens *e, void *data, size_t size);
void ens_input(struct ens *e, const struct ens_segment *seg, int64_t now);
/* 1: send *out (its data points into scratch, at least mss bytes); 0: nothing now */
int ens_poll(struct ens *e, int64_t now, struct ens_segment *out, uint8_t *scratch);
void ens_reset(struct ens *e); /* queue one RST for ens_poll to emit */
static inline bool ens_closed(const struct ens *e) { return e->rst_rcvd; }
static inline size_t ens_tx_free(const struct ens *e) { return e->tx_cap - e->tx_len; }
static inline bool ens_tx_empty(const struct ens *e) { return e->tx_len == 0; }
