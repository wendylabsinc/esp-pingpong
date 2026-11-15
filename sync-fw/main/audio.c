/*
 * XIAO ESP32S3 Sense microphone: 32 kHz capture, multi-band trigger, and
 * fragments sent to the Anchor over ESP-NOW.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "rom/ets_sys.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "proto.h"
#include "sweepdet.h"

static void nlogf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#define RATE 32000
#define BLOCK 640                       /* 20 ms */
#define RING (RATE * 4)                 /* 4 s of samples */
#define STAMPS 256                      /* > 4 s of blocks */
#define FLOOR_ALPHA 0.0067f             /* ~3 s time constant at 50 blocks/s */
#define LEVEL_ALPHA 0.35f               /* ~60 ms smoothing of each band */

typedef struct {
    float hz, thr_db, floor_db, level_db;
    float pow_s, floor_p; /* smoothed power (~60 ms) and noise floor (~3 s), linear */
    int hits;
} band_t;

static i2s_chan_handle_t s_rx;
/* Completion time of every DMA buffer (one 20 ms block each), stamped in the
 * I2S interrupt: independent of when the capture task gets to run. */
#define DMA_STAMPS 64
static volatile int64_t s_dma_done[DMA_STAMPS];
static volatile uint32_t s_dma_count;

static bool IRAM_ATTR on_dma_recv(i2s_chan_handle_t h, i2s_event_data_t *ev, void *ctx)
{
    (void)h; (void)ev; (void)ctx;
    s_dma_done[s_dma_count % DMA_STAMPS] = esp_timer_get_time();
    s_dma_count++;
    return false;
}
static int16_t *s_ring;
static ssm_stamp_t s_stamps[STAMPS];
static volatile int64_t s_n; /* samples captured so far */
static band_t s_bands[MAX_BANDS];
static int s_nbands;
static int s_mode_all;           /* 0: any band, 1: all bands */
static int s_need_blocks = 3;    /* consecutive blocks over threshold */
static int s_pre_ms = 300, s_post_ms = 700;  /* 1 s fragments: half the airtime of 2 s */
static int s_send_slot;              /* 0..3, from the MAC; "slot <n>" overrides */
#define SLOT_MS 400                  /* stagger of fresh fragments between nodes; resend requests repair collisions */
static volatile int s_force;
static volatile int s_outbox_n; /* fragments the sender still holds */
static volatile int s_enabled = 1;
static volatile int s_print_levels;
/* "trace on": every 5 blocks (100 ms) print what the trigger saw, to explain a
 * missed sound afterwards:
 *   V,<first sample>,<max dB over floor x10>,<max consecutive blocks over>,
 *     <fire bits, one per block, oldest first>,<onset bits>,<fragments recording>,
 *     <floor dB x10 of band 0> */
static volatile int s_trace;
static uint32_t s_frag_id;
/* Sweeps this board finds by itself (sweepdet.c), told to the Anchor as fragments of 0.7 KB in place of
 * the audio: with those the boards hear sweeps all the time, where a listening round's recordings left
 * them deaf for three seconds in five while they were sent. "detect 0" switches it off. */
#define HITS 8
static volatile int s_detect = 1;
static sd_hit_t s_hits[HITS];
static uint32_t s_hit_id[HITS];
static uint32_t s_heard;
static int64_t s_heard_n = -(int64_t)RATE * 10; /* sample at which the last sweep found began */
static SemaphoreHandle_t s_sent;
typedef struct { int64_t first, trig; uint32_t ns, hit, id; } job_t;
static QueueHandle_t s_jobs;
static volatile esp_now_send_status_t s_send_status;

static void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    s_send_status = status;
    xSemaphoreGive(s_sent);
}

/* Linear power of one frequency (Goertzel), or broadband mean square for hz <= 0. */
static float band_power(const int16_t *x, int n, float hz)
{
    if (hz <= 0) {
        double acc = 0;
        for (int i = 0; i < n; i++) acc += (double)x[i] * x[i];
        return (float)(acc / n);
    }
    float w = 2.0f * (float)M_PI * hz / RATE;
    float c = 2.0f * cosf(w), s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) {
        float s0 = x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    float p = s1 * s1 + s2 * s2 - c * s1 * s2;
    return p / ((float)n * n);
}

static size_t build_blob(uint8_t **out, uint32_t frag_id, int64_t first_n, uint32_t n_samples, int64_t trig_n, uint32_t hit)
{
    /* A round's recording goes out at half the sample rate: its sounds lie under 7 kHz, and sending
     * it is most of what a listening round takes (measured: three boards, 87 KB each, 4.6 s on the
     * air). Sample numbers in the fragment then count half-rate samples, block stamps included. */
    /* A sweep report (HEARD_HIT) is not audio: its values come 8000 a second, so its numbers count
     * every fourth sample. It carries the block stamps of the 200 ms before it too (the Anchor wants
     * five to place it) and only three seconds of beacons: reports come often, and each beacon is 24
     * bytes. */
    int heard = (hit & (RANGE_HIT | HEARD_HIT)) == HEARD_HIT; /* a round's number may have that bit set too */
    int every = heard ? 4 : (hit & RANGE_HIT) ? 2 : 1;
    if (every == 2 && (first_n & 1)) {
        first_n++;
        n_samples--;
    }
    /* stamps covering the fragment */
    ssm_stamp_t st[STAMPS];
    int ns = 0;
    for (int i = 0; i < STAMPS; i++) {
        const ssm_stamp_t *s = &s_stamps[i];
        if (s->esp_us && s->n >= first_n - BLOCK * (heard ? 10 : 1) && s->n < first_n + n_samples) {
            st[ns] = *s;
            st[ns++].n /= every; /* block starts are multiples of 640: no remainder */
        }
    }
    uint32_t n_out = heard ? 2 * SD_LEN + 2 : n_samples / every;
    /* beacons heard in the last 20 s */
    static ssm_beacon_t bc[BEACON_LOG];
    int nb = 0;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&g_beacon_lock);
    uint32_t total = g_beacon_log_count;
    uint32_t avail = total < BEACON_LOG ? total : BEACON_LOG;
    for (uint32_t k = 0; k < avail; k++) {
        const ssm_beacon_t *b = &g_beacon_log[(total - 1 - k) % BEACON_LOG];
        if (now - b->esp_us > (heard ? 3000000 : 20000000)) break;
        bc[nb++] = *b;
    }
    portEXIT_CRITICAL(&g_beacon_lock);

    size_t len = sizeof(ssm_head_t) + s_nbands * sizeof(ssm_band_t) + ns * sizeof(ssm_stamp_t) +
                 nb * sizeof(ssm_beacon_t) + n_out * 2;
    uint8_t *b = heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (!b) return 0;
    ssm_head_t h = {.magic = {'S', 'S', 'M', '1'}, .n_bands = s_nbands, .frag_id = frag_id, .rate = RATE / every,
                    .first_n = first_n / every, .n_samples = n_out, .trigger_n = trig_n / every, .bands_hit = hit,
                    .n_stamps = ns, .n_beacons = nb};
    esp_read_mac(h.node_mac, ESP_MAC_WIFI_STA);
    uint8_t *p = b;
    memcpy(p, &h, sizeof h); p += sizeof h;
    for (int i = 0; i < s_nbands; i++) {
        ssm_band_t sb = {s_bands[i].hz, s_bands[i].thr_db, s_bands[i].level_db, s_bands[i].floor_db};
        memcpy(p, &sb, sizeof sb); p += sizeof sb;
    }
    memcpy(p, st, ns * sizeof(ssm_stamp_t)); p += ns * sizeof(ssm_stamp_t);
    memcpy(p, bc, nb * sizeof(ssm_beacon_t)); p += nb * sizeof(ssm_beacon_t);
    if (heard) {
        const sd_hit_t *sd = &s_hits[frag_id % HITS];
        if (s_hit_id[frag_id % HITS] != frag_id) { /* overwritten while it waited: nothing true to send */
            free(b);
            return 0;
        }
        memcpy(p, sd->iq, n_out * 2);
        *out = b;
        return len;
    }
    for (uint32_t i = 0; i < n_out; i++) {
        int64_t at = first_n + (int64_t)i * every;
        int16_t v = s_ring[at % RING];
        if (every == 2 && at > 0) {
            /* (1 2 1) / 4 around the sample kept: nothing is delayed, and what lies above 8 kHz,
             * which would fold down, is 10 dB weaker at 10 kHz and 28 dB at 14 kHz */
            v = (int16_t)(((int32_t)s_ring[(at - 1) % RING] + 2 * (int32_t)v + (int32_t)s_ring[(at + 1) % RING]) / 4);
        }
        memcpy(p, &v, 2); p += 2;
    }
    *out = b;
    return len;
}

/* One piece, acknowledged by the receiver's radio (ESP-NOW unicast), with up
 * to 4 attempts. A confirmation left over from an earlier timed-out attempt is
 * cleared first, so it cannot be mistaken for this piece's. */
static int send_piece(const uint8_t *dst, const uint8_t *blob, size_t len, uint32_t frag_id, uint16_t i, uint16_t count)
{
    uint8_t pkt[SSF_HDR + SSF_CHUNK + TAG_LEN];
    size_t off = (size_t)i * SSF_CHUNK, n = len - off < SSF_CHUNK ? len - off : SSF_CHUNK;
    memcpy(pkt, "SSF1", 4);
    memcpy(pkt + 4, &frag_id, 4);
    memcpy(pkt + 8, &i, 2);
    memcpy(pkt + 10, &count, 2);
    uint32_t bl = len;
    memcpy(pkt + 12, &bl, 4);
    memcpy(pkt + 16, &g_boot_id, 4);
    memcpy(pkt + SSF_HDR, blob + off, n);
    if (!make_tag(pkt, SSF_HDR + n, pkt + SSF_HDR + n)) {
        return 0;
    }
    for (int attempt = 0; attempt < 4; attempt++) {
        xSemaphoreTake(s_sent, 0);
        if (esp_now_send(dst, pkt, SSF_HDR + n + TAG_LEN) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (xSemaphoreTake(s_sent, pdMS_TO_TICKS(200)) == pdTRUE &&
            s_send_status == ESP_NOW_SEND_SUCCESS) {
            return 1;
        }
    }
    return 0;
}

/* Sends every piece, then resends only the pieces that failed, for up to 4
 * rounds. Returns -1 if the Anchor looks gone (3 failures in a row), else the
 * number of pieces still unacknowledged (0 = delivered). A fragment is never
 * reported delivered with holes: the caller keeps it and retries later, and
 * the Anchor merges pieces it already has. */
static uint16_t s_failed[(RING * 2 + BEACON_LOG * sizeof(ssm_beacon_t) + 512) / SSF_CHUNK + 8];

static int send_blob(const uint8_t *blob, size_t len, uint32_t frag_id)
{
    const uint8_t *dst = g_anchor_mac;
    if (!esp_now_is_peer_exist(dst)) {
        esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peer.peer_addr, dst, 6);
        esp_now_add_peer(&peer);
    }
    uint16_t count = (len + SSF_CHUNK - 1) / SSF_CHUNK;
    if (count > sizeof s_failed / sizeof s_failed[0]) {
        return -1;
    }
    int nfail = 0, run = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (send_piece(dst, blob, len, frag_id, i, count)) {
            run = 0;
            continue;
        }
        s_failed[nfail++] = i;
        if (++run >= 3) {
            return -1;
        }
    }
    for (int round = 0; round < 4 && nfail; round++) {
        vTaskDelay(pdMS_TO_TICKS(20 * (round + 1)));
        int still = 0;
        for (int k = 0; k < nfail; k++) {
            if (!send_piece(dst, blob, len, frag_id, s_failed[k], count)) {
                s_failed[still++] = s_failed[k];
            }
        }
        nlogf("R,%lu,round=%d,resent=%d,still=%d\n", (unsigned long)frag_id, round + 1, nfail, still);
        nfail = still;
    }
    return nfail;
}

/* Node log over the radio. Event lines (D detection, S send, N resend, R
 * resend round, W warning, H heartbeat, B boot) and ESP_LOG output (prefixed
 * "L,") collect in a buffer that the sender task ships to the Anchor as signed
 * SSF1 pieces with frag_id = 0x80000000 | sequence, so the dongle forwards
 * them unchanged and the Anchor prints them (wendy device logs). A gap in the
 * sequence at the Anchor means log frames were lost; the node counts lines it
 * had no room for ("logdrop" in the H line). */
#define NLOG_SIZE 8192
static char s_nlog[NLOG_SIZE];
static size_t s_nlog_len;
static portMUX_TYPE s_nlog_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_nlog_seq, s_nlog_dropped;
static vprintf_like_t s_prev_vprintf;
static volatile int s_trace_radio;

static void nlog_append(const char *buf, size_t n)
{
    portENTER_CRITICAL(&s_nlog_lock);
    if (s_nlog_len + n <= NLOG_SIZE) {
        memcpy(s_nlog + s_nlog_len, buf, n);
        s_nlog_len += n;
    } else {
        s_nlog_dropped++;
    }
    portEXIT_CRITICAL(&s_nlog_lock);
}

/* printf to the USB console and append to the radio log */
static void nlogf(const char *fmt, ...)
{
    char buf[224];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof buf) {
        n = sizeof buf - 1;
        buf[n - 1] = '\n';
    }
    fputs(buf, stdout);
    nlog_append(buf, n);
}

static int nlog_vprintf(const char *fmt, va_list ap)
{
    char buf[224];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf + 2, sizeof buf - 2, fmt, ap2);
    va_end(ap2);
    if (n > 0) {
        buf[0] = 'L';
        buf[1] = ',';
        n += 2;
        if (n >= (int)sizeof buf) {
            n = sizeof buf - 1;
            buf[n - 1] = '\n';
        }
        nlog_append(buf, n);
    }
    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
}

static const char *reset_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    default: return "other";
    }
}

static int anchor_fresh(void)
{
    return g_have_anchor && esp_timer_get_time() - g_anchor_last_us < 3000000;
}

/* Fragments are built as soon as their post-roll is captured (the ring holds
 * 4 s, a fragment the last 2 s), then wait in a PSRAM outbox until an Anchor
 * is in range. A send that loses the Anchor midway is kept and repeated whole;
 * the Anchor merges pieces it already has. Delivery is asynchronous by design:
 * every sample carries its own timestamps. */
#define OUTBOX 16
typedef struct { uint8_t *blob; size_t len; uint32_t id; int tries; int64_t ready_us; } outbox_t;

/* Delivered fragments stay here a while so the Anchor can ask for pieces its
 * dongle acknowledged but never passed on (SSN1, see proto.h). */
#define RECENT 6
static outbox_t s_recent[RECENT];
static int s_recent_next;
typedef struct { uint32_t frag; uint16_t n; uint16_t idx[SSN_MAX]; } nack_t;
static QueueHandle_t s_nacks;

static void range_from_nack(uint32_t round, const uint8_t *vals, int n_vals);
static void beep_from_nack(int times);

void nack_received(const uint8_t *src, const uint8_t *data, int len)
{
    (void)src;
    uint32_t boot, frag;
    uint16_t n;
    memcpy(&boot, data + 4, 4);
    memcpy(&frag, data + 8, 4);
    memcpy(&n, data + 12, 2);
    if (boot == RANGE_BOOT && n > 0 && n <= SSN_MAX && len == SSN_HDR + 2 * n + TAG_LEN && tag_ok(data, len)) {
        range_from_nack(frag, data + SSN_HDR, n);
        return;
    }
    if (boot == DETECT_BOOT && n > 0 && n <= SSN_MAX && len == SSN_HDR + 2 * n + TAG_LEN && tag_ok(data, len)) {
        uint16_t on;
        memcpy(&on, data + SSN_HDR, 2);
        s_detect = on ? 1 : 0;
        nlogf("W,sweep reports %s\n", s_detect ? "on" : "off");
        return;
    }
    if (boot == BEEP_BOOT && n > 0 && n <= SSN_MAX && len == SSN_HDR + 2 * n + TAG_LEN && tag_ok(data, len)) {
        uint16_t times;
        memcpy(&times, data + SSN_HDR, 2);
        beep_from_nack(times);
        return;
    }
    if (!s_nacks || boot != g_boot_id || n == 0 || n > SSN_MAX || len != SSN_HDR + 2 * n + TAG_LEN ||
        !tag_ok(data, len)) {
        return;
    }
    nack_t k = {.frag = frag, .n = n};
    memcpy(k.idx, data + SSN_HDR, 2 * n);
    xQueueSend(s_nacks, &k, 0);
}

static void keep_recent(outbox_t *o)
{
    outbox_t *r = &s_recent[s_recent_next];
    free(r->blob);
    *r = *o;
    s_recent_next = (s_recent_next + 1) % RECENT;
}

static int send_piece(const uint8_t *dst, const uint8_t *blob, size_t len, uint32_t frag_id, uint16_t i, uint16_t count);

static void serve_nacks(outbox_t *box, int head, int n)
{
    nack_t k;
    while (xQueueReceive(s_nacks, &k, 0) == pdTRUE) {
        const outbox_t *o = NULL;
        for (int i = 0; i < RECENT && !o; i++) {
            if (s_recent[i].blob && s_recent[i].id == k.frag) o = &s_recent[i];
        }
        for (int i = 0; i < n && !o; i++) {
            if (box[(head + i) % OUTBOX].id == k.frag) o = &box[(head + i) % OUTBOX];
        }
        if (!o) {
            nlogf("N,%lu,asked=%u,gone\n", (unsigned long)k.frag, k.n);
            continue;
        }
        uint16_t count = (o->len + SSF_CHUNK - 1) / SSF_CHUNK;
        int ok = 0;
        for (int i = 0; i < k.n; i++) {
            if (k.idx[i] < count) ok += send_piece(g_anchor_mac, o->blob, o->len, o->id, k.idx[i], count);
        }
        nlogf("N,%lu,asked=%u,resent=%d\n", (unsigned long)k.frag, k.n, ok);
    }
}

static void ensure_peer(const uint8_t *dst)
{
    if (!esp_now_is_peer_exist(dst)) {
        esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peer.peer_addr, dst, 6);
        esp_now_add_peer(&peer);
    }
}

/* Ship whole log lines, up to one piece, at most every 500 ms (sooner when the
 * buffer is filling). A frame stays queued until its piece is acknowledged. */
static void flush_log(int outbox_n)
{
    static int64_t last_flush, last_beat;
    int64_t now = esp_timer_get_time();
    if (now - last_beat > 10000000) {
        last_beat = now;
        nlogf("H,up=%llds,outbox=%d,logdrop=%lu,psram_free=%u,detect=%d,heard=%lu\n", (long long)(now / 1000000), outbox_n,
              (unsigned long)s_nlog_dropped, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), s_detect,
              (unsigned long)s_heard);
    }
    if (!anchor_fresh() || (now - last_flush < 500000 && s_nlog_len < NLOG_SIZE / 2)) return;
    static uint8_t blob[SSF_CHUNK];
    size_t n;
    portENTER_CRITICAL(&s_nlog_lock);
    n = s_nlog_len;
    if (n > SSF_CHUNK) {
        n = SSF_CHUNK;
        while (n > 0 && s_nlog[n - 1] != '\n') n--;
        if (n == 0) n = SSF_CHUNK;
    }
    memcpy(blob, s_nlog, n);
    portEXIT_CRITICAL(&s_nlog_lock);
    if (!n) return;
    last_flush = now;
    ensure_peer(g_anchor_mac);
    if (send_piece(g_anchor_mac, blob, n, 0x80000000u | (s_nlog_seq & 0x7fffffffu), 0, 1)) {
        portENTER_CRITICAL(&s_nlog_lock);
        memmove(s_nlog, s_nlog + n, s_nlog_len - n);
        s_nlog_len -= n;
        portEXIT_CRITICAL(&s_nlog_lock);
        s_nlog_seq++;
    }
}

static void accept_job(outbox_t *box, int *head, int *n, const job_t *j)
{
    uint8_t *blob = NULL;
    size_t len = build_blob(&blob, j->id, j->first, j->ns, j->trig, j->hit);
    if (!blob) {
        nlogf("W,no memory for fragment %lu\n", (unsigned long)j->id);
        return;
    }
    if (*n == OUTBOX) {
        nlogf("W,outbox full, dropped fragment %lu\n", (unsigned long)box[*head].id);
        free(box[*head].blob);
        *head = (*head + 1) % OUTBOX;
        (*n)--;
    }
    box[(*head + *n) % OUTBOX] = (outbox_t){blob, len, j->id, 0, esp_timer_get_time()};
    (*n)++;
}

static void sender_task(void *arg)
{
    (void)arg;
    static outbox_t box[OUTBOX];
    int head = 0, n = 0;
    job_t j;
    uint32_t slot_waited = 0;
    for (;;) {
        /* take new jobs; don't block while something waits to go out */
        serve_nacks(box, head, n);
        s_outbox_n = n;
        flush_log(n);
        while (xQueueReceive(s_jobs, &j, n ? 0 : pdMS_TO_TICKS(100)) == pdTRUE) {
            accept_job(box, &head, &n, &j);
        }
        if (!n) continue;
        if (!anchor_fresh()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int64_t until = box[head].ready_us + (int64_t)s_send_slot * SLOT_MS * 1000;
        if (box[head].tries == 0 && s_send_slot > 0 && slot_waited != box[head].id && esp_timer_get_time() < until &&
            box[head].len > 4096 /* a sweep report is two pieces: it goes at once */) {
            /* A fresh fragment waits for this node's slot after it finished
             * recording, so nodes that heard the same sound do not all start
             * at once; a fragment that has already waited in the outbox goes
             * immediately. Finished recordings keep being copied out of the
             * 4 s audio ring meanwhile. */
            slot_waited = box[head].id;
            for (int64_t left; (left = until - esp_timer_get_time()) > 0;) {
                if (xQueueReceive(s_jobs, &j, pdMS_TO_TICKS(left / 1000 + 1)) == pdTRUE) {
                    accept_job(box, &head, &n, &j);
                }
            }
            continue; /* the head may have changed if the outbox overflowed */
        }
        outbox_t *o = &box[head];
        int64_t t0 = esp_timer_get_time();
        int r = send_blob(o->blob, o->len, o->id);
        o->tries++;
        nlogf("S,%lu,%u,%u,%d,%lld,try=%d,queued=%d\n", (unsigned long)o->id, (unsigned)o->len,
               (unsigned)((o->len + SSF_CHUNK - 1) / SSF_CHUNK), r, (long long)(esp_timer_get_time() - t0), o->tries, n - 1);
        if (r == 0) {
            keep_recent(o);
            head = (head + 1) % OUTBOX;
            n--;
        } else if (o->tries >= 20) {
            free(o->blob);
            head = (head + 1) % OUTBOX;
            n--;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/* ── Ranging rounds ───────────────────────────────────────────────────
 * The Anchor broadcasts "round r: boards A, B, C, one slot each" (SSR1). Every
 * board records the whole round as one fragment and plays one sweep in its
 * own slot. The boards' clocks are already synchronised by the beacons, so
 * the Anchor reads every distance from when each microphone heard each sweep
 * (anchor/ranging.py): nothing is detected or timed here, and a slot only has
 * to be right to some milliseconds. Sweep and wiring are ../range's: 2 to
 * 6 kHz in 64 ms, piezo between D0 and D1, driven in opposite phase. */
#define RANGE_PIN_A 1
#define RANGE_PIN_B 2
#define RANGE_F0 2000
#define RANGE_F1 6000
#define RANGE_SWEEP_MS 64
#define RANGE_HALVES ((RANGE_F0 + RANGE_F1) * RANGE_SWEEP_MS / 1000)
#define RANGE_PRE_MS 60   /* recorded before the round starts */
#define RANGE_TAIL_MS 150 /* and after its last slot */
static uint16_t s_sweep_gap[RANGE_HALVES]; /* microseconds per half cycle */
static TaskHandle_t s_sweep_task;
static esp_timer_handle_t s_range_timer;
static struct {
    uint32_t round;
    int64_t t0, end; /* esp_timer: start of slot 0, end of the recording */
} s_range;
static volatile int s_range_pending;
static uint32_t s_range_last;
static uint32_t s_sweeps_played;
static volatile int64_t s_sweep_at; /* esp_timer at the first edge of this board's last sweep; 0: none this round */
/* A board in use for rounds reports no triggered sounds for a while after each: with the Anchor listening
 * round after round, a loud repeating sound set off every board's trigger as well, and those one-second
 * fragments held the rounds' recordings up by tens of seconds. */
#define ROUND_QUIET_US 8000000
static int64_t s_round_done_us = -ROUND_QUIET_US;

/* The sweep is toggled out by hand with interrupts off on this core, as the
 * tone bursts further down are: 64 ms in which nothing else runs here. The
 * audio task shares the core and catches up from the microphone's buffers
 * (160 ms of them); their time stamps are taken on the other core. The RMT
 * driver that ../range uses was tried first: in this firmware it reported
 * every sweep as sent and most made no sound. */
static void sweep_task(void *arg)
{
    (void)arg;
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    for (;;) {
        uint32_t times = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* 1 in a round; a "beep" asks for several */
        for (uint32_t t = 0; t < times && t < 5; t++) {
            if (t) {
                vTaskDelay(pdMS_TO_TICKS(90));
            }
            portENTER_CRITICAL(&mux);
            int level = 1;
            s_sweep_at = esp_timer_get_time(); /* interrupts are off: true to a few microseconds */
            for (int i = 0; i < RANGE_HALVES; i++) {
                gpio_set_level(RANGE_PIN_A, level);
                gpio_set_level(RANGE_PIN_B, !level); /* bridge drive: twice the voltage across the piezo */
                esp_rom_delay_us(s_sweep_gap[i]);
                level = !level;
            }
            gpio_set_level(RANGE_PIN_A, 0);
            gpio_set_level(RANGE_PIN_B, 0);
            portEXIT_CRITICAL(&mux);
            s_sweeps_played++;
        }
    }
}

static void beep_from_nack(int times)
{
    for (int i = 0; s_sweep_task && i < times && i < 5; i++) {
        xTaskNotifyGive(s_sweep_task); /* the task plays as many sweeps as it was notified */
    }
}

static void range_play(void *arg)
{
    (void)arg;
    if (s_sweep_task) {
        xTaskNotifyGive(s_sweep_task);
    }
}

static void buzzer_start(void)
{
    /* Half cycles end where the phase f0 t + (f1 - f0) t^2 / (2 T) crosses k / 2. */
    double T = RANGE_SWEEP_MS * 1e-3, k1 = (RANGE_F1 - RANGE_F0) / T;
    long prev = 0;
    for (int k = 1; k <= RANGE_HALVES; k++) {
        long at = lround((-RANGE_F0 + sqrt((double)RANGE_F0 * RANGE_F0 + k1 * k)) / k1 * 1e6);
        s_sweep_gap[k - 1] = (uint16_t)(at - prev);
        prev = at;
    }
    for (int pin = RANGE_PIN_A; pin <= RANGE_PIN_B; pin++) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
        gpio_set_level(pin, 0);
    }
    esp_timer_create_args_t args = {.callback = range_play, .name = "sweep"};
    esp_timer_create(&args, &s_range_timer);
    xTaskCreatePinnedToCore(sweep_task, "sweep", 3072, NULL, 13, &s_sweep_task, 1);
}

static void range_begin(uint32_t round, uint16_t slot, uint16_t lead, int n, const uint8_t *macs, int64_t now_us);

/* The board's one LED (yellow, GPIO 21, lit when low) cannot show a colour, so it counts: the board
 * that is colour k on the Anchor's page (0 red, 1 green, 2 blue, ...) blinks k + 1 times, then
 * pauses. The Anchor says which with every round it starts. Dark until then. */
#define LED_PIN 21
static volatile int s_colour = -1;

static void led_task(void *arg)
{
    (void)arg;
    gpio_config_t c = {.pin_bit_mask = 1ULL << LED_PIN, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&c);
    gpio_set_level(LED_PIN, 1);
    int last = -2;
    for (;;) {
        int k = s_colour;
        if (k != last) {
            nlogf("C,%d\n", k); /* so the Anchor's log shows which colour this board believes it has */
            last = k;
        }
        if (k < 0) { /* no colour yet: one short flash every 3 s, to show the LED works */
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(40));
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        for (int i = 0; i <= k; i++) {
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}

void range_received(const uint8_t *data, int len, int64_t now_us)
{
    uint32_t round;
    uint16_t slot, lead;
    uint8_t n = data[12];
    if (n == 0 || n > SSR_MAX || len != SSR_HDR + 6 * n + TAG_LEN || !tag_ok(data, len)) {
        return;
    }
    memcpy(&round, data + 4, 4);
    memcpy(&slot, data + 8, 2);
    memcpy(&lead, data + 10, 2);
    range_begin(round, slot, lead, n, data + SSR_HDR, now_us);
}

/* The round as an SSN1 frame (RANGE_BOOT in proto.h); the tag is already checked. */
static void range_from_nack(uint32_t round, const uint8_t *vals, int n_vals)
{
    uint16_t v[4 + 3 * SSR_MAX];
    if (n_vals < 6 || n_vals > 4 + 3 * SSR_MAX) {
        return;
    }
    memcpy(v, vals, 2 * n_vals);
    if (v[2] == 0 || v[2] > SSR_MAX || (n_vals != 3 + 3 * v[2] && n_vals != 4 + 3 * v[2])) {
        return;
    }
    if (n_vals == 4 + 3 * v[2] && v[n_vals - 1] < 8) {
        s_colour = v[n_vals - 1]; /* one value more: which colour this board has on the Anchor's page */
    }
    range_begin(round, v[0], v[1], v[2], (const uint8_t *)&v[3], esp_timer_get_time());
}

static void range_begin(uint32_t round, uint16_t slot, uint16_t lead, int n, const uint8_t *macs, int64_t now_us)
{
    /* A round is sent a few times: the first copy heard starts it. */
    if (round == s_range_last || s_range_pending || slot < 100 || slot > 1000 || lead > 1000) {
        return;
    }
    s_range_last = round;
    s_sweep_at = 0;
    s_range.round = round;
    s_range.t0 = now_us + (int64_t)lead * 1000;
    s_range.end = s_range.t0 + (int64_t)n * slot * 1000 + RANGE_TAIL_MS * 1000;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for (int i = 0; i < n; i++) {
        if (!memcmp(macs + 6 * i, mac, 6) && s_range_timer) {
            esp_timer_stop(s_range_timer);
            esp_timer_start_once(s_range_timer, (uint64_t)lead * 1000 + (uint64_t)i * slot * 1000);
            break;
        }
    }
    s_range_pending = 1;
}

static void audio_task(void *arg)
{
    (void)arg;
    int16_t blk[BLOCK];
    int stamp_i = 0;
    uint32_t read_blocks = 0;
    /* Triggers waiting for their post-roll. Each new onset (the bands going
     * from quiet to over threshold) at least RETRIGGER_MS after the previous
     * one gets its own fragment, even while an earlier fragment is still
     * recording, so a sound right after another one is not missed. */
#define MAX_PENDING 3
#define RETRIGGER_MS 250
    int npend = 0;
    int64_t pend_n[MAX_PENDING];
    uint32_t pend_hit[MAX_PENDING];
    int prev_fire = 0;
    int64_t last_trig = -(int64_t)RATE * 10;
    int64_t last_print = 0;
    int tr_blocks = 0, tr_maxhits = 0, tr_fire = 0, tr_onset = 0;
    float tr_maxexc = -99.0f;
    int64_t tr_n0 = 0;
    for (;;) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, blk, sizeof blk, &got, 200) != ESP_OK || got != sizeof blk) continue;
        int64_t now = esp_timer_get_time();
        int64_t n0 = s_n;
        /* Reads return DMA buffers in order, so this block is the
         * read_blocks-th completed buffer. If capture fell further behind than
         * the DMA queue (dropped buffers), resynchronise to the newest one. */
        uint32_t done = s_dma_count;
        if (done - read_blocks > 8 || read_blocks >= done) read_blocks = done - 1;
        int64_t completed = s_dma_done[read_blocks % DMA_STAMPS];
        read_blocks++;
        if (completed <= 0 || completed > now) completed = now;
        s_stamps[stamp_i] = (ssm_stamp_t){.n = n0, .esp_us = completed - (int64_t)BLOCK * 1000000 / RATE};
        stamp_i = (stamp_i + 1) % STAMPS;
        for (int i = 0; i < BLOCK; i++) s_ring[(n0 + i) % RING] = blk[i];
        s_n = n0 + BLOCK;

        /* The sweep detector sees every block. What it finds during a ranging round is the boards' own
         * sweeps, which the round's recording already holds, so that is not reported. */
        if (s_detect) {
            static sd_hit_t found;
            if (sd_feed(blk, BLOCK, n0, &found) && !s_range_pending && now - s_round_done_us > 300000) {
                uint32_t id = ++s_frag_id;
                s_hits[id % HITS] = found;
                s_hit_id[id % HITS] = id;
                s_heard++;
                s_heard_n = (found.first + SD_PRE) * 4;
                job_t j = {.first = found.first * 4, .trig = (found.first + SD_PRE) * 4, .ns = SD_LEN * 4, .hit = HEARD_HIT, .id = id};
                if (xQueueSend(s_jobs, &j, 0) != pdTRUE) nlogf("W,sender busy, sweep report %lu dropped\n", (unsigned long)id);
            }
        }

        /* bands */
        uint32_t over = 0;
        for (int b = 0; b < s_nbands; b++) {
            band_t *bd = &s_bands[b];
            float pw = band_power(blk, BLOCK, bd->hz) + 1e-3f;
            if (bd->floor_p == 0) bd->floor_p = bd->pow_s = pw;
            bd->pow_s += LEVEL_ALPHA * (pw - bd->pow_s);
            bd->level_db = 10.0f * log10f(bd->pow_s);
            bd->floor_db = 10.0f * log10f(bd->floor_p);
            if (bd->level_db > bd->floor_db + bd->thr_db) {
                bd->hits++;
            } else {
                bd->hits = 0;
            }
            if (bd->hits >= s_need_blocks) over |= 1u << b;
            float exc = bd->level_db - bd->floor_db;
            if (exc > tr_maxexc) tr_maxexc = exc;
            if (bd->hits > tr_maxhits) tr_maxhits = bd->hits;
            /* The floor learns only from quiet blocks: no fragment recording, the
             * smoothed level not over threshold, and the raw block power itself
             * under threshold. Without the last test the first block of every
             * sound (raw power high, smoothed level not yet over) was learned,
             * and sounds 2 s apart ratcheted the floor up until they no longer
             * triggered (measured: -4.5 dB to +13.8 dB over 20 beeps).
             * A faster fall (0.4 s) for every quieter block was tried: the floor
             * then tracked the gaps in room noise and room sounds triggered 2.5
             * times as often.
             * But a floor more than 10 dB above the current block (a loud first
             * block after boot started it at +22.8 dB and it took 40 s to
             * settle) falls at 0.05 per block, about 0.4 s. */
            int quiet_block = 10.0f * log10f(pw) < bd->floor_db + bd->thr_db;
            if (pw < 0.1f * bd->floor_p) {
                bd->floor_p += 0.05f * (pw - bd->floor_p);
            } else if (!npend && bd->hits == 0 && quiet_block) {
                bd->floor_p += FLOOR_ALPHA * (pw - bd->floor_p);
            }
        }
        uint32_t all = s_nbands >= 32 ? 0xffffffffu : ((1u << s_nbands) - 1);
        int fire = s_nbands && (s_mode_all ? over == all : over != 0);
        if (s_force) { fire = 1; s_force = 0; over = 0; }
        if (s_range_pending) {
            fire = 0; /* the sweeps of a round are not a sound to report */
            if (now >= s_range.end) {
                int64_t want = (now - s_range.t0 + RANGE_PRE_MS * 1000) * RATE / 1000000;
                if (want > RING - BLOCK * 4) want = RING - BLOCK * 4;
                int64_t first = s_n > want ? s_n - want : 0;
                job_t j = {.first = first, .trig = first + RANGE_PRE_MS * RATE / 1000, .ns = (uint32_t)(s_n - first),
                           .hit = RANGE_HIT | (s_range.round & 0x7fffffffu), .id = ++s_frag_id};
                if (xQueueSend(s_jobs, &j, 0) != pdTRUE) {
                    nlogf("W,sender busy, ranging round %lu dropped\n", (unsigned long)s_range.round);
                } else {
                    /* round, fragment, sweeps played since boot, and when this board's sweep began: with
                     * that the Anchor knows how far the buzzer is from this board's own microphone */
                    nlogf("G,%lu,%lu,%lu,%lld\n", (unsigned long)s_range.round, (unsigned long)j.id,
                          (unsigned long)s_sweeps_played, (long long)s_sweep_at);
                }
                s_range_pending = 0;
                s_round_done_us = now;
            }
        }
        if (s_print_levels && now - last_print > 250000) {
            last_print = now;
            printf("L,%lld", (long long)n0);
            for (int b = 0; b < s_nbands; b++) printf(",%.0f:%.1f/%.1f", s_bands[b].hz, s_bands[b].level_db, s_bands[b].floor_db);
            printf("\n");
        }
        int onset = fire && !prev_fire;
        prev_fire = fire;
        if (s_trace) {
            if (tr_blocks == 0) tr_n0 = n0;
            tr_fire |= (fire ? 1 : 0) << tr_blocks;
            tr_onset |= (onset ? 1 : 0) << tr_blocks;
            if (++tr_blocks == 5) {
                if (s_trace_radio) {
                    nlogf("V,%lld,%d,%d,%d,%d,%d,%d\n", (long long)tr_n0, (int)(tr_maxexc * 10), tr_maxhits, tr_fire,
                       tr_onset, npend, s_nbands ? (int)(s_bands[0].floor_db * 10) : 0);
                } else {
                    printf("V,%lld,%d,%d,%d,%d,%d,%d\n", (long long)tr_n0, (int)(tr_maxexc * 10), tr_maxhits, tr_fire,
                       tr_onset, npend, s_nbands ? (int)(s_bands[0].floor_db * 10) : 0);
                }
                tr_blocks = tr_maxhits = tr_fire = tr_onset = 0;
                tr_maxexc = -99.0f;
            }
        }
        if (s_enabled && onset && n0 - last_trig >= (int64_t)RETRIGGER_MS * RATE / 1000 && now - s_round_done_us > ROUND_QUIET_US &&
            s_outbox_n < 2 /* a sound every few hundred ms filled the outbox and dropped fragments: no new one while two wait */ &&
            n0 - s_heard_n > (int64_t)RATE * 3 / 2 /* sweeps are being reported: their audio would only hold those reports up */) {
            if (npend < MAX_PENDING) {
                pend_n[npend] = n0;
                pend_hit[npend] = over;
                npend++;
                last_trig = n0;
                nlogf("D,%lu,%lld,%lu\n", (unsigned long)(s_frag_id + npend), (long long)n0, (unsigned long)over);
            } else {
                nlogf("W,trigger skipped, %d fragments already recording\n", npend);
            }
        }
        /* oldest first: emit every pending fragment whose post-roll is in */
        while (npend && s_n >= pend_n[0] + (int64_t)s_post_ms * RATE / 1000) {
            int64_t trig_n = pend_n[0];
            int64_t first = trig_n - (int64_t)s_pre_ms * RATE / 1000;
            if (first < 0) first = 0;
            uint32_t ns = (uint32_t)(s_n - first);
            if (ns > RING - BLOCK * 4) ns = RING - BLOCK * 4;
            if (s_heard_n > trig_n - (int64_t)RATE * 3 / 10) {
                /* what set the trigger off was a sweep, found while this recording ran: the report is enough */
            } else {
                job_t j = {.first = first, .trig = trig_n, .ns = ns, .hit = pend_hit[0], .id = ++s_frag_id};
                if (xQueueSend(s_jobs, &j, 0) != pdTRUE) nlogf("W,sender busy, fragment %lu dropped\n", (unsigned long)j.id);
            }
            for (int i = 1; i < npend; i++) {
                pend_n[i - 1] = pend_n[i];
                pend_hit[i - 1] = pend_hit[i];
            }
            npend--;
        }
    }
}

void IRAM_ATTR audio_force_from_isr(void)
{
    s_force = 1;
}

/* Ground truth for audio timestamps: play a tone burst on a buzzer next to
 * the microphone and stamp its first edge on esp_timer. The burst is
 * bit-banged with interrupts held off, so the stamp is the true start to about
 * a microsecond. Prints "K,<pin>,<hz>,<cycles>,<esp_us of first edge>". */
static int s_beep_pin = -1, s_beep_hz = 4000, s_beep_cycles = 20, s_beep_every_ms;
/* Chirp mode: a linear sweep f0 -> f1 over dur_us has no repeating shape, so
 * its correlation peak is unique (a steady tone repeats every half cycle). */
static int s_chirp_f0, s_chirp_f1, s_chirp_us;
#define CHIRP_MAX_TOGGLES 400
static uint16_t s_chirp_gap[CHIRP_MAX_TOGGLES]; /* us between toggles */
static int s_chirp_n;

static void chirp_plan(void)
{
    /* toggle where the phase 2*pi*(f0*t + (f1-f0)*t^2/(2T)) crosses k*pi */
    double T = s_chirp_us * 1e-6, f0 = s_chirp_f0, k1 = (s_chirp_f1 - s_chirp_f0) / T;
    double prev = 0;
    s_chirp_n = 0;
    for (int k = 1; s_chirp_n < CHIRP_MAX_TOGGLES; k++) {
        /* solve f0 t + k1 t^2 / 2 = k / 2 */
        double t = (-f0 + sqrt(f0 * f0 + k1 * k)) / k1;
        if (t > T) break;
        s_chirp_gap[s_chirp_n++] = (uint16_t)llround((t - prev) * 1e6);
        prev = t;
    }
}
#define BEEP_LOG 64
static int64_t s_beep_log[BEEP_LOG]; /* stamps kept for "beeps": serial lines can drop */
static uint32_t s_beep_count;

static void beep_once(void)
{
    if (s_beep_pin < 0) return;
    int half = 500000 / s_beep_hz;
    s_force = 1; /* capture a fragment around it */
    vTaskDelay(pdMS_TO_TICKS(100));
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    int64_t t = esp_timer_get_time();
    if (s_chirp_us) {
        int level = 1;
        gpio_set_level(s_beep_pin, level);
        for (int i = 0; i < s_chirp_n; i++) {
            esp_rom_delay_us(s_chirp_gap[i]);
            level = !level;
            gpio_set_level(s_beep_pin, level);
        }
        gpio_set_level(s_beep_pin, 0);
    } else {
        for (int c = 0; c < s_beep_cycles; c++) {
            gpio_set_level(s_beep_pin, 1);
            esp_rom_delay_us(half);
            gpio_set_level(s_beep_pin, 0);
            esp_rom_delay_us(half);
        }
    }
    portEXIT_CRITICAL(&mux);
    s_beep_log[s_beep_count % BEEP_LOG] = t;
    s_beep_count++;
    if (s_chirp_us)
        printf("Q,%d,%d,%d,%d,%lld\n", s_beep_pin, s_chirp_f0, s_chirp_f1, s_chirp_us, (long long)t);
    else
        printf("K,%d,%d,%d,%lld\n", s_beep_pin, s_beep_hz, s_beep_cycles, (long long)t);
}

static void beep_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_beep_every_ms > 0) {
            beep_once();
            vTaskDelay(pdMS_TO_TICKS(s_beep_every_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

int audio_command(const char *line)
{
    int bp, bh = 4000, bc = 20;
    if (sscanf(line, "detect %d", &bp) == 1) { /* sweep reports on (1) or off (0) */
        s_detect = bp ? 1 : 0;
        printf("OK detect=%d heard=%lu\n", s_detect, (unsigned long)s_heard);
        return 1;
    }
    if (sscanf(line, "beeppin %d", &bp) == 1) {
        gpio_config_t c = {.pin_bit_mask = 1ULL << bp, .mode = GPIO_MODE_OUTPUT};
        gpio_config(&c);
        gpio_set_level(bp, 0);
        s_beep_pin = bp;
        return 1;
    }
    if (sscanf(line, "beep %d %d", &bh, &bc) >= 1 || !strcmp(line, "beep")) {
        if (bh > 100 && bh < 16000) s_beep_hz = bh;
        if (bc > 0 && bc < 400) s_beep_cycles = bc;
        beep_once();
        return 1;
    }
    int c0, c1, cus;
    if (sscanf(line, "chirp %d %d %d", &c0, &c1, &cus) == 3) {
        s_chirp_f0 = c0; s_chirp_f1 = c1; s_chirp_us = cus;
        if (cus > 0) chirp_plan();
        printf("C,%d toggles\n", s_chirp_n);
        return 1;
    }
    if (!strcmp(line, "beeps")) {
        uint32_t n = s_beep_count < BEEP_LOG ? s_beep_count : BEEP_LOG;
        for (uint32_t i = s_beep_count - n; i < s_beep_count; i++) {
            if (s_chirp_us)
                printf("Q,%d,%d,%d,%d,%lld\n", s_beep_pin, s_chirp_f0, s_chirp_f1, s_chirp_us, (long long)s_beep_log[i % BEEP_LOG]);
            else
                printf("K,%d,%d,%d,%lld\n", s_beep_pin, s_beep_hz, s_beep_cycles, (long long)s_beep_log[i % BEEP_LOG]);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        printf("KEND,%lu\n", (unsigned long)s_beep_count);
        return 1;
    }
    if (sscanf(line, "beepevery %d", &bp) == 1) {
        s_beep_every_ms = bp;
        return 1;
    }
    float hz, db;
    if (sscanf(line, "band %f %f", &hz, &db) == 2) {
        for (int i = 0; i < s_nbands; i++)
            if (fabsf(s_bands[i].hz - hz) < 0.5f) { s_bands[i].thr_db = db; return 1; }
        if (s_nbands < MAX_BANDS) s_bands[s_nbands++] = (band_t){.hz = hz, .thr_db = db};
        return 1;
    }
    if (!strcmp(line, "bands clear")) { s_nbands = 0; return 1; }
    if (!strcmp(line, "mode any")) { s_mode_all = 0; return 1; }
    if (!strcmp(line, "mode all")) { s_mode_all = 1; return 1; }
    if (!strcmp(line, "force")) { s_force = 1; return 1; }
    if (!strcmp(line, "sweep")) { range_play(NULL); return 1; } /* the ranging sweep once, to hear the buzzer */
    if (!strcmp(line, "levels on")) { s_print_levels = 1; return 1; }
    if (!strcmp(line, "trace on")) { s_trace = 1; return 1; }
    if (!strcmp(line, "trace radio on")) { s_trace = 1; s_trace_radio = 1; return 1; }
    if (!strcmp(line, "trace radio off")) { s_trace_radio = 0; return 1; }
    if (!strcmp(line, "trace off")) { s_trace = 0; return 1; }
    if (!strcmp(line, "levels off")) { s_print_levels = 0; return 1; }
    if (!strcmp(line, "audio on")) { s_enabled = 1; return 1; }
    if (!strcmp(line, "audio off")) { s_enabled = 0; return 1; }
    int v;
    if (sscanf(line, "pre %d", &v) == 1) { s_pre_ms = v; return 1; }
    if (sscanf(line, "post %d", &v) == 1) { s_post_ms = v; return 1; }
    if (sscanf(line, "blocks %d", &v) == 1) { s_need_blocks = v; return 1; }
    if (sscanf(line, "slot %d", &v) == 1) { s_send_slot = v < 0 ? 0 : v; return 1; }
    if (!strcmp(line, "bands")) {
        printf("B,mode=%s,blocks=%d,pre=%d,post=%d,slot=%d", s_mode_all ? "all" : "any", s_need_blocks, s_pre_ms, s_post_ms, s_send_slot);
        for (int i = 0; i < s_nbands; i++) printf(",%.0fHz+%.1fdB", s_bands[i].hz, s_bands[i].thr_db);
        printf("\n");
        return 1;
    }
    return 0;
}

void audio_start(void)
{
    s_prev_vprintf = esp_log_set_vprintf(nlog_vprintf);
    nlogf("B,boot=%08lx,reset=%s\n", (unsigned long)g_boot_id, reset_name(esp_reset_reason()));
    s_ring = heap_caps_calloc(RING, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    sd_init();
    s_sent = xSemaphoreCreateBinary();
    esp_now_register_send_cb(send_cb);
    /* Defaults that work without a laptop: the base station's code reaches
     * about +25 dB at 2.7 kHz (the buzzer's resonance); 1.8 kHz mostly caught
     * room noise. +12 dB doubled room triggers and swamped the radio; +15 dB
     * held over three 20 ms blocks keeps the 235 ms code and drops short room
     * sounds (clicks, syllables). */
    s_bands[0] = (band_t){.hz = 2700, .thr_db = 15};
    s_nbands = 1;
    s_need_blocks = 3;
    /* Send slot from this board's MAC, so nodes that hear the same sound do
     * not all send at once: the dongle's USB link drops pieces when they do. */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    s_send_slot = (mac[4] ^ mac[5]) % 4;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_frame_num = BLOCK;
    cc.dma_desc_num = 8;
    ESP_ERROR_CHECK(i2s_new_channel(&cc, NULL, &s_rx));
    i2s_pdm_rx_config_t pc = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.clk = GPIO_NUM_42, .din = GPIO_NUM_41},
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_rx, &pc));
    i2s_event_callbacks_t cbs = {.on_recv = on_dma_recv};
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(s_rx, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
    s_jobs = xQueueCreate(8, sizeof(job_t));
    s_nacks = xQueueCreate(8, sizeof(nack_t));
    xTaskCreatePinnedToCore(audio_task, "audio", 8192, NULL, 12, NULL, 1);
    xTaskCreatePinnedToCore(sender_task, "sender", 8192, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(beep_task, "beep", 3072, NULL, 5, NULL, 0);
    buzzer_start();
    xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 2, NULL, 0);
}
