/* ESP-NOW link of an acoustic node. See include/wendy_espnow.h.
 *
 * Frames, all signed with the shared soundsync key (HMAC-SHA256, 8 bytes):
 *   SSB1  beacon broadcast by the Anchor dongle: time sync only. Its hardware
 *         receive time is converted to esp_timer here and logged.
 *   SSF1  node -> Anchor, here carrying one stream segment: frag_id has the top
 *         bits 01 and the connection id, idx = receive window, count = flags,
 *         blen = sequence number, boot = acknowledgement, then the payload.
 *   SSN1  Anchor -> node, here carrying one stream segment: boot = 01 marker
 *         and connection id, frag = sequence number, 16-bit values: ack low,
 *         ack high, window, flags | length << 8, then the payload bytes.
 * The dongle firmware forwards SSF1 frames and sends SSN1 frames already, so
 * the stream needs nothing new on it. */
#include "wendy_espnow.h"

#include <string.h>

#include "ens.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_private/wifi.h" /* esp_wifi_internal_get_mac_clock_time */
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "psa/crypto.h"
#include "sdkconfig.h"
#include "secret.h"

#define TAG_LEN 8
#define SSB_LEN 28
#define SSF_HDR 20
#define SSN_HDR 14
#define SSN_MAX 64
#define UP_MSS 1400
#define STREAM_MARK 0x40000000u
#define STREAM_MASK 0xC0000000u
#define TX_BUFFER 16384
#define RX_BUFFER 8192
#define ANCHOR_FRESH_US 3000000
#define REOPEN_US 3000000
#define SILENT_US 15000000
#define RX_STAMP_MAX_LATE_US 20000
#define BEACON_LOG 256

static const char *TAG = "wendy_espnow";

static psa_key_id_t s_key;
static bool s_espnow_up;
static TaskHandle_t s_task;
static SemaphoreHandle_t s_lock, s_sent, s_waiter;
static volatile esp_now_send_status_t s_send_status;
static void (*volatile s_wake)(void);
static volatile bool s_rx_event; /* set in the Wi-Fi task, acted on in the link task */

/* time sync */
static struct wendy_espnow_beacon s_log[BEACON_LOG];
static uint32_t s_log_count;
static portMUX_TYPE s_log_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_anchor[6];
static volatile bool s_have_anchor;
static volatile int64_t s_anchor_last_us;
static uint32_t s_anchor_boot, s_anchor_seq;
static volatile uint32_t s_bad_tags, s_replays;

/* stream */
static struct ens s_ens;
static uint8_t *s_tx, *s_rx;
static bool s_open;       /* owned by the Wendy Net peer */
static int s_resets_left; /* RST frames still to send after a release */
static int64_t s_reopen_at;
static uint32_t s_opened, s_send_failures;

static bool make_tag(const void *data, size_t len, uint8_t out[TAG_LEN])
{
    uint8_t mac[32];
    size_t mac_len;
    if (psa_mac_compute(s_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), data, len, mac, sizeof mac, &mac_len) != PSA_SUCCESS)
        return false;
    memcpy(out, mac, TAG_LEN);
    return true;
}

static bool tag_ok(const uint8_t *frame, size_t len)
{
    uint8_t expect[TAG_LEN], diff = 0;
    if (len <= TAG_LEN || !make_tag(frame, len - TAG_LEN, expect))
        return false;
    for (int i = 0; i < TAG_LEN; i++)
        diff |= expect[i] ^ frame[len - TAG_LEN + i];
    return diff == 0;
}

/* When the frame just handed to us arrived, on esp_timer: the radio stamped it
 * on its own clock; read that clock between two esp_timer reads and subtract
 * the frame's age (as in esp-pingpong firmware/main/main.c arrival()). */
static int64_t arrival(uint32_t rx_ts, bool *soft)
{
    int64_t now = 0, gap = INT64_MAX;
    uint32_t radio_now = 0;
    for (int i = 0; i < 3 && gap > 20; i++) {
        int64_t before = esp_timer_get_time();
        uint32_t radio = esp_wifi_internal_get_mac_clock_time();
        int64_t after = esp_timer_get_time();
        if (after - before < gap) {
            gap = after - before;
            now = before + gap / 2;
            radio_now = radio;
        }
    }
    uint32_t late = radio_now - rx_ts;
    *soft = late >= RX_STAMP_MAX_LATE_US;
    return *soft ? now : now - (int64_t)late;
}

static void wake_owner(void)
{
    void (*wake)(void) = s_wake;
    if (wake)
        wake();
    xSemaphoreGive(s_waiter);
}

static void on_beacon(const esp_now_recv_info_t *info, const uint8_t *data, int len, int64_t now)
{
    if (!tag_ok(data, len)) {
        s_bad_tags++;
        return;
    }
    uint32_t seq, boot;
    memcpy(&seq, data + 4, 4);
    memcpy(&boot, data + 16, 4);
    /* one Anchor: a frame must be newer than the last one under the same boot id */
    if (s_have_anchor && !memcmp(s_anchor, info->src_addr, 6) && boot == s_anchor_boot && seq <= s_anchor_seq) {
        s_replays++;
        return;
    }
    bool soft;
    int64_t at = arrival(info->rx_ctrl->timestamp, &soft);
    memcpy(s_anchor, info->src_addr, 6);
    s_anchor_boot = boot;
    s_anchor_seq = seq;
    s_anchor_last_us = now;
    s_have_anchor = true;
    if (soft)
        return; /* callback-time stamps are not good enough for time sync */
    portENTER_CRITICAL(&s_log_lock);
    s_log[s_log_count % BEACON_LOG] = (struct wendy_espnow_beacon){.seq = seq, .arrival_us = at};
    s_log_count++;
    portEXIT_CRITICAL(&s_log_lock);
#if CONFIG_WENDY_ESPNOW_LOG_BEACONS
    ESP_LOGI(TAG, "B,%lu,%lld,%d", (unsigned long)seq, (long long)at, info->rx_ctrl->rssi);
#endif
}

static void on_downlink(const uint8_t *data, int len, int64_t now)
{
    uint32_t boot, frag;
    uint16_t n;
    memcpy(&boot, data + 4, 4);
    memcpy(&frag, data + 8, 4);
    memcpy(&n, data + 12, 2);
    if ((boot & STREAM_MASK) != STREAM_MARK || n < 4 || n > SSN_MAX || len != SSN_HDR + 2 * n + TAG_LEN)
        return;
    if (!tag_ok(data, len)) {
        s_bad_tags++;
        return;
    }
    uint16_t v[4];
    memcpy(v, data + SSN_HDR, sizeof v);
    struct ens_segment seg = {.conn = boot & 0x3FFFFFFFu, .seq = frag, .ack = v[0] | ((uint32_t)v[1] << 16),
                              .wnd = v[2], .flags = v[3] & 0xFF, .len = v[3] >> 8,
                              .data = data + SSN_HDR + 8};
    if (seg.len > 2 * (n - 4))
        return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_open)
        ens_input(&s_ens, &seg, now);
    xSemaphoreGive(s_lock);
    /* The owner is woken from the link task: its wake writes to a socket,
     * which must not happen in the Wi-Fi task. */
    s_rx_event = true;
    xTaskNotifyGive(s_task);
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    int64_t now = esp_timer_get_time();
    if (len == SSB_LEN && !memcmp(data, "SSB1", 4))
        on_beacon(info, data, len, now);
    else if (len >= SSN_HDR + 8 + TAG_LEN && !memcmp(data, "SSN1", 4))
        on_downlink(data, len, now);
}

static void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    s_send_status = status;
    xSemaphoreGive(s_sent);
}

static bool anchor_fresh(void)
{
    return s_have_anchor && esp_timer_get_time() - s_anchor_last_us < ANCHOR_FRESH_US;
}

static void send_segment(const struct ens_segment *seg)
{
    static uint8_t pkt[SSF_HDR + UP_MSS + TAG_LEN];
    uint32_t frag = STREAM_MARK | (seg->conn & 0x3FFFFFFFu);
    size_t n = (seg->flags & ENS_ACKONLY) || !seg->len ? 1 : seg->len;
    memcpy(pkt, "SSF1", 4);
    memcpy(pkt + 4, &frag, 4);
    memcpy(pkt + 8, &seg->wnd, 2);
    memcpy(pkt + 10, &seg->flags, 2);
    memcpy(pkt + 12, &seg->seq, 4);
    memcpy(pkt + 16, &seg->ack, 4);
    if ((seg->flags & ENS_ACKONLY) || !seg->len)
        pkt[SSF_HDR] = 0; /* the dongle forwards only frames with a payload */
    else
        memcpy(pkt + SSF_HDR, seg->data, n);
    if (!make_tag(pkt, SSF_HDR + n, pkt + SSF_HDR + n))
        return;
    if (!esp_now_is_peer_exist(s_anchor)) {
        esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peer.peer_addr, s_anchor, 6);
        esp_now_add_peer(&peer);
    }
    xSemaphoreTake(s_sent, 0); /* drop a confirmation left over from a timed-out send */
    if (esp_now_send(s_anchor, pkt, SSF_HDR + n + TAG_LEN) != ESP_OK ||
        xSemaphoreTake(s_sent, pdMS_TO_TICKS(100)) != pdTRUE || s_send_status != ESP_NOW_SEND_SUCCESS) {
        s_send_failures++; /* the stream retransmits; nothing else to do here */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void link_task(void *arg)
{
    (void)arg;
    static uint8_t scratch[UP_MSS];
    int64_t last_report = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        if (!s_espnow_up) {
            /* Wi-Fi is started by wendy_wifi; ESP-NOW can only follow it. */
            if (esp_now_init() != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            esp_now_register_recv_cb(recv_cb);
            esp_now_register_send_cb(send_cb);
            s_espnow_up = true;
            uint8_t channel = 0;
            wifi_second_chan_t second;
            esp_wifi_get_channel(&channel, &second);
            ESP_LOGI(TAG, "ESP-NOW up on Wi-Fi channel %u", channel);
        }
        int64_t now = esp_timer_get_time();
        for (int burst = 0; burst < 4; burst++) {
            struct ens_segment seg;
            bool have = false;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_open || s_resets_left > 0) {
                if (s_resets_left > 0) {
                    ens_reset(&s_ens);
                    s_resets_left--;
                }
                have = ens_poll(&s_ens, esp_timer_get_time(), &seg, scratch) == 1;
            }
            xSemaphoreGive(s_lock);
            if (!have)
                break;
            send_segment(&seg);
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_open && !ens_closed(&s_ens) &&
            (now - s_ens.last_rx_us > SILENT_US || (!anchor_fresh() && now - s_ens.last_rx_us > 5000000))) {
            ESP_LOGW(TAG, "stream %08lx: Anchor silent, closing", (unsigned long)s_ens.conn);
            s_ens.rst_rcvd = true; /* the owner sees end of stream and retires the link */
            xSemaphoreGive(s_lock);
            wake_owner();
        } else {
            xSemaphoreGive(s_lock);
            if (s_rx_event) { /* new bytes to read, or acknowledgements freed send space */
                s_rx_event = false;
                wake_owner();
            }
        }
        if (now - last_report > 10000000) {
            last_report = now;
            uint8_t channel = 0;
            wifi_second_chan_t second;
            esp_wifi_get_channel(&channel, &second);
            xSemaphoreTake(s_lock, portMAX_DELAY);
            ESP_LOGI(TAG, "ch=%u anchor=%d beacons=%lu bad_tags=%lu replays=%lu stream=%s conn=%08lx tx_segments=%lu "
                          "rx_segments=%lu retx=%lu send_failures=%lu opened=%lu",
                     channel, anchor_fresh(), (unsigned long)s_log_count, (unsigned long)s_bad_tags,
                     (unsigned long)s_replays, s_open ? (ens_closed(&s_ens) ? "closed" : "open") : "none",
                     (unsigned long)s_ens.conn, (unsigned long)s_ens.stat_tx_segments,
                     (unsigned long)s_ens.stat_rx_segments, (unsigned long)s_ens.stat_retx,
                     (unsigned long)s_send_failures, (unsigned long)s_opened);
            xSemaphoreGive(s_lock);
        }
    }
}

static ssize_t stream_read(void *context, void *data, size_t size)
{
    (void)context;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = s_open ? ens_read(&s_ens, data, size) : 0;
    bool closed = !s_open || ens_closed(&s_ens);
    xSemaphoreGive(s_lock);
    if (n) {
        xTaskNotifyGive(s_task); /* a window update may be due */
        return (ssize_t)n;
    }
    return closed ? 0 : -2;
}

static ssize_t stream_write(void *context, const void *data, size_t size)
{
    (void)context;
    if (!size)
        return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_open || ens_closed(&s_ens)) {
        xSemaphoreGive(s_lock);
        return -1;
    }
    size_t n = ens_write(&s_ens, data, size);
    xSemaphoreGive(s_lock);
    if (!n)
        return -3;
    xTaskNotifyGive(s_task);
    return (ssize_t)n;
}

static int stream_wait(void *context, bool writing, int64_t deadline)
{
    (void)context;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool available = !s_open || ens_closed(&s_ens) || (writing ? ens_tx_free(&s_ens) > 0 : s_ens.rx_len > 0);
    xSemaphoreGive(s_lock);
    int64_t left = deadline - esp_timer_get_time();
    if (available || left <= 0)
        return 0;
    if (left > 50000)
        left = 50000;
    xSemaphoreTake(s_waiter, pdMS_TO_TICKS((left + 999) / 1000) ?: 1);
    return 0; /* a hint: the caller rechecks state and time */
}

bool wendy_espnow_take(struct wendy_pki_stream *stream)
{
    if (!s_task || !s_espnow_up || !anchor_fresh())
        return false;
    bool taken = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    if (!s_open && s_resets_left <= 0 && now >= s_reopen_at) {
        uint32_t conn = esp_random() & 0x3FFFFFFFu;
        ens_init(&s_ens, conn ? conn : 1, true, s_tx, TX_BUFFER, s_rx, RX_BUFFER, UP_MSS, 4 * UP_MSS, 150000, now);
        s_open = true;
        s_opened++;
        taken = true;
        ESP_LOGI(TAG, "stream %08lx: opening to Anchor " MACSTR, (unsigned long)s_ens.conn, MAC2STR(s_anchor));
    }
    xSemaphoreGive(s_lock);
    if (taken) {
        *stream = (struct wendy_pki_stream){.context = NULL, .read = stream_read, .write = stream_write,
                                            .wait = stream_wait};
        xTaskNotifyGive(s_task);
    }
    return taken;
}

void wendy_espnow_release(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_open) {
        ESP_LOGI(TAG, "stream %08lx: released; %lu segments sent, %lu received, %lu retransmits",
                 (unsigned long)s_ens.conn, (unsigned long)s_ens.stat_tx_segments,
                 (unsigned long)s_ens.stat_rx_segments, (unsigned long)s_ens.stat_retx);
        s_open = false;
        s_resets_left = 3; /* tell the Anchor, more than once: frames can be lost */
        s_reopen_at = esp_timer_get_time() + REOPEN_US;
    }
    xSemaphoreGive(s_lock);
    if (s_task)
        xTaskNotifyGive(s_task);
}

bool wendy_espnow_output_drained(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool drained = !s_open || ens_tx_empty(&s_ens);
    xSemaphoreGive(s_lock);
    return drained;
}

void wendy_espnow_set_wake(void (*wake)(void)) { s_wake = wake; }

int wendy_espnow_beacons(struct wendy_espnow_beacon *out, int max)
{
    int n = 0;
    portENTER_CRITICAL(&s_log_lock);
    uint32_t total = s_log_count, avail = total < BEACON_LOG ? total : BEACON_LOG;
    for (uint32_t k = 0; k < avail && n < max; k++)
        out[n++] = s_log[(total - 1 - k) % BEACON_LOG];
    portEXIT_CRITICAL(&s_log_lock);
    return n;
}

esp_err_t wendy_espnow_start(void)
{
    static const uint8_t key[32] = SOUNDSYNC_KEY;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    if (psa_crypto_init() != PSA_SUCCESS || psa_import_key(&attr, key, sizeof key, &s_key) != PSA_SUCCESS)
        return ESP_FAIL;
    s_tx = heap_caps_malloc(TX_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_rx = heap_caps_malloc(RX_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_lock = xSemaphoreCreateMutex();
    s_sent = xSemaphoreCreateBinary();
    s_waiter = xSemaphoreCreateBinary();
    if (!s_tx || !s_rx || !s_lock || !s_sent || !s_waiter)
        return ESP_ERR_NO_MEM;
    return xTaskCreate(link_task, "espnow_link", 4096, NULL, 10, &s_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
