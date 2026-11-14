/* ESP-NOW link of an acoustic node. See include/wendy_espnow.h.
 *
 * Carrier for the signed mesh leaf: the leaf dials a TCP port on this node
 * itself (127.0.0.1); the proxy task below moves those bytes over a reliable
 * ESP-NOW stream to the Anchor, which relays them to its WendyOS agent's leaf
 * listener. The leaf's preface and TLS pass through unchanged, so the dongle
 * and the Anchor only carry ciphertext.
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
#include "lwip/sockets.h"
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

static void wake_owner(void) { xSemaphoreGive(s_waiter); }

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

static bool stream_open(void)
{
    if (!s_espnow_up || !anchor_fresh())
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
    if (taken)
        xTaskNotifyGive(s_task);
    return taken;
}

static void stream_release(uint32_t up, uint32_t down)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_open) {
        ESP_LOGI(TAG, "stream %08lx: closed; %lu B up, %lu B down, %lu segments sent, %lu received, %lu retransmits",
                 (unsigned long)s_ens.conn, (unsigned long)up, (unsigned long)down,
                 (unsigned long)s_ens.stat_tx_segments, (unsigned long)s_ens.stat_rx_segments,
                 (unsigned long)s_ens.stat_retx);
        s_open = false;
        s_resets_left = 3; /* tell the Anchor, more than once: frames can be lost */
        s_reopen_at = esp_timer_get_time() + REOPEN_US;
    }
    xSemaphoreGive(s_lock);
    xTaskNotifyGive(s_task);
}

/* The mesh leaf's TCP carrier connects here instead of to a router on Wi-Fi. */
static void proxy_task(void *arg)
{
    (void)arg;
    static uint8_t up_buf[2048], down_buf[2048];
    int listener = -1;
    for (;;) {
        if (listener < 0) {
            listener = socket(AF_INET, SOCK_STREAM, 0);
            struct sockaddr_in local = {.sin_family = AF_INET, .sin_port = htons(CONFIG_WENDY_ESPNOW_PROXY_PORT),
                                        .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
            int yes = 1;
            if (listener >= 0)
                setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
            if (listener < 0 || bind(listener, (struct sockaddr *)&local, sizeof local) || listen(listener, 1)) {
                ESP_LOGW(TAG, "proxy listener failed (errno %d), retrying", errno);
                if (listener >= 0)
                    close(listener);
                listener = -1;
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            ESP_LOGI(TAG, "mesh carrier proxy listening on 127.0.0.1:%d", CONFIG_WENDY_ESPNOW_PROXY_PORT);
        }
        int fd = accept(listener, NULL, NULL);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!stream_open()) { /* no Anchor in range: the leaf sees a closed carrier and retries */
            close(fd);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        size_t pending = 0, offset = 0;
        uint32_t up = 0, down = 0;
        const char *why = "leaf closed";
        for (;;) {
            bool progress = false;
            if (!pending) { /* leaf -> stream */
                int n = recv(fd, up_buf, sizeof up_buf, 0);
                if (n > 0) {
                    pending = (size_t)n;
                    offset = 0;
                } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    break;
                }
            }
            xSemaphoreTake(s_lock, portMAX_DELAY);
            bool closed = !s_open || ens_closed(&s_ens);
            size_t wrote = pending && !closed ? ens_write(&s_ens, up_buf + offset, pending) : 0;
            size_t got = closed ? 0 : ens_read(&s_ens, down_buf, sizeof down_buf);
            xSemaphoreGive(s_lock);
            if (wrote) {
                offset += wrote;
                pending -= wrote;
                up += (uint32_t)wrote;
                progress = true;
            }
            if (wrote || got)
                xTaskNotifyGive(s_task);
            if (got) { /* stream -> leaf */
                size_t sent = 0;
                int64_t deadline = esp_timer_get_time() + 10000000;
                while (sent < got && esp_timer_get_time() < deadline) {
                    int n = send(fd, down_buf + sent, got - sent, 0);
                    if (n > 0)
                        sent += (size_t)n;
                    else if (errno == EAGAIN || errno == EWOULDBLOCK)
                        vTaskDelay(1);
                    else
                        break;
                }
                if (sent < got) {
                    why = "leaf socket write failed";
                    break;
                }
                down += (uint32_t)got;
                progress = true;
            }
            if (closed) {
                why = "Anchor closed the stream";
                break;
            }
            if (!progress)
                xSemaphoreTake(s_waiter, pdMS_TO_TICKS(5));
        }
        ESP_LOGI(TAG, "mesh carrier over ESP-NOW ended: %s", why);
        close(fd);
        stream_release(up, down);
    }
}

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
    if (xTaskCreate(link_task, "espnow_link", 4096, NULL, 10, &s_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    return xTaskCreate(proxy_task, "espnow_proxy", 4096, NULL, 6, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
