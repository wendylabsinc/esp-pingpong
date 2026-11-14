/*
 * soundsync-fw: timing radio for acoustic nodes and the Anchor dongle.
 *
 * Every device, whatever its role:
 *   - listens on one Wi-Fi channel in promiscuous mode, never associating;
 *   - logs every access point beacon (BSSID, the AP's TSF from the frame,
 *     our hardware receive stamp, esp_timer at the callback);
 *   - logs every soundsync ESP-NOW beacon it hears, with the same stamps;
 *   - can broadcast ESP-NOW beacons itself ("beacon on").
 *
 * Reference Broadcast Synchronization: devices that hear the same beacon
 * received it at the same instant (radio propagation is ~3.3 ns/m), so their
 * hardware receive stamps for it differ only by their clock offset.
 *
 * Output: one text line per event on the USB serial console.
 *   A,<bssid>,<ap_tsf_us>,<rx_stamp_us>,<esp_us>,<rssi>,<channel>,<arrival_esp_us>,<soft>
 *   E,<src_mac>,<seq>,<rx_stamp_us>,<esp_us>,<rssi>,<arrival_esp_us>,<soft>
 * arrival_esp_us is the hardware receive stamp converted to esp_timer on the
 * device (radio clock read between two esp_timer reads, as in esp-pingpong);
 * soft=1 means the frame was older than 20 ms and the callback time is used.
 *   T,<seq>,<esp_us_before_send>
 *   G,<pin>,<level>,<esp_us_in_isr>
 *   P,<token>,<esp_us>            reply to "ping <token>" (host clock mapping)
 *   R,<src_mac>,<group>,<type>,<round>,<own_ns>,<heard_ns>,<with_mac>
 *                                 a ranging frame from ../range, as it came
 * Commands (one per line): "beacon on", "beacon off", "rate <hz>",
 *   "ch <n>", "ap on", "ap off", "gpio <pin>", "drive <pin> <hz>",
 *   "ping <token>", "id".
 * "drive" makes this device toggle <pin> itself (push-pull, still stamped by
 * its own interrupt), so a single wire to another device's "gpio" pin gives
 * both a shared physical edge: the ground truth for sync accuracy.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h" /* esp_wifi_internal_get_mac_clock_time */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "psa/crypto.h"
#include "esp_random.h"
#include "secret.h" /* run ./genkey.sh */
#include "proto.h"

ssm_beacon_t g_beacon_log[BEACON_LOG];
volatile uint32_t g_beacon_log_count;
portMUX_TYPE g_beacon_lock = portMUX_INITIALIZER_UNLOCKED;
volatile int g_have_anchor;
volatile int64_t g_anchor_last_us;
uint8_t g_anchor_mac[6];
uint32_t g_boot_id;
static psa_key_id_t s_key;
static volatile uint32_t s_bad_tags, s_replays;

bool make_tag(const void *data, size_t len, uint8_t out[TAG_LEN])
{
    uint8_t mac[32];
    size_t mac_len;
    if (psa_mac_compute(s_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), data, len, mac, sizeof mac, &mac_len) != PSA_SUCCESS) {
        return false;
    }
    memcpy(out, mac, TAG_LEN);
    return true;
}

bool tag_ok(const uint8_t *frame, size_t len)
{
    uint8_t expect[TAG_LEN], diff = 0;
    if (len <= TAG_LEN || !make_tag(frame, len - TAG_LEN, expect)) {
        return false;
    }
    for (int i = 0; i < TAG_LEN; i++) {
        diff |= expect[i] ^ frame[len - TAG_LEN + i];
    }
    return diff == 0;
}

static void crypto_init(void)
{
    static const uint8_t key[32] = SOUNDSYNC_KEY;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    ESP_ERROR_CHECK(psa_crypto_init() == PSA_SUCCESS && psa_import_key(&attr, key, sizeof key, &s_key) == PSA_SUCCESS
                        ? ESP_OK : ESP_FAIL);
    g_boot_id = esp_random();
}

/* Replay guard, one slot per beacon source (MAC): a frame must carry a newer
 * sequence number than the last one accepted from that source under the same
 * boot id; a new boot id restarts the sequence. Oldest slot is reused. */
#define REPLAY_SLOTS 16
static struct {
    uint8_t src[6];
    uint32_t boot, seq;
    int64_t used;
} s_replay[REPLAY_SLOTS];

static bool replay_accept(const uint8_t *src, uint32_t boot, uint32_t seq, int64_t now)
{
    int slot = -1, oldest = 0;
    for (int i = 0; i < REPLAY_SLOTS; i++) {
        if (s_replay[i].used && !memcmp(s_replay[i].src, src, 6)) {
            slot = i;
            break;
        }
        if (s_replay[i].used < s_replay[oldest].used) {
            oldest = i;
        }
    }
    if (slot >= 0 && s_replay[slot].boot == boot && seq <= s_replay[slot].seq) {
        return false;
    }
    if (slot < 0) {
        slot = oldest;
        memcpy(s_replay[slot].src, src, 6);
    }
    s_replay[slot].boot = boot;
    s_replay[slot].seq = seq;
    s_replay[slot].used = now;
    return true;
}

/* Fragment chunks heard over ESP-NOW, forwarded to USB as base64 lines
 * ("F,<src>,<b64>"): this is the Anchor-radio role. */
typedef struct {
    uint8_t src[6];
    uint16_t len;
    uint8_t data[SSF_HDR + SSF_CHUNK + TAG_LEN];
} fwd_t;
static QueueHandle_t s_fwd;
static volatile uint32_t s_fwd_drops, s_fwd_rx, s_fwd_out, s_out_drops, s_nacks_sent;

/* All lines the Anchor parses go out through one lock and the USB driver
 * directly, blocking up to 1 s for room instead of letting the console drop
 * characters when the host is slow. A line that still cannot be written is
 * counted (out_drops in the I line), never silently cut. */
static SemaphoreHandle_t s_out_lock;

static void emit(const char *buf, size_t len)
{
    xSemaphoreTake(s_out_lock, portMAX_DELAY);
    while (len) {
        int n = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(1000));
        if (n <= 0) {
            s_out_drops++;
            break;
        }
        buf += n;
        len -= n;
    }
    xSemaphoreGive(s_out_lock);
}

static void emitf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        emit(buf, n < (int)sizeof buf ? (size_t)n : sizeof buf - 1);
    }
}

#define SSB_MAGIC "SSB1"

/* ../range/main/main.c frame_t: "PP", u8 group, u8 type, u32 round,
 * i64 own_ns, i64 heard_ns, u8 with[6]. */
#define PP_LEN 30

typedef enum { EV_AP, EV_ESPNOW, EV_TX, EV_GPIO, EV_RANGE } ev_kind_t;

typedef struct {
    ev_kind_t kind;
    int64_t arrival; /* receive stamp on esp_timer */
    uint8_t soft;
    uint8_t mac[6];
    uint8_t with[6]; /* EV_RANGE: the other board of the round */
    uint64_t a;      /* AP TSF, beacon seq, or tx seq */
    uint32_t rx;     /* hardware receive stamp, microseconds (32-bit) */
    int64_t esp_us;  /* esp_timer at the callback / before send / in ISR */
    int8_t rssi;
    uint8_t channel;
    uint8_t level;
    uint8_t pin;
} ev_t;

static QueueHandle_t s_q;
static volatile bool s_beacon_on;
static volatile bool s_ap_log = true;
static volatile int s_rate_hz = 10;
static volatile uint8_t s_channel = 1;
static volatile uint32_t s_drops;
static const uint8_t BCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#define RX_STAMP_MAX_LATE_US 20000

/* Wi-Fi task: when the frame just handed to us arrived, on esp_timer. The radio
 * stamped it on its own clock; read that clock between two esp_timer reads
 * (closest of three tries) and subtract the frame's age. Adopted from
 * esp-pingpong (firmware/main/main.c arrival()). */
static int64_t arrival(uint32_t rx_ts, uint8_t *soft)
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

static void push(const ev_t *e)
{
    if (xQueueSend(s_q, e, 0) != pdTRUE) {
        s_drops++;
    }
}

/* Access point beacons, straight from the radio. Runs in the Wi-Fi task. */
static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    int64_t now = esp_timer_get_time();
    if (type != WIFI_PKT_MGMT || !s_ap_log) {
        return;
    }
    const wifi_promiscuous_pkt_t *p = buf;
    const uint8_t *f = p->payload;
    if (p->rx_ctrl.sig_len < 36 || f[0] != 0x80) { /* management, subtype 8: beacon */
        return;
    }
    ev_t e = {.kind = EV_AP, .rx = p->rx_ctrl.timestamp, .esp_us = now,
              .rssi = p->rx_ctrl.rssi, .channel = p->rx_ctrl.channel};
    e.arrival = arrival(e.rx, &e.soft);
    memcpy(e.mac, f + 16, 6); /* BSSID */
    uint64_t tsf = 0;
    for (int i = 7; i >= 0; i--) {
        tsf = (tsf << 8) | f[24 + i];
    }
    e.a = tsf;
    push(&e);
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    int64_t now = esp_timer_get_time();
    if (len > SSF_HDR + TAG_LEN && len <= SSF_HDR + SSF_CHUNK + TAG_LEN && memcmp(data, "SSF1", 4) == 0) {
        static fwd_t f;
        memcpy(f.src, info->src_addr, 6);
        f.len = (uint16_t)len;
        memcpy(f.data, data, len);
        if (xQueueSend(s_fwd, &f, 0) != pdTRUE) {
            s_fwd_drops++;
        } else {
            s_fwd_rx++;
        }
        return;
    }
#if CONFIG_IDF_TARGET_ESP32S3
    if (len >= SSN_HDR + TAG_LEN && memcmp(data, SSN_MAGIC, 4) == 0) {
        nack_received(info->src_addr, data, len);
        return;
    }
#endif
    /* Ranging frames carry no tag: they go to USB as they are, and the Anchor
     * treats the distances they give as unauthenticated. In ev_t: a = round,
     * pin = group, level = type, esp_us = own_ns, arrival = heard_ns. */
    if (len == PP_LEN && data[0] == 'P' && data[1] == 'P') {
        ev_t e = {.kind = EV_RANGE, .pin = data[2], .level = data[3]};
        uint32_t round;
        memcpy(e.mac, info->src_addr, 6);
        memcpy(&round, data + 4, 4);
        e.a = round;
        memcpy(&e.esp_us, data + 8, 8);
        memcpy(&e.arrival, data + 16, 8);
        memcpy(e.with, data + 24, 6);
        push(&e);
        return;
    }
    if (len != SSB_LEN || memcmp(data, SSB_MAGIC, 4) != 0) {
        return;
    }
    if (!tag_ok(data, len)) {
        s_bad_tags++;
        return;
    }
    ev_t e = {.kind = EV_ESPNOW, .rx = info->rx_ctrl->timestamp, .esp_us = now,
              .rssi = info->rx_ctrl->rssi};
    e.arrival = arrival(e.rx, &e.soft);
    memcpy(e.mac, info->src_addr, 6);
    uint32_t seq, boot;
    memcpy(&seq, data + 4, 4);
    memcpy(&boot, data + 16, 4);
    if (!replay_accept(info->src_addr, boot, seq, now)) {
        s_replays++;
        return;
    }
    e.a = seq;
    push(&e);
    /* Log it for fragments, and treat the beacon source as the Anchor. */
    ssm_beacon_t b = {.seq = seq, .rx = e.rx, .esp_us = e.arrival, .pad = e.soft};
    memcpy(b.src, info->src_addr, 6);
    portENTER_CRITICAL(&g_beacon_lock);
    g_beacon_log[g_beacon_log_count % BEACON_LOG] = b;
    g_beacon_log_count++;
    memcpy(g_anchor_mac, info->src_addr, 6);
    g_have_anchor = 1;
    g_anchor_last_us = now;
    portEXIT_CRITICAL(&g_beacon_lock);
}

static void forward_task(void *arg)
{
    (void)arg;
    static fwd_t f;
    static unsigned char b64[2 * (SSF_HDR + SSF_CHUNK + TAG_LEN) + 24];
    char mac[13];
    for (;;) {
        if (xQueueReceive(s_fwd, &f, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        size_t olen = 0;
        mbedtls_base64_encode(b64, sizeof b64, &olen, f.data, f.len);
        b64[olen] = 0;
        sprintf(mac, "%02x%02x%02x%02x%02x%02x", f.src[0], f.src[1], f.src[2], f.src[3], f.src[4], f.src[5]);
        /* F,<src>,<b64>,<n>: n counts forwarded lines, so the Anchor sees any
         * line lost between here and its parser as a gap. */
        uint32_t n = ++s_fwd_out;
        int k = (int)olen;
        k += snprintf((char *)b64 + k, sizeof b64 - k, ",%lu\n", (unsigned long)n);
        char head[16];
        int h = snprintf(head, sizeof head, "F,%s,", mac);
        xSemaphoreTake(s_out_lock, portMAX_DELAY);
        const char *parts[2] = {head, (const char *)b64};
        size_t lens[2] = {(size_t)h, (size_t)k};
        for (int i = 0; i < 2; i++) {
            const char *b = parts[i];
            size_t left = lens[i];
            while (left) {
                int w = usb_serial_jtag_write_bytes(b, left, pdMS_TO_TICKS(1000));
                if (w <= 0) {
                    s_out_drops++;
                    break;
                }
                b += w;
                left -= w;
            }
        }
        xSemaphoreGive(s_out_lock);
    }
}

static void beacon_task(void *arg)
{
    (void)arg;
    uint32_t seq = 0;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        int hz = s_rate_hz > 0 ? s_rate_hz : 1;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / hz));
        if (!s_beacon_on) {
            continue;
        }
        uint8_t msg[SSB_LEN];
        memcpy(msg, SSB_MAGIC, 4);
        seq++;
        memcpy(msg + 4, &seq, 4);
        int64_t t = esp_timer_get_time();
        memcpy(msg + 8, &t, 8);
        memcpy(msg + 16, &g_boot_id, 4);
        if (!make_tag(msg, SSB_LEN - TAG_LEN, msg + SSB_LEN - TAG_LEN)) {
            continue;
        }
        ev_t e = {.kind = EV_TX, .a = seq, .esp_us = t};
        if (esp_now_send(BCAST, msg, sizeof msg) == ESP_OK) {
            push(&e);
        }
    }
}

static int s_drive_pin = -1;
volatile int g_gpio_trigger; /* "gpiotrig on": an input edge forces a fragment */
extern void audio_force_from_isr(void);

static void IRAM_ATTR gpio_isr(void *arg)
{
    int pin = (int)(intptr_t)arg;
    ev_t e = {.kind = EV_GPIO, .esp_us = esp_timer_get_time(), .pin = (uint8_t)pin,
              .level = (uint8_t)gpio_get_level(pin)};
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_q, &e, &woken) != pdTRUE) {
        s_drops++;
    }
#if CONFIG_IDF_TARGET_ESP32S3
    if (g_gpio_trigger && pin != s_drive_pin) {
        audio_force_from_isr();
    }
#endif
    portYIELD_FROM_ISR(woken);
}

static int s_drive_level = 1;
static esp_timer_handle_t s_drive_timer;

static void drive_cb(void *arg)
{
    (void)arg;
    s_drive_level = !s_drive_level;
    gpio_set_level(s_drive_pin, s_drive_level);
}

static void attach_isr(int pin, gpio_mode_t mode)
{
    gpio_config_t c = {.pin_bit_mask = 1ULL << pin, .mode = mode,
                       .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&c);
    gpio_isr_handler_add(pin, gpio_isr, (void *)(intptr_t)pin);
}

static void hex6(char *out, const uint8_t *m)
{
    sprintf(out, "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void print_task(void *arg)
{
    (void)arg;
    ev_t e;
    char mac[13], with[13];
    for (;;) {
        if (xQueueReceive(s_q, &e, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (e.kind) {
        case EV_AP:
            hex6(mac, e.mac);
            emitf("A,%s,%llu,%lu,%lld,%d,%u,%lld,%u\n", mac, (unsigned long long)e.a, (unsigned long)e.rx,
                   (long long)e.esp_us, e.rssi, e.channel, (long long)e.arrival, e.soft);
            break;
        case EV_ESPNOW:
            hex6(mac, e.mac);
            emitf("E,%s,%llu,%lu,%lld,%d,%lld,%u\n", mac, (unsigned long long)e.a, (unsigned long)e.rx,
                   (long long)e.esp_us, e.rssi, (long long)e.arrival, e.soft);
            break;
        case EV_TX:
            emitf("T,%llu,%lld\n", (unsigned long long)e.a, (long long)e.esp_us);
            break;
        case EV_GPIO:
            emitf("G,%u,%u,%lld\n", e.pin, e.level, (long long)e.esp_us);
            break;
        case EV_RANGE:
            hex6(mac, e.mac);
            hex6(with, e.with);
            emitf("R,%s,%u,%u,%llu,%lld,%lld,%s\n", mac, e.pin, e.level, (unsigned long long)e.a,
                   (long long)e.esp_us, (long long)e.arrival, with);
            break;
        }
    }
}

static void set_channel(uint8_t ch)
{
    s_channel = ch;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

static void print_id(void)
{
    uint8_t m[6];
    char mac[13];
    esp_read_mac(m, ESP_MAC_WIFI_STA);
    hex6(mac, m);
    uint32_t ver = 0;
    esp_now_get_version(&ver);
    emitf("I,%s,ch=%u,beacon=%d,rate=%d,ap=%d,drops=%lu,fwd_drops=%lu,espnow_v=%lu,anchor=%d,boot=%08lx,bad_tags=%lu,"
          "replays=%lu,fwd_rx=%lu,fwd_out=%lu,out_drops=%lu,nacks=%lu\n",
          mac, s_channel, s_beacon_on, s_rate_hz, s_ap_log, (unsigned long)s_drops, (unsigned long)s_fwd_drops,
          (unsigned long)ver, g_have_anchor, (unsigned long)g_boot_id, (unsigned long)s_bad_tags,
          (unsigned long)s_replays, (unsigned long)s_fwd_rx, (unsigned long)s_fwd_out, (unsigned long)s_out_drops,
          (unsigned long)s_nacks_sent);
}

/* "nack <node mac hex> <node boot hex> <frag> <i,i,...>": ask a node to resend
 * the listed pieces of one fragment (SSN1, unicast, signed). The node's boot
 * id makes a request from before its last restart harmless. */
static void send_nack(const char *args)
{
    char mac_hex[13] = {0}, list[400] = {0};
    unsigned long boot = 0, frag = 0;
    if (sscanf(args, "%12s %lx %lu %399s", mac_hex, &boot, &frag, list) != 4) {
        return;
    }
    uint8_t dst[6];
    for (int i = 0; i < 6; i++) {
        unsigned v;
        if (sscanf(mac_hex + 2 * i, "%2x", &v) != 1) {
            return;
        }
        dst[i] = (uint8_t)v;
    }
    uint8_t msg[SSN_HDR + 2 * SSN_MAX + TAG_LEN];
    uint16_t n = 0;
    for (char *tok = strtok(list, ","); tok && n < SSN_MAX; tok = strtok(NULL, ",")) {
        uint16_t idx = (uint16_t)atoi(tok);
        memcpy(msg + SSN_HDR + 2 * n, &idx, 2);
        n++;
    }
    if (!n) {
        return;
    }
    uint32_t b = boot, fr = frag;
    memcpy(msg, SSN_MAGIC, 4);
    memcpy(msg + 4, &b, 4);
    memcpy(msg + 8, &fr, 4);
    memcpy(msg + 12, &n, 2);
    size_t len = SSN_HDR + 2 * n;
    if (!make_tag(msg, len, msg + len)) {
        return;
    }
    if (!esp_now_is_peer_exist(dst)) {
        esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peer.peer_addr, dst, 6);
        esp_now_add_peer(&peer);
    }
    if (esp_now_send(dst, msg, len + TAG_LEN) == ESP_OK) {
        s_nacks_sent++;
    }
}

static void cmd_task(void *arg)
{
    (void)arg;
    char line[512];
    for (;;) {
        if (!fgets(line, sizeof line, stdin)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        line[strcspn(line, "\r\n")] = 0;
        if (!strcmp(line, "beacon on")) {
            s_beacon_on = true;
        } else if (!strcmp(line, "beacon off")) {
            s_beacon_on = false;
        } else if (!strncmp(line, "rate ", 5)) {
            s_rate_hz = atoi(line + 5);
        } else if (!strncmp(line, "ch ", 3)) {
            set_channel((uint8_t)atoi(line + 3));
        } else if (!strcmp(line, "ap on")) {
            s_ap_log = true;
            esp_wifi_set_promiscuous(true);
        } else if (!strcmp(line, "ap off")) {
            /* also stop promiscuous receive: in a busy venue every management
             * frame would otherwise take a receive buffer from ESP-NOW */
            s_ap_log = false;
            esp_wifi_set_promiscuous(false);
        } else if (!strncmp(line, "nack ", 5)) {
            send_nack(line + 5);
            continue;
        } else if (!strcmp(line, "gpiotrig on")) {
            g_gpio_trigger = 1;
        } else if (!strcmp(line, "gpiotrig off")) {
            g_gpio_trigger = 0;
        } else if (!strncmp(line, "gpio ", 5)) {
            attach_isr(atoi(line + 5), GPIO_MODE_INPUT);
        } else if (!strncmp(line, "drive ", 6)) {
            int pin = -1, hz = 5;
            sscanf(line + 6, "%d %d", &pin, &hz);
            if (s_drive_timer) {
                esp_timer_stop(s_drive_timer);
            } else {
                esp_timer_create_args_t a = {.callback = drive_cb, .name = "drive"};
                esp_timer_create(&a, &s_drive_timer);
            }
            if (pin >= 0 && hz > 0) {
                s_drive_pin = pin;
                attach_isr(pin, GPIO_MODE_INPUT_OUTPUT);
                gpio_set_level(pin, s_drive_level);
                esp_timer_start_periodic(s_drive_timer, 1000000 / (2 * hz));
            }
#if CONFIG_IDF_TARGET_ESP32S3
        } else if (audio_command(line)) {
            continue;
#endif
        } else if (!strncmp(line, "ping ", 5)) {
            emitf("P,%s,%lld\n", line + 5, (long long)esp_timer_get_time());
            continue;
        }
        print_id();
    }
}

void app_main(void)
{
    crypto_init();
    s_q = xQueueCreate(256, sizeof(ev_t));
    s_fwd = xQueueCreate(24, sizeof(fwd_t));

    s_out_lock = xSemaphoreCreateMutex();
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.tx_buffer_size = 16384; /* about 8 forwarded pieces of slack */
    ucfg.rx_buffer_size = 1024;
    usb_serial_jtag_driver_install(&ucfg);
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    wcfg.nvs_enable = 0; /* leave the board's Wendy NVS alone */
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(promisc_cb);
    esp_wifi_set_promiscuous(true);
    set_channel(1);

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peer.peer_addr, BCAST, 6);
    esp_now_add_peer(&peer);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    xTaskCreate(print_task, "print", 4096, NULL, 5, NULL);
    xTaskCreate(beacon_task, "beacon", 3072, NULL, 10, NULL);
    xTaskCreate(cmd_task, "cmd", 4096, NULL, 4, NULL);
    xTaskCreate(forward_task, "fwd", 6144, NULL, 7, NULL);
#if CONFIG_IDF_TARGET_ESP32S3
    audio_start();
#endif
    print_id();
}
