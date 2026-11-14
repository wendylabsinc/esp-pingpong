/* Tap locator board firmware: Seeed XIAO ESP32S3 Sense on Wendy Lite.
 *
 * The board measures and the box does the maths. Three measurements:
 *   1. It plays a short sweep on its buzzer now and then, and reports when its
 *      microphone hears sweeps: its own and other boards'. From those times
 *      alone the box gets the distance between boards and how their clocks
 *      relate, so the boards never talk to each other by radio.
 *   2. It watches the microphone and, on a tap, sends the samples around it.
 *   3. Unless switched off, it broadcasts a timing beacon over ESP-NOW and
 *      timestamps other boards' beacons as its radio receives them. Two
 *      boards that hear the same beacon hear it at the same instant, which
 *      relates their clocks without any buzzer. This keeps the radio awake.
 * Every time on the wire is on this board's sample clock: samples counted
 * since the microphone started.
 * Reports leave three ways at once: by UDP on the local network; as a
 * SensorLink channel, which Wendy carries over USB, Bluetooth or Wendy Net;
 * and over ESP-NOW to neighbouring boards, which pass them on by whichever of
 * the first two they have. One board with a way out serves all in radio range.
 * The wire format is defined in box/taploc.py. Every frame ends in a
 * truncated HMAC-SHA256 under the key from tools/genkey.sh.
 */
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/i2s_pdm.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_private/wifi.h" /* the radio's own clock, which is what stamps received frames */
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "psa/crypto.h"

#include "wendy_core.h"
#include "wendy_wifi.h"
#include "wendy_pki_crypto.h"
#if CONFIG_WENDY_BLE
#include "wendy_ble.h"
#endif
#if CONFIG_WENDY_NET_PEER
#include "wendy_net_peer.h"
#endif

#include "chirp.h"

#if !__has_include("secret.h")
#error "No shared key yet: run tools/genkey.sh"
#endif
#include "secret.h"

static const char *TAG = "taploc";

#define RATE CONFIG_TAPLOC_SAMPLE_RATE
#define BLOCK (RATE / 50) /* samples per 20 ms microphone buffer */
#define TAP_SAMPLES 512   /* sent per tap */
#define TAP_PRE 128       /* of which before the trigger, so the box sees the background */
#define HIST 16384        /* samples kept; a power of two, several detector blocks long */
#define TAG_LEN 8
#define MAX_HITS 32       /* sweeps per report */
#define MAX_OBS 40        /* beacons heard per report */
#define MAX_PEERS 8       /* boards in radio range */
#define MAX_FRAME 1400    /* the largest report, ours or one carried for a neighbour */
#define RADIO (CONFIG_TAPLOC_BEACON_MS > 0)
/* The radio stamps each frame on arrival, on its own clock, and the callback
 * runs a little later. A gap beyond this means the stamp cannot be trusted. */
#define RX_STAMP_MAX_LATE_US 20000
#define TAP_HOLD_US 300000 /* a tap waits this long, in case it turns out to be a sweep */
#define SWEEP_LEN (RATE * CONFIG_TAPLOC_SWEEP_MS / 1000)
#define SWEEP_HALVES ((CONFIG_TAPLOC_SWEEP_F0 + CONFIG_TAPLOC_SWEEP_F1) * CONFIG_TAPLOC_SWEEP_MS / 1000)
_Static_assert(SWEEP_LEN <= CHIRP_N / 2 && CONFIG_TAPLOC_SWEEP_F1 < RATE / 2 &&
               CONFIG_TAPLOC_SWEEP_F0 < CONFIG_TAPLOC_SWEEP_F1 && HIST >= 4 * CHIRP_N,
               "the sweep does not fit the detector: see chirp.h");

enum { T_BEACON = 1, T_OBS = 2, T_TAP = 3, T_ECHO = 4, T_CHIRPS = 5 };
#define HIT_MINE 1
#define RX_SOFT 1

typedef struct __attribute__((packed)) {
    char magic[2];
    uint8_t version, type;
    uint8_t node[6];
    uint32_t boot, seq;
    int64_t t_send;
} hdr_t;
typedef struct __attribute__((packed)) {
    uint32_t box_rtt_us;
    uint8_t count;
} chirps_hdr_t;
typedef struct __attribute__((packed)) {
    int64_t t_ns;
    int16_t snr_db10;
    uint8_t flags, epoch;
} hit_t;
typedef struct __attribute__((packed)) {
    int64_t t0;
    uint32_t rate;
    uint16_t count;
    uint8_t epoch;
} tap_hdr_t;
typedef struct __attribute__((packed)) {
    uint8_t node[6];
    uint32_t boot, seq;
    int64_t t_tx, t_rx;
    uint8_t flags;
} obs_t;
_Static_assert(sizeof(hdr_t) == 26 && sizeof(chirps_hdr_t) == 5 && sizeof(hit_t) == 12 &&
               sizeof(tap_hdr_t) == 15 && sizeof(obs_t) == 31, "wire format must match box/taploc.py");
#define ECHO_LEN (sizeof(hdr_t) + TAG_LEN)
#define BEACON_LEN ECHO_LEN

typedef struct {
    uint8_t frame[BEACON_LEN];
    int64_t t_rx;
    uint8_t flags;
} heard_t;
typedef struct {
    uint16_t len;
    uint8_t data[MAX_FRAME];
} carried_t;

typedef struct {
    int64_t first; /* sample index of samples[0] */
    uint8_t epoch;
    int16_t samples[TAP_SAMPLES];
} tap_t;

static psa_key_id_t s_key;
static uint8_t s_node[6];
static uint32_t s_boot, s_seq;
static QueueHandle_t s_hits, s_taps;
/* For the status line only. */
static volatile int32_t s_mic_level;
static volatile uint32_t s_tap_count;
static atomic_uint s_played, s_heard_own, s_heard_others, s_busy_us;
static atomic_int s_own_db, s_others_db; /* our last sweep, and the strongest of the others' */

/* ── Authentication ───────────────────────────────────────────────── */

static bool make_tag(const void *data, size_t len, uint8_t out[TAG_LEN])
{
    uint8_t mac[32];
    size_t mac_len;
    if (psa_mac_compute(s_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), data, len, mac, sizeof mac,
                        &mac_len) != PSA_SUCCESS) {
        return false;
    }
    memcpy(out, mac, TAG_LEN);
    return true;
}

static bool tag_ok(const uint8_t *frame, size_t len)
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

/* ── Microphone and sample clock ──────────────────────────────────── */

static i2s_chan_handle_t s_mic;
static portMUX_TYPE s_mic_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_isr_samples, s_dropped;
static int64_t s_anchor[16]; /* esp_timer time of sample 0, as each recent buffer saw it */
static unsigned s_anchor_n;
static int64_t s_written; /* samples in the history so far */
static uint8_t s_epoch;   /* counts losses: sample times either side of one do not compare */
static int16_t *s_hist;
static TaskHandle_t s_chirp_task;

/* Runs when the driver fills a buffer. The sample clock and esp_timer share one
 * crystal, so "now minus samples so far" is when sample 0 was captured. */
static bool on_block(i2s_chan_handle_t chan, i2s_event_data_t *event, void *ctx)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_mic_lock);
    s_isr_samples += event->size / sizeof(int16_t);
    s_anchor[s_anchor_n++ % 16] = now - s_isr_samples * 1000000 / RATE;
    portEXIT_CRITICAL_ISR(&s_mic_lock);
    return false;
}

static bool on_overflow(i2s_chan_handle_t chan, i2s_event_data_t *event, void *ctx)
{
    portENTER_CRITICAL_ISR(&s_mic_lock);
    s_dropped += event->size / sizeof(int16_t);
    portEXIT_CRITICAL_ISR(&s_mic_lock);
    return false;
}

/* esp_timer time -> this board's sample clock, in microseconds. Without a
 * microphone the sample clock is simply time since boot. */
static int64_t to_clock(int64_t t)
{
    /* A delayed interrupt only ever reads late, so the earliest recent anchor is
     * the truest. What bias remains is the same on every board and cancels. */
    int64_t anchor = INT64_MAX;
    portENTER_CRITICAL(&s_mic_lock);
    for (int i = 0; i < 16; i++) {
        if (s_anchor[i] < anchor) {
            anchor = s_anchor[i];
        }
    }
    portEXIT_CRITICAL(&s_mic_lock);
    return anchor == INT64_MAX ? t : t - anchor;
}

static int64_t clock_us(void)
{
    return to_clock(esp_timer_get_time());
}

/* Sample index being captured right now. */
static int64_t sample_now(void)
{
    return clock_us() * RATE / 1000000;
}

static esp_err_t mic_start(void)
{
    s_hist = heap_caps_malloc_prefer(HIST * sizeof(int16_t), 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                     MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(s_hist, ESP_ERR_NO_MEM, TAG, "no memory for the sample history");
    for (int i = 0; i < 16; i++) {
        s_anchor[i] = INT64_MAX;
    }
    /* PDM receive exists only on I2S0 on the S3. The pins are the Sense
     * expansion board's microphone, as in Wendy Lite's own driver. */
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.dma_frame_num = BLOCK;
    chan.dma_desc_num = 8;
    i2s_pdm_rx_config_t pdm = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.clk = GPIO_NUM_42, .din = GPIO_NUM_41},
    };
    i2s_event_callbacks_t callbacks = {.on_recv = on_block, .on_recv_q_ovf = on_overflow};
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, NULL, &s_mic), TAG, "i2s_new_channel");
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_mic, &pdm), TAG, "pdm rx mode");
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_mic, &callbacks, NULL), TAG, "callbacks");
    return i2s_channel_enable(s_mic);
}

static void mic_task(void *arg)
{
    static int16_t block[BLOCK];
    static tap_t tap;
    int64_t index = 0, pending = -1;
    int64_t holdoff = RATE;    /* skip the microphone's start-up transient */
    int32_t dc = 0, floor = 0; /* running mean and mean deviation, both times 256 */

    for (;;) {
        size_t bytes = 0;
        if (i2s_channel_read(s_mic, block, sizeof block, &bytes, portMAX_DELAY) != ESP_OK) {
            continue;
        }
        portENTER_CRITICAL(&s_mic_lock);
        int64_t dropped = s_dropped;
        s_dropped = 0;
        if (dropped) {
            s_epoch++;
        }
        uint8_t epoch = s_epoch;
        portEXIT_CRITICAL(&s_mic_lock);
        if (dropped) {
            ESP_LOGW(TAG, "microphone overran, %lld samples lost", (long long)dropped);
            index += dropped;
            pending = -1;
        }
        for (size_t k = 0; k < bytes / sizeof block[0]; k++, index++) {
            int16_t x = block[k];
            s_hist[index % HIST] = x;
            dc += ((int32_t)x * 256 - dc) >> 8;
            int32_t level = abs(x - (dc >> 8));
            if (pending < 0) {
                int32_t threshold = (floor >> 8) * CONFIG_TAPLOC_TRIGGER_RATIO;
                if (threshold < CONFIG_TAPLOC_TRIGGER_MIN) {
                    threshold = CONFIG_TAPLOC_TRIGGER_MIN;
                }
                if (index >= holdoff && level > threshold) {
                    pending = index;
                } else {
                    floor += (level * 256 - floor) >> 10;
                }
            } else if (index == pending + (TAP_SAMPLES - TAP_PRE) - 1) {
                tap.first = pending - TAP_PRE;
                tap.epoch = epoch;
                for (int n = 0; n < TAP_SAMPLES; n++) {
                    tap.samples[n] = s_hist[(tap.first + n) % HIST];
                }
                xQueueSend(s_taps, &tap, 0);
                holdoff = index + RATE / 2; /* let the ring die away before listening again */
                pending = -1;
            }
        }
        s_mic_level = floor >> 8;
        portENTER_CRITICAL(&s_mic_lock);
        s_written = index;
        portEXIT_CRITICAL(&s_mic_lock);
        if (s_chirp_task) {
            xTaskNotifyGive(s_chirp_task);
        }
    }
}

/* ── Sweeps: playing ours, hearing everyone's ─────────────────────── */

static rmt_channel_handle_t s_buzzer;
static rmt_encoder_handle_t s_copy;
static rmt_symbol_word_t s_sweep[SWEEP_HALVES / 2];

/* The buzzer gets a square wave whose pitch rises through the sweep; the RMT
 * peripheral times every edge in hardware. */
static esp_err_t buzzer_start(void)
{
#if CONFIG_TAPLOC_BUZZER_GPIO >= 0
    static uint16_t half[SWEEP_HALVES];
    ESP_RETURN_ON_FALSE(chirp_edges(CONFIG_TAPLOC_SWEEP_F0, CONFIG_TAPLOC_SWEEP_F1, CONFIG_TAPLOC_SWEEP_MS,
                                    half, SWEEP_HALVES) == SWEEP_HALVES, ESP_FAIL, TAG, "sweep edges");
    for (int i = 0; i < SWEEP_HALVES / 2; i++) {
        s_sweep[i] = (rmt_symbol_word_t){
            .duration0 = half[2 * i], .level0 = 1, .duration1 = half[2 * i + 1], .level1 = 0};
    }
    rmt_tx_channel_config_t channel = {
        .gpio_num = CONFIG_TAPLOC_BUZZER_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 96,
        .trans_queue_depth = 1,
    };
    rmt_copy_encoder_config_t copy = {};
    rmt_channel_handle_t buzzer;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel, &buzzer), TAG, "rmt channel");
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy, &s_copy), TAG, "rmt encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(buzzer), TAG, "rmt enable");
    /* Two seconds of steady 3 kHz at start-up: a buzzer that is wired right is
     * heard across the room, so a silent one is known before any measuring. */
    static rmt_symbol_word_t tone[48];
    for (int i = 0; i < 48; i++) {
        tone[i] = (rmt_symbol_word_t){.duration0 = 167, .level0 = 1, .duration1 = 166, .level1 = 0};
    }
    rmt_transmit_config_t beep = {.loop_count = 125};
    if (rmt_transmit(buzzer, s_copy, tone, sizeof tone, &beep) != ESP_OK) {
        ESP_LOGW(TAG, "could not play the start-up tone; sweeps are not affected");
    }
    s_buzzer = buzzer;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED; /* a board that only listens */
#endif
}

static void chirp_task(void *arg)
{
    chirp_t *c = arg;
    static int16_t block[CHIRP_N];
    static const rmt_transmit_config_t once = {};
    chirp_hit_t hits[4];
    int64_t base = -1, next_sweep = esp_timer_get_time() + 3000000 + esp_random() % 3000000;
    int64_t own_base = -1; /* where to look for the sweep we last played, or -1 */
    uint8_t epoch = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        portENTER_CRITICAL(&s_mic_lock);
        int64_t written = s_written;
        uint8_t now_epoch = s_epoch;
        portEXIT_CRITICAL(&s_mic_lock);
        if (base < 0 || now_epoch != epoch || written - base > HIST - 2 * BLOCK) {
            /* Starting, samples were lost, or we fell behind: begin afresh. */
            base = written;
            epoch = now_epoch;
            own_base = -1;
            continue;
        }
        int found = 0;
        if (own_base >= 0 && written - own_base >= CHIRP_N) {
            /* The sweep we played is in the history by now: time it. */
            for (int n = 0; n < CHIRP_N; n++) {
                block[n] = s_hist[(own_base + n) % HIST];
            }
            found = chirp_own(c, block, own_base, hits);
            own_base = -1;
        }
        for (;; base += chirp_hop(c)) {
            for (int i = 0; i < found; i++) {
                hit_t hit = {
                    .t_ns = (int64_t)(hits[i].at * 1e9 / RATE),
                    .snr_db10 = (int16_t)(hits[i].snr_db * 10),
                    .flags = hits[i].mine ? HIT_MINE : 0,
                    .epoch = epoch,
                };
                xQueueSend(s_hits, &hit, 0);
                atomic_fetch_add(hits[i].mine ? &s_heard_own : &s_heard_others, 1);
                if (hits[i].mine) {
                    atomic_store(&s_own_db, (int)hits[i].snr_db);
                } else if (hits[i].snr_db > atomic_load(&s_others_db)) {
                    atomic_store(&s_others_db, (int)hits[i].snr_db);
                }
            }
            if (written - base < CHIRP_N) {
                break;
            }
            for (int n = 0; n < CHIRP_N; n++) {
                block[n] = s_hist[(base + n) % HIST];
            }
            int64_t began = esp_timer_get_time();
            found = chirp_scan(c, block, base, hits, 4);
            atomic_fetch_add(&s_busy_us, (unsigned)(esp_timer_get_time() - began));
        }

        int64_t now = esp_timer_get_time();
        if (!s_buzzer || now < next_sweep) {
            continue;
        }
        int64_t at = sample_now();
        if (at < c->quiet_until + RATE / 10) {
            /* Someone else's sweep is still ringing: wait for quiet. */
            next_sweep = now + 100000 + esp_random() % 300000;
        } else if (rmt_transmit(s_buzzer, s_copy, s_sweep, sizeof s_sweep, &once) == ESP_OK) {
            chirp_played(c, at);
            own_base = at - RATE / 80; /* a little before, in case our idea of "now" runs late */
            atomic_fetch_add(&s_played, 1);
            /* Random gaps: the box tells boards apart by them, and boards
             * started together stop sweeping over each other. */
            next_sweep = now + CONFIG_TAPLOC_SWEEP_PERIOD_MS * 750LL +
                         esp_random() % (CONFIG_TAPLOC_SWEEP_PERIOD_MS * 500);
        }
    }
}

/* ── Reports over Wendy Net ───────────────────────────────────────── */

/* SensorLink has no channel type for opaque records, so the reports travel as
 * a 16-bit PCM "audio" channel that the wendy CLI and the WendyOS agent already
 * know how to carry. Each report is prefixed "TLF1" and its length. */
#define LINK_CHANNEL 1
static atomic_int s_link_client = -1; /* the subscribed client, or -1 */
static atomic_bool s_link_busy;       /* a frame is with the transport */
static uint8_t s_link_pending[4096], s_link_tx[sizeof s_link_pending];
static size_t s_link_pending_len;     /* net task only */

static void link_done(uint32_t channel)
{
    atomic_store(&s_link_busy, false);
}

/* Net task: add one report to the batch for the next SensorLink frame. */
static void link_queue(const uint8_t *frame, size_t len)
{
    size_t need = 6 + len + (len & 1);
    if (atomic_load(&s_link_client) < 0 || !len ||
        s_link_pending_len + need > sizeof s_link_pending) {
        return; /* nobody listening, or the listener is too slow: drop */
    }
    uint8_t *p = s_link_pending + s_link_pending_len;
    memcpy(p, "TLF1", 4);
    p[4] = len & 0xff;
    p[5] = len >> 8;
    memcpy(p + 6, frame, len);
    if (len & 1) {
        p[6 + len] = 0; /* pad to whole 16-bit samples */
    }
    s_link_pending_len += need;
}

/* Net task: hand the batch to the transport once the previous one has gone. */
static void link_flush(void)
{
    int client = atomic_load(&s_link_client);
    if (client < 0) {
        s_link_pending_len = 0;
        return;
    }
    if (!s_link_pending_len || atomic_exchange(&s_link_busy, true)) {
        return;
    }
    size_t len = s_link_pending_len;
    memcpy(s_link_tx, s_link_pending, len);
    s_link_pending_len = 0;
    if (wendy_core_sensor_stream_try_push(client, LINK_CHANNEL, s_link_tx, len, esp_timer_get_time(),
                                          link_done) != WendyComResult_WENDY_COM_RESULT_OK) {
        atomic_store(&s_link_busy, false);
    }
}

static enum wcom_sensor_link_result link_manifest(struct wcom_sensor_descriptor *out, size_t capacity,
                                                  size_t *count)
{
    *count = 0;
    if (capacity) {
        out[0] = (struct wcom_sensor_descriptor){
            .channel_id = LINK_CHANNEL,
            .input_id = LINK_CHANNEL,
            .name = "taploc",
            .format_kind = WCOM_SENSOR_FORMAT_AUDIO,
            .format.audio = {.codec = wendy_lite_sensorlink_AudioFormat_Codec_PCM_S16LE,
                             .sample_rate = 16000,
                             .channels = 1},
        };
        *count = 1;
    }
    return WCOM_SENSOR_LINK_OK;
}

static bool link_ids_ok(const uint32_t *ids, size_t count)
{
    return !count || (ids && count == 1 && ids[0] == LINK_CHANNEL);
}

static enum wcom_sensor_link_result link_subscribe(int client, const uint32_t *ids, size_t count)
{
    if (!link_ids_ok(ids, count)) {
        return WCOM_SENSOR_LINK_FAIL;
    }
    if (!count) {
        return WCOM_SENSOR_LINK_OK;
    }
    int nobody = -1;
    if (!atomic_compare_exchange_strong(&s_link_client, &nobody, client)) {
        return nobody == client ? WCOM_SENSOR_LINK_OK : WCOM_SENSOR_LINK_BUSY;
    }
    wendy_core_sensor_stream_begin(client, LINK_CHANNEL);
    return WCOM_SENSOR_LINK_OK;
}

static void link_closed(uint32_t channel)
{
}

static void link_drop(int client)
{
    int expected = client;
    if (atomic_compare_exchange_strong(&s_link_client, &expected, -1)) {
        wendy_core_sensor_stream_end(client, LINK_CHANNEL, link_closed);
    }
}

static enum wcom_sensor_link_result link_unsubscribe(int client, const uint32_t *ids, size_t count)
{
    if (!link_ids_ok(ids, count)) {
        return WCOM_SENSOR_LINK_FAIL;
    }
    if (count) {
        link_drop(client);
    }
    return WCOM_SENSOR_LINK_OK;
}

/* ── Radio between boards ─────────────────────────────────────────── */

static const uint8_t EVERYONE[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static QueueHandle_t s_heard, s_carried;
static bool s_radio;         /* ESP-NOW is up */
static atomic_bool s_uplink; /* we have a way out, so can carry reports for neighbours */
static atomic_uint s_radio_beacons, s_radio_soft, s_radio_carried, s_radio_failed;
static atomic_uint s_radio_late_min = UINT32_MAX, s_radio_late_max; /* callback after arrival, us */

/* Wi-Fi task: when the frame just handed to us arrived, on our sample clock.
 * The radio stamped it on its own clock; our times are on esp_timer's. Read
 * both clocks at one moment: the radio's between two readings of esp_timer,
 * taking the closest of three tries in case an interrupt lands in between. */
static int64_t arrival(const wifi_pkt_rx_ctrl_t *rx, uint8_t *flags)
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
    uint32_t late = radio_now - rx->timestamp;
    *flags = late < RX_STAMP_MAX_LATE_US ? 0 : RX_SOFT;
    if (!*flags) {
        if (late < atomic_load(&s_radio_late_min)) {
            atomic_store(&s_radio_late_min, late);
        }
        if (late > atomic_load(&s_radio_late_max)) {
            atomic_store(&s_radio_late_max, late);
        }
    }
    return to_clock(*flags ? now : now - late);
}

/* Wi-Fi task. Authentication waits for the net task. */
static void on_radio(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)BEACON_LEN || len > MAX_FRAME || data[0] != 'T' || data[1] != 'L' || data[2] != 2) {
        return;
    }
    if (data[3] == T_BEACON && len == (int)BEACON_LEN) {
        heard_t h;
        memcpy(h.frame, data, BEACON_LEN);
        h.t_rx = arrival(info->rx_ctrl, &h.flags);
        xQueueSend(s_heard, &h, 0);
    } else if ((data[3] == T_TAP || data[3] == T_CHIRPS || data[3] == T_OBS) && atomic_load(&s_uplink)) {
        static carried_t item; /* only ever this one task here */
        item.len = (uint16_t)len;
        memcpy(item.data, data, len);
        xQueueSend(s_carried, &item, 0);
    }
}

/* Accepts each sender's frames only in increasing (boot, seq) order, so a
 * recorded frame cannot be replayed later, nor a carried report sent on twice. */
typedef struct {
    uint8_t node[6];
    uint32_t boot, seq;
    bool used;
} last_t;

static bool fresh(last_t *peers, const hdr_t *h)
{
    for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].used && memcmp(peers[i].node, h->node, 6) != 0) {
            continue;
        }
        if (peers[i].used &&
            (h->boot < peers[i].boot || (h->boot == peers[i].boot && h->seq <= peers[i].seq))) {
            return false;
        }
        memcpy(peers[i].node, h->node, 6);
        peers[i].boot = h->boot;
        peers[i].seq = h->seq;
        peers[i].used = true;
        return true;
    }
    return false; /* ponytail: fixed table of 8 senders; grow MAX_PEERS for bigger arrays */
}

/* All boards must be on one channel. With a channel set in menuconfig the board
 * leaves its Wi-Fi network and stays there, so nothing about the place it is in
 * can put boards on different channels; Wendy Lite keeps the saved network but
 * stops trying to join it. With channel 0 the board stays on Wi-Fi and ESP-NOW
 * rides that network's channel. Returns whether the radio is where it should be. */
static bool radio_tune(bool joined)
{
#if CONFIG_TAPLOC_RADIO_CHANNEL > 0
    uint8_t channel = 0;
    wifi_second_chan_t second;
    if (joined || esp_wifi_get_channel(&channel, &second) != ESP_OK || channel != CONFIG_TAPLOC_RADIO_CHANNEL) {
        wendy_wifi_set_station_enabled(false);
        esp_wifi_start(); /* Wendy Lite only starts the radio to join a network */
        return esp_wifi_set_channel(CONFIG_TAPLOC_RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) == ESP_OK;
    }
    return true;
#else
    return joined;
#endif
}

static void radio_start(void)
{
    esp_now_peer_info_t peer = {.ifidx = WIFI_IF_STA};
    memcpy(peer.peer_addr, EVERYONE, sizeof EVERYONE);
    if (esp_now_init() != ESP_OK || esp_now_register_recv_cb(on_radio) != ESP_OK ||
        esp_now_add_peer(&peer) != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW would not start; will try again");
        esp_now_deinit();
        return;
    }
    s_radio = true;
}

/* ── Reports ──────────────────────────────────────────────────────── */

static int s_sock = -1;
static struct sockaddr_in s_box;

/* Stamps and signs a frame of ours. `frame` has room for the header before
 * `len` bytes of payload and for the tag after. Returns its length, or 0. */
static size_t seal(uint8_t *frame, uint8_t type, size_t len)
{
    hdr_t h = {{'T', 'L'}, 2, type, {0}, s_boot, ++s_seq, clock_us()};
    memcpy(h.node, s_node, sizeof h.node);
    memcpy(frame, &h, sizeof h);
    len += sizeof h;
    return make_tag(frame, len, frame + len) ? len + TAG_LEN : 0;
}

/* Sends a signed report, ours or a neighbour's, by every way out we have. */
static void deliver(const uint8_t *frame, size_t len)
{
    if (s_sock >= 0) {
        sendto(s_sock, frame, len, 0, (struct sockaddr *)&s_box, sizeof s_box);
    }
    link_queue(frame, len);
}

static void report(uint8_t *frame, uint8_t type, size_t len)
{
    len = seal(frame, type, len);
    if (!len) {
        return;
    }
    deliver(frame, len);
    /* And to the neighbours: one of them may reach the box when we cannot. */
    if (s_radio && esp_now_send(EVERYONE, frame, len) != ESP_OK) {
        atomic_fetch_add(&s_radio_failed, 1);
    }
}

static void net_task(void *arg)
{
    static uint8_t frame[MAX_FRAME];
    static hit_t batch[MAX_HITS];
    static obs_t obs[MAX_OBS];
    static tap_t tap;
    static carried_t carried;
    static last_t beacon_from[MAX_PEERS], report_from[MAX_PEERS];
    _Static_assert(sizeof frame >= sizeof(hdr_t) + sizeof(chirps_hdr_t) + sizeof batch + TAG_LEN &&
                   sizeof frame >= sizeof(hdr_t) + sizeof(tap_hdr_t) + sizeof tap.samples + TAG_LEN &&
                   sizeof frame >= sizeof(hdr_t) + 1 + sizeof obs + TAG_LEN &&
                   MAX_FRAME <= ESP_NOW_MAX_DATA_LEN_V2, "frame");
    uint8_t *payload = frame + sizeof(hdr_t);

    struct sockaddr_in fallback = {.sin_family = AF_INET, .sin_port = htons(CONFIG_TAPLOC_PORT)};
    fallback.sin_addr.s_addr = strlen(CONFIG_TAPLOC_BOX_HOST) ? inet_addr(CONFIG_TAPLOC_BOX_HOST)
                                                               : htonl(INADDR_BROADCAST);
    s_box = fallback;
    int64_t box_seen = 0, next_report = 0, next_status = 0, next_beacon = 0, tap_due = 0;
    uint32_t box_rtt = 0;
    size_t n_batch = 0, n_obs = 0;
    bool have_tap = false, obs_due = false;
    ESP_LOGI(TAG, "board " MACSTR " boot %lu", MAC2STR(s_node), (unsigned long)s_boot);

    for (;;) {
        /* Sleep on the socket so the box's echo is timed the moment it lands. */
        fd_set readable;
        FD_ZERO(&readable);
        if (s_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            FD_SET(s_sock, &readable);
            struct timeval wait = {.tv_usec = 20000};
            select(s_sock + 1, &readable, NULL, NULL, &wait);
        }
        int64_t now = esp_timer_get_time();
        if (s_sock >= 0 && FD_ISSET(s_sock, &readable)) {
            uint8_t echo[ECHO_LEN + 1];
            struct sockaddr_in from;
            socklen_t from_len = sizeof from;
            int got = recvfrom(s_sock, echo, sizeof echo, MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
            int64_t clock = clock_us();
            hdr_t e;
            memcpy(&e, echo, sizeof e);
            if (got == ECHO_LEN && e.type == T_ECHO && !memcmp(e.node, s_node, 6) &&
                e.boot == s_boot && clock > e.t_send && clock - e.t_send < 2000000 &&
                tag_ok(echo, ECHO_LEN)) {
                box_rtt = clock - e.t_send;
                s_box = from;
                box_seen = now;
            }
        }

        /* Beacons other boards sent: when our radio heard each one. */
        heard_t h;
        while (RADIO && xQueueReceive(s_heard, &h, 0)) {
            hdr_t b;
            memcpy(&b, h.frame, sizeof b);
            if (n_obs < MAX_OBS && memcmp(b.node, s_node, 6) != 0 && tag_ok(h.frame, BEACON_LEN) &&
                fresh(beacon_from, &b)) {
                obs_t *o = &obs[n_obs++];
                memcpy(o->node, b.node, 6);
                o->boot = b.boot;
                o->seq = b.seq;
                o->t_tx = b.t_send;
                o->t_rx = h.t_rx;
                o->flags = h.flags;
                atomic_fetch_add(&s_radio_beacons, 1);
                atomic_fetch_add(&s_radio_soft, h.flags & RX_SOFT);
            }
        }
        /* Reports other boards sent: pass on each one once, if we have a way out. */
        atomic_store(&s_uplink, box_rtt || atomic_load(&s_link_client) >= 0);
        while (RADIO && xQueueReceive(s_carried, &carried, 0)) {
            hdr_t r;
            memcpy(&r, carried.data, sizeof r);
            if (memcmp(r.node, s_node, 6) != 0 && tag_ok(carried.data, carried.len) && fresh(report_from, &r)) {
                deliver(carried.data, carried.len);
                atomic_fetch_add(&s_radio_carried, 1);
            }
        }
        if (s_radio && now >= next_beacon) {
            /* Jittered so that boards started together do not keep colliding. */
            next_beacon = now + CONFIG_TAPLOC_BEACON_MS * 1000LL + esp_random() % (CONFIG_TAPLOC_BEACON_MS * 250 + 1);
            size_t len = seal(frame, T_BEACON, 0);
            if (!len || esp_now_send(EVERYONE, frame, len) != ESP_OK) {
                atomic_fetch_add(&s_radio_failed, 1);
            }
        }

        if (!have_tap && xQueueReceive(s_taps, &tap, 0)) {
            have_tap = true;
            tap_due = now + TAP_HOLD_US;
        }
        hit_t hit;
        while (n_batch < MAX_HITS && xQueueReceive(s_hits, &hit, 0)) {
            batch[n_batch++] = hit;
            /* A sweep sets off the tap trigger too. The detector names it a
             * sweep a moment later, and then it is not a tap. */
            int64_t at = hit.t_ns / 1000 * RATE / 1000000;
            if (have_tap && hit.epoch == tap.epoch && at > tap.first - SWEEP_LEN &&
                at < tap.first + TAP_SAMPLES) {
                have_tap = false;
            }
        }
        if (have_tap && now >= tap_due) {
            tap_hdr_t th = {tap.first * 1000000 / RATE, RATE, TAP_SAMPLES, tap.epoch};
            memcpy(payload, &th, sizeof th);
            memcpy(payload + sizeof th, tap.samples, sizeof tap.samples);
            report(frame, T_TAP, sizeof th + sizeof tap.samples);
            s_tap_count++;
            have_tap = false;
            obs_due = true; /* the box wants the beacons around the tap with it */
            ESP_LOGI(TAG, "tap at sample %lld", (long long)tap.first);
        }
        if (n_obs && (obs_due || n_obs == MAX_OBS || now >= next_report)) {
            payload[0] = (uint8_t)n_obs;
            memcpy(payload + 1, obs, n_obs * sizeof obs[0]);
            report(frame, T_OBS, 1 + n_obs * sizeof obs[0]);
            n_obs = 0;
        }
        obs_due = false;

        link_flush();
        if (now < next_report && n_batch < MAX_HITS) {
            continue;
        }
        next_report = now + CONFIG_TAPLOC_REPORT_MS * 1000LL;
        wifi_ap_record_t ap;
        bool joined = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
        if (s_sock < 0 && joined) {
            int yes = 1;
            s_sock = socket(AF_INET, SOCK_DGRAM, 0);
            setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);
        }
        if (RADIO && radio_tune(joined)) {
            if (!s_radio) {
                radio_start();
            }
            /* A sleeping radio misses beacons and stamps the rest late. With
             * beacons switched off it is left to sleep between reports. */
            wifi_ps_type_t ps;
            if (esp_wifi_get_ps(&ps) == ESP_OK && ps != WIFI_PS_NONE) {
                esp_wifi_set_ps(WIFI_PS_NONE);
            }
        }
        if (now - box_seen > 4 * CONFIG_TAPLOC_REPORT_MS * 1000LL) { /* box gone quiet: look for it again */
            s_box = fallback;
            box_rtt = 0;
        }
        chirps_hdr_t ch = {box_rtt, (uint8_t)n_batch};
        memcpy(payload, &ch, sizeof ch);
        memcpy(payload + sizeof ch, batch, n_batch * sizeof batch[0]);
        report(frame, T_CHIRPS, sizeof ch + n_batch * sizeof batch[0]);
        n_batch = 0;

        if (now >= next_status) {
            unsigned played = atomic_exchange(&s_played, 0), own = atomic_exchange(&s_heard_own, 0);
            unsigned busy = atomic_exchange(&s_busy_us, 0) / 600000; /* percent of the minute */
            ESP_LOGI(TAG, "last minute: %u sweeps played, %u of them heard (last at %d dB), %u heard from others (strongest %d dB); detector used %u%% of a core; box by UDP %s (round trip %lu us), by SensorLink %s; microphone level %ld, %lu taps",
                     played, own, atomic_exchange(&s_own_db, 0), atomic_exchange(&s_heard_others, 0),
                     atomic_exchange(&s_others_db, 0), next_status ? busy : 0,
                     box_rtt ? "found" : "not found", (unsigned long)box_rtt,
                     atomic_load(&s_link_client) < 0 ? "not subscribed" : "subscribed",
                     (long)s_mic_level, (unsigned long)s_tap_count);
            if (RADIO) {
                unsigned soonest = atomic_exchange(&s_radio_late_min, UINT32_MAX);
                uint8_t channel = 0;
                wifi_second_chan_t second;
                esp_wifi_get_channel(&channel, &second);
                ESP_LOGI(TAG, "radio: %s on channel %u%s; %u beacons heard, %u of them without a hardware timestamp, handled %u to %u us after arrival; %u reports carried for neighbours; %u sends failed",
                         s_radio ? "up" : "not started", channel, joined ? " (joined to Wi-Fi)" : "",
                         atomic_exchange(&s_radio_beacons, 0), atomic_exchange(&s_radio_soft, 0),
                         soonest == UINT32_MAX ? 0 : soonest, atomic_exchange(&s_radio_late_max, 0),
                         atomic_exchange(&s_radio_carried, 0), atomic_exchange(&s_radio_failed, 0));
            }
            if (played > 2 && !own) {
                ESP_LOGW(TAG, "the microphone hears none of our own sweeps: check the buzzer on GPIO %d",
                         CONFIG_TAPLOC_BUZZER_GPIO);
            }
            next_status = now + 60000000;
        }
    }
}

/* Counts boots in flash, so the box can tell a restarted board from a replay. */
static uint32_t next_boot(void)
{
    nvs_handle_t nvs;
    uint32_t boot = 0;
    if (nvs_open("taploc", NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "no storage for the boot counter; the box may ignore this board until it restarts");
        return esp_random();
    }
    nvs_get_u32(nvs, "boot", &boot);
    boot++;
    if (nvs_set_u32(nvs, "boot", boot) != ESP_OK || nvs_commit(nvs) != ESP_OK) {
        ESP_LOGE(TAG, "could not save the boot counter");
    }
    nvs_close(nvs);
    return boot;
}

void app_main(void)
{
    /* Same start-up order as stock Wendy Lite, so the board stays manageable
     * and stays a Wendy Net peer. */
    static const struct wcom_sensor_link_delegate link = {
        .on_sensor_link_get_manifest = link_manifest,
        .on_sensor_link_subscribe = link_subscribe,
        .on_sensor_link_unsubscribe = link_unsubscribe,
        .on_sensor_link_disconnected = link_drop,
    };
    wendy_core_register_sensor_link_source(&link); /* must precede wendy_core_init() */
    ESP_ERROR_CHECK(wendy_pki_memory_init() ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(wendy_pki_crypto_init() ? ESP_FAIL : ESP_OK);
#if CONFIG_WENDY_NET_PEER
    bool boot_ble, boot_wifi;
    ESP_ERROR_CHECK(wendy_net_peer_boot_radios(&boot_ble, &boot_wifi));
#if CONFIG_WENDY_NET_SELECT_RADIO_AT_BOOT
    ESP_ERROR_CHECK(wendy_core_set_boot_radios(boot_ble, boot_wifi));
#endif
#endif
    ESP_ERROR_CHECK(wendy_core_init());
#if CONFIG_WENDY_BLE
    wendy_ble_power_init();
#endif
#if CONFIG_WENDY_NET_PEER
    ESP_ERROR_CHECK(wendy_net_peer_start());
#endif

    /* From here on nothing aborts: a tap locator fault must not take device
     * management down with it. */
    static const uint8_t key[32] = TAPLOC_KEY;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    if (psa_crypto_init() != PSA_SUCCESS || psa_import_key(&attr, key, sizeof key, &s_key) != PSA_SUCCESS) {
        ESP_LOGE(TAG, "could not load the shared key; tap locator is off");
        return;
    }
    esp_read_mac(s_node, ESP_MAC_WIFI_STA);
    s_boot = next_boot();
    s_hits = xQueueCreate(MAX_HITS, sizeof(hit_t));
    s_taps = xQueueCreate(2, sizeof(tap_t));
    s_heard = xQueueCreate(32, sizeof(heard_t));
    s_carried = xQueueCreate(3, sizeof(carried_t));
    /* The detector's tables are read on every pass, so they go in fast memory. */
    chirp_t *detector = heap_caps_malloc_prefer(sizeof *detector, 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
                                                MALLOC_CAP_DEFAULT);
    if (!s_hits || !s_taps || !s_heard || !s_carried || !detector ||
        !chirp_init(detector, RATE, CONFIG_TAPLOC_SWEEP_F0, CONFIG_TAPLOC_SWEEP_F1, CONFIG_TAPLOC_SWEEP_MS,
                    CONFIG_TAPLOC_SWEEP_DETECT_DB)) {
        ESP_LOGE(TAG, "out of memory; tap locator is off");
        return;
    }
    if (mic_start() == ESP_OK) {
        if (buzzer_start() != ESP_OK) {
            ESP_LOGW(TAG, "no buzzer: this board hears sweeps and taps but cannot be ranged");
        }
        xTaskCreatePinnedToCore(chirp_task, "taploc_chirp", 4096, detector, 3, &s_chirp_task, 1);
        xTaskCreate(mic_task, "taploc_mic", 4096, NULL, 6, NULL);
    } else {
        ESP_LOGE(TAG, "no microphone (is the Sense expansion board fitted?); nothing to measure");
    }
    xTaskCreate(net_task, "taploc_net", 8192, NULL, 5, NULL);
}
