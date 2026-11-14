/* Ping-pong range: two Seeed XIAO ESP32S3 Sense boards, each with a piezo
 * buzzer on D0, measure the distance between them by sound. No box, no
 * network: both boards print the result on their console (`wendy run` shows it).
 *
 * One round:
 *   1. The board with the lower MAC (the initiator) tells the other by ESP-NOW
 *      "round r starts", then plays a 50 ms sweep on its buzzer.
 *   2. The other board (the responder) hears it and, REPLY_MS later, plays its
 *      own sweep.
 *   3. Every board timestamps, on its own microphone sample clock, its own
 *      sweep and the other's. The initiator has a = heard - own, the responder
 *      has b = own - heard. Clock offsets cancel inside each of them, so
 *      a - b = 2 * (travel time) - (own buzzer-to-mic times), whatever the two
 *      clocks say. The boards swap (own, heard) by ESP-NOW and both work out
 *      d = c * (a - b) / 2 + self.
 * Nothing is timed by radio; the radio only says whose turn it is and carries
 * the numbers. Unauthenticated: anyone on the channel can inject frames.
 */
#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_private/rmt.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "wendy_core.h"
#include "wendy_wifi.h"
#include "wendy_pki_crypto.h"
#if CONFIG_WENDY_BLE
#include "wendy_ble.h"
#endif
#if CONFIG_WENDY_NET_PEER
#include "wendy_net_peer.h"
#endif

#include "agree.h"
#include "chirp.h"

static const char *TAG = "range";

#define RATE 16000 /* the sweep detector in chirp.h is tuned for this */
#define BLOCK (RATE / 50)
#define HIST 16384
#define SWEEP_LEN (RATE * CONFIG_PP_SWEEP_MS / 1000)
#define SWEEP_HALVES ((CONFIG_PP_SWEEP_F0 + CONFIG_PP_SWEEP_F1) * CONFIG_PP_SWEEP_MS / 1000)
#define BOOT_QUIET_US 4000000 /* start-up tone and microphone settling */
#define PING_LEAD_US 20000    /* the initiator plays this long after its ping */
_Static_assert(SWEEP_LEN <= CHIRP_N / 2 && CONFIG_PP_SWEEP_F1 < RATE / 2 &&
               CONFIG_PP_SWEEP_F0 < CONFIG_PP_SWEEP_F1 && HIST >= 4 * CHIRP_N,
               "the sweep does not fit the detector: see chirp.h");

/* ── Wire format ──────────────────────────────────────────────────── */

enum { F_HELLO = 1, F_PING = 2, F_MEAS = 3 };

typedef struct __attribute__((packed)) {
    char magic[2]; /* "PP" */
    uint8_t group, type;
    uint32_t round;
    int64_t own_ns, heard_ns; /* F_MEAS only: on the sender's sample clock */
} frame_t;

typedef struct {
    uint8_t src[6];
    int64_t clock_us; /* when we got it, on our sample clock */
    frame_t f;
} rx_t;

enum { EV_OWN = 1, EV_HEARD = 2 };
typedef struct {
    uint8_t kind, epoch;
    uint32_t round; /* EV_OWN only: the round we played it for */
    int64_t t_ns;
    int16_t snr_db10;
} ev_t;

static uint8_t s_mac[6];
static QueueHandle_t s_ev, s_rx;
static atomic_uint s_played, s_heard_own, s_heard_other;

/* ── Microphone and sample clock (as in ../firmware) ──────────────── */

static i2s_chan_handle_t s_mic;
static portMUX_TYPE s_mic_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_isr_samples, s_dropped;
static int64_t s_anchor[16];
static unsigned s_anchor_n;
static int64_t s_written;
static uint8_t s_epoch; /* counts losses: times either side of one do not compare */
static int16_t *s_hist;
static TaskHandle_t s_chirp_task;

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

/* esp_timer time -> this board's sample clock, in microseconds. Only used to
 * order things within a second or so; the measurements are pure sample counts. */
static int64_t clock_us(void)
{
    int64_t anchor = INT64_MAX;
    portENTER_CRITICAL(&s_mic_lock);
    for (int i = 0; i < 16; i++) {
        if (s_anchor[i] < anchor) {
            anchor = s_anchor[i];
        }
    }
    portEXIT_CRITICAL(&s_mic_lock);
    int64_t t = esp_timer_get_time();
    return anchor == INT64_MAX ? t : t - anchor;
}

static esp_err_t mic_start(void)
{
    s_hist = heap_caps_malloc_prefer(HIST * sizeof(int16_t), 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                     MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(s_hist, ESP_ERR_NO_MEM, TAG, "no memory for the sample history");
    for (int i = 0; i < 16; i++) {
        s_anchor[i] = INT64_MAX;
    }
    /* The Sense expansion board's PDM microphone: CLK GPIO 42, DATA GPIO 41. */
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
    int64_t index = 0;

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
        portEXIT_CRITICAL(&s_mic_lock);
        if (dropped) {
            ESP_LOGW(TAG, "microphone overran, %lld samples lost", (long long)dropped);
            index += dropped;
        }
        for (size_t k = 0; k < bytes / sizeof block[0]; k++, index++) {
            s_hist[index % HIST] = block[k];
        }
        portENTER_CRITICAL(&s_mic_lock);
        s_written = index;
        portEXIT_CRITICAL(&s_mic_lock);
        if (s_chirp_task) {
            xTaskNotifyGive(s_chirp_task);
        }
    }
}

/* ── Sweeps: playing ours on request, timing everyone's ───────────── */

static rmt_channel_handle_t s_buzzer;
static rmt_encoder_handle_t s_copy;
static rmt_symbol_word_t s_sweep[SWEEP_HALVES / 2];
static portMUX_TYPE s_play_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_play_at; /* esp_timer time to play at, 0: nothing wanted */
static uint32_t s_play_round;

static void play_request(int64_t at_us, uint32_t round)
{
    portENTER_CRITICAL(&s_play_lock);
    s_play_at = at_us;
    s_play_round = round;
    portEXIT_CRITICAL(&s_play_lock);
}

/* Sample indexes: the detector only looks for sweeps that start in here. A
 * detection anywhere else cannot be the other board's sweep, and it would
 * leave the detector deaf (chirp.h's hold) when the real one arrives. */
static int64_t s_listen_from, s_listen_to = INT64_MAX;

static void listen_window(int64_t from_us, int64_t to_us)
{
    portENTER_CRITICAL(&s_play_lock);
    s_listen_from = from_us * RATE / 1000000;
    s_listen_to = to_us == INT64_MAX ? INT64_MAX : to_us * RATE / 1000000;
    portEXIT_CRITICAL(&s_play_lock);
}

static esp_err_t buzzer_start(void)
{
    static uint16_t half[SWEEP_HALVES];
    ESP_RETURN_ON_FALSE(chirp_edges(CONFIG_PP_SWEEP_F0, CONFIG_PP_SWEEP_F1, CONFIG_PP_SWEEP_MS, half,
                                    SWEEP_HALVES) == SWEEP_HALVES, ESP_FAIL, TAG, "sweep edges");
    for (int i = 0; i < SWEEP_HALVES / 2; i++) {
        s_sweep[i] = (rmt_symbol_word_t){
            .duration0 = half[2 * i], .level0 = 1, .duration1 = half[2 * i + 1], .level1 = 0};
    }
    rmt_tx_channel_config_t channel = {
        .gpio_num = CONFIG_PP_BUZZER_GPIO,
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
    /* A piezo is driven by voltage, and a bare pin at its default strength
     * (20 mA) is the limit of what the firmware can do for loudness. The
     * strongest setting is about 40 mA. */
    gpio_set_drive_capability(CONFIG_PP_BUZZER_GPIO, GPIO_DRIVE_CAP_3);
#if CONFIG_PP_BUZZER_GPIO2 >= 0
    /* Bridge drive: the same RMT signal also goes to a second pin, inverted in
     * the GPIO matrix, so the two pins are exactly opposite and the piezo sees
     * twice the voltage. (While idle the piezo sits at a steady 3.3 V, which a
     * piezo, being a capacitor, does not mind.) */
    int ch = -1;
    if (rmt_get_channel_id(buzzer, &ch) == ESP_OK && ch >= 0 && ch < 4) {
        gpio_reset_pin(CONFIG_PP_BUZZER_GPIO2);
        gpio_set_direction(CONFIG_PP_BUZZER_GPIO2, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(CONFIG_PP_BUZZER_GPIO2, RMT_SIG_OUT0_IDX + ch, true, false);
        gpio_set_drive_capability(CONFIG_PP_BUZZER_GPIO2, GPIO_DRIVE_CAP_3);
        ESP_LOGI(TAG, "bridge drive: buzzer between GPIO %d and GPIO %d", CONFIG_PP_BUZZER_GPIO,
                 CONFIG_PP_BUZZER_GPIO2);
    } else {
        ESP_LOGW(TAG, "bridge drive not available; driving GPIO %d alone", CONFIG_PP_BUZZER_GPIO);
    }
#endif
    s_buzzer = buzzer;
    return ESP_OK;
}

/* Back-to-back sweeps for a while, before anything else runs, so the buzzer can
 * be heard and its wires wiggled. */
static void buzzer_selftest(int seconds)
{
    static const rmt_transmit_config_t once = {};
    if (!s_buzzer || seconds <= 0) {
        return;
    }
    ESP_LOGI(TAG, "buzzer test: sweeps for %d s; ranging starts after that", seconds);
    for (int i = 0; i < seconds * 10; i++) {
        if (rmt_transmit(s_buzzer, s_copy, s_sweep, sizeof s_sweep, &once) == ESP_OK) {
            rmt_tx_wait_all_done(s_buzzer, 200);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "buzzer test done");
}

static void chirp_task(void *arg)
{
    chirp_t *c = arg;
    static int16_t block[CHIRP_N];
    static const rmt_transmit_config_t once = {};
    chirp_hit_t hits[4];
    int64_t base = -1, own_base = -1; /* own_base: where to look for the sweep we last played */
    uint32_t own_round = 0, found_round = 0;
    uint8_t epoch = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        portENTER_CRITICAL(&s_mic_lock);
        int64_t written = s_written;
        uint8_t now_epoch = s_epoch;
        portEXIT_CRITICAL(&s_mic_lock);
        portENTER_CRITICAL(&s_play_lock);
        int64_t listen_from = s_listen_from, listen_to = s_listen_to;
        portEXIT_CRITICAL(&s_play_lock);
        if (base < 0 || now_epoch != epoch || written - base > HIST - 2 * BLOCK) {
            base = written;
            epoch = now_epoch;
            own_base = -1;
            continue;
        }
        int found = 0;
        if (own_base >= 0 && written - own_base >= CHIRP_N) {
            for (int n = 0; n < CHIRP_N; n++) {
                block[n] = s_hist[(own_base + n) % HIST];
            }
            found = chirp_own(c, block, own_base, hits);
            found_round = own_round;
            own_base = -1;
        }
        for (;; base += chirp_hop(c)) {
            for (int i = 0; i < found; i++) {
                ev_t ev = {
                    .kind = hits[i].mine ? EV_OWN : EV_HEARD,
                    .epoch = epoch,
                    .round = hits[i].mine ? found_round : 0,
                    .t_ns = (int64_t)(hits[i].at * 1e9 / RATE),
                    .snr_db10 = (int16_t)(hits[i].snr_db * 10),
                };
                xQueueSend(s_ev, &ev, 0);
                atomic_fetch_add(hits[i].mine ? &s_heard_own : &s_heard_other, 1);
            }
            if (written - base < CHIRP_N) {
                break;
            }
            found = 0;
            if (base + chirp_hop(c) <= listen_from || base >= listen_to) {
                continue; /* nothing is expected in this block */
            }
            if (c->quiet_until < listen_from) {
                c->quiet_until = listen_from; /* nor in the start of this one */
            }
            for (int n = 0; n < CHIRP_N; n++) {
                block[n] = s_hist[(base + n) % HIST];
            }
            found = chirp_scan(c, block, base, hits, 4);
        }

        /* A sweep somebody asked for. */
        int64_t now = esp_timer_get_time(), play_at;
        uint32_t play_round;
        portENTER_CRITICAL(&s_play_lock);
        play_at = s_play_at;
        play_round = s_play_round;
        portEXIT_CRITICAL(&s_play_lock);
        if (!s_buzzer || !play_at || now < play_at || own_base >= 0) {
            continue;
        }
        /* The sample being captured right now. `written` is stale by however long
         * the scans above took, which would put our sweep outside the window
         * chirp_own() looks in. */
        int64_t at = clock_us() * RATE / 1000000;
        if (at < c->quiet_until + RATE / 10) {
            play_request(now + 50000, play_round); /* the other board's sweep is still ringing */
        } else if (rmt_transmit(s_buzzer, s_copy, s_sweep, sizeof s_sweep, &once) == ESP_OK) {
            chirp_played(c, at);
            own_base = at - RATE / 40; /* a little before, in case "now" runs late */
            own_round = play_round;
            play_request(0, 0);
            atomic_fetch_add(&s_played, 1);
        }
    }
}

/* ── Radio: whose turn it is, and the numbers ─────────────────────── */

static const uint8_t EVERYONE[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static bool s_radio;

static void on_radio(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    rx_t rx;
    if (len != (int)sizeof rx.f) {
        return;
    }
    memcpy(&rx.f, data, sizeof rx.f);
    if (rx.f.magic[0] != 'P' || rx.f.magic[1] != 'P' || rx.f.group != CONFIG_PP_GROUP ||
        !memcmp(info->src_addr, s_mac, 6)) {
        return;
    }
    memcpy(rx.src, info->src_addr, 6);
    rx.clock_us = clock_us();
    xQueueSend(s_rx, &rx, 0);
}

/* Both boards stay on one channel, off Wi-Fi. Wendy Lite only starts the radio
 * to join a network, so it is started here. */
static bool radio_tune(void)
{
    uint8_t channel = 0;
    wifi_second_chan_t second;
    wifi_ap_record_t ap;
    bool joined = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    if (joined || esp_wifi_get_channel(&channel, &second) != ESP_OK || channel != CONFIG_PP_CHANNEL) {
        wendy_wifi_set_station_enabled(false);
        esp_wifi_start();
        if (esp_wifi_set_channel(CONFIG_PP_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
            return false;
        }
    }
    wifi_ps_type_t ps;
    if (esp_wifi_get_ps(&ps) == ESP_OK && ps != WIFI_PS_NONE) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
    return true;
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

static void send_frame(uint8_t type, uint32_t round, int64_t own_ns, int64_t heard_ns)
{
    frame_t f = {{'P', 'P'}, CONFIG_PP_GROUP, type, round, own_ns, heard_ns};
    if (s_radio) {
        esp_now_send(EVERYONE, (const uint8_t *)&f, sizeof f);
    }
}

/* ── Rounds and the answer ────────────────────────────────────────── */

typedef struct {
    uint32_t id;
    bool own, heard, peer, sent, done;
    uint8_t epoch_own, epoch_heard;
    int16_t snr_db10;
    int64_t own_ns, heard_ns, peer_own_ns, peer_heard_ns;
} round_t;

#define SLOTS 8
#define AGREE_MM 100 /* rounds this close together count as agreeing; at 10-14 dB they came 83 mm apart */
#define AGREE_MIN 3  /* this many must agree before the distance is called settled */
static round_t s_rounds[SLOTS];
static int s_dist_mm[AGREE_KEEP];
static unsigned s_good, s_bad;
static int s_last_mm = INT32_MIN;

static round_t *slot(uint32_t id)
{
    round_t *r = &s_rounds[id % SLOTS];
    if (r->id != id) {
        memset(r, 0, sizeof *r);
        r->id = id;
    }
    return r;
}

static double speed_of_sound(void)
{
    return 331.3 + 0.606 * CONFIG_PP_TEMP_C10 / 10.0;
}

static void try_finish(round_t *r, bool initiator)
{
    if (r->own && r->heard && !r->sent) {
        r->sent = true;
        if (r->epoch_own != r->epoch_heard) {
            r->done = true;
            s_bad++;
            ESP_LOGW(TAG, "round %lu: the microphone lost samples during it; skipped", (unsigned long)r->id);
            return;
        }
        send_frame(F_MEAS, r->id, r->own_ns, r->heard_ns);
        vTaskDelay(pdMS_TO_TICKS(30));
        send_frame(F_MEAS, r->id, r->own_ns, r->heard_ns); /* broadcasts are not acknowledged */
    }
    if (!(r->own && r->heard && r->peer) || r->done) {
        return;
    }
    r->done = true;
    /* The initiator measured a = heard - own, the responder b = own - heard. */
    int64_t a = initiator ? r->heard_ns - r->own_ns : r->peer_heard_ns - r->peer_own_ns;
    int64_t b = initiator ? r->peer_own_ns - r->peer_heard_ns : r->own_ns - r->heard_ns;
    double d = speed_of_sound() * (a - b) * 1e-9 / 2 + CONFIG_PP_SELF_MM / 1000.0;
    if (a < 100000000 || a > 3000000000LL || b < 100000000 || b > 3000000000LL || d < 0 || d > 20) {
        s_bad++;
        ESP_LOGW(TAG, "round %lu: implausible (a %lld us, b %lld us); skipped", (unsigned long)r->id,
                 (long long)(a / 1000), (long long)(b / 1000));
        return;
    }
    int mm = (int)lround(d * 1000);
    s_dist_mm[s_good++ % AGREE_KEEP] = mm;
    int n = s_good < AGREE_KEEP ? (int)s_good : AGREE_KEEP, agree, apart;
    int best = agree_distance(s_dist_mm, n, AGREE_MM, &agree, &apart);
    if (agree >= AGREE_MIN) {
        s_last_mm = best;
    }
    ESP_LOGI(TAG, "%s %d mm   (%d of the last %d rounds agree, %d mm apart; this round %d mm; "
                  "a=%lld us b=%lld us; heard at %d.%d dB)",
             agree >= AGREE_MIN ? "DISTANCE" : "not settled, best guess", best, agree, n, apart, mm,
             (long long)(a / 1000), (long long)(b / 1000), r->snr_db10 / 10, abs(r->snr_db10 % 10));
}

static void proto_task(void *arg)
{
    static rx_t rx;
    static ev_t ev;
    uint8_t peer[6] = {0};
    bool have_peer = false;
    int64_t peer_seen = 0, next_hello = 0, next_round = BOOT_QUIET_US, next_status = 10000000,
            next_tune = 0;
    uint32_t round_counter = esp_random() & 0xffff0000u, cur_round = 0;
    int64_t window_from = 0, window_to = 0; /* our sample clock: the other's sweep must start in here */
    uint32_t ignored_heard = 0;

    ESP_LOGI(TAG, "board " MACSTR ", group %d, channel %d", MAC2STR(s_mac), CONFIG_PP_GROUP, CONFIG_PP_CHANNEL);

    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_tune) {
            next_tune = now + (s_radio ? 5000000 : 1000000);
            if (radio_tune() && !s_radio) {
                radio_start();
            }
        }
        bool initiator = have_peer && memcmp(s_mac, peer, 6) < 0;

        if (xQueueReceive(s_rx, &rx, pdMS_TO_TICKS(10))) {
            if (!have_peer || !memcmp(rx.src, peer, 6) || now - peer_seen > 6000000) {
                if (!have_peer || memcmp(rx.src, peer, 6)) {
                    memcpy(peer, rx.src, 6);
                    have_peer = true;
                    ESP_LOGI(TAG, "other board found: " MACSTR ", this board is the %s", MAC2STR(peer),
                             memcmp(s_mac, peer, 6) < 0 ? "initiator" : "responder");
                }
                peer_seen = now;
                initiator = memcmp(s_mac, peer, 6) < 0;
                if (rx.f.type == F_PING && !initiator && rx.f.round != cur_round) {
                    cur_round = rx.f.round;
                    /* The initiator plays PING_LEAD_US after its ping, so its
                     * sweep cannot get here before the ping did. */
                    window_from = rx.clock_us;
                    window_to = rx.clock_us + 300000;
                    listen_window(window_from, window_to);
                    play_request(now + CONFIG_PP_REPLY_MS * 1000LL, cur_round);
                } else if (rx.f.type == F_MEAS) {
                    round_t *r = slot(rx.f.round);
                    if (!r->peer && !r->done) {
                        r->peer = true;
                        r->peer_own_ns = rx.f.own_ns;
                        r->peer_heard_ns = rx.f.heard_ns;
                        try_finish(r, initiator);
                    }
                }
            }
        }

        while (xQueueReceive(s_ev, &ev, 0)) {
            if (ev.kind == EV_OWN) {
                round_t *r = slot(ev.round);
                if (!r->own) {
                    r->own = true;
                    r->own_ns = ev.t_ns;
                    r->epoch_own = ev.epoch;
                    try_finish(r, initiator);
                }
            } else if (cur_round && ev.t_ns / 1000 >= window_from && ev.t_ns / 1000 <= window_to) {
                round_t *r = slot(cur_round);
                if (!r->heard) {
                    r->heard = true;
                    r->heard_ns = ev.t_ns;
                    r->epoch_heard = ev.epoch;
                    r->snr_db10 = ev.snr_db10;
                    try_finish(r, initiator);
                }
            } else {
                ignored_heard++;
            }
        }

        now = esp_timer_get_time();
        if (s_radio && now >= next_hello) {
            next_hello = now + (have_peer ? 1500000 : 400000) + esp_random() % 200000;
            send_frame(F_HELLO, 0, 0, 0);
        }
        if (have_peer && now - peer_seen > 10000000) {
            ESP_LOGW(TAG, "lost the other board");
            have_peer = false;
            listen_window(0, INT64_MAX); /* so the status line still says what the microphone hears */
        }
        if (have_peer && initiator && now >= next_round) {
            next_round = now + CONFIG_PP_ROUND_MS * 900LL + esp_random() % (CONFIG_PP_ROUND_MS * 200);
            cur_round = ++round_counter;
            send_frame(F_PING, cur_round, 0, 0);
            play_request(now + PING_LEAD_US, cur_round);
            /* The reply is played REPLY_MS after the other board gets our ping
             * (later only if it has to wait for quiet), so it cannot get here
             * sooner than that. The 20 ms is for clock_us() being approximate. */
            int64_t start = clock_us();
            window_from = start + CONFIG_PP_REPLY_MS * 1000LL - 20000;
            window_to = start + CONFIG_PP_REPLY_MS * 1000LL + 450000;
            listen_window(window_from, window_to);
        }

        if (now >= next_status) {
            next_status = now + 10000000;
            unsigned played = atomic_exchange(&s_played, 0), own = atomic_exchange(&s_heard_own, 0),
                     other = atomic_exchange(&s_heard_other, 0);
            if (s_last_mm == INT32_MIN) {
                ESP_LOGI(TAG, "status: %s; %u sweeps played, %u heard back from our own buzzer, "
                              "%u from the other board (%lu unmatched); %u good rounds, %u skipped",
                         have_peer ? "other board in contact" : "NO other board heard on the radio", played,
                         own, other, (unsigned long)ignored_heard, s_good, s_bad);
            } else {
                ESP_LOGI(TAG, "status: last distance %d mm; %u good rounds, %u skipped; %u sweeps played (%u heard), "
                              "%u from the other board",
                         s_last_mm, s_good, s_bad, played, own, other);
            }
            if (played > 1 && !own) {
                ESP_LOGW(TAG, "the microphone hears none of our own sweeps: check the buzzer on GPIO %d and GND",
                         CONFIG_PP_BUZZER_GPIO);
            }
        }
    }
}

void app_main(void)
{
    /* Same start-up order as stock Wendy Lite, so the board stays manageable
     * and `wendy run` can keep redeploying to it. */
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

    /* From here on nothing aborts: a ranging fault must not take device
     * management down with it. */
    esp_read_mac(s_mac, ESP_MAC_WIFI_STA);
    s_ev = xQueueCreate(16, sizeof(ev_t));
    s_rx = xQueueCreate(8, sizeof(rx_t));
    chirp_t *detector = heap_caps_malloc_prefer(sizeof *detector, 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
                                                MALLOC_CAP_DEFAULT);
    if (!s_ev || !s_rx || !detector ||
        !chirp_init(detector, RATE, CONFIG_PP_SWEEP_F0, CONFIG_PP_SWEEP_F1, CONFIG_PP_SWEEP_MS,
                    CONFIG_PP_DETECT_DB)) {
        ESP_LOGE(TAG, "out of memory; ranging is off");
        return;
    }
    if (mic_start() != ESP_OK) {
        ESP_LOGE(TAG, "no microphone (is the Sense expansion board fitted?); nothing to measure");
        return;
    }
    if (buzzer_start() != ESP_OK) {
        ESP_LOGE(TAG, "buzzer on GPIO %d would not start; this board can only listen", CONFIG_PP_BUZZER_GPIO);
    }
    buzzer_selftest(CONFIG_PP_SELFTEST_S);
    xTaskCreatePinnedToCore(chirp_task, "pp_chirp", 4096, detector, 3, &s_chirp_task, 1);
    xTaskCreate(mic_task, "pp_mic", 4096, NULL, 6, NULL);
    xTaskCreate(proto_task, "pp_proto", 6144, NULL, 5, NULL);
}
