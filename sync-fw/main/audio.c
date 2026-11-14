/*
 * XIAO ESP32S3 Sense microphone: 32 kHz capture, multi-band trigger, and
 * fragments sent to the Anchor over ESP-NOW.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_pdm.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "proto.h"

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
static int16_t *s_ring;
static ssm_stamp_t s_stamps[STAMPS];
static volatile int64_t s_n; /* samples captured so far */
static band_t s_bands[MAX_BANDS];
static int s_nbands;
static int s_mode_all;           /* 0: any band, 1: all bands */
static int s_need_blocks = 3;    /* consecutive blocks over threshold */
static int s_pre_ms = 500, s_post_ms = 1500;
static volatile int s_force;
static volatile int s_enabled = 1;
static volatile int s_print_levels;
static uint32_t s_frag_id;
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
    /* stamps covering the fragment */
    ssm_stamp_t st[STAMPS];
    int ns = 0;
    for (int i = 0; i < STAMPS; i++) {
        const ssm_stamp_t *s = &s_stamps[i];
        if (s->esp_us && s->n >= first_n - BLOCK && s->n < first_n + n_samples) st[ns++] = *s;
    }
    /* beacons heard in the last 20 s */
    static ssm_beacon_t bc[BEACON_LOG];
    int nb = 0;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&g_beacon_lock);
    uint32_t total = g_beacon_log_count;
    uint32_t avail = total < BEACON_LOG ? total : BEACON_LOG;
    for (uint32_t k = 0; k < avail; k++) {
        const ssm_beacon_t *b = &g_beacon_log[(total - 1 - k) % BEACON_LOG];
        if (now - b->esp_us > 20000000) break;
        bc[nb++] = *b;
    }
    portEXIT_CRITICAL(&g_beacon_lock);

    size_t len = sizeof(ssm_head_t) + s_nbands * sizeof(ssm_band_t) + ns * sizeof(ssm_stamp_t) +
                 nb * sizeof(ssm_beacon_t) + n_samples * 2;
    uint8_t *b = heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (!b) return 0;
    ssm_head_t h = {.magic = {'S', 'S', 'M', '1'}, .n_bands = s_nbands, .frag_id = frag_id, .rate = RATE,
                    .first_n = first_n, .n_samples = n_samples, .trigger_n = trig_n, .bands_hit = hit,
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
    for (uint32_t i = 0; i < n_samples; i++) {
        int16_t v = s_ring[(first_n + i) % RING];
        memcpy(p, &v, 2); p += 2;
    }
    *out = b;
    return len;
}

static int send_blob(const uint8_t *blob, size_t len, uint32_t frag_id)
{
    const uint8_t *dst = g_anchor_mac;
    if (!esp_now_is_peer_exist(dst)) {
        esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peer.peer_addr, dst, 6);
        esp_now_add_peer(&peer);
    }
    uint16_t count = (len + SSF_CHUNK - 1) / SSF_CHUNK;
    uint8_t pkt[SSF_HDR + SSF_CHUNK + TAG_LEN];
    int failed = 0, run = 0;
    for (uint16_t i = 0; i < count && run < 3; i++) {
        size_t off = (size_t)i * SSF_CHUNK, n = len - off < SSF_CHUNK ? len - off : SSF_CHUNK;
        memcpy(pkt, "SSF1", 4);
        memcpy(pkt + 4, &frag_id, 4);
        memcpy(pkt + 8, &i, 2);
        memcpy(pkt + 10, &count, 2);
        uint32_t bl = len;
        memcpy(pkt + 12, &bl, 4);
        memcpy(pkt + 16, &g_boot_id, 4);
        memcpy(pkt + SSF_HDR, blob + off, n);
        if (!make_tag(pkt, SSF_HDR + n, s_own_mac, pkt + SSF_HDR + n)) {
            failed++;
            continue;
        }
        int ok = 0;
        for (int attempt = 0; attempt < 4 && !ok; attempt++) {
            if (esp_now_send(dst, pkt, SSF_HDR + n + TAG_LEN) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            if (xSemaphoreTake(s_sent, pdMS_TO_TICKS(200)) == pdTRUE &&
                s_send_status == ESP_NOW_SEND_SUCCESS) {
                ok = 1;
            }
        }
        if (!ok) {
            failed++;
            run++;
        } else {
            run = 0;
        }
    }
    return run >= 3 ? -1 : failed;
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
typedef struct { uint8_t *blob; size_t len; uint32_t id; int tries; } outbox_t;

static void sender_task(void *arg)
{
    (void)arg;
    static outbox_t box[OUTBOX];
    int head = 0, n = 0;
    job_t j;
    for (;;) {
        /* take new jobs; don't block while something waits to go out */
        while (xQueueReceive(s_jobs, &j, n ? 0 : pdMS_TO_TICKS(500)) == pdTRUE) {
            uint8_t *blob = NULL;
            size_t len = build_blob(&blob, j.id, j.first, j.ns, j.trig, j.hit);
            if (!blob) {
                printf("W,no memory for fragment %lu\n", (unsigned long)j.id);
                continue;
            }
            if (n == OUTBOX) {
                printf("W,outbox full, dropped fragment %lu\n", (unsigned long)box[head].id);
                free(box[head].blob);
                head = (head + 1) % OUTBOX;
                n--;
            }
            box[(head + n) % OUTBOX] = (outbox_t){blob, len, j.id, 0};
            n++;
        }
        if (!n) continue;
        if (!anchor_fresh()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        outbox_t *o = &box[head];
        int64_t t0 = esp_timer_get_time();
        int r = send_blob(o->blob, o->len, o->id);
        o->tries++;
        printf("S,%lu,%u,%u,%d,%lld,try=%d,queued=%d\n", (unsigned long)o->id, (unsigned)o->len,
               (unsigned)((o->len + SSF_CHUNK - 1) / SSF_CHUNK), r, (long long)(esp_timer_get_time() - t0), o->tries, n - 1);
        if (r >= 0 || o->tries >= 20) {
            free(o->blob);
            head = (head + 1) % OUTBOX;
            n--;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void audio_task(void *arg)
{
    (void)arg;
    int16_t blk[BLOCK];
    int stamp_i = 0;
    int pending = 0;          /* a trigger is waiting for its post-roll */
    int64_t trig_n = 0, holdoff_until = 0;
    uint32_t trig_hit = 0;
    int64_t last_print = 0;
    for (;;) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, blk, sizeof blk, &got, 200) != ESP_OK || got != sizeof blk) continue;
        int64_t now = esp_timer_get_time();
        int64_t n0 = s_n;
        s_stamps[stamp_i] = (ssm_stamp_t){.n = n0, .esp_us = now - (int64_t)BLOCK * 1000000 / RATE};
        stamp_i = (stamp_i + 1) % STAMPS;
        for (int i = 0; i < BLOCK; i++) s_ring[(n0 + i) % RING] = blk[i];
        s_n = n0 + BLOCK;

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
            /* the floor learns only from quiet blocks, so a long sound does not raise it */
            if (!pending && bd->hits == 0) bd->floor_p += FLOOR_ALPHA * (pw - bd->floor_p);
        }
        uint32_t all = s_nbands >= 32 ? 0xffffffffu : ((1u << s_nbands) - 1);
        int fire = s_nbands && (s_mode_all ? over == all : over != 0);
        if (s_force) { fire = 1; s_force = 0; over = 0; }
        if (s_print_levels && now - last_print > 250000) {
            last_print = now;
            printf("L,%lld", (long long)n0);
            for (int b = 0; b < s_nbands; b++) printf(",%.0f:%.1f/%.1f", s_bands[b].hz, s_bands[b].level_db, s_bands[b].floor_db);
            printf("\n");
        }
        if (s_enabled && fire && !pending && n0 >= holdoff_until) {
            pending = 1;
            trig_n = n0;
            trig_hit = over;
            printf("D,%lu,%lld,%lu\n", (unsigned long)(s_frag_id + 1), (long long)trig_n, (unsigned long)over);
        }
        if (pending && s_n >= trig_n + (int64_t)s_post_ms * RATE / 1000) {
            pending = 0;
            int64_t first = trig_n - (int64_t)s_pre_ms * RATE / 1000;
            if (first < 0) first = 0;
            uint32_t ns = (uint32_t)(s_n - first);
            if (ns > RING - BLOCK * 4) ns = RING - BLOCK * 4;
            job_t j = {.first = first, .trig = trig_n, .ns = ns, .hit = trig_hit, .id = ++s_frag_id};
            if (xQueueSend(s_jobs, &j, 0) != pdTRUE) printf("W,sender busy, fragment %lu dropped\n", (unsigned long)j.id);
            holdoff_until = s_n + RATE / 2;
        }
    }
}

int audio_command(const char *line)
{
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
    if (!strcmp(line, "levels on")) { s_print_levels = 1; return 1; }
    if (!strcmp(line, "levels off")) { s_print_levels = 0; return 1; }
    if (!strcmp(line, "audio on")) { s_enabled = 1; return 1; }
    if (!strcmp(line, "audio off")) { s_enabled = 0; return 1; }
    int v;
    if (sscanf(line, "pre %d", &v) == 1) { s_pre_ms = v; return 1; }
    if (sscanf(line, "post %d", &v) == 1) { s_post_ms = v; return 1; }
    if (sscanf(line, "blocks %d", &v) == 1) { s_need_blocks = v; return 1; }
    if (!strcmp(line, "bands")) {
        printf("B,mode=%s,blocks=%d,pre=%d,post=%d", s_mode_all ? "all" : "any", s_need_blocks, s_pre_ms, s_post_ms);
        for (int i = 0; i < s_nbands; i++) printf(",%.0fHz+%.1fdB", s_bands[i].hz, s_bands[i].thr_db);
        printf("\n");
        return 1;
    }
    return 0;
}

void audio_start(void)
{
    s_ring = heap_caps_calloc(RING, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    s_sent = xSemaphoreCreateBinary();
    esp_now_register_send_cb(send_cb);
    /* Default: broadband energy 10 dB over its floor. */
    s_bands[0] = (band_t){.hz = 0, .thr_db = 10};
    s_nbands = 1;

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
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
    s_jobs = xQueueCreate(4, sizeof(job_t));
    xTaskCreatePinnedToCore(audio_task, "audio", 8192, NULL, 12, NULL, 1);
    xTaskCreatePinnedToCore(sender_task, "sender", 8192, NULL, 6, NULL, 0);
}
