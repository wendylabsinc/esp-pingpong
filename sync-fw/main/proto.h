#pragma once
/*
 * soundsync wire formats (ESP-NOW payloads), little-endian.
 *
 * Every frame ends in an 8-byte tag: HMAC-SHA256 under the shared key
 * (genkey.sh), truncated, over everything before it (scheme from esp-pingpong).
 *
 * SSB1 beacon (28 B): "SSB1", u32 seq, i64 sender esp_timer before send,
 *   u32 boot id (random per boot), tag.
 *
 * SSF1 fragment chunk: "SSF1", u32 frag_id, u16 idx, u16 count, u32 blob_len,
 *   u32 boot id, then up to SSF_CHUNK data bytes of the fragment blob, tag.
 *
 * Fragment blob = meta, then PCM (int16 mono at meta.rate):
 *   "SSM1", u8 node_mac[6], u16 n_bands, u32 frag_id, u32 rate,
 *   i64 first_n, u32 n_samples, i64 trigger_n, u32 bands_hit,
 *   u16 n_stamps, u16 n_beacons,
 *   n_bands   x { f32 hz, f32 thr_db, f32 level_db, f32 floor_db }
 *   n_stamps  x { i64 n, i64 esp_us }           block stamps (first sample of
 *                                               each 20 ms block, "now - dur")
 *   n_beacons x { u8 src[6], u16 soft, u32 seq, u32 rx_stamp, i64 arrival_esp_us }
 *     (arrival_esp_us: receive stamp converted to esp_timer on the node)
 *   PCM.
 */
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#define SSF_CHUNK 1400
#define SSF_HDR 20
#define TAG_LEN 8
#define SSB_LEN (20 + TAG_LEN)
#define MAX_BANDS 8

typedef struct __attribute__((packed)) {
    char magic[4];
    uint8_t node_mac[6];
    uint16_t n_bands;
    uint32_t frag_id;
    uint32_t rate;
    int64_t first_n;
    uint32_t n_samples;
    int64_t trigger_n;
    uint32_t bands_hit;
    uint16_t n_stamps;
    uint16_t n_beacons;
} ssm_head_t;

typedef struct __attribute__((packed)) {
    float hz, thr_db, level_db, floor_db;
} ssm_band_t;

typedef struct __attribute__((packed)) {
    int64_t n, esp_us;
} ssm_stamp_t;

typedef struct __attribute__((packed)) {
    uint8_t src[6];
    uint16_t pad;
    uint32_t seq;
    uint32_t rx;
    int64_t esp_us;
} ssm_beacon_t;

/* Beacon receive log, filled by the ESP-NOW receive callback (main.c). */
#define BEACON_LOG 512
extern ssm_beacon_t g_beacon_log[BEACON_LOG];
extern volatile uint32_t g_beacon_log_count; /* total ever written */
extern portMUX_TYPE g_beacon_lock;
extern volatile int g_have_anchor;
extern volatile int64_t g_anchor_last_us; /* esp_timer of the last valid beacon */
extern uint8_t g_anchor_mac[6];

/* main.c: authentication */
#include <stdbool.h>
#include <stddef.h>
bool make_tag(const void *data, size_t len, uint8_t out[TAG_LEN]);
bool tag_ok(const uint8_t *frame, size_t len);
extern uint32_t g_boot_id;

/* audio.c (targets with the XIAO PDM microphone) */
void audio_start(void);
int audio_command(const char *line); /* returns 1 if handled */
