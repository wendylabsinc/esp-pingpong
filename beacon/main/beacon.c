/* Tap locator timing beacon: any ESP32 with Wi-Fi, no microphone needed.
 *
 * Two boards that hear the same broadcast hear it at the same instant, which
 * relates their clocks, but only if someone other than the two of them sent
 * it. This is that someone. It transmits the same signed beacon the boards
 * send each other (firmware/main/main.c) and does nothing else: it does not
 * listen, report or join a network. Put it in radio range of the boards.
 *
 *     idf.py set-target esp32c5 && idf.py -p PORT flash monitor
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "psa/crypto.h"

#if !__has_include("secret.h")
#error "No shared key yet: run tools/genkey.sh"
#endif
#include "secret.h"

#define CHANNEL 1     /* must match the boards' ESP-NOW channel (menuconfig -> Tap locator) */
#define PERIOD_MS 250 /* as the boards: the box fits a line through 20 s of these */
#define TAG_LEN 8

static const char *TAG = "beacon";

/* Wire format, as in firmware/main/main.c and box/taploc.py. */
typedef struct __attribute__((packed)) {
    char magic[2];
    uint8_t version, type;
    uint8_t node[6];
    uint32_t boot, seq;
    int64_t t_send;
} hdr_t;
_Static_assert(sizeof(hdr_t) == 26, "wire format must match box/taploc.py");

/* Counts boots in flash: the boards accept each sender's beacons only in
 * increasing (boot, seq) order, so a restart must not begin again from zero. */
static uint32_t next_boot(void)
{
    nvs_handle_t nvs;
    uint32_t boot = 0;
    if (nvs_open("taploc", NVS_READWRITE, &nvs) != ESP_OK) {
        return esp_random();
    }
    nvs_get_u32(nvs, "boot", &boot);
    boot++;
    nvs_set_u32(nvs, "boot", boot);
    nvs_commit(nvs);
    nvs_close(nvs);
    return boot;
}

void app_main(void)
{
    static const uint8_t everyone[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static const uint8_t key[32] = TAPLOC_KEY;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    esp_now_peer_info_t peer = {.ifidx = WIFI_IF_STA};
    memcpy(peer.peer_addr, everyone, sizeof everyone);
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    psa_key_id_t key_id;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
    psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    ESP_ERROR_CHECK(psa_crypto_init() == PSA_SUCCESS ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(psa_import_key(&attr, key, sizeof key, &key_id) == PSA_SUCCESS ? ESP_OK : ESP_FAIL);

    hdr_t h = {{'T', 'L'}, 2, 1 /* beacon */, {0}, next_boot(), 0, 0};
    esp_read_mac(h.node, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "beacon " MACSTR " boot %lu on channel %d", MAC2STR(h.node), (unsigned long)h.boot, CHANNEL);

    uint32_t sent = 0, failed = 0;
    for (;;) {
        uint8_t frame[sizeof h + TAG_LEN], mac[32];
        size_t mac_len;
        h.seq++;
        h.t_send = esp_timer_get_time();
        memcpy(frame, &h, sizeof h);
        if (psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256), frame, sizeof h, mac, sizeof mac,
                            &mac_len) == PSA_SUCCESS) {
            memcpy(frame + sizeof h, mac, TAG_LEN);
            if (esp_now_send(everyone, frame, sizeof frame) == ESP_OK) {
                sent++;
            } else {
                failed++;
            }
        }
        if (h.seq % 240 == 0) {
            ESP_LOGI(TAG, "%lu beacons sent, %lu failed", (unsigned long)sent, (unsigned long)failed);
        }
        /* Jittered so that it does not keep colliding with a board's beacons. */
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS + esp_random() % (PERIOD_MS / 4)));
    }
}
