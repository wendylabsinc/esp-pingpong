/* ESP-NOW link of an acoustic node: signed time-sync beacons from the Anchor
 * dongle, and a carrier for the signed mesh leaf: a local TCP port
 * (127.0.0.1:CONFIG_WENDY_ESPNOW_PROXY_PORT) whose bytes travel over a reliable
 * ESP-NOW stream to the Anchor. Point CONFIG_WENDY_MESH_ROUTER_HOST at it. */
#pragma once
#include <stdint.h>

#include "esp_err.h"

/* Starts the link and proxy tasks. ESP-NOW itself starts once Wi-Fi is up. */
esp_err_t wendy_espnow_start(void);

/* Time sync: the newest beacons heard, newest first. Returns how many were
 * copied. seq is the Anchor's beacon number, arrival_us its hardware receive
 * time converted to esp_timer on this node. */
struct wendy_espnow_beacon {
    uint32_t seq;
    int64_t arrival_us;
};
int wendy_espnow_beacons(struct wendy_espnow_beacon *out, int max);
