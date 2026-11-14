/* ESP-NOW link of an acoustic node: signed time-sync beacons from the Anchor
 * dongle, and a reliable byte stream to the Anchor that Wendy Net uses as a
 * carrier (TLS runs over it, like over a BLE channel). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "wendy_pki.h"

/* Starts the link task. ESP-NOW itself starts once Wi-Fi is up. */
esp_err_t wendy_espnow_start(void);

/* Carrier side, called by the Wendy Net peer task only.
 * take: true when an Anchor is in range and no stream is open; opens one and
 *   returns the stream to run TLS over. The caller owns it until release.
 * release: resets the stream (the Anchor closes its relay) and frees it. */
bool wendy_espnow_take(struct wendy_pki_stream *stream);
void wendy_espnow_release(void);
/* True when everything written has been acknowledged by the Anchor. */
bool wendy_espnow_output_drained(void);
/* Called (from the Wi-Fi or link task) whenever the stream may make progress. */
void wendy_espnow_set_wake(void (*wake)(void));

/* Time sync: the newest beacons heard, newest first. Returns how many were
 * copied. seq is the Anchor's beacon number, arrival_us its hardware receive
 * time converted to esp_timer on this node. */
struct wendy_espnow_beacon {
    uint32_t seq;
    int64_t arrival_us;
};
int wendy_espnow_beacons(struct wendy_espnow_beacon *out, int max);
