# Ping-pong range

Two Seeed XIAO ESP32S3 Sense boards measure the distance between them by sound
and print it on their own consoles. No box, no network: a stand-alone version of
the sweep ranging in `../firmware`, using the same sweep detector (`chirp.h`,
copied from `../firmware/main`).

## How a round works

1. The board with the lower MAC (the initiator) sends an ESP-NOW "round r" frame
   and plays a sweep.
2. The other board (the responder) hears it and plays its own sweep
   `PP_REPLY_MS` (500 ms) after the frame arrived.
3. Each board timestamps both sweeps on its own microphone sample clock. The
   initiator has `a = heard - own`, the responder `b = own - heard`. Clock
   offsets cancel inside each, so `a - b = 2 * travel time - own buzzer-to-mic
   times`.
4. The boards swap their two timestamps over ESP-NOW and both print
   `d = c * (a - b) / 2 + PP_SELF_MM` for the round. The `DISTANCE` they print
   is the median of the largest group of the last 15 rounds that lie within
   100 mm of each other, with how many rounds are in that group. Until 3 rounds
   agree the line says `not settled` instead.

Each board only looks for the other's sweep when it can arrive: the responder
for 300 ms from the ping, the initiator from 20 ms before the reply is due.

The radio only says whose turn it is and carries the numbers; nothing is timed
by radio. Frames are not authenticated.

## Wiring

Piezo between **D0** (GPIO 1) and **D1** (GPIO 2). The firmware drives the two
pins in opposite phase (bridge drive, about 6 dB louder than one pin). With
`PP_BUZZER_GPIO2 = -1` it drives D0 alone and the piezo goes to GND instead.

## Build and deploy (WSL)

The boards need Wendy Lite with native app support. Attach each board to WSL
(`usbipd attach --wsl --busid <id>`), then once per board:

```sh
./push.sh
```

and pick the board in `wendy`'s picker. `wendy` does not accept `/dev/ttyACM0`
or a board name in `--device` for USB boards. The picker remembers the last row
used, so check the highlighted row before pressing Enter. Watch with
`wendy device logs`.

Do not use `idf.py flash`: it writes the bootloader and partition table and
erases Wendy Lite.

Each boot plays `PP_SELFTEST_S` (30) seconds of sweeps so the buzzer can be heard
and its wires checked; ranging starts after that.

## Settings (`main/Kconfig.projbuild`)

| Setting | Default | |
| --- | --- | --- |
| `PP_SELF_MM` | 40 | Buzzer to microphone, mm. A guess: measure it. Adds straight onto every distance. |
| `PP_TEMP_C10` | 200 | Air temperature, tenths of a degree C. |
| `PP_SWEEP_MS` | 64 | Sweep length. |
| `PP_DETECT_DB` | 10 | Detection threshold over background. |
| `PP_REPLY_MS` | 500 | Responder's wait before its sweep. |
| `PP_BUZZER_GPIO2` | 2 | Second pin for bridge drive, -1 for none. |
| `PP_SELFTEST_S` | 30 | Seconds of buzzer test at boot. |

## Measured so far (2026-11-14, two boards, one desk)

- With the 50 ms sweep, 13 dB threshold and single-pin drive, boards a measured
  distance apart read 512-531 mm over nine consecutive rounds (both boards
  printed the same values). The true spacing at that moment was not recorded.
- At about 54 cm, a later run gave two clusters, around 310-340 mm and
  500-520 mm. The cause is not established.
- At 140 cm, with bridge drive on one board only, each board detected the
  other's sweep in only 0-2 of about 7 per 10 s: 2 good rounds in 30 s (1332 mm,
  2753 mm). The 64 ms sweep, 10 dB threshold and narrow reply window in this
  version were made for that case.
- At 127 cm (tape measure), with this version and bridge drive on both boards,
  41 rounds were attempted in about 2.5 minutes. 10 read 1248-1331 mm (median
  1268 mm), 4 read 1479-2310 mm, 8 read 5.8-17.4 m and 19 were skipped as
  implausible. Every sweep from the other board was heard at 10-14 dB, just over
  the threshold. The printed median of 9 moved between 1268 and 1797 mm, because
  the wrong rounds get into it. `PP_SELF_MM = 40` was not changed.
- Loose buzzer wires made boards miss their own sweeps; tape or solder them.
