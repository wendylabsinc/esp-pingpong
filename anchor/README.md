# soundsync Anchor

Receives fragments from soundsync-fw nodes through an ESP32 dongle (soundsync-fw,
beacon role) on USB, places every sample on one timeline and groups fragments of
the same sound into events. See the docstring in `anchor.py`.

```bash
# once: shared key for nodes, dongle and Anchor (writes secret.key here)
../sync-fw/genkey.sh
# on a laptop
python anchor.py /dev/cu.usbmodem101 fragments
# on a WendyOS device, dongle on ttyACM0
SOUNDSYNC_KEY=$(cat secret.key) wendy run --device <device>
```

HTTP needs `Authorization: Bearer <token>` on every request; the token is the
first 16 hex characters of HMAC-SHA256(key, "anchor-http") (see `anchor.py`).
Control endpoints are POST-only and allowlisted. `GET /stats` reports line
counts, fragments still missing pieces (with which pieces), base64 errors on
the USB link and the dongle's last `I` line, for diagnosing delivery loss.

The ESP-IDF USB-Serial-JTAG console gives up on output after a short timeout
when the host is not reading, so `usbserial.py` drains the endpoint on its own
thread and parsing runs separately. Pieces still went missing after this change, but not on USB: with numbered
lines the dongle's receive count, its forwarded count and the lines parsed here
matched exactly, so the loss is between the dongle radio's acknowledgement and
its receive callback (inferred from those counters). The Anchor asks the node
for the missing pieces (`nack`). Each line keeps the time of the USB
read that delivered it, so the USB ping map is not widened by parsing delays.

Output: `<OUT_DIR>/<node>/<anchor_us>-f<n>.wav|json` per fragment and
`<OUT_DIR>/events/<anchor_us>.wav|json` per event (one channel per node).
`clocks.py` (Line, band) is adopted from esp-pingpong's `box/taploc.py`.

## Mesh distances (`mesh.py`, `mesh.html`)

The ranging boards (`../range`) broadcast the two timestamps of every round they
finish. The dongle passes those frames to USB as `R` lines, `mesh.py` pairs the
two boards' frames of a round into a distance, and keeps the distance that most
of the last 15 rounds agree on (the rule in `range/main/agree.h`).

Open `http://<device>:8091/mesh#<token>`: boards as dots, distances as lines
drawn to scale. `GET /mesh.json` (with the token) is the same data.

```bash
python3 mesh.py              # self-check
python3 mesh.py demo         # made-up boards on http://127.0.0.1:8092/mesh, no token
```

- The dongle needs the firmware from this commit or later (`../sync-fw`), on the
  boards' channel (`PP_CHANNEL`, default 1, which is also the dongle's).
- `RANGE_TEMP_C` (default 20) and `RANGE_SELF_MM` (default 40) must match the
  boards' `PP_TEMP_C10` and `PP_SELF_MM`. Each adds straight onto every distance.
- Ranging frames carry no tag, so anything on the channel can forge a distance.
- **Not yet run with real boards**: checked with `mesh.py`'s self-check (the 22
  rounds recorded at 1270 mm, then three boards) and with `anchor.py` reading
  made-up `R` lines from a fake serial port.
- Every board needs the `range` firmware whose frames name the other board of
  the round (30 bytes); the two-board firmware's frames (24 bytes) are ignored.
- The Anchor is drawn as a square joined by dotted lines to the boards its radio
  hears. It has no buzzer or microphone in this scheme, so no distance.

## Accuracy, measured against a wire (2026-11-14)

A XIAO drove a GPIO wire into the Anchor's C5; both stamped every edge.

| Mapping | Edges | Bias | Std | p95 | Worst |
| --- | --- | --- | --- | --- | --- |
| Node to node, shared AP beacons, on-device arrival stamps | 1401 | +0.01 us | 0.45 us | 0.60 us | 8.4 us |
| Node to Anchor time, the C5's beacon send times (`place()`) | 1400 | +772 us | 241 us | 1.5 ms | 1.8 ms |

Absolute Anchor time is millisecond-grade (send queueing and airtime are not
observable from the sender); relative node timing is microsecond-grade.
`anchor_sigma_us` carries a 1 ms floor for that reason.
