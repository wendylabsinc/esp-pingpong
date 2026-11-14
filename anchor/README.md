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

Output: `<OUT_DIR>/<node>/<anchor_us>-f<n>.wav|json` per fragment and
`<OUT_DIR>/events/<anchor_us>.wav|json` per event (one channel per node).
`clocks.py` (Line, band) is adopted from esp-pingpong's `box/taploc.py`.

## Accuracy, measured against a wire (2026-11-14)

A XIAO drove a GPIO wire into the Anchor's C5; both stamped every edge.

| Mapping | Edges | Bias | Std | p95 | Worst |
| --- | --- | --- | --- | --- | --- |
| Node to node, shared AP beacons, on-device arrival stamps | 1401 | +0.01 us | 0.45 us | 0.60 us | 8.4 us |
| Node to Anchor time, the C5's beacon send times (`place()`) | 1400 | +772 us | 241 us | 1.5 ms | 1.8 ms |

Absolute Anchor time is millisecond-grade (send queueing and airtime are not
observable from the sender); relative node timing is microsecond-grade.
`anchor_sigma_us` carries a 1 ms floor for that reason.
