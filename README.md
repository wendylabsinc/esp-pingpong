# Tap locator

Seeed XIAO ESP32S3 Sense boards, each with a piezo buzzer, work out where they
are relative to each other by sound. They then hear a tap through the air, and
a WendyOS box shows where it came from on a live map, with the uncertainty of
every number.

```
firmware/   board app (C, ESP-IDF, Wendy Lite native app)
beacon/     beacon-only app for any spare ESP32 (C, plain ESP-IDF)
box/        WendyOS app (Python, no dependencies): maths, UDP ingest, map page
tools/      genkey.sh
sync-fw/    sound-sync firmware for any ESP32: radio timing, mic fragments (C, plain ESP-IDF)
anchor/     WendyOS app for sync-fw: fragments in through a USB dongle, one timeline (Python)
range/      two boards measure the distance between them and print it, no box (C, Wendy Lite native app)
```

## How it works

- **Sweeps.** Every few seconds, at random, a board plays a 50 ms sweep from 2
  to 6 kHz on its buzzer. Every board listens for sweeps all the time and
  reports when each one arrived at its microphone: its own and everyone else's.
- **Distance.** Board A hears its own sweep at once and B's after the sound has
  crossed; B sees the reverse. Comparing the two gaps gives the travel time,
  whatever the two clocks say. Each board only counts its own samples.
- **Board clocks.** The same arrival times give how far apart the two clocks
  are and how fast they drift. No radio timestamp is involved, and the result
  is already on the clock the microphone samples are counted on.
- **Radio beacons.** Four times a second each board also broadcasts a small
  ESP-NOW beacon, and every board reports when its radio received the others'.
  Two boards that hear the same beacon hear it at the same instant, so the
  sender's delays drop out and the two clocks are related to a few
  microseconds, with no buzzer. It needs a third board to listen to. Two
  boards alone time the beacons they send each other: a board cannot stamp
  its own transmission, but most beacons leave after nearly the same wait, so
  the crowd of typical ones in each direction pins the offset. That takes
  about a minute of beacons to settle, and on real boards two independent
  halves of a 75 s recording agreed to about 20 us. With a third sender (a
  spare ESP32-C5 running `beacon/`), the same two boards' 206 shared beacons
  over 63 s lay within 1.5 us of one straight line, and the two-way estimate
  was 8 us from it. The box uses whichever of
  sweeps, beacons and calibration taps is tightest at the moment of each tap.
  Beacons give no distance.
- **No Wi-Fi needed.** By default each board leaves its Wi-Fi network and
  keeps its radio on one fixed ESP-NOW channel, so all boards are certain to
  share it. Reports reach the box through whichever board is on USB or Wendy
  Net.
- **Layout.** The box turns the distances into a map. Place two or three boards
  by hand to fix where the map sits and which way round it is; the sweeps place
  the rest. With none placed, the first board is the origin and the second
  lies along the x axis.
- **Taps.** A board that hears a sharp rise sends 512 samples around it. The
  box aligns the waveforms by cross-correlation, converts the arrival times to
  one clock and solves for the position. Three boards give a position on the
  plane; two give one along the line between them.
- **Without buzzers.** Place the boards by hand, press **Calibrate here** on
  one, tap right next to it four or five times over ten seconds or more, and
  press **Done**. Those taps relate the clocks, good to a few centimetres of
  sound travel, and the page shows the agreement loosening as the clocks
  drift. Calibrate again when it has grown too wide.
- **Error region.** Clock spread, pick error, board position error and the
  uncertainty in the speed of sound are propagated into a 95% ellipse.
- **Wall-clock time** comes from the box. The page shows the error bound the
  Linux kernel reports for its clock, or says so when the clock is not synced.
- **Security.** Every report carries an HMAC-SHA256 tag under a shared key, and
  (boot, sequence) must increase, so recorded frames cannot be replayed.

## Power and radio

Sweeps need no radio between boards: with the beacons switched off (beacon
time 0 in menuconfig) each board sends the box one small report every five
seconds and one per tap, and its Wi-Fi radio sleeps in between. With beacons
on, the radio stays awake to hear them, which costs tens of milliamps more.
The CPU runs at 160 MHz instead of 240.

The microphone must be clocked all the time, so the chip cannot sleep. Expect
tens of milliamps, not microamps: this is a design for a small battery pack or
USB power, not a coin cell.

Reports leave each board three ways at once, and the box drops duplicates:

- **UDP on the local network**, found by broadcast, when the board has joined
  Wi-Fi. This is the only path that measures board-to-box latency.
- **A SensorLink channel** (channel 1, "taploc"), which Wendy carries over USB,
  Bluetooth or Wendy Net. `box/link.py` reads it through the `wendy` CLI and
  feeds the box server. Use it for boards beyond the box's Wi-Fi.

- **ESP-NOW to neighbouring boards**, when beacons are on. A board that has
  either of the first two ways out passes on every report it hears, so one
  board on USB or on Wendy Net serves all the boards in radio range of it.
  This is one hop, and it is this app's own relay: Wendy Net itself still
  runs over Bluetooth or Wi-Fi.

Delay on any path does not matter: nothing is timed by when a report arrives.

## Setup

1. Wire a piezo disc or passive piezo buzzer between pad **D0** (GPIO 1) and
   **GND** on each board, a few centimetres from the microphone and the same on
   every board. A magnetic buzzer or a speaker needs a transistor.
2. Flash each board with the Wendy Lite variant `esp32s3_seeed_xiao_native`
   (`wendy install`). Nothing else is needed for the default set-up, where the
   boards talk over ESP-NOW and one of them is on USB. To use Wi-Fi or Wendy
   Net instead, join the board (`wendy device wifi connect --ssid <network>`,
   `wendy device wendy-net configure --participate=true ...`) and set the
   ESP-NOW channel to 0.
3. Make the shared key: `tools/genkey.sh`.
4. Deploy the board app to each board in turn: `cd firmware && wendy run`.
5. Deploy the box app: `cd box && TAPLOC_KEY=$(cat secret.key) wendy run`.
6. For boards the box cannot hear by UDP, bridge their reports through Wendy:
   `python3 box/link.py wendy-lite:<board> [...]` on the machine running the
   box server.
7. Open `http://<box>:8000`. Boards appear on the map within a minute of
   hearing each other. Measure **self_m**, the distance from buzzer to
   microphone, and set the air temperature. Place boards by hand where you know
   their position, or press **Use measured positions** and save.

Each board logs a line a minute (`wendy device logs`): sweeps played, how many
of its own it heard, how many from others, and how much of a core the detector
used. "Hears none of our own sweeps" means the buzzer is not making sound.

Without hardware, `cd box && TAPLOC_KEY=$(printf '00%.0s' $(seq 32)) TAPLOC_CONFIG=./config.json python3 server.py`
and `python3 test_taploc.py localhost` in a second terminal streams a simulated
array to the page.

A spare ESP32 of any kind makes a third sender for two boards: it needs no
microphone. `cd beacon && idf.py set-target <chip> && idf.py -p <port> flash`.
It uses the same key and must be on the boards' channel (`CHANNEL` at the top
of `beacon/main/beacon.c`).

Self-checks: `python3 box/test_taploc.py` for the box maths, and
`clang -O2 -o /tmp/test_chirp firmware/test_chirp.c -lm && /tmp/test_chirp`
for the sweep detector.

## Tuning

| Where | Setting | What it does |
| --- | --- | --- |
| page | `temp_c`, `temp_sigma_c` | Air temperature and how well you know it. Sets the speed of sound: 0.6 m/s per °C, which is 1.7 mm per metre of distance. |
| page | `self_m` | Buzzer to microphone on a board. An error here adds directly to every distance. |
| page | `x`, `y` | Boards placed by hand. A 1 cm error is worth 29 µs. |
| page | `bias_us` | Shifts one board's arrival time. Tap at a known spot and adjust until it lands there. |
| `idf.py menuconfig` → Tap locator | buzzer GPIO | -1 for a board that only listens. |
| same | sweep start, end, length | A wider sweep times more sharply; a longer one carries further. All boards must match. |
| same | time between sweeps | More sweeps follow the clocks more closely; fewer leave more silence. |
| same | sweep detection level | Lower hears sweeps from further away. |
| same | time between reports | Longer keeps the radio asleep longer. |
| same | time between beacons | 0 switches ESP-NOW off: no radio clock agreement, no carrying of reports, and the radio sleeps. |
| same | ESP-NOW channel | 1 to 13 takes the board off Wi-Fi onto that channel; 0 keeps it on Wi-Fi. All boards must match. |
| same | trigger ratio, trigger minimum | How far above the background a sample must be to count as a tap. |
| same | sample rate | 16 kHz is 2.1 cm of sound travel per sample. All boards must match. |
| same | box address | Set it on networks that block broadcast. |
| `sdkconfig.defaults` | CPU frequency | 80 MHz saves more if the logged detector load is under 25%. |

## Limits

- **Not yet run on boards.** The firmware builds and the detector and box maths
  pass their self-checks against simulated sound, but how far a 3.3 V piezo
  carries, and the current the board draws, have not been measured.
- Two boards must hear each other's sweeps to be ranged, and a tap is located
  only by boards that all hear the sweeps of one of them. The radio does not
  extend this: sound is the only ruler.
- Clock agreement is as good as the sweeps are loud. The page shows it per pair.
- **With the fixed ESP-NOW channel (the default) a board is off Wi-Fi**: it
  cannot reach the box by UDP or be deployed to over the network, only over
  USB. Set the channel to 0 in menuconfig to stay on Wi-Fi and use its
  channel instead; then all boards must be on the same access point, because
  a network with several can put them on different channels.
- Clocks set by calibration taps are only as good as the tap was close to the
  board, and they drift: expect to recalibrate every few minutes. Sweeps do
  neither.
- Positions are flat: the map has no height. Without three boards placed by
  hand it may be a mirror image of the room.
- Three microphones can admit two answers for a tap; the page marks the second
  as a dashed circle. Outside the array the error region grows quickly.
- One tap at a time: reports within 0.6 s are treated as the same tap. A board
  is deaf to other sweeps for a quarter of a second after each one it hears,
  so more than about eight boards in earshot will start to miss each other.
- A sound in line of sight is assumed. A sweep that only arrives by an echo
  reads long.
- Anyone on the network can read the page and change positions. The key
  protects the measurements, not the page.

## Sound sync: triggered audio fragments on one timeline (`sync-fw/`, `anchor/`)

A second, simpler pipeline for long audio. Nodes (XIAO ESP32S3 Sense) record
32 kHz audio and, when a sound crosses a per-frequency threshold
(`band 6700 18`: 6.7 kHz, 18 dB over its floor), send 1 s around it (`pre`, `post`) to an
Anchor. The Anchor is a WendyOS device with an ESP32 on USB as its radio
(`sync-fw` in dongle role; it broadcasts beacons and forwards what it hears).
Everything travels over ESP-NOW, signed with a shared key (`sync-fw/genkey.sh`).

- **Timestamps.** Every node logs the radio receive time of every beacon it
  hears, Wi-Fi access point beacons included (promiscuous mode), converted to
  `esp_timer` on the device (`arrival()`, from `firmware/`). Two nodes that
  heard the same beacon are related directly, as in `box/`. Access point
  beacons give that without a third ESP32 wherever an access point is in range.
- **Fragments.** 2 s at 32 kHz (the earlier default) is about 134 KB, sent as about 96 signed pieces
  of 1.4 KB to the Anchor (unicast, acknowledged, retried). A node keeps up to
  16 fragments in PSRAM while no Anchor is in range. Measured: about 85 KB/s,
  no lost pieces over about 20 fragments at a metre.
- **Anchor** (`anchor/anchor.py`): reassembles, maps samples to node time and
  node time to the Anchor's and to the other nodes', groups fragments of one
  sound into an event (one aligned WAV channel per node), serves it all over
  HTTP. `anchor/clocks.py` reuses `Line` and the densest-band fit from
  `box/taploc.py`. On WendyOS images without `cdc_acm` it reads the dongle
  through libusb (`anchor/usbserial.py`).
- **Mesh distances.** The dongle also passes on the frames the ranging boards
  (`range/`) broadcast, and the Anchor draws the boards and the distances
  between them at `/mesh`. See `anchor/README.md`.
- **Ground truth** (`sync-fw/tools/`): `drive <pin> <hz>` on one board and
  `gpio <pin>` on another put one electrical edge on both clocks.
  `gpio_truth.py` and `anchor_truth.py` compare those stamps with what the
  radio mappings predict.

Measured against that wire (XIAO ESP32S3 and ESP32-C5, 2026-11-14):

| Mapping | Edges | Bias | Std | p95 | Worst |
| --- | --- | --- | --- | --- | --- |
| Node to node, shared access point beacons, `arrival()` stamps (run 1) | 1201 | -0.33 us | 0.52 us | 1.04 us | 4.9 us |
| Node to node, same method, C5 on the WendyOS Anchor (run 2) | 1401 | +0.01 us | 0.45 us | 0.60 us | 8.4 us |
| Node to Anchor time, from the dongle's beacon send times | 1400 | +772 us | 241 us | 1.5 ms | 1.8 ms |

So node-to-node timing is good to about a microsecond with no per-chip
calibration, and absolute Anchor time to about 2 ms: a sender cannot stamp its
own transmission, and the queueing and airtime before it are not observable.
**Not yet measured:** how exactly a recorded sound's samples sit on the node's
clock (the microphone pipeline's fixed delay and its scatter).
`sync-fw/tools/audio_truth.py` measures it with contact taps next to the
microphone; until it has been run, assume only the node-to-node figures above.

### Audio timestamps and end-to-end alignment (2026-11-14)

A recorded sound's time, measured with a sound whose emission time is known:

- **On one node's clock.** The XIAO drives a buzzer 5 cm from its own
  microphone and stamps each chirp (`beeppin`, `chirp`, `beepevery`,
  `sync-fw/tools/beep_truth.py`). 27 chirps: a constant +354 us (microphone
  pipeline plus buzzer response plus distance), 3.1 us std, 7.5 us worst.
  Audio blocks are stamped in the I2S DMA interrupt.
- **Base station to node.** The WendyOS Anchor plays a pseudo-random code on a
  buzzer on its own header (Dragonwing IQ-9075: pin 33, TCA9538 at I2C bus 0
  address 0x3b, line 7) in one I2C write, so every level change is clocked by
  the controller (`anchor.py /dchirp?prn=<seed>`). It reaches the dongle's
  clock through USB pings. 6 codes recorded by a XIAO 5 cm away:
  -82 us mean, 139 us std, 377 us worst (`sync-fw/tools/e2e_truth.py`). A
  stepped tone did not work: it repeats within each step, so the correlation
  slips by whole cycles.
- **Node to node.** Two XIAOs record the same code; node B's clock is mapped
  onto node A's through the beacons both heard (`two_node_truth.py`). Two runs:
  10 of 10 codes with 10.2 us std (worst 22.5 us), then 9 of 10 with 8.5 us
  std plus one outlier at +92 us (132 us away from the others), possibly a
  one-chip correlation slip. The median (-53 us, then -41 us) is consistent
  with the nodes' different distances to the buzzer; the distances were not
  measured.

Known limit: after two nodes sent fragments at the same time, the Anchor was
left with 32 incomplete fragments (2,599 pieces waiting), while the dongle's
radio queue reported no drops. The likely cause is the base64 stream over the
ESP32's built-in USB serial console falling behind. The tests above staggered
the nodes' sends. Binary framing, a larger blocking USB buffer and
acknowledgements from the dongle are the fix.

### Three nodes (2026-11-14)

Three XIAOs about 5 cm from the Dragonwing's buzzer, 10 codes
(`sync-fw/tools/run_three.sh`). Node-to-node difference per code, B - A:

| Pair | Codes in both | Median | Std | Worst from median |
| --- | --- | --- | --- | --- |
| 4824, 4e58 | 4 | -161 us | 67 us | 150 us |
| 4e58, 4ff8 | 4 | +48 us | 13 us | 21 us |
| 4824, 4ff8 | 4 | -113 us | 62 us | 141 us |

The medians include the nodes' different distances to the buzzer (not
measured). For each code all three nodes caught, the three pairwise
differences sum to within 0.1 us of zero. Each node's code location enters all
of its pairs, so this shows the beacon fits agree with each other, not that
the code was located correctly in the audio. Both larger scatters come from one
code that node 4824 located about 150 us off at low correlation quality. Four
codes per pair is a small sample.

Delivery was the open problem: only 4 of 10 codes reached the Anchor from both
nodes of a pair. Changes, then a second run (r4, below):

- **Room noise.** At +12 dB, nodes kept triggering on room noise and filled
  the radio link; a 16 s level sample showed room peaks of about +8 dB at
  1.8 kHz. The code reached +25 dB at 2.7 kHz on both nodes sampled but only +5
  to +10 dB at 1.8 kHz. Fragments from the third node, recorded with both bands
  at +12 dB, show which band fired: every code fragment on 2.7 kHz; room
  fragments on 1.8 kHz 21 times and on 2.7 kHz 11 times. At +18 dB one node
  caught only 1 of 4 codes. The firmware default is now 2.7 kHz only at +15 dB,
  with 1 s fragments (`pre 300`, `post 700`), half the airtime of 2 s ones.
- **Send slots.** Each node waits `slot x 1.8 s` (slot from its MAC address,
  `slot <n>` overrides) before its first send of a fragment, so nodes that heard
  the same sound do not all send at once.
- **Failed pieces are resent.** The node firmware skipped a piece that failed
  all 4 send attempts and still reported the fragment delivered. It now resends
  only the failed pieces, in up to 4 rounds (`R,<frag>,round=..,resent=..,
  still=..` on serial), and keeps the fragment queued until every piece is
  acknowledged. In r4 no node needed a resend, so this path is untested in the
  field.
- **Pieces still lost after the dongle acknowledged them.** In r4 every piece
  from every node was acknowledged on the first try (serial `S` lines), yet 2
  of 88 fragments reached the Anchor with holes (5 and 8 pieces). `GET /stats`
  counted 3 malformed base64 lines since the Anchor started; one line that
  loses its newline on the USB link swallows the next, so a few malformed
  lines can account for several pieces. The Anchor drains USB on its own
  thread (longest gap between reads 14 ms). The fix belongs in the dongle:
  binary framing, blocking USB writes, and acknowledgements from the Anchor.

#### Run r4: 20 codes after the changes

All three nodes on the new firmware, Anchor reached through Wendy Cloud, run
inside the Anchor container (`sync-fw/tools/run_on_anchor.py`). Codes caught:
18, 19 and 15 of 20. While codes played, each code's fragments arrived before
the next code (10 s apart). Node-to-node difference via shared beacons, the
path the Anchor's events use:

| Pair | Codes in both | Median | Std | Worst from median | Max - min |
| --- | --- | --- | --- | --- | --- |
| 4824, 4e58 | 18 | -512 us | 46 us | 167 us | 249 us |
| 4824, 4ff8 | 14 | -128 us | 97 us | 335 us | 376 us |
| 4e58, 4ff8 | 15 | +395 us | 125 us | 381 us | 404 us |

The nodes were rearranged between runs (cables moved), so the medians differ
from the first run. Within r4 the earliest 1 or 2 codes are off in every pair, and the
last 2 only in the two pairs with node 4ff8, by the same amount in both (about
130 to 160 us, about 5 cm of sound path), consistent with nodes being moved
during the run. Leaving those out, each pair spans 45 to 57 us (std 13 to
17 us, 11 to 16 codes). Including them, every difference stays within 405 us
of every other for the same pair, under the 1 ms target for the error between
nodes' detections of one sound.

#### Run r7: resend requests (dongle, node and Anchor changes)

- **Dongle:** every forwarded line carries a running number (`F,<src>,<b64>,<n>`),
  output goes through one lock straight to the USB driver and blocks up to 1 s
  instead of dropping, the I line reports `fwd_rx`, `fwd_out`, `out_drops` and
  `nacks`, Wi-Fi receive buffers went from 10 static and 32 dynamic to 16 and 64,
  and `ap off` also turns promiscuous receive off.
- **Nodes:** keep their last 6 delivered fragments and answer a signed SSN1
  request ("resend pieces i, j of fragment f", current boot id only).
- **Anchor:** asks for the missing pieces of a fragment that has been silent for
  1.5 s (up to 3 requests, `nack` command through the dongle), salvages after
  20 s, and counts gaps in the dongle's line numbers.

r7, 20 codes, all four boards on this firmware: every fragment completed (none
salvaged or lost). The Anchor sent 18 requests and 8 fragments completed
because of them. The dongle received 4,134 pieces, forwarded 4,134 lines, and
the Anchor parsed 4,134 with no numbering gaps or damaged lines, so no piece
was lost on USB; the pieces that went missing were acknowledged by the dongle's
radio but never reached its receive callback. The dongle's boot id stayed the
same through the run.

| Pair | Codes in both | Std | Worst from median |
| --- | --- | --- | --- |
| 4e58, 4ff8 | 17 | 10 us | 21 us |
| 4824, 4e58 | 19 | 71 us | 311 us |
| 4824, 4ff8 | 17 | 74 us | 310 us |

The 95th percentile spread in node 4824's pairs is 53 to 77 us; the worst value
is one code. Absolute base station to node time came out 1.5 to 2 ms off on
average with 6 of 19 codes over 5 ms, so that path is unreliable here; it is not
the node-to-node timing above.

The Anchor-clock path (each node placed by the dongle's beacon send times) did
worse here than in the first run: std 185 to 385 us, worst 1.4 ms from the
median. Absolute time, base station to node, came out 1.5 to 2 ms early on
average, against -82 us in the earlier end-to-end test. Not investigated.

The WendyOS agent's `gpio` entitlement mounts only `/dev/gpiochip0`-`7`; the
Dragonwing's ninth chip carries pin 33, so the Anchor drives that expander over
I2C (`anchor/tca9538.py`, `i2c` entitlement) instead.
