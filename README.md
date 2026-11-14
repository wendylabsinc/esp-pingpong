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
