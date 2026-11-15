# soundsync Anchor

Receives fragments from soundsync-fw nodes through an ESP32 dongle (soundsync-fw,
beacon role) on USB, places every sample on one timeline and groups fragments of
the same sound into events. See the docstring in `anchor.py`.

```bash
# once: shared key for nodes, dongle and Anchor (writes secret.key here)
../sync-fw/genkey.sh
# on a laptop: the dongle's port, where recordings go; HTTP_PORT switches the map and its buttons on
HTTP_PORT=8091 python anchor.py /dev/cu.usbmodem101 fragments
python live_page.py 8093 http://127.0.0.1:8091 secret.key mesh.html   # then http://localhost:8093/mesh
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

## Where a sound came from (`locate.py`)

**The placing has run on boards (next section); placing from an ordinary
triggered event's audio has only run on made-up sounds.** The Anchor turns the ranging
distances into a flat map of the boards, takes the aligned audio of an event
that three or more mapped boards heard, works out how much later each board
heard it, and solves for the point whose distances to the boards explain those
differences. The result goes into the event's json (`source`), onto the log
(`sound at x ..., y ...`) and onto the mesh page: a cross with the region the
sound is 95% likely to be in, and its distance to every board.

```bash
python3 locate.py                # self-check (needs numpy)
python3 locate.py sim            # expected error, the table below
python3 locate.py demo           # http://127.0.0.1:8093/mesh: click places a sound or a board, drag moves a board, double-click removes it
python3 locate.py wav chord chord.wav        # a test sound to play from a phone
python3 locate.py <out>/events <out>/layout.json   # place recorded events again
```

In the demo only the dashed rings come from where things really are. The
distances come from ranging rounds (`ranging.py`) on audio synthesised for the
boards' true places, and each sound is placed from the audio synthesised at
each board's microphone. A board that was just dragged is wrong on the map
until three rounds agree on its new distances. The demo drops a moved board's
old readings; a real Anchor cannot know a board moved (see below).

Expected error, from `locate.py sim` (200 made-up sounds per row; boards about
1.3 m apart; every ranging distance off by 30 mm sigma; one echo at half
strength; clocks 1 us apart):

| Sound | Inside the boards | At the edge | 1 m outside | 3 m outside |
| --- | --- | --- | --- | --- |
| Sweep, 2 to 6 kHz in 64 ms (the ranging boards' own) | 1.6 cm | 2.5 cm | 12 cm | 77 cm |
| Chord of 2.7, 3.3, 4.1 and 5.3 kHz | 1.7 cm | 3.0 cm | 11 cm | 77 cm |
| Click | 1.7 cm | 2.7 cm | 12 cm | 67 cm |
| Steady 2.7 kHz tone | 8.3 cm | 19 cm | 80 cm | 2.4 m |

Medians, three boards; four boards are about a fifth better inside. With the
clocks 100 us apart (the worst node-to-node figure measured on real audio) the
sweep inside goes from 1.6 to 4.0 cm. Outside the boards the direction stays
good (under 1 degree for all but the tone) while the distance does not: the
page draws such a sound as a ray.

- **A steady pitch is the wrong test sound.** Its correlation repeats every
  cycle (0.37 ms, 13 cm of path at 2.7 kHz) and only the onset says which cycle
  is right; in the simulation that pick was right one time in five. A sweep, a
  click or several pitches at once have one peak that stands out. The chord
  keeps 2.7 kHz in it, so `sync-fw`'s detector should trigger on it as it is
  (not tried).
- **The listening boards can range each other** (next section), so one
  firmware does both. Boards with the `../range` firmware still work as before
  and can be mixed in. Either way the map is kept in `<OUT_DIR>/layout.json`:
  saved whenever the live distances make one, read back at start.
  A board stays on the map once it is on it, also when it goes quiet; delete
  the file to start over.
  A `layout.json` with `"manual": true` (boards placed with a tape measure,
  `{"nodes": {"<mac>": [x, y]}}` in metres) is never overwritten.
- Both firmwares name a board by its Wi-Fi station address, so the map and the
  events agree on who is who (`esp_read_mac(..., ESP_MAC_WIFI_STA)` in both).
- `sync-fw` triggers on 2.7 kHz held for 60 ms. The 64 ms sweep passes that
  band in a few milliseconds and will not trigger it as configured.
- The map is flat: boards and sound in one plane. A sound 20 cm above a table
  of boards is placed as if it were on the table.
- Distances alone leave the map free to turn and mirror. Positions are metres
  on that map, with the lowest board address at the origin when the map is
  first drawn; a mirrored room puts the sound on the mirrored side too.
- A wrong `PP_SELF_MM` adds the same error to every ranging distance and bends
  the map; with four boards the page's "flat to x cm" shows it.
- A board that is moved further away keeps its old, shorter distance until
  the old readings have left the last 40: the rule that picks the settled
  distance prefers the shortest group of three, because echoes only read long.
- Three boards leave nothing to check with, and sometimes two spots fit (the
  page marks the second). A fourth board gives the fit a spare measurement.
- The simulation's microphones are identical and its echo is a single one.
  Real picks will be worse than the 1 to 5 us it shows; the 100 us row above
  is the realistic one.
- `LOCATE_PICK_SIGMA_US` (60), `LOCATE_POS_SIGMA_MM` (30) and
  `LOCATE_WINDOW_MS` (40) set the size of the drawn region and how much audio
  after the onset is compared.

## Distances between the listening boards (`ranging.py`)

`sync-fw` boards with a piezo on D0 measure the distances between themselves,
so the boards that listen need no second firmware. **Run on three XIAOs on
2026-11-14**, with the Anchor on the Dragonwing; the true distances were not
taped, so what follows is consistency, not accuracy.

A round is started here (`POST /range?rounds=5`, the button on the mesh page,
or `RANGE_EVERY_S=<seconds>`). Every board records the round and plays one
sweep (2 to 6 kHz, 64 ms) in its own 160 ms slot; the recordings come in as
ordinary fragments. The clocks are already synchronised, so a reading is read
straight from when each microphone heard each sweep: board k hears its own at
`t_k`, board j at `t_j`, and `c * (t_j - t_k) + RANGE_SELF_MM`. One round
measures every pair both ways.

```bash
python3 ranging.py       # self-check: made-up rounds, buzzers on wires, a listening round
python3 sim_boards.py    # made-up boards with clocks of their own, through anchor.py's own line handling
```

What the boards showed:

| Pair | One way | Other way | Used (the longer) |
| --- | --- | --- | --- |
| 4824, 4ff8 | 633 mm | 662 mm | 662 mm |
| 4824, 4e58 | 628 mm | 832 mm | 832 mm |
| 4e58, 4ff8 | 1101 mm | 1154 mm | 1154 mm |

- Each direction repeats to about 5 mm over rounds, apart from one round in
  ten or so that lands 10 to 14 cm off (next point); `mesh.agree` drops those.
- **An arrival is not one peak.** The matched filter shows a cluster of humps
  about 0.4 ms (14 cm) apart, the piezo ringing and reflections near the
  board, and an echo about 3 m longer is often the strongest of all. The first
  strong hump of the first cluster is taken (`find_sweep`). Whether that is
  the direct sound is the largest uncertainty left: a neighbouring hump moves
  a distance by 10 to 14 cm.
- **Further apart it gets worse.** With the boards moved to 1.4 to 1.7 m
  (2026-11-15) a board sometimes took an echo for the direct sound and read
  4 m, steadily. A pair whose two directions differ by more than a wire can
  explain (`WIRE_MM`, 220) now uses the shorter one, since an echo only reads
  long. Even so, two pairs' directions stayed 27 to 30 cm apart, and nothing
  here says which is right.
- **The two directions of a pair differ**, steadily: a reading runs from one
  board's buzzer to the other's microphone, less the buzzer's distance to its
  own microphone, so a buzzer on a wire can only shorten it (by up to twice
  the wire). The longer direction is used. Board 4824's buzzer is about 10 cm
  from its microphone by this reckoning, the others closer.
- Each buzzer is itself placed from its sweep's arrival at the three
  microphones, as a standing check: they land 2, 9 and 12 cm from their
  boards, where the buzzers are.
- Boards report when their sweep began (`G` line). Buzzer to own microphone
  then reads 22 to 30 cm, which is the piezo's and the microphone's delay more
  than distance, and differs per board. `solve_geometry` (microphones and
  buzzers placed apart from true distances) is therefore off
  (`RANGE_SOLVE_BUZZERS=1`); with three boards it has several exact answers
  anyway, four pin it down (tried on made-up boards).
- The sweep is toggled out with interrupts off on one core. The RMT driver
  that `../range` uses reported every sweep as sent and most made no sound.
- Rounds go out through the dongle's `nack` command (one frame per board), so
  a dongle with older firmware starts them too.
- A round with three boards is a 0.7 s recording per board; a few seconds per
  round. Eight boards at most.

### Listening rounds

`POST /range?rounds=1&listen=1` (the page's "Listen for a sweep"): the boards
record two seconds and none sweeps. Every 2 to 6 kHz sweep in those recordings
is found by matched filter, repeats of the same sound are merged, and it is
placed on the map. No trigger is involved, so a quiet source works.

Run with the Mac the boards were plugged into playing sweeps every 300 ms at
19% volume, one speaker at a time (2026-11-14):

| Speaker | Placed at | Sweeps | Arrival differences repeat to |
| --- | --- | --- | --- |
| Left | (-0.16, 0.67) m, 6 cm from board 4ff8 | 8 | 30 us |
| Right | (-0.06, 0.84) m | 7 | 25 us |

The two spots are 19 cm apart; where the speakers really were against the
boards was not measured. A spot right beside a board gets a long region: there
a few centimetres of map error move the answer a long way along one line.

- `locate.py wav sweep <file>` writes sweeps to play from a phone.
- A loud source next to a board also triggers that board's ordinary sound
  detector over and over; its queue then delays the round's recording.
  Listening rounds wait 40 s for recordings for that reason.
- The Dragonwing's buzzer was not heard by the boards at these distances
  (it was already too faint at 56 cm earlier), so it was not placed.
- Ordinary triggered sounds (`locate()` on an event's audio) placed the Mac
  playing a chord from both speakers nowhere sensible: quiet, two sources at
  once, and picks that disagreed by up to a millisecond. `locate()` now
  refuses a sound whose pairs disagree by more than 150 us.

### Any sound in a listening round

A listening round's recordings are also treated as an event, so `locate()`
runs on them. When no start stands out it falls back to correlating each
pair's whole two seconds (`arrivals_whole`): a steady or repeated broad sound
adds up while the noise does not. Each pair keeps its three best peaks and
the combination in which the three pairs agree (to 40 us) is taken; a steady
pitch (a peak per cycle) is refused.

Run with 150 ms noise bursts (1.5 to 7 kHz) from the Mac at 19% volume, barely
above the room noise at the boards:

| Speaker | Result |
| --- | --- |
| Right | Placed at (-0.10, 0.95) m on six tries, arrival differences the same to 6 us, pairs agreeing to 3 to 6 us. The sweeps put the same speaker at (-0.06, 0.84) m: 11 cm away, along the direction that is least certain there. |
| Left, 6 cm from board 4ff8 | Refused on every try: the pairs disagree by about 70 us. With the limit loosened to 100 us it was placed in two different spots, one 4 m off, so the limit stays at 40 us. Sweeps place this speaker; whole recordings do not. |

- A recording that arrives before the reference board's cannot be put on the
  shared timeline yet; the Anchor now tries each held recording again when
  another of its round or event arrives. Before that, rounds after a pause
  were dropped.
- A listening round with sweeps in it gets two marks, one from the matched
  filter and one from the whole recordings. On the last try they were 17 cm
  apart for the left speaker ((-0.12, 0.63) and (0.00, 0.75) m), each with
  its pairs agreeing; which is nearer the truth is not known.

### Watching it live

- **Keep listening** (`POST /listen?on=1`, a button on the page): one-second
  listening rounds one after another, so a sound played at any moment is
  placed within about six seconds. A board that has just recorded a round
  reports no triggered sounds for 8 s, and none while two fragments wait to
  be sent; without that a loud repeating sound filled every board's outbox
  and the rounds' recordings came in tens of seconds late.
- On the page a placed sound fades out over ten seconds.
- **Measure again** forgets every distance, the map and the sounds on it,
  runs six rounds and places the boards anew (`POST /range?rounds=6&reset=1`).
  Until the shape is known the boards sit grouped in the middle of the page
  with no lines; then they spread out to their places. Keep-listening pauses
  for the run and takes up again after it.
- The map zooms (buttons, or scroll over it) and moves (drag); "Fit" undoes both.
- A dashed ray that runs off the map is a sound known by direction only.
- The Sounds panel has a switch per kind of sound (with a clear start, faint,
  sweeps, the boards' own buzzers); the choice is kept in the browser.
- **Where the delay is** (one cycle timed from the Anchor's log, 2026-11-15):
  the boards record 1.2 s; sending three recordings to the dongle took 4.6 s
  (87 KB each, one after another); USB and working out the position about
  0.1 s; the cloud tunnel adds 2 to 4 s to what a laptop sees. A cycle was
  6.3 s, of which the boards listen for 1 s. Since then a round's recording
  goes out at half the sample rate (46 KB, about half the time on the air),
  missing pieces are asked for after 0.5 s instead of 1.5 s, and
  `live_page.py` asks in three overlapping requests: 5.0 s per cycle with one
  of the three boards still on the older firmware.
- **Boards are colours, not addresses.** The Anchor hands each board a colour
  number (red, green, blue, ...) and sends it with every round. The XIAO has
  one yellow LED and no colour LED, so the board blinks its number: red once,
  green twice, blue three times, then a pause. Before its first round it
  gives one short flash every 3 s.
- **Click a board on the page and it beeps** (`POST /beep?board=<mac>`): three
  sweeps from that board's piezo.
- **Buzz the anchor**: one listening round with the Dragonwing's own buzzer
  sounded inside it. On 2026-11-15 the buzzer (properly connected by then)
  showed 22 to 32 dB over the room at all three boards and was placed at the
  same spot on two tries, 38 cm from one board and 41 cm from another, with a
  phone playing sweeps in the room at the same time.
- **Movement**: an orange arrow joins two heard sweeps when the second came
  within 12 seconds, at a different place (further apart than half of what the
  two 95% regions reach towards each other, and 10 cm), and near enough to have
  been walked (2.5 m/s, with the regions' reach as slack): `tracks()` in
  `mesh.html`. The band around the arrow is where the path could lie: as wide
  at each end as that sweep's region is across the line, strongest along the
  line. Other kinds of sound get no arrow. This is drawn by the page from two
  placed sounds; the Anchor does no tracking. The dotted orange lines (a ray for
  a sound with a direction only, and lines from the newest sound to each board)
  are gone: a sound without a place is listed in the table only.
  Sweeps heard one after another in the same listening round count as well:
  `place_heard` in `anchor.py` gives each placed sweep the moment it was heard,
  and only merges repeats that follow each other. A path does not fork. A sweep
  on a path is drawn as its cross alone; its own region is left out, and the
  band runs on past the path's two loose ends by what those regions reach.
  How much of a path there can be (measured 2026-11-15): a listening round
  records 1.27 s per board and one starts about every 5.0 s (4.5 to 6.6), the
  rest being radio time for three recordings. The boards hear a quarter of the
  time. A clip with a sweep every 0.4 s gives three points per round, then
  nothing for about 3.7 s.
- **Turn left, Turn right, Mirror** (and Shift with scroll): turn or mirror the
  picture so it matches the room as you stand in it. Distances alone fix the
  boards' shape but not which way it faces, nor whether it is mirrored. This
  changes the drawing only: the coordinates in the tables and the saved map
  stay as they are. The button between them shows the angle and puts it back.
- `live_page.py <port> <anchor url> <key file> [page]` serves the page on a
  laptop without the token in the address and answers at once from a copy of
  `/mesh.json` it keeps fresh, for when the Anchor is reached through
  `wendy cloud tunnel` (a second or two per request).

### The boards report sweeps themselves (2026-11-15)

A listening round left the boards deaf three seconds in five, while its recordings were sent. Since the
firmware of 2026-11-15 every board runs the sweep's matched filter itself, all the time
(`sync-fw/main/sweepdet.c`: the samples halved to 16 kHz, matched 8000 times a second, about a quarter of
one core), and tells the Anchor of each sweep it finds in a fragment of 1.6 KB: 20 ms of the filter's
output around the strongest point, as complex numbers, with three seconds of beacons. Sent in about
18 ms; the audio stays on the board. `ranging.heard_arrival` evens that output out over the band, fills
it in eight times and picks the arrival by `find_sweep`'s rule; `Ranger.detected` gathers the reports
of one sweep (within 15 ms on the shared timeline, waited for up to a second) and `place_detected` in
`anchor.py` places it. Sweeps that keep arriving with the same differences are one point ("sweep heard,
8 times"); other differences start a new point, which is what a path is drawn from.

Measured: a clip with a sweep every 0.4 s played from a laptop for 12 s gave 80 reports from three
boards, 22 sweeps placed, the laptop's place repeating to about 1 cm. `test_sweepdet.py` builds the
detector for a laptop and feeds it made-up audio: arrival differences between boards right to 1 us,
20 us for a sweep no louder than the noise, nothing reported on noise alone.

- A board does not report during a ranging round (the sweeps are the boards' own), and sends no
  one-second recording for a sound that turned out to be a sweep.
- `POST /detect?on=0` switches the reports off on every board heard lately, `on=1` on again; on a
  board's console, `detect 0` and `detect 1`. On at boot.
- Sweeps in listening rounds are not placed a second time while reports are coming in.
- The three boards run the build made before a one-line fix in `build_blob` (a round whose number has
  bit 30 set was taken for a report). The Anchor keeps round numbers under that bit, so it does not
  matter until someone starts rounds from elsewhere; the fix is in the source for the next build.

### The laptop is the Anchor (2026-11-15)

The dongle moved from the Dragonwing to the laptop and `anchor.py` runs there. The reason is the route
to the Dragonwing, not the Dragonwing: the office Wi-Fi does not let two of its clients talk to each
other, which left `wendy cloud tunnel`, and that stopped passing anything for up to a minute at a time
(measured: during a stall the Anchor answered on the device itself in under 16 ms and the device reached
the internet in under 250 ms). On the laptop the page's data is 0.1 s old and a press is acted on at
once. The Dragonwing's buzzer ("Buzz the anchor") is not there on a laptop. To go back, a cable straight
from the Dragonwing's Ethernet port to the laptop avoids both the Wi-Fi's rule and the cloud.

For an hour in between, branch `locate` was taken back to the version before the boards' own sweep
reports (commits bd35a0e and e318c0a there); with the tunnel gone, this branch is in use again.

### Boards in shells (2026-11-15)

With each board in a closed shell its own sweep is about fifty times louder at its own microphone
(peak near 30000 of 32767) and the other boards' sweeps are faint: one to eight times the matched
filter's floor, with reflections at ten to fourteen. One round no longer shows the direct sound. The
buzzer is already driven across two pins at the pin's full swing, so there is no louder setting. The
Anchor averages instead: `faint_arrival` in `ranging.py` keeps the last `STACK` (8) rounds' matched
filter envelopes for every buzzer and microphone pair on a 64 kHz grid (`GRID_HZ`), averages them, and
takes the first hump that stands at least ten spreads over the floor and a quarter of the strongest.
"Measure again" empties the stacks. Two runs from nothing, in shells, gave 1414 / 1442 / 1713 mm and
1415 / 1453 / 1711 mm for 4824–4e58, 4824–4ff8 and 4e58–4ff8. An earlier run in shells gave
1775 / 1372 / 2096 mm, and the same boards without shells 1395 / 1671 / 1379 mm; nothing was measured
with a tape, so which of these is the room is open.

### Presses that the tunnel holds up

`wendy cloud tunnel` holds about one request in twelve for many seconds, and can hang altogether after
the Anchor is redeployed. Three things follow from that. `live_page.py` answers every button press at
once (202) and passes it on behind the page's back, because a browser keeps only six requests open to
one address and a few held presses used to stop the page refreshing. Each press carries the page
server's clock (`&t=<ms>`); the Anchor compares it with the quickest trip of the last minute
(`lateness_ms` in `anchor.py`) and drops a beep that is over 2 s behind and any other press over 6 s
behind, so held presses no longer play in a burst. And the page acts on the press of a button, not the
click: the table is rewritten twice a second, and a click that straddled a rewrite was lost. A press
that came to nothing is shown next to the page's title for a few seconds.

## Accuracy, measured against a wire (2026-11-14)

A XIAO drove a GPIO wire into the Anchor's C5; both stamped every edge.

| Mapping | Edges | Bias | Std | p95 | Worst |
| --- | --- | --- | --- | --- | --- |
| Node to node, shared AP beacons, on-device arrival stamps | 1401 | +0.01 us | 0.45 us | 0.60 us | 8.4 us |
| Node to Anchor time, the C5's beacon send times (`place()`) | 1400 | +772 us | 241 us | 1.5 ms | 1.8 ms |

Absolute Anchor time is millisecond-grade (send queueing and airtime are not
observable from the sender); relative node timing is microsecond-grade.
`anchor_sigma_us` carries a 1 ms floor for that reason.
