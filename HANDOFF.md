# Handoff: sound sync and Wendy Net over ESP-NOW (state on 2026-11-14)

What works, what is left, and the traps that cost time. Details and numbers
are in `README.md` (sound sync, three nodes) and `wendy-lite-espnow/README.md`
(Wendy Net over ESP-NOW).

## Where things run

| Piece | Firmware or code | State |
| --- | --- | --- |
| C5 dongle `10bda3e7f058` (on the Dragonwing) | `sync-fw`, as of commit `ee3576e` | unchanged since; forwards frames, sends beacons |
| Anchor app on the Dragonwing | `anchor/` at this branch's head | deployed, includes the stream relay |
| Node `441bf6804ff8` ("esp-camera-02") | Wendy Lite `9fba014` + `wendy-lite-espnow/leaf` | Wendy Net over ESP-NOW; no audio |
| Node `441bf6804824` | `sync-fw` between commits `a23b4df` and `8adcc19` (an intermediate noise-floor version that was never committed) | audio and fragments; no Wendy Net |
| Node `441bf6804e58` | older `sync-fw` (before node logs) | audio and fragments; no Wendy Net |

The Wendy Lite changes are not in a Wendy Lite branch. `wendy-lite-espnow/`
holds the component sources and the patches to apply. Local copies: worktree
`wendy-lite-leaf` (detached at `9fba014`, uncommitted) and branch
`hackathon/sound-sync` in `wendy-lite-soundsync` (commit `bcff0d6`, not pushed).

## What is left

1. **Time sync and the mesh link measured at the same time.** Each was measured
   on its own: the link with the dongle on the Dragonwing, time sync with the
   dongle on a laptop. To measure both: dongle on the Dragonwing, node on a
   laptop's USB; `POST /cmd?line=ap on` and `POST /capture?seconds=120` on the
   Anchor for the dongle's `A` lines; read the node's `A` lines from its USB
   console; `sync-fw/tools/beacon_sync.py --ap <node log> <dongle capture>`.
   The figure to match is 0.76 to 0.85 us (std).
2. **Audio on the Wendy Net firmware.** The node firmware that joins Wendy Net
   does not detect sound or send fragments; `sync-fw` does, but only signs its
   frames (shared-key tag), it does not encrypt them. Porting `sync-fw/main/audio.c`
   into the Wendy Lite build and sending fragments to the Anchor over the mesh
   address is the step that makes the audio path end-to-end encrypted.
3. **The Dragonwing's agent is an October build.** It speaks the "signed mesh
   leaf" protocol (preface `WML3`, protocol name `wendy-local-mesh-leaf/3`),
   which WendyOS carried from 2026-10-18 to 2026-10-28. Current Wendy Lite
   cannot talk to it. `wendy-lite-espnow/wendy_espnow` plus `wendy-lite.patch`
   is the variant for a current agent and current Wendy Lite: it compiles and
   has never run on hardware.
4. **Downlink is 120 bytes per frame.** The stream's downlink rides the
   dongle's `nack` command so the dongle did not need reflashing. A proper
   "send this frame to that node" command in `sync-fw/main/main.c` would allow
   full-size frames. The agent's side of one handshake is about 30 KB.
5. **One unexplained stall.** Once, the leaf-firmware node went silent on its
   USB console about a second after opening a stream that no relay answered
   (dongle on the laptop), until reset. Three later runs in the same situation
   were clean.
6. **Only one node is on the Wendy Net firmware.** Whether `4824` and `4e58`
   are enrolled in the Dragonwing's tenant was not checked.
7. **Detection misses.** In the last 50-beep run, every detected beep was
   delivered and placed (140 of 140); the 10 misses were nodes not triggering.
   A node that hears the buzzer about 10 dB weaker stays under the +15 dB
   threshold. `trace on` (the `V` lines) and `sync-fw/tools/ledger.py` explain
   each miss. No node currently runs the final noise-floor fix (commit
   `8adcc19`): `4ff8` had it and now runs the Wendy Lite firmware; `4824` and
   `4e58` were never flashed with it.
8. **Absolute time, base station to node, is unreliable** (1.5 to 2 ms off in
   the three-node runs). Node-to-node timing does not depend on it.
9. **Open branches elsewhere:** `sender-bound-tags` here (the tag also covers
   the sender address; needs nodes and dongle reflashed together), and a local
   WendyOS fix for the gpio entitlement (`fix/gpio-entitlement-all-chips`).

## Security limits found in review (not fixed)

A read-only review of the radio-facing code found no memory-safety bug in
`ens.c` or the node component, no way for an untagged frame to reach the
stream code, and no way to make the relay connect anywhere but its configured
target. It did find these, all rooted in one shared key with no freshness:

1. **Replay without the key.** The tag covers neither a nonce nor the sender
   address. A recorded stream "hello" frame, replayed, makes the Anchor close
   that node's live relay connection and open a new one
   (`anchor/enstream.py`, "superseded"). The node's TLS session then has to
   start over. Binding the sender into the tag (`sender-bound-tags`) and a
   per-connection nonce would close this.
2. **Downlink segments are not checked for sender or freshness** on the node
   (`on_downlink`); a replayed segment of the live connection can stall it
   until the next probe.
3. **No global cap on relay connections.** One per sender address, each with a
   socket to the agent and up to 96 KB of buffers; a key holder (or the replay
   above with spoofed addresses) can exhaust sockets. The connect also runs on
   the USB reader thread.
4. **No rate limit on resets** sent for unknown connections: each costs a
   dongle command and a radio frame.
5. **A fragment's piece count is trusted from the frame.** A tagged frame with
   a count of 65,535 makes the Anchor scan that range twice a second and build
   a 92 MB zero-filled fragment after 20 s.
6. **Node log frames can grow disk use without bound** (one file per sender
   address).

Items 3 to 6 need the shared key. TLS between node and agent is unaffected by
all six: they cost availability, not confidentiality.

## Gotchas

**Reaching the Dragonwing**

- It is enrolled in another organization's tenant and reachable through Wendy
  Cloud only. There is no route to the Anchor's HTTP port from a laptop: run
  things inside the Anchor container with `container exec` and call
  `http://127.0.0.1:8091` there. Files go in as base64 in the command.
- The test tools live on the persistent volume, `/data/soundsync/tools`.
  `/tmp` in the container is emptied by every redeploy.
- After the dongle is replugged, restart the Anchor app: its libusb context
  does not see the new device.
- From a laptop on the venue Wi-Fi the Dragonwing's Wendy Net port 43024 was
  unreachable, and a node with Wendy Net enabled formed no Wi-Fi link to it.

**Wendy Net (leaf generation)**

- The leaf listener resets any connection whose first 36 bytes are not the
  `WML3` preface, without a TLS alert, and waits on shorter input (5 s in that
  generation's source). A test client sees bare resets until the preface is
  right.
- The leaf needs signed time from the internet over Wi-Fi, and blocks IP
  traffic for 125 s after each boot. The Dragonwing shows the node's route as
  unreachable until then.
- The node's log calls the carrier "Wi-Fi TCP": the leaf dials the local proxy
  at `127.0.0.1:43124`.
- ESP-NOW runs on the Wi-Fi channel of the access point the node is associated
  with. At the venue that was channel 1, the dongle's default. Elsewhere, move
  the dongle: `POST /cmd?line=ch <n>`.
- The installed Wendy CLI (2026.10.29) cannot read or set Wendy Net on Lite
  boards; a CLI built from current WendyOS source can. The leaf build needs
  neither: its mesh settings are compiled in (`leaf/mesh.defaults`).

**Building Wendy Lite at `9fba014`**

- Run `set-target esp32s3` before `build` (`leaf/build-leaf.sh` does).
- A native build fails to configure with "Missing required kconfig option"
  until two dependency rules stop referring to `WENDY_NET`
  (`leaf/wendy-lite-9fba014.patch`).
- Set `CONFIG_MDNS_ENABLE_CONSOLE_CLI=n`, and fetch the `esp-pqc` submodule.
- Its partition table equals `sync-fw`'s, so flashing the app at `0x10000`
  keeps the board's enrollment.

**Frames and keys**

- One shared key signs everything on the radio: `sync-fw/genkey.sh` writes
  `secret.h` for the firmware and `secret.key` for the Anchor. Neither is in
  the repository. The Wendy Lite component needs the same `secret.h`.
- `frag_id` is overloaded in SSF1 frames: top bit set is a node log frame, top
  bits `01` a stream segment. Real fragment numbers must stay below
  `0x40000000`.
- Receive stamps: read the radio clock and `esp_timer` with interrupts off.
  Without that the Wendy Lite build measured 1.32 us against 0.76 us for
  `sync-fw` on the same boards.

**sync-fw**

- The noise floor must not learn from the first block of a sound: with beeps
  2 s apart it climbed until the node went deaf after about 9 beeps.
- A radio acknowledgement does not mean the dongle's software got the frame;
  the Anchor asks for missing pieces (`nack`), which is what makes fragments
  complete.
- `/buzz` on the Anchor is off unless `ALLOW_GPIO_BUZZ=1`; `/dchirp` is the
  buzzer path that works (expander at I2C address 0x3b, line 7).
- Flash only after the build printed "Project build complete": a filtered
  build log once hid a failed build and an old image was flashed.
