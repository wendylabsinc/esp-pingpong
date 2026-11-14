# Wendy Net over ESP-NOW (work in progress)

Makes ESP-NOW a carrier for Wendy Net on the acoustic nodes, next to the
time-sync beacons, which stay plain ESP-NOW broadcasts.

```
 node (Wendy Lite, XIAO ESP32S3)               Dragonwing (WendyOS)
 -------------------------------               --------------------
 Wendy Net peer                                WendyOS agent, Wendy Net
   TLS 1.3, device certificates   <========>     TLS listener, TCP 43024
   over a byte stream                                  ^
 wendy_espnow: reliable stream (ens.c)                 | TCP, ciphertext only
   SSF1 frames up, SSN1 frames down            Anchor app: enstream.Relay
        |                                              ^
        +--- ESP-NOW --->  C5 dongle  --- USB lines ---+
 SSB1 beacons (time sync) <--- broadcast by the dongle, stamped on receive
```

- `wendy_espnow/` is a Wendy Lite component: beacon receive stamps (the
  reference-broadcast time sync, as in `sync-fw`), and a reliable byte stream
  to the Anchor over ESP-NOW (`src/ens.c`, go-back-N with a receiver window).
- `wendy-lite.patch` hooks the stream into Wendy Lite's net peer
  (`components/wendy_net_peer/src/peer.c`): when the Anchor's dongle is in
  range the node dials it over the stream, exactly as it would dial a peer on a
  LAN, and runs its usual mutually authenticated TLS over it.
- `anchor/enstream.py` is the Anchor side: the same stream, relayed to the
  WendyOS agent's Wendy Net TLS listener on `127.0.0.1:43024`. TLS runs between
  the node and the agent, so the dongle and the Anchor only carry ciphertext.
- The dongle firmware is unchanged: it already forwards SSF1 frames to USB and
  sends SSN1 frames on its `nack` command; the stream rides those two paths.
  The downlink therefore carries at most 120 bytes per frame.

Build: copy `wendy_espnow/` into `wendy-lite/components/`, add the shared key
as `wendy_espnow/src/secret.h` (`sync-fw/genkey.sh` writes one), apply
`wendy-lite.patch`, set `CONFIG_WENDY_ESPNOW=y`, and build the
`seeed_xiao_esp32s3` native profile (it already enables the Wendy Net peer).
The node must be enrolled in the same tenant as the WendyOS device, with
`wendy device wendy-net configure --participate=true`.

## Which Wendy Lite generation

The Dragonwing's agent decides this, and it is not the current one. Probing its
listener on TCP 43024 (2026-11-14): it expects the 36-byte preface `WML3`,
tenant id, device id, then TLS 1.3 with the protocol name
`wendy-local-mesh-leaf/3`; with those, a test client's handshake completes up
to the agent's request for a client certificate. That is the "signed mesh
leaf" protocol, which the WendyOS source carried from 2026-10-18 to
2026-10-28 (the agent's Wendy Net status still lists a "Leaf" carrier).
Current Wendy Lite speaks its successor and gets a reset from this agent.

So there are two variants here:

- `leaf/` matches the Dragonwing as it is. Wendy Lite at `9fba014` (the last
  commit with the leaf protocol), configured by `leaf/mesh.defaults` to dial its
  mesh router at `127.0.0.1:43124`. `leaf/wendy_espnow` listens there and
  carries the bytes over the ESP-NOW stream, so the leaf code is unchanged:
  its preface and TLS reach the agent through the Anchor's relay.
  `leaf/wendy-lite-9fba014.patch` starts the component and adjusts two
  dependency rules that otherwise stop a native build from configuring.
- `wendy_espnow/` and `wendy-lite.patch` are for current Wendy Lite and a
  current agent: the stream plugs into the net peer as a carrier directly.

## Status (2026-11-14)

- `test/test_ens.py`: the C stream against the Python stream over a simulated
  link. Both directions arrive intact with 0%, 5% and 20% frame loss,
  duplication and reordering; no retransmissions on a clean link.
- The Anchor relay runs on the Dragonwing; a simulated node's TLS hello reaches
  the agent's listener through it.
- The Wendy Lite build with the component and the patch compiles.
- Both firmware variants compile for the XIAO ESP32S3 (ESP-IDF 6.0.3).
- Time sync before this change, from run r12 (three nodes, old firmware,
  `sync-fw/tools/beacon_sync.py`): two nodes agree on the beacons both heard to
  0.94 to 1.01 us std, 1.9 to 2.2 us p95, 4.9 to 5.9 us worst, in 20 s windows.
- **Not yet run on a node.** Whether the node's handshake with the agent
  completes over this stream is untested, as is time sync alongside it.
