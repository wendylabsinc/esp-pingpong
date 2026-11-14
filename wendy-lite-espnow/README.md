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

## Status (2026-11-14)

- `test/test_ens.py`: the C stream against the Python stream over a simulated
  link. Both directions arrive intact with 0%, 5% and 20% frame loss,
  duplication and reordering; no retransmissions on a clean link.
- The Anchor relay runs on the Dragonwing; a simulated node's TLS hello reaches
  the agent's listener through it.
- The Wendy Lite build with the component and the patch compiles.
- **Not yet run on a node.** Whether the node's Wendy Net TLS handshake with the
  agent completes over this stream is untested, as is time sync alongside it.
