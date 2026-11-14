"""Reliable byte stream over ESP-NOW frames, Anchor side.

Mirrors wendy-lite-espnow/wendy_espnow/src/ens.c (cumulative acknowledgement, go-back-N
on a retransmit timer, receiver window, delayed acknowledgements) and maps the
segments onto the frames the dongle already forwards:

  uplink, node -> Anchor: an SSF1 frame whose frag_id has the top bits 01
      frag_id = 0x40000000 | (conn & 0x3FFFFFFF), idx = window, count = flags,
      blen = seq, boot = ack, then the payload (one dummy byte when ACKONLY)
  downlink, Anchor -> node: the dongle's "nack" command, i.e. an SSN1 frame
      boot = 0x40000000 | conn, frag = seq, values[0..1] = ack, [2] = window,
      [3] = flags | payload length << 8, [4..] = payload, two bytes per value

Each connection is relayed to a local TCP port: the Wendy Net TLS listener of
the WendyOS agent. The node runs TLS to the agent through this relay, so the
dongle and this process only carry ciphertext."""
import socket
import struct
import threading
import time

SYN, FIN, RST, ACKONLY = 1, 2, 4, 8
STREAM_MARK, STREAM_MASK = 0x40000000, 0xC0000000
DOWN_MSS = 120            # payload bytes in one SSN1 frame (60 values)
ACK_DELAY_US, PROBE_US, MAX_BACKOFF = 10_000, 200_000, 4


class Ens:
    """Same state machine as struct ens in ens.c."""

    def __init__(self, conn, opener, tx_cap, rx_cap, mss, max_inflight, rto_us, now):
        self.conn, self.opener = conn, opener
        self.tx, self.rx = bytearray(), bytearray()
        self.tx_cap, self.rx_cap = tx_cap, rx_cap
        self.snd_una = self.snd_nxt = self.rcv_nxt = 0
        self.mss, self.peer_wnd, self.max_inflight = mss, mss, max_inflight
        self.rto_us, self.rto_at, self.ack_at, self.probe_at = rto_us, 0, 0, 0
        self.last_rx_us = self.last_progress_us = now
        self.backoff = self.unacked_segments = self.dup_acks = 0
        self.peer_seen = self.rst_rcvd = self.rst_send = self.recovering = False
        self.ack_now = opener
        self.stat_retx = self.stat_tx_segments = self.stat_rx_segments = self.stat_rx_dropped = 0

    def rx_window(self):
        return min(self.rx_cap - len(self.rx), 65535)

    def write(self, data):
        n = min(len(data), self.tx_cap - len(self.tx))
        self.tx += data[:n]
        return n

    def read(self, size):
        was_tight = self.rx_cap - len(self.rx) < self.mss
        out = bytes(self.rx[:size])
        del self.rx[:len(out)]
        if out and was_tight and self.rx_cap - len(self.rx) >= self.mss:
            self.ack_now = True
        return out

    def input(self, conn, seq, ack, wnd, flags, data, now):
        if conn != self.conn:
            return
        self.stat_rx_segments += 1
        self.peer_seen = True
        self.last_rx_us = now
        if flags & RST:
            self.rst_rcvd = True
            return
        acked = (ack - self.snd_una) & 0xFFFFFFFF
        if acked and acked <= len(self.tx):
            del self.tx[:acked]
            self.snd_una = (self.snd_una + acked) & 0xFFFFFFFF
            if ((self.snd_nxt - self.snd_una) & 0xFFFFFFFF) >= 0x80000000:
                self.snd_nxt = self.snd_una
            self.backoff = 0
            self.last_progress_us = now
            self.rto_at = now + self.rto_us
            self.dup_acks, self.recovering = 0, False
        elif not acked and flags & ACKONLY and self.snd_nxt != self.snd_una and not self.recovering:
            self.dup_acks += 1
            if self.dup_acks >= 2:       # a segment was lost: go back now, not at the timer
                self.snd_nxt = self.snd_una
                self.recovering = True
                self.stat_retx += 1
        self.peer_wnd = wnd
        if flags & ACKONLY or not data:
            return
        if seq == self.rcv_nxt:
            n = min(len(data), self.rx_cap - len(self.rx))
            self.rx += data[:n]
            self.rcv_nxt = (self.rcv_nxt + n) & 0xFFFFFFFF
            if n < len(data):
                self.stat_rx_dropped += 1
            self.unacked_segments += 1
            if self.unacked_segments >= 2 or n < len(data):
                self.ack_now = True
            elif not self.ack_at:
                self.ack_at = now + ACK_DELAY_US
        else:
            self.stat_rx_dropped += 1
            self.ack_now = True

    def reset(self):
        self.rst_send = True

    def _fill(self, flags, data=b""):
        if self.opener and not self.peer_seen:
            flags |= SYN
        return (self.conn, self.snd_nxt, self.rcv_nxt, self.rx_window(), flags, data)

    def poll(self, now):
        """-> (conn, seq, ack, wnd, flags, data) to send, or None"""
        if self.rst_send:
            self.rst_send = False
            return self._fill(RST | ACKONLY)
        if self.rst_rcvd:
            return None
        inflight = (self.snd_nxt - self.snd_una) & 0xFFFFFFFF
        if inflight and now >= self.rto_at:
            self.snd_nxt = self.snd_una
            inflight = 0
            self.stat_retx += 1
            self.backoff = min(self.backoff + 1, MAX_BACKOFF)
        unsent = len(self.tx) - inflight
        window = min(self.max_inflight, self.peer_wnd)
        if unsent and inflight < window:
            n = min(unsent, self.mss, window - inflight)
            seg = self._fill(0, bytes(self.tx[inflight:inflight + n]))
            if not inflight:
                self.rto_at = now + (self.rto_us << self.backoff)
            self.snd_nxt = (self.snd_nxt + n) & 0xFFFFFFFF
            self.stat_tx_segments += 1
            self.ack_now, self.ack_at, self.unacked_segments = False, 0, 0
            return seg
        probe = False
        if unsent and not inflight and self.peer_wnd == 0 and now >= self.probe_at:
            self.probe_at = now + PROBE_US
            probe = True
        hello = self.opener and not self.peer_seen and now >= self.probe_at
        if hello:
            self.probe_at = now + PROBE_US
        if self.ack_now or (self.ack_at and now >= self.ack_at) or probe or hello:
            self.ack_now, self.ack_at, self.unacked_segments = False, 0, 0
            return self._fill(ACKONLY)
        return None


def parse_uplink(pkt, hdr=20, tag=8):
    """SSF1 stream frame -> (conn, seq, ack, wnd, flags, data), or None if not a stream frame"""
    frag, idx, count, blen, boot = struct.unpack_from("<IHHII", pkt, 4)
    if frag & STREAM_MASK != STREAM_MARK:
        return None
    return frag & 0x3FFFFFFF, blen, boot, idx, count, pkt[hdr:-tag]


def nack_line(mac, seg):
    """Segment -> the dongle command that sends it to the node as an SSN1 frame"""
    conn, seq, ack, wnd, flags, data = seg
    assert len(data) <= DOWN_MSS
    vals = [ack & 0xFFFF, ack >> 16, wnd, (flags & 0xFF) | (len(data) << 8)]
    padded = data + b"\0" * (len(data) & 1)
    vals += [padded[i] | (padded[i + 1] << 8) for i in range(0, len(padded), 2)]
    return f"nack {mac} {STREAM_MARK | conn:08x} {seq} {','.join(map(str, vals))}"


class Relay:
    """All stream connections of all nodes, each relayed to one TCP target."""

    def __init__(self, send_cmd, target=("127.0.0.1", 43024), log=print):
        self.send_cmd, self.target, self.log = send_cmd, target, log
        self.conns = {}          # (mac, conn) -> dict(ens, sock, ...)
        self.lock = threading.Lock()
        self.stats = dict(opened=0, closed=0, up_bytes=0, down_bytes=0, up_frames=0, down_frames=0, resets_sent=0)
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _now():
        return time.monotonic_ns() // 1000

    def uplink(self, mac, seg):
        conn, seq, ack, wnd, flags, data = seg
        now = self._now()
        with self.lock:
            self.stats["up_frames"] += 1
            c = self.conns.get((mac, conn))
            if c is None:
                if flags & RST:
                    return
                if not flags & SYN:      # a connection we do not know: tell the node to drop it
                    self._send(mac, (conn, 0, 0, 0, RST | ACKONLY, b""))
                    self.stats["resets_sent"] += 1
                    return
                for key in [k for k in self.conns if k[0] == mac]:   # one connection per node
                    self._close(key, "superseded")
                try:
                    sock = socket.create_connection(self.target, timeout=3)
                    sock.setblocking(False)
                except OSError as e:
                    self.log(f"stream {mac}/{conn:08x}: relay target {self.target} refused: {e}")
                    self._send(mac, (conn, 0, 0, 0, RST | ACKONLY, b""))
                    return
                c = dict(ens=Ens(conn, False, 32768, 65536, DOWN_MSS, 8 * DOWN_MSS, 250_000, now),
                         sock=sock, opened=now, up=0, down=0)
                c["ens"].ack_now = True
                self.conns[(mac, conn)] = c
                self.stats["opened"] += 1
                self.log(f"stream {mac}/{conn:08x}: opened, relaying to {self.target[0]}:{self.target[1]}")
            c["ens"].input(conn, seq, ack, wnd, flags, data, now)

    def _send(self, mac, seg):
        try:
            self.send_cmd(nack_line(mac, seg))
            self.stats["down_frames"] += 1
        except Exception as e:
            self.log(f"stream downlink: {e!r}")

    def _close(self, key, why):
        c = self.conns.pop(key, None)
        if not c:
            return
        try:
            c["sock"].close()
        except OSError:
            pass
        self.stats["closed"] += 1
        self.log(f"stream {key[0]}/{key[1]:08x}: closed ({why}), {c['up']} B up, {c['down']} B down, "
                 f"retx {c['ens'].stat_retx}, {(self._now() - c['opened']) / 1e6:.1f} s")

    def _run(self):
        while True:
            time.sleep(0.003)
            now = self._now()
            with self.lock:
                for key, c in list(self.conns.items()):
                    ens, sock = c["ens"], c["sock"]
                    if ens.rst_rcvd:
                        self._close(key, "reset by node")
                        continue
                    if now - ens.last_rx_us > 20_000_000:
                        self._send(key[0], (ens.conn, 0, 0, 0, RST | ACKONLY, b""))
                        self._close(key, "node silent for 20 s")
                        continue
                    # node -> agent
                    try:
                        while ens.rx:
                            n = sock.send(bytes(ens.rx[:8192]))
                            ens.read(n)
                            c["up"] += n
                            self.stats["up_bytes"] += n
                    except BlockingIOError:
                        pass
                    except OSError as e:
                        self._send(key[0], (ens.conn, 0, 0, 0, RST | ACKONLY, b""))
                        self._close(key, f"agent side: {e}")
                        continue
                    # agent -> node
                    try:
                        room = ens.tx_cap - len(ens.tx)
                        if room:
                            data = sock.recv(room)
                            if not data:
                                self._send(key[0], (ens.conn, 0, 0, 0, RST | ACKONLY, b""))
                                self._close(key, "agent closed the connection")
                                continue
                            ens.write(data)
                            c["down"] += len(data)
                            self.stats["down_bytes"] += len(data)
                    except BlockingIOError:
                        pass
                    except OSError as e:
                        self._send(key[0], (ens.conn, 0, 0, 0, RST | ACKONLY, b""))
                        self._close(key, f"agent side: {e}")
                        continue
                    for _ in range(2):          # pace the downlink: at most 2 frames per 3 ms
                        seg = ens.poll(now)
                        if not seg:
                            break
                        self._send(key[0], seg)

    def snapshot(self):
        with self.lock:
            return dict(self.stats, open=[dict(node=k[0], conn=f"{k[1]:08x}", up=c["up"], down=c["down"],
                                               retx=c["ens"].stat_retx, tx_queued=len(c["ens"].tx))
                                          for k, c in self.conns.items()])
