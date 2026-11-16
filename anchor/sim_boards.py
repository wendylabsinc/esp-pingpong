"""Made-up boards that talk to anchor.py in the dongle's own lines.

Each board has its own clock (booted at another moment, some ppm fast or slow)
and sends what sync-fw would: signed SSF1 pieces of a fragment with block
stamps, the beacons it heard and audio. The dongle's lines ("T" for beacons
sent, "F" for pieces) go into anchor.handle(), so placing fragments on one
timeline, ranging rounds (ranging.py) and placing a sound (locate.py) run as
they would with boards, minus the radio and the air.

    python3 sim_boards.py      three rounds, then a sound; prints what the Anchor made of them
"""
import base64
import hashlib
import hmac
import math
import os
import struct
import sys
import tempfile

import numpy as np

RATE, BLOCK, CHUNK = 32000, 640, 1400


class Board:
    def __init__(self, mac, xy, rng):
        self.mac, self.xy, self.rng = mac, xy, rng
        self.boot_at = -rng.uniform(50e6, 500e6)         # true time (us) its esp_timer read 0
        self.ppm = rng.uniform(-20, 20)
        self.first = rng.uniform(0, 1e6 / RATE)          # its esp_timer when sample 0 was taken
        self.boot_id = int(rng.integers(1, 2**32))
        self.frag = 0
        self.beacons = []                                # (seq, arrival on its esp_timer)

    def esp(self, true_us):
        return (true_us - self.boot_at) * (1 + self.ppm * 1e-6)

    def true(self, esp_us):
        return esp_us / (1 + self.ppm * 1e-6) + self.boot_at

    def sample_at(self, true_us):
        return int((self.esp(true_us) - self.first) * RATE / 1e6)

    def times(self, first_n, n):
        """True time (us) of samples first_n .. first_n + n."""
        return self.true(self.first + (first_n + np.arange(n)) * 1e6 / RATE)

    def hear_beacon(self, seq, sent_true_us):
        self.beacons.append((seq, self.esp(sent_true_us + 350 + abs(self.rng.normal(0, 1)))))

    def fragment(self, key, first_n, trig_n, pcm, hit):
        """The "F" lines of one fragment, as the dongle prints them."""
        self.frag += 1
        blocks = range(first_n // BLOCK, (first_n + len(pcm)) // BLOCK + 1)
        stamps = [(b * BLOCK, int(self.first + b * BLOCK * 1e6 / RATE + self.rng.uniform(0, 300))) for b in blocks]
        heard = self.beacons[-200:]
        blob = struct.pack("<4s6sHIIqIqIHH", b"SSM1", bytes.fromhex(self.mac), 1, self.frag, RATE, first_n, len(pcm), trig_n,
                           hit, len(stamps), len(heard))
        blob += struct.pack("<4f", 2700, 15, 0, 0)
        blob += b"".join(struct.pack("<qq", *s) for s in stamps)
        blob += b"".join(struct.pack("<6sHIIq", b"\xff" * 6, 0, seq, 0, int(arr)) for seq, arr in heard)
        blob += np.asarray(pcm, "<i2").tobytes()
        count = (len(blob) + CHUNK - 1) // CHUNK
        for i in range(count):
            body = b"SSF1" + struct.pack("<IHHII", self.frag, i, count, len(blob), self.boot_id) + blob[i * CHUNK:(i + 1) * CHUNK]
            frame = body + hmac.new(key, body, hashlib.sha256).digest()[:8]
            yield f"F,{self.mac},{base64.b64encode(frame).decode()}"


def main():
    out = tempfile.mkdtemp(prefix="sim-boards-")
    key = os.urandom(32)
    os.environ["SOUNDSYNC_KEY"] = key.hex()
    sys.argv = ["anchor.py", "/dev/null", out]
    import anchor
    import locate
    import ranging
    rng = np.random.default_rng(4)
    sent = []
    anchor.dongle[0] = type("Dongle", (), {"write": staticmethod(lambda b: sent.append(b.decode().strip()))})()
    places = {"441bf6804824": (0.0, 0.0), "441bf6804e58": (1.3, 0.0), "441bf6804ff8": (0.6, 1.1), "441bf6800001": (1.5, 1.2)}
    boards = {m: Board(m, xy, rng) for m, xy in places.items()}
    order = sorted(boards)
    now, seq = 0.0, 0

    def beacons(seconds):
        nonlocal now, seq
        for _ in range(int(seconds * 10)):
            seq += 1
            now += 100_000
            anchor.handle(f"T,{seq},{int(now)}")
            for b in boards.values():
                b.hear_beacon(seq, now)

    def deliver(lines_by_board):
        lines = [l for ls in lines_by_board for l in ls]
        for i in rng.permutation(len(lines)):            # pieces of all boards, in any order
            anchor.handle(lines[i])

    beacons(20)
    for _ in range(3):
        rid = anchor.ranger.start(order)
        assert sent[-1].startswith(f"nack {order[-1]} {ranging.RANGE_BOOT:08x} {rid} {ranging.SLOT_MS},"), sent[-1]
        t0 = now + 150_000
        frags, times = [], {}
        for b in boards.values():
            # sync-fw: the recording ends when the block after the round's end has been read
            end_n = b.sample_at(t0 + (len(order) * ranging.SLOT_MS + 150) * 1000 + rng.uniform(0, 40_000))
            first_n = end_n - int((ranging.PRE_MS + len(order) * ranging.SLOT_MS + 150) * RATE / 1000) - int(rng.uniform(0, 640))
            times[b.mac] = (first_n, b.times(first_n, end_n - first_n))
        recs = ranging.record_round(places, order, rng=rng, t0=t0, times={m: t for m, (_, t) in times.items()})
        beacons(1.5)
        for b in boards.values():
            first_n = times[b.mac][0]
            frags.append(list(b.fragment(key, first_n, first_n + ranging.PRE_MS * RATE // 1000, recs[b.mac][0], ranging.RANGE_HIT | rid)))
        deliver(frags)
    state = anchor.mesh_view()
    worst = max(abs(e["mm"] - 1000 * math.dist(places[e["a"]], places[e["b"]])) for e in state["edges"])
    assert len(state["edges"]) == 6 and all(e["settled"] for e in state["edges"]), state["edges"]
    assert worst < 10, worst
    print(f"distances: all six settled, worst {worst:.1f} mm off")

    src = (0.9, 0.4)
    made = now + 500_000
    frags, times = [], []
    for m in order:
        b = boards[m]
        first_n = b.sample_at(made - 300_000 + rng.uniform(0, 60_000))
        times.append((first_n, (b.times(first_n, RATE) - made) * 1e-6))
    chans, _ = locate.record("chord", [places[m] for m in order], src, ranging.SPEED, rng=rng, times=[t for _, t in times])
    beacons(1.5)
    for m, (first_n, _), pcm in zip(order, times, chans):
        frags.append(list(boards[m].fragment(key, first_n, first_n + 300 * RATE // 1000, pcm, 1)))
    deliver(frags)
    placed = anchor.locator.view()["sources"][-1]
    there = locate.align(dict(anchor.locator.points, src=(placed["x"], placed["y"])), places)["src"]
    off = math.dist(there, src)
    assert off < 0.03, (off, placed)
    print(f"sound: placed {off * 100:.1f} cm off, 95% region {placed['ellipse']['major_m'] * 100:.0f} x {placed['ellipse']['minor_m'] * 100:.0f} cm, "
          f"pairs disagree by {placed['closure_us']} us")
    print("ok")


if __name__ == "__main__":
    main()
