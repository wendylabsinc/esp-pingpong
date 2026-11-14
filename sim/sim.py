"""Sound-level simulation: boards in a room, heard through the firmware's own detector and trigger.

    python3 sim/sim.py                  self-check: range the boards, lay them out, locate taps
    python3 sim/sim.py HOST[:PORT]      run in real time, reporting to a live box server (box/server.py)

Each board's microphone stream is drawn sample by sample on that board's own
drifting clock: every sweep and tap, with its echoes off the walls, over room
noise. The streams go through sim/board.c, which is the firmware's chirp.h and
tap.h, and what comes out is the reports a board would send. Radio beacons are
left out: box/test_taploc.py covers those.

Needs numpy and a C compiler (CC, default cc). Runs on Linux and macOS.
"""
import argparse
import ctypes
import glob
import itertools
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "box"))
import taploc  # noqa: E402

KEY = bytes.fromhex(os.environ.get("TAPLOC_KEY", "00" * 32))

# The firmware's settings (firmware/main/Kconfig.projbuild). Must match the boards'.
RATE = 16000
F0, F1, SWEEP_MS, DETECT_DB = 2000, 6000, 50, 13
TRIGGER_RATIO, TRIGGER_MIN = 12, 200
SWEEP_PERIOD_S, REPORT_S = 3.0, 5.0
BLOCK = RATE // 50

# The room. Change these to match the real one.
ROOM = (6.0, 4.5, 2.6)  # metres: x, y, height
HEIGHT = 1.0            # of every board and every tap above the floor
TEMP_C = 15.0           # the box's default; set the same on its page
SELF_M = 0.02           # buzzer to microphone on a board
ECHO = 0.7              # share of a sound's amplitude a wall sends back
# ponytail: echoes that touched at most two surfaces. Raise for a longer tail;
# the paths grow with the cube of this.
ECHO_ORDER = 2
BOARDS = {  # node id -> x, y (m), clock rate error (ppm), how long it has been running (s)
    bytes.fromhex("aa0000000001"): (0.6, 0.5, 0.0, 12.0),
    bytes.fromhex("aa0000000002"): (5.4, 0.6, 12.0, 3.812),
    bytes.fromhex("aa0000000003"): (5.3, 4.0, -7.0, 91.1),
    bytes.fromhex("aa0000000004"): (0.7, 3.9, 21.0, 77.0),
    bytes.fromhex("aa0000000005"): (3.1, 2.4, -15.0, 0.4),
}

# Loudness, in microphone counts (of 32768). None of these has been measured
# on a board: they are the knobs to turn until the simulation matches one.
SWEEP_AT_1M = 16.0    # peak of a sweep one metre from the buzzer
TAP_AT_1M = 1500.0    # peak of a tap one metre away: a sharp knock or a clap
NOISE_RMS = 6.0       # room noise
HISS_RMS = 1.0        # the microphone's own noise
OFFSET = 300          # the microphone's constant offset
BUZZER_HZ, BUZZER_Q = 4000.0, 4.0  # a piezo disc rings at one pitch and is quiet away from it
MIC_DELAY_S = 354e-6  # sound to sample, as measured on sync-fw at 32 kHz

# Share of room noise per band, in dB of the whole, measured from
# recordings/background-mac-2026-11-14.wav. Used when there is no recording.
NOISE_BANDS = [(0, 100, -17.0), (100, 300, -4.8), (300, 1000, -2.0), (1000, 2000, -18.9),
               (2000, 6000, -26.3), (6000, 8000, -38.3)]

C = taploc.speed_of_sound(TEMP_C)
FS = 16 * RATE  # sounds are drawn this finely, then read off at each board's own sample times
LEAD = int(0.001 * FS)  # silence before a sound, for the filter's ripple ahead of it
AHEAD = 2 * RATE        # samples of the future kept per board


def name(node):
    return node.hex(":")


def build():
    """The firmware's detector and trigger, as a library."""
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "board.so")
        subprocess.run([os.environ.get("CC", "cc"), "-O2", "-shared", "-fPIC", "-o", out,
                        os.path.join(HERE, "board.c"), "-lm"], check=True)
        lib = ctypes.CDLL(out)
    lib.board_new.restype = ctypes.c_void_p
    lib.board_new.argtypes = [ctypes.c_int] * 4 + [ctypes.c_float, ctypes.c_int, ctypes.c_int, ctypes.c_int64]
    lib.board_feed.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
                               ctypes.c_void_p, ctypes.c_void_p]
    lib.board_play.restype = ctypes.c_int64
    lib.board_play.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.board_edges.argtypes = [ctypes.c_int] * 3 + [ctypes.c_void_p, ctypes.c_int]
    return lib


# ── Sounds ───────────────────────────────────────────────────────────


def shape(x, response=None):
    """A sound drawn at FS -> as the microphone passes it on, peak 1, LEAD samples late."""
    x = np.concatenate([np.zeros(LEAD), x, np.zeros(3 * LEAD)])
    f = np.fft.rfftfreq(len(x), 1 / FS)
    spectrum = np.fft.rfft(x)
    if response:
        spectrum *= response(f)
    # The microphone's own filter: nothing above half the sample rate comes through.
    spectrum *= np.clip((RATE / 2 - f) / 1000, 0, 1)
    x = np.fft.irfft(spectrum, len(x))
    return x / np.abs(x).max()


def sweep(lib):
    """The sweep as it leaves a buzzer: the firmware's square wave through a piezo's resonance."""
    half = np.zeros(1024, np.uint16)
    n = lib.board_edges(F0, F1, SWEEP_MS, half.ctypes.data, len(half))
    assert n, "the sweep has too many edges"
    edges = np.cumsum(half[:n].astype(float))
    t_us = np.arange(int(edges[-1] * FS / 1e6)) * 1e6 / FS
    square = np.where(np.searchsorted(edges, t_us, side="right") % 2 == 0, 1.0, -1.0)

    def piezo(f):
        r = f / BUZZER_HZ
        return (1j * r / BUZZER_Q) / (1 - r * r + 1j * r / BUZZER_Q)
    return shape(square, piezo)


def knock(rng):
    """A tap: a few short rings at pitches of its own, different every time."""
    t = np.arange(int(0.012 * FS)) / FS
    x = np.zeros(len(t))
    for _ in range(3):
        tau = rng.uniform(0.0002, 0.0008)
        x += rng.uniform(0.3, 1) * (t / tau) * np.exp(1 - t / tau) * np.sin(2 * np.pi * rng.uniform(700, 5000) * t)
    return shape(x)


def background(path, rng):
    """Room noise at RATE with rms 1: a recording, or noise with the spectrum one had."""
    if path:
        with wave.open(path) as w:
            assert w.getsampwidth() == 2, "the recording must be 16-bit"
            x = np.frombuffer(w.readframes(w.getnframes()), np.int16)[::w.getnchannels()].astype(float)
            n = round(len(x) * RATE / w.getframerate())
        x = np.fft.irfft(np.fft.rfft(x - x.mean())[:n // 2 + 1], n)  # resampled to RATE
    else:
        n = 60 * RATE
        spectrum, f = np.fft.rfft(rng.normal(size=n)), np.fft.rfftfreq(n, 1 / RATE)
        spectrum[0] = 0
        for lo, hi, db in NOISE_BANDS:
            band = (f > lo) & (f <= hi)
            spectrum[band] *= math.sqrt(10 ** (db / 10) / np.sum(np.abs(spectrum[band]) ** 2))
        x = np.fft.irfft(spectrum, n)
    return x / x.std()


def paths(src, mic):
    """Ways a sound gets from src to mic, straight and by the walls: [(seconds, gain)]."""
    images = []
    for s, size in zip(src, ROOM):  # mirror images of the source in each pair of walls
        images.append([(2 * n * size + s, abs(2 * n)) for n in range(-ECHO_ORDER, ECHO_ORDER + 1)] +
                      [(2 * n * size - s, abs(2 * n - 1)) for n in range(-ECHO_ORDER, ECHO_ORDER + 1)])
    out = []
    for (x, i), (y, j), (z, k) in itertools.product(*images):
        if i + j + k <= ECHO_ORDER:
            d = max(math.dist((x, y, z), mic), SELF_M)
            out.append((d / C + MIC_DELAY_S, ECHO ** (i + j + k) / d))
    return out


# ── Boards ───────────────────────────────────────────────────────────


class Board:
    def __init__(self, lib, node, spec, rng, boot):
        x, y, ppm, running_s = spec
        self.lib, self.node, self.boot, self.rng = lib, node, boot, rng
        self.mic, self.buzzer = (x, y, HEIGHT), (x + SELF_M, y, HEIGHT)
        self.rate = RATE * (1 + ppm * 1e-6)  # samples per true second
        self.first = round(running_s * RATE)  # sample index at true time `phase`
        self.phase = rng.uniform(0, BLOCK / RATE)
        self.fed = self.first                 # index of the next sample its microphone takes
        self.sound = np.zeros(AHEAD)          # what will reach the microphone, from sample `fed` on
        self.noise_at = int(rng.integers(1 << 30))
        self.next_sweep = 3 + rng.uniform(0, 3)
        self.next_report = rng.uniform(0, REPORT_S)
        self.seq, self.batch = 0, []
        self.handle = lib.board_new(RATE, F0, F1, SWEEP_MS, DETECT_DB, TRIGGER_RATIO, TRIGGER_MIN, self.first)
        assert self.handle, "the sweep does not fit the detector: see chirp.h"

    def time(self, index):
        """True time, in seconds, at which this board takes sample `index`."""
        return self.phase + (index - self.first) / self.rate

    def hear(self, t, ways, sound, level):
        """A sound that left its source at true time t reaches this microphone by `ways`."""
        when = lambda at: self.first + (at - self.phase) * self.rate  # noqa: E731
        lo = max(math.ceil(when(t + min(w[0] for w in ways) - LEAD / FS)), self.fed)
        hi = min(math.floor(when(t + max(w[0] for w in ways) + len(sound) / FS)), self.fed + AHEAD - 1)
        if hi < lo:
            return
        into = (self.time(np.arange(lo, hi + 1)) - t) * FS + LEAD  # position in `sound` if it took no time
        at = np.arange(len(sound))
        self.sound[lo - self.fed:hi + 1 - self.fed] += level * sum(
            gain * np.interp(into - delay * FS, at, sound, left=0, right=0) for delay, gain in ways)

    def listen(self, noise):
        """Takes the next block from the microphone. Returns the reports the board sends: [(kind, payload)]."""
        heard = (self.sound[:BLOCK] + NOISE_RMS * np.take(noise, self.noise_at + np.arange(BLOCK), mode="wrap")
                 + self.rng.normal(0, HISS_RMS, BLOCK) + OFFSET)
        self.sound[:-BLOCK] = self.sound[BLOCK:]
        self.sound[-BLOCK:] = 0
        self.noise_at += BLOCK
        samples = np.clip(np.rint(heard), -32768, 32767).astype(np.int16)
        hits, tap_first, tap = np.zeros((16, 3)), ctypes.c_int64(), np.zeros(512, np.int16)
        n = self.lib.board_feed(self.handle, samples.ctypes.data, BLOCK, hits.ctypes.data, len(hits),
                                ctypes.byref(tap_first), tap.ctypes.data)
        self.fed += BLOCK
        self.batch += [(int(at * 1e9 / RATE), int(db * 10), int(mine), 0) for at, db, mine in hits[:n]]
        out = []
        if tap_first.value >= 0:
            out.append((taploc.TAP, taploc.pack_tap(tap_first.value * 1_000_000 // RATE, RATE, tap.tolist())))
        now = self.time(self.fed)
        while now >= self.next_report or len(self.batch) >= 32:
            out.append((taploc.CHIRPS, taploc.pack_chirps(self.batch[:32])))
            self.batch = self.batch[32:]
            self.next_report = now + REPORT_S
        return out

    def seal(self, kind, payload):
        self.seq += 1
        return taploc.pack(KEY, kind, self.node, self.boot, self.seq, self.fed * 1_000_000 // RATE, payload)


def simulate(seed, taps, seconds=None, noise_path=None, boot=1, log=lambda text: None):
    """Yields (true time in seconds, datagram) as the boards report.

    taps: (true time in seconds, x, y), in time order; may go on for ever.
    """
    rng = np.random.default_rng(seed)
    lib = build()
    noise = background(noise_path, rng)
    chirp = sweep(lib)
    boards = [Board(lib, node, spec, rng, boot) for node, spec in sorted(BOARDS.items())]
    ways = {(a, b): paths(a.buzzer, b.mic) for a in boards for b in boards}
    taps = iter(taps)
    tap = next(taps, None)
    while True:
        # Always the board furthest behind, so no two are more than a block
        # apart and a sound made now is still in every board's future.
        board = min(boards, key=lambda b: b.time(b.fed))
        now = board.time(board.fed)
        if seconds is not None and now >= seconds:
            return
        while tap and tap[0] <= now + 2 * BLOCK / RATE:
            when, x, y = tap
            sound = knock(rng)
            for b in boards:
                b.hear(when, paths((x, y, HEIGHT), b.mic), sound, TAP_AT_1M)
            log(f"tap at {when:.2f} s, ({x:.2f}, {y:.2f})")
            tap = next(taps, None)
        for kind, payload in board.listen(noise):
            yield board.time(board.fed), board.seal(kind, payload)
        now = board.time(board.fed)
        if now >= board.next_sweep:
            # chirp_task: play, unless someone else's sweep is still ringing.
            at = lib.board_play(board.handle, BLOCK)
            if at < 0:
                board.next_sweep = now + rng.uniform(0.1, 0.4)
            else:
                for b in boards:
                    b.hear(board.time(at), ways[board, b], chirp, SWEEP_AT_1M)
                board.next_sweep = now + SWEEP_PERIOD_S * rng.uniform(0.75, 1.25)


# ── Running it ───────────────────────────────────────────────────────


def check(seed, noise_path, taps=None):
    """Five boards settle, then taps: what the box makes of it, against the truth."""
    spots = [(2.0, 1.5), (4.2, 3.1), (1.2, 3.2), (4.6, 1.2), (3.0, 0.8), (2.2, 3.4),
             (3.9, 2.0), (1.5, 2.2), (4.8, 2.6), (2.8, 1.4), (1.1, 1.0), (3.6, 3.5)]
    taps = taps or [(35 + 3.3 * i, x, y) for i, (x, y) in enumerate(spots)]
    nodes = sorted(BOARDS)
    truth = {name(n): BOARDS[n][:2] for n in nodes}
    clocks, pending, results = taploc.Clocks(), {}, []

    def survey():
        """As State.survey in box/server.py, with the first three boards placed by hand."""
        edges = {}
        for a, b in itertools.combinations(sorted(truth), 2):
            edge = clocks.edge(a, b, C, SELF_M, SELF_M)
            if edge and "distance_m" in edge:
                edges[a, b] = edge
        points, misfit = taploc.layout({pair: e["distance_m"] for pair, e in edges.items()})
        return edges, taploc.align(points, {name(n): truth[name(n)] for n in nodes[:3]}), misfit

    def solve():
        _, placed, _ = survey()
        boards = {n: {"x": x, "y": y} for n, (x, y) in placed.items()}
        fix, why = taploc.locate_tap(clocks, {n: t for n, t in pending.items() if n in boards}, boards, TEMP_C)
        made = [t for t in taps if 0 < deadline - t[0] < 1.5]  # reports follow a tap by 0.3 s, and wait 0.6 s more
        results.append((made[-1] if made else None, sorted(pending), fix, why))
        pending.clear()

    deadline = 0
    for t, data in simulate(seed, taps, taps[-1][0] + 2, noise_path):
        if pending and t >= deadline:
            solve()
        msg = taploc.parse(KEY, data)
        if msg["kind"] == taploc.CHIRPS:
            for t_us, _, mine, _ in msg["chirps"]:
                clocks.add(msg["node"], t_us, mine)
        else:
            if not pending:
                deadline = t + 0.6  # as the box does: reports this close together are one tap
            pending.setdefault(msg["node"], msg)
    if pending:
        solve()

    edges, placed, misfit = survey()
    worst_range = 0.0
    for (a, b), edge in sorted(edges.items()):
        off = edge["distance_m"] - math.dist(truth[a], truth[b])
        worst_range = max(worst_range, abs(off))
        print(f"{a[-2:]}-{b[-2:]}  {edge['distance_m']:.3f} m  off by {off * 100:+5.1f} cm"
              f"  clocks +-{edge['sigma_us']:.0f} us  {edge['samples']} sweeps")
    print(f"{len(edges)} of {len(truth) * (len(truth) - 1) // 2} pairs ranged")
    worst_place = max((math.dist(placed[n], truth[n]) for n in placed), default=math.inf)
    print(f"layout: {len(placed)} of {len(truth)} boards, worst off by {worst_place * 100:.1f} cm,"
          f" misfit {misfit * 100:.1f} cm")
    located, worst_tap = 0, 0.0
    for made, heard_by, fix, why in results:
        what = f"tap ({made[1]}, {made[2]})" if made else "no tap was made, but"
        if not fix:
            print(f"{what}: heard by {len(heard_by)}, not located: {why}")
            continue
        line = f"{what} -> ({fix['x']:.3f}, {fix['y']:.3f})  heard by {len(heard_by)}"
        if made:
            err = math.dist((fix["x"], fix["y"]), made[1:])
            located, worst_tap = located + 1, max(worst_tap, err)
            line += f"  error {err * 100:.1f} cm  region {fix['major_m'] * 100:.1f} x {fix['minor_m'] * 100:.1f} cm"
        print(line)
    print(f"{located} of {len(taps)} taps located, worst error {worst_tap * 100:.1f} cm")
    # Echoes off a wall beside two boards shorten their range by about 3 cm.
    assert len(edges) == 10 and worst_range < 0.05, "the boards did not range each other"
    assert len(placed) == len(truth) and worst_place < 0.05, "the layout is off"
    # Only that taps get through at all: the firmware drops most of them, because
    # its detector takes a loud tap for a sweep. Tighten this when that is fixed.
    assert located, "no tap was located"
    print("ok")


def stream(target, seed, noise_path, place):
    host, _, port = target.partition(":")
    if place:  # put the first three boards where they really are, as someone would on the page
        config = {"temp_c": TEMP_C, "temp_sigma_c": 3.0, "self_m": SELF_M,
                  "boards": {name(n): {"name": f"sim {i + 1}", "x": BOARDS[n][0], "y": BOARDS[n][1]}
                             for i, n in enumerate(sorted(BOARDS)[:3])}}
        request = urllib.request.Request(place.rstrip("/") + "/config", json.dumps(config).encode(),
                                         {"Content-Type": "application/json"})
        urllib.request.urlopen(request, timeout=5).close()
    rng = np.random.default_rng(seed)
    taps = ((25 + 4 * i, rng.uniform(1, ROOM[0] - 1), rng.uniform(1, ROOM[1] - 1)) for i in itertools.count())
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.time()  # the boot number: a fresh one each run, so the box does not take it for a replay
    for t, data in simulate(seed, taps, None, noise_path, int(start), lambda text: print(text, flush=True)):
        time.sleep(max(0, start + t - time.time()))
        sock.sendto(data, (host, int(port or 47800)))


if __name__ == "__main__":
    recorded = sorted(glob.glob(os.path.join(HERE, "..", "recordings", "*.wav")))
    args = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    args.add_argument("target", nargs="?", help="HOST[:PORT] of a box server; without it, run the self-check")
    args.add_argument("--seed", type=int, default=1)
    args.add_argument("--noise", default=recorded[0] if recorded else "",
                      help="room noise as a 16-bit WAV (default: the first in recordings/, else made up)")
    args.add_argument("--place", metavar="URL", help="the box's page, e.g. http://HOST:8000: place three boards on it")
    args = args.parse_args()
    print("room noise:", os.path.relpath(args.noise) if args.noise else "made up, with a measured room's spectrum",
          flush=True)
    if args.target:
        stream(args.target, args.seed, args.noise, args.place)
    else:
        check(args.seed, args.noise)
