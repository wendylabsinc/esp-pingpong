"""Distances between the listening boards, measured by the boards that listen.

One round, started here: the dongle broadcasts `range <round> <slot ms>
<boards>`; every board records the round and plays one sweep in its own slot
(sync-fw/main/audio.c, "Ranging rounds"). The recordings arrive as ordinary
fragments, marked bands_hit = 0x80000000 | round, and anchor.py has already
put them on one timeline (shared beacons, about 1 us).

In every recording each sweep is found with a matched filter. Board k heard
its own sweep at t_k (its microphone is SELF from its buzzer) and board j heard
it at t_j, so the sweep travelled d = c * (t_j - t_k) + SELF. One round gives
every pair both ways. Each direction is kept and judged like the ranging
boards' own readings (mesh.agree: settled when three lie within 100 mm), and
the distance is the longer of the two: a reading runs from one board's buzzer
to the other's microphone, and a buzzer on a wire away from its board can
only shorten it.

Same sweep, wiring and SELF as ../range, so the two kinds of board mix.

    python3 ranging.py      self-check: made-up boards, through the Anchor's map and a located sound
"""
import collections
import math
import os
import threading
import time

import numpy as np

import locate
from mesh import AGREE_MIN, KEEP, SELF_M, SPEED, agree

RANGE_HIT = 0x80000000              # sync-fw proto.h
RANGE_BOOT = 0x31474E52             # sync-fw proto.h: a round carried by the dongle's "nack" command
BEEP_BOOT = 0x31504542              # sync-fw proto.h: "play your sweep this many times"
DETECT_BOOT = 0x31544544    # "DET1": switch a board's own sweep reports on or off
LEAD_MS = 250                       # from the first board being told to the start of the round
SLOT_MS = 160                       # one sweep (64 ms), its echoes, and the next board's turn
PRE_MS = 60                         # sync-fw RANGE_PRE_MS: recorded before slot 0
SWEEP = (2000.0, 6000.0, 0.064)     # sync-fw RANGE_F0, RANGE_F1, RANGE_SWEEP_MS
MAX_M = 15.0                        # further than a buzzer carries
LIVE_S, FORGET_S = 120, 600
MAX_BOARDS = 8                      # sync-fw SSR_MAX
LISTEN_SLOTS, LISTEN_SLOT_MS = 6, 350   # a listening round: two seconds with no board sweeping
STACK = 8                           # rounds averaged per buzzer and microphone when sweeps are faint (faint_arrival)
GRID_HZ = 64000                     # the matched filter's envelope is worked out on this grid, whatever a board's rate
# The most a buzzer on its wire can shorten a reading: twice 11 cm, the longest wire reckoned on these boards.
# With boards 1.4 to 1.7 m apart two pairs' directions differed by 27 to 30 cm, more than any wire: at 300 the
# choice flipped from run to run and the map jumped by 30 cm.
WIRE_MM = 220
# How much later the audio timeline puts a sound than it reached the microphone. Measured on a
# XIAO with a buzzer 5 cm off: 354 us from the first edge to the sound in the audio (README,
# beep_truth.py), of which 146 us is the 5 cm. Each 10 us off moves every buzzer 3.4 mm.
MIC_LATENCY_US = float(os.environ.get("RANGE_MIC_LATENCY_US", 208))


def sweep(rate):
    f0, f1, dur = SWEEP
    t = np.arange(int(dur * rate)) / rate
    return np.sin(2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t * t))


def find_sweep(x, rate, earliest=False, up=4):
    """Where the sweep starts in x, in samples (fractional), and how far it stands out of the rest.

    Matched filter, flattened over the sweep's band so a peaky buzzer does not
    smear it, then the envelope: one hump per arrival, no cycles to mistake.
    `earliest`: the direct sound, not the strongest. Measured on the boards:
    an arrival is a cluster of humps about 0.4 ms (14 cm) apart, the piezo
    ringing and reflections near the board, and an echo 3 m longer is often
    stronger than all of them. So: the first hump at least 0.35 of the
    strongest anywhere starts the first cluster; within 1 ms of it the first
    hump at least 0.6 of that cluster's strongest is the arrival. Which hump
    is the direct sound is not certain: taking a neighbour moves a distance by
    10 to 14 cm, the largest error left in ranging.
    """
    if len(x) < len(sweep(rate)) // 2:
        return None, 0.0
    env = _envelope(x, rate, up)
    k = int(np.argmax(env))
    floor = float(np.median(env)) + 1e-12
    if earliest:
        k = _direct(env, 0, k, rate * up)
    return _fine(env, k) / up, float(env[k] / floor)


def _envelope(x, rate, up=4, flatten=True):
    """The matched filter's envelope over x, `up` points per sample: env[m] = a sweep starting at sample m / up.

    flatten: even out the sweep's band, which sharpens one sweep. Only for a
    stretch with a single sweep in it: with several, flattening their combined
    spectrum makes copies appear where none were (seen with three sweeps
    300 ms apart: a fourth, a third as strong).
    """
    tpl = sweep(rate)
    n = 1 << (len(x) + len(tpl)).bit_length()
    cross = np.fft.rfft(x - np.mean(x), n) * np.conj(np.fft.rfft(tpl, n))
    hz = np.fft.rfftfreq(n, 1 / rate)
    cross[(hz < SWEEP[0] * 0.9) | (hz > SWEEP[1] * 1.1)] = 0
    if flatten:
        mag = np.abs(cross)
        cross = cross / (mag + 0.05 * mag.max() + 1e-12)
    full = np.zeros(n * up, complex)
    full[: len(cross)] = 2 * cross            # positive frequencies only: the envelope comes out as the magnitude
    return np.abs(np.fft.ifft(full))[: len(x) * up]


HEARD_HIT = 0x40000000      # in bands_hit: a board's own report of a sweep it heard (sync-fw/main/sweepdet.c)
HEARD_RATE = 8000           # values per second in such a report
REPORT_WAIT_S = 1.0         # how long the reports of one sweep are waited for


def is_report(bands_hit):
    """A sweep report, not a round's recording whose number happens to have the same bit set."""
    return bands_hit & (RANGE_HIT | HEARD_HIT) == HEARD_HIT


def heard_arrival(iq, up=8):
    """A board's own report of a sweep -> (where in the report the sweep arrived, in values, fractional;
    how far it stands out of the usual level).

    The board ran the matched filter itself and sent 20 ms of its output around the strongest point, as
    complex numbers 8000 times a second, then its usual level. That output has the sweep's 2 to 6 kHz on
    one side only, so 8000 a second holds all of it; here it is evened out over the band, filled in to
    `up` times as many points, and the arrival is picked on its envelope by find_sweep's rule.
    """
    v = np.asarray(iq, float)
    usual = max(float(v[-2]), 1.0)
    c = v[:-2:2] + 1j * v[1:-2:2]
    n = len(c)
    spec = np.fft.fft(c)
    hz = np.arange(n) * HEARD_RATE / n
    spec[(hz < SWEEP[0] * 0.9) | (hz > SWEEP[1] * 1.1)] = 0
    mag = np.abs(spec)
    spec = spec / (mag + 0.05 * mag.max() + 1e-12)
    full = np.zeros(n * up, complex)
    full[:n] = spec
    env = np.abs(np.fft.ifft(full))
    k = _direct(env, 0, int(np.argmax(env)), HEARD_RATE * up)
    return _fine(env, k) / up, 30000.0 / usual


def _direct(env, lo, k, per_s):
    """The direct arrival behind the strong hump at k, looking back as far as lo (the rule in find_sweep)."""
    seg = env[lo: k + 2]
    tops = np.nonzero((seg[1:-1] > seg[:-2]) & (seg[1:-1] >= seg[2:]) & (seg[1:-1] >= 0.35 * env[k]))[0] + 1 + lo
    if not len(tops):
        return k
    near = tops[tops <= int(tops[0]) + int(0.001 * per_s)]
    return int(near[env[near] >= 0.6 * env[near].max()][0])


def _fine(env, k):
    if 0 < k < len(env) - 1:
        lo, mid, hi = env[k - 1], env[k], env[k + 1]
        if lo - 2 * mid + hi < 0:
            return k + 0.5 * (lo - hi) / (lo - 2 * mid + hi)
    return float(k)


def find_sweeps(x, rate, least=12.0, apart_ms=150):
    """Every sweep in x -> [(start in samples, how far it stands out)], in order of time.

    For a stretch in which sweeps may come from anywhere at any time. Each is
    found roughly, strongest first, with `apart_ms` after it left out as its
    ringing and echoes; then timed by itself as find_sweep times a board's.
    """
    if len(x) < len(sweep(rate)):
        return []
    rough = _envelope(x, rate, 1, flatten=False)
    floor = float(np.median(rough)) + 1e-12
    out = []
    for _ in range(40):
        k = int(np.argmax(rough))
        if rough[k] < least * floor:
            break
        rough[max(0, k - int(0.03 * rate)): k + int(apart_ms / 1000 * rate)] = 0
        lo = max(0, k - int(0.012 * rate))                     # the direct sound may precede the strongest arrival
        at, clear = find_sweep(x[lo: k + int(0.1 * rate)], rate, earliest=True)
        if at is not None:
            out.append((lo + at, clear))
    return sorted(out)


def faint_arrival(y, rate, stack, part=0.0):
    """The direct arrival of a sweep in y, in samples, helped by the rounds before it; None if not found.

    For boards in shells (2026-11-15): there the direct sound reached another
    board only 3 to 8 times over the matched filter's floor, under what one
    round can tell from noise, while a reflection half a metre to a metre
    longer came in at 10 to 14 times. The boards do not move between rounds
    and every round is lined up on the buzzer's own start, so the rounds'
    envelopes are averaged (`stack`, the last STACK of them): the arrival stays
    and the noise evens out. On that average the first hump well out of the
    noise, and not a mere side lobe of the strongest, begins the direct sound;
    then as in find_sweep. y begins `part` of a sample before the moment that
    is the same every round (1 ms before the buzzer's own start).
    """
    up = max(1, round(GRID_HZ / rate))
    env = _envelope(np.asarray(y, float), rate, up)
    n = int((0.001 + MAX_M / SPEED) * rate * up)
    skip = int(round(part * up))          # so that every round's envelope starts at the same moment, to a grid step
    if len(env) < skip + n:
        return None
    env = env[skip:skip + n] / (float(np.median(env[skip:skip + n])) + 1e-12)
    while stack and len(stack[0]) != n:       # a board that changed its sample rate: start over
        stack.popleft()
    stack.append(env.astype(np.float32))
    mean = np.mean(stack, axis=0)
    floor = float(np.median(mean))
    spread = 1.4826 * float(np.median(np.abs(mean - floor))) + 1e-12
    z = (mean - floor) / spread
    tops = np.nonzero((mean[1:-1] > mean[:-2]) & (mean[1:-1] >= mean[2:]) & (z[1:-1] >= max(10.0, 0.25 * z.max())))[0] + 1
    if not len(tops):
        return None
    near = tops[tops <= int(tops[0]) + int(0.001 * rate * up)]
    k = int(near[mean[near] >= 0.6 * mean[near].max()][0])
    return (skip + _fine(mean, k)) / up


def measure(order, recs, slot_ms=SLOT_MS, speed=SPEED, self_m=SELF_M, stand_out=6.0, heard=None, stacks=None):
    """One round's recordings -> ({(chirper, listener): metres}, [what went wrong]).

    order: the boards in slot order. recs: {board: (samples, start_us, rate)},
    sample n taken at start_us + n / rate on the shared timeline.
    heard, if given, is filled with {chirper: {board: when it heard that sweep, us}},
    the chirper's own microphone included: enough to place the buzzer itself.
    stacks, if given ({} kept between rounds), makes each microphone's view of
    each buzzer the average of the last STACK rounds: see faint_arrival.
    """
    out, notes = {}, []
    for k, chirper in enumerate(order):
        if chirper not in recs:
            continue
        x, start, rate = recs[chirper]
        # Its own sweep is the loud one in its own slot; the slot is only right to some tens of ms.
        lo = max(0, int((PRE_MS + k * slot_ms - 60) * rate / 1000))
        hi = min(len(x), int((PRE_MS + k * slot_ms + 64 + 80) * rate / 1000))
        at, clear = find_sweep(x[lo:hi], rate, earliest=True)
        if at is None or clear < 3 * stand_out:
            notes.append(f"{chirper[-4:]} did not hear its own sweep")
            continue
        t_own = start + (lo + at) / rate * 1e6
        if heard is not None:
            heard[chirper] = {chirper: t_own}
        for listener in order:
            if listener == chirper or listener not in recs:
                continue
            y, y_start, y_rate = recs[listener]
            first = (t_own - 1000 - y_start) * 1e-6 * y_rate          # from 1 ms before it could arrive
            a = max(0, int(first))
            b = min(len(y), int(first + (0.001 + SWEEP[2] + MAX_M / speed) * y_rate))
            if stacks is not None:
                at = faint_arrival(y[a:b], y_rate, stacks.setdefault((chirper, listener), collections.deque(maxlen=STACK)),
                                   part=max(0.0, first - a))
                clear = stand_out if at is not None else 0.0
            else:
                at, clear = find_sweep(y[a:b], y_rate, earliest=True)
            if at is None or clear < stand_out:
                notes.append(f"{listener[-4:]} did not hear {chirper[-4:]}")
                continue
            d = speed * (y_start + (a + at) / y_rate * 1e6 - t_own) * 1e-6 + self_m
            if not 0 <= d <= MAX_M:
                notes.append(f"{chirper[-4:]} to {listener[-4:]} reads {d:.2f} m")
                continue
            out[chirper, listener] = d
            if heard is not None:
                heard[chirper][listener] = y_start + (a + at) / y_rate * 1e6
    return out, notes


def solve_geometry(boards, paths_m, tries=60, seed=0):
    """Where every microphone and every buzzer is, from all buzzer-to-microphone distances.

    paths_m[chirper, mic] in metres, a board's own pair included (its buzzer to
    its own microphone). A buzzer on a wire sits centimetres from its board, so
    "board to board" is not one distance: 4N - 3 unknowns, N * N distances,
    which three boards just determine and four pin down. With three there
    are usually two to eight layouts that fit exactly (found by trying it on
    made-up boards); the one whose board-to-board distances lie closest to the
    longer one-way readings is returned, and `other_mm` says how far the
    furthest of the others is from it, in its worst board-to-board distance.
    -> (mics {board: (x, y)}, buzzers {board: (x, y)}, rms misfit in metres,
    other_mm), or None when a distance is missing.
    """
    n = len(boards)
    if n < 3 or any((c, m) not in paths_m for c in boards for m in boards):
        return None
    want = np.array([[paths_m[c, m] for m in boards] for c in boards])       # [chirper][mic]

    def unpack(v):
        return v[: 2 * n].reshape(n, 2), v[2 * n:].reshape(n, 2)

    def residuals(v):
        mics, buzz = unpack(v)
        d = np.linalg.norm(buzz[:, None, :] - mics[None, :, :], axis=2) - want
        return np.concatenate((d.ravel(), [mics[0, 0], mics[0, 1], mics[1, 1]]))   # pinned: turning and shifting cost nothing

    # Start from the boards as their one-way readings draw them (each is short by what its
    # buzzer's wire adds, so the longer of the two directions is the better guess).
    guess = {(a, b): max(want[i, j] - want[i, i], want[j, i] - want[j, j]) for i, a in enumerate(boards) for j, b in enumerate(boards) if i < j}
    start, _ = locate.layout(guess)
    start = locate.align(start, {})
    rng = np.random.default_rng(seed)
    found = []
    for _ in range(tries):
        mics = np.array([start[b] for b in boards]) + rng.normal(0, 0.03, (n, 2))
        turn = rng.uniform(0, 2 * np.pi, n)
        buzz = mics + np.diag(want)[:, None] * np.stack((np.cos(turn), np.sin(turn)), axis=1)
        v = np.concatenate((mics.ravel(), buzz.ravel()))
        lam = 1e-3
        r = residuals(v)
        cost = float(r @ r)
        for _ in range(80):
            jac = np.empty((len(r), len(v)))
            for k in range(len(v)):
                step = np.zeros(len(v))
                step[k] = 1e-6
                jac[:, k] = (residuals(v + step) - r) / 1e-6
            if not np.isfinite(jac).all():
                break
            a = jac.T @ jac
            try:
                dv = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), -jac.T @ r)
            except np.linalg.LinAlgError:
                break
            if not np.isfinite(dv).all():
                break
            r2 = residuals(v + dv)
            cost2 = float(r2 @ r2)
            if cost2 < cost:
                small = np.abs(dv).max() < 1e-7
                v, r, cost, lam = v + dv, r2, cost2, lam / 4
                if small:
                    break
            else:
                lam *= 8
                if lam > 1e10:
                    break
        if np.isfinite(cost):
            found.append((cost, v))
    if not found:
        return None
    least = min(c for c, _ in found)
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]

    def apart(v):
        mics, _ = unpack(v)
        return np.array([np.linalg.norm(mics[i] - mics[j]) for i, j in pairs])

    # Every layout that fits as well as the best does (to 2 mm rms more), then the one nearest the one-way readings.
    fits = [v for c, v in found if math.sqrt(c / (n * n)) <= math.sqrt(least / (n * n)) + 0.002]
    longer = np.array([guess[boards[i], boards[j]] for i, j in pairs])
    pick = min(fits, key=lambda v: float(np.sum((apart(v) - longer) ** 2)))
    other_mm = max(float(np.abs(apart(v) - apart(pick)).max()) for v in fits) * 1000
    cost = float(residuals(pick) @ residuals(pick))
    mics, buzz = unpack(pick)
    if n > 2 and mics[2, 1] < 0:      # the mirror image fits as well: keep the third board above the first two
        mics, buzz = mics * [1, -1], buzz * [1, -1]
    return ({b: (float(mics[i, 0]), float(mics[i, 1])) for i, b in enumerate(boards)},
            {b: (float(buzz[i, 0]), float(buzz[i, 1])) for i, b in enumerate(boards)},
            math.sqrt(cost / (n * n)), round(other_mm))


class Ranger:
    """Starts rounds, gathers their recordings, keeps the readings of every pair."""

    def __init__(self, send_cmd, log=print):
        self.send_cmd, self.log = send_cmd, log
        self.lock = threading.Lock()
        self.seen = {}                              # board -> when last heard on the radio, monotonic s
        self.rounds = collections.OrderedDict()     # round -> dict(order, slot_ms, at, recs, done)
        self.edges = {}                             # (lower mac, higher mac) -> readings, as mesh.Mesh.edges
        self.last_round = 0
        self.running = False
        self.stats = collections.Counter()
        self.on_solved = None                       # called after every round that was worked out
        self.on_sweeps = None                       # ... with {chirper: {board: when it heard that sweep}}
        self.on_heard = None                        # a listening round: [{board: when it heard a sweep from elsewhere}, ...]
        self.on_detected = None                     # the boards' own reports of one sweep: {board: when it heard it}
        self.reports = []                           # (when on the shared timeline, board, when it came in)
        self.last_report = -1e9
        self.emits = collections.OrderedDict()      # (round, board) -> when its sweep began, on its own clock (its "G" line)
        self.paths = {}                             # (chirper, microphone's board) -> buzzer-to-microphone readings, mm
        self.geometry = None                        # solve_geometry()'s answer, once every path is settled
        self.mic_latency_us = MIC_LATENCY_US
        self.colours = {}                           # board -> its colour number on the page, which its LED blinks
        self.stacks = {}                            # (buzzer, microphone) -> the last rounds' envelopes (faint_arrival)
        # Placing buzzers apart from microphones needs true buzzer-to-microphone distances. On the boards
        # the buzzer's own microphone read 22 to 30 cm away (2026-11-14): that is the piezo's and the
        # microphone's delay, not distance, and it differs per board. Off until those delays are known.
        self.solve_buzzers = os.environ.get("RANGE_SOLVE_BUZZERS") == "1"

    def reset(self):
        """Forget every distance measured: the next rounds start the picture from nothing."""
        with self.lock:
            self.edges.clear()
            self.paths.clear()
            self.stacks.clear()
            self.geometry = None

    def beep(self, board, times=3):
        """Have one board play its sweep `times` times, to tell the boards apart by ear."""
        self.send_cmd(f"nack {board} {BEEP_BOOT:08x} 0 {max(1, min(int(times), 5))}")

    def colour(self, board):
        """This board's colour number: handed out once, in address order among the boards known then."""
        with self.lock:
            if board not in self.colours:
                for b in sorted(set(self.seen) | {board}):
                    self.colours.setdefault(b, len(self.colours))
            return self.colours[board]

    def heard(self, board, now=None):
        self.seen[board] = time.monotonic() if now is None else now

    def live(self, now=None):
        now = time.monotonic() if now is None else now
        return sorted(b for b, t in self.seen.items() if now - t <= LIVE_S)[:MAX_BOARDS]

    def start(self, order=None, slot_ms=SLOT_MS, listen=False, slots=LISTEN_SLOTS):
        """Begin one round with `order` (default: every board heard lately). -> round id, or None.

        listen: the boards record the round and none of them sweeps. Any sweep
        in those recordings came from elsewhere, and is placed like a sound.
        """
        order = list(order or self.live())
        if len(order) < 2:
            return None
        if listen:
            slot_ms = LISTEN_SLOT_MS
        with self.lock:
            # Round numbers stay under HEARD_HIT: the boards' firmware of 2026-11-15 takes a recording whose
            # number has that bit set for a sweep report, and sends no audio for it.
            rid = self.last_round = max(int(time.time()) & (HEARD_HIT - 1), self.last_round + 1) & (HEARD_HIT - 1)
            self.rounds[rid] = dict(order=order, slot_ms=slot_ms, at=time.monotonic(), recs={}, done=False, listen=listen)
            while len(self.rounds) > 16:
                self.rounds.popitem(last=False)
        self.stats["rounds"] += 1
        # One frame per board through the dongle's "nack" command, which every dongle firmware has
        # (its "range" command, one broadcast, needs the firmware from this branch). Each frame
        # says how long is left until the round starts, so the boards start together.
        # A listening round names LISTEN_SLOTS addresses no board has, so each board records that many slots in silence.
        slots = [f"02{i:010x}" for i in range(max(1, min(slots, MAX_BOARDS)))] if listen else order
        words = [b[i] | b[i + 1] << 8 for b in map(bytes.fromhex, slots) for i in (0, 2, 4)]
        t0 = time.monotonic()
        for board in order:
            lead = int(LEAD_MS - (time.monotonic() - t0) * 1000)
            self.send_cmd(f"nack {board} {RANGE_BOOT:08x} {rid} {slot_ms},{max(lead, 0)},{len(slots)}," + ",".join(map(str, words))
                          + f",{self.colour(board) % 8}")     # last: the board's colour, for its LED
            time.sleep(0.004)
        return rid

    def fragment(self, meta, pcm, placed):
        """A recording marked RANGE_HIT. True when it belonged to a round started here."""
        rid = meta["bands_hit"] & 0x7FFFFFFF
        with self.lock:
            r = self.rounds.get(rid)
            if r is None or r["done"] or meta["node"] not in r["order"]:
                return False
            if not placed or "trigger_ref_esp_us" not in placed:
                return True          # not on the shared timeline yet: anchor.py offers it again when it is
            else:
                shift = placed["trigger_ref_esp_us"] - placed["trigger_node_esp_us"]     # this board's clock -> the shared one
                r["recs"][meta["node"]] = (np.asarray(pcm, float), placed["first_sample_node_esp_us"] + shift, meta["rate"])
                r.setdefault("shift", {})[meta["node"]] = shift
            complete = len(r["recs"]) >= len(r["order"])
        if complete:
            self.solve(rid)
        return True

    def solve(self, rid):
        """Turn a round's recordings into readings; called when all are in, or when waiting is over."""
        with self.lock:
            r = self.rounds.get(rid)
            if r is None or r["done"]:
                return
            r["done"] = True
            order, recs, slot = r["order"], dict(r["recs"]), r["slot_ms"]
            listening = r.get("listen")
        if listening:
            return self._listened(rid, recs)
        heard = {}
        got, notes = measure(order, recs, slot, heard=heard, stacks=self.stacks) if len(recs) >= 2 else ({}, [f"{len(recs)} of {len(order)} recordings arrived"])
        now, said = time.monotonic(), []
        with self.lock:
            for i, a in enumerate(order):
                for b in order[i + 1:]:
                    if a not in recs or b not in recs:
                        continue
                    pair = tuple(sorted((a, b)))
                    edge = self.edges.setdefault(pair, {"from": {}, "good": 0, "bad": 0, "at": now})
                    ways = {c: round(1000 * got[c, l]) for c, l in ((a, b), (b, a)) if (c, l) in got}
                    for chirper, mm in ways.items():
                        edge["from"].setdefault(chirper, collections.deque(maxlen=KEEP)).append(mm)
                    if not ways:
                        edge["bad"] += 1
                        continue
                    edge["good"] += 1
                    edge["at"] = now
                    said.append(f"{a[-4:]}-{b[-4:]} " + "/".join(str(ways[c]) if c in ways else "-" for c in (a, b)) + " mm")
        self.stats["solved" if got else "empty"] += 1
        self.log(f"round {rid}: " + "; ".join(said + notes or ["nothing measured"]))
        with self.lock:
            r["heard"] = heard
        self._paths(rid)
        if self.on_solved:
            self.on_solved()
        if self.on_sweeps and heard:
            self.on_sweeps(rid, heard)

    def listening(self, rid):
        """True for a round in which the boards only recorded."""
        with self.lock:
            r = self.rounds.get(rid)
            return bool(r and r.get("listen"))

    def _listened(self, rid, recs):
        """A listening round: every sweep that three or more boards heard, as {board: when} per sweep."""
        found = {b: [(start + at / rate * 1e6, clear) for at, clear in find_sweeps(x, rate)] for b, (x, start, rate) in recs.items()}
        first = max(found, key=lambda b: len(found[b])) if found else None
        groups = []
        for t, _ in found.get(first, []):
            group = {first: t}
            for b, got in found.items():
                near = [u for u, _ in got if abs(u - t) < 15000]      # 15 ms: boards up to 5 m apart
                if b != first and near:
                    group[b] = min(near, key=lambda u: abs(u - t))
            if len(group) >= 3:
                groups.append(group)
        self.stats["listened"] += 1
        self.log(f"round {rid}: listened, " + ", ".join(f"{b[-4:]} heard {len(v)}" for b, v in sorted(found.items()))
                 + f"; {len(groups)} sweeps heard by three or more boards")
        if self.on_heard and groups and time.monotonic() - self.last_report > 15:   # boards that report sweeps themselves
            self.on_heard(rid, groups)                                             # have told of these already

    def detected(self, board, t_us, now=None):
        """A board's own report of a sweep (HEARD_HIT), at t_us on the shared timeline. The reports of one
        sweep come within 15 ms of each other (boards up to 5 m apart) and mostly within a tenth of a second
        over the radio, a resent one later: once every board heard lately has told of it, or after REPORT_WAIT_S
        with at least three, it goes to on_detected."""
        now = time.monotonic() if now is None else now
        with self.lock:
            self.stats["reports"] += 1
            self.last_report = now
            self.reports.append((t_us, board, now))
        self._gather(now)
        threading.Timer(REPORT_WAIT_S + 0.05, self._gather).start()   # and once more when no further report can be expected

    def _gather(self, now=None):
        now = time.monotonic() if now is None else now
        want = max(3, len(self.live(now)))
        ready = []
        with self.lock:
            self.reports.sort()
            groups, at = [], 0
            while at < len(self.reports):
                end = at
                while end + 1 < len(self.reports) and self.reports[end + 1][0] - self.reports[at][0] < 15000:
                    end += 1
                groups.append(self.reports[at:end + 1])
                at = end + 1
            keep = []
            for g in groups:
                heard = {}
                for t, board, _ in g:
                    heard.setdefault(board, t)
                waited = now - min(came for _, _, came in g)
                if len(heard) >= want or (waited >= REPORT_WAIT_S and len(heard) >= 3):
                    ready.append(heard)
                    self.stats["sweeps"] += 1
                elif waited < REPORT_WAIT_S:
                    keep += g
                else:
                    self.stats["unheard"] += 1       # fewer than three boards told of it: no place for it
            self.reports = keep
        for heard in ready:
            if self.on_detected:
                self.on_detected(heard)

    def emitted(self, board, rid, esp_us):
        """A board's "G" line: when its sweep of round `rid` began, on its own clock (0: it played none)."""
        if esp_us <= 0:
            return
        with self.lock:
            self.emits[rid, board] = esp_us
            while len(self.emits) > 200:
                self.emits.popitem(last=False)
        self._paths(rid)        # the line may come after the recordings

    def _paths(self, rid):
        """Buzzer-to-microphone distances of a round, for every sweep whose start is known: the sound
        left at the start and reached each microphone when its audio shows it, less the microphone's lag."""
        fresh = False
        with self.lock:
            r = self.rounds.get(rid)
            if r is None or "heard" not in r:
                return
            done = r.setdefault("pathed", set())
            for chirper, times in r["heard"].items():
                if chirper in done or (rid, chirper) not in self.emits or chirper not in r.get("shift", {}):
                    continue
                done.add(chirper)
                left = self.emits[rid, chirper] + r["shift"][chirper]
                for mic, t in times.items():
                    mm = SPEED * (t - self.mic_latency_us - left) * 1e-3
                    if -30 <= mm <= MAX_M * 1000:
                        self.paths.setdefault((chirper, mic), collections.deque(maxlen=KEEP)).append(round(max(mm, 1.0)))
                        fresh = True
        if fresh:
            self._geometry()
            if self.on_solved:
                self.on_solved()

    def _geometry(self):
        with self.lock:
            boards = sorted({c for c, _ in self.paths})
            firm = {k: agree(v) for k, v in self.paths.items()}
            ready = len(boards) >= 3 and all((c, m) in firm and firm[c, m][1] >= AGREE_MIN for c in boards for m in boards)
            paths = {k: v[0] / 1000 for k, v in firm.items()}
        if not ready or not self.solve_buzzers:
            return
        got = solve_geometry(boards, paths)
        if got:
            with self.lock:
                first = self.geometry is None
                self.geometry = dict(mics=got[0], buzzers=got[1], misfit_mm=round(got[2] * 1000, 1), other_mm=got[3])
            if first:
                self.log("ranging: microphones and buzzers placed apart; buzzer to its own microphone: "
                         + ", ".join(f"{b[-4:]} {paths[b, b] * 1000:.0f} mm" for b in boards) + f"; fits to {got[2] * 1000:.1f} mm"
                         + (f"; another layout fits as well, up to {got[3]} mm different" if got[3] > 5 else ""))

    def run(self, rounds=5, wait_s=10.0, pause_s=0.5, order=None, listen=False, after=None):
        """`rounds` rounds one after another, each waiting for its recordings. Returns at once if one is running."""
        with self.lock:
            if self.running:
                return False
            self.running = True

        def loop():
            try:
                for _ in range(rounds):
                    rid = self.start(order, listen=listen)
                    if rid is None:
                        self.log("ranging: fewer than two boards heard lately, nothing to range")
                        return
                    until = time.monotonic() + (max(wait_s, 40.0) if listen else wait_s)   # two seconds of audio per board take a while to come in
                    while time.monotonic() < until and not self.rounds[rid]["done"]:
                        time.sleep(0.1)
                    self.solve(rid)      # with what arrived, if not all did
                    time.sleep(pause_s)
            finally:
                self.running = False
                if after:
                    after()

        threading.Thread(target=loop, daemon=True).start()
        return True

    def state(self, now=None):
        """Edges and boards in the shape of mesh.Mesh.state(), for the map and the page."""
        now = time.monotonic() if now is None else now
        with self.lock:
            for b in [b for b, t in self.seen.items() if now - t > FORGET_S]:
                del self.seen[b]
            rows = []
            for (a, b), e in sorted(self.edges.items()):
                row = {"a": a, "b": b, "good": e["good"], "bad": e["bad"], "mm": None, "settled": False, "by": "listening boards"}
                # Each direction is judged by itself, and the longer one is the distance. A reading runs
                # from one board's buzzer to the other's microphone, less the buzzer's own distance to
                # its microphone, so a buzzer on a wire can only make it short: by up to twice the wire.
                # Measured: 625 mm one way and 833 mm the other, each steady to 5 mm, with that board's
                # buzzer about 10 cm from its microphone.
                each = {c: agree(mm) for c, mm in e["from"].items() if mm}
                if each:
                    firm = {c: v for c, v in each.items() if v[1] >= AGREE_MIN} or each
                    # ... unless the two are further apart than a wire explains (WIRE_MM): then the longer one timed
                    # an echo, which only ever reads long, and the shorter is the distance. Seen with boards
                    # 1.6 m apart: 1598 mm one way, 4045 mm the other, each repeating to a few centimetres.
                    best = [v[0] for v in firm.values()]
                    row.update(mm=max(best) if max(best) - min(best) <= WIRE_MM else min(best),
                               settled=any(v[1] >= AGREE_MIN for v in each.values()),
                               agree=min(v[1] for v in firm.values()), of=min(len(e["from"][c]) for c in firm),
                               apart_mm=max(v[2] for v in firm.values()),
                               last_mm=round(sum(e["from"][c][-1] for c in each) / len(each)), age_s=round(now - e["at"], 1),
                               ways={c: v[0] for c, v in each.items()})
                    if len(each) == 2:
                        row["ways_apart_mm"] = abs(each[a][0] - each[b][0])
                g = self.geometry
                if g and a in g["mics"] and b in g["mics"]:     # microphone to microphone, with the buzzers' wires taken out
                    row.update(mm=round(1000 * math.dist(g["mics"][a], g["mics"][b])), settled=True, by="listening boards, buzzers solved")
                rows.append(row)
            g = self.geometry
            buzzers = {b: {"own_mm": round(1000 * math.dist(g["mics"][b], g["buzzers"][b]))} for b in g["mics"]} if g else {}
            return {"edges": rows, "running": self.running, "stats": dict(self.stats), "buzzers": buzzers,
                    "fit_mm": g["misfit_mm"] if g else None, "other_mm": g["other_mm"] if g else None,
                    "paths_mm": {f"{c}>{m}": agree(v)[0] for (c, m), v in sorted(self.paths.items())},
                    "nodes": [{"id": b, "age_s": round(now - t, 1), "live": now - t <= LIVE_S} for b, t in sorted(self.seen.items())]}


def merge(mesh_state, ranger_state):
    """mesh.state() with the listening boards' distances and boards added; theirs win for a pair both measured."""
    mine = {(e["a"], e["b"]) for e in ranger_state["edges"] if e["mm"] is not None}
    out = dict(mesh_state)
    out["edges"] = sorted([e for e in mesh_state["edges"] if (e["a"], e["b"]) not in mine]
                          + [e for e in ranger_state["edges"] if (e["a"], e["b"]) in mine or
                             not any((m["a"], m["b"]) == (e["a"], e["b"]) for m in mesh_state["edges"])],
                          key=lambda e: (e["a"], e["b"]))
    known = {n["id"] for n in mesh_state["nodes"]}
    out["nodes"] = mesh_state["nodes"] + [n for n in ranger_state["nodes"] if n["id"] not in known]
    out["ranging"] = {"running": ranger_state["running"], "buzzers": ranger_state.get("buzzers"), "fit_mm": ranger_state.get("fit_mm"),
                      "other_mm": ranger_state.get("other_mm"), "paths_mm": ranger_state.get("paths_mm"),
                      **ranger_state["stats"]}
    return out


# ── Made-up rounds ───────────────────────────────────────────────────


def record_round(boards, order, slot_ms=SLOT_MS, rate=32000, noise=0.05, echo=0.5, sync_us=1.0, mute=(), rng=None,
                 speed=SPEED, self_m=SELF_M, t0=1e6, times=None, buzzers=None, played=None, outside=(), slots=None):
    """What each board would record in one round -> {board: (int16 samples, start_us, rate)}.

    Its own sweep overdrives the microphone (40 mm away); another board's
    arrives after its distance at 1/distance of that; one echo each; every
    board starts its slots and its recording some milliseconds off the others.
    The round starts at t0 (us). `times`: {board: when it took each sample, us}
    instead of a recording that starts just before t0 (sim_boards.py).
    `buzzers`: {board: (x, y)} where each buzzer really is (default: self_m
    from its own microphone and, for the others, at its board). `played`, if
    given, is filled with {board: when its sweep began, us}. `outside`:
    [((x, y), when in us), ...] sweeps from somewhere else, as loud at 0.5 m as
    a board's at its own microphone's distance times 12. `slots`: how many
    slots are recorded, when not one per board (a listening round).
    """
    rng = rng or np.random.default_rng()
    f0, f1, dur = SWEEP

    def tone(t):
        return np.sin(2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t * t)) * ((t >= 0) & (t <= dur))

    began = {b: t0 + i * slot_ms * 1000 + rng.uniform(-8000, 8000) for i, b in enumerate(order) if b not in mute}
    if played is not None:
        played.update(began)
    recs = {}
    for b in order:
        start = t0 - PRE_MS * 1000 + rng.uniform(-25000, 25000)
        n = int((PRE_MS + (slots or len(order)) * slot_ms + 150) * rate / 1000)
        t = start + np.arange(n) / rate * 1e6 + rng.normal(0, sync_us)     # us, this board's clock a little off
        if times is not None:
            t, n = times[b], len(times[b])
        x = rng.normal(0, noise, n)
        for k, t_play in began.items():
            d = math.dist(buzzers[k], boards[b]) if buzzers else self_m if k == b else math.dist(boards[k], boards[b])
            extra = rng.uniform(0.4, 1.2)
            x += 8 * self_m / d * tone((t - t_play) * 1e-6 - d / speed)
            x += echo * 8 * self_m / (d + extra) * tone((t - t_play) * 1e-6 - (d + extra) / speed)
        for xy, when in outside:
            d = math.dist(xy, boards[b])
            extra = rng.uniform(0.4, 1.2)
            x += 0.5 / d * tone((t - when) * 1e-6 - d / speed) + echo * 0.5 / (d + extra) * tone((t - when) * 1e-6 - (d + extra) / speed)
        recs[b] = (np.clip(np.round(x * 16000), -32768, 32767).astype(np.int16), start, rate)
    return recs


def check():
    import tempfile
    import locate
    rng = np.random.default_rng(3)
    boards = {"441bf6804824": (0.0, 0.0), "441bf6804e58": (1.3, 0.0), "441bf6804ff8": (0.6, 1.1), "441bf6800001": (1.5, 1.2)}
    order = sorted(boards)
    # One round, every pair both ways.
    got, notes = measure(order, record_round(boards, order, rng=rng))
    assert len(got) == 12 and not notes, (len(got), notes)
    worst = max(abs(d - math.dist(boards[a], boards[b])) for (a, b), d in got.items())
    assert worst < 0.01, worst
    # A board with no buzzer is still measured, from the others' sweeps.
    got, notes = measure(order, record_round(boards, order, mute=[order[0]], rng=rng))
    assert len(got) == 9 and notes == [f"{order[0][-4:]} did not hear its own sweep"], (len(got), notes)
    # Through the keeper, as anchor.py feeds it: rounds started, recordings as fragments, in any order.
    sent, said = [], []
    keep = Ranger(sent.append, log=said.append)
    assert keep.start() is None
    for b in order:
        keep.heard(b)
    for _ in range(4):
        rid = keep.start()
        assert len(sent) % 4 == 0 and sent[-1].startswith(f"nack {order[-1]} 31474e52 {rid} {SLOT_MS},2"), sent[-1]
        assert sent[-1].endswith(",4,6980,33014,256,6980,33014,9288,6980,33014,22606,6980,33014,63567,3"), sent[-1]
        recs = record_round(boards, order, rng=rng)
        for b in reversed(order):
            pcm, start, rate = recs[b]
            placed = dict(first_sample_node_esp_us=start - 5e6, trigger_node_esp_us=7e6, trigger_ref_esp_us=12e6)
            assert keep.fragment(dict(node=b, bands_hit=RANGE_HIT | rid, rate=rate), pcm, placed)
    assert not keep.fragment(dict(node=order[0], bands_hit=RANGE_HIT | 5, rate=32000), recs[order[0]][0], placed)   # no such round
    state = keep.state()
    assert len(state["edges"]) == 6 and all(e["settled"] and e["agree"] == 4 for e in state["edges"]), (state["edges"], said)
    for e in state["edges"]:
        assert abs(e["mm"] - 1000 * math.dist(boards[e["a"]], boards[e["b"]])) < 10, e
    # A round with one recording missing is solved from the rest when waiting is over.
    rid = keep.start()
    recs = record_round(boards, order, rng=rng)
    for b in order[1:]:
        keep.fragment(dict(node=b, bands_hit=RANGE_HIT | rid, rate=32000), recs[b][0],
                      dict(first_sample_node_esp_us=recs[b][1], trigger_node_esp_us=0, trigger_ref_esp_us=0))
    assert not keep.rounds[rid]["done"]
    keep.solve(rid)
    assert keep.rounds[rid]["done"] and keep.state()["edges"][-1]["good"] == 5
    # A listening round: no board sweeps, and three sweeps from one spot elsewhere are heard by all four and placed.
    heard_all = []
    keep.on_heard = lambda rid, groups: heard_all.extend(groups)
    rid = keep.start(listen=True)
    assert sent[-1].split()[4].split(",")[2] == str(LISTEN_SLOTS) and not keep.rounds[rid]["done"]
    spot = (0.3, 0.6)
    recs = record_round(boards, order, rng=rng, mute=order, slots=LISTEN_SLOTS, slot_ms=LISTEN_SLOT_MS,
                        outside=[(spot, 1e6 + k * 3e5) for k in range(3)])
    for b in order:
        keep.fragment(dict(node=b, bands_hit=RANGE_HIT | rid, rate=32000), recs[b][0],
                      dict(first_sample_node_esp_us=recs[b][1], trigger_node_esp_us=0, trigger_ref_esp_us=0))
    assert len(heard_all) == 3 and all(len(g) == 4 for g in heard_all), (heard_all, said[-1])
    for g in heard_all:
        fix = locate.solve([boards[b] for b in g], [t * 1e-6 for t in g.values()], [30e-6] * 4, SPEED)
        assert math.dist((fix["x"], fix["y"]), spot) < 0.03, fix
    assert keep.state()["edges"][-1]["good"] == 5            # a listening round measures no distances
    # Buzzers on wires: with each sweep's start known, microphones and buzzers are placed apart.
    wired = {b: (x + 0.12 * math.cos(i * 2.1), y + 0.12 * math.sin(i * 2.1)) for i, (b, (x, y)) in enumerate(sorted(boards.items()))}
    apartly = Ranger(lambda line: None, log=said.append)
    apartly.mic_latency_us, apartly.solve_buzzers = 0, True
    for b in order:
        apartly.heard(b)
    for _ in range(3):
        rid, played = apartly.start(), {}
        recs = record_round(boards, order, rng=rng, buzzers=wired, played=played, t0=2e7)
        for b in order:
            apartly.emitted(b, rid, played[b] - 5e6)           # on the board's own clock, as below
            apartly.fragment(dict(node=b, bands_hit=RANGE_HIT | rid, rate=32000), recs[b][0],
                             dict(first_sample_node_esp_us=recs[b][1] - 5e6, trigger_node_esp_us=7e6, trigger_ref_esp_us=12e6))
    g = apartly.geometry
    assert g and g["misfit_mm"] < 3 and g["other_mm"] < 5, g
    for e in apartly.state()["edges"]:
        assert abs(e["mm"] - 1000 * math.dist(boards[e["a"]], boards[e["b"]])) < 10, (e, g)
        assert abs(e["ways"][e["a"]] + e["ways"][e["b"]]) / 2 - 1000 * math.dist(boards[e["a"]], boards[e["b"]]) < 60    # what the mean alone would say
    assert all(abs(v["own_mm"] - 120) < 6 for v in apartly.state()["buzzers"].values()), apartly.state()["buzzers"]
    # Those distances make the map, and a sound is placed on it (locate.py).
    merged = merge({"edges": [], "nodes": [], "speed": SPEED}, keep.state())
    assert len(merged["edges"]) == 6 and len(merged["nodes"]) == 4
    with tempfile.TemporaryDirectory() as d:
        where = locate.Locator(d, log=lambda *_: None)
        where.update(merged)
        src = (0.7, 0.5)
        chans, starts = locate.record("sweep", [boards[b] for b in order], src, SPEED, rng=rng)
        placed = where.event("e", order, chans, 32000, starts, SPEED)
        there = locate.align(dict(where.points, src=(placed["x"], placed["y"])), boards)["src"]
        assert math.dist(there, src) < 0.03, (there, placed)
    print(__doc__.strip(), "\n\nself-check ok; worst single reading", round(worst * 1000, 1), "mm off;", said[-1])


if __name__ == "__main__":
    check()
