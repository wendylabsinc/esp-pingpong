"""Tap locator maths: wire format, ranging and clocks from sweeps, arrival picking, TDOA solve.

Stdlib only. Board timestamps are microseconds on that board's own sample
clock; the solver works in metres and seconds.
"""
import collections
import hashlib
import hmac
import math
import struct

MAGIC, VERSION = b"TL", 2
BEACON, OBSERVATIONS, TAP, ECHO, CHIRPS = 1, 2, 3, 4, 5
TAG = 8
HDR = struct.Struct("<2sBB6sIIq")  # magic, version, type, node, boot, seq, t_send
OBS = struct.Struct("<6sIIqqB")    # beacon sender, its boot, its seq, t_tx, t_rx, flags
RX_SOFT = 1                        # obs flag: t_rx is when software saw the beacon, not the radio's own stamp
CHIRPH = struct.Struct("<IB")      # last round trip to the box in us (0 = none yet), count
CHIRP = struct.Struct("<qhBB")     # arrival in ns, strength in 0.1 dB, flags, epoch
TAPH = struct.Struct("<qIHB")      # t0 of sample 0, sample rate, sample count, epoch
MINE = 1                           # chirp flag: the board's own sweep, heard by its own microphone


def _tag(key, body):
    return hmac.new(key, body, hashlib.sha256).digest()[:TAG]


def pack(key, kind, node, boot, seq, t_send, payload=b""):
    body = HDR.pack(MAGIC, VERSION, kind, node, boot, seq, t_send) + payload
    return body + _tag(key, body)


def pack_obs(obs):
    return bytes([len(obs)]) + b"".join(OBS.pack(*o) for o in obs)


def pack_chirps(chirps, box_rtt_us=0):
    return CHIRPH.pack(box_rtt_us, len(chirps)) + b"".join(CHIRP.pack(*c) for c in chirps)


def pack_tap(t0, rate, samples, epoch=0):
    return TAPH.pack(t0, rate, len(samples), epoch) + struct.pack(f"<{len(samples)}h", *samples)


def parse(key, data):
    """Authenticated datagram -> dict, or None if it is not ours."""
    if len(data) < HDR.size + TAG:
        return None
    body, tag = data[:-TAG], data[-TAG:]
    if not hmac.compare_digest(_tag(key, body), tag):
        return None
    magic, version, kind, node, boot, seq, t_send = HDR.unpack_from(body)
    if magic != MAGIC or version != VERSION:
        return None
    msg = {"kind": kind, "node": node.hex(":"), "boot": boot, "seq": seq, "t_send": t_send}
    rest = body[HDR.size:]
    if kind == CHIRPS:
        if len(rest) < CHIRPH.size:
            return None
        msg["box_rtt_us"], count = CHIRPH.unpack_from(rest)
        if len(rest) != CHIRPH.size + count * CHIRP.size:
            return None
        msg["chirps"] = [(t_ns / 1000, db10 / 10, bool(flags & MINE), epoch)
                         for t_ns, db10, flags, epoch in CHIRP.iter_unpack(rest[CHIRPH.size:])]
    elif kind == OBSERVATIONS:
        if not rest or len(rest) != 1 + rest[0] * OBS.size:
            return None
        msg["obs"] = [(tx.hex(":"), boot, seq, t_tx, t_rx, flags)
                      for tx, boot, seq, t_tx, t_rx, flags in OBS.iter_unpack(rest[1:])]
    elif kind == TAP:
        if len(rest) < TAPH.size:
            return None
        t0, rate, n, epoch = TAPH.unpack_from(rest)
        if rate == 0 or len(rest) != TAPH.size + 2 * n:
            return None
        msg.update(t0=t0, rate=rate, epoch=epoch,
                   samples=list(struct.unpack_from(f"<{n}h", rest, TAPH.size)))
    else:
        return None
    return msg


# ── Ranging and clocks ───────────────────────────────────────────────
#
# Every board plays a sweep now and then and reports when its microphone hears
# sweeps: its own and everyone else's. For boards a and b, with clocks that
# differ by theta and sound taking tau to cross between them:
#
#     a's sweep:  (heard at b) - (heard at a) = theta + tau
#     b's sweep:  (heard at b) - (heard at a) = theta - tau
#
# Half the difference is the distance; half the sum is the clock offset. Neither
# needs a radio timestamp, and tau needs no speed of sound.
#
# Clocks can be related two other ways, with no distance from either. Boards
# that hear the same radio beacon hear it at the same instant; and a tap made
# next to a board happened at a known time on every clock. Whichever is
# tightest at the moment in question is used.


def _invert3(m):
    cols = [_solve3(m, [1.0 if i == j else 0.0 for i in range(3)]) for j in range(3)]
    return None if None in cols else [[cols[j][i] for j in range(3)] for i in range(3)]


class Link:
    """Distance and clock relation between two boards, fitted to their sweeps."""

    def __init__(self, fwd, back):
        # fwd: (t_a, t_b) for a's sweeps; back: (t_a, t_b) for b's sweeps.
        rows = [(ta, tb - ta, 1.0) for ta, tb in fwd] + [(ta, tb - ta, -1.0) for ta, tb in back]
        self.t0 = sum(r[0] for r in rows) / len(rows)
        self.method = "sweeps"
        self.ok = self._solve(rows)
        for _ in range(4):
            # Drop outliers, with the limit set from the median so that they do
            # not set it themselves: an echo or a chance pairing is not Gaussian.
            if not self.ok or self.n < 8:
                break
            miss = sorted(abs(self._residual(r)) for r in rows)
            limit = 4 * (1.4826 * miss[len(miss) // 2] + 1.0)
            keep = [r for r in rows if abs(self._residual(r)) <= limit]
            if not 6 <= len(keep) < len(rows):
                break
            rows = keep
            self.ok = self._solve(rows)

    def _solve(self, rows):
        # y = offset + drift * t + flight * (+1 for a's sweeps, -1 for b's)
        xs = [(1.0, (t - self.t0) * 1e-6, sign) for t, _, sign in rows]
        m = [[sum(x[i] * x[j] for x in xs) for j in range(3)] for i in range(3)]
        p = _solve3(m, [sum(x[i] * r[1] for x, r in zip(xs, rows)) for i in range(3)])
        self.inv = _invert3(m)
        self.n = len(rows)
        if p is None or self.inv is None or self.n < 4:
            return False
        self.offset, self.drift, self.flight = p
        self.s = math.sqrt(sum(self._residual(r) ** 2 for r in rows) / (self.n - 3))
        return True

    def _residual(self, row):
        t, y, sign = row
        return y - self.offset - self.drift * (t - self.t0) * 1e-6 - self.flight * sign

    def __call__(self, t):
        """t on a's clock -> (t on b's clock, 1-sigma in us)."""
        x = (t - self.t0) * 1e-6
        var = self.inv[0][0] + 2 * x * self.inv[0][1] + x * x * self.inv[1][1]
        return t + self.offset + self.drift * x, self.s * math.sqrt(max(var, 0.0))

    @property
    def flight_sigma(self):
        return self.s * math.sqrt(max(self.inv[2][2], 0.0))

    @property
    def skew_ppm(self):
        return self.drift


class Line:
    """Clock relation between two boards from instants known on both clocks.

    One instant gives the offset; instants spread over several seconds give
    the drift as well. pick_us is how well one instant is timed; floor_us is
    error that more instants do not average away.
    """
    DRIFT_PPM = 20.0  # how fast two crystals may part when the drift is not measured

    def __init__(self, pairs, method, pick_us, floor_us=0.0):
        self.method, self.floor = method, floor_us
        self.t0 = sum(p[0] for p in pairs) / len(pairs)
        rows = [((p[0] - self.t0) * 1e-6, p[1] - p[0]) for p in pairs]
        self._solve(rows, pick_us)
        for _ in range(4):
            # Drop outliers, with the limit set from the median: a timestamp
            # taken late is not Gaussian.
            if self.n < 8:
                break
            miss = sorted(abs(y - self.offset - self.drift * x) for x, y in rows)
            limit = 4 * (1.4826 * miss[len(miss) // 2] + 1.0)
            keep = [(x, y) for x, y in rows if abs(y - self.offset - self.drift * x) <= limit]
            if not 6 <= len(keep) < len(rows):
                break
            rows = keep
            self._solve(rows, pick_us)

    def _solve(self, rows, pick_us):
        self.n = len(rows)
        mx = sum(x for x, _ in rows) / self.n
        self.sxx = sum((x - mx) ** 2 for x, _ in rows)
        self.mx = mx
        my = sum(y for _, y in rows) / self.n
        self.measured = self.n >= 2 and self.sxx >= 8.0  # e.g. two instants four seconds apart
        self.drift = sum((x - mx) * (y - my) for x, y in rows) / self.sxx if self.measured else 0.0
        self.offset = my - self.drift * mx
        self.s = pick_us
        if self.n > 2:
            miss = sum((y - self.offset - self.drift * x) ** 2 for x, y in rows)
            self.s = max(self.s, math.sqrt(miss / (self.n - 2)))

    def __call__(self, t):
        """t on the first clock -> (t on the second, 1-sigma in us). The spread grows away from the instants."""
        x = (t - self.t0) * 1e-6
        if self.measured:
            var = self.s ** 2 * (1 / self.n + (x - self.mx) ** 2 / self.sxx)
        else:
            var = self.s ** 2 / self.n + (self.DRIFT_PPM * (x - self.mx)) ** 2
        return t + self.offset + self.drift * x, math.sqrt(var + self.floor ** 2)

    @property
    def skew_ppm(self):
        return self.drift if self.measured else None


def _band(pairs, share=0.4):
    """Line through the densest band of pairs' gaps, and how many pairs lie in it.

    Most of a set of delays crowd together; stragglers trail off to one side.
    The narrowest band holding `share` of them follows the crowd.
    """
    line = Line(pairs, "", 1.0)
    count = max(5, int(share * len(pairs)))
    for _ in range(3):
        gaps = sorted((p[1] - line(p[0])[0], p) for p in pairs)
        start = min(range(len(gaps) - count + 1), key=lambda i: gaps[i + count - 1][0] - gaps[i][0])
        line = Line([p for _, p in gaps[start:start + count]], "", 1.0)
    return line, count


class TwoWay:
    """Clock relation between two boards from beacons each sent the other.

    A board cannot stamp its own transmission, so each beacon's send time is
    early by however long the radio took to get it out. That delay is a
    millisecond or more for some beacons, but most leave after very nearly the
    same wait, and boards built alike share it. So in each direction the
    crowd of typical beacons marks a line, and the clock offset lies midway
    between the two.
    """
    method = "two-way radio"
    FLOOR_US = 20.0  # how far two boards' typical send delays may differ

    def __init__(self, fwd, back):
        # fwd: (t_a sent, t_b heard). back: (t_a heard, t_b sent).
        self.lo, n_lo = _band(fwd)
        self.hi, n_hi = _band(back)
        self.n, self.t0 = n_lo + n_hi, self.lo.t0

    def __call__(self, t):
        """t on a's clock -> (t on b's clock, 1-sigma in us)."""
        (lo, s_lo), (hi, s_hi) = self.lo(t), self.hi(t)
        return (lo + hi) / 2, math.hypot(math.hypot(s_lo, s_hi) / 2, self.FLOOR_US)

    @property
    def skew_ppm(self):
        drifts = [d for d in (self.lo.skew_ppm, self.hi.skew_ppm) if d is not None]
        return sum(drifts) / len(drifts) if drifts else None


MIN_MATCHES = 4      # sweeps each way before two boards count as hearing each other
MAX_SCATTER_US = 100  # and how well those must agree; chance pairings scatter by milliseconds


def _match(played, heard, bin_us=4000):
    """Pair one board's own sweeps with their arrivals at another: [(t_played, t_heard)].

    The boards share no clock, so the pairing is found from the sweeps
    themselves: only true arrivals keep the same gap sweep after sweep, because
    every board plays at random intervals.
    """
    gaps = collections.Counter((h - p) // bin_us for p in played for h in heard)
    if not gaps:
        return []
    best = max(gaps, key=lambda k: gaps[k] + gaps.get(k + 1, 0))
    if gaps[best] + gaps.get(best + 1, 0) < MIN_MATCHES:
        return []
    lo, hi = best * bin_us, (best + 2) * bin_us
    return [(p, h) for p in played for h in heard if lo <= h - p < hi]


class Clocks:
    """What every board reported about sweeps, beacons and calibration taps, each on its own sample clock."""

    RADIO_WINDOW_US = 20_000_000  # beacons this recent are fitted with one straight line
    # Two boards alone need longer: on real boards the two-way fit wandered by
    # 85 us over 10 s of beacons, 38 us over 20 s and 9 us over 40 s.
    TWO_WAY_WINDOW_US = 60_000_000
    TWO_WAY_MIN = 40              # beacons each way before the crowd can be told from the stragglers

    def __init__(self, window_us=60_000_000):
        self.window = window_us
        self.own, self.heard = {}, {}  # node -> [arrival us]
        self.known = {}  # (a, b), a < b -> [(t_a, t_b)]: instants timed on both clocks
        self.radio = {}  # rx node -> {(tx node, tx boot, tx seq): (t_tx, t_rx, flags)}

    def hear(self, rx, tx, boot, seq, t_tx, t_rx, flags=0):
        """Board rx's radio received a beacon that board tx sent."""
        got = self.radio.setdefault(rx, {})
        got.setdefault((tx, boot, seq), (t_tx, t_rx, flags))  # first reception wins: a replay is later
        if len(got) > 512:
            for k in [k for k, v in got.items() if v[1] < t_rx - self.TWO_WAY_WINDOW_US]:
                del got[k]

    def _beacons(self, node, window=None):
        got = self.radio.get(node, {})
        if not got:
            return {}
        newest = max(v[1] for v in got.values())
        return {k: v for k, v in got.items() if v[1] >= newest - (window or self.RADIO_WINDOW_US)}

    def beacons(self, node):
        """How many beacons the board heard lately, and how many of those the radio did not stamp."""
        got = self._beacons(node).values()
        return len(got), sum(1 for v in got if v[2] & RX_SOFT)

    def shared(self, a, b):
        """Beacons from other boards that both a and b heard: (t_a, t_b) of one instant, and whether stamped late."""
        got_a, got_b = self._beacons(a), self._beacons(b)
        both = [(got_a[k], got_b[k]) for k in got_a.keys() & got_b.keys()]
        sharp = [(x[1], y[1]) for x, y in both if not (x[2] | y[2]) & RX_SOFT]
        return (sharp, False) if len(sharp) >= 4 else ([(x[1], y[1]) for x, y in both], True)

    def exchanges(self, a, b):
        """Beacons a and b sent each other, each as (time on a's clock, time on b's): a's, then b's."""
        fwd = [(v[0], v[1]) for k, v in self._beacons(b, self.TWO_WAY_WINDOW_US).items() if k[0] == a]   # a sent, b heard
        back = [(v[1], v[0]) for k, v in self._beacons(a, self.TWO_WAY_WINDOW_US).items() if k[0] == b]  # a heard, b sent
        return fwd, back

    def calibrate(self, instants):
        """One tap at a known place: {node: when it happened, on that node's clock}."""
        nodes = sorted(instants)
        for i, a in enumerate(nodes):
            for b in nodes[i + 1:]:
                self.known.setdefault((a, b), []).append((instants[a], instants[b]))

    def uncalibrate(self):
        self.known = {}

    def add(self, node, t_us, mine):
        """Board `node` heard a sweep: its own, or someone else's."""
        times = (self.own if mine else self.heard).setdefault(node, [])
        times.append(t_us)
        if times[0] < t_us - 2 * self.window:
            times[:] = [t for t in times if t >= t_us - self.window]

    def forget(self, node):
        """The board's sample clock restarted: what it reported before is void."""
        self.own.pop(node, None)
        self.heard.pop(node, None)
        self.known = {pair: v for pair, v in self.known.items() if node not in pair}
        self.radio.pop(node, None)

    def _recent(self, times):
        return [t for t in times if t >= times[-1] - self.window] if times else []

    def counts(self, node):
        return len(self._recent(self.own.get(node, []))), len(self._recent(self.heard.get(node, [])))

    def link(self, a, b):
        """What the sweeps between a and b give, or None when they have not heard enough of each other."""
        own_a, own_b = self._recent(self.own.get(a, [])), self._recent(self.own.get(b, []))
        fwd = _match(own_a, self._recent(self.heard.get(b, [])))
        back = [(ta, tb) for tb, ta in _match(own_b, self._recent(self.heard.get(a, [])))]
        if not fwd or not back:
            return None
        link = Link(fwd, back)
        return link if link.ok and link.s <= MAX_SCATTER_US else None

    def fit(self, a, b, at=None):
        """Map from a's clock to b's, by whichever means is tightest at time `at` on a's clock.

        None when nothing relates the two clocks.
        """
        fits = []
        link = self.link(a, b)
        if link:
            fits.append(link)
        pairs, late = self.shared(a, b)
        if len(pairs) >= 4:
            # The sender's own delays reach both listeners alike and drop out.
            fits.append(Line(pairs, "radio", 100.0 if late else 2.0))
        else:
            # No third board to listen to: only the beacons the two sent each other.
            fwd, back = self.exchanges(a, b)
            if len(fwd) >= self.TWO_WAY_MIN and len(back) >= self.TWO_WAY_MIN:
                fits.append(TwoWay(fwd, back))
        known = self.known.get((a, b)) or [(tb, ta) for ta, tb in self.known.get((b, a), [])]
        if known:
            # A tap "next to" a board is a couple of centimetres from it: 60 us.
            fits.append(Line(known, "taps", 20.0, 60.0))
        if not fits:
            return None
        return fits[0] if at is None else min(fits, key=lambda f: f(at)[1])

    def edge(self, a, b, c, self_a=0.0, self_b=0.0, c_sigma=0.0, now_us=None):
        """Link measurements between a and b for display and for the layout.

        now_us: the present on a's clock, so that the agreement shown is today's.
        """
        fit = self.fit(a, b, now_us)
        if fit is None:
            return None
        out = {"a": a, "b": b, "sigma_us": fit(fit.t0 if now_us is None else now_us)[1],
               "skew_ppm": fit.skew_ppm, "samples": fit.n, "method": fit.method}
        link = self.link(a, b)
        if link:
            # Each board hears its own sweep after it has crossed from buzzer to
            # microphone, which shortens both gaps; add that back.
            distance = link.flight * 1e-6 * c + (self_a + self_b) / 2
            if distance >= -0.05:  # else an echo or a stranger: sound cannot arrive early
                out.update(distance_m=max(distance, 0.0),
                           distance_sigma_m=math.hypot(link.flight_sigma * 1e-6 * c, distance * c_sigma / c))
        return out


# ── Layout from distances ────────────────────────────────────────────


def layout(distances):
    """Pairwise distances {(a, b): metres} -> ({node: (x, y)}, rms misfit in metres).

    Positions are relative: any rotation, shift or mirror image fits as well.
    Boards with no measured path to the largest group are left out.
    """
    near = collections.defaultdict(dict)
    for (a, b), d in distances.items():
        near[a][b] = near[b][a] = d
    groups, seen = [], set()
    for start in sorted(near):
        if start not in seen:
            group, queue = [], [start]
            seen.add(start)
            while queue:
                node = queue.pop()
                group.append(node)
                for other in near[node]:
                    if other not in seen:
                        seen.add(other)
                        queue.append(other)
            groups.append(sorted(group))
    if not groups:
        return {}, 0.0
    nodes = max(groups, key=len)
    n = len(nodes)
    # Unmeasured pairs start as the shortest route through measured ones.
    d = [[0.0 if i == j else near[a].get(b, math.inf) for j, b in enumerate(nodes)] for i, a in enumerate(nodes)]
    for k in range(n):
        for i in range(n):
            for j in range(n):
                d[i][j] = min(d[i][j], d[i][k] + d[k][j])
    # Classical scaling: the two strongest axes of the centred squared distances.
    row = [sum(v * v for v in r) / n for r in d]
    total = sum(row) / n
    b = [[-0.5 * (d[i][j] ** 2 - row[i] - row[j] + total) for j in range(n)] for i in range(n)]
    axes = []
    for seed in range(2):
        v = [math.sin(1.0 + i * (seed + 1.7)) for i in range(n)]
        value = 0.0
        for _ in range(200):
            w = [sum(b[i][j] * v[j] for j in range(n)) for i in range(n)]
            for axis, _ in axes:  # stay clear of the axis already found
                dot = sum(x * y for x, y in zip(w, axis))
                w = [x - dot * y for x, y in zip(w, axis)]
            value = math.sqrt(sum(x * x for x in w))
            if value < 1e-12:
                break
            v = [x / value for x in w]
        axes.append((v, value))
    pos = [[axes[0][0][i] * math.sqrt(axes[0][1]), axes[1][0][i] * math.sqrt(axes[1][1])] for i in range(n)]
    # Then let the measured pairs alone pull the points into place.
    index = {node: i for i, node in enumerate(nodes)}
    pairs = [(index[a], index[b], dist) for (a, b), dist in distances.items() if a in index and b in index]
    for _ in range(300):
        for i, j, dist in pairs:
            dx, dy = pos[j][0] - pos[i][0], pos[j][1] - pos[i][1]
            now = math.hypot(dx, dy) or 1e-9
            push = 0.25 * (now - dist) / now
            pos[i][0] += push * dx
            pos[i][1] += push * dy
            pos[j][0] -= push * dx
            pos[j][1] -= push * dy
    misfit = math.sqrt(sum((math.hypot(pos[j][0] - pos[i][0], pos[j][1] - pos[i][1]) - dist) ** 2
                           for i, j, dist in pairs) / len(pairs))
    return {node: tuple(pos[index[node]]) for node in nodes}, misfit


def align(points, anchors):
    """Turn, shift and if need be mirror a layout onto boards whose place is known.

    points, anchors: {node: (x, y)}. With fewer than two anchors the first board
    goes to the origin and the second onto the x axis.
    """
    nodes = sorted(points)
    if not nodes:
        return {}
    p = {n: complex(*points[n]) for n in nodes}
    shared = [n for n in nodes if n in anchors]
    if len(shared) < 2:
        origin = p[nodes[0]]
        turn = 1 + 0j
        if len(nodes) > 1 and abs(p[nodes[1]] - origin) > 1e-9:
            turn = abs(p[nodes[1]] - origin) / (p[nodes[1]] - origin)
        out = {n: (p[n] - origin) * turn for n in nodes}
        if len(nodes) > 2 and out[nodes[2]].imag < 0:
            out = {n: v.conjugate() for n, v in out.items()}
        if len(shared) == 1:  # one known place: keep the shape, pin that board
            shift = complex(*anchors[shared[0]]) - out[shared[0]]
            out = {n: v + shift for n, v in out.items()}
        return {n: (v.real, v.imag) for n, v in out.items()}
    q = {n: complex(*anchors[n]) for n in shared}
    q_mid = sum(q.values()) / len(shared)
    best = None
    for mirror in (False, True) if len(shared) > 2 else (False,):
        m = {n: v.conjugate() if mirror else v for n, v in p.items()}
        m_mid = sum(m[n] for n in shared) / len(shared)
        spin = sum((q[n] - q_mid) * (m[n] - m_mid).conjugate() for n in shared)
        turn = spin / abs(spin) if abs(spin) > 1e-12 else 1 + 0j
        out = {n: (v - m_mid) * turn + q_mid for n, v in m.items()}
        miss = sum(abs(out[n] - q[n]) ** 2 for n in shared)
        if best is None or miss < best[0] - 1e-9:
            best = (miss, out)
    return {n: (v.real, v.imag) for n, v in best[1].items()}


# ── Arrival picking ──────────────────────────────────────────────────


def onset(samples, pre=96, frac=0.25):
    """Index where the tap first rises out of the noise, or None if there is no tap.

    The first `pre` samples are assumed to be background.
    """
    if len(samples) <= pre:
        return None
    dc = sum(samples[:pre]) / pre
    env = [abs(s - dc) for s in samples]
    noise = max(env[:pre]) or 1.0
    peak = max(env[pre:])
    if peak < 4 * noise:
        return None
    threshold = max(frac * peak, 2 * noise)
    return next(i for i in range(pre, len(env)) if env[i] >= threshold)


def lag(ref, sig, ref_at, sig_at, before=32, after=96, reach=24):
    """Samples by which sig's tap trails its coarse pick, aligned to ref's waveform.

    Normalised cross-correlation over a short window around each onset, so the
    direct sound decides and later echoes do not. Returns (lag, peak correlation).
    """
    def window(x, start, n):
        if start < 0 or start + n > len(x):
            return None
        w = x[start:start + n]
        m = sum(w) / n
        return [v - m for v in w]

    n = before + after
    r = window(ref, ref_at - before, n)
    if r is None:
        return 0.0, 0.0
    r_energy = math.sqrt(sum(v * v for v in r)) or 1.0
    score = {}
    for k in range(-reach, reach + 1):
        w = window(sig, sig_at - before + k, n)
        if w is not None:
            energy = math.sqrt(sum(v * v for v in w)) or 1.0
            score[k] = sum(a * b for a, b in zip(r, w)) / (r_energy * energy)
    if not score:
        return 0.0, 0.0
    k = max(score, key=score.get)
    fine = 0.0
    if k - 1 in score and k + 1 in score:  # parabola through the peak: sub-sample lag
        lo, mid, hi = score[k - 1], score[k], score[k + 1]
        if lo - 2 * mid + hi < 0:
            fine = 0.5 * (lo - hi) / (lo - 2 * mid + hi)
    return k + fine, score[k]


# ── Locating ─────────────────────────────────────────────────────────


def speed_of_sound(temp_c):
    return 331.3 + 0.606 * temp_c


def _solve3(m, v):
    """Solve the 3x3 system m x = v by Gaussian elimination; None if singular."""
    a = [row[:] + [v[i]] for i, row in enumerate(m)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-18:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        for r in range(3):
            if r != col:
                f = a[r][col] / a[col][col]
                a[r] = [x - f * y for x, y in zip(a[r], a[col])]
    return [a[i][3] / a[i][i] for i in range(3)]


def _descend(mics, ranges, sigmas, start):
    """Levenberg-Marquardt on (x, y, d0): ranges[i] = d0 + |p - mic_i|."""
    p, lam = list(start), 1e-3

    def residuals(q):
        return [(ranges[i] - q[2] - math.hypot(q[0] - mx, q[1] - my)) / sigmas[i]
                for i, (mx, my) in enumerate(mics)]

    r = residuals(p)
    cost = sum(v * v for v in r)
    jtj = None
    for _ in range(60):
        jac = []
        for i, (mx, my) in enumerate(mics):
            d = math.hypot(p[0] - mx, p[1] - my) or 1e-9
            jac.append([-(p[0] - mx) / d / sigmas[i], -(p[1] - my) / d / sigmas[i], -1 / sigmas[i]])
        jtj = [[sum(j[a] * j[b] for j in jac) for b in range(3)] for a in range(3)]
        jtr = [sum(j[a] * v for j, v in zip(jac, r)) for a in range(3)]
        damped = [[jtj[a][b] + (lam * jtj[a][a] if a == b else 0) for b in range(3)] for a in range(3)]
        step = _solve3(damped, [-v for v in jtr])
        if step is None:
            break
        q = [p[i] + step[i] for i in range(3)]
        r2 = residuals(q)
        cost2 = sum(v * v for v in r2)
        if cost2 < cost:
            done = max(abs(s) for s in step) < 1e-7
            p, r, cost, lam = q, r2, cost2, lam / 4
            if done:
                break
        else:
            lam *= 8
            if lam > 1e12:
                break
    return p, cost, jtj


def locate(mics, times, sigmas, c, c_sigma=0.0):
    """Source of a sound heard at mics [(x, y) m] at times [s], each +/- sigmas [s].

    Returns x, y, the position covariance (m^2), the misfit in sigmas, and a
    second candidate when the geometry allows two. None when it cannot be solved.
    """
    if len(mics) < 3:
        return None
    t_min = min(times)
    ranges = [(t - t_min) * c for t in times]
    range_sigmas = [max(s, 1e-9) * c for s in sigmas]
    cx = sum(m[0] for m in mics) / len(mics)
    cy = sum(m[1] for m in mics) / len(mics)
    size = max(math.hypot(m[0] - cx, m[1] - cy) for m in mics) or 1.0
    starts = [(cx, cy, -size)] + [
        (cx + k * size * math.cos(a), cy + k * size * math.sin(a), -k * size)
        for k in (1.5, 4) for a in (i * math.pi / 4 for i in range(8))
    ]
    found = []
    for s in starts:
        p, cost, jtj = _descend(mics, ranges, range_sigmas, s)
        if jtj and not any(math.hypot(p[0] - f[0][0], p[1] - f[0][1]) < 0.01 for f in found):
            found.append((p, cost, jtj))
    if not found:
        return None
    best_cost = min(f[1] for f in found)
    # Three mics can admit two exact answers; prefer the one nearer the array.
    exact = sorted((f for f in found if f[1] <= best_cost + 1.0),
                   key=lambda f: math.hypot(f[0][0] - cx, f[0][1] - cy))
    p, cost, jtj = exact[0]
    cols = [_solve3(jtj, [1.0 if i == j else 0.0 for i in range(3)]) for j in range(2)]
    if None in cols:
        return None
    cov = [[cols[0][0], cols[1][0]], [cols[0][1], cols[1][1]]]
    if c_sigma:  # a wrong speed of sound moves the answer; fold that shift into the region
        q, _, _ = _descend(mics, [(t - t_min) * (c + c_sigma) for t in times], range_sigmas, p)
        dx, dy = q[0] - p[0], q[1] - p[1]
        cov = [[cov[0][0] + dx * dx, cov[0][1] + dx * dy], [cov[1][0] + dx * dy, cov[1][1] + dy * dy]]
    out = {"x": p[0], "y": p[1], "cov": cov, "misfit": math.sqrt(cost)}
    if len(exact) > 1:
        out["alt"] = {"x": exact[1][0][0], "y": exact[1][0][1]}
    return out


def ellipse(cov, k=2.45):
    """95% region of a 2x2 covariance: (semi-major m, semi-minor m, angle rad)."""
    a, b, d = cov[0][0], cov[0][1], cov[1][1]
    mean, diff = (a + d) / 2, math.hypot((a - d) / 2, b)
    return (k * math.sqrt(max(mean + diff, 0)), k * math.sqrt(max(mean - diff, 0)),
            0.5 * math.atan2(2 * b, a - d))


def arrivals(taps, boards):
    """When one tap reached each placed board, on that board's own clock.

    taps: {node: parsed TAP message}. Returns {node: (us, correlation)}, or a
    string saying why not.
    """
    picks = {}
    for node, tap in taps.items():
        at = onset(tap["samples"])
        if node in boards and at is not None:
            picks[node] = at
    if len({taps[n]["rate"] for n in picks}) > 1:
        return "boards disagree on sample rate"
    if not picks:
        return {}
    wave_ref = max(picks, key=lambda n: max(abs(s) for s in taps[n]["samples"]))
    out = {}
    for n in sorted(picks):
        tap = taps[n]
        shift, corr = (0.0, 1.0) if n == wave_ref else lag(
            taps[wave_ref]["samples"], tap["samples"], picks[wave_ref], picks[n])
        out[n] = (tap["t0"] + (picks[n] + shift) * 1e6 / tap["rate"] - boards[n].get("bias_us", 0.0), corr)
    return out


def calibration(taps, boards, at, temp_c=15.0):
    """A tap made right next to board `at`: when it happened, on each board's clock.

    Returns ({node: us}, None) for Clocks.calibrate, or (None, why not).
    """
    heard = arrivals(taps, boards)
    if isinstance(heard, str):
        return None, heard
    if at not in heard or len(heard) < 2:
        return None, "a calibration tap must be heard by the board it is next to and one other"
    c = speed_of_sound(temp_c)
    here = (boards[at]["x"], boards[at]["y"])
    return {n: t - math.dist(here, (boards[n]["x"], boards[n]["y"])) / c * 1e6 for n, (t, _) in heard.items()}, None


def locate_tap(clocks, taps, boards, temp_c=15.0, temp_sigma_c=3.0, pick_sigma_us=20.0,
               pos_sigma_m=0.01):
    """Combine one tap report per board into a position.

    taps: {node: parsed TAP message}. boards: {node: {"x", "y", "bias_us"}}.
    Returns (result, None) or (None, why not). Two boards give a position on
    the line between them; three or more give one on the plane.
    """
    picked = arrivals(taps, boards)
    if isinstance(picked, str):
        return None, picked
    if len(picked) < 2:
        return None, f"heard by {len(picked)} placed board(s), need 2"
    nodes = sorted(picked)
    clock_ref = nodes[0]
    c = speed_of_sound(temp_c)
    mics, times, sigmas, heard = [], [], [], []
    for n in nodes:
        local, corr = picked[n]
        if n == clock_ref:
            t_ref, clock_sigma = local, 0.0
        else:
            # ponytail: the boards' clocks must be related directly. Chain
            # through a shared neighbour if taps reach boards that are not.
            link = clocks.fit(n, clock_ref, local)
            if link is None:
                return None, f"the clocks of {n} and {clock_ref} are not related yet: calibrate"
            # Boards built alike are assumed. If one has its buzzer further from
            # its microphone, half the extra travel time belongs in its bias_us.
            t_ref, clock_sigma = link(local)
        sigma = math.sqrt(clock_sigma ** 2 + pick_sigma_us ** 2 + (pos_sigma_m / c * 1e6) ** 2)
        mics.append((boards[n]["x"], boards[n]["y"]))
        times.append(t_ref)
        sigmas.append(sigma)
        heard.append({"node": n, "local_us": local, "clock_sigma_us": clock_sigma,
                      "sigma_us": sigma, "correlation": corr})
    first = min(times)
    for h, t in zip(heard, times):
        h["delay_us"] = t - first
    common = {"heard": heard, "speed": c, "first_node": heard[times.index(first)]["node"]}
    if len(nodes) == 2:
        # One time difference places the tap along the line, and only if it is on it.
        (ax, ay), (bx, by) = mics
        apart = math.hypot(bx - ax, by - ay)
        if apart < 1e-6:
            return None, "the two boards are placed on the same spot"
        from_a = (apart + (times[0] - times[1]) * 1e-6 * c) / 2
        sigma_m = math.hypot(sigmas[0], sigmas[1]) * 1e-6 * c / 2
        sigma_m = math.hypot(sigma_m, abs(from_a - apart / 2) * 0.606 * temp_sigma_c / c)
        if not -3 * sigma_m - 0.05 <= from_a <= apart + 3 * sigma_m + 0.05:
            return None, "not on the line between the two boards, or their clocks have drifted: calibrate again"
        return {"x": ax + (bx - ax) * from_a / apart, "y": ay + (by - ay) * from_a / apart,
                "major_m": 2.45 * sigma_m, "minor_m": 0.0, "angle": math.atan2(by - ay, bx - ax),
                "misfit": 0.0, "line": True, **common}, None
    fix = locate(mics, [(t - first) * 1e-6 for t in times], [s * 1e-6 for s in sigmas],
                 c, 0.606 * temp_sigma_c)
    if fix is None:
        return None, "geometry cannot be solved"
    major, minor, angle = ellipse(fix["cov"])
    fix.update(major_m=major, minor_m=minor, angle=angle, **common)
    del fix["cov"]
    return fix, None
