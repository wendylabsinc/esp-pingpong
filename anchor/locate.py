"""Where a sound came from.

Three things, in order:

  layout    the ranging boards' distances to each other (mesh.py) -> a flat map
            of the boards. Distances alone leave the map free to turn, shift
            and mirror; the first board goes to the origin, the second onto
            the x axis and the third above it.
  arrivals  the aligned audio of one event (anchor.py, events/<t>.wav) -> when
            the sound reached each board, relative to the others. Every pair of
            boards is cross-correlated; the peak nearest the difference of the
            two onsets wins, which is what settles a steady tone (its
            correlation repeats every cycle).
  solve     the point whose distances to the boards explain those arrival
            times: |p - board_i| = c * (t_i - t0), least squares over p and t0.

The map, flat-layout fit, solver and ellipse are box/taploc.py's (layout,
align, locate, ellipse), copied here because the Anchor image is built from
this directory alone.

    python3 locate.py                     self-check
    python3 locate.py sim                 expected error, from made-up boards and sounds
    python3 locate.py demo [port]         made-up boards to add and drag, sounds placed by a click: http://127.0.0.1:8093/mesh
    python3 locate.py wav <sweep|chord|click|tone> <file.wav>   a test sound to play from a phone, ten times
    python3 locate.py <events dir> <layout.json>    locate recorded events again
"""
import collections
import glob
import json
import math
import os
import sys
import threading
import time
import wave

import numpy as np

PICK_SIGMA_US = float(os.environ.get("LOCATE_PICK_SIGMA_US", 60))   # one arrival time, from `sim`
POS_SIGMA_MM = float(os.environ.get("LOCATE_POS_SIGMA_MM", 30))     # one board's place on the map
WINDOW_MS = float(os.environ.get("LOCATE_WINDOW_MS", 40))           # audio after the onset that is compared
CLOSURE_MAX_US = float(os.environ.get("LOCATE_CLOSURE_MAX_US", 150))  # pairs may disagree this much, no more
KEEP_SOURCES = 20
LAYOUT_FILE = "layout.json"


# ── Map of the boards (box/taploc.py) ────────────────────────────────


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


def map_boards(edges, previous=None):
    """Settled distances from mesh.state()["edges"] -> ({board: (x, y)}, misfit m), or (None, why).

    A flat map needs at least 2n - 3 distances for n boards (three for three).
    `previous` keeps the map from turning when a board joins or a distance moves.
    """
    dist = {(e["a"], e["b"]): e["mm"] / 1000 for e in edges if e.get("settled") and e.get("mm")}
    points, misfit = layout(dist)
    n = len(points)
    if n < 3:
        return None, f"{n} boards with settled distances, 3 needed"
    have = sum(1 for a, b in dist if a in points and b in points)
    if have < 2 * n - 3:
        return None, f"{have} settled distances between {n} boards, {2 * n - 3} needed"
    return align(points, previous or {}), misfit


# ── Arrival times from the aligned audio ─────────────────────────────


def _onset(x, rate):
    """-> (sample, fractional, where the sound rises to a quarter of its peak; samples it lasts; dB over the noise).

    Loudness is averaged over 2 ms, which keeps a noise spike from passing for
    the sound. The sample is None when nothing stands 10 dB out of the noise.
    This is only good to a fraction of a millisecond: it chooses between
    correlation peaks, it is not the arrival time.
    """
    k = max(1, int(rate * 0.002))
    env = np.convolve(np.abs(x), np.ones(k) / k, "same")
    quiet = env[k: max(2 * k, min(int(rate * 0.1), len(env) // 8))]
    noise = float(quiet.mean()) + 1e-9
    peak = float(env.max()) + 1e-9
    db = 20 * math.log10(peak / noise)
    level = noise + 0.25 * (peak - noise)
    rise = np.nonzero(env >= level)[0]
    if peak < 3 * noise or not len(rise) or rise[0] == 0:
        return None, 0, db
    i = int(rise[0])
    loud = np.nonzero(env[i:i + int(rate * WINDOW_MS / 1000)] >= noise + 0.1 * (peak - noise))[0]
    lasts = int(loud[-1]) + 1 if len(loud) else 0   # to the last loud moment: a beating sound dips in between
    return i - 1 + (level - env[i - 1]) / (env[i] - env[i - 1] + 1e-12), lasts, db


def _correlate(a, b, rate, up=8):
    """Cross-correlation of b against a, whitened in the band the sound occupies.

    -> (lags in seconds, values); a peak at +lag means b heard it that much later.
    """
    n = 1 << (2 * len(a) - 1).bit_length()
    fa, fb = np.fft.rfft(a, n), np.fft.rfft(b, n)
    cross = fb * np.conj(fa)
    mag = np.abs(cross)
    cross = cross / (mag + 0.05 * mag.max() + 1e-12)   # flat where the sound is, small where it is not
    hz = np.fft.rfftfreq(n, 1 / rate)
    cross[(hz < 300) | (hz > 0.45 * rate)] = 0
    cc = np.fft.irfft(cross, n * up)
    cc = np.concatenate((cc[-n * up // 2:], cc[: n * up // 2]))
    return (np.arange(len(cc)) - n * up // 2) / (rate * up), cc


def arrivals_whole(chans, rate, starts_us, reach_s, up=4, least=6.0):
    """Like arrivals(), for a sound with no start to see: each pair's whole recordings against each other.

    A steady or repeated sound adds up over the whole recording while the
    noise does not, so it is found well under the noise (two seconds of a
    4 kHz wide sound: some 40 dB). A peak must lie within what the boards'
    spacing allows and stand `least` times out of the rest. A reflection
    gives a pair a second peak (seen with a loudspeaker by a laptop's screen):
    each pair keeps its three best, and the combination in which the pairs
    agree with each other is taken, strongest first. A steady pitch has a peak
    per cycle and is refused. -> as arrivals(), or (None, why).
    """
    m = min(len(c) for c in chans)
    n = 1 << (2 * m - 1).bit_length()
    spec = [np.fft.rfft(np.asarray(c[:m], float) - float(np.mean(c[:m])), n) for c in chans]
    hz = np.fft.rfftfreq(n, 1 / rate)
    band = (hz >= 500) & (hz <= 0.45 * rate)
    count = len(chans)
    pairs, picks = [], []
    for i in range(count):
        for j in range(i + 1, count):
            cross = spec[j] * np.conj(spec[i])
            mag = np.abs(cross)
            cross = np.where(band, cross / (mag + np.median(mag[band]) + 1e-12), 0)
            cc = np.fft.irfft(cross, n * up)
            skew = (starts_us[j] - starts_us[i]) * 1e-6
            half = int((reach_s[i][j] + 0.0003 + abs(skew)) * rate * up)
            seg = np.concatenate((cc[-half:], cc[: half + 1]))          # lags -half .. +half
            rest = float(np.std(cc)) + 1e-12
            top = float(seg.max())
            if top < least * rest:
                return None, f"channels {i} and {j} do not line up on any sound (peak {top / rest:.1f} times the rest)"
            found, left = [], seg.copy()
            for _ in range(8):
                k = int(np.argmax(left))
                if left[k] < 0.5 * top:
                    break
                fine = 0.0
                if 0 < k < len(seg) - 1 and seg[k - 1] - 2 * seg[k] + seg[k + 1] < 0:
                    fine = 0.5 * (seg[k - 1] - seg[k + 1]) / (seg[k - 1] - 2 * seg[k] + seg[k + 1])
                found.append(((k - half + fine) / (rate * up) + skew, float(seg[k] / rest)))
                left[max(0, k - int(0.0002 * rate * up)): k + int(0.0002 * rate * up)] = 0
            if sum(1 for _, v in found if v >= 0.8 * top / rest) > 3:
                return None, f"channels {i} and {j} line up in many places: a steady pitch, not one sound"
            pairs.append((i, j))
            picks.append(found[:3])
    rows = []
    for i, j in pairs:
        row = [0.0] * count
        row[i], row[j] = -1.0, 1.0
        rows.append(row)
    a = np.array(rows)[:, 1:]

    def fit(choice):
        rhs = np.array([picks[p][c][0] for p, c in enumerate(choice)])
        t, *_ = np.linalg.lstsq(a, rhs, rcond=None)
        return t, float(np.sqrt(np.mean((a @ t - rhs) ** 2)) * 1e6), sum(picks[p][c][1] for p, c in enumerate(choice))

    choices = [tuple(0 for _ in pairs)]
    if count == 3:            # few enough to try every combination of each pair's peaks
        choices = [(x, y, z) for x in range(len(picks[0])) for y in range(len(picks[1])) for z in range(len(picks[2]))]
    fits = [(c,) + fit(c) for c in choices]
    # 40 us and no looser: at 100 us a loudspeaker 6 cm from one board (pairs 70 us apart; a grille 10 cm long is
    # no point that close) was placed in two different spots on two tries, one of them 4 m away.
    agreeing = [f for f in fits if f[2] <= 40.0] or ([fits[0]] if count == 2 else [])
    if not agreeing:
        return None, f"the pairs' peaks do not agree with each other (best {min(f[2] for f in fits):.0f} us apart)"
    choice, t, closure, strength = max(agreeing, key=lambda f: f[3])
    t = np.concatenate(([0.0], t))
    stand = [(i, j, picks[p][c][1]) for p, ((i, j), c) in enumerate(zip(pairs, choice))]
    return dict(times_us=[float(starts_us[0] + v * 1e6) for v in t], closure_us=closure if count > 2 else 0.0,
                level_db=[round(20 * math.log10(min(s for i, j, s in stand if c in (i, j))), 1) for c in range(count)],
                unclear=[]), None


def pair_delay(a, b, rate, coarse_s, reach_s):
    """How much later b heard it than a, in seconds -> (delay, peak height 0..1, True if another peak came close)."""
    lags, cc = _correlate(a, b, rate)
    keep = np.abs(lags) <= reach_s
    lags, cc = lags[keep], cc[keep]
    tops = np.nonzero((cc[1:-1] > cc[:-2]) & (cc[1:-1] >= cc[2:]))[0] + 1
    if not len(tops):
        return coarse_s, 0.0, True
    high = cc[tops].max()
    if high <= 0:
        return coarse_s, 0.0, True
    # A steady tone gives a peak every cycle, all about as high: the difference
    # between the two onsets picks among them. 2 ms off costs the whole height.
    score = cc[tops] / high - np.abs(lags[tops] - coarse_s) / 0.002
    k = int(tops[int(np.argmax(score))])
    lo, mid, hi = cc[k - 1], cc[k], cc[k + 1]
    fine = 0.5 * (lo - hi) / (lo - 2 * mid + hi) if lo - 2 * mid + hi < 0 else 0.0
    rivals = int(np.sum(cc[tops] >= 0.9 * high))
    return float(lags[k] + fine * (lags[1] - lags[0])), float(mid / high), rivals > 1


def arrivals(chans, rate, starts_us, reach_s):
    """Aligned audio of one sound -> when it reached each channel.

    chans: one array per board; sample n of channel i was taken at
    starts_us[i] + n / rate on the shared timeline. reach_s[i][j]: the longest
    the sound can take between boards i and j.
    -> dict(times_us=[...], closure_us, level_db=[...], unclear=[(i, j), ...]) or (None, why)
    """
    m = min(len(c) for c in chans)
    x = [np.asarray(c[:m], float) - float(np.mean(c[:m])) for c in chans]
    picks = [_onset(c, rate) for c in x]
    if any(p[0] is None for p in picks):
        return None, "no clear onset on " + ", ".join(str(i) for i, p in enumerate(picks) if p[0] is None)
    onsets = [p[0] for p in picks]
    # Compare the sound itself and little else: a short one in a long window is mostly noise.
    lasts = min(WINDOW_MS / 1000, max(0.003, max(p[1] for p in picks) / rate))
    lo = max(0, int(min(onsets) - 0.005 * rate))
    hi = min(m, int(max(onsets) + (lasts + 0.002) * rate))
    if hi - lo < 0.008 * rate:
        return None, "too little audio after the onset"
    fade = np.ones(hi - lo)
    edge = (hi - lo) // 8
    fade[:edge] = 0.5 - 0.5 * np.cos(np.pi * np.arange(edge) / edge)
    fade[-edge:] = fade[:edge][::-1]
    w = [c[lo:hi] * fade for c in x]
    n = len(chans)
    rows, rhs, unclear = [], [], []
    for i in range(n):
        for j in range(i + 1, n):
            skew = (starts_us[j] - starts_us[i]) * 1e-6
            coarse = (onsets[j] - onsets[i]) / rate
            delay, height, rival = pair_delay(w[i], w[j], rate, coarse, reach_s[i][j] + 0.0005 + abs(skew))
            if rival:
                unclear.append((i, j))
            row = [0.0] * n
            row[i], row[j] = -1.0, 1.0
            rows.append(row)
            rhs.append(delay + skew)
    # One time per channel that fits every pair; what is left over is the check.
    a = np.array(rows)[:, 1:]
    t, *_ = np.linalg.lstsq(a, np.array(rhs), rcond=None)
    left = a @ t - np.array(rhs)
    t = np.concatenate(([0.0], t))
    base = starts_us[0] + onsets[0] / rate * 1e6
    return dict(times_us=[float(base + (v - 0.0) * 1e6) for v in t],
                closure_us=float(np.sqrt(np.mean(left ** 2)) * 1e6) if n > 2 else 0.0,
                level_db=[round(p[2], 1) for p in picks], unclear=unclear), None


# ── The point that explains the arrival times (box/taploc.py) ────────


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


def solve(mics, times, sigmas, c):
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
    out = {"x": p[0], "y": p[1], "cov": [[cols[0][0], cols[1][0]], [cols[0][1], cols[1][1]]], "misfit": math.sqrt(cost)}
    if len(exact) > 1:
        out["alt"] = {"x": exact[1][0][0], "y": exact[1][0][1]}
    return out


def ellipse(cov, k=2.45):
    """95% region of a 2x2 covariance: (semi-major m, semi-minor m, angle rad)."""
    a, b, d = cov[0][0], cov[0][1], cov[1][1]
    mean, diff = (a + d) / 2, math.hypot((a - d) / 2, b)
    return (k * math.sqrt(max(mean + diff, 0)), k * math.sqrt(max(mean - diff, 0)),
            0.5 * math.atan2(2 * b, a - d))


def locate(points, nodes, chans, rate, starts_us, speed, sync_sigma_us=None):
    """One event -> where it came from on the map, or (None, why).

    points: {board: (x, y)} from map_boards; nodes: the board of each channel.
    """
    use = [i for i, n in enumerate(nodes) if n in points]
    if len(use) < 3:
        return None, f"{len(use)} of this sound's boards are on the map, 3 needed"
    # A board that sent audio without the sound standing out in it is left out, not allowed to sink the rest.
    clear = [i for i in use if _onset(np.asarray(chans[i], float) - float(np.mean(chans[i])), rate)[0] is not None]
    faint = len(clear) < 3
    if not faint:
        use = clear
    mics = [points[nodes[i]] for i in use]
    reach = [[math.dist(p, q) / speed for q in mics] for p in mics]
    if faint:   # nothing stands out: a steady or repeated sound may still be in there, under the noise
        got, why = arrivals_whole([chans[i] for i in use], rate, [starts_us[i] for i in use], reach)
        if got is None:
            return None, f"{len(clear)} mapped boards heard it clearly, 3 needed; and as a whole: {why}"
    else:
        got, why = arrivals([chans[i] for i in use], rate, [starts_us[i] for i in use], reach)
        # Three or more boards give each pair's delay twice over. When those disagree the starts were
        # picked on different things (seen: 1.5 ms apart with one board beside the loudspeaker and the
        # others barely hearing it); the whole recordings are tried, and failing that nothing is placed.
        if got is not None and got["closure_us"] > CLOSURE_MAX_US:
            use = [i for i, n in enumerate(nodes) if n in points]
            mics = [points[nodes[i]] for i in use]
            reach = [[math.dist(p, q) / speed for q in mics] for p in mics]
            apart = got["closure_us"]
            got, why = arrivals_whole([chans[i] for i in use], rate, [starts_us[i] for i in use], reach)
            if got is None:
                return None, f"the boards' pairs disagree by {apart:.0f} us; and as a whole: {why}"
            faint = True
    if got is None:
        return None, why
    sync = [(sync_sigma_us[i] or 0.0) if sync_sigma_us else 0.0 for i in use]
    out, why = place_times(points, [nodes[i] for i in use], got["times_us"], speed,
                           [math.sqrt(PICK_SIGMA_US ** 2 + s ** 2) for s in sync], got["level_db"])
    if out is not None:
        out.update(closure_us=round(got["closure_us"], 1), unclear=[[nodes[use[i]], nodes[use[j]]] for i, j in got["unclear"]])
        if faint:
            out["note"] = "faint sound"
    return out, why


def place_times(points, nodes, times_us, speed, sigma_us=None, levels=None):
    """Arrival times at mapped boards -> where the sound came from on the map, or (None, why).

    For a sound whose arrival times are already known: locate() gets them from
    the audio, a ranging round from its matched filter (ranging.py). sigma_us:
    how well each time is known, default PICK_SIGMA_US.
    """
    use = [i for i, n in enumerate(nodes) if n in points]
    if len(use) < 3:
        return None, f"{len(use)} of this sound's boards are on the map, 3 needed"
    mics = [points[nodes[i]] for i in use]
    times = [times_us[i] for i in use]
    # A board drawn 30 mm off looks the same to the solver as a sound that came 87 us off.
    sigmas = [math.sqrt((sigma_us[i] if sigma_us else PICK_SIGMA_US) ** 2 + (POS_SIGMA_MM / 1000 / speed * 1e6) ** 2) * 1e-6
              for i in use]
    fix = solve(mics, [t * 1e-6 for t in times], sigmas, speed)
    if fix is None:
        return None, "no point explains these arrival times"
    major, minor, angle = ellipse(fix["cov"])
    first = min(times)
    out = dict(x=round(fix["x"], 4), y=round(fix["y"], 4),
               ellipse=dict(major_m=round(major, 4), minor_m=round(minor, 4), angle_rad=round(angle, 4)),
               misfit=round(fix["misfit"], 2), spare=len(use) - 3, closure_us=0.0, unclear=[],
               boards=[dict(node=nodes[i], distance_mm=round(math.dist(points[nodes[i]], (fix["x"], fix["y"])) * 1000),
                            later_us=round(t - first, 1), level_db=levels[k] if levels else None)
                       for k, (i, t) in enumerate(zip(use, times))])
    if "alt" in fix:
        out["alt"] = dict(x=round(fix["alt"]["x"], 4), y=round(fix["alt"]["y"], 4))
    cx, cy = sum(m[0] for m in mics) / len(mics), sum(m[1] for m in mics) / len(mics)
    size = max(math.dist(m, (cx, cy)) for m in mics)
    # Well outside the boards the arrival times give a direction and hardly a distance. Right beside a
    # board the region also runs away along one line (seen with a loudspeaker 6 cm from a board: the
    # point was right and the region kilometres long), so the point's own distance decides, and the
    # region is drawn no longer than the boards are spread.
    away = math.dist((fix["x"], fix["y"]), (cx, cy))
    out["far"] = away > 3 * size or (major > size and away > 1.5 * size)
    out["ellipse"]["major_m"] = round(min(major, 2 * size), 4)
    out["ellipse"]["minor_m"] = round(min(minor, 2 * size), 4)
    out["bearing_rad"] = round(math.atan2(fix["y"] - cy, fix["x"] - cx), 4)
    return out, None


# ── Kept by the Anchor ───────────────────────────────────────────────


class Locator:
    """The map of the boards (kept on disk) and the last few sounds placed on it.

    The ranging firmware and the listening firmware are two firmwares today, so
    the map has to outlive the boards being reflashed: it is saved whenever the
    live distances make one, and read back at start. A board stays on the map
    once it is on it. A layout.json with "manual": true (boards placed by tape
    measure) is never overwritten.
    """

    def __init__(self, out_dir, log=print):
        self.path = os.path.join(out_dir, LAYOUT_FILE)
        self.lock = threading.Lock()
        self.log = log
        self.points, self.info = {}, {"from": None}
        self.sources = collections.deque(maxlen=KEEP_SOURCES)
        self.why = "no map of the boards yet"
        try:
            saved = json.load(open(self.path))
            self.points = {k: tuple(v) for k, v in saved["nodes"].items()}
            self.info = {"from": "manual" if saved.get("manual") else "saved", "misfit_mm": saved.get("misfit_mm"),
                         "saved_utc": saved.get("saved_utc")}
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def reset(self):
        """Forget the map and the sounds on it (not a map placed by hand). True when it was forgotten."""
        with self.lock:
            if self.info["from"] == "manual":
                return False
            self.points, self.info, self.why = {}, {"from": None}, "the distances are being measured"
            self.sources.clear()
            try:
                os.remove(self.path)
            except OSError:
                pass
            return True

    def update(self, state):
        """mesh.state() -> the map, redrawn and saved when the live distances make one."""
        with self.lock:
            if self.info["from"] == "manual":
                return
            points, misfit = map_boards(state["edges"], self.points)
            if points is None:
                self.why = misfit
                return
            # A board that has gone quiet (reflashed to listen, or out of radio reach) keeps its place:
            # the new map is laid over the old one, so the old place still fits. Delete layout.json to start over.
            points = dict(self.points, **points)
            moved = set(points) != set(self.points) or any(math.dist(points[n], self.points[n]) > 0.005 for n in points)
            self.points = points
            self.info = {"from": "live", "misfit_mm": round(misfit * 1000, 1),
                         "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            if moved:
                try:
                    with open(self.path + ".tmp", "w") as f:
                        json.dump({"nodes": {k: [round(v[0], 4), round(v[1], 4)] for k, v in points.items()},
                                   "misfit_mm": self.info["misfit_mm"], "saved_utc": self.info["saved_utc"], "manual": False,
                                   "edges_mm": {f"{e['a']}-{e['b']}": e["mm"] for e in state["edges"] if e.get("settled")}}, f, indent=1)
                    os.replace(self.path + ".tmp", self.path)
                except OSError as e:
                    self.log(f"layout: not saved: {e}")

    def event(self, key, nodes, chans, rate, starts_us, speed, sync_sigma_us=None):
        """Place one event; returns what goes into its json under "source"."""
        with self.lock:
            points = dict(self.points)
        if not points:
            return {"error": self.why}
        try:
            got, why = locate(points, nodes, chans, rate, starts_us, speed, sync_sigma_us)
        except Exception as e:  # a sound that cannot be placed must not stop the Anchor
            got, why = None, repr(e)
        if got is None:
            return {"error": why}
        got["event"], got["at"] = key, time.time()
        with self.lock:
            for old in [s for s in self.sources if s["event"] == key]:
                self.sources.remove(old)
            self.sources.append(got)
        return got

    def times(self, key, nodes, times_us, speed, note=None, sigma_us=None, at=None):
        """Place a sound from arrival times that are already known (a buzzer's sweep in a ranging round).
        `at`: when it was heard (time.time()), for sounds placed some time after they were recorded."""
        with self.lock:
            points = dict(self.points)
        if not points:
            return {"error": self.why}
        try:
            got, why = place_times(points, nodes, times_us, speed, sigma_us)
        except Exception as e:
            got, why = None, repr(e)
        if got is None:
            return {"error": why}
        got.update(event=key, at=time.time() if at is None else at, note=note)
        with self.lock:
            for old in [s for s in self.sources if s["event"] == key]:
                self.sources.remove(old)
            self.sources.append(got)
        return got

    def view(self, now=None):
        """What /mesh.json adds: the map and the sounds on it."""
        now = time.time() if now is None else now
        with self.lock:
            return {"layout": dict(self.info, nodes={k: [round(v[0], 4), round(v[1], 4)] for k, v in self.points.items()})
                    if self.points else None,
                    "layout_why": None if self.points else self.why,
                    "sources": [dict({k: v for k, v in s.items() if k != "at"}, age_s=round(now - s["at"], 2))
                                for s in self.sources]}


# ── Made-up sounds: self-check, expected error, demo ─────────────────


def sound(kind):
    """A test sound as a function of time in seconds (numpy array in, amplitude out)."""
    def ramp(t, dur, edge):
        return np.clip(np.minimum(t, dur - t) / edge, 0, 1) * ((t >= 0) & (t <= dur))
    if kind == "sweep":     # the ranging boards' own: 2 to 6 kHz in 64 ms
        return lambda t: np.sin(2 * np.pi * (2000 * t + 0.5 * (4000 / 0.064) * t * t)) * ramp(t, 0.064, 0.002)
    if kind == "tone":      # a steady pitch that fades in over 5 ms, as a video would play it
        return lambda t: np.sin(2 * np.pi * 2700 * t) * ramp(t, 0.3, 0.005)
    if kind == "chord":     # four pitches that share no cycle: sounds like one note, correlates like a sweep
        return lambda t: sum(np.sin(2 * np.pi * f * t) for f in (2700, 3300, 4100, 5300)) / 2 * ramp(t, 0.3, 0.005)
    if kind == "hiss":      # a steady broad noise (a fan, a rustle), the same every time it is asked for
        tones = np.random.default_rng(11).uniform((1500, 0), (7000, 2 * np.pi), (300, 2))
        return lambda t: sum(np.sin(2 * np.pi * f * t + p) for f, p in tones) / 12 * ramp(t + 0.3, 3.0, 0.005)
    if kind == "click":     # a clap or a tap: a few cycles of ringing, four times as loud as the others
        return lambda t: 4 * np.sin(2 * np.pi * 3500 * t) * np.exp(-np.maximum(t, 0) / 0.0007) * (t >= 0)
    raise ValueError(kind)


def record(kind, boards, source, speed, rate=32000, snr_db=20, echo=0.5, sync_us=1.0, rng=None, seconds=0.9, times=None):
    """What each board's microphone would send for a sound at `source` -> (chans, starts_us).

    Each board hears the sound after its distance, weaker with distance, plus
    one echo off a wall 0.4 to 1.2 m further and noise. Each channel starts at
    its own fraction of a sample, and its clock is `sync_us` off (sigma).
    `times`: instead, when each board took each sample, in seconds from the
    moment the sound was made (sim_boards.py: boards with clocks of their own).
    """
    rng = rng or np.random.default_rng()
    s = sound(kind)
    chans, starts = [], []
    near = max(0.3, min(math.dist(b, source) for b in boards))   # full scale at 0.3 m and closer
    for i, b in enumerate(boards):
        d = math.dist(b, source)
        start = rng.uniform(0, 1e6 / rate)                       # us, where this channel's sample 0 falls
        t = start * 1e-6 + np.arange(int(seconds * rate)) / rate - 0.25 if times is None else times[i]
        clock = rng.normal(0, sync_us) * 1e-6
        extra = rng.uniform(0.4, 1.2)
        x = near / max(d, near) * s(t - d / speed + clock) + echo * near / (d + extra) * s(t - (d + extra) / speed + clock)
        x = x + rng.normal(0, 10 ** (-snr_db / 20) / math.sqrt(2), len(t))
        chans.append(np.clip(np.round(x * 5000), -32768, 32767).astype(np.int16))
        starts.append(start)
    return chans, starts


def _spread(boards, source, kind, speed, ranging_mm, sync_us, trials, rng, snr_db=20):
    """`trials` made-up sounds: boards mapped from noisy distances, then located.

    -> (errors in metres, direction errors in degrees, how many had an unclear pair, how many were not placed)
    """
    names = [f"b{i}" for i in range(len(boards))]
    truth = dict(zip(names, boards))
    cx, cy = sum(b[0] for b in boards) / len(boards), sum(b[1] for b in boards) / len(boards)
    errs, turns, unclear, lost = [], [], 0, 0
    for _ in range(trials):
        edges = [dict(a=a, b=b, settled=True, mm=1000 * math.dist(truth[a], truth[b]) + rng.normal(0, ranging_mm))
                 for i, a in enumerate(names) for b in names[i + 1:]]
        points, _ = map_boards(edges)
        chans, starts = record(kind, boards, source, speed, sync_us=sync_us, rng=rng, snr_db=snr_db)
        got, why = locate(points, names, chans, 32000, starts, speed)
        if got is None:
            lost += 1
            continue
        unclear += bool(got["unclear"])
        placed = align(dict(points, src=(got["x"], got["y"])), truth)   # onto the true boards, to compare
        errs.append(math.dist(placed["src"], source))
        off = math.atan2(placed["src"][1] - cy, placed["src"][0] - cx) - math.atan2(source[1] - cy, source[0] - cx)
        turns.append(abs(math.degrees(math.atan2(math.sin(off), math.cos(off)))))
    return errs, turns, unclear, lost


def sim(trials=200, seed=1):
    speed = 343.4
    rng = np.random.default_rng(seed)
    three = [(0, 0), (1.3, 0), (0.6, 1.1)]
    four = [(0, 0), (1.3, 0), (1.4, 1.2), (0.1, 1.3)]
    spots = {"inside": (0.65, 0.45), "at the edge": (0.65, -0.2), "1 m outside": (0.65, -1.0), "3 m outside": (0.65, -3.0)}
    cm = lambda v: f"{v:7.1f} cm" if v < 500 else "    > 5 m "
    print(f"Made-up sounds, {trials} per row: ranging off by 30 mm (sigma) per distance, clocks by 1 us,\n"
          "sound 20 dB over the noise (a click 32 dB) with one echo at half strength. Error = distance from the true spot;\n"
          "direction = angle off, seen from the middle of the boards.\n")
    print(f"{'boards':<7}{'sound':<7}{'where':<13}{'median':>10}{'95%':>10}{'direction':>11}{'unclear':>9}{'not placed':>12}")
    for label, boards in (("3", three), ("4", four)):
        for kind in ("sweep", "chord", "click", "tone"):
            for where, spot in spots.items():
                errs, turns, unclear, lost = _spread(boards, spot, kind, speed, 30, 1.0, trials, rng)
                e = np.array(errs) * 100 if errs else np.array([np.nan])
                print(f"{label:<7}{kind:<7}{where:<13}{cm(np.median(e))}{cm(np.percentile(e, 95))}"
                      f"{np.median(turns) if turns else float('nan'):>8.1f} deg{100 * unclear / trials:>8.0f}%{lost:>12}")
    print("\nArrival-time picks alone (two boards 1.3 m apart, source 0.4 m from one, 1000 sounds):")
    for kind in ("sweep", "chord", "click", "tone"):
        d = []
        for _ in range(1000):
            chans, starts = record(kind, three[:2], (0.3, 0.25), speed, sync_us=0, rng=rng)
            got, _ = arrivals(chans, 32000, starts, [[0, 1.3 / speed], [1.3 / speed, 0]])
            if got:
                true = (math.dist(three[1], (0.3, 0.25)) - math.dist(three[0], (0.3, 0.25))) / speed * 1e6
                d.append(got["times_us"][1] - got["times_us"][0] - true)
        d = np.array(d)
        if not len(d):
            print(f"  {kind:<6} never stood 10 dB out of the noise")
            continue
        good = np.abs(d) < 150
        print(f"  {kind:<6} picked {len(d) / 10:5.1f}%, within 150 us: {100 * good.mean():5.1f}%, of those sigma {d[good].std():5.1f} us; "
              f"95% within {np.percentile(np.abs(d), 95):6.0f} us")


def check():
    speed = 343.4
    rng = np.random.default_rng(7)
    boards = {"aa01": (0.0, 0.0), "aa02": (1.3, 0.0), "aa03": (0.6, 1.1), "aa04": (1.5, 1.2)}
    names = sorted(boards)
    edges = [dict(a=a, b=b, settled=True, mm=round(1000 * math.dist(boards[a], boards[b])))
             for i, a in enumerate(names) for b in names[i + 1:]]
    points, misfit = map_boards(edges)
    assert misfit < 0.002 and points["aa01"] == (0.0, 0.0) and abs(points["aa02"][1]) < 1e-9 and points["aa03"][1] > 0
    assert all(abs(math.dist(points[a], points[b]) - math.dist(boards[a], boards[b])) < 0.003 for a in names for b in names)
    assert map_boards(edges[:2])[0] is None and map_boards([dict(e, settled=False) for e in edges])[0] is None
    # Exact arrival times: the solver alone.
    src = (0.4, 0.5)
    fix = solve([boards[n] for n in names], [math.dist(boards[n], src) / speed for n in names], [50e-6] * 4, speed)
    assert math.dist((fix["x"], fix["y"]), src) < 1e-4 and fix["misfit"] < 1e-3, fix
    # Each pair's delay from audio, sign included: the second board is further, so later.
    for kind, limit in (("sweep", 20), ("click", 20), ("tone", 40)):
        two = [boards["aa01"], boards["aa02"]]
        chans, starts = record(kind, two, (0.3, 0.2), speed, snr_db=40, echo=0, sync_us=0, rng=rng)
        got, why = arrivals(chans, 32000, starts, [[0, 1.3 / speed], [1.3 / speed, 0]])
        true = (math.dist(two[1], (0.3, 0.2)) - math.dist(two[0], (0.3, 0.2))) / speed * 1e6
        assert got and abs(got["times_us"][1] - got["times_us"][0] - true) < limit, (kind, got, true, why)
    # Audio to place, through the map made from distances.
    for kind in ("sweep", "click"):
        chans, starts = record(kind, [boards[n] for n in names], src, speed, snr_db=30, echo=0.3, rng=rng)
        got, why = locate(points, names, chans, 32000, starts, speed)
        assert got and math.dist((got["x"], got["y"]), src) < 0.03 and got["spare"] == 1, (kind, got, why)
        assert [b["node"] for b in got["boards"]] == names and abs(got["boards"][0]["distance_mm"] - 640) < 30
    # A steady hiss 6 dB under the noise: no start to see, placed from the whole two seconds.
    quiet, qstarts = record("hiss", [boards[n] for n in names], src, speed, snr_db=-6, echo=0.3, rng=rng, seconds=2.2)
    got, why = locate(points, names, quiet, 32000, qstarts, speed)
    assert got and got["note"] == "faint sound" and math.dist((got["x"], got["y"]), src) < 0.05, (got, why)
    got, why = locate(points, names, record("tone", [boards[n] for n in names], src, speed, snr_db=-6, rng=rng, seconds=2.2)[0], 32000, qstarts, speed)
    assert got is None, got                                           # a faint steady pitch: refused, not guessed
    assert locate(points, ["aa01", "aa02", "zz"], chans[:3], 32000, starts[:3], speed)[0] is None
    assert locate(points, names, [np.zeros(20000, np.int16)] * 4, 32000, [0] * 4, speed)[0] is None   # silence
    got, why = locate(points, names, chans[:3] + [np.zeros(len(chans[0]), np.int16)], 32000, starts, speed)   # one board deaf
    assert got and got["spare"] == 0 and math.dist((got["x"], got["y"]), src) < 0.05, (got, why)
    # The Anchor's keeper: saves the map, reads it back, leaves a manual one alone.
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        keep = Locator(d, log=lambda *_: None)
        assert keep.event("e1", names, chans, 32000, starts, speed) == {"error": "no map of the boards yet"}
        keep.update({"edges": edges})
        placed = keep.event("e1", names, chans, 32000, starts, speed)
        assert math.dist((placed["x"], placed["y"]), src) < 0.03 and keep.view()["sources"][0]["event"] == "e1"
        keep.event("e1", names, chans, 32000, starts, speed)
        assert len(keep.view()["sources"]) == 1
        again = Locator(d)
        assert again.info["from"] == "saved" and again.points.keys() == keep.points.keys()
        keep.update({"edges": edges[:2]})                    # the boards went away: the map stays
        assert len(keep.points) == 4
        before = dict(keep.points)
        keep.update({"edges": [e for e in edges if "aa04" not in (e["a"], e["b"])]})   # one board went quiet
        assert len(keep.points) == 4 and all(math.dist(before[n], keep.points[n]) < 0.005 for n in before)
        json.dump({"nodes": {"aa01": [0, 0], "aa02": [1, 0], "aa03": [0, 1]}, "manual": True}, open(again.path, "w"))
        manual = Locator(d)
        manual.update({"edges": edges})
        assert manual.info["from"] == "manual" and manual.points["aa02"] == (1, 0)
    print(__doc__.strip(), "\n\nself-check ok")


def read_event(stem):
    """events/<t>.json and .wav as anchor.py writes them -> (nodes, chans, rate, starts_us, sync sigmas)."""
    ev = json.load(open(stem + ".json"))
    with wave.open(stem + ".wav") as w:
        rate, nch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").reshape(-1, nch)
    return ([n["node"] for n in ev["nodes"]], [x[:, i] for i in range(nch)], rate,
            [n.get("start_us", ev["aligned_start_us"]) for n in ev["nodes"]], [n.get("sigma_us") for n in ev["nodes"]], ev)


def again(events_dir, layout_path, temp_c=20.0):
    saved = json.load(open(layout_path))
    points = {k: tuple(v) for k, v in saved["nodes"].items()}
    speed = 331.3 + 0.606 * temp_c
    for js in sorted(glob.glob(os.path.join(events_dir, "*.json"))):
        if not os.path.exists(js[:-5] + ".wav"):
            continue
        nodes, chans, rate, starts, sigmas, ev = read_event(js[:-5])
        if ev.get("timeline") != "reference node esp_timer":
            print(os.path.basename(js), "not on the shared-beacon timeline, skipped")
            continue
        got, why = locate(points, nodes, chans, rate, starts, speed, sigmas)
        print(os.path.basename(js), json.dumps(got) if got else why)


def test_wav(kind, path, rate=44100, times=10, every_s=2.0):
    """The made-up sound as a file to play near the boards: `times` of them, `every_s` apart."""
    one = sound(kind)(np.arange(int(every_s * rate)) / rate - 0.2)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.tile(np.clip(one * 20000, -32768, 32767).astype("<i2"), times).tobytes())


def demo(port):
    """Made-up boards to add, drag and remove, and sounds placed by a click, on http://127.0.0.1:<port>/mesh.

    Only the rings on the page come from where things really are. The distances
    between the boards come from ranging rounds (ranging.py) on recordings
    synthesised for the boards' true places, and every sound is synthesised at
    each board's microphone and placed from that audio alone, on the map those
    distances made. So a board that was just moved is wrong on the map until
    three rounds agree on its new distances, and sounds are placed wrongly
    until then, as they would be.
    """
    import http.server
    import itertools
    import tempfile
    import ranging
    from mesh import Mesh, PAGE, SPEED
    mesh = Mesh()
    mesh.dongle("I,dc5475aa00ff,ch=1")
    keep = Locator(tempfile.mkdtemp(prefix="locate-demo-"), log=lambda *_: None)
    ranger = ranging.Ranger(lambda line: None, log=lambda *_: None)
    lock = threading.Lock()
    START = {"dc5475aa0001": (0.0, 0.0), "dc5475aa0002": (1.27, 0.0), "dc5475aa0003": (0.4, 0.9), "dc5475aa0004": (1.6, 1.1)}
    boards = dict(START)
    sounds = {}                     # event -> where it really was
    numbers, heard_n = itertools.count(5), itertools.count(1)
    rng = np.random.default_rng()

    def frames():
        """The boards' true places, their places on the map, and the boards that have both."""
        with lock:
            truth = dict(boards)
        with keep.lock:
            drawn = dict(keep.points)
        return truth, drawn, [b for b in truth if b in drawn]

    def to_room(p):
        """A point on the map -> where that is in the made-up room (the map may be turned or mirrored)."""
        truth, drawn, both = frames()
        if len(both) < 2:
            return tuple(p)
        return align(dict({b: drawn[b] for b in both}, **{"~": tuple(p)}), {b: truth[b] for b in both})["~"]

    def to_map(points):
        """{name: true place} -> the same on the map."""
        truth, drawn, both = frames()
        if len(both) < 2 or not points:
            return dict(points)
        out = align(dict({b: truth[b] for b in both}, **{"~" + k: v for k, v in points.items()}), {b: drawn[b] for b in both})
        return {k[1:]: v for k, v in out.items() if k.startswith("~")}

    def rounds():
        while True:
            with lock:
                truth = dict(boards)
            order = sorted(truth)[:ranging.MAX_BOARDS]
            if len(order) >= 2:
                for b in order:
                    ranger.heard(b)
                rid = ranger.start(order)
                recs = ranging.record_round(truth, order, sync_us=30, rng=rng)
                with lock:
                    still = all(boards.get(b) == truth[b] for b in order)
                if still:           # a board moved while the round was on the air: that round is lost
                    for b in order:
                        pcm, start, rate = recs[b]
                        ranger.fragment(dict(node=b, bands_hit=ranging.RANGE_HIT | rid, rate=rate), pcm,
                                        dict(first_sample_node_esp_us=start, trigger_node_esp_us=0, trigger_ref_esp_us=0))
                    keep.update(ranging.merge(mesh.state(), ranger.state()))
            time.sleep(0.3)

    def forget(board, gone):
        """A moved board's old distances are history; a removed board leaves the map too."""
        with ranger.lock:
            for pair in [p for p in ranger.edges if board in p]:
                del ranger.edges[pair]
            if gone:
                ranger.seen.pop(board, None)
        with keep.lock:
            # Moved: off the map until it is ranged again, so the others hold the map still (needs three of them).
            if gone or len(keep.points) >= 4:
                keep.points.pop(board, None)

    def act(path, q):
        if path == "/demo/sound":
            with lock:
                truth = dict(boards)
            names = sorted(truth)
            src = to_room((float(q["x"]), float(q["y"])))
            kind = q.get("kind", "sweep")
            # 30 dB over the noise at the nearest board: a test sound played on purpose. At the simulation's
            # 20 dB a board 2 m further away no longer hears it and the sound is often not placed at all.
            chans, starts = record(kind, [truth[b] for b in names], src, SPEED, snr_db=30, rng=rng)
            key = f"s{next(heard_n)}"
            got = keep.event(key, names, chans, 32000, starts, SPEED)
            with lock:
                with keep.lock:
                    live = {s["event"] for s in keep.sources}
                for old in [k for k in sounds if k not in live]:
                    del sounds[old]                     # the last sound that could not be placed
                sounds[key] = src
            if "x" in got:
                got = dict(got, off_cm=round(100 * math.dist(to_room((got["x"], got["y"])), src), 1), kind=kind)
            return got
        if path == "/demo/board":
            place = to_room((float(q["x"]), float(q["y"])))     # before the lock: it takes the lock itself
            with lock:
                if len(boards) >= ranging.MAX_BOARDS:
                    return {"error": f"{ranging.MAX_BOARDS} boards is the most one ranging round takes"}
                name = f"dc5475aa{next(numbers):04x}"
                boards[name] = place
            return {"board": name}
        if path == "/demo/move":
            place = to_room((float(q["x"]), float(q["y"])))
            with lock:
                if q["id"] not in boards:
                    return {"error": "no such board"}
                boards[q["id"]] = place
            forget(q["id"], gone=False)
            return {"board": q["id"]}
        if path == "/demo/remove":
            with lock:
                boards.pop(q["id"], None)
            forget(q["id"], gone=True)
            return {"removed": q["id"]}
        if path == "/demo/reset":
            with lock:
                boards.clear()
                boards.update(START)
                sounds.clear()
            with ranger.lock:
                ranger.edges.clear()
                ranger.seen.clear()
            with keep.lock:
                keep.points.clear()
                keep.sources.clear()
            return {}
        if path == "/demo/clear":
            with keep.lock:
                keep.sources.clear()
            with lock:
                sounds.clear()
            return {}
        return None

    class Handler(http.server.BaseHTTPRequestHandler):
        def reply(self, body, kind="application/json", code=200):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path != "/mesh.json":
                return self.reply(open(PAGE, "rb").read(), "text/html")     # read each time: the page can be edited live
            with lock:
                truth, heard = dict(boards), dict(sounds)
            state = ranging.merge(mesh.state(), ranger.state())
            state["nodes"] = [n for n in state["nodes"] if n["id"] in truth]
            state["edges"] = [e for e in state["edges"] if e["a"] in truth and e["b"] in truth]
            state.update(keep.view(), demo=True, max_boards=ranging.MAX_BOARDS,
                         truth={"nodes": {k: [round(v[0], 4), round(v[1], 4)] for k, v in to_map(truth).items()},
                                "sounds": {k: [round(v[0], 4), round(v[1], 4)] for k, v in to_map(heard).items()}})
            self.reply(json.dumps(state).encode())

        def do_POST(self):
            try:
                q = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0) or b"{}")
                got = act(self.path, q)
            except (KeyError, ValueError, TypeError) as e:
                return self.reply(json.dumps({"error": f"bad request: {e!r}"}).encode(), code=400)
            if got is None:
                return self.reply(b'{"error": "no such action"}', code=404)
            self.reply(json.dumps(got).encode())

        def log_message(self, *args):
            pass

    threading.Thread(target=rounds, daemon=True).start()
    print(f"http://127.0.0.1:{port}/mesh", flush=True)
    http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    if sys.argv[1:2] == ["sim"]:
        sim(int(sys.argv[2]) if len(sys.argv) > 2 else 200)
    elif sys.argv[1:2] == ["demo"]:
        demo(int(sys.argv[2]) if len(sys.argv) > 2 else 8093)
    elif sys.argv[1:2] == ["wav"]:
        test_wav(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3:
        again(sys.argv[1], sys.argv[2])
    else:
        check()
