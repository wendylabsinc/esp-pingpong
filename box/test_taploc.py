"""Self-check: simulate drifting boards that sweep, and taps; expect the layout and the taps back.

    python3 test_taploc.py                 run the check
    python3 test_taploc.py HOST[:PORT]     stream the same simulation to a live server
"""
import math
import os
import random
import socket
import sys
import time

import taploc

KEY = bytes.fromhex(os.environ.get("TAPLOC_KEY", "00" * 32))
RATE = 16000
SELF_M = 0.02   # buzzer to microphone on every board
EARSHOT_M = 8.0  # how far a sweep carries
BOARDS = {  # node id -> position (m), clock start (us), clock rate error (ppm)
    bytes.fromhex("aa0000000001"): (0.0, 0.0, 0, 0.0),
    bytes.fromhex("aa0000000002"): (4.0, 0.0, 3_812_000, 12.0),
    bytes.fromhex("aa0000000003"): (2.0, 3.5, -911_000, -7.0),
    bytes.fromhex("aa0000000004"): (6.5, 3.0, 77_000_000, 21.0),
    bytes.fromhex("aa0000000005"): (-2.5, 4.0, 5_000, -15.0),  # out of earshot of board 4
}
C = taploc.speed_of_sound(15.0)


def name(node):
    return node.hex(":")


def local(node, true_us):
    _, _, start, ppm = BOARDS[node]
    return (true_us + start) * (1 + ppm * 1e-6)


def apart(a, b):
    return math.hypot(BOARDS[a][0] - BOARDS[b][0], BOARDS[a][1] - BOARDS[b][1])


def pulse(t):
    """A tap as a microphone hears it: a short decaying ring, t in seconds from arrival."""
    return 0.0 if t < 0 else (t / 0.0004) * math.exp(-t / 0.0004) * math.sin(2 * math.pi * 1800 * t)


def datagrams(rng, seconds, taps, boot=1, boards=BOARDS, buzzers=True, radio=False):
    """(true time us, datagram) in time order. taps: [(true us, x, y)]."""
    out = []
    seq = dict.fromkeys(boards, 0)
    if radio:
        # Each board beacons four times a second; every other board's radio stamps it.
        got = {n: [] for n in boards}  # node -> [(true us, record)]
        number = dict.fromkeys(boards, 0)
        for node in boards:
            t = rng.uniform(0, 250_000)
            while t < seconds * 1e6:
                number[node] += 1
                # Send-side delay the listeners never see, shaped as measured on real boards:
                # most beacons leave after nearly the same wait, a few sooner, and a tail much later.
                kind = rng.random()
                on_air = t + (rng.uniform(600, 950) if kind < 0.1 else
                              1000 + rng.gauss(0, 20) if kind < 0.55 else 1000 + rng.expovariate(1 / 1500))
                for other in boards:
                    if other != node:
                        got[other].append((on_air, (node, boot, number[node], round(local(node, t)),
                                                    round(local(other, on_air) + rng.gauss(0, 2)), 0)))
                t += rng.uniform(250_000, 312_000)
        # Boards send what they heard every five seconds, and at once after a tap.
        flushes = sorted([k * 5e6 for k in range(1, int(seconds / 5) + 1)] + [when + 350_000 for when, _, _ in taps])
        for node in boards:
            last = 0
            for now in flushes:
                batch = sorted((r for at, r in got[node] if last <= at < now), key=lambda r: r[4])
                last = now
                for i in range(0, len(batch), 40):
                    out.append((now + i, node, taploc.OBSERVATIONS, taploc.pack_obs(batch[i:i + 40])))
    heard = {n: [] for n in boards}  # node -> [(true us it was heard, record)]
    for node in boards:
        t = rng.uniform(0, 3e6) if buzzers else math.inf
        while t < seconds * 1e6:
            for other in boards:
                if other == node:
                    at = t + SELF_M / C * 1e6 + rng.gauss(0, 2)
                elif apart(node, other) <= EARSHOT_M:
                    at = t + apart(node, other) / C * 1e6 + rng.gauss(0, 8)
                else:
                    continue
                heard[other].append((at, (round(local(other, at) * 1000), 250, int(other == node), 0)))
            t += rng.uniform(2e6, 4e6)  # random gaps are what lets the box tell boards apart
        for _ in range(int(seconds / 20)):  # and now and then a board hears a sweep that was not one
            at = rng.uniform(0, seconds * 1e6)
            heard[node].append((at, (round(local(node, at) * 1000), 140, 0, 0)))
    for node in boards:
        for step in range(1, int(seconds / 5) + 1):  # a report every five seconds
            now = step * 5_000_000 + rng.uniform(0, 50_000)
            batch = sorted(r for at, r in heard[node] if now - 5_050_000 <= at < now)
            heard[node] = [(at, r) for at, r in heard[node] if at >= now]
            out.append((now, node, taploc.CHIRPS, taploc.pack_chirps(batch)))
    for when, x, y in taps:
        for node, (bx, by, _, _) in boards.items():
            dist = math.hypot(x - bx, y - by)
            arrival = local(node, when + dist / C * 1e6)
            t0 = round(arrival - (128 + rng.random()) * 1e6 / RATE)  # each board samples at its own phase
            samples = [round(6000 / max(dist, 0.3) * pulse((t0 + k * 1e6 / RATE - arrival) * 1e-6)
                             + rng.gauss(0, 8)) for k in range(512)]
            out.append((when + 60_000, node, taploc.TAP, taploc.pack_tap(t0, RATE, samples)))
    packed = []
    for sent, node, kind, payload in sorted(out, key=lambda d: d[0]):  # numbered in the order sent, as a board does
        seq[node] += 1
        packed.append((sent, taploc.pack(KEY, kind, node, boot, seq[node], round(local(node, sent)), payload)))
    return packed


def feed(clocks, data, taps=None):
    msg = taploc.parse(KEY, data)
    if msg["kind"] == taploc.CHIRPS:
        for t_us, _, mine, _ in msg["chirps"]:
            clocks.add(msg["node"], t_us, mine)
    elif msg["kind"] == taploc.OBSERVATIONS:
        for obs in msg["obs"]:
            clocks.hear(msg["node"], *obs)
    elif taps is not None:
        taps[msg["node"]] = msg


def check():
    rng = random.Random(1)
    assert taploc.parse(KEY, b"junk") is None
    nodes = sorted(BOARDS)
    worst = 0.0
    for x, y in [(1.6, 1.2), (2.4, 0.9), (0.9, 0.5), (3.0, 2.0), (5.5, 1.0)]:
        clocks, taps = taploc.Clocks(), {}
        for _, data in datagrams(rng, 60, [(57_000_000, x, y)]):
            assert taploc.parse(KEY, data[:-1] + bytes([data[-1] ^ 1])) is None, "forged tag accepted"
            feed(clocks, data, taps)
        # Distances and clocks against the truth the simulation knows.
        distances = {}
        for i, a in enumerate(nodes):
            for b in nodes[i + 1:]:
                edge = clocks.edge(name(a), name(b), C, SELF_M, SELF_M)
                if apart(a, b) > EARSHOT_M:
                    assert edge is None, "ranged two boards that cannot hear each other"
                    continue
                assert abs(edge["distance_m"] - apart(a, b)) < 0.01, (edge, apart(a, b))
                assert edge["distance_sigma_m"] < 0.01 and edge["sigma_us"] < 10, edge
                distances[name(a), name(b)] = edge["distance_m"]
                got, sigma = clocks.fit(name(a), name(b))(local(a, 57_000_000))
                assert abs(got - local(b, 57_000_000)) < 10, (got - local(b, 57_000_000), sigma)
        # Layout: three boards placed by hand pin the map; the rest follow.
        points, misfit = taploc.layout(distances)
        placed = taploc.align(points, {name(n): BOARDS[n][:2] for n in nodes[:3]})
        off = max(math.hypot(placed[name(n)][0] - BOARDS[n][0], placed[name(n)][1] - BOARDS[n][1]) for n in nodes)
        assert len(placed) == len(BOARDS) and off < 0.03 and misfit < 0.02, (off, misfit)
        boards = {n: {"x": p[0], "y": p[1]} for n, p in placed.items()}
        fix, why = taploc.locate_tap(clocks, {n: t for n, t in taps.items() if n in boards}, boards)
        assert fix, why
        err = math.hypot(fix["x"] - x, fix["y"] - y)
        print(f"tap ({x}, {y}) -> ({fix['x']:.3f}, {fix['y']:.3f})  error {err * 100:.1f} cm"
              f"  region {fix['major_m'] * 100:.1f} x {fix['minor_m'] * 100:.1f} cm"
              f"  layout off by {off * 100:.1f} cm")
        assert err < fix["major_m"] + 0.02, "true position outside the reported region"
        worst = max(worst, err)
    assert worst < 0.10, worst
    # No hand-placed boards: the layout still has the right shape.
    free = taploc.align(points, {})
    a, b = name(nodes[0]), name(nodes[3])
    assert free[a] == (0.0, 0.0) and abs(math.dist(free[a], free[b]) - apart(nodes[0], nodes[3])) < 0.03
    # Two boards alone still range each other, and refuse to locate.
    pair = {n: BOARDS[n] for n in nodes[:2]}
    clocks = taploc.Clocks()
    for _, data in datagrams(rng, 30, [], boards=pair):
        feed(clocks, data)
    edge = clocks.edge(name(nodes[0]), name(nodes[1]), C, SELF_M, SELF_M)
    assert abs(edge["distance_m"] - 4.0) < 0.01 and abs(edge["skew_ppm"] - 12) < 1, edge
    assert taploc.layout({(a, name(nodes[1])): 4.0})[0][name(nodes[1])] != (0.0, 0.0)
    assert taploc.locate_tap(clocks, {}, {})[0] is None
    # No buzzers at all: taps next to one board set the clocks, and later taps are placed.
    for count, where in ((3, "plane"), (2, "line")):
        some = {n: BOARDS[n] for n in nodes[:count]}
        placed = {name(n): {"x": v[0], "y": v[1]} for n, v in some.items()}
        knocks = [(t * 1e6, 0.03, -0.02) for t in (3, 9, 16, 24)] + [(40e6, 1.4, 0.0 if count == 2 else 0.9)]
        clocks, heard = taploc.Clocks(), []
        for _, data in datagrams(rng, 45, knocks, boards=some, buzzers=False):
            msg = taploc.parse(KEY, data)
            if msg["kind"] == taploc.TAP:
                heard.append(msg)
        *setting, last = [{m["node"]: m for m in heard[i:i + count]} for i in range(0, len(heard), count)]
        assert taploc.locate_tap(clocks, last, placed)[0] is None, "located a tap with unrelated clocks"
        for taps in setting:
            instants, why = taploc.calibration(taps, placed, name(nodes[0]))
            assert instants, why
            clocks.calibrate(instants)
        edge = clocks.edge(name(nodes[0]), name(nodes[1]), C, now_us=local(nodes[0], 40e6))
        assert edge["method"] == "taps" and edge["sigma_us"] < 100 and abs(edge["skew_ppm"] - 12) < 6, edge
        fix, why = taploc.locate_tap(clocks, last, placed)
        assert fix, why
        err = math.hypot(fix["x"] - 1.4, fix["y"] - knocks[-1][2])
        print(f"no buzzers, {count} boards: tap on the {where} off by {err * 100:.1f} cm"
              f" (region {fix['major_m'] * 100:.0f} cm, clocks +-{edge['sigma_us']:.0f} us)")
        assert err < 0.10 and err < fix["major_m"] + 0.03 and bool(fix.get("line")) == (count == 2), fix
    # No buzzers, radio beacons: three boards share broadcasts, two can only exchange them.
    for count, method, limit_us in ((3, "radio", 10), (2, "two-way radio", 40)):
        some = {n: BOARDS[n] for n in nodes[:count]}
        placed = {name(n): {"x": v[0], "y": v[1]} for n, v in some.items()}
        target = (1.4, 0.0 if count == 2 else 0.9)
        clocks, taps = taploc.Clocks(), {}
        for _, data in datagrams(rng, 64, [(62e6, *target)], boards=some, buzzers=False, radio=True):
            feed(clocks, data, taps)
        edge = clocks.edge(name(nodes[0]), name(nodes[1]), C, now_us=local(nodes[0], 62e6))
        got, _ = clocks.fit(name(nodes[0]), name(nodes[1]), local(nodes[0], 62e6))(local(nodes[0], 62e6))
        off = got - local(nodes[1], 62e6)
        assert edge["method"] == method and "distance_m" not in edge and abs(off) < limit_us, (edge, off)
        fix, why = taploc.locate_tap(clocks, taps, placed)
        assert fix, why
        err = math.hypot(fix["x"] - target[0], fix["y"] - target[1])
        print(f"no buzzers, {count} boards, {method}: clocks off by {off:+.0f} us (said +-{edge['sigma_us']:.0f}),"
              f" tap off by {err * 100:.1f} cm (region {fix['major_m'] * 100:.0f} cm)")
        assert err < 0.06 and err < fix["major_m"] + 0.02, fix
    print("ok")


def stream(target):
    host, _, port = target.partition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rng = random.Random()
    while True:  # each minute is a fresh "boot", numbered so the box never sees it as a replay
        taps = [(20_000_000 + i * 3_000_000, rng.uniform(0.5, 3.5), rng.uniform(0.3, 2.5)) for i in range(14)]
        start = time.time()
        for when, data in datagrams(rng, 62, taps, boot=int(start)):
            time.sleep(max(0, start + when / 1e6 - time.time()))
            sock.sendto(data, (host, int(port or 47800)))


if __name__ == "__main__":
    stream(sys.argv[1]) if len(sys.argv) > 1 else check()
