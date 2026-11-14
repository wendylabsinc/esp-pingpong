"""Distances between the ranging boards (../range), gathered for the Anchor.

Two boards that finish a ping-pong round each broadcast when they heard their
own sweep and the other's, on their own sample clock, and which board the
round was with (F_MEAS in ../range/main/main.c). The dongle passes every
ranging frame it hears to USB:

    R,<src mac>,<group>,<type>,<round>,<own_ns>,<heard_ns>,<with mac>

One such frame from each board of a round gives the distance, the way the
boards work it out themselves: the board with the lower MAC started the round
and measured a = heard - own, the other measured b = own - heard, and
d = c * (a - b) / 2 + self. The frames carry no tag, so anything on the channel
can forge a distance: the same trust the boards give each other.

    python3 mesh.py               self-check
    python3 mesh.py demo [port]   made-up boards on http://127.0.0.1:8092/mesh
"""
import collections
import itertools
import json
import math
import os
import random
import sys
import threading
import time

# Must match the boards (range/main/Kconfig.projbuild); each adds straight onto every distance.
TEMP_C = float(os.environ.get("RANGE_TEMP_C", 20))          # PP_TEMP_C10 / 10
SELF_M = float(os.environ.get("RANGE_SELF_MM", 40)) / 1000  # PP_SELF_MM: buzzer to microphone
SPEED = 331.3 + 0.606 * TEMP_C
MEAS = 3                                  # F_MEAS
KEEP, AGREE_MM, AGREE_MIN = 40, 100, 3    # as range/main/agree.h and main.c
LIVE_S, FORGET_S = 10, 300                # boards say hello every second or two
MAX_NODES, MAX_ROUNDS = 64, 64
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mesh.html")


def agree(mm, width=AGREE_MM, least=AGREE_MIN):
    """The shortest stretch `width` wide that holds at least `least` of `mm`, or
    failing that the one that holds the most -> (its median, how many are in
    it, how far apart they are). range/main/agree.h in Python. Shortest, not
    fullest: rounds that timed an echo agree with each other too and can
    outnumber the rest, but an echo only ever reads long."""
    s = sorted(mm)
    first = count = j = 0
    for i in range(len(s)):
        while j < len(s) and s[j] - s[i] <= width:
            j += 1
        if count >= least and s[i] - s[first] > width:
            break
        if j - i > count:
            first, count = i, j - i
    return s[first + count // 2], count, s[first + count - 1] - s[first]


class Mesh:
    def __init__(self):
        self.lock = threading.Lock()  # fed by the dongle reader, read by HTTP threads
        self.nodes = {}                          # mac -> when last heard, monotonic seconds
        self.rounds = collections.OrderedDict()  # (pair, round) -> {mac: (own_ns, heard_ns)}, None once used
        self.anchor = None                       # the dongle, from its "I" line
        self.edges = {}                          # (lower mac, higher mac) -> rounds between them

    def feed(self, line, now=None):
        """One "R" line from the dongle. Anything malformed is dropped."""
        now = time.monotonic() if now is None else now
        try:
            _, src, _group, kind, rnd, own, heard, other = line.split(",")
            kind, rnd, own, heard = int(kind), int(rnd), int(own), int(heard)
            if len(src) != 12 or len(other) != 12 or len(bytes.fromhex(src + other)) != 12:
                return
        except ValueError:
            return
        src, other = src.lower(), other.lower()
        with self.lock:
            if src not in self.nodes and len(self.nodes) >= MAX_NODES:
                return
            self.nodes[src] = now
            if kind != MEAS or other == src:
                return
            pair = lo, hi = tuple(sorted((src, other)))  # the lower MAC started the round
            got = self.rounds.setdefault((pair, rnd), {})
            if got is None or src in got:  # every frame is sent twice
                return
            got[src] = (own, heard)
            if len(self.rounds) > MAX_ROUNDS:
                self.rounds.popitem(last=False)
            if len(got) < 2:
                return
            self.rounds[pair, rnd] = None
            a = got[lo][1] - got[lo][0]
            b = got[hi][0] - got[hi][1]
            d = SPEED * (a - b) * 1e-9 / 2 + SELF_M
            edge = self.edges.setdefault((lo, hi), {"mm": collections.deque(maxlen=KEEP), "good": 0, "bad": 0, "at": now})
            if 1e8 <= a <= 3e9 and 1e8 <= b <= 3e9 and 0 <= d <= 20:  # the boards' own test
                edge["mm"].append(round(d * 1000))
                edge["good"] += 1
                edge["at"] = now
            else:
                edge["bad"] += 1

    def dongle(self, line):
        """The dongle's "I" line: I,<mac>,ch=<n>,..."""
        fields = line.split(",")
        if len(fields) > 2 and fields[2].startswith("ch="):
            self.anchor = {"id": fields[1], "channel": fields[2][3:]}

    def state(self, now=None):
        """What the page draws: boards heard on the radio and the distances between them."""
        now = time.monotonic() if now is None else now
        with self.lock:
            for mac in [m for m, t in self.nodes.items() if now - t > FORGET_S]:
                del self.nodes[mac]
            for pair in [p for p in self.edges if p[0] not in self.nodes or p[1] not in self.nodes]:
                del self.edges[pair]
            edges = []
            for (a, b), e in sorted(self.edges.items()):
                row = {"a": a, "b": b, "good": e["good"], "bad": e["bad"], "mm": None, "settled": False}
                if e["mm"]:
                    best, count, apart = agree(e["mm"])
                    row.update(mm=best, settled=count >= AGREE_MIN, agree=count, of=len(e["mm"]), apart_mm=apart,
                               last_mm=e["mm"][-1], age_s=round(now - e["at"], 1))
                edges.append(row)
            return {"temp_c": TEMP_C, "self_mm": SELF_M * 1000, "speed": SPEED, "edges": edges, "anchor": self.anchor,
                    "nodes": [{"id": m, "age_s": round(now - t, 1), "live": now - t <= LIVE_S}
                              for m, t in sorted(self.nodes.items())]}


def round_lines(lo, hi, rnd, metres):
    """The two lines the dongle prints for a round that measures `metres`."""
    b = 500_000_000  # the reply wait
    a = b + round(2 * (metres - SELF_M) / SPEED * 1e9)
    return (f"R,{lo},1,{MEAS},{rnd},{10**9},{10**9 + a},{hi}", f"R,{hi},1,{MEAS},{rnd},{5 * 10**9 + b},{5 * 10**9},{lo}")


def check():
    assert agree([500]) == (500, 1, 0) and agree([900, 100, 110, 905, 120]) == (110, 3, 20)
    echo = [2663, 1277, 2645, 2691, 1235, 16627, 2682, 1268, 2703, 4018, 2665, 1104]  # as range/test_agree.c
    assert agree(echo) == (1268, 3, 42) and agree(echo[:7])[:2] == (2682, 4)
    assert agree([1160, 1250, 1255, 1268, 1277, 1279, 1340, 2663, 2665]) == (1277, 6, 90)
    # The 22 rounds two boards printed at a taped 1270 mm (range/test_agree.c).
    rounds = [5848, 1614, 1287, 8239, 17387, 14003, 1331, 1254, 1263, 1264, 13547,
              13826, 1268, 1479, 1248, 7174, 1797, 14780, 1318, 1269, 2310, 1292]
    mesh, lo, hi = Mesh(), "aa0000000001", "aa0000000002"
    for i, mm in enumerate(rounds):
        first, second = round_lines(lo, hi, i + 1, mm / 1000)
        for line in (second, first, first, second, "R,junk", f"R,zz0000000001,1,3,1,2,3,{lo}", f"R,{lo},1,3,1,2,3", ""):
            mesh.feed(line, now=i)  # either order, second copies, rubbish
        edge, = mesh.state(now=i)["edges"]
        assert (edge["a"], edge["b"], edge["good"], edge["bad"]) == (lo, hi, i + 1, 0), edge
        assert abs(edge["last_mm"] - mm) <= 1, edge
        assert not edge["settled"] or 1248 <= edge["mm"] <= 1331, edge
    assert edge["settled"] and edge["agree"] >= 7, edge
    for line in round_lines(lo, hi, 99, 25.0):  # further than the boards believe
        mesh.feed(line, now=30)
    mesh.feed("R,AA0000000003,1,1,0,0,0,000000000000", now=30)  # a hello
    third = "aa0000000003"  # two more pairs reuse a round number: three boards, three distances
    for a, b, metres in ((lo, third, 0.8), (hi, third, 2.1)):
        for n in (200, 201, 202):
            for line in round_lines(a, b, n, metres):
                mesh.feed(line, now=30)
    state = mesh.state(now=31)
    assert [(e["a"], e["b"], e["mm"], e["settled"]) for e in state["edges"][1:]] == [
        (lo, third, 800, True), (hi, third, 2100, True)], state["edges"]
    mesh.dongle("I,dc5475aa00ff,ch=1,beacon=1")
    assert mesh.state(now=31)["anchor"] == {"id": "dc5475aa00ff", "channel": "1"}
    assert state["edges"][0]["bad"] == 1 and state["edges"][0]["good"] == len(rounds)
    assert [n["id"] for n in state["nodes"]] == [lo, hi, "aa0000000003"] and all(n["live"] for n in state["nodes"])
    gone = mesh.state(now=31 + FORGET_S)
    assert gone["nodes"] == [] and gone["edges"] == []
    print(__doc__.strip(), "\n\nself-check ok")


def demo(port):
    """Four made-up boards, through the same lines the dongle sends."""
    import http.server
    mesh, page = Mesh(), open(PAGE, "rb").read()
    mesh.dongle("I,dc5475aa00ff,ch=1")
    boards = {"dc5475aa0001": (0, 0), "dc5475aa0002": (1.27, 0), "dc5475aa0003": (0.4, 0.9), "dc5475aa0004": (1.6, 1.1)}

    def run():
        for rnd in itertools.count(1):
            for (lo, p), (hi, q) in itertools.combinations(sorted(boards.items()), 2):
                wrong = random.random() < 0.3  # as measured: about a third of rounds time an echo
                metres = random.uniform(0, 20) if wrong else math.dist(p, q) + random.gauss(0, 0.01)
                for line in round_lines(lo, hi, rnd, metres):
                    mesh.feed(line)
                time.sleep(0.3)

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            body, kind = (json.dumps(dict(mesh.state(), demo=True)).encode(), "application/json") if self.path == "/mesh.json" else (page, "text/html")
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    threading.Thread(target=run, daemon=True).start()
    print(f"http://127.0.0.1:{port}/mesh", flush=True)
    http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    if sys.argv[1:2] == ["demo"]:
        demo(int(sys.argv[2]) if len(sys.argv) > 2 else 8092)
    else:
        check()
