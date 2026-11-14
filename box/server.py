"""Tap locator box app: takes reports from the boards over UDP, serves the live map.

The boards range each other and agree their clocks by sound; this works out
where they are from those distances, and where each tap came from.

Stdlib only. Run: TAPLOC_KEY=<64 hex chars> python3 server.py
"""
import collections
import ctypes
import itertools
import json
import math
import os
import socket
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import taploc

HERE = Path(__file__).parent
UDP_PORT = int(os.environ.get("TAPLOC_UDP_PORT", 47800))
HTTP_PORT = int(os.environ.get("PORT", 8000))
CONFIG = Path(os.environ.get("TAPLOC_CONFIG", "/data/config.json"))
DEFAULTS = {"temp_c": 15.0, "temp_sigma_c": 3.0, "self_m": 0.02, "boards": {}}
LIVE_US = 20_000_000  # boards report every few seconds to keep their radios asleep


def number(value, low, high, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
        raise ValueError(f"{what} must be a number between {low} and {high}")
    return float(value)


def validate(cfg):
    """Config as typed into the page -> clean config. Raises ValueError with the reason."""
    if not isinstance(cfg, dict) or not isinstance(cfg.get("boards", {}), dict):
        raise ValueError("expected an object with a 'boards' object")
    out = {"temp_c": number(cfg.get("temp_c", 15.0), -40, 60, "temp_c"),
           "temp_sigma_c": number(cfg.get("temp_sigma_c", 3.0), 0, 30, "temp_sigma_c"),
           "self_m": number(cfg.get("self_m", 0.02), 0, 1, "self_m"),
           "boards": {}}
    if len(cfg.get("boards", {})) > 16:
        raise ValueError("at most 16 boards")
    for node, b in cfg.get("boards", {}).items():
        try:
            if len(bytes.fromhex(node.replace(":", ""))) != 6:
                raise ValueError
        except (ValueError, AttributeError):
            raise ValueError(f"board id {node!r} is not a MAC address") from None
        if not isinstance(b, dict):
            raise ValueError(f"board {node} must be an object")
        name = b.get("name", "")
        if not isinstance(name, str) or len(name) > 16:
            raise ValueError(f"board {node}: name must be text of at most 16 characters")
        out["boards"][node.lower()] = {
            "name": name,
            "x": number(b.get("x"), -1000, 1000, f"board {node} x"),
            "y": number(b.get("y"), -1000, 1000, f"board {node} y"),
            "bias_us": number(b.get("bias_us", 0.0), -100_000, 100_000, f"board {node} bias_us"),
        }
    return out


class _Timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("rest", ctypes.c_byte * 256)]


def kernel_clock():
    """What the Linux kernel says about its own clock, or None where it will not say."""
    try:
        tx = _Timex()
        state = ctypes.CDLL(None, use_errno=True).adjtimex(ctypes.byref(tx))
    except (AttributeError, OSError):
        return None
    if state < 0:
        return None
    return {"max_error_us": tx.maxerror, "synced": state != 5 and not tx.status & 0x40}


def clean(value):
    """JSON has no infinity; a spread that cannot be measured becomes null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, collections.deque)):
        return [clean(v) for v in value]
    return value


class State:
    def __init__(self, key, config):
        self.key, self.config = key, config
        self.lock = threading.Lock()
        self.clocks = taploc.Clocks()
        self.nodes = {}
        self.pending = {}
        self.taps = collections.deque(maxlen=50)
        self.surveyed = (0, [], {}, 0.0)
        self.calibrating = None  # the board that taps are being made next to, to set the clocks

    def ingest(self, data, wall_us, reply):
        msg = taploc.parse(self.key, data)
        if msg is None:
            return
        with self.lock:
            node = self.nodes.setdefault(msg["node"], {
                "boot": 0, "top": 0, "recent": collections.deque(maxlen=64), "epoch": None,
                "rtt": collections.deque(maxlen=64), "wall": collections.deque(maxlen=64)})
            # A report can arrive twice, and out of order, when a neighbouring
            # board carries it as well. Each (boot, seq) counts once, and
            # nothing older than the last 64 is taken, so a recording cannot
            # be replayed.
            boot, seq = msg["boot"], msg["seq"]
            if boot < node["boot"] or (boot == node["boot"] and (seq in node["recent"] or seq <= node["top"] - 64)):
                return
            if boot != node["boot"]:  # rebooted: its clock restarted
                node.update(boot=boot, top=0, epoch=None)
                node["recent"].clear()
                node["rtt"].clear()
            node["recent"].append(seq)
            node["top"], node["seen"] = max(node["top"], seq), wall_us
            epochs = {taploc.TAP: lambda: [msg["epoch"]], taploc.CHIRPS: lambda: [c[3] for c in msg["chirps"]]}.get(
                msg["kind"], list)()
            for epoch in epochs:
                if epoch != node["epoch"]:  # samples were lost: its sample clock slipped
                    self.clocks.forget(msg["node"])
                    node["wall"].clear()
                    node["epoch"] = epoch
            node["wall"].append(wall_us - msg["t_send"])
            if msg["kind"] == taploc.TAP:
                # ponytail: one tap at a time. Reports within 0.6 s are taken as the
                # same tap; cluster by arrival time if taps come faster than that.
                if not self.pending:
                    threading.Timer(0.6, self.solve).start()
                self.pending.setdefault(msg["node"], msg)
            elif msg["kind"] == taploc.OBSERVATIONS:
                for obs in msg["obs"]:
                    self.clocks.hear(msg["node"], *obs)
            else:
                if msg["box_rtt_us"]:
                    node["rtt"].append(msg["box_rtt_us"])
                for t_us, _, mine, epoch in msg["chirps"]:
                    if epoch == node["epoch"]:
                        self.clocks.add(msg["node"], t_us, mine)
        # The echo lets the board time its round trip to us and learn our address.
        reply(taploc.pack(self.key, taploc.ECHO, bytes.fromhex(msg["node"].replace(":", "")),
                          msg["boot"], msg["seq"], msg["t_send"]))

    def wall(self, node, local_us):
        """Board-local time -> (wall clock us, error bound us or None if unknown)."""
        n = self.nodes[node]
        rtt = min(n["rtt"]) if n["rtt"] else None
        # Network delay only ever adds, so the smallest gap is the truest one.
        wall = local_us + min(n["wall"]) - (rtt / 2 if rtt else 0)
        kernel = kernel_clock()
        bound = rtt / 2 + kernel["max_error_us"] if rtt and kernel and kernel["synced"] else None
        return wall, bound

    def survey(self, now):
        """Links between boards, where that puts them, and how well it all fits. Call with the lock held."""
        # ponytail: every pair is refitted from scratch, at most every 2 s. Keep
        # running fits per pair if the array grows past a few dozen boards.
        if now - self.surveyed[0] > 2_000_000:
            cfg = self.config
            c = taploc.speed_of_sound(cfg["temp_c"])
            edges = []
            for a, b in itertools.combinations(sorted(self.nodes), 2):
                ahead = self.nodes[a]["wall"]  # the present on a's clock, to age calibration taps
                edge = self.clocks.edge(a, b, c, cfg["self_m"], cfg["self_m"], 0.606 * cfg["temp_sigma_c"],
                                        now - min(ahead) if ahead else None)
                if edge:
                    edges.append(edge)
            points, misfit = taploc.layout({(e["a"], e["b"]): e["distance_m"] for e in edges if "distance_m" in e})
            placed = {n: (b["x"], b["y"]) for n, b in cfg["boards"].items()}
            self.surveyed = (now, edges, taploc.align(points, placed), misfit)
        return self.surveyed[1:]

    def solve(self):
        with self.lock:
            taps, self.pending = self.pending, {}
            cfg = self.config
            # A board nobody placed by hand sits where the sweeps put it.
            _, measured, _ = self.survey(time.time_ns() // 1000)
            boards = {n: {"x": x, "y": y} for n, (x, y) in measured.items()} | cfg["boards"]
            if self.calibrating:
                instants, why = taploc.calibration(taps, boards, self.calibrating, cfg["temp_c"])
                if instants:
                    self.clocks.calibrate(instants)
                    self.surveyed = (0, [], {}, 0.0)
                fix = {"calibration": self.calibrating, "first_node": self.calibrating,
                       "heard": [{"node": self.calibrating, "local_us": instants[self.calibrating]}]} if instants else None
            else:
                fix, why = taploc.locate_tap(self.clocks, taps, boards, cfg["temp_c"], cfg["temp_sigma_c"])
            if fix:
                first = fix["first_node"]
                local = next(h["local_us"] for h in fix["heard"] if h["node"] == first)
            else:
                first = min(taps)
                local = taps[first]["t0"]
            wall, bound = self.wall(first, local)
            entry = {"wall_us": wall, "wall_bound_us": bound, "heard_by": sorted(taps)}
            entry.update(fix or {"why": why})
            self.taps.appendleft(entry)
        print(json.dumps(clean(entry)), flush=True)

    def snapshot(self):
        now = time.time_ns() // 1000
        with self.lock:
            nodes = []
            for node, n in sorted(self.nodes.items()):
                rtt = sorted(n["rtt"])
                own, heard = self.clocks.counts(node)
                beacons, late = self.clocks.beacons(node)
                nodes.append({
                    "id": node, "boot": n["boot"], "age_s": (now - n["seen"]) / 1e6,
                    "live": now - n["seen"] < LIVE_US, "own": own, "heard": heard,
                    "beacons": beacons, "beacons_late": late,
                    "box_latency_us": rtt[0] / 2 if rtt else None,
                    "box_jitter_us": (statistics.median(rtt) - rtt[0]) / 2 if rtt else None,
                })
            edges, measured, misfit = self.survey(now)
            return clean({"now_us": now, "kernel_clock": kernel_clock(), "config": self.config,
                          "speed": taploc.speed_of_sound(self.config["temp_c"]),
                          "nodes": nodes, "edges": edges, "taps": self.taps, "calibrating": self.calibrating,
                          "measured": {n: {"x": x, "y": y} for n, (x, y) in measured.items()},
                          "misfit_m": misfit})

    def calibrate(self, at):
        """Start taking taps next to board `at` as clock calibration, or stop with None."""
        with self.lock:
            if at is not None and at not in self.nodes:
                raise ValueError("no such board")
            if at is not None:
                self.clocks.uncalibrate()  # old taps describe a drift that has since changed
            self.calibrating = at
            self.surveyed = (0, [], {}, 0.0)

    def set_config(self, cfg):
        cfg = validate(cfg)
        CONFIG.parent.mkdir(parents=True, exist_ok=True)
        tmp = CONFIG.with_suffix(".tmp")
        tmp.write_text(json.dumps(cfg, indent=2))
        os.replace(tmp, CONFIG)
        with self.lock:
            self.config = cfg
            self.surveyed = (0, [], {}, 0.0)


def handler(state):
    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, kind="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/state":
                self.send(200, json.dumps(state.snapshot()).encode())
            elif self.path == "/health":
                self.send(200, b"{}")
            else:
                self.send(404, b"{}")

        def do_POST(self):
            # Requiring JSON makes browsers refuse cross-site posts from other pages.
            length = int(self.headers.get("Content-Length") or 0)
            if self.path not in ("/config", "/calibrate") or self.headers.get("Content-Type") != "application/json":
                return self.send(404, b"{}")
            if not 0 < length <= 16384:
                return self.send(413, b'{"error": "config too large"}')
            try:
                body = json.loads(self.rfile.read(length))
                if self.path == "/config":
                    state.set_config(body)
                else:
                    state.calibrate(body.get("at") if isinstance(body, dict) else None)
            except (ValueError, OSError) as e:
                return self.send(400, json.dumps({"error": str(e)}).encode())
            self.send(200, b"{}")

        def log_message(self, *args):
            pass

    return Handler


def listen(state):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    while True:
        data, addr = sock.recvfrom(2048)
        try:
            state.ingest(data, time.time_ns() // 1000, lambda d: sock.sendto(d, addr))
        except Exception as e:  # one bad datagram must not stop the listener
            print(f"dropped datagram from {addr[0]}: {e!r}", file=sys.stderr, flush=True)


def main():
    text = os.environ.get("TAPLOC_KEY") or (HERE / "secret.key").read_text()
    key = bytes.fromhex(text.strip())
    if len(key) != 32:
        sys.exit("TAPLOC_KEY must be 64 hex characters")
    try:
        config = validate(json.loads(CONFIG.read_text()))
    except FileNotFoundError:
        config = DEFAULTS
    state = State(key, config)
    threading.Thread(target=listen, args=(state,), daemon=True).start()
    print(f"map on http://0.0.0.0:{HTTP_PORT}  boards report to udp/{UDP_PORT}", flush=True)
    ThreadingHTTPServer(("", HTTP_PORT), handler(state)).serve_forever()


if __name__ == "__main__":
    main()
