"""soundsync Anchor: receives node fragments through an ESP32 radio dongle
(soundsync-fw, beacon role) on USB serial, and places every sample on one
timeline.

Timeline: the dongle's esp_timer, in microseconds ("anchor time").
  sample index n -> node esp_timer   block stamps: least-squares slope, lower
                                     envelope ("now - duration" is late-only)
  node esp_timer -> anchor time      beacon seq: the node's on-device arrival
                                     time vs the anchor's esp_timer just before
                                     it sent that seq; densest band of the
                                     one-way delays (absolute, tens of us)
  node -> reference node             reference broadcast: both nodes' arrival
                                     times of the same beacons (robust line,
                                     about 1 us); used for relative alignment
Fragments whose triggers fall within EVENT_GAP of each other form one event:
events/<t>.json lists every node's trigger on the shared timeline, and
events/<t>.wav holds one channel per node, aligned.

usage: anchor.py <dongle-port> [out-dir]
"""
import base64, hashlib, hmac, json, os, struct, sys, threading, time, collections, wave
from datetime import datetime, timezone
import numpy as np
import serial
from clocks import Line, band

PORT = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ANCHOR_PORT", "/dev/ttyACM0")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.environ.get("OUT_DIR", "fragments")
os.makedirs(OUT, exist_ok=True)

# Shared key (soundsync-fw/genkey.sh). Fail closed: no key, no Anchor.
_key_text = os.environ.get("SOUNDSYNC_KEY") or open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "secret.key")).read()
KEY = bytes.fromhex(_key_text.strip())
assert len(KEY) == 32, "SOUNDSYNC_KEY must be 64 hex characters"
TAG_LEN, SSF_HDR = 8, 20
bad_tags = [0]

def tag_ok(frame):
    return len(frame) > TAG_LEN and hmac.compare_digest(
        hmac.new(KEY, frame[:-TAG_LEN], hashlib.sha256).digest()[:TAG_LEN], frame[-TAG_LEN:])

tx_seq = {}                      # beacon seq -> anchor esp_us before send
tx_lock = threading.Lock()
host_map = collections.deque(maxlen=200)   # (host_ns_mid, anchor_esp_us, half_rtt_ns)
pending_ping = {}
parts = {}                       # (src, frag) -> {idx: bytes}

def mac(b):
    return b.hex()

def lower_envelope_fit(x, y, bucket):
    """Fit y = a + b*x through the per-bucket minima of y (late-only noise)."""
    x = np.asarray(x, float); y = np.asarray(y, float)
    k = np.floor((x - x[0]) / bucket)
    pts = np.array([(x[k == i].mean(), y[k == i].min()) for i in np.unique(k)])
    if len(pts) >= 2:
        b, a = np.polyfit(pts[:, 0] - x[0], pts[:, 1], 1)
    else:
        b, a = 0.0, pts[0, 1]
    return lambda q: a + b * (np.asarray(q, float) - x[0]), b

def parse_blob(blob):
    h = struct.unpack_from("<4s6sHIIqIqIHH", blob, 0)
    magic, node, nb, frag, rate, first_n, ns, trig_n, hit, nst, nbc = h
    assert magic == b"SSM1", magic
    p = struct.calcsize("<4s6sHIIqIqIHH")
    bands = []
    for _ in range(nb):
        bands.append(dict(zip(("hz", "thr_db", "level_db", "floor_db"), struct.unpack_from("<4f", blob, p)))); p += 16
    stamps = [struct.unpack_from("<qq", blob, p + 16 * i) for i in range(nst)]; p += 16 * nst
    beacons = []
    for i in range(nbc):
        src, soft, seq, rx, arr = struct.unpack_from("<6sHIIq", blob, p); p += 24
        beacons.append((src.hex(), seq, rx, arr, soft))
    pcm = np.frombuffer(blob, dtype="<i2", count=ns, offset=p)
    return dict(node=node.hex(), frag=frag, rate=rate, first_n=first_n, n_samples=ns, trigger_n=trig_n,
                bands_hit=hit, bands=bands, stamps=stamps, beacons=beacons), pcm

beacon_hist = collections.defaultdict(dict)   # node -> {seq: arrival esp_us}
ref_node = [None]

def tx_pairs(arrivals):
    with tx_lock:
        return [(a, tx_seq[s]) for s, a in arrivals.items() if s in tx_seq]

def place(meta):
    """sample index -> node esp -> anchor time; plus the node->reference line."""
    st = np.array(meta["stamps"], float)
    if len(st) < 5:
        return None, "too few block stamps"
    n, esp = st[:, 0], st[:, 1]
    # The microphone clock is divided from the same crystal as esp_timer
    # (measured: exactly nominal), so the period is nominal. Fitting it from one
    # fragment's late-only block stamps adds microseconds of slope error.
    fitted_us_per_sample, _ = np.polyfit(n, esp, 1)
    b_us_per_sample = 1e6 / meta["rate"]
    resid = esp - b_us_per_sample * n
    a = resid.min()
    n2esp = lambda q: a + b_us_per_sample * q
    node = meta["node"]
    for src, seq, rx, arr, soft in meta["beacons"]:
        if not soft:
            beacon_hist[node][seq] = arr
    if len(beacon_hist[node]) > 6000:
        for k in sorted(beacon_hist[node])[:2000]:
            del beacon_hist[node][k]
    recent = {s: v for s, v in beacon_hist[node].items() if abs(v - n2esp(meta["trigger_n"])) < 30e6}
    pairs = tx_pairs(recent)
    if len(pairs) < 10:
        return None, f"only {len(pairs)} beacons matched to anchor sends"
    # One-way send times include Wi-Fi queueing and airtime that this cannot
    # separate from clock offset. Wire test 2026-11-14 (tools/anchor_truth.py):
    # placed time early by +772 us mean, 241 us std, 1.8 ms worst. So the
    # error floor is 1 ms: absolute Anchor time is millisecond-grade. Node-to-
    # node timing uses shared beacons instead (0.45 us std on the same wire).
    to_anchor = band(pairs, floor_us=1000.0)
    first, sig = to_anchor(n2esp(meta["first_n"]))
    trig, _ = to_anchor(n2esp(meta["trigger_n"]))
    out = dict(first_sample_anchor_us=first, trigger_anchor_us=trig, anchor_sigma_us=sig,
               trigger_node_esp_us=n2esp(meta["trigger_n"]), first_sample_node_esp_us=n2esp(meta["first_n"]),
               node_us_per_sample=b_us_per_sample, node_rate_hz_fitted=1e6 / fitted_us_per_sample,
               node_vs_anchor_ppm=to_anchor.ppm, beacons_matched=len(pairs),
               stamp_late_p95_us=float(np.percentile(resid - a, 95)))
    if ref_node[0] is None:
        ref_node[0] = node
    out["reference_node"] = ref_node[0]
    if node != ref_node[0]:
        ref = beacon_hist[ref_node[0]]
        common = [(recent[s], ref[s]) for s in recent if s in ref]
        if len(common) >= 10:
            rbs = Line(common, 1.0)
            t_ref, s_ref = rbs(out["trigger_node_esp_us"])
            out.update(trigger_ref_esp_us=t_ref, ref_sigma_us=s_ref, ref_ppm=rbs.ppm, ref_beacons=len(common))
    else:
        out.update(trigger_ref_esp_us=out["trigger_node_esp_us"], ref_sigma_us=0.0, ref_beacons=0)
    return out, None

def anchor_to_host(t_us):
    if not host_map:
        return None
    best = min(host_map, key=lambda m: m[2])
    return best[0] + (t_us - best[1]) * 1000  # host monotonic ns (ms-grade)

def write(meta, pcm, placed):
    d = os.path.join(OUT, meta["node"]); os.makedirs(d, exist_ok=True)
    stem = os.path.join(d, f"{int(placed['first_sample_anchor_us']) if placed else 0:014d}-f{meta['frag']}")
    with wave.open(stem + ".wav", "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(meta["rate"]); w.writeframes(pcm.tobytes())
    host_ns = anchor_to_host(placed["first_sample_anchor_us"]) if placed else None
    utc = None
    if host_ns is not None:
        utc = datetime.fromtimestamp(time.time() - (time.monotonic_ns() - host_ns) / 1e9, timezone.utc).isoformat()
    j = {k: v for k, v in meta.items() if k not in ("stamps",)}
    j.update(placed or {}); j["first_sample_utc_approx"] = utc
    j["timeline"] = "anchor dongle esp_timer, microseconds"
    with open(stem + ".json", "w") as f:
        json.dump(j, f, indent=1)
    return stem

EVENT_GAP_US = 500_000
events = []   # list of dict(t=anchor us, frags=[(meta, pcm, placed)])

def add_to_event(meta, pcm, placed):
    t = placed["trigger_anchor_us"]
    ev = next((e for e in events if abs(e["t"] - t) < EVENT_GAP_US), None)
    if ev is None:
        ev = {"t": t, "frags": []}
        events.append(ev)
        del events[:-50]
    ev["frags"] = [f for f in ev["frags"] if f[0]["node"] != meta["node"]] + [(meta, pcm, placed)]
    write_event(ev)

def write_event(ev):
    """Shared timeline for an event: the reference node's esp_timer when every
    fragment has a reference-broadcast line, else anchor time."""
    frs = ev["frags"]
    use_ref = all("trigger_ref_esp_us" in p for _, _, p in frs)
    d = os.path.join(OUT, "events"); os.makedirs(d, exist_ok=True)
    stem = os.path.join(d, f"{int(ev['t']):014d}")
    nodes, starts = [], []
    for meta, pcm, p in frs:
        if use_ref:
            shift = p["trigger_ref_esp_us"] - p["trigger_node_esp_us"]
            start = p["first_sample_node_esp_us"] + shift
        else:
            start = p["first_sample_anchor_us"]
        starts.append(start)
        nodes.append(dict(node=meta["node"], frag=meta["frag"], bands_hit=meta["bands_hit"],
                          trigger_us=(p["trigger_ref_esp_us"] if use_ref else p["trigger_anchor_us"]),
                          sigma_us=(p.get("ref_sigma_us") if use_ref else p["anchor_sigma_us"])))
    t0 = min(n["trigger_us"] for n in nodes)
    for n in nodes:
        n["trigger_minus_first_us"] = n["trigger_us"] - t0
    rate = frs[0][0]["rate"]
    begin = max(starts)
    length = min(st + len(pc) * 1e6 / rate for st, (_, pc, _) in zip(starts, frs)) - begin
    chans = []
    for st, (_, pc, _) in zip(starts, frs):
        k = int(round((begin - st) * rate / 1e6))
        chans.append(pc[k:k + max(0, int(length * rate / 1e6))])
    m = min(len(c) for c in chans) if chans else 0
    if m > 0:
        inter = np.stack([c[:m] for c in chans], axis=1).astype("<i2")
        with wave.open(stem + ".wav", "wb") as w:
            w.setnchannels(len(chans)); w.setsampwidth(2); w.setframerate(rate); w.writeframes(inter.tobytes())
    with open(stem + ".json", "w") as f:
        json.dump(dict(timeline="reference node esp_timer" if use_ref else "anchor esp_timer",
                       reference_node=ref_node[0], aligned_start_us=begin, channels=[n["node"] for n in nodes],
                       nodes=nodes), f, indent=1)
    if len(frs) > 1:
        print(f"event {int(ev['t'])}: {len(frs)} nodes, " + ", ".join(
            f"{n['node'][-4:]} {n['trigger_minus_first_us']:+.1f} us (±{n['sigma_us']:.1f})" for n in nodes), flush=True)

line_counts = collections.Counter()
dongle = [None]
dongle_lock = threading.Lock()
capture = {"f": None, "until": 0.0, "path": None}

def send_cmd(line):
    with dongle_lock:
        dongle[0].write((line.strip() + "\n").encode())

def reader(s):
    last = time.monotonic()
    while True:
        if time.monotonic() - last > 10:
            last = time.monotonic()
            pending_pieces = sum(len(v) for v in parts.values())
            print(f"stats: lines {dict(line_counts)}, bad tags {bad_tags[0]}, pieces waiting {pending_pieces} in {len(parts)} fragments", flush=True)
        try:
            raw = s.readline().decode(errors="replace").strip()
            if capture["f"]:
                if time.monotonic() > capture["until"]:
                    capture["f"].close(); capture["f"] = None
                    print(f"capture done: {capture['path']}", flush=True)
                elif raw:
                    capture["f"].write(f"{time.monotonic_ns()},{raw}\n")
            if raw:
                line_counts[raw[0] if raw[0] in "TFPAEI" else "?"] += 1
            if raw.startswith("I,"):
                print(raw, flush=True)
            handle(raw)
        except Exception as e:  # one bad line must not stop the anchor
            print(f"reader: {e!r}", flush=True)

def handle(line):
    if True:
        if not line:
            return
        kind = line[0]
        if kind == "T":
            _, seq, esp = line.split(",")
            with tx_lock:
                tx_seq[int(seq)] = int(esp)
                if len(tx_seq) > 20000:
                    for k in sorted(tx_seq)[:5000]:
                        del tx_seq[k]
        elif kind == "P":
            _, tok, esp = line.split(",")
            t0 = pending_ping.pop(tok, None)
            if t0:
                t1 = time.monotonic_ns()
                host_map.append(((t0 + t1) // 2, int(esp), (t1 - t0) // 2))
        elif kind == "F":
            _, src, b64 = line.split(",", 2)
            try:
                pkt = base64.b64decode(b64)
            except Exception:
                return
            if not tag_ok(pkt):
                bad_tags[0] += 1
                if bad_tags[0] in (1, 10, 100) or bad_tags[0] % 1000 == 0:
                    print(f"dropped {bad_tags[0]} fragment pieces with bad tags", flush=True)
                return
            frag, idx, count, blen, boot = struct.unpack_from("<IHHII", pkt, 4)
            key = (src, boot, frag)
            parts.setdefault(key, {})[idx] = pkt[SSF_HDR:-TAG_LEN]
            if len(parts[key]) == count:
                got = parts.pop(key)
                blob = b"".join(got[i] for i in range(count))[:blen]
                try:
                    meta, pcm = parse_blob(blob)
                    placed, why = place(meta)
                    stem = write(meta, pcm, placed)
                    if placed:
                        add_to_event(meta, pcm, placed)
                    peak = int(np.abs(pcm.astype(np.int32)).max()) if len(pcm) else 0
                    if placed:
                        print(f"fragment {src} #{frag}: {meta['n_samples']/meta['rate']:.2f} s, bands_hit={meta['bands_hit']:#x}, "
                              f"first sample at anchor {placed['first_sample_anchor_us']/1e6:.6f} s, node {placed['node_rate_hz_fitted']:.3f} Hz (fit), "
                              f"{(placed['node_vs_anchor_ppm'] or 0):+.1f} ppm, ±{placed['anchor_sigma_us']:.0f} us, {placed['beacons_matched']} beacons, peak {peak} -> {stem}.wav", flush=True)
                    else:
                        print(f"fragment {src} #{frag}: not placed ({why}) -> {stem}.wav", flush=True)
                except Exception as e:
                    print(f"fragment {src} #{frag}: parse error {e}", flush=True)

def ping_map(host_ns):
    """Base-station monotonic ns -> dongle esp_timer us, from the tightest
    recent USB ping round trip (half its width is the bound)."""
    if not host_map:
        return None, None
    best = min(host_map, key=lambda m: m[2])
    return best[1] + (host_ns - best[0]) / 1000.0, best[2] / 1000.0

def play_chirp(f0, f1, ms, gain):
    """Play a linear chirp on the base station's speaker and record when its
    first sample reached the DAC, on the monotonic clock (PortAudio's
    outputBufferDacTime, mapped by bracketing the stream clock between two
    monotonic reads) and on the dongle's clock (USB ping map)."""
    import sounddevice as sd
    rate = 48000
    n = int(rate * ms / 1000)
    t = np.arange(n) / rate
    chirp = (gain * np.sin(2 * np.pi * (f0 * t + (f1 - f0) / (ms / 1000) * t ** 2 / 2))).astype(np.float32)
    lead = int(0.2 * rate)                     # 200 ms of silence first
    buf = np.concatenate([np.zeros(lead, np.float32), chirp, np.zeros(int(0.2 * rate), np.float32)])
    st = {"pos": 0, "dac": None, "map": None}
    def cb(out, frames, tinfo, status):
        pos = st["pos"]
        chunk = buf[pos:pos + frames]
        out[:len(chunk), 0] = chunk; out[len(chunk):, 0] = 0
        if st["dac"] is None and pos <= lead < pos + frames:
            a = time.monotonic_ns(); s_now = stream.time; b = time.monotonic_ns()
            st["map"] = ((a + b) / 2) - s_now * 1e9              # monotonic ns - stream s
            st["dac"] = tinfo.outputBufferDacTime + (lead - pos) / rate
        st["pos"] = pos + frames
        if st["pos"] >= len(buf):
            raise sd.CallbackStop
    stream = sd.OutputStream(samplerate=rate, channels=1, dtype="float32", callback=cb, latency="low")
    with stream:
        while stream.active:
            time.sleep(0.02)
    if st["dac"] is None:
        return {"error": "no DAC time"}
    mono_ns = st["dac"] * 1e9 + st["map"]
    esp_us, bound = ping_map(mono_ns)
    rec = dict(f0=f0, f1=f1, ms=ms, rate=rate, dac_monotonic_ns=mono_ns, dac_dongle_esp_us=esp_us,
               ping_half_rtt_us=bound, latency_s=stream.latency, wall=time.time())
    with open(os.path.join(OUT, "chirps.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")
    print(f"chirp played: dac at dongle {esp_us:.0f} us (ping bound {bound:.0f} us), stream latency {stream.latency*1000:.1f} ms", flush=True)
    return rec

# Stepped chirp on the base station's buzzer: (bytes per half period, cycles).
# At the bus's ~10.7 KB/s that is ~1.3, 1.8, 2.7 and 5.4 kHz.
DCHIRP_STEPS = ((4, 80), (3, 100), (2, 150), (1, 300))  # ~230 ms: long enough to hear and record

def prn_levels(n, seed=0xACE1):
    """16-bit Galois LFSR (taps 0xB400, maximal length): n pseudo-random +-1
    levels. Deterministic, so the analysis rebuilds the exact sequence played."""
    s, out = seed & 0xFFFF or 1, []
    for _ in range(n):
        lsb = s & 1
        s >>= 1
        if lsb:
            s ^= 0xB400
        out.append(1 if lsb else -1)
    return out

PRN_BYTES = 2500   # ~235 ms at ~93.8 us per byte

def dchirp(addr=0x3B, line=7, prn_seed=None):
    """Play the stepped chirp on the base station's buzzer (TCA9538 at I2C bus 0
    `addr`, `line`) in ONE I2C write, so the controller clocks every level
    change at the bus byte rate. Returns the write's monotonic bracket, the byte
    rate measured by a silent calibration write just before, and the first
    sound byte's start on the monotonic and dongle clocks."""
    import tca9538
    e = tca9538.Expander(0, addr)
    try:
        cfg, out = e.read(3), e.read(1)
        bit = 1 << line
        hi, lo = (out | bit) & 0xFF, out & ~bit & 0xFF
        e.write(1, lo); e.write(3, cfg & ~bit)
        cal = bytes([1]) + bytes([lo]) * 3000                 # silent: level stays low
        a = time.monotonic_ns(); os.write(e.fd, cal); b = time.monotonic_ns()
        byte_ns = (b - a) / (len(cal) + 1)                     # + address byte
        if prn_seed is not None:
            body = bytes(hi if v > 0 else lo for v in prn_levels(PRN_BYTES, prn_seed))
        else:
            body = b"".join((bytes([hi]) * h + bytes([lo]) * h) * c for h, c in DCHIRP_STEPS)
        msg = bytes([1]) + bytes([lo]) * 20 + body + bytes([lo]) * 20
        a = time.monotonic_ns(); os.write(e.fd, msg); b = time.monotonic_ns()
        e.write(1, out); e.write(3, cfg)
    finally:
        e.close()
    # The pin follows each data byte; the first sound byte is byte index
    # 1 (address) + 1 (register) + 20 (lead) from the start of the transfer.
    # The transfer ends at ~b, so count back from the end (wake-up slop is
    # smaller than start latency); also record the front estimate.
    n_total = len(msg) + 1
    first_sound_from_end = b - (n_total - 22) * byte_ns
    first_sound_from_start = a + 22 * byte_ns
    esp_us, bound = ping_map(first_sound_from_end)
    rec = dict(addr=addr, line=line, steps=DCHIRP_STEPS, prn_seed=prn_seed, prn_bytes=PRN_BYTES if prn_seed is not None else 0, byte_ns=byte_ns, write_ns=b - a,
               expected_write_ns=n_total * byte_ns, mono_first_sound_ns=first_sound_from_end,
               mono_first_sound_front_ns=first_sound_from_start, dongle_esp_us=esp_us,
               ping_half_rtt_us=bound, wall=time.time())
    with open(os.path.join(OUT, "dchirps.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")
    return rec

def serve_files():
    """Browse and download fragments and events: http://<device>:<HTTP_PORT>/"""
    import functools, http.server
    port = int(os.environ.get("HTTP_PORT", "0") or 0)
    if not port:
        return
    from urllib.parse import urlparse, parse_qs

    import re
    # Every request needs the token: fragments are room audio, and the control
    # endpoints drive the radio. Token = first 16 hex of HMAC-SHA256(key,
    # "anchor-http"); print it with: python -c "import hmac,hashlib;print(hmac.new(
    # bytes.fromhex(open('secret.key').read().strip()),b'anchor-http',hashlib.sha256).hexdigest()[:16])"
    token = hmac.new(KEY, b"anchor-http", hashlib.sha256).hexdigest()[:16]
    allowed = re.compile(r"(beacon (on|off)|rate \d{1,3}|ap (on|off)|gpio \d{1,2}|drive \d{1,2} \d{1,3}|ch \d{1,2}|id)")

    class Handler(http.server.SimpleHTTPRequestHandler):
        """With header "Authorization: Bearer <token>":
        GET  /...                         fragments, events, captures
        POST /cmd?line=<dongle command>   allowlisted, e.g. line=gpio%206
        POST /capture?seconds=<n>         raw dongle lines -> captures/<t>.csv"""
        def _authorized(self):
            got = self.headers.get("Authorization", "")
            if hmac.compare_digest(got.encode(), f"Bearer {token}".encode()):
                return True
            self.send_error(401, "token required")
            return False

        def do_GET(self):
            if not self._authorized():
                return
            if self.path == "/gpiochips":
                import gpioline
                return self._text(json.dumps(gpioline.chips(), indent=1) + "\n")
            return super().do_GET()

        def do_HEAD(self):
            if not self._authorized():
                return
            return super().do_HEAD()

        def do_POST(self):
            if not self._authorized():
                return
            u = urlparse(self.path); q = parse_qs(u.query)
            if u.path == "/cmd" and "line" in q:
                line = q["line"][0].strip()
                if not allowed.fullmatch(line):
                    return self.send_error(400, "command not allowed")
                send_cmd(line)
                return self._text(f"sent: {line}\n")
            if u.path == "/buzz":
                # Toggling an arbitrary GPIO line can disturb the board (one
                # expander line on the Dragonwing is not free); off by default.
                if os.environ.get("ALLOW_GPIO_BUZZ") != "1":
                    return self.send_error(403, "set ALLOW_GPIO_BUZZ=1 to enable /buzz")
                import gpioline
                chip = q.get("chip", ["/dev/gpiochip4"])[0]
                if not chip.startswith("/dev/gpiochip") or not chip[13:].isdigit():
                    return self.send_error(400, "bad chip")
                try:
                    line = int(q["line"][0]); hz = min(max(float(q.get("hz", ["2000"])[0]), 50.0), 8000.0)
                    ms = min(max(float(q.get("ms", ["300"])[0]), 1.0), 2000.0)
                    ln = gpioline.Line(chip, line)
                    try:
                        edges = gpioline.square(ln, hz, ms)
                    finally:
                        ln.close()
                except (KeyError, ValueError, OSError) as e:
                    return self.send_error(400, f"buzz failed: {e}")
                gaps = np.diff(np.array(edges, float)) / 1e3 if len(edges) > 1 else np.array([0.0])
                want = 1e6 / hz / 2
                rec = dict(chip=chip, line=line, hz=hz, ms=ms, edges=len(edges), first_edge_monotonic_ns=edges[0] if edges else None,
                           half_period_us=want, gap_err_us_p50=float(np.median(np.abs(gaps - want))),
                           gap_err_us_max=float(np.abs(gaps - want).max()))
                return self._text(json.dumps(rec) + "\n")
            if u.path == "/dchirp":
                try:
                    seed = q.get("prn", [None])[0]
                    rec = dchirp(prn_seed=int(seed, 0) if seed else None)
                except (OSError, ValueError) as ex:
                    return self.send_error(400, f"dchirp failed: {ex}")
                return self._text(json.dumps(rec) + "\n")
            if u.path == "/chirp":
                try:
                    f0 = float(q.get("f0", ["2000"])[0]); f1 = float(q.get("f1", ["6000"])[0])
                    ms = min(max(float(q.get("ms", ["10"])[0]), 1.0), 100.0)
                    gain = min(max(float(q.get("gain", ["0.5"])[0]), 0.0), 1.0)
                except ValueError:
                    return self.send_error(400, "bad chirp parameters")
                rec = play_chirp(f0, f1, ms, gain)
                return self._text(json.dumps(rec) + "\n")
            if u.path == "/capture":
                try:
                    secs = min(max(float(q.get("seconds", ["60"])[0]), 1.0), 3600.0)
                except ValueError:
                    return self.send_error(400, "bad seconds")
                d = os.path.join(OUT, "captures"); os.makedirs(d, exist_ok=True)
                path = os.path.join(d, f"{int(time.time())}.csv")
                capture.update(f=open(path, "w"), until=time.monotonic() + secs, path=path)
                return self._text(f"capturing {secs:.0f} s to /captures/{os.path.basename(path)}\n")
            return self.send_error(404)

        def _text(self, body):
            b = body.encode()
            self.send_response(200); self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    handler = functools.partial(Handler, directory=OUT)
    import socket
    class DualStack(http.server.ThreadingHTTPServer):
        address_family = socket.AF_INET6
        def server_bind(self):
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            super().server_bind()
    srv = DualStack(("::", port), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"serving {OUT} on :{port}", flush=True)

def main():
    serve_files()
    while True:
        try:
            if PORT.startswith("usb"):
                # "usb" or "usb:<serial>": libusb, for hosts without cdc_acm
                from usbserial import UsbSerial
                s = UsbSerial(serial=PORT.split(":", 1)[1] if ":" in PORT else None)
            else:
                s = serial.Serial(PORT, 115200, timeout=0.2)
            dongle[0] = s
            break
        except Exception as e:
            print(f"waiting for dongle on {PORT}: {e}", flush=True)
            time.sleep(3)
    for c in ("beacon on", "rate 10", "ap off"):
        send_cmd(c); time.sleep(0.05)
    threading.Thread(target=reader, args=(s,), daemon=True).start()
    print(f"anchor on {PORT}, writing to {OUT}", flush=True)
    i = 0
    while True:
        i += 1
        tok = str(i)
        pending_ping[tok] = time.monotonic_ns()
        send_cmd(f"ping {tok}")
        if i % 75 == 0:
            send_cmd("id")
        time.sleep(0.2)

if __name__ == "__main__":
    main()
