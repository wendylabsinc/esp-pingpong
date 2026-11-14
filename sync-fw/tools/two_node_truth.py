"""Node-to-node alignment: two nodes record the same base-station code.

For each code, the code is located in both nodes' fragments on each node's own
clock (first_sample_node_esp_us + index / rate). Node B's clock is mapped onto
node A's through the ESP-NOW beacons both heard (same sequence number, on-device
arrival stamps, robust line: reference broadcast). The difference B - A is the
node-to-node timing of one sound, which arrival-time-difference localisation
depends on. Nodes at different distances from the source add a constant
(distance difference / 343 m/s); the scatter is the measured alignment.
Also reported: the same difference via the Anchor's clock (production path).
usage: two_node_truth.py codes.jsonl dirA dirB"""
import glob, json, os, sys, wave
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "anchor"))
from clocks import Line
from e2e_truth_lib import template, xcorr

codes = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]

def load(d):
    out = []
    for js in glob.glob(os.path.join(d, "*.json")):
        m = json.load(open(js))
        if "first_sample_anchor_us" not in m:
            continue
        with wave.open(js[:-5] + ".wav") as w:
            x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
            r = w.getframerate()
        x = x - np.convolve(x, np.ones(32) / 32, mode="same"); x[:32] = 0; x[-32:] = 0
        out.append((m, x, r))
    return out

def locate(c, frags):
    """-> (fragment meta, node-clock time of the code start, anchor time, quality)"""
    exp = c["dongle_esp_us"]
    for m, x, r in frags:
        t0 = m["first_sample_anchor_us"]; k0 = (exp - t0) * r / 1e6
        if not (0 < k0 < len(x) - 0.25 * r):
            continue
        best = None
        for scale in np.arange(0.995, 1.0051, 0.0002):
            tpl = template(c, r, scale)
            lo = max(0, int(k0 - 0.03 * r))
            seg = x[lo: lo + int(0.06 * r) + len(tpl)]
            if len(seg) <= len(tpl):
                continue
            a = np.abs(xcorr(seg, tpl)); j = int(np.argmax(a))
            if best is None or a[j] > best[0]:
                best = (a[j], a[j] / (np.median(a) + 1e-9), lo + j)
        if best is None:
            return None
        _, q, k = best
        us = 1e6 / r
        return m, m["first_sample_node_esp_us"] + k * us, t0 + k * us, q
    return None

A, B = load(sys.argv[2]), load(sys.argv[3])
rows = []
for c in codes:
    la, lb = locate(c, A), locate(c, B)
    if not la or not lb:
        continue
    (ma, ta, aa, qa), (mb, tb, ab, qb) = la, lb
    arr_a = {b[1]: b[3] for b in ma["beacons"] if not b[4]}
    arr_b = {b[1]: b[3] for b in mb["beacons"] if not b[4]}
    pairs = [(arr_b[s], arr_a[s]) for s in arr_b if s in arr_a]
    if len(pairs) < 10:
        continue
    line = Line(pairs, 1.0)
    tb_in_a, sig = line(tb)
    rows.append(dict(seed=c["prn_seed"], rbs=tb_in_a - ta, sig=sig, anchor=ab - aa, qa=qa, qb=qb, n=len(pairs)))

print(f"codes {len(codes)}, located in both nodes: {len(rows)}")
for r in rows:
    print(f"  seed {r['seed']:#06x}: B - A via shared beacons {r['rbs']:+8.1f} us (fit ±{r['sig']:.1f}, {r['n']} beacons)   via Anchor clock {r['anchor']:+8.0f} us   quality {r['qa']:.0f}/{r['qb']:.0f}")
for key, name in (("rbs", "via shared beacons (node-to-node path)"), ("anchor", "via the Anchor clock (production path)")):
    e = np.array([r[key] for r in rows])
    if len(e):
        c = e - np.median(e)
        print(f"{name}: median {np.median(e):+.1f} us, std {e.std():.1f} us, scatter p95 {np.percentile(np.abs(c), 95):.1f}, worst {np.abs(c).max():.1f} us")
