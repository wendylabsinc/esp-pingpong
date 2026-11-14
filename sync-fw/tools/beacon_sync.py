"""Time sync between two nodes from the beacons both received.

Each node stamps every Anchor beacon (hardware receive time, converted to its
esp_timer). Two nodes that heard the same beacon heard it at the same instant,
so the pairs (stamp on A, stamp on B) lie on a line: B's clock against A's.
This prints how tightly they do, in windows, which is the precision of the
reference-broadcast time sync. It needs no wire and no sound.

Accepts serial captures of either firmware:
  sync-fw:      ...,E,<src>,<seq>,<rx>,<esp>,<rssi>,<arrival_us>,<soft>
  Wendy Lite:   ... wendy_espnow: B,<seq>,<arrival_us>,<rssi>   (ESP_LOG text or JSON chunks)
With --ap the reference is the Wi-Fi access point's own beacons instead (any
two receivers on the channel share those; a beacon is identified by the AP
and the timestamp it carries):
  sync-fw:      ...,A,<bssid>,<ap_tsf>,<rx>,<esp>,<rssi>,<channel>,<arrival_us>,<soft>
  Wendy Lite:   ... wendy_espnow: A,<bssid>,<ap_tsf>,<arrival_us>
usage: beacon_sync.py [--ap] <capture A> <capture B> [window_s=20]"""
import json, re, sys
import numpy as np

AP = "--ap" in sys.argv
sys.argv = [a for a in sys.argv if a != "--ap"]

def load(path):
    text = open(path, errors="replace").read()
    if '"data"' in text[:4000]:                       # wendy device logs: JSON lines carrying chunks
        buf = []
        for line in text.splitlines():
            try:
                buf.append(json.loads(line).get("data", ""))
            except ValueError:
                pass
        text = "".join(buf)
    out = {}
    if AP:
        for m in re.finditer(r"\bA,([0-9a-f]{12}),(\d+),\d+,-?\d+,-?\d+,\d+,(-?\d+),([01])", text):
            if m.group(4) == "0":
                out[(m.group(1), int(m.group(2)))] = int(m.group(3))
        for m in re.finditer(r"\bA,([0-9a-f]{12}),(\d+),(-?\d+)(?![,\d])", text):
            out[(m.group(1), int(m.group(2)))] = int(m.group(3))
        return out
    for m in re.finditer(r"\bE,[0-9a-f]{12},(\d+),\d+,-?\d+,-?\d+,(-?\d+),([01])", text):
        if m.group(3) == "0":
            out[int(m.group(1))] = int(m.group(2))
    for m in re.finditer(r"\bB,(\d+),(-?\d+),-?\d+", text):
        out[int(m.group(1))] = int(m.group(2))
    return out

a, b = load(sys.argv[1]), load(sys.argv[2])
window = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
if AP:   # keep the access point both heard most
    shared = [k for k in a if k in b]
    if shared:
        best = max(set(k[0] for k in shared), key=lambda bs: sum(1 for k in shared if k[0] == bs))
        a = {k: v for k, v in a.items() if k[0] == best}
        b = {k: v for k, v in b.items() if k[0] == best}
        print(f"access point {best}")
seqs = sorted(set(a) & set(b))
print(f"beacons: A {len(a)}, B {len(b)}, heard by both {len(seqs)}")
if len(seqs) < 30:
    sys.exit("too few shared beacons")
ta = np.array([a[s] for s in seqs], float)
tb = np.array([b[s] for s in seqs], float)
order = np.argsort(ta)
ta, tb = ta[order], tb[order]
keep = np.concatenate([[True], np.diff(ta) > 0]) & np.concatenate([[True], np.diff(tb) > 0])   # one boot each
ta, tb = ta[keep], tb[keep]
res, ppm = [], []
t0 = ta[0]
while t0 < ta[-1]:
    m = (ta >= t0) & (ta < t0 + window * 1e6)
    if m.sum() >= 20:
        x, y = ta[m] - ta[m].mean(), tb[m] - tb[m].mean()
        k = (x * y).sum() / (x * x).sum()
        r = y - k * x
        r = r[np.abs(r - np.median(r)) < 50]          # a late stamp (radio busy) is not a sync error
        res.append(r)
        ppm.append((k - 1) * 1e6)
    t0 += window * 1e6
allr = np.concatenate(res)
print(f"{len(res)} windows of {window:g} s, {len(allr)} beacons: residual std {allr.std():.2f} us, "
      f"p95 {np.percentile(np.abs(allr), 95):.2f} us, worst {np.abs(allr).max():.2f} us; "
      f"B runs {np.mean(ppm):+.2f} ppm against A (window spread {np.std(ppm):.2f} ppm)")
