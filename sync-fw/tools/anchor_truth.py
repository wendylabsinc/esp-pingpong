"""Ground truth for the Anchor's node-to-Anchor placement.

The node drives a wire into the Anchor's dongle. The dongle's esp_timer IS the
Anchor timeline, so its interrupt stamp of an edge is the true Anchor time.
The node's own stamp of the same edge is placed on Anchor time exactly as
anchor.py places samples: band() over (node arrival of beacon seq, dongle
esp_timer just before sending seq), beacons within 30 s. (This test showed
the original 20 us error floor was wrong by ~30x; anchor.py now uses 1 ms.)
usage: anchor_truth.py node.csv dongle.csv"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "anchor"))
from clocks import band

def load(path):
    E, T, G = {}, {}, []
    for line in open(path):
        p = line.strip().split(",")
        try:
            if p[1] == "E" and len(p) >= 9 and p[8] == "0":
                E[int(p[3])] = int(p[7])
            elif p[1] == "T" and len(p) >= 4:
                T[int(p[2])] = int(p[3])
            elif p[1] == "G" and len(p) >= 5:
                G.append((int(p[4]), int(p[3])))
        except (ValueError, IndexError):
            pass
    return E, T, G

nE, _, nG = load(sys.argv[1])
_, dT, dG = load(sys.argv[2])
pairs = sorted((nE[s], dT[s]) for s in nE if s in dT)
print(f"beacons sent by dongle {len(dT)}, heard by node {len(nE)}, matched {len(pairs)}; edges node {len(nG)}, dongle {len(dG)}")
pa = np.array(pairs, float)
de = np.array([e for e, _ in dG], float); dl = np.array([l for _, l in dG])
errs, sig = [], []
for e, lvl in nG:
    w = np.abs(pa[:, 0] - e) < 30e6
    if w.sum() < 20:
        continue
    line = band([tuple(r) for r in pa[w]], floor_us=1000.0)
    pred, s = line(e)
    j = np.argmin(np.abs(de - pred))
    if abs(de[j] - pred) < 5000 and dl[j] == lvl:
        errs.append(de[j] - pred); sig.append(s)
e = np.array(errs); s = np.array(sig); a = np.abs(e)
print(f"matched edges {len(e)}; claimed sigma {s.min():.1f}-{s.max():.1f} us")
print(f"actual - predicted: mean {e.mean():+.2f} us, median {np.median(e):+.2f}, std {e.std():.2f}")
print(f"|error| p50 {np.percentile(a,50):.2f}  p95 {np.percentile(a,95):.2f}  p99 {np.percentile(a,99):.2f}  max {a.max():.2f} us")
print(f"within claimed 1-sigma: {(a <= s).mean()*100:.0f}%   within 2-sigma: {(a <= 2*s).mean()*100:.0f}%")
