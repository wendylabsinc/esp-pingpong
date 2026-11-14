"""Ground truth for sync accuracy: one wire, one electrical edge, two stamps.

Device a drives a pin ("drive"), device b listens on a wired pin ("gpio"); both
stamp every edge with esp_timer in their interrupt handler. For each edge:
  a esp_timer -> a radio counter   (lower envelope of a's beacon callbacks)
  a radio     -> b radio            (RBS: shared AP beacons, windowed fit)
  b radio     -> b esp_timer       (lower envelope of b's beacon callbacks)
The prediction is compared with b's own stamp of the same edge. The error
includes everything: radio sync, both counter mappings, and the difference in
interrupt latency between the two chips.
usage: gpio_truth.py a.csv b.csv [window_s]"""
import sys
import numpy as np

def load(path):
    beacons, edges = {}, []
    for line in open(path):
        p = line.strip().split(",")
        try:
            if len(p) >= 8 and p[1] == "A":
                arr = int(p[8]) if len(p) >= 10 and p[9] == "0" else None
                beacons[(p[2], int(p[3]))] = (int(p[4]), int(p[5]), arr)
            elif len(p) >= 5 and p[1] == "G":
                edges.append((int(p[4]), int(p[3])))
        except ValueError:
            pass
    return beacons, edges

def counter_map(beacons):
    """esp_timer -> radio counter. esp - rx = offset + callback delay (>= 0),
    so the per-bucket minimum is the offset; fit a line through those minima."""
    v = np.array(sorted(b[:2] for b in beacons.values()), dtype=np.int64)
    rx, esp = v[:, 0], v[:, 1]
    d = esp - rx
    b = (esp - esp[0]) // 2_000_000
    pts = [(esp[b == k].mean(), d[b == k].min()) for k in np.unique(b)]
    pts = np.array(pts, dtype=float)
    k, c = np.polyfit(pts[:, 0] - esp[0], pts[:, 1], 1)
    e0 = float(esp[0])
    return lambda e: e - (c + k * (e - e0)), lambda r: r + (c + k * (r - e0))  # esp->rx, rx->esp (approx)

A, Ea = load(sys.argv[1])
B, Eb = load(sys.argv[2])
W = float(sys.argv[3]) * 1e6 if len(sys.argv) > 3 else 10e6
a_esp2rx, _ = counter_map(A)
_, b_rx2esp = counter_map(B)

keys = sorted(set(A) & set(B), key=lambda k: A[k][0])
xa = np.array([A[k][0] for k in keys], dtype=float)
xb = np.array([B[k][0] for k in keys], dtype=float)
print(f"shared beacons {len(keys)}; edges a {len(Ea)}, b {len(Eb)}")

eb = np.array([e for e, _ in Eb], dtype=float)
lb = np.array([l for _, l in Eb])
errs = []
for ea, la in Ea:
    ra = a_esp2rx(ea)
    w = (xa > ra - W) & (xa < ra + W)
    if w.sum() < 20:
        continue
    k, c = np.polyfit(xa[w] - ra, xb[w] - xa[w], 1)
    rb = ra + c
    pred = b_rx2esp(rb)
    j = np.argmin(np.abs(eb - pred))
    if abs(eb[j] - pred) < 2000 and lb[j] == la:
        errs.append(eb[j] - pred)
e = np.array(errs)
if len(e) == 0:
    sys.exit("no matched edges")
c = e - np.median(e)
a = np.abs(e)
print(f"matched edges: {len(e)}")
print(f"absolute error (b actual - predicted): mean {e.mean():+.2f} us, median {np.median(e):+.2f}, std {e.std():.2f}")
print(f"  |error| p50 {np.percentile(a,50):.2f}  p95 {np.percentile(a,95):.2f}  p99 {np.percentile(a,99):.2f}  max {a.max():.2f} us")
print(f"after removing the constant bias: std {c.std():.2f}, p95 {np.percentile(np.abs(c),95):.2f}, max {np.abs(c).max():.2f} us")

# Method 2: receive stamps converted to esp_timer on each device (arrival).
ka = [k for k in keys if A[k][2] is not None and B[k][2] is not None]
if ka:
    ta = np.array([A[k][2] for k in ka], dtype=float)
    tb = np.array([B[k][2] for k in ka], dtype=float)
    errs2 = []
    for ea, la in Ea:
        w = (ta > ea - W) & (ta < ea + W)
        if w.sum() < 20:
            continue
        k, c = np.polyfit(ta[w] - ea, tb[w] - ta[w], 1)
        pred = ea + c
        j = np.argmin(np.abs(eb - pred))
        if abs(eb[j] - pred) < 2000 and lb[j] == la:
            errs2.append(eb[j] - pred)
    e2 = np.array(errs2); c2 = e2 - np.median(e2); a2 = np.abs(e2)
    print(f"\n[on-device arrival] matched edges: {len(e2)} (beacons with hard stamps on both: {len(ka)})")
    print(f"absolute error: mean {e2.mean():+.2f} us, median {np.median(e2):+.2f}, std {e2.std():.2f}; |error| p95 {np.percentile(a2,95):.2f}, max {a2.max():.2f}")
    print(f"after removing the constant bias: std {c2.std():.2f}, p95 {np.percentile(np.abs(c2),95):.2f}, max {np.abs(c2).max():.2f} us")
