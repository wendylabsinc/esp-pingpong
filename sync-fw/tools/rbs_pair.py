"""Two-device Reference Broadcast Synchronization from soundsync-fw captures.

Both devices stamp the same access point beacons in hardware. A beacon is
identified by (BSSID, AP TSF). For every pair of stamps (rx_a, rx_b) of one
beacon, rx_b - rx_a is the clock offset between the devices at that instant.

Honest error: fit the a->b mapping on beacons of one access point only (sliding
window), then predict b's stamp for beacons of a *different* access point and
compare with what b actually stamped. A different AP is an independent
reference, so this residual is the real device-to-device sync error.
usage: rbs_pair.py a.csv b.csv [window_s]"""
import sys, collections
import numpy as np

def load(path):
    beacons, espnow_rx, tx = {}, [], []
    for line in open(path):
        p = line.strip().split(",")
        if len(p) < 3:
            continue
        try:
            if p[1] == "A" and len(p) >= 8:
                beacons[(p[2], int(p[3]))] = int(p[4])
            elif p[1] == "E" and len(p) >= 7:
                espnow_rx.append((int(p[3]), int(p[4]), int(p[5])))
            elif p[1] == "T" and len(p) >= 4:
                tx.append((int(p[2]), int(p[3])))
        except ValueError:
            pass
    return beacons, espnow_rx, tx

def ap_of(bssid):
    return bssid[4:]  # virtual BSSIDs of one radio differ in the first bytes

A, Ae, At = load(sys.argv[1])
B, Be, Bt = load(sys.argv[2])
W = float(sys.argv[3]) * 1e6 if len(sys.argv) > 3 else 10e6

keys = sorted(set(A) & set(B), key=lambda k: A[k])
x = np.array([A[k] for k in keys], dtype=np.int64)
y = np.array([B[k] for k in keys], dtype=np.int64)
aps = np.array([ap_of(k[0]) for k in keys])
print(f"beacons stamped by both: {len(keys)} (a {len(A)}, b {len(B)})")
cnt = collections.Counter(aps)
print("per access point:", dict(cnt.most_common(6)))
ref, *tests = [a for a, _ in cnt.most_common(4)]

d = (y - x).astype(float)
xf = (x - x[0]).astype(float)
b1, a1 = np.polyfit(xf, d, 1)
print(f"\nclock b vs a: offset {d[0]/1e3:.3f} ms, drift {b1*1e6:+.2f} ppm over {xf[-1]/1e6:.0f} s")

def stats(e):
    a = np.abs(e)
    return f"n={len(e)}  std {e.std():.2f} us  mean {e.mean():+.2f}  p95 {np.percentile(a,95):.2f}  p99 {np.percentile(a,99):.2f}  max {a.max():.2f}"

r = ref == aps
xr, dr = xf[r], d[r]
for t in [ref] + tests:
    m = aps == t
    errs = []
    for xi, di in zip(xf[m], d[m]):
        w = (xr > xi - W) & (xr < xi + W)
        if t == ref:
            w &= xr != xi  # leave this beacon out of its own fit
        if w.sum() < 20:
            continue
        bb, aa = np.polyfit(xr[w] - xi, dr[w], 1)
        errs.append(di - aa)
    errs = np.array(errs)
    label = "reference AP, leave-one-out" if t == ref else "independent AP"
    print(f"AP {t} ({label}): {stats(errs)}")

# ESP-NOW beacons: b transmitted (T), a received (E). One-way send latency,
# with b's tx esp_timer mapped onto b's rx counter via its beacon callbacks.
if Bt and Ae:
    print(f"\nESP-NOW beacons: b sent {len(Bt)}, a received {len(Ae)} ({100*len(Ae)/max(1,len(Bt)):.1f}%)")
