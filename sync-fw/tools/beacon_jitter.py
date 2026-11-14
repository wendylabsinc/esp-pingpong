"""Receive-stamp jitter against an access point's own clock.

An AP writes its TSF into each beacon in hardware at transmit time. Fitting our
hardware receive stamp against that TSF leaves, as residual, the jitter of our
receive stamping (plus the AP's, which is hardware too). Also fits the
callback's esp_timer against the receive stamp: the drift between the two
counters and the callback delay distribution.
usage: beacon_jitter.py capture.csv"""
import sys, collections
import numpy as np

rows = collections.defaultdict(list)
for line in open(sys.argv[1]):
    p = line.strip().split(",")
    if len(p) >= 8 and p[1] == "A":
        try:
            rows[p[2]].append((int(p[3]), int(p[4]), int(p[5]), int(p[6])))
        except ValueError:
            pass

def unwrap(x):
    x = np.array(x, dtype=np.int64)
    d = np.diff(x)
    wraps = np.concatenate([[0], np.cumsum(d < -(1 << 31))])
    return x + wraps * (1 << 32)

def stats(r):
    a = np.abs(r)
    return "std %.2f us | p95 |r| %.2f | p99 %.2f | max %.2f" % (r.std(), np.percentile(a, 95), np.percentile(a, 99), a.max())

print("BSSIDs:", {k: len(v) for k, v in sorted(rows.items(), key=lambda kv: -len(kv[1]))})
for bssid, v in sorted(rows.items(), key=lambda kv: -len(kv[1]))[:3]:
    if len(v) < 200:
        continue
    v.sort(key=lambda t: t[0])
    tsf = np.array([t[0] for t in v], dtype=np.int64)
    rx = unwrap([t[1] for t in v])
    esp = np.array([t[2] for t in v], dtype=np.int64)
    x = (tsf - tsf[0]).astype(float)
    y = (rx - rx[0]).astype(float)
    b, a = np.polyfit(x, y, 1)
    r = y - (a + b * x)
    # Drop gross outliers (>200 us) before reporting, and say how many.
    out = np.abs(r) > 200
    b2, a2 = np.polyfit(x[~out], y[~out], 1)
    r2 = y[~out] - (a2 + b2 * x[~out])
    print(f"\n{bssid}: {len(v)} beacons over {x[-1]/1e6:.0f} s")
    print(f"  our rx clock vs AP TSF: {(b2-1)*1e6:+.2f} ppm")
    print(f"  rx-stamp residual (all):          {stats(r)}")
    print(f"  rx-stamp residual (|r|<200, n={len(r2)}, dropped {out.sum()}): {stats(r2)}")
    # Callback esp_timer vs rx stamp: d = esp - rx; min envelope is pure offset.
    d = (esp - rx).astype(float)
    xs = (rx - rx[0]).astype(float)
    bb, aa = np.polyfit(xs, d, 1)
    lat = d - (aa + bb * xs)
    lat -= lat.min()
    print(f"  esp_timer vs rx-stamp counter: {bb*1e6:+.2f} ppm; callback delay p50 {np.percentile(lat,50):.0f} us, p95 {np.percentile(lat,95):.0f} us, max {lat.max():.0f} us")
