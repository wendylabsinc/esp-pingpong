"""Ground truth for audio timestamps: contact taps next to the microphone.

Two wires (one on a GPIO with "gpio <pin>", one on GND) are tapped together
about D cm from the microphone. The contact is an electrical edge, stamped by
the node's interrupt on its esp_timer, and a click, recorded in a fragment.
For every tap inside a fragment:
  expected = edge_esp + D / 343 m/s
  measured = fragment first_sample_node_esp_us + onset_index / rate
  error    = measured - expected
The mean is the microphone pipeline's fixed delay (PDM decimation + DMA); the
scatter is the accuracy of "when did the recorded sound happen" on the node's
clock. Onset = first sample whose high-passed amplitude exceeds K x the noise
RMS of the 50 ms before the edge.
usage: audio_truth.py node_capture.csv fragments_dir [distance_cm] [K]"""
import glob, json, os, sys, wave
import numpy as np

cap, fdir = sys.argv[1], sys.argv[2]
dist_cm = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
K = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0
travel_us = dist_cm / 100 / 343 * 1e6

edges = []
for line in open(cap):
    p = line.strip().split(",")
    if len(p) >= 5 and p[1] == "G":  # any edge; the first of each touch is the contact
        try:
            edges.append(int(p[4]))
        except ValueError:
            pass
edges = np.array(sorted(edges), float)
# debounce: keep the first edge of each contact (bounces within 50 ms)
if len(edges):
    keep = np.concatenate([[True], np.diff(edges) > 50_000])
    edges = edges[keep]
print(f"taps (first contact edges): {len(edges)}; mic distance {dist_cm} cm = {travel_us:.0f} us of travel")

rows = []
for js in sorted(glob.glob(os.path.join(fdir, "*.json"))):
    m = json.load(open(js))
    if "first_sample_node_esp_us" not in m:
        continue
    with wave.open(js[:-5] + ".wav") as w:
        rate = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
    hp = np.concatenate([[0.0], np.diff(x)])            # crude high-pass: first difference
    t0 = m["first_sample_node_esp_us"]
    us_per = 1e6 / rate
    for e in edges[(edges > t0 + 60_000) & (edges < t0 + len(x) * us_per - 10_000)]:
        k_edge = int((e - t0) / us_per)
        noise = hp[k_edge - int(0.05 * rate): k_edge - int(0.002 * rate)]
        thr = K * np.sqrt(np.mean(noise ** 2)) + 1e-9
        win = np.abs(hp[k_edge - int(0.002 * rate): k_edge + int(0.01 * rate)])
        idx = np.nonzero(win > thr)[0]
        if len(idx) == 0:
            rows.append((os.path.basename(js), e, None)); continue
        k = k_edge - int(0.002 * rate) + idx[0]
        # sub-sample: linear interpolation of the threshold crossing
        a0, a1 = abs(hp[k - 1]), abs(hp[k])
        frac = (thr - a0) / (a1 - a0) if a1 != a0 else 0.0
        onset = t0 + (k - 1 + min(max(frac, 0), 1)) * us_per
        rows.append((os.path.basename(js), e, onset - (e + travel_us)))

errs = np.array([r[2] for r in rows if r[2] is not None])
missed = sum(1 for r in rows if r[2] is None)
print(f"taps inside fragments: {len(rows)}, onset found: {len(errs)}, no onset above threshold: {missed}")
if len(errs):
    c = errs - np.median(errs)
    print(f"audio time - true time: mean {errs.mean():+.1f} us, median {np.median(errs):+.1f} us  (= fixed microphone pipeline delay)")
    print(f"scatter around it: std {c.std():.1f} us, p95 {np.percentile(np.abs(c), 95):.1f} us, max {np.abs(c).max():.1f} us")
    for r in rows:
        print(f"  {r[0]}  error {('%+.1f us' % r[2]) if r[2] is not None else 'no onset'}")
