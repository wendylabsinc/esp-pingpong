"""Ground truth for audio timestamps with a buzzer the node drives itself.

The node plays a tone burst ("beeppin <pin>", "beep [hz] [cycles]" or
"beepevery <ms>") and stamps the first edge on esp_timer ("K" lines). The burst
is recorded in a fragment. For each burst inside a fragment, the recording is
cross-correlated with the burst that was played; the best lag gives where the
burst starts on the fragment's own timeline (first_sample_node_esp_us +
index / rate). error = that - (stamp + distance / 343 m/s).
Mean error = fixed microphone pipeline delay (+ buzzer response); scatter =
how well a recorded sound's time is known on the node's clock.
usage: beep_truth.py node_capture.csv fragments_dir [distance_cm]"""
import glob, json, os, sys, wave
import numpy as np

cap, fdir = sys.argv[1], sys.argv[2]
dist_cm = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
travel = dist_cm / 100 / 343 * 1e6
beeps = []   # (stamp, kind, a, b): tone (hz, cycles) or chirp (f0, f1, dur_us)
for line in open(cap):
    p = line.strip().split(",")
    try:
        if len(p) >= 6 and p[1] == "K":
            beeps.append((int(p[5]), "tone", int(p[3]), int(p[4]), 0))
        elif len(p) >= 7 and p[1] == "Q":
            beeps.append((int(p[6]), "chirp", int(p[3]), int(p[4]), int(p[5])))
    except ValueError:
        pass
beeps = sorted(set(beeps))  # a stamp may be read back twice
print(f"bursts played: {len(beeps)}; buzzer distance {dist_cm} cm = {travel:.0f} us")
rows = []
for js in sorted(glob.glob(os.path.join(fdir, "*.json"))):
    m = json.load(open(js))
    if "first_sample_node_esp_us" not in m:
        continue
    with wave.open(js[:-5] + ".wav") as w:
        r = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
    t0, us = m["first_sample_node_esp_us"], 1e6 / r
    for t, kind, a, b, dur in beeps:
        k0 = (t + travel - t0) / us                       # expected start, samples
        if not (0.05 * r < k0 < len(x) - 0.05 * r):
            continue
        if kind == "tone":
            n = int(round(b * r / a))
            tt = np.arange(n) / r
            phase = 2 * np.pi * a * tt
        else:  # linear chirp a -> b Hz over dur us: unique correlation peak
            n = int(round(dur * 1e-6 * r))
            tt = np.arange(n) / r
            phase = 2 * np.pi * (a * tt + (b - a) / (dur * 1e-6) * tt ** 2 / 2)
        tpl = np.exp(-1j * phase)
        lo, hi = int(k0 - 0.005 * r), int(k0 + 0.020 * r)
        seg = x[lo: hi + n] - x[lo: hi + n].mean()
        c = np.array([np.dot(seg[i:i + n], tpl) for i in range(len(seg) - n + 1)])
        env = np.abs(c)
        j = int(np.argmax(env))
        frac = 0.0
        if 0 < j < len(env) - 1:
            y0, y1, y2 = env[j - 1], env[j], env[j + 1]
            d = y0 - 2 * y1 + y2
            frac = 0.5 * (y0 - y2) / d if d else 0.0
        noise = np.median(env) + 1e-9
        start = t0 + (lo + j + frac) * us
        rows.append((os.path.basename(js), t, start - (t + travel), env[j] / noise))
if not rows:
    sys.exit("no bursts inside fragments")
e = np.array([r[2] for r in rows]); q = np.array([r[3] for r in rows])
good = q > 5
print(f"bursts inside fragments: {len(rows)}, clear (peak > 5x median): {good.sum()}")
if good.sum():
    g = e[good]; c = g - np.median(g)
    print(f"audio time - true time: mean {g.mean():+.1f} us, median {np.median(g):+.1f} us (fixed delay)")
    print(f"scatter: std {c.std():.1f} us, p95 {np.percentile(np.abs(c), 95):.1f} us, max {np.abs(c).max():.1f} us")
for f, t, err, qq in rows:
    print(f"  {f}  {t / 1e6:10.3f} s  error {err:+9.1f} us  quality {qq:5.1f}")
