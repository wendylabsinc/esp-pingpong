"""Check alignment on multi-node events written by anchor.py.

For every event with two or more nodes: the aligned WAV puts each channel on
the shared timeline, so a sound heard by nodes side by side should line up at
lag ~0, and a node d metres further from the source should lag by d / 343 m/s.
Reports, per node pair, the cross-correlation lag of the aligned audio with
sub-sample (parabolic) interpolation, around the loudest moment.
usage: event_report.py <out-dir>/events [max_lag_ms]"""
import glob, json, os, sys, wave
import numpy as np

d = sys.argv[1]
max_ms = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
for js in sorted(glob.glob(os.path.join(d, "*.json"))):
    ev = json.load(open(js))
    wav = js[:-5] + ".wav"
    if len(ev.get("channels", [])) < 2 or not os.path.exists(wav):
        continue
    with wave.open(wav) as w:
        rate, nch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").reshape(-1, nch).astype(float)
    # window: 100 ms around the loudest moment across channels
    env = np.abs(x).sum(axis=1)
    c = int(np.argmax(env)); h = int(0.05 * rate)
    seg = x[max(0, c - h): c + h]
    seg = seg - seg.mean(axis=0)
    L = int(max_ms * rate / 1000)
    print(f"{os.path.basename(js)}  timeline={ev['timeline']}  peak at {c / rate:.3f} s")
    for i in range(nch):
        for j in range(i + 1, nch):
            a, b = seg[:, i], seg[:, j]
            full = np.correlate(b, a, mode="full")          # lag k: b[n+k] ~ a[n]
            mid = len(a) - 1
            win = full[mid - L: mid + L + 1]
            k = int(np.argmax(win))
            frac = 0.0
            if 0 < k < len(win) - 1:
                y0, y1, y2 = win[k - 1], win[k], win[k + 1]
                den = y0 - 2 * y1 + y2
                frac = 0.5 * (y0 - y2) / den if den else 0.0
            lag = (k - L + frac) / rate * 1e6
            r = win[k] / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
            ti = ev["nodes"][i]["trigger_us"]; tj = ev["nodes"][j]["trigger_us"]
            print(f"  {ev['channels'][i][-4:]} -> {ev['channels'][j][-4:]}: audio lag {lag:+9.1f} us "
                  f"(= {lag * 343e-6 * 100:+.1f} cm), corr {r:.2f}; trigger diff {tj - ti:+.0f} us "
                  f"(block-quantised, 20 ms)")
