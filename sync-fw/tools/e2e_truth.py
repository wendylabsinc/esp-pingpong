"""End-to-end alignment: a sound the base station plays vs where the node's
recording, placed by the Anchor pipeline, says it was.

The base station plays a stepped chirp on its buzzer in one I2C write and logs
when its first sound byte left, on the dongle's clock (Anchor time), via the
USB ping map (anchor.py dchirp -> dchirps.jsonl). The node records it; anchor.py
writes each fragment with first_sample_anchor_us. For each chirp inside a
fragment, the fragment is correlated with the byte-timed waveform that was
played. error = located start - (logged start + travel + mic offset).
usage: e2e_truth.py dchirps.jsonl fragments_dir distance_cm [mic_offset_us]"""
import glob, json, os, sys, wave
import numpy as np

chirps = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
fdir = sys.argv[2]
travel = float(sys.argv[3]) / 100 / 343 * 1e6
mic = float(sys.argv[4]) if len(sys.argv) > 4 else 354.0

def prn_levels(n, seed):
    s, out = seed & 0xFFFF or 1, []
    for _ in range(n):
        lsb = s & 1
        s >>= 1
        if lsb:
            s ^= 0xB400
        out.append(1 if lsb else -1)
    return out

def template(c, rate, scale=1.0):
    """Pin level per data byte, sampled at `rate`, with the byte period scaled:
    the calibration write's Linux start/end delays leave ~0.1-0.3% error, which
    over a 230 ms chirp drifts by more than a 5 kHz cycle."""
    bt = c["byte_ns"] / 1e3 * scale
    if c.get("prn_seed") is not None:
        levels = prn_levels(c["prn_bytes"], c["prn_seed"])
    else:
        levels = []
        for h, cyc in c["steps"]:
            levels += ([1] * h + [-1] * h) * cyc
    edges = np.arange(len(levels) + 1) * bt
    n = int(edges[-1] * rate / 1e6) + 1
    idx = np.minimum(np.searchsorted(edges, np.arange(n) * 1e6 / rate, side="right") - 1, len(levels) - 1)
    return np.array(levels, float)[idx]

def xcorr(seg, t):
    n = 1 << int(np.ceil(np.log2(len(seg) + len(t))))
    return np.fft.irfft(np.fft.rfft(seg, n) * np.conj(np.fft.rfft(t, n)), n)[:len(seg) - len(t) + 1]

frags = []
for js in glob.glob(os.path.join(fdir, "*.json")):
    m = json.load(open(js))
    if "first_sample_anchor_us" not in m:
        continue
    with wave.open(js[:-5] + ".wav") as w:
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
        # high-pass the WHOLE fragment once (filtering a slice leaves edge spikes
        # that the correlation latches onto)
        x = x - np.convolve(x, np.ones(32) / 32, mode="same")
        x[:32] = 0; x[-32:] = 0
        frags.append((m, x, w.getframerate()))

rows = []
for c in chirps:
    if c.get("dongle_esp_us") is None:
        continue
    t_end = c["dongle_esp_us"]
    t_front = t_end - (c["mono_first_sound_ns"] - c["mono_first_sound_front_ns"]) / 1e3
    expect = t_end + travel + mic
    for m, x, r in frags:
        t0 = m["first_sample_anchor_us"]
        k0 = (expect - t0) * r / 1e6
        if not (0 < k0 < len(x)):
            continue
        W = float(os.environ.get("SEARCH_S", "0.02"))
        best = None
        for scale in np.arange(0.995, 1.0051, 0.0002):
            tpl = template(c, r, scale)
            lo = max(0, int(k0 - W * r))
            seg = x[lo: lo + int(2 * W * r) + len(tpl)]
            if len(seg) <= len(tpl):
                continue
            a = np.abs(xcorr(seg, tpl))
            j = int(np.argmax(a))
            if best is None or a[j] > best[0]:
                best = (a[j], a[j] / (np.median(a) + 1e-9), lo + j, scale)
        if best is None:
            break
        _, q, kk, scale = best
        found = t0 + kk * 1e6 / r
        rows.append(dict(err_end=found - (t_end + travel + mic), err_front=found - (t_front + travel + mic),
                         q=q, scale=scale, ping=c["ping_half_rtt_us"], slop=(c["write_ns"] - c["expected_write_ns"]) / 1e3,
                         sigma=m.get("anchor_sigma_us"), frag=m["frag"]))
        break

print(f"chirps: {len(chirps)}, found in a fragment: {len(rows)}; travel {travel:.0f} us, mic offset {mic:.0f} us")
good = [r for r in rows if abs(r["err_end"]) < 5000]
print(f"within +-5 ms (the rest are mismatches): {len(good)} of {len(rows)}")
for key, name in (("err_end", "stamp from the write's end"), ("err_front", "stamp from the write's start")):
    e = np.array([r[key] for r in good])
    if len(e):
        print(f"{name}: mean {e.mean():+.0f} us, median {np.median(e):+.0f}, std {e.std():.0f}, "
              f"|err| p95 {np.percentile(np.abs(e), 95):.0f}, worst {np.abs(e).max():.0f} us")
for r in rows:
    print(f"  frag {r['frag']:3d}  end-ref {r['err_end']:+8.0f}  front-ref {r['err_front']:+8.0f}  q {r['q']:5.1f}  "
          f"usb ±{r['ping']:.0f}  write slop {r['slop']:.0f}  anchor sigma {r['sigma']:.0f}")
