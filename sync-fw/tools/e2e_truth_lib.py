"""Shared helpers for e2e_truth.py and two_node_truth.py."""
import numpy as np

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

