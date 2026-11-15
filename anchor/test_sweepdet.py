"""The boards' own sweep detector (sync-fw/main/sweepdet.c), built for this machine and fed made-up
microphone audio: three boards hear the same sweeps with their own delays, echoes and noise, and the
arrival differences read from their reports must match the ones put in.

    cc -O2 -shared -fPIC -I ../sync-fw/main -o /tmp/libsweepdet.so ../sync-fw/main/sweepdet.c -lm
    python3 test_sweepdet.py /tmp/libsweepdet.so
"""
import ctypes, sys
import numpy as np
import ranging

PRE, LEN = 112, 160
lib = ctypes.CDLL(sys.argv[1])


class Hit(ctypes.Structure):
    _fields_ = [("first", ctypes.c_int64), ("iq", ctypes.c_int16 * (2 * LEN + 2))]


lib.sd_feed.argtypes = [ctypes.POINTER(ctypes.c_int16), ctypes.c_int, ctypes.c_int64, ctypes.POINTER(Hit)]


def detect(x):
    """All the reports for 32 kHz audio x -> [(arrival in seconds, how far it stands out)]."""
    lib.sd_init()
    x = np.clip(np.round(x), -32768, 32767).astype(np.int16)
    out, hit = [], Hit()
    for n0 in range(0, len(x) - 639, 640):
        blk = np.ascontiguousarray(x[n0:n0 + 640])
        if lib.sd_feed(blk.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)), 640, n0, ctypes.byref(hit)):
            at, clear = ranging.heard_arrival(np.array(hit.iq))
            out.append(((hit.first + at) / ranging.HEARD_RATE, clear))
    return out


def chirp(t):
    """The sweep as heard at times t (seconds after it began); zero outside it."""
    f0, f1, dur = ranging.SWEEP
    on = (t >= 0) & (t < dur)
    return np.where(on, np.sin(2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t * t)), 0.0)


def room(starts, delay, level, noise, rng, echoes=()):
    t = np.arange(int(32000 * (max(starts) + 0.5))) / 32000
    x = rng.normal(0, noise, len(t))
    for s in starts:
        x += level * chirp(t - s - delay)
        for later, gain in echoes:
            x += level * gain * chirp(t - s - delay - later)
    return x


def check():
    rng = np.random.default_rng(5)
    starts = [0.3 + 0.4 * i + 0.0137 * i for i in range(12)]          # a clip: a sweep every 0.4 s or so
    delays = [0.00231, 0.00507, 0.00894]                               # 0.8, 1.7 and 3.1 m away
    worst = {}
    for name, level, noise, echoes in (("loud and clean", 3000, 30, ()),
                                       ("faint", 60, 60, ()),
                                       ("echo 9 ms later and stronger", 800, 40, ((0.009, 1.4),)),
                                       ("near reflection", 800, 40, ((0.0031, 0.5),))):
        got = [detect(room(starts, d, level, noise, rng, echoes)) for d in delays]
        assert all(len(g) == len(starts) for g in got), (name, [len(g) for g in got])
        off = np.array([[g[i][0] - starts[i] - d for i in range(len(starts))] for g, d in zip(got, delays)]) * 1e6
        apart = off - off[0]                                           # what locating uses: differences between boards
        worst[name] = (float(np.abs(apart).max()), float(off.mean()), min(c for g in got for _, c in g))
        print(f"{name}: differences between boards off by at most {worst[name][0]:.1f} us; every arrival read "
              f"{worst[name][1]:+.0f} us from when it came; standing out at least {worst[name][2]:.0f} times")
        assert worst[name][0] < 25, name
    quiet = detect(rng.normal(0, 200, 32000 * 20))                     # 20 s of noise, and a speech-like rumble
    t = np.arange(32000 * 5) / 32000
    voice = detect(2000 * np.sin(2 * np.pi * 300 * t) * (1 + np.sin(2 * np.pi * 3 * t)) + rng.normal(0, 50, len(t)))
    print(f"nothing but noise: {len(quiet)} reports; a 300 Hz hum that comes and goes: {len(voice)}")
    assert not quiet and not voice
    print("ok")


if __name__ == "__main__":
    check()
