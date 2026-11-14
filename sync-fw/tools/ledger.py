"""Per-code, per-node ledger: how far each played code got, so every miss is
attributed to one stage instead of folded into a percentage.

Stages, per code and node:
  played    the Anchor logged the code (codes.jsonl from run_on_anchor.py)
  detected  the node printed a D line (trigger) within -50..+300 ms of it
  sent      the node printed an S line for that fragment
  arrived   the Anchor wrote the fragment (complete or salvaged)
  placed    its trigger lands within -50..+300 ms of the code on Anchor time

Node D/S lines come from serial captures (tools/capture.py) or from the
Anchor's node logs (<OUT_DIR>/nodelogs/<node>.log, sent over the radio); their sample
indices are mapped to Anchor time through the node's own placed fragments
(first_n <-> first_sample_anchor_us), so no host clock is involved.

With the node's "trace on" V lines in the capture, each miss also gets the
trigger's view of the 0..300 ms after the code: peak dB over the noise floor,
the longest run of blocks over threshold, whether the trigger was already high
from an earlier sound (no fresh onset), and the noise floor.

usage: ledger.py <run_dir> <node_mac>=<serial_capture.csv> [...]
  run_dir: a copy of <OUT_DIR>/runs/<name> (codes.jsonl and <node>/*.json)"""
import glob, json, os, sys

run = sys.argv[1]
captures = dict(a.split("=", 1) for a in sys.argv[2:])
codes = [json.loads(l) for l in open(os.path.join(run, "codes.jsonl")) if l.strip()]
LO, HI = -50e3, 300e3
THR_DB, NEED_BLOCKS = float(__import__("os").environ.get("THR_DB", "15")), int(__import__("os").environ.get("NEED_BLOCKS", "3"))
peaks_detected = []
total = {"played": 0, "detected": 0, "sent": 0, "arrived": 0, "placed": 0}
misses = []

for node, cap in sorted(captures.items()):
    frags = {}
    for js in glob.glob(os.path.join(run, node, "*.json")):
        m = json.load(open(js))
        frags[m["frag"]] = m
    placed = [m for m in frags.values() if "first_sample_anchor_us" in m]
    if not placed:
        print(f"{node}: no placed fragments, cannot map node time")
        continue
    ref = max(placed, key=lambda m: m["frag"])
    us_per_n = 1e6 / ref["rate"]

    def n_to_anchor(n):
        # nearest placed fragment in sample index gives the local mapping
        m = min(placed, key=lambda m: abs(m["first_n"] - n))
        return m["first_sample_anchor_us"] + (n - m["first_n"]) * us_per_n

    det, sent, trace = {}, set(), []
    for line in open(cap, errors="replace"):
        line = line.strip()
        if line[:1].isdigit() and " " in line.split(",", 1)[0]:
            # Anchor node log (<OUT>/nodelogs/<node>.log): "<utc> <boot> <line>"
            line = "0," + line.split(" ", 2)[-1]
        f = line.split(",")
        if len(f) >= 4 and f[1] == "D":
            try:
                det[int(f[2])] = n_to_anchor(int(f[3]))
            except ValueError:
                pass
        elif len(f) >= 9 and f[1] == "V":
            try:
                trace.append((n_to_anchor(int(f[2])), int(f[3]) / 10, int(f[4]), int(f[5]), int(f[6]), int(f[7]), int(f[8]) / 10))
            except ValueError:
                pass
        elif len(f) >= 3 and f[1] == "S":
            try:
                sent.add(int(f[2]))
            except ValueError:
                pass
    stages = {k: 0 for k in total}
    for i, c in enumerate(codes, 1):
        t = c["dongle_esp_us"]
        stages["played"] += 1
        hit = [fid for fid, ta in det.items() if LO < ta - t < HI]
        win = [v for v in trace if -100e3 < v[0] - t < HI]
        if hit and win:
            peaks_detected.append(max(v[1] for v in win))
        if not hit:
            why = "not detected"
            if win:
                peak = max(v[1] for v in win)
                hits = max(v[2] for v in win)
                before = [v for v in win if v[0] < t]
                high_before = any(v[3] & 0x10 for v in before)  # last block before the code over threshold
                onsets = sum(bin(v[4]).count("1") for v in win if v[0] >= t)
                floor = min(v[6] for v in win)
                pend = max(v[5] for v in win)
                if high_before and not onsets:
                    cause = "no fresh onset: already over threshold from an earlier sound"
                elif onsets:
                    cause = f"onset seen but not triggered ({pend} fragments recording, or within 250 ms of the previous trigger)"
                elif peak < THR_DB:
                    cause = "too quiet"
                elif hits < NEED_BLOCKS:
                    cause = f"over threshold for only {hits} block(s)"
                else:
                    cause = "unclear"
                why += f": {cause}; peak {peak:+.1f} dB over floor, longest run {hits} blocks, floor {floor:.1f} dB"
            elif trace:
                why += ": no trace lines around it"
            misses.append((node, i, why))
            continue
        stages["detected"] += 1
        fid = hit[0]
        if fid not in sent:
            misses.append((node, i, f"detected as fragment {fid}, never sent"))
            continue
        stages["sent"] += 1
        if fid not in frags:
            misses.append((node, i, f"fragment {fid} sent, never written by the Anchor"))
            continue
        stages["arrived"] += 1
        m = frags[fid]
        if "trigger_anchor_us" not in m or not (LO < m["trigger_anchor_us"] - t < HI):
            where = "unplaced" if "trigger_anchor_us" not in m else f"placed {(m['trigger_anchor_us'] - t) / 1e3:+.1f} ms off"
            misses.append((node, i, f"fragment {fid} arrived, {where}"))
            continue
        stages["placed"] += 1
    for k in total:
        total[k] += stages[k]
    print(f"{node}: " + ", ".join(f"{k} {v}" for k, v in stages.items()))

print("ALL: " + ", ".join(f"{k} {v}" for k, v in total.items()))
if peaks_detected:
    pk = sorted(peaks_detected)
    print(f"detected codes: peak over floor min {pk[0]:+.1f} dB, median {pk[len(pk) // 2]:+.1f} dB (threshold +{THR_DB:g} dB for {NEED_BLOCKS} blocks)")
for node, i, why in misses:
    print(f"  miss: {node} code {i}: {why}")
