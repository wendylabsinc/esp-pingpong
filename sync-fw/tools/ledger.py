"""Per-code, per-node ledger: how far each played code got, so every miss is
attributed to one stage instead of folded into a percentage.

Stages, per code and node:
  played    the Anchor logged the code (codes.jsonl from run_on_anchor.py)
  detected  the node printed a D line (trigger) within -50..+300 ms of it
  sent      the node printed an S line for that fragment
  arrived   the Anchor wrote the fragment (complete or salvaged)
  placed    its trigger lands within -50..+300 ms of the code on Anchor time

Node D/S lines come from serial captures (tools/capture.py); their sample
indices are mapped to Anchor time through the node's own placed fragments
(first_n <-> first_sample_anchor_us), so no host clock is involved.

usage: ledger.py <run_dir> <node_mac>=<serial_capture.csv> [...]
  run_dir: a copy of <OUT_DIR>/runs/<name> (codes.jsonl and <node>/*.json)"""
import glob, json, os, sys

run = sys.argv[1]
captures = dict(a.split("=", 1) for a in sys.argv[2:])
codes = [json.loads(l) for l in open(os.path.join(run, "codes.jsonl")) if l.strip()]
LO, HI = -50e3, 300e3
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

    det, sent = {}, set()
    for line in open(cap, errors="replace"):
        f = line.strip().split(",")
        if len(f) >= 4 and f[1] == "D":
            try:
                det[int(f[2])] = n_to_anchor(int(f[3]))
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
        if not hit:
            misses.append((node, i, "not detected"))
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
for node, i, why in misses:
    print(f"  miss: {node} code {i}: {why}")
