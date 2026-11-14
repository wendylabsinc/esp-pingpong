#!/bin/bash
# Multi-node alignment run against a WendyOS Anchor with a buzzer on its header.
#
# Plays N pseudo-random codes (POST /dchirp?prn=<seed>), waits until the nodes'
# fragments stop arriving, downloads the new fragments, then runs
# e2e_truth.py per node (base station to node) and two_node_truth.py for every
# pair (node to node, the figure arrival-time-difference localisation needs).
#
# usage: ANCHOR_URL=http://<anchor>:8091 run_three.sh <out_dir> <n_codes> <spacing_s> <node> <node> [<node> ...]
#   node = the node's MAC as the Anchor names its directory, e.g. 441bf6804824
#   PYTHON defaults to python3 (needs numpy); DISTANCE_CM (default 5) feeds e2e_truth.py.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ANCHOR_DIR=$HERE/../../anchor
PY=${PYTHON:-python3}
B=${ANCHOR_URL:?set ANCHOR_URL, e.g. http://169.254.229.87:8091}
OUT=$1; N=$2; GAP=$3; shift 3
NODES=("$@")
[ ${#NODES[@]} -ge 2 ] || { echo "need at least two nodes"; exit 2; }
TOK=$(cd "$ANCHOR_DIR" && python3 -c "import hmac,hashlib;print(hmac.new(bytes.fromhex(open('secret.key').read().strip()),b'anchor-http',hashlib.sha256).hexdigest()[:16])")
H="Authorization: Bearer $TOK"
rm -rf "$OUT"; mkdir -p "$OUT"
listing() { for n in "${NODES[@]}"; do curl -s -H "$H" "$B/$n/" | grep -o 'href="[^"]*"' | cut -d'"' -f2 | sed "s|^|$n/|"; done | sort; }
listing > "$OUT/before.txt"
L0=$(curl -s -H "$H" "$B/dchirps.jsonl" | wc -l | tr -d ' ')
for i in $(seq 1 "$N"); do
  seed=$(printf "0x%04X" $(( (RANDOM % 65000) + 300 )))
  code=$(curl -s -o /dev/null -w "%{http_code}" --max-time 20 -X POST -H "$H" "$B/dchirp?prn=$seed")
  [ "$code" = 200 ] || echo "code $i seed $seed: HTTP $code"
  sleep "$GAP"
done
# Codes are sent within seconds of being heard (send slots stagger nodes by up
# to 5.4 s), so 30 s after the last code everything of interest is in; room
# events that keep arriving after that are not waited for.
sleep 30
listing > "$OUT/after.txt"
for n in "${NODES[@]}"; do mkdir -p "$OUT/$n"; done
comm -13 "$OUT/before.txt" "$OUT/after.txt" | while read -r f; do curl -s -H "$H" "$B/$f" -o "$OUT/$f"; done
curl -s -H "$H" "$B/dchirps.jsonl" | tail -n +$((L0+1)) | grep '"prn_seed": [0-9]' > "$OUT/codes.jsonl"
echo "codes: $(wc -l < "$OUT/codes.jsonl" | tr -d ' ')"
for n in "${NODES[@]}"; do
  tot=$(ls "$OUT/$n" | grep -c json); placed=$(grep -l first_sample_anchor_us "$OUT/$n"/*.json 2>/dev/null | wc -l | tr -d ' ')
  echo "$n: $tot fragments, $placed placed"
done
for n in "${NODES[@]}"; do
  echo "== e2e $n"; "$PY" "$HERE/e2e_truth.py" "$OUT/codes.jsonl" "$OUT/$n" "${DISTANCE_CM:-5}" 2>&1 | sed -n 1,4p
done
for ((i = 0; i < ${#NODES[@]}; i++)); do
  for ((j = i + 1; j < ${#NODES[@]}; j++)); do
    a=${NODES[$i]}; b=${NODES[$j]}; f="$OUT/pair_${a: -4}_${b: -4}.txt"
    "$PY" "$HERE/two_node_truth.py" "$OUT/codes.jsonl" "$OUT/$a" "$OUT/$b" > "$f" 2>&1
    echo "== pair ${a: -4} ${b: -4}"; cat "$f"
  done
done
