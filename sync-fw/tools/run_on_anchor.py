"""Multi-node alignment run executed inside the Anchor container, for when the
Anchor is reachable only through Wendy Cloud (no route to its HTTP port).

Copy this file plus two_node_truth.py, e2e_truth.py and e2e_truth_lib.py into
the container, then start it in the background with container exec, e.g.
  PYTHONPATH=/app nohup python3 /tmp/tools/run_on_anchor.py r4 20 10 &
and read <OUT_DIR>/runs/<name>.log. It plays N codes on the Anchor's own
HTTP port, waits until fragments stop arriving, reads them from OUT_DIR
directly, then runs e2e_truth.py per node and two_node_truth.py per pair.
After the last code it waits only until every detection the nodes logged
over the radio during the run (D lines in <OUT_DIR>/nodelogs) has arrived as
a fragment: at least 1.5 s (node logs ship every 0.5 s), at most 10 s.
usage: run_on_anchor.py <name> <n_codes> <spacing_s> [distance_cm] [--timing]
  --timing also runs e2e_truth.py per node and two_node_truth.py per pair"""
import hashlib, hmac, itertools, json, os, random, subprocess, sys, time, urllib.request

timing = "--timing" in sys.argv
argv = [a for a in sys.argv if a != "--timing"]
name, n_codes, gap = argv[1], int(argv[2]), float(argv[3])
dist = argv[4] if len(argv) > 4 else "5"
out = os.environ.get("OUT_DIR", "/data/soundsync")
port = os.environ.get("HTTP_PORT", "8091")
here = os.path.dirname(os.path.abspath(__file__))
key = bytes.fromhex(os.environ["SOUNDSYNC_KEY"].strip())
token = hmac.new(key, b"anchor-http", hashlib.sha256).hexdigest()[:16]
run = os.path.join(out, "runs", name)
os.makedirs(run, exist_ok=True)
log = open(os.path.join(out, "runs", name + ".log"), "w", buffering=1)

def say(*a):
    print(*a, file=log, flush=True)

def http(method, path):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", method=method,
                                 headers={"Authorization": f"Bearer {token}"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.status, r.read()

nodes = sorted(d for d in os.listdir(out) if len(d) == 12 and all(c in "0123456789abcdef" for c in d))

def fragments():
    return {n: set(f for f in os.listdir(os.path.join(out, n)) if f.endswith(".json"))
            for n in nodes if os.path.isdir(os.path.join(out, n))}

before = fragments()
logdir = os.path.join(out, "nodelogs")

def log_sizes():
    try:
        return {f: os.path.getsize(os.path.join(logdir, f)) for f in os.listdir(logdir)}
    except FileNotFoundError:
        return {}

log_start = log_sizes()
t_start = time.time()
dchirps = os.path.join(out, "dchirps.jsonl")
l0 = sum(1 for _ in open(dchirps)) if os.path.exists(dchirps) else 0
say(f"run {name}: {n_codes} codes, {gap} s apart, nodes {nodes}")
for i in range(n_codes):
    seed = random.randrange(0x0300, 0xFF00)
    try:
        st, _ = http("POST", f"/dchirp?prn={seed:#06x}")
    except Exception as e:
        st = repr(e)
    if st != 200:
        say(f"code {i + 1} seed {seed:#06x}: {st}")
    time.sleep(gap)

def expected():
    """(node, frag id) of every detection logged since the run started"""
    exp = set()
    for f, size in log_sizes().items():
        node = f[:-4]
        with open(os.path.join(logdir, f), errors="replace") as fh:
            fh.seek(log_start.get(f, 0))
            for line in fh:
                parts = line.split(" ", 2)
                if len(parts) == 3 and parts[2].startswith("D,"):
                    try:
                        exp.add((node, int(parts[2].split(",")[1])))
                    except ValueError:
                        pass
    return exp

def arrived(exp):
    """only fragment files written during this run count: ids restart at boot"""
    cur = fragments()
    new = {n: cur.get(n, set()) - before.get(n, set()) for n in cur}
    have = set()
    for node, fid in exp:
        if any(f.endswith(f"-f{fid}.json") for f in new.get(node, ())):
            have.add((node, fid))
    return have

t_last = time.time()
while True:
    time.sleep(0.5)
    waited = time.time() - t_last
    exp = expected()
    missing = exp - arrived(exp)
    if waited >= 1.5 and not missing:
        break
    if waited >= 10:
        say(f"stopped waiting after 10 s; detections logged but not arrived: {sorted(missing)}")
        break
say(f"waited {time.time() - t_last:.1f} s after the last code for {len(exp)} logged detections")

now = fragments()
for n in nodes:
    d = os.path.join(run, n)
    os.makedirs(d, exist_ok=True)
    for f in now[n] - before[n]:
        for ext in (".json", ".wav"):
            src = os.path.join(out, n, f[:-5] + ext)
            if os.path.exists(src) and not os.path.exists(os.path.join(d, f[:-5] + ext)):
                os.symlink(src, os.path.join(d, f[:-5] + ext))
codes = [l for l in open(dchirps).read().splitlines()[l0:] if '"prn_seed": ' in l and '"prn_seed": null' not in l]
open(os.path.join(run, "codes.jsonl"), "w").write("\n".join(codes) + "\n")
say(f"codes logged: {len(codes)}")
for n in nodes:
    new = now[n] - before[n]
    placed = sum(1 for f in new if "first_sample_anchor_us" in open(os.path.join(out, n, f)).read())
    say(f"{n}: {len(new)} fragments, {placed} placed")
try:
    st = json.loads(http("GET", "/stats")[1])
    st["incomplete"] = [(i["src"][-4:], i["frag"], i["have"], i["count"]) for i in st["incomplete"]]
    say("stats:", json.dumps(st))
except Exception as e:
    say("stats:", repr(e))
# Detection per node: a fragment triggered by the code itself (trigger within
# -50..+300 ms of the code start), the code only inside a fragment triggered by
# something else, or no fragment at all. This is the "did every node catch
# every sound" count, independent of the timing analysis below.
code_list = [json.loads(l) for l in codes]
for n in nodes:
    metas = []
    for f in now[n] - before[n]:
        m = json.load(open(os.path.join(out, n, f)))
        if "trigger_anchor_us" in m:
            metas.append(m)
    got, inside, none = 0, [], []
    for i, c in enumerate(code_list, 1):
        t = c["dongle_esp_us"]
        if any(-50e3 < m["trigger_anchor_us"] - t < 300e3 for m in metas):
            got += 1
        elif any(0 < t - m["first_sample_anchor_us"] < m["n_samples"] / m["rate"] * 1e6 for m in metas):
            inside.append(i)
        else:
            none.append(i)
    say(f"DETECTED {n}: {got}/{len(code_list)}" + (f", only inside another fragment: {inside}" if inside else "")
        + (f", no fragment: {none}" if none else ""))
env = dict(os.environ, PYTHONPATH="/app:" + here)
if not timing:
    say("done")
    sys.exit(0)
codes_path = os.path.join(run, "codes.jsonl")
active = [n for n in nodes if now[n] - before[n]]
for n in active:
    r = subprocess.run([sys.executable, os.path.join(here, "e2e_truth.py"), codes_path, os.path.join(run, n), dist],
                       capture_output=True, text=True, env=env)
    say(f"== e2e {n}\n" + "\n".join((r.stdout + r.stderr).splitlines()[:4]))
for a, b in itertools.combinations(active, 2):
    r = subprocess.run([sys.executable, os.path.join(here, "two_node_truth.py"), codes_path,
                        os.path.join(run, a), os.path.join(run, b)], capture_output=True, text=True, env=env)
    say(f"== pair {a[-4:]} {b[-4:]}\n" + (r.stdout + r.stderr).rstrip())
say("done")
