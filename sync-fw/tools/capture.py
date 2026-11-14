"""Log soundsync-fw serial lines to a file for N seconds, prefixed with host time.
usage: capture.py <port> <seconds> <out.csv> [command ...]"""
import sys, time, serial
port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
s = serial.Serial(port, 115200, timeout=0.2)
for c in sys.argv[4:]:
    s.write((c + "\n").encode()); time.sleep(0.05)
end = time.time() + secs
n = 0
with open(out, "w") as f:
    while time.time() < end:
        l = s.readline()
        if not l:
            continue
        f.write(f"{time.monotonic_ns()},{l.decode(errors='replace').strip()}\n")
        n += 1
print(f"{n} lines -> {out}")
