"""Minimal Linux GPIO character-device (uAPI v2) access, no libgpiod needed.
Requests one line as an output and toggles it, stamping every edge."""
import fcntl, glob, os, struct, time

def _iowr(nr, size):
    return (3 << 30) | (size << 16) | (0xB4 << 8) | nr

def _ior(nr, size):
    return (2 << 30) | (size << 16) | (0xB4 << 8) | nr

CHIPINFO = _ior(0x01, 68)
REQ_FMT = "<64I32sQI5I" + "IIQQ" * 10 + "II5Ii"
REQ_SIZE = struct.calcsize(REQ_FMT)          # 592
GET_LINE = _iowr(0x07, REQ_SIZE)
SET_VALUES = _iowr(0x0F, 16)
FLAG_OUTPUT = 1 << 3

def chips():
    out = []
    for path in sorted(glob.glob("/dev/gpiochip*")):
        try:
            fd = os.open(path, os.O_RDWR)
            info = bytearray(68)
            fcntl.ioctl(fd, CHIPINFO, info, True)
            os.close(fd)
            name, label, lines = struct.unpack("<32s32sI", info)
            out.append(dict(path=path, name=name.rstrip(b"\0").decode(), label=label.rstrip(b"\0").decode(), lines=lines))
        except OSError as e:
            out.append(dict(path=path, error=str(e)))
    return out

class Line:
    def __init__(self, chip, offset, consumer=b"soundsync"):
        self.chipfd = os.open(chip, os.O_RDWR)
        vals = [offset] + [0] * 63 + [consumer, FLAG_OUTPUT, 0, 0, 0, 0, 0, 0] + [0] * 40 + [1, 0, 0, 0, 0, 0, 0, -1]
        buf = bytearray(struct.pack(REQ_FMT, *vals))
        fcntl.ioctl(self.chipfd, GET_LINE, buf, True)
        self.fd = struct.unpack_from("<i", buf, REQ_SIZE - 4)[0]

    def set(self, v):
        fcntl.ioctl(self.fd, SET_VALUES, struct.pack("<QQ", 1 if v else 0, 1))

    def close(self):
        os.close(self.fd); os.close(self.chipfd)

def square(line, hz, ms):
    """Toggle at hz for ms with a busy-wait schedule; return the monotonic ns of
    every edge as it actually happened (Linux will not hit the schedule exactly)."""
    half = 1e9 / hz / 2
    n = int(ms / 1000 * hz * 2)
    edges = []
    t0 = time.monotonic_ns()
    for k in range(n):
        target = t0 + k * half
        while time.monotonic_ns() < target:
            pass
        line.set(k % 2 == 0)
        edges.append(time.monotonic_ns())
    line.set(0)
    return edges
