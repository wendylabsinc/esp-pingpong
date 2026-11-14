"""ens.c (node side, as compiled C) against anchor/enstream.py (Anchor side)
over a link that loses, duplicates and reorders frames. Both directions carry
random data at once; the test passes when both sides received exactly what the
other wrote. usage: test_ens.py <path to esp-pingpong/anchor> [seed]"""
import ctypes, os, random, subprocess, sys, tempfile

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, sys.argv[1])
import enstream

lib_path = os.path.join(tempfile.mkdtemp(), "libens.dylib")
subprocess.check_call(["cc", "-O1", "-Wall", "-Wextra", "-Werror", "-shared", "-o", lib_path,
                       os.path.join(here, "..", "src", "ens.c")])
lib = ctypes.CDLL(lib_path)
# offset of stat_retx inside struct ens, from a tiny C program, so the test can read it
_probe = os.path.join(os.path.dirname(lib_path), "probe")
subprocess.run(["cc", "-o", _probe, "-x", "c", "-", "-I", os.path.join(here, "..", "src")], check=True,
               input=b'#include <stdio.h>\n#include <stddef.h>\n#include "ens.h"\nint main(void){printf("%zu %zu",offsetof(struct ens,stat_retx),sizeof(struct ens));}')
node_retx_offset, ens_size = map(int, subprocess.check_output([_probe]).split())

class Seg(ctypes.Structure):
    _fields_ = [("conn", ctypes.c_uint32), ("seq", ctypes.c_uint32), ("ack", ctypes.c_uint32),
                ("wnd", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("data", ctypes.POINTER(ctypes.c_uint8)), ("len", ctypes.c_uint16)]

lib.ens_write.restype = lib.ens_read.restype = ctypes.c_size_t
lib.ens_write.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
lib.ens_read.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
lib.ens_input.argtypes = [ctypes.c_void_p, ctypes.POINTER(Seg), ctypes.c_int64]
lib.ens_poll.argtypes = [ctypes.c_void_p, ctypes.c_int64, ctypes.POINTER(Seg), ctypes.c_char_p]
lib.ens_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_bool, ctypes.c_char_p, ctypes.c_size_t,
                         ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint16, ctypes.c_uint32, ctypes.c_int64,
                         ctypes.c_int64]

def run(seed, loss, dup, reorder, up_bytes, down_bytes):
    rng = random.Random(seed)
    node = ctypes.create_string_buffer(ens_size)
    tx, rx, scratch = (ctypes.create_string_buffer(n) for n in (8192, 4096, 1400))
    conn = rng.getrandbits(30)
    lib.ens_init(node, conn, True, tx, len(tx), rx, len(rx), 1400, 4 * 1400, 150_000, 0)
    anchor = None
    up, down = rng.randbytes(up_bytes), rng.randbytes(down_bytes)
    up_at = down_at = 0
    got_up, got_down = bytearray(), bytearray()
    to_anchor, to_node = [], []                              # (deliver_at_us, frame)
    now, frames = 0, 0
    last_at = {}
    def channel(queue, frame):
        nonlocal frames
        frames += 1
        if rng.random() < loss:
            return
        # a real ESP-NOW link from one sender keeps order; reordering only when asked for
        at = max(now + rng.randint(1000, 4000), last_at.get(id(queue), 0) + 1)
        last_at[id(queue)] = at
        delay = at - now + (rng.randint(5000, 30000) if rng.random() < reorder else 0)
        queue.append((now + delay, frame))
        if rng.random() < dup:
            queue.append((now + delay + rng.randint(0, 20000), frame))
    while (len(got_up) < len(up) or len(got_down) < len(down)) and now < 600_000_000:
        now += 1000
        # applications
        if up_at < len(up):
            up_at += lib.ens_write(node, up[up_at:up_at + 3000], min(3000, len(up) - up_at))
        buf = ctypes.create_string_buffer(2048)
        n = lib.ens_read(node, buf, len(buf))
        got_down += buf.raw[:n]
        if anchor:
            if down_at < len(down):
                down_at += anchor.write(down[down_at:down_at + 3000])
            got_up += anchor.read(4096)
        # deliveries
        for q, is_up in ((to_anchor, True), (to_node, False)):
            due = [f for f in q if f[0] <= now]
            q[:] = [f for f in q if f[0] > now]
            for _, f in sorted(due, key=lambda f: f[0]):
                if is_up:
                    c, seq, ack, wnd, flags, data = f
                    if anchor is None:
                        if not flags & enstream.SYN:
                            continue
                        anchor = enstream.Ens(c, False, 32768, 65536, enstream.DOWN_MSS, 8 * enstream.DOWN_MSS, 250_000, now)
                    anchor.input(c, seq, ack, wnd, flags, data, now)
                else:
                    c, seq, ack, wnd, flags, data = f
                    arr = (ctypes.c_uint8 * max(1, len(data))).from_buffer_copy(data or b"\0")
                    seg = Seg(c, seq, ack, wnd, flags, arr, len(data))
                    lib.ens_input(node, ctypes.byref(seg), now)
        # transmissions
        seg = Seg()
        for _ in range(2):
            if not lib.ens_poll(node, now, ctypes.byref(seg), scratch):
                break
            data = bytes(seg.data[:seg.len]) if seg.len else b""
            # the wire mapping, both ways, as the firmware and the Anchor do it
            pkt = b"SSF1" + enstream.struct.pack("<IHHII", enstream.STREAM_MARK | seg.conn, seg.wnd, seg.flags,
                                                 seg.seq, seg.ack) + (data or b"\0") + b"\0" * 8
            parsed = enstream.parse_uplink(pkt)
            if parsed[4] & enstream.ACKONLY:
                parsed = parsed[:5] + (b"",)
            channel(to_anchor, parsed)
        if anchor:
            for _ in range(2):
                s = anchor.poll(now)
                if not s:
                    break
                line = enstream.nack_line("441bf6804ff8", s)
                assert len(line) < 500, len(line)
                # decode the command the way the dongle and the node do
                _, mac, boot, frag, vals = line.split(" ")
                v = [int(x) for x in vals.split(",")]
                assert len(v) <= 64
                flags, ln = v[3] & 0xFF, v[3] >> 8
                payload = b"".join(x.to_bytes(2, "little") for x in v[4:])[:ln]
                channel(to_node, (int(boot, 16) & 0x3FFFFFFF, int(frag), v[0] | (v[1] << 16), v[2], flags, payload))
    ok = bytes(got_up) == up and bytes(got_down) == down
    print(f"seed {seed} loss {loss:.0%} dup {dup:.0%} reorder {reorder:.0%}: {'OK' if ok else 'FAIL'} "
          f"up {len(got_up)}/{len(up)} down {len(got_down)}/{len(down)} in {now / 1e6:.1f} s simulated, {frames} frames, "
          f"anchor retx {anchor.stat_retx if anchor else None}, "
          f"node retx {ctypes.c_uint32.from_buffer(node, node_retx_offset).value}")
    return ok

seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
results = [run(seed, 0.0, 0.0, 0.0, 60_000, 20_000),
           run(seed + 1, 0.05, 0.02, 0.05, 60_000, 20_000),
           run(seed + 2, 0.20, 0.05, 0.10, 40_000, 12_000),
           run(seed + 3, 0.02, 0.0, 0.0, 300_000, 8_000)]
sys.exit(0 if all(results) else 1)
