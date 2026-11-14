"""Carry board reports from Wendy into the box server.

Each board publishes its reports as SensorLink channel 1, which Wendy routes
over USB, the local network, Bluetooth or Wendy Net. This reads that channel
with the wendy CLI and hands every report to server.py on this machine.

    python3 link.py DEVICE [DEVICE ...]     e.g. wendy-lite:/dev/cu.usbmodem101
    python3 link.py                         self-check
"""
import os
import socket
import subprocess
import sys
import threading
import time

MAGIC = b"TLF1"
WENDY = os.environ.get("WENDY_CLI", "wendy")
TARGET = ("127.0.0.1", int(os.environ.get("TAPLOC_UDP_PORT", 47800)))


def split(buf):
    """Byte stream -> (reports found, unconsumed tail)."""
    out = []
    while True:
        at = buf.find(MAGIC)
        if at < 0:
            return out, buf[-3:]  # a magic may straddle the next read
        buf = buf[at:]
        if len(buf) < 6:
            return out, buf
        size = buf[4] | buf[5] << 8
        if not 0 < size <= 1500:
            buf = buf[1:]  # not a real header; the box rejects anything forged anyway
            continue
        if len(buf) < 6 + size:
            return out, buf
        out.append(buf[6:6 + size])
        buf = buf[6 + size + (size & 1):]


def pump(device, sock):
    while True:
        proc = subprocess.Popen(
            [WENDY, "device", "audio", "listen", "--device", device, "--id", "1",
             "--sample-rate", "16000", "--channels", "1", "--stdout", "--non-interactive"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        buf, count = b"", 0
        while chunk := proc.stdout.read1(65536):
            reports, buf = split(buf + chunk)
            for report in reports:
                sock.sendto(report, TARGET)
            if not count and reports:
                print(f"{device}: reports arriving", flush=True)
            count += len(reports)
        why = proc.stderr.read().decode(errors="replace").strip().splitlines()[-1:] or ["stream ended"]
        print(f"{device}: {why[0][:200]} (after {count} reports); retrying in 3 s", flush=True)
        time.sleep(3)


def check():
    a, b = b"x" * 33, b"y" * 40
    stream = b"\0\0" + MAGIC + bytes([33, 0]) + a + b"\0" + b"junk" + MAGIC + bytes([40, 0]) + b
    got, tail = [], b""
    for i in range(0, len(stream), 7):  # arrives in arbitrary pieces
        reports, tail = split(tail + stream[i:i + 7])
        got += reports
    assert got == [a, b], got
    print(__doc__.strip(), "\n\nself-check ok")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        check()
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for device in sys.argv[1:]:
            threading.Thread(target=pump, args=(device, sock), daemon=True).start()
        while True:
            time.sleep(3600)
