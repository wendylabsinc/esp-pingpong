"""Talk to an ESP32's built-in USB serial (USB-Serial-JTAG, 303a:1001) with
libusb from user space, for hosts whose kernel has no cdc_acm driver loaded.

Only the CDC data interface's two bulk endpoints are used. DTR/RTS are never
touched: on an ESP32 those lines drive reset and boot mode."""
import threading
import time
import usb.core
import usb.util


class UsbSerial:
    def __init__(self, vid=0x303A, pid=0x1001, serial=None, timeout=0.2):
        devs = [d for d in usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
                if serial is None or usb.util.get_string(d, d.iSerialNumber) == serial]
        if not devs:
            raise IOError(f"no USB device {vid:04x}:{pid:04x}" + (f" serial {serial}" if serial else ""))
        self.dev = devs[0]
        cfg = self.dev.get_active_configuration()
        data = next(i for i in cfg if i.bInterfaceClass == 0x0A)  # CDC data
        self.intf = data.bInterfaceNumber
        try:
            if self.dev.is_kernel_driver_active(self.intf):
                self.dev.detach_kernel_driver(self.intf)
        except (NotImplementedError, usb.core.USBError):
            pass
        usb.util.claim_interface(self.dev, self.intf)
        self.ep_in = usb.util.find_descriptor(data, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        self.ep_out = usb.util.find_descriptor(data, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.timeout_ms = int(timeout * 1000)
        self.buf = bytearray()
        self.cv = threading.Condition()
        self.read_bytes = 0      # total bytes ever read
        self.consumed = 0        # total bytes handed out by readline
        self.marks = []          # (read_bytes after a read, monotonic_ns of that read)
        self.line_ns = 0         # arrival time of the last line readline returned
        self.max_gap_ms = 0.0
        self.max_data_gap_ms = 0.0  # longest gap between two reads that both returned data
        # The ESP32's USB-Serial-JTAG driver silently drops output once the
        # host stops reading for ~50 ms, so a dedicated thread drains the
        # endpoint continuously; parsing happens elsewhere at its own pace.
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        last = time.monotonic()
        last_data = None
        while True:
            try:
                data = self.ep_in.read(16384, timeout=100)
            except usb.core.USBTimeoutError:
                data = b""
            t_ns = time.monotonic_ns()
            now = t_ns / 1e9
            self.max_gap_ms = max(self.max_gap_ms, (now - last) * 1000)
            last = now
            if data:
                if last_data is not None:
                    self.max_data_gap_ms = max(self.max_data_gap_ms, (now - last_data) * 1000)
                last_data = now
            else:
                last_data = None
            if data:
                with self.cv:
                    self.buf += data
                    self.read_bytes += len(data)
                    self.marks.append((self.read_bytes, t_ns))
                    self.cv.notify()

    def take_max_gap_ms(self):
        g, self.max_gap_ms = self.max_gap_ms, 0.0
        return g

    def write(self, data):
        for i in range(0, len(data), 64):
            self.ep_out.write(data[i:i + 64], timeout=1000)
        return len(data)

    def readline(self):
        deadline = time.monotonic() + self.timeout_ms / 1000
        with self.cv:
            while True:
                nl = self.buf.find(b"\n")
                if nl >= 0:
                    line = bytes(self.buf[:nl + 1]); del self.buf[:nl + 1]
                    self.consumed += nl + 1
                    while self.marks and self.marks[0][0] < self.consumed:
                        self.marks.pop(0)
                    # the read that delivered this line's newline
                    self.line_ns = self.marks[0][1] if self.marks else time.monotonic_ns()
                    return line
                left = deadline - time.monotonic()
                if left <= 0:
                    return b""
                self.cv.wait(left)
