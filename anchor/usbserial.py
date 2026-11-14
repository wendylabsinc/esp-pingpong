"""Talk to an ESP32's built-in USB serial (USB-Serial-JTAG, 303a:1001) with
libusb from user space, for hosts whose kernel has no cdc_acm driver loaded.

Only the CDC data interface's two bulk endpoints are used. DTR/RTS are never
touched: on an ESP32 those lines drive reset and boot mode."""
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

    def write(self, data):
        for i in range(0, len(data), 64):
            self.ep_out.write(data[i:i + 64], timeout=1000)
        return len(data)

    def readline(self):
        deadline = time.monotonic() + self.timeout_ms / 1000
        while True:
            nl = self.buf.find(b"\n")
            if nl >= 0:
                line = bytes(self.buf[:nl + 1]); del self.buf[:nl + 1]
                return line
            left = deadline - time.monotonic()
            if left <= 0:
                return b""
            try:
                self.buf += self.ep_in.read(16384, timeout=max(1, int(left * 1000)))
            except usb.core.USBTimeoutError:
                return b""
