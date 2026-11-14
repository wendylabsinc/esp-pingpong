"""Direct register access to a TCA9538 I/O expander on a Linux I2C bus.
Registers: 0 input, 1 output, 2 polarity inversion, 3 configuration (1 = input).
Uses I2C_SLAVE_FORCE because the kernel pca953x driver owns the address."""
import fcntl, os, time

I2C_SLAVE_FORCE = 0x0706

class Expander:
    def __init__(self, bus, addr):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, addr)

    def read(self, reg):
        os.write(self.fd, bytes([reg]))
        return os.read(self.fd, 1)[0]

    def write(self, reg, val):
        os.write(self.fd, bytes([reg, val & 0xFF]))

    def regs(self):
        return {name: self.read(r) for r, name in ((0, "input"), (1, "output"), (2, "polarity"), (3, "config"))}

    def close(self):
        os.close(self.fd)
