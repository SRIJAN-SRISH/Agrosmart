from machine import Pin
import time
import framebuf

class ILI9341:
    def __init__(self, spi, cs, dc, rst, width=240, height=320):
        self.spi = spi
        self.cs = cs
        self.dc = dc
        self.rst = rst
        self.width = width
        self.height = height
        self.cs.init(self.cs.OUT, value=1)
        self.dc.init(self.dc.OUT, value=0)
        self.rst.init(self.rst.OUT, value=1)
        self.buf = bytearray(width * height * 2)
        self.fb = framebuf.FrameBuffer(self.buf, width, height, framebuf.RGB565)
        self.reset()
        self.init_display()

    def reset(self):
        self.rst.value(0); time.sleep_ms(50)
        self.rst.value(1); time.sleep_ms(150)

    def write_cmd(self, cmd):
        self.cs.value(0); self.dc.value(0)
        self.spi.write(bytearray([cmd]))
        self.cs.value(1)

    def write_data(self, data):
        self.cs.value(0); self.dc.value(1)
        self.spi.write(bytearray(data) if isinstance(data, list) else data)
        self.cs.value(1)

    def init_display(self):
        self.write_cmd(0x01); time.sleep_ms(150)
        self.write_cmd(0x28)
        self.write_cmd(0x3A); self.write_data([0x55])
        self.write_cmd(0x36); self.write_data([0xC8])   # confirmed correct - do not change
        self.write_cmd(0x11); time.sleep_ms(150)
        self.write_cmd(0x29); time.sleep_ms(50)

    def set_window(self, x0, y0, x1, y1):
        self.write_cmd(0x2A)
        self.write_data([x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF])
        self.write_cmd(0x2B)
        self.write_data([y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF])
        self.write_cmd(0x2C)

    def show(self):
        self.set_window(0, 0, self.width - 1, self.height - 1)
        self.cs.value(0); self.dc.value(1)
        self.spi.write(self.buf)
        self.cs.value(1)

    def fill(self, color):
        self.fb.fill(color)

    def fill_rect(self, x, y, w, h, color):
        self.fb.fill_rect(x, y, w, h, color)

    def rect(self, x, y, w, h, color):
        self.fb.rect(x, y, w, h, color)

    def hline(self, x, y, w, color):
        self.fb.hline(x, y, w, color)

    def vline(self, x, y, h, color):
        self.fb.vline(x, y, h, color)

    def line(self, x0, y0, x1, y1, color):
        self.fb.line(x0, y0, x1, y1, color)

    def ellipse(self, x, y, xr, yr, color, fill=False):
        self.fb.ellipse(x, y, xr, yr, color, fill)

    def pixel(self, x, y, color):
        self.fb.pixel(x, y, color)

    def text(self, s, x, y, color):
        self.fb.text(s, x, y, color)

def color565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
