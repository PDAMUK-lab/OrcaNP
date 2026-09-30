#!/usr/bin/env python3
"""Drive an Xvfb display: screenshots from its framebuffer file, keys and clicks through XTest.

  xgui.py shot FBDIR OUT.png [x y w h]   convert Xvfb_screen0 (XWD) to PNG, optionally cropped
  xgui.py key DISPLAY KEYSYM [MOD...]    press a key, e.g. key :99 r Control_L
  xgui.py click DISPLAY X Y [BUTTON]     move and click
  xgui.py move DISPLAY X Y
  xgui.py drag DISPLAY X0 Y0 X1 Y1       press at one point, move in steps, release at the other
  xgui.py keys DISPLAY KEYSYM COUNT      press a key COUNT times
"""
import ctypes
import struct
import sys
import time

from PIL import Image


def shot(fbdir, out, crop=None):
    data = open(fbdir + '/Xvfb_screen0', 'rb').read()
    f = struct.unpack('>25I', data[:100])
    header_size, fmt, depth, width, height = f[0], f[2], f[3], f[4], f[5]
    byte_order, bpp, bpl, ncolors = f[7], f[11], f[12], f[19]
    off = header_size + ncolors * 12
    raw = data[off:off + bpl * height]
    assert bpp == 32, bpp
    mode = 'BGRX' if byte_order == 0 else 'XRGB'
    img = Image.frombuffer('RGB', (width, height), raw, 'raw', mode, bpl, 1)
    if crop:
        x, y, w, h = crop
        img = img.crop((x, y, x + w, y + h))
    img.save(out)
    print(out, img.size)


class X:
    def __init__(self, display):
        self.x11 = ctypes.CDLL('libX11.so.6')
        self.xtst = ctypes.CDLL('libXtst.so.6')
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XStringToKeysym.restype = ctypes.c_ulong
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        for fn in ('XTestFakeKeyEvent', 'XTestFakeButtonEvent', 'XTestFakeMotionEvent'):
            getattr(self.xtst, fn).argtypes = None
        self.d = self.x11.XOpenDisplay(display.encode())
        if not self.d:
            raise SystemExit('cannot open display ' + display)

    def flush(self):
        self.x11.XFlush(ctypes.c_void_p(self.d))

    def keycode(self, name):
        sym = self.x11.XStringToKeysym(name.encode())
        return self.x11.XKeysymToKeycode(ctypes.c_void_p(self.d), sym)

    def key(self, name, mods=()):
        d = ctypes.c_void_p(self.d)
        for m in mods:
            self.xtst.XTestFakeKeyEvent(d, ctypes.c_uint(self.keycode(m)), ctypes.c_int(1), ctypes.c_ulong(0))
        kc = ctypes.c_uint(self.keycode(name))
        self.xtst.XTestFakeKeyEvent(d, kc, ctypes.c_int(1), ctypes.c_ulong(0))
        self.xtst.XTestFakeKeyEvent(d, kc, ctypes.c_int(0), ctypes.c_ulong(30))
        for m in reversed(mods):
            self.xtst.XTestFakeKeyEvent(d, ctypes.c_uint(self.keycode(m)), ctypes.c_int(0), ctypes.c_ulong(0))
        self.flush()
        # A release can be lost while a menu grabs the keyboard; a stuck key auto-repeats.
        time.sleep(0.2)
        self.xtst.XTestFakeKeyEvent(d, kc, ctypes.c_int(0), ctypes.c_ulong(0))
        for m in mods:
            self.xtst.XTestFakeKeyEvent(d, ctypes.c_uint(self.keycode(m)), ctypes.c_int(0), ctypes.c_ulong(0))
        self.flush()

    def move(self, x, y):
        self.xtst.XTestFakeMotionEvent(ctypes.c_void_p(self.d), ctypes.c_int(-1), ctypes.c_int(x), ctypes.c_int(y), ctypes.c_ulong(0))
        self.flush()

    def click(self, x, y, button=1):
        # ImGui needs a rendered frame with the pointer over the control before the press.
        self.move(x - 2, y)
        time.sleep(0.3)
        self.move(x, y)
        time.sleep(0.6)
        d = ctypes.c_void_p(self.d)
        self.xtst.XTestFakeButtonEvent(d, ctypes.c_uint(button), ctypes.c_int(1), ctypes.c_ulong(0))
        self.flush()
        time.sleep(0.05)
        self.xtst.XTestFakeButtonEvent(d, ctypes.c_uint(button), ctypes.c_int(0), ctypes.c_ulong(0))
        self.flush()


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'shot':
        shot(sys.argv[2], sys.argv[3], tuple(map(int, sys.argv[4:8])) if len(sys.argv) > 4 else None)
    elif cmd == 'key':
        X(sys.argv[2]).key(sys.argv[3], sys.argv[4:])
    elif cmd == 'click':
        X(sys.argv[2]).click(int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 1)
    elif cmd == 'move':
        X(sys.argv[2]).move(int(sys.argv[3]), int(sys.argv[4]))
    elif cmd == 'drag':
        x = X(sys.argv[2])
        x0, y0, x1, y1 = map(int, sys.argv[3:7])
        x.move(x0, y0); time.sleep(0.6)
        d = ctypes.c_void_p(x.d)
        x.xtst.XTestFakeButtonEvent(d, ctypes.c_uint(1), ctypes.c_int(1), ctypes.c_ulong(0)); x.flush(); time.sleep(0.3)
        for k in range(1, 21):
            x.move(x0 + (x1 - x0) * k // 20, y0 + (y1 - y0) * k // 20); time.sleep(0.1)
        time.sleep(0.4)
        x.xtst.XTestFakeButtonEvent(d, ctypes.c_uint(1), ctypes.c_int(0), ctypes.c_ulong(0)); x.flush()
    elif cmd == 'keys':
        x = X(sys.argv[2])
        for _ in range(int(sys.argv[4])):
            x.key(sys.argv[3])
