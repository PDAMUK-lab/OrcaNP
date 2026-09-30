#!/usr/bin/env python3
"""resize.py W H [MATCH_W MATCH_H]: move the top-level window of MATCH_W x MATCH_H (default 1200 x 800) on :99 to
(0, 0) and resize it to W x H (there is no window manager to do it)."""
import ctypes, sys
x = ctypes.CDLL('libX11.so.6')
x.XOpenDisplay.restype = ctypes.c_void_p
x.XDefaultRootWindow.restype = ctypes.c_ulong
x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
d = x.XOpenDisplay(b':99')
root = x.XDefaultRootWindow(d)
r, p = ctypes.c_ulong(), ctypes.c_ulong()
kids, n = ctypes.POINTER(ctypes.c_ulong)(), ctypes.c_uint()
x.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
                         ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint)]
x.XQueryTree(d, root, ctypes.byref(r), ctypes.byref(p), ctypes.byref(kids), ctypes.byref(n))
x.XGetGeometry.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)] + [ctypes.POINTER(ctypes.c_int)] * 2 + \
                          [ctypes.POINTER(ctypes.c_uint)] * 4
x.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
want = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (1200, 800)
for i in range(n.value):
    w = kids[i]
    rr, gx, gy, gw, gh, bw, dp = ctypes.c_ulong(), ctypes.c_int(), ctypes.c_int(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
    x.XGetGeometry(d, w, ctypes.byref(rr), ctypes.byref(gx), ctypes.byref(gy), ctypes.byref(gw), ctypes.byref(gh), ctypes.byref(bw), ctypes.byref(dp))
    print(hex(w), gx.value, gy.value, gw.value, gh.value)
    if (gw.value, gh.value) == want:
        x.XMoveResizeWindow(d, w, 0, 0, int(sys.argv[1]), int(sys.argv[2]))
        print('resized', hex(w))
x.XFlush(ctypes.c_void_p(d))
