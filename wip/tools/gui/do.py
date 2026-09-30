#!/usr/bin/env python3
"""do.py OUT ACTION...: run xgui actions on :99, then shoot the screen to OUT.
Actions: c:X:Y (click), r:X:Y (right click), w:X:Y:N (wheel down N, negative up), m:X:Y (move), k:NAME (key), t:TEXT (type), s:SEC (wait)."""
import sys, time
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui
x = xgui.X(':99')
for a in sys.argv[2:]:
    p = a.split(':', 1)
    if p[0] == 'c':
        cx, cy = map(int, p[1].split(':')); x.click(cx, cy, 1); time.sleep(0.8)
    elif p[0] == 'r':
        cx, cy = map(int, p[1].split(':')); x.click(cx, cy, 3); time.sleep(0.8)
    elif p[0] == 'w':
        cx, cy, n = map(int, p[1].split(':'))
        for _ in range(abs(n)):
            x.click(cx, cy, 5 if n > 0 else 4); time.sleep(0.15)
        time.sleep(0.8)
    elif p[0] == 'm':
        cx, cy = map(int, p[1].split(':')); x.move(cx, cy); time.sleep(0.5)
    elif p[0] == 'k':
        *mods, name = p[1].split('+')
        x.key(name, tuple(mods)); time.sleep(0.5)
    elif p[0] == 't':
        for ch in p[1]:
            x.key(ch); time.sleep(0.05)
    elif p[0] == 's':
        time.sleep(float(p[1]))
time.sleep(1.5)
xgui.shot(S + '/gui/fb', sys.argv[1])
