#!/usr/bin/env python3
"""typein.py X Y TEXT [X Y TEXT ...]: click each field, select all, type the text (digits and '.')."""
import sys, time
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui
x = xgui.X(':99')
a = sys.argv[1:]
for i in range(0, len(a), 3):
    x.click(int(a[i]), int(a[i + 1]), 1); time.sleep(0.4)
    x.key('a', ('Control_L',)); time.sleep(0.2)
    for c in a[i + 2]:
        x.key({'.': 'period', '-': 'minus'}.get(c, c)); time.sleep(0.05)
    x.key('Tab'); time.sleep(0.3)
