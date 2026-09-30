#!/usr/bin/env python3
"""cal.py ITEM_Y OUT [SUB_Y]: open Calibration menu, pick item (and submenu item), screenshot the dialog."""
import sys, time
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui
x = xgui.X(':99')
y, out = int(sys.argv[1]), sys.argv[2]
x.click(330, 152, 1); time.sleep(3)
x.click(475, 117, 1); time.sleep(1.5)
x.click(520, y, 1); time.sleep(1.5)
if len(sys.argv) > 3:
    x.click(700, int(sys.argv[3]), 1); time.sleep(1.5)
time.sleep(3)
xgui.shot(S + '/gui/fb', out)
