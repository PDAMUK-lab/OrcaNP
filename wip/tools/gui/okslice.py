#!/usr/bin/env python3
"""okslice.py OKX OKY OUT [WAIT]: press the dialog's OK, let the model load, slice, screenshot the preview."""
import sys, time
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui
x = xgui.X(':99')
x.click(int(sys.argv[1]), int(sys.argv[2]), 1); time.sleep(10)
xgui.shot(S + '/gui/fb', sys.argv[3].replace('.png', '_prep.png'))
x.click(int(__import__('os').environ.get('SLICEX', '1155')), 152, 1); time.sleep(float(sys.argv[4]) if len(sys.argv) > 4 else 60)
xgui.shot(S + '/gui/fb', sys.argv[3])
