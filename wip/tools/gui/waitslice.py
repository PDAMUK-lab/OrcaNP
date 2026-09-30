#!/usr/bin/env python3
"""waitslice.py [TIMEOUT]: click Slice plate (maximised window) and wait until the Export G-code button turns teal."""
import sys, time
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui
from PIL import Image
x = xgui.X(':99')
x.click(1302, 52, 1)
t0, timeout = time.time(), float(sys.argv[1]) if len(sys.argv) > 1 else 600
while time.time() - t0 < timeout:
    time.sleep(3)
    xgui.shot(S + '/gui/fb', S + '/gui/opt/_poll.png')
    r, g, b = Image.open(S + '/gui/opt/_poll.png').convert('RGB').getpixel((1500, 52))
    if g > 150 and r < 130:
        print('sliced after %.0f s' % (time.time() - t0)); break
else:
    print('not sliced after %.0f s' % timeout)
