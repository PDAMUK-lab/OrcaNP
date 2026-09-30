#!/usr/bin/env python3
"""waitshot.py OUT [TIMEOUT]: wait until the app's main window is drawn (the screen stops being blank), then shoot."""
import sys, time
sys.path.insert(0, '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad')
import xgui
from PIL import Image
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
out, timeout = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 120
t0 = time.time()
while True:
    xgui.shot(S + '/gui/fb', out)
    im = Image.open(out).convert('L').resize((80, 50))
    if len(set(im.getdata())) > 40 or time.time() - t0 > timeout:
        break
    time.sleep(3)
time.sleep(15)
xgui.shot(S + '/gui/fb', out)
print('shot after %.0f s' % (time.time() - t0))
