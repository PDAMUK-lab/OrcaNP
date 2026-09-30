#!/usr/bin/env python3
"""shots.py MODEL VARIANT ZOOM [MAX_WAIT]: slice MODEL in the GUI with the variant's environment and take an
external view (all layers) and an internal one (layers up to half height). Controls are found on the
screen rather than at fixed places: the legend's width and the warnings shown differ between slices."""
import os, subprocess, sys, time
from PIL import Image

S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
sys.path.insert(0, S)
import xgui

VARIANTS = {
    'today': {},
    'graded2W': {'S4_PROTO_FACET_SIZE': '2', 'S4_PROTO_FACET_DISTANCE': '0.1', 'S4_PROTO_WEIGHTED': '1', 'S4_PROTO_WARM': '1'},
    'today_opt': {'S4_PROTO_WARM': '1', 'S4_PROTO_MT': '1'},
    'graded2W_opt': {'S4_PROTO_FACET_SIZE': '2', 'S4_PROTO_FACET_DISTANCE': '0.1', 'S4_PROTO_WEIGHTED': '1', 'S4_PROTO_WARM': '1',
                     'S4_PROTO_MT': '1'},
}
OX, OY = 200, 100  # window origin on the screen
x = xgui.X(':99')


def screen():
    xgui.shot(S + '/gui/fb', S + '/gui/q/_poll.png')
    return Image.open(S + '/gui/q/_poll.png').convert('RGB')


def click(cx, cy, button=1):
    x.click(OX + cx, OY + cy, button)


def sliced(img):
    r, g, b = img.getpixel((OX + 1100, OY + 52))  # the Export G-code button turns teal
    return g > 170 and r < 130


def legend_open(img):
    # The legend's dark panel spans the canvas top right; closed, only its header row remains.
    return sum(img.getpixel((OX + 900, OY + y))[0] < 90 for y in range(140, 400, 10)) > 15


def legend_icon(img):
    for cx in range(560, 1100):
        if all(sum(img.getpixel((OX + cx + k, OY + 96))) < 260 for k in range(4)):
            return cx + 17, 96
    return None


def warning_closers(img):
    tops, prev = [], False
    for cy in range(420, 790):
        p = img.getpixel((OX + 693, OY + cy))
        orange = p[0] > 200 and 90 < p[1] < 200 and p[2] < 90
        if orange and not prev:
            tops.append(cy)
        prev = orange
    out = []
    for top in tops:
        dark = [cx for cx in range(1040, 1105) if sum(img.getpixel((OX + cx, OY + top + 19))) < 300]
        if dark:
            out.append(((min(dark) + max(dark)) // 2, top + 19))
    return out


def main():
    model, variant, zoom = sys.argv[1], sys.argv[2], int(sys.argv[3])
    max_wait = int(sys.argv[4]) if len(sys.argv) > 4 else 900
    out = '%s/gui/q/%s_%s' % (S, model, variant)
    env = dict(os.environ, **VARIANTS[variant])
    subprocess.run([S + '/gui/launch.sh', '%s/clitest/%s.stl' % (S, model), out + '.log'], env=env, check=True)
    time.sleep(55)
    click(560, 600)
    time.sleep(1)
    click(902, 52)  # Slice plate
    t0 = time.time()
    while not sliced(screen()):
        if time.time() - t0 > max_wait:
            sys.exit('%s %s: not sliced after %d s' % (model, variant, max_wait))
        time.sleep(5)
    print('%s %s sliced after %.0f s' % (model, variant, time.time() - t0), flush=True)
    time.sleep(20)
    click(560, 600)
    time.sleep(1)
    for _ in range(3):
        img = screen()
        if not legend_open(img):
            break
        icon = legend_icon(img)
        if icon:
            click(*icon)
        time.sleep(2)
    for _ in range(4):
        closers = warning_closers(screen())
        if not closers:
            break
        click(*closers[0])
        time.sleep(2)
    click(446, 83)  # collapse the sidebar
    time.sleep(2)
    img = screen()
    if legend_open(img) or warning_closers(img):
        print('warning: legend or a warning still shown', flush=True)
    for _ in range(zoom):
        x.click(800, 520, 4)
        time.sleep(0.3)
    time.sleep(2)
    x.move(1560, 960)
    time.sleep(2)
    xgui.shot(S + '/gui/fb', out + '_external.png', (OX, OY, 1200, 800))
    subprocess.run(['python3', S + '/xgui.py', 'drag', ':99', '1380', '288', '1380', '536'], check=True)  # top layer to half height
    time.sleep(3)
    x.move(1560, 960)
    time.sleep(2)
    xgui.shot(S + '/gui/fb', out + '_internal.png', (OX, OY, 1200, 800))


main()
