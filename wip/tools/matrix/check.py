#!/usr/bin/env python3
"""check.py GCODE MACHINE_JSON: sanity metrics of a sliced G-code (Cartesian or polar machine G-code)."""
import sys, json, math, re
WORD = re.compile(r'([A-Z])([-+]?(?:[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?|nan|inf))', re.I)
def check(gpath, mcfg):
    m = mcfg
    first = lambda v: (v[0] if isinstance(v, list) else v)
    polar = str(m.get('polar_kinematics', '0')) == '1'
    area = [tuple(map(float, p.split('x'))) for p in m.get('printable_area', ['0x0', '200x200'])]
    xs, ys = [p[0] for p in area], [p[1] for p in area]
    height = float(first(m.get('printable_height', 250)))
    pivot = float(first(m.get('polar_tilt_pivot_length', 0) or 0)); tsign = -1. if str(m.get('polar_reverse_tilt', '0')) == '1' else 1.
    ax = (m.get('polar_axis_names') or 'CXB')
    r = dict(nan=0, e_pos=0., moves=0, print_moves=0, out_xy=0, out_z=0, low_print=0, below_bed=0, layers=0, wipe_rise=0)
    pos = {k: 0. for k in 'XYZCB'}; absxyz = True; rel_e = True; started = False; seen = set()
    lo = [math.inf] * 3; hi = [-math.inf] * 3
    wipe_z0 = None; wiping = False
    tagged = 'MACHINE_START_GCODE_END' in open(gpath, errors='replace').read()
    for raw in open(gpath, errors='replace'):
        if 'MACHINE_START_GCODE_END' in raw: started = True; continue
        if 'MACHINE_END_GCODE_START' in raw or raw.startswith('; filament end gcode') or 'EXECUTABLE_BLOCK_END' in raw: break
        if raw.startswith(';LAYER_CHANGE') or raw.startswith('; CHANGE_LAYER'):
            r['layers'] += 1
            started = started or not tagged
        if raw.startswith(';WIPE_START'): wiping = True; wipe_z0 = None
        if raw.startswith(';WIPE_END'): wiping = False
        if not started: continue
        b = raw.split(';', 1)[0].strip()
        if not b: continue
        cmd = b.split()[0].upper()
        if cmd == 'G90': absxyz = True
        elif cmd == 'G91': absxyz = False
        elif cmd == 'M82': rel_e = False
        elif cmd == 'M83': rel_e = True
        if cmd not in ('G0', 'G1', 'G2', 'G3'): continue
        w = {}
        for k, v in WORD.findall(b[len(cmd):]):
            if v.lower() in ('nan', 'inf', '-inf', '+inf'): r['nan'] += 1; continue
            w[k.upper()] = float(v)
        for k in 'XYZCB':
            if k in w:
                pos[k] = w[k] if absxyz else pos[k] + w[k]; seen.add(k)
        e = w.get('E', 0.)
        r['moves'] += 1
        if polar:
            if not {'C', 'X'} <= seen: continue
            t = math.radians(tsign * pos['B']); rad = pos['X'] - pivot * math.sin(t); z = pos['Z'] - pivot * (math.cos(t) - 1.)
            a = math.radians(pos['C']); tip = (rad * math.cos(a), rad * math.sin(a), z)
        else:
            if not {'X', 'Y'} <= seen: continue
            tip = (pos['X'], pos['Y'], pos['Z'])
        printing = e > 0 and rel_e
        if wiping and 'Z' in seen:
            wipe_z0 = tip[2] if wipe_z0 is None else wipe_z0
            if tip[2] - wipe_z0 > 3: r['wipe_rise'] += 1; wipe_z0 = tip[2]
        if 'Z' in seen and tip[2] < -0.01: r['below_bed'] += 1
        if printing:
            r['e_pos'] += e; r['print_moves'] += 1
            for i in range(3): lo[i] = min(lo[i], tip[i]); hi[i] = max(hi[i], tip[i])
            if polar:
                rr = max(abs(x) for x in xs + ys)
                if math.hypot(tip[0], tip[1]) > rr + 2: r['out_xy'] += 1
            elif not (min(xs) - 2 <= tip[0] <= max(xs) + 2 and min(ys) - 2 <= tip[1] <= max(ys) + 2): r['out_xy'] += 1
            if tip[2] > height + 1: r['out_z'] += 1
            if tip[2] < 0.05: r['low_print'] += 1
    r['bbox'] = [round(x, 1) for x in lo + hi] if r['print_moves'] else None
    txt = open(gpath, errors='replace').read(60000)
    mt = re.search(r'model printing time: ([^;\n]+)', txt); r['time'] = mt.group(1).strip() if mt else None
    return r
if __name__ == '__main__':
    print(check(sys.argv[1], json.load(open(sys.argv[2]))))
