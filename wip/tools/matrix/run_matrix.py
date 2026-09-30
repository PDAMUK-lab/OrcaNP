#!/usr/bin/env python3
"""run_matrix.py [filter]: slice printers x models x settings with the CLI; results in results.jsonl."""
import json, os, sys, subprocess, time, shutil, itertools
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.dirname(__file__))
from flatten import load, META
from check import check
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad'
M = f'{S}/matrix'; T = f'{S}/clitest'; B = '/home/user/OrcaNP/build/src/Release/orca-slicer'
OUT = f'{M}/out'; os.makedirs(OUT, exist_ok=True)
USER = f'{S}/gui/datadir/user/default/machine'
# Toolhead clearances are assumed except on the Core R-Theta (its profile) and the Voron test values.
ASSUMED_BBL = {'nonplanar_nozzle_clearance_angle': '20', 'nonplanar_nozzle_length': '4.2', 'nonplanar_head_radius': '30'}
PRINTERS = {
    'theta':   ('Custom', 'ThetaFirm Core R-Theta 0.4 nozzle', {}),
    'thetaxy': ('user', 'Theta as CoreXY', {}),
    'polar':   ('Custom', 'Generic Polar Printer 0.4 nozzle', {}),
    'klipper': ('Custom', 'MyKlipper 0.4 nozzle', {'nonplanar_nozzle_clearance_angle': '30', 'nonplanar_nozzle_length': '3', 'nonplanar_head_radius': '25'}),
    'voron':   ('Voron', 'Voron 2.4 300 0.4 nozzle', {'nonplanar_nozzle_clearance_angle': '2.2', 'nonplanar_head_radius': '32.75', 'nonplanar_nozzle_length': '1.24'}),
    'voron06': ('Voron', 'Voron 2.4 300 0.6 nozzle', {'nonplanar_nozzle_clearance_angle': '2.2', 'nonplanar_head_radius': '32.75', 'nonplanar_nozzle_length': '1.24'}),
    'x1c':     ('BBL', 'Bambu Lab X1 Carbon 0.4 nozzle', ASSUMED_BBL),
    'x1c02':   ('BBL', 'Bambu Lab X1 Carbon 0.2 nozzle', ASSUMED_BBL),
    'x1c06':   ('BBL', 'Bambu Lab X1 Carbon 0.6 nozzle', ASSUMED_BBL),
    'x1c08':   ('BBL', 'Bambu Lab X1 Carbon 0.8 nozzle', ASSUMED_BBL),
    'a1':      ('BBL', 'Bambu Lab A1 0.4 nozzle', ASSUMED_BBL),
    'h2d':     ('BBL', 'Bambu Lab H2D 0.4 nozzle', dict(ASSUMED_BBL, nonplanar_head_radius='35')),
}
def presets(key):
    vendor, name, over = PRINTERS[key]
    if vendor == 'user':
        own = json.load(open(f'{USER}/{name}.json')); v = 'Custom' if 'ThetaFirm' in own.get('inherits', '') else 'Voron'
        m = load(v, own['inherits']); m.update(own); vendor = v
    else:
        m = load(vendor, name)
    m.update(over)
    p = load(vendor, m['default_print_profile'])
    f = load(vendor, m['default_filament_profile'][0])
    return vendor, m, p, f
MAIN = ['3DBenchy.stl', 'cube.obj']
S4M = ['pi.stl', '3DBenchy.stl', 'bracket.stl']
PLANAR = {  # name: (overrides, models)
    'base': ({}, MAIN), 'arachne': ({'wall_generator': 'arachne'}, MAIN), 'classic': ({'wall_generator': 'classic'}, MAIN),
    'shrink': ({'filament_shrink': ['98%'], 'filament_shrinkage_compensation_z': ['99%']}, ['3DBenchy.stl']),
    'pa': ({'enable_pressure_advance': ['1'], 'pressure_advance': ['0.035']}, ['3DBenchy.stl']),
    'adaptive_pa': ({'enable_pressure_advance': ['1'], 'adaptive_pressure_advance': ['1']}, ['3DBenchy.stl']),
    'flow': ({'filament_flow_ratio': ['0.93'], 'print_flow_ratio': '1.05'}, ['3DBenchy.stl']),
    'supp_normal': ({'enable_support': '1', 'support_type': 'normal(auto)'}, ['3DBenchy.stl', 'bracket.stl']),
    'supp_tree': ({'enable_support': '1', 'support_type': 'tree(auto)'}, ['3DBenchy.stl', 'bracket.stl']),
    'brim': ({'brim_type': 'outer_only', 'brim_width': '8'}, ['cube.obj']),
    'skirt': ({'skirt_loops': '3', 'skirt_distance': '4'}, ['cube.obj']),
    'raft': ({'raft_layers': '3'}, ['cube.obj']),
    'ironing': ({'ironing_type': 'top'}, ['cube.obj']),
    'fuzzy': ({'fuzzy_skin': 'external'}, ['3DBenchy.stl']),
    'arc': ({'enable_arc_fitting': '1'}, ['3DBenchy.stl']),
    'zhop_spiral': ({'z_hop': ['0.4'], 'z_hop_types': ['Spiral Lift']}, ['3DBenchy.stl']),
    'seam_random': ({'seam_position': 'random'}, ['cube.obj']), 'seam_back': ({'seam_position': 'back'}, ['3DBenchy.stl']),
    'gyroid': ({'sparse_infill_pattern': 'gyroid', 'sparse_infill_density': '30%'}, ['cube.obj']),
    'lightning': ({'sparse_infill_pattern': 'lightning'}, ['3DBenchy.stl']),
    'honeycomb': ({'sparse_infill_pattern': 'honeycomb'}, ['cube.obj']),
    'adaptivecubic': ({'sparse_infill_pattern': 'adaptivecubic', 'sparse_infill_density': '20%'}, ['cube.obj']),
    'lh008': ({'layer_height': '0.08'}, ['cube.obj']), 'lh028': ({'layer_height': '0.28'}, ['3DBenchy.stl']),
    'walls5': ({'wall_loops': '5', 'top_shell_layers': '6', 'bottom_shell_layers': '5'}, ['cube.obj']),
    'precise': ({'precise_outer_wall': '1'}, ['cube.obj']),
    'vase': ({'spiral_mode': '1'}, ['cup.stl']),
    'petg': ({'@filament': 'Generic PETG @System'}, ['3DBenchy.stl']), 'tpu': ({'@filament': 'Generic TPU @System'}, ['cube.obj']),
    'sequential': ({'print_sequence': 'by object'}, ['cube.obj+cube.obj']),
}
S4BASE = {'s4_enabled': '1', 'use_relative_e_distances': '1', 'layer_change_gcode': 'G92 E0'}
S4 = {
    's4opt': ({'s4_layer_shape': 'optimized'}, S4M),
    's4opt_arachne': ({'s4_layer_shape': 'optimized', 'wall_generator': 'arachne'}, ['pi.stl']),
    's4opt_classic': ({'s4_layer_shape': 'optimized', 'wall_generator': 'classic'}, ['pi.stl']),
    's4opt_supp': ({'s4_layer_shape': 'optimized', 'enable_support': '1', 'support_type': 'normal(auto)'}, ['bracket.stl']),
    's4opt_pa': ({'s4_layer_shape': 'optimized', 'enable_pressure_advance': ['1'], 'pressure_advance': ['0.035']}, ['pi.stl']),
    's4opt_shrink': ({'s4_layer_shape': 'optimized', 'filament_shrink': ['98%'], 'filament_shrinkage_compensation_z': ['99%']}, ['pi.stl']),
    's4opt_fuzzy': ({'s4_layer_shape': 'optimized', 'fuzzy_skin': 'external'}, ['pi.stl']),
    's4opt_ironing': ({'s4_layer_shape': 'optimized', 'ironing_type': 'top'}, ['pi.stl']),
    's4opt_zhop': ({'s4_layer_shape': 'optimized', 'z_hop': ['0.4'], 'z_hop_types': ['Normal Lift']}, ['pi.stl']),
    's4cone': ({'s4_layer_shape': 'cone'}, ['pi.stl', '3DBenchy.stl']),
    's4pillar': ({'s4_layer_shape': 'offset', 's4_surface_core': 'pillar', 's4_surface_size': 'auto'}, ['3DBenchy.stl']),
    's4dome': ({'s4_layer_shape': 'offset', 's4_surface_core': 'dome', 's4_surface_size': 'auto'}, ['lens.stl']),
    's4cavity': ({'s4_layer_shape': 'offset', 's4_surface_core': 'cavity'}, ['hollow_dome.stl']),
    's4vase': ({'s4_layer_shape': 'optimized', 'spiral_mode': '1'}, ['cup.stl']),
    's4absE': ({'s4_layer_shape': 'optimized', 'use_relative_e_distances': '0'}, ['pi.stl']),
}
S4PRINTERS = ['theta', 'polar', 'x1c', 'voron', 'klipper', 'h2d', 'a1']
PLANAR_PRINTERS = ['theta', 'thetaxy', 'polar', 'klipper', 'voron', 'x1c', 'a1', 'h2d']
NOZZLE_CASES = [(p, v) for p in ['x1c02', 'x1c06', 'x1c08', 'voron06'] for v in ['base', 'arachne', 'supp_tree', 'vase']] + \
               [(p, 's4opt') for p in ['x1c06', 'voron06']]
def cases():
    for p in PLANAR_PRINTERS:
        for v, (o, models) in PLANAR.items():
            for mdl in models: yield p, v, dict(o, **({'s4_enabled': '0'} if p == 'theta' else {})), mdl
    for p in S4PRINTERS:
        for v, (o, models) in S4.items():
            for mdl in models: yield p, v, dict(S4BASE, **o), mdl
    for p, v in NOZZLE_CASES:
        o, models = (S4[v] if v in S4 else PLANAR[v])
        for mdl in models[:1]: yield p, v, (dict(S4BASE, **o) if v in S4 else o), mdl
def run(case):
    p, v, over, mdl = case
    name = f'{p}__{v}__{mdl.replace("+", "_")}'
    d = f'{OUT}/{name}'
    if os.path.exists(f'{d}/done.json'): return json.load(open(f'{d}/done.json'))
    shutil.rmtree(d, ignore_errors=True); os.makedirs(d)
    vendor, m, pr, f = presets(p)
    over = dict(over)
    if '@filament' in over: f = load('OrcaFilamentLibrary' if vendor != 'BBL' else vendor, over.pop('@filament'))
    for k, val in over.items():
        tgt = m if k in m else f if k in f else pr
        if isinstance(tgt.get(k), list) and not isinstance(val, list): val = [val]
        if not isinstance(tgt.get(k), list) and isinstance(val, list) and k in tgt: val = val[0]
        tgt[k] = val
    for c, n in ((m, 'm'), (pr, 'p'), (f, 'f')):
        for k in META: c.pop(k, None)
        c['from'] = 'User'; c['name'] = f'{name} {n}'
        json.dump(c, open(f'{d}/{n}.json', 'w'), indent=1)
    models = ' '.join(f'{T}/{x}' for x in mdl.split('+'))
    t0 = time.time()
    rc = subprocess.run(f'timeout 1200 {B} --datadir {T}/datadir --load-settings "{d}/m.json;{d}/p.json" --load-filaments "{d}/f.json" '
                        f'{"--arrange 1 " if "+" in mdl else ""}--slice 0 --outputdir {d} {models} > {d}/log 2>&1', shell=True).returncode
    res = dict(case=name, printer=p, variant=v, model=mdl, exit=rc, secs=round(time.time() - t0))
    try:
        rj = json.load(open(f'{d}/result.json')); res['rc'] = rj.get('return_code'); res['error'] = rj.get('error_string')
        res['warn'] = ' | '.join(pl.get('warning_message', '') for pl in rj.get('sliced_plates', []) if pl.get('warning_message'))[:600]
    except Exception as e: res['rc'] = None
    g = f'{d}/plate_1.gcode'
    if os.path.exists(g):
        try: res.update(check(g, m))
        except Exception as e: res['check_error'] = repr(e)
        os.remove(g) if os.path.getsize(g) > 30e6 else None
    json.dump(res, open(f'{d}/done.json', 'w'))
    return res
if __name__ == '__main__':
    flt = sys.argv[1] if len(sys.argv) > 1 else ''
    cs = [c for c in cases() if flt in f'{c[0]}__{c[1]}__{c[3]}']
    print(len(cs), 'cases', flush=True)
    with ThreadPoolExecutor(3) as ex, open(f'{M}/results.jsonl', 'a') as out:
        for r in ex.map(run, cs):
            out.write(json.dumps(r) + '\n'); out.flush()
            bad = r.get('exit') or r.get('rc') or r.get('nan') or r.get('out_xy') or r.get('out_z') or r.get('below_bed') or r.get('low_print') or r.get('wipe_rise') or not r.get('print_moves')
            print(('FLAG ' if bad else 'ok   ') + r['case'], r.get('secs'), r.get('time'), (r.get('error') or '')[:80], flush=True)
