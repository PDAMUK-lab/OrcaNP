#!/usr/bin/env python3
"""Flatten system presets (inherits + include) into CLI-loadable JSON."""
import json, os, sys
R = '/home/user/OrcaNP/resources/profiles'
META = {'inherits', 'include', 'instantiation', 'setting_id', 'filament_id', 'from', 'compatible_printers',
        'compatible_printers_condition', 'compatible_prints', 'compatible_prints_condition', 'renamed_from', 'upward_compatible_machine'}
_index = {}
def index(vendor):
    if vendor not in _index:
        j = json.load(open(f'{R}/{vendor}.json'))
        m = {}
        for lst in ('machine_list', 'process_list', 'filament_list'):
            for e in j.get(lst, []):
                m[e['name']] = f'{R}/{vendor}/{e["sub_path"]}'
        _index[vendor] = m
    return _index[vendor]
def find(vendor, name):
    for v in (vendor, 'OrcaFilamentLibrary'):
        p = index(v).get(name)
        if p: return v, p
    raise KeyError(f'{name} not in {vendor}')
def load(vendor, name):
    v, p = find(vendor, name)
    own = json.load(open(p))
    cfg = load(v, own['inherits']) if own.get('inherits') else {}
    for inc in own.get('include', []):
        cfg.update({k: x for k, x in load(v, inc).items() if k not in META and k not in ('name', 'type')})
    cfg.update(own)
    return cfg
def flat(vendor, name, out, overrides=None):
    cfg = load(vendor, name)
    for k in META: cfg.pop(k, None)
    cfg['from'] = 'User'
    cfg['name'] = name + ' (flat)'
    if overrides: cfg.update(overrides)
    json.dump(cfg, open(out, 'w'), indent=1)
    return cfg
if __name__ == '__main__':
    c = flat(sys.argv[1], sys.argv[2], sys.argv[3])
    print(len(c), 'keys', c.get('type'))
