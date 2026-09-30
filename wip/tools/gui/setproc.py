import json, sys
p = sys.argv[1]; name = sys.argv[2]
c = json.load(open(p))
for e in c.get('orca_presets', []):
    if e.get('machine', '').startswith('ThetaFirm'):
        e['process'] = name
c['app']['preview_polar_turn_bed'] = True
json.dump(c, open(p, 'w'), indent=4)
print([(e.get('machine'), e.get('process')) for e in c.get('orca_presets', [])])
