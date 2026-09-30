import re, sys
root = '/home/user/OrcaNP/'

# --- ToolheadClearance.cpp: use the shared calibration print helpers ---
p = root + 'src/libslic3r/NonPlanar/ToolheadClearance.cpp'
s = open(p).read()
start = s.index('// A prism: the planar convex `loop`')
end = s.index('} // namespace\n\ndouble ToolheadClearance::reach()')
s = s[:start] + s[end:]
# the anonymous namespace now holds tan_deg only
s = s.replace('#include "../BoundingBox.hpp"\n', '#include "../BoundingBox.hpp"\n#include "../CalibrationPrints.hpp"\n')
s = s.replace('namespace NonPlanar {\n\nnamespace {', 'namespace NonPlanar {\n\nnamespace CP = CalibrationPrints;\n\nnamespace {')
# drop the label functions at the end
start = s.index('void append_label(indexed_triangle_set &its, const std::string &text, double x, double y, double z, double height, double depth)\n{')
end = s.index('} // namespace NonPlanar')
s = s[:start] + s[end:]
for a, b in [('append_extrusion(', 'CP::append_prism('), ('append_box(', 'CP::append_box('), ('lay_out(', 'CP::lay_out('),
             ('format_value(', 'CP::format_value('), ('append_label(', 'CP::append_label('), ('label_width(', 'CP::label_width(')]:
    s = re.sub(r'(?<![\w:])' + re.escape(a), b, s)
open(p, 'w').write(s)

# --- ToolheadClearance.hpp: the labels are CalibrationPrints' now ---
p = root + 'src/libslic3r/NonPlanar/ToolheadClearance.hpp'
s = open(p).read()
start = s.index("// Raised seven-segment digits (0-9 and '.')")
end = s.index('} // namespace NonPlanar')
s = s[:start] + s[end:]
open(p, 'w').write(s)

# --- the label tests moved to test_calibration_prints.cpp ---
p = root + 'tests/libslic3r/test_toolhead_clearance.cpp'
s = open(p).read()
start = s.index('TEST_CASE("A label is as wide as label_width says"')
s = s[:start].rstrip() + '\n'
open(p, 'w').write(s)
print('ok')
