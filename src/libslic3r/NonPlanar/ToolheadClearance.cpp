#include "ToolheadClearance.hpp"

#include "../BoundingBox.hpp"
#include "../I18N.hpp"
#include "../format.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r {
namespace NonPlanar {

namespace {

// 89.9 degrees: the clearance angle is at least 0.1 degree and less than 90, as the setting allows.
double tan_deg(double angle) { return std::tan(std::clamp(angle, 0.1, 89.9) * M_PI / 180.); }

// A prism: the planar convex `loop` and the same loop moved by `extrude`, closed, facing out.
void append_extrusion(indexed_triangle_set &its, std::vector<Vec3d> loop, const Vec3d &extrude)
{
    // Faces out when the loop turns clockwise seen along the extrusion.
    Vec3d normal = Vec3d::Zero();
    for (size_t i = 0; i < loop.size(); ++ i)
        normal += loop[i].cross(loop[(i + 1) % loop.size()]);
    if (normal.dot(extrude) > 0.)
        std::reverse(loop.begin(), loop.end());
    const int n    = int(loop.size());
    const int base = int(its.vertices.size());
    for (const Vec3d &p : loop)
        its.vertices.emplace_back(p.cast<float>());
    for (const Vec3d &p : loop)
        its.vertices.emplace_back((p + extrude).cast<float>());
    for (int i = 1; i + 1 < n; ++ i) {
        its.indices.emplace_back(base, base + i, base + i + 1);             // start cap
        its.indices.emplace_back(base + n, base + n + i + 1, base + n + i); // end cap
    }
    for (int i = 0; i < n; ++ i) {
        const int j = (i + 1) % n;
        its.indices.emplace_back(base + i, base + n + j, base + j);
        its.indices.emplace_back(base + i, base + n + i, base + n + j);
    }
}

void append_box(indexed_triangle_set &its, double x0, double y0, double z0, double x1, double y1, double z1)
{
    append_extrusion(its, { { x0, y0, z0 }, { x1, y0, z0 }, { x1, y1, z0 }, { x0, y1, z0 } }, Vec3d(0., 0., z1 - z0));
}

// Seven segments, a (top) to g (middle), lit for each digit.
constexpr const char *SEGMENTS[10] = { "abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg" };

double stroke(double height) { return std::max(0.8, 0.16 * height); }
double digit_width(double height) { return 0.6 * height; }
double advance(char c, double height) { return (c == '.' ? stroke(height) : digit_width(height)) + stroke(height); }

// Pieces laid out in rows no wider than `max_width`, left to right, then centred on the origin.
indexed_triangle_set lay_out(const std::vector<indexed_triangle_set> &pieces, double max_width, double gap)
{
    indexed_triangle_set out;
    double x = 0., y = 0., row_depth = 0.;
    for (const indexed_triangle_set &piece : pieces) {
        BoundingBoxf3 box;
        for (const Vec3f &v : piece.vertices)
            box.merge(v.cast<double>());
        const Vec3d size = box.size();
        if (x > 0. && x + size.x() > max_width) {
            x = 0.;
            y += row_depth + gap;
            row_depth = 0.;
        }
        indexed_triangle_set placed = piece;
        const Vec3f shift = Vec3d(x - box.min.x(), y - box.min.y(), -box.min.z()).cast<float>();
        for (Vec3f &v : placed.vertices)
            v += shift;
        its_merge(out, placed);
        x += size.x() + gap;
        row_depth = std::max(row_depth, size.y());
    }
    BoundingBoxf3 box;
    for (const Vec3f &v : out.vertices)
        box.merge(v.cast<double>());
    const Vec3f centre = Vec3d(box.center().x(), box.center().y(), 0.).cast<float>();
    for (Vec3f &v : out.vertices)
        v -= centre;
    return out;
}

std::string format_value(double value, int decimals)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    return buf;
}

} // namespace

double ToolheadClearance::reach() const
{
    return std::max(radius, 0.5 * tip_diameter + std::max(length, 0.) / tan_deg(angle));
}

double ToolheadClearance::underside(double distance) const
{
    const double r0 = 0.5 * tip_diameter;
    if (distance > reach())
        return std::numeric_limits<double>::infinity();
    return distance <= r0 ? 0. : std::min((distance - r0) * tan_deg(angle), std::max(length, 0.));
}

namespace {

// The cone out to the reach, from the tip's edge at `angle` rounded down to 0.1 degree: its length
// rounded down, the reach up, both erring on the side of the toolhead.
ToolheadClearance cone_to_reach(double angle, double tip_diameter, double reach)
{
    ToolheadClearance t;
    t.tip_diameter = tip_diameter;
    t.angle        = std::max(0.1, std::floor(angle * 10. + 1e-9) / 10.);
    t.radius       = std::ceil(reach * 10. - 1e-9) / 10.;
    t.length       = std::floor((t.radius - 0.5 * tip_diameter) * tan_deg(t.angle) * 100.) / 100.;
    return t;
}

} // namespace

bool fit_feeler_readings(const std::vector<FeelerReading> &readings, double tip_diameter, double reach,
                         ToolheadClearance &out, std::string &error)
{
    const double r0 = 0.5 * tip_diameter;
    if (readings.empty()) {
        error = _u8L("No side was measured.");
        return false;
    }
    double angle = 90., farthest = reach;
    for (const FeelerReading &r : readings) {
        if (! (r.gap > 0.) || ! (r.distance > r0 + 1.)) {
            error = format(_u8L("Each gap must be more than 0 and each distance more than %1% mm, 1 mm past the nozzle tip's edge."),
                           format_value(r0 + 1., 1));
            return false;
        }
        angle    = std::min(angle, std::atan2(r.gap, r.distance - r0) * 180. / M_PI);
        farthest = std::max(farthest, r.distance);
    }
    out = cone_to_reach(angle, tip_diameter, farthest);
    return true;
}

bool fit_wedge_readings(const std::vector<double> &angles, double tip_diameter, double reach, ToolheadClearance &out,
                        std::string &error)
{
    if (angles.empty()) {
        error = _u8L("No side was measured.");
        return false;
    }
    if (! (reach > 0.5 * tip_diameter + 1.)) {
        error = format(_u8L("The toolhead reach must be more than %1% mm, 1 mm past the nozzle tip's edge."),
                       format_value(0.5 * tip_diameter + 1., 1));
        return false;
    }
    double angle = 90.;
    for (double a : angles) {
        if (! (a > 0.) || ! (a < 90.)) {
            error = _u8L("Each wedge angle must be more than 0 and less than 90 degrees.");
            return false;
        }
        angle = std::min(angle, a);
    }
    out = cone_to_reach(angle, tip_diameter, reach);
    return true;
}

std::vector<double> feeler_thicknesses() { return { 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.4, 2.8, 3.2, 4.0, 5.0, 6.0 }; }

indexed_triangle_set feeler_blades(const std::vector<double> &thicknesses, double max_width)
{
    // A 25 x 10 mm blade and a 16 x 14 mm handle numbered with the thickness.
    constexpr double blade_length = 25., blade_width = 10., handle_length = 16., handle_width = 14., digits = 5., relief = 0.6;
    std::vector<indexed_triangle_set> pieces;
    for (double t : thicknesses) {
        indexed_triangle_set piece;
        append_box(piece, 0., 0., 0., blade_length, blade_width, t);
        const double handle = std::max(t, 1.2), side = 0.5 * (handle_width - blade_width);
        append_box(piece, blade_length, -side, 0., blade_length + handle_length, blade_width + side, handle);
        const std::string text = format_value(t, 1);
        append_label(piece, text, blade_length + 0.5 * (handle_length - label_width(text, digits)), 0.5 * (blade_width - digits), handle,
                     digits, relief);
        pieces.emplace_back(std::move(piece));
    }
    return lay_out(pieces, max_width, 5.);
}

std::vector<double> wedge_angles()
{
    std::vector<double> out;
    for (int a = 5; a <= 75; a += 5)
        out.emplace_back(a);
    return out;
}

indexed_triangle_set angle_wedges(const std::vector<double> &angles, double reach, double max_width)
{
    // A slope 16 mm wide, from its thin edge out past the reach or up to 35 mm, and a 14 x 10 mm tab
    // behind it numbered with the angle.
    constexpr double width = 16., max_height = 35., tab_length = 14., tab_width = 10., tab = 1.2, digits = 5., relief = 0.6;
    std::vector<indexed_triangle_set> pieces;
    for (double a : angles) {
        const double k = tan_deg(a);
        const double length = std::max(6., std::min(reach + 3., max_height / k));
        indexed_triangle_set piece;
        append_extrusion(piece, { { 0., 0., 0. }, { length, 0., 0. }, { length, 0., length * k } }, Vec3d(0., width, 0.));
        const double y0 = 0.5 * (width - tab_width);
        append_box(piece, length, y0, 0., length + tab_length, y0 + tab_width, tab);
        const std::string text = format_value(a, 0);
        append_label(piece, text, length + 0.5 * (tab_length - label_width(text, digits)), y0 + 0.5 * (tab_width - digits), tab, digits,
                     relief);
        pieces.emplace_back(std::move(piece));
    }
    return lay_out(pieces, max_width, 5.);
}

indexed_triangle_set fin_test(const ToolheadClearance &t, double fin_thickness)
{
    const double r0 = 0.5 * t.tip_diameter, k = tan_deg(t.angle), length = std::max(t.length, 0.);
    const double reach = t.reach(), end = reach + 6., T = fin_test_plate, m = fin_test_margin;
    // Tops at least 0.4 mm above the plate, two layers.
    constexpr double min_fin = 0.4;
    const double top_out  = std::min(length + 3., 30.);
    const double d_start  = std::max(r0 + 1.5, r0 + (m + min_fin) / k);
    const double d_cone   = std::min(r0 + length / k, reach);
    const double flat_top = length - m;

    indexed_triangle_set its;
    const int plate_segments = 96;
    std::vector<Vec3d> disc;
    for (int i = 0; i < plate_segments; ++ i) {
        const double a = 2. * M_PI * i / plate_segments;
        disc.emplace_back((end + 3.) * std::cos(a), (end + 3.) * std::sin(a), 0.);
    }
    append_extrusion(its, disc, Vec3d(0., 0., T));

    for (int f = 0; f < 12; ++ f) {
        const double a = 2. * M_PI * f / 12.;
        const Vec3d  u(std::cos(a), std::sin(a), 0.), n(-std::sin(a), std::cos(a), 0.);
        // A piece of fin from d0 to d1 whose top rises from h0 to h1 above the plate.
        auto piece = [&](double d0, double d1, double h0, double h1) {
            if (d1 - d0 < 0.1 || std::max(h0, h1) < min_fin)
                return;
            auto at = [&](double d, double z) { return Vec3d(d * u + Vec3d(0., 0., z) - 0.5 * fin_thickness * n); };
            append_extrusion(its, { at(d0, T), at(d1, T), at(d1, T + h1), at(d0, T + h0) }, fin_thickness * n);
        };
        if (d_start < d_cone)
            piece(d_start, d_cone, (d_start - r0) * k - m, (d_cone - r0) * k - m);
        piece(std::max(d_start, d_cone), reach + 1., flat_top, flat_top);
        piece(reach + 1., end, top_out, top_out);
    }
    return its;
}

void append_label(indexed_triangle_set &its, const std::string &text, double x, double y, double z, double height, double depth)
{
    const double s = stroke(height), w = digit_width(height), h = height, half = 0.5 * height;
    for (char c : text) {
        if (c == '.')
            append_box(its, x, y, z, x + s, y + s, z + depth);
        else if (c >= '0' && c <= '9')
            for (const char *seg = SEGMENTS[c - '0']; *seg; ++ seg)
                switch (*seg) {
                case 'a': append_box(its, x, y + h - s, z, x + w, y + h, z + depth); break;
                case 'b': append_box(its, x + w - s, y + half, z, x + w, y + h, z + depth); break;
                case 'c': append_box(its, x + w - s, y, z, x + w, y + half, z + depth); break;
                case 'd': append_box(its, x, y, z, x + w, y + s, z + depth); break;
                case 'e': append_box(its, x, y, z, x + s, y + half, z + depth); break;
                case 'f': append_box(its, x, y + half, z, x + s, y + h, z + depth); break;
                case 'g': append_box(its, x, y + half - 0.5 * s, z, x + w, y + half + 0.5 * s, z + depth); break;
                }
        x += advance(c, height);
    }
}

double label_width(const std::string &text, double height)
{
    double width = 0.;
    for (char c : text)
        width += advance(c, height);
    return text.empty() ? 0. : width - stroke(height);
}

} // namespace NonPlanar
} // namespace Slic3r
