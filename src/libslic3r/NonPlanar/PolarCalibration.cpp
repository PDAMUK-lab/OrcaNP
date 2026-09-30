#include "PolarCalibration.hpp"

#include "../CalibrationPrints.hpp"
#include "../ExPolygon.hpp"
#include "../I18N.hpp"
#include "../format.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r {
namespace NonPlanar {

namespace CP = CalibrationPrints;

namespace {

double deg2rad(double a) { return a * M_PI / 180.; }
double rad2deg(double a) { return a * 180. / M_PI; }

// A point `r` from the rotation axis at `angle` (radians) and height `z`.
Vec3d polar(const TestParams &p, double r, double angle, double z)
{
    return { p.centre.x() + r * std::cos(angle), p.centre.y() + r * std::sin(angle), z };
}

// Layer heights from the first layer up to `height`.
std::vector<double> layer_tops(const TestParams &p, double height)
{
    std::vector<double> out { p.first_layer_height };
    while (out.back() + p.layer_height < height + 1e-6)
        out.emplace_back(out.back() + p.layer_height);
    return out;
}

// Appends moves, keeping the tilt of the move before unless one is given.
struct PathWriter
{
    TestPrint &print;
    double     tilt  = 0.;
    double     width = 0.45;

    void layer(double z, double height) { print.layers.push_back({ z, height, print.moves.size() }); }
    void travel(const Vec3d &to, double speed) { print.moves.push_back({ to, false, 0., speed, tilt }); }
    void extrude(const Vec3d &to, double speed) { print.moves.push_back({ to, true, width, speed, tilt }); }
    Vec3d last() const { return print.moves.empty() ? Vec3d::Zero() : print.moves.back().to; }
    // Up by `hop` where the nozzle is, over to `to` (tilting there first), and down onto it.
    void go_to(const Vec3d &to, double hop, double speed, double new_tilt)
    {
        if (! print.moves.empty())
            travel(last() + Vec3d(0., 0., hop), speed);
        tilt = new_tilt;
        travel(to + Vec3d(0., 0., hop), speed);
        travel(to, speed);
    }
};

// A seven-segment digit drawn in lines on the layer at `z`: `origin` its bottom left, `u` along
// it and `v` up it, `height` tall.
void draw_digit(PathWriter &w, const TestParams &p, char digit, const Vec2d &origin, const Vec2d &u, const Vec2d &v, double height, double z)
{
    static constexpr const char *SEGMENTS[10] = { "abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg" };
    if (digit < '0' || digit > '9')
        return;
    const double wd = 0.6 * height, h = height, half = 0.5 * height;
    auto at = [&](double x, double y) { const Vec2d q = origin + x * u + y * v; return Vec3d(q.x(), q.y(), z); };
    for (const char *seg = SEGMENTS[digit - '0']; *seg; ++ seg) {
        Vec3d a, b;
        switch (*seg) {
        case 'a': a = at(0., h), b = at(wd, h); break;
        case 'b': a = at(wd, h), b = at(wd, half); break;
        case 'c': a = at(wd, half), b = at(wd, 0.); break;
        case 'd': a = at(wd, 0.), b = at(0., 0.); break;
        case 'e': a = at(0., 0.), b = at(0., half); break;
        case 'f': a = at(0., half), b = at(0., h); break;
        default: a = at(0., half), b = at(wd, half); break;
        }
        w.go_to(a, p.hop, p.travel_speed, w.tilt);
        w.extrude(b, p.first_layer_speed);
    }
}

// A loop of `radius` about the axis at `z`, from angle 0 counter-clockwise, in segments of at
// most a millimetre.
void draw_loop(PathWriter &w, const TestParams &p, double radius, double z, double speed)
{
    const int n = std::max(36, int(std::ceil(2. * M_PI * radius)));
    for (int i = 1; i <= n; ++ i)
        w.extrude(polar(p, radius, 2. * M_PI * i / n, z), speed);
}

} // namespace

// --- Bed level ------------------------------------------------------------------------------

BedLevelRings bed_level_rings(double bed_radius, double thickness)
{
    BedLevelRings r;
    r.outer_radius = 0.8 * bed_radius;
    r.inner_radius = 0.35 * bed_radius;
    r.thickness    = thickness;
    return r;
}

std::vector<Vec2d> bed_level_marks(const BedLevelRings &rings)
{
    std::vector<Vec2d> out;
    for (double r : { rings.outer_radius, rings.inner_radius })
        for (int k = 0; k < 4; ++ k)
            out.emplace_back(r * std::cos(k * M_PI / 2.), r * std::sin(k * M_PI / 2.));
    return out;
}

indexed_triangle_set bed_level_mesh(const BedLevelRings &rings)
{
    indexed_triangle_set its;
    for (double r : { rings.outer_radius, rings.inner_radius }) {
        auto ring = [](double radius) {
            Polygon out;
            for (const Vec3d &q : CP::circle(Vec3d::Zero(), radius, 360))
                out.points.emplace_back(scaled(q.x()), scaled(q.y()));
            return out;
        };
        ExPolygon annulus(ring(r + 0.5 * rings.width));
        Polygon   hole = ring(r - 0.5 * rings.width);
        hole.reverse();
        annulus.holes.emplace_back(std::move(hole));
        CP::append_slab(its, annulus, 0., rings.thickness);
    }
    // The numbers, standing on the bed beside the marks: outside the outer ring, inside the inner.
    constexpr double digit = 5.;
    const std::vector<Vec2d> marks = bed_level_marks(rings);
    for (size_t i = 0; i < marks.size(); ++ i) {
        const bool   outer  = i < 4;
        const double r      = marks[i].norm();
        const double offset = 0.5 * rings.width + 1. + 0.5 * digit;
        const Vec2d  at     = marks[i] / r * (outer ? r + offset : r - offset);
        const std::string text = std::to_string(i + 1);
        CP::append_label(its, text, at.x() - 0.5 * CP::label_width(text, digit), at.y() - 0.5 * digit, 0., digit, rings.thickness + 0.4);
    }
    return its;
}

bool fit_bed_level(const BedLevelRings &rings, const std::array<double, 8> &thickness, BedLevelFit &out, std::string &error)
{
    for (double t : thickness)
        if (! (t > 0.3 * rings.thickness && t < 3. * rings.thickness)) {
            error = format(_u8L("Each ring should measure about %1% mm thick: please check the measurements."),
                           CP::format_value(rings.thickness, 2));
            return false;
        }
    // thickness = t0 - (bed height above the axis's), the height a x + b y + c r.
    const std::vector<Vec2d>        marks = bed_level_marks(rings);
    Eigen::Matrix<double, 8, 4>     A;
    Eigen::Matrix<double, 8, 1>     t;
    for (int i = 0; i < 8; ++ i) {
        A.row(i) << 1., -marks[i].x(), -marks[i].y(), -marks[i].norm();
        t(i) = thickness[i];
    }
    const Eigen::Vector4d x = A.colPivHouseholderQr().solve(t);
    out.tilt_x = x(1);
    out.tilt_y = x(2);
    out.cone   = x(3);
    return true;
}

// --- Backlash -------------------------------------------------------------------------------

BacklashTest backlash_test(const TestParams &p)
{
    BacklashTest t;
    t.outer_radius = std::max(20., std::min(0.8 * p.bed_radius, p.bed_radius - 8.));
    t.inner_radius = t.outer_radius - std::min(16., 0.5 * t.outer_radius);
    return t;
}

TestPrint backlash_print(const TestParams &p, const BacklashTest &test)
{
    TestPrint  out;
    PathWriter w { out };
    w.width = p.line_width;
    const double ri = test.inner_radius, ro = test.outer_radius, lw = p.line_width;
    // The pairs' middles, and each fin's two lines at the outer radius: the lower-angle fin's
    // lines, then the higher's.
    const double pair_angles[2] = { deg2rad(40.), deg2rad(-40.) };
    const double approach       = deg2rad(8.);
    auto fins = [&](double middle) {
        const double k = 1. / ro;
        return std::array<double, 4> { middle - (0.5 * test.gap + 1.5 * lw) * k, middle - (0.5 * test.gap + 0.5 * lw) * k,
                                       middle + (0.5 * test.gap + 0.5 * lw) * k, middle + (0.5 * test.gap + 1.5 * lw) * k };
    };
    // A fin of two radial lines, the bed turning the way of `dir` to the first and on to the second.
    auto fin = [&](double first, double second, int dir, double z, double speed) {
        const Vec3d up(0., 0., p.hop);
        if (! out.moves.empty())
            w.travel(w.last() + up, p.travel_speed);
        w.travel(polar(p, ri, first - dir * approach, z) + up, p.travel_speed);
        w.travel(polar(p, ri, first, z) + up, p.travel_speed);
        w.travel(polar(p, ri, first, z), p.travel_speed);
        w.extrude(polar(p, ro, first, z), speed);
        w.extrude(polar(p, ro, second, z), speed);
        w.extrude(polar(p, ri, second, z), speed);
    };
    const std::vector<double> tops = layer_tops(p, test.height);
    for (size_t l = 0; l < tops.size(); ++ l) {
        const double z = tops[l], speed = l == 0 ? p.first_layer_speed : p.print_speed;
        w.layer(z, l == 0 ? p.first_layer_height : p.layer_height);
        if (l == 0)
            for (int i = 0; i < 2; ++ i) {
                // The pair's number inside it, reading outward.
                constexpr double digit = 5.;
                const Vec2d      out_dir(std::cos(pair_angles[i]), std::sin(pair_angles[i])), along(-out_dir.y(), out_dir.x());
                const Vec2d      origin = p.centre + (ri - 3. - digit) * out_dir - 0.3 * digit * along;
                draw_digit(w, p, char('1' + i), origin, along, out_dir, digit, z);
            }
        for (int i = 0; i < 2; ++ i) {
            const std::array<double, 4> a = fins(pair_angles[i]);
            fin(a[0], a[1], 1, z, speed);
            // Pair 1: the bed turns up to the second fin as to the first; pair 2: down to it.
            if (i == 0)
                fin(a[2], a[3], 1, z, speed);
            else
                fin(a[3], a[2], -1, z, speed);
        }
    }
    w.travel(w.last() + Vec3d(0., 0., p.hop), p.travel_speed);
    return out;
}

double fit_backlash(const BacklashTest &test, double pair_1, double pair_2) { return rad2deg((pair_2 - pair_1) / test.outer_radius); }

// --- Tilt pivot -----------------------------------------------------------------------------

double tilt_pivot_test_angle(double tip_diameter, double layer_height, double max_tilt)
{
    // The tilted tip's lower edge is half its diameter times tan(tilt) below its centre: keep it
    // 0.05 mm above the layer below.
    const double a = rad2deg(std::atan(std::max(0., layer_height - 0.05) / (0.5 * std::max(tip_diameter, 0.1))));
    return std::floor(std::min({ a, 20., max_tilt }) + 1e-9);
}

TiltPivotTest tilt_pivot_test(const TestParams &p, double tilt)
{
    TiltPivotTest t;
    t.inner_radius = std::max(15., 0.25 * p.bed_radius);
    t.outer_radius = std::min(t.inner_radius + std::max(12., 0.15 * p.bed_radius), p.bed_radius - 10.);
    t.tilt         = tilt;
    return t;
}

TestPrint tilt_pivot_print(const TestParams &p, const TiltPivotTest &test)
{
    TestPrint  out;
    PathWriter w { out };
    w.width = p.line_width;
    const double lw = p.line_width, hop = std::max(p.hop, 1.);
    const std::vector<double> tops = layer_tops(p, test.height);
    for (size_t l = 0; l < tops.size(); ++ l) {
        const double z = tops[l], speed = l == 0 ? p.first_layer_speed : p.print_speed;
        const bool   tilted = z > test.lower + 1e-6;
        w.layer(z, l == 0 ? p.first_layer_height : p.layer_height);
        // Each tube's two loops, first the one the tilted tip's lower edge is not over: the inner
        // tube's inner loop first (leaning outward), the outer tube's outer loop.
        for (int tube = 0; tube < 2; ++ tube) {
            const double r    = tube == 0 ? test.inner_radius : test.outer_radius;
            const double tilt = ! tilted ? 0. : tube == 0 ? test.tilt : -test.tilt;
            const double first = tube == 0 ? r - 1.5 * lw : r - 0.5 * lw, second = tube == 0 ? r - 0.5 * lw : r - 1.5 * lw;
            w.go_to(polar(p, first, 0., z), hop, p.travel_speed, tilt);
            draw_loop(w, p, first, z, speed);
            w.travel(polar(p, second, 0., z), p.travel_speed);
            draw_loop(w, p, second, z, speed);
        }
    }
    w.travel(w.last() + Vec3d(0., 0., hop), p.travel_speed);
    return out;
}

bool fit_tilt_pivot(const TiltPivotTest &test, const TiltPivotReading &m, const TiltPivotFit &printed_with, double tilt_sign,
                    TiltPivotFit &out, std::string &error)
{
    const double a = deg2rad(test.tilt);
    auto near = [](double measured, double designed, double tolerance) { return std::abs(measured - designed) < tolerance; };
    if (! near(m.inner_lower, 2. * test.inner_radius, 3.) || ! near(m.inner_upper, 2. * test.inner_radius, 3.) ||
        ! near(m.outer_lower, 2. * test.outer_radius, 3.) || ! near(m.outer_upper, 2. * test.outer_radius, 3.) ||
        ! near(m.inner_height, test.height, 2.) || ! near(m.outer_height, test.height, 2.) || ! (a > 0.)) {
        error = format(_u8L("The tubes should measure about %1% and %2% mm across and %3% mm high: please check the measurements."),
                       CP::format_value(2. * test.inner_radius, 0), CP::format_value(2. * test.outer_radius, 0),
                       CP::format_value(test.height, 0));
        return false;
    }
    // The tip leaning by t lands, relative to where it lands vertical, (L set - L) sin t further
    // out and (L set - L)(cos t - 1) + L e sin t higher, e the lean the tilt offset is off by.
    const double pivot_error = ((m.inner_upper - m.inner_lower) - (m.outer_upper - m.outer_lower)) / (4. * std::sin(a));
    out.pivot_length = printed_with.pivot_length - pivot_error;
    if (! (out.pivot_length > 0.)) {
        error = _u8L("These measurements give no tilt pivot distance: please check them.");
        return false;
    }
    const double lean_error = (m.inner_height - m.outer_height) / (2. * std::sin(a) * out.pivot_length);
    out.tilt_offset   = printed_with.tilt_offset - rad2deg(lean_error) * tilt_sign;
    // Vertical, the tip leaned by the lean error, which the radius offset has been making up for.
    out.radius_offset = printed_with.radius_offset + out.pivot_length * std::sin(lean_error);
    return true;
}

// --- Rotation speed -------------------------------------------------------------------------

RotationSpeedTest rotation_speed_test(double slowest, double fastest, double linear_speed)
{
    constexpr int     rings = 8;
    RotationSpeedTest t;
    t.linear_speed = linear_speed;
    for (int i = 0; i < rings; ++ i) {
        const double s = slowest * std::pow(fastest / slowest, double(i) / (rings - 1));
        t.speeds.emplace_back(s);
        t.radii.emplace_back(linear_speed / deg2rad(s));
    }
    return t;
}

RotationSpeedTest plan_rotation_speed_test(double current_max, double bed_radius, double max_linear_speed)
{
    constexpr double min_radius = 4.;
    const double     slowest    = 0.5 * current_max;
    // Along each ring as fast as allowed, the outermost within the bed.
    const double linear_speed = std::min(max_linear_speed, 0.85 * bed_radius * deg2rad(slowest));
    return rotation_speed_test(slowest, std::min(3. * current_max, rad2deg(linear_speed / min_radius)), linear_speed);
}

double polar_bed_radius(const std::vector<Vec2d> &printable_area, const Vec2d &centre)
{
    double radius = std::numeric_limits<double>::max();
    for (size_t i = 0; i < printable_area.size(); ++ i) {
        const Vec2d  a = printable_area[i], b = printable_area[(i + 1) % printable_area.size()];
        const Vec2d  ab = b - a;
        const double t  = std::clamp((centre - a).dot(ab) / std::max(ab.squaredNorm(), 1e-12), 0., 1.);
        radius          = std::min(radius, (a + t * ab - centre).norm());
    }
    return printable_area.empty() ? 0. : radius;
}

TestPrint rotation_speed_print(const TestParams &p, const RotationSpeedTest &test)
{
    TestPrint  out;
    PathWriter w { out };
    w.width = p.line_width;
    const double z = p.first_layer_height;
    w.layer(z, p.first_layer_height);
    for (double r : test.radii) {
        const double half_gap = 0.5 * test.gap / r, sweep = 2. * M_PI - 2. * half_gap;
        w.go_to(polar(p, r, half_gap, z), p.hop, p.travel_speed, 0.);
        const int n = std::max(36, int(std::ceil(sweep * r)));
        for (int i = 1; i <= n; ++ i)
            w.extrude(polar(p, r, half_gap + sweep * i / n, z), test.linear_speed);
    }
    w.travel(w.last() + Vec3d(0., 0., p.hop), p.travel_speed);
    return out;
}

} // namespace NonPlanar
} // namespace Slic3r
