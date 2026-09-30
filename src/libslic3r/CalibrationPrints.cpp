#include "CalibrationPrints.hpp"

#include "BoundingBox.hpp"
#include "I18N.hpp"
#include "Tesselate.hpp"
#include "format.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace Slic3r {
namespace CalibrationPrints {

// --- Building blocks ------------------------------------------------------------------------

void append_prism(indexed_triangle_set &its, std::vector<Vec3d> loop, const Vec3d &extrude)
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
    append_prism(its, { { x0, y0, z0 }, { x1, y0, z0 }, { x1, y1, z0 }, { x0, y1, z0 } }, Vec3d(0., 0., z1 - z0));
}

std::vector<Vec3d> octagon(const Vec3d &centre, const Vec3d &u, const Vec3d &v, double across)
{
    // Corners between the sides, which face every 45 degrees from u.
    const double       corner = 0.5 * across / std::cos(M_PI / 8.);
    std::vector<Vec3d> out;
    for (int k = 0; k < 8; ++ k) {
        const double a = M_PI / 8. + k * M_PI / 4.;
        out.emplace_back(centre + corner * (std::cos(a) * u + std::sin(a) * v));
    }
    return out;
}

std::vector<Vec3d> circle(const Vec3d &centre, double radius, int segments)
{
    std::vector<Vec3d> out;
    for (int i = 0; i < segments; ++ i) {
        const double a = 2. * M_PI * i / segments;
        out.emplace_back(centre + radius * Vec3d(std::cos(a), std::sin(a), 0.));
    }
    return out;
}

namespace {

BoundingBoxf3 bounds(const indexed_triangle_set &its)
{
    BoundingBoxf3 box;
    for (const Vec3f &v : its.vertices)
        box.merge(v.cast<double>());
    return box;
}

void translate(indexed_triangle_set &its, const Vec3d &shift)
{
    for (Vec3f &v : its.vertices)
        v += shift.cast<float>();
}

} // namespace

void append_slab(indexed_triangle_set &its, const ExPolygon &shape, double z0, double z1)
{
    const int base = int(its.vertices.size());
    // The caps: the triangulation's triangles turn counter-clockwise, which faces +Z.
    for (const Vec2d &p : triangulate_expolygon_2d(shape))
        its.vertices.emplace_back(Vec3f(float(p.x()), float(p.y()), float(z1)));
    const int top = int(its.vertices.size()) - base;
    for (int i = 0; i < top; i += 3)
        its.indices.emplace_back(base + i, base + i + 1, base + i + 2);
    for (int i = 0; i < top; i += 3) {
        const int b = int(its.vertices.size());
        for (int k = 0; k < 3; ++ k) {
            const Vec3f &p = its.vertices[base + i + k];
            its.vertices.emplace_back(p.x(), p.y(), float(z0));
        }
        its.indices.emplace_back(b, b + 2, b + 1);
    }
    // The sides: a contour turns counter-clockwise and a hole clockwise, both with the solid on
    // their left, so each edge's wall faces its right.
    auto walls = [&](const Polygon &polygon) {
        const Points &pts = polygon.points;
        for (size_t i = 0; i < pts.size(); ++ i) {
            const Vec2d a = unscaled(pts[i]), b = unscaled(pts[(i + 1) % pts.size()]);
            const int   v = int(its.vertices.size());
            its.vertices.emplace_back(float(a.x()), float(a.y()), float(z0));
            its.vertices.emplace_back(float(b.x()), float(b.y()), float(z0));
            its.vertices.emplace_back(float(b.x()), float(b.y()), float(z1));
            its.vertices.emplace_back(float(a.x()), float(a.y()), float(z1));
            its.indices.emplace_back(v, v + 1, v + 2);
            its.indices.emplace_back(v, v + 2, v + 3);
        }
    };
    walls(shape.contour);
    for (const Polygon &hole : shape.holes)
        walls(hole);
}

namespace {

Polygon polygon(const std::vector<Vec3d> &points)
{
    Polygon out;
    for (const Vec3d &p : points)
        out.points.emplace_back(scaled(p.x()), scaled(p.y()));
    return out;
}

// Seven segments, a (top) to g (middle), lit for each character.
const char *segments(char c)
{
    static constexpr const char *DIGITS[10] = { "abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg" };
    if (c >= '0' && c <= '9')
        return DIGITS[c - '0'];
    switch (c) {
    case 'A': return "abcefg";
    case 'b': return "cdefg";
    case 'C': return "adef";
    case 'd': return "bcdeg";
    case '-': return "g";
    default: return "";
    }
}

double stroke(double height) { return std::max(0.8, 0.16 * height); }
double char_width(double height) { return 0.6 * height; }
double advance(char c, double height) { return (c == '.' ? stroke(height) : char_width(height)) + stroke(height); }

} // namespace

indexed_triangle_set lay_out(const std::vector<indexed_triangle_set> &pieces, double max_width, double gap)
{
    indexed_triangle_set out;
    double x = 0., y = 0., row_depth = 0.;
    for (const indexed_triangle_set &piece : pieces) {
        const BoundingBoxf3 box  = bounds(piece);
        const Vec3d         size = box.size();
        if (x > 0. && x + size.x() > max_width) {
            x = 0.;
            y += row_depth + gap;
            row_depth = 0.;
        }
        indexed_triangle_set placed = piece;
        translate(placed, Vec3d(x - box.min.x(), y - box.min.y(), -box.min.z()));
        its_merge(out, placed);
        x += size.x() + gap;
        row_depth = std::max(row_depth, size.y());
    }
    const BoundingBoxf3 box = bounds(out);
    translate(out, Vec3d(-box.center().x(), -box.center().y(), 0.));
    return out;
}

std::string format_value(double value, int decimals)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    return buf;
}

double inscribed_radius(const std::vector<Vec2d> &printable_area, const Vec2d &centre)
{
    double radius = std::numeric_limits<double>::max();
    for (size_t i = 0; i < printable_area.size(); ++ i) {
        const Vec2d  a = printable_area[i], ab = printable_area[(i + 1) % printable_area.size()] - a;
        const double t = std::clamp((centre - a).dot(ab) / std::max(ab.squaredNorm(), 1e-12), 0., 1.);
        radius         = std::min(radius, (a + t * ab - centre).norm());
    }
    return printable_area.empty() ? 0. : radius;
}

Vec2d usable_area(const std::vector<Vec2d> &printable_area, double margin)
{
    const BoundingBoxf box(printable_area);
    if (printable_area.size() > 8) {
        const double side = std::sqrt(2.) * std::max(0., inscribed_radius(printable_area, box.center()) - margin);
        return { side, side };
    }
    return { std::max(0., box.size().x() - 2. * margin), std::max(0., box.size().y() - 2. * margin) };
}

std::vector<PiecePlacement> place_on_plates(const std::vector<indexed_triangle_set> &pieces, const Vec2d &area, double gap)
{
    std::vector<PiecePlacement> out(pieces.size());
    // Each piece's lower left corner on its plate, from the plate's first piece's.
    std::vector<Vec2d> corner(pieces.size());
    std::vector<Vec2d> size(pieces.size());
    size_t plate = 0;
    double x = 0., y = 0., row_depth = 0.;
    for (size_t i = 0; i < pieces.size(); ++ i) {
        const BoundingBoxf3 box = bounds(pieces[i]);
        size[i]                 = Vec2d(box.size().x(), box.size().y());
        if (x > 0. && x + size[i].x() > area.x() + 1e-6) {
            x = 0.;
            y += row_depth + gap;
            row_depth = 0.;
        }
        if (y > 0. && y + size[i].y() > area.y() + 1e-6) {
            ++ plate;
            x = y = row_depth = 0.;
        }
        out[i].plate = plate;
        corner[i]    = Vec2d(x, y);
        x += size[i].x() + gap;
        row_depth = std::max(row_depth, size[i].y());
    }
    // Centre each plate's pieces on it.
    for (size_t p = 0; p <= plate; ++ p) {
        BoundingBoxf used;
        for (size_t i = 0; i < pieces.size(); ++ i)
            if (out[i].plate == p) {
                used.merge(corner[i]);
                used.merge(Vec2d(corner[i] + size[i]));
            }
        for (size_t i = 0; i < pieces.size(); ++ i)
            if (out[i].plate == p)
                out[i].centre = corner[i] + 0.5 * size[i] - used.center();
    }
    return out;
}

void append_label(indexed_triangle_set &its, const std::string &text, double x, double y, double z, double height, double depth)
{
    const double s = stroke(height), w = char_width(height), h = height, half = 0.5 * height;
    for (char c : text) {
        if (c == '.')
            append_box(its, x, y, z, x + s, y + s, z + depth);
        else
            for (const char *seg = segments(c); *seg; ++ seg)
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

// --- Shrinkage and skew ---------------------------------------------------------------------

namespace {

// An upright frame: its A boss centred at `a` (on the bed's plane at half a boss up), its first
// axis `u` and its thickness along `n`, both horizontal; the second axis is Z. Bars join the boss
// centres, the bottom one down to the bed; the window's top rises at 45 degrees from both sides
// instead of bridging. A tab on the bed beside the bottom bar, nearer A, carries `number`.
void append_upright(indexed_triangle_set &its, const SquareFrames &f, const Vec3d &a, const Vec3d &u, const Vec3d &n,
                    const std::string &number)
{
    const Vec3d  z = Vec3d::UnitZ();
    const double U = f.upright, w = f.wall, t = f.thickness, R = 0.5 * f.boss;
    // A point of the frame's plane, u along and h above the bed.
    auto at     = [&](double along, double h) { return Vec3d(a + along * u + (h - R) * z - 0.5 * t * n); };
    auto piece  = [&](std::vector<Vec3d> loop) { append_prism(its, std::move(loop), t * n); };
    auto square = [&](double u0, double h0, double u1, double h1) { piece({ at(u0, h0), at(u1, h0), at(u1, h1), at(u0, h1) }); };

    const double gable = R + 0.5 * U; // where the window's top starts rising
    square(0., 0., U, R + 0.5 * w);
    square(-0.5 * w, R, 0.5 * w, gable);
    square(U - 0.5 * w, R, U + 0.5 * w, gable);
    piece({ at(-0.5 * w, gable), at(0.5 * w, gable), at(0.5 * U, R + U - 0.5 * w), at(0.5 * U, R + U + 0.5 * w), at(-0.5 * w, R + U + 0.5 * w) });
    piece({ at(U + 0.5 * w, gable), at(U + 0.5 * w, R + U + 0.5 * w), at(0.5 * U, R + U + 0.5 * w), at(0.5 * U, R + U - 0.5 * w), at(U - 0.5 * w, gable) });
    for (const Vec3d &corner : { Vec3d(0., 0., 0.), Vec3d(U, 0., 0.), Vec3d(0., U, 0.), Vec3d(U, U, 0.) }) {
        std::vector<Vec3d> loop = octagon(a + corner.x() * u + corner.y() * z, u, z, f.boss);
        for (Vec3d &p : loop)
            p -= 0.5 * t * n;
        piece(std::move(loop));
    }

    // The tab: 12 mm along the bottom bar, 10 mm out from it on the side -n, 1.2 mm high.
    constexpr double tab_length = 12., tab_depth = 10., tab_height = 1.2, digit = 5., relief = 0.6;
    const Vec3d      tab0 = a + 0.25 * U * u - R * z - 0.5 * t * n;
    append_prism(its,
                 { tab0 - 0.5 * tab_length * u, tab0 + 0.5 * tab_length * u, tab0 + 0.5 * tab_length * u - tab_depth * n,
                   tab0 - 0.5 * tab_length * u - tab_depth * n },
                 tab_height * z);
    const Vec3d centre = tab0 - 0.5 * tab_depth * n;
    append_label(its, number, centre.x() - 0.5 * label_width(number, digit), centre.y() - 0.5 * digit, tab_height, digit, relief);
}

} // namespace

namespace {

indexed_triangle_set flat_frame(const SquareFrames &f)
{
    indexed_triangle_set its;
    const double s = f.side, w = f.wall, h = f.height;

    // The flat frame: bars between the boss centres, and the bosses.
    append_box(its, -0.5 * s, -0.5 * s - 0.5 * w, 0., 0.5 * s, -0.5 * s + 0.5 * w, h);
    append_box(its, -0.5 * s, 0.5 * s - 0.5 * w, 0., 0.5 * s, 0.5 * s + 0.5 * w, h);
    append_box(its, -0.5 * s - 0.5 * w, -0.5 * s, 0., -0.5 * s + 0.5 * w, 0.5 * s, h);
    append_box(its, 0.5 * s - 0.5 * w, -0.5 * s, 0., 0.5 * s + 0.5 * w, 0.5 * s, h);
    for (const Vec3d &c : { Vec3d(-0.5 * s, -0.5 * s, 0.), Vec3d(0.5 * s, -0.5 * s, 0.), Vec3d(0.5 * s, 0.5 * s, 0.), Vec3d(-0.5 * s, 0.5 * s, 0.) })
        append_prism(its, octagon(c, Vec3d::UnitX(), Vec3d::UnitY(), f.boss), Vec3d(0., 0., f.boss_height));
    // The corners' letters on the bars beside them.
    constexpr double letter = 4., relief = 0.6;
    const double     inset = 0.5 * f.boss + 2., lw = label_width("A", letter);
    append_label(its, "A", -0.5 * s + inset, -0.5 * s - 0.5 * letter, h, letter, relief);
    append_label(its, "b", 0.5 * s - inset - lw, -0.5 * s - 0.5 * letter, h, letter, relief);
    append_label(its, "C", 0.5 * s - inset - lw, 0.5 * s - 0.5 * letter, h, letter, relief);
    append_label(its, "d", -0.5 * s + inset, 0.5 * s - 0.5 * letter, h, letter, relief);
    return its;
}

// Upright frame 1 stands along X, 2 along Y.
void append_upright_number(indexed_triangle_set &its, const SquareFrames &f, int number, const Vec2d &a)
{
    const bool along_x = number == 1;
    append_upright(its, f, Vec3d(a.x(), a.y(), 0.5 * f.boss), along_x ? Vec3d::UnitX() : Vec3d::UnitY(),
                   along_x ? Vec3d::UnitY() : Vec3d::UnitX(), std::to_string(number));
}

} // namespace

bool uprights_inside(const SquareFrames &f) { return f.upright <= f.side - 40. + 1e-9; }

SquareFrames square_frames_for(const Vec2d &area)
{
    SquareFrames f;
    const double room = std::min(area.x(), area.y()) - f.boss;
    f.side            = std::clamp(std::floor(room / 5. + 1e-9) * 5., 40., 100.);
    f.upright         = std::clamp(std::floor(room / 5. + 1e-9) * 5., 30., 60.);
    return f;
}

std::vector<indexed_triangle_set> square_frame_pieces(const SquareFrames &f)
{
    indexed_triangle_set flat = flat_frame(f);
    const double         s    = f.side;
    if (uprights_inside(f)) {
        // The upright frames inside the flat one, clear of each other and of it.
        append_upright_number(flat, f, 1, Vec2d(-0.5 * s + 12., -0.5 * s + 15.));
        append_upright_number(flat, f, 2, Vec2d(0.5 * s - 15., -0.5 * s + 28.));
        return { flat };
    }
    std::vector<indexed_triangle_set> out { flat, {}, {} };
    append_upright_number(out[1], f, 1, Vec2d::Zero());
    append_upright_number(out[2], f, 2, Vec2d::Zero());
    return out;
}

double skew_angle(double ac, double bd, double ad)
{
    const double side = 0.5 * std::sqrt(std::max(0., 2. * ac * ac + 2. * bd * bd - 4. * ad * ad));
    const double cos_a = std::clamp((ac * ac - side * side - ad * ad) / (2. * side * ad), -1., 1.);
    return 90. - std::acos(cos_a) * 180. / M_PI;
}

bool fit_square_frames(const SquareFrames &f, double boss, const FrameReading &flat, const FrameReading &upright_x,
                       const FrameReading &upright_y, FramesFit &out, std::string &error)
{
    const double U = f.upright;
    // Boss centre to boss centre: the parallelogram the frame's corners make.
    auto centres = [boss](const FrameReading &r) { return FrameReading { r.ac - boss, r.bd - boss, r.ad - boss }; };
    auto valid   = [](const FrameReading &r, double side) {
        const double diagonal = side * std::sqrt(2.);
        return std::abs(r.ac - diagonal) < 0.05 * diagonal && std::abs(r.bd - diagonal) < 0.05 * diagonal &&
               std::abs(r.ad - side) < 0.05 * side;
    };
    if (! (boss > 0.5 * f.boss && boss < 1.5 * f.boss)) {
        error = format(_u8L("The boss measured must be near %1% mm."), format_value(f.boss, 0));
        return false;
    }
    const FrameReading xy = centres(flat), xz = centres(upright_x), yz = centres(upright_y);
    if (! valid(xy, f.side) || ! valid(xz, U) || ! valid(yz, U)) {
        error = format(_u8L("Each diagonal should measure about %1% mm and each side about %2% mm on the flat frame, about %3% mm "
                            "and %4% mm on the upright frames: please check the measurements."),
                       format_value(f.side * std::sqrt(2.) + f.boss, 1), format_value(f.side + f.boss, 1),
                       format_value(U * std::sqrt(2.) + f.boss, 1), format_value(U + f.boss, 1));
        return false;
    }
    // Side AB from the parallelogram's diagonals and AD.
    auto ab = [](const FrameReading &r) { return 0.5 * std::sqrt(2. * r.ac * r.ac + 2. * r.bd * r.bd - 4. * r.ad * r.ad); };
    out.scale_x = ab(xy) / f.side;
    out.scale_y = xy.ad / f.side;
    out.scale_z = 0.5 * (xz.ad + yz.ad) / U;
    out.skew_xy = skew_angle(xy.ac, xy.bd, xy.ad);
    out.skew_xz = skew_angle(xz.ac, xz.bd, xz.ad);
    out.skew_yz = skew_angle(yz.ac, yz.bd, yz.ad);
    return true;
}

// --- Holes and pegs, elephant foot ----------------------------------------------------------

std::vector<double> hole_peg_diameters() { return { 5., 10., 15., 20. }; }

std::vector<indexed_triangle_set> hole_and_peg_pieces(const std::vector<double> &diameters)
{
    // A square plate round the feature, the feature's diameter and a margin each side, with a strip
    // in front for its number.
    constexpr double margin = hole_peg_margin, digit = 4., relief = 0.6, strip = digit + 3.;
    auto piece = [&](double d, bool hole) {
        const double half = 0.5 * d + margin, thickness = hole ? hole_plate_thickness : peg_base_thickness;
        ExPolygon    shape(Polygon({ Point(scaled(-half), scaled(-half - strip)), Point(scaled(half), scaled(-half - strip)),
                                     Point(scaled(half), scaled(half)), Point(scaled(-half), scaled(half)) }));
        if (hole) {
            Polygon h = polygon(circle(Vec3d::Zero(), 0.5 * d, 128));
            h.reverse();
            shape.holes.emplace_back(std::move(h));
        }
        indexed_triangle_set its;
        append_slab(its, shape, 0., thickness);
        if (! hole)
            append_prism(its, circle(Vec3d(0., 0., thickness), 0.5 * d, 128), Vec3d(0., 0., peg_height));
        const std::string text = format_value(d, 0);
        append_label(its, text, -0.5 * label_width(text, digit), -half - strip + 1.5, thickness, digit, relief);
        return its;
    };
    std::vector<indexed_triangle_set> out;
    for (bool hole : { true, false })
        for (double d : diameters)
            out.emplace_back(piece(d, hole));
    return out;
}

bool fit_xy_compensation(const std::vector<double> &designed, const std::vector<double> &measured, double &out, std::string &error)
{
    double sum = 0.;
    int    count = 0;
    for (size_t i = 0; i < designed.size() && i < measured.size(); ++ i) {
        if (! (measured[i] > 0.))
            continue;
        const double shortfall = 0.5 * (designed[i] - measured[i]);
        if (std::abs(shortfall) > 1.) {
            error = format(_u8L("The %1% mm feature measures %2% mm, too far off to be the one printed: please check the measurement."),
                           format_value(designed[i], 0), format_value(measured[i], 2));
            return false;
        }
        sum += shortfall;
        ++ count;
    }
    if (count == 0) {
        error = _u8L("Please input at least one measurement.");
        return false;
    }
    out = sum / count;
    return true;
}

indexed_triangle_set elephant_foot_block()
{
    indexed_triangle_set its;
    const double         s = elephant_foot_block_size;
    append_box(its, -0.5 * s, -0.5 * s, 0., 0.5 * s, 0.5 * s, 10.);
    return its;
}

// --- Overhangs and gaps ---------------------------------------------------------------------

std::vector<double> overhang_angles()
{
    std::vector<double> out;
    for (int a = 30; a <= 80; a += 5)
        out.emplace_back(a);
    return out;
}

std::vector<indexed_triangle_set> overhang_samples(const std::vector<double> &angles)
{
    // A 4 x 10 mm post; from 2 mm up its side, a ledge overhanging 10 mm at the angle from
    // vertical, 2 mm thick at its tip; a tab in front numbered with the angle.
    constexpr double post = 4., width = 10., start = 2., reach = 10., tip = 2., tab = 1.2, tab_depth = 8., digit = 5., relief = 0.6;
    std::vector<indexed_triangle_set> pieces;
    for (double a : angles) {
        const double rise = reach / std::tan(a * M_PI / 180.);
        const double top  = std::ceil((start + rise + tip) * 5. - 1e-9) / 5.;
        indexed_triangle_set piece;
        append_box(piece, 0., 0., 0., post, width, top);
        append_prism(piece, { { post, 0., start }, { post + reach, 0., start + rise }, { post + reach, 0., top }, { post, 0., top } },
                     Vec3d(0., width, 0.));
        append_box(piece, 0., -tab_depth, 0., post + reach, 0., tab);
        const std::string text = format_value(a, 0);
        append_label(piece, text, 0.5 * (post + reach - label_width(text, digit)), -0.5 * (tab_depth + digit), tab, digit, relief);
        pieces.emplace_back(std::move(piece));
    }
    return pieces;
}

indexed_triangle_set support_gap_sample(const std::string &label)
{
    constexpr double post = 3., top = 10., height = 8., thickness = 2., digit = 5., relief = 0.6;
    indexed_triangle_set its;
    append_box(its, -post, -post, 0., post, post, height);
    append_box(its, -top, -top, height, top, top, height + thickness);
    append_label(its, label, -0.5 * label_width(label, digit), -0.5 * digit, height + thickness, digit, relief);
    return its;
}

indexed_triangle_set surface_gap_sample(const std::string &label)
{
    constexpr double thickness = 2.4, digit = 4., relief = 0.6;
    indexed_triangle_set its;
    append_prism(its, circle(Vec3d::Zero(), 0.5 * surface_gap_sample_diameter, 96), Vec3d(0., 0., thickness));
    append_label(its, label, -0.5 * label_width(label, digit), -0.5 * digit, thickness, digit, relief);
    return its;
}

} // namespace CalibrationPrints
} // namespace Slic3r
