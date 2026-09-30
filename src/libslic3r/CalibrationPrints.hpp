#pragma once

// Test prints for OrcaNP's calibrations (Calibration > Dimensions, Overhangs and gaps, Toolhead
// clearance, Polar calibration), and the fits that turn what is measured on them into settings.
//
// The prints are meshes built from simple prisms, which may overlap: slicing joins them. Heights
// are whole multiples of 0.1 mm, so they print true in 0.1 or 0.2 mm layers. Labels are raised
// seven-segment characters.

#include "ExPolygon.hpp"
#include "TriangleMesh.hpp"

#include <string>
#include <vector>

namespace Slic3r {
namespace CalibrationPrints {

// --- Building blocks ------------------------------------------------------------------------

// A prism: the planar convex `loop` and the same loop moved by `extrude`, closed, facing out.
void append_prism(indexed_triangle_set &its, std::vector<Vec3d> loop, const Vec3d &extrude);
void append_box(indexed_triangle_set &its, double x0, double y0, double z0, double x1, double y1, double z1);
// A regular octagon about `centre` in the plane of the unit vectors `u` and `v`, `across` between
// opposite sides, with sides facing +-u, +-v and the diagonals between them.
std::vector<Vec3d> octagon(const Vec3d &centre, const Vec3d &u, const Vec3d &v, double across);
// A slab of `shape`, holes and all, from z0 to z1.
void append_slab(indexed_triangle_set &its, const ExPolygon &shape, double z0, double z1);
// A polygon approximating a circle of `radius` about `centre` in the XY plane at `z`.
std::vector<Vec3d> circle(const Vec3d &centre, double radius, int segments);
// Pieces laid out in rows no wider than `max_width`, left to right, then centred on the origin,
// standing on Z = 0.
indexed_triangle_set lay_out(const std::vector<indexed_triangle_set> &pieces, double max_width, double gap);
// `value` with `decimals` digits after the point.
std::string format_value(double value, int decimals);

// --- Laying pieces out on the bed -----------------------------------------------------------

// The radius of the largest circle about `centre` inside the printable area.
double inscribed_radius(const std::vector<Vec2d> &printable_area, const Vec2d &centre);
// The rectangle, width and depth, pieces are laid out in, centred on the bed: the bed less
// `margin` all round, or the square inside a round bed (more than 8 corners).
Vec2d usable_area(const std::vector<Vec2d> &printable_area, double margin);
// Where each piece goes: on which plate, and where its bounding box's centre goes from the bed's
// centre. Pieces are laid in order in rows within `area`, `gap` apart, starting a plate when a
// plate is full, and each plate's pieces are centred on it.
struct PiecePlacement
{
    size_t plate  = 0;
    Vec2d  centre = Vec2d::Zero();
};
std::vector<PiecePlacement> place_on_plates(const std::vector<indexed_triangle_set> &pieces, const Vec2d &area, double gap);

// Raised seven-segment characters for `text`, `height` tall and `depth` thick, from (x, y) along
// +X on the plane Z = z: the digits, '.', '-', and the letters A, b, C and d. Other characters
// leave a space.
void append_label(indexed_triangle_set &its, const std::string &text, double x, double y, double z, double height, double depth);
// Width of append_label()'s text.
double label_width(const std::string &text, double height);

// --- Shrinkage and skew ---------------------------------------------------------------------

// A flat square frame with an octagonal boss at each corner, for the XY plane, and two upright
// square frames, one along X (upright 1) and one along Y (upright 2), for XZ and YZ. On each, A is
// the corner at the lowest of both axes, B along the first axis from it, D along the second and C
// opposite A; the bosses are measured across between their outer sides. The upright frames stand
// inside the flat one when it is large enough, otherwise beside it.
struct SquareFrames
{
    double side    = 100.; // distance between the flat frame's boss centres
    double upright = 60.;  // distance between an upright frame's boss centres
    double wall    = 5.;   // width of the frames' bars
    double height  = 3.;   // height of the flat frame's bars
    double boss    = 8.;   // across a boss's sides
    double boss_height = 6.;  // height of the flat frame's bosses
    double thickness   = 4.;  // thickness of the upright frames
};
// Frames as large as fit an `area` (usable_area()), up to 100 mm flat and 60 mm upright.
SquareFrames square_frames_for(const Vec2d &area);
bool         uprights_inside(const SquareFrames &f);
// The flat frame with the upright ones inside it, or the three frames apart, each centred on the
// origin.
std::vector<indexed_triangle_set> square_frame_pieces(const SquareFrames &f);

// A frame measured with calipers across its bosses: diagonals AC and BD, and side AD.
struct FrameReading
{
    double ac = 0., bd = 0., ad = 0.;
};

// The skew angle, degrees, of a parallelogram ABCD from its diagonals and side AD, as Klipper
// computes it: how far the angle at A falls short of 90 degrees (the second axis leaning toward
// the first).
double skew_angle(double ac, double bd, double ad);

// What the frames measure: the scale of each axis (as printed over as designed) and the skew of
// each plane, degrees. `boss` is a boss as measured across; readings are between bosses' outer
// sides, so it is taken off each.
struct FramesFit
{
    double scale_x = 1., scale_y = 1., scale_z = 1.;
    double skew_xy = 0., skew_xz = 0., skew_yz = 0.;
};
bool fit_square_frames(const SquareFrames &f, double boss, const FrameReading &flat, const FrameReading &upright_x,
                       const FrameReading &upright_y, FramesFit &out, std::string &error);

// --- Holes and pegs, elephant foot ----------------------------------------------------------

// Diameters of the holes and of the pegs, mm.
std::vector<double> hole_peg_diameters();
// A plate with a round hole through it per diameter, then a base with a round peg on it per
// diameter, each numbered, centred on the hole or peg. The plate and base reach `hole_peg_margin`
// beyond the feature, with a strip in front for the number.
constexpr double hole_peg_margin      = 6.;
constexpr double hole_plate_thickness = 4.;
constexpr double peg_base_thickness   = 2.;
constexpr double peg_height           = 10.;
std::vector<indexed_triangle_set> hole_and_peg_pieces(const std::vector<double> &diameters);
// The X-Y compensation to add to the current one for features of these diameters, as designed and
// as measured (0 where not measured): half the mean shortfall. False, with `error` set, when none
// is measured or a measurement is far off.
bool fit_xy_compensation(const std::vector<double> &designed, const std::vector<double> &measured, double &out, std::string &error);

// A block for the elephant foot: its width is measured at the very bottom and half way up.
constexpr double elephant_foot_block_size = 20.;
indexed_triangle_set elephant_foot_block();

// --- Overhangs and gaps ---------------------------------------------------------------------

// Overhang angles from vertical, degrees.
std::vector<double> overhang_angles();
// A sample per angle: a post with a ledge whose underside overhangs at the angle, numbered.
std::vector<indexed_triangle_set> overhang_samples(const std::vector<double> &angles);

// A small table for a support gap: a post and a flat top overhanging it all round, supported,
// labelled with the gap.
indexed_triangle_set support_gap_sample(const std::string &label);
// A disc labelled with a surface gap, printed over a generated pillar (Non-planar (S4), Offset from
// print surface).
constexpr double surface_gap_sample_diameter = 16.;
indexed_triangle_set surface_gap_sample(const std::string &label);

} // namespace CalibrationPrints
} // namespace Slic3r
