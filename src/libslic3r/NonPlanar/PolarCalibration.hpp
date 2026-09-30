#pragma once

// Test prints for a polar printer (Calibration > Polar calibration), and the fits that turn what
// is measured on them into its settings.
//
// The bed level test is a mesh sliced as usual. The backlash, tilt pivot and rotation speed tests
// need what a sliced model cannot say (which way the bed turns before a line, a nozzle tilted on
// flat layers, the bed turning faster than its limit), so they are paths, written as G-code in
// place of the model's (CalibMode::Calib_Polar_*). All positions are in the part frame, which
// turns with the bed; tilts lean outward, away from the rotation axis, when positive.

#include "../Point.hpp"
#include "../TriangleMesh.hpp"

#include <array>
#include <string>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

// --- Bed level ------------------------------------------------------------------------------

// Two rings about the rotation axis, a few layers thick, each numbered at four marks a quarter
// turn apart (1 to 4 on the outer ring from +X counter-clockwise, 5 to 8 on the inner one): the
// rings' thickness at the marks shows how the bed's height varies under the nozzle.
struct BedLevelRings
{
    double inner_radius = 35.; // middle of the inner ring
    double outer_radius = 80.; // middle of the outer ring
    double width        = 6.;
    double thickness    = 0.6;
};
BedLevelRings bed_level_rings(double bed_radius, double thickness);
// Centred on the rotation axis at the origin.
indexed_triangle_set bed_level_mesh(const BedLevelRings &rings);
// Where the marks are, in the order numbered.
std::vector<Vec2d> bed_level_marks(const BedLevelRings &rings);

// How much higher the bed is than the rings were printed for, mm per mm of distance from the
// axis: toward +X, toward +Y and outward all round (PolarKinematicsConfig::bed_tilt_x, _y, cone).
// A ring prints thinner where the bed is higher.
struct BedLevelFit
{
    double tilt_x = 0., tilt_y = 0., cone = 0.;
};
bool fit_bed_level(const BedLevelRings &rings, const std::array<double, 8> &thickness, BedLevelFit &out, std::string &error);

// --- Paths written as G-code ----------------------------------------------------------------

struct TestMove
{
    Vec3d  to      = Vec3d::Zero();
    bool   extrude = false;
    double width   = 0.;  // of the bead laid, when extruding
    double speed   = 0.;  // mm/s
    double tilt    = 0.;  // degrees, from this move on
};

struct TestLayer
{
    double z          = 0.;
    double height     = 0.;
    size_t first_move = 0;
};

struct TestPrint
{
    std::vector<TestMove>  moves;
    std::vector<TestLayer> layers;
};

struct TestParams
{
    Vec2d  centre             = Vec2d::Zero(); // the rotation axis, in G-code coordinates
    double bed_radius         = 100.;
    double line_width         = 0.45;
    double first_layer_height = 0.2;
    double layer_height       = 0.2;
    double first_layer_speed  = 20.;
    double print_speed        = 40.;
    double travel_speed       = 100.;
    double hop                = 0.6;
};

// --- Backlash -------------------------------------------------------------------------------

// Two pairs of thin radial fins, numbered 1 and 2 on the bed beside them. The bed turns the same
// way before each fin of pair 1, and opposite ways before the two fins of pair 2, so that play in
// its drive the setting does not take up moves pair 2's fins apart (or together, if the setting
// takes up more than there is).
struct BacklashTest
{
    double inner_radius = 0.;
    double outer_radius = 0.; // where the pairs are measured
    double gap          = 3.; // between a pair's fins at the outer radius
    double height       = 6.;
};
BacklashTest backlash_test(const TestParams &params);
TestPrint    backlash_print(const TestParams &params, const BacklashTest &test);
// The width across each pair's fins at their outer ends: the backlash the setting leaves, degrees
// (to add to it).
double fit_backlash(const BacklashTest &test, double pair_1, double pair_2);

// --- Tilt pivot -----------------------------------------------------------------------------

// Two tubes about the rotation axis, printed with the nozzle vertical for their lower half and
// tilted for the upper: the inner tube leaning outward, the outer one inward, by `tilt`. A pivot
// distance off moves the tilted halves' walls in or out; a tilt offset off moves them up or down.
struct TiltPivotTest
{
    double inner_radius = 25.; // outer radius of each tube
    double outer_radius = 40.;
    double tilt         = 15.; // degrees
    double lower        = 4.;  // height printed vertical
    double height       = 8.;
};
// The steepest whole-degree tilt at which the tip's edge stays clear of the layer below, up to 20.
double tilt_pivot_test_angle(double tip_diameter, double layer_height, double max_tilt);
TiltPivotTest tilt_pivot_test(const TestParams &params, double tilt);
TestPrint     tilt_pivot_print(const TestParams &params, const TiltPivotTest &test);

struct TiltPivotReading
{
    double inner_lower = 0., inner_upper = 0.; // outer diameters of the inner tube's halves
    double outer_lower = 0., outer_upper = 0.;
    double inner_height = 0., outer_height = 0.;
};
// The settings that correct what was measured, from those the tubes were printed with, on a machine
// whose tilt axis counts `tilt_sign` (-1 when positive leans toward the rotation axis). The radius
// offset changes with the tilt offset so that vertical printing stays where it was.
struct TiltPivotFit
{
    double pivot_length  = 0.;
    double tilt_offset   = 0.;
    double radius_offset = 0.;
};
bool fit_tilt_pivot(const TiltPivotTest &test, const TiltPivotReading &reading, const TiltPivotFit &printed_with, double tilt_sign,
                    TiltPivotFit &out, std::string &error);

// --- Rotation speed -------------------------------------------------------------------------

// Rings about the rotation axis, one layer, outermost first, each at the same speed along the
// ring, so the bed turns faster for each smaller ring. Each ring stops short of a full turn,
// leaving a gap at +X: the gaps line up until the bed turned too fast for its motor and lost
// steps, turning every ring after it.
struct RotationSpeedTest
{
    std::vector<double> speeds; // degrees per second, one per ring, outermost first
    std::vector<double> radii;
    double              linear_speed = 0.; // mm/s along each ring
    double              gap          = 2.; // mm
};
// Rings from `slowest` to `fastest` degrees per second, evenly apart in ratio, at `linear_speed`.
RotationSpeedTest rotation_speed_test(double slowest, double fastest, double linear_speed);
// The rings for a bed now limited to `current_max` degrees per second: from half to three times
// it, at a speed along them no more than `max_linear_speed`, within the bed and none smaller than
// 4 mm (lowering the fastest).
RotationSpeedTest plan_rotation_speed_test(double current_max, double bed_radius, double max_linear_speed);
TestPrint         rotation_speed_print(const TestParams &params, const RotationSpeedTest &test);

// The radius of the largest circle about `centre` inside the printable area.
double polar_bed_radius(const std::vector<Vec2d> &printable_area, const Vec2d &centre);

} // namespace NonPlanar
} // namespace Slic3r
