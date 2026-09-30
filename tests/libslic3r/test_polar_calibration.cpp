#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/PolarCalibration.hpp"
#include "libslic3r/NonPlanar/PolarKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::NonPlanar;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

double deg2rad(double a) { return a * M_PI / 180.; }

double wrap_pi(double a)
{
    while (a > M_PI)
        a -= 2. * M_PI;
    while (a <= -M_PI)
        a += 2. * M_PI;
    return a;
}

TestParams params()
{
    TestParams p;
    p.centre       = Vec2d::Zero();
    p.bed_radius   = 100.;
    p.line_width   = 0.45;
    p.layer_height = 0.2;
    return p;
}

// The angle each extrusion ends at on a bed whose drive has `play` degrees of it, the angle
// commanded half `setting` further in the way the bed last turned, as the polar conversion does:
// the bed follows its motor only once the play is taken up. With the radius each ends at.
struct Landed
{
    double radius, angle;
};
std::vector<Landed> land(const TestPrint &print, double play, double setting)
{
    std::vector<Landed> out;
    double nominal = 0., motor = 0., bed = 0.;
    int    dir     = 0;
    bool   first   = true;
    for (const TestMove &m : print.moves) {
        const double a = std::atan2(m.to.y(), m.to.x());
        if (first) {
            nominal = motor = bed = a;
            first               = false;
        } else {
            const double turn = wrap_pi(a - nominal);
            nominal += turn;
            if (std::abs(turn) > 1e-9)
                dir = turn > 0. ? 1 : -1;
            motor = nominal + 0.5 * deg2rad(setting) * dir;
            bed   = std::clamp(bed, motor - 0.5 * deg2rad(play), motor + 0.5 * deg2rad(play));
        }
        // The part's angle under the nozzle is the bed's.
        if (m.extrude)
            out.push_back({ m.to.head<2>().norm(), bed });
    }
    return out;
}

// Across each backlash pair's fins at their outer ends, in the last layer: pair 1 above the X
// axis, pair 2 below it.
std::array<double, 2> pair_widths(const TestPrint &print, const BacklashTest &test, double play, double setting)
{
    const std::vector<Landed> landed = land(print, play, setting);
    const size_t              from   = print.layers.back().first_move;
    std::array<double, 2>     lo { 1e9, 1e9 }, hi { -1e9, -1e9 };
    size_t                    k = 0;
    for (size_t i = 0; i < print.moves.size(); ++ i) {
        if (! print.moves[i].extrude)
            continue;
        const Landed &l = landed[k ++];
        if (i < from || std::abs(l.radius - test.outer_radius) > 1e-6)
            continue;
        const int pair = l.angle > 0. ? 0 : 1;
        lo[pair]       = std::min(lo[pair], l.angle);
        hi[pair]       = std::max(hi[pair], l.angle);
    }
    const double w = params().line_width;
    return { test.outer_radius * (hi[0] - lo[0]) + w, test.outer_radius * (hi[1] - lo[1]) + w };
}

} // namespace

TEST_CASE("The bed level fit finds how the bed's height varies under the rings", "[PolarCalibration]")
{
    const BedLevelRings rings = bed_level_rings(100., 0.6);
    // A bed 0.1 mm higher per 100 mm toward +X, 0.05 lower toward +Y and 0.08 higher outward.
    const double           a = 0.001, b = -0.0005, c = 0.0008;
    const std::vector<Vec2d> marks = bed_level_marks(rings);
    REQUIRE(marks.size() == 8);
    std::array<double, 8> thickness;
    for (size_t i = 0; i < 8; ++ i)
        thickness[i] = 0.6 - (a * marks[i].x() + b * marks[i].y() + c * marks[i].norm());
    BedLevelFit fit;
    std::string error;
    REQUIRE(fit_bed_level(rings, thickness, fit, error));
    CHECK_THAT(fit.tilt_x, WithinAbs(a, 1e-9));
    CHECK_THAT(fit.tilt_y, WithinAbs(b, 1e-9));
    CHECK_THAT(fit.cone, WithinAbs(c, 1e-9));
}

TEST_CASE("Bed level readings far from the rings' thickness are refused", "[PolarCalibration]")
{
    const BedLevelRings   rings = bed_level_rings(100., 0.6);
    std::array<double, 8> thickness;
    thickness.fill(0.6);
    thickness[3] = 6.;
    BedLevelFit fit;
    std::string error;
    CHECK_FALSE(fit_bed_level(rings, thickness, fit, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("The bed level rings stand on the bed about the axis", "[PolarCalibration]")
{
    const BedLevelRings  rings = bed_level_rings(100., 0.6);
    indexed_triangle_set its   = bed_level_mesh(rings);
    its_merge_vertices(its);
    CHECK(its_num_open_edges(its) == 0);
    double far = 0.;
    for (const Vec3f &v : its.vertices)
        far = std::max(far, double(v.head<2>().norm()));
    // The outer ring's edge, the numbers beyond it within the bed.
    CHECK(far > rings.outer_radius + 0.5 * rings.width);
    CHECK(far < 100.);
}

TEST_CASE("Play the backlash setting leaves moves the second pair's fins apart", "[PolarCalibration]")
{
    const TestParams   p    = params();
    const BacklashTest test = backlash_test(p);
    const TestPrint    print = backlash_print(p, test);
    const double       play = GENERATE(0., 0.3, 1.);
    const double       setting = GENERATE(0., 0.2);
    const std::array<double, 2> w = pair_widths(print, test, play, setting);
    // The fit finds the play left over.
    CHECK_THAT(fit_backlash(test, w[0], w[1]), WithinAbs(play - setting, 1e-6));
}

TEST_CASE("The tilt pivot fit corrects the pivot distance and tilt offset", "[PolarCalibration]")
{
    // The machine: pivot 43 mm, vertical at a commanded tilt of 0.8 degrees, its tilt counting
    // positive toward the rotation axis. Set up with a 41 mm pivot and no tilt offset, and the
    // radius offset that puts vertical printing where it should be.
    PolarKinematicsConfig truth;
    truth.tilt_pivot_length = 43.;
    truth.tilt_offset       = 0.8;
    truth.tilt_sign         = -1.;
    truth.radius_offset     = 0.3;
    PolarKinematicsConfig set = truth;
    set.tilt_pivot_length     = 41.;
    set.tilt_offset           = 0.;
    // Vertical as commanded, the nozzle leans 0.8 degrees outward.
    set.radius_offset         = truth.radius_offset - truth.tilt_pivot_length * std::sin(deg2rad(0.8));
    const PolarKinematics machine(truth), slicer(set);
    auto landed = [&](const PolarKinematics &k, const Vec3d &tip, double tilt) {
        ToolPose pose;
        pose.tip  = tip;
        pose.tilt = deg2rad(tilt);
        return machine.to_tool(k.to_machine(pose, nullptr)).tip;
    };

    const TestParams    p    = params();
    const TiltPivotTest test = tilt_pivot_test(p, 15.);
    const TestPrint     print = tilt_pivot_print(p, test);
    // Each tube's outer loop radius and its top, lower half and upper half.
    double lower[2] = { 0., 0. }, upper[2] = { 0., 0. }, top[2] = { 0., 0. };
    for (const TestMove &m : print.moves) {
        if (! m.extrude)
            continue;
        const int    tube = std::abs(m.to.head<2>().norm() - test.inner_radius) < 5. ? 0 : 1;
        const double r    = tube == 0 ? test.inner_radius : test.outer_radius;
        const Vec3d  at   = landed(slicer, m.to, m.tilt);
        top[tube]         = std::max(top[tube], at.z());
        if (std::abs(m.to.head<2>().norm() - (r - 0.5 * p.line_width)) < 1e-6)
            (m.tilt == 0. ? lower : upper)[tube] = at.head<2>().norm() + 0.5 * p.line_width;
    }
    TiltPivotReading reading;
    reading.inner_lower  = 2. * lower[0];
    reading.inner_upper  = 2. * upper[0];
    reading.outer_lower  = 2. * lower[1];
    reading.outer_upper  = 2. * upper[1];
    reading.inner_height = top[0];
    reading.outer_height = top[1];
    TiltPivotFit fit;
    std::string  error;
    REQUIRE(fit_tilt_pivot(test, reading, { set.tilt_pivot_length, set.tilt_offset, set.radius_offset }, set.tilt_sign, fit, error));
    CHECK_THAT(fit.pivot_length, WithinAbs(43., 0.01));
    CHECK_THAT(fit.tilt_offset, WithinAbs(0.8, 0.01));

    // With the settings fitted, vertical and tilted printing land where they are commanded.
    PolarKinematicsConfig fitted = set;
    fitted.tilt_pivot_length     = fit.pivot_length;
    fitted.tilt_offset           = fit.tilt_offset;
    fitted.radius_offset         = fit.radius_offset;
    const PolarKinematics corrected(fitted);
    for (double tilt : { 0., 15., -15. }) {
        const Vec3d tip(40., 10., 5.), at = landed(corrected, tip, tilt);
        DYNAMIC_SECTION("tilt " << tilt) {
            CHECK_THAT(at.head<2>().norm(), WithinAbs(tip.head<2>().norm(), 0.01));
            CHECK_THAT(at.z(), WithinAbs(tip.z(), 0.01));
        }
    }
}

TEST_CASE("The tilt pivot test tilts no further than the tip's edge clears the layer below", "[PolarCalibration]")
{
    // tan(tilt) at most 0.15 / 0.4 for a 0.8 mm tip on 0.2 mm layers: 20.6 degrees.
    CHECK_THAT(tilt_pivot_test_angle(0.8, 0.2, 90.), WithinAbs(20., 1e-9));
    CHECK_THAT(tilt_pivot_test_angle(1.0, 0.2, 90.), WithinAbs(16., 1e-9));
    CHECK_THAT(tilt_pivot_test_angle(0.8, 0.2, 10.), WithinAbs(10., 1e-9));
}

TEST_CASE("The rotation speed rings step up from the slowest to the fastest speed", "[PolarCalibration]")
{
    const RotationSpeedTest t = plan_rotation_speed_test(360., 100., 100.);
    REQUIRE(t.speeds.size() == t.radii.size());
    REQUIRE(t.speeds.size() >= 2);
    CHECK_THAT(t.speeds.front(), WithinRel(180., 1e-9));
    // Three times 360, as the innermost ring at 100 mm/s is then 5.3 mm in radius.
    CHECK_THAT(t.speeds.back(), WithinRel(1080., 1e-9));
    for (size_t i = 0; i < t.speeds.size(); ++ i)
        DYNAMIC_SECTION("ring " << i) {
            CHECK_THAT(t.radii[i] * deg2rad(t.speeds[i]), WithinRel(t.linear_speed, 1e-9));
            CHECK(t.radii[i] >= 4. - 1e-9);
            CHECK(t.radii[i] <= 85. + 1e-9);
        }
}

TEST_CASE("Each rotation speed ring leaves its gap at +X", "[PolarCalibration]")
{
    const TestParams        p     = params();
    const RotationSpeedTest t     = plan_rotation_speed_test(360., 100., 100.);
    const TestPrint         print = rotation_speed_print(p, t);
    for (const TestMove &m : print.moves)
        if (m.extrude) {
            const double r = m.to.head<2>().norm(), a = std::atan2(m.to.y(), m.to.x());
            // Half the gap each side of +X, less a little for rounding.
            CHECK(std::abs(a) >= 0.5 * t.gap / r - 1e-6);
        }
}

TEST_CASE("The polar bed radius is the largest circle about the axis on the bed", "[PolarCalibration]")
{
    std::vector<Vec2d> square = { { -100., -50. }, { 100., -50. }, { 100., 50. }, { -100., 50. } };
    CHECK_THAT(polar_bed_radius(square, Vec2d::Zero()), WithinAbs(50., 1e-9));
    CHECK_THAT(polar_bed_radius(square, Vec2d(90., 0.)), WithinAbs(10., 1e-9));
}
