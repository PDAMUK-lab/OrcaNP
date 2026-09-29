#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/PolarKinematics.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r::NonPlanar;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double PI = 3.14159265358979323846;

ToolPose pose(double x, double y, double z, double tilt_deg = 0.)
{
    ToolPose p;
    p.tip  = Eigen::Vector3d(x, y, z);
    p.tilt = tilt_deg * PI / 180.;
    return p;
}

// Machine poses parsed back out of converted G-code, turned into Cartesian tool poses.
struct Replayed
{
    std::vector<ToolPose> poses;
    std::vector<double>   minutes; // duration of each G93 move
    double                e_total = 0.;
    size_t                g94_e_only = 0;
};

double word(const std::string &line, char letter, double fallback)
{
    const size_t semi = line.find(';');
    const std::string code = line.substr(0, semi);
    for (size_t i = 2; i < code.size(); ++i)
        if (code[i] == letter && code[i - 1] == ' ')
            return std::stod(code.substr(i + 1));
    return fallback;
}

Replayed replay(const std::string &gcode, const PolarKinematics &kin)
{
    Replayed           r;
    std::istringstream in(gcode);
    std::string        line;
    MachinePose        m;
    bool               inverse_time = false;
    while (std::getline(in, line)) {
        if (line.find("MACHINE_END_GCODE_START") != std::string::npos)
            break;
        if (line.rfind("G93", 0) == 0)
            inverse_time = true;
        else if (line.rfind("G94", 0) == 0)
            inverse_time = false;
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        const bool has_axes = line.find(" C") != std::string::npos;
        r.e_total += word(line, 'E', 0.);
        if (! has_axes) {
            if (! inverse_time && line.find(" E") != std::string::npos)
                ++r.g94_e_only;
            continue;
        }
        m.angle  = word(line, 'C', m.angle);
        m.radius = word(line, 'X', m.radius);
        m.z      = word(line, 'Z', m.z);
        m.tilt   = word(line, 'B', m.tilt);
        r.poses.push_back(kin.to_tool(m));
        if (inverse_time)
            r.minutes.push_back(1. / word(line, 'F', 0.));
    }
    return r;
}

double distance_to_segment(const Eigen::Vector3d &p, const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    const Eigen::Vector3d ab = b - a;
    const double          t  = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0., 1.);
    return (a + t * ab - p).norm();
}

} // namespace

TEST_CASE("A machine pose converts back to the tool pose it came from", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.center            = Eigen::Vector2d(100., 90.);
    cfg.angle_sign        = GENERATE(1., -1.);
    cfg.tilt_pivot_length = GENERATE(0., 42.);
    cfg.tilt_sign         = GENERATE(1., -1.);
    cfg.radius_offset     = GENERATE(0., 0.7);
    cfg.radius_scale      = GENERATE(1., 1.02);
    const PolarKinematics kin(cfg);

    const std::vector<ToolPose> poses = { pose(130., 90., 5., 10.), pose(100., 120., 2., -25.), pose(60., 50., 0.2, 0.),
                                          pose(100.3, 89.9, 7., 3.) };
    MachinePose prev;
    for (size_t i = 0; i < poses.size(); ++i) {
        const MachinePose m    = kin.to_machine(poses[i], i == 0 ? nullptr : &prev);
        const ToolPose    back = kin.to_tool(m);
        CHECK_THAT((back.tip - poses[i].tip).norm(), WithinAbs(0., 1e-9));
        CHECK_THAT(back.tilt, WithinAbs(poses[i].tilt, 1e-9));
        prev = m;
    }
}

TEST_CASE("A calibrated radius axis is commanded where it puts the tip", "[PolarKinematics]")
{
    // The axis puts the tip 1.02 R + 0.7 mm from the rotation axis for a commanded R: a tip 20 mm
    // out is commanded (20 - 0.7) / 1.02, on either side of the axis.
    PolarKinematicsConfig cfg;
    cfg.radius_offset = 0.7;
    cfg.radius_scale  = 1.02;
    const PolarKinematics kin(cfg);
    CHECK_THAT(kin.to_machine(pose(20., 0., 1., 0.), nullptr).radius, WithinAbs((20. - 0.7) / 1.02, 1e-9));
    const MachinePose across = kin.to_machine(pose(-20., 0., 1., 0.), nullptr);
    CHECK_THAT(std::abs(across.radius * 1.02 + 0.7), WithinAbs(20., 1e-9));
}

TEST_CASE("Circling the rotation axis turns the angle axis continuously", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    const PolarKinematics kin(cfg);

    // Three full turns in 5 degree steps, starting on the -X axis where atan2 wraps.
    MachinePose prev = kin.to_machine(pose(-20., 0., 0.), nullptr);
    const double start = prev.angle;
    for (int step = 1; step <= 3 * 72; ++step) {
        const double      a = PI + step * 5. * PI / 180.;
        const MachinePose m = kin.to_machine(pose(20. * std::cos(a), 20. * std::sin(a), 0.), &prev);
        CHECK_THAT(m.angle - prev.angle, WithinAbs(5., 1e-9));
        CHECK_THAT(m.radius, WithinAbs(20., 1e-9));
        prev = m;
    }
    CHECK_THAT(prev.angle - start, WithinAbs(3. * 360., 1e-6));
}

TEST_CASE("A path through the rotation axis continues on a negative radius", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.signed_radius = true;
    const PolarKinematics kin(cfg);

    const ToolPose from = pose(-30., 0.01, 1.);
    const ToolPose to   = pose(30., -0.01, 1.);
    const MachinePose m0 = kin.to_machine(from, nullptr);
    const auto waypoints = kin.interpolate(from, m0, to);

    REQUIRE_FALSE(waypoints.empty());
    // The bed barely turns: the head slides through the axis instead.
    CHECK(std::abs(waypoints.back().pose.angle - m0.angle) < 1.);
    CHECK(m0.radius > 0.);
    CHECK(waypoints.back().pose.radius < 0.);
}

TEST_CASE("Interpolated segments respect the angle and length limits and stay on the line", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.max_angle_step     = 1.;
    cfg.max_segment_length = 2.;
    const PolarKinematics kin(cfg);

    const ToolPose from = pose(40., -35., 0.3, 0.);
    const ToolPose to   = pose(-25., 50., 1.8, 12.);
    MachinePose    prev = kin.to_machine(from, nullptr);
    const auto     waypoints = kin.interpolate(from, prev, to);

    REQUIRE(waypoints.size() > 10);
    Eigen::Vector3d prev_tip = from.tip;
    for (const auto &w : waypoints) {
        CHECK(std::abs(w.pose.angle - prev.angle) <= cfg.max_angle_step + 1e-9);
        const ToolPose p = kin.to_tool(w.pose);
        CHECK((p.tip - prev_tip).norm() <= cfg.max_segment_length + 1e-9);
        // Each waypoint is exactly on the Cartesian line; the machine interpolates between them.
        CHECK_THAT(distance_to_segment(p.tip, from.tip, to.tip), WithinAbs(0., 1e-9));
        prev     = w.pose;
        prev_tip = p.tip;
    }
    CHECK_THAT((kin.to_tool(waypoints.back().pose).tip - to.tip).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(waypoints.back().t, WithinAbs(1., 1e-12));
}

TEST_CASE("Converted G-code reproduces the Cartesian toolpath, extrusion and timing", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.center = Eigen::Vector2d(100., 100.);
    const PolarKinematics kin(cfg);

    // A 40 mm square around the rotation axis, printed at F1200 with relative extrusion.
    const std::string cartesian =
        "G28 ; home\n"
        "M83\n"
        "; MACHINE_START_GCODE_END\n"
        "G1 X80 Y80 Z0.2 F6000\n"
        "G1 F1200\n"
        "G1 X120 Y80 E2\n"
        "G1 X120 Y120 E2\n"
        "G1 X80 Y120 E2\n"
        "G1 X80 Y80 E2 ; close\n"
        "G1 E-0.8 F2400\n"
        "; MACHINE_END_GCODE_START\n"
        "G1 C0 X0 Z50 ; park\n";

    std::istringstream  in(cartesian);
    std::ostringstream  out;
    PolarGCodeConverter converter(cfg);
    converter.process(in, out);
    const std::string polar = out.str();

    SECTION("machine blocks are copied verbatim") {
        CHECK(polar.find("G28 ; home\n") != std::string::npos);
        CHECK(polar.find("G1 C0 X0 Z50 ; park\n") != std::string::npos);
    }

    const Replayed r = replay(polar, kin);
    REQUIRE(r.poses.size() > 4);

    SECTION("every emitted pose lies on the square") {
        const std::vector<Eigen::Vector3d> corners = { { 80., 80., 0.2 }, { 120., 80., 0.2 }, { 120., 120., 0.2 },
                                                       { 80., 120., 0.2 }, { 80., 80., 0.2 } };
        for (const ToolPose &p : r.poses) {
            double best = 1e9;
            for (size_t k = 0; k + 1 < corners.size(); ++k)
                best = std::min(best, distance_to_segment(p.tip, corners[k], corners[k + 1]));
            CHECK_THAT(best, WithinAbs(0., 1e-3));
        }
        CHECK_THAT((r.poses.back().tip - corners.back()).norm(), WithinAbs(0., 1e-3));
    }

    SECTION("extrusion is preserved and the retraction is a plain feedrate move") {
        CHECK_THAT(r.e_total, WithinAbs(8. - 0.8, 1e-4));
        CHECK(r.g94_e_only == 1);
    }

    SECTION("inverse time durations add up to the Cartesian print time") {
        double minutes = 0.;
        for (double m : r.minutes)
            minutes += m;
        // 160 mm at 1200 mm/min; the bed's 360 degree turn is far below its speed limit.
        CHECK_THAT(minutes, WithinRel(160. / 1200., 1e-3));
    }

    CHECK(converter.stats().cartesian_moves == 5);
    CHECK_THAT(converter.stats().total_angle, WithinAbs(360., 1.));
}

TEST_CASE("Clockwise arcs are linearized onto their circle", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.center        = Eigen::Vector2d(-50., -50.);
    cfg.has_tilt_axis = false;
    const PolarKinematics kin(cfg);

    // Quarter circle of radius 10 around (10, 0), clockwise from (0, 0) to (10, 10).
    const std::string   cartesian = "M83\nG1 X0 Y0 Z1 F600\nG2 X10 Y10 I10 J0 E1\n";
    std::istringstream  in(cartesian);
    std::ostringstream  out;
    PolarGCodeConverter converter(cfg);
    converter.process(in, out);

    CHECK(converter.stats().arcs_linearized == 1);
    CHECK(out.str().find(" B") == std::string::npos);
    const Replayed r = replay(out.str(), kin);
    REQUIRE(r.poses.size() > 3);
    for (size_t i = 1; i < r.poses.size(); ++i)
        CHECK_THAT((r.poses[i].tip.head<2>() - Eigen::Vector2d(10., 0.)).norm(), WithinAbs(10., 1e-3));
    CHECK_THAT(r.e_total, WithinAbs(1., 1e-4));
}

TEST_CASE("Tilt beyond the axis travel is printed at the limit", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.center   = Eigen::Vector2d(0., 0.);
    cfg.min_tilt = -20.;
    cfg.max_tilt = 30.;
    const PolarKinematics kin(cfg);

    // On the +X side (positive radius) an outward lean is a positive machine tilt.
    const MachinePose outward = kin.to_machine(pose(50., 0., 5., 40.), nullptr);
    CHECK(outward.tilt_limited);
    CHECK_THAT(outward.tilt, WithinAbs(30., 1e-9));
    const MachinePose inward = kin.to_machine(pose(50., 0., 5., -25.), nullptr);
    CHECK(inward.tilt_limited);
    CHECK_THAT(inward.tilt, WithinAbs(-20., 1e-9));
    const MachinePose within = kin.to_machine(pose(50., 0., 5., 15.), nullptr);
    CHECK_FALSE(within.tilt_limited);
    CHECK_THAT(within.tilt, WithinAbs(15., 1e-9));

    std::istringstream  in("M83\nG1 X50 Y0 Z5 B0 F600\nG1 X60 Y0 B40 E1\n");
    std::ostringstream  out;
    PolarGCodeConverter converter(cfg);
    converter.process(in, out);
    CHECK(converter.stats().tilt_limited > 0);
    double             max_b = -1e9;
    std::istringstream lines(out.str());
    std::string        line;
    while (std::getline(lines, line))
        if (line.rfind("G1", 0) == 0)
            if (const size_t at = line.find(" B"); at != std::string::npos)
                max_b = std::max(max_b, std::stod(line.substr(at + 2)));
    CHECK_THAT(max_b, WithinAbs(30., 1e-6));
}

TEST_CASE("A reversed tilt axis gets the negated angle", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.tilt_sign = -1.;
    const PolarKinematics kin(cfg);
    // Leaning outward on the +X side: positive toward +radius, so negative on a reversed axis.
    CHECK_THAT(kin.to_machine(pose(50., 0., 5., 30.), nullptr).tilt, WithinAbs(-30., 1e-9));
}

TEST_CASE("The machine end block runs in units per minute and radius travel is checked", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    cfg.max_travel_radius = 15.;
    std::istringstream  in("M83\n; MACHINE_START_GCODE_END\nG1 X10 Y0 Z1 F600\nG1 X20 Y0 E1\n; MACHINE_END_GCODE_START\nG91\nG1 Z10 F600\n");
    std::ostringstream  out;
    PolarGCodeConverter converter(cfg);
    converter.process(in, out);

    const std::string gcode = out.str();
    const size_t      end   = gcode.find("MACHINE_END_GCODE_START");
    REQUIRE(end != std::string::npos);
    CHECK(gcode.find("G93", 0) < end);
    CHECK(gcode.find("G94", end) < gcode.find("G1 Z10", end));
    CHECK(converter.stats().radius_outside > 0);
    CHECK_THAT(converter.stats().max_radius, WithinAbs(20., 1e-9));
}

TEST_CASE("Each input line is recorded with its last output line and the machine pose after it", "[PolarKinematics]")
{
    PolarKinematicsConfig cfg;
    const std::vector<std::string> input { "M83", "; MACHINE_START_GCODE_END", "G1 X10 Y0 Z1 F600", "G1 X0 Y10 E1 B20", "M106 S0" };
    std::string joined;
    for (const std::string &l : input)
        joined += l + "\n";
    std::istringstream  in(joined);
    std::ostringstream  out;
    PolarGCodeConverter converter(cfg);
    converter.process(in, out);

    std::vector<std::string> output;
    std::istringstream       lines(out.str());
    for (std::string line; std::getline(lines, line);)
        output.push_back(line);
    const std::vector<PolarGCodeConverter::LineRecord> &records = converter.line_records();
    REQUIRE(records.size() == input.size());
    // Line numbers count from 1, and the output starts with a header.
    CHECK(output[records[0].out_lines - 1] == "M83");
    CHECK_FALSE(records[1].posed);
    // A quarter turn about the axis: subdivided, the last output line of the move ends on it.
    const PolarGCodeConverter::LineRecord &turn = records[3];
    REQUIRE(turn.posed);
    CHECK(turn.out_lines > records[2].out_lines + 1);
    CHECK_THAT(turn.angle, WithinAbs(90., 1e-4));
    CHECK_THAT(turn.radius, WithinAbs(10., 1e-4));
    CHECK_THAT(turn.tilt, WithinAbs(20., 1e-4));
    const std::string &last = output[turn.out_lines - 1];
    CHECK_THAT(word(last, 'C', 0.), WithinAbs(90., 1e-3));
    CHECK_THAT(word(last, 'B', 0.), WithinAbs(20., 1e-3));
    // A line after the move keeps the pose.
    CHECK(output[records[4].out_lines - 1] == "M106 S0");
    CHECK_THAT(records[4].angle, WithinAbs(90., 1e-4));
}
