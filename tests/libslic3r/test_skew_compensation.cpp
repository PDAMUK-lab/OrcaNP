#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/SkewCompensation.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// Where a machine whose axes are skewed by `s` puts the nozzle for axis positions `p`: moving
// along Y also moves it along X, and along Z moves it along X and Y.
Vec3d skewed_machine(const SkewCorrection &s, const Vec3d &p)
{
    return { p.x() + p.y() * s.xy + p.z() * s.xz, p.y() + p.z() * s.yz, p.z() };
}

std::vector<std::string> lines(const std::string &text)
{
    std::vector<std::string> out;
    std::istringstream       in(text);
    for (std::string l; std::getline(in, l);)
        out.emplace_back(l);
    return out;
}

double word(const std::string &line, char letter)
{
    const size_t i = line.find(std::string(" ") + letter);
    return i == std::string::npos ? std::nan("") : std::stod(line.substr(i + 2));
}

} // namespace

TEST_CASE("The skew correction sends a skewed machine's nozzle where the G-code means", "[SkewCompensation]")
{
    const SkewCorrection s = SkewCorrection::from_degrees(0.4, -0.3, 0.2);
    for (const Vec3d &p : { Vec3d(0., 0., 0.), Vec3d(120., 80., 0.2), Vec3d(-30., 200., 150.) }) {
        const Vec3d reached = skewed_machine(s, s.apply(p));
        DYNAMIC_SECTION("at " << p.transpose()) {
            CHECK_THAT(reached.x(), WithinAbs(p.x(), 1e-9));
            CHECK_THAT(reached.y(), WithinAbs(p.y(), 1e-9));
            CHECK_THAT(reached.z(), WithinAbs(p.z(), 1e-9));
        }
    }
}

TEST_CASE("Only the print's moves are sheared, arcs straightened, and the line counts kept", "[SkewCompensation]")
{
    const SkewCorrection s = SkewCorrection::from_degrees(1., 0., 0.);
    const std::string    in = "G1 X10 Y100 F3000\n"
                              "; MACHINE_START_GCODE_END\n"
                              "G1 Z0.2\n"
                              "G1 X50 Y50 E1 ; wall\n"
                              "G2 X60 Y50 I5 J0 E2\n"
                              "; MACHINE_END_GCODE_START\n"
                              "G1 X0 Y200\n";
    SkewGCodeConverter c(s, BoundingBoxf(Vec2d(0., 0.), Vec2d(250., 250.)));
    std::istringstream is(in);
    std::ostringstream os;
    c.process(is, os);
    const std::vector<std::string> out = lines(os.str());

    // The start and end blocks are as they were.
    CHECK(out[1] == "G1 X10 Y100 F3000");
    CHECK(out.back() == "G1 X0 Y200");
    // The body's Z move goes to the sheared X of where the nozzle is, the start's X10 Y100.
    CHECK_THAT(word(out[3], 'X'), WithinAbs(10. - 100. * s.xy, 1e-3));
    // The wall ends sheared, its comment kept.
    CHECK_THAT(word(out[4], 'X'), WithinAbs(50. - 50. * s.xy, 1e-3));
    CHECK(out[4].find("; wall") != std::string::npos);
    CHECK(os.str().find("G2") == std::string::npos);
    CHECK(c.stats().arcs_linearized == 1);
    // One count per input line, the last covering all the output.
    REQUIRE(c.out_lines().size() == 7);
    CHECK(c.out_lines().back() == out.size());
}

TEST_CASE("A relative move is sheared by its own length", "[SkewCompensation]")
{
    const SkewCorrection s = SkewCorrection::from_degrees(0., 2., 0.);
    std::istringstream   is("G91\nG1 Z1\nG90\n");
    std::ostringstream   os;
    SkewGCodeConverter   c(s, BoundingBoxf());
    c.process(is, os);
    const std::vector<std::string> out = lines(os.str());
    // Lifting 1 mm on a Z axis leaning toward +X: move back along X as much.
    CHECK_THAT(word(out[2], 'X'), WithinAbs(-std::tan(2. * M_PI / 180.), 1e-3));
    CHECK_THAT(word(out[2], 'Z'), WithinAbs(1., 1e-9));
}

TEST_CASE("Extruding moves sheared off the bed are counted", "[SkewCompensation]")
{
    const SkewCorrection s = SkewCorrection::from_degrees(2., 0., 0.);
    // At Y 200 a 2 degree skew moves X back by 7 mm: X 3 ends beyond the bed's edge at 0.
    std::istringstream is("G1 X50 Y200 Z0.2\nG1 X3 Y200 E1\nG1 X100 Y200 E2\n");
    std::ostringstream os;
    SkewGCodeConverter c(s, BoundingBoxf(Vec2d(0., 0.), Vec2d(250., 250.)));
    c.process(is, os);
    CHECK(c.stats().outside_bed == 1);
}
