#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/PolarCalibrationGCode.hpp"
#include "libslic3r/Print.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;
using Catch::Matchers::WithinAbs;

namespace {

DynamicPrintConfig config_with(std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "printable_area", "0x0,200x0,200x200,0x200" }, { "use_relative_e_distances", true },
                                    { "layer_change_gcode", "G92 E0" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" },
                                    { "layer_height", 0.2 }, { "initial_layer_print_height", 0.2 } });
    config.set_deserialize_strict(items);
    return config;
}

// A word's value on a G-code line, or NaN.
double word(const std::string &line, char letter)
{
    const std::string code = line.substr(0, line.find(';'));
    const size_t      i    = code.find(std::string(" ") + letter);
    return i == std::string::npos ? std::nan("") : std::stod(code.substr(i + 2));
}

// The lines after the machine start G-code.
std::vector<std::string> body(const std::string &gcode)
{
    std::vector<std::string> out;
    std::istringstream       in(gcode.substr(gcode.find("MACHINE_START_GCODE_END")));
    for (std::string line; std::getline(in, line);) {
        if (line.find("MACHINE_END_GCODE_START") != std::string::npos)
            break;
        out.emplace_back(line);
    }
    return out;
}

// G-code for a small stand-in cube printed as the polar calibration `mode`.
std::string polar_test(CalibMode mode, double start, double end, double step, std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = config_with({ { "polar_kinematics", true } });
    config.set_deserialize_strict(items);
    Print print;
    Model model;
    init_print({ make_cube(5., 5., 1.) }, print, model, config);
    Calib_Params params;
    params.mode  = mode;
    params.start = start;
    params.end   = end;
    params.step  = step;
    print.set_calib_params(params);
    return gcode(print);
}

} // namespace

TEST_CASE("Axis skew shears the print's moves but not the machine start G-code", "[Calibration][Skew]")
{
    // The same cube with and without 1 degree of XY skew: each move goes back along X by its Y
    // times tan 1 degree.
    const std::string plain = slice({ TestMesh::cube_20x20x20 }, config_with({})),
                      skewed = slice({ TestMesh::cube_20x20x20 }, config_with({ { "skew_xy", 1. } }));
    const std::vector<std::string> a = body(plain), b = body(skewed);
    REQUIRE(a.size() == b.size());
    size_t checked = 0;
    for (size_t i = 0; i < a.size(); ++ i)
        if (a[i].rfind("G1 X", 0) == 0 && a[i].find(" E") != std::string::npos) {
            DYNAMIC_SECTION("line " << i) {
                CHECK_THAT(word(b[i], 'X'), WithinAbs(word(a[i], 'X') - word(a[i], 'Y') * std::tan(M_PI / 180.), 2e-3));
                CHECK_THAT(word(b[i], 'Y'), WithinAbs(word(a[i], 'Y'), 2e-3));
            }
            ++ checked;
        }
    CHECK(checked > 10);
    // Without skew nothing changes.
    CHECK(skewed.find("axis skew compensation") != std::string::npos);
    CHECK(plain.find("axis skew compensation") == std::string::npos);
}

TEST_CASE("The polar backlash test prints its fins in machine G-code, layer by layer", "[Calibration][PolarCalibration]")
{
    const std::string g = polar_test(CalibMode::Calib_Polar_Backlash, 0., 0., 0., {});
    // 6 mm of fins in 0.2 mm layers.
    size_t layers = 0;
    for (size_t at = g.find(";LAYER_CHANGE"); at != std::string::npos; at = g.find(";LAYER_CHANGE", at + 1))
        ++ layers;
    CHECK(layers == 30);
    CHECK(g.find("G1 C") != std::string::npos);
}

TEST_CASE("The polar rotation speed test turns the bed faster than the printer's limit", "[Calibration][PolarCalibration]")
{
    // Rings at 100 mm/s from 180 to 1080 degrees per second, on a bed limited to 360.
    const std::string        g = polar_test(CalibMode::Calib_Polar_Speed, 180., 1080., 100., { { "polar_max_rotation_speed", 360 } });
    const std::vector<std::string> lines = body(g);
    double fastest = 0., angle = std::nan("");
    for (const std::string &line : lines) {
        if (line.rfind("G1 C", 0) != 0)
            continue;
        const double c = word(line, 'C'), f = word(line, 'F');
        // Inverse time feed: F is moves per minute.
        if (! std::isnan(angle) && ! std::isnan(f) && line.find(" E") != std::string::npos)
            fastest = std::max(fastest, std::abs(c - angle) * f / 60.);
        angle = c;
    }
    CHECK(fastest > 1000.);
}

TEST_CASE("The polar tilt pivot test tilts the tubes' upper halves both ways", "[Calibration][PolarCalibration]")
{
    const std::string g = polar_test(CalibMode::Calib_Polar_Tilt, 15., 0., 0., { { "polar_tilt_axis", true } });
    double lowest = 0., highest = 0.;
    for (const std::string &line : body(g))
        if (line.rfind("G1 C", 0) == 0 && line.find(" B") != std::string::npos) {
            lowest  = std::min(lowest, word(line, 'B'));
            highest = std::max(highest, word(line, 'B'));
        }
    CHECK_THAT(lowest, WithinAbs(-15., 1e-3));
    CHECK_THAT(highest, WithinAbs(15., 1e-3));
}
