#pragma once

// G-code for the polar printer tests that are paths rather than models (Calibration > Polar
// calibration: backlash, tilt pivot, rotation speed). Written in the part frame like any print,
// with the tilt as the tilt axis letter, and turned into machine G-code with the rest.

#include "../calib.hpp"
#include "../NonPlanar/PolarCalibration.hpp"

#include <string>

namespace Slic3r {

class DynamicPrintConfig;
class GCode;
class Print;

bool is_polar_calibration(CalibMode mode);

// The tests' common parameters from a printer's, a process's and a filament's settings.
NonPlanar::TestParams polar_test_params(const DynamicPrintConfig &config);
// The fastest the rotation speed test may go along its rings: its one layer within the filament's
// maximum volumetric speed, and at most 150 mm/s.
double polar_speed_test_max_linear_speed(const DynamicPrintConfig &config);

// The test `print` is set up for (its calibration parameters), in place of its objects.
std::string polar_calibration_gcode(GCode &gcodegen, const Print &print);

} // namespace Slic3r
