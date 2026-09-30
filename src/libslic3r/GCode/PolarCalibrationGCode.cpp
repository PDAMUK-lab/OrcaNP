#include "PolarCalibrationGCode.hpp"

#include "../BoundingBox.hpp"
#include "../Flow.hpp"
#include "../GCode.hpp"
#include "../NonPlanar/GCodeWords.hpp"
#include "../Print.hpp"
#include "GCodeProcessor.hpp"

#include <algorithm>
#include <sstream>

namespace Slic3r {

bool is_polar_calibration(CalibMode mode)
{
    return mode == CalibMode::Calib_Polar_Backlash || mode == CalibMode::Calib_Polar_Tilt || mode == CalibMode::Calib_Polar_Speed;
}

NonPlanar::TestParams polar_test_params(const DynamicPrintConfig &config)
{
    NonPlanar::TestParams p;
    const Pointfs        &area = config.option<ConfigOptionPoints>("printable_area")->values;
    p.centre                   = BoundingBoxf(area).center();
    p.bed_radius               = NonPlanar::polar_bed_radius(std::vector<Vec2d>(area.begin(), area.end()), p.centre);
    p.line_width               = 1.125 * config.option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    p.first_layer_height       = config.opt_float("initial_layer_print_height");
    p.layer_height             = config.opt_float("layer_height");
    p.first_layer_speed        = config.option<ConfigOptionFloatsNullable>("initial_layer_speed")->get_at(0);
    p.print_speed              = config.option<ConfigOptionFloatsNullable>("outer_wall_speed")->get_at(0);
    p.travel_speed             = config.option<ConfigOptionFloatsNullable>("travel_speed")->get_at(0);
    return p;
}

double polar_speed_test_max_linear_speed(const DynamicPrintConfig &config)
{
    const NonPlanar::TestParams p   = polar_test_params(config);
    const double                mvs = config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(0);
    return std::min(150., mvs > 0. ? mvs / (p.line_width * p.first_layer_height) : 150.);
}

std::string polar_calibration_gcode(GCode &gcodegen, const Print &print)
{
    // The calibration parameters: the tilt test's tilt (start), the rotation speed test's slowest
    // and fastest ring (start, end, degrees per second) and speed along them (step, mm/s).
    const Calib_Params         &cp = print.calib_params();
    const NonPlanar::TestParams p  = polar_test_params(print.full_print_config());
    NonPlanar::TestPrint        test;
    switch (cp.mode) {
    case CalibMode::Calib_Polar_Backlash: test = NonPlanar::backlash_print(p, NonPlanar::backlash_test(p)); break;
    case CalibMode::Calib_Polar_Tilt: test = NonPlanar::tilt_pivot_print(p, NonPlanar::tilt_pivot_test(p, cp.start)); break;
    case CalibMode::Calib_Polar_Speed:
        test = NonPlanar::rotation_speed_print(p, NonPlanar::rotation_speed_test(cp.start, cp.end, cp.step));
        break;
    default: return {};
    }

    const FullPrintConfig &c      = gcodegen.config();
    GCodeWriter           &writer = gcodegen.writer();
    const double filament_area = M_PI * std::pow(0.5 * c.filament_diameter.get_at(0), 2);
    const double flow_ratio    = c.print_flow_ratio.value * c.filament_flow_ratio.get_at(0);
    const double nozzle        = c.nozzle_diameter.get_at(0);
    const bool   tilt_axis     = c.polar_kinematics.value && c.polar_tilt_axis.value;
    const char   tilt_letter   = c.polar_axis_names.value.size() == 3 ? c.polar_axis_names.value[2] : 'B';
    const Vec3d  z_offset(0., 0., c.z_offset.value);

    std::ostringstream gcode;
    double             tilt = 0., height = p.first_layer_height;
    bool               retracted = false;
    size_t             layer     = 0;
    for (size_t i = 0; i < test.moves.size(); ++ i) {
        for (; layer < test.layers.size() && test.layers[layer].first_move == i; ++ layer) {
            const NonPlanar::TestLayer &l = test.layers[layer];
            height                        = l.height;
            gcode << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) << "\n";
            gcode << (print.is_BBL_printer() ? "; Z_HEIGHT: " : ";Z:") << l.z + z_offset.z() << "\n";
            gcode << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) << l.height << "\n";
            if (layer == 1)
                gcode << writer.set_fan(unsigned(std::clamp(c.fan_max_speed.get_at(0), 0., 100.)));
        }
        const NonPlanar::TestMove &m  = test.moves[i];
        const Vec3d                to = m.to + z_offset;
        if (tilt_axis && std::abs(m.tilt - tilt) > 1e-9) {
            // The tilt word on its own: the polar conversion tilts the nozzle where it is.
            gcode << "G1 " << tilt_letter << NonPlanar::GCodeWords::number(m.tilt, 3) << "\n";
            tilt = m.tilt;
        }
        if (m.extrude) {
            if (retracted) {
                gcode << writer.unretract();
                retracted = false;
            }
            const double length = (to - writer.get_position()).norm();
            const double e      = Flow(float(m.width), float(height), float(nozzle)).mm3_per_mm() * flow_ratio / filament_area;
            gcode << writer.set_speed(m.speed * 60.);
            gcode << writer.extrude_to_xyz(to, e * length);
        } else {
            if (! retracted && (to - writer.get_position()).head<2>().norm() > 2.) {
                gcode << writer.retract();
                retracted = true;
            }
            gcode << writer.travel_to_xyz(to);
        }
    }
    if (retracted)
        gcode << writer.unretract();
    return ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) + "Outer wall\n" + gcode.str();
}

} // namespace Slic3r
