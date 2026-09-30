#include "PolarPrinterCalibration.hpp"

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "MainFrame.hpp"
#include "Plater.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/CalibrationPrints.hpp"
#include "libslic3r/GCode/PolarCalibrationGCode.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/NonPlanar/PolarKinematics.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Utils.hpp"

#include <cmath>

namespace Slic3r {
namespace GUI {

namespace CP = CalibrationPrints;
using namespace CalibrationKit;

namespace {

wxString mm() { return _L("mm"); }
wxString degrees() { return wxString::FromUTF8(u8"°"); }

// The layers the tests print in.
double test_layer() { return nozzle_diameter() < 0.3 ? 0.1 : 0.2; }

// The printer, process and filament as the tests are printed with, in the test's layers.
DynamicPrintConfig test_config()
{
    DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(test_layer()));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(test_layer()));
    return config;
}

bool has_tilt_axis() { return is_polar() && printer_config().opt_bool("polar_tilt_axis"); }

// What the tests last created were, kept for entering what they measure.
double remembered(const std::string &key, double fallback)
{
    const std::string value = wxGetApp().app_config->get("polar_calibration_" + key);
    return value.empty() ? fallback : string_to_double_decimal_point(value);
}
void remember_value(const std::string &key, double value)
{
    wxGetApp().app_config->set("polar_calibration_" + key, float_to_string_decimal_point(value));
}

double tilt_test_angle()
{
    const DynamicPrintConfig &p = printer_config();
    return NonPlanar::tilt_pivot_test_angle(p.opt_float("nonplanar_nozzle_tip_diameter"), test_layer(),
                                            std::min(-p.opt_float("polar_tilt_min"), p.opt_float("polar_tilt_max")));
}

NonPlanar::RotationSpeedTest planned_speed_test()
{
    const DynamicPrintConfig config = test_config();
    return NonPlanar::plan_rotation_speed_test(config.opt_float("polar_max_rotation_speed"), polar_test_params(config).bed_radius,
                                               polar_speed_test_max_linear_speed(config));
}

// The rotation speed test last created, or the one Create would make.
NonPlanar::RotationSpeedTest speed_test()
{
    const NonPlanar::RotationSpeedTest plan = planned_speed_test();
    return NonPlanar::rotation_speed_test(remembered("speed_slowest", plan.speeds.front()), remembered("speed_fastest", plan.speeds.back()),
                                          remembered("speed_linear", plan.linear_speed));
}

wxString ring_list(const NonPlanar::RotationSpeedTest &t)
{
    wxString out;
    for (size_t i = 0; i < t.speeds.size(); ++ i)
        out += (i == 0 ? "" : ", ") + wxString::Format("%d: %s", int(i + 1), number(t.speeds[i], 0));
    return out;
}

} // namespace

PolarCalibrationDialog::PolarCalibrationDialog(wxWindow *parent, Plater *plater)
    : CalibrationTabsDialog(parent, plater, _L("Polar calibration"), "polar_calibration",
                            _L("Calibrates a polar printer: where its bed's rotation axis is, how its bed sits under the nozzle, the "
                               "play in the bed's drive, its tilting nozzle, and how fast its bed may turn. The results go into the "
                               "printer's settings (Printer settings > Basic information > Polar kinematics), which correct the "
                               "machine G-code; save the printer preset to keep them. Each test sets up its print as a new project: "
                               "print it, then open this again to enter what you measure. Start with the radius, then the bed level."))
{
    // 1. Radius
    wxPanel *radius = add_page(_L("1. Radius"),
                               _L("Prints two thin rings about the bed's rotation axis. Measure each ring's outer diameter with "
                                  "calipers: if the rings come out too large, the rotation axis is farther from the head's zero radius "
                                  "than the printer assumes, and too small, nearer. Apply to printer corrects the Radius offset and "
                                  "Radius scale from the two."));
    auto *ring_grid = add_grid(radius, 2);
    auto  row       = [&](wxWindow *page, wxFlexGridSizer *grid, const wxString &label, TextInput *in) {
        grid->Add(new wxStaticText(page, wxID_ANY, label + ": "), 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(in);
        return in;
    };
    m_inner  = row(radius, ring_grid, _L("Inner ring radius"), add_input(radius, "inner_radius", mm(), "20"));
    m_outer  = row(radius, ring_grid, _L("Outer ring radius"), add_input(radius, "outer_radius", mm(), "40"));
    m_height = row(radius, ring_grid, _L("Height"), add_input(radius, "ring_height", mm(), "3"));
    auto *create_rings = add_button(radius, _L("Create rings"));
    auto *measured     = add_grid(radius, 2);
    m_inner_measured   = row(radius, measured, _L("Inner ring, outer diameter measured"), add_input(radius, "inner_measured", mm()));
    m_outer_measured   = row(radius, measured, _L("Outer ring, outer diameter measured"), add_input(radius, "outer_measured", mm()));
    auto *apply_rings  = add_button(radius, _L("Apply to printer"));

    // 2. Bed level
    wxPanel *bed = add_page(_L("2. Bed level"),
                            _L("Prints two rings about the rotation axis, three layers thick, numbered 1 to 4 outside the outer ring "
                               "and 5 to 8 inside the inner one. Measure each ring's thickness beside each number. A ring prints "
                               "thinner where the bed is higher under the nozzle: a bed not square to its rotation axis, or a radius "
                               "axis not parallel to the bed. Apply to printer sets how much higher the bed is toward +X, toward +Y and "
                               "outward (Bed tilt X, Bed tilt Y, Bed rise outward), and the nozzle is raised as the bed is, so the "
                               "first layer is as thick all round. Print the rings again to check."));
    auto *create_bed = add_button(bed, _L("Create rings"));
    auto *bed_grid   = add_grid(bed, 8);
    for (int i = 0; i < 8; ++ i)
        m_bed[i] = row(bed, bed_grid, wxString::Format("%d", i + 1), add_input(bed, "bed_" + std::to_string(i + 1), mm()));
    auto *apply_bed = add_button(bed, _L("Apply to printer"));

    // 3. Backlash
    wxPanel *backlash = add_page(_L("3. Backlash"),
                                 _L("Prints two pairs of thin fins standing out from the rotation axis, numbered 1 and 2 on the bed "
                                    "beside them. The bed turns the same way before each fin of pair 1, and opposite ways before the "
                                    "two fins of pair 2, so play in the bed's drive moves pair 2's fins apart, and a Bed rotation "
                                    "backlash setting taking up more play than there is moves them together. Measure across each pair "
                                    "at the fins' outer ends, outside to outside, and apply. Pairs measuring the same mean the play is "
                                    "taken up. Leave the setting at 0 if the firmware takes the play up (RepRapFirmware M425)."));
    auto *create_backlash = add_button(backlash, _L("Create fins"));
    auto *pair_grid       = add_grid(backlash, 2);
    m_pair_1              = row(backlash, pair_grid, _L("Pair 1"), add_input(backlash, "pair_1", mm()));
    m_pair_2              = row(backlash, pair_grid, _L("Pair 2"), add_input(backlash, "pair_2", mm()));
    auto *apply_backlash  = add_button(backlash, _L("Apply to printer"));

    // 4. Tilt pivot
    const double tilt = remembered("tilt_angle", tilt_test_angle());
    wxPanel     *tilt_page =
        add_page(_L("4. Tilt pivot"),
                 has_tilt_axis() ?
                     wxString::Format(_L("For a tilting nozzle. Prints two tubes about the rotation axis, their lower halves with the "
                                         "nozzle vertical and their upper halves with it tilted %s degrees: the inner tube leaning "
                                         "outward, the outer one inward. A Tilt pivot distance off moves the tilted halves' walls in or "
                                         "out, and a Tilt offset off (the nozzle not vertical when it should be) moves them up or down. "
                                         "Measure each half's outer diameter and each tube's height, and apply: this sets both, and "
                                         "moves the Radius offset with the tilt offset so that vertical printing stays where it was."),
                                      number(tilt, 0)) :
                     _L("For a printer with a tilting nozzle (Printer settings > Basic information > Polar kinematics: Tilt axis). "
                        "This printer has none."));
    auto *create_tilt = add_button(tilt_page, _L("Create tubes"));
    auto *tube_grid   = add_grid(tilt_page, 4);
    tube_grid->Add(new wxStaticText(tilt_page, wxID_ANY, ""));
    for (const wxString &column : { _L("Lower half"), _L("Upper half"), _L("Height") })
        tube_grid->Add(new wxStaticText(tilt_page, wxID_ANY, column));
    const char *tube_keys[2] = { "inner", "outer" };
    for (int t = 0; t < 2; ++ t) {
        tube_grid->Add(new wxStaticText(tilt_page, wxID_ANY, (t == 0 ? _L("Inner tube") : _L("Outer tube")) + ": "), 0, wxALIGN_CENTER_VERTICAL);
        m_tubes[2 * t]     = add_input(tilt_page, std::string(tube_keys[t]) + "_lower", mm());
        m_tubes[2 * t + 1] = add_input(tilt_page, std::string(tube_keys[t]) + "_upper", mm());
        m_tubes[4 + t]     = add_input(tilt_page, std::string(tube_keys[t]) + "_height", mm());
        tube_grid->Add(m_tubes[2 * t]);
        tube_grid->Add(m_tubes[2 * t + 1]);
        tube_grid->Add(m_tubes[4 + t]);
    }
    auto *apply_tilt = add_button(tilt_page, _L("Apply to printer"));
    create_tilt->GetOK()->Enable(has_tilt_axis());
    apply_tilt->GetOK()->Enable(has_tilt_axis());

    // 5. Rotation speed
    const NonPlanar::RotationSpeedTest speeds = speed_test();
    wxPanel *speed = add_page(_L("5. Rotation speed"),
                              wxString::Format(_L("Prints rings about the rotation axis in one layer, outermost first, each at %s mm/s "
                                                  "along it, so the bed turns faster for each smaller ring. Each ring stops short of "
                                                  "a full turn, leaving a gap: the gaps line up until the bed turned too fast for its "
                                                  "motor and lost steps, which turns every ring after. Enter the last ring, counting "
                                                  "from the outside, whose gap lines up with those outside it, and apply: it sets "
                                                  "Maximum bed rotation speed, which slows moves that would turn the bed faster (with "
                                                  "inverse time feed). The firmware's own speed limit for the bed must allow the "
                                                  "fastest ring, or it slows the bed and every ring passes. Ring speeds, degrees per "
                                                  "second: %s."),
                                               number(speeds.linear_speed, 0), ring_list(speeds)));
    auto *create_speed = add_button(speed, _L("Create rings"));
    auto *speed_grid   = add_grid(speed, 2);
    m_last_ring        = row(speed, speed_grid, _L("Last ring lined up"), add_input(speed, "last_ring", ""));
    auto *apply_speed  = add_button(speed, _L("Apply to printer"));

    create_rings->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_create_rings, this);
    apply_rings->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_apply_rings, this);
    create_bed->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_create_bed, this);
    apply_bed->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_apply_bed, this);
    create_backlash->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_create_backlash, this);
    apply_backlash->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_apply_backlash, this);
    create_tilt->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_create_tilt, this);
    apply_tilt->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_apply_tilt, this);
    create_speed->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_create_speed, this);
    apply_speed->GetOK()->Bind(wxEVT_BUTTON, &PolarCalibrationDialog::on_apply_speed, this);
    finish();
}

// --- 1. Radius -------------------------------------------------------------------------------

bool PolarCalibrationDialog::read_rings(double &r1, double &r2, double &height)
{
    if (! read(m_inner, r1) || ! read(m_outer, r2) || ! read(m_height, height) || r1 < 5. || r2 < r1 + 5. || height <= 0.) {
        warn(_L("Please input valid values: the inner ring at least 5 mm in radius, the outer ring at least 5 mm larger, and a "
                "height."));
        return false;
    }
    return true;
}

void PolarCalibrationDialog::on_create_rings(wxCommandEvent &)
{
    double r1, r2, height;
    if (! read_rings(r1, r2, height))
        return;
    remember();
    EndModal(wxID_OK);
    // Walls two lines thick, printed as one loop each side: the outer one's edge where the ring was
    // designed.
    const double         wall = 2.5 * nozzle_diameter();
    indexed_triangle_set its;
    for (double r : { r1, r2 }) {
        auto ring = [](double radius) {
            Polygon out;
            for (const Vec3d &q : CP::circle(Vec3d::Zero(), radius, 360))
                out.points.emplace_back(scaled(q.x()), scaled(q.y()));
            return out;
        };
        ExPolygon annulus(ring(r));
        Polygon   hole = ring(r - wall);
        hole.reverse();
        annulus.holes.emplace_back(std::move(hole));
        CP::append_slab(its, annulus, 0., height);
    }
    TestObject rings { std::move(its), Vec2d::Zero(), {} };
    rings.config.set_key_value("wall_loops", new ConfigOptionInt(1));
    rings.config.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    rings.config.set_key_value("bottom_shell_layers", new ConfigOptionInt(0));
    rings.config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    open_test_print(m_plater, _L("Polar alignment rings"), { rings });
}

void PolarCalibrationDialog::on_apply_rings(wxCommandEvent &)
{
    remember();
    double r1, r2, height, d1, d2;
    if (! read_rings(r1, r2, height))
        return;
    if (! read(m_inner_measured, d1) || ! read(m_outer_measured, d2) || d1 <= 0. || d2 <= d1) {
        warn(_L("Please input both measured diameters, the outer ring's the larger."));
        return;
    }
    const DynamicPrintConfig             &printer = printer_config();
    const NonPlanar::RadiusCalibration    c = NonPlanar::solve_radius_calibration(r1, d1, r2, d2, printer.opt_float("polar_radius_offset"),
                                                                                  printer.opt_float("polar_radius_scale") / 100.);
    if (c.scale < 0.5 || c.scale > 1.5) {
        warn(_L("These measurements give a radius scale beyond 50 to 150 %: please check them."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("polar_radius_offset", new ConfigOptionFloat(c.offset));
    values.set_key_value("polar_radius_scale", new ConfigOptionFloat(100. * c.scale));
    apply(Preset::TYPE_PRINTER, values);
    show_result(wxString::Format(_L("Radius offset %s mm, radius scale %s %%. Save the printer preset to keep them."), number(c.offset, 3),
                                 number(100. * c.scale, 3)));
}

// --- 2. Bed level ----------------------------------------------------------------------------

void PolarCalibrationDialog::on_create_bed(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    const NonPlanar::BedLevelRings rings = NonPlanar::bed_level_rings(polar_test_params(test_config()).bed_radius, 3. * test_layer());
    TestObject                     test { NonPlanar::bed_level_mesh(rings), Vec2d::Zero(), {} };
    // Concentric, so the nozzle follows the rings round.
    for (const char *key : { "top_surface_pattern", "bottom_surface_pattern", "internal_solid_infill_pattern" })
        test.config.set_key_value(key, new ConfigOptionEnum<InfillPattern>(ipConcentric));
    open_test_print(m_plater, _L("Bed level rings"), { test });
}

void PolarCalibrationDialog::on_apply_bed(wxCommandEvent &)
{
    remember();
    std::vector<double> values;
    if (! read_all({ m_bed.begin(), m_bed.end() }, values, _L("the rings' thickness at all eight numbers")))
        return;
    const NonPlanar::BedLevelRings rings = NonPlanar::bed_level_rings(polar_test_params(test_config()).bed_radius, 3. * test_layer());
    std::array<double, 8>          thickness;
    std::copy(values.begin(), values.end(), thickness.begin());
    NonPlanar::BedLevelFit fit;
    std::string            error;
    if (! NonPlanar::fit_bed_level(rings, thickness, fit, error)) {
        warn(from_u8(error));
        return;
    }
    // Printed with the correction already set, the rings show what is left; the settings are per 100 mm.
    const DynamicPrintConfig &printer = printer_config();
    const double x = printer.opt_float("polar_bed_tilt_x") + 100. * fit.tilt_x, y = printer.opt_float("polar_bed_tilt_y") + 100. * fit.tilt_y,
                 cone = printer.opt_float("polar_bed_cone") + 100. * fit.cone;
    if (std::abs(x) > 5. || std::abs(y) > 5. || std::abs(cone) > 5.) {
        warn(_L("These measurements give a bed more than 5 mm higher or lower 100 mm out: please check them, and level the bed."));
        return;
    }
    DynamicPrintConfig config;
    config.set_key_value("polar_bed_tilt_x", new ConfigOptionFloat(x));
    config.set_key_value("polar_bed_tilt_y", new ConfigOptionFloat(y));
    config.set_key_value("polar_bed_cone", new ConfigOptionFloat(cone));
    apply(Preset::TYPE_PRINTER, config);
    show_result(wxString::Format(_L("Bed tilt X %s, bed tilt Y %s, bed rise outward %s mm per 100 mm. Save the printer preset to keep "
                                    "them, and print the rings again to check."),
                                 number(x, 3), number(y, 3), number(cone, 3)));
}

// --- 3. Backlash -----------------------------------------------------------------------------

void PolarCalibrationDialog::open_gcode_test(const wxString &name, const Calib_Params &params)
{
    remember();
    EndModal(wxID_OK);
    if (m_plater->new_project(false, false, name) == wxID_CANCEL)
        return;
    wxGetApp().mainframe->select_tab(TAB_ID_PREPARE);
    // A stand-in object, as for the pressure advance lines: the test's G-code replaces its own.
    if (! m_plater->add_model(false, Slic3r::resources_dir() + "/calib/pressure_advance/pressure_advance_test.drc"))
        return;
    m_plater->model().objects.back()->config.set_key_value("s4_enabled", new ConfigOptionBool(false));
    DynamicPrintConfig layers;
    layers.set_key_value("layer_height", new ConfigOptionFloat(test_layer()));
    layers.set_key_value("initial_layer_print_height", new ConfigOptionFloat(test_layer()));
    apply(Preset::TYPE_PRINT, layers);
    m_plater->fff_print().set_calib_params(params);
    m_plater->changed_objects({ 0 });
}

void PolarCalibrationDialog::on_create_backlash(wxCommandEvent &)
{
    Calib_Params params;
    params.mode = CalibMode::Calib_Polar_Backlash;
    open_gcode_test(_L("Backlash fins"), params);
}

void PolarCalibrationDialog::on_apply_backlash(wxCommandEvent &)
{
    remember();
    std::vector<double> w;
    if (! read_all({ m_pair_1, m_pair_2 }, w, _L("both pairs' widths")))
        return;
    const NonPlanar::BacklashTest test = NonPlanar::backlash_test(polar_test_params(test_config()));
    const double                  left = NonPlanar::fit_backlash(test, w[0], w[1]);
    if (std::abs(w[0] - w[1]) > 5. || std::abs(left) > 5.) {
        warn(_L("The pairs should measure within a few tenths of a millimetre of each other: please check the measurements."));
        return;
    }
    const double old = printer_config().opt_float("polar_rotation_backlash"), v = std::max(0., old + left);
    DynamicPrintConfig values;
    values.set_key_value("polar_rotation_backlash", new ConfigOptionFloat(v));
    apply(Preset::TYPE_PRINTER, values);
    wxString text = wxString::Format(_L("Bed rotation backlash %s degrees. Save the printer preset to keep it."), number(v, 3));
    if (old + left < 0.)
        text += " " + _L("The fins show less play than none: check that the firmware does not take the play up as well.");
    show_result(text);
}

// --- 4. Tilt pivot ---------------------------------------------------------------------------

void PolarCalibrationDialog::on_create_tilt(wxCommandEvent &)
{
    Calib_Params params;
    params.mode  = CalibMode::Calib_Polar_Tilt;
    params.start = tilt_test_angle();
    remember_value("tilt_angle", params.start);
    open_gcode_test(_L("Tilt pivot tubes"), params);
}

void PolarCalibrationDialog::on_apply_tilt(wxCommandEvent &)
{
    remember();
    std::vector<double> v;
    if (! read_all({ m_tubes.begin(), m_tubes.end() }, v, _L("both tubes' diameters and heights")))
        return;
    const NonPlanar::TiltPivotTest test = NonPlanar::tilt_pivot_test(polar_test_params(test_config()),
                                                                     remembered("tilt_angle", tilt_test_angle()));
    NonPlanar::TiltPivotReading reading;
    reading.inner_lower  = v[0];
    reading.inner_upper  = v[1];
    reading.outer_lower  = v[2];
    reading.outer_upper  = v[3];
    reading.inner_height = v[4];
    reading.outer_height = v[5];
    const DynamicPrintConfig &printer = printer_config();
    NonPlanar::TiltPivotFit   printed_with { printer.opt_float("polar_tilt_pivot_length"), printer.opt_float("polar_tilt_offset"),
                                           printer.opt_float("polar_radius_offset") };
    NonPlanar::TiltPivotFit   fit;
    std::string               error;
    if (! NonPlanar::fit_tilt_pivot(test, reading, printed_with, printer.opt_bool("polar_reverse_tilt") ? -1. : 1., fit, error)) {
        warn(from_u8(error));
        return;
    }
    if (std::abs(fit.tilt_offset) > 10.) {
        warn(_L("These measurements give a tilt offset of more than 10 degrees: please check them."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("polar_tilt_pivot_length", new ConfigOptionFloat(fit.pivot_length));
    values.set_key_value("polar_tilt_offset", new ConfigOptionFloat(fit.tilt_offset));
    values.set_key_value("polar_radius_offset", new ConfigOptionFloat(fit.radius_offset));
    apply(Preset::TYPE_PRINTER, values);
    show_result(wxString::Format(_L("Tilt pivot distance %s mm, tilt offset %s degrees, radius offset %s mm. Save the printer preset to "
                                    "keep them, and print the tubes again to check."),
                                 number(fit.pivot_length, 2), number(fit.tilt_offset, 2), number(fit.radius_offset, 3)));
}

// --- 5. Rotation speed -----------------------------------------------------------------------

void PolarCalibrationDialog::on_create_speed(wxCommandEvent &)
{
    const NonPlanar::RotationSpeedTest t = planned_speed_test();
    Calib_Params                       params;
    params.mode  = CalibMode::Calib_Polar_Speed;
    params.start = t.speeds.front();
    params.end   = t.speeds.back();
    params.step  = t.linear_speed;
    remember_value("speed_slowest", params.start);
    remember_value("speed_fastest", params.end);
    remember_value("speed_linear", params.step);
    open_gcode_test(_L("Rotation speed rings"), params);
}

void PolarCalibrationDialog::on_apply_speed(wxCommandEvent &)
{
    remember();
    const NonPlanar::RotationSpeedTest t = speed_test();
    double                             ring = 0.;
    if (! read(m_last_ring, ring) || ring < 1. || ring > double(t.speeds.size()) || ring != std::floor(ring)) {
        warn(wxString::Format(_L("Please input the ring's number, 1 to %d."), int(t.speeds.size())));
        return;
    }
    const double v = std::round(t.speeds[size_t(ring) - 1]);
    DynamicPrintConfig values;
    values.set_key_value("polar_max_rotation_speed", new ConfigOptionFloat(v));
    apply(Preset::TYPE_PRINTER, values);
    wxString text = wxString::Format(_L("Maximum bed rotation speed %s degrees per second. Save the printer preset to keep it."), number(v, 0));
    if (size_t(ring) == t.speeds.size())
        text += " " + _L("Every ring lined up: the bed may turn faster still. Create the rings again to test faster.");
    show_result(text);
}

} // namespace GUI
} // namespace Slic3r
