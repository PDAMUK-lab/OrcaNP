#include "ToolheadCalibration.hpp"

#include "I18N.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/NonPlanar/ToolheadClearance.hpp"

#include <string>
#include <vector>

namespace Slic3r {
namespace GUI {

using namespace CalibrationKit;

namespace {

// The sides of the toolhead, as seen from the front of the printer.
constexpr const char *SIDES[4] = { "front", "back", "left", "right" };

wxString side_label(int i)
{
    switch (i) {
    case 0: return _L_CONTEXT("Front", "Camera View");
    case 1: return _L_CONTEXT("Back", "Camera View");
    case 2: return _L_CONTEXT("Left", "Camera View");
    default: return _L_CONTEXT("Right", "Camera View");
    }
}

NonPlanar::ToolheadClearance current_toolhead()
{
    const DynamicPrintConfig    &c = printer_config();
    NonPlanar::ToolheadClearance t;
    t.tip_diameter = c.opt_float("nonplanar_nozzle_tip_diameter");
    t.angle        = c.opt_float("nonplanar_nozzle_clearance_angle");
    t.length       = c.opt_float("nonplanar_nozzle_length");
    t.radius       = c.opt_float("nonplanar_head_radius");
    return t;
}

} // namespace

ToolheadClearanceDialog::ToolheadClearanceDialog(wxWindow *parent, Plater *plater)
    : CalibrationTabsDialog(parent, plater, _L("Toolhead clearance"), "toolhead_clearance",
                            _L("Measures how close to the nozzle tip the toolhead comes, for the toolhead settings that non-planar "
                               "(S4) printing and its clearance check rely on (Printer settings > Basic information > Non-planar "
                               "toolhead). Measure with the feeler blades (1) or the angle wedges (2), whichever suits your toolhead, "
                               "and apply what you measure; then check the settings with the fin test (3) before printing non-planar. "
                               "Each test sets up its print as a new project: print it, then open this again."))
{
    const int gap = FromDIP(10);
    auto *reach_row = new wxBoxSizer(wxHORIZONTAL);
    reach_row->Add(new wxStaticText(this, wxID_ANY, _L("Toolhead reach") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_reach = add_input(this, "reach", _L("mm"));
    reach_row->Add(m_reach, 0, wxALIGN_CENTER_VERTICAL);
    auto *reach_text = new wxStaticText(this, wxID_ANY,
                                        _L("How far the toolhead reaches sideways from the nozzle's centre at its widest, at any "
                                           "height: half its width for a nozzle in the middle."));
    reach_text->Wrap(FromDIP(430));
    reach_row->Add(reach_text, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap);
    GetSizer()->Add(reach_row, 0, wxLEFT | wxRIGHT, gap);

    wxPanel *blades = add_page(_L("1. Feeler blades"),
                               _L("For a toolhead with a wide, nearly flat underside close to the nozzle tip, such as the fan ducts "
                                  "of most Cartesian printers. Its clearance angle is a few degrees, too shallow to judge by eye, so "
                                  "it is measured as a gap over a distance. Print the blades, each numbered with its thickness in mm. "
                                  "With the printer cold, lower the nozzle until it just touches the bed. On each side, slide blades "
                                  "under the outer edge of the toolhead's underside: the thickest that slides under is the gap. "
                                  "Measure the distance from the nozzle's centre to that edge."));
    auto *create_blades = add_button(blades, _L("Create blades"));
    auto *blade_grid    = add_grid(blades, 3);
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, ""));
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, _L("Gap")));
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, _L("Distance")));
    for (int i = 0; i < 4; ++ i) {
        blade_grid->Add(new wxStaticText(blades, wxID_ANY, side_label(i) + ": "), 0, wxALIGN_CENTER_VERTICAL);
        m_gap[i]      = add_input(blades, std::string("gap_") + SIDES[i], _L("mm"));
        m_distance[i] = add_input(blades, std::string("distance_") + SIDES[i], _L("mm"));
        blade_grid->Add(m_gap[i]);
        blade_grid->Add(m_distance[i]);
    }
    auto *apply_blades = add_button(blades, _L("Apply to printer"));

    wxPanel *wedges = add_page(_L("2. Angle wedges"),
                               _L("For a nozzle standing well below the toolhead, such as a long nozzle or a tilting toolhead. Its "
                                  "clearance angle is steep and is read directly. Print the wedges, each numbered with its angle in "
                                  "degrees. With the printer cold and the nozzle touching the bed, slide a wedge along the bed, thin "
                                  "edge first, straight at the nozzle until the edge touches it. If the toolhead touches the slope "
                                  "first, the wedge is too steep for that side. Enter the steepest wedge that reaches the nozzle on "
                                  "each side, and the toolhead reach."));
    auto *create_wedges = add_button(wedges, _L("Create wedges"));
    auto *wedge_grid    = add_grid(wedges, 2);
    wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, ""));
    wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, _L("Steepest wedge")));
    for (int i = 0; i < 4; ++ i) {
        wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, side_label(i) + ": "), 0, wxALIGN_CENTER_VERTICAL);
        m_angle[i] = add_input(wedges, std::string("angle_") + SIDES[i], wxString::FromUTF8(u8"°"));
        wedge_grid->Add(m_angle[i]);
    }
    auto *apply_wedges = add_button(wedges, _L("Apply to printer"));

    // Where the printer puts the nozzle over the plate's centre: G-code positions are the bed's
    // less the extruder offset.
    const Vec2d    nozzle = bed_centre() - printer_config().option<ConfigOptionPoints>("extruder_offset")->get_at(0);
    const wxString centre = is_polar() ? _L("the bed's rotation axis") :
                                         wxString::Format(_L("X %s mm, Y %s mm"), number(nozzle.x(), 1), number(nozzle.y(), 1));
    wxPanel *fins = add_page(_L("3. Fin test"),
                             wxString::Format(_L("Checks the printer's toolhead settings. The blades and wedges measure a few points "
                                                 "on each side; the fin test checks the whole toolhead all round, including any part "
                                                 "lower than the points measured, so do it after either, and after changing the "
                                                 "settings by hand. It prints thin fins around a plate, their tops %s mm below the "
                                                 "toolhead the settings describe and, beyond its radius, taller. Leave the print on "
                                                 "the bed and let the printer cool. Move the nozzle over the plate's centre (%s) and "
                                                 "lower it slowly until it touches the plate. If the toolhead touches or knocks over a "
                                                 "fin, the settings are too generous on that side: measure again, or lower the "
                                                 "clearance angle or raise the toolhead radius, and repeat. If it touches none, the "
                                                 "settings are safe. Take the plate off the bed before homing the printer: homing may "
                                                 "lower the nozzle onto it."),
                                              number(NonPlanar::fin_test_margin, 1), centre));
    m_fin_text = new wxStaticText(fins, wxID_ANY, "");
    fins->GetSizer()->Add(m_fin_text, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *create_fins = add_button(fins, _L("Create fin test"));

    create_blades->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_blades, this);
    create_wedges->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_wedges, this);
    create_fins->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_fin_test, this);
    apply_blades->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_apply_blades, this);
    apply_wedges->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_apply_wedges, this);

    update_fin_text();
    finish();
}

void ToolheadClearanceDialog::update_fin_text()
{
    const NonPlanar::ToolheadClearance t = current_toolhead();
    m_fin_text->SetLabel(wxString::Format(_L("Now: nozzle clearance angle %s degrees, nozzle length %s mm, toolhead radius %s mm."),
                                          number(t.angle, 1), number(t.length, 2), number(t.radius, 1)));
    m_fin_text->GetParent()->Layout();
}

void ToolheadClearanceDialog::on_create_blades(wxCommandEvent &)
{
    remember();
    // Closed, so the gauge can be sliced and printed.
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Toolhead clearance feeler blades"),
                    { { NonPlanar::feeler_blades(NonPlanar::feeler_thicknesses(), layout_width()), Vec2d::Zero(), {} } });
}

void ToolheadClearanceDialog::on_create_wedges(wxCommandEvent &)
{
    remember();
    double reach = 0.;
    if (! read(m_reach, reach) || reach <= 0.)
        reach = current_toolhead().reach();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Toolhead clearance angle wedges"),
                    { { NonPlanar::angle_wedges(NonPlanar::wedge_angles(), reach, layout_width()), Vec2d::Zero(), {} } });
}

void ToolheadClearanceDialog::on_create_fin_test(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Toolhead clearance fin test"),
                    { { NonPlanar::fin_test(current_toolhead(), 2.5 * nozzle_diameter()), Vec2d::Zero(), {} } });
}

namespace {

void apply_toolhead(const NonPlanar::ToolheadClearance &t)
{
    DynamicPrintConfig values;
    values.set_key_value("nonplanar_nozzle_clearance_angle", new ConfigOptionFloat(t.angle));
    values.set_key_value("nonplanar_nozzle_length", new ConfigOptionFloat(t.length));
    values.set_key_value("nonplanar_head_radius", new ConfigOptionFloat(t.radius));
    apply(Preset::TYPE_PRINTER, values);
}

wxString applied(const NonPlanar::ToolheadClearance &t)
{
    return wxString::Format(_L("Nozzle clearance angle %s degrees, nozzle length %s mm, toolhead radius %s mm. Save the printer preset to "
                               "keep them, then check them with the fin test."),
                            number(t.angle, 1), number(t.length, 2), number(t.radius, 1));
}

} // namespace

void ToolheadClearanceDialog::on_apply_blades(wxCommandEvent &)
{
    remember();
    std::vector<NonPlanar::FeelerReading> readings(4);
    for (int i = 0; i < 4; ++ i)
        if (! read(m_gap[i], readings[i].gap) || ! read(m_distance[i], readings[i].distance)) {
            warn(_L("Please input the gap and the distance for every side."));
            return;
        }
    double reach = 0.;
    if (! read(m_reach, reach))
        reach = 0.;
    NonPlanar::ToolheadClearance t;
    std::string                  error;
    if (! NonPlanar::fit_feeler_readings(readings, current_toolhead().tip_diameter, reach, t, error)) {
        warn(from_u8(error));
        return;
    }
    apply_toolhead(t);
    update_fin_text();
    show_result(applied(t));
}

void ToolheadClearanceDialog::on_apply_wedges(wxCommandEvent &)
{
    remember();
    std::vector<double> angles(4);
    double              reach = 0.;
    for (int i = 0; i < 4; ++ i)
        if (! read(m_angle[i], angles[i])) {
            warn(_L("Please input the steepest wedge for every side."));
            return;
        }
    if (! read(m_reach, reach)) {
        warn(_L("Please input the toolhead reach."));
        return;
    }
    NonPlanar::ToolheadClearance t;
    std::string                  error;
    if (! NonPlanar::fit_wedge_readings(angles, current_toolhead().tip_diameter, reach, t, error)) {
        warn(from_u8(error));
        return;
    }
    apply_toolhead(t);
    update_fin_text();
    show_result(applied(t));
}

} // namespace GUI
} // namespace Slic3r
