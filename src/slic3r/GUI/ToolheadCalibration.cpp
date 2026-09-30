#include "ToolheadCalibration.hpp"

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "MainFrame.hpp"
#include "MsgDialog.hpp"
#include "PartPlate.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/Label.hpp"
#include "Widgets/StateColor.hpp"
#include "Widgets/TabCtrl.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/NonPlanar/ToolheadClearance.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <boost/filesystem.hpp>

#include <wx/simplebook.h>

#include <string>
#include <vector>

namespace Slic3r {
namespace GUI {

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

// What was last entered, kept in the app config: the dialog is opened again to enter what the
// printed gauges measure.
wxString remembered(const std::string &key) { return wxString::FromUTF8(wxGetApp().app_config->get("toolhead_clearance_" + key)); }
void     store(const std::string &key, TextInput *in) { wxGetApp().app_config->set("toolhead_clearance_" + key, in->GetTextCtrl()->GetValue().ToUTF8().data()); }

bool read(TextInput *in, double &value) { return in->GetTextCtrl()->GetValue().ToDouble(&value); }

TextInput *make_input(wxWindow *parent, const wxString &value, const wxString &unit)
{
    auto *in = new TextInput(parent, value, unit, "", wxDefaultPosition, parent->FromDIP(wxSize(90, -1)));
    in->GetTextCtrl()->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
    return in;
}

wxStaticText *make_text(wxWindow *parent, const wxString &text, int wrap)
{
    auto *label = new wxStaticText(parent, wxID_ANY, text);
    label->Wrap(wrap);
    return label;
}

DialogButtons *make_button(wxWindow *parent, const wxString &label)
{
    auto *button = new DialogButtons(parent, { "OK" });
    button->GetOK()->SetLabel(label);
    return button;
}

DynamicPrintConfig &printer_config() { return wxGetApp().preset_bundle->printers.get_edited_preset().config; }

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

const Pointfs &printable_area() { return printer_config().option<ConfigOptionPoints>("printable_area")->values; }

// Width for the gauges' rows: most of a rectangular bed, the square inside a round one.
double gauge_width()
{
    const double width = BoundingBoxf(printable_area()).size().x();
    return printable_area().size() > 8 ? 0.7 * width : 0.9 * width;
}

wxString number(double value, int decimals) { return wxString::FromUTF8(float_to_string_decimal_point(value, decimals)); }

// Opens `its` as a new project called `name`, centred on the bed, planar and without a brim, in
// 0.2 mm layers from the first on (0.1 mm for nozzles under 0.3 mm), so the gauges' heights, whole
// tenths of 0.2 mm, print true.
void open_gauge(Plater *plater, const wxString &name, const indexed_triangle_set &its, const std::string &file)
{
    if (plater->new_project(false, false, name) == wxID_CANCEL)
        return;
    wxGetApp().mainframe->select_tab(TAB_ID_PREPARE);
    const std::string path = (boost::filesystem::temp_directory_path() / file).string();
    if (! its_write_stl_binary(path.c_str(), file.c_str(), its) || ! plater->add_model(false, path))
        return;

    ModelObject *object = plater->model().objects.front();
    const Vec2d  centre = BoundingBoxf(printable_area()).center();
    const Vec3d  origin = plater->get_partplate_list().get_curr_plate()->get_origin();
    object->instances.front()->set_offset(Vec3d(origin.x() + centre.x(), origin.y() + centre.y(), 0.));
    object->ensure_on_bed();
    object->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    object->config.set_key_value("s4_enabled", new ConfigOptionBool(false));

    const double nozzle = printer_config().option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    const double layer  = nozzle < 0.3 ? 0.1 : 0.2;
    auto        *print  = &wxGetApp().preset_bundle->prints.get_edited_preset().config;
    print->set_key_value("layer_height", new ConfigOptionFloat(layer));
    print->set_key_value("initial_layer_print_height", new ConfigOptionFloat(layer));
    print->set_key_value("spiral_mode", new ConfigOptionBool(false));
    plater->changed_objects({ 0 });
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_dirty();
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_ui_from_settings();
}

} // namespace

ToolheadClearanceDialog::ToolheadClearanceDialog(wxWindow *parent, Plater *plater)
    : DPIDialog(parent, wxID_ANY, _L("Toolhead clearance"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE), m_plater(plater)
{
    SetBackgroundColour(*wxWHITE);
    SetForegroundColour(wxColour("#363636"));
    SetFont(Label::Body_14);
    const int wrap = FromDIP(640), gap = FromDIP(10);

    auto *v_sizer = new wxBoxSizer(wxVERTICAL);
    SetSizer(v_sizer);
    v_sizer->Add(make_text(this,
                           _L("Measures how close to the nozzle tip the toolhead comes, for the toolhead settings that non-planar (S4) "
                              "printing and its clearance check rely on (Printer settings > Basic information > Non-planar toolhead). "
                              "Measure with the feeler blades (1) or the angle wedges (2), whichever suits your toolhead, and apply "
                              "what you measure; then check the settings with the fin test (3) before printing non-planar. Each test "
                              "sets up its print as a new project: print it, then open this again."),
                           wrap),
                 0, wxALL, gap);

    auto *reach_row = new wxBoxSizer(wxHORIZONTAL);
    reach_row->Add(new wxStaticText(this, wxID_ANY, _L("Toolhead reach") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_reach = make_input(this, remembered("reach"), _L("mm"));
    reach_row->Add(m_reach, 0, wxALIGN_CENTER_VERTICAL);
    reach_row->Add(make_text(this,
                             _L("How far the toolhead reaches sideways from the nozzle's centre at its widest, at any height: half "
                                "its width for a nozzle in the middle."),
                             FromDIP(430)),
                   0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap);
    v_sizer->Add(reach_row, 0, wxLEFT | wxRIGHT, gap);

    // A tab per test, as in the Preferences dialog.
    auto *tabs = new TabCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                             wxTR_NO_BUTTONS | wxTR_HIDE_ROOT | wxTR_SINGLE | wxTR_NO_LINES | wxBORDER_NONE | wxWANTS_CHARS | wxTR_FULL_ROW_HIGHLIGHT);
    tabs->SetFont(Label::Body_14);
    auto *book = new wxSimplebook(this, wxID_ANY);
    auto  page = [&](const wxString &title) {
        auto *panel = new wxPanel(book);
        panel->SetBackgroundColour(*wxWHITE);
        panel->SetSizer(new wxBoxSizer(wxVERTICAL));
        tabs->AppendItem(title);
        book->AddPage(panel, title);
        return panel;
    };

    wxPanel *blades = page(_L("1. Feeler blades"));
    blades->GetSizer()->Add(make_text(blades,
                                      _L("For a toolhead with a wide, nearly flat underside close to the nozzle tip, such as the fan "
                                         "ducts of most Cartesian printers. Its clearance angle is a few degrees, too shallow to judge by "
                                         "eye, so it is measured as a gap over a distance. Print the blades, each numbered with its "
                                         "thickness in mm. With the printer cold, lower the nozzle until it just touches the bed. On "
                                         "each side, slide blades under the outer edge of the toolhead's underside: the thickest that "
                                         "slides under is the gap. Measure the distance from the nozzle's centre to that edge."),
                                      wrap),
                            0, wxALL, gap);
    auto *create_blades = make_button(blades, _L("Create blades"));
    blades->GetSizer()->Add(create_blades, 0, wxEXPAND);
    auto *blade_grid = new wxFlexGridSizer(3, FromDIP(4), gap);
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, ""));
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, _L("Gap")));
    blade_grid->Add(new wxStaticText(blades, wxID_ANY, _L("Distance")));
    for (int i = 0; i < 4; ++ i) {
        blade_grid->Add(new wxStaticText(blades, wxID_ANY, side_label(i) + ": "), 0, wxALIGN_CENTER_VERTICAL);
        m_gap[i]      = make_input(blades, remembered(std::string("gap_") + SIDES[i]), _L("mm"));
        m_distance[i] = make_input(blades, remembered(std::string("distance_") + SIDES[i]), _L("mm"));
        blade_grid->Add(m_gap[i]);
        blade_grid->Add(m_distance[i]);
    }
    blades->GetSizer()->Add(blade_grid, 0, wxALL, gap);
    auto *apply_blades = make_button(blades, _L("Apply to printer"));
    blades->GetSizer()->Add(apply_blades, 0, wxEXPAND);

    wxPanel *wedges = page(_L("2. Angle wedges"));
    wedges->GetSizer()->Add(make_text(wedges,
                                      _L("For a nozzle standing well below the toolhead, such as a long nozzle or a tilting toolhead. "
                                         "Its clearance angle is steep and is read directly. Print the wedges, each numbered with its "
                                         "angle in degrees. With the printer cold and the nozzle touching the bed, slide a wedge along "
                                         "the bed, thin edge first, straight at the nozzle until the edge touches it. If the toolhead "
                                         "touches the slope first, the wedge is too steep for that side. Enter the steepest wedge that "
                                         "reaches the nozzle on each side, and the toolhead reach."),
                                      wrap),
                            0, wxALL, gap);
    auto *create_wedges = make_button(wedges, _L("Create wedges"));
    wedges->GetSizer()->Add(create_wedges, 0, wxEXPAND);
    auto *wedge_grid = new wxFlexGridSizer(2, FromDIP(4), gap);
    wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, ""));
    wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, _L("Steepest wedge")));
    for (int i = 0; i < 4; ++ i) {
        wedge_grid->Add(new wxStaticText(wedges, wxID_ANY, side_label(i) + ": "), 0, wxALIGN_CENTER_VERTICAL);
        m_angle[i] = make_input(wedges, remembered(std::string("angle_") + SIDES[i]), wxString::FromUTF8(u8"\u00b0"));
        wedge_grid->Add(m_angle[i]);
    }
    wedges->GetSizer()->Add(wedge_grid, 0, wxALL, gap);
    auto *apply_wedges = make_button(wedges, _L("Apply to printer"));
    wedges->GetSizer()->Add(apply_wedges, 0, wxEXPAND);

    wxPanel *fins = page(_L("3. Fin test"));
    // Where the printer puts the nozzle over the plate's centre: G-code positions are the bed's
    // less the extruder offset.
    const Vec2d    nozzle = BoundingBoxf(printable_area()).center() -
                         printer_config().option<ConfigOptionPoints>("extruder_offset")->get_at(0);
    const wxString centre = printer_config().opt_bool("polar_kinematics") ?
                                _L("the bed's rotation axis") :
                                wxString::Format(_L("X %s mm, Y %s mm"), number(nozzle.x(), 1), number(nozzle.y(), 1));
    fins->GetSizer()->Add(make_text(fins,
                                    wxString::Format(_L("Checks the printer's toolhead settings. The blades and wedges measure a few "
                                                        "points on each side; the fin test checks the whole toolhead all round, "
                                                        "including any part lower than the points measured, so do it after either, and "
                                                        "after changing the settings by hand. It prints thin fins around a plate, their "
                                                        "tops %s mm below the toolhead the settings describe and, beyond its radius, "
                                                        "taller. Leave the print on the bed and let the printer cool. Move the nozzle over "
                                                        "the plate's centre (%s) and lower it slowly until it touches the plate. If the "
                                                        "toolhead touches or knocks over a fin, the settings are too generous on that "
                                                        "side: measure again, or lower the clearance angle or raise the toolhead radius, "
                                                        "and repeat. If it touches none, the settings are safe. Take the plate off the "
                                                        "bed before homing the printer: homing may lower the nozzle onto it."),
                                                     number(NonPlanar::fin_test_margin, 1), centre),
                                    wrap),
                          0, wxALL, gap);
    m_fin_text = new wxStaticText(fins, wxID_ANY, "");
    fins->GetSizer()->Add(m_fin_text, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *create_fins = make_button(fins, _L("Create fin test"));
    fins->GetSizer()->Add(create_fins, 0, wxEXPAND);

    const StateColor tab_colour(std::make_pair(wxColour("#6B6B6C"), (int) StateColor::NotChecked),
                                std::make_pair(wxColour("#363636"), (int) StateColor::Normal));
    for (size_t i = 0; i < tabs->GetCount(); ++ i)
        tabs->SetItemTextColour(i, tab_colour);
    tabs->Bind(wxEVT_TAB_SEL_CHANGED, [tabs, book](wxCommandEvent &e) {
        for (size_t i = 0; i < tabs->GetCount(); ++ i)
            tabs->SetItemBold(i, int(i) == e.GetSelection());
        book->SetSelection(e.GetSelection());
        wxGetApp().app_config->set("toolhead_clearance_tab", std::to_string(e.GetSelection()));
    });
    // Its best size spans all it could hold: the pages set the width.
    tabs->SetMinSize(wxSize(FromDIP(300), tabs->GetBestSize().y));
    v_sizer->Add(tabs, 0, wxEXPAND | wxTOP, gap);
    v_sizer->Add(book, 0, wxEXPAND | wxLEFT | wxRIGHT, gap);

    // Room for the result, as the dialog is fitted once only: the tab row stretches to the width
    // it is given, so fitting the dialog again would widen it.
    m_result = new wxStaticText(this, wxID_ANY, "");
    m_result->SetMinSize(wxSize(wrap, 3 * GetTextExtent("Ag").y));
    v_sizer->Add(m_result, 0, wxALL, gap);

    create_blades->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_blades, this);
    create_wedges->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_wedges, this);
    create_fins->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_create_fin_test, this);
    apply_blades->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_apply_blades, this);
    apply_wedges->GetOK()->Bind(wxEVT_BUTTON, &ToolheadClearanceDialog::on_apply_wedges, this);

    update_fin_text();
    // The tab last used: the one whose gauge was printed, to enter what it measures.
    const std::string last = wxGetApp().app_config->get("toolhead_clearance_tab");
    tabs->SelectItem(last == "1" ? 1 : last == "2" ? 2 : 0);
    wxGetApp().UpdateDlgDarkUI(this);
    Layout();
    Fit();
    v_sizer->SetSizeHints(this);
    CenterOnParent();
}

void ToolheadClearanceDialog::on_dpi_changed(const wxRect &)
{
    Layout();
    Refresh();
}

void ToolheadClearanceDialog::remember()
{
    store("reach", m_reach);
    for (int i = 0; i < 4; ++ i) {
        store(std::string("gap_") + SIDES[i], m_gap[i]);
        store(std::string("distance_") + SIDES[i], m_distance[i]);
        store(std::string("angle_") + SIDES[i], m_angle[i]);
    }
}

void ToolheadClearanceDialog::update_fin_text()
{
    const NonPlanar::ToolheadClearance t = current_toolhead();
    m_fin_text->SetLabel(wxString::Format(_L("Now: nozzle clearance angle %s degrees, nozzle length %s mm, toolhead radius %s mm."),
                                          number(t.angle, 1), number(t.length, 2), number(t.radius, 1)));
    m_fin_text->GetParent()->Layout();
}

void ToolheadClearanceDialog::show_result(const wxString &text)
{
    m_result->SetLabel(text);
    m_result->Wrap(FromDIP(640));
    Layout();
}

void ToolheadClearanceDialog::on_create_blades(wxCommandEvent &)
{
    remember();
    // Closed, so the gauge can be sliced and printed.
    EndModal(wxID_OK);
    open_gauge(m_plater, _L("Toolhead clearance feeler blades"),
               NonPlanar::feeler_blades(NonPlanar::feeler_thicknesses(), gauge_width()), "OrcaNP feeler blades.stl");
}

void ToolheadClearanceDialog::on_create_wedges(wxCommandEvent &)
{
    remember();
    double reach = 0.;
    if (! read(m_reach, reach) || reach <= 0.)
        reach = current_toolhead().reach();
    EndModal(wxID_OK);
    open_gauge(m_plater, _L("Toolhead clearance angle wedges"),
               NonPlanar::angle_wedges(NonPlanar::wedge_angles(), reach, gauge_width()), "OrcaNP angle wedges.stl");
}

void ToolheadClearanceDialog::on_create_fin_test(wxCommandEvent &)
{
    remember();
    const double nozzle = printer_config().option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    EndModal(wxID_OK);
    open_gauge(m_plater, _L("Toolhead clearance fin test"), NonPlanar::fin_test(current_toolhead(), 2.5 * nozzle),
               "OrcaNP toolhead fin test.stl");
}

namespace {

void apply_toolhead(const NonPlanar::ToolheadClearance &t)
{
    DynamicPrintConfig &printer = printer_config();
    printer.set_key_value("nonplanar_nozzle_clearance_angle", new ConfigOptionFloat(t.angle));
    printer.set_key_value("nonplanar_nozzle_length", new ConfigOptionFloat(t.length));
    printer.set_key_value("nonplanar_head_radius", new ConfigOptionFloat(t.radius));
    Tab *tab = wxGetApp().get_tab(Preset::TYPE_PRINTER);
    tab->reload_config();
    tab->update_dirty();
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
            MessageDialog(this, _L("Please input the gap and the distance for every side."), wxEmptyString, wxICON_WARNING | wxOK).ShowModal();
            return;
        }
    double reach = 0.;
    if (! read(m_reach, reach))
        reach = 0.;
    NonPlanar::ToolheadClearance t;
    std::string                  error;
    if (! NonPlanar::fit_feeler_readings(readings, current_toolhead().tip_diameter, reach, t, error)) {
        MessageDialog(this, from_u8(error), wxEmptyString, wxICON_WARNING | wxOK).ShowModal();
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
            MessageDialog(this, _L("Please input the steepest wedge for every side."), wxEmptyString, wxICON_WARNING | wxOK).ShowModal();
            return;
        }
    if (! read(m_reach, reach)) {
        MessageDialog(this, _L("Please input the toolhead reach."), wxEmptyString, wxICON_WARNING | wxOK).ShowModal();
        return;
    }
    NonPlanar::ToolheadClearance t;
    std::string                  error;
    if (! NonPlanar::fit_wedge_readings(angles, current_toolhead().tip_diameter, reach, t, error)) {
        MessageDialog(this, from_u8(error), wxEmptyString, wxICON_WARNING | wxOK).ShowModal();
        return;
    }
    apply_toolhead(t);
    update_fin_text();
    show_result(applied(t));
}

} // namespace GUI
} // namespace Slic3r
