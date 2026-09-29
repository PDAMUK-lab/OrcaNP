#include "PolarCalibration.hpp"

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "MainFrame.hpp"
#include "MsgDialog.hpp"
#include "PartPlate.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/Label.hpp"
#include "Widgets/LabeledStaticBox.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/NonPlanar/PolarKinematics.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace Slic3r {
namespace GUI {

namespace {

// A ring about the Z axis: `r_out` its outer radius, `thickness` its wall, standing on Z = 0.
void append_ring(indexed_triangle_set &its, double r_out, double thickness, double height, int segments = 360)
{
    const int    base = int(its.vertices.size());
    const double r_in = r_out - thickness;
    for (int i = 0; i < segments; ++ i) {
        const double a = 2. * M_PI * i / segments;
        for (double r : { r_out, r_in })
            for (double z : { 0., height })
                its.vertices.emplace_back(float(r * std::cos(a)), float(r * std::sin(a)), float(z));
    }
    // Vertex of segment i: outer bottom, outer top, inner bottom, inner top.
    auto v = [&](int i, int k) { return base + 4 * (i % segments) + k; };
    for (int i = 0; i < segments; ++ i) {
        const int j = i + 1;
        its.indices.emplace_back(v(i, 0), v(j, 0), v(j, 1)); // outside
        its.indices.emplace_back(v(i, 0), v(j, 1), v(i, 1));
        its.indices.emplace_back(v(i, 2), v(j, 3), v(j, 2)); // inside
        its.indices.emplace_back(v(i, 2), v(i, 3), v(j, 3));
        its.indices.emplace_back(v(i, 1), v(j, 1), v(j, 3)); // top
        its.indices.emplace_back(v(i, 1), v(j, 3), v(i, 3));
        its.indices.emplace_back(v(i, 0), v(j, 2), v(j, 0)); // bottom
        its.indices.emplace_back(v(i, 0), v(i, 2), v(j, 2));
    }
}

TextInput *add_row(wxWindow *parent, wxSizer *sizer, const wxString &label, const wxString &value, int label_width)
{
    auto *row  = new wxBoxSizer(wxHORIZONTAL);
    auto *text = new wxStaticText(parent, wxID_ANY, label, wxDefaultPosition, wxSize(label_width, -1), wxALIGN_LEFT);
    auto *in   = new TextInput(parent, value, _L("mm"), "", wxDefaultPosition, parent->FromDIP(wxSize(120, -1)));
    in->GetTextCtrl()->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
    row->Add(text, 0, wxALL | wxALIGN_CENTER_VERTICAL, parent->FromDIP(2));
    row->Add(in, 0, wxALL | wxALIGN_CENTER_VERTICAL, parent->FromDIP(2));
    sizer->Add(row, 0, wxLEFT, parent->FromDIP(3));
    return in;
}

bool read(TextInput *in, double &value) { return in->GetTextCtrl()->GetValue().ToDouble(&value); }

// The rings last created, kept in the app config: the dialog is opened again to enter their
// measurements after they are printed.
wxString remembered(const std::string &key, const char *fallback)
{
    const std::string value = wxGetApp().app_config->get(key);
    return value.empty() ? wxString(fallback) : wxString::FromUTF8(value);
}

int widest(wxWindow *parent, const std::vector<wxString> &labels)
{
    int width = 0;
    for (const wxString &label : labels)
        width = std::max(width, parent->GetTextExtent(label).x);
    return width + parent->FromDIP(10);
}

} // namespace

PolarAlignmentDialog::PolarAlignmentDialog(wxWindow *parent, Plater *plater)
    : DPIDialog(parent, wxID_ANY, _L("Polar alignment"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE), m_plater(plater)
{
    SetBackgroundColour(*wxWHITE);
    SetForegroundColour(wxColour("#363636"));
    SetFont(Label::Body_14);

    auto *v_sizer = new wxBoxSizer(wxVERTICAL);
    SetSizer(v_sizer);
    auto *intro = new wxStaticText(this, wxID_ANY,
                                   _L("Prints two thin rings about the bed's rotation axis. Measure each ring's outer diameter with "
                                      "calipers and enter it below: if the rings come out too large, the rotation axis is farther "
                                      "from the head's zero radius than the printer assumes, and too small, nearer. The printer's "
                                      "radius offset and scale are corrected from the two. Create the rings, print them, then "
                                      "open this again to enter what you measured."));
    intro->Wrap(FromDIP(420));
    v_sizer->Add(intro, 0, wxALL, FromDIP(10));

    const wxString inner_str = _L("Inner ring radius") + ": ", outer_str = _L("Outer ring radius") + ": ", height_str = _L("Height") + ": ",
                   inner_measured_str = _L("Inner ring, outer diameter measured") + ": ",
                   outer_measured_str = _L("Outer ring, outer diameter measured") + ": ";
    const int label_width = widest(this, { inner_str, outer_str, height_str, inner_measured_str, outer_measured_str });

    auto *rings_box   = new LabeledStaticBox(this, _L("Rings"));
    auto *rings_sizer = new wxStaticBoxSizer(rings_box, wxVERTICAL);
    m_inner           = add_row(this, rings_sizer, inner_str, remembered("polar_alignment_inner_radius", "20"), label_width);
    m_outer           = add_row(this, rings_sizer, outer_str, remembered("polar_alignment_outer_radius", "40"), label_width);
    m_height          = add_row(this, rings_sizer, height_str, remembered("polar_alignment_height", "3"), label_width);
    auto *create      = new DialogButtons(this, { "OK" });
    create->GetOK()->SetLabel(_L("Create rings"));
    rings_sizer->Add(create, 0, wxEXPAND);
    v_sizer->Add(rings_sizer, 0, wxLEFT | wxRIGHT | wxEXPAND, FromDIP(10));

    auto *measure_box   = new LabeledStaticBox(this, _L("Measurements"));
    auto *measure_sizer = new wxStaticBoxSizer(measure_box, wxVERTICAL);
    m_inner_measured    = add_row(this, measure_sizer, inner_measured_str, "", label_width);
    m_outer_measured    = add_row(this, measure_sizer, outer_measured_str, "", label_width);
    m_result            = new wxStaticText(this, wxID_ANY, "");
    measure_sizer->Add(m_result, 0, wxALL, FromDIP(5));
    auto *apply = new DialogButtons(this, { "OK" });
    apply->GetOK()->SetLabel(_L("Apply to printer"));
    measure_sizer->Add(apply, 0, wxEXPAND);
    v_sizer->Add(measure_sizer, 0, wxALL | wxEXPAND, FromDIP(10));

    create->GetOK()->Bind(wxEVT_BUTTON, &PolarAlignmentDialog::on_create, this);
    apply->GetOK()->Bind(wxEVT_BUTTON, &PolarAlignmentDialog::on_apply, this);

    wxGetApp().UpdateDlgDarkUI(this);
    Layout();
    Fit();
    v_sizer->SetSizeHints(this);
}

void PolarAlignmentDialog::on_dpi_changed(const wxRect &)
{
    Refresh();
    Fit();
}

bool PolarAlignmentDialog::read_rings(double &r1, double &r2, double &height)
{
    if (! read(m_inner, r1) || ! read(m_outer, r2) || ! read(m_height, height) || r1 < 5. || r2 < r1 + 5. || height <= 0.) {
        MessageDialog(this, _L("Please input valid values: the inner ring at least 5 mm in radius, the outer ring at least 5 mm "
                               "larger, and a height."),
                      wxEmptyString, wxICON_WARNING | wxOK)
            .ShowModal();
        return false;
    }
    return true;
}

void PolarAlignmentDialog::on_create(wxCommandEvent &)
{
    double r1, r2, height;
    if (! read_rings(r1, r2, height))
        return;
    AppConfig *app_config = wxGetApp().app_config;
    app_config->set("polar_alignment_inner_radius", float_to_string_decimal_point(r1));
    app_config->set("polar_alignment_outer_radius", float_to_string_decimal_point(r2));
    app_config->set("polar_alignment_height", float_to_string_decimal_point(height));
    // Closed, so the rings can be sliced and printed; opened again to enter their measurements.
    EndModal(wxID_OK);
    if (m_plater->new_project(false, false, _L("Polar alignment")) == wxID_CANCEL)
        return;
    wxGetApp().mainframe->select_tab(TAB_ID_PREPARE);

    // Walls two lines thick: one loop each side, the outer one's edge where the ring was designed.
    const auto  &printer = wxGetApp().preset_bundle->printers.get_edited_preset().config;
    const double nozzle  = printer.option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    indexed_triangle_set its;
    append_ring(its, r1, 2.5 * nozzle, height);
    append_ring(its, r2, 2.5 * nozzle, height);
    const std::string path = (boost::filesystem::temp_directory_path() / "OrcaNP polar alignment rings.stl").string();
    if (! its_write_stl_binary(path.c_str(), "polar alignment rings", its) || ! m_plater->add_model(false, path))
        return;

    // Centred on the rotation axis, the centre of the printable area.
    ModelObject *object = m_plater->model().objects.front();
    const Vec2d  axis   = BoundingBoxf(printer.option<ConfigOptionPoints>("printable_area")->values).center();
    const Vec3d  origin = m_plater->get_partplate_list().get_curr_plate()->get_origin();
    object->instances.front()->set_offset(Vec3d(origin.x() + axis.x(), origin.y() + axis.y(), 0.));
    object->ensure_on_bed();
    object->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    object->config.set_key_value("s4_enabled", new ConfigOptionBool(false));

    auto *print = &wxGetApp().preset_bundle->prints.get_edited_preset().config;
    print->set_key_value("wall_loops", new ConfigOptionInt(1));
    print->set_key_value("top_shell_layers", new ConfigOptionInt(0));
    print->set_key_value("bottom_shell_layers", new ConfigOptionInt(0));
    print->set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print->set_key_value("spiral_mode", new ConfigOptionBool(false));
    m_plater->changed_objects({ 0 });
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_dirty();
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_ui_from_settings();
}

void PolarAlignmentDialog::on_apply(wxCommandEvent &)
{
    double r1, r2, height, d1, d2;
    if (! read_rings(r1, r2, height))
        return;
    if (! read(m_inner_measured, d1) || ! read(m_outer_measured, d2) || d1 <= 0. || d2 <= d1) {
        MessageDialog(this, _L("Please input both measured diameters, the outer ring's the larger."), wxEmptyString,
                      wxICON_WARNING | wxOK)
            .ShowModal();
        return;
    }
    auto        &printer = wxGetApp().preset_bundle->printers.get_edited_preset().config;
    const double offset  = printer.opt_float("polar_radius_offset");
    const double scale   = printer.opt_float("polar_radius_scale") / 100.;
    const NonPlanar::RadiusCalibration c = NonPlanar::solve_radius_calibration(r1, d1, r2, d2, offset, scale);
    if (c.scale < 0.5 || c.scale > 1.5) {
        MessageDialog(this, _L("These measurements give a radius scale beyond 50 to 150 %: please check them."), wxEmptyString,
                      wxICON_WARNING | wxOK)
            .ShowModal();
        return;
    }
    printer.set_key_value("polar_radius_offset", new ConfigOptionFloat(c.offset));
    printer.set_key_value("polar_radius_scale", new ConfigOptionFloat(100. * c.scale));
    Tab *tab = wxGetApp().get_tab(Preset::TYPE_PRINTER);
    tab->reload_config();
    tab->update_dirty();
    m_result->SetLabel(wxString::Format(_L("Radius offset %.3f mm, radius scale %.3f %%. Save the printer preset to keep them."),
                                        c.offset, 100. * c.scale));
    m_result->Wrap(FromDIP(420));
    Layout();
    Fit();
}

} // namespace GUI
} // namespace Slic3r
