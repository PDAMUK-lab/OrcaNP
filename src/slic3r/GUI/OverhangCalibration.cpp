#include "OverhangCalibration.hpp"

#include "I18N.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/CalibrationPrints.hpp"

namespace Slic3r {
namespace GUI {

namespace CP = CalibrationPrints;
using namespace CalibrationKit;

namespace {

// The layers the tests print in, as open_test_print() sets them.
double test_layer() { return nozzle_diameter() < 0.3 ? 0.1 : 0.2; }

// Gaps from half a layer to one and a half, a quarter layer apart.
std::vector<double> gaps()
{
    std::vector<double> out;
    for (double k : { 0.5, 0.75, 1., 1.25, 1.5 })
        out.emplace_back(k * test_layer());
    return out;
}

wxString gap_list()
{
    wxString out;
    for (double g : gaps())
        out += (out.empty() ? "" : ", ") + number(g, 3);
    return out;
}

// A row of samples, `pitch` apart, centred on the bed.
std::vector<CalibrationKit::TestObject> row(const std::vector<indexed_triangle_set> &meshes, const std::vector<DynamicPrintConfig> &configs,
                                            double pitch)
{
    std::vector<CalibrationKit::TestObject> out;
    for (size_t i = 0; i < meshes.size(); ++ i)
        out.push_back({ meshes[i], Vec2d((double(i) - 0.5 * (meshes.size() - 1)) * pitch, 0.), configs[i] });
    return out;
}

} // namespace

OverhangGapsDialog::OverhangGapsDialog(wxWindow *parent, Plater *plater)
    : CalibrationTabsDialog(parent, plater, _L("Overhangs and gaps"), "overhang_calibration",
                            _L("Finds what the printer prints unsupported, for the settings that rely on it. Each test sets up its "
                               "print as a new project: print it, then open this again to enter the result."))
{
    wxPanel *overhangs = add_page(_L("1. Overhang limit"),
                                  _L("Prints ledges overhanging at 30 to 80 degrees from vertical, each numbered with its angle, flat "
                                     "and without support. Enter the steepest that printed with a clean underside, and apply: it "
                                     "becomes Maximum overhang (Process > Quality > Non-planar (S4)). Non-planar printing bends the "
                                     "layers under overhangs steeper than that toward printable, so set to what the printer prints, it "
                                     "bends no more than it must."));
    auto *create_overhangs = add_button(overhangs, _L("Create overhangs"));
    auto *overhang_grid    = add_grid(overhangs, 2);
    overhang_grid->Add(new wxStaticText(overhangs, wxID_ANY, _L("Steepest clean overhang") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_overhang = add_input(overhangs, "overhang", wxString::FromUTF8(u8"°"));
    overhang_grid->Add(m_overhang);
    auto *apply_overhangs = add_button(overhangs, _L("Apply to process"));

    wxPanel *support = add_page(_L("2. Support gap"),
                                wxString::Format(_L("Prints small tables on support, each with a different gap between the support "
                                                    "and the table top (%s mm), numbered with it. Take the support off: enter the "
                                                    "smallest gap that came off cleanly and left a smooth underside, and apply: it "
                                                    "becomes Top Z distance (Process > Support > Advanced). The tables print with "
                                                    "Independent support layer height on; with it off, the gap is rounded to whole "
                                                    "layers."),
                                                 gap_list()));
    auto *create_support = add_button(support, _L("Create tables"));
    auto *support_grid   = add_grid(support, 2);
    support_grid->Add(new wxStaticText(support, wxID_ANY, _L("Best gap") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_support_gap = add_input(support, "support_gap", _L("mm"));
    support_grid->Add(m_support_gap);
    auto *apply_support = add_button(support, _L("Apply to process"));

    wxPanel *surface = add_page(_L("3. Surface gap (S4)"),
                                wxString::Format(_L("Prints discs over generated pillars (Non-planar (S4), Offset from print surface), "
                                                    "each with a different gap between the pillar and the disc (%s mm), numbered with "
                                                    "it. Take the discs off: enter the smallest gap whose disc came off cleanly with a "
                                                    "smooth underside, and apply: it becomes Surface gap (Process > Quality > "
                                                    "Non-planar (S4))."),
                                                 gap_list()));
    auto *create_surface = add_button(surface, _L("Create discs"));
    auto *surface_grid   = add_grid(surface, 2);
    surface_grid->Add(new wxStaticText(surface, wxID_ANY, _L("Best gap") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_surface_gap = add_input(surface, "surface_gap", _L("mm"));
    surface_grid->Add(m_surface_gap);
    auto *apply_surface = add_button(surface, _L("Apply to process"));

    create_overhangs->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_create_overhangs, this);
    apply_overhangs->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_apply_overhangs, this);
    create_support->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_create_support, this);
    apply_support->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_apply_support, this);
    create_surface->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_create_surface, this);
    apply_surface->GetOK()->Bind(wxEVT_BUTTON, &OverhangGapsDialog::on_apply_surface, this);
    finish();
}

void OverhangGapsDialog::on_create_overhangs(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Overhang limit"), { { CP::overhang_samples(CP::overhang_angles(), layout_width()), Vec2d::Zero(), {} } });
}

void OverhangGapsDialog::on_apply_overhangs(wxCommandEvent &)
{
    remember();
    double angle = 0.;
    if (! read(m_overhang, angle) || angle < 10. || angle > 89.) {
        warn(_L("Please input the steepest overhang that printed cleanly, in degrees from vertical."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("s4_max_overhang", new ConfigOptionFloat(angle));
    apply(Preset::TYPE_PRINT, values);
    show_result(wxString::Format(_L("Maximum overhang %s degrees. Save the process preset to keep it."), number(angle, 0)));
}

void OverhangGapsDialog::on_create_support(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    std::vector<indexed_triangle_set> meshes;
    std::vector<DynamicPrintConfig>   configs;
    for (double g : gaps()) {
        meshes.emplace_back(CP::support_gap_sample(CP::format_value(g, 2)));
        DynamicPrintConfig &c = configs.emplace_back();
        c.set_key_value("enable_support", new ConfigOptionBool(true));
        c.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
        c.set_key_value("support_top_z_distance", new ConfigOptionFloat(g));
    }
    if (! open_test_print(m_plater, _L("Support gap"), row(meshes, configs, 30.)))
        return;
    DynamicPrintConfig values;
    values.set_key_value("independent_support_layer_height", new ConfigOptionBool(true));
    apply(Preset::TYPE_PRINT, values);
}

void OverhangGapsDialog::on_apply_support(wxCommandEvent &)
{
    remember();
    double gap = 0.;
    if (! read(m_support_gap, gap) || gap < 0. || gap > 2.) {
        warn(_L("Please input the best gap, in mm."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("support_top_z_distance", new ConfigOptionFloat(gap));
    apply(Preset::TYPE_PRINT, values);
    show_result(wxString::Format(_L("Top Z distance %s mm. Save the process preset to keep it."), number(gap, 3)));
}

void OverhangGapsDialog::on_create_surface(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    std::vector<indexed_triangle_set> meshes;
    std::vector<DynamicPrintConfig>   configs;
    for (double g : gaps()) {
        meshes.emplace_back(CP::surface_gap_sample(CP::format_value(g, 2)));
        DynamicPrintConfig &c = configs.emplace_back();
        c.set_key_value("s4_enabled", new ConfigOptionBool(true));
        c.set_key_value("s4_layer_shape", new ConfigOptionEnum<S4LayerShape>(S4LayerShape::Offset));
        c.set_key_value("s4_surface_core", new ConfigOptionEnum<S4SurfaceCore>(S4SurfaceCore::Pillar));
        c.set_key_value("s4_surface_size", new ConfigOptionEnum<S4SurfaceSize>(S4SurfaceSize::Custom));
        // A pillar wider than the disc, so all of the disc is over its flat top.
        c.set_key_value("s4_surface_diameter", new ConfigOptionFloat(CP::surface_gap_sample_diameter + 4.));
        c.set_key_value("s4_surface_height", new ConfigOptionFloat(4.));
        c.set_key_value("s4_surface_gap", new ConfigOptionFloat(g));
    }
    open_test_print(m_plater, _L("Surface gap"), row(meshes, configs, 30.));
}

void OverhangGapsDialog::on_apply_surface(wxCommandEvent &)
{
    remember();
    double gap = 0.;
    if (! read(m_surface_gap, gap) || gap < 0. || gap > 2.) {
        warn(_L("Please input the best gap, in mm."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("s4_surface_gap", new ConfigOptionFloat(gap));
    apply(Preset::TYPE_PRINT, values);
    show_result(wxString::Format(_L("Surface gap %s mm. Save the process preset to keep it."), number(gap, 3)));
}

} // namespace GUI
} // namespace Slic3r
