#include "CalibrationDialogKit.hpp"

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
#include "libslic3r/CalibrationPrints.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <boost/filesystem.hpp>

#include <wx/simplebook.h>

#include <limits>

namespace Slic3r {
namespace GUI {

CalibrationTabsDialog::CalibrationTabsDialog(wxWindow *parent, Plater *plater, const wxString &title, const std::string &key,
                                             const wxString &intro)
    : DPIDialog(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE), m_plater(plater), m_key(key)
{
    SetBackgroundColour(*wxWHITE);
    SetForegroundColour(wxColour("#363636"));
    SetFont(Label::Body_14);
    auto *v_sizer = new wxBoxSizer(wxVERTICAL);
    SetSizer(v_sizer);
    auto *text = new wxStaticText(this, wxID_ANY, intro);
    text->Wrap(wrap_width());
    v_sizer->Add(text, 0, wxALL, FromDIP(10));

    // A tab per test, as in the Preferences dialog.
    m_tabs = new TabCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                         wxTR_NO_BUTTONS | wxTR_HIDE_ROOT | wxTR_SINGLE | wxTR_NO_LINES | wxBORDER_NONE | wxWANTS_CHARS | wxTR_FULL_ROW_HIGHLIGHT);
    m_tabs->SetFont(Label::Body_14);
    m_book = new wxSimplebook(this, wxID_ANY);
}

int CalibrationTabsDialog::wrap_width() const { return FromDIP(640); }

wxPanel *CalibrationTabsDialog::add_page(const wxString &title, const wxString &text)
{
    auto *panel = new wxPanel(m_book);
    panel->SetBackgroundColour(*wxWHITE);
    panel->SetSizer(new wxBoxSizer(wxVERTICAL));
    m_tabs->AppendItem(title);
    m_book->AddPage(panel, title);
    add_text(panel, text);
    return panel;
}

wxStaticText *CalibrationTabsDialog::add_text(wxPanel *page, const wxString &text)
{
    auto *label = new wxStaticText(page, wxID_ANY, text);
    label->Wrap(wrap_width());
    page->GetSizer()->Add(label, 0, wxALL, FromDIP(10));
    return label;
}

DialogButtons *CalibrationTabsDialog::add_button(wxPanel *page, const wxString &label)
{
    auto *button = new DialogButtons(page, { "OK" });
    button->GetOK()->SetLabel(label);
    page->GetSizer()->Add(button, 0, wxEXPAND);
    return button;
}

wxFlexGridSizer *CalibrationTabsDialog::add_grid(wxPanel *page, int columns)
{
    auto *grid = new wxFlexGridSizer(columns, FromDIP(4), FromDIP(10));
    page->GetSizer()->Add(grid, 0, wxALL, FromDIP(10));
    return grid;
}

TextInput *CalibrationTabsDialog::add_input(wxWindow *parent, const std::string &name, const wxString &unit, const wxString &fallback)
{
    const std::string value = wxGetApp().app_config->get(m_key + "_" + name);
    auto *in = new TextInput(parent, value.empty() ? fallback : wxString::FromUTF8(value), unit, "", wxDefaultPosition,
                             parent->FromDIP(wxSize(90, -1)));
    in->GetTextCtrl()->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
    m_inputs.emplace_back(name, in);
    return in;
}

void CalibrationTabsDialog::finish()
{
    const StateColor tab_colour(std::make_pair(wxColour("#6B6B6C"), (int) StateColor::NotChecked),
                                std::make_pair(wxColour("#363636"), (int) StateColor::Normal));
    for (size_t i = 0; i < m_tabs->GetCount(); ++ i)
        m_tabs->SetItemTextColour(i, tab_colour);
    m_tabs->Bind(wxEVT_TAB_SEL_CHANGED, [this](wxCommandEvent &e) {
        for (size_t i = 0; i < m_tabs->GetCount(); ++ i)
            m_tabs->SetItemBold(i, int(i) == e.GetSelection());
        m_book->SetSelection(e.GetSelection());
        wxGetApp().app_config->set(m_key + "_tab", std::to_string(e.GetSelection()));
    });
    // Its best size spans all it could hold: the pages set the width.
    m_tabs->SetMinSize(wxSize(FromDIP(300), m_tabs->GetBestSize().y));
    const int gap = FromDIP(10);
    GetSizer()->Add(m_tabs, 0, wxEXPAND | wxTOP, gap);
    GetSizer()->Add(m_book, 0, wxEXPAND | wxLEFT | wxRIGHT, gap);
    // Room for the result, as the dialog is fitted once only: the tab row stretches to the width
    // it is given, so fitting the dialog again would widen it.
    m_result = new wxStaticText(this, wxID_ANY, "");
    m_result->SetMinSize(wxSize(wrap_width(), 4 * GetTextExtent("Ag").y));
    GetSizer()->Add(m_result, 0, wxALL, gap);

    const int last = std::atoi(wxGetApp().app_config->get(m_key + "_tab").c_str());
    m_tabs->SelectItem(last >= 0 && last < int(m_tabs->GetCount()) ? last : 0);
    wxGetApp().UpdateDlgDarkUI(this);
    Layout();
    Fit();
    GetSizer()->SetSizeHints(this);
    CenterOnParent();
}

void CalibrationTabsDialog::on_dpi_changed(const wxRect &)
{
    Layout();
    Refresh();
}

void CalibrationTabsDialog::remember()
{
    for (const auto &[name, in] : m_inputs)
        wxGetApp().app_config->set(m_key + "_" + name, in->GetTextCtrl()->GetValue().ToUTF8().data());
}

void CalibrationTabsDialog::show_result(const wxString &text)
{
    m_result->SetLabel(text);
    m_result->Wrap(wrap_width());
    Layout();
}

void CalibrationTabsDialog::warn(const wxString &text) { MessageDialog(this, text, wxEmptyString, wxICON_WARNING | wxOK).ShowModal(); }

bool CalibrationTabsDialog::read(TextInput *in, double &value) { return in->GetTextCtrl()->GetValue().ToDouble(&value); }

bool CalibrationTabsDialog::read_all(const std::vector<TextInput *> &inputs, std::vector<double> &values, const wxString &what)
{
    values.assign(inputs.size(), 0.);
    for (size_t i = 0; i < inputs.size(); ++ i)
        if (! read(inputs[i], values[i])) {
            warn(wxString::Format(_L("Please input %s."), what));
            return false;
        }
    return true;
}

namespace CalibrationKit {

DynamicPrintConfig &printer_config() { return wxGetApp().preset_bundle->printers.get_edited_preset().config; }
DynamicPrintConfig &print_config() { return wxGetApp().preset_bundle->prints.get_edited_preset().config; }
DynamicPrintConfig &filament_config() { return wxGetApp().preset_bundle->filaments.get_edited_preset().config; }

void apply(Preset::Type type, const DynamicPrintConfig &values)
{
    DynamicPrintConfig &config = type == Preset::TYPE_PRINTER ? printer_config() : type == Preset::TYPE_PRINT ? print_config() : filament_config();
    config.apply(values);
    Tab *tab = wxGetApp().get_tab(type);
    tab->reload_config();
    tab->update_dirty();
}

wxString number(double value, int decimals) { return wxString::FromUTF8(float_to_string_decimal_point(value, decimals)); }

double nozzle_diameter() { return printer_config().option<ConfigOptionFloats>("nozzle_diameter")->get_at(0); }

Vec2d bed_centre() { return BoundingBoxf(printer_config().option<ConfigOptionPoints>("printable_area")->values).center(); }

Vec2d usable_area()
{
    const Pointfs &area = printer_config().option<ConfigOptionPoints>("printable_area")->values;
    return CalibrationPrints::usable_area(std::vector<Vec2d>(area.begin(), area.end()), 5.);
}

bool is_polar()
{
    const auto *opt = printer_config().option<ConfigOptionBool>("polar_kinematics");
    return opt != nullptr && opt->value;
}

std::vector<TestObject> lay_out(std::vector<indexed_triangle_set> pieces, bool split, const std::vector<DynamicPrintConfig> &configs)
{
    const Pointfs &area   = printer_config().option<ConfigOptionPoints>("printable_area")->values;
    const Vec2d    usable = CalibrationPrints::usable_area(std::vector<Vec2d>(area.begin(), area.end()), 5.);
    // Not split, the rows go on as deep as they need.
    const std::vector<CalibrationPrints::PiecePlacement> at =
        CalibrationPrints::place_on_plates(pieces, split ? usable : Vec2d(usable.x(), std::numeric_limits<double>::max()), 5.);
    std::vector<TestObject> out(pieces.size());
    for (size_t i = 0; i < pieces.size(); ++ i) {
        out[i].mesh   = std::move(pieces[i]);
        out[i].offset = at[i].centre;
        out[i].plate  = at[i].plate;
        if (i < configs.size())
            out[i].config = configs[i];
    }
    return out;
}

bool open_test_print(Plater *plater, const wxString &name, const std::vector<TestObject> &objects, double layer)
{
    if (plater->new_project(false, false, name) == wxID_CANCEL)
        return false;
    wxGetApp().mainframe->select_tab(TAB_ID_PREPARE);
    const Vec2d    centre = bed_centre();
    PartPlateList &plates = plater->get_partplate_list();
    std::vector<size_t> added;
    for (size_t i = 0; i < objects.size(); ++ i) {
        // Centred on its bounding box, which goes where it is laid out.
        indexed_triangle_set mesh = objects[i].mesh;
        BoundingBoxf3        box;
        for (const Vec3f &v : mesh.vertices)
            box.merge(v.cast<double>());
        for (Vec3f &v : mesh.vertices)
            v -= Vec3f(float(box.center().x()), float(box.center().y()), 0.f);
        const std::string file = "OrcaNP test print " + std::to_string(i + 1) + ".stl";
        const std::string path = (boost::filesystem::temp_directory_path() / file).string();
        if (! its_write_stl_binary(path.c_str(), file.c_str(), mesh) || ! plater->add_model(false, path))
            return false;
        while (plates.get_plate_count() <= int(objects[i].plate))
            plates.create_plate();
        const size_t obj_idx = plater->model().objects.size() - 1;
        plates.add_to_plate(int(obj_idx), 0, int(objects[i].plate));
        const Vec3d  origin = plates.get_plate(int(objects[i].plate))->get_origin();
        ModelObject *object = plater->model().objects.back();
        object->name        = into_u8(name) + (objects.size() > 1 ? " " + std::to_string(i + 1) : std::string());
        object->instances.front()->set_offset(Vec3d(origin.x() + centre.x() + objects[i].offset.x(), origin.y() + centre.y() + objects[i].offset.y(), 0.));
        object->ensure_on_bed();
        object->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
        object->config.set_key_value("s4_enabled", new ConfigOptionBool(false));
        for (const std::string &key : objects[i].config.keys())
            object->config.set_key_value(key, objects[i].config.option(key)->clone());
        added.emplace_back(plater->model().objects.size() - 1);
    }

    const double l     = layer > 0. ? layer : nozzle_diameter() < 0.3 ? 0.1 : 0.2;
    auto        *print = &print_config();
    print->set_key_value("layer_height", new ConfigOptionFloat(l));
    print->set_key_value("initial_layer_print_height", new ConfigOptionFloat(l));
    print->set_key_value("spiral_mode", new ConfigOptionBool(false));
    plater->changed_objects(added);
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_dirty();
    wxGetApp().get_tab(Preset::TYPE_PRINT)->update_ui_from_settings();
    return true;
}

} // namespace CalibrationKit

} // namespace GUI
} // namespace Slic3r
