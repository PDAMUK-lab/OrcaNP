#pragma once

// Orca: what OrcaNP's calibration dialogs share (Calibration > Dimensions, Overhangs and gaps,
// Toolhead clearance, Polar calibration): a tab per test, each explaining when and how to use it,
// with a button that sets its print up as a new project, what to measure on it, and a button that
// applies the result to the settings. What is entered is kept in the app config, as the dialog is
// opened again to enter it after printing.

#include "GUI_Utils.hpp"

#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <string>
#include <utility>
#include <vector>

class TabCtrl;
class TextInput;
class wxFlexGridSizer;
class wxPanel;
class wxSimplebook;
class wxStaticText;

namespace Slic3r {
namespace GUI {

class DialogButtons;
class Plater;

class CalibrationTabsDialog : public DPIDialog
{
public:
    void on_dpi_changed(const wxRect &suggested_rect) override;

protected:
    // `key` names what the dialog keeps in the app config; `intro` goes above the tabs.
    CalibrationTabsDialog(wxWindow *parent, Plater *plater, const wxString &title, const std::string &key, const wxString &intro);

    // A tab headed by its explanation.
    wxPanel *add_page(const wxString &title, const wxString &text);
    // A full-width button on `page`.
    DialogButtons *add_button(wxPanel *page, const wxString &label);
    // A grid of `columns` for inputs, added to `page`.
    wxFlexGridSizer *add_grid(wxPanel *page, int columns);
    // A number input showing what was last entered under `name` (or `fallback`), kept by remember().
    TextInput *add_input(wxWindow *parent, const std::string &name, const wxString &unit, const wxString &fallback = wxEmptyString);
    // Text wrapped to the dialog's width.
    wxStaticText *add_text(wxPanel *page, const wxString &text);
    // Lays the dialog out on the tab last used: the one whose print was made, to enter what it
    // measures.
    void finish();

    void remember();
    void show_result(const wxString &text);
    void warn(const wxString &text);
    // The input's number; false when it is empty or not a number.
    static bool read(TextInput *in, double &value);
    // Each input's number, or a warning naming `what` and false.
    bool read_all(const std::vector<TextInput *> &inputs, std::vector<double> &values, const wxString &what);

    int wrap_width() const;

    Plater *m_plater;

private:
    std::string                                     m_key;
    TabCtrl                                        *m_tabs;
    wxSimplebook                                   *m_book;
    wxStaticText                                   *m_result;
    std::vector<std::pair<std::string, TextInput *>> m_inputs;
};

namespace CalibrationKit {

DynamicPrintConfig &printer_config();
DynamicPrintConfig &print_config();
DynamicPrintConfig &filament_config();
// Sets `values` in the edited preset of `type` and refreshes its tab: the preset is then modified,
// to be saved or not.
void apply(Preset::Type type, const DynamicPrintConfig &values);

wxString number(double value, int decimals);
double   nozzle_diameter();
// The bed's centre (a polar printer's rotation axis), in bed coordinates.
Vec2d bed_centre();
bool  is_polar();
// The rectangle test pieces are laid out in (CalibrationPrints::usable_area()).
Vec2d usable_area();

// A test print's piece: its mesh, the plate it goes on, where its bounding box's centre goes
// relative to the bed's centre, and the settings it is printed with beyond the project's.
struct TestObject
{
    indexed_triangle_set mesh;
    Vec2d                offset = Vec2d::Zero();
    DynamicPrintConfig   config;
    size_t               plate  = 0;
};
// Test pieces laid out on the bed in rows, their settings `configs` (if any). Pieces measured on
// their own may go on as many plates as they need (`split`); pieces compared with each other stay
// on one plate, in rows past its edge if they must.
std::vector<TestObject> lay_out(std::vector<indexed_triangle_set> pieces, bool split, const std::vector<DynamicPrintConfig> &configs = {});
// Sets `objects` up as a new project called `name`, printed flat (Non-planar (S4) off unless an
// object's settings turn it on) and without a brim, in `layer` mm layers from the first on (0.1
// mm for nozzles under 0.3 mm if 0), so heights in whole tenths of 0.2 mm print true. False if
// the user keeps the current project.
bool open_test_print(Plater *plater, const wxString &name, const std::vector<TestObject> &objects, double layer = 0.);

} // namespace CalibrationKit

} // namespace GUI
} // namespace Slic3r
