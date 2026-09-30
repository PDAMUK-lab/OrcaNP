#pragma once

// Orca: toolhead clearance calibration. Printed feeler blades or angle wedges measure how close to
// the nozzle tip the toolhead comes, and the non-planar toolhead settings are fitted to what is
// measured; a printed fin test then checks the settings all round.

#include "GUI_Utils.hpp"

#include <array>

class TextInput;
class wxStaticText;

namespace Slic3r {
namespace GUI {

class Plater;

class ToolheadClearanceDialog : public DPIDialog
{
public:
    ToolheadClearanceDialog(wxWindow *parent, Plater *plater);
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    void on_create_blades(wxCommandEvent &event);
    void on_create_wedges(wxCommandEvent &event);
    void on_create_fin_test(wxCommandEvent &event);
    void on_apply_blades(wxCommandEvent &event);
    void on_apply_wedges(wxCommandEvent &event);
    // Keeps what was entered, for when the dialog is opened again after printing a gauge.
    void remember();
    // The printer's current toolhead settings, which the fin test checks.
    void update_fin_text();
    void show_result(const wxString &text);

    Plater                   *m_plater;
    TextInput                *m_reach;
    std::array<TextInput *, 4> m_gap;
    std::array<TextInput *, 4> m_distance;
    std::array<TextInput *, 4> m_angle;
    wxStaticText             *m_fin_text;
    wxStaticText             *m_result;
};

} // namespace GUI
} // namespace Slic3r
