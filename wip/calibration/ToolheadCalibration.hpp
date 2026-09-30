#pragma once

// Orca: toolhead clearance calibration. Printed feeler blades or angle wedges measure how close to
// the nozzle tip the toolhead comes, and the non-planar toolhead settings are fitted to what is
// measured; a printed fin test then checks the settings all round.

#include "CalibrationDialogKit.hpp"

#include <array>

namespace Slic3r {
namespace GUI {

class ToolheadClearanceDialog : public CalibrationTabsDialog
{
public:
    ToolheadClearanceDialog(wxWindow *parent, Plater *plater);

private:
    void on_create_blades(wxCommandEvent &event);
    void on_create_wedges(wxCommandEvent &event);
    void on_create_fin_test(wxCommandEvent &event);
    void on_apply_blades(wxCommandEvent &event);
    void on_apply_wedges(wxCommandEvent &event);
    // The printer's current toolhead settings, which the fin test checks.
    void update_fin_text();

    TextInput                *m_reach;
    std::array<TextInput *, 4> m_gap;
    std::array<TextInput *, 4> m_distance;
    std::array<TextInput *, 4> m_angle;
    wxStaticText             *m_fin_text;
};

} // namespace GUI
} // namespace Slic3r
