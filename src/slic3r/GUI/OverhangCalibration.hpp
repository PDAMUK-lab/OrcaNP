#pragma once

// Orca: the limits of what a printer prints unsupported (Calibration > Overhangs and gaps): the
// steepest overhang, for Non-planar (S4)'s Maximum overhang; the gap under a supported surface
// that still comes away cleanly, for support; and the same over a generated print surface (S4).

#include "CalibrationDialogKit.hpp"

namespace Slic3r {
namespace GUI {

class OverhangGapsDialog : public CalibrationTabsDialog
{
public:
    OverhangGapsDialog(wxWindow *parent, Plater *plater);

private:
    void on_create_overhangs(wxCommandEvent &event);
    void on_apply_overhangs(wxCommandEvent &event);
    void on_create_support(wxCommandEvent &event);
    void on_apply_support(wxCommandEvent &event);
    void on_create_surface(wxCommandEvent &event);
    void on_apply_surface(wxCommandEvent &event);

    TextInput *m_overhang;
    TextInput *m_support_gap;
    TextInput *m_surface_gap;
};

} // namespace GUI
} // namespace Slic3r
