#pragma once

// Orca: polar alignment calibration. Two thin rings are printed about the bed's rotation axis; from
// their measured outer diameters the radius axis offset and scale of the printer are corrected.

#include "GUI_Utils.hpp"

class TextInput;
class wxStaticText;

namespace Slic3r {
namespace GUI {

class Plater;

class PolarAlignmentDialog : public DPIDialog
{
public:
    PolarAlignmentDialog(wxWindow *parent, Plater *plater);
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    // The ring radii and height, or false after telling the user what is wrong.
    bool read_rings(double &r1, double &r2, double &height);
    void on_create(wxCommandEvent &event);
    void on_apply(wxCommandEvent &event);

    Plater       *m_plater;
    TextInput    *m_inner;
    TextInput    *m_outer;
    TextInput    *m_height;
    TextInput    *m_inner_measured;
    TextInput    *m_outer_measured;
    wxStaticText *m_result;
};

} // namespace GUI
} // namespace Slic3r
