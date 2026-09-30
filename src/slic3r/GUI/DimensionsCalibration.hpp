#pragma once

// Orca: dimensional calibration (Calibration > Dimensions): the filament's shrinkage and the
// printer's axis skew from printed square frames, X-Y hole and contour compensation from holes
// and pegs, and elephant foot compensation from a block.

#include "CalibrationDialogKit.hpp"

#include <array>

namespace Slic3r {
namespace CalibrationPrints {
struct FramesFit;
}
namespace GUI {

class DimensionsDialog : public CalibrationTabsDialog
{
public:
    DimensionsDialog(wxWindow *parent, Plater *plater);

private:
    void on_create_frames(wxCommandEvent &event);
    void on_apply_shrinkage(wxCommandEvent &event);
    void on_apply_skew(wxCommandEvent &event);
    void on_create_holes(wxCommandEvent &event);
    void on_apply_holes(wxCommandEvent &event);
    void on_create_block(wxCommandEvent &event);
    void on_apply_block(wxCommandEvent &event);
    // The frames' fit from what is entered, or false after saying what is wrong.
    bool fit_frames(CalibrationPrints::FramesFit &out);

    TextInput                *m_boss;
    std::array<TextInput *, 9> m_frames; // AC, BD, AD of the flat frame, upright 1, upright 2
    std::array<TextInput *, 4> m_holes;
    std::array<TextInput *, 4> m_pegs;
    TextInput                *m_bottom;
    TextInput                *m_middle;
};

} // namespace GUI
} // namespace Slic3r
