#pragma once

// Orca: a polar printer's calibration (Calibration > Polar calibration): where its rotation axis is
// (radius offset and scale), how its bed sits under the nozzle, the play in its bed's drive, its
// tilt pivot distance and tilt offset, and how fast its bed may turn.

#include "CalibrationDialogKit.hpp"

#include "libslic3r/calib.hpp"

#include <array>

namespace Slic3r {
namespace GUI {

class PolarCalibrationDialog : public CalibrationTabsDialog
{
public:
    PolarCalibrationDialog(wxWindow *parent, Plater *plater);

private:
    // The rings' radii and height, or false after saying what is wrong.
    bool read_rings(double &r1, double &r2, double &height);
    void on_create_rings(wxCommandEvent &event);
    void on_apply_rings(wxCommandEvent &event);
    void on_create_bed(wxCommandEvent &event);
    void on_apply_bed(wxCommandEvent &event);
    void on_create_backlash(wxCommandEvent &event);
    void on_apply_backlash(wxCommandEvent &event);
    void on_create_tilt(wxCommandEvent &event);
    void on_apply_tilt(wxCommandEvent &event);
    void on_create_speed(wxCommandEvent &event);
    void on_apply_speed(wxCommandEvent &event);
    // Sets up a test that is written as G-code (`params`) as a new project.
    void open_gcode_test(const wxString &name, const Calib_Params &params);

    TextInput                *m_inner;
    TextInput                *m_outer;
    TextInput                *m_height;
    TextInput                *m_inner_measured;
    TextInput                *m_outer_measured;
    std::array<TextInput *, 8> m_bed;
    TextInput                *m_pair_1;
    TextInput                *m_pair_2;
    std::array<TextInput *, 6> m_tubes; // inner tube lower and upper diameter, outer tube's, then heights
    TextInput                *m_last_ring;
};

} // namespace GUI
} // namespace Slic3r
