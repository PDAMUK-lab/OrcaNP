#include "DimensionsCalibration.hpp"

#include "I18N.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/TextInput.hpp"

#include "libslic3r/CalibrationPrints.hpp"

#include <cmath>

namespace Slic3r {
namespace GUI {

namespace CP = CalibrationPrints;
using namespace CalibrationKit;

namespace {

constexpr const char *FRAME_ROWS[3]    = { "flat", "upright_1", "upright_2" };
constexpr const char *FRAME_COLUMNS[3] = { "ac", "bd", "ad" };

wxString mm() { return _L("mm"); }

} // namespace

DimensionsDialog::DimensionsDialog(wxWindow *parent, Plater *plater)
    : CalibrationTabsDialog(parent, plater, _L("Dimensions"), "dimensions_calibration",
                            _L("Measures how true to size parts print, for the settings that correct it: the filament's shrinkage, "
                               "the printer's axis skew, and the process's X-Y hole and contour compensation and elephant foot "
                               "compensation. Each test sets up its print as a new project: print it, let it cool, then open this "
                               "again and enter what you measure with calipers. Do the tests in order, as each corrects what the "
                               "next measures."))
{
    const std::vector<double> diameters = CP::hole_peg_diameters();

    // 1. Shrinkage and skew
    wxPanel *frames = add_page(_L("1. Shrinkage and skew"),
                               _L("Prints a flat square frame and two upright ones, numbered 1 (along X) and 2 (along Y) on a tab at "
                                  "their foot, with an eight-sided boss at each corner. Print it with the filament to calibrate and "
                                  "let it cool fully. The flat frame's corners are lettered A, b, C and d; on an upright frame, A is "
                                  "the bottom corner nearer its number, b the other bottom corner, d above A and C above b. Measure "
                                  "between the bosses' outer sides: across the diagonals A to C and b to d, and along the side A to "
                                  "d; and one boss across, side to side. Shrinkage is the filament's, as a part cools, and applies to "
                                  "the filament preset. Skew, axes out of square, is the printer's and applies to the printer, which "
                                  "then shears the print's moves to square them; leave it to the firmware instead if it compensates "
                                  "skew (Klipper SET_SKEW, Marlin M852, RepRapFirmware M556)."));
    auto *create_frames = add_button(frames, _L("Create frames"));
    auto *boss_row      = add_grid(frames, 2);
    boss_row->Add(new wxStaticText(frames, wxID_ANY, _L("A boss, across") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_boss = add_input(frames, "boss", mm());
    boss_row->Add(m_boss);
    auto *grid = add_grid(frames, 4);
    grid->Add(new wxStaticText(frames, wxID_ANY, ""));
    for (const wxString &column : { _L("A to C"), _L("b to d"), _L("A to d") })
        grid->Add(new wxStaticText(frames, wxID_ANY, column));
    const wxString rows[3] = { _L("Flat frame"), _L("Upright frame 1 (along X)"), _L("Upright frame 2 (along Y)") };
    for (int r = 0; r < 3; ++ r) {
        grid->Add(new wxStaticText(frames, wxID_ANY, rows[r] + ": "), 0, wxALIGN_CENTER_VERTICAL);
        for (int c = 0; c < 3; ++ c) {
            m_frames[3 * r + c] = add_input(frames, std::string(FRAME_ROWS[r]) + "_" + FRAME_COLUMNS[c], mm());
            grid->Add(m_frames[3 * r + c]);
        }
    }
    auto *apply_shrinkage = add_button(frames, _L("Apply shrinkage to filament"));
    DialogButtons *apply_skew = nullptr;
    if (! is_polar())
        apply_skew = add_button(frames, _L("Apply skew to printer"));

    // 2. Holes and pegs
    wxPanel *holes = add_page(_L("2. Holes and pegs"),
                              _L("Prints a plate with a round hole and a round peg of each size, numbered with its diameter in mm. "
                                 "Measure the diameters of as many as you like and apply: X-Y hole compensation sizes holes and X-Y "
                                 "contour compensation the outside of parts (Process > Quality > Precision), each grown by half what "
                                 "the features measure short. Calibrate shrinkage first, as it scales every size."));
    auto *create_holes = add_button(holes, _L("Create holes and pegs"));
    auto *hole_grid    = add_grid(holes, 3);
    hole_grid->Add(new wxStaticText(holes, wxID_ANY, ""));
    hole_grid->Add(new wxStaticText(holes, wxID_ANY, _L("Hole")));
    hole_grid->Add(new wxStaticText(holes, wxID_ANY, _L("Peg")));
    for (size_t i = 0; i < diameters.size() && i < 4; ++ i) {
        const std::string d = CP::format_value(diameters[i], 0);
        hole_grid->Add(new wxStaticText(holes, wxID_ANY, wxString::Format(_L("%s mm"), wxString::FromUTF8(d)) + ": "), 0,
                       wxALIGN_CENTER_VERTICAL);
        m_holes[i] = add_input(holes, "hole_" + d, mm());
        m_pegs[i]  = add_input(holes, "peg_" + d, mm());
        hole_grid->Add(m_holes[i]);
        hole_grid->Add(m_pegs[i]);
    }
    auto *apply_holes = add_button(holes, _L("Apply to process"));

    // 3. Elephant foot
    wxPanel *block = add_page(_L("3. Elephant foot"),
                              wxString::Format(_L("Prints a %s mm block. The first layer, pressed onto the bed, spreads wider than the "
                                                  "layers above it. Measure the block across the same two sides at the very bottom, "
                                                  "the first layer, and half way up, and apply: Elephant foot compensation (Process > "
                                                  "Quality > Precision) shrinks the first layer by half the difference."),
                                               CP::format_value(CP::elephant_foot_block_size, 0)));
    auto *create_block = add_button(block, _L("Create block"));
    auto *block_grid   = add_grid(block, 2);
    block_grid->Add(new wxStaticText(block, wxID_ANY, _L("At the bottom") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_bottom = add_input(block, "block_bottom", mm());
    block_grid->Add(m_bottom);
    block_grid->Add(new wxStaticText(block, wxID_ANY, _L("Half way up") + ": "), 0, wxALIGN_CENTER_VERTICAL);
    m_middle = add_input(block, "block_middle", mm());
    block_grid->Add(m_middle);
    auto *apply_block = add_button(block, _L("Apply to process"));

    create_frames->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_create_frames, this);
    apply_shrinkage->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_apply_shrinkage, this);
    if (apply_skew != nullptr)
        apply_skew->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_apply_skew, this);
    create_holes->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_create_holes, this);
    apply_holes->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_apply_holes, this);
    create_block->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_create_block, this);
    apply_block->GetOK()->Bind(wxEVT_BUTTON, &DimensionsDialog::on_apply_block, this);
    finish();
}

void DimensionsDialog::on_create_frames(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Shrinkage and skew frames"), { { CP::square_frames(CP::SquareFrames()), Vec2d::Zero(), {} } });
}

bool DimensionsDialog::fit_frames(CP::FramesFit &out)
{
    std::vector<double> values;
    double              boss = 0.;
    if (! read(m_boss, boss) || ! read_all({ m_frames.begin(), m_frames.end() }, values, _L("the boss and every frame's diagonals and side")))
        return false;
    std::string error;
    if (! CP::fit_square_frames(CP::SquareFrames(), boss, { values[0], values[1], values[2] }, { values[3], values[4], values[5] },
                                { values[6], values[7], values[8] }, out, error)) {
        warn(from_u8(error));
        return false;
    }
    return true;
}

void DimensionsDialog::on_apply_shrinkage(wxCommandEvent &)
{
    remember();
    CP::FramesFit fit;
    if (! fit_frames(fit))
        return;
    // The frames were printed scaled by the shrinkage already set: what they measure is the rest.
    const DynamicPrintConfig &filament = filament_config();
    const double xy = filament.option<ConfigOptionPercents>("filament_shrink")->get_at(0) * 0.5 * (fit.scale_x + fit.scale_y);
    const double z  = filament.option<ConfigOptionPercents>("filament_shrinkage_compensation_z")->get_at(0) * fit.scale_z;
    DynamicPrintConfig values;
    values.set_key_value("filament_shrink", new ConfigOptionPercents({ xy }));
    values.set_key_value("filament_shrinkage_compensation_z", new ConfigOptionPercents({ z }));
    apply(Preset::TYPE_FILAMENT, values);
    wxString text = wxString::Format(_L("Shrinkage (XY) %s %%, Shrinkage (Z) %s %%. Save the filament preset to keep them."),
                                     number(xy, 2), number(z, 2));
    if (std::abs(fit.scale_x - fit.scale_y) > 0.002)
        text += " " + wxString::Format(_L("X printed %s %% larger than Y: that is the printer's (its steps or belts), not the "
                                          "filament's, and the shrinkage set is their mean."),
                                       number(100. * (fit.scale_x / fit.scale_y - 1.), 2));
    show_result(text);
}

void DimensionsDialog::on_apply_skew(wxCommandEvent &)
{
    remember();
    CP::FramesFit fit;
    if (! fit_frames(fit))
        return;
    // Printed with the skew already set, the frames show what is left of it.
    const DynamicPrintConfig &printer = printer_config();
    const double xy = printer.opt_float("skew_xy") + fit.skew_xy, xz = printer.opt_float("skew_xz") + fit.skew_xz,
                 yz = printer.opt_float("skew_yz") + fit.skew_yz;
    if (std::abs(xy) > 5. || std::abs(xz) > 5. || std::abs(yz) > 5.) {
        warn(_L("These measurements give a skew of more than 5 degrees: please check them, and the printer's frame."));
        return;
    }
    DynamicPrintConfig values;
    values.set_key_value("skew_xy", new ConfigOptionFloat(xy));
    values.set_key_value("skew_xz", new ConfigOptionFloat(xz));
    values.set_key_value("skew_yz", new ConfigOptionFloat(yz));
    apply(Preset::TYPE_PRINTER, values);
    show_result(wxString::Format(_L("XY skew %s°, XZ skew %s°, YZ skew %s° (Printer settings > Basic information > Axis skew). Save "
                                    "the printer preset to keep them. Frames printed again should then measure square."),
                                 number(xy, 3), number(xz, 3), number(yz, 3)));
}

void DimensionsDialog::on_create_holes(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Holes and pegs"), { { CP::holes_and_pegs(CP::hole_peg_diameters()), Vec2d::Zero(), {} } });
}

void DimensionsDialog::on_apply_holes(wxCommandEvent &)
{
    remember();
    const std::vector<double> designed = CP::hole_peg_diameters();
    std::vector<double>       holes(designed.size(), 0.), pegs(designed.size(), 0.);
    bool                      any_hole = false, any_peg = false;
    for (size_t i = 0; i < designed.size() && i < 4; ++ i) {
        any_hole |= read(m_holes[i], holes[i]) && holes[i] > 0.;
        any_peg |= read(m_pegs[i], pegs[i]) && pegs[i] > 0.;
    }
    if (! any_hole && ! any_peg) {
        warn(_L("Please input at least one hole or peg diameter."));
        return;
    }
    DynamicPrintConfig values;
    wxString           text;
    std::string        error;
    double             grow = 0.;
    if (any_hole) {
        if (! CP::fit_xy_compensation(designed, holes, grow, error)) {
            warn(from_u8(error));
            return;
        }
        const double v = print_config().opt_float("xy_hole_compensation") + grow;
        values.set_key_value("xy_hole_compensation", new ConfigOptionFloat(v));
        text += wxString::Format(_L("X-Y hole compensation %s mm."), number(v, 3)) + " ";
    }
    if (any_peg) {
        if (! CP::fit_xy_compensation(designed, pegs, grow, error)) {
            warn(from_u8(error));
            return;
        }
        const double v = print_config().opt_float("xy_contour_compensation") + grow;
        values.set_key_value("xy_contour_compensation", new ConfigOptionFloat(v));
        text += wxString::Format(_L("X-Y contour compensation %s mm."), number(v, 3)) + " ";
    }
    apply(Preset::TYPE_PRINT, values);
    show_result(text + _L("Save the process preset to keep them."));
}

void DimensionsDialog::on_create_block(wxCommandEvent &)
{
    remember();
    EndModal(wxID_OK);
    open_test_print(m_plater, _L("Elephant foot block"), { { CP::elephant_foot_block(), Vec2d::Zero(), {} } });
}

void DimensionsDialog::on_apply_block(wxCommandEvent &)
{
    remember();
    std::vector<double> values;
    if (! read_all({ m_bottom, m_middle }, values, _L("both widths")))
        return;
    const double size = CP::elephant_foot_block_size;
    if (std::abs(values[0] - size) > 2. || std::abs(values[1] - size) > 2.) {
        warn(wxString::Format(_L("The block should measure about %s mm: please check the measurements."), CP::format_value(size, 0)));
        return;
    }
    // Printed with the compensation already set, the block shows what is left.
    const double v = std::max(0., print_config().opt_float("elefant_foot_compensation") + 0.5 * (values[0] - values[1]));
    DynamicPrintConfig config;
    config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(v));
    apply(Preset::TYPE_PRINT, config);
    show_result(wxString::Format(_L("Elephant foot compensation %s mm. Save the process preset to keep it."), number(v, 3)));
}

} // namespace GUI
} // namespace Slic3r
