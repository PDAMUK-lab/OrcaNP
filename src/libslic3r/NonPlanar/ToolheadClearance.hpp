#pragma once

// Toolhead clearance calibration (Calibration > Toolhead clearance).
//
// The non-planar clearance check models the toolhead as a cone widening from the edge of the flat
// nozzle tip, rising at the clearance angle, up to the nozzle length above the tip, and a cylinder
// of the toolhead radius above that (Printer settings > Basic information > Non-planar toolhead).
// Two printed gauges measure a real toolhead, from which those settings are fitted: feeler blades
// slid under a flat underside (small clearance angles, a gap over a distance) and angle wedges slid
// against the nozzle tip (steep clearance angles, read directly). A third print, thin fins shaped
// to the settings just inside the modelled toolhead, checks them all round: the nozzle lowered
// into their middle touches no fin unless the settings are too generous somewhere.

#include "../TriangleMesh.hpp"

#include <string>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

// The toolhead as the printer settings describe it.
struct ToolheadClearance
{
    double tip_diameter = 0.8;  // nonplanar_nozzle_tip_diameter
    double angle        = 50.;  // nonplanar_nozzle_clearance_angle, degrees above the tip's face
    double length       = 5.;   // nonplanar_nozzle_length
    double radius       = 20.;  // nonplanar_head_radius

    // How far the modelled toolhead reaches from the nozzle's axis: its radius, or the cone's top
    // where that is wider.
    double reach() const;
    // Height above the nozzle tip of the modelled toolhead's underside at `distance` from the
    // nozzle's axis, up to reach(): 0 across the tip, then the cone, then the cylinder's base.
    double underside(double distance) const;
};

// A side of the toolhead measured with the feeler blades: with the nozzle touching the bed, the
// thickest blade that slides under the outer edge of the toolhead's underside (`gap`), and that
// edge's distance from the nozzle's centre.
struct FeelerReading
{
    double gap      = 0.;
    double distance = 0.;
};

// The settings for feeler readings: the clearance angle is the shallowest rise from the tip's edge
// to a reading, and the cone reaches out to the toolhead's reach (the farthest of `reach` and the
// readings' distances), where the cylinder takes over. False, with `error` set, for readings that
// fit no toolhead.
bool fit_feeler_readings(const std::vector<FeelerReading> &readings, double tip_diameter, double reach,
                         ToolheadClearance &out, std::string &error);

// The settings for wedge readings: the steepest wedge that reaches the nozzle tip on each side. The
// clearance angle is the smallest; the cone reaches out to `reach`.
bool fit_wedge_readings(const std::vector<double> &angles, double tip_diameter, double reach, ToolheadClearance &out,
                        std::string &error);

// The gauges, each as one mesh of separate pieces laid out in rows no wider than `max_width`,
// centred on the origin, standing on Z = 0. Heights are whole multiples of 0.1 mm, so they print
// true in 0.1 or 0.2 mm layers.

// Blade thicknesses, mm.
std::vector<double> feeler_thicknesses();
// A feeler blade per thickness: a thin blade and a thicker handle numbered with its thickness.
indexed_triangle_set feeler_blades(const std::vector<double> &thicknesses, double max_width);

// Wedge angles, degrees.
std::vector<double> wedge_angles();
// A wedge per angle: a slope rising from a thin edge at the angle, long enough to reach `reach`
// (up to 30 mm high), and a tab numbered with the angle.
indexed_triangle_set angle_wedges(const std::vector<double> &angles, double reach, double max_width);

// The fin test for the toolhead settings `t`: a plate `fin_test_plate` thick with twelve radial
// fins, `fin_thickness` thick, whose tops stay `fin_test_margin` below the modelled toolhead's
// underside with the nozzle tip on the plate's top at its centre, out to the toolhead's reach;
// beyond it, 1 mm on, the fins rise 3 mm above the nozzle length (at most 30 mm), so a toolhead
// wider than its radius meets them. Centred on the origin.
constexpr double fin_test_plate  = 1.;
constexpr double fin_test_margin = 0.3;
indexed_triangle_set fin_test(const ToolheadClearance &t, double fin_thickness);

// Raised seven-segment digits (0-9 and '.') for `text`, `height` tall and `depth` thick, from
// (x, y) along +X on the plane Z = z. Other characters leave a space.
void append_label(indexed_triangle_set &its, const std::string &text, double x, double y, double z, double height, double depth);
// Width of append_label()'s text.
double label_width(const std::string &text, double height);

} // namespace NonPlanar
} // namespace Slic3r
