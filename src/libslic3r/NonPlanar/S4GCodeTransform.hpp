#pragma once

// Turns G-code sliced from an S4-deformed mesh into G-code for the real part.
//
// Every move is subdivided in the sliced (deformed) space and mapped back through the
// tetrahedral mesh; flat sliced layers become curved. The result is Cartesian G-code in the
// real part's frame with the nozzle tilt as an extra axis word, ready for the polar conversion.
//
// Repair passes then make it printable: extrusion is rescaled by how much each layer was
// squashed (bounded, with the clipped material carried forward), nothing goes below the bed,
// zero-length moves are folded away, Z steps are limited, and travels that would drag the
// nozzle through printed material are lifted over it and retracted.
//
// Structure: one record per source line owning the lines it produces, so markers, feedrates and
// the end block cannot drift out of step with the motion.

#include "S4Mapping.hpp"

#include <Eigen/Core>

#include <iosfwd>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

struct S4GCodeConfig
{
    // G-code coordinates = mesh coordinates + offset, for both the sliced input and the output.
    Eigen::Vector3d offset { 0., 0., 0. };

    // Subdivision in sliced space.
    double seg_size              = 0.6; // mm, printing moves
    double travel_seg_size       = 2.;  // mm, travels
    int    max_segments_per_move = 256;

    // Extrusion scaling by the undeformed/deformed volume ratio.
    enum class FlowModel { Volumetric, Length, Hybrid };
    FlowModel flow_model        = FlowModel::Volumetric;
    double    min_flow          = 0.25;
    double    max_flow          = 3.;
    bool      conserve_clipped  = true;  // carry clipped material into the next segments
    double    max_residual      = 0.05;  // mm of filament the carry may hold per segment

    // Geometry.
    double z_floor            = 0.;
    double max_dz_per_segment = 0.;    // 0: seg_size, i.e. at most a 45 degree step
    double degenerate_tol     = 5e-4;  // mm; shorter moves are folded into the next one

    // Travels.
    bool   travel_safety                   = true;
    double min_travel_for_retract          = 12.;
    double min_travel_for_collision_retract = 3.;
    double retract_length                  = 0.; // 0: most common retraction in the file
    double retract_feed                    = 0.;
    double z_hop                           = 0.5;
    double z_hop_feed                      = 600.;
    double travel_clearance                = 0.15;
    double height_field_res                = 0.8;
    double nozzle_radius                   = 0.4;

    // Nozzle tilt (radians, positive leaning away from the rotation axis). Where the layer leans
    // less than `tilt_threshold` the nozzle stays vertical; from there to twice the threshold it
    // catches up with the layer. It never leans more than `max_tilt` either way, nor outside
    // [min_tilt_toward, max_tilt_away].
    double tilt_threshold  = 0.;
    double max_tilt        = 1.5707963267948966; // 90 degrees
    double min_tilt_toward = -1.5707963267948966;
    double max_tilt_away   = 1.5707963267948966;

    // Clearance of the nozzle and the head around material already printed, and above the bed
    // (at z_floor). The nozzle is a cone of half-angle `nozzle_cone_angle` (radians, from the
    // nozzle axis) widening from `nozzle_tip_radius` (the edge of the flat tip) up to
    // `nozzle_length`, and the head above it a cylinder of `head_radius`, both along the nozzle
    // axis. Material within `clearance_ignore_radius` of the tip (the beads it is laying
    // and touching) and less than `clearance_tolerance` inside the head is not a collision.
    bool   clearance_check          = false;
    double nozzle_cone_angle        = 0.698; // 40 degrees
    double nozzle_tip_radius        = 0.4;
    double nozzle_length            = 5.;
    double head_radius              = 20.;
    double clearance_tolerance      = 0.2;
    double clearance_ignore_radius  = 2.;
    double clearance_check_interval = 2.; // mm of nozzle travel between checks

    // Output.
    bool emit_tilt = true;
    char tilt_axis = 'B';
};

struct S4GCodeReport
{
    size_t source_lines = 0, output_lines = 0, segments = 0;
    size_t inside = 0, nearest = 0, idw = 0, outside = 0;
    size_t floor_clamped = 0, flow_clipped_high = 0, flow_clipped_low = 0;
    double filament_in = 0., filament_out = 0.;
    size_t degenerate_dropped = 0, z_steps_limited = 0;
    size_t collisions = 0, lifts = 0, retractions_added = 0;
    double min_z = 0., max_z = 0., max_tilt_deg = 0.;
    size_t tilt_limited = 0; // points whose tilt was cut back to max_tilt
    size_t trimmed      = 0; // extruding segments outside their object's kept window
    size_t junctions    = 0; // travels between differently mapped toolpaths
    // Checked positions where the nozzle or the head would hit printed material, with the first
    // few described.
    size_t                   head_collisions = 0;
    std::vector<std::string> head_collision_samples;
    // Names and details of failed structural checks; empty when the output is sound.
    std::vector<std::string> failed;
};

// How the toolpath of one object is mapped back.
struct S4ObjectMapping
{
    const S4Mapper *mapper = nullptr; // null: printed as sliced
    // Sliced space up to this height is the object's print surface, printed as sliced.
    double identity_below_z = -std::numeric_limits<double>::infinity();
    // Only extrusion whose sliced X lies in [keep_min_x, keep_max_x) is printed: an unwrapped
    // layer is sliced over more than a turn, and the rest repeats what is printed here.
    double keep_min_x = -std::numeric_limits<double>::infinity();
    double keep_max_x = std::numeric_limits<double>::infinity();
};

// The mapping of each object, by the id in the G-code's object markers: the lines between
// "; NONPLANAR_OBJECT <id>" and "; NONPLANAR_OBJECT_END" belong to object <id>. Everything else
// (and objects not listed) uses the fallback. A travel between differently mapped toolpaths
// becomes a straight line between where they end and start in the part.
struct S4MapperSet
{
    S4ObjectMapping                fallback;
    std::map<int, S4ObjectMapping> objects;
};

// Throws std::runtime_error for input it cannot handle (absolute extrusion, no body).
S4GCodeReport s4_transform_gcode(std::istream &in, std::ostream &out, const S4MapperSet &mappers, const S4GCodeConfig &config);
// All of the G-code through one mapper.
S4GCodeReport s4_transform_gcode(std::istream &in, std::ostream &out, const S4Mapper &mapper, const S4GCodeConfig &config);

} // namespace NonPlanar
} // namespace Slic3r
