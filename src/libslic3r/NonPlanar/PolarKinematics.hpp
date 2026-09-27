#pragma once

// Polar (R-theta) kinematics for non-planar printers.
//
// The machine has a rotating bed (the angle axis), a head that moves along one radial line
// through the bed's rotation axis (the radius axis), Z, and optionally a nozzle that tilts in
// the radial plane (the tilt axis). The slicer works in the part's Cartesian frame; this module
// converts Cartesian tool poses to machine coordinates and back, and rewrites a Cartesian
// G-code stream into machine G-code.
//
// Only Eigen and the standard library are used, so the kinematics can be tested without the
// rest of libslic3r.

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

struct PolarKinematicsConfig
{
    // Rotation axis of the bed, in the slicer's XY frame.
    Eigen::Vector2d center { 0., 0. };
    // +1 when the angle axis value equals the polar angle of the point under the nozzle,
    // -1 when the bed turns the other way.
    double angle_sign = 1.;
    // Allow a negative radius, so a path crossing the rotation axis continues through it
    // instead of turning the bed by 180 degrees.
    bool signed_radius = true;
    // Inside this radius the polar angle is ill-conditioned: the angle is held and only the
    // radius moves. The path deviates from the Cartesian line by at most this distance.
    double min_radius = 0.05;
    // Limits for splitting a straight Cartesian move into machine segments.
    double max_angle_step     = 1.;  // degrees of angle-axis travel per segment
    double max_segment_length = 2.;  // mm of Cartesian travel per segment
    // Axis speed limits, used to stretch the duration of a segment in inverse-time mode.
    double max_angular_speed = 36000.; // deg/min for the angle axis
    double max_tilt_speed    = 18000.; // deg/min for the tilt axis
    // G93 inverse-time feed (F = 1 / minutes per move) instead of a G94 feedrate.
    bool inverse_time_feed = true;
    // The machine has a tilt axis. Without one the tilt is ignored and never emitted.
    bool has_tilt_axis = true;
    // Travel of the tilt axis (degrees, positive leaning toward +radius). A pose leaning further
    // is printed at the limit.
    double min_tilt = -90.;
    double max_tilt = 90.;
    // -1 when the machine's tilt axis counts the other way: positive toward the rotation axis.
    double tilt_sign = 1.;
    // Travel of the radius axis (mm, as commanded, after pivot compensation). Moves outside it
    // are counted, not changed.
    double min_travel_radius = -std::numeric_limits<double>::infinity();
    double max_travel_radius = std::numeric_limits<double>::infinity();
    // Distance from the nozzle tip to the tilt pivot. Non-zero when the firmware positions the
    // pivot rather than the tip, so the commanded radius and Z must be compensated.
    double tilt_pivot_length = 0.;
    char angle_axis  = 'C';
    char radius_axis = 'X';
    char tilt_axis   = 'B';
};

// Tool pose in the part's Cartesian frame.
struct ToolPose
{
    Eigen::Vector3d tip { 0., 0., 0. };
    // Nozzle axis angle from vertical in the radial plane (radians), positive when the nozzle
    // leans outward, away from the rotation axis.
    double tilt = 0.;
};

// Machine coordinates.
struct MachinePose
{
    double angle  = 0.; // degrees, continuous (not wrapped)
    double radius = 0.; // mm along the head's radial line, signed when signed_radius
    double z      = 0.;
    double tilt   = 0.; // degrees, as commanded (positive toward +radius unless tilt_sign is -1)
    bool   tilt_limited = false; // the pose wanted more tilt than the axis travel allows
};

class PolarKinematics
{
public:
    explicit PolarKinematics(const PolarKinematicsConfig &config) : m_config(config) {}

    const PolarKinematicsConfig& config() const { return m_config; }

    // Machine pose reaching `pose`, continuing from `prev` with the least angle-axis travel.
    // Without `prev` the angle is taken in (-180, 180].
    MachinePose to_machine(const ToolPose &pose, const MachinePose *prev) const;
    // Inverse of to_machine().
    ToolPose to_tool(const MachinePose &machine) const;

    // Splits the straight Cartesian move `from` -> `to` into machine poses (excluding the start,
    // including the end) so that every segment stays within max_angle_step and
    // max_segment_length. Each entry carries the fraction of the Cartesian move it ends at.
    struct Waypoint
    {
        double      t;
        MachinePose pose;
    };
    std::vector<Waypoint> interpolate(const ToolPose &from, const MachinePose &from_machine, const ToolPose &to) const;

private:
    PolarKinematicsConfig m_config;
};

// Rewrites Cartesian G-code into polar machine G-code.
//
// G0/G1 moves are converted and subdivided, G2/G3 arcs are linearized first, and everything else
// (comments, M-codes, retractions) is passed through. The machine start and end G-code, delimited
// by Orca's "; MACHINE_START_GCODE_END" and "; MACHINE_END_GCODE_START" tags, are already written
// for the machine and are copied verbatim. A tilt word (the tilt axis letter) on a Cartesian move
// is read as the radial tilt in degrees and is modal.
class PolarGCodeConverter
{
public:
    struct Stats
    {
        size_t cartesian_moves  = 0;
        size_t machine_moves    = 0;
        size_t arcs_linearized  = 0;
        size_t held_near_center = 0; // machine poses inside min_radius
        size_t tilt_limited     = 0; // machine poses whose tilt was cut back to the tilt travel
        size_t radius_outside   = 0; // machine poses beyond the radius travel
        double min_radius = std::numeric_limits<double>::infinity(), max_radius = -std::numeric_limits<double>::infinity();
        double total_angle      = 0.; // degrees of angle-axis travel
    };

    // What became of an input line: the output lines written up to and including it, and the
    // machine pose after it (once the machine position is known).
    struct LineRecord
    {
        uint32_t out_lines = 0;
        bool     posed     = false;
        float    angle = 0.f, radius = 0.f, z = 0.f, tilt = 0.f; // as commanded
    };

    explicit PolarGCodeConverter(const PolarKinematicsConfig &config) : m_kinematics(config) {}

    void               process(std::istream &in, std::ostream &out);
    const Stats&       stats() const { return m_stats; }
    // One record per input line, in order.
    const std::vector<LineRecord>& line_records() const { return m_lines; }

private:
    void process_line(const std::string &line, std::ostream &out);
    void emit_move(const ToolPose &to, double e, double feed, bool rapid, std::ostream &out, const std::string &comment);
    void set_feed_mode(bool inverse_time, std::ostream &out);

    PolarKinematics         m_kinematics;
    Stats                   m_stats;
    std::vector<LineRecord> m_lines;

    enum class Block { Body, Start, End };
    Block       m_block            = Block::Body;
    bool        m_absolute_xyz     = true;
    bool        m_relative_e       = false;
    double      m_e                = 0.;   // absolute E position of the Cartesian stream
    double      m_e_rounding       = 0.;   // relative E not yet emitted because of rounding
    double      m_feed             = 1500.;
    ToolPose    m_pose;
    bool        m_have_machine     = false;
    MachinePose m_machine;
    int         m_feed_mode        = -1;  // -1 unknown, 0 G94, 1 G93
};

// Converts a whole G-code file in place of `dst`. Returns the conversion statistics.
PolarGCodeConverter::Stats convert_gcode_to_polar(const std::string &src, const std::string &dst, const PolarKinematicsConfig &config);

} // namespace NonPlanar
} // namespace Slic3r
