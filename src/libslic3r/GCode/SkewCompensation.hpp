#pragma once

// Axis skew compensation for Cartesian printers (Printer settings > Basic information > Axis
// skew): the print's moves are sheared so that axes out of square print square parts. The
// correction is Klipper's (SET_SKEW), applied to the G-code instead of in the firmware.

#include "../BoundingBox.hpp"
#include "../Point.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace Slic3r {

struct SkewCorrection
{
    // Tangents of the skew angles: how far each axis leans toward the one before it.
    double xy = 0., xz = 0., yz = 0.;

    static SkewCorrection from_degrees(double xy, double xz, double yz);
    bool empty() const { return xy == 0. && xz == 0. && yz == 0.; }
    // Where to send the axes for the nozzle to reach `p`. Linear, so it also takes relative moves.
    Vec3d apply(const Vec3d &p) const
    {
        return { p.x() - p.y() * xy - p.z() * (xz - xy * yz), p.y() - p.z() * yz, p.z() };
    }
};

// Rewrites the print's G0/G1 moves through a SkewCorrection, linearizing G2/G3 arcs first (an arc
// sheared is not an arc). The machine start and end G-code, delimited by Orca's
// "; MACHINE_START_GCODE_END" and "; MACHINE_END_GCODE_START" tags, are copied as they are.
class SkewGCodeConverter
{
public:
    struct Stats
    {
        size_t moves            = 0;
        size_t arcs_linearized  = 0;
        size_t outside_bed      = 0; // extruding moves ending outside the bed once sheared
    };

    SkewGCodeConverter(const SkewCorrection &skew, const BoundingBoxf &bed) : m_skew(skew), m_bed(bed) {}

    void         process(std::istream &in, std::ostream &out);
    const Stats &stats() const { return m_stats; }
    // For each input line, in order, the number of output lines written up to and including it.
    const std::vector<uint32_t> &out_lines() const { return m_out_lines; }

private:
    void process_line(const std::string &line, std::ostream &out);
    void write_move(const char *cmd, const Vec3d &to, bool has_z, const std::string &other_words, const std::string &comment, std::ostream &out);

    SkewCorrection        m_skew;
    BoundingBoxf          m_bed;
    Stats                 m_stats;
    std::vector<uint32_t> m_out_lines;

    enum class Block { Body, Start, End };
    Block  m_block        = Block::Body;
    bool   m_absolute_xyz = true;
    bool   m_relative_e   = false;
    double m_e            = 0.;
    // The nozzle's position as the G-code means it, and whether each axis is known yet.
    Vec3d m_pos   = Vec3d::Zero();
    bool  m_known[3] = { false, false, false };
};

} // namespace Slic3r
