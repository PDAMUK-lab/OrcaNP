#include "PolarKinematics.hpp"
#include "GCodeWords.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace Slic3r {
namespace NonPlanar {

namespace {

constexpr double PI = 3.14159265358979323846;

double rad2deg(double a) { return a * 180. / PI; }
double deg2rad(double a) { return a * PI / 180.; }

// Wraps an angle difference into (-pi, pi].
double wrap_pi(double a)
{
    a = std::fmod(a + PI, 2. * PI);
    if (a <= 0.)
        a += 2. * PI;
    return a - PI;
}

} // namespace

MachinePose PolarKinematics::to_machine(const ToolPose &pose, const MachinePose *prev) const
{
    const PolarKinematicsConfig &c = m_config;
    const Eigen::Vector2d d = pose.tip.head<2>() - c.center;
    const double r = d.norm();

    // Polar angle of the head's radial line in the part frame, and the radius along it.
    double phi;
    double radius;
    const double prev_phi = prev ? deg2rad(prev->angle / c.angle_sign) : 0.;
    if (prev && r < c.min_radius) {
        // Too close to the axis for a well-conditioned angle: keep the bed still and slide
        // the head along its current line.
        phi    = prev_phi;
        radius = d.dot(Eigen::Vector2d(std::cos(phi), std::sin(phi)));
        if (radius < 0. && ! c.signed_radius) {
            phi += PI;
            radius = -radius;
        }
    } else {
        phi    = std::atan2(d.y(), d.x());
        radius = r;
        if (prev) {
            phi = prev_phi + wrap_pi(phi - prev_phi);
            if (c.signed_radius) {
                // The same point is reached from the opposite side of the axis with a negative
                // radius; take whichever needs less rotation.
                const double flipped = prev_phi + wrap_pi(phi + PI - prev_phi);
                if (std::abs(flipped - prev_phi) < std::abs(phi - prev_phi)) {
                    phi    = flipped;
                    radius = -r;
                }
            }
        }
    }

    // Tilt is defined against the point's outward direction; the machine measures it against
    // the +radius direction of the head, which points inward when the radius is negative.
    double tilt = c.has_tilt_axis ? pose.tilt : 0.;
    if (radius < 0.)
        tilt = -tilt;
    const double limited = std::max(deg2rad(c.min_tilt), std::min(deg2rad(c.max_tilt), tilt));

    MachinePose m;
    m.tilt_limited = limited != tilt;
    tilt           = limited;
    m.angle  = rad2deg(phi) * c.angle_sign;
    m.radius = (radius + c.tilt_pivot_length * std::sin(tilt) - c.radius_offset) / c.radius_scale;
    m.z      = pose.tip.z() + c.tilt_pivot_length * (std::cos(tilt) - 1.);
    m.tilt   = rad2deg(tilt) * c.tilt_sign;
    return m;
}

ToolPose PolarKinematics::to_tool(const MachinePose &m) const
{
    const PolarKinematicsConfig &c = m_config;
    const double phi    = deg2rad(m.angle / c.angle_sign);
    const double tilt   = deg2rad(m.tilt * c.tilt_sign);
    const double radius = m.radius * c.radius_scale + c.radius_offset - c.tilt_pivot_length * std::sin(tilt);

    ToolPose pose;
    pose.tip.head<2>() = c.center + radius * Eigen::Vector2d(std::cos(phi), std::sin(phi));
    pose.tip.z()       = m.z - c.tilt_pivot_length * (std::cos(tilt) - 1.);
    pose.tilt          = radius < 0. ? -tilt : tilt;
    return pose;
}

std::vector<PolarKinematics::Waypoint> PolarKinematics::interpolate(const ToolPose &from, const MachinePose &from_machine, const ToolPose &to) const
{
    // Below this Cartesian length a segment is accepted whatever its angle change: the only
    // way to get there is leaving the min_radius hold zone, where the bed turns with the tip
    // (almost) on the axis.
    constexpr double min_split_length = 1e-4;

    const double length = (to.tip - from.tip).norm();
    auto lerp = [&from, &to](double t) {
        ToolPose p;
        p.tip  = from.tip + t * (to.tip - from.tip);
        p.tilt = from.tilt + t * (to.tilt - from.tilt);
        return p;
    };

    std::vector<Waypoint> out;
    // Depth-first bisection, left half first, so the output stays ordered.
    struct Span { double t0, t1; };
    std::vector<Span> stack { { 0., 1. } };
    MachinePose prev = from_machine;
    while (! stack.empty()) {
        const Span  s    = stack.back();
        const MachinePose end = to_machine(lerp(s.t1), &prev);
        const double seg_len  = length * (s.t1 - s.t0);
        const bool   too_long = seg_len > m_config.max_segment_length;
        const bool   too_much_turn = std::abs(end.angle - prev.angle) > m_config.max_angle_step ||
                                     std::abs(end.tilt - prev.tilt) > m_config.max_angle_step;
        if ((too_long || too_much_turn) && seg_len > min_split_length) {
            stack.back() = { 0.5 * (s.t0 + s.t1), s.t1 };
            stack.push_back({ s.t0, 0.5 * (s.t0 + s.t1) });
            continue;
        }
        stack.pop_back();
        out.push_back({ s.t1, end });
        prev = end;
    }
    return out;
}

// ------------------------------------------------------------------------------------------
// G-code conversion

void PolarGCodeConverter::set_feed_mode(bool inverse_time, std::ostream &out)
{
    const int mode = inverse_time ? 1 : 0;
    if (m_feed_mode == mode)
        return;
    out << (inverse_time ? "G93 ; inverse time feed\n" : "G94 ; units per minute feed\n");
    m_feed_mode = mode;
}

void PolarGCodeConverter::emit_move(const ToolPose &to, double e, double feed, bool rapid, std::ostream &out, const std::string &comment)
{
    const PolarKinematicsConfig &c   = m_kinematics.config();
    const char                  *cmd = rapid ? "G0" : "G1";
    ++m_stats.cartesian_moves;

    auto write_pose = [&](const MachinePose &m, double seg_e, double seg_feed, bool with_comment) {
        m_stats.min_radius = std::min(m_stats.min_radius, m.radius);
        m_stats.max_radius = std::max(m_stats.max_radius, m.radius);
        if (m.radius < c.min_travel_radius || m.radius > c.max_travel_radius)
            ++m_stats.radius_outside;
        out << cmd << ' ' << c.angle_axis << GCodeWords::number(m.angle, 4) << ' ' << c.radius_axis << GCodeWords::number(m.radius, 4) << " Z"
            << GCodeWords::number(m.z, 4);
        if (c.has_tilt_axis)
            out << ' ' << c.tilt_axis << GCodeWords::number(m.tilt, 3);
        if (seg_e != 0.) {
            if (m_relative_e) {
                // Carry the rounding error forward: splitting one move into hundreds of short
                // segments would otherwise lose (or add) material at every rounded digit.
                const double wanted = seg_e + m_e_rounding;
                const double shown  = std::round(wanted * 1e5) * 1e-5;
                m_e_rounding        = wanted - shown;
                out << " E" << GCodeWords::number(shown, 5);
            } else
                out << " E" << GCodeWords::number(m_e, 5);
        }
        if (seg_feed > 0.)
            out << " F" << GCodeWords::number(seg_feed, 4);
        if (with_comment && ! comment.empty())
            out << ' ' << comment;
        out << '\n';
        ++m_stats.machine_moves;
        if (std::abs(m.radius * c.radius_scale + c.radius_offset) < c.min_radius)
            ++m_stats.held_near_center;
    };

    if (! m_have_machine) {
        // First move after the start block: the machine position is unknown, so go straight
        // to the target as a single positioning move.
        const MachinePose m = m_kinematics.to_machine(to, nullptr);
        m_stats.tilt_limited += m.tilt_limited;
        set_feed_mode(false, out);
        m_e += e;
        write_pose(m, e, feed, true);
        m_pose         = to;
        m_machine      = m;
        m_have_machine = true;
        return;
    }

    const double length = (to.tip - m_pose.tip).norm();
    const std::vector<PolarKinematics::Waypoint> waypoints = m_kinematics.interpolate(m_pose, m_machine, to);
    double t_prev    = 0.;
    double pending_e = 0.; // extrusion of segments too short to emit, carried forward
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const PolarKinematics::Waypoint &w     = waypoints[i];
        const double                     dt    = w.t - t_prev;
        const double                     seg_e = e * dt + pending_e;
        // Duration from the Cartesian feed, stretched when an axis would exceed its limit.
        double minutes = feed > 0. ? length * dt / feed : 0.;
        minutes = std::max(minutes, std::abs(w.pose.angle - m_machine.angle) / c.max_angular_speed);
        minutes = std::max(minutes, std::abs(w.pose.tilt - m_machine.tilt) / c.max_tilt_speed);
        t_prev  = w.t;
        if (minutes <= 0.) {
            pending_e = seg_e;
            continue;
        }
        pending_e = 0.;
        m_stats.tilt_limited += w.pose.tilt_limited;
        set_feed_mode(c.inverse_time_feed, out);
        m_e += seg_e;
        m_stats.total_angle += std::abs(w.pose.angle - m_machine.angle);
        write_pose(w.pose, seg_e, c.inverse_time_feed ? 1. / minutes : feed, i + 1 == waypoints.size());
        m_machine = w.pose;
    }
    if (pending_e != 0.) {
        // A zero-length move that still extrudes: emit the material as an E-only move.
        set_feed_mode(false, out);
        m_e += pending_e;
        out << "G1 E" << GCodeWords::number(m_relative_e ? pending_e : m_e, 5) << " F" << GCodeWords::number(feed, 4);
        if (! comment.empty())
            out << ' ' << comment;
        out << '\n';
    }
    m_pose = to;
}

void PolarGCodeConverter::process_line(const std::string &raw, std::ostream &out)
{
    if (raw.find("MACHINE_START_GCODE_END") != std::string::npos) {
        m_block = Block::Body;
        out << raw << '\n';
        return;
    }
    if (raw.find("MACHINE_END_GCODE_START") != std::string::npos) {
        // The end block is written for the machine and its feedrates are units per minute.
        m_block = Block::End;
        out << raw << '\n';
        set_feed_mode(false, out);
        return;
    }

    const GCodeWords::Line line = GCodeWords::split(raw);
    const std::string      cmd  = GCodeWords::command(line.code);

    // Modal state applies to the machine blocks too.
    if (cmd == "M82")
        m_relative_e = false;
    else if (cmd == "M83")
        m_relative_e = true;
    else if (cmd == "G90")
        m_absolute_xyz = true;
    else if (cmd == "G91")
        m_absolute_xyz = false;
    else if (cmd == "G92") {
        double v;
        if (GCodeWords::find(line.code, 'E', v))
            m_e = v;
    }

    if (m_block != Block::Body) {
        if (cmd == "G93" || cmd == "G94")
            m_feed_mode = cmd == "G93" ? 1 : 0;
        out << raw << '\n';
        return;
    }

    const PolarKinematicsConfig &c = m_kinematics.config();
    const bool motion = cmd == "G0" || cmd == "G1";
    const bool arc    = cmd == "G2" || cmd == "G3";
    if (! motion && ! arc) {
        if (cmd == "G28")
            m_have_machine = false;
        out << raw << '\n';
        return;
    }

    double v;
    if (GCodeWords::find(line.code, 'F', v))
        m_feed = v;

    ToolPose target = m_pose;
    bool     moves  = false;
    for (int axis = 0; axis < 3; ++axis) {
        if (GCodeWords::find(line.code, "XYZ"[axis], v)) {
            target.tip[axis] = m_absolute_xyz ? v : target.tip[axis] + v;
            moves            = true;
        }
    }
    if (c.has_tilt_axis && GCodeWords::find(line.code, c.tilt_axis, v)) {
        target.tilt = deg2rad(v);
        moves       = true;
    }
    double e = 0.;
    if (GCodeWords::find(line.code, 'E', v))
        e = m_relative_e ? v : v - m_e;

    if (! moves) {
        if (e != 0.) {
            // Retraction or prime: no axis moves, so a plain feedrate is correct.
            set_feed_mode(false, out);
            m_e += e;
        } else if (m_feed_mode == 1) {
            // A bare feedrate change means nothing in inverse-time mode; it is applied to the
            // following moves instead.
            if (! line.comment.empty())
                out << line.comment << '\n';
            return;
        }
        out << raw << '\n';
        return;
    }

    if (! arc) {
        emit_move(target, e, m_feed, cmd == "G0", out, line.comment);
        return;
    }

    // Linearize the arc in the XY plane; Z and tilt change linearly along it.
    double i_off = 0., j_off = 0.;
    GCodeWords::find(line.code, 'I', i_off);
    GCodeWords::find(line.code, 'J', j_off);
    const Eigen::Vector2d start  = m_pose.tip.head<2>();
    const Eigen::Vector2d centre = start + Eigen::Vector2d(i_off, j_off);
    const double          radius = (start - centre).norm();
    const double          a0     = std::atan2(start.y() - centre.y(), start.x() - centre.x());
    double                a1     = std::atan2(target.tip.y() - centre.y(), target.tip.x() - centre.x());
    double                sweep  = a1 - a0;
    if (cmd == "G2") { // clockwise
        while (sweep >= 0.)
            sweep -= 2. * PI;
    } else {
        while (sweep <= 0.)
            sweep += 2. * PI;
    }
    const int n = std::max(1, (int) std::ceil(std::abs(sweep) * radius / std::max(0.1, c.max_segment_length * 0.25)));
    ++m_stats.arcs_linearized;
    const ToolPose from = m_pose;
    for (int k = 1; k <= n; ++k) {
        const double t = double(k) / n;
        ToolPose     p;
        p.tip.head<2>() = k == n ? Eigen::Vector2d(target.tip.head<2>()) :
                                   Eigen::Vector2d(centre + radius * Eigen::Vector2d(std::cos(a0 + sweep * t), std::sin(a0 + sweep * t)));
        p.tip.z() = from.tip.z() + t * (target.tip.z() - from.tip.z());
        p.tilt    = from.tilt + t * (target.tilt - from.tilt);
        emit_move(p, e / n, m_feed, false, out, k == n ? line.comment : std::string());
    }
}

void PolarGCodeConverter::process(std::istream &in, std::ostream &out)
{
    // Read everything first: whether the machine start block exists is only known once the
    // start tag has (or has not) been seen.
    std::vector<std::string> lines;
    std::string              line;
    bool                     has_start_tag = false;
    while (std::getline(in, line)) {
        if (! line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.find("MACHINE_START_GCODE_END") != std::string::npos)
            has_start_tag = true;
        lines.emplace_back(std::move(line));
    }
    m_block = has_start_tag ? Block::Start : Block::Body;

    const PolarKinematicsConfig &c = m_kinematics.config();
    out << "; polar kinematics: angle " << c.angle_axis << ", radius " << c.radius_axis;
    if (c.has_tilt_axis)
        out << ", tilt " << c.tilt_axis;
    out << ", " << (c.inverse_time_feed ? "inverse time feed (G93)" : "feedrate (G94)") << '\n';
    // Each line goes through a buffer, to count what it became.
    uint32_t           out_lines = 1;
    std::ostringstream buffer;
    m_lines.clear();
    m_lines.reserve(lines.size());
    for (const std::string &l : lines) {
        buffer.str(std::string());
        process_line(l, buffer);
        const std::string text = buffer.str();
        out_lines += uint32_t(std::count(text.begin(), text.end(), '\n'));
        out << text;
        LineRecord &r = m_lines.emplace_back();
        r.out_lines   = out_lines;
        r.posed       = m_have_machine;
        if (m_have_machine) {
            r.angle  = float(m_machine.angle);
            r.radius = float(m_machine.radius);
            r.z      = float(m_machine.z);
            r.tilt   = float(m_machine.tilt);
        }
    }
    if (m_feed_mode == 1)
        out << "G94 ; units per minute feed\n";
}

PolarGCodeConverter::Stats convert_gcode_to_polar(const std::string &src, const std::string &dst, const PolarKinematicsConfig &config)
{
    std::ifstream in(src);
    if (! in)
        throw std::runtime_error("Cannot open G-code for polar conversion: " + src);
    std::ofstream out(dst);
    if (! out)
        throw std::runtime_error("Cannot write polar G-code: " + dst);
    PolarGCodeConverter converter(config);
    converter.process(in, out);
    out.close();
    if (! out)
        throw std::runtime_error("Failed writing polar G-code: " + dst);
    return converter.stats();
}

} // namespace NonPlanar
} // namespace Slic3r
