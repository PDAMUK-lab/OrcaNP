#include "SkewCompensation.hpp"

#include "../NonPlanar/GCodeWords.hpp"

#include <cmath>
#include <istream>
#include <ostream>
#include <sstream>

namespace Slic3r {

namespace Words = NonPlanar::GCodeWords;

SkewCorrection SkewCorrection::from_degrees(double xy, double xz, double yz)
{
    auto factor = [](double angle) { return std::tan(angle * M_PI / 180.); };
    return { factor(xy), factor(xz), factor(yz) };
}

void SkewGCodeConverter::write_move(const char *cmd, const Vec3d &to, bool has_z, const std::string &other_words,
                                    const std::string &comment, std::ostream &out)
{
    const Vec3d machine = m_skew.apply(to);
    out << cmd << " X" << Words::number(machine.x(), 3) << " Y" << Words::number(machine.y(), 3);
    if (has_z)
        out << " Z" << Words::number(machine.z(), 3);
    out << other_words;
    if (! comment.empty())
        out << ' ' << comment;
    out << '\n';
    ++ m_stats.moves;
}

void SkewGCodeConverter::process_line(const std::string &raw, std::ostream &out)
{
    if (raw.find("MACHINE_START_GCODE_END") != std::string::npos) {
        m_block = Block::Body;
        out << raw << '\n';
        return;
    }
    if (raw.find("MACHINE_END_GCODE_START") != std::string::npos) {
        m_block = Block::End;
        out << raw << '\n';
        return;
    }

    const Words::Line line = Words::split(raw);
    const std::string cmd  = Words::command(line.code);
    if (cmd == "M82")
        m_relative_e = false;
    else if (cmd == "M83")
        m_relative_e = true;
    else if (cmd == "G90")
        m_absolute_xyz = true;
    else if (cmd == "G91")
        m_absolute_xyz = false;
    else if (cmd == "G28")
        m_known[0] = m_known[1] = m_known[2] = false;

    const bool motion = cmd == "G0" || cmd == "G1";
    const bool arc    = cmd == "G2" || cmd == "G3";
    double     v;
    // Where the move ends, as the G-code means it, and what else it says.
    Vec3d target = m_pos, delta = Vec3d::Zero();
    bool  has[3] = { false, false, false };
    if (motion || arc || cmd == "G92")
        for (int axis = 0; axis < 3; ++ axis)
            if (Words::find(line.code, "XYZ"[axis], v)) {
                has[axis]    = true;
                delta[axis]  = cmd != "G92" && ! m_absolute_xyz ? v : v - m_pos[axis];
                target[axis] = m_pos[axis] + delta[axis];
            }
    double e = 0.;
    const bool has_e = Words::find(line.code, 'E', v);
    if (has_e)
        e = cmd == "G92" ? v : m_relative_e ? v : v - m_e;

    // Whether the whole position is known once the move is made.
    const bool known = (m_known[0] || has[0]) && (m_known[1] || has[1]) && (m_known[2] || has[2]);
    auto       advance = [&]() {
        m_pos = target;
        for (int axis = 0; axis < 3; ++ axis)
            m_known[axis] = m_known[axis] || has[axis];
        if (cmd == "G92") {
            if (has_e)
                m_e = e;
        } else if (has_e)
            m_e += e;
    };

    // Only the print's moves are sheared, once the whole position is known; a G92 of an axis
    // is passed on as it is (the firmware then counts from the sheared position). Relative moves
    // need no position, being sheared as they are; relative arcs are left as they are.
    if (m_block != Block::Body || ! (motion || arc) || ! (has[0] || has[1] || has[2]) || (m_absolute_xyz && ! known) ||
        (arc && ! m_absolute_xyz)) {
        advance();
        out << raw << '\n';
        return;
    }

    // The words other than X, Y, Z (and I, J of an arc), in their order.
    std::string other;
    {
        std::istringstream words(line.code);
        std::string        word;
        words >> word; // the command
        while (words >> word) {
            const char letter = char(std::toupper((unsigned char) word[0]));
            if (letter == 'X' || letter == 'Y' || letter == 'Z' || (arc && (letter == 'I' || letter == 'J' || letter == 'E')))
                continue;
            other += ' ' + word;
        }
    }

    if (motion && ! m_absolute_xyz) {
        const Vec3d d = m_skew.apply(delta);
        out << (cmd == "G0" ? "G0" : "G1") << " X" << Words::number(d.x(), 3) << " Y" << Words::number(d.y(), 3);
        if (has[2])
            out << " Z" << Words::number(d.z(), 3);
        out << other;
        if (! line.comment.empty())
            out << ' ' << line.comment;
        out << '\n';
        ++ m_stats.moves;
        advance();
        return;
    }
    if (motion) {
        if (has_e && e > 0. && m_bed.defined) {
            const Vec3d machine = m_skew.apply(target);
            if (machine.x() < m_bed.min.x() - 0.01 || machine.x() > m_bed.max.x() + 0.01 || machine.y() < m_bed.min.y() - 0.01 ||
                machine.y() > m_bed.max.y() + 0.01)
                ++ m_stats.outside_bed;
        }
        write_move(cmd == "G0" ? "G0" : "G1", target, has[2], other, line.comment, out);
        advance();
        return;
    }

    // An arc in the XY plane, Z and E changing evenly along it: straight segments of at most
    // half a millimetre, the arc's feedrate on the first.
    double i_off = 0., j_off = 0.;
    Words::find(line.code, 'I', i_off);
    Words::find(line.code, 'J', j_off);
    const Vec2d  start  = m_pos.head<2>();
    const Vec2d  centre = start + Vec2d(i_off, j_off);
    const double radius = (start - centre).norm();
    const double a0     = std::atan2(start.y() - centre.y(), start.x() - centre.x());
    double       sweep  = std::atan2(target.y() - centre.y(), target.x() - centre.x()) - a0;
    if (cmd == "G2")
        while (sweep >= 0.)
            sweep -= 2. * M_PI;
    else
        while (sweep <= 0.)
            sweep += 2. * M_PI;
    const int   n    = std::max(1, int(std::ceil(std::abs(sweep) * radius / 0.5)));
    const Vec3d from = m_pos;
    const double e0  = m_e;
    ++ m_stats.arcs_linearized;
    for (int k = 1; k <= n; ++ k) {
        const double t = double(k) / n;
        Vec3d        p;
        p.head<2>() = k == n ? Vec2d(target.head<2>()) : Vec2d(centre + radius * Vec2d(std::cos(a0 + sweep * t), std::sin(a0 + sweep * t)));
        p.z()       = from.z() + t * (target.z() - from.z());
        std::string words;
        if (has_e)
            words += " E" + Words::number(m_relative_e ? e / n : e0 + e * t, 5);
        // The arc's other words (its feedrate) go with the first segment.
        if (k == 1)
            words += other;
        write_move("G1", p, has[2], words, k == n ? line.comment : std::string(), out);
    }
    advance();
}

void SkewGCodeConverter::process(std::istream &in, std::ostream &out)
{
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

    out << "; axis skew compensation: XY " << Words::number(std::atan(m_skew.xy) * 180. / M_PI, 4) << ", XZ "
        << Words::number(std::atan(m_skew.xz) * 180. / M_PI, 4) << ", YZ " << Words::number(std::atan(m_skew.yz) * 180. / M_PI, 4)
        << " degrees\n";
    uint32_t           out_lines = 1;
    std::ostringstream buffer;
    m_out_lines.clear();
    m_out_lines.reserve(lines.size());
    for (const std::string &l : lines) {
        buffer.str(std::string());
        process_line(l, buffer);
        const std::string text = buffer.str();
        out_lines += uint32_t(std::count(text.begin(), text.end(), '\n'));
        out << text;
        m_out_lines.emplace_back(out_lines);
    }
}

} // namespace Slic3r
