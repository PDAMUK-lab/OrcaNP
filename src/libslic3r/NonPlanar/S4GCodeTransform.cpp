#include "S4GCodeTransform.hpp"
#include "GCodeWords.hpp"

#include "../AABBMesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <istream>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <unordered_map>

namespace Slic3r {
namespace NonPlanar {

namespace {

constexpr double PI = 3.14159265358979323846;

struct Move
{
    int                   id = 0;
    bool                  rapid = false;
    Eigen::Vector3d       pos;    // real-space position, G-code frame
    Eigen::Vector3d       source; // sliced-space position, G-code frame
    std::optional<double> e;
    std::optional<double> feed;
    double                tilt = 0.; // radians
    double                flow = 1.;
    S4Mapper::Tier        tier = S4Mapper::Tier::Outside;
    bool                  printing  = false;
    bool                  synthetic = false;
    bool                  pure_e    = false; // retract / prime: E and F only
    bool                  dropped   = false;
    std::string           axes;              // axis words the source line commanded
    int                   object    = -1;    // object marker the line is in, -1 outside
    bool                  support   = false; // after a ";TYPE:Support..." marker
    const S4Mapper       *mapped_by = nullptr; // mapper that placed it, null when printed as sliced
};

struct Record
{
    std::string           raw;
    bool                  motion   = false;
    bool                  verbatim = false; // copied untouched (machine end block)
    std::vector<Move>     moves;
    std::optional<double> source_e;
};

bool is_layer_marker(const std::string &line)
{
    size_t i = 0;
    while (i < line.size() && std::isspace((unsigned char) line[i]))
        ++i;
    return line.compare(i, 13, ";LAYER_CHANGE") == 0 || line.compare(i, 7, ";LAYER:") == 0;
}

bool extrudes_xy(const std::string &raw)
{
    const GCodeWords::Line l   = GCodeWords::split(raw);
    const std::string      cmd = GCodeWords::command(l.code);
    double                 e = 0., v;
    return (cmd == "G1" || cmd == "G2" || cmd == "G3") && GCodeWords::find(l.code, 'E', e) && e > 0. &&
           (GCodeWords::find(l.code, 'X', v) || GCodeWords::find(l.code, 'Y', v));
}

class Transform
{
public:
    Transform(const S4MapperSet &mappers, const S4GCodeConfig &cfg) : m_mappers(mappers), m_cfg(cfg)
    {
        m_axis = Eigen::Vector2d::Zero();
        if (mappers.fallback.mapper)
            m_axis = mappers.fallback.mapper->axis();
        else
            for (const auto &[id, m] : mappers.objects)
                if (m.mapper) {
                    m_axis = m.mapper->axis();
                    break;
                }
    }

    S4GCodeReport run(std::istream &in, std::ostream &out);

private:
    void split_blocks(const std::vector<std::string> &lines);
    void build_records();
    void map_moves();
    // Where a sliced point (G-code coordinates) prints through an object's mapper, flat near the
    // bed and easing into the mapped layers above (see S4ObjectMapping). The tilt is unshaped.
    S4Mapper::Result map_point(const S4ObjectMapping &om, const Eigen::Vector3d &source);
    // Where a sliced support point prints: in its column between the part (or the bed) below and
    // the part above (see S4ObjectMapping::support_surface). Empty with no part above it nearby.
    // `loose` is set for a point that holds nothing up: beside the part and above the top of the
    // column it follows, or level with the part and nothing above it within reach.
    std::optional<S4Mapper::Result> map_support(const S4ObjectMapping &om, const Eigen::Vector3d &source, bool &loose);
    // map_support() for support where it applies, map_point() for the rest.
    S4Mapper::Result place(const S4ObjectMapping &om, bool support, const Eigen::Vector3d &source, bool &loose);
    void pass_junctions();
    const S4ObjectMapping &mapping(int object) const
    {
        auto it = m_mappers.objects.find(object);
        return it == m_mappers.objects.end() ? m_mappers.fallback : it->second;
    }
    void pass_z_floor();
    void pass_extrusion();
    void pass_drop_degenerate();
    void pass_limit_z_rate();
    void pass_travel_safety();
    void pass_head_clearance();
    double shape_tilt(double tilt);
    std::vector<std::string> emit();
    void validate(const std::vector<std::string> &out_lines);

    Move make_move(const Eigen::Vector3d &p)
    {
        Move m;
        m.id     = m_next_id++;
        m.pos    = p;
        m.source = p;
        return m;
    }

    const S4MapperSet   &m_mappers;
    Eigen::Vector2d      m_axis;
    const S4GCodeConfig &m_cfg;
    S4GCodeReport        m_report;

    std::vector<std::string> m_start, m_body;
    std::vector<Record>      m_records;
    bool                     m_relative_e     = false;
    bool                     m_start_retracted = false;
    double                   m_retract_length = 0., m_retract_feed = 0.;
    int                      m_next_id        = 0;
    // Slicer output revisits coordinates constantly; each distinct point is mapped once.
    std::unordered_map<std::string, S4Mapper::Result> m_map_cache;
};

void Transform::split_blocks(const std::vector<std::string> &lines)
{
    // The machine start G-code is copied verbatim. Orca tags its end; other slicers are cut at
    // the first layer marker, or failing that at the first extruding move.
    size_t cut = lines.size();
    for (size_t i = 0; i < lines.size(); ++i)
        if (lines[i].find("MACHINE_START_GCODE_END") != std::string::npos) {
            cut = i + 1;
            break;
        }
    if (cut == lines.size())
        for (size_t i = 0; i < lines.size() && cut == lines.size(); ++i)
            if (is_layer_marker(lines[i]))
                cut = i;
    if (cut == lines.size())
        for (size_t i = 0; i < lines.size() && cut == lines.size(); ++i)
            if (extrudes_xy(lines[i]))
                cut = i;
    if (cut == lines.size())
        throw std::runtime_error("S4: no toolpath found in the G-code");
    m_start.assign(lines.begin(), lines.begin() + cut);
    m_body.assign(lines.begin() + cut, lines.end());

    // Modal state the start block leaves behind.
    for (const std::string &raw : m_start) {
        const GCodeWords::Line l   = GCodeWords::split(raw);
        const std::string      cmd = GCodeWords::command(l.code);
        double                 v;
        if (cmd == "M83")
            m_relative_e = true;
        else if (cmd == "M82")
            m_relative_e = false;
        else if ((cmd == "G0" || cmd == "G1") && GCodeWords::find(l.code, 'E', v) && ! GCodeWords::find(l.code, 'X', v) &&
                 ! GCodeWords::find(l.code, 'Y', v) && ! GCodeWords::find(l.code, 'Z', v)) {
            GCodeWords::find(l.code, 'E', v);
            m_start_retracted = v < 0.;
        }
    }

    // The retraction the slicer uses, for the ones added to travels.
    std::map<std::pair<double, double>, int> seen;
    for (const std::string &raw : lines) {
        const GCodeWords::Line l = GCodeWords::split(raw);
        double                 e, v, f = 0.;
        if (GCodeWords::command(l.code) == "G1" && GCodeWords::find(l.code, 'E', e) && e < 0. && ! GCodeWords::find(l.code, 'X', v) &&
            ! GCodeWords::find(l.code, 'Y', v) && ! GCodeWords::find(l.code, 'Z', v)) {
            GCodeWords::find(l.code, 'F', f);
            ++seen[{ std::round(-e * 1e5) * 1e-5, f }];
        }
    }
    m_retract_length = m_cfg.retract_length;
    m_retract_feed   = m_cfg.retract_feed;
    if (! seen.empty() && (m_retract_length <= 0. || m_retract_feed <= 0.)) {
        const auto best = std::max_element(seen.begin(), seen.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
        if (m_retract_length <= 0.)
            m_retract_length = best->first.first;
        if (m_retract_feed <= 0.)
            m_retract_feed = best->first.second;
    }
    if (m_retract_length <= 0.)
        m_retract_length = 0.8;
    if (m_retract_feed <= 0.)
        m_retract_feed = 2100.;
}

void Transform::build_records()
{
    Eigen::Vector3d pos      = Eigen::Vector3d::Zero();
    bool            have_pos = false; // unknown until the first move after the start block
    double          feed     = 1500.;
    bool            absolute = true;

    // The machine end G-code is copied verbatim: from Orca's tag, or else everything after the
    // last extruding move (park, cool-down: machine coordinates, not part coordinates).
    size_t end_block = m_body.size();
    for (size_t i = 0; i < m_body.size(); ++i)
        if (m_body[i].find("MACHINE_END_GCODE_START") != std::string::npos) {
            end_block = i;
            break;
        }
    if (end_block == m_body.size())
        for (size_t i = m_body.size(); i > 0; --i)
            if (extrudes_xy(m_body[i - 1])) {
                end_block = i;
                break;
            }

    int  object  = -1;
    bool support = false;
    for (size_t i = 0; i < m_body.size(); ++i) {
        const std::string     &raw = m_body[i];
        const GCodeWords::Line l   = GCodeWords::split(raw);
        const std::string      cmd = GCodeWords::command(l.code);
        Record                 rec;
        rec.raw = raw;
        if (const size_t at = raw.find("NONPLANAR_OBJECT_END"); at != std::string::npos)
            object = -1;
        else if (const size_t at = raw.find("NONPLANAR_OBJECT "); at != std::string::npos)
            object = std::atoi(raw.c_str() + at + 17);
        if (raw.rfind(";TYPE:", 0) == 0)
            support = raw.compare(6, 7, "Support") == 0;
        const S4ObjectMapping &om = mapping(object);
        if (cmd == "G90")
            absolute = true;
        else if (cmd == "G91")
            absolute = false;
        else if (cmd == "M83")
            m_relative_e = true; // Orca's preamble follows the machine start block
        else if (cmd == "M82" && i < end_block)
            throw std::runtime_error("S4: the G-code switches to absolute extrusion (M82); relative extrusion is required");

        const bool line_motion = cmd == "G0" || cmd == "G1" || cmd == "G2" || cmd == "G3";
        if (i >= end_block || ! line_motion) {
            rec.verbatim = i >= end_block;
            m_records.push_back(std::move(rec));
            continue;
        }
        rec.motion = true;

        double v;
        if (GCodeWords::find(l.code, 'F', v))
            feed = v;
        std::optional<double> e;
        if (GCodeWords::find(l.code, 'E', v))
            e = v;
        rec.source_e = e;

        Eigen::Vector3d target = pos;
        std::string     axes;
        for (int k = 0; k < 3; ++k)
            if (GCodeWords::find(l.code, "XYZ"[k], v)) {
                target[k] = absolute ? v : target[k] + v;
                axes += "XYZ"[k];
            }

        if (axes.empty()) {
            Move m   = make_move(pos);
            m.e      = e;
            m.feed   = feed;
            m.pure_e  = true;
            m.object  = object;
            m.support = support;
            rec.moves.push_back(m);
            m_records.push_back(std::move(rec));
            continue;
        }
        if (e && ! m_relative_e)
            throw std::runtime_error("S4: absolute extrusion (M82) is not supported; enable relative extrusion (M83)");

        const bool printing = e && *e > 0.;
        const double step   = printing ? m_cfg.seg_size : m_cfg.travel_seg_size;

        // Polyline of the move in sliced space: the straight line, or the arc linearized.
        std::vector<Eigen::Vector3d> path;
        if (cmd == "G2" || cmd == "G3") {
            double ii = 0., jj = 0.;
            GCodeWords::find(l.code, 'I', ii);
            GCodeWords::find(l.code, 'J', jj);
            const Eigen::Vector2d centre = pos.head<2>() + Eigen::Vector2d(ii, jj);
            const double          radius = (pos.head<2>() - centre).norm();
            const double          a0     = std::atan2(pos.y() - centre.y(), pos.x() - centre.x());
            double                sweep  = std::atan2(target.y() - centre.y(), target.x() - centre.x()) - a0;
            if (cmd == "G2")
                while (sweep >= 0.)
                    sweep -= 2. * PI;
            else
                while (sweep <= 0.)
                    sweep += 2. * PI;
            const int n = std::max(1, int(std::ceil(std::abs(sweep) * radius / step)));
            for (int k = 1; k <= n; ++k) {
                const double t = double(k) / n;
                path.emplace_back(k == n ? target :
                                           Eigen::Vector3d(centre.x() + radius * std::cos(a0 + sweep * t),
                                                           centre.y() + radius * std::sin(a0 + sweep * t), pos.z() + t * (target.z() - pos.z())));
            }
        } else {
            const double dist = (target - pos).norm();
            // A travel leaving or entering the mesh (bed-level repositioning, the run in from the
            // purge line) is not deformed by anything; subdividing it only inflates the file.
            const bool outside = ! printing && (! om.mapper || ! om.mapper->in_bbox(pos - m_cfg.offset) ||
                                                ! om.mapper->in_bbox(target - m_cfg.offset));
            const int  n       = ! have_pos || outside || dist <= step ? 1 : std::min(m_cfg.max_segments_per_move, int(std::ceil(dist / step)));
            for (int k = 1; k <= n; ++k)
                path.push_back(pos + (target - pos) * (double(k) / n));
        }

        // Extrusion outside the object's kept window is repeated inside it: split the path where
        // it crosses the window's edges and print only the inside.
        const bool windowed = printing && (std::isfinite(om.keep_min_x) || std::isfinite(om.keep_max_x));
        if (windowed) {
            std::vector<Eigen::Vector3d> split;
            Eigen::Vector3d              a = pos;
            for (const Eigen::Vector3d &b : path) {
                std::vector<double> ts;
                for (double edge : { om.keep_min_x, om.keep_max_x })
                    if (std::isfinite(edge) && (a.x() - edge) * (b.x() - edge) < 0.)
                        ts.push_back((edge - a.x()) / (b.x() - a.x()));
                std::sort(ts.begin(), ts.end());
                for (double t : ts)
                    split.push_back(a + (b - a) * t);
                split.push_back(b);
                a = b;
            }
            path.swap(split);
        }

        std::vector<double> lengths;
        double              total = 0.;
        Eigen::Vector3d     prev  = pos;
        for (const Eigen::Vector3d &p : path) {
            lengths.push_back((p - prev).norm());
            total += lengths.back();
            prev = p;
        }
        prev = pos;
        for (size_t k = 0; k < path.size(); ++k) {
            Move m     = make_move(path[k]);
            m.rapid    = cmd == "G0";
            m.feed     = feed;
            m.printing = printing;
            m.axes     = axes;
            m.object   = object;
            m.support  = support;
            if (e)
                m.e = *e * (total > 0. ? lengths[k] / total : 1. / path.size());
            if (windowed) {
                const double mid = 0.5 * (prev.x() + path[k].x());
                if (mid < om.keep_min_x || mid >= om.keep_max_x) {
                    m.e.reset();
                    m.printing = false;
                    ++m_report.trimmed;
                }
            }
            prev = path[k];
            rec.moves.push_back(m);
        }
        if (windowed && e) {
            // The line's material is what its kept part needs.
            double kept = 0.;
            for (const Move &m : rec.moves)
                if (m.e)
                    kept += *m.e;
            rec.source_e = kept;
        }
        pos      = target;
        have_pos = true;
        m_records.push_back(std::move(rec));
    }
}

void Transform::map_moves()
{
    const Move *last = nullptr;
    for (Record &r : m_records) {
        bool dropped = false;
        for (Move &m : r.moves) {
            if (m.pure_e) {
                if (last) {
                    m.pos  = last->pos;
                    m.tilt = last->tilt;
                }
                continue;
            }
            const S4ObjectMapping &om = mapping(m.object);
            if (! om.mapper || m.source.z() <= om.identity_below_z) {
                // Printed as sliced: no mapper, or the object's flat print surface.
                m.pos       = m.source;
                m.tilt      = 0.;
                m.flow      = 1.;
                m.tier      = S4Mapper::Tier::Outside;
                m.mapped_by = nullptr;
                ++m_report.outside;
                last = &m;
                continue;
            }
            bool                   loose;
            const S4Mapper::Result res = place(om, m.support, m.source, loose);
            if (loose && m.printing) {
                m.e.reset();
                m.printing = false;
                dropped    = true;
                ++m_report.support_dropped;
            }
            m.mapped_by                = om.mapper;
            m.pos                      = res.point;
            m.tilt                     = shape_tilt(res.tilt);
            m.flow                     = res.flow;
            m.tier                     = res.tier;
            switch (res.tier) {
            case S4Mapper::Tier::Inside: ++m_report.inside; break;
            case S4Mapper::Tier::Nearest: ++m_report.nearest; break;
            case S4Mapper::Tier::Idw: ++m_report.idw; break;
            case S4Mapper::Tier::Outside: ++m_report.outside; break;
            }
            last = &m;
        }
        if (dropped && r.source_e) {
            // The line's material is what its printed part needs.
            double kept = 0.;
            for (const Move &m : r.moves)
                if (! m.pure_e && m.e)
                    kept += *m.e;
            r.source_e = kept;
        }
    }
}

S4Mapper::Result Transform::map_point(const S4ObjectMapping &om, const Eigen::Vector3d &source)
{
    auto mapped = [&](const Eigen::Vector3d &q) {
        char key[96];
        std::snprintf(key, sizeof(key), "%p %.6f %.6f %.6f", (const void *) om.mapper, q.x(), q.y(), q.z());
        auto it = m_map_cache.find(key);
        if (it == m_map_cache.end())
            it = m_map_cache.emplace(key, om.mapper->map(q - m_cfg.offset)).first;
        S4Mapper::Result res = it->second;
        res.point += m_cfg.offset;
        return res;
    };
    S4Mapper::Result res = mapped(source);
    if (const double z = source.z(), base = om.flat_top_z; z < om.blend_top_z) {
        if (z <= base + 1e-3) {
            res.point.z() = z;
            res.flow      = 1.;
            res.tilt      = 0.;
        } else {
            const double span = om.blend_top_z - base;
            const double top  = mapped(Eigen::Vector3d(source.x(), source.y(), om.blend_top_z)).point.z();
            // Lifted so far that the mapper puts the blend's top below the flat layers, a point
            // still keeps rising, so the layers never fold over each other.
            const double rise = std::max(top - base, 0.05 * span);
            const double frac = (z - base) / span;
            res.point.z()     = base + rise * frac;
            res.flow          = rise / span;
            res.tilt *= frac;
        }
    }
    return res;
}

std::optional<S4Mapper::Result> Transform::map_support(const S4ObjectMapping &om, const Eigen::Vector3d &source, bool &loose)
{
    // In the mapper's frame. The part above is looked for straight up. Support also reaches past
    // the overhang it holds up; a column there follows the nearest point of the part above, found
    // on rings up to 6 mm out and then narrowed down to the edge, so that neighbouring columns
    // follow neighbouring points and their layers stay in step. Support beside the part and
    // higher than the top of such a column, or level with the part and nothing above it, holds
    // nothing up: it is `loose`.
    const S4SupportSurface &surface = *om.support_surface;
    const Eigen::Vector3d   p       = source - m_cfg.offset;
    bool                    beside  = false; // the part is level with the point within reach
    auto                    up_from = [&](const Eigen::Vector2d &q) { return surface.next(Eigen::Vector3d(q.x(), q.y(), p.z()), true, &beside); };
    Eigen::Vector2d         at      = p.head<2>();
    std::optional<double>   above   = up_from(at);
    double                  inner   = 0.; // the last ring without the part above
    for (double r : { 0.25, 0.5, 1., 1.5, 2., 3., 4., 6. }) {
        if (above)
            break;
        constexpr int                dirs = 16;
        std::vector<Eigen::Vector2d> hits;
        Eigen::Vector2d              sum = Eigen::Vector2d::Zero();
        for (int k = 0; k < dirs; ++k) {
            const Eigen::Vector2d d(std::cos(2. * PI * k / dirs), std::sin(2. * PI * k / dirs));
            if (up_from(p.head<2>() + r * d)) {
                hits.push_back(d);
                sum += d;
            }
        }
        if (hits.empty()) {
            inner = r;
            continue;
        }
        // The hit direction closest to their mean, and the edge between the rings along it.
        Eigen::Vector2d dir = hits.front();
        for (const Eigen::Vector2d &d : hits)
            if (d.dot(sum) > dir.dot(sum))
                dir = d;
        double lo = inner, hi = r;
        for (int i = 0; i < 5; ++i) {
            const double mid = 0.5 * (lo + hi);
            (up_from(p.head<2>() + mid * dir) ? hi : lo) = mid;
        }
        at    = p.head<2>() + hi * dir;
        above = up_from(at);
    }
    if (! above) {
        loose = beside;
        return std::nullopt;
    }

    // The column's ends: the part's underside above, and the part's surface below or the bed.
    // Each is placed where the part itself prints (the underside and a surface below exactly;
    // the bed under the flat first layer), with the support gap kept between it and the support.
    const Eigen::Vector3d  top_source = Eigen::Vector3d(at.x(), at.y(), *above) + m_cfg.offset;
    const S4Mapper::Result top        = map_point(om, top_source);
    const Eigen::Vector3d  top_shift  = top.point - top_source;
    const double           top_sliced = top_source.z() - om.support_top_gap;
    Eigen::Vector3d        bottom_shift = Eigen::Vector3d::Zero();
    double                 bottom_sliced, bottom_real;
    if (const std::optional<double> below = surface.next(p, false)) {
        const Eigen::Vector3d  bottom_source = Eigen::Vector3d(p.x(), p.y(), *below) + m_cfg.offset;
        const S4Mapper::Result bottom        = map_point(om, bottom_source);
        bottom_shift  = bottom.point - bottom_source;
        bottom_sliced = bottom_source.z() + om.support_bottom_gap;
        bottom_real   = bottom.point.z() + om.support_bottom_gap;
    } else
        // The bed, or the top of what prints as sliced: the flat first layers or a print surface.
        bottom_sliced = bottom_real = std::max({ m_cfg.z_floor, om.flat_top_z, om.identity_below_z });
    // A column the real part leaves less room for than the sliced one still rises, so its layers
    // never fold over each other.
    const double span_sliced = top_sliced - bottom_sliced;
    const double top_real    = std::max(top.point.z() - om.support_top_gap, bottom_real + 0.05 * std::max(span_sliced, 0.));

    S4Mapper::Result res;
    res.tier     = S4Mapper::Tier::Inside;
    const double z = source.z();
    loose = z > top_sliced && at != p.head<2>();
    if (z <= bottom_sliced) {
        res.point = source + bottom_shift;
    } else if (z >= top_sliced || span_sliced <= 1e-9) {
        res.point     = source + top_shift;
        res.point.z() = top_real + (z - top_sliced);
        res.tilt      = top.tilt;
    } else {
        const double t = (z - bottom_sliced) / span_sliced;
        res.point.head<2>() = source.head<2>() + (1. - t) * bottom_shift.head<2>() + t * top_shift.head<2>();
        res.point.z()       = bottom_real + t * (top_real - bottom_real);
        res.flow            = (top_real - bottom_real) / span_sliced;
        res.tilt            = t * top.tilt;
    }
    return res;
}

S4Mapper::Result Transform::place(const S4ObjectMapping &om, bool support, const Eigen::Vector3d &source, bool &loose)
{
    loose = false;
    if (support && om.support_surface) {
        if (std::optional<S4Mapper::Result> res = map_support(om, source, loose))
            return *res;
        if (! loose)
            ++m_report.support_unanchored;
    }
    return map_point(om, source);
}

void Transform::pass_junctions()
{
    // A run of non-printing moves between prints that were mapped differently (another object,
    // an object's flat print surface, lines outside any object) was planned in two unrelated
    // spaces. It becomes a straight line in the part from where the last print ended to where
    // the next one starts; the travel pass then lifts it over whatever is in the way.
    struct Ref { size_t record, move; };
    struct Job { Ref from; std::vector<Ref> run; Ref to; };
    std::vector<Job>   jobs;
    std::vector<Ref>   run;
    std::optional<Ref> last_print;
    auto at = [this](const Ref &r) -> Move & { return m_records[r.record].moves[r.move]; };
    for (size_t ri = 0; ri < m_records.size(); ++ri)
        for (size_t mi = 0; mi < m_records[ri].moves.size(); ++mi) {
            const Move &m = m_records[ri].moves[mi];
            if (m.pure_e)
                continue;
            if (! m.printing) {
                run.push_back({ ri, mi });
                continue;
            }
            if (last_print && ! run.empty()) {
                const S4Mapper *to    = m.mapped_by;
                bool            mixed = at(*last_print).mapped_by != to || at(*last_print).support != m.support;
                for (const Ref &r : run)
                    mixed |= at(r).mapped_by != to || at(r).object != m.object || at(r).support != m.support;
                if (mixed)
                    jobs.push_back({ *last_print, run, { ri, mi } });
            }
            run.clear();
            last_print = Ref { ri, mi };
        }

    // Back to front, so inserting moves does not shift the ones still to come.
    for (auto job = jobs.rbegin(); job != jobs.rend(); ++job) {
        const Move           &from = at(job->from);
        const Move           &to   = at(job->to);
        const Ref             last = job->run.back();
        Move                 &end  = at(last);
        Eigen::Vector3d       target = end.source;
        double                tilt   = 0.;
        if (to.mapped_by) {
            bool                   loose;
            const S4Mapper::Result res = place(mapping(to.object), to.support, end.source, loose);
            target                     = res.point;
            tilt                       = shape_tilt(res.tilt);
        }
        for (const Ref &r : job->run)
            if (r.record != last.record || r.move != last.move)
                at(r).dropped = true;
        const Eigen::Vector3d start = from.pos;
        const double          start_tilt = from.tilt;
        end.pos       = target;
        end.tilt      = tilt;
        end.mapped_by = to.mapped_by;
        const int n = std::clamp(int(std::ceil((target - start).norm() / m_cfg.travel_seg_size)), 1, m_cfg.max_segments_per_move);
        std::vector<Move> path;
        for (int k = 1; k < n; ++k) {
            const double t = double(k) / n;
            Move         m = make_move(start + (target - start) * t);
            m.pos       = m.source;
            m.rapid     = end.rapid;
            m.feed      = end.feed;
            m.tilt      = start_tilt + (tilt - start_tilt) * t;
            m.object    = end.object;
            m.synthetic = true;
            path.push_back(m);
        }
        std::vector<Move> &moves = m_records[last.record].moves;
        moves.insert(moves.begin() + last.move, path.begin(), path.end());
        ++m_report.junctions;
    }
}

// The nozzle stays vertical over gently sloped layers, catches up with the layer between the
// threshold and twice the threshold, and never leans past max_tilt.
double Transform::shape_tilt(double tilt)
{
    const double th  = m_cfg.tilt_threshold;
    double       mag = std::abs(tilt);
    if (th > 0.)
        mag = mag <= th ? 0. : std::min(mag, 2. * (mag - th));
    const double shaped = std::copysign(mag, tilt);
    const double lo = std::max(-m_cfg.max_tilt, m_cfg.min_tilt_toward), hi = std::min(m_cfg.max_tilt, m_cfg.max_tilt_away);
    const double out = std::clamp(shaped, std::min(lo, 0.), std::max(hi, 0.));
    if (out != shaped)
        ++m_report.tilt_limited;
    return out;
}

void Transform::pass_z_floor()
{
    for (Record &r : m_records)
        for (Move &m : r.moves)
            if (! m.pure_e && m.pos.z() < m_cfg.z_floor) {
                m.pos.z() = m_cfg.z_floor;
                ++m_report.floor_clamped;
            }
}

void Transform::pass_extrusion()
{
    // The slicer laid a bead of volume E in the sliced space; mapping it back scales the
    // region's volume by the cell's volume ratio. The printer cannot follow an unbounded flow
    // change, so the ratio is clipped, and what is clipped is carried into the next segments
    // instead of being lost. Within one source line, material follows the mapped length.
    double                          residual = 0.;
    std::optional<Eigen::Vector3d>  cursor;
    for (const Record &r : m_records)
        for (const Move &m : r.moves)
            if (! m.pure_e && m.e && *m.e > 0.)
                m_report.filament_in += *m.e;

    for (Record &r : m_records) {
        if (! r.source_e || *r.source_e <= 0.) {
            for (const Move &m : r.moves)
                if (! m.pure_e)
                    cursor = m.pos;
            continue;
        }
        std::vector<Move *> segs;
        for (Move &m : r.moves)
            if (! m.pure_e && m.e)
                segs.push_back(&m);
        if (segs.empty())
            continue;
        std::vector<double> lengths;
        std::optional<Eigen::Vector3d> prev = cursor;
        for (Move *m : segs) {
            lengths.push_back(prev ? (m->pos - *prev).norm() : 0.);
            prev = m->pos;
        }
        if (lengths.size() > 1 && lengths[0] == 0.) {
            double sum = 0.;
            for (size_t k = 1; k < lengths.size(); ++k)
                sum += lengths[k];
            lengths[0] = sum / double(lengths.size() - 1);
        }
        double total = 0.;
        for (double l : lengths)
            total += l;
        for (size_t k = 0; k < segs.size(); ++k) {
            Move        &m     = *segs[k];
            const double share = total > 1e-12 ? lengths[k] / total : 1. / segs.size();
            const double base  = *r.source_e * share;
            double       j     = m.flow;
            if (m_cfg.flow_model == S4GCodeConfig::FlowModel::Length)
                j = 1.;
            else if (m_cfg.flow_model == S4GCodeConfig::FlowModel::Hybrid)
                j = std::sqrt(std::max(j, 1e-6));
            const double want    = base * j;
            double       clamped = std::clamp(want, base * m_cfg.min_flow, base * m_cfg.max_flow);
            if (want > clamped)
                ++m_report.flow_clipped_high;
            else if (want < clamped)
                ++m_report.flow_clipped_low;
            if (m_cfg.conserve_clipped) {
                const double give = std::max(-clamped * 0.5, std::min({ residual, clamped * 0.5, m_cfg.max_residual }));
                residual += (want - clamped) - give;
                residual = std::clamp(residual, -m_cfg.max_residual * 20., m_cfg.max_residual * 20.);
                clamped += give;
            }
            m.e    = std::max(clamped, 0.);
            cursor = m.pos;
        }
    }
    for (const Record &r : m_records)
        for (const Move &m : r.moves)
            if (! m.pure_e && m.e && *m.e > 0.)
                m_report.filament_out += *m.e;
}

void Transform::pass_drop_degenerate()
{
    // A move that neither goes anywhere nor turns the nozzle is folded into the next real
    // move, extrusion included.
    double                         carry = 0.;
    std::optional<Eigen::Vector3d> prev;
    double                         prev_tilt = 0.;
    for (Record &r : m_records) {
        for (Move &m : r.moves) {
            if (m.pure_e)
                continue;
            if (prev && (m.pos - *prev).norm() <= m_cfg.degenerate_tol && std::abs(m.tilt - prev_tilt) < 1e-4) {
                if (m.e)
                    carry += *m.e;
                m.dropped = true;
                ++m_report.degenerate_dropped;
                continue;
            }
            if (carry != 0. && (m.e || ! m.rapid)) {
                m.e   = m.e.value_or(0.) + carry;
                carry = 0.;
            }
            prev      = m.pos;
            prev_tilt = m.tilt;
        }
        r.moves.erase(std::remove_if(r.moves.begin(), r.moves.end(), [](const Move &m) { return m.dropped; }), r.moves.end());
    }
}

void Transform::pass_limit_z_rate()
{
    // One cell disagreeing with its neighbours can put a step in the path that the nozzle rams
    // into; cap it to a ramp.
    const double                   cap = m_cfg.max_dz_per_segment > 0. ? m_cfg.max_dz_per_segment : m_cfg.seg_size;
    std::optional<Eigen::Vector3d> prev;
    for (Record &r : m_records)
        for (Move &m : r.moves) {
            if (m.pure_e)
                continue;
            if (prev && m.printing) {
                const double dz = m.pos.z() - prev->z();
                if (std::abs(dz) > cap) {
                    m.pos.z() = prev->z() + std::copysign(cap, dz);
                    ++m_report.z_steps_limited;
                }
            }
            prev = m.pos;
        }
}

// Highest material deposited so far over each XY cell.
class HeightField
{
public:
    HeightField(const Eigen::Vector2d &lo, const Eigen::Vector2d &hi, double res, double radius)
        : m_res(std::max(res, 1e-3)), m_origin(lo - Eigen::Vector2d::Constant(2.))
    {
        const Eigen::Vector2d span = hi + Eigen::Vector2d::Constant(2.) - m_origin;
        m_nx  = std::max(int(std::ceil(span.x() / m_res)) + 1, 1);
        m_ny  = std::max(int(std::ceil(span.y() / m_res)) + 1, 1);
        m_pad = std::max(int(std::lround(radius / m_res)), 1);
        m_grid.assign(size_t(m_nx) * m_ny, -std::numeric_limits<double>::infinity());
    }
    void stamp(const Eigen::Vector3d &p)
    {
        int i, j;
        cell(p, i, j);
        for (int a = std::max(i - m_pad, 0); a <= std::min(i + m_pad, m_nx - 1); ++a)
            for (int b = std::max(j - m_pad, 0); b <= std::min(j + m_pad, m_ny - 1); ++b)
                m_grid[size_t(a) * m_ny + b] = std::max(m_grid[size_t(a) * m_ny + b], p.z());
    }
    // The whole bead, not just where it ended: the next layer must find support along it.
    void stamp_segment(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1)
    {
        const int n = std::max(int(std::ceil((p1 - p0).head<2>().norm() / (m_res * 0.5))), 1);
        for (int k = 0; k <= n; ++k)
            stamp(p0 + (p1 - p0) * (double(k) / n));
    }
    // Calls f(cell centre, height) for every cell holding material within `radius` of `c`.
    template<class F> void for_each_within(const Eigen::Vector2d &c, double radius, F f) const
    {
        const int i0 = std::max(int(std::floor((c.x() - radius - m_origin.x()) / m_res)), 0);
        const int i1 = std::min(int(std::floor((c.x() + radius - m_origin.x()) / m_res)), m_nx - 1);
        const int j0 = std::max(int(std::floor((c.y() - radius - m_origin.y()) / m_res)), 0);
        const int j1 = std::min(int(std::floor((c.y() + radius - m_origin.y()) / m_res)), m_ny - 1);
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j) {
                const double h = m_grid[size_t(i) * m_ny + j];
                if (h == -std::numeric_limits<double>::infinity())
                    continue;
                const Eigen::Vector2d centre = m_origin + Eigen::Vector2d(i + 0.5, j + 0.5) * m_res;
                if ((centre - c).squaredNorm() <= radius * radius && ! f(centre, h))
                    return;
            }
    }
    double probe(const Eigen::Vector3d &p) const
    {
        int i, j;
        cell(p, i, j);
        double h = -std::numeric_limits<double>::infinity();
        for (int a = std::max(i - m_pad, 0); a <= std::min(i + m_pad, m_nx - 1); ++a)
            for (int b = std::max(j - m_pad, 0); b <= std::min(j + m_pad, m_ny - 1); ++b)
                h = std::max(h, m_grid[size_t(a) * m_ny + b]);
        return h;
    }

private:
    void cell(const Eigen::Vector3d &p, int &i, int &j) const
    {
        i = std::clamp(int((p.x() - m_origin.x()) / m_res), 0, m_nx - 1);
        j = std::clamp(int((p.y() - m_origin.y()) / m_res), 0, m_ny - 1);
    }
    double              m_res;
    Eigen::Vector2d     m_origin;
    int                 m_nx = 1, m_ny = 1, m_pad = 1;
    std::vector<double> m_grid;
};

void Transform::pass_travel_safety()
{
    // A travel is lifted when it would plough through material already printed along its path
    // (a curved layer's travel is not at a safe height by construction), and retracted when it
    // is long or had to hop. Travels the slicer already retracted for keep its retraction.
    if (! m_cfg.travel_safety)
        return;
    Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::max()), hi = -lo;
    for (const Record &r : m_records)
        for (const Move &m : r.moves)
            if (! m.pure_e) {
                lo = lo.cwiseMin(m.pos.head<2>());
                hi = hi.cwiseMax(m.pos.head<2>());
            }
    if (lo.x() > hi.x())
        return;
    HeightField hf(lo, hi, m_cfg.height_field_res, m_cfg.nozzle_radius);

    std::vector<char> already(m_next_id, 0); // first travel move after a slicer retraction
    {
        std::optional<double> last_pure_e = m_start_retracted ? std::optional<double>(-1.) : std::nullopt;
        for (const Record &r : m_records)
            for (const Move &m : r.moves) {
                if (m.pure_e)
                    last_pure_e = m.e;
                else {
                    if (last_pure_e && *last_pure_e < 0.)
                        already[m.id] = 1;
                    last_pure_e.reset();
                }
            }
    }

    struct Ref { size_t record; int id; };
    auto find = [this](const Ref &ref) -> Move & {
        std::vector<Move> &v = m_records[ref.record].moves;
        return *std::find_if(v.begin(), v.end(), [&ref](const Move &m) { return m.id == ref.id; });
    };
    auto index_of = [this](const Ref &ref) {
        std::vector<Move> &v = m_records[ref.record].moves;
        return size_t(std::find_if(v.begin(), v.end(), [&ref](const Move &m) { return m.id == ref.id; }) - v.begin());
    };

    std::optional<Eigen::Vector3d> anchor; // last printing position: where the nozzle is
    auto flush = [&](std::vector<Ref> &run) {
        if (run.empty())
            return;
        double                         length = 0.;
        std::optional<Eigen::Vector3d> prev;
        for (const Ref &ref : run) {
            const Move &m = find(ref);
            if (prev)
                length += (m.pos - *prev).norm();
            prev = m.pos;
        }
        // Only the interior can collide: the ends sit on the beads the travel leaves and rejoins.
        double obstruction = -std::numeric_limits<double>::infinity();
        for (size_t k = 1; k + 1 < run.size(); ++k) {
            const Move  &m = find(run[k]);
            const double h = hf.probe(m.pos);
            if (h - m.pos.z() > m_cfg.travel_clearance)
                obstruction = std::max(obstruction, h);
        }
        const bool collided = std::isfinite(obstruction);
        if (collided)
            ++m_report.collisions;
        const bool pre_retracted = already[run.front().id];
        const bool want_retract  = ! pre_retracted &&
                                  (length >= m_cfg.min_travel_for_retract || (collided && length >= m_cfg.min_travel_for_collision_retract));

        if (collided) {
            const double target = obstruction + m_cfg.travel_clearance + m_cfg.z_hop;
            const double land_z = find(run.back()).pos.z();
            for (const Ref &ref : run) {
                Move &m = find(ref);
                if (m.pos.z() < target) {
                    m.pos.z() = target;
                    m.rapid   = true;
                }
            }
            // Climb and descend vertically: a diagonal lift drags the nozzle across whatever
            // it is trying to clear.
            const Move &first = find(run.front());
            Move        lift  = make_move(Eigen::Vector3d::Zero());
            const Eigen::Vector3d at = anchor ? *anchor : first.pos;
            lift.pos       = Eigen::Vector3d(at.x(), at.y(), target);
            lift.source    = first.source;
            lift.tilt      = first.tilt;
            lift.feed      = m_cfg.z_hop_feed;
            lift.synthetic = true;
            m_records[run.front().record].moves.insert(m_records[run.front().record].moves.begin() + index_of(run.front()), lift);
            const Move &last = find(run.back());
            if (land_z < target - 1e-9) {
                Move descend      = make_move(Eigen::Vector3d(last.pos.x(), last.pos.y(), land_z));
                descend.source    = last.source;
                descend.tilt      = last.tilt;
                descend.feed      = m_cfg.z_hop_feed;
                descend.synthetic = true;
                m_records[run.back().record].moves.insert(m_records[run.back().record].moves.begin() + index_of(run.back()) + 1, descend);
            }
            ++m_report.lifts;
        }
        if (! want_retract)
            return;
        {
            const Move &first   = find(run.front());
            Move        retract = make_move(first.pos);
            retract.tilt        = first.tilt;
            retract.e           = -m_retract_length;
            retract.feed        = m_retract_feed;
            retract.pure_e      = true;
            retract.synthetic   = true;
            const size_t at     = index_of(run.front());
            auto &moves         = m_records[run.front().record].moves;
            moves.insert(moves.begin() + (collided && at > 0 ? at - 1 : at), retract);
        }
        {
            const Move &last  = find(run.back());
            Move        prime = make_move(last.pos);
            prime.tilt        = last.tilt;
            prime.e           = m_retract_length;
            prime.feed        = m_retract_feed;
            prime.pure_e      = true;
            prime.synthetic   = true;
            auto  &moves      = m_records[run.back().record].moves;
            size_t at         = index_of(run.back()) + 1;
            while (at < moves.size() && moves[at].synthetic && ! moves[at].pure_e)
                ++at; // after the descend
            moves.insert(moves.begin() + at, prime);
        }
        ++m_report.retractions_added;
    };

    std::vector<Ref> run;
    for (size_t ri = 0; ri < m_records.size(); ++ri) {
        std::vector<int> ids;
        for (const Move &m : m_records[ri].moves)
            ids.push_back(m.id);
        for (int id : ids) {
            const Move &m = find({ ri, id });
            if (m.pure_e) {
                flush(run);
                run.clear();
            } else if (m.e && *m.e > 0.) {
                flush(run);
                run.clear();
                const Move &p = find({ ri, id });
                if (anchor)
                    hf.stamp_segment(*anchor, p.pos);
                else
                    hf.stamp(p.pos);
                anchor = p.pos;
            } else
                run.push_back({ ri, id });
        }
    }
    flush(run);
}

// Smallest value of a z^2 + b z + c over [lo, hi] is <= 0.
bool quadratic_reaches_zero(double a, double b, double c, double lo, double hi)
{
    if (lo > hi)
        return false;
    auto f = [a, b, c](double z) { return (a * z + b) * z + c; };
    if (f(lo) <= 0. || f(hi) <= 0.)
        return true;
    if (a > 0.) {
        const double v = -b / (2. * a);
        return v > lo && v < hi && f(v) <= 0.;
    }
    return false;
}

void Transform::pass_head_clearance()
{
    // Material printed so far is a height field; each checked nozzle position tests every column
    // of it against the nozzle cone and the head cylinder along the nozzle axis. A column is
    // solid from the bed to its top, so it hits the head when the vertical line below its top
    // (less the tolerance) enters either volume: a quadratic in height for each.
    if (! m_cfg.clearance_check)
        return;
    Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::max()), hi = -lo;
    for (const Record &r : m_records)
        for (const Move &m : r.moves)
            if (! m.pure_e) {
                lo = lo.cwiseMin(m.pos.head<2>());
                hi = hi.cwiseMax(m.pos.head<2>());
            }
    if (lo.x() > hi.x())
        return;
    HeightField hf(lo, hi, m_cfg.height_field_res, m_cfg.nozzle_radius);

    const double k        = std::tan(std::clamp(m_cfg.nozzle_cone_angle, 0., 1.5));
    const double r0       = m_cfg.nozzle_tip_radius;
    const double len      = std::max(m_cfg.nozzle_length, 0.);
    const double big_r    = std::max(m_cfg.head_radius, r0 + k * len);
    const double ignore2  = m_cfg.clearance_ignore_radius * m_cfg.clearance_ignore_radius;
    double       top_z    = -std::numeric_limits<double>::infinity();
    auto         collides = [&](const Eigen::Vector3d &tip, double tilt) {
        Eigen::Vector2d rhat = tip.head<2>() - m_cfg.offset.head<2>() - m_axis;
        rhat                 = rhat.norm() > 1e-9 ? Eigen::Vector2d(rhat.normalized()) : Eigen::Vector2d(1., 0.);
        const Eigen::Vector3d axis(std::sin(tilt) * rhat.x(), std::sin(tilt) * rhat.y(), std::cos(tilt));
        const double          az = axis.z();
        // The head reaches sideways by its radius plus how far its axis leans below the highest
        // material.
        const double s_max = (std::max(top_z - tip.z(), 0.) + big_r * axis.head<2>().norm()) / az;
        const double reach = big_r + std::max(len, s_max) * axis.head<2>().norm() + m_cfg.height_field_res;
        // The bed: the lowest points of the cone and of the cylinder's lower rim, on the side the
        // nozzle leans away from.
        const double lean     = axis.head<2>().norm();
        const double cone_low = std::min(tip.z() - r0 * lean, tip.z() + len * az - (r0 + k * len) * lean);
        const double cyl_low  = tip.z() + len * az - big_r * lean;
        if (std::min(cone_low, cyl_low) < m_cfg.z_floor - m_cfg.clearance_tolerance)
            return true;
        bool         hit   = false;
        hf.for_each_within(tip.head<2>(), std::min(reach, 250.), [&](const Eigen::Vector2d &c, double h) {
            const Eigen::Vector2d d = c - tip.head<2>();
            if (d.squaredNorm() <= ignore2)
                return true;
            const double top = h - m_cfg.clearance_tolerance - tip.z(); // column top, relative to the tip
            const double s0  = d.x() * axis.x() + d.y() * axis.y();       // axis coordinate at the tip's height
            const double w2  = d.squaredNorm();
            // Along the column, z -> axis coordinate s = s0 + az z and squared distance from
            // the axis w2 + z^2 - s^2.
            const double cone_a = 1. - az * az - k * k * az * az;
            const double cone_b = -2. * s0 * az - 2. * k * az * (r0 + k * s0);
            const double cone_c = w2 - s0 * s0 - (r0 + k * s0) * (r0 + k * s0);
            const double cyl_a = 1. - az * az, cyl_b = -2. * s0 * az, cyl_c = w2 - s0 * s0 - big_r * big_r;
            hit = quadratic_reaches_zero(cone_a, cone_b, cone_c, -s0 / az, std::min((len - s0) / az, top)) ||
                  quadratic_reaches_zero(cyl_a, cyl_b, cyl_c, (len - s0) / az, top);
            return ! hit;
        });
        return hit;
    };

    std::optional<Eigen::Vector3d> prev;
    double                         travelled = std::numeric_limits<double>::infinity();
    int                            layer     = 0;
    for (const Record &r : m_records) {
        if (! r.motion && is_layer_marker(r.raw))
            ++layer;
        for (const Move &m : r.moves) {
            if (m.pure_e || m.dropped)
                continue;
            if (prev) {
                travelled += (m.pos - *prev).norm();
                if (travelled >= m_cfg.clearance_check_interval) {
                    travelled = 0.;
                    if (collides(m.pos, m_cfg.emit_tilt ? m.tilt : 0.)) {
                        if (m_report.head_collisions++ < 5) {
                            char buf[160];
                            std::snprintf(buf, sizeof(buf), "layer %d at X%.2f Y%.2f Z%.2f", layer, m.pos.x(), m.pos.y(), m.pos.z());
                            m_report.head_collision_samples.emplace_back(buf);
                        }
                    }
                }
                if (m.printing && m.e && *m.e > 0.) {
                    hf.stamp_segment(*prev, m.pos);
                    top_z = std::max(top_z, std::max(prev->z(), m.pos.z()));
                }
            }
            prev = m.pos;
        }
    }
}

std::vector<std::string> Transform::emit()
{
    // Modal output: an axis word is written when its value changes, or when the source line
    // commanded it, so a Z-only source move stays Z-only.
    std::vector<std::string>       out;
    std::optional<Eigen::Vector3d> pos;
    std::optional<double>          feed, tilt_deg;
    double                         e_rounding = 0.;
    for (const Record &r : m_records) {
        if (! r.motion || r.verbatim) {
            out.push_back(r.raw);
            continue;
        }
        if (r.moves.empty()) {
            out.push_back("; [collapsed] " + r.raw);
            continue;
        }
        for (const Move &m : r.moves) {
            std::string line = m.rapid && ! m.e ? "G0" : "G1";
            const size_t bare = line.size();
            if (! m.pure_e) {
                for (int k = 0; k < 3; ++k) {
                    const double v = m.pos[k];
                    if (! pos || std::abs(v - (*pos)[k]) > 0.5e-4 || m.axes.find("XYZ"[k]) != std::string::npos)
                        line += std::string(" ") + "XYZ"[k] + GCodeWords::number(v, 4);
                }
                const double t = m.tilt * 180. / PI;
                if (m_cfg.emit_tilt && (! tilt_deg || std::abs(t - *tilt_deg) > 0.5e-3)) {
                    line += std::string(" ") + m_cfg.tilt_axis + GCodeWords::number(t, 3);
                    tilt_deg = t;
                }
            }
            if (m.e) {
                // Carry the rounding error so subdivided moves keep the total exact.
                const double wanted = *m.e + e_rounding;
                const double shown  = std::round(wanted * 1e5) * 1e-5;
                e_rounding          = wanted - shown;
                line += " E" + GCodeWords::number(shown, 5);
            }
            if (m.feed && (! feed || *m.feed != *feed || m.synthetic)) {
                line += " F" + GCodeWords::number(*m.feed, 3);
                feed = m.feed;
            }
            if (line.size() == bare)
                continue;
            if (! m.pure_e)
                pos = m.pos;
            out.push_back(std::move(line));
        }
    }
    return out;
}

void Transform::validate(const std::vector<std::string> &out_lines)
{
    auto fail = [this](const std::string &what) { m_report.failed.push_back(what); };

    size_t in_layers = 0, out_layers = 0;
    for (const std::string &l : m_body)
        in_layers += is_layer_marker(l);
    for (const std::string &l : out_lines)
        out_layers += is_layer_marker(l);
    if (in_layers != out_layers)
        fail("layer markers: " + std::to_string(out_layers) + " of " + std::to_string(in_layers) + " kept");

    // Everything from the machine end block on is copied, so it must end the output verbatim.
    size_t tail = 0;
    for (auto it = m_records.rbegin(); it != m_records.rend() && it->verbatim; ++it)
        ++tail;
    for (size_t k = 0; k < tail; ++k)
        if (tail > out_lines.size() || out_lines[out_lines.size() - tail + k] != m_body[m_body.size() - tail + k]) {
            fail("end block not reproduced verbatim");
            break;
        }

    bool   retracted = m_start_retracted;
    size_t double_retract = 0, unmatched_prime = 0;
    for (const Record &r : m_records)
        for (const Move &m : r.moves)
            if (m.pure_e && m.e) {
                if (*m.e < 0.) {
                    double_retract += retracted;
                    retracted = true;
                } else if (*m.e > 0.) {
                    unmatched_prime += ! retracted;
                    retracted = false;
                }
            }
    if (double_retract + unmatched_prime > 0)
        fail("retractions unbalanced: " + std::to_string(double_retract) + " double retract, " + std::to_string(unmatched_prime) +
             " unmatched prime");

    bool                           first = true;
    size_t                         run = 0, worst = 0;
    std::optional<Eigen::Vector3d> prev;
    for (const Record &r : m_records)
        for (const Move &m : r.moves) {
            if (m.pure_e)
                continue;
            m_report.min_z        = first ? m.pos.z() : std::min(m_report.min_z, m.pos.z());
            m_report.max_z        = first ? m.pos.z() : std::max(m_report.max_z, m.pos.z());
            m_report.max_tilt_deg = std::max(m_report.max_tilt_deg, std::abs(m.tilt) * 180. / PI);
            first                 = false;
            if (prev && (m.pos - *prev).norm() <= m_cfg.degenerate_tol)
                worst = std::max(worst, ++run);
            else
                run = 0;
            prev = m.pos;
        }
    if (! first && m_report.min_z < m_cfg.z_floor - 1e-9)
        fail("moves below the bed");
    if (worst > 2)
        fail("stationary run of " + std::to_string(worst) + " moves");
}

S4GCodeReport Transform::run(std::istream &in, std::ostream &out)
{
    std::vector<std::string> lines;
    std::string              line;
    while (std::getline(in, line)) {
        if (! line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(std::move(line));
    }
    split_blocks(lines);
    build_records();
    for (const Record &r : m_records)
        m_report.segments += r.moves.size();
    map_moves();
    pass_junctions();
    pass_z_floor();
    pass_extrusion();
    pass_drop_degenerate();
    pass_limit_z_rate();
    pass_travel_safety();
    pass_drop_degenerate(); // lift/descend twins at the same spot
    pass_head_clearance();
    const std::vector<std::string> body = emit();
    validate(body);

    for (const std::string &l : m_start)
        out << l << '\n';
    out << "; S4 non-planar toolpath: sliced on the deformed mesh, mapped back to the part\n";
    for (const std::string &l : body)
        out << l << '\n';
    m_report.source_lines = m_body.size();
    m_report.output_lines = body.size();
    return m_report;
}

} // namespace

// The tree keeps a pointer to the mesh it was built on.
struct S4SupportSurface::Tree
{
    indexed_triangle_set mesh;
    AABBMesh             aabb;
    explicit Tree(const indexed_triangle_set &surface) : mesh(surface), aabb(mesh) {}
};

S4SupportSurface::S4SupportSurface(const indexed_triangle_set &surface) : m_tree(std::make_unique<Tree>(surface)) {}

S4SupportSurface::~S4SupportSurface() = default;

std::optional<double> S4SupportSurface::next(const Eigen::Vector3d &p, bool up, bool *inside) const
{
    const AABBMesh::hit_result hit = m_tree->aabb.query_ray_hit(p, Vec3d(0., 0., up ? 1. : -1.));
    if (inside && hit.is_inside())
        *inside = true;
    if (! hit.is_hit() || hit.is_inside())
        return std::nullopt;
    return p.z() + (up ? hit.distance() : -hit.distance());
}

S4GCodeReport s4_transform_gcode(std::istream &in, std::ostream &out, const S4MapperSet &mappers, const S4GCodeConfig &config)
{
    Transform t(mappers, config);
    return t.run(in, out);
}

S4GCodeReport s4_transform_gcode(std::istream &in, std::ostream &out, const S4Mapper &mapper, const S4GCodeConfig &config)
{
    S4MapperSet mappers;
    mappers.fallback.mapper = &mapper;
    return s4_transform_gcode(in, out, mappers, config);
}

} // namespace NonPlanar
} // namespace Slic3r
