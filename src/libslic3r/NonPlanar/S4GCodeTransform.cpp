#include "S4GCodeTransform.hpp"
#include "GCodeWords.hpp"

#include <algorithm>
#include <cmath>
#include <istream>
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
    Transform(const S4Mapper &mapper, const S4GCodeConfig &cfg) : m_mapper(mapper), m_cfg(cfg) {}

    S4GCodeReport run(std::istream &in, std::ostream &out);

private:
    void split_blocks(const std::vector<std::string> &lines);
    void build_records();
    void map_moves();
    void pass_z_floor();
    void pass_extrusion();
    void pass_drop_degenerate();
    void pass_limit_z_rate();
    void pass_travel_safety();
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

    const S4Mapper      &m_mapper;
    const S4GCodeConfig &m_cfg;
    S4GCodeReport        m_report;

    std::vector<std::string> m_start, m_body;
    std::vector<Record>      m_records;
    bool                     m_relative_e     = false;
    bool                     m_start_retracted = false;
    double                   m_retract_length = 0., m_retract_feed = 0.;
    int                      m_next_id        = 0;
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

    for (size_t i = 0; i < m_body.size(); ++i) {
        const std::string     &raw = m_body[i];
        const GCodeWords::Line l   = GCodeWords::split(raw);
        const std::string      cmd = GCodeWords::command(l.code);
        Record                 rec;
        rec.raw = raw;
        if (cmd == "G90")
            absolute = true;
        else if (cmd == "G91")
            absolute = false;
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
            m.pure_e = true;
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
            const bool outside = ! printing && (! m_mapper.in_bbox(pos - m_cfg.offset) || ! m_mapper.in_bbox(target - m_cfg.offset));
            const int  n       = ! have_pos || outside || dist <= step ? 1 : std::min(m_cfg.max_segments_per_move, int(std::ceil(dist / step)));
            for (int k = 1; k <= n; ++k)
                path.push_back(pos + (target - pos) * (double(k) / n));
        }

        std::vector<double> lengths;
        double              total = 0.;
        Eigen::Vector3d     prev  = pos;
        for (const Eigen::Vector3d &p : path) {
            lengths.push_back((p - prev).norm());
            total += lengths.back();
            prev = p;
        }
        for (size_t k = 0; k < path.size(); ++k) {
            Move m     = make_move(path[k]);
            m.rapid    = cmd == "G0";
            m.feed     = feed;
            m.printing = printing;
            m.axes     = axes;
            if (e)
                m.e = *e * (total > 0. ? lengths[k] / total : 1. / path.size());
            rec.moves.push_back(m);
        }
        pos      = target;
        have_pos = true;
        m_records.push_back(std::move(rec));
    }
}

void Transform::map_moves()
{
    // Slicer output revisits coordinates constantly; map each distinct point once.
    std::unordered_map<std::string, S4Mapper::Result> cache;
    char                                              key[96];
    const Move                                       *last = nullptr;
    for (Record &r : m_records)
        for (Move &m : r.moves) {
            if (m.pure_e) {
                if (last) {
                    m.pos  = last->pos;
                    m.tilt = last->tilt;
                }
                continue;
            }
            std::snprintf(key, sizeof(key), "%.6f %.6f %.6f", m.source.x(), m.source.y(), m.source.z());
            auto it = cache.find(key);
            if (it == cache.end())
                it = cache.emplace(key, m_mapper.map(m.source - m_cfg.offset)).first;
            const S4Mapper::Result &res = it->second;
            m.pos  = res.point + m_cfg.offset;
            m.tilt = res.tilt;
            m.flow = res.flow;
            m.tier = res.tier;
            switch (res.tier) {
            case S4Mapper::Tier::Inside: ++m_report.inside; break;
            case S4Mapper::Tier::Nearest: ++m_report.nearest; break;
            case S4Mapper::Tier::Idw: ++m_report.idw; break;
            case S4Mapper::Tier::Outside: ++m_report.outside; break;
            }
            last = &m;
        }
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
    pass_z_floor();
    pass_extrusion();
    pass_drop_degenerate();
    pass_limit_z_rate();
    pass_travel_safety();
    pass_drop_degenerate(); // lift/descend twins at the same spot
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

S4GCodeReport s4_transform_gcode(std::istream &in, std::ostream &out, const S4Mapper &mapper, const S4GCodeConfig &config)
{
    Transform t(mapper, config);
    return t.run(in, out);
}

} // namespace NonPlanar
} // namespace Slic3r
