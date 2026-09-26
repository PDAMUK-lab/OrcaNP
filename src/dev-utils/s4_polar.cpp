// s4_polar: the S4 non-planar pipeline for polar printers, run around any slicer.
//
//   s4_polar deform <model.stl> <prefix> [options]
//       Fills the model with tetrahedra and deforms it so overhangs become printable.
//       Writes <prefix>_deformed.stl (slice this) and <prefix>.s4mesh (keep it for `map`).
//   s4_polar map <prefix>.s4mesh <sliced.gcode> <out.gcode> [options]
//       Maps G-code sliced from the deformed model back onto the real part, with nozzle tilt,
//       and with --polar converts it to polar machine coordinates.
//   s4_polar polar <in.gcode> <out.gcode> [options]
//       Converts Cartesian G-code to polar machine coordinates.
//
// Slice with relative extrusion (M83) and arc fitting off. Where the slicer places the part does
// not matter: `map` finds it, and the rotation axis moves with the part.

#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/NonPlanar/PolarKinematics.hpp"
#include "libslic3r/NonPlanar/S4Deformation.hpp"
#include "libslic3r/NonPlanar/S4GCodeTransform.hpp"
#include "libslic3r/NonPlanar/S4Mapping.hpp"
#include "libslic3r/NonPlanar/Tetrahedralize.hpp"

#include <admesh/stl.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Slic3r::NonPlanar;

namespace {

const char *usage =
    "usage:\n"
    "  s4_polar deform <model.stl> <prefix> [--axis X Y] [--cell-size MM] [--max-overhang DEG]\n"
    "                  [--neighbour-weight W] [--rotation-multiplier M] [--max-rotation-near DEG]\n"
    "                  [--max-rotation-far DEG] [--passes N]\n"
    "  s4_polar map <prefix>.s4mesh <sliced.gcode> <out.gcode> [--offset X Y Z] [--polar] [--no-tilt]\n"
    "                  [--seg-size MM] [--z-hop MM] [polar options]\n"
    "  s4_polar polar <in.gcode> <out.gcode> [--center X Y] [polar options]\n"
    "polar options: [--angle-sign 1|-1] [--max-angle-step DEG] [--min-radius MM] [--feedrate]\n"
    "               [--no-signed-radius] [--tilt-pivot MM] [--axes C X B]\n";

struct Args
{
    std::vector<std::string>                        positional;
    std::map<std::string, std::vector<std::string>> options;

    bool   has(const std::string &k) const { return options.count(k) > 0; }
    double num(const std::string &k, size_t i, double fallback) const
    {
        auto it = options.find(k);
        return it == options.end() || it->second.size() <= i ? fallback : std::stod(it->second[i]);
    }
};

// Options and how many values each takes; anything else is positional.
Args parse(int argc, char **argv)
{
    static const std::map<std::string, size_t> arity = {
        { "axis", 2 }, { "center", 2 }, { "offset", 3 }, { "axes", 3 }, { "polar", 0 }, { "no-tilt", 0 }, { "feedrate", 0 },
        { "no-signed-radius", 0 } };
    Args a;
    for (int i = 2; i < argc; ++i) {
        const std::string w = argv[i];
        if (w.rfind("--", 0) != 0) {
            a.positional.push_back(w);
            continue;
        }
        const std::string name  = w.substr(2);
        const auto        it    = arity.find(name);
        const size_t      count = it == arity.end() ? 1 : it->second;
        std::vector<std::string> &values = a.options[name];
        for (size_t k = 0; k < count; ++k) {
            if (++i >= argc)
                throw std::runtime_error("--" + name + " needs " + std::to_string(count) + " value(s)");
            values.push_back(argv[i]);
        }
    }
    return a;
}

void load_stl(const std::string &path, std::vector<Eigen::Vector3d> &vertices, std::vector<std::array<int, 3>> &triangles)
{
    stl_file stl;
    if (! stl_open(&stl, path.c_str()))
        throw std::runtime_error("cannot read " + path);
    stl_check_facets_exact(&stl);
    indexed_triangle_set its;
    stl_generate_shared_vertices(&stl, its);
    for (const stl_vertex &v : its.vertices)
        vertices.emplace_back(v.cast<double>());
    for (const stl_triangle_vertex_indices &t : its.indices)
        triangles.push_back({ t[0], t[1], t[2] });
}

// Binary STL of the tetrahedral mesh's boundary at the given vertex positions.
void write_boundary_stl(const std::string &path, const TetMesh &mesh, const std::vector<Eigen::Vector3d> &pts)
{
    const std::vector<std::array<int, 3>> faces = s4_boundary_triangles(mesh, pts);
    std::ofstream                         out(path, std::ios::binary);
    char                                  header[80] = "S4 deformed model";
    out.write(header, 80);
    const uint32_t count = uint32_t(faces.size());
    out.write(reinterpret_cast<const char *>(&count), 4);
    for (const std::array<int, 3> &f : faces) {
        const Eigen::Vector3d n = (pts[f[1]] - pts[f[0]]).cross(pts[f[2]] - pts[f[0]]).normalized();
        float                 rec[12];
        for (int c = 0; c < 3; ++c)
            rec[c] = float(n[c]);
        for (int v = 0; v < 3; ++v)
            for (int c = 0; c < 3; ++c)
                rec[3 + v * 3 + c] = float(pts[f[v]][c]);
        out.write(reinterpret_cast<const char *>(rec), 48);
        const uint16_t attr = 0;
        out.write(reinterpret_cast<const char *>(&attr), 2);
    }
    if (! out)
        throw std::runtime_error("cannot write " + path);
}

struct S4MeshFile
{
    Eigen::Vector2d              axis { 0., 0. };
    TetMesh                      mesh;
    std::vector<Eigen::Vector3d> deformed;
};

void write_s4mesh(const std::string &path, const S4MeshFile &f)
{
    std::ofstream out(path);
    out.precision(17);
    out << "S4MESH 1\naxis " << f.axis.x() << ' ' << f.axis.y() << '\n' << f.mesh.points.size() << ' ' << f.mesh.tets.size() << '\n';
    for (size_t i = 0; i < f.mesh.points.size(); ++i)
        out << f.mesh.points[i].transpose() << ' ' << f.deformed[i].transpose() << '\n';
    for (const auto &t : f.mesh.tets)
        out << t[0] << ' ' << t[1] << ' ' << t[2] << ' ' << t[3] << '\n';
    if (! out)
        throw std::runtime_error("cannot write " + path);
}

S4MeshFile read_s4mesh(const std::string &path)
{
    std::ifstream in(path);
    std::string   magic, word;
    int           version = 0;
    size_t        np = 0, nt = 0;
    S4MeshFile    f;
    in >> magic >> version >> word >> f.axis.x() >> f.axis.y() >> np >> nt;
    if (! in || magic != "S4MESH" || version != 1 || word != "axis")
        throw std::runtime_error(path + " is not an S4 mesh file");
    f.mesh.points.resize(np);
    f.deformed.resize(np);
    for (size_t i = 0; i < np; ++i)
        in >> f.mesh.points[i].x() >> f.mesh.points[i].y() >> f.mesh.points[i].z() >> f.deformed[i].x() >> f.deformed[i].y() >>
            f.deformed[i].z();
    f.mesh.tets.resize(nt);
    for (auto &t : f.mesh.tets)
        in >> t[0] >> t[1] >> t[2] >> t[3];
    if (! in)
        throw std::runtime_error(path + " is truncated");
    return f;
}

PolarKinematicsConfig polar_config(const Args &a, const Eigen::Vector2d &center, bool tilt)
{
    PolarKinematicsConfig c;
    c.center            = center;
    c.angle_sign        = a.num("angle-sign", 0, 1.);
    c.max_angle_step    = a.num("max-angle-step", 0, c.max_angle_step);
    c.min_radius        = a.num("min-radius", 0, c.min_radius);
    c.inverse_time_feed = ! a.has("feedrate");
    c.signed_radius     = ! a.has("no-signed-radius");
    c.tilt_pivot_length = a.num("tilt-pivot", 0, 0.);
    c.has_tilt_axis     = tilt;
    if (a.has("axes") && a.options.at("axes").size() == 3) {
        c.angle_axis  = a.options.at("axes")[0][0];
        c.radius_axis = a.options.at("axes")[1][0];
        c.tilt_axis   = a.options.at("axes")[2][0];
    }
    return c;
}

// Where the slicer put the part: the centre of the part's own extrusion (no skirt, brim or purge)
// over the centre of the deformed mesh, and the bed under the mesh's lowest point. The body starts
// after Orca's start tag, or at the first layer marker.
Eigen::Vector3d detect_offset(const std::string &gcode, const std::vector<Eigen::Vector3d> &deformed)
{
    Eigen::Vector2d    lo = Eigen::Vector2d::Constant(1e300), hi = -lo;
    std::istringstream in(gcode);
    std::string        line, type;
    double             x = 0., y = 0.;
    const bool         tagged = gcode.find("MACHINE_START_GCODE_END") != std::string::npos;
    bool               body   = false;
    while (std::getline(in, line)) {
        if (! body) {
            body = tagged ? line.find("MACHINE_START_GCODE_END") != std::string::npos : line.rfind(";LAYER", 0) == 0;
            continue;
        }
        if (line.find("MACHINE_END_GCODE_START") != std::string::npos)
            break;
        if (line.rfind(";TYPE:", 0) == 0) {
            type = line.substr(6);
            std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return char(std::toupper(c)); });
            continue;
        }
        if (line.rfind("G1", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string        w;
        words >> w;
        bool   xy = false;
        double e  = 0.;
        while (words >> w) {
            if (w[0] == 'X' || w[0] == 'Y') {
                (w[0] == 'X' ? x : y) = std::stod(w.substr(1));
                xy = true;
            } else if (w[0] == 'E')
                e = std::stod(w.substr(1));
        }
        const bool helper = type.find("SKIRT") != std::string::npos || type.find("BRIM") != std::string::npos ||
                            type.find("PRIME") != std::string::npos || type.find("PURGE") != std::string::npos;
        if (xy && e > 0. && ! helper) {
            lo = lo.cwiseMin(Eigen::Vector2d(x, y));
            hi = hi.cwiseMax(Eigen::Vector2d(x, y));
        }
    }
    if (lo.x() > hi.x())
        throw std::runtime_error("no extruding moves of the part found in the sliced G-code; pass --offset");
    Eigen::Vector3d mlo = deformed.front(), mhi = mlo;
    for (const Eigen::Vector3d &p : deformed) {
        mlo = mlo.cwiseMin(p);
        mhi = mhi.cwiseMax(p);
    }
    const Eigen::Vector2d offset_xy = 0.5 * (lo + hi) - 0.5 * (mlo + mhi).head<2>();
    return Eigen::Vector3d(offset_xy.x(), offset_xy.y(), -mlo.z());
}

int cmd_deform(const Args &a)
{
    if (a.positional.size() != 2)
        throw std::runtime_error(usage);
    std::vector<Eigen::Vector3d>    vertices;
    std::vector<std::array<int, 3>> triangles;
    load_stl(a.positional[0], vertices, triangles);
    Eigen::Vector3d lo = vertices.front(), hi = lo;
    for (const Eigen::Vector3d &v : vertices) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }

    S4MeshFile file;
    file.axis = Eigen::Vector2d(a.num("axis", 0, 0.5 * (lo.x() + hi.x())), a.num("axis", 1, 0.5 * (lo.y() + hi.y())));

    const auto           t0 = std::chrono::steady_clock::now();
    TetrahedralizeParams tp;
    tp.cell_size = a.num("cell-size", 0, 0.);
    file.mesh    = tetrahedralize(vertices, triangles, tp);
    const auto t1 = std::chrono::steady_clock::now();

    S4Params p;
    p.axis                = file.axis;
    p.max_overhang        = a.num("max-overhang", 0, p.max_overhang);
    p.neighbour_weight    = a.num("neighbour-weight", 0, p.neighbour_weight);
    p.rotation_multiplier = a.num("rotation-multiplier", 0, p.rotation_multiplier);
    p.max_rotation_near   = a.num("max-rotation-near", 0, p.max_rotation_near);
    p.max_rotation_far    = a.num("max-rotation-far", 0, p.max_rotation_far);
    p.passes              = int(a.num("passes", 0, p.passes));
    const S4Result result = s4_deform(file.mesh, p);
    const auto     t2     = std::chrono::steady_clock::now();
    file.deformed         = result.deformed;

    const std::string prefix = a.positional[1];
    write_boundary_stl(prefix + "_deformed.stl", file.mesh, file.deformed);
    write_s4mesh(prefix + ".s4mesh", file);

    double zmax = -1e300;
    for (const Eigen::Vector3d &q : file.deformed)
        zmax = std::max(zmax, q.z());
    std::printf("%zu triangles -> %zu tetrahedra in %.2f s, deformed in %.2f s\n", triangles.size(), file.mesh.tets.size(),
                std::chrono::duration<double>(t1 - t0).count(), std::chrono::duration<double>(t2 - t1).count());
    for (size_t i = 0; i < result.passes.size(); ++i) {
        const S4PassData &d = result.passes[i];
        const auto [rmin, rmax] = std::minmax_element(d.rotation.begin(), d.rotation.end());
        std::printf("pass %zu: rotation %.1f .. %.1f deg, %d solve(s), %zu cells inverted\n", i + 1, *rmin * 180. / 3.14159265358979,
                    *rmax * 180. / 3.14159265358979, d.rounds, d.inverted);
    }
    std::printf("height %.2f -> %.2f mm; rotation axis at X%.3f Y%.3f\nwrote %s_deformed.stl and %s.s4mesh\n", hi.z() - lo.z(),
                zmax - lo.z(), file.axis.x(), file.axis.y(), prefix.c_str(), prefix.c_str());
    return 0;
}

int cmd_map(const Args &a)
{
    if (a.positional.size() != 3)
        throw std::runtime_error(usage);
    const S4MeshFile file = read_s4mesh(a.positional[0]);
    std::ifstream    in(a.positional[1]);
    if (! in)
        throw std::runtime_error("cannot read " + a.positional[1]);
    std::stringstream sliced;
    sliced << in.rdbuf();

    const bool      tilt   = ! a.has("no-tilt");
    Eigen::Vector3d offset = a.has("offset") ? Eigen::Vector3d(a.num("offset", 0, 0.), a.num("offset", 1, 0.), a.num("offset", 2, 0.)) :
                                               detect_offset(sliced.str(), file.deformed);
    const S4Mapper mapper(file.mesh, file.deformed, file.axis);
    S4GCodeConfig  cfg;
    cfg.offset          = offset;
    cfg.emit_tilt       = tilt;
    cfg.seg_size        = a.num("seg-size", 0, cfg.seg_size);
    cfg.z_hop           = a.num("z-hop", 0, cfg.z_hop);
    std::stringstream real;
    const S4GCodeReport r = s4_transform_gcode(sliced, real, mapper, cfg);

    std::printf("part offset X%.3f Y%.3f Z%.3f%s\n", offset.x(), offset.y(), offset.z(), a.has("offset") ? "" : " (detected)");
    std::printf("%zu source lines -> %zu segments: %zu inside, %zu beside, %zu interpolated, %zu outside the mesh\n", r.source_lines,
                r.segments, r.inside, r.nearest, r.idw, r.outside);
    std::printf("filament %.2f -> %.2f mm (%zu segments clipped); %zu moves raised to the bed; %zu travel lifts, %zu retractions "
                "added\nZ %.3f .. %.3f mm, nozzle tilt up to %.1f deg\n",
                r.filament_in, r.filament_out, r.flow_clipped_high + r.flow_clipped_low, r.floor_clamped, r.lifts, r.retractions_added,
                r.min_z, r.max_z, r.max_tilt_deg);
    for (const std::string &f : r.failed)
        std::printf("CHECK FAILED: %s\n", f.c_str());

    std::ofstream out(a.positional[2]);
    if (a.has("polar")) {
        const Eigen::Vector2d center = file.axis + offset.head<2>();
        PolarGCodeConverter   conv(polar_config(a, center, tilt));
        real.seekg(0);
        conv.process(real, out);
        std::printf("polar: rotation axis at X%.3f Y%.3f, %zu machine moves, bed turns %.0f deg in total\n", center.x(), center.y(),
                    conv.stats().machine_moves, conv.stats().total_angle);
    } else
        out << real.str();
    if (! out)
        throw std::runtime_error("cannot write " + a.positional[2]);
    std::printf("wrote %s\n", a.positional[2].c_str());
    return r.failed.empty() ? 0 : 2;
}

int cmd_polar(const Args &a)
{
    if (a.positional.size() != 2)
        throw std::runtime_error(usage);
    const Eigen::Vector2d                center(a.num("center", 0, 0.), a.num("center", 1, 0.));
    const PolarGCodeConverter::Stats s = convert_gcode_to_polar(a.positional[0], a.positional[1], polar_config(a, center, ! a.has("no-tilt")));
    std::printf("%zu moves -> %zu machine moves, bed turns %.0f deg in total, %zu arcs linearized\nwrote %s\n", s.cartesian_moves,
                s.machine_moves, s.total_angle, s.arcs_linearized, a.positional[1].c_str());
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fputs(usage, stderr);
        return 1;
    }
    try {
        // G-code and STL numbers use '.' decimals whatever the user's locale.
        const Slic3r::CNumericLocalesSetter locales;
        const std::string                   cmd = argv[1];
        const Args        a   = parse(argc, argv);
        if (cmd == "deform")
            return cmd_deform(a);
        if (cmd == "map")
            return cmd_map(a);
        if (cmd == "polar")
            return cmd_polar(a);
        std::fputs(usage, stderr);
        return 1;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "s4_polar: %s\n", e.what());
        return 1;
    }
}
