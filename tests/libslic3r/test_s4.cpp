#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/LayerShapes.hpp"
#include "libslic3r/NonPlanar/S4Deformation.hpp"
#include "libslic3r/NonPlanar/S4GCodeTransform.hpp"
#include "libslic3r/NonPlanar/S4Mapping.hpp"
#include "libslic3r/NonPlanar/Tetrahedralize.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace Slic3r::NonPlanar;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double PI = 3.14159265358979323846;

// Fills unit voxels (given by their minimum corner, in voxel units) with six tetrahedra each,
// all sharing the cube's main diagonal, so neighbouring voxels conform.
TetMesh voxel_mesh(const std::vector<std::array<int, 3>> &voxels, double size)
{
    TetMesh                                  mesh;
    std::map<std::tuple<int, int, int>, int> index;
    auto vertex = [&](int x, int y, int z) {
        auto [it, inserted] = index.emplace(std::make_tuple(x, y, z), int(mesh.points.size()));
        if (inserted)
            mesh.points.emplace_back(x * size, y * size, z * size);
        return it->second;
    };
    const int perms[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
    for (const auto &v : voxels)
        for (const auto &p : perms) {
            std::array<int, 3> c = v;
            std::array<int, 4> tet;
            tet[0] = vertex(c[0], c[1], c[2]);
            for (int k = 0; k < 3; ++k) {
                ++c[p[k]];
                tet[k + 1] = vertex(c[0], c[1], c[2]);
            }
            mesh.tets.push_back(tet);
        }
    return mesh;
}

struct Surface
{
    std::vector<Eigen::Vector3d>    vertices;
    std::vector<std::array<int, 3>> triangles;
};

// Closed surface of a union of unit voxels: every voxel face not shared with another voxel.
Surface voxel_surface(const std::vector<std::array<int, 3>> &voxels, double size)
{
    Surface                            s;
    std::map<std::array<int, 3>, int>  index;
    std::map<std::array<int, 3>, bool> filled;
    for (const auto &v : voxels)
        filled[v] = true;
    auto vertex = [&](std::array<int, 3> p) {
        auto [it, inserted] = index.emplace(p, int(s.vertices.size()));
        if (inserted)
            s.vertices.emplace_back(p[0] * size, p[1] * size, p[2] * size);
        return it->second;
    };
    for (const auto &v : voxels)
        for (int axis = 0; axis < 3; ++axis)
            for (int side = 0; side < 2; ++side) {
                std::array<int, 3> nb = v;
                nb[axis] += side ? 1 : -1;
                if (filled.count(nb))
                    continue;
                const int          u = (axis + 1) % 3, w = (axis + 2) % 3;
                std::array<int, 3> c[4];
                for (int k = 0; k < 4; ++k) {
                    c[k] = v;
                    c[k][axis] += side;
                }
                c[1][u] += 1;
                c[2][u] += 1;
                c[2][w] += 1;
                c[3][w] += 1;
                const int q[4] = { vertex(c[0]), vertex(c[1]), vertex(c[2]), vertex(c[3]) };
                // Winding does not matter: the mesher re-orients the soup.
                s.triangles.push_back({ q[0], q[1], q[2] });
                s.triangles.push_back({ q[0], q[2], q[3] });
            }
    return s;
}

// A post standing on the bed with an arm cantilevered outward (+X, away from the rotation axis
// at the origin): the arm's underside is a horizontal overhang.
std::vector<std::array<int, 3>> cantilever_voxels()
{
    std::vector<std::array<int, 3>> voxels;
    for (int z = 0; z < 10; ++z)
        for (int y = -1; y < 1; ++y)
            for (int x = 0; x < 2; ++x)
                voxels.push_back({ x, y, z });
    for (int z = 8; z < 10; ++z)
        for (int y = -1; y < 1; ++y)
            for (int x = 2; x < 8; ++x)
                voxels.push_back({ x, y, z });
    return voxels;
}

double signed_volume(const std::vector<Eigen::Vector3d> &pts, const std::array<int, 4> &t)
{
    return (pts[t[1]] - pts[t[0]]).cross(pts[t[2]] - pts[t[0]]).dot(pts[t[3]] - pts[t[0]]) / 6.;
}

double volume(const TetMesh &m)
{
    double v = 0.;
    for (const auto &t : m.tets)
        v += std::abs(signed_volume(m.points, t));
    return v;
}

// Steepest overhang (degrees from +Z) among the cantilever arm's underside faces, the boundary
// faces that start out at z = 16 facing down.
double arm_underside_overhang(const TetMesh &mesh, const std::vector<Eigen::Vector3d> &pts)
{
    std::map<std::array<int, 3>, std::pair<int, int>> faces; // face -> (count, opposite vertex)
    for (const auto &t : mesh.tets)
        for (int k = 0; k < 4; ++k) {
            std::array<int, 3> f;
            for (int i = 0, j = 0; i < 4; ++i)
                if (i != k)
                    f[j++] = t[i];
            std::sort(f.begin(), f.end());
            auto &e = faces[f];
            ++e.first;
            e.second = t[k];
        }
    double steepest = 0.;
    for (const auto &[f, e] : faces) {
        if (e.first != 1)
            continue;
        bool underside = true;
        for (int v : f)
            underside = underside && std::abs(mesh.points[v].z() - 16.) < 1e-9 && mesh.points[v].x() > 4. - 1e-9;
        if (! underside)
            continue;
        Eigen::Vector3d n = (pts[f[1]] - pts[f[0]]).cross(pts[f[2]] - pts[f[0]]).normalized();
        if (n.dot(pts[e.second] - pts[f[0]]) > 0.)
            n = -n;
        steepest = std::max(steepest, std::acos(std::clamp(n.z(), -1., 1.)) * 180. / PI);
    }
    return steepest;
}

struct Parsed
{
    std::vector<Eigen::Vector3d> printing; // end points of extruding moves
    double                       e_total  = 0.;
    double                       max_z    = 0.;
    size_t                       retracts = 0;
};

// Positions of the extruding moves and the extrusion total of a Cartesian G-code body.
Parsed parse_body(const std::string &gcode)
{
    Parsed             p;
    std::istringstream in(gcode);
    std::string        line;
    Eigen::Vector3d    pos = Eigen::Vector3d::Zero();
    while (std::getline(in, line)) {
        if (line.rfind("G1", 0) != 0 && line.rfind("G0", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string        w;
        words >> w;
        double e     = 0.;
        bool   has_e = false, moves = false;
        while (words >> w) {
            const double v = std::stod(w.substr(1));
            if (w[0] == 'X' || w[0] == 'Y' || w[0] == 'Z') {
                pos[w[0] - 'X'] = v;
                moves           = true;
            } else if (w[0] == 'E') {
                e     = v;
                has_e = true;
            }
        }
        p.max_z = std::max(p.max_z, pos.z());
        if (has_e) {
            p.e_total += e;
            if (moves && e > 0.)
                p.printing.push_back(pos);
            if (! moves && e < 0.)
                ++p.retracts;
        }
    }
    return p;
}

std::string transform(const std::string &gcode, const S4Mapper &mapper, S4GCodeReport *report = nullptr,
                      const S4GCodeConfig &config = S4GCodeConfig())
{
    std::istringstream  in(gcode);
    std::ostringstream  out;
    const S4GCodeReport r = s4_transform_gcode(in, out, mapper, config);
    if (report)
        *report = r;
    return out.str();
}

// A square perimeter inside the cantilever's post at height z, with relative extrusion.
std::string square(double z)
{
    std::ostringstream g;
    g << "G1 X1 Y-1 Z" << z << " F3000\n"
      << "G1 X3 Y-1 E0.1 F1200\nG1 X3 Y1 E0.1\nG1 X1 Y1 E0.1\nG1 X1 Y-1 E0.1\n";
    return g.str();
}

const std::string start_block = "G28 ; home\nM83\n; MACHINE_START_GCODE_END\n";
const std::string end_block   = "; MACHINE_END_GCODE_START\nG1 X170 Y180 F3000 ; park\nM84\n";

} // namespace

TEST_CASE("Vertices on the bed stay in place", "[S4]")
{
    const TetMesh  mesh   = voxel_mesh(cantilever_voxels(), 2.);
    const S4Result result = s4_deform(mesh, S4Params());

    size_t on_bed = 0;
    for (size_t v = 0; v < mesh.points.size(); ++v)
        if (mesh.points[v].z() == 0.) {
            ++on_bed;
            CHECK_THAT((result.deformed[v] - mesh.points[v]).norm(), WithinAbs(0., 1e-12));
        }
    CHECK(on_bed == 3 * 3); // the post's 2x2 voxel footprint has 3x3 vertices
}

TEST_CASE("An outward overhang turns towards its support within the rotation limits", "[S4]")
{
    const TetMesh mesh = voxel_mesh(cantilever_voxels(), 2.);
    S4Params      params;
    params.max_rotation_near = 45.;
    params.max_rotation_far  = 20.;
    const S4Result    result = s4_deform(mesh, params);
    const S4PassData &pass   = result.passes.front();

    bool any_target = false;
    for (size_t c = 0; c < mesh.tets.size(); ++c) {
        CHECK(std::abs(pass.rotation[c]) <= pass.limit[c] + 1e-9);
        if (! std::isnan(pass.target[c])) {
            any_target = true;
            // The support is inward (towards the axis), so the cells roll inward: negative.
            CHECK(pass.target[c] < 0.);
        }
    }
    REQUIRE(any_target);

    const double before = arm_underside_overhang(mesh, mesh.points);
    const double after  = arm_underside_overhang(mesh, result.deformed);
    CHECK_THAT(before, WithinAbs(180., 1e-9));
    CHECK(after < before - 10.);
}

TEST_CASE("Deformation does not invert any tetrahedron", "[S4]")
{
    const TetMesh mesh = voxel_mesh(cantilever_voxels(), 2.);
    S4Params      params;
    params.passes         = 2;
    const S4Result result = s4_deform(mesh, params);

    REQUIRE(result.passes.size() == 2);
    for (const auto &t : mesh.tets)
        CHECK(signed_volume(mesh.points, t) * signed_volume(result.deformed, t) > 0.);
}

TEST_CASE("Aggressive rotations are cut back until no tetrahedron is inverted", "[S4]")
{
    const TetMesh mesh = voxel_mesh(cantilever_voxels(), 2.);
    S4Params      params;
    params.rotation_multiplier = 4.;
    params.max_rotation_near   = 80.;
    params.max_rotation_far    = 80.;
    params.neighbour_weight    = 0.5;
    const S4Result result = s4_deform(mesh, params);

    // The first solve folds cells over; the repair has to kick in.
    CHECK(result.passes.front().rounds > 1);
    CHECK(result.passes.front().inverted == 0);
    for (const auto &t : mesh.tets)
        CHECK(signed_volume(mesh.points, t) * signed_volume(result.deformed, t) > 0.);
}

TEST_CASE("Warm start and multi-threading leave the deformation as it is", "[S4]")
{
    // Rotations aggressive enough to need several repair rounds, so the warm start has rounds to
    // start from; a zero factor size sends the multi-threaded solve down the iterative route.
    const TetMesh mesh = voxel_mesh(cantilever_voxels(), 2.);
    S4Params      params;
    params.rotation_multiplier = 4.;
    params.max_rotation_near   = 80.;
    params.max_rotation_far    = 80.;
    params.neighbour_weight    = 0.5;
    params.warm_start          = false;
    params.multithreading      = false;
    const S4Result plain = s4_deform(mesh, params);
    REQUIRE(plain.passes.front().rounds > 1);
    CHECK_FALSE(plain.passes.front().iterative);

    params.warm_start          = true;
    const S4Result warm        = s4_deform(mesh, params);
    params.multithreading      = true;
    params.iterative_factor_size = 0;
    const S4Result threaded    = s4_deform(mesh, params);
    CHECK(threaded.passes.front().iterative);

    CHECK(warm.passes.front().solves < plain.passes.front().solves);
    CHECK(warm.passes.front().rounds == plain.passes.front().rounds);
    double warm_dev = 0., threaded_dev = 0.;
    for (size_t v = 0; v < mesh.points.size(); ++v) {
        warm_dev     = std::max(warm_dev, (warm.deformed[v] - plain.deformed[v]).norm());
        threaded_dev = std::max(threaded_dev, (threaded.deformed[v] - plain.deformed[v]).norm());
    }
    CHECK(warm_dev < 1e-6);
    CHECK(threaded_dev < 1e-6);
}

TEST_CASE("Only the base stays at bed level", "[S4]")
{
    const TetMesh  mesh   = voxel_mesh(cantilever_voxels(), 2.);
    S4Params       params;
    params.bottom_threshold = 0.3;
    const S4Result result   = s4_deform(mesh, params);

    for (size_t v = 0; v < mesh.points.size(); ++v)
        CHECK(result.deformed[v].z() >= std::min(mesh.points[v].z(), params.bottom_threshold) - 1e-9);
}

TEST_CASE("A planar base stays as it is while the part above it still turns", "[S4]")
{
    // The post is 20 mm tall and the arm's underside is at 16 mm; flat layers up to 10 mm.
    const TetMesh mesh = voxel_mesh(cantilever_voxels(), 2.);
    S4Params      params;
    params.planar_height  = 10.;
    const S4Result result = s4_deform(mesh, params);

    double moved = 0.;
    for (size_t v = 0; v < mesh.points.size(); ++v) {
        const double z = mesh.points[v].z();
        if (z <= 10.)
            CHECK_THAT((result.deformed[v] - mesh.points[v]).norm(), WithinAbs(0., 1e-12));
        else {
            CHECK(result.deformed[v].z() >= 10. + std::min(z - 10., params.bottom_threshold) - 1e-9);
            moved = std::max(moved, (result.deformed[v] - mesh.points[v]).norm());
        }
    }
    CHECK(moved > 1.);
    CHECK(result.passes.front().inverted == 0);
}

TEST_CASE("The rotation field follows its targets and stays within its bounds", "[S4]")
{
    // A chain of four cells; the ends carry targets 1 and -1, the middle cells follow.
    const std::vector<std::array<int, 2>> chain  = { { 0, 1 }, { 1, 2 }, { 2, 3 } };
    const double                          nan    = std::nan("");
    const std::vector<double>             target = { 1., nan, nan, -1. };

    SECTION("loose bounds give the unconstrained least squares solution") {
        const std::vector<double> x = s4_solve_rotation_field(4, chain, 1., target, std::vector<double>(4, 10.));
        // Stationarity: 2 x0 - x1 = 1, 2 x1 - x0 - x2 = 0, 2 x2 - x1 - x3 = 0, 2 x3 - x2 = -1.
        CHECK_THAT(x[0], WithinAbs(0.6, 1e-9));
        CHECK_THAT(x[1], WithinAbs(0.2, 1e-9));
        CHECK_THAT(x[2], WithinAbs(-0.2, 1e-9));
        CHECK_THAT(x[3], WithinAbs(-0.6, 1e-9));
    }

    SECTION("tight bounds are active and the free cells are optimal given them") {
        const std::vector<double> x = s4_solve_rotation_field(4, chain, 1., target, { 0.2, 10., 10., 10. });
        CHECK_THAT(x[0], WithinAbs(0.2, 1e-9));
        // With x0 fixed at 0.2: x1 = (x0 + x2) / 2, x2 = (x1 + x3) / 2, 2 x3 = x2 - 1.
        const double x3 = (0.2 - 3.) / 4.;
        CHECK_THAT(x[3], WithinAbs(x3, 1e-9));
        CHECK_THAT(x[1], WithinAbs(0.2 + (x3 - 0.2) / 3., 1e-9));
    }
}

TEST_CASE("Tetrahedralizing a closed surface fills exactly its volume", "[S4]")
{
    // A 20 mm cube and an L of three 10 mm cubes.
    const auto [voxels, size, expected] = GENERATE(table<std::vector<std::array<int, 3>>, double, double>(
        { { { { 0, 0, 0 } }, 20., 8000. }, { { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 } }, 10., 3000. } }));
    const Surface surface = voxel_surface(voxels, size);

    TetrahedralizeParams params;
    params.cell_size   = 4.;
    const TetMesh mesh = tetrahedralize(surface.vertices, surface.triangles, params);

    REQUIRE(mesh.tets.size() > 20);
    CHECK_THAT(volume(mesh), WithinRel(expected, 1e-6));
    for (const Eigen::Vector3d &p : mesh.points) {
        CHECK(p.minCoeff() >= -1e-9);
        CHECK(p.z() <= 2. * size + 1e-9);
    }
}

TEST_CASE("An open surface is rejected", "[S4]")
{
    Surface open = voxel_surface({ { 0, 0, 0 } }, 10.);
    open.triangles.pop_back();
    CHECK_THROWS(tetrahedralize(open.vertices, open.triangles));
}

TEST_CASE("Mapping through an identity deformation returns the point, upright and at nominal flow", "[S4]")
{
    const TetMesh  mesh = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper(mesh, mesh.points, Eigen::Vector2d::Zero());

    for (const Eigen::Vector3d &p : { Eigen::Vector3d(1.3, -0.7, 5.1), Eigen::Vector3d(12.2, 1.9, 17.3), Eigen::Vector3d(3.99, 0., 0.2) }) {
        const S4Mapper::Result r = mapper.map(p);
        CHECK(r.tier == S4Mapper::Tier::Inside);
        CHECK_THAT((r.point - p).norm(), WithinAbs(0., 1e-9));
        CHECK_THAT(r.tilt, WithinAbs(0., 1e-9));
        CHECK_THAT(r.flow, WithinAbs(1., 1e-9));
    }
}

TEST_CASE("Mapping undoes a rigid tilt and tilts the nozzle against it", "[S4]")
{
    // The whole part on the +X side of the axis, tilted outward by 20 degrees about the
    // tangential (+Y) axis through a point on its base: sliced layers are then leaning inward
    // in the real part, so the nozzle leans inward by the same angle.
    const TetMesh               mesh  = voxel_mesh(cantilever_voxels(), 2.);
    const double                angle = 20. * PI / 180.;
    const Eigen::Vector3d       pivot(10., 0., 0.);
    const Eigen::Matrix3d       rot = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    TetMesh                     real = mesh;
    for (Eigen::Vector3d &p : real.points)
        p += Eigen::Vector3d(10., 0., 0.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : real.points)
        deformed.push_back(pivot + rot * (p - pivot));
    const S4Mapper mapper(real, deformed, Eigen::Vector2d::Zero());

    const Eigen::Vector3d  inside_real(21., 0., 17.);
    const S4Mapper::Result r = mapper.map(pivot + rot * (inside_real - pivot));
    CHECK(r.tier == S4Mapper::Tier::Inside);
    CHECK_THAT((r.point - inside_real).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(r.tilt, WithinAbs(-angle, 1e-9));
    CHECK_THAT(r.flow, WithinAbs(1., 1e-9));
}

TEST_CASE("Squashing layers raises the flow by the volume ratio", "[S4]")
{
    const TetMesh                mesh = voxel_mesh(cantilever_voxels(), 2.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : mesh.points)
        deformed.emplace_back(p.x(), p.y(), 0.5 * p.z());
    const S4Mapper mapper(mesh, deformed, Eigen::Vector2d::Zero());

    const S4Mapper::Result r = mapper.map(Eigen::Vector3d(1., 0.3, 4.));
    CHECK_THAT(r.point.z(), WithinAbs(8., 1e-9));
    CHECK_THAT(r.flow, WithinAbs(2., 1e-9));
}

TEST_CASE("Points beside the mesh are extrapolated and far points are left alone", "[S4]")
{
    const TetMesh                mesh = voxel_mesh(cantilever_voxels(), 2.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : mesh.points)
        deformed.push_back(p + Eigen::Vector3d(0., 0., 1.)); // lifted by 1 mm
    const S4Mapper mapper(mesh, deformed, Eigen::Vector2d::Zero(), 2.);

    // Half a bead outside the post's side wall: the nearby displacement still applies.
    const S4Mapper::Result beside = mapper.map(Eigen::Vector3d(-0.2, 0., 6.));
    CHECK(beside.tier == S4Mapper::Tier::Nearest);
    CHECK_THAT((beside.point - Eigen::Vector3d(-0.2, 0., 5.)).norm(), WithinAbs(0., 1e-9));

    const S4Mapper::Result far = mapper.map(Eigen::Vector3d(-40., 0., 6.));
    CHECK(far.tier == S4Mapper::Tier::Outside);
    CHECK_THAT((far.point - Eigen::Vector3d(-40., 0., 6.)).norm(), WithinAbs(0., 1e-12));
}

TEST_CASE("G-code mapped through an identity deformation keeps its path and extrusion", "[S4]")
{
    const TetMesh  mesh = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper(mesh, mesh.points, Eigen::Vector2d::Zero());

    S4GCodeReport     report;
    const std::string out = transform(start_block + ";LAYER_CHANGE\n" + square(0.2) + end_block, mapper, &report);
    const Parsed      p   = parse_body(out.substr(out.find("MACHINE_START_GCODE_END")));

    CHECK(report.failed.empty());
    CHECK_THAT(p.e_total, WithinAbs(0.4, 1e-9));
    for (const Eigen::Vector3d &q : p.printing) {
        // Each 2 mm side is split into 0.6 mm segments; all of them stay on the square.
        const bool on_x = std::abs(q.x() - 1.) < 1e-9 || std::abs(q.x() - 3.) < 1e-9;
        const bool on_y = std::abs(q.y() - 1.) < 1e-9 || std::abs(q.y() + 1.) < 1e-9;
        CHECK((on_x || on_y));
        CHECK_THAT(q.z(), WithinAbs(0.2, 1e-9));
    }
    CHECK(p.printing.size() == 4 * 4);
    CHECK(out.find("B0") != std::string::npos); // upright nozzle
    // Machine blocks are copied untouched.
    CHECK(out.find("G28 ; home\n") != std::string::npos);
    CHECK(out.find("G1 X170 Y180 F3000 ; park\nM84\n") != std::string::npos);
}

TEST_CASE("Material follows the volume of squashed layers", "[S4]")
{
    const TetMesh                mesh = voxel_mesh(cantilever_voxels(), 2.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : mesh.points)
        deformed.emplace_back(p.x(), p.y(), 0.5 * p.z()); // sliced layers are twice as thick in the part
    const S4Mapper mapper(mesh, deformed, Eigen::Vector2d::Zero());

    const Parsed p = parse_body(transform(start_block + square(2.), mapper));
    CHECK_THAT(p.e_total, WithinAbs(0.8, 1e-6));
    for (const Eigen::Vector3d &q : p.printing)
        CHECK_THAT(q.z(), WithinAbs(4., 1e-9));
}

TEST_CASE("A travel through printed material is lifted over it and retracted", "[S4]")
{
    const TetMesh  mesh = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper(mesh, mesh.points, Eigen::Vector2d::Zero());

    // A wall printed up to z = 3, then a travel back across it at z = 0.6 (as on a layer that
    // curves down): the nozzle would plough through the wall.
    std::string g = start_block;
    for (double z : { 1., 2., 3. })
        g += "G1 X2 Y-1.8 Z" + std::to_string(z) + " F3000\nG1 X2 Y1.8 E0.2 F1200\n";
    g += "G1 X0.5 Y1.8 Z0.6 F6000\nG1 X0.5 Y-1.8\nG1 X3.5 Y-1.8\nG1 X3.5 Y1.8\nG1 X2.5 Y1.8 E0.1 F1200\n";

    S4GCodeReport     report;
    const std::string out = transform(g, mapper, &report);
    CHECK(report.collisions >= 1);
    CHECK(report.lifts >= 1);
    CHECK(report.retractions_added >= 1);
    CHECK(report.failed.empty());
    // Lifted to the wall top + clearance + hop.
    CHECK_THAT(parse_body(out).max_z, WithinAbs(3. + 0.15 + 0.5, 1e-6));
}

TEST_CASE("Absolute extrusion is refused", "[S4]")
{
    const TetMesh  mesh = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper(mesh, mesh.points, Eigen::Vector2d::Zero());
    CHECK_THROWS(transform("G28\nM82\n; MACHINE_START_GCODE_END\n" + square(0.2), mapper));
}

TEST_CASE("The nozzle stays vertical below the tilt threshold and never leans past the limit", "[S4]")
{
    // The rigid 20 degree tilt of the mapping test: every layer leans inward by 20 degrees about
    // +Y, which is a little less in the radial plane of points off the X axis.
    const TetMesh         mesh  = voxel_mesh(cantilever_voxels(), 2.);
    const double          angle = 20. * PI / 180.;
    const Eigen::Vector3d pivot(10., 0., 0.);
    const Eigen::Matrix3d rot = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    TetMesh               real = mesh;
    for (Eigen::Vector3d &p : real.points)
        p += Eigen::Vector3d(10., 0., 0.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : real.points)
        deformed.push_back(pivot + rot * (p - pivot));
    const S4Mapper mapper(real, deformed, Eigen::Vector2d::Zero());

    // A square in the post at real height 5, written in the sliced (deformed) space.
    std::ostringstream g;
    g << start_block;
    const Eigen::Vector3d corners[] = { { 11., -1., 5. }, { 13., -1., 5. }, { 13., 1., 5. }, { 11., 1., 5. }, { 11., -1., 5. } };
    for (size_t i = 0; i < 5; ++i) {
        const Eigen::Vector3d q = pivot + rot * (corners[i] - pivot);
        g << "G1 X" << q.x() << " Y" << q.y() << " Z" << q.z() << (i == 0 ? " F3000\n" : " E0.1 F1200\n");
    }
    auto tilts = [&](double threshold_deg, double max_deg, S4GCodeReport &report) {
        S4GCodeConfig cfg;
        cfg.emit_tilt      = true;
        cfg.tilt_threshold = threshold_deg * PI / 180.;
        cfg.max_tilt       = max_deg * PI / 180.;
        const std::string   out = transform(g.str(), mapper, &report, cfg);
        std::vector<double> b;
        std::istringstream  lines(out);
        std::string         line;
        while (std::getline(lines, line))
            if (const size_t at = line.find(" B"); line.rfind("G", 0) == 0 && at != std::string::npos)
                b.push_back(std::stod(line.substr(at + 2)));
        return b;
    };

    S4GCodeReport report;
    for (double b : tilts(25., 90., report)) // the layer leans less than the threshold
        CHECK_THAT(b, WithinAbs(0., 1e-9));
    for (double b : tilts(5., 90., report)) // twice the threshold is reached: the layer is followed
        CHECK_THAT(b, WithinAbs(-20., 0.1));
    for (double b : tilts(0., 10., report)) // cut back to the limit
        CHECK_THAT(b, WithinAbs(-10., 1e-3));
    CHECK(report.tilt_limited > 0);
}

TEST_CASE("The clearance check finds the toolhead hitting a taller print beside it", "[S4]")
{
    const TetMesh  mesh = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper(mesh, mesh.points, Eigen::Vector2d::Zero());
    S4GCodeConfig  cfg;
    cfg.clearance_check = true; // 40 degree nozzle cone 5 mm long, head radius 20 mm

    // A 2 mm square column printed layer by layer up to 15 mm clears itself.
    std::string column = start_block;
    for (int layer = 1; layer <= 30; ++layer)
        column += square(0.5 * layer);
    S4GCodeReport report;
    transform(column, mapper, &report, cfg);
    CHECK(report.head_collisions == 0);

    // Then a line on the bed 6 mm beside it: the head, 5 mm above the nozzle tip and 20 mm
    // wide, runs into the column.
    const std::string beside = column + "G1 X9 Y-4 Z0.5 F3000\nG1 X9 Y4 E0.3 F1200\n";
    transform(beside, mapper, &report, cfg);
    CHECK(report.head_collisions > 0);
    CHECK_FALSE(report.head_collision_samples.empty());

    // Far enough away, the head clears it.
    const std::string far = column + "G1 X40 Y-4 Z0.5 F3000\nG1 X40 Y4 E0.3 F1200\n";
    transform(far, mapper, &report, cfg);
    CHECK(report.head_collisions == 0);
}

TEST_CASE("The distance to a print surface is signed, negative inside", "[S4]")
{
    const Surface         cube = voxel_surface({ { 0, 0, 0 } }, 10.);
    const SurfaceDistance d(cube.vertices, cube.triangles);
    CHECK_THAT(d.signed_distance({ 5., 5., 13. }), WithinAbs(3., 1e-9));
    CHECK_THAT(d.signed_distance({ 13., 14., 5. }), WithinAbs(5., 1e-9)); // off an edge
    CHECK_THAT(d.signed_distance({ 5., 5., 4. }), WithinAbs(-4., 1e-9));
    CHECK_THAT(d.signed_distance({ 25., -5., 30. }), WithinAbs(std::sqrt(15. * 15. + 5. * 5. + 20. * 20.), 1e-9));
}

TEST_CASE("A print surface's faces on the bed close it but are not measured from", "[S4]")
{
    const Surface         cube = voxel_surface({ { 0, 0, 0 } }, 10.);
    const SurfaceDistance d(cube.vertices, cube.triangles, 0.);
    // Just above the bottom face: 0.5 mm from it, but measured to the nearest wall.
    CHECK_THAT(d.signed_distance({ 2., 5., 0.5 }), WithinAbs(-2., 1e-9));
    CHECK_THAT(d.signed_distance({ 5., 5., 13. }), WithinAbs(3., 1e-9));
}

TEST_CASE("Layers offset from a print surface start the gap away from it, all around", "[S4]")
{
    // A 10 mm cube core from -5 to 5 in X and Y; seen from (0, 0, 5), 5 mm below its top.
    std::vector<std::array<int, 3>> voxel { { 0, 0, 0 } };
    Surface                         cube = voxel_surface(voxel, 10.);
    for (Eigen::Vector3d &v : cube.vertices)
        v -= Eigen::Vector3d(5., 5., 0.);
    const SurfaceDistance d(cube.vertices, cube.triangles);
    const double          gap = 0.3, base = 10.2, d_out = 1.5;
    const Eigen::Vector3d centre(0., 0., 5.);
    const double          scale = 5. + gap;
    const std::vector<Eigen::Vector3d> points { { 0., 0., 10. + gap + d_out }, { 5. + gap + d_out, 0., 5. }, { 0., 0., 10. + 0.1 } };
    const std::vector<Eigen::Vector3d> out = deform_offset_from_above(points, d, centre, scale, base, gap);
    // Straight above: the centre of the layout, d_out above the first layer's bottom.
    CHECK_THAT(out[0].head<2>().norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(out[0].z(), WithinAbs(base + d_out, 1e-9));
    // Beside it, a quarter turn from straight up.
    CHECK_THAT(out[1].x(), WithinAbs(scale * PI / 2., 1e-9));
    CHECK_THAT(out[1].z(), WithinAbs(base + d_out, 1e-9));
    // Within the gap: below the print surface's layers, so never printed.
    CHECK(out[2].z() < base);
}

TEST_CASE("Unwrapping around the axis turns angle into length and height into width", "[S4]")
{
    std::vector<std::array<int, 3>> voxel { { 0, 0, 0 } };
    Surface                         cube = voxel_surface(voxel, 10.);
    for (Eigen::Vector3d &v : cube.vertices)
        v -= Eigen::Vector3d(5., 5., 0.);
    const SurfaceDistance d(cube.vertices, cube.triangles);
    const Eigen::Vector2d axis(0., 0.);
    // On +Y (a quarter turn), 2 mm off the face, 4 mm up; unwrapped from -90 degrees.
    const std::vector<Eigen::Vector3d> out = deform_offset_around_axis({ { 0., 7., 4. } }, d, axis, -PI / 2., 5.2, 10., 0.2, -1.);
    CHECK_THAT(out[0].x(), WithinAbs(5.2 * PI - 1., 1e-9));
    CHECK_THAT(out[0].y(), WithinAbs(4., 1e-9));
    CHECK_THAT(out[0].z(), WithinAbs(10. + 2. - 0.2, 1e-9));
}

TEST_CASE("Conical layers rise with the distance from the axis", "[S4]")
{
    const std::vector<Eigen::Vector3d> out = deform_cone({ { 3., 4., 1. } }, Eigen::Vector2d::Zero(), PI / 4., 0.);
    CHECK_THAT(out[0].z(), WithinAbs(1. + 5., 1e-9));
    CHECK_THAT(out[0].x(), WithinAbs(3., 1e-12));
}

TEST_CASE("Conical layers are flat on the side of the flat radius the part stands on", "[S4]")
{
    const std::vector<Eigen::Vector3d> points { { 0., 0., 1. }, { 3., 4., 1. }, { 6., 8., 1. } };
    // Descending away from the axis beyond 5 mm.
    std::vector<Eigen::Vector3d> out = deform_cone(points, Eigen::Vector2d::Zero(), PI / 4., 5.);
    CHECK_THAT(out[0].z(), WithinAbs(1., 1e-9));
    CHECK_THAT(out[1].z(), WithinAbs(1., 1e-9));
    CHECK_THAT(out[2].z(), WithinAbs(1. + 5., 1e-9));
    // Descending towards the axis within 5 mm.
    out = deform_cone(points, Eigen::Vector2d::Zero(), -PI / 4., 5.);
    CHECK_THAT(out[0].z(), WithinAbs(1. + 5., 1e-9));
    CHECK_THAT(out[1].z(), WithinAbs(1., 1e-9));
    CHECK_THAT(out[2].z(), WithinAbs(1., 1e-9));
}

TEST_CASE("The footprint radii span where the part stands on the bed", "[S4]")
{
    // A 10 x 10 x 12 mm cup upside down, standing on a ring around a 6 x 6 mm cavity.
    std::vector<std::array<int, 3>> ring, block;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 6; ++z) {
                block.push_back({ x, y, z });
                if (! (x >= 1 && x <= 3 && y >= 1 && y <= 3 && z <= 3))
                    ring.push_back({ x, y, z });
            }
    const Surface cup = voxel_surface(ring, 2.);
    auto [inner, outer] = footprint_radii(cup.vertices, cup.triangles, Eigen::Vector2d(5., 5.), 0.1);
    CHECK_THAT(inner, WithinAbs(3., 1e-9));
    CHECK_THAT(outer, WithinAbs(std::sqrt(50.), 1e-9));

    // Standing on the axis.
    const Surface box = voxel_surface(block, 2.);
    std::tie(inner, outer) = footprint_radii(box.vertices, box.triangles, Eigen::Vector2d(5., 5.), 0.1);
    CHECK_THAT(inner, WithinAbs(0., 1e-12));
    CHECK_THAT(outer, WithinAbs(std::sqrt(50.), 1e-9));

    // Off to the side of the axis.
    std::tie(inner, outer) = footprint_radii(box.vertices, box.triangles, Eigen::Vector2d(-3., 5.), 0.1);
    CHECK_THAT(inner, WithinAbs(3., 1e-9));
    CHECK_THAT(outer, WithinAbs(std::sqrt(13. * 13. + 25.), 1e-9));
}

namespace {

// Everything sliced 1 mm above where it prints.
S4Mapper raised_mapper(const TetMesh &mesh)
{
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : mesh.points)
        deformed.push_back(p + Eigen::Vector3d(0., 0., 1.));
    return S4Mapper(mesh, deformed, Eigen::Vector2d::Zero());
}

} // namespace

TEST_CASE("Each object is mapped through its own deformation, the rest printed as sliced", "[S4]")
{
    const TetMesh  mesh   = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper = raised_mapper(mesh);
    S4MapperSet    set;
    set.objects[7].mapper = &mapper;

    const std::string g = start_block + "; NONPLANAR_OBJECT 7\n" + square(2.4) + "; NONPLANAR_OBJECT_END\n" +
                          "G1 X30 Y0 Z2.4 F3000\nG1 X32 Y0 E0.1 F1200\n";
    std::istringstream  in(g);
    std::ostringstream  out;
    const S4GCodeReport report = s4_transform_gcode(in, out, set, S4GCodeConfig());
    const Parsed        p      = parse_body(out.str());
    REQUIRE(! p.printing.empty());
    for (const Eigen::Vector3d &pt : p.printing)
        CHECK_THAT(pt.z(), WithinAbs(pt.x() > 20. ? 2.4 : 1.4, 1e-6));
    CHECK(report.junctions >= 1);

    // Up to its print surface's top an object is printed as sliced.
    set.objects[7].identity_below_z = 2.5;
    std::istringstream in2(g);
    std::ostringstream out2;
    s4_transform_gcode(in2, out2, set, S4GCodeConfig());
    for (const Eigen::Vector3d &pt : parse_body(out2.str()).printing)
        CHECK_THAT(pt.z(), WithinAbs(2.4, 1e-6));
}

TEST_CASE("Extrusion outside an unwrapped object's window is left out", "[S4]")
{
    const TetMesh  mesh   = voxel_mesh(cantilever_voxels(), 2.);
    const S4Mapper mapper = raised_mapper(mesh);
    S4MapperSet    set;
    set.objects[3].mapper     = &mapper;
    set.objects[3].keep_min_x = 2.; // the square runs from X1 to X3
    std::istringstream  in(start_block + "; NONPLANAR_OBJECT 3\n" + square(2.4) + "; NONPLANAR_OBJECT_END\n");
    std::ostringstream  out;
    const S4GCodeReport report = s4_transform_gcode(in, out, set, S4GCodeConfig());
    CHECK(report.trimmed > 0);
    // Half of the first and third sides and all of the second remain: 0.05 + 0.1 + 0.05.
    CHECK_THAT(parse_body(out.str()).e_total, WithinAbs(0.2, 1e-4));
    for (const Eigen::Vector3d &pt : parse_body(out.str()).printing)
        CHECK(pt.x() >= 2. - 1e-6);
}

TEST_CASE("The clearance check finds a leaning toolhead reaching the bed", "[S4]")
{
    // Layers leaning 20 degrees (the rigid tilt of the mapping test); a 20 mm head 5 mm up the
    // nozzle dips 2.1 mm below the tip on the downhill side.
    const TetMesh         mesh  = voxel_mesh(cantilever_voxels(), 2.);
    const double          angle = 20. * PI / 180.;
    const Eigen::Vector3d pivot(10., 0., 0.);
    const Eigen::Matrix3d rot = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    TetMesh               real = mesh;
    for (Eigen::Vector3d &p : real.points)
        p += Eigen::Vector3d(10., 0., 0.);
    std::vector<Eigen::Vector3d> deformed;
    for (const Eigen::Vector3d &p : real.points)
        deformed.push_back(pivot + rot * (p - pivot));
    const S4Mapper mapper(real, deformed, Eigen::Vector2d::Zero());
    S4GCodeConfig  cfg;
    cfg.emit_tilt       = true;
    cfg.clearance_check = true;

    auto collisions_at = [&](double real_z) {
        std::ostringstream g;
        g << start_block;
        const Eigen::Vector3d corners[] = { { 11., -1., real_z }, { 13., -1., real_z }, { 13., 1., real_z }, { 11., 1., real_z } };
        for (size_t i = 0; i < 4; ++i) {
            const Eigen::Vector3d q = pivot + rot * (corners[i] - pivot);
            g << "G1 X" << q.x() << " Y" << q.y() << " Z" << q.z() << (i == 0 ? " F3000\n" : " E0.1 F1200\n");
        }
        S4GCodeReport report;
        transform(g.str(), mapper, &report, cfg);
        return report.head_collisions;
    };
    CHECK(collisions_at(1.) > 0);
    CHECK(collisions_at(12.) == 0);
}

TEST_CASE("A sphere core is fitted to the part's cavity", "[S4]")
{
    // A 10 mm cube with a 6 mm cubic cavity in its middle.
    std::vector<std::array<int, 3>> voxels;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 5; ++z)
                if (x == 0 || y == 0 || z == 0 || x == 4 || y == 4 || z == 4)
                    voxels.push_back({ x, y, z });
    const Surface         box = voxel_surface(voxels, 2.);
    const SurfaceDistance part(box.vertices, box.triangles);
    const FittedCore      fit = fit_sphere_core(part);
    CHECK_THAT((fit.base - Eigen::Vector3d(5., 5., 5.)).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(fit.radius, WithinAbs(3., 1e-9));
}

TEST_CASE("A cylinder core is fitted up to the roof of the part's cavity", "[S4]")
{
    // A 10 x 10 x 12 mm cup upside down: a 6 x 6 mm cavity open to the bed, 8 mm high.
    std::vector<std::array<int, 3>> voxels;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 6; ++z)
                if (! (x >= 1 && x <= 3 && y >= 1 && y <= 3 && z <= 3))
                    voxels.push_back({ x, y, z });
    const Surface         cup = voxel_surface(voxels, 2.);
    const SurfaceDistance part(cup.vertices, cup.triangles);
    const FittedCore      fit = fit_cylinder_core(cup.vertices, cup.triangles, part, Eigen::Vector2d(5., 5.));
    CHECK_THAT(fit.radius, WithinAbs(3., 1e-9));
    CHECK_THAT(fit.height, WithinAbs(8., 0.1));

    // A part covering the axis at the bed has nowhere to stand a core.
    CHECK_THROWS(fit_cylinder_core(cup.vertices, cup.triangles, part, Eigen::Vector2d(1., 1.)));

    // Nor has one standing over the axis on a bottom not quite level, 0.2 mm off the bed there.
    std::vector<std::array<int, 3>> block;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            block.push_back({ x, y, 0 });
    Surface slab = voxel_surface(block, 2.);
    for (Eigen::Vector3d &v : slab.vertices)
        v.z() += 0.04 * v.x();
    const SurfaceDistance tilted(slab.vertices, slab.triangles);
    CHECK_THROWS_WITH(fit_cylinder_core(slab.vertices, slab.triangles, tilted, Eigen::Vector2d(5., 5.)),
                      Catch::Matchers::ContainsSubstring("covers the rotation axis"));
}


namespace {

// A slab over [-10, 10]^2 up to 5 mm whose underside is the dome z = 3 (1 - (x^2 + y^2) / 200),
// reaching the bed at the corners: a lens with a concave underside.
Surface concave_slab(int n = 40)
{
    Surface s;
    auto    at = [n](int i, int j, int side) { return side * (n + 1) * (n + 1) + j * (n + 1) + i; };
    for (int side = 0; side < 2; ++side)
        for (int j = 0; j <= n; ++j)
            for (int i = 0; i <= n; ++i) {
                const double x = -10. + 20. * i / n, y = -10. + 20. * j / n;
                s.vertices.emplace_back(x, y, side == 0 ? 3. * (1. - (x * x + y * y) / 200.) : 5.);
            }
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            s.triangles.push_back({ at(i, j, 0), at(i + 1, j + 1, 0), at(i + 1, j, 0) });
            s.triangles.push_back({ at(i, j, 0), at(i, j + 1, 0), at(i + 1, j + 1, 0) });
            s.triangles.push_back({ at(i, j, 1), at(i + 1, j, 1), at(i + 1, j + 1, 1) });
            s.triangles.push_back({ at(i, j, 1), at(i + 1, j + 1, 1), at(i, j + 1, 1) });
        }
    std::vector<std::pair<int, int>> border; // counter-clockwise seen from above
    for (int k = 0; k < n; ++k)
        border.emplace_back(k, 0);
    for (int k = 0; k < n; ++k)
        border.emplace_back(n, k);
    for (int k = n; k > 0; --k)
        border.emplace_back(k, n);
    for (int k = n; k > 0; --k)
        border.emplace_back(0, k);
    for (size_t k = 0; k < border.size(); ++k) {
        const auto [i0, j0] = border[k];
        const auto [i1, j1] = border[(k + 1) % border.size()];
        s.triangles.push_back({ at(i0, j0, 0), at(i1, j1, 0), at(i1, j1, 1) });
        s.triangles.push_back({ at(i0, j0, 0), at(i1, j1, 1), at(i0, j0, 1) });
    }
    return s;
}

} // namespace

TEST_CASE("A post is a cylinder with an optional dome on top", "[S4]")
{
    Post post;
    post.centre = Eigen::Vector2d(1., 2.);
    post.radius = 10.;
    post.height = 5.;
    CHECK_THAT(post.top_at({ 1., 2. }), WithinAbs(5., 1e-12));
    CHECK_THAT(post.top_at({ 10.5, 2. }), WithinAbs(5., 1e-12));
    CHECK(post.top_at({ 11.5, 2. }) < -1e9);

    // A 2 mm dome over the 10 mm radius: a sphere of radius (100 + 4) / 4 = 26 mm.
    post.dome_height = 2.;
    CHECK_THAT(post.top_at({ 1., 2. }), WithinAbs(7., 1e-12));
    CHECK_THAT(post.top_at({ 11., 2. }), WithinAbs(5., 1e-9));
    CHECK_THAT(post.top_at({ 7., 2. }), WithinAbs(5. + 2. - 26. + std::sqrt(26. * 26. - 36.), 1e-12));

    // Its surface is closed and outward: the volume of the cylinder and the cap.
    std::vector<Eigen::Vector3d>    vertices;
    std::vector<std::array<int, 3>> triangles;
    post.mesh(vertices, triangles, 360);
    double volume = 0.;
    for (const std::array<int, 3> &t : triangles)
        volume += vertices[t[0]].dot(vertices[t[1]].cross(vertices[t[2]])) / 6.;
    const double cap = PI * 2. * 2. * (3. * 26. - 2.) / 3.;
    CHECK_THAT(volume, WithinRel(PI * 100. * 5. + cap, 0.01));
}

TEST_CASE("A part is lifted onto a post the surface gap above it", "[S4]")
{
    // A flat base stands the gap above a flat post.
    std::vector<std::array<int, 3>> voxels;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            voxels.push_back({ x, y, 0 });
    const Surface box = voxel_surface(voxels, 2.);
    const auto [middle, radius] = part_base(box.vertices, 0.1);
    CHECK_THAT((middle - Eigen::Vector2d(5., 5.)).norm(), WithinAbs(0., 1e-12));
    CHECK_THAT(radius, WithinAbs(std::sqrt(50.), 1e-12));
    Post post;
    post.centre = middle;
    post.radius = 4.;
    post.height = 6.;
    CHECK_THAT(lift_onto_post(box.vertices, box.triangles, post, 0.2), WithinAbs(6.2, 1e-9));

    // A concave underside nests on a dome: its middle, 3 mm up, the gap above the dome's top; a
    // flat post as high must lift it until its lower edge over the post clears it.
    const Surface lens = concave_slab();
    post        = Post();
    post.radius = 10.;
    post.dome_height = 3.;
    CHECK_THAT(lift_onto_post(lens.vertices, lens.triangles, post, 0.2), WithinAbs(0.2, 0.02));
    post.dome_height = 0.;
    post.height      = 3.;
    // At 10 mm out the underside is 1.5 mm up.
    CHECK_THAT(lift_onto_post(lens.vertices, lens.triangles, post, 0.2), WithinAbs(3.2 - 1.5, 0.05));
    // A post inside a cavity lifts nothing.
    post.radius = 2.;
    post.height = 1.;
    CHECK_THAT(lift_onto_post(lens.vertices, lens.triangles, post, 0.2), WithinAbs(0., 1e-12));
}
