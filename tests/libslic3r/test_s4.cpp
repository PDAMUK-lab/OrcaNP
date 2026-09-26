#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/S4Deformation.hpp"
#include "libslic3r/NonPlanar/S4Mapping.hpp"
#include "libslic3r/NonPlanar/Tetrahedralize.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
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

    for (const Eigen::Vector3d p : { Eigen::Vector3d(1.3, -0.7, 5.1), Eigen::Vector3d(12.2, 1.9, 17.3), Eigen::Vector3d(3.99, 0., 0.2) }) {
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
