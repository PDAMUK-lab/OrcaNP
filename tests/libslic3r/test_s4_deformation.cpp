#include <catch2/catch_all.hpp>

#include "libslic3r/NonPlanar/S4Deformation.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <vector>

using namespace Slic3r::NonPlanar;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double PI = 3.14159265358979323846;

// Fills unit voxels (given by their minimum corner, in voxel units) with six tetrahedra each,
// all sharing the cube's main diagonal, so neighbouring voxels conform.
TetMesh voxel_mesh(const std::vector<std::array<int, 3>> &voxels, double size)
{
    TetMesh                                    mesh;
    std::map<std::tuple<int, int, int>, int>   index;
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

// A post standing on the bed with an arm cantilevered outward (+X, away from the rotation axis
// at the origin): the arm's underside is a horizontal overhang.
TetMesh cantilever()
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
    return voxel_mesh(voxels, 2.);
}

double signed_volume(const std::vector<Eigen::Vector3d> &pts, const std::array<int, 4> &t)
{
    return (pts[t[1]] - pts[t[0]]).cross(pts[t[2]] - pts[t[0]]).dot(pts[t[3]] - pts[t[0]]) / 6.;
}

// Steepest overhang (degrees from +Z) among the arm's underside faces, i.e. the boundary faces
// that start out at z = 16 facing down.
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
        const Eigen::Vector3d &a = mesh.points[f[0]], &b = mesh.points[f[1]], &c = mesh.points[f[2]];
        if (std::abs(a.z() - 16.) > 1e-9 || std::abs(b.z() - 16.) > 1e-9 || std::abs(c.z() - 16.) > 1e-9 || a.x() < 4. - 1e-9 ||
            b.x() < 4. - 1e-9 || c.x() < 4. - 1e-9)
            continue;
        Eigen::Vector3d n = (pts[f[1]] - pts[f[0]]).cross(pts[f[2]] - pts[f[0]]).normalized();
        if (n.dot(pts[e.second] - pts[f[0]]) > 0.)
            n = -n;
        steepest = std::max(steepest, std::acos(std::clamp(n.z(), -1., 1.)) * 180. / PI);
    }
    return steepest;
}

} // namespace

TEST_CASE("Vertices on the bed stay in place", "[S4Deformation]")
{
    const TetMesh  mesh   = cantilever();
    const S4Result result = s4_deform(mesh, S4Params());

    size_t on_bed = 0;
    for (size_t v = 0; v < mesh.points.size(); ++v)
        if (mesh.points[v].z() == 0.) {
            ++on_bed;
            CHECK_THAT((result.deformed[v] - mesh.points[v]).norm(), WithinAbs(0., 1e-12));
        }
    CHECK(on_bed == 3 * 3); // the post's 2x2 voxel footprint has 3x3 vertices
}

TEST_CASE("An outward overhang turns towards its support within the rotation limits", "[S4Deformation]")
{
    const TetMesh mesh = cantilever();
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

TEST_CASE("Deformation does not invert any tetrahedron", "[S4Deformation]")
{
    const TetMesh  mesh = cantilever();
    S4Params       params;
    params.passes = 2;
    const S4Result result = s4_deform(mesh, params);

    REQUIRE(result.passes.size() == 2);
    for (const auto &t : mesh.tets)
        CHECK(signed_volume(mesh.points, t) * signed_volume(result.deformed, t) > 0.);
}

TEST_CASE("The rotation field follows its targets and stays within its bounds", "[S4Deformation]")
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
