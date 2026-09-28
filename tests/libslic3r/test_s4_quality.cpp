// EXPERIMENT, for evaluation only (hidden): compares tetrahedral meshes and solvers for the S4
// deformation on one model. Remove with the experiment.
#include <catch2/catch_all.hpp>

#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/NonPlanar/LayerShapes.hpp"
#include "libslic3r/NonPlanar/S4Deformation.hpp"
#include "libslic3r/NonPlanar/S4Mapping.hpp"
#include "libslic3r/NonPlanar/Tetrahedralize.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <tbb/global_control.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r::NonPlanar;

// S4_QUALITY_MODEL=<stl> S4_QUALITY_VARIANTS=... libslic3r_tests "[S4Quality]"
TEST_CASE("S4 quality of tetrahedral meshes is measured", "[S4Quality][.]")
{
    const char *path = std::getenv("S4_QUALITY_MODEL");
    REQUIRE(path != nullptr);
    Slic3r::TriangleMesh tm;
    REQUIRE(tm.ReadSTLFile(path));
    const indexed_triangle_set     &its = tm.its;
    std::vector<Eigen::Vector3d>    V;
    std::vector<std::array<int, 3>> F;
    for (const auto &v : its.vertices)
        V.emplace_back(v.cast<double>());
    for (const auto &f : its.indices)
        F.push_back({ f[0], f[1], f[2] });
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e9), hi = -lo;
    for (const auto &v : V) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }

    // Surface samples (fidelity) and an interior grid 2 mm or more inside (layer tilt).
    std::vector<Eigen::Vector3d> surface;
    for (size_t k = 0; k < F.size(); k += std::max<size_t>(1, F.size() / 20000))
        surface.push_back((V[F[k][0]] + V[F[k][1]] + V[F[k][2]]) / 3.);
    const SurfaceDistance        part(V, F);
    std::vector<Eigen::Vector3d> interior;
    for (double z = lo.z() + 3.; z < hi.z() - 2.; z += 3.)
        for (double y = lo.y() + 2.; y < hi.y() - 2.; y += 3.)
            for (double x = lo.x() + 2.; x < hi.x() - 2.; x += 3.) {
                const Eigen::Vector3d p(x, y, z);
                if (part.inside(p) && part.distance(p) >= 2.)
                    interior.push_back(p);
            }
    S4Params sp;
    sp.max_overhang        = 30.;
    sp.neighbour_weight    = 20.;
    sp.rotation_multiplier = 2.;
    sp.max_rotation_near   = 45.;
    sp.max_rotation_far    = 20.;
    sp.passes              = 1;
    sp.planar_height       = 0.2;
    sp.axis                = 0.5 * (lo + hi).head<2>();

    // Share of the surface facing down steeper than 30 degrees below horizontal once deformed.
    auto unprintable = [&](auto &&map) {
        double bad = 0., total = 0.;
        for (const auto &f : F) {
            const Eigen::Vector3d a = V[f[0]], b = V[f[1]], c = V[f[2]];
            const double          area = 0.5 * (b - a).cross(c - a).norm();
            total += area;
            if (std::max({ a.z(), b.z(), c.z() }) < lo.z() + 0.3)
                continue; // on the bed
            const Eigen::Vector3d n = (map(b) - map(a)).cross(map(c) - map(a));
            if (n.norm() > 0. && n.z() / n.norm() < -0.5)
                bad += area;
        }
        return 100. * bad / total;
    };
    std::printf("model %s  size %.1f x %.1f x %.1f  auto cell %.2f  interior samples %zu\n", path, hi.x() - lo.x(), hi.y() - lo.y(),
                hi.z() - lo.z(), (hi - lo).maxCoeff() / 20., interior.size());
    std::printf("planar: unprintable %.2f %%\n", unprintable([](const Eigen::Vector3d &p) { return p; }));

    auto set_env = [](const char *name, const char *value) {
#ifdef _WIN32
        _putenv_s(name, value ? value : "");
#else
        if (value)
            setenv(name, value, 1);
        else
            unsetenv(name);
#endif
    };
    // S4_QUALITY_VARIANTS: "name|cell|facet|distance|weighted|mt|threads[|warm];..." (facet, distance: "-"
    // for the default; weighted, mt, warm: 0 or 1).
    struct Variant
    {
        std::string name, facet, distance;
        double      cell = 0.;
        bool        weighted = false, mt = false, warm = false;
        int         threads = 1;
    };
    std::vector<Variant> variants;
    {
        std::string spec = std::getenv("S4_QUALITY_VARIANTS") ? std::getenv("S4_QUALITY_VARIANTS") : "uniform 2 mm|2|-|-|0|0|1;uniform auto|0|-|-|0|0|1";
        std::stringstream all(spec);
        for (std::string item; std::getline(all, item, ';');) {
            std::stringstream one(item);
            std::vector<std::string> f;
            for (std::string field; std::getline(one, field, '|');)
                f.push_back(field);
            REQUIRE(f.size() >= 7);
            Variant v;
            v.name     = f[0];
            v.cell     = std::stod(f[1]);
            v.facet    = f[2];
            v.distance = f[3];
            v.weighted = f[4] == "1";
            v.mt       = f[5] == "1";
            v.threads  = std::stoi(f[6]);
            v.warm     = f.size() > 7 && f[7] == "1";
            variants.push_back(v);
        }
    }
    // The reference interior tilts (from the first variant of the first run) in S4_QUALITY_REF.
    const char         *ref_path = std::getenv("S4_QUALITY_REF");
    std::vector<double> ref_tilt;
    if (ref_path)
        if (FILE *f = std::fopen(ref_path, "rb")) {
            ref_tilt.resize(interior.size());
            const size_t got = std::fread(ref_tilt.data(), sizeof(double), ref_tilt.size(), f);
            std::fclose(f);
            if (got != interior.size())
                ref_tilt.clear();
        }
    // Deformed vertices by mesh, to compare solvers on the same mesh.
    std::map<std::string, std::vector<Eigen::Vector3d>> by_mesh;
    std::map<std::string, TetMesh>                      meshes;
    for (const Variant &var : variants) {
        set_env("S4_PROTO_FACET_SIZE", var.facet == "-" ? nullptr : var.facet.c_str());
        set_env("S4_PROTO_FACET_DISTANCE", var.distance == "-" ? nullptr : var.distance.c_str());
        set_env("S4_PROTO_WEIGHTED", var.weighted ? "1" : nullptr);
        set_env("S4_PROTO_MT", var.mt ? "1" : nullptr);
        set_env("S4_PROTO_WARM", var.warm ? "1" : nullptr);
        set_env("S4_PROTO_STATS", "1");
        tbb::global_control  threads(tbb::global_control::max_allowed_parallelism, size_t(var.threads));
        // Variants with the same mesh settings share the mesh, so solvers are compared on one mesh.
        const std::string key = std::to_string(var.cell) + " " + var.facet + " " + var.distance;
        TetrahedralizeParams tp;
        tp.cell_size    = var.cell;
        const auto t0   = std::chrono::steady_clock::now();
        auto       mit  = meshes.find(key);
        if (mit == meshes.end())
            mit = meshes.emplace(key, tetrahedralize(V, F, tp)).first;
        const TetMesh &mesh = mit->second;
        const auto     t1   = std::chrono::steady_clock::now();
        const S4Result res  = s4_deform(mesh, sp);
        const auto     t2   = std::chrono::steady_clock::now();

        // Fidelity: the original surface's distance to the tetrahedra's skin.
        indexed_triangle_set skin;
        for (const auto &q : mesh.points)
            skin.vertices.emplace_back(q.cast<float>());
        for (const auto &t : s4_boundary_triangles(mesh, mesh.points))
            skin.indices.emplace_back(t[0], t[1], t[2]);
        const Slic3r::AABBMesh skin_tree(skin);
        std::vector<double>    dev;
        for (const auto &q : surface)
            dev.push_back(std::sqrt(skin_tree.squared_distance(q)));
        std::sort(dev.begin(), dev.end());

        // The deformation forward: real points into the sliced space.
        const TetMesh  swapped { res.deformed, mesh.tets };
        const S4Mapper fwd(swapped, mesh.points, sp.axis, 10.);
        auto           map = [&](const Eigen::Vector3d &q) { return fwd.map(q).point; };
        const double   bad = unprintable(map);

        // Interior layer tilt: the gradient of the sliced height.
        std::vector<double> tilt;
        for (const auto &q : interior) {
            Eigen::Vector3d g;
            for (int i = 0; i < 3; ++i) {
                Eigen::Vector3d d = Eigen::Vector3d::Zero();
                d[i]              = 0.5;
                g[i]              = map(q + d).z() - map(q - d).z();
            }
            tilt.push_back(std::acos(std::clamp(g.z() / g.norm(), -1., 1.)) * 180. / PI);
        }
        double mean_tilt = 0., diff = 0.;
        for (size_t i = 0; i < tilt.size(); ++i) {
            mean_tilt += tilt[i];
            if (! ref_tilt.empty())
                diff += std::abs(tilt[i] - ref_tilt[i]);
        }
        mean_tilt /= double(std::max<size_t>(1, tilt.size()));
        diff /= double(std::max<size_t>(1, tilt.size()));
        if (ref_tilt.empty()) {
            ref_tilt = tilt;
            if (ref_path)
                if (FILE *f = std::fopen(ref_path, "wb")) {
                    std::fwrite(ref_tilt.data(), sizeof(double), ref_tilt.size(), f);
                    std::fclose(f);
                }
        }
        double same_mesh = -1.;
        if (auto it = by_mesh.find(key + " " + std::to_string(var.weighted)); it != by_mesh.end() && it->second.size() == res.deformed.size()) {
            same_mesh = 0.;
            for (size_t i = 0; i < res.deformed.size(); ++i)
                same_mesh = std::max(same_mesh, (res.deformed[i] - it->second[i]).norm());
        } else
            by_mesh[key + " " + std::to_string(var.weighted)] = res.deformed;
        std::printf("%-26s tets %7zu  mesh %6.1f s  deform %6.1f s  skin p50 %.3f p99 %.3f max %.3f  unprintable %5.2f %%  "
                    "interior tilt %5.2f  vs ref %5.2f deg  inverted %zu  vs same mesh %.2g mm\n",
                    var.name.c_str(), mesh.tets.size(), std::chrono::duration<double>(t1 - t0).count(),
                    std::chrono::duration<double>(t2 - t1).count(), dev[dev.size() / 2], dev[dev.size() * 99 / 100], dev.back(), bad,
                    mean_tilt, diff, res.passes.back().inverted, same_mesh);
        std::fflush(stdout);
    }
    set_env("S4_PROTO_FACET_SIZE", nullptr);
    set_env("S4_PROTO_FACET_DISTANCE", nullptr);
    set_env("S4_PROTO_WEIGHTED", nullptr);
    set_env("S4_PROTO_MT", nullptr);
    set_env("S4_PROTO_WARM", nullptr);
    set_env("S4_PROTO_STATS", nullptr);
}
