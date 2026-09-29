#include "PaintedSurface.hpp"

#include "libslic3r/AABBMesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace Slic3r::NonPlanar {

namespace {

Vec3d vertex(const indexed_triangle_set &its, int i) { return its.vertices[i].cast<double>(); }

Vec3d face_normal(const indexed_triangle_set &its, const stl_triangle_vertex_indices &t)
{
    return (vertex(its, t[1]) - vertex(its, t[0])).cross(vertex(its, t[2]) - vertex(its, t[0]));
}

// The faces kept, their vertices shared, the rest dropped.
indexed_triangle_set keep_faces(const indexed_triangle_set &its, const std::vector<bool> &keep)
{
    indexed_triangle_set out;
    out.vertices = its.vertices;
    for (size_t i = 0; i < its.indices.size(); ++i)
        if (keep[i])
            out.indices.push_back(its.indices[i]);
    its_merge_vertices(out);
    its_compactify_vertices(out);
    return out;
}

} // namespace

indexed_triangle_set painted_surface_from_above(const indexed_triangle_set &painted, double gap, double floor_z)
{
    // The painted faces facing down, which the part is printed onto.
    std::vector<bool> down(painted.indices.size());
    for (size_t i = 0; i < painted.indices.size(); ++i) {
        const Vec3d n = face_normal(painted, painted.indices[i]);
        down[i]       = n.norm() > 0. && n.z() < -0.05 * n.norm();
    }
    indexed_triangle_set faces = keep_faces(painted, down);
    if (faces.indices.empty())
        throw std::runtime_error("Print surface: no face painted as print surface faces down. Paint the underside the part "
                                 "is printed onto.");
    // Where painted faces stand over one another, the print surface goes under the lowest only.
    std::vector<bool> lowest(faces.indices.size(), true);
    {
        const AABBMesh aabb(faces);
        for (size_t i = 0; i < faces.indices.size(); ++i) {
            const auto &t = faces.indices[i];
            const Vec3d c = (vertex(faces, t[0]) + vertex(faces, t[1]) + vertex(faces, t[2])) / 3.;
            lowest[i]     = ! aabb.query_ray_hit(c - Vec3d(0., 0., 1e-3), -Vec3d::UnitZ()).is_hit();
        }
    }
    faces = keep_faces(faces, lowest);

    // Each vertex moved off the part along its faces' normals, far enough to keep the gap from every
    // face around it, and no lower than the floor.
    const size_t       nv = faces.vertices.size();
    std::vector<Vec3d> normal(nv, Vec3d::Zero());
    std::vector<std::vector<Vec3d>> around(nv);
    for (const auto &t : faces.indices) {
        const Vec3d n = face_normal(faces, t);
        for (int k = 0; k < 3; ++k) {
            normal[t[k]] += n;
            around[t[k]].push_back(n.normalized());
        }
    }
    std::vector<Vec3d> top(nv);
    for (size_t v = 0; v < nv; ++v) {
        const Vec3d n = normal[v].normalized();
        double      c = 1.;
        for (const Vec3d &f : around[v])
            c = std::min(c, n.dot(f));
        top[v]    = vertex(faces, int(v)) + gap / std::max(c, 1. / 3.) * n;
        top[v].z() = std::max(top[v].z(), floor_z);
    }

    // The top reversed (it faces the part), walls down from its open edges, and the bottom on the floor.
    indexed_triangle_set out;
    for (const Vec3d &p : top)
        out.vertices.emplace_back(p.cast<float>());
    for (const Vec3d &p : top)
        out.vertices.emplace_back(float(p.x()), float(p.y()), float(floor_z));
    const int floor = int(nv);
    std::map<std::pair<int, int>, int> edges;
    for (const auto &t : faces.indices) {
        out.indices.emplace_back(t[0], t[2], t[1]);
        out.indices.emplace_back(t[0] + floor, t[1] + floor, t[2] + floor);
        for (int k = 0; k < 3; ++k)
            ++edges[{ t[k], t[(k + 1) % 3] }];
    }
    for (const auto &[e, count] : edges)
        if (edges.find({ e.second, e.first }) == edges.end()) {
            const int a = e.first, b = e.second;
            out.indices.emplace_back(a, b, b + floor);
            out.indices.emplace_back(a, b + floor, a + floor);
        }
    return out;
}

indexed_triangle_set painted_surface_around_axis(const indexed_triangle_set &painted, const Vec2d &axis, double gap, double floor_z)
{
    // The painted faces facing the axis, which the part is printed onto.
    std::vector<bool> inward(painted.indices.size());
    for (size_t i = 0; i < painted.indices.size(); ++i) {
        const auto &t = painted.indices[i];
        const Vec3d n = face_normal(painted, t);
        const Vec3d c = (vertex(painted, t[0]) + vertex(painted, t[1]) + vertex(painted, t[2])) / 3.;
        Vec3d       r(c.x() - axis.x(), c.y() - axis.y(), 0.);
        inward[i] = n.norm() > 0. && r.norm() > 1e-6 && n.dot(r.normalized()) < -0.05 * n.norm();
    }
    const indexed_triangle_set faces = keep_faces(painted, inward);
    if (faces.indices.empty())
        throw std::runtime_error("Print surface: no face painted as print surface faces the rotation axis. Paint the inside "
                                 "the part is printed onto.");
    double z_lo = std::numeric_limits<double>::max(), z_hi = std::numeric_limits<double>::lowest(), r_max = 0.;
    for (const stl_vertex &v : faces.vertices) {
        z_lo  = std::min(z_lo, double(v.z()));
        z_hi  = std::max(z_hi, double(v.z()));
        r_max = std::max(r_max, (v.head<2>().cast<double>() - axis).norm());
    }
    // A grid of rays out from the axis, about half a millimetre apart on the painted faces.
    const int rows = std::max(2, int(std::ceil((z_hi - z_lo) / 0.5)) + 1);
    const int cols = std::clamp(int(std::ceil(2. * PI * r_max / 0.5)), 64, 4096);
    const AABBMesh      aabb(faces);
    std::vector<double> radius(size_t(rows) * cols, -1.);
    for (int i = 0; i < cols; ++i) {
        const double a   = 2. * PI * i / cols;
        const Vec3d  dir(std::cos(a), std::sin(a), 0.);
        for (int j = 0; j < rows; ++j) {
            // Just inside the painted faces' ends, which the rays would graze.
            const double z   = z_lo + (z_hi - z_lo) * std::clamp(double(j) / (rows - 1), 1e-4, 1. - 1e-4);
            const auto   hit = aabb.query_ray_hit(Vec3d(axis.x(), axis.y(), z), dir);
            if (! hit.is_hit())
                continue;
            // The gap from the painted face along its normal, measured along the ray.
            Vec3d n = face_normal(faces, faces.indices[hit.face()]).normalized();
            radius[size_t(j) * cols + i] = std::max(hit.distance() - gap / std::max(std::abs(n.dot(dir)), 1. / 3.), 0.1);
        }
        // Where a ray missed, the nearest ray above or below that hit at this angle.
        int nearest = -1;
        for (int j = 0; j < rows; ++j)
            if (radius[size_t(j) * cols + i] >= 0.)
                nearest = j;
        if (nearest < 0)
            throw std::runtime_error("Print surface: the faces painted as print surface do not go all the way round the "
                                     "rotation axis. Paint the part's inside all round, or lay the layers out from above.");
        for (int j = 0; j < rows; ++j) {
            if (radius[size_t(j) * cols + i] >= 0.)
                continue;
            int best = -1;
            for (int k = 0; k < rows; ++k)
                if (radius[size_t(k) * cols + i] >= 0. && (best < 0 || std::abs(k - j) < std::abs(best - j)))
                    best = k;
            radius[size_t(j) * cols + i] = radius[size_t(best) * cols + i];
        }
    }

    // The grid's rings, from the lowest up, with a ring on the floor under the lowest, then the axis
    // at the floor and at the top.
    const bool           column = z_lo > floor_z + 1e-3;
    const int            rings  = rows + (column ? 1 : 0);
    indexed_triangle_set out;
    auto                 ring_vertex = [&](int ring, int i) { return ring * cols + i; };
    for (int ring = 0; ring < rings; ++ring) {
        const int    j = column ? std::max(ring - 1, 0) : ring;
        const double z = column && ring == 0 ? floor_z : z_lo + (z_hi - z_lo) * double(j) / (rows - 1);
        for (int i = 0; i < cols; ++i) {
            const double a = 2. * PI * i / cols, r = radius[size_t(j) * cols + i];
            out.vertices.emplace_back(float(axis.x() + r * std::cos(a)), float(axis.y() + r * std::sin(a)), float(z));
        }
    }
    const int bottom = int(out.vertices.size());
    out.vertices.emplace_back(float(axis.x()), float(axis.y()), float(column ? floor_z : z_lo));
    const int top = int(out.vertices.size());
    out.vertices.emplace_back(float(axis.x()), float(axis.y()), float(z_hi));
    for (int i = 0; i < cols; ++i) {
        const int n = (i + 1) % cols;
        for (int ring = 0; ring + 1 < rings; ++ring) {
            out.indices.emplace_back(ring_vertex(ring, i), ring_vertex(ring, n), ring_vertex(ring + 1, n));
            out.indices.emplace_back(ring_vertex(ring, i), ring_vertex(ring + 1, n), ring_vertex(ring + 1, i));
        }
        out.indices.emplace_back(top, ring_vertex(rings - 1, i), ring_vertex(rings - 1, n));
        out.indices.emplace_back(bottom, ring_vertex(0, n), ring_vertex(0, i));
    }
    return out;
}

} // namespace Slic3r::NonPlanar
