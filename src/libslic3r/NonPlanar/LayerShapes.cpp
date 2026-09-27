#include "LayerShapes.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Slic3r {
namespace NonPlanar {

namespace {

constexpr double PI = 3.14159265358979323846;

// Closest point to p on triangle abc (Ericson, Real-Time Collision Detection, 5.1.5).
Eigen::Vector3d closest_on_triangle(const Eigen::Vector3d &p, const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
{
    const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
    const double          d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0. && d2 <= 0.)
        return a;
    const Eigen::Vector3d bp = p - b;
    const double          d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0. && d4 <= d3)
        return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.)
        return a + ab * (d1 / (d1 - d3));
    const Eigen::Vector3d cp = p - c;
    const double          d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0. && d5 <= d6)
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.)
        return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double denom = 1. / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

double squared_distance_to_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    const Eigen::Vector2d ab = b - a;
    const double          l2 = ab.squaredNorm();
    const double          t  = l2 > 0. ? std::clamp((p - a).dot(ab) / l2, 0., 1.) : 0.;
    return (a + ab * t - p).squaredNorm();
}

double wrap_2pi(double a)
{
    a = std::fmod(a, 2. * PI);
    return a < 0. ? a + 2. * PI : a;
}

} // namespace

SurfaceDistance::SurfaceDistance(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                                 double floor_z)
    : m_vertices(vertices), m_triangles(triangles), m_on_floor(triangles.size(), false)
{
    if (m_vertices.empty() || m_triangles.empty())
        throw std::runtime_error("Print surface: empty mesh");
    m_min = m_max = m_vertices.front();
    for (const Eigen::Vector3d &v : m_vertices) {
        m_min = m_min.cwiseMin(v);
        m_max = m_max.cwiseMax(v);
    }
    const Eigen::Vector3d span = m_max - m_min;
    if (m_max.z() <= floor_z + 1e-4)
        throw std::runtime_error("Print surface: flat on the bed");
    m_cell                     = std::max(span.maxCoeff() / 48., 0.25);
    for (int d = 0; d < 3; ++d)
        m_n[d] = std::max(int(std::ceil(span[d] / m_cell)), 1);
    m_grid.assign(size_t(m_n[0]) * m_n[1] * m_n[2], {});
    for (int t = 0; t < int(m_triangles.size()); ++t) {
        Eigen::Vector3d lo = m_vertices[m_triangles[t][0]], hi = lo;
        for (int k = 1; k < 3; ++k) {
            lo = lo.cwiseMin(m_vertices[m_triangles[t][k]]);
            hi = hi.cwiseMax(m_vertices[m_triangles[t][k]]);
        }
        m_on_floor[t] = hi.z() <= floor_z + 1e-4;
        int i0, j0, k0, i1, j1, k1;
        cell_of(lo, i0, j0, k0);
        cell_of(hi, i1, j1, k1);
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                for (int k = k0; k <= k1; ++k)
                    m_grid[(size_t(i) * m_n[1] + j) * m_n[2] + k].push_back(t);
    }
}

void SurfaceDistance::cell_of(const Eigen::Vector3d &p, int &i, int &j, int &k) const
{
    i = std::clamp(int((p.x() - m_min.x()) / m_cell), 0, m_n[0] - 1);
    j = std::clamp(int((p.y() - m_min.y()) / m_cell), 0, m_n[1] - 1);
    k = std::clamp(int((p.z() - m_min.z()) / m_cell), 0, m_n[2] - 1);
}

double SurfaceDistance::distance(const Eigen::Vector3d &p) const
{
    // Search shells of cells around the point's cell until nothing nearer can remain.
    int ci, cj, ck;
    cell_of(p, ci, cj, ck);
    double    best2   = std::numeric_limits<double>::infinity();
    const int max_ring = std::max({ m_n[0], m_n[1], m_n[2] });
    auto visit = [&](int i, int j, int k) {
        if (k < 0 || k >= m_n[2])
            return;
        for (int t : m_grid[(size_t(i) * m_n[1] + j) * m_n[2] + k]) {
            if (m_on_floor[t])
                continue;
            const std::array<int, 3> &tri = m_triangles[t];
            const Eigen::Vector3d     q   = closest_on_triangle(p, m_vertices[tri[0]], m_vertices[tri[1]], m_vertices[tri[2]]);
            best2                         = std::min(best2, (q - p).squaredNorm());
        }
    };
    for (int r = 0; r <= max_ring; ++r) {
        // Only the cells on the surface of the cube of radius r.
        for (int i = std::max(ci - r, 0); i <= std::min(ci + r, m_n[0] - 1); ++i)
            for (int j = std::max(cj - r, 0); j <= std::min(cj + r, m_n[1] - 1); ++j) {
                if (std::abs(i - ci) == r || std::abs(j - cj) == r)
                    for (int k = std::max(ck - r, 0); k <= std::min(ck + r, m_n[2] - 1); ++k)
                        visit(i, j, k);
                else {
                    visit(i, j, ck - r);
                    if (r > 0)
                        visit(i, j, ck + r);
                }
            }
        // Cells of the next shell are at least r cells away from anywhere in the point's cell.
        const double bound = r * m_cell;
        if (best2 <= bound * bound)
            break;
    }
    return std::sqrt(best2);
}

bool SurfaceDistance::inside(const Eigen::Vector3d &p_in) const
{
    // Parity of the crossings of a vertical ray upward, nudged off edges and vertices.
    const Eigen::Vector3d p = p_in + Eigen::Vector3d(1.3e-7, 2.9e-7, 0.);
    if (p.x() < m_min.x() || p.y() < m_min.y() || p.x() > m_max.x() || p.y() > m_max.y() || p.z() > m_max.z())
        return false;
    int ci, cj, ck;
    cell_of(p, ci, cj, ck);
    std::vector<int> candidates;
    for (int k = ck; k < m_n[2]; ++k) {
        const std::vector<int> &cell = m_grid[(size_t(ci) * m_n[1] + cj) * m_n[2] + k];
        candidates.insert(candidates.end(), cell.begin(), cell.end());
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    int crossings = 0;
    for (int t : candidates) {
        const Eigen::Vector3d &a = m_vertices[m_triangles[t][0]], &b = m_vertices[m_triangles[t][1]], &c = m_vertices[m_triangles[t][2]];
        // Barycentric coordinates of p in the triangle's XY projection.
        const double det = (b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y());
        if (std::abs(det) < 1e-18)
            continue;
        const double u = ((p.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (p.y() - a.y())) / det;
        const double v = ((b.x() - a.x()) * (p.y() - a.y()) - (p.x() - a.x()) * (b.y() - a.y())) / det;
        if (u < 0. || v < 0. || u + v > 1.)
            continue;
        const double z = a.z() + u * (b.z() - a.z()) + v * (c.z() - a.z());
        if (z > p.z())
            ++crossings;
    }
    return crossings % 2 == 1;
}

double SurfaceDistance::signed_distance(const Eigen::Vector3d &p) const
{
    const double d = distance(p);
    return inside(p) ? -d : d;
}

double Post::top_at(const Eigen::Vector2d &xy) const
{
    const double r = (xy - centre).norm();
    if (r > radius)
        return -std::numeric_limits<double>::infinity();
    if (dome_height <= 0.)
        return bottom + height;
    // A cap of a sphere through the rim, `dome_height` above it in the middle.
    const double sphere = (radius * radius + dome_height * dome_height) / (2. * dome_height);
    return bottom + height + dome_height - sphere + std::sqrt(sphere * sphere - r * r);
}

void Post::mesh(std::vector<Eigen::Vector3d> &vertices, std::vector<std::array<int, 3>> &triangles, int segments) const
{
    // A profile in (r, z) from the bottom's middle round the rim to the top's middle, revolved.
    std::vector<Eigen::Vector2d> profile { { 0., bottom }, { radius, bottom } };
    if (height > 0.)
        profile.emplace_back(radius, bottom + height);
    const int arcs = dome_height > 0. ? 24 : 0;
    for (int k = 1; k < arcs; ++k) {
        const double r = radius * (1. - double(k) / arcs);
        profile.emplace_back(r, top_at(centre + Eigen::Vector2d(r, 0.)));
    }
    profile.emplace_back(0., bottom + height + dome_height);
    vertices.clear();
    triangles.clear();
    std::vector<int> first;
    for (const Eigen::Vector2d &p : profile) {
        first.push_back(int(vertices.size()));
        const int n = p.x() > 0. ? segments : 1;
        for (int k = 0; k < n; ++k) {
            const double a = 2. * PI * k / segments;
            vertices.emplace_back(centre.x() + p.x() * std::cos(a), centre.y() + p.x() * std::sin(a), p.y());
        }
    }
    auto at = [&](size_t i, int k) { return profile[i].x() > 0. ? first[i] + k % segments : first[i]; };
    for (size_t i = 0; i + 1 < profile.size(); ++i)
        for (int k = 0; k < segments; ++k) {
            const int a = at(i, k), b = at(i, k + 1), c = at(i + 1, k), d = at(i + 1, k + 1);
            if (a != b)
                triangles.push_back({ a, b, c });
            if (c != d)
                triangles.push_back({ b, d, c });
        }
}

std::pair<Eigen::Vector2d, double> part_base(const std::vector<Eigen::Vector3d> &vertices, double tolerance)
{
    double floor_z = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &v : vertices)
        floor_z = std::min(floor_z, v.z());
    Eigen::AlignedBox2d box;
    for (const Eigen::Vector3d &v : vertices)
        if (v.z() <= floor_z + tolerance)
            box.extend(v.head<2>());
    const Eigen::Vector2d middle = box.center();
    double                radius = 0.;
    for (const Eigen::Vector3d &v : vertices)
        if (v.z() <= floor_z + tolerance)
            radius = std::max(radius, (v.head<2>() - middle).norm());
    return { middle, radius };
}

namespace {

// The part's lowest point over each cell of a grid over the post (infinity where the part is not
// over it), by rasterizing its faces; cell centres off the round coordinates models are drawn in.
struct Underside
{
    Eigen::Vector2d     origin;
    double              cell;
    int                 n;
    std::vector<double> lowest;

    Eigen::Vector2d centre(int i, int j) const { return origin + cell * Eigen::Vector2d(i + 0.5, j + 0.5); }
};

Underside underside(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles, const Post &post)
{
    const double          cell   = std::max(0.25, post.radius / 200.);
    const int             n      = int(std::ceil(2. * post.radius / cell)) + 1;
    const Eigen::Vector2d origin = post.centre - Eigen::Vector2d::Constant(post.radius) + cell * Eigen::Vector2d(0.0137, 0.0291);
    std::vector<double>   lowest(size_t(n) * n, std::numeric_limits<double>::infinity());
    for (const std::array<int, 3> &t : triangles) {
        const Eigen::Vector3d &a = vertices[t[0]], &b = vertices[t[1]], &c = vertices[t[2]];
        const double           area = (b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y());
        if (std::abs(area) < 1e-12)
            continue; // vertical
        auto range = [&](double lo, double hi, double o) {
            return std::make_pair(std::max(0, int(std::ceil((lo - o) / cell - 0.5))), std::min(n - 1, int(std::floor((hi - o) / cell - 0.5))));
        };
        const auto [i0, i1] = range(std::min({ a.x(), b.x(), c.x() }), std::max({ a.x(), b.x(), c.x() }), origin.x());
        const auto [j0, j1] = range(std::min({ a.y(), b.y(), c.y() }), std::max({ a.y(), b.y(), c.y() }), origin.y());
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const Eigen::Vector2d p  = origin + cell * Eigen::Vector2d(i + 0.5, j + 0.5);
                const double          w0 = ((b.x() - p.x()) * (c.y() - p.y()) - (c.x() - p.x()) * (b.y() - p.y())) / area;
                const double          w1 = ((c.x() - p.x()) * (a.y() - p.y()) - (a.x() - p.x()) * (c.y() - p.y())) / area;
                if (w0 >= 0. && w1 >= 0. && w0 + w1 <= 1.) {
                    double &z = lowest[size_t(j) * n + i];
                    z         = std::min(z, w0 * a.z() + w1 * b.z() + (1. - w0 - w1) * c.z());
                }
            }
    }
    return { origin, cell, n, std::move(lowest) };
}

double lift_over(const Underside &u, const Post &post, double gap)
{
    double lift = 0.;
    for (int j = 0; j < u.n; ++j)
        for (int i = 0; i < u.n; ++i)
            if (const double z = u.lowest[size_t(j) * u.n + i]; std::isfinite(z))
                lift = std::max(lift, post.top_at(u.centre(i, j)) + gap - z);
    return lift;
}

} // namespace

double lift_onto_post(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles, const Post &post,
                      double gap)
{
    return lift_over(underside(vertices, triangles, post), post, gap);
}

FittedCore fit_sphere_core(const SurfaceDistance &part)
{
    const Eigen::Vector3d lo = part.bbox_min(), hi = part.bbox_max();
    const Eigen::Vector3d centre(0.5 * (lo.x() + hi.x()), 0.5 * (lo.y() + hi.y()),
                                 std::max(lo.z(), hi.z() - 0.5 * std::max(hi.x() - lo.x(), hi.y() - lo.y())));
    // Just above the bottom, where a dome's centre lies on the bed.
    const Eigen::Vector3d probe = centre + Eigen::Vector3d(0., 0., 1e-3);
    if (part.inside(probe))
        throw std::runtime_error("Print surface: the part has no cavity at its centre to fit a sphere into");
    return { centre, part.distance(centre), 0. };
}

FittedCore fit_cylinder_core(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                             const SurfaceDistance &part, const Eigen::Vector2d &axis)
{
    const Eigen::Vector3d lo = part.bbox_min(), hi = part.bbox_max();
    // The cavity must reach down to the bed, where the core stands; its roof is where the axis
    // enters the part again.
    const double step = 0.05;
    double       roof = hi.z();
    for (double z = lo.z() + step; z < hi.z(); z += step)
        if (part.inside({ axis.x(), axis.y(), z })) {
            roof = z - step;
            break;
        }
    // A part standing over the axis on a bottom not quite level meets it just above the bed.
    if (roof - lo.z() <= 0.5)
        throw std::runtime_error("Print surface: the part covers the rotation axis at the bed; a cylinder core needs a cavity "
                                 "open to the bed around the axis");
    // Nearest wall: horizontal distance from the axis to the part below the roof.
    double r2 = std::numeric_limits<double>::infinity();
    for (const std::array<int, 3> &t : triangles) {
        const Eigen::Vector3d &a = vertices[t[0]], &b = vertices[t[1]], &c = vertices[t[2]];
        if (std::min({ a.z(), b.z(), c.z() }) >= roof)
            continue;
        r2 = std::min({ r2, squared_distance_to_segment(axis, a.head<2>(), b.head<2>()),
                        squared_distance_to_segment(axis, b.head<2>(), c.head<2>()),
                        squared_distance_to_segment(axis, c.head<2>(), a.head<2>()) });
    }
    if (! std::isfinite(r2) || r2 <= 0.)
        throw std::runtime_error("Print surface: the part has no cavity around the rotation axis to fit a cylinder into");
    return { Eigen::Vector3d(axis.x(), axis.y(), lo.z()), std::sqrt(r2), roof - lo.z() };
}

std::vector<Eigen::Vector3d> deform_offset_from_above(const std::vector<Eigen::Vector3d> &points, const SurfaceDistance &surface,
                                                      const Eigen::Vector3d &centre, double scale, double base_z, double gap)
{
    std::vector<Eigen::Vector3d> out;
    out.reserve(points.size());
    for (const Eigen::Vector3d &p : points) {
        const Eigen::Vector3d v   = p - centre;
        const double          h   = v.head<2>().norm();
        const double          psi = std::atan2(h, v.z()); // from straight up
        const double          az  = h > 1e-12 ? std::atan2(v.y(), v.x()) : 0.;
        out.emplace_back(centre.x() + scale * psi * std::cos(az), centre.y() + scale * psi * std::sin(az),
                         base_z + surface.signed_distance(p) - gap);
    }
    return out;
}

std::vector<Eigen::Vector3d> deform_offset_around_axis(const std::vector<Eigen::Vector3d> &points, const SurfaceDistance &surface,
                                                       const Eigen::Vector2d &axis, double from_angle, double scale, double base_z,
                                                       double gap, double shift)
{
    std::vector<Eigen::Vector3d> out;
    out.reserve(points.size());
    for (const Eigen::Vector3d &p : points) {
        const double theta = wrap_2pi(std::atan2(p.y() - axis.y(), p.x() - axis.x()) - from_angle);
        out.emplace_back(axis.x() + scale * theta + shift, axis.y() + p.z(), base_z + surface.signed_distance(p) - gap);
    }
    return out;
}

std::pair<double, double> footprint_radii(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                                          const Eigen::Vector2d &axis, double tolerance)
{
    double floor_z = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &v : vertices)
        floor_z = std::min(floor_z, v.z());
    auto   on_floor = [&](int i) { return vertices[i].z() <= floor_z + tolerance; };
    auto   cross    = [](const Eigen::Vector2d &u, const Eigen::Vector2d &v) { return u.x() * v.y() - u.y() * v.x(); };
    double inner2 = std::numeric_limits<double>::infinity(), outer2 = 0.;
    for (int i = 0; i < int(vertices.size()); ++i)
        if (on_floor(i)) {
            const double r2 = (vertices[i].head<2>() - axis).squaredNorm();
            inner2          = std::min(inner2, r2);
            outer2          = std::max(outer2, r2);
        }
    for (const std::array<int, 3> &t : triangles) {
        for (int j = 0; j < 3; ++j)
            if (on_floor(t[j]) && on_floor(t[(j + 1) % 3]))
                inner2 = std::min(inner2, squared_distance_to_segment(axis, vertices[t[j]].head<2>(), vertices[t[(j + 1) % 3]].head<2>()));
        if (on_floor(t[0]) && on_floor(t[1]) && on_floor(t[2])) {
            const Eigen::Vector2d a = vertices[t[0]].head<2>(), b = vertices[t[1]].head<2>(), c = vertices[t[2]].head<2>();
            const double          area = cross(b - a, c - a);
            if (area != 0. && cross(b - a, axis - a) * area >= 0. && cross(c - b, axis - b) * area >= 0. &&
                cross(a - c, axis - c) * area >= 0.)
                inner2 = 0.;
        }
    }
    return { std::sqrt(inner2), std::sqrt(outer2) };
}

std::vector<Eigen::Vector3d> deform_cone(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector2d &axis, double angle,
                                         double flat_radius)
{
    const double                 slope = std::tan(angle);
    std::vector<Eigen::Vector3d> out;
    out.reserve(points.size());
    for (const Eigen::Vector3d &p : points)
        out.emplace_back(p.x(), p.y(), p.z() + std::max(0., slope * ((p.head<2>() - axis).norm() - flat_radius)));
    return out;
}

} // namespace NonPlanar
} // namespace Slic3r
