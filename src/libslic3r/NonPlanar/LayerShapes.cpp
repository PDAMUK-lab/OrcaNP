#include "LayerShapes.hpp"

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

double wrap_2pi(double a)
{
    a = std::fmod(a, 2. * PI);
    return a < 0. ? a + 2. * PI : a;
}

} // namespace

SurfaceDistance::SurfaceDistance(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles)
    : m_vertices(vertices), m_triangles(triangles)
{
    if (m_vertices.empty() || m_triangles.empty())
        throw std::runtime_error("Print surface: empty mesh");
    m_min = m_max = m_vertices.front();
    for (const Eigen::Vector3d &v : m_vertices) {
        m_min = m_min.cwiseMin(v);
        m_max = m_max.cwiseMax(v);
    }
    const Eigen::Vector3d span = m_max - m_min;
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

std::vector<Eigen::Vector3d> deform_cone(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector2d &axis, double angle)
{
    const double                 slope = std::tan(angle);
    std::vector<Eigen::Vector3d> out;
    out.reserve(points.size());
    for (const Eigen::Vector3d &p : points)
        out.emplace_back(p.x(), p.y(), p.z() + slope * (p.head<2>() - axis).norm());
    return out;
}

} // namespace NonPlanar
} // namespace Slic3r
