#include "S4Mapping.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

namespace Slic3r {
namespace NonPlanar {

namespace {

constexpr double inside_tolerance = 1e-9;

Eigen::Matrix3d edge_matrix(const std::vector<Eigen::Vector3d> &pts, const std::array<int, 4> &t)
{
    Eigen::Matrix3d m;
    m.col(0) = pts[t[1]] - pts[t[0]];
    m.col(1) = pts[t[2]] - pts[t[0]];
    m.col(2) = pts[t[3]] - pts[t[0]];
    return m;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
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
        return a + d1 / (d1 - d3) * ab;
    const Eigen::Vector3d cp = p - c;
    const double          d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0. && d5 <= d6)
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.)
        return a + d2 / (d2 - d6) * ac;
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0. && d4 - d3 >= 0. && d5 - d6 >= 0.)
        return b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b);
    const double denom = 1. / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

} // namespace

S4Mapper::S4Mapper(const TetMesh &undeformed, const std::vector<Eigen::Vector3d> &deformed, const Eigen::Vector2d &axis, double bbox_margin)
    : m_tets(undeformed.tets), m_deformed(deformed), m_axis(axis), m_margin(bbox_margin)
{
    if (deformed.size() != undeformed.points.size())
        throw std::runtime_error("S4: deformed and undeformed meshes differ in vertex count");
    const size_t nv = deformed.size(), nc = m_tets.size();
    m_displacement.resize(nv);
    for (size_t v = 0; v < nv; ++v)
        m_displacement[v] = deformed[v] - undeformed.points[v];

    // Per cell: inverse edge matrix for barycentric coordinates, volume ratio, and the layer
    // normal. The deformation gradient F = D1 D0^-1 carries real vectors into the sliced space,
    // so the sliced layers (normal +Z there) have the real normal F^T Z.
    m_inverse.resize(nc);
    m_flow.assign(nc, 1.);
    m_normal.assign(nv, Eigen::Vector3d::Zero());
    for (size_t c = 0; c < nc; ++c) {
        const Eigen::Matrix3d d1 = edge_matrix(m_deformed, m_tets[c]);
        const Eigen::Matrix3d d0 = edge_matrix(undeformed.points, m_tets[c]);
        const double          v1 = std::abs(d1.determinant()) / 6., v0 = std::abs(d0.determinant()) / 6.;
        if (v1 > 1e-12 && v0 > 1e-12) {
            m_inverse[c] = d1.inverse();
            m_flow[c]    = v0 / v1;
            const Eigen::Vector3d n = (d1 * d0.inverse()).transpose() * Eigen::Vector3d::UnitZ();
            for (int v : m_tets[c])
                m_normal[v] += v0 * n.normalized();
        } else
            m_inverse[c] = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    for (Eigen::Vector3d &n : m_normal)
        n = n.squaredNorm() > 0. ? Eigen::Vector3d(n.normalized()) : Eigen::Vector3d::UnitZ();

    m_min = m_max = m_deformed.front();
    for (const Eigen::Vector3d &p : m_deformed) {
        m_min = m_min.cwiseMin(p);
        m_max = m_max.cwiseMax(p);
    }

    // Grid buckets about two cells across, capped in count.
    const Eigen::Vector3d extent = (m_max - m_min).cwiseMax(1e-6);
    m_grid_step   = std::max(2. * std::cbrt(extent.prod() / double(std::max<size_t>(nc, 1))), extent.maxCoeff() / 200.);
    m_grid_origin = m_min;
    for (int k = 0; k < 3; ++k)
        m_grid_size[k] = int(std::floor(extent[k] / m_grid_step)) + 1;
    m_buckets.assign(size_t(m_grid_size[0]) * m_grid_size[1] * m_grid_size[2], {});
    for (size_t c = 0; c < nc; ++c) {
        Eigen::Vector3d lo = m_deformed[m_tets[c][0]], hi = lo;
        for (int v : m_tets[c]) {
            lo = lo.cwiseMin(m_deformed[v]);
            hi = hi.cwiseMax(m_deformed[v]);
        }
        int i0, j0, k0, i1, j1, k1;
        grid_cell(lo, i0, j0, k0);
        grid_cell(hi, i1, j1, k1);
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                for (int k = k0; k <= k1; ++k)
                    m_buckets[(size_t(i) * m_grid_size[1] + j) * m_grid_size[2] + k].push_back(int(c));
    }
}

void S4Mapper::grid_cell(const Eigen::Vector3d &p, int &i, int &j, int &k) const
{
    const Eigen::Vector3d g = (p - m_grid_origin) / m_grid_step;
    i = std::clamp(int(std::floor(g.x())), 0, m_grid_size[0] - 1);
    j = std::clamp(int(std::floor(g.y())), 0, m_grid_size[1] - 1);
    k = std::clamp(int(std::floor(g.z())), 0, m_grid_size[2] - 1);
}

bool S4Mapper::in_bbox(const Eigen::Vector3d &p) const
{
    return (p.array() >= (m_min.array() - m_margin)).all() && (p.array() <= (m_max.array() + m_margin)).all();
}

Eigen::Vector4d S4Mapper::barycentric(int cell, const Eigen::Vector3d &p) const
{
    const Eigen::Vector3d l = m_inverse[cell] * (p - m_deformed[m_tets[cell][0]]);
    return Eigen::Vector4d(1. - l.sum(), l.x(), l.y(), l.z());
}

double S4Mapper::distance_to_cell(int cell, const Eigen::Vector3d &p) const
{
    const Eigen::Vector4d w = barycentric(cell, p);
    if (w.allFinite() && w.minCoeff() >= 0.)
        return 0.;
    const std::array<int, 4> &t    = m_tets[cell];
    double                    best = std::numeric_limits<double>::infinity();
    for (int skip = 0; skip < 4; ++skip) {
        int f[3];
        for (int i = 0, j = 0; i < 4; ++i)
            if (i != skip)
                f[j++] = t[i];
        best = std::min(best, (closest_on_triangle(p, m_deformed[f[0]], m_deformed[f[1]], m_deformed[f[2]]) - p).norm());
    }
    return best;
}

S4Mapper::Result S4Mapper::map(const Eigen::Vector3d &p) const
{
    Result r;
    r.point = p;
    if (! in_bbox(p))
        return r;

    int i, j, k;
    grid_cell(p, i, j, k);
    int             cell = -1;
    Eigen::Vector4d w;
    for (int c : m_buckets[(size_t(i) * m_grid_size[1] + j) * m_grid_size[2] + k]) {
        w = barycentric(c, p);
        if (w.allFinite() && w.minCoeff() >= -inside_tolerance) {
            cell   = c;
            r.tier = Tier::Inside;
            break;
        }
    }

    if (cell < 0) {
        // Closest cell, searching outward shell by shell until no closer cell can exist.
        double best       = std::numeric_limits<double>::infinity();
        const int max_shell = std::max({ m_grid_size[0], m_grid_size[1], m_grid_size[2] });
        std::unordered_set<int> seen;
        for (int shell = 0; shell <= max_shell && best > (shell - 1) * m_grid_step; ++shell)
            for (int a = i - shell; a <= i + shell; ++a)
                for (int b = j - shell; b <= j + shell; ++b)
                    for (int c = k - shell; c <= k + shell; ++c) {
                        if (std::max({ std::abs(a - i), std::abs(b - j), std::abs(c - k) }) != shell || a < 0 || b < 0 || c < 0 ||
                            a >= m_grid_size[0] || b >= m_grid_size[1] || c >= m_grid_size[2])
                            continue;
                        for (int t : m_buckets[(size_t(a) * m_grid_size[1] + b) * m_grid_size[2] + c]) {
                            if (! seen.insert(t).second)
                                continue;
                            const double d = distance_to_cell(t, p);
                            if (d < best) {
                                best = d;
                                cell = t;
                            }
                        }
                    }
        if (cell >= 0) {
            w = barycentric(cell, p);
            if (! w.allFinite())
                cell = -1;
            else if (w.minCoeff() >= -inside_tolerance)
                r.tier = Tier::Inside;
            else {
                // Clamp onto the cell so the extrapolated displacement stays bounded.
                w = w.cwiseMax(0.).cwiseMin(1.);
                const double s = w.sum();
                w = s > 1e-12 ? Eigen::Vector4d(w / s) : Eigen::Vector4d::Constant(0.25);
                r.tier = Tier::Nearest;
            }
        }
    }

    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal       = Eigen::Vector3d::Zero();
    if (cell >= 0) {
        for (int v = 0; v < 4; ++v) {
            displacement += w[v] * m_displacement[m_tets[cell][v]];
            normal += w[v] * m_normal[m_tets[cell][v]];
        }
        r.cell = cell;
        r.flow = m_flow[cell];
    } else {
        // Inverse distance weighting over the nearest vertices.
        constexpr size_t    k_nearest = 8;
        std::vector<size_t> order(m_deformed.size());
        std::iota(order.begin(), order.end(), 0);
        const size_t kk = std::min(k_nearest, order.size());
        std::partial_sort(order.begin(), order.begin() + kk, order.end(),
                          [&](size_t a, size_t b) { return (m_deformed[a] - p).squaredNorm() < (m_deformed[b] - p).squaredNorm(); });
        double total = 0.;
        for (size_t n = 0; n < kk; ++n) {
            const double wt = 1. / std::max((m_deformed[order[n]] - p).squaredNorm(), 1e-18);
            displacement += wt * m_displacement[order[n]];
            normal += wt * m_normal[order[n]];
            total += wt;
        }
        displacement /= total;
        r.tier = Tier::Idw;
    }
    r.point = p - displacement;

    const Eigen::Vector2d radial = r.point.head<2>() - m_axis;
    if (normal.squaredNorm() > 0. && radial.norm() > 1e-9)
        r.tilt = std::atan2(normal.head<2>().dot(radial.normalized()), normal.z());
    return r;
}

} // namespace NonPlanar
} // namespace Slic3r
