#include "S4Deformation.hpp"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>

namespace Slic3r {
namespace NonPlanar {

namespace {

constexpr double PI  = 3.14159265358979323846;

// Conjugate gradients over threads, for the rotation field of a large mesh. The work is split in
// fixed chunks and summed in a fixed order, so the result is the same on any number of threads.
using RowMatrix        = Eigen::SparseMatrix<double, Eigen::RowMajor>;
constexpr size_t chunk = 4096;

template<class F> void par_for(size_t n, F &&f)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, n, chunk), [&f](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i < r.end(); ++i)
            f(i);
    });
}

template<class F> double par_sum(size_t n, F &&f)
{
    return tbb::parallel_deterministic_reduce(
        tbb::blocked_range<size_t>(0, n, chunk), 0.,
        [&f](const tbb::blocked_range<size_t> &r, double s) {
            for (size_t i = r.begin(); i < r.end(); ++i)
                s += f(i);
            return s;
        },
        std::plus<double>());
}

// y = H p on the free rows (state 0), 0 on the others.
void free_rows_product(const RowMatrix &H, const std::vector<int> &state, const Eigen::VectorXd &p, Eigen::VectorXd &y)
{
    par_for(size_t(H.rows()), [&](size_t i) {
        double s = 0.;
        if (state[i] == 0)
            for (RowMatrix::InnerIterator it(H, int(i)); it; ++it)
                s += it.value() * p[it.col()];
        y[i] = s;
    });
}

// The free cells' rows of H x = g, the other cells held at their x, by Jacobi-preconditioned
// conjugate gradients from the current x, to a residual 1e-10 of the right-hand side. False when
// that is not reached.
bool solve_free_cg(const RowMatrix &H, const Eigen::VectorXd &diag, const Eigen::VectorXd &g, const std::vector<int> &state,
                   Eigen::VectorXd &x)
{
    const size_t    n = size_t(x.size());
    Eigen::VectorXd held(n), r(n), z(n), p(n), q(n);
    par_for(n, [&](size_t i) { held[i] = state[i] == 0 ? 0. : x[i]; });
    free_rows_product(H, state, held, q);
    const double b_norm = std::sqrt(par_sum(n, [&](size_t i) { return state[i] == 0 ? (g[i] - q[i]) * (g[i] - q[i]) : 0.; }));
    free_rows_product(H, state, x, q);
    par_for(n, [&](size_t i) {
        r[i] = state[i] == 0 ? g[i] - q[i] : 0.;
        z[i] = r[i] / diag[i];
        p[i] = z[i];
    });
    double       rz  = par_sum(n, [&](size_t i) { return r[i] * z[i]; });
    const double tol = 1e-10 * std::max(b_norm, 1e-30);
    for (int k = 0; k < 20000; ++k) {
        if (std::sqrt(par_sum(n, [&](size_t i) { return r[i] * r[i]; })) <= tol)
            return true;
        free_rows_product(H, state, p, q);
        const double alpha = rz / par_sum(n, [&](size_t i) { return p[i] * q[i]; });
        par_for(n, [&](size_t i) {
            x[i] += alpha * p[i];
            r[i] -= alpha * q[i];
            z[i] = r[i] / diag[i];
        });
        const double rz_next = par_sum(n, [&](size_t i) { return r[i] * z[i]; });
        const double beta    = rz_next / rz;
        rz                   = rz_next;
        par_for(n, [&](size_t i) { p[i] = z[i] + beta * p[i]; });
    }
    return false;
}

// How one deformation solves its rotation fields: by direct factorization, or once one of those
// turned out large (when multi-threaded), by conjugate gradients.
struct FieldSolver
{
    bool   multithreading = false;
    size_t iterative_factor_size = 0;
    bool   iterative   = false;
    int    solves      = 0;
    size_t factor_size = 0; // nonzeros of the first direct factorization
};

const double     NaN = std::numeric_limits<double>::quiet_NaN();

double deg2rad(double a) { return a * PI / 180.; }

struct BoundaryFace
{
    std::array<int, 3> v;
    int                cell;
    int                opposite; // the cell's fourth vertex
};

struct Topology
{
    std::vector<std::array<int, 2>> face_pairs;     // cells sharing a face
    std::vector<BoundaryFace>       boundary;       // faces on the surface
    std::vector<std::vector<int>>   point_neighbours; // cells sharing at least a vertex
    std::vector<std::vector<int>>   edge_neighbours;  // cells sharing at least an edge
};

Topology build_topology(const TetMesh &mesh)
{
    const size_t n = mesh.tets.size();
    Topology     topo;

    std::map<std::array<int, 3>, std::array<int, 3>> faces; // sorted face -> {cell, opposite, other cell}
    for (size_t c = 0; c < n; ++c) {
        const std::array<int, 4> &t = mesh.tets[c];
        for (int k = 0; k < 4; ++k) {
            std::array<int, 3> f;
            for (int i = 0, j = 0; i < 4; ++i)
                if (i != k)
                    f[j++] = t[i];
            std::sort(f.begin(), f.end());
            auto it = faces.find(f);
            if (it == faces.end())
                faces.emplace(f, std::array<int, 3>{ int(c), t[k], -1 });
            else
                it->second[2] = int(c);
        }
    }
    for (const auto &[f, v] : faces) {
        if (v[2] < 0)
            topo.boundary.push_back({ f, v[0], v[1] });
        else
            topo.face_pairs.push_back({ std::min(v[0], v[2]), std::max(v[0], v[2]) });
    }
    std::sort(topo.face_pairs.begin(), topo.face_pairs.end());

    std::vector<std::vector<int>> vertex_cells(mesh.points.size());
    for (size_t c = 0; c < n; ++c)
        for (int v : mesh.tets[c])
            vertex_cells[v].push_back(int(c));
    topo.point_neighbours.resize(n);
    topo.edge_neighbours.resize(n);
    std::vector<int> shared(n, 0);
    std::vector<int> touched;
    for (size_t c = 0; c < n; ++c) {
        touched.clear();
        for (int v : mesh.tets[c])
            for (int o : vertex_cells[v])
                if (o != int(c) && shared[o]++ == 0)
                    touched.push_back(o);
        std::sort(touched.begin(), touched.end());
        for (int o : touched) {
            topo.point_neighbours[c].push_back(o);
            if (shared[o] >= 2)
                topo.edge_neighbours[c].push_back(o);
            shared[o] = 0;
        }
    }
    return topo;
}

struct Attributes
{
    std::vector<Eigen::Vector3d> center;
    std::vector<char>            bottom;
    std::vector<char>            in_air;
    std::vector<double>          overhang;
    std::vector<double>          distance; // to the nearest bottom cell, all reachable cells
    std::vector<int>             nearest_bottom;
};

Attributes compute_attributes(const std::vector<Eigen::Vector3d> &pts, const TetMesh &mesh, const Topology &topo, const S4Params &params)
{
    const size_t n = mesh.tets.size();
    Attributes   a;
    a.center.resize(n);
    for (size_t c = 0; c < n; ++c) {
        const std::array<int, 4> &t = mesh.tets[c];
        a.center[c] = 0.25 * (pts[t[0]] + pts[t[1]] + pts[t[2]] + pts[t[3]]);
    }

    // Per boundary cell: the most downward outward face normal and the lowest face centre.
    std::vector<Eigen::Vector3d> normal(n, Eigen::Vector3d::Constant(NaN));
    std::vector<double>          face_z(n, NaN);
    for (const BoundaryFace &f : topo.boundary) {
        const Eigen::Vector3d &p0 = pts[f.v[0]], &p1 = pts[f.v[1]], &p2 = pts[f.v[2]];
        Eigen::Vector3d        nrm = (p1 - p0).cross(p2 - p0);
        const double           len = nrm.norm();
        if (len <= 0.)
            continue;
        nrm /= len;
        if (nrm.dot(pts[f.opposite] - p0) > 0.)
            nrm = -nrm;
        const double z = (p0.z() + p1.z() + p2.z()) / 3.;
        if (std::isnan(normal[f.cell].z()) || nrm.z() < normal[f.cell].z())
            normal[f.cell] = nrm;
        if (std::isnan(face_z[f.cell]) || z < face_z[f.cell])
            face_z[f.cell] = z;
    }
    double min_face_z = std::numeric_limits<double>::infinity();
    for (double z : face_z)
        if (! std::isnan(z))
            min_face_z = std::min(min_face_z, z);
    if (! std::isfinite(min_face_z))
        throw std::runtime_error("S4: the tetrahedral mesh has no boundary faces");
    // Cells of the planar base are the "bed" of the part above it.
    double min_z = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &p : pts)
        min_z = std::min(min_z, p.z());
    const double base_z = min_z + params.planar_height;

    a.bottom.assign(n, 0);
    a.overhang.assign(n, NaN);
    for (size_t c = 0; c < n; ++c) {
        if (params.planar_height > 0. && a.center[c].z() <= base_z) {
            a.bottom[c] = 1;
            continue;
        }
        if (std::isnan(face_z[c]))
            continue;
        a.bottom[c] = face_z[c] < min_face_z + params.bottom_threshold;
        if (! a.bottom[c])
            a.overhang[c] = std::acos(std::clamp(normal[c].z(), -1., 1.));
    }

    // Multi-source Dijkstra from the bottom cells over vertex-sharing neighbours, weighted by
    // centre distance. Along the way, track the highest cell on each cell's path to the bed.
    a.distance.assign(n, std::numeric_limits<double>::infinity());
    a.nearest_bottom.assign(n, -1);
    std::vector<double> path_top(n, -std::numeric_limits<double>::infinity());
    std::vector<int>    parent(n, -1);
    std::vector<char>   done(n, 0);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    for (size_t c = 0; c < n; ++c)
        if (a.bottom[c]) {
            a.distance[c]       = 0.;
            a.nearest_bottom[c] = int(c);
            queue.push({ 0., int(c) });
        }
    a.in_air.assign(n, 0);
    while (! queue.empty()) {
        const auto [d, c] = queue.top();
        queue.pop();
        if (done[c])
            continue;
        done[c]     = 1;
        path_top[c] = parent[c] < 0 ? a.center[c].z() : std::max(a.center[c].z(), path_top[parent[c]]);
        a.in_air[c] = parent[c] >= 0 && path_top[c] > a.center[c].z() + params.in_air_threshold;
        for (int o : topo.point_neighbours[c]) {
            const double nd = d + (a.center[c] - a.center[o]).norm();
            if (nd < a.distance[o]) {
                a.distance[o]       = nd;
                a.nearest_bottom[o] = a.nearest_bottom[c];
                parent[o]           = c;
                queue.push({ nd, o });
            }
        }
    }
    return a;
}

// Radial unit vector of a cell centre around the printer axis (NaN on the axis).
Eigen::Vector2d radial(const Eigen::Vector3d &center, const Eigen::Vector2d &axis)
{
    const Eigen::Vector2d r = center.head<2>() - axis;
    return r / r.norm();
}

// Direction each overhanging cell should turn: towards the bed, measured along the radial
// direction, as the downhill slope of the path-length-to-bed field (-1 .. 1).
std::vector<double> rotation_direction(const Attributes &a, const Topology &topo, const S4Params &params, std::vector<double> &distance)
{
    const size_t n         = a.center.size();
    const double threshold = deg2rad(90. + params.max_overhang);
    distance.assign(n, NaN);
    for (size_t c = 0; c < n; ++c)
        if (! a.bottom[c] && a.overhang[c] > threshold && std::isfinite(a.distance[c]))
            distance[c] = a.distance[c];

    std::vector<double> dir(n, 0.);
    std::vector<int>    local;
    for (size_t c = 0; c < n; ++c) {
        if (std::isnan(distance[c]))
            continue;
        local.clear();
        for (int o : topo.edge_neighbours[c])
            if (! std::isnan(distance[o]))
                local.push_back(o);
        local.push_back(int(c));
        const Eigen::Vector2d rhat = radial(a.center[c], params.axis);
        double                v;
        if (local.size() < 3) {
            // Too few overhanging neighbours for a slope: roll towards the nearest bed cell.
            const Eigen::Vector2d to_bed = (a.center[a.nearest_bottom[c]].head<2>() - a.center[c].head<2>()).normalized();
            const double          dot    = rhat.dot(to_bed);
            v = std::isnan(dot) || dot == 0. ? 0. : (dot > 0. ? 1. : -1.);
        } else {
            // Least-squares plane through (x, y, distance); its upward normal's radial
            // component is minus the radial slope of the distance.
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (int o : local)
                centroid += Eigen::Vector3d(a.center[o].x(), a.center[o].y(), distance[o]);
            centroid /= double(local.size());
            Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
            for (int o : local) {
                const Eigen::Vector3d p = Eigen::Vector3d(a.center[o].x(), a.center[o].y(), distance[o]) - centroid;
                scatter += p * p.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
            Eigen::Vector3d                                nrm = eig.eigenvectors().col(0);
            if (nrm.z() < 0.)
                nrm = -nrm;
            v = rhat.dot(nrm.head<2>());
            if (std::isnan(v)) {
                double sum = 0.;
                for (int o : local)
                    sum += dir[o];
                v = sum / double(local.size());
            }
        }
        dir[c] = v;
    }

    // Average over the two-ring neighbourhood, among overhanging cells only.
    std::vector<char> active(n);
    for (size_t c = 0; c < n; ++c)
        active[c] = dir[c] != 0.;
    std::vector<std::vector<int>> rings(n);
    for (size_t c = 0; c < n; ++c) {
        if (! active[c])
            continue;
        std::vector<int> &ring = rings[c];
        for (int o : topo.point_neighbours[c]) {
            ring.push_back(o);
            ring.insert(ring.end(), topo.point_neighbours[o].begin(), topo.point_neighbours[o].end());
        }
        std::sort(ring.begin(), ring.end());
        ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
        ring.erase(std::remove_if(ring.begin(), ring.end(), [&active](int o) { return ! active[o]; }), ring.end());
    }
    std::vector<double> next(n, 0.);
    for (int it = 0; it < params.smoothing_iterations; ++it) {
        for (size_t c = 0; c < n; ++c) {
            if (! active[c] || rings[c].empty()) {
                next[c] = dir[c];
                continue;
            }
            double sum = 0.;
            for (int o : rings[c])
                sum += dir[o];
            next[c] = sum / double(rings[c].size());
        }
        dir.swap(next);
    }
    if (! params.zero_initial_rotation)
        for (double &v : dir)
            if (v == 0.)
                v = NaN;
    return dir;
}

std::vector<double> rotation_limits(const std::vector<double> &distance, const S4Params &params)
{
    const double near = deg2rad(params.max_rotation_near), far = deg2rad(params.max_rotation_far);
    double       lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (double d : distance)
        if (! std::isnan(d)) {
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
    std::vector<double> limit(distance.size(), near);
    const double        span = std::max(hi - lo, 1e-12);
    for (size_t c = 0; c < distance.size(); ++c)
        if (! std::isnan(distance[c]))
            limit[c] = near - (near - far) * std::pow((distance[c] - lo) / span, params.max_rotation_falloff);
    return limit;
}

// Per-cell rotation matrices: angle about the horizontal tangential axis.
std::vector<Eigen::Matrix3d> rotation_matrices(const std::vector<Eigen::Vector3d> &center, const std::vector<double> &rotation,
                                               const Eigen::Vector2d &axis)
{
    std::vector<Eigen::Matrix3d> out(center.size());
    for (size_t c = 0; c < center.size(); ++c) {
        const Eigen::Vector2d r = center[c].head<2>() - axis;
        Eigen::Vector3d       t(-r.y(), r.x(), 0.);
        const double          len = t.norm();
        t                         = len > 0. ? Eigen::Vector3d(t / len) : Eigen::Vector3d::UnitX();
        out[c]                    = Eigen::AngleAxisd(rotation[c], t).toRotationMatrix();
    }
    return out;
}

// One coordinate of the vertex positions whose cells best match their rotated shapes (least
// squares on the centred cell vertices). Vertices with `fixed` set hold `value`.
Eigen::VectorXd solve_axis(const std::vector<Eigen::Vector3d> &pts, const TetMesh &mesh, const std::vector<Eigen::Matrix3d> &rot, int axis,
                           const std::vector<char> &fixed, const std::vector<double> &value, const std::vector<double> *cell_w = nullptr)
{
    const size_t     n = pts.size();
    std::vector<int> index(n, -1);
    int              num_free = 0;
    for (size_t v = 0; v < n; ++v)
        if (! fixed[v])
            index[v] = num_free++;

    // Normal equations: sum over cells of S^T N S x = S^T T, with N = I - 1/4 the centring
    // matrix (symmetric, idempotent) and T the rotated centred cell.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(mesh.tets.size() * 16);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(num_free);
    for (size_t c = 0; c < mesh.tets.size(); ++c) {
        const std::array<int, 4> &t    = mesh.tets[c];
        const Eigen::Vector3d     mean = 0.25 * (pts[t[0]] + pts[t[1]] + pts[t[2]] + pts[t[3]]);
        const double              wc   = cell_w ? (*cell_w)[c] : 1.;
        for (int i = 0; i < 4; ++i) {
            const int row = index[t[i]];
            if (row < 0)
                continue;
            rhs[row] += wc * rot[c].row(axis).dot(pts[t[i]] - mean);
            for (int j = 0; j < 4; ++j) {
                const double w   = wc * ((i == j ? 1. : 0.) - 0.25);
                const int    col = index[t[j]];
                if (col >= 0)
                    triplets.emplace_back(row, col, w);
                else
                    rhs[row] -= w * value[t[j]];
            }
        }
    }
    // A vanishing regularisation keeps parts that do not touch the bed (and would float
    // freely) where they are instead of making the system singular.
    const double eps = 1e-9;
    for (size_t v = 0; v < n; ++v)
        if (index[v] >= 0) {
            triplets.emplace_back(index[v], index[v], eps);
            rhs[index[v]] += eps * pts[v][axis];
        }
    Eigen::SparseMatrix<double> L(num_free, num_free);
    L.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(L);
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("S4: deformation system could not be factorized");
    const Eigen::VectorXd x = solver.solve(rhs);

    Eigen::VectorXd out(n);
    for (size_t v = 0; v < n; ++v)
        out[v] = index[v] >= 0 ? x[index[v]] : value[v];
    return out;
}

// Deformed vertex positions: `pinned` vertices stay put, and no vertex may end up below its
// `z_floor`. The axes are independent, so the floor is an active set on the Z solve alone.
std::vector<Eigen::Vector3d> solve_deformation(const std::vector<Eigen::Vector3d> &pts, const TetMesh &mesh,
                                               const std::vector<Eigen::Matrix3d> &rot, const std::vector<char> &pinned,
                                               const std::vector<double> &z_floor, bool multithreading,
                                               const std::vector<double> *cell_w = nullptr)
{
    const size_t                 n = pts.size();
    std::vector<Eigen::Vector3d> out(n);
    auto                         solve_one = [&](int axis) {
        std::vector<double> value(n);
        std::vector<char> fixed = pinned;
        for (size_t v = 0; v < n; ++v)
            value[v] = pts[v][axis];
        Eigen::VectorXd x;
        for (int round = 0; round < 100; ++round) {
            x = solve_axis(pts, mesh, rot, axis, fixed, value, cell_w);
            if (axis != 2)
                break;
            bool violated = false;
            for (size_t v = 0; v < n; ++v)
                if (! fixed[v] && x[v] < z_floor[v] - 1e-9) {
                    fixed[v] = 1;
                    value[v] = z_floor[v];
                    violated = true;
                }
            if (! violated)
                break;
        }
        for (size_t v = 0; v < n; ++v)
            out[v][axis] = x[v];
    };
    if (multithreading)
        tbb::parallel_for(0, 3, solve_one);
    else
        for (int axis = 0; axis < 3; ++axis)
            solve_one(axis);
    return out;
}

double signed_volume(const std::vector<Eigen::Vector3d> &pts, const std::array<int, 4> &t)
{
    return (pts[t[1]] - pts[t[0]]).cross(pts[t[2]] - pts[t[0]]).dot(pts[t[3]] - pts[t[0]]);
}

std::vector<double> solve_rotation_field(size_t n, const std::vector<std::array<int, 2>> &pairs, double weight,
                                         const std::vector<double> &target, const std::vector<double> &limit,
                                         const std::vector<double> *pair_w, const std::vector<double> *target_w,
                                         FieldSolver &solver, const std::vector<double> *initial);

} // namespace

std::vector<double> s4_solve_rotation_field(size_t n, const std::vector<std::array<int, 2>> &pairs, double weight,
                                            const std::vector<double> &target, const std::vector<double> &limit)
{
    FieldSolver solver;
    return solve_rotation_field(n, pairs, weight, target, limit, nullptr, nullptr, solver, nullptr);
}

namespace {

std::vector<double> solve_rotation_field(size_t n, const std::vector<std::array<int, 2>> &pairs, double weight,
                                         const std::vector<double> &target, const std::vector<double> &limit,
                                         const std::vector<double> *pair_w, const std::vector<double> *target_w,
                                         FieldSolver &solver, const std::vector<double> *initial)
{
    // Minimize 1/2 x^T H x - g^T x in the box, H = w * graph Laplacian + target indicator
    // (+ a vanishing ridge for cells no target reaches). H is an M-matrix, so the primal-dual
    // active set method converges monotonically in a handful of solves.
    std::vector<Eigen::Triplet<double>> h;
    h.reserve(pairs.size() * 4 + n);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    for (size_t k = 0; k < pairs.size(); ++k) {
        const auto [i, j] = pairs[k];
        const double wk   = weight * (pair_w ? (*pair_w)[k] : 1.);
        h.emplace_back(i, i, wk);
        h.emplace_back(j, j, wk);
        h.emplace_back(i, j, -wk);
        h.emplace_back(j, i, -wk);
    }
    for (size_t c = 0; c < n; ++c) {
        h.emplace_back(c, c, 1e-12);
        if (! std::isnan(target[c])) {
            const double tw = target_w ? (*target_w)[c] : 1.;
            h.emplace_back(c, c, tw);
            g[c] = tw * target[c];
        }
    }
    Eigen::SparseMatrix<double> H(n, n);
    H.setFromTriplets(h.begin(), h.end());
    RowMatrix       Hr;   // for conjugate gradients
    Eigen::VectorXd diag;

    Eigen::VectorXd  x      = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd  lambda = Eigen::VectorXd::Zero(n); // g - H x on active cells
    if (initial) {
        // Warm start: from a previous solution with its multipliers at the bounds, so the active set
        // starts where that one ended. The optimum is unique, so only the number of solves changes.
        for (size_t c = 0; c < n; ++c)
            x[c] = std::clamp((*initial)[c], -limit[c], limit[c]);
        lambda = g - H * x;
        for (size_t c = 0; c < n; ++c)
            if (std::abs(x[c]) < limit[c] - 1e-12)
                lambda[c] = 0.;
    }
    std::vector<int> state(n, 0), prev_state;           // -1 at lower bound, +1 at upper, 0 free
    const double     c_scale = weight + 1.;
    for (int it = 0; it < 200; ++it) {
        for (size_t c = 0; c < n; ++c) {
            if (lambda[c] + c_scale * (x[c] - limit[c]) > 0.)
                state[c] = 1;
            else if (lambda[c] + c_scale * (x[c] + limit[c]) < 0.)
                state[c] = -1;
            else
                state[c] = 0;
        }
        if (it > 0 && state == prev_state)
            break;
        prev_state = state;

        std::vector<int> index(n, -1);
        int              num_free = 0;
        for (size_t c = 0; c < n; ++c)
            if (state[c] == 0)
                index[c] = num_free++;
            else
                x[c] = state[c] * limit[c];
        if (num_free > 0)
            ++solver.solves;
        if (num_free > 0 && solver.iterative && Hr.rows() == 0) {
            Hr   = H;
            diag = H.diagonal();
        }
        if (num_free > 0 && solver.iterative && solve_free_cg(Hr, diag, g, state, x)) {
            // solved; a system it does not converge on is factorized below instead
        } else if (num_free > 0) {
            std::vector<Eigen::Triplet<double>> t;
            Eigen::VectorXd                     rhs(num_free);
            for (size_t c = 0; c < n; ++c)
                if (index[c] >= 0)
                    rhs[index[c]] = g[c];
            for (int k = 0; k < H.outerSize(); ++k)
                for (Eigen::SparseMatrix<double>::InnerIterator it2(H, k); it2; ++it2) {
                    const int r = index[it2.row()], col = index[it2.col()];
                    if (r < 0)
                        continue;
                    if (col >= 0)
                        t.emplace_back(r, col, it2.value());
                    else
                        rhs[r] -= it2.value() * x[it2.col()];
                }
            Eigen::SparseMatrix<double> Hff(num_free, num_free);
            Hff.setFromTriplets(t.begin(), t.end());
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(Hff);
            if (ldlt.info() != Eigen::Success)
                throw std::runtime_error("S4: rotation field system could not be factorized");
            const Eigen::VectorXd xf = ldlt.solve(rhs);
            if (solver.factor_size == 0) {
                solver.factor_size = size_t(ldlt.matrixL().nestedExpression().nonZeros());
                solver.iterative   = solver.multithreading && solver.factor_size > solver.iterative_factor_size;
            }
            for (size_t c = 0; c < n; ++c)
                if (index[c] >= 0)
                    x[c] = xf[index[c]];
        }
        lambda = g - H * x;
        for (size_t c = 0; c < n; ++c)
            if (state[c] == 0)
                lambda[c] = 0.;
    }
    return std::vector<double>(x.data(), x.data() + n);
}

} // namespace

std::vector<std::array<int, 3>> s4_boundary_triangles(const TetMesh &mesh, const std::vector<Eigen::Vector3d> &pts)
{
    const Topology                  topo = build_topology(mesh);
    std::vector<std::array<int, 3>> out;
    out.reserve(topo.boundary.size());
    for (const BoundaryFace &f : topo.boundary) {
        std::array<int, 3>    t = f.v;
        const Eigen::Vector3d n = (pts[t[1]] - pts[t[0]]).cross(pts[t[2]] - pts[t[0]]);
        if (n.dot(pts[f.opposite] - pts[t[0]]) > 0.)
            std::swap(t[1], t[2]);
        out.push_back(t);
    }
    return out;
}

S4Result s4_deform(const TetMesh &mesh, const S4Params &params)
{
    if (mesh.tets.empty())
        throw std::runtime_error("S4: empty tetrahedral mesh");
    const Topology topo = build_topology(mesh);
    const size_t   n    = mesh.tets.size();

    // Vertices on the bed (or in the planar base) stay put. Everything else keeps clear of the
    // base: a region pushed down to its level would be sliced into those layers and printed in
    // mid-air.
    double min_z = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &p : mesh.points)
        min_z = std::min(min_z, p.z());
    const double        base_z = min_z + std::max(params.planar_height, 0.);
    std::vector<char>   pinned(mesh.points.size());
    std::vector<double> z_floor(mesh.points.size());
    for (size_t v = 0; v < mesh.points.size(); ++v) {
        pinned[v]  = mesh.points[v].z() <= base_z + 1e-6;
        z_floor[v] = base_z + std::min(mesh.points[v].z() - base_z, params.bottom_threshold);
    }
    // Cells of the planar base do not turn, so the rotation field starts from zero at its top.
    std::vector<char> frozen(n, 0);
    if (params.planar_height > 0.)
        for (size_t c = 0; c < n; ++c)
            frozen[c] = std::all_of(mesh.tets[c].begin(), mesh.tets[c].end(), [&pinned](int v) { return pinned[v] != 0; });

    // Cells turned inside out get their rotation limits (and their neighbourhood's) cut back,
    // and the pass is solved again. No rotation means no inversion, so this terminates.
    constexpr int    max_rounds = 10;
    constexpr double shrink     = 0.7;

    // EXPERIMENT, for evaluation only: S4_PROTO_WEIGHTED=1 weights each cell's terms by its size (1 on
    // a uniform mesh), so a mesh refined at the surface solves the same problem. Unset, nothing changes.
    std::vector<double> cell_w, pair_w, target_w;
    const bool          weighted = std::getenv("S4_PROTO_WEIGHTED") != nullptr;
    if (weighted) {
        std::vector<double> vol(n), side(n);
        double              mv = 0., ms = 0., ma = 0.;
        for (size_t c = 0; c < n; ++c) {
            const std::array<int, 4> &t = mesh.tets[c];
            vol[c]  = std::abs((mesh.points[t[1]] - mesh.points[t[0]]).dot((mesh.points[t[2]] - mesh.points[t[0]]).cross(mesh.points[t[3]] - mesh.points[t[0]]))) / 6.;
            side[c] = std::cbrt(vol[c]);
            mv += vol[c];
            ms += side[c];
            ma += side[c] * side[c];
        }
        mv /= double(n);
        ms /= double(n);
        ma /= double(n);
        cell_w.resize(n);
        target_w.resize(n);
        for (size_t c = 0; c < n; ++c) {
            cell_w[c]   = vol[c] / mv;
            target_w[c] = side[c] * side[c] / ma;
        }
        pair_w.resize(topo.face_pairs.size());
        for (size_t k = 0; k < topo.face_pairs.size(); ++k)
            pair_w[k] = 0.5 * (side[topo.face_pairs[k][0]] + side[topo.face_pairs[k][1]]) / ms;
    }

    FieldSolver field;
    field.multithreading        = params.multithreading;
    field.iterative_factor_size = params.iterative_factor_size;
    S4Result result;
    result.deformed = mesh.points;
    const double threshold = deg2rad(90. + params.max_overhang);
    for (int pass = 0; pass < std::max(1, params.passes); ++pass) {
        const Attributes a = compute_attributes(result.deformed, mesh, topo, params);
        S4PassData       data;
        data.bottom    = a.bottom;
        data.in_air    = a.in_air;
        data.overhang  = a.overhang;
        data.direction = rotation_direction(a, topo, params, data.distance);
        const std::vector<double> limit = rotation_limits(data.distance, params);

        std::vector<double>          raw_target(n, NaN);
        for (size_t c = 0; c < n; ++c) {
            double t = std::abs(threshold - a.overhang[c]);
            if (params.steep_overhang_compensation && a.in_air[c])
                t += 2. * (PI - a.overhang[c]);
            raw_target[c] = t * data.direction[c] * params.rotation_multiplier;
        }

        std::vector<double>          scale(n, 1.);
        std::vector<Eigen::Vector3d> deformed;
        const int                    solves_before = field.solves;
        for (int round = 0;; ++round) {
            data.limit.resize(n);
            data.target.assign(n, NaN);
            for (size_t c = 0; c < n; ++c) {
                data.limit[c] = frozen[c] ? 0. : limit[c] * scale[c];
                if (! std::isnan(raw_target[c]))
                    data.target[c] = std::clamp(raw_target[c], -data.limit[c], data.limit[c]);
            }
            // Warm start: each round after the first changes only the limits near inverted cells.
            data.rotation = solve_rotation_field(n, topo.face_pairs, params.neighbour_weight, data.target, data.limit,
                                                 weighted ? &pair_w : nullptr, weighted ? &target_w : nullptr, field,
                                                 params.warm_start && round > 0 ? &data.rotation : nullptr);
            deformed = solve_deformation(result.deformed, mesh, rotation_matrices(a.center, data.rotation, params.axis), pinned, z_floor,
                                         params.multithreading, weighted ? &cell_w : nullptr);

            std::vector<int> inverted;
            for (size_t c = 0; c < n; ++c)
                if (signed_volume(result.deformed, mesh.tets[c]) * signed_volume(deformed, mesh.tets[c]) <= 0.)
                    inverted.push_back(int(c));
            data.inverted = inverted.size();
            data.rounds   = round + 1;
            if (inverted.empty() || round + 1 == max_rounds)
                break;
            std::vector<char> hit(n, 0);
            for (int c : inverted) {
                hit[c] = 1;
                for (int o : topo.point_neighbours[c]) {
                    hit[o] = 1;
                    for (int oo : topo.point_neighbours[o])
                        hit[oo] = 1;
                }
            }
            for (size_t c = 0; c < n; ++c)
                if (hit[c])
                    scale[c] *= shrink;
        }
        data.solves      = field.solves - solves_before;
        data.factor_size = field.factor_size;
        data.iterative   = field.iterative;
        result.deformed  = std::move(deformed);
        result.passes.emplace_back(std::move(data));
    }
    return result;
}

S4SolverOptions &s4_solver_options()
{
    static S4SolverOptions options;
    return options;
}

} // namespace NonPlanar
} // namespace Slic3r
