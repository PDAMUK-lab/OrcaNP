#pragma once

// S4 non-planar deformation, after Joshua Bird's S4 Slicer (GPL-3.0).
//
// Instead of slicing curved layers, S4 bends the part: every tetrahedron of a volume mesh gets a
// rotation that makes its overhang printable, the rotations are smoothed into a field, and the
// mesh is deformed to follow that field. Flat layers sliced from the deformed mesh become curved
// layers once mapped back into the real part.
//
// Cells rotate about the horizontal axis tangential to circles around the printer's rotation
// axis, so layers lean in the radial plane: the plane a polar printer's tilting nozzle moves in.
//
// Only Eigen and the standard library are used.

#include <Eigen/Core>

#include <array>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

struct TetMesh
{
    std::vector<Eigen::Vector3d>    points;
    std::vector<std::array<int, 4>> tets;
};

struct S4Params
{
    // Overhangs steeper than this (degrees from vertical) are rotated towards printable.
    double max_overhang = 30.;
    // Weight of rotation differences between face-adjacent cells against following each
    // overhang's target rotation. Larger is smoother.
    double neighbour_weight = 20.;
    // Scales the target rotations.
    double rotation_multiplier = 2.;
    // Passes of neighbourhood averaging applied to the rotation direction field.
    int smoothing_iterations = 30;
    // Give every non-overhanging boundary cell a zero target instead of no target, which
    // holds the rest of the part still. Useful when the direction field is noisy.
    bool zero_initial_rotation = false;
    // Cells whose way to the bed climbs over higher cells (overhangs past horizontal) get an
    // extra rotation so they turn the right way.
    bool steep_overhang_compensation = true;
    // Rotation limits (degrees), tapering from cells next to the support to the farthest ones.
    double max_rotation_near    = 45.;
    double max_rotation_far     = 20.;
    double max_rotation_falloff = 1.75; // exponent of the taper
    // Boundary cells whose lowest face is within this height of the lowest face sit on the bed.
    double bottom_threshold = 0.3;
    // The part up to this height above its lowest point stays as it is (flat layers) and is the
    // base the rest is deformed from, in place of the bed. 0: the bed.
    double planar_height = 0.;
    // A cell is "in air" when its path to the bed rises more than this above it.
    double in_air_threshold = 1.;
    // Printer rotation axis in the mesh frame.
    Eigen::Vector2d axis { 0., 0. };
    // Deformation passes; each pass deforms the result of the previous one.
    int passes = 1;

    // Speed-ups that leave the result as it is (the application's preferences, s4_solver_options()).
    // Each round that cuts back rotation limits where cells inverted starts from the last round's
    // solution and the rotation limits it found binding, instead of from nothing.
    bool warm_start = true;
    // The deformation's three axes are solved at once, and the rotation field, once its direct
    // factorization is larger than `iterative_factor_size` nonzeros, by conjugate gradients split
    // over threads. That size, not a timing, decides, so a model always takes the same route.
    bool   multithreading        = true;
    size_t iterative_factor_size = 3000000;
};

// Per-cell values of one pass, for inspection and tests. Angles in radians; NaN where a value
// does not apply to the cell.
struct S4PassData
{
    std::vector<char>   bottom;
    std::vector<char>   in_air;
    std::vector<double> overhang; // angle of the lowest boundary face normal from +Z
    std::vector<double> distance; // path length to the bed, overhanging cells only
    std::vector<double> direction; // smoothed rotation direction (-1 .. 1)
    std::vector<double> target;   // target rotation
    std::vector<double> limit;    // rotation bound, after cutting back where cells inverted
    std::vector<double> rotation; // optimized rotation
    size_t              inverted = 0; // cells still inside out after the last round
    int                 rounds   = 0; // solves needed to remove inversions
    int                 solves   = 0; // rotation field systems solved, over all rounds
    size_t              factor_size = 0;  // nonzeros of the first direct factorization
    bool                iterative   = false; // switched to conjugate gradients
};

struct S4Result
{
    // Deformed vertex positions, numbered like the input points. Vertices on the bed stay put;
    // all others stay at least min(their height, bottom_threshold) above the bed.
    std::vector<Eigen::Vector3d> deformed;
    std::vector<S4PassData>      passes;
};

S4Result s4_deform(const TetMesh &mesh, const S4Params &params);

// Boundary triangles of the tetrahedral mesh, oriented outward for the vertex positions `pts`
// (numbered like mesh.points): the surface to slice once the mesh has been deformed.
std::vector<std::array<int, 3>> s4_boundary_triangles(const TetMesh &mesh, const std::vector<Eigen::Vector3d> &pts);

// Solves  min  w * sum_{(i,j) in pairs} (x_i - x_j)^2 + sum_{i in targets} (x_i - t_i)^2
//         s.t. -limit_i <= x_i <= limit_i
// Cells without a target (NaN) follow their neighbours. Exposed for testing.
std::vector<double> s4_solve_rotation_field(size_t num_cells, const std::vector<std::array<int, 2>> &pairs, double weight,
                                            const std::vector<double> &target, const std::vector<double> &limit);

// The application's speed-up preferences, copied into the S4Params of each slice. Set once at
// startup (a change takes a restart).
struct S4SolverOptions
{
    bool warm_start     = true;
    bool multithreading = true;
};
S4SolverOptions &s4_solver_options();

} // namespace NonPlanar
} // namespace Slic3r
