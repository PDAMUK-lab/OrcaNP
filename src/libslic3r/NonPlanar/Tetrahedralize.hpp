#pragma once

// Fills a closed triangle mesh with tetrahedra for the S4 deformation (CGAL Mesh_3).

#include "S4Deformation.hpp"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

struct TetrahedralizeParams
{
    // Target tetrahedron edge length in mm. 0 picks 1/20 of the largest bounding box side.
    double cell_size = 0.;
    // Graded mesh: tetrahedra this size (mm) at the surface, which they follow to within a twentieth
    // of it, growing to cell_size inside. 0: uniform, the surface followed to within cell_size / 10.
    double surface_cell_size = 0.;
    // Surface edges sharper than this dihedral angle (degrees) are kept exactly.
    double feature_angle = 60.;
    // Remove sliver tetrahedra (by exudation). Slivers are nearly flat, so the
    // deformation easily turns them inside out.
    bool optimize = true;
};

// The cell size of an automatic uniform mesh: 1/20 of the largest side of the bounding box.
double automatic_cell_size(const std::vector<Eigen::Vector3d> &vertices);

// Throws std::runtime_error when the input is not a closed, orientable surface.
TetMesh tetrahedralize(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                       const TetrahedralizeParams &params = {});

} // namespace NonPlanar
} // namespace Slic3r
