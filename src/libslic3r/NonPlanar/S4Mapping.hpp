#pragma once

// Maps points from the deformed (sliced) space of an S4 deformation back into the real part,
// with the nozzle tilt and flow correction that go with them.

#include "S4Deformation.hpp"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

class S4Mapper
{
public:
    enum class Tier : unsigned char {
        Inside,  // inside a deformed tetrahedron: exact barycentric mapping
        Nearest, // outside: displacement of the closest tetrahedron, weights clamped to it
        Idw,     // no usable tetrahedron: inverse-distance weighted vertex displacement
        Outside, // too far outside the mesh to be part of it: left where it is
    };

    struct Result
    {
        Eigen::Vector3d point;
        // Nozzle axis angle from vertical in the radial plane (radians), positive leaning
        // outward: the real-space normal of the sliced layer at the point.
        double tilt = 0.;
        // Undeformed / deformed volume of the tetrahedron: how much the layer was squashed.
        double flow = 1.;
        int    cell = -1;
        Tier   tier = Tier::Outside;
    };

    // `bbox_margin`: how far outside the deformed mesh a point may be and still be mapped.
    S4Mapper(const TetMesh &undeformed, const std::vector<Eigen::Vector3d> &deformed, const Eigen::Vector2d &axis, double bbox_margin = 2.);

    Result map(const Eigen::Vector3d &deformed_point) const;

    const Eigen::Vector2d& axis() const { return m_axis; }
    const Eigen::Vector3d& bbox_min() const { return m_min; }
    const Eigen::Vector3d& bbox_max() const { return m_max; }
    bool                   in_bbox(const Eigen::Vector3d &p) const;

private:
    // Signed barycentric weights of `p` in deformed tetrahedron `cell`.
    Eigen::Vector4d barycentric(int cell, const Eigen::Vector3d &p) const;
    double          distance_to_cell(int cell, const Eigen::Vector3d &p) const;
    void            grid_cell(const Eigen::Vector3d &p, int &i, int &j, int &k) const;

    std::vector<std::array<int, 4>> m_tets;
    std::vector<Eigen::Vector3d> m_deformed;
    std::vector<Eigen::Vector3d> m_displacement; // deformed - undeformed, per vertex
    std::vector<Eigen::Vector3d> m_normal;       // real-space layer normal, per vertex
    std::vector<double>          m_flow;         // per cell
    std::vector<Eigen::Matrix3d> m_inverse;      // per cell, inverse of the deformed edge matrix
    Eigen::Vector2d              m_axis;
    double                       m_margin;
    Eigen::Vector3d              m_min, m_max;

    // Uniform grid over the deformed mesh; each bucket lists the cells overlapping it.
    Eigen::Vector3d               m_grid_origin;
    double                        m_grid_step = 1.;
    std::array<int, 3>            m_grid_size { 1, 1, 1 };
    std::vector<std::vector<int>> m_buckets;
};

} // namespace NonPlanar
} // namespace Slic3r
