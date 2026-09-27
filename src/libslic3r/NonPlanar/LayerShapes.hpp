#pragma once

// Analytic layer shapes: deformations whose flat slices become chosen surfaces in the part, as
// alternatives to the optimized S4 deformation. Like S4 they move the vertices of a tetrahedral
// mesh, so slicing, mapping back, nozzle tilt and flow work the same way.

#include <Eigen/Core>

#include <array>
#include <limits>
#include <vector>

namespace Slic3r {
namespace NonPlanar {

// Signed distance to a closed triangle surface (positive outside), from a uniform grid of the
// triangles and a vertical ray for inside/outside. Faces lying at `floor_z` stand on the bed:
// they close the surface for inside/outside but are no surface to measure from, so the distance
// inside near the bed is to the walls.
class SurfaceDistance
{
public:
    SurfaceDistance(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                    double floor_z = -std::numeric_limits<double>::infinity());

    double signed_distance(const Eigen::Vector3d &p) const;
    double distance(const Eigen::Vector3d &p) const;
    bool   inside(const Eigen::Vector3d &p) const;

    const Eigen::Vector3d &bbox_min() const { return m_min; }
    const Eigen::Vector3d &bbox_max() const { return m_max; }

private:
    void cell_of(const Eigen::Vector3d &p, int &i, int &j, int &k) const;

    std::vector<Eigen::Vector3d>    m_vertices;
    std::vector<std::array<int, 3>> m_triangles;
    Eigen::Vector3d                 m_min, m_max;
    double                          m_cell = 1.;
    int                             m_n[3] { 1, 1, 1 };
    std::vector<std::vector<int>>   m_grid;     // triangles overlapping each cell
    std::vector<bool>               m_on_floor; // per triangle: lies at the floor
};

// A core fitted into a part's cavity, for the part to be printed over.
struct FittedCore
{
    Eigen::Vector3d base;   // sphere: its centre; cylinder: the axis at the part's bottom
    double          radius; // out to the part's inner surface
    double          height; // cylinder: from the base up to the cavity's roof
};

// A sphere about a centre below the part's top by half the part's width (a dome's or sphere's
// centre), out to the nearest point of the part. Throws when the centre is inside the part.
FittedCore fit_sphere_core(const SurfaceDistance &part);

// A cylinder about a vertical axis, from the part's bottom up to the roof of its cavity, out to
// the part's nearest wall below the roof. Throws when the part covers the axis at its bottom
// (the core must stand on the bed) or has no cavity around the axis.
FittedCore fit_cylinder_core(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                             const SurfaceDistance &part, const Eigen::Vector2d &axis);

// Layers at constant distance from the print surface, stacked above `base_z` (the print surface's
// top, where its own flat layers end). A point at distance d from the surface goes to height
// base_z + d - gap, so the first layer lies `gap` off the surface and what is closer (or inside)
// ends up below base_z, where it is not printed.
//
// Projected from above: the horizontal position comes from the direction of the point seen from
// `centre`, as an azimuthal equidistant projection about the upward direction scaled by `scale`
// (the distance from the centre to the first layer at the top, so the top is not distorted).
// Suits domes, spheres and capped cylinders around the centre.
std::vector<Eigen::Vector3d> deform_offset_from_above(const std::vector<Eigen::Vector3d> &points, const SurfaceDistance &surface,
                                                      const Eigen::Vector3d &centre, double scale, double base_z, double gap);

// Projected around a vertical axis: the angle about the axis, measured from `from_angle` over
// [0, 2 pi), becomes arc length at `scale` along X, and the height becomes Y. Suits sleeves
// around the axis. `shift` moves the result along X (for the copies that continue the unwrap past
// a full turn).
std::vector<Eigen::Vector3d> deform_offset_around_axis(const std::vector<Eigen::Vector3d> &points, const SurfaceDistance &surface,
                                                       const Eigen::Vector2d &axis, double from_angle, double scale, double base_z,
                                                       double gap, double shift);

// Conical layers about a vertical axis (the radial slicer's): z' = z + tan(angle) r. A positive
// angle makes layers descend away from the axis.
std::vector<Eigen::Vector3d> deform_cone(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector2d &axis, double angle);

} // namespace NonPlanar
} // namespace Slic3r
