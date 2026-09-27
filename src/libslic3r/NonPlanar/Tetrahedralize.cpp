#include "Tetrahedralize.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/make_mesh_3.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace Slic3r {
namespace NonPlanar {

namespace {

using Kernel   = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point    = Kernel::Point_3;
using Surface  = CGAL::Surface_mesh<Point>;
using Domain   = CGAL::Polyhedral_mesh_domain_with_features_3<Kernel, Surface>;
using Tr       = CGAL::Mesh_triangulation_3<Domain, CGAL::Default, CGAL::Sequential_tag>::type;
using Complex  = CGAL::Mesh_complex_3_in_triangulation_3<Tr, Domain::Corner_index, Domain::Curve_index>;
using Criteria = CGAL::Mesh_criteria_3<Tr>;

namespace PMP = CGAL::Polygon_mesh_processing;

} // namespace

TetMesh tetrahedralize(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3>> &triangles,
                       const TetrahedralizeParams &params)
{
    std::vector<Point>               points;
    std::vector<std::vector<size_t>> polygons;
    points.reserve(vertices.size());
    for (const Eigen::Vector3d &v : vertices)
        points.emplace_back(v.x(), v.y(), v.z());
    polygons.reserve(triangles.size());
    for (const std::array<int, 3> &t : triangles)
        polygons.push_back({ size_t(t[0]), size_t(t[1]), size_t(t[2]) });

    // Model files do not guarantee consistent winding; fix it before building the surface.
    PMP::orient_polygon_soup(points, polygons);
    Surface surface;
    PMP::polygon_soup_to_polygon_mesh(points, polygons, surface);
    if (surface.is_empty() || ! CGAL::is_closed(surface))
        throw std::runtime_error("S4: the model is not a closed surface and cannot be filled with tetrahedra");
    if (! PMP::is_outward_oriented(surface))
        PMP::reverse_face_orientations(surface);

    double size = params.cell_size;
    if (size <= 0.) {
        Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max()), hi = -lo;
        for (const Eigen::Vector3d &v : vertices) {
            lo = lo.cwiseMin(v);
            hi = hi.cwiseMax(v);
        }
        size = (hi - lo).maxCoeff() / 20.;
    }

    Domain domain(surface);
    domain.detect_features(params.feature_angle);
    namespace p = CGAL::parameters;
    // The sharp edges are protected by balls, which shrink where edges come close. Without a
    // lower bound a model with fine detail (lettering, small holes) keeps them shrinking
    // practically forever, so they stop at a quarter of the cell size.
    const Criteria criteria(p::edge_size = size, p::edge_min_size = size / 4., p::facet_angle = 25., p::facet_size = size,
                            p::facet_distance = size / 10., p::cell_radius_edge_ratio = 3., p::cell_size = size);
    const Complex complex = params.optimize ? CGAL::make_mesh_3<Complex>(domain, criteria, p::perturb(), p::exude()) :
                                              CGAL::make_mesh_3<Complex>(domain, criteria, p::no_perturb(), p::no_exude());

    TetMesh                                                        mesh;
    std::unordered_map<Tr::Vertex_handle, int, CGAL::Handle_hash_function> index;
    auto vertex_index = [&](Tr::Vertex_handle v) {
        auto [it, inserted] = index.emplace(v, int(mesh.points.size()));
        if (inserted) {
            const auto &p = v->point().point();
            mesh.points.emplace_back(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
        }
        return it->second;
    };
    for (auto c = complex.cells_in_complex_begin(); c != complex.cells_in_complex_end(); ++c)
        mesh.tets.push_back({ vertex_index(c->vertex(0)), vertex_index(c->vertex(1)), vertex_index(c->vertex(2)), vertex_index(c->vertex(3)) });
    if (mesh.tets.empty())
        throw std::runtime_error("S4: tetrahedralization produced no cells");
    return mesh;
}

} // namespace NonPlanar
} // namespace Slic3r
