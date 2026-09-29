#pragma once

// Orca: the print surface of non-planar (S4) offset layers, generated from the part's faces painted
// as print surface. It fills the space under the painted faces down to the bed (laid out from
// above), or between them and the rotation axis and down to the bed below them (laid out around
// the axis), the surface gap short of them, so the part's first layer is its painted faces.

#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r::NonPlanar {

// Under the painted faces that face down, closed by walls down to floor_z and a bottom there.
// Throws std::runtime_error, with a message for the user, if no painted face faces down.
indexed_triangle_set painted_surface_from_above(const indexed_triangle_set &painted, double gap, double floor_z);

// Inside the painted faces that face the rotation axis, closed by a cap at their top and a column
// down to floor_z under them. Throws std::runtime_error, with a message for the user, if no painted
// face faces the axis or the painted faces do not go all the way round it.
indexed_triangle_set painted_surface_around_axis(const indexed_triangle_set &painted, const Vec2d &axis, double gap, double floor_z);

} // namespace Slic3r::NonPlanar
