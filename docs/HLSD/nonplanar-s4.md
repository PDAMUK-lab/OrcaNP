# S4 non-planar printing for polar printers — High Level Design

## Purpose and scope

S4 (after Joshua Bird's S4 Slicer) prints overhangs without support by bending the part
instead of the layers. The model is filled with tetrahedra, each tetrahedron gets a rotation
that makes its overhang printable, the rotations are smoothed into a field, and the mesh is
deformed to follow it. The deformed mesh is sliced with ordinary flat layers; mapping the
toolpath back into the real part turns those flat layers into curved ones.

The target machine is a polar (R-theta) printer: a rotating bed (the angle axis), a head on one
radial line through the bed's rotation axis (the radius axis), Z, and a nozzle that tilts in the
radial plane (the tilt axis). Cells therefore rotate about the horizontal axis tangential to
circles around the rotation axis, so every layer leans in the plane the nozzle can tilt in.

The code lives in `src/libslic3r/NonPlanar/` and uses only Eigen and the standard library,
except the mesher, which uses CGAL and is built into `libslic3r_cgal`.

## Pipeline

1. **Tetrahedralize** (`Tetrahedralize`): CGAL Mesh_3 fills the closed model. The triangle
   soup is re-oriented first, sharp edges are protected, and slivers are removed; slivers are
   nearly flat and the deformation turns them inside out easily. Cell size defaults to 1/20 of
   the largest bounding box side.
2. **Deform** (`S4Deformation`), per pass:
   - Cell attributes: the most downward boundary face normal gives the overhang angle; cells
     whose lowest face is within `bottom_threshold` of the lowest face sit on the bed; a
     multi-source Dijkstra from those cells over vertex-sharing neighbours gives each cell's
     path length to the bed, and a cell whose path climbs over higher cells is "in air".
   - Rotation direction: for overhanging cells, the downhill slope of the path length along the
     radial direction, from a plane fitted through the cell and its overhanging edge
     neighbours (the plane normal is oriented before use), or the direction of the nearest
     bed cell when there are too few neighbours. The direction field is averaged over two-ring
     neighbourhoods.
   - Target rotation: how far the overhang exceeds `max_overhang`, times the direction and
     `rotation_multiplier`, bounded by a limit tapering from `max_rotation_near` next to the
     support to `max_rotation_far` for the cells farthest from it.
   - Rotation field: minimize `w * sum (r_i - r_j)^2` over face-adjacent cells plus
     `sum (r_i - t_i)^2` over cells with a target, within the limits. The Hessian is an
     M-matrix, so a primal-dual active set method converges in a few sparse solves.
   - Deformation: vertex positions whose cells best match their rotated, centred shapes, a
     linear least-squares problem whose axes decouple. Vertices on the bed are pinned; all
     other vertices stay at least `min(height, bottom_threshold)` above the bed (an active set
     on the Z solve), because a region pushed down to bed level would be sliced into the first
     layers and printed in mid-air.
   - Inversion repair: cells turned inside out get their limits, and those of their two-ring
     neighbourhood, cut to 70 % and the pass is solved again, up to ten times. Zero rotation
     cannot invert, so this converges.
3. **Slice** the deformed mesh with flat layers. The G-code must use relative extrusion and no
   arcs are required (arcs are linearized).
4. **Map back** (`S4Mapping`, `S4GCodeTransform`): every move is subdivided in the sliced space
   (`seg_size` for printing, `travel_seg_size` for travels) and mapped through the tetrahedra:
   barycentric inside a cell, the clamped displacement of the closest cell just outside it
   (bead centres of outer walls sit half a line width outside the mesh), inverse-distance
   weighting as a last resort, and far points unchanged. Each point also gets:
   - the nozzle tilt: the real normal of the sliced layer, `F^T z` for the deformation gradient
     `F = D1 D0^-1`, interpolated from volume-weighted vertex normals and measured in the radial
     plane;
   - the flow factor: the cell's undeformed / deformed volume ratio.
   Repair passes follow: extrusion scaled by the flow factor (clipped to 0.25–3 with the clipped
   material carried forward, redistributed by mapped length within each source line), a Z
   floor, zero-length moves folded into the next move (a move that only turns the nozzle is not
   zero-length), Z steps capped at 45°, and travels lifted over printed material they would
   plough through. The last uses a running height field of deposited material; a lifted travel
   climbs and descends vertically and is retracted when long or when it had to hop, unless the
   slicer already retracted. One record per source line owns its output, so layer markers,
   feedrates and the end block stay in step with the motion.
5. **Convert to polar** (`PolarKinematics`): each Cartesian move (tip position plus radial tilt)
   becomes machine moves in angle, radius, Z and tilt. Moves are split until each segment turns
   the bed by at most `max_angle_step` and covers at most `max_segment_length`. The angle is
   unwrapped. A signed radius lets a path cross the rotation axis without turning the bed by
   180°. Within `min_radius` of the axis the angle is held and only the radius moves, which is
   the only place the path deviates from the Cartesian line (by at most `min_radius`). Feed is
   inverse time (G93) by default, with each segment's duration taken from the Cartesian feed and
   stretched to respect the angle and tilt speed limits; retractions switch to G94. An optional
   pivot length compensates for firmware that positions the tilt pivot rather than the tip.

## Frames and placement

The rotation axis is a point in the model's frame (by default the centre of its bounding box);
the deformation is defined relative to it. G-code coordinates are mesh coordinates plus an
offset, and the polar centre is the axis plus the same offset, so the axis moves with the part:
where a slicer places the part on its bed does not change the machine output.

## Machine blocks

The machine start and end G-code are written for the polar machine and are copied verbatim by
both the S4 transform and the polar conversion, delimited by Orca's `MACHINE_START_GCODE_END`
and `MACHINE_END_GCODE_START` tags. Without the tags, the body starts at the first layer marker
(or the first extruding move) and the end block starts after the last extruding move.

## Integration in OrcaSlicer

Two groups of settings switch the pipeline on:

- **Print settings > Quality > Non-planar (S4)** (`s4_*`, per object): `s4_enabled` plus the
  deformation parameters above. Any change re-slices the object.
- **Printer settings > Basic information > Polar kinematics** (`polar_*`): `polar_kinematics`, the
  axis letters, the tilt axis, and the conversion and speed limits. These only affect G-code export.

The pipeline hooks into the print steps as follows:

- **Slicing** (`PrintObject::slice()`): with `s4_enabled`, `deform_s4()` meshes and deforms the
  object's single model part in its slicing frame (`trafo_centered()`) before the layers are
  laid out. The layer heights are then computed for the deformed height, and `slice_volumes()`
  slices the deformed surface in place of the part. Perimeters, infill, supports, seams and the
  rest of the pipeline run unchanged on those flat slices. The rotation axis in that frame is
  the centre of the printable area moved into the object's frame through the instance shift and
  plate origin. So moving the object, or changing the bed shape, re-slices it.
- **Validation** (`Print::validate()`): an S4 object must be one model part with no modifiers or
  negative volumes, placed once. The mapping only handles relative extrusion, and spiral vase
  is refused.
- **Export** (`GCode::do_export()`): when an object is deformed, generation writes the sliced-space
  G-code to a side file without the G-code processor. `NonPlanarExport::s4_mapper()` joins every
  deformed object's tetrahedra in G-code coordinates (instance shift less plate origin in XY;
  Z offset and raft height in Z), and `s4_transform_gcode()` maps the file back. Only the mapped
  G-code is then streamed through the processor, so the preview, time estimate and filament
  statistics describe the curved toolpath that will be printed. With `polar_kinematics`, the
  processed file is converted to machine coordinates last, just before the export rename. The
  preview therefore stays Cartesian, and its G-code text view does not match the polar file line
  for line.

## Tool

`s4_polar` (`src/dev-utils/`, built with `ORCA_TOOLS`) runs the pipeline around any slicer:
`deform` writes the deformed STL and the mesh pair (`.s4mesh`), `map` maps sliced G-code back
(detecting the part's placement from its own extrusion, skirt/brim/purge excluded) and with
`--polar` converts it, and `polar` converts Cartesian G-code alone.

## Tests

`tests/libslic3r/test_polar_kinematics.cpp` and `tests/libslic3r/test_s4.cpp` cover the
kinematics round trip, continuity and axis crossing, G-code conversion (path, extrusion,
timing, machine blocks, arcs), the deformation's guarantees (pinned base, bed clearance, no
inverted cells, limits and rotation direction), the bounded rotation solver against closed-form
solutions, meshing volume, the mapper (identity, rigid tilt, squash, outside points) and the
G-code transform (identity, flow, travel lifts, absolute extrusion refused).
