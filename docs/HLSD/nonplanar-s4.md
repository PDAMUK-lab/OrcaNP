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
   - Planar base (`planar_height`): the part up to that height keeps its shape and plays the
     bed's role. Its vertices are pinned, its cells get a zero limit (so the rotation field
     starts from zero at its top), its cells are the Dijkstra sources, and the Z floor keeps the
     rest above it. The base prints with ordinary flat layers.
   - Holding the rest (`zero_initial_rotation`): boundary cells that do not overhang get a zero
     target instead of none, so the bend stays near the overhangs instead of spreading through
     the part.
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
   The tilt is then shaped for the machine: below `tilt_threshold` the nozzle stays vertical,
   between it and twice it the tilt ramps up to the layer's, and it never exceeds `max_tilt`.
   Repair passes follow: extrusion scaled by the flow factor (clipped to 0.25–3 with the clipped
   material carried forward, redistributed by mapped length within each source line), a Z
   floor, zero-length moves folded into the next move (a move that only turns the nozzle is not
   zero-length), Z steps capped at 45°, and travels lifted over printed material they would
   plough through. The last uses a running height field of deposited material; a lifted travel
   climbs and descends vertically and is retracted when long or when it had to hop, unless the
   slicer already retracted. One record per source line owns its output, so layer markers,
   feedrates and the end block stay in step with the motion.
   A last, optional pass checks the head's clearance: material printed so far is a height field,
   and every couple of millimetres of nozzle motion each column of it is tested against a cone
   (the nozzle, from its tip radius at the given half-angle up to the nozzle length) and a
   cylinder (the head) around the nozzle axis. A column is solid from the bed up, so it hits
   either volume when the vertical line below its top enters it, a quadratic in height. Columns
   next to the tip, which are the beads being laid and touched, and overlaps under a small
   tolerance are ignored. Hits are counted and reported, not fixed.
5. **Convert to polar** (`PolarKinematics`): each Cartesian move (tip position plus radial tilt)
   becomes machine moves in angle, radius, Z and tilt. Moves are split until each segment turns
   the bed by at most `max_angle_step` and covers at most `max_segment_length`. The angle is
   unwrapped. A signed radius lets a path cross the rotation axis without turning the bed by
   180°. Within `min_radius` of the axis the angle is held and only the radius moves, which is
   the only place the path deviates from the Cartesian line (by at most `min_radius`). Feed is
   inverse time (G93) by default, with each segment's duration taken from the Cartesian feed and
   stretched to respect the angle and tilt speed limits; retractions switch to G94. An optional
   pivot length compensates for firmware that positions the tilt pivot rather than the tip.
   Machine tilt is kept within the axis travel (`min_tilt` .. `max_tilt`); poses beyond it are
   printed at the limit and counted. `tilt_sign` reverses the tilt axis for machines that count
   it positive toward the rotation axis. With `signed_radius` off the radius never goes negative:
   a path through the centre stops there while the bed turns half a turn, for machines whose
   radius travel past the axis is short. Commanded radii outside the radius travel (after pivot
   compensation) are counted, not changed. The machine end block runs in units per minute (G94).

## Layer shapes

Besides the optimized S4 field, the same deform → slice → map-back pipeline takes deformations
whose flat slices are chosen surfaces (`LayerShapes`):

- **Offset from a print surface.** Parts of the object set as print surface form a core that is
  printed first with flat layers. The object's other part is printed over it in layers at
  constant distance from the core's surface, like support: a point at signed distance d from
  the core goes to height H + d - gap. H is the top of the layer holding the core's top, so the
  core's layers all come first, the first layer over the core lies the gap away from it all
  around (not only above it), and what is within the gap or inside the core falls below H and
  is cut off the deformed part before slicing.
  - The distance is exact (a uniform grid of the core's triangles, and a vertical ray for
    inside/outside).
  - Laid out *from above*: the horizontal position is the direction from a centre below the
    core's top, as an azimuthal equidistant projection scaled to be undistorted at the top, for
    domes, spheres and capped cylinders. Circumferential lengths shrink towards the equator
    (to 64 % there); the flow correction keeps the material right.
  - Laid out *around the rotation axis*: the angle becomes length and the height width, for
    sleeves. A closed ring has no seam-free unwrap: sliced as a strip, its ends would become
    walls. So the part is cut in half through the axis, each half unwrapped and placed twice,
    giving a strip of two turns. Only one turn from the middle of it (a window in sliced X) is
    printed: the walls at the strip's ends are never printed, and paths leaving the window
    meet their continuation at the other edge.
- **Conical** (the radial slicer's layers): z' = z + tan(angle) r about the rotation axis.

These shapes have no optimization and no inversion repair: the offset map is injective where
the core is star-shaped about the layout's centre (or axis), and the cone's is always.

## Frames and placement

The rotation axis is a point in the model's frame (by default the centre of its bounding box);
the deformation is defined relative to it. G-code coordinates are mesh coordinates plus an
offset, and the polar centre is the axis plus the same offset, so the axis moves with the part:
where a slicer places the part on its bed does not change the machine output.

## Machine blocks

The machine start and end G-code are written for the polar machine and are copied verbatim by
both the S4 transform and the polar conversion, delimited by Orca's `MACHINE_START_GCODE_END`
and `MACHINE_END_GCODE_START` tags. Orca writes the tags whenever polar kinematics or an S4
object is in the print. Without them (G-code from another slicer), the S4 transform starts the
body at the first layer marker (or the first extruding move) and ends it after the last
extruding move.

## Target machine

The reference machine is Joshua Bird's Core R-Theta 4-axis printer running the ThetaFirm
RepRapFirmware configuration (https://github.com/PDAMUK/ThetaFirm) in 4-axis mode. It moves the
bed angle C (continuous), the radius X and the tilt B through a mixing matrix (`M669 K0`) and
takes inverse time feed. That gives the "ThetaFirm Core R-Theta" printer profile its settings:
- axis letters CXB and G93;
- the bed angle counting with the polar angle;
- a reversed tilt axis (B0 points straight down; a nozzle leaning outward is negative B);
- 43 mm from the tilt pivot to the nozzle tip, which the firmware positions (not the tip);
- radius travel -37.5 to 115.5 mm. The travel past the centre is for pivot compensation, not
  for crossing the axis, so axis crossing is off.

## Integration in OrcaSlicer

Two groups of settings switch the pipeline on:

- **Print settings > Quality > Non-planar (S4)** (`s4_*`, per object): `s4_enabled`, the layer
  shape (`s4_layer_shape`), the S4 deformation parameters, the surface gap and layout, and the
  cone angle. Any change re-slices the object.
- **Print surface** (`s4_print_surface`, a per-part setting added to a part in the object list):
  the part is the core that layers are offset from. OrcaSlicer places every object on the bed,
  so a core and what is printed over it are two parts of one object.
- **Printer settings > Basic information > Polar kinematics** (`polar_*`): `polar_kinematics`, the
  axis letters and directions, axis crossing, the radius and tilt travel, the tilt threshold,
  and the conversion and speed limits.
- **Printer settings > Basic information > Non-planar toolhead** (`nonplanar_*`): the clearance
  check and the nozzle cone angle, nozzle length and head radius it uses.

Most printer settings only affect G-code export. The tilt travel also bounds the S4 deformation:
with a tilting nozzle the S4 rotation limits are capped at the travel both sides of vertical
share, because with axis crossing the polar conversion may reach a point from either side of
the rotation axis, which flips the sign of the tilt. The G-code transform caps the tilt at the
same value, or, without axis crossing, at the travel on each side. Changing the travel, the tilt
axis or polar kinematics therefore re-slices S4 objects.

The pipeline hooks into the print steps as follows:

- **Slicing** (`PrintObject::slice()`): with `s4_enabled`, `deform_s4()` meshes the object's
  non-planar part in its slicing frame (`trafo_centered()`) and deforms it by the layer shape
  before the layers are laid out. The layer heights are then computed for the deformed height,
  and `slice_volumes()` slices the deformed surface in place of the part (and the print surface
  parts as they are). Perimeters, infill, supports, seams and the rest of the pipeline run
  unchanged on those flat slices, per region, so the core can have its own settings. The
  rotation axis in that frame is the centre of the printable area moved into the object's
  frame through the instance shift and plate origin. So moving the object, or changing the bed
  shape, re-slices it.
- **Validation** (`Print::validate()`): an S4 object has one part printed non-planar, besides
  print surface parts (only, and at least one, with offset layers), no modifiers or negative
  volumes, and one instance. The mapping only handles relative extrusion, and spiral vase is
  refused.
- **Export** (`GCode::do_export()`): when an object is deformed, generation writes the
  sliced-space G-code to a side file without the G-code processor, marking each object's
  toolpath (`; NONPLANAR_OBJECT <id>` ... `; NONPLANAR_OBJECT_END`).
  - `NonPlanarExport::s4_mappers()` builds a mapper per deformed object in G-code coordinates
    (instance shift less plate origin in XY; Z offset and raft height in Z). It also records the
    height up to which the object is its print surface, and the window of an unwrapped layout.
  - `s4_transform_gcode()` maps each object through its own mapper. Its print surface layers
    and everything outside objects (skirt, brim, other objects) are printed as sliced.
  - A travel between toolpaths mapped differently was planned in unrelated spaces. It becomes a
    straight line in the part from where one print ends to where the next starts, and the
    travel pass lifts it over whatever is in the way.
  - Only the mapped G-code is then streamed through the processor, so the preview, time
    estimate and filament statistics describe the curved toolpath that will be printed.
  - With `polar_kinematics`, the processed file is converted to machine coordinates last, just
    before the export rename. The preview therefore stays Cartesian, and its G-code text view
    does not match the polar file line for line.
  - Clearance hits (with printed material or, for a leaning head, the bed) and radius travel
    overruns become slicing warnings.

## Tool

`s4_polar` (`src/dev-utils/`, built with `ORCA_TOOLS`) runs the pipeline around any slicer:
`deform` writes the deformed STL and the mesh pair (`.s4mesh`), `map` maps sliced G-code back
(detecting the part's placement from its own extrusion, skirt/brim/purge excluded) and with
`--polar` converts it, and `polar` converts Cartesian G-code alone.

## Tests

`tests/libslic3r/test_polar_kinematics.cpp` and `tests/libslic3r/test_s4.cpp` cover the kinematics
and the non-planar building blocks:
- **Polar kinematics:** round trip (also with a reversed tilt and pivot compensation),
  continuity and axis crossing, tilt and radius travel.
- **Polar G-code conversion:** path, extrusion, timing, machine blocks and their feed mode,
  arcs.
- **S4 deformation:** its guarantees (pinned base, planar base height, bed clearance, no
  inverted cells, limits and rotation direction), and the bounded rotation solver against
  closed-form solutions.
- **Meshing:** volume.
- **Layer shapes:** signed distance, the gap all around, the unwrap, cones.
- **Mapper:** identity, rigid tilt, squash, outside points.
- **G-code transform:**
  - identity, flow, travel lifts, absolute extrusion refused;
  - tilt threshold and limits;
  - toolhead clearance against printed material and the bed;
  - per-object mapping with flat print surfaces, and kept windows.

`tests/fff_print/test_nonplanar.cpp` slices through the whole pipeline:
- S4 curves layers, holds them flat below the planar height, and needs relative extrusion;
- polar export drives angle and radius, and tilts the nozzle over S4 layers;
- a dome printed over a print surface dome keeps the gap all around it, after the whole core.
