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
   soup is re-oriented first, sharp edges are protected, and slivers are removed by exudation;
   slivers are nearly flat and the deformation turns them inside out easily. CGAL's sliver
   perturbation is left out, since exudation alone leaves as few inverted cells and fewer
   overhangs. Exudation runs without a time limit: by default CGAL stops it after as long as the
   refinement took, so the mesh, and the slice, depended on how busy the machine was.
   The mesh is graded by default:
   tetrahedra of the surface cell size at the surface, which they follow to within a twentieth
   of it, growing to the interior cell size inside. Walls are printed where the mesh's skin is,
   so a coarse skin moves them (on a 100 mm part with the uniform default of 1/20 of its largest
   side, 3 % of outer wall points ended up over 0.5 mm off the model); the inside only carries
   smoothly varying layers and can stay coarse. A uniform mesh of one cell size (by default
   1/20 of the largest bounding box side, following the surface to a tenth of it) remains
   available. The balls protecting sharp edges shrink where edges come close and stop at a
   quarter of the surface cell size: fine detail (a Benchy's lettering) would otherwise shrink
   them practically forever.
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
     M-matrix, so a primal-dual active set method converges; each of its steps is a sparse
     solve, and on fine meshes it takes dozens of steps, the bulk of the slicing time.
   - Size weights (graded mesh): each cell's terms count in proportion to its size (the shape
     term by volume, the target by area, the link between neighbours by their mean edge
     length, all scaled to a mean of 1). Unweighted, the many small surface cells of a graded
     mesh outvote the few large inner ones, which stiffens the skin and distorts the inside;
     weighted, a graded mesh solves the problem a uniform one does, so the settings keep their
     meaning.
   - Deformation: vertex positions whose cells best match their rotated, centred shapes, a
     linear least-squares problem whose axes decouple. Vertices on the bed are pinned; all
     other vertices stay at least `min(height, bottom_threshold)` above the bed (an active set
     on the Z solve), because a region pushed down to bed level would be sliced into the first
     layers and printed in mid-air.
   - Inversion repair: cells turned inside out get their limits, and those of their two-ring
     neighbourhood, cut to 70 % and the pass is solved again, up to ten times. Zero rotation
     cannot invert, so this converges. With warm start each of these rounds starts the active
     set method from the last round's solution and its binding limits: the optimum is unique,
     so only the number of steps changes (447 to 49 on a 102 mm Benchy with 2 mm cells).
   - Multi-threading: the three axes of the deformation are solved at once. The rotation field
     is factorized directly while that is cheap; once its first factorization has more than
     3 million nonzeros it is solved by Jacobi-preconditioned conjugate gradients, warm-started
     from the last step, over threads. The work is split in fixed chunks and summed in a fixed
     order, so the result does not depend on the number of threads, and the switch depends on
     the problem's size, not on a timing, so a model always takes the same route. Below the
     switch the direct solve is faster; above it the iterative one is, the more so with more
     cores. Warm start and multi-threading are application preferences, read at startup.
   - Planar base (`planar_height`): the part up to that height keeps its shape and plays the
     bed's role. Its vertices are pinned, its cells get a zero limit (so the rotation field
     starts from zero at its top), its cells are the Dijkstra sources, and the Z floor keeps the
     rest above it. The base prints with ordinary flat layers, blending into the curved ones over
     the tetrahedra that reach above the base's top (about one cell).
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
  - The core is printed first, so it must stand on the bed; a core standing on the part would
    start in mid-air and is refused. All of the part is layered by its distance from the core,
    including what stands beside the core on the bed, where the layers stand vertical. So the
    shape suits parts printed over their core (domes, spheres, cups upside down, sleeves), not a
    core propping up one overhang of a part (a bracket's arm): the rest of that part, printed
    after the core in shells around it, would need a horizontal nozzle next to the bed, which
    the clearance check reports.
  - The core is either the object's parts set as print surface, or generated to fit the part:
    - A **sphere** centred below the part's top by half its width, out to the part's nearest
      point.
    - A **cylinder** about the rotation axis, from the bed to the roof of the part's cavity, out to
      its nearest wall below the roof.

    - A **pillar**, **dome** or **domed pillar** under the part's base (the middle of its faces
      on the bed): a cylinder, a hemisphere, or a cylinder topped by a hemisphere as wide. Its
      size is automatic (as wide as the base, a pillar as high as the toolhead needs to lean as
      far as the tilt axis goes under the part without reaching the bed) or custom: a diameter,
      and a pillar's height, in mm. The part is lifted onto it, the gap above it: column by column
      over it, the part's lowest point must clear its top by the gap, so a concave underside
      nests on a dome and a part with a cavity over it sits no higher than it has to. Off the bed,
      the toolhead can lean under the part, where the layers wrap round the rim; the pillar is
      what a raised build platform is on the reference machine.

    The sphere and the cylinder are shrunk by the gap, so the part's first layer is its
    modelled inner surface. A generated core is sliced with the part (and printed with its
    settings). A cavity that does not reach the bed, is too small, or (for a cylinder) does not
    surround the axis is refused: the core must stand on the bed to be printed first.
  - The distance is exact (a uniform grid of the core's triangles, and a vertical ray for
    inside/outside). The core's faces on the bed close it but are not measured from: nothing is
    printed over them, and counting them would pull the layers meeting the bed into the core.
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
- **Conical** (the radial slicer's layers): z' = z + max(0, tan(angle) (r - r_flat)) about the
  rotation axis. Pure cones meet a flat bottom only near the axis: the first layer would be a
  speck (an empty first layer) and the rest of the footprint printed ring by ring in later
  layers, over the skirt. So the layers are flat over the radii at which the part stands on the
  bed (its faces within half the first layer of its lowest point) and conical outside them:
  r_flat is the farthest of those radii for a positive angle (layers descending away from the
  axis beyond it) and the nearest for a negative one (descending towards the axis within it). A
  part standing on the axis at a point gets pure cones.

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
  shape (`s4_layer_shape`), the S4 deformation parameters, the surface gap and layout, the
  cone angle, and the mesh: graded (`s4_graded_mesh`, `s4_surface_cell_size`,
  `s4_interior_cell_size`) or uniform (`s4_cell_size`, shown only when the mesh is not graded).
  Any change re-slices the object.
- **Preferences > General > Non-planar slicing**: warm start and multi-threading of the S4
  solver (`s4_warm_start`, `s4_multithreading` in the application's configuration), on by
  default; they change the time, not the result, and are read at startup.
- **Print surface** (`s4_surface_core`): what offset layers are offset from. Either the parts set
  as print surface (`s4_print_surface`, a per-part setting, set from a part's context menu with
  *Print surface (non-planar)* or among its settings), or a generated sphere, cylinder, pillar,
  dome or domed pillar (`s4_surface_size`, `s4_surface_diameter`, `s4_surface_height`). A part
  on a pillar is lifted only in slicing: the plater shows it on the bed, the preview where it
  prints.
  OrcaSlicer places every object on the bed, so a modelled core and what is printed over it are
  two parts of one object. Orca's check for overhangs needing support is skipped for offset
  layers: they lie on the print surface, not on the layers sliced below them.
- **Printer settings > Basic information > Polar kinematics** (`polar_*`): `polar_kinematics`, the
  axis letters and directions, axis crossing, the radius and tilt travel, the tilt threshold,
  and the conversion and speed limits.
- **Printer settings > Basic information > Non-planar toolhead** (`nonplanar_*`): the clearance
  check and what it models. The nozzle tip's flat diameter and the angle of absolute clearance
  (rising from the tip's face, 0 degrees flat) define the nozzle cone. The nozzle length, where
  the toolhead's radius takes over, and that radius complete it.

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
  toolpath (`; NONPLANAR_OBJECT <id>` ... `; NONPLANAR_OBJECT_END`, the id being the object's
  place in the print).
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
    before the export rename. The converter records what became of each line: the last machine
    line written for it and the machine pose after it. `map_preview_to_polar()` gives the
    processor result the pose at the end of each move and moves the moves' line ids and the
    line ends to the machine G-code, so the preview's G-code window shows what the machine runs.
  - Clearance hits (with printed material or, for a leaning head, the bed) and radius travel
    overruns become slicing warnings.
- **Preview** (`GCodeViewer`): the toolpath stays in the part's Cartesian frame. For a non-planar
  print (S4 or polar) the processor result also carries the toolhead of the clearance check
  (`GCodeProcessorResult::NonPlanarPreview`), and for a polar printer the machine poses.
  - The tool marker leans with the nozzle, and a see-through clearance body is drawn along its
    axis at the tip: the cone from the flat tip up to the nozzle length, and the head above it
    as a cylinder two of its radii tall.
  - The nozzle's axis comes from the machine pose, not from the tilt in the part: the head's
    radial line in the part is at the machine angle over its sign, and the nozzle leans in that
    plane by the machine tilt times its sign. This is well defined at the rotation axis too.
  - *Turn bed* draws the bed, the plate and the toolpath turned about the rotation axis by the
    bed's turn at the current move, so the head stays on its rail along +X as on the machine.
    It is a transform of the camera's view for the preview's scene passes only (the camera's
    view matrix is swapped for the while), so nothing is re-uploaded as the slider moves.
  - The marker's window adds the machine pose (bed angle, radius, tilt, as commanded) and the
    *Toolhead* and *Turn bed* switches, which are kept in the app config.
  - Play and pause buttons at the moves slider's left end play the toolpath back: each frame
    spends the elapsed time times the speed (10x to 1000x) of the moves' estimated print time,
    stepping the moves slider through the top layer, then the layers slider up one layer, and
    stopping at the end of the print. Played to the end, it starts again from the first layer.

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
  arcs, and the record of each line's last machine line and pose.
- **S4 deformation:** its guarantees (pinned base, planar base height, bed clearance, no
  inverted cells, limits and rotation direction), the bounded rotation solver against
  closed-form solutions, warm start and multi-threading (the iterative route forced) giving
  the same deformation in fewer solves, and size weights of 1 on a uniform mesh.
- **Meshing:** volume; a graded mesh fine at the surface and coarse inside.
- **Layer shapes:** signed distance, the gap all around, the unwrap, cones, fitting sphere and
  cylinder cores.
- **Mapper:** identity, rigid tilt, squash, outside points.
- **G-code transform:**
  - identity, flow, travel lifts, absolute extrusion refused;
  - tilt threshold and limits;
  - toolhead clearance against printed material and the bed;
  - per-object mapping with flat print surfaces, and kept windows.

`tests/libslic3r/test_s4_quality.cpp` is a benchmark, hidden from normal runs (`[S4Quality]`):
on a model given by `S4_QUALITY_MODEL` it compares mesh and solver variants
(`S4_QUALITY_VARIANTS`) by how closely the mesh's skin follows the model, the share of the
surface still overhanging, the layer tilt inside against a reference, and time.

`tests/fff_print/test_nonplanar.cpp` slices through the whole pipeline:
- S4 curves layers, holds them flat below the planar height, and needs relative extrusion;
- polar export drives angle and radius, and tilts the nozzle over S4 layers;
- the preview of a polar print has each move's machine pose, and its line is the machine move
  that ends it;
- a dome printed over a print surface dome keeps the gap all around it, after the whole core,
  whichever of the two parts was added first; a print surface above the bed is refused;
- a hollow dome gets a generated core and its first layer on its inner surface;
- a block printed on a pillar stands the gap above it, lifted off the bed, its corners beyond
  the pillar; an automatic pillar is as high as the toolhead needs; a dome is a hemisphere.
