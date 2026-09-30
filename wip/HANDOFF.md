# Handoff: OrcaNP calibration suite (work in progress)

For a fresh session picking this up. Read `CLAUDE.md` (standing rules) first, then this file.
`wip/calibration/RESUME.md` is the long running log of the whole project (history, earlier releases,
decisions); read its last two sections. Everything under `wip/` is scaffolding: delete the folder in
the commit before the release commit.

State at hand-off: branch `claude/s4-slicer-parsing-tfwcjo`, commit 2ab97d91 (WIP), pushed. Last
release published: `orcanp-s4-test-16`. Next release: `orcanp-s4-test-17`.

## 1. What the user asked, in order

1. "Think of more useful in-slicer calibration tools for polar, non-planar, or standard. Can we add
   built-in filament shrinkage calibration? XYZ skew compensation similar to Calilantern, but of our
   own design? Polar pressure advance support?" -> I proposed the list in section 2.
2. "Add and test all of these ideas, bar the Extruder rotation distance/e-steps calibration tool."
3. "Bear in mind, bed sizes change. Support calibration on beds as small as 120mm square or round
   diameter. If multiple plates are required and would logically still work, split the test among
   plates for best-fit."
4. "Logically, some tests must remain on one plate."
5. "Test with the full matrix as before. Now unattended." (= the CLI release matrix of the test-14
   release, task #55, plus GUI tests of every calibration on each test printer, task #56.)
6. Weekly limit -> WIP commit; "Save context to repo, leave instructions for a fresh session".

Working mode: unattended, semi-autonomous, short user-facing messages, a quicklog at the end;
step-wise (build, suites, GUI test, commit + push each step); release when ready (section 6).

## 2. The calibrations (agreed scope and design)

Menu: Calibration > **Dimensions**, **Overhangs and gaps** (any printer), **Toolhead clearance**
(exists), **Polar calibration** (polar printers only; replaces "Polar alignment"). Each dialog is a
`CalibrationTabsDialog` (tabs; each tab: explanation, Create button, inputs, Apply button; inputs
remembered in the app config under the dialog's key).

| Tab | Print | Measure | Applies | Plates |
| --- | --- | --- | --- | --- |
| Dimensions 1. Shrinkage and skew | flat square frame (boss centres 100 mm apart, bosses 8 mm octagons) + upright frame 1 (along X) and 2 (along Y), 60 mm, windows with 45 degree gable tops (no bridges); sized to the bed by `square_frames_for(usable_area)`; uprights inside the flat frame when side >= upright + 40, else separate pieces | one boss across; AC, BD, AD across bosses (outside to outside) per frame | filament `filament_shrink` *= (sx+sy)/2, `filament_shrinkage_compensation_z` *= sz; printer `skew_xy/xz/yz` += fitted (not on polar) | may split |
| Dimensions 2. Holes and pegs | per diameter 5/10/15/20: a plate with the hole, a base with the peg, numbered (separate pieces) | diameters | process `xy_hole_compensation`, `xy_contour_compensation` += half the mean shortfall | may split |
| Dimensions 3. Elephant foot | 20 mm block | width at bottom and half way up | `elefant_foot_compensation` = max(0, old + (bottom-middle)/2) | one |
| Overhangs 1. Overhang limit | ledges 30..80 deg from vertical, numbered | steepest clean | process `s4_max_overhang` | one (compared) |
| Overhangs 2. Support gap | 5 tables, per-object `support_top_z_distance` = layer x {0.5,0.75,1,1.25,1.5}, `enable_support`, `independent_support_layer_height` on for the test | best gap | `support_top_z_distance` | one (compared) |
| Overhangs 3. Surface gap (S4) | 5 discs on generated pillars (per object: S4 on, offset, pillar, custom 20 mm x 4 mm, `s4_surface_gap` = the same gaps) | best gap | `s4_surface_gap` | one (compared) |
| Polar 1. Radius | existing two rings about the axis | outer diameters | radius offset/scale (existing solver) | one, centred on axis |
| Polar 2. Bed level | two rings 3 layers thick at 0.8R and 0.35R, numbers 1-4 / 5-8 at 0/90/180/270 deg; concentric patterns per object | thickness at 8 marks | printer `polar_bed_tilt_x/y`, `polar_bed_cone` += 100*fit (mm per 100 mm) | one |
| Polar 3. Backlash | G-code: two pairs of radial fins at +-40 deg, pair 1 both approached increasing angle, pair 2 opposite ways | width across each pair at outer ends | `polar_rotation_backlash` = max(0, old + (W2-W1)/r_out in deg) | one |
| Polar 4. Tilt pivot (tilt axis only) | G-code: two tubes about the axis, lower half vertical, upper half tilted +a (inner, leaning out) and -a (outer, leaning in); a = `tilt_pivot_test_angle` (<= 20, tip edge clears the layer) | lower/upper outer diameters of each tube, each tube's height | `polar_tilt_pivot_length`, `polar_tilt_offset`, `polar_radius_offset` (moved so vertical printing stays) | one |
| Polar 5. Rotation speed | G-code: 8 rings in one layer, outermost first, ring i at w_i from 0.5x to 3x the current max (capped so the innermost >= 4 mm), each leaving a 2 mm gap at +X; the export lifts the rotation cap to 1.2x the fastest ring for this test | last ring (from outside) whose gap lines up | `polar_max_rotation_speed` | one |
| PA on polar | PA line: lines beside the axis along +X, numbers on the -X side; PA pattern and all model calibrations arranged clear of a circle about the axis, radius = max print speed / max rotation speed | - | - | existing |

Test pieces measured on their own may be split across plates (`CalibrationKit::lay_out(pieces,
split=true)` -> `CalibrationPrints::place_on_plates`); compared samples stay on one plate (split=false);
everything centred on the rotation axis is one piece. Beds down to 120 mm square/round must work:
`usable_area()` = bed less 5 mm margin, or the square inside a round bed.

## 3. Code state at 2ab97d91

Built and compiling (the header rebuild finished with 0 errors):
- `PrintConfig.hpp/.cpp`, `Preset.cpp`, `Print.cpp` (G-code-only invalidation), `Tab.cpp` (Polar
  kinematics group + new "Axis skew" group, hidden on polar): settings `skew_xy/xz/yz` (deg),
  `polar_tilt_offset` (deg, machine units), `polar_rotation_backlash` (deg), `polar_bed_tilt_x/y`,
  `polar_bed_cone` (mm per 100 mm); tooltips point to the new dialogs.
- `calib.hpp`: `CalibMode::Calib_Polar_Backlash/Tilt/Speed` appended.
- `NonPlanar/PolarKinematics.*`: `tilt_offset` (commanded = lean*sign + offset), `bed_height()` added to
  z, backlash take-up in `emit_move` (commanded angle = nominal + dir*b/2, dir from the last angle
  change, reset at the first move after the start block/G28), durations use the commanded angle.
- `GCode/NonPlanarExport.cpp`: maps the new settings into the kinematics config (bed values /100).
- `tests/libslic3r/test_polar_kinematics.cpp`: round trip with the new fields, tilt offset, bed
  height, backlash taken up by a simulated bed with play.

Written, NOT registered in any CMakeLists yet (never compiled):
- `src/libslic3r/CalibrationPrints.{hpp,cpp}`: prism/box/octagon/circle/slab builders, 7-segment
  labels (digits . - A b C d), `inscribed_radius`, `usable_area`, `place_on_plates`, frames
  (`square_frames_for`, `square_frame_pieces`, `fit_square_frames`, `skew_angle` = Klipper),
  `hole_and_peg_pieces`, `fit_xy_compensation`, `elephant_foot_block`, `overhang_samples` (pieces),
  `support_gap_sample`, `surface_gap_sample`, `lay_out` (rows, used by the toolhead gauges today).
- `src/libslic3r/GCode/SkewCompensation.{hpp,cpp}`: `SkewCorrection` (Klipper: x -= y*xy + z*(xz -
  xy*yz), y -= z*yz), `SkewGCodeConverter` (body G0/G1 sheared, G2/G3 linearized, start/end blocks
  verbatim, relative moves sheared by delta, out_lines per input line, outside-bed count).
- `src/libslic3r/GCode/PolarCalibrationGCode.{hpp,cpp}`: `is_polar_calibration`,
  `polar_test_params(DynamicPrintConfig)`, `polar_speed_test_max_linear_speed`,
  `polar_calibration_gcode(GCode&, Print&)` (writes TestPrint moves with the writer, layer tags, tilt
  word as its own "G1 B.." line, retract on travels > 2 mm, fan from layer 2). Calib params: Tilt
  start = tilt angle; Speed start/end = slowest/fastest ring deg/s, step = linear speed mm/s.
- `src/libslic3r/NonPlanar/PolarCalibration.{hpp,cpp}`: bed-level rings mesh + LSQ fit; backlash test
  paths + fit; tilt pivot paths + fit; rotation speed plan/paths; `polar_bed_radius` (duplicate of
  `CalibrationPrints::inscribed_radius`: replace it, see step 5).
- GUI: `CalibrationDialogKit.{hpp,cpp}` (base dialog + `CalibrationKit::` helpers: configs, apply,
  number, bed_centre, is_polar, usable_area, lay_out, open_test_print with plates),
  `DimensionsCalibration.*`, `OverhangCalibration.*`, `PolarPrinterCalibration.*`.
- Tests: `tests/libslic3r/test_calibration_prints.cpp`, `test_skew_compensation.cpp`,
  `test_polar_calibration.cpp` (simulated machine errors through the real kinematics: fits must
  recover backlash, pivot length, tilt offset), `tests/fff_print/test_calibration.cpp` (skew export,
  polar test G-code: 30 layers of fins, ring speed > 1000 deg/s, tilt +-15).
- `wip/calibration/ToolheadCalibration.{hpp,cpp}`: the Toolhead clearance dialog rewritten on the base
  dialog (same app-config keys); `wip/calibration/patch_toolhead_lib.py` moves the gauges onto the
  shared CalibrationPrints helpers and moves the label tests.

## 4. Remaining steps, in order

1. Register sources: `src/libslic3r/CMakeLists.txt` (CalibrationPrints, GCode/SkewCompensation,
   GCode/PolarCalibrationGCode, NonPlanar/PolarCalibration), `src/slic3r/CMakeLists.txt`
   (CalibrationDialogKit, DimensionsCalibration, OverhangCalibration, PolarPrinterCalibration; remove
   GUI PolarCalibration.{hpp,cpp} and delete them), tests CMakeLists (three libslic3r files, one
   fff_print file).
2. Toolhead: copy `wip/calibration/ToolheadCalibration.*` into `src/slic3r/GUI/`, run
   `patch_toolhead_lib.py`. Then change `feeler_blades` / `angle_wedges` to return pieces (drop
   `max_width` and `lay_out`) and use `CalibrationKit::lay_out(pieces, true)` in the dialog (replace
   `layout_width()`, removed from the kit); give `fin_test` a max radius (bed's inscribed radius - 2)
   and clip the plate and fins to it, saying in the tab when the tall outer fins do not fit. Update
   `test_toolhead_clearance.cpp` (pieces, not laid-out meshes).
3. Dialog fixes: Dimensions `on_create_frames` -> `square_frames_for(usable_area())`, remember side and
   upright in the app config and fit with them; holes/pegs -> `hole_and_peg_pieces` + `lay_out(...,
   true)`; Overhangs -> `lay_out(overhang_samples(...), false)`, support/surface samples via
   `lay_out(meshes, false, configs)` (delete the local `row()`); remove all `layout_width()` uses.
4. `MainFrame.cpp` (both calibration menus, near "Toolhead clearance"): Dimensions, Overhangs and gaps
   (view3D shown), Polar calibration (view3D and `polar_printer_selected()`), Toolhead clearance;
   include the new headers instead of PolarCalibration.hpp.
5. Replace `NonPlanar::polar_bed_radius` with `CalibrationPrints::inscribed_radius` (PolarCalibration,
   PolarCalibrationGCode, its test).
6. `GCode.cpp`: include the two new GCode headers; after the `Calib_PA_Line` branch add `else if
   (is_polar_calibration(print.calib_params().mode)) file.write(polar_calibration_gcode(*this, print));`;
   in the polar export block build the kinematics config first and, for `Calib_Polar_Speed`, raise
   `max_angular_speed` to `max(it, 1.2 * 60 * params.end)`; after the polar block, for `!polar` with a
   non-empty `SkewCorrection`, run `SkewGCodeConverter` over the processed file, remap the preview
   with a new `NonPlanarExport::remap_preview_lines(result, out_lines, path)` (factor the gcode_id and
   lines_ends part out of `map_preview_to_polar`), warn when moves end outside the bed.
7. `calib.cpp` polar PA line: in `generate_test` and `print_extents`, for `polar_kinematics`, lines run
   from axis+3 mm along +X (fast segment away from the axis), lengths/count fitted to the inscribed
   radius (fit a round 120 mm bed: >= ~12 lines), numbers box drawn on the -X side after
   `print_pa_lines` (turn `m_draw_numbers` off for that call; `draw_box`/`draw_number` are protected
   members, callable from `generate_test`). No calib.hpp change (it rebuilds ~600 files).
8. `Plater.cpp`: on polar printers arrange calibration objects clear of a circle about the axis (use
   `arrangement::arrange(items, excludes, bedpts, params)` with the circle as an exclude); call it
   where `calibration_objects_planar(model())` is called; in `_calib_pa_pattern` pass the same exclude
   to its arrange call.
9. Build (`cd build && ninja -f build-Release.ninja -k 0 fff_print_tests libslic3r_tests
   src/Release/orca-slicer`, only .cpp changes now: minutes, not an hour), fix compile errors, run
   `[CalibrationPrints]`, `[SkewCompensation]`, `[PolarCalibration]`, `[PolarKinematics]`,
   `[Calibration]`, `[ToolheadClearance]`, then both suites (`ctest --test-dir build/tests/libslic3r -C
   Release -j2`, same for fff_print). Commit + push per working step.
10. GUI end-user tests (section 5), CLI matrix, then README (new "## Calibration" section: Dimensions,
    Overhangs and gaps, Polar calibration moved from "Polar alignment", PA on polar; settings table rows
    for the new settings; screenshots under docs/images/nonplanar/) and `docs/HLSD/calibration.md`
    (the measurement designs and fits of section 7, which the code does not make evident); update the
    Polar alignment references (README, tooltips done).
11. Release test-17 (section 6), then the quicklog.

## 5. Testing

- Unit/fff: as step 9. The fits are checked against simulated machines; keep that style for any new fit.
- CLI matrix (like test-14's): `wip/tools/matrix/run_matrix.py` slices printers x models x settings and
  checks each G-code (`check.py`); printers: Core R-Theta (was "ThetaFirm Core R-Theta" in the script:
  update names), Generic Polar Printer, a Klipper Cartesian, Voron 2.4 300 (test toolhead 2.2 deg /
  32.75 mm / 1.24 mm), Bambu X1C; add skew on a Cartesian and the new polar corrections on the Core
  R-Theta; models in `wip/tools/clitest/` (large Benchy STLs were not copied; 3DBenchy.drc is there).
  Paths in the scripts point at the old scratchpad: set `S` to your scratchpad and copy the tools there.
- GUI (Xvfb): `Xvfb :99 -screen 0 1600x1000x24 -fbdir <fb> -nolisten tcp`, launch the app with
  `DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 ... --datadir <dir>` (`wip/tools/gui/launch.sh`), drive with
  `wip/tools/xgui.py` (screenshots from the framebuffer, XTest clicks/keys), `gui/cal.py` opens a
  Calibration menu item by y position (items shift as menu entries are added: re-measure), `typein.py`
  types values. Dropdowns do not render under Xvfb: set values through presets/config. Kill with
  `pgrep -f "^/home/user/OrcaNP/build/src/Release/orca-slicer"`. User presets used by the tests are in
  `wip/tools/datadir_user/` (copy into `<datadir>/user/default/`).
- GUI matrix: every tab of every dialog: Create (check the project: pieces, plates, per-object settings,
  slice it, look at the preview), enter plausible readings, Apply (check the preset changes and the
  result text), on: Voron 2.4 300, Bambu X1C, Core R-Theta, Generic Polar Printer, and 120 mm square
  and 120 mm round beds (edit a printer's printable area). Polar tests: check the machine G-code (C/X/B
  words) and the preview. PA line/pattern/tower on the Core R-Theta: patterns clear of the axis.

## 6. Releasing

First line of `.github/test-release-win64` is the tag (`orcanp-s4-test-17`); notes live in
`.github/workflows/test_release_win64.yml` (keep the structure: "New in this build", "Fixed since the
previous test build", "Earlier features", "Known limitations"; preset names with "@" in code spans).
Commit only `.github/` in the release commit, after the tested code is pushed; the run takes ~30-60 min.
Check it with the GitHub MCP tools (`actions_get` get_workflow_run, `actions_list` list_workflow_jobs,
`get_job_logs`, `get_release_by_tag` for both assets). The workflow also has `workflow_dispatch`: re-run
by hand after a code-only fix. Test-16's first run failed only on a Windows-only AppConfig test (the
Windows loader needs a trailing newline): Windows CI runs every suite, so watch for platform behaviour.

## 7. Design notes (the maths, so it need not be derived again)

- Skew (Klipper): side = sqrt(2AC^2 + 2BD^2 - 4AD^2)/2, A = acos((AC^2 - side^2 - AD^2)/(2 side AD)),
  skew = 90 - A (positive: the second axis leans toward the first). Readings are across bosses:
  subtract the measured boss width. Printed with skew already set, the frames show the rest: add.
- Shrinkage: printed scaled by the current setting, measured/designed = remaining factor: multiply.
- Backlash: the bed lags its motor by up to b/2; the part angle printed = the bed's. Pair 2's fins
  (approached from opposite sides) spread by (b_true - b_set) * r_out; W2 - W1 = that. Add to setting.
- Tilt pivot (L = pivot length, e = lean error, a = test tilt): relative to vertical printing, the tip
  lands (L_set - L) sin t + L e (1 - cos t) further out and (L_set - L)(cos t - 1) + L e sin t higher.
  So dL = ((dA_up - dA_low) - (dB_up - dB_low)) / (4 sin a), L_new = L_set - dL;
  e = (hA - hB) / (2 L_new sin a); tilt_offset_new = offset - deg(e) * tilt_sign;
  radius_offset_new = radius_offset + L_new sin e (vertical printing had been absorbing -L sin e).
- Bed level: thickness_i = t0 - (a x_i + b y_i + c r_i), least squares over the 8 marks; settings are
  per 100 mm and additive.
- Rotation speed: ring radius r_i = v / w_i; rings outer to inner, so a lost step turns every later
  ring's gap; the export's cap is raised for this test only.
- Polar PA: lines along a radius beside the axis keep the bed nearly still during the fast segment
  (w = v y / r^2 stays low); across the axis the bed would have to turn faster than its limit.
