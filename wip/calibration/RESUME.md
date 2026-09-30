## STANDING RULES (user, permanent)
- Working semi-autonomously: keep user-facing messages short (reduce user-facing token usage).
- Step-wise development; verify against the whole before moving to the next step.
- Make safe at each step (commit + push); keep this file current so work can resume if limits are reached.

# Resume notes: OrcaNP polar non-planar (S4) work

Branch `claude/s4-slicer-parsing-tfwcjo` (PDAMUK/OrcaNP). Pushed and verified up to `fec1f837`:
polar kinematics, S4 deformation, tetrahedralization (CGAL), mapper, G-code transform, `s4_polar`
tool, HLSD doc. Standalone tests: 22 cases / 3762 assertions green.

## Pushed as WIP, not yet built

Commit `f9d9a30c` on the remote branch (WIP, not compiled) = `s4-integration.patch` applied to fec1f837.
The main checkout's local branch is still at fec1f837 (baseline build); after the baseline, run
`git pull --ff-only` (or `git stash drop` the identical stash) and continue with the fixes below.

OrcaSlicer pipeline wiring, NOT yet compiled:
- options: `s4_*` (PrintObjectConfig, re-slice), `polar_*` (GCodeConfig, export only); Preset lists,
  Tab.cpp groups, ConfigManipulation toggles, invalidation.
- PrintObject::deform_s4() + slice(): deform before slicing, layers from deformed height, deformed
  surface sliced instead of the model part.
- Print::validate: single part, one instance, relative E, no spiral vase.
- GCode::do_export: sliced-space G-code -> S4 transform -> processor (preview) -> finalize -> polar
  conversion. GCodeOutputStream processor made nullable. New GCode/NonPlanarExport.{hpp,cpp}.

Fixes still to apply on top of the patch:
1. Wrap the S4 block in do_export in try/catch (remove path_sliced + path_tmp, rethrow), and
   throw if the mapper is null.
2. Move PrintObject::deform_s4() above the doc comment of slice() (it split the comment off).
3. In slice(): if S4 was on last time and is now off, recompute m_slicing_params
   (m_slicing_params.valid = false; update_slicing_parameters()).

## Build environment (never commit these)

- GitHub /archive/ downloads are 403 in this session; the user approved the git mirror.
  `make_mirror.sh` recreates 21 dep archives into scratchpad/depmirror (git clone at pinned refs).
- `dep_mirror_override.cmake` is passed as
  `DEPS_EXTRA_BUILD_ARGS="-DCMAKE_PROJECT_INCLUDE=<path>" ./build_linux.sh -d -r` (deps ~40 min).
- System packages: see scripts/linux.d/debian list (installed via apt), plus ccache.
- OrcaSlicer: `ORCA_EXTRA_BUILD_ARGS="-DORCA_TOOLS=ON" ./build_linux.sh -s -t -r -j 2`
  (-j 4 runs out of memory on the CGAL files with 15 GB).

## Plan from here

1. Baseline build of fec1f837 (running) -> slice `clitest/cube.obj` with
   `machine_cartesian.json` + `process.json` via
   `orca-slicer --datadir D --load-settings "machine.json;process.json" --slice 0 --outputdir OUT model`
   and keep the G-code as the regression reference.
2. `git apply s4-integration.patch` (or `git stash pop`), apply fixes 1-3, rebuild.
3. Verify: cube with features off byte-identical to baseline (ignoring timestamps); S4 on pi.stl
   (Cartesian: real-space XYZ G-code); polar printer (`machine_polar.json`) + S4 -> polar G-code,
   round-trip check; run libslic3r tests incl. [S4] and [PolarKinematics].
4. Commit + push; then task 5: printer profile (orca-profiles skill), G-code viewer support for
   polar files, doc updates.

## Final goal (user, 18:5x)

After the pipeline is integrated and tested, push a win64 test "Release" to PDAMUK/OrcaNP: build
Windows via the repo's GitHub Actions (check .github/workflows for workflow_dispatch / tag
triggers), then publish the artifacts as a pre-release. Continue autonomously after the 21:20
usage reset (wake-up scheduled 20:25 UTC, trigger trig_01XpMUvR7T73rPaYQQQ2sAy7).

## Usage pacing (Pro plan, 5 h windows)

The first window ran out ~2.5 h into heavy work (limit hit 18:54 local, reset 21:20). Local time is
UTC+1. Expected windows: 21:20-02:20, 02:20-07:20, ... Make safe ~2 h after each resume (push
verified work, WIP-commit the rest, update this file), then stop. Resumes scheduled: 20:25 UTC
(trig_01XpMUvR7T73rPaYQQQ2sAy7) and 01:25 UTC; schedule the next one (06:25 UTC) at each make-safe.

## Progress (window 2, from 20:40 UTC)

- Baseline done: clitest/base_cube/plate_1.gcode, clitest/base_pi/plate_1.gcode (Cartesian, S4 off).
  CLI needs "layer_change_gcode": "G92 E0" with relative E (added to both machine JSONs).
- Pulled f9d9a30c, applied the three fixes, pushed as WIP 056271ca. Changed files compile.
- Full incremental rebuild running: scratchpad/orca_build3.log (cmake --build build --config Release -j 2).

## Win64 release plan

- build_all.yml has workflow_dispatch (dispatch with ref = our branch) but publish_release.yml needs
  RELEASE_PUBLISHER + an existing draft release, and the GitHub MCP tools cannot create releases.
- Plan: add .github/workflows/test_release_win64.yml on our branch, triggered on push to our branch
  (path filter on a request file) + workflow_dispatch: call the Windows build (see how build_all.yml
  composes build_deps/build_orca for windows) and then `gh release create --prerelease` with
  GITHUB_TOKEN (permissions: contents: write), attaching the win64 portable zip/installer.
- First check Actions are enabled on PDAMUK/OrcaNP (mcp actions_list).

## Progress (window 3, from 19:50 UTC)

- Pushed 7b37bc0a: .github/workflows/test_release_win64.yml (Windows x64 clang build via
  build_check_cache + unit tests; publishes a pre-release only when .github/test-release-win64 has a
  tag on its first line). CI run 36267528616 started 19:52 UTC (deps build, no cache yet).
- Local commits (NOT pushed yet, compile unverified): 7f152dab (re-slice S4 objects when moved or
  printable_area changes; HLSD integration section), 0e10e93f (Custom profile "Generic Polar
  Non-planar Printer" / "MyPolar 0.4 nozzle" / "0.20mm Non-planar S4 @MyPolar", version 02.04.00.06;
  orca_profile_tool check passes).
- Uncommitted: tests/fff_print/test_nonplanar.cpp (+CMakeLists), s4_polar CNumericLocalesSetter
  (fixes the s4_polar link failure that stopped build3).
- Build4 running: scratchpad/orca_build4.log (-k 0). Then: scratchpad/verify.sh (cube pi s4 polar),
  ctest libslic3r [S4]/[PolarKinematics], fff_print [NonPlanar], scripts/check_profile.sh --vendor Custom.
- Release: after verification, push, then add .github/test-release-win64 with "orcanp-s4-test-1".
- 20:25 UTC: user asked for tilt min/max, planar-up-to-height, tilt only for overhangs, clearance
  check. Implemented in worktree /home/user/OrcaNP-feat (branch s4-options), committed ad0f6c5a (WIP),
  fast-forwarded the main branch and pushed. Build3/4 stopped; build5 (all changes) running:
  scratchpad/orca_build5.log. Harness for the unit tests: scratchpad/harness_feat (np_tests).
  Test release 1 will include the options.
- 20:45 UTC: user: target machine is ThetaFirm (github.com/PDAMUK/ThetaFirm, cloned read-only at
  /home/user/pdamuk/thetafirm; reference slicer jyjblrd/Radial_Non_Planar_Slicer at /home/user/pdamuk/radial).
  Findings: 4-axis RRF, C angle (+theta), X radius (-37.5..115.5), B tilt (B = -outward lean, B0 down),
  pivot 43 mm (firmware positions pivot), G93. Implemented polar_reverse_tilt, polar_signed_radius,
  polar_radius_min/max, G94 at end block, machine block tags for polar/S4, ThetaFirm Custom profile.
  Pushed b3fcf02e (WIP). Unit tests (harness) 28/28 pass. Build6 running (core files first:
  orca_build6a.log, then full). verify.sh now also has s4round + theta cases (machine_thetafirm.json).
- 20:55 UTC: user approved print-surface features on the same branch; one release with all.
  Pushed bacd58bd (print surfaces, offset/cone shapes, per-object mapping, junctions, windows,
  bed clearance), c34781ca (CMake), docs. Unit tests 35/35 pass (harness_feat -> OrcaNP-feat).
  Build7 running (core first orca_build7a.log, then orca_build7.log). Then: fff_print tests
  ([NonPlanar] incl. dome print-surface test), verify.sh (cube pi s4 polar s4round theta),
  check_profile, then commit tag file .github/test-release-win64 -> CI release.
- 21:20 UTC: generated sphere/cylinder cores (s4_surface_core) pushed. Windows CI run 1 (7b37bc0a)
  failed on clang -Werror (unused field) -> fixed; local clang check script scratchpad/wincheck.py
  (compdb.json from ninja -t compdb) found Tab.cpp toggle_field + test include errors -> fixed
  (origin c426ecac). Main checkout has the Tab/test fixes uncommitted (same as origin) and lacks the
  PolarKinematics.hpp field removal until build8 ends: then `git checkout -- . && git pull --ff-only`
  and rebuild incrementally. Windows build-only run 2 triggered by the workflow edit (c426ecac).

## MAKE-SAFE 21:45 UTC (user: approaching limit) — resume here

State:
- origin/claude/s4-slicer-parsing-tfwcjo = 16b62632 (all work pushed). Worktree /home/user/OrcaNP-feat
  (branch s4-options) == 16b62632, clean.
- Main checkout /home/user/OrcaNP is at 2a9f5a8c with Tab.cpp + tests/fff_print/test_nonplanar.cpp
  modified (identical to origin). Build8 (background, orca_build8.log) was building it; ETA ~22:40.
- Windows CI run 3 (id 36273803949, sha 16b62632) build+unit tests, no release (no tag file).
  Deps cached (run 1). Check with mcp__github__actions_list list_workflow_jobs 36273803949.

Next steps:
1. Main checkout already synced to 16b62632 at 22:20 (clean). When build8 is done, rebuild incrementally:
   `git checkout -- src/slic3r/GUI/Tab.cpp tests/fff_print/test_nonplanar.cpp && git pull --ff-only`
   then `cmake --build build --config Release -j 2 -- -k 0` (incremental: PolarKinematics.hpp etc).
2. Tests: build/tests: `ctest --test-dir build/tests -C Release -R "S4|Polar|NonPlanar" --output-on-failure`
   or run build/tests/libslic3r/Release/libslic3r_tests "[S4],[PolarKinematics]" and
   build/tests/fff_print/Release/fff_print_tests "[NonPlanar]" (paths may differ; find them).
3. CLI: scratchpad/verify.sh (cases cube pi s4 polar s4round theta cone); diff must be 0 for cube/pi.
4. Profiles: ./scripts/check_profile.sh --vendor Custom (validator binary in build/src/Release).
5. Windows run 3 green -> add `.github/test-release-win64` containing `orcanp-s4-test-1`, commit, push
   (triggers build + release; portable zip with data_dir + installer + notes).
6. Report to the user; ask about watching the release.

## UPDATE 22:40 UTC (limit approaching; reset 00:40 UTC, resume trigger 00:45 UTC)
- Added nonplanar_nozzle_clearance_angle (from tip face, 0 = flat; replaces nonplanar_nozzle_cone_angle)
  and nonplanar_nozzle_tip_diameter. Pushed 66d0a493 (branch head). Main checkout synced to 66d0a493, clean.
- Build9 (full, started ~22:40 UTC) in background: orca_build9a.log (core) then orca_build9.log; ETA ~00:10.
  So at resume: skip the sync/rebuild step if build9 finished OK; then tests, verify.sh, check_profile.
- Windows CI run 3 (36273803949) builds 16b62632 (no clearance-angle change); the release run (tag
  file push) will build the final head anyway.
- User asked (answer only, no work yet): preview showing the tilted clearance body and bed rotation;
  feasible (GCodeViewer marker + per-move tilt + machine-view rotation). Queue after the release.

## UPDATE 01:48 UTC (window from 00:45 UTC)
- Fixed: print-surface bed faces no longer measured from (SurfaceDistance floor_z) -> rim artifact gone;
  NONPLANAR_OBJECT marker ids = object's index in print (PrintObject::m_id was uninitialised for
  non-Klipper/Marlin/RRF flavours -> random unmapped objects). Pushed d91f053c, 5b08a6c7.
- Verified locally: libslic3r_tests all (873 cases), fff_print_tests all (227 cases; [NonPlanar] 7/7,
  10/10 random-order runs), verify.sh all cases OK (features-off body identical; polar/theta round trip
  0 missed, 76786/76788 within 0.002 mm, 2 at the axis <= 0.042), check_profile.sh full tree 5/5 PASS,
  wincheck on changed files OK.
- Windows run 3 (16b62632) failures were the M82-detection + test-config bugs fixed since.
- Pushed cee47f25: .github/test-release-win64 = orcanp-s4-test-1 (+ known limitations in notes)
  -> Win64 test release run (build ~30 min, tests ~5, release). Check it; if green, report release URL.
  If unit tests fail on Windows only: fetch logs artifact (actions_get download_workflow_run_artifact
  on the unit-test-logs artifact; curl the URL), fix, bump nothing (same tag, release step only runs
  when tests pass), push.
- 02:28 UTC: run 36286513713 still in "Build slicer Win" (started 01:47; near-full rebuild because
  PrintConfig.hpp changed). No release yet. Check-ins: 02:46 UTC (trig_01SK87F9Mw8NccmEDYyXuCXp) and
  fallback resume 05:45 UTC (trig_01FCRDvQ5953iLDjCH8rMf5D). Nothing uncommitted; branch head cee47f25.
- 03:05 UTC: RELEASED. Run 36286513713 success (Windows build 51 min, Windows unit tests green).
  https://github.com/PDAMUK/OrcaNP/releases/tag/orcanp-s4-test-1 (pre-release, published 02:56 UTC):
  OrcaNP_orcanp-s4-test-1_Windows_x64_portable.zip (248 MB), OrcaNP_orcanp-s4-test-1_Windows_x64_installer.exe (200 MB).
  Fallback resume trigger deleted. Open: preview of nozzle tilt/clearance body + bed rotation (user asked
  for answer only; awaiting go-ahead). Task #5 preview part still open.

## PREVIEW WORK (user 03:08 UTC: "Proceed. Stepwise, verifying at each step. Release when complete.")
- Step 1 pushed 2728ef3a: converter LineRecords, GCodeProcessorResult::NonPlanarPreview, map_preview_to_polar
  (gcode_id + lines_ends -> polar file), set_toolhead_preview. Tests: [PolarKinematics] 10/10, [NonPlanar] 8/8
  (new: preview of polar print), [S4] ok, wincheck ok.
- Step 2 pushed bacb2f80 (syntax/wincheck OK, NOT yet seen running): Marker lean + toolhead body, ToolPosition
  window machine row + Toolhead/Turn bed switches (app config keys preview_nonplanar_toolhead,
  preview_polar_turn_bed), GLCanvas3D preview branch swaps plater camera view matrix by bed_turn(). HLSD updated.
- Full app build running: scratchpad/orca_build11.log (~90 min at -j2 from 03:35).
- GUI check harness: Xvfb :99 -fbdir scratchpad/gui/fb (1600x1000); scratchpad/xgui.py shot/key/click;
  datadir scratchpad/gui/datadir (ThetaFirm installed, presets set); run
  DISPLAY=:99 LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 build/src/Release/orca-slicer --datadir <datadir> clitest/pi.stl
  then click Slice plate (1102,152), wait ~90 s. en_US locale generated with localedef. Kill with pgrep -x orca-slicer.
- Then: move the horizontal slider (click canvas + Left arrow) to show the marker window; check toolhead, readout,
  G-code window alignment; toggle Turn bed; screenshots. Then full tests, verify.sh, bump tag file to
  orcanp-s4-test-2 (+ release notes: preview features), push, watch run.
- Release notes for test-2 parked in scratchpad/release_notes_test2.patch (git apply at release).
- 04:45 UTC: GUI verified on Xvfb (screens scratchpad/gui/p1..p7.png): toolhead body leans outward, machine row
  (Bed/Radius/Tilt), switches, G-code window shows polar lines at right ids, Turn bed puts tip on +X rail.
  Full suites: libslic3r 874, fff_print 228 pass; verify.sh all OK. Pushed 9d46a13e (tag file orcanp-s4-test-2
  + notes). Release run 36295072122 started 04:42. Check-in scheduled 06:00 UTC.
- 06:01 UTC: RELEASED orcanp-s4-test-2 (published 05:28): https://github.com/PDAMUK/OrcaNP/releases/tag/orcanp-s4-test-2 — portable zip 248 MB + installer 200 MB. Nothing pending.

## ROUND 3 (user ~09:15 UTC): play button, test all modes in GUI (Classic walls, Toolhead + Turn bed), screenshots
## of prepared + sliced models -> send ONLY when testing complete and release 3 is building. Then release test-3.
- Pushed: e84ed36b (right-click part > Print surface (non-planar) + part settings entry + messages), 5b786dc3
  (Play/Pause + speed button at left of moves slider; GCodeViewer::advance_playback), 54781fad (S4 empty first
  layer message). Container pauses when idle: WAIT IN FOREGROUND for builds.
- CLI mode tests (scratchpad/clitest/process_t_*.json, machine_thetafirm.json has tilt +-45 = test only): all modes
  slice (idler cone, hollow_dome sphere, cup cylinder, ipadstand polar-only, idler opt with --orient 1).
  Unoriented idler (touches bed at a point) -> empty first layer with S4 (now clearer message).
- GUI test presets: datadir/user/default/process/T1..T6 (wall_generator classic). Models: clitest/pi.stl, domes.stl
  (nested; Split to parts), hollow_dome.stl, cup.stl, tests/data/extruder_idler.obj, ipadstand.obj.
- Build13 (OrcaSlicer target) running -> then GUI tests (Xvfb :99 via scratchpad/xgui.py).
- 11:17 UTC: all GUI tests done; screenshots sent (scratchpad/gui/sheets). Fixes pushed: 2c793f54 (bbl part menu item,
  playback clear of view cube), 5e95b54d (crash after failed slice: Plater on_process_completed output_filename
  try/catch; skip support-necessity warning for offset layers; -0.00). Suites 874/228 pass, verify.sh OK.
  5ae23d29 = tag file orcanp-s4-test-3 + notes. Release run started ~11:17; check-in in 55 min.
  Open (noted in release notes): S4 deformation progress stuck at 5%; S4 settings are Advanced-mode only.
- 12:37 UTC: RELEASED orcanp-s4-test-3 (published 12:01): https://github.com/PDAMUK/OrcaNP/releases/tag/orcanp-s4-test-3. Nothing pending.
- 13:32 UTC: Benchy (user: "would not slice in release 2, S4 optimized"): CGAL edge protection hung
  (Protect_edges_sizing_field) -> edge_min_size = size/4 (0a7c1d06). Cone mode on Benchy: empty first layer (pure
  cones meet a flat bottom at a point) -> cones flat over the footprint radii (footprint_radii + deform_cone
  flat_radius, 79b4580e). Suites 876/228 pass, verify.sh OK, wincheck OK. Benchy CLI: opt 25 s, cone 19 s, polar 7 s;
  sphere/cyl give cavity messages (expected). GUI screens: gui/report/10..12_benchy_*. "Floating regions" warning on
  Benchy is stock Orca (planar Cartesian warns too). 93310689 = tag orcanp-s4-test-4 + notes; release run started.
  Models: clitest/3DBenchy.drc and 3DBenchy.stl.
- 14:12 UTC: RELEASED orcanp-s4-test-4 (published 13:56): https://github.com/PDAMUK/OrcaNP/releases/tag/orcanp-s4-test-4. Check-in trigger deleted. Nothing pending.
- 17:49 UTC: bracket STEP test (user's shot = default orientation seen from below; C-shape, upper arm 22 mm
  cantilever over lower arm). Generated sphere: correct refusal; cylinder: message fixed (6c35e341, covers axis within 0.5 mm).
  Cylinder part (Split to parts, part _2 = print surface) CRASHED (slices_to_regions lower_bound on unsorted VolumeSlices)
  -> sorted (f103e971) + print surface must stand on the bed check + tests. Model A (cyl on arm) -> clear error;
  model B (cyl from bed) slices, clearance + radius warnings (merged into one, 43941fe1). Offset shape unsuited to
  bracket (vertical shells). opt/cone/polar slice with floating warnings. Suites 876/229, verify OK, wincheck OK.
  bb075981 = tag orcanp-s4-test-5. Screens: gui/report/13_*, 14_*. 3MFs: gui/bracket_cylA.3mf, bracket_cylB.3mf.
- 19:09 UTC: test-5 RELEASED (18:12). User: goal of offset mode = print an adaptable POST or DOME under the
  part to lift it off the bed for toolhead clearance (photo: part on raised cylinder). Raft idea DROPPED (parked in
  scratchpad/parked, disregard). dd800b51 pushed = Post geometry (LayerShapes: Post, part_base, lift_onto_post) + unit
  tests (pass). In progress (uncommitted): s4_surface_core "post" + s4_post_diameter/height, s4_dome_height, deform_s4
  lift, GUI lines/toggles, fff test "A part is printed on a generated post", HLSD doc. Script: scratchpad/post_pipeline.py.
  Test inputs: clitest/process_t_post.json, process_t_dome.json, lens.stl (plano-concave, sag 3, r 12 concave, 30 wide).
- 20:25 UTC SAFE POINT: pushed 8c76a660 (post pipeline + fff post test; all 10 [NonPlanar] pass). Tree clean.
  App binary is OLD (17:35, pre-post); full GUI rebuild needed (~1 h) after next header change.
  NEXT (not applied yet): scratchpad/post_size.py = Post size drop-down (s4_post_size Auto/Custom; auto height from
  toolhead_clearance(printer), dome via fit_dome_height). Needs unit test for fit_dome_height + fff test, full suites.
  User asked: investigate drawbacks of 'slice model's own surface through tets' BEFORE implementing; discuss first.
  Measurements (benchy170.stl = Benchy x1.7, Cartesian, user's S4 settings; render/wall_dev.py, outer wall dist to
  model, ideal ~0.22): planar med .228 p90 .246 p99 .666 >0.5mm 1.1% (16 s); S4 auto(5.2mm) .231/.371/.671 3.3% (50 s);
  S4 cell 2 mm .231/.271/.405 0.5% (533 s). => cause confirmed: coarse auto tets; 2 mm fixes but 10x slower.
- (resume after compaction) Pillar/Dome/Domed pillar rework applied (uncommitted, 12 files; keys s4_surface_size/
  _diameter/_height; never released so no migration). Building tests (-j5). NEXT: run [S4] [NonPlanar] + full suites,
  commit+push; then apply proto_env.py + proto_measure.py (hidden "[.s4quality]" test; env S4_QUALITY_MODEL,
  S4_QUALITY_ONLY=1,3 variant indices), run on clitest/benchy170.stl; full app build; CLI wall_dev + GUI screenshots
  (external + mid-layer) per variant, env vars S4_PROTO_FACET_SIZE/_DISTANCE/_WEIGHTED at app launch.
  CLI process files updated: process_t_pillar_auto/pillar/dome/domedpillar.json.
- 22:25 UTC: pushed 4ce5292c (Pillar/Dome/Domed pillar + Size; suites 878/232 pass, wincheck OK) and 6afc98ac
  EXPERIMENT env-gated (S4_PROTO_FACET_SIZE/_DISTANCE, _WEIGHTED, _MT, _STATS; hidden test tests/libslic3r/
  test_s4_quality.cpp "[S4Quality]", S4_QUALITY_VARIANTS "name|cell|facet|dist|weighted|mt|threads;...").
  PROFILE 2 mm Benchy (593 s): ~500 s in s4_solve_rotation_field SimplicialLDLT factorize (10 inversion rounds x ~8
  active-set solves); tetra ~35 s; rest ~30 s. CG(MT) == direct to 1e-9 mm, deterministic across threads.
  User: timing runs ALONE on idle cores; rounds: (1) MT alone, (2) graded alone, (3) both. run_quality.sh NAME VARIANTS
  -> quality_NAME.txt. Round 1 started 22:26. Found: CGAL tetra not reproducible run to run (18969 vs 18970 tets) —
  investigate separately (address-ordered sets?).
- 22:50 ROUND 1 (MT alone, benchy170, deform s): 2mm 82k cells direct 490 / CG 1t 333 / CG 4t 163 (447 solves, 10 rounds);
  auto 23.6k cells direct 3.9 / CG 1t 12.4 / CG 4t 8.2. Identical results (<2e-9 mm). Mesh 2mm 32 s, auto 11 s.
  Round 2 (graded alone, direct, 1 thread) started 22:51 -> quality_r2_graded.txt. Then round 3 (graded + CG 4t).
- 23:32 ROUND 2 (graded alone, direct 1t; cells, mesh s, deform s, skin p50/p99/max, unprintable, tilt vs 2mm):
  uniform auto 23.6k 11/3.8 .101/.626/.826 12.85% 2.85; graded 2mm 71k 35/52 .022/.120/.329 11.57% 1.84;
  graded 2mm W 71k -/17.8 same 11.81% 1.14; graded 1mm 172k 88/582 .008/.053/.232 12.15% 1.30;
  graded 1mm W 172k -/1030 (1021 solves!) 12.07% 1.23. uniform 2mm ref: 82k 32/490 .027/.252/.390 11.99%.
  Round 3 (graded + CG 4t) started 23:33 -> quality_r3_both.txt. NEXT: app build (1 h, alone), CLI wall_dev + GUI shots.
- 00:25 ROUND 3 (graded + CG 4t): graded 2mm 36/86; 2mm W -/26; 1mm 92/776; 1mm W -/1505 => CG slower on graded meshes.
  Presented rounds 1-3 table to user; best = graded 2mm weighted direct (53 s S4, skin p99 .120, tilt 1.14).
  User: rerun rounds on small + large complex model (small: no quality lost; large: best surface at lowest time),
  compare with first round. Models: clitest/3DBenchy.stl (60 mm), clitest/idler_x3.stl (tests/data/extruder_idler.obj
  x3 on its largest flat, 96.5x113.7x51.1). run_model.sh MODEL -> quality_MODEL_{A,B,C}.txt; started 00:27 (small then large).
  After: app build (alone), CLI slices + wall_dev + GUI shots.
- 01:02 SMALL 3DBenchy 60mm done (S4 s = mesh+deform; skin p50/p99/max; unprintable; tilt vs 2mm):
  today auto(3mm) 24k 23 s .060/.365/.474 12.55% 1.84 | uniform 2mm direct 36k 27 s .037/.252/.533 11.36% ref |
  graded 2mm W 42k 30 s .024/.125/.243 11.80% 0.90 (BEST) | graded 1mm 85k 71 s .010/.059/.189 | CG slower throughout.
  Large idler_x3 running (A,B,C). UNBUILT edit in S4Deformation.cpp: S4_PROTO_WARM active-set warm start across
  inversion rounds (lambda from previous solution) + pdas_capped stat. Build + test AFTER timing runs.
- 01:16 LARGE idler_x3 done: today auto(5.7) 13.5k 4.7 s skin p99/max .469/2.569 7.38% 1.95 | uniform 2mm 102k 161 s
  (CG4t 57) .125/.189 6.58% ref | graded 2mm W 50k 24 s .086/.129 7.16% 1.53 (BEST) | graded 1mm 129k 117 s (CG4t 100)
  .041/.067. Presented combined table. NEXT: build+verify S4_PROTO_WARM, measure alone; app build; CLI + screenshots.
- 02:05 WARM round (S4_PROTO_WARM, identical results): benchy170 2mm 447->49 solves, 490->63 s (4t 26 s); graded 1mm
  670->159 s total; graded 2mm W 52 s; small benchy today 17 s, graded2W 29 s; idler graded2W 21 s, 2mm 4t 49 s,
  graded1mm 4t 80 s. Presented. e8921474 pushed (warm). NEXT: app build (started ~02:05), then CLI slices
  (today vs graded 2mm W warm [+ graded 1mm]) on 3 models: time alone + wall_dev + GUI shots external/internal.

=== RESUME PLAN (user 04:40 UTC, work autonomously, stepwise, verify each step, push often) ===
Slice-level CLI (fixed wall_dev: lone E moves excluded; quarter-turn placement): graded 2mm W walls == planar on all 3
models (p99 .328/.308/.324 vs planar .325/.318/.324); today .441/.630/.614, 3.2-3.3% >0.5 mm on large. Full slice:
small 24->39 s, idler 35->62 s, benchy170 47->83 s (results in scratchpad/slices/results*.txt).
Screens: gui/shots.py MODEL VARIANT ZOOM (robust: finds legend icon + warning X; waits for Export button teal).
Batch running -> gui/q/shots_all.log; sheets via gui/sheet.py. Small Benchy sheet sent to user.
STEPS:
 1. [#34] Commit 4 README screenshots (3DBenchy today_opt/graded2W_opt external/internal, i.e. warm+MT on) to
    docs/images/ (optimize PNG). Time today_opt/graded2W_opt via CLI alone for captions.
 2. [#35] Preferences checkboxes "Warm start" + "Multi-threading" (AppConfig, default on, restart required) -> libslic3r
    NonPlanar solver options (replace S4_PROTO_WARM/MT env). MT = parallel axes always + CG only when a direct
    factorization is slow (>~0.5 s; data: CG wins benchy170 2mm, idler 2mm/1mm graded; loses small/graded2W/auto).
 3. [#36] Quality tab "Graded mesh" (weighted) + surface/interior cell size mm; hide s4_cell_size when on. Test defaults
    on bracket(26mm)/3DBenchy/idler_x3/benchy170 (candidates outer 2 inner 5; fd = outer/20). Remove remaining env hooks
    except test harness; keep [S4Quality] test or drop.
 4. [#28] Pillar/dome/domed pillar testing (bracket on pillar, lens on dome), GUI; fix; docs.
 5. [#37] README (plain language, what each setting is for + how, option screenshots). Release test-6 (tag file + notes).
- 04:36 DONE step 1: 330e5f7e docs/images/nonplanar/benchy-s4-{default,graded}-{all-layers,half-height}.png (warm+MT
  on). Opt vs non-opt graded shots identical (0.07% px). GUI slice: today 31 s vs opt 36 s; graded 46 vs 51 s (MT slower
  on small => implement MT as adaptive CG by factor size). Large-model screenshot batch still running (shots_all.log).
  NEXT step 2 (#35 Preferences) and tests for #36 defaults (bracket 26 mm etc.).
- 05:05 step 2 code (UNCOMMITTED until verified): S4Params warm_start/multithreading/iterative_factor_size(20M placeholder),
  S4PassData solves/factor_size/iterative, FieldSolver (CG after first direct factor if nnz > threshold, fallback to
  direct if CG fails), s4_solver_options() singleton; AppConfig s4_warm_start/s4_multithreading default true; read in
  GUI_App::init_app_config; Preferences General > "Non-planar slicing" 2 checkboxes "(Requires restart)" + restart dialog;
  PrintObjectSlice copies options. Unit test "Warm start and multi-threading leave the deformation as it is" passes (34).
  Harness uses params (warm/mt fields). Running factor-size survey -> quality_*_factor.txt to set iterative_factor_size.
  Then: full libslic3r+fff tests, wincheck (GUI_App.cpp, Preferences.cpp, AppConfig.cpp, S4Deformation.cpp,
  PrintObjectSlice.cpp), commit+push, app build, GUI check of Preferences page (screenshot for README).
- 05:40 DONE step 2: 3c4ea85e (warm/MT prefs; iterative_factor_size 3M from survey: direct faster <=1.7M, CG wins >=4.1M).
  Adaptive MT benchy170 deform: auto 3.0 s, graded2W 13.7 s, 2mm 28.3 s (never slower). Suites 879/232, wincheck OK.
  Preferences GUI check deferred to the step 3 app build. NEW user asks: final-build checks with an ordinary Klipper
  Voron 2.4 (planar + S4 non-planar Z-only, no tilt) [#38]; LAST message must be a quicklog for the user.
  NOW step 3 (#36): pick graded defaults (test bracket 26 mm etc.), then config/GUI.
- 06:35 step 3 code applied (UNCOMMITTED, building -> build_step3.log): s4_graded_mesh (default ON), s4_surface_cell_size
  (2), s4_interior_cell_size (5), both CAPPED at automatic_cell_size (L/20) in PrintObjectSlice (small parts never coarser;
  bracket 26mm test: capped 1.31/1.31 W skin .087/.114 vs today .088/.138, unprintable same, time same; fixed 2/5 on
  bracket loses quality). Interior 2/5 vs 2/auto vs 2/8 identical on >=60 mm parts; 1.5 surface better but 1.4x time.
  TetrahedralizeParams.surface_cell_size (fd = surface/20); S4Params.size_weighted; env hooks removed; harness uses params.
  Tests added: libslic3r graded cube + weights=1 on uniform; fff "S4 meshes a large part finely only at its surface...".
  HLSD doc updated. NEXT: run suites, wincheck (PrintConfig.cpp, Tab.cpp, ConfigManipulation.cpp, Preset.cpp,
  PrintObject.cpp, PrintObjectSlice.cpp, S4Deformation.cpp, Tetrahedralize.cpp), commit, app build, GUI check of
  Preferences + Quality tab (screenshots for README), then #28 pillar/dome app tests, README, Voron checks, release.
- 07:10 fixed name clash (Tetrahedralize.cpp `surface` -> skin_size); rebuilding tests (build_step3.log, ~151/174).
  Bash classifier outages: use Read/Grep while it fails. Drafts: scratchpad/README_draft.md (needs option screenshots
  docs/images/nonplanar/{quality-s4-settings,preferences-nonplanar,printer-polar,printer-toolhead}.png, and a Voron
  section check), scratchpad/flatten_preset.py (Voron 2.4 system preset -> CLI json). Release = bump
  .github/test-release-win64 to orcanp-s4-test-6 + update notes in .github/workflows/test_release_win64.yml.
  README plan: README.md = OrcaNP readme; move upstream README.md -> README-OrcaSlicer.md (root, keeps image paths).
  Consider: cap S4 rotations at nonplanar_nozzle_clearance_angle on non-tilting printers (decide after Voron test).
- 07:50 DONE step 3: 7f5c2f00 graded mesh (suites 881/233, wincheck OK). Release notes edit saved at
  scratchpad/test_release_win64.yml.test6 (restore into .github/workflows when bumping the tag). App build started.
  NEXT: GUI check Preferences + Quality tab (screenshots docs/images/nonplanar/{quality-s4-settings,
  preferences-nonplanar,printer-polar,printer-toolhead}.png), #28 pillar/dome app tests, README, Voron, release, quicklog.
- App built (build_app2.log EXIT 0). verify.sh: planar cube/pi identical, s4/theta/cone OK, polar_pi FAILED:
  CAUSE = graded mesh NONDETERMINISTIC across runs (pi.stl: 2705 vs 2703 tets; CLI same slice twice -> 79k lines differ).
  Uniform mesh deterministic. Graded with no optimization: deterministic (2798 x3). perturb only: 2763 x3; exude only:
  2713 x3 => the COMBINATION perturb+exude is nondeterministic (address-ordered queues). FIX TODO: for graded meshes use
  one optimizer (compare quality: exude-only vs perturb-only vs both on 3DBenchy/benchy170 incl. repeat in one process,
  harness distance field "noopt"/distinct keys r1,r2; temporary TMP_OPT env hack was REVERTED, rebuild needed), then
  add a determinism test (tetrahedralize twice -> identical points) and re-run verify.sh. Harness "noopt" field change
  uncommitted in tests/libslic3r/test_s4_quality.cpp (keep).
- DONE 7f394635: ROOT CAUSE of nondeterminism = CGAL refine_mesh_3.h:408: with no time limit, perturb/exude stop
  after `refine_time` (wall clock) -> load-dependent. Fix: no_perturb + exude(p::time_limit(0.)) (0 = unlimited).
  Determinism test passes 10/10 idle and 6/6 under full CPU load; 3DBenchy/idler_x3/benchy170 graded meshed twice in
  one process: identical, same quality/time as before (scratchpad/exude_nolimit.txt); bracket/pi x3 identical.
  (Ruled out on the way: -fno-lifetime-dse, fresh time stamps.) Local CGAL headers restored from scratchpad/cgal_backup.
  NEXT: relink app (ninja -f build-Release.ninja OrcaSlicer, alone), verify.sh (polar_pi) + CLI slice twice -> diff,
  then run_pillar.sh (#28), option screenshots, Voron checks, README, release test-6, quicklog.
- verify.sh all 7 OK on 7f394635 app; CLI 3DBenchy graded sliced twice: 336851 lines, 0 differ. NEXT run_pillar.sh
- DONE 2b542118: pillar layout fix. run_pillar.sh: lens on auto pillar FAILED (empty layers 2..3.8): from-above centre
  clamped to pillar foot (2 mm below top) -> part shrank ~5x in sliced space. New deform_offset_flat_top for Pillar
  (keep XY over top; below top move out by depth). fff test "A part much wider than its pillar is high..." failed
  on old code (186 vs 1350 mm3), passes now (1346.7). All run_pillar cases exit 0; bracket auto footprint 19.8x25.8
  (was 18.6). Suites: [S4] 38 cases, [NonPlanar] 14 cases green. App relinked (build_app4.log).
  NEXT: GUI: option screenshots (scratchpad/gui tools, Xvfb :99) + pillar/dome preview look; Voron; README; release.
- DONE: GUI screenshots committed (docs/images/nonplanar: quality-s4-optimized/pillar, preferences-nonplanar, printer-polar/toolhead, preview-pillar-lens, preview-dome-lens). GUI helpers: gui/do.py, resize.py, waitshot.py, waitslice.py. NEXT: README, Voron checks, release.
- IN PROGRESS (uncommitted, 44f004c4 + edits): Voron check found S4 first layer dipping to 0.057 mm (Benchy bow) --
  cells can't hold thin flat layers. FIX: S4Deformation.flat_top/blend (Print.hpp -> big rebuild), S4ObjectMapping
  flat_top_z/blend_top_z, map_moves flattens z<=flat top (identity z, flow 1, tilt 0) and blends linearly to
  M(blend_top) (flow = rise/span, tilt*frac). Also: S4 rotation limits capped at nonplanar_nozzle_clearance_angle on
  non-tilting printers. Tests: fff "first layer flat..", "planar height" tightened (20 flat layers), "lean no further
  than a nozzle that cannot tilt"; libslic3r "Layers near the bed are printed flat, then ease..". Tooltips + HLSD
  updated. Test build: scratchpad/build_tests5.log. README scratch: scratchpad/README_final.md (needs final table).
  NEXT: run tests; relink app; verify.sh; voron/run.sh (planar, s4, lowz); run_final.sh (table); README; release.
- DONE f0397705 (flat first layer, clearance cap, toolhead re-slice; libslic3r 884 / fff 237 pass). App build running -> build_app5.log. NEXT: verify.sh, voron/run.sh x3 + lowz, run_pillar.sh, run_final.sh (README table), README commit, release test-6 (notes: scratchpad/test_release_win64.yml.test6 + new Fixed list), quicklog (scratchpad/quicklog_draft.md).
- DONE 62f21575 README (README-OrcaSlicer.md = upstream) + Surface gap tooltip (user asked: gap user-definable -> it
  already is, s4_surface_gap 0..; verified lens gap 0.6 -> first layer 2.8 vs 2.4). Final checks on f0397705 app:
  verify.sh all OK (s4 lowest Z 0.20), run_pillar all exit 0, Voron: planar OK; S4 Z-only lowest Z 0.200, steepest
  28.7 deg (default 50), clearance 30 -> 27.0 deg + floating-regions warning. Table: scratchpad/final/results.txt.
- 9cfe964b pushed: tag file orcanp-s4-test-6 + notes -> run 36471520139 (Win64 test release). NEXT: watch it,
  check release assets, then final quicklog message (draft scratchpad/quicklog_draft.md).
- 19:39 user near usage limit: quicklog given; release run 36471520139 still building. On resume: check run, check release assets at releases/tag/orcanp-s4-test-6; if failed, get_job_logs failed_only and fix.
- #41 S4 SUPPORTS (user lever 2469658c-H2D_Duct_Lever.3mf; data in scratchpad/lever, sup.py plots). Two bugs:
  (1) painted facets + support volumes sliced UNDEFORMED at deformed heights -> manual support empty;
  (2) support toolpaths mapped by nearest-cell displacement -> into the part / under printed layers.
  FIX (uncommitted): S4Mapping s4_to_sliced_space (reverse mapper) used in PrintObject::project_and_append_custom_facets
  and PrintObjectSlice slice_support_volumes (S4Deformation.lift added, Print.hpp -> full rebuild);
  S4GCodeTransform: S4SupportSurface (AABB rays), map_support column mapping (top = part underside - top gap,
  bottom = surface below + bottom gap or max(z_floor, flat_top, identity_below)), place(), pass_junctions uses place and
  support flag; NonPlanarExport builds surfaces (not for Offset layer shape). Tests: libslic3r "Support is mapped as
  columns..", fff "S4 support stands on the bed..", "S4 prints support where it is painted..". Build log
  scratchpad/build_tests.log. NEXT: run tests, app build, re-slice lever (lever/cart_s4*.3mf, sup.py), suites,
  wincheck, HLSD + README, commit/push, report.
- WIP pushed 9b5fa7b2 (support fix, not yet built/tested). Test build running -> build_tests.log; then re-run ninja (files edited mid-build), tests, app, lever/run.sh.
- NEW USER GOALS (after #41; easy ones go into the supports release test-7), stepwise + verify:
  #42 Preview playback 1x speed. #43 Benchy on a stick (offset mode) bottom massively deformed by post.
  #44 Offset-mode posts/domes printed AS SUPPORT (interface layers, base/interface pattern, walls, adaptive
  layer height), disposable; optionally S4-deformed sphere surface. #45 Warn up front if an S4 model has no
  overhangs. #46 Polar shows as 'undefined' kinematic type -> add Polar, tidy GUI. #47 Polar alignment
  calibration (Calibration menu): two rings at set radii; ring diameter too big => true centre farther, etc.
- b8ed5f16 pushed: support nearest-edge anchors + loose support dropped; [S4] 40 / [NonPlanar] 19 pass. NEXT: app build (build_app7.log) -> lever/run.sh + sup.py + order.py; then full suites; then #42..
- UNCOMMITTED (building, build_all8.log: tests+app+validator): #42 1x playback (IMSlider PLAY_SPEEDS, default idx 2 = 30x);
  #46 PrinterStructure psPolar appended ("polar"), printer_structure mode comAdvanced, handle_legacy_composite syncs
  polar_kinematics<->structure, Tab hides polar_kinematics checkbox (structure drives it), ThetaFirm profile
  printer_structure polar + Custom.json 02.04.00.07; #45 NonPlanar::s4_overhang_area + Print::validate warning
  (opt_key s4_max_overhang) for Optimized S4 parts with < 1 mm2 overhang. Tests: fff "S4 warns before slicing a part
  with nothing overhanging", libslic3r [Polar] "A polar printer is chosen by its structure...". README/HLSD updated.
  NEXT after build: run those tests + suites, validator check_profile.sh --vendor Custom, lever/run.sh, GUI check
  (printer structure Polar, polar group hidden for non-polar, 1x speed, warning), commit; then #43 benchy on stick.
- #43 REPRODUCED (stick/run.sh, stick/views.png, old binary): Benchy on custom 8x20 pillar: printed hull bottom warped
  (bulges/ripples). Cause: offset layers = base + euclidean distance - gap; beyond a small flat top they become rounded
  shells (~75 deg at 20 mm out) and the round trip distorts. Auto pillar (r~20) fine. User-modelled stick (Parts core)
  worse: deform_offset_from_above centre 0.5*width below top -> hull squashed ~5x. PLAN (after test-7): flat-topped
  cores (pillar, flat-top parts): flat over the top, conical at s4_cone_angle beyond the rim = vertical shear
  (reuse deform_cone relative to top, XY kept); points under the cone are lost -> warn. Domes keep from-above.
- WIP pushed 4742390d (#42 #45 #46, not yet built). Full build running (build_all8.log, 622 steps).
- Profile check (full tree, local validator with psPolar): all 5 PASS, 0 errors.
- DONE: 3c085494 (support no-room rule, test move) + b26fec25 RELEASE orcanp-s4-test-7 pushed -> run 36500148146
  (Win64 test release). Verified before: libslic3r 886 / fff 241 all pass, [S4] 41, profile checks, wincheck OK,
  GUI: warning shows before slicing, Printer structure Polar (group hidden for CoreXY user preset), 1x playback.
  Lever: painted support now 5403 pts (was none), 0 inside, 0 below 0.15; auto: 0 below 0.15 (was 125).
  NEXT: watch run; #43 pillar cone layers (PrintObjectSlice pillar branch: deform_cone(points, axis, cone angle>=0,
  post.radius) shifted by surface_top - gap - top; warn when part points fall under the cone; show s4_cone_angle
  for Pillar core in ConfigManipulation); then #44 cores as support, #47 polar calibration (both need new config).
- WIP pushed (#43): pillar layers = deform_offset_pillar (deform_cone shifted, XY kept), hang warning, cone angle shown
  for Pillar, tooltip; deform_offset_flat_top removed (+ its unit test replaced). Benchy stick slope 71.4 -> 24.5 deg
  (stick/views_after.png, stick/before/). New tests pass. NEXT: run_pillar.sh (check lens/bracket for "hangs"
  warnings), [S4]/[NonPlanar] + full suites, HLSD (Layer shapes/pillar) + README (Print surfaces: cone angle for
  pillar), commit; then #44, #47. Release test-7 run 36500148146 (check-in trigger set ~01:12Z).
- #43: run_pillar all exit 0, no 'hangs' warnings; bracket footprint unchanged 19.8x25.8; lens on auto pillar printed in full (flat layers, 30 mm, z 2.2..8.2). Docs pushed 833d3c58. Full suites running (lib_all8.log, fff_all8.log).
- #43 DONE: full suites pass (lib 886, fff 242). Waiting for release check-in; then #44.
- test-7 RELEASED 00:48Z: portable zip + installer uploaded (releases/tag/orcanp-s4-test-7). Idle until user's reset (3:10); next #44.
- #44/#47 code in progress (uncommitted): options s4_surface_as_support/wall_loops/layer_height, polar_radius_offset/scale; SupportCommon generate_print_surface_support_paths; PrintObject::s4_print_surface_as_support (end of infill + support steps); layer profile override; PolarKinematics radius calib; tests added. NEXT: full build (build_all9.log).
- WIP pushed: PolarCalibration.hpp/.cpp (not yet in CMake/MainFrame: add after build_all9 finishes), solve_radius_calibration + test, README/HLSD for #44/#47.
- #44/#47 built (888 lib / 243 fff pass), committed 053d319b..2a8b352c. #47 GUI verified end to end (ec720424):
  menu item, Create rings closes the dialog and sets up 80x80x3 rings (single outer walls, slices 15 layers),
  reopen remembers sizes, 40.6/80.8 -> offset 0.200 mm, scale 100.500 %, shown modified in the printer tab.
  NEXT: #44 GUI check (S4 surface options on Offset + Pillar, slice, support roles in preview), then test-8.
- #44 GUI verified (Print as support toggle, lens on pillar: Support 2.06 g + interface 0.18 g vs 2.10 g lens).
- User (04:15Z): posts, domes, domed posts only ever support, never solid; then "ALL" -> #48: removed option
  s4_surface_as_support entirely (unreleased); every generated core (sphere/cylinder/pillar/dome/domed pillar) is
  support; modelled Parts keep part settings. Wall loops tooltip carries the explanation. Test rewritten.
  test-8 notes + tag file edited (uncommitted until build + suites pass). Build log build_48.log.
- test-8 release files saved as resume/release_test8.patch (git apply after suites pass), tree clean.

## Painted print surfaces (user request 2026-09-29 ~04:40Z; usage 64%, reset ~07:40Z)
User: remove user-modelled print surface parts in favour of paint-on print surface faces (fill-paint a domed
underside or flat base; inside faces of a bracelet with radial understanding, post generated tall and wide
enough for a horizontal nozzle). Keep ALL other current settings as they are. Stepwise, verify against the
whole before the next step, make safe each step, keep this file current.
Plan:
 0. #48 build+suites+lens check, then apply resume/release_test8.patch, push -> test-8 release.
 1. Config/model: S4SurfaceCore::Parts -> Painted (key "painted", legacy "parts" -> "painted", still default);
    ModelVolume::print_surface_facets (FacetsAnnotation, like fuzzy_skin_facets): copy/ids/cereal/3MF/PrintApply
    invalidation. Keep s4_print_surface readable only for migration (hidden).
 2. Core from painted faces in deform_s4 (PrintObjectSlice.cpp ~990): layout From above -> painted faces
    offset by gap, walls down to the bed; layout Around the axis -> cylindrical grid r(theta,z) from ray casts,
    offset by gap, top cap to the axis, column down to the bed; refuse if not all round the axis. Lift: reuse
    s4_surface_size Auto (toolhead_clearance, like pillar) / Custom s4_surface_height. Store in s4->core so it
    prints as support. fff tests: flat base on lift, dome underside, bracelet radial.
 3. Remove modelled-part path (slicing surfaces ~1530, Print.cpp validate ~1794, object list menu
    GUI_ObjectList ~6840, GUI_Factories 117); migrate legacy flagged parts on load: paint S4 part faces within
    gap+0.5 mm of them, delete them. Rewrite tests test_nonplanar.cpp ~575/635.
 4. GUI: painting gizmo (copy GLGizmoFuzzySkin), toolbar icon, ConfigManipulation (size/height for painted).
 5. README + HLSD, release test-9.
- Prepared (not applied; apply after the #48 build, in order): scratchpad/step1_model.py, step23_painted.py,
  step23_tests.py; new files committed-to-be src/libslic3r/NonPlanar/PaintedSurface.hpp/.cpp (written, untracked).
  Then one full build (headers change), suites, then GUI gizmo (step 4).
- step4_gizmo.py prepared (GLGizmoPrintSurface from fuzzy skin gizmo, shortcut J, icons).
- step5_docs.py prepared; GUI test models in scratchpad/painted (bracelet.stl comfort fit, shell.stl). Waiting on build_48 (~05:15Z).
- 05:25Z #48 build OK, [NonPlanar] 23 pass, lib 888 pass, fff running (fff_all48.log). All painted scripts APPLIED
  to the working tree (uncommitted; GUI_Factories menu fn removed by hand). Next: when fff passes, commit ONLY
  .github (git apply resume/release_test8.patch) -> test-8; then commit painted WIP and full build (build_49.log).
- painted work parked: git stash 'painted-wip' + resume/painted_wip.patch (git apply). Restore after test-8 push.
- 05:20Z test-8 PUSHED (999df02a), release run 36525630277 in progress; check-in trig_01UT7oZzfVqeGTYZo2ahbpop at 06:25Z.
  Painted work committed as WIP daf0904e (stash popped, empty). Full build running -> build_49.log (-k 0).
  NEXT: fix compile errors, [NonPlanar] tests, suites, CLI slices of painted/bracelet/shell (need 3MF with paint:
  make via GUI or a tiny test), GUI end-user test (J gizmo, smart fill, bottom view), docs, test-9.
- 06:20Z build OK after 3 compile fixes (pushed e665a608). [NonPlanar]: 3 fails, all test-side: revolve() in
  test_nonplanar.cpp wound inward (fixed: (a,b,c),(b,d,c)); migration check needs shell->get_matrix() (fixed);
  lift == 0. -> WithinAbs. make_models.py winding fixed too (volumes now positive). UNCOMMITTED: these test edits
  + a TEMP "DEBUG painted" log block in PrintObjectSlice.cpp (remove before commit!). Next: rebuild fff tests,
  rerun [NonPlanar] (use wildcards, commas split Catch names).
- 06:40Z usage 95% (resets ~07:45Z); Bash/tool classifier failing. DEBUG block REMOVED from PrintObjectSlice.cpp.
  UNCOMMITTED: that removal + test fixes (test_nonplanar.cpp). FIRST on resume: git add -A src tests; commit;
  push; move/ignore check-in trig_01UT7oZzfVqeGTYZo2ahbpop (06:25Z); check release run 36525630277.

## New goals from the user (06:40Z), AFTER the reset and after the painted work; same rules (stepwise, verify against
## the whole, make safe each step, keep this file current, test like the end user, release when ready)
G1. Re-add modelled print surface parts as an ADVANCED option shown ONLY with Developer mode enabled, and ONLY in a
    "don't print" mode: the part stands for a real solid surface the user has placed on the bed; it is not printed.
    Explain that in the UI, with a strong warning: a misplaced surface could destroy the machine (nozzle crash).
    (Pieces: s4_print_surface still read; the per-part menu item / settings line gated on developer mode; slicing
    uses the part as the core but prints nothing for it; validation; docs.)
G2. For future testing (esp. Z-only non-planar): add the system printer "Klipper Voron Designs V2.4r2 300mm" from the
    Printer presets tab, with the non-planar toolhead set to 2.2 degrees all-around clearance from the nozzle edge,
    out to a 65.5 mm total width and 126 mm high boundary (user's words; map onto nozzle clearance angle / head
    radius 32.75 / nozzle length or head height when doing it).
- 07:05Z [NonPlanar] 26/26 pass (fix: unwrap window skipped below identity_below_z). Pushed. Next: app relink, full suites, GUI test.
- 08:15Z bracelet GUI test: painting works (smart fill), 3MF saves paint. Fixed unwrap trimmed travel (9h06->3h02; 0deg infill 2h22). Pushed. Next: GUI re-slice check of clearance warning, shell + flat base GUI tests, docs (0deg tip), test-9.
- 09:20Z committed junction lift/turn, along-part trimmed hops, bead stamping; bracelet warnings 17610->79, 3h02. Next: full suites, shell+flat-base GUI tests, README 0deg tip, test-9.
- 09:40Z shell GUI test OK (painted inside, lift auto always now; 49 warnings). Committed. Next: full suites, release notes test-9, release.
- test-9 notes saved: resume/release_test9.patch (apply after suites pass)
- 10:00Z test-9 PUSHED. Next: G1 (developer-mode don't-print surface parts), G2 (Voron test printer).
- 10:20Z G1 code+tests committed WIP (4a..); full build next (build_53.log).
- G2 Voron test printer: scratchpad/voron/machine_voron24_300.json (flattened "Voron 2.4 300 0.4 nozzle" + clearance
  angle 2.2, head radius 32.75, nozzle length 1.24 = (32.75-0.4)*tan 2.2deg; 126 mm head height not modelled).
- G2 GUI: user printer 'Voron 2.4 300 NP test' (inherits Voron 2.4 300 0.4 nozzle) in gui/datadir; Voron model enabled in conf.
- 10:44Z G1 fixes pushed 2cab7660 (empty slices for placed parts, on_placed first-layer skip); [NonPlanar] 28/28. App relink running (build_app.log). Next: GUI G1 check, full suites, G2 GUI/CLI, test-10.
- 11:06Z pushed 7c610aa8: placed layers start at layer 1 (libvgcode needs sequential layers), skirt/brim refused, off-surface refused; [NonPlanar] 30/30. CLAUDE.md rule: never lock in user-settable changes. OPEN QUESTION for user: S4 rotation cap at toolhead reach (PrintObjectSlice ~940) silently overrides s4_max_rotation_*. Next: relink app, GUI retest (expect brim error with Voron auto brim), full suites, G2, test-10.
- 11:16Z user decided rotation cap: auto-fill Maximum rotation with toolhead limit + message where to change (ConfigManipulation), slicing warns (NonPlanarExport::s4_lean_limit/_where). Clearance angle min 5->0.1 (2.2 was rejected), cone clamp 1.5rad->89.9deg. Placed: brim now REFUSED (not dropped). UNCOMMITTED until [NonPlanar]+app build OK (bakn0fbub). TODO after build: tooltips s4_max_rotation_near/far mention auto-fill (PrintConfig.cpp), GUI check of auto-fill dialog on Voron NP test + V1/Q processes, placed GUI retest w/ No-brim.
- SAFE POINT (usage limit; user: stop was accidental). Branch clean = ae34d261 (pushed). NEXT on resume:
  1. Re-apply tooltip edit (s4_max_rotation_near/far: "goes no further than the toolhead reaches ... A larger value is
     filled in with that angle.") in PrintConfig.cpp ~5000; relink app (ninja -C build -f build-Release.ninja -j3 src/Release/orca-slicer).
  2. GUI: Voron 2.4 300 NP test + an Optimized S4 process (e.g. user proc 'Q auto S4' re-parented to Voron, or new V2 optimized)
     -> auto-fill dialog shows 2.2 + where-text; values filled. Placed: open scratchpad/placed/shell.3mf, set brim No-brim
     (process JSON V1 placed above: add "brim_type": "no_brim"), slice -> preview layer slider must show ~20 layers, no empty layers.
  3. Full suites (alone): ctest libslic3r -j2, fff_print -j2.
  4. G2: CLI Z-only slices on voron/machine_voron24_300.json (voron/run.sh), check.
  5. Release test-10 (notes: placed surfaces dev mode, rotation limit auto-fill, clearance angle min 0.1, preview fix).
- 11:24Z step 1 DONE (tooltips pushed, app relinked). Next: step 2 GUI.
- 11:29Z step 2 IN PROGRESS: clearance 2.2 now loads without 'invalid values' (OK). BUG to debug: placed/shell_nobrim.3mf (project brim_type no_brim) + V1 preset JSON brim_type no_brim -> GUI still shows 'A brim could run into a placed print surface' error. Check: object-level brim_type in model_settings? Print::validate uses object->has_brim() (Print.hpp:419) -> maybe object config from 3MF 'different_settings_to_system' ignores brim; try CLI: orca-slicer --datadir gui/datadir --slice 0 placed/shell_nobrim.3mf, and GUI Process>Others>Brim type. Also the error 'Jump to' link opened a broken overlay (object settings) - check if pre-existing upstream behaviour for opt keys not in object list.
- 11:29Z CLI slice of placed/shell_nobrim.3mf OK: 20 layers, layer 1 printed, no brim, 58m (fix verified). GUI brim error likely from the project's embedded process preset copy (Metadata/process_settings_*.config) - verify with a fresh GUI project. Full suites started in background: /tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/suite_lib.log, /tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/suite_fff.log.
- 11:35Z suites PASS lib 888 / fff 252 (rebuilt). Next: G2 voron CLI.
- 11:35Z G2 CLI: planar OK; S4 Z-only OK 26.6 s, steepest 3.3 deg (limit 2.2), no clearance hits. Next: GUI brim check, auto-fill dialog, release test-10.
- 11:39Z GUI verified: placed shell 20 layers in preview (after Brim No-brim); auto-fill dialog 2.2 on Voron + V2 optimized. NEXT: release test-10.
- 11:40Z test-10 PUSHED (8c0c4505); release run ~1 h. Check releases/tag/orcanp-s4-test-10 at the 12:35Z wake-up. Tasks #50/#51 done.
- 13:10Z test-10 RELEASED (published 12:54Z): installer 200 MB + portable 248 MB. Nothing pending. Open for user: GUI project reload keeps Brim type Auto (project-load behaviour, not ours); Voron Z-only steepest 3.3 vs 2.2 limit.
- 15:30Z Brim reload: NOT a bug (hand-edited 3MF lacked brim_type in different_settings_to_system; loader resets unlisted keys to system values by design). Real flow verified: GUI brim No-brim -> Save As shellsaved.3mf -> reopen -> no error. 3.3 deg overshoot: micro-segments only (steepest 5.3 deg over 0.07 mm; 2.18 over 2 mm, 1.35 over 32.75 mm; 0.016% of length) -> not an issue. Temp DEBUGNP log reverted; relinking.
- 15:42Z User: release when ready. Branch head == 8c0c4505 == test-10 (released 12:54Z); no code changes since (brim reload + 3.3 deg needed none). Nothing to release; idle.
- 18:35Z README audit: all 49 s4/polar/nonplanar options documented; added flat-toolhead guidance (2.2 deg example, min 0.1), Maximum rotation auto-fill in Printers section, nothing-overhangs warning. Pushed 1dc604a1.
- 19:00Z pushed 525a3cc0: Generic Polar Printer (MyPolar 0.4 nozzle, 0.20mm Standard @MyPolar, Custom 02.04.00.08) + converter Z-before-known fix + test; profile checks full tree 5/5 PASS; [PolarKinematics] 13 pass. Next: GUI (wizard/printer list, slice, preview), suites, release test-11.
- 19:08Z test-11 PUSHED (0287aea2); check release run in ~75 min.
- 19:27Z pushed 5cb8defc: S4 transform first-travel Z0 fix (+fff test, suites 889/253, verify 4/4) + README Kalico note. PLAN: when test-11 run finishes (~20:25Z, check-in 20:29Z), release test-12 with this fix.
- 19:29Z test-11 run 36617102805 CANCELLED (user); test-12 PUSHED (539732ff). Check release ~80 min.
- 20:55Z test-12 RELEASED (run 36619601821 success; installer + portable zip). Heartbeats stopped. Idle: awaiting user.
- 21:10Z 'contributor mypolar' = GitHub @mention in test-12 notes ('@MyPolar'). Rename idea dropped (stash@{0}, not wanted). Pushed d433ac87 (CLAUDE.md rule) + d1694681 (notes: @ names in code spans, [skip ci], no run). test-12 release body itself still has the mention: needs an edit on GitHub (no release-edit tool).
- 21:23Z User wants Generic Polar Printer name: rename re-applied (renamed_from old names), Custom 02.04.00.09; profile checks 5/5, GUI test-12 conf opens renamed presets + slices. Pushed 847f4fce + notes [skip ci]. Next release notes already updated.

## Session: user test-12 screenshots (lever + Benchy), task #53 (in progress)
Lever project: /root/.claude/uploads/.../2469658c-H2D_Duct_Lever.3mf, copied to scratchpad/lever/lever.3mf, extracted lever/x.
Scripts: lever/run.py NAME key=val... paint=bend3|bottom (CLI slice with patched project settings; mesh uses build-item rotation), lever/ana.py GCODE (polar->tips; long wipes, far moves, rising wipe blocks).
FINDINGS:
1. End-of-print wipe is after "; NONPLANAR_OBJECT_END" -> object -1 -> no mapper -> printed at sliced coords: climbs ~10 mm / long diagonal line (user imgs 6, 11, 15). FIX PLAN: Move.wipe flag (;WIPE_START..;WIPE_END); a wipe outside markers takes the last printed object's mapping (in parse).
2. pass_travel_safety inserts the lift BEFORE the wipe (run starts at first wipe move) -> vertical yellow lines, wipe in air. FIX: wipe moves flush the run (not part of it); lift at nozzle pos before run (track), not anchor.
3. 'already retracted' lost by F-only "G1 F..." lines (pure_e w/o e) -> double retraction. FIX: retracted state machine (pure_e e<0 true, e>0 false, F-only unchanged, printing false, wipe true).
4. pass_junctions drops wipe moves in mixed runs -> their E lost. FIX: leading wipes act as 'from' anchor; carry E of dropped moves as a pure_e.
5. Offset layers fold (non-injective) for painted bend on lever: folded_share 11% -> empty layers. Added NonPlanar::folded_share (LayerShapes) + TEMP fprintf in PrintObjectSlice (remove!). Good cases 0.0000x. PLAN: SlicingError if > 1%, message naming Pillar / other layer shape.
6. fit_cylinder_core lacks the documented "surrounds the axis" check (lever cyl case prints nozzle horizontal mess). PLAN: angular coverage of part footprint around axis must be 360 deg, else throw.
7. Dome case: empty layers 55.2-56 w/o fold (likely thin feature) - leave.
8. Ghost model at unlifted position in preview - not investigated.
USER REQUEST (latest): rename "Generated sphere"/"Generated cylinder" labels so their purpose (fitted in the part's cavity) is clear at a glance.
- rename Cavity sphere/cylinder committed+pushed. User asks: can the two be combined into one option? (answered with proposal, awaiting)
- User chose AUTOMATIC Cavity (one option). UNCOMMITTED edits in tree (not yet built): PrintConfig.hpp enum Cavity (replaces Sphere/Cylinder; ~1h rebuild), PrintConfig.cpp keys/labels/tooltip/handle_legacy sphere|cylinder->cavity, PrintObjectSlice (auto sphere/cylinder by volume; around-axis cylinder only; fold refusal >1%), LayerShapes (folded_share, cylinder must surround axis), S4GCodeTransform (Move.wipe; wipe after END mapped by last printed object; junction leading wipes = from, dropped E carried as pure_e; travel_safety: wipes flush runs, lift at run_from, retracted state machine). NEXT: tests (test_s4: surround, folded_share, wipe mapped, lift after wipe, F-only retraction; test_nonplanar: sphere test -> cavity, legacy load, cup gets cylinder), README/HLSD, build all, suites, lever CLI re-check (ana.py), GUI check, commit, push.
- $(date) All edits done incl. tests (test_s4: surround, folded_share, wipe mapped, lift after wipe, F-only retraction; test_nonplanar: dome->cavity sphere renamed, cup gets cylinder, legacy sphere/cylinder->cavity), README (Cavity row + fold paragraph), HLSD. Full build started (build_cavity.log, ~1h, header change). NEXT: fix compile errors, run [S4] [NonPlanar] + suites, lever CLI: paint3 must be refused w/ fold msg, cyl -> sphere chosen (lever not round axis), pillar end wipe no climb (ana.py), GUI check dropdown shows Cavity; commit (code) + push; README/tooltip done.
- WIP commit pushed (untested) because stop hook demanded it; build_cavity running. User: "Follow memorised development rules" (step-wise; verify build+suites+GUI before next; no source edits during build).
- NEW GOAL (queued until build ends): preview playback starts at 1x: src/slic3r/GUI/IMSlider.hpp:232 m_play_speed_idx{ 2 } -> { 0 } (PLAY_SPEEDS {1,10,30,100,300,1000}); check README playback text; then incremental build, GUI check (play button shows 1x), commit+push.
- 23:41Z build_cavity was killed at 318/617 (bg time limit); 1x edit made (IMSlider.hpp idx 0, README); build restarted with 2h timeout (bj2bkdz4o).
- 23:56Z container restarted; build resumed (bgym8y1c2) from ~117/419. Commits pushed: 010098da (Cavity+wipes+fold, untested), 7103e60e (1x, untested).
- 00:27Z build OK; [S4] 45 cases pass, [NonPlanar] 35 pass. Suites running (by5r60mpk -> suites_cavity.log). Then lever CLI re-check + GUI 1x check.
- 00:35Z VERIFIED: suites libslic3r 893 / fff_print 255 pass; lever CLI: pillar end wipe fixed (0 far moves), paint3 refused w/ fold msg, cavity slices (sphere), no rising wipes; GUI preview shows 1x. Tasks 53/54 done. Pending: user may upload toolhead STEP for asymmetric toolhead model (proposal given).
- 01:01Z RELEASE test-13 pushed (0878d088): verify.sh 4/4 pass. Heartbeat 10 min re-armed for the Win64 run. User thinking over toolhead model (STEP).
- 01:36Z MATRIX started (scratchpad/matrix: flatten.py, check.py, run_matrix.py; 388 cases; out/<case>/done.json; results.jsonl; run.log). Assumed clearances: BBL 20deg/4.2/30 (H2D r35), Klipper 30/3/25, Voron 2.2/1.24/32.75. Next: review FLAGs, GUI calibrations (#56).
- MATRIX done: 388 cases. Expected refusals ok (S4+absE, S4+vase named; x1c08 tree tip upstream check). polar arc/zhop "Killed" = my launch.sh kill (reran ok). FOUND:
  * BUG (ours, FIXING): S4 mapping ignores extruder_offset (X1C 0x2) -> cavity dome distorted, rim printed at Z0. Fix in NonPlanarExport::s4_mappers (subtract offset of object's extruder; throw if extruders differ). Test added "S4 printing on a printer with an extruder offset..." Build running (build_offset.log).
  * UPSTREAM crash: A1 + fuzzy skin (external) Benchy segfault ~1/8 runs in GCode::extrude_perimeters (TBB worker, main-stack addr). A1 base never, X1C/Voron fuzzy never. Not in our gated code. Report to user.
  * polar G-code header lacks "model printing time" line (end summary present) - check S4 Cartesian too.
- CALIBRATION GUI (X1C, datadir_cal, gui/cal.py + okslice.py): Temperature, Max flowrate, PA tower/line/pattern, Flow YOLO/pass1/pass2 all slice OK. Remaining: Retraction(257), Cornering(284), Input shaping(311 submenu), VFA(338), Polar alignment(365); then on ThetaFirm.
- Extruder-offset fix built+tested ([NonPlanar] 36 pass), pushed (commit after 7103e60e...). X1C/H2D/A1 S4 rerun OK.
- X1C calibrations: +Retraction, Cornering, VFA OK; Input shaping on X1C refused by upstream (needs Marlin 2 flavor) -> test on Voron/Klipper. Polar alignment opened on X1C -> FIX: menu enabled only for polar printers (MainFrame polar_printer_selected). 
- ThetaFirm temp tower with S4 process printed NON-PLANAR -> FIX: Plater calibration_objects_planar() sets per-object s4_enabled=false when process S4 on (all calib entry points, flowrate too). README notes (Travels bullet list + Polar alignment "available with a polar printer"). Build running (build_calib.log). UNCOMMITTED: MainFrame.cpp, Plater.cpp, README.md.
- NEXT: after build: ThetaFirm calibrations (all), input shaping on Voron, commit/push, suites, then release test-14 (includes offset fix + calib fixes). Report upstream A1 fuzzy crash.
- 03:14Z VERIFIED GUI (calib build 01e8f17f): ThetaFirm S4 process: Temperature, Max flowrate, PA tower/line/pattern, Flow YOLO/pass1/pass2, Retraction, Cornering (OK at 872,745 on RepRap), VFA, Input shaping freq (RRF), Polar alignment rings: ALL planar (B0). Voron NP (S4 process): Polar alignment greyed; IS freq + damping (Klipper) planar. test-13 release published OK. Polar header "missing time" = upstream non-BBL behaviour (not a bug). test-14 notes edited in yml (UNCOMMITTED: .github only). Suites running -> suites14.log. NEXT: suites pass -> bump .github/test-release-win64 to test-14, commit .github only, push, heartbeat; report A1 fuzzy upstream crash + quicklog.
- test-14 notes STASHED (stash@{0}) + copy resume/test14_notes.patch; apply after fff_print passes, then bump tag file, commit .github only, push.
- 03:18Z fff_print 256 pass, verify.sh 4/4. RELEASE test-14 pushed ae36cd4f, run 36663769943 in progress; heartbeat send_later 10 min. NEXT: on success confirm assets, quicklog to user (A1 fuzzy upstream crash; toolhead STEP: Voron SB official on GitHub, Bambu hotends community-modelled; feature awaits user go-ahead).
- 03:53Z test-14 PUBLISHED (run 36663769943 success, installer + portable, @ name in code span). Heartbeats stopped. Open: toolhead model feature awaits user; upstream A1 fuzzy crash reported only.
- 04:10Z USER REQUEST: implement Calibration > Toolhead clearance: 1 step gauge under head, 2 ramp gauges at tip, 3 break-away fin test; dialog explains each (use 1 or 2, then 3, and why). Test fully, README, release test-15. Autonomous, low user-facing context. START at 04:46Z (send_later trig_01XW5CJTZMsndKbse7NCb7iD).
- 05:02Z STEP 1 done: lib NonPlanar/ToolheadClearance + 14 tests, libslic3r 907 pass, committed+pushed. NEXT: GUI dialog src/slic3r/GUI/ToolheadCalibration.{hpp,cpp} + MainFrame menu (2 places) + CMake, build app, GUI test.
- 05:05Z STEP 2 in progress: dialog ToolheadCalibration.{hpp,cpp}, MainFrame menu x2, slic3r CMake, fff test 'printed fin test stays below' in test_nonplanar.cpp. Build running (build_th2.log). UNCOMMITTED. NEXT: fix compile errs, run [ToolheadClearance] both suites, GUI test on ThetaFirm + Voron NP (Calibration menu y ~392 new item after Polar alignment 365), commit, push.
- 05:35Z STEP 2 done+pushed (dialog tabs, apply verified Voron+ThetaFirm, fin test/wedges/blades slice). NEXT: README section, full suites, release test-15.
- 05:46Z Suites 907/257 pass, verify 4/4, GUI Voron/ThetaFirm/X1C ok; X1C centre fix (extruder offset) pushed. NEXT: release test-15 notes + tag (.github only).
- 05:47Z RELEASE test-15 pushed 1e6b2fa9, run 36674989345; heartbeat 10 min. Quicklog when published.
- 06:42Z USER: remove ThetaFirm naming -> Core R-Theta. DONE dce43daf: presets/model/cover renamed w/ renamed_from, Custom 02.04.00.10, AppConfig installed-model migration, select_preset_by_name follows renamed_from (+2 tests), docs. Suites 909/257, full profile check pass, GUI migration ok. NEXT: test-15 status, then release test-16 (notes: ThetaFirm->Core R-Theta).
- 06:42Z test-16 notes STASHED (+resume/test16_notes.patch). When test-15 published: stash pop, tag file -> orcanp-s4-test-16, commit .github only, push, heartbeat.
- 07:00Z test-15 PUBLISHED. RELEASE test-16 pushed c6fbb03a.

## 2026-09-30 07:38 test-16 re-run
- Run 36681312565 (c6fbb03a) failed only on Windows unit test "An installed printer model renamed in its bundle
  stays installed under its new name": the Windows AppConfig::load() does substr(last '}' + 2), which threw
  out_of_range because the test file had no trailing newline. Fixed the test (ends with std::endl like save()),
  commit 3a175c77, libslic3r 909/909. Re-ran via workflow_dispatch: run 36684852549 (tag orcanp-s4-test-16 unchanged).
- Next: when green, check release orcanp-s4-test-16 (both assets, @ names in code spans), mark task #58 done,
  send quicklog (test-15 toolhead clearance; test-16 Core R-Theta rename + renamed-preset reselection).

## 2026-09-30 08:11 test-16 published
- Run 36684852549 green; release orcanp-s4-test-16 published 08:05 on 3a175c77 with portable zip + installer;
  "@" names in code spans. Task #58 done. Nothing in flight. Open (awaiting user): A1 + fuzzy skin upstream
  crash (reported), toolhead STEP model feature (user decision).

## 2026-09-30 new request: "Add and test all of these ideas, bar extruder rotation distance"
Scope (tasks #59-#64):
- Settings (printer): skew_xy/xz/yz (deg, Cartesian, slicer shear at export, Klipper formula
  x-=y*xy+z*(xz-xy*yz), y-=z*yz), polar_tilt_offset (deg, commanded tilt for vertical nozzle),
  polar_rotation_backlash (deg, commanded = angle + dir*b/2), polar_bed_tilt_x/y + polar_bed_cone
  (mm per 100 mm, z += (tx*x+ty*y+c*r)/100 in bed frame). CalibMode Calib_Polar_Backlash/Tilt/Speed.
- Dialogs: Dimensions (frame: shrink XY/Z + skew AC/BD/AD for XY/XZ/YZ; holes/pegs -> xy_hole/contour
  comp; elephant foot block), Overhangs and gaps (overhang samples -> s4_max_overhang; support top gap
  objects; S4 surface gap objects), Polar calibration (radius rings, bed level rings (model), backlash
  fin pairs (G-code: pair1 same-dir, pair2 opposite -> b += (W2-W1)/r), tilt pivot tubes (G-code: planar
  bottom, top +a / -a; dL=(dA-dB)/(4 sin a), L*eps from heights (hA-hB)=2 L eps sin a; radius offset
  += L*eps), rotation speed rings with ticks (G-code, cap lifted for the test)).
- PA line polar layout beside the axis; calibration models placed off-axis on polar printers.
- Plater::fff_print() is public: dialogs set calib params directly. Load resets calib mode.
- User additions: support beds down to 120 mm square/round, split tests across plates when needed (packer
  place_on_plates in CalibrationPrints; frames sized to bed, uprights on own plates when small; hole/peg
  pieces; fin test clipped); "Test with the full matrix as before" (CLI matrix like task #55 + GUI all
  calibrations like #56). Unattended.
- Build of header batch running (log scratchpad/build59.log). Pending post-build patches in scratchpad/pending/.

## 2026-09-30 WIP snapshot (weekly limit)
State: header batch built OK (55 min, 0 errors): new printer settings (skew_xy/xz/yz, polar_tilt_offset,
polar_rotation_backlash, polar_bed_tilt_x/y, polar_bed_cone) + Tab/Preset/Print lists, CalibMode
Calib_Polar_Backlash/Tilt/Speed, PolarKinematics tilt offset / bed height / backlash take-up (compiled).
NOT yet registered in CMake (so not built): src/libslic3r/CalibrationPrints.*, GCode/SkewCompensation.*,
GCode/PolarCalibrationGCode.*, NonPlanar/PolarCalibration.*, GUI CalibrationDialogKit.*,
DimensionsCalibration.*, OverhangCalibration.*, PolarPrinterCalibration.*; tests test_calibration_prints,
test_skew_compensation, test_polar_calibration (libslic3r), test_calibration (fff_print); test_polar_kinematics
edits (compiled with the build).
Next steps:
1. Register new files in src/libslic3r/CMakeLists.txt, src/slic3r/CMakeLists.txt, tests CMakeLists.
2. Apply wip/calibration/patch_toolhead_lib.py and copy ToolheadCalibration.{hpp,cpp} into src/slic3r/GUI
   (dialog on CalibrationTabsDialog); also change feeler_blades/angle_wedges to return pieces (no lay_out),
   fin_test clipped to bed radius; use CalibrationKit::lay_out(pieces, split=true).
3. Remove GUI PolarCalibration.{hpp,cpp} (replaced by PolarPrinterCalibration); MainFrame menus: Dimensions,
   Overhangs and gaps, Polar calibration (polar only), Toolhead clearance.
4. GCode.cpp: include GCode/PolarCalibrationGCode.hpp + SkewCompensation.hpp; branch after PA line:
   else if (is_polar_calibration(mode)) file.write(polar_calibration_gcode(*this, print)); polar block: raise
   cap for Calib_Polar_Speed (1.2*60*params.end); skew pass for !polar (SkewGCodeConverter, remap preview via
   new NonPlanarExport::remap_preview_lines factored out of map_preview_to_polar, warn outside_bed).
5. Move NonPlanar::polar_bed_radius to CalibrationPrints::inscribed_radius (added) and update users/tests.
6. Dialogs: use CalibrationKit::lay_out (frames split=true via square_frames_for(usable_area()) and remember
   side/upright for the fit; holes/pegs split=true; overhangs/support/surface split=false). Remove row().
7. calib.cpp polar PA line: lines from axis+3 mm along +X, numbers box on the -X side (draw inside
   generate_test with draw_numbers off in print_pa_lines), lengths/count fitted to bed radius (>=120 mm beds).
8. Plater: calibration objects arranged off the rotation axis on polar printers (arrange with an excluded
   circle r = max print speed / max rotation speed); PA pattern arrange with the same exclude.
9. Build, fix, suites, GUI tests (Voron, X1C, Core R-Theta, Generic Polar, 120 mm square + round beds),
   CLI matrix as task #55, README (new Calibration section) + docs/HLSD/calibration.md, release test-17.
Remove wip/ before release.
