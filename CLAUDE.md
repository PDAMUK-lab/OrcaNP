# CLAUDE.md — permanent memory (condensed; replaces @AGENTS.md, whose rules are all below)

## Standing rules (user, always)
- Semi-autonomous: keep user-facing messages short. End a work session with a quicklog (brief log of what was done).
- Step-wise development; verify each step against the whole (build, suites, end-user GUI test) before the next.
- Make safe at each step: commit + push. Keep a resume file (`<scratchpad>/resume/RESUME.md`) current so work
  resumes after usage limits. Test like the end user; release when ready.
- Never lock the user in to a change a setting already lets them make (e.g. infill direction): no forced or silently
  overridden settings; suggest the setting (docs, tooltips), or refuse an unsafe combination with a message naming it.

## Project
OrcaNP: OrcaSlicer fork (C++17, selective C++20, wxWidgets, CMake) adding S4 non-planar slicing and polar
printers (ThetaFirm Core R-Theta). Non-planar code: `src/libslic3r/NonPlanar/` (S4Deformation, Tetrahedralize,
LayerShapes, PaintedSurface, S4Mapping, S4GCodeTransform, PolarKinematics), `PrintObjectSlice.cpp` (`deform_s4`),
`GCode/NonPlanarExport`, tests `tests/fff_print/test_nonplanar.cpp` (`[NonPlanar]`), design `docs/HLSD/nonplanar-s4.md`,
user docs `README.md`. Key upstream entry points: `src/OrcaSlicer.cpp` (startup), `src/libslic3r/Print.cpp` (pipeline),
`src/libslic3r/PrintConfig.cpp` (all settings), `src/slic3r/GUI/`, `src/libslic3r/{GCode,Fill,Support,Geometry,Format,Arachne}/`,
profiles `resources/profiles/<Vendor>.json`.

## Git / releases
- Work only on branch `claude/s4-slicer-parsing-tfwcjo`; push with `git push -u origin <branch>`. No PR unless asked.
  No model names/IDs in commits; end commits with the attribution trailer the session supplies.
- Win64 test pre-release: first line of `.github/test-release-win64` is the tag (`orcanp-s4-test-N`); notes live in
  `.github/workflows/test_release_win64.yml`. Pushing either file starts the run (~1 h). Never push unbuilt or
  untested code in the release commit; commit only `.github/` for it.
- Release notes: names with `@` (process presets) go in code spans; GitHub reads `@name` as a user mention and
  lists that user as a release contributor.
- `docs/superpowers/` (plans, brainstorms) is gitignored: never commit it.

## Build and test (Linux container)
- `cd build && ninja -f build-Release.ninja [-k 0] fff_print_tests libslic3r_tests src/Release/orca-slicer`.
  A header change (Model.hpp, PrintConfig.hpp) rebuilds ~600 files (~1 h): batch header edits; never edit sources
  while a build runs. Other platforms: `cmake --build build --config RelWithDebInfo --target all` (Linux/macOS
  `build/arm64`), Windows `cmake --build . --config <type> --target ALL_BUILD -- -m`.
- Tests (Catch2): `build/tests/fff_print/Release/fff_print_tests "[NonPlanar]"`; suites
  `ctest --test-dir build/tests/libslic3r -C Release -j2`, same for `tests/fff_print`. Run timing/multithreaded
  tests alone on idle cores. Catch name filters split on commas: use tags or `*` wildcards. Test-writing rules:
  `tests/AGENTS.md` (suite by production code, `test_<subsystem>.cpp`, behavioral names, PascalCase tag, set the
  keys you rely on, `WithinAbs`/`WithinRel` not `==` for floats, no `&&`/`||` in one assertion, `DYNAMIC_SECTION`
  in loops, no asserts from threads, green under `--order rand`).
- CLI slice: `orca-slicer --datadir <dir> --load-settings "machine.json;process.json" --slice 0 --outputdir <out> model.stl`
  (needs relative E and `layer_change_gcode` "G92 E0").
- GUI end-user tests: `Xvfb :99 -screen 0 1600x1000x24 -fbdir <dir> -nolisten tcp`, run the app with
  `DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 ... --datadir <dir>`; kill with
  `pgrep -f "^/home/user/OrcaNP/build/src/Release/orca-slicer"` (unanchored `pkill -f` kills its own shell).
  Dropdowns do not render under Xvfb: set values through presets/config. XTest key for '.' is `period`.
- Test printers: ThetaFirm Core R-Theta (polar); "Klipper Voron Designs V2.4r2 300mm" (Printer presets tab) for
  Z-only non-planar, toolhead 2.2° all-round clearance from the nozzle edge out to 65.5 mm wide, 126 mm high.

## Documentation
Docs in `docs/`; a subsystem's high-level design in `docs/HLSD/<subsystem>.md`: the design as it stands (what,
why, constraints), no phases/task lists/status or before-after framing. Write one only when the code does not make
the design evident; update it in the same change that invalidates it.

## Code style and constraints
- PascalCase classes, snake_case functions/variables, `#pragma once`, RAII/smart pointers, TBB parallelism (mind
  shared state). Top-level windows: `SetSizerAndFit(sizer)`, or `SetSizer` then `sizer->SetSizeHints(window)`.
- Backward compatible .3mf projects and printer profiles; format/profile changes need migration. Cross-platform
  (Windows, macOS, Linux). Dependencies build separately in `deps/build/`.
- Review focus: no regressions in behaviour, defaults, profiles or project compatibility; option-gated features
  change nothing when off; follow existing style/architecture (justify architectural changes in comments and
  PR text); reuse before adding helpers; concise code; targeted tests or documented verification for behaviour
  changes. Profile changes under `resources/profiles/<Vendor>/` bump `version` in `resources/profiles/<Vendor>.json`.

## Localization (`localization/i18n/<lang>/OrcaSlicer_<lang>.po`, template `OrcaSlicer.pot`)
- Terms per the Localization glossary (OrcaSlicer_WIKI developer_reference/localization_glossary.md): same English
  term, same translation; keep brand/product names, acronyms, materials, file formats, G-code tokens, macros and
  identifiers in English. Changing an established term: update the .po files and `localization_glossary.tsv`.
- Translate meaning, not words (Flow ratio = multiplier, Flow Rate = throughput, Flow Dynamics = pressure
  compensation; "extruder" may mean toolhead, feeder or nozzle). One template per recurring message shape.
- Edit only `msgstr`, never `msgid` (report wrong English). Keep placeholders (`%s %d %1% %zu %%`), every `\n`
  (count and position), leading/trailing spaces, HTML tags, `℃`, encoding and line endings. Never reorder
  positional arguments in c-format strings. Read `msgctxt` (e.g. Back/Camera View vs Back/Navigation); disambiguate
  in source with `_L_CONTEXT`/`_u8L_CONTEXT`. A literal `%` in a possible-c-format string: add
  `// xgettext:no-c-format, no-boost-format` above it in the source.
- Plurals: use the catalog's `nplurals` (ja/ko/zh/th/vi 1, ru/cs/pl/lt 3, uk 4); inflect each form (repeating one
  is a bug in Slavic/Baltic, correct in Turkish/Hungarian). msgstr == msgid or an empty plural form = untranslated.
- Mark machine translations `# AI Translated` (not on human translations you did not rewrite). Do not reflow
  unrelated entries. Verify: `scripts/run_gettext.bat --full` exits 0, or
  `msgfmt --check-format -o out.mo <file>.po`. Clear `fuzzy` on entries you fix.
