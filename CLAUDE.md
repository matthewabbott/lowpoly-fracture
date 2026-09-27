# lowpoly-fracture: agent map

A destruction-first engine: low-poly convex pieces instead of voxels, on vendored Box3D. Priorities: agent-friendliness,
then performance, then everything else. Determinism is mandatory (docs/determinism-rules.md).

## Read this, skip that

First-party code is about 90k tokens and fits in one context. `extern/` (sokol, imgui, Box3D; about 1.9M tokens) and
`app/sandbox/shaders/generated/` are vendored or generated: never read them whole. The Box3D API is in
`extern/box3d/include/box3d/*.h`; our patches and known Box3D issues are in `extern/box3d/PATCHES.md`.

## Map

| path | what it is |
|---|---|
| `include/lpf/lpf.h` | the whole public API: materials, world and object defs, impacts, pulls, blows, stats, queries |
| `src/core.h/.c` | asserts, growable arrays (`LP_ARRAY`), PCG32 random, `lpMix64` |
| `src/poly.h/.c` | convex polyhedron (`lpPoly`), plane clipping, mass, `lpShape` (compact immutable copy) |
| `src/fracture.h/.c` | fracture patterns (Voronoi, grain, radial), impact sites, sliver absorption, keeper merging, cell bonds |
| `src/facet.h/.c` | flat-shaded render meshes per piece, interior colours |
| `src/tasks.h/.c` | thread pool with a blocking parallel-for (fracture jobs) |
| `src/world.h` | internal layout of `lpWorld`, shared by world.c, debris.c and tests |
| `src/world.c` | materials table, objects, bodies, bonds, fracture jobs (3 phases), impacts, weight check, step, hash, validate |
| `src/debris.c` | debris tiers: ghosts, scrap, light and full debris, loose grid, shove, blow, budget ladder, filters |
| `scenes/` | procedural scenes (walls, house, town, tower, pile, lumber) and scripted bombardment |
| `bench/main.c` | headless benchmark: `lpf_bench --scene town --workers 1,8 --json out.json` |
| `test/` | `lpf_test [poly|fracture|world|debris]`; `test_debris.c` covers the tiers |
| `app/sandbox/` | sokol + imgui sandbox: tools, record and replay, renderer (vertex pulling), PNG screenshots |
| `tools/` | `build.ps1`, `devenv.ps1` (MSVC environment), `check-determinism.ps1`, `get-shdc.ps1` |
| `scripts/` | sandbox replay scripts (`tick tool origin dir [n]`) |
| `docs/` | feasibility, architecture, determinism rules, roadmap, perf log |

## Commands (PowerShell; run from the repo root)

```powershell
pwsh tools/build.ps1 -Test                      # build msvc-release, run all tests
pwsh tools/build.ps1 -Preset msvc-asan -Test    # ASan build and tests
pwsh tools/build.ps1 -Shaders                   # after editing app/sandbox/shaders/scene.glsl
pwsh tools/check-determinism.ps1 -Scene walls -Script scripts/walls_demo.txt   # 1/4/8 workers must match
build/msvc-release/bin/lpf_bench.exe --scene pile --workers 1,8
build/msvc-release/bin/sandbox.exe --scene lumber --script scripts/lumber_demo.txt --frames 120 --screenshot build/shots/x.png --hide-ui
```

Visual checks: run the sandbox with `--frames N --screenshot path --hide-ui` (optionally `--camera x,y,z,yawDeg,pitchDeg`)
and look at the PNG.

## Rules of the house

- Determinism: no FMA or fast-math, no C-library trig in simulation code, PCG32 seeded from state, index-order
  iteration, total-order sorts, pure parallel jobs, nothing reads the camera or the clock. A refactor that is meant to
  be behaviour-preserving must keep the `lpf_bench` hashes identical.
- Performance: every per-step cap is a count, never a time budget. Log before and after numbers in docs/perf-log.md.
- Gotchas: the agent harness turns `\n` inside Bash heredocs and inline python strings into real newlines, so write C
  string escapes with the Edit or Write tools. ASan binaries need the MSVC runtime on PATH: dot-source
  `tools/devenv.ps1` first.
- Never commit `.env` (API keys), `refs/` (third-party reference art), `build/` or `tools/bin/`.
