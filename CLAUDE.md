# lowpoly-fracture: agent map

A destruction-first engine: low-poly convex pieces instead of voxels, on vendored Box3D (behind one interface,
`src/phys.h`). Priorities: agent-friendliness,
then performance, then everything else. Determinism is mandatory (docs/determinism-rules.md).

## Read this, skip that

First-party code is about 240k tokens (the core, `src/` and `lpf.h`, about 130k; the tests 60k): read what a task
needs. `extern/` (sokol, imgui, Box3D; about 1.9M tokens) and
`app/sandbox/shaders/generated/` are vendored or generated: never read them whole. The Box3D API is in
`extern/box3d/include/box3d/*.h`; our patches and known Box3D issues are in `extern/box3d/PATCHES.md`.
`docs/research/` (milestone 7's track reports, about 150k tokens) is evidence for `docs/multiplayer-research.md`: read
a track only when a decision needs its detail.

## Map

| path | what it is |
|---|---|
| `include/lpf/lpf.h` | the whole public API: materials, world and object defs, links, vehicles, rigs, impacts, pulls, commands, stats, queries |
| `include/lpf/lplab.h` | the two-world lab for experiments, never a game: causal units, injected desyncs, the hash followed down between two worlds, repair by unit |
| `include/lpf/lpmath.h`, `src/lpmath.c` | vector maths (`lpVec3`, `lpQuat`, `lpTransform`, `lpPos`, ...), Box3D's own taken over with each operation kept; the trig is hand coded for determinism |
| `src/core.h/.c` | asserts, growable arrays (`LP_ARRAY`), PCG32 random, `lpMix64`, `lpCbrt`, `lpFloatToInt`, a radix sort, the timer (`lpGetTicks`), the floating-point guard (`lpFpGuard`) and the determinism self-test |
| `src/poly.h/.c` | convex polyhedron (`lpPoly`), plane clipping, mass, `lpShape` (compact immutable copy) |
| `src/phys.h`, `src/phys_box3d.c` | the physics interface: every rigid-body operation the core uses (bodies, hull shapes, joints and motors, contacts, hits, moves, overlap and casts; quickhull and GJK) on opaque handles, reports in piece and body indices and in our order; the Box3D backend is the only file that sees Box3D's headers (the build enforces it) |
| `src/fracture.h/.c` | fracture patterns (Voronoi, grain, radial, masonry), impact sites, sliver absorption, keeper merging, cell bonds, and the fracture job (`lpFracture_RunJob`: a snapshot in, classified cells, hulls and bonds out, world-free) |
| `src/facet.h/.c` | flat-shaded render meshes per piece, interior colours |
| `src/tasks.h/.c` | thread pool with a blocking parallel-for (fracture jobs) |
| `src/world.h` | internal layout of `lpWorld` and the core's shared internals, used by tests too |
| `src/world.c` | materials table, world, objects, pieces, bodies, bonds, piece queries, stats, validation |
| `src/command.c` | commands: the tick-stamped queue (`lpWorld_Submit`), applied in (peer, seq) order as the step begins; who controls a vehicle or rig; templates for spawns |
| `src/hash.c` | the state hash: an element per body, piece, stress state, link, ...; categories as order-free sums, kept per slot and rehashed where marked; the descent API and `lpWorld_CheckHash` |
| `src/session.c` | what machines that play together must agree on, as "key value" lines (`lpWorld_DescribeSession`: the def by field, digests of materials, joints and templates, the self-test, the hash), and `lpSessionCompare`, which names the first key that differs |
| `src/units.c` | causal units: union-find over bodies joined by contacts (not through anchored pieces or rubble), links, vehicles, rigs, the ground under wheels and feet, shared detonators and pools; each element of the hash mapped to its unit (for the two-world lab) |
| `src/lab.c` | the two-world lab: injected desyncs (a velocity, a warm start), `lpLab_Diff` (the hash followed down to elements), repair by unit (motion, warm starts, sleep timers); `bench/twin.c` runs it (`lpf_bench --twin warm:200 --repair motion,warm,sleep@4 --cone path`) |
| `src/impact.c` | impacts: choosing the pieces and integrating their fracture jobs, bond damage, detonators and fuses, blast forces, collision hits |
| `src/split.c` | splitting bodies into components (tiered by volume); structures are queued for the stress check |
| `src/solve.h/.c` | a structure's stress system and its math, world-free: beam kernel, K·x, block-Jacobi, conjugate gradient (whole, or on a region with the rest held) |
| `src/stress.c` | the stress check per structure (and per moving body that asks for it: inertia relief): scheduling and budgets, loads, building systems (kept per body while solving), the speed of propagation (region solves that grow `stressHopsPerTick` bonds a step), judging joints and slender pieces, strain, settling at load |
| `src/link.c` | links: physics joints between objects that break under load or blasts and follow their pieces; motors (servos toward a target, capped by health and supply, jammed by damage) |
| `src/wheel.c` | vehicles: wheels are links with no joint (a shape-cast suspension and an impulse solve for grip per chassis body), controls, wheels that come off |
| `src/rig.c` | rigs (walkers): limbs as chains of motorised hinges, the kinematic model from link frames and angles, IK, capability per limb, the rig's centre of mass and support margin, reaching and touching, a game's own walking (`lp_walkerNone`: foot targets and a pose), state |
| `src/gait.c` | the built-in walker (`lpWalkRig`, tuned by `lpRigDef.gait`): desired pose, free gait with a balance check, swings and foothold casts, holds, crawling when maimed, strikes |
| `src/supply.c` | supply channels: which pieces each channel's sources reach over carrier bonds and links (fuel to the engine, power to the wheels), recomputed when carriers change, its changes travelling `supplyHopsPerTick` carriers a step (the supply wave); pools (hydraulic fluid) and their leaks |
| `src/step.c` | pulls, wakes, freezing rubble, and the order of `lpWorld_Step` |
| `src/debris.c` | debris tiers: ghosts, scrap, light and full debris, loose grid, shove, budget ladder, filters |
| `scenes/` | procedural scenes (walls, house, town, tower, pile, lumber, ruins, yard, keep, track, mech, contraption), the car kit (`lpAddCar`), a crane (`lpAddCrane`), the hexapod mech (`lpAddHexapod`), scripted bombardment and drivers (`lpSceneDrive`: laps, the mech's patrol, the crane; the scene's commands); `lpBuildScene` settles their structures. `script.c`: the replay scripts (commands as text: read, write, submit; the sandbox's tools live here), `dump.c`: a world's state as JSON |
| `app/net/` | lockstep co-op, outside the core: transports (`net.c`: TCP, in memory, and `lpFault`: a link delayed or stalled, for tests) and the protocol (`lockstep.c`: session handshake, the host's clock, one packet a tick, hash reports, a mismatch followed down over the wire); `bench/pair.c` runs it headless (`lpf_bench --host 7777` / `--join host:7777`, `--inject-desync`, `--net-delay ms,jitter`, `--net-stall tick,ms`, `--leave-at tick`) |
| `bench/main.c` | headless benchmark: `lpf_bench --scene town --workers 1,8 --json out.json` (`--check-hash`: the kept hash against a full one every tick) |
| `test/` | `lpf_test` runs everything; `lpf_test stress` runs one suite (`poly`, `fracture`, `world`, `debris`, `stress`, `links`, `vehicles`, `systems`, `rigs`), `lpf_test stress TestKeepBreach` one test. Each test has a kind (outcome, determinism: the contract; mechanism; timing): `--contract`, `--kind k`, `--list`, `--check-catalogue docs/catalogue.md`. `test/coop/*.py`: co-op scenarios in sandbox windows, driven by agents' tools (`python tools/coop.py test`; local, Windows) |
| `app/sandbox/` | sokol + imgui sandbox: tools, record and replay, co-op (`--host 7777` / `--join host:7777`, lockstep through app/net), driving and walking (`drive.cpp`: keys to recorded controls, chase camera, a mech's strikes and grabs), renderer (vertex pulling; wheels drawn from their state), PNG screenshots; the control port (`control.h`, `--control 0`): an agent drives a window over loopback, one request a line, JSON back (keys, the camera, the clock held and stepped, turns, screenshots with or without the panel, state) |
| `tools/` | `build.ps1`, `devenv.ps1` (MSVC environment), `check-determinism.ps1` (headless), `bench.ps1` (ladder), `get-shdc.ps1`, `catalogue-shots.ps1` (the catalogue's contact sheets), `ref-frames.ps1` (reference clips beside them), `coop.py` (agent-driven co-op: launch one sandbox window per player, send them requests, step or take turns, screenshots side by side, the scenarios; `python tools/coop.py help`) |
| `bench/baseline.json` | committed benchmark baseline that `tools/bench.ps1` compares against |
| `scripts/` | replay scripts of commands for the sandbox and `lpf_bench --script` (`tick[:peer] tool origin dir`, `tick drive vehicle throttle brake steer handbrake`, `tick walk rig forward strafe turn crouch`, `tick reach rig limb active x y z`, `tick grab rig limb`, `tick impact origin dir radius energy [impulse]`; the full list in `scenes/script.h`) |
| `docs/` | goals (the north star, the kinds of game the engine serves, the feel), the outcome catalogue (`catalogue.md`: the contract, every outcome with its test or contact sheet in `catalogue/`), references (clips to aspire to; links only), feasibility, architecture, determinism rules, materials catalog, roadmap, perf log, multiplayer research (milestone 7's decisions; evidence in `docs/research/`) |

## Commands (PowerShell; run from the repo root)

```powershell
pwsh tools/build.ps1 -Test                      # build msvc-release, run all tests
pwsh tools/build.ps1 -Preset msvc-asan -Test    # ASan build and tests, with lpf asserts on
pwsh tools/build.ps1 -Shaders                   # after editing app/sandbox/shaders/scene.glsl
pwsh tools/check-determinism.ps1 -Scene walls -Script scripts/walls_demo.txt   # 1/4/8 workers must match (headless)
build/msvc-release/bin/lpf_test.exe --contract                 # the outcome catalogue's tests (docs/catalogue.md)
build/msvc-release/bin/lpf_bench.exe --scene mech --script scripts/mech_arms.txt --period 0 --ticks 900 --dump 600:build/s.json
pwsh tools/bench.ps1 -Repeat 3                  # perf ladder vs bench/baseline.json; exit 3 if a sim hash changed
pwsh tools/bench.ps1 -StrictSolver              # also exit 3 if a stress solver hash changed (solver refactors)
build/msvc-release/bin/lpf_bench.exe --scene pile --workers 1,8
build/msvc-release/bin/lpf_bench.exe --scene town --period 3 --ticks 300 --check-hash   # the kept hash against a full one
build/msvc-release/bin/lpf_bench.exe --scene town --ticks 300 --host 7777   # then, in a second shell: --join 127.0.0.1:7777
python tools/coop.py launch --scene track --players 2   # agent-driven co-op windows, held; then p1 key V tap, step 30, shot, stop
python tools/coop.py test                               # the co-op scenarios in test/coop (windows, so local only)
build/msvc-release/bin/lpf_bench.exe --scene town --period 3 --ticks 600 --twin warm:200 --repair motion,warm,sleep
build/msvc-release/bin/sandbox.exe --scene lumber --script scripts/lumber_demo.txt --frames 120 --screenshot build/shots/x.png --hide-ui
```

Visual checks: run the sandbox with `--frames N --screenshot path --hide-ui` (optionally `--camera x,y,z,yawDeg,pitchDeg`)
and look at the PNG. Multiplayer checks: `python tools/coop.py launch`, then drive each player's window by its requests
(`coop.py help` says how the sandbox plays), step the shared clock, and read `coop.py shot` and `coop.py all state`;
several agents can each play one player with `coop.py turn pK N`.

## Rules of the house

- Determinism (docs/determinism-rules.md; bit-identical across worker counts, compilers, OSes, x64 and ARM64): no FMA
  or fast-math, no C-library transcendentals in simulation or scene code (`lpCbrt`, `lpAtan2`, `lpComputeCosSin`),
  PCG32 seeded from state, index-order iteration, total-order sorts, what the physics engine reports acted on in our
  own order, no side effects inside one call's arguments or one initializer, `lpFloatToInt` for unbounded
  conversions, pure parallel jobs, nothing reads the camera or the clock. A refactor that is meant to be
  behaviour-preserving must keep the `lpf_bench` hashes identical (`tools/bench.ps1` checks them; `-StrictSolver` for
  the stress solver's state too). The `determinism` CI (GitHub Actions, 14 legs, on every push to `sandbox`) checks
  every OS, CPU and compiler; read a run with `gh run list` / `gh run view`.
- Performance: every per-step cap is a count, never a time budget. Log before and after numbers in docs/perf-log.md.
- Stress solver changes: the oracle tests (`lpf_test stress TestKeepLocalHit`, `TestDriftSmallStructures`,
  `TestKeepBreach`, `TestKeepHole`) check every judgement on a reduced system against an exact solve.
- Gotchas: the agent harness turns `\n` inside Bash heredocs and inline python strings into real newlines, so write C
  string escapes with the Edit or Write tools. ASan binaries need the MSVC runtime on PATH: dot-source
  `tools/devenv.ps1` first.
- Never commit `.env` (API keys), `refs/` (third-party reference art), `build/` or `tools/bin/`.
- The engine is MIT. The games live in their own repositories under their own, more restrictive licences: never put
  game code, game assets, game-specific policy or game pitches here (docs name the games only neutrally, e.g. "a
  rigging-race game"). The kits in `scenes/` are example content. The game designs are in the private sibling repo
  `lowpoly-games` (at `../lowpoly-games` when checked out, `docs/games.md`): read it for context, never copy from it.
