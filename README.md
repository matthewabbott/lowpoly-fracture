# lowpoly-fracture

Teardown-style destruction for a low-poly art style: everything is built from convex polygon pieces that break into
jagged low-poly shards (stone, brick, plaster), long splinters (wood) or radial shards (glass), on the
[Box3D](https://github.com/erincatto/box3d) physics engine. Deterministic across thread counts, built for many
debris bodies.

- `src/`, `include/lpf/lpf.h`: the destruction core (C17), engine-agnostic
- `scenes/`: procedural low-poly test scenes (walls, house, town, tower, pile, lumber, ruins, yard, keep, track, mech, contraption)
- `app/sandbox/`: playable sandbox (sokol D3D11 + Dear ImGui)
- `test/`, `bench/`: unit, fuzz and determinism tests; headless benchmark
- `docs/`: [goals](docs/goals.md) (the games, the feel), [feasibility report](docs/feasibility.md), [roadmap](docs/roadmap.md), [architecture](docs/architecture.md),
  [determinism rules](docs/determinism-rules.md), [materials](docs/materials.md), [perf log](docs/perf-log.md),
  [outcome catalogue](docs/catalogue.md) (the contract a redesign must meet), [references](docs/references.md)

## Build (Windows)

Needs Visual Studio 2022 C++ tools (or Build Tools), CMake and Ninja (`winget install Kitware.CMake Ninja-build.Ninja`).

```powershell
pwsh tools/build.ps1 -Test               # msvc-release: configure, build, run tests
pwsh tools/build.ps1 -Preset msvc-asan -Test
pwsh tools/build.ps1 -Shaders            # after editing app/sandbox/shaders/*.glsl (fetches sokol-shdc)
```

## Run

```powershell
build/msvc-release/bin/sandbox.exe --scene town
build/msvc-release/bin/lpf_bench.exe --scene town --workers 1,4,8
pwsh tools/check-determinism.ps1 -Scene walls -Script scripts/walls_demo.txt
```

Sandbox controls: hold right mouse to look, WASD/QE to move (shift is fast), left mouse fires.
Tools 1-7: rifle, grenade, cannon blast, sledgehammer, cannonball, volatile flask, grab/pull (hold; the mouse wheel
sets the distance). R reloads, B toggles scripted bombardment,
P pauses, L shows every link coloured by its load, F1 hides the UI, F12 takes a screenshot. V gets into the nearest
car (WASD drives, S brakes then reverses, space is the handbrake, the camera chases it) or mech (WASD walks and turns,
Q/E step sideways, C crouches, F held strikes at the crosshair with the nearest leg, G grabs and lifts what its claw
touches, G again lets go), and V again gets out.

Automation (used by agents and CI): `--script file` replays a script of commands (`scenes/script.h`; see `scripts/`:
walls, house flasks, tower collapse, tower topple, lumber, ruins, yard, keep, track, mech, mech arms, mech crawl): the
tools by name (`tick rifle ox oy oz dx dy dz`), `tick drive vehicle throttle brake steer handbrake` sets a car's controls
until the next such line, `tick walk rig forward strafe turn crouch` a mech's, `tick reach rig limb active x y z` sends a
leg at a point or back into its gait, `tick grab rig limb` grabs or lets go, `tick impact ox oy oz dx dy dz radius energy
[impulse]` is an impact of your own where the ray hits; `--record file` writes the commands each step applied,
`--frames N` runs exactly N ticks and quits, `--screenshot out.png` saves the last frame (with `--screenshot-at f1,f2`,
those frames, numbered), `--dump tick:out.json` writes the state after that tick as JSON, `--hash-log file` writes the
per-tick state hash, `--camera x,y,z,yawDeg,pitchDeg`, `--follow` (the camera chases the car or mech the commands drive), `--hide-ui`,
`--vsync 0`, `--workers N`, `--input-delay N` (every command of the player applies N ticks late: a feel test of lockstep),
`--render-scale 0.5` (chunky retro pixels). Set `LPF_DEBUG=1` to log impacts and stress solves. Headless,
`lpf_bench --scene walls --script scripts/walls_demo.txt --period 0 --ticks 240 --hash-log out` replays the same events
with the same hashes (the script player is `scenes/script.c`).

## License

MIT (see [LICENSE](LICENSE)).

## Third-party

Box3D (MIT, vendored at a pinned commit, `extern/box3d/PATCHES.md`), sokol (zlib/libpng), Dear ImGui (MIT); each keeps
its own licence file in `extern/`. Design ideas credited to [Nebenan](https://github.com/Holz231/Nebenan) (MIT) in
`docs/architecture.md`.
