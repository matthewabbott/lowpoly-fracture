# lowpoly-fracture

Teardown-style destruction for a low-poly art style: everything is built from convex polygon pieces that break into
jagged low-poly shards (stone, brick, plaster), long splinters (wood) or radial shards (glass), on the
[Box3D](https://github.com/erincatto/box3d) physics engine. Deterministic across thread counts, built for many
debris bodies.

- `src/`, `include/lpf/lpf.h`: the destruction core (C17), engine-agnostic
- `scenes/`: procedural low-poly test scenes (walls, house, town, tower, pile)
- `app/sandbox/`: playable sandbox (sokol D3D11 + Dear ImGui)
- `test/`, `bench/`: unit, fuzz and determinism tests; headless benchmark
- `docs/`: [feasibility report](docs/feasibility.md), [roadmap](docs/roadmap.md), [architecture](docs/architecture.md),
  [determinism rules](docs/determinism-rules.md)

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
sets the distance). R reloads, B toggles scripted bombardment, P pauses, F1 hides the UI, F12 takes a screenshot.

Automation (used by agents and CI): `--script file` replays tool events (see `scripts/`), `--record file` writes them,
`--frames N` runs exactly N ticks and quits, `--screenshot out.png` saves the last frame, `--hash-log file` writes the
per-tick state hash, `--camera x,y,z,yawDeg,pitchDeg`, `--hide-ui`, `--vsync 0`, `--workers N`,
`--render-scale 0.5` (chunky retro pixels). Set `LPF_DEBUG=1` to log impacts and weight checks.

## Third-party

Box3D (MIT, vendored at a pinned commit, `extern/box3d/PATCHES.md`), sokol (zlib/libpng), Dear ImGui (MIT).
Design ideas credited to [Nebenan](https://github.com/Holz231/Nebenan) (MIT) in `docs/architecture.md`.
