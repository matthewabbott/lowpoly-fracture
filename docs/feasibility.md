# Feasibility: low-poly Teardown-style destruction

> A snapshot of the first push (commit 8a855c6). Numbers since then are in [perf-log.md](perf-log.md) (the debris
> tiers made every scene 4 to 10 times cheaper), and plans are in [roadmap.md](roadmap.md).

**Verdict: feasible, and now demonstrated.** The sandbox in this repo breaks buildings, walls, trees, fences and glass
into jagged convex low-poly pieces on Box3D. It runs at about 200 fps on an RTX 3060 laptop (i7-10870H) under
continuous bombardment of a 12-house town, and stays bit-for-bit deterministic across thread counts and builds.

Measured 2026-09-27. Numbers come from `lpf_bench` (headless, 600 ticks at 60 Hz, a grenade or cannon blast every
12 ticks, i.e. 5 blasts per second) and the sandbox's `--frames` mode.

## What was built

| Area | State |
|---|---|
| Convex fracture core: clipping, impact / grain (splinter) / radial (glass) patterns | done, fuzz-tested |
| Destructibles: bonds, damage, splitting, anchors, debris budget, rubble freezing | done, invariants validated every tick in tests |
| Collision damage (Box3D hit events), explosions, detonating objects, pull/grab | done |
| Stress solve: structures crack, hinge and topple where they are weak (joints, masonry, slender beams) | done |
| Parallel fracture (3-phase, thread-count independent) | done |
| Renderer: flat-shaded facets, procedural cut-face colours, shadows, fog, dust, retro render scale | done |
| Sandbox: 8 scenes, 8 tools, record/replay, scripted runs, screenshots, hash logs | done |
| Links: breakable welds, hinges, ball joints and ropes between objects; winches | done |
| Vehicles, characters, authored art | not started (next steps) |

## Measured performance (8 workers unless noted)

| Scene | step avg | step p95 | fracture avg | physics avg | peak pieces | awake debris cap |
|---|---|---|---|---|---|---|
| town (12 houses, trees, tower) | 4.0 ms | 9.0 ms | 0.54 ms | 3.4 ms | 10.8k | 1500 (reached) |
| walls | 2.9 ms | 6.1 ms | 0.48 ms | 2.3 ms | 3.8k | 1500 (reached) |
| pile (512 loose crates/rocks) | 9.9 ms | 15.7 ms | 0.56 ms | 9.2 ms | 4.9k | 1500 (reached) |
| town, 1 worker | 12.5 ms | 24.9 ms | 0.76 ms | 11.7 ms | | |

- Full sandbox frame on the town under fire: **5.1 ms average** (4.3 ms simulation, 0.64 ms CPU render and sync,
  234k triangles in 14 draw calls).
- Parallelizing fracture cut the town's worst fracture step from 40 ms to 18 ms and the p95 step from 18 ms to 8 ms.
- For reference, Box3D's own `convex_pile` benchmark on this laptop: 21 ms per step at 1 thread, 5.2 ms at 6.
- **Budget reality:** 1,500 simultaneously moving debris bodies is where physics costs 3 to 9 ms. Frozen rubble and
  sleeping bodies are nearly free, so the cap on *awake* debris is the number that matters, as in Teardown.

## Research findings that shaped it

- **Box3D** (Erin Catto, v0.1 alpha, June 2026, MIT, C17): soft-step solver, SIMD hull collision aimed at destruction,
  cross-platform determinism independent of thread count, record/replay. Its gaps (no vehicle controller, meshes only
  on static bodies) are not blockers. Chosen over Jolt (mature, has vehicles, larger and harder to modify) for
  hackability and determinism.
- **yacineMTB's engine** is an unreleased CUDA port of Box3D. GPU rigid bodies are not bit-exact across GPUs, so they
  are unsuited to lockstep play anyway.
- **Teardown** uses per-object voxel volumes, flood-fill island splitting and no stress model. Its multiplayer
  replicates destruction as deterministic commands (fixed-point cuts) and syncs body motion separately.
- **Proven polygon lineage**: Müller et al. 2013 (convex pieces ∩ impact-centred Voronoi pattern), Gustafsson's own
  *Smash Hit* (runtime convex fracture), NVIDIA Blast (bond graph + stress solver).
- **Nebenan** (independent, MIT) does this on Box3D too; it was mined for ideas (see architecture.md).

## Answers to the original questions

- **Polygons instead of voxels?** Yes. Convex cells are cheaper colliders than voxel volumes, meshes are tiny, and
  the jagged Voronoi rims *are* the low-poly look. The trade-off: holes are carved in cells (about 7 to 20 cm), not
  10 cm voxels everywhere, and very fine repeated carving costs more pieces.
- **Do objects need modelled interiors?** No. Each material has a procedural "solid" interior colour evaluated per
  cut face in object space (wood rings follow the grain across all fragments). Composite objects are assemblies of
  parts with their own materials, like Teardown's.
- **What must art look like?** Destructibles are authored as *convex parts* (boxes, prisms, hulls). Low-poly art is
  naturally built that way. Arbitrary imported meshes need an offline convex decomposition (CoACD/V-HACD) or should
  stay non-destructible (characters, props).
- **Determinism?** Achieved for same-binary play across thread counts, reruns and build configs. Cross-OS is designed
  for but unverified.
- **Can it support a rigging-race game?** The core pieces are there: blowing up walls, collapsing structures, hauling
  objects (pull), explosive props (detonators), heavy impacts (cannonball), breakable ropes, hinges and winches
  (links). Missing: vehicles (raycast vehicle on Box3D's wheel joint), AI drivers, level tooling.
- **And a hauling game with volatile cargo?** Detonators already model volatile flasks with a trigger speed. The
  "jostle" mechanic needs lower-threshold contact events and liquid state, which are gameplay code on top.

## Known limits and risks

1. **Fracture spikes.** One blast that refractures 100+ pieces still costs 20 to 90 ms once. Fix: time-slice jobs
   across frames (the three-phase design allows it), cap cells per blast, or build hulls directly from cell topology
   instead of Box3D quickhull.
2. **Stress at scale.** The stress solve's per-structure budget suits buildings of a few hundred pieces; a
   2000-piece building would take hundreds of steps to decide. Local solves and a coarse far-field solve are the next
   milestone (roadmap section 4).
3. **Box3D is alpha.** It is pinned and vendored; upstream changes need re-validating with the hash tests.
4. **Visual polish.** Glass cracks are invisible until pieces fall (plane-keyed colours hide them); foliage blasts
   into many small chunks; debris pops out of existence at the budget (fade or shrink instead).
5. **Content pipeline.** Destructible art must be convex parts; tooling to author or decompose it is not built yet.

## Sources

- Box3D: https://github.com/erincatto/box3d, https://box2d.org/posts/2026/06/announcing-box3d/,
  https://box2d.org/posts/2024/08/determinism/, https://box2d.org/posts/2026/07/simd-for-collision/
- Jolt Physics: https://github.com/jrouwe/JoltPhysics (Architecture.md, determinism)
- Teardown: https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html, https://acko.net/blog/teardown-frame-teardown/,
  https://80.lv/articles/teardown-developer-breaks-down-multiplayer-and-voxel-destruction-tech
- Smash Hit fracture: https://blog.voxagon.se/2014/05/13/cracking-destruction.html
- Müller, Chentanez, Kim 2013: https://matthias-research.github.io/pages/publications/fractureSG2013.pdf
- NVIDIA Blast: https://nvidia-omniverse.github.io/PhysX/blast/index.html
- Nebenan: https://github.com/Holz231/Nebenan
- RetroDiffusion low-poly API: https://github.com/Retro-Diffusion/api-examples; Meshy API: https://docs.meshy.ai/en
