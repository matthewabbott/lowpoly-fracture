# Determinism rules

Same inputs in the same order must give bit-identical simulation state, whatever the worker count. This is what
makes replays, lockstep multiplayer and golden-hash tests possible. Box3D guarantees it for the physics; these rules
keep the destruction layer (`src/`) and the app from breaking it.

## Rules

1. **No FMA contraction, no fast-math.** clang: `-ffp-contract=off`; MSVC: `/fp:precise` without `/fp:contract`.
   Set once in the top-level `CMakeLists.txt`; never add `/fp:fast`, `-ffast-math` or `/arch:AVX2` to a target
   that touches simulation state.
2. **No trigonometry from the C library in simulation code.** Use `b3ComputeCosSin` / `b3Atan2` (Box3D's own,
   identical everywhere). Random unit vectors use rejection sampling, not `sin`/`cos`.
3. **Randomness is PCG32 seeded from simulation state**: world seed, tick, piece index and generation
   (`lpMix64`). Never time, addresses or `rand()`.
4. **Iteration order is always an index order.** Pools are arrays with LIFO free lists; nothing iterates a hash map
   or pointer-keyed container. Box3D query results are copied and sorted (`lpQueryPieces`) before use.
5. **Sorts use a total order.** Every comparator breaks ties by an index (`lpCompareHits`, `lpCompareBudget`,
   Voronoi neighbour keys embed the site index), so `qsort` instability cannot leak.
6. **Parallel work is pure.** A fracture job reads only its own snapshot and writes only its own output; results are
   integrated sequentially in job order (`world.c`, "fracture jobs"). Box3D's threading is deterministic by design.
7. **Nothing in the simulation looks at the camera or wall-clock time.** Debris budgets rank by volume, age and
   index; rubble freezing uses Box3D sleep events and tick ages. Timing is measured but never branched on.
8. **Cosmetic state lives outside the simulation.** Dust particles are emitted by the core but simulated only by
   the app; they never feed back.
9. **Inputs are events stamped with a tick.** The sandbox records tool use as text (`--record`), replays it
   (`--script`) and applies it before the step of that tick. A pull/grab is one event per tick.

## How it is checked

- `lpf_test world`: `TestDeterminism` compares per-tick `lpWorld_Hash` across reruns and 1 vs 4 workers.
- `lpf_bench`: final hash per worker count; the run fails (exit 2) if they differ.
- `tools/check-determinism.ps1`: runs the real sandbox with a script at 1, 4 and 8 workers and diffs the per-tick
  hash logs.
- Release and ASan (`RelWithDebInfo`) builds produce the same test hash.

## Known limits

- Cross-compiler and cross-OS determinism is designed for (rules 1 to 5, and Box3D's own guarantee) but only
  Windows/MSVC is exercised so far. Box3D's author calls cross-platform determinism "brittle" across compiler
  versions: pin the toolchain for any multiplayer build.
- `lpWorld_Hash` covers body transforms and velocities, piece geometry and bonds. Rendering and particles are
  deliberately excluded.
