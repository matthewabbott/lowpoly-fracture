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
   integrated sequentially in job order (`impact.c`, "fracture jobs"). A stress job reads its own structure and writes
   only that structure's pieces and its own scratch; budgets are handed out before and joints judged after, both in
   queue order (`stress.c`). Links are synced, polled and damaged sequentially in link order (`link.c`), so Box3D
   joint ids follow a deterministic create and destroy order. Box3D's threading is deterministic by design.
7. **Nothing in the simulation looks at the camera or wall-clock time.** Debris budgets rank by volume, age and
   index; rubble freezing uses Box3D sleep events and tick ages. Timing is measured but never branched on. Every
   per-step cap (fracture jobs, freezes, ghost ray casts, demotions, stress bond-iterations) is a count taken in index
   order, never a time budget. Each structure's solve is sequential within its job, with double accumulators.
8. **Ghosts are simulated by the core, not Box3D, and are part of the state.** Their integration copies Box3D's
   math; their landing ray casts are ordinary deterministic world queries whose callback keeps the closest hit
   (no dependence on callback order). Their state is in `lpWorld_Hash`.
9. **Cosmetic state lives outside the simulation.** Dust particles are emitted by the core but simulated only by
   the app; they never feed back.
10. **Inputs are events stamped with a tick.** The sandbox records tool use as text (`--record`), replays it
   (`--script`) and applies it before the step of that tick. A pull/grab or a held blower is one event per tick.
   Vehicle controls are persistent simulation state (hashed): a `drive` event sets them when they change, and they
   hold until the next one. Scripted drivers (`lpSceneDrive`) set controls from simulation state only, every tick.

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
- `lpWorld_Hash` covers body transforms and velocities, ghost and scrap state, piece geometry, bonds and links (and
  gravity scales that are not 1, and vehicles: a world without them hashes as before). Rendering and particles are deliberately
  excluded. `lpWorld_HashStress` covers the stress solver's state, which a solver refactor must also keep.
- Checked scenes: walls, house (flasks), tower (collapse), lumber, the blower demo, ruins (its demo and under
  bombardment), town under a barrage (`-Bombard 3`, many structures solving at once), the yard (its demo and
  under bombardment), the keep (`scripts/keep_demo.txt`, one 2000-piece structure), and the track (its demo under
  bombardment: scripted drivers, a driven car, wheels coming off), at 1, 4 and 8 workers.
