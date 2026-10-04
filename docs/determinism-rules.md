# Determinism rules

Same inputs in the same order must give bit-identical simulation state, whatever the worker count, compiler,
operating system or CPU (x64 and ARM64, and x64 under emulation on ARM64). This is what makes replays, lockstep
multiplayer and golden-hash tests possible. Box3D guarantees it for the physics; these rules keep the destruction layer
(`src/`), the scenes and the app from breaking it. Milestone 7 measured it ([multiplayer-research.md](multiplayer-research.md)).

## Rules

1. **No FMA contraction, no fast-math.** clang and gcc: `-ffp-contract=off` (gcc contracts by default wherever the CPU
   has FMA, every ARM64 among them); MSVC: `/fp:precise` without `/fp:contract`.
   Set once in the top-level `CMakeLists.txt`; never add `/fp:fast`, `-ffast-math` or `/arch:AVX2` to a target
   that touches simulation state.
2. **No transcendental functions from the C library in simulation or scene code.** Use `lpComputeCosSin` / `lpAtan2`
   (`lpmath.h`, from Box3D) and `lpCbrt`: built from `+ - * /`, identical everywhere. Allowed from the C library: `sqrtf`,
   `floorf`, `ceilf`, `fabsf`, `remainderf`, `nextafterf` (exact). C libraries differ in `cbrtf`, `sinf` and `atan2f`
   (Windows' `cbrtf` misses correct rounding on 27% of inputs), and a scene built from them differs before the first
   step. Random unit vectors use rejection sampling, not `sin`/`cos`.
3. **Randomness is PCG32 seeded from simulation state**: world seed, tick, piece index and generation
   (`lpMix64`). Never time, addresses or `rand()`.
4. **Iteration order is always an index order.** Pools are arrays with LIFO free lists; nothing iterates a hash map
   or pointer-keyed container. Physics query results come back sorted from the backend (`src/phys.h`), and
   `lpQueryPieces` keeps a piece by its own bounds, never by the engine's fattened ones.
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
10. **Inputs are commands stamped with a tick.** Everything from outside the simulation is an `lpCommand`
   (`lpWorld_Submit`), stamped `(tick, peer, seq)` and applied as that tick's step begins, in `(peer, seq)` order,
   whatever order it arrived in. A command names slots with their generations and is dropped if they went stale. Logic
   that every machine runs from world state alone (the scene's drivers and bombardment) submits as `LP_PEER_SCENE`,
   applied after the players; anything else that does not come from world state must be a command. A player's control,
   limb or claw command takes a free vehicle or rig over (the scene's drivers leave it) until that player releases it;
   another player's are dropped meanwhile.
   Controls, limb targets and a claw's grip are persistent simulation state. A held pull is one command per tick. The
   sandbox and `lpf_bench --script` replay scripts of commands (`scenes/script.h`); recordings (`--record`) write the
   commands each step applied, so they replay their own session.
   An app reads held input once per tick, just before the step, not once per frame (which would tie what it sends to
   the frame rate). A claim is the world's to grant: a window treats its own as a request until the claim's tick has
   applied, and lets go if another player's came first.
11. **Hash fields, not structs with padding.** A struct copy fills its padding with whatever was on the stack (pointers,
   under ASLR different each run): hashing `lpVehicleControl` whole made the hash differ in one run in twenty at the
   ticks its controls changed, while the simulation itself was identical. Hash the fields (a bool as a byte).
12. **What the physics engine reports is acted on in our own total order.** Hit events (by piece pair, speed, point),
   body move events (by body index), a body's contact list (by piece, then what it touches) and casts (ties by piece
   index: the cast clips one float past its best hit so an equal hit is still seen) never act in the engine's report
   or traversal order, so a different engine, or a restored one, gives the same decisions. The order is made in one
   place, the physics backend (`src/phys_box3d.c`): everything behind `src/phys.h` comes back in it, in piece and
   body indices, so the core never sees the engine's ids or order.
13. **No side effects inside one call's arguments or one braced initializer.** C leaves their order open: two random
   draws as arguments of one call came out in opposite orders on MSVC and gcc x64 against clang and gcc ARM64. One draw
   per statement.
14. **Float to int only through a clamp** (`lpFloatToInt`) when the value is not bounded: an out-of-range conversion is
   undefined, gives `INT_MIN` on x86, saturates on ARM and traps in WebAssembly.
15. **The floating-point control word is round to nearest, without flush-to-zero or denormals-are-zero.** A library or
   driver can change it on our threads, and flush-to-zero changes the stress solver (milestone 7's E9). `lpFpGuard`
   puts it back at step entry, per task in our pool and in Box3D's scheduler (a patch), and at the API calls that
   compute in float between steps; `lpStats.fpRepairs` counts the repairs. `lpDeterminismSelfTest` checks the
   arithmetic (no contraction, ties to even, no flush, correct rounding, the min/max convention, our trig and cube
   root) and returns a hash that machines playing together must share.
16. **The C runtime is linked statically on Windows.** Under x64 emulation on ARM64 (Prism, box64), calls into a
   dynamic C library run as native ARM64 code, whose results differ.
17. **NaN in simulation state is a bug.** It is asserted in the hash path in assert builds.
18. **Game code that feeds the simulation follows these rules too.** The games' AI, scripts and logic either run on
   the host and send their results as commands, or keep to the rules above under a hash check in their own CI.
19. **Whatever writes hashed state marks it.** The hash keeps each body's and piece's element and rehashes only what
   was marked since (`src/hash.c`): code that changes a body's or piece's hashed fields calls `lpHashMark`,
   `lpHashMarkStress` or `lpHashMarkPiece` (`lpTouchPiece` marks the piece and its body), and a new physics setter
   records its body in the backend's touched list. A missed mark does not change the simulation, only the kept hash,
   and `lpWorld_CheckHash` names the element that changed unmarked.
20. **Machines agree on a session before the first tick.** `lpWorld_DescribeSession` lists what the simulation reads
   besides commands: every simulation field of `lpWorldDef` (a new field must join its table in `src/session.c`, or be
   left out there with a reason: a size check fails to compile until it is), a digest per material, joint and
   template, the self-test's hash and the state hash; the apps add their build, protocol, scene, period, time step and
   substeps (`lpSceneDescribeSession`). A peer whose description differs is refused by the key that does
   (`lpSessionCompare`). Text carries floats as `%.9g` in the C locale: nothing calls `setlocale`, and the self-test
   checks that the C library prints and reads them back exactly.
21. **In a session, only packets step the world** (`app/net/lockstep.h`). The host keeps the clock and puts every
   player's commands into their tick's packet; every machine, the host too, applies a tick's commands only from its
   packet, never its own directly, in `(peer, seq)` order whatever order they arrived in, and each machine runs the
   scene's logic from its own world. Every peer reports its hash for every tick, and the host follows the first
   difference down to the element over the wire.

## How it is checked

- `lpf_test world`: `TestDeterminism` compares per-tick `lpWorld_Hash` across reruns and 1 vs 4 workers;
  `TestFpGuard` turns flush-to-zero on between steps and expects identical world and solver hashes and a counted
  repair; `TestDeterminismSelfTest` checks the self-test's known answers and its hash against the reference.
- The `determinism` CI workflow (`.github/workflows/determinism.yml`), on every push to `sandbox` and `master`:
  every bench rung at 1 and 8 workers on Windows (MSVC, clang-cl, ARM64), Linux (gcc, clang, x64 and ARM64) and
  macOS (arm64, and x86_64 under Rosetta 2), plus the x64 binaries under box64 and Prism, diffed per tick against
  Windows MSVC; `lpf_test` on every native leg; a gcc ARM64 leg with contraction on as the positive control.
- `lpf_bench`: final hash per worker count; the run fails (exit 2) if they differ. `--check-hash` compares the kept
  hash with a full recompute every tick and fails (exit 4) naming the element; `TestHashIncremental` does the same on
  seven scenes.
- `tools/check-determinism.ps1`: runs the real sandbox with a script at 1, 4 and 8 workers and diffs the per-tick
  hash logs.
- Release and ASan (`RelWithDebInfo`) builds produce the same test hash.

## Known limits

- Cross-platform determinism holds across Windows, Linux and macOS, x64 and ARM64, MSVC, clang-cl, clang, gcc and
  AppleClang (versions not pinned), and x64 under Rosetta 2, Prism and box64 (with `BOX64_DYNAREC_FASTNAN=0` and
  `BOX64_DYNAREC_FASTROUND=0`; its defaults differed on one transient line), as measured in milestone 7
  ([research/m7-experiments.md](research/m7-experiments.md)) and checked by CI since. Untested: FEX, WebAssembly.
- A desync check must hash the stress solver's state too (`lpWorld_HashStress`): flush-to-zero changed it while the
  world hash stayed equal for 600 ticks.
- `lpWorld_Hash` covers the whole simulation state in eleven categories (`lpHashCategory`): the world's counters and
  queues, bodies, pieces with their bonds, the stress solver's state, the physics engine's hidden state of each body
  (sleep timers, contact warm starts), links, vehicles, wheels, rigs, pools and detonators. Rendering and particles
  are deliberately excluded, and so are pending commands. `lpWorld_HashStress` is the stress category, which a solver
  refactor must also keep; `lpWorld_HashCategories` and `lpWorld_HashElement` follow a mismatch down to the object.
- Checked scenes: walls, house (flasks), tower (collapse), lumber, ruins (its demo and under
  bombardment), town under a barrage (`-Period 3`, many structures solving at once), the yard (its demo and
  under bombardment), the keep (`scripts/keep_demo.txt`, one 2000-piece structure), the track (its demo under
  bombardment: scripted drivers, a driven car, wheels coming off), and the mech yard (its demo under bombardment: the
  patrol, a walked mech losing legs; `scripts/mech_arms.txt`: reaches, a grab and a stomp), at 1, 4 and 8 workers.
  The contraption (a minute of dominoes, each frozen and woken in turn) is checked by `TestContraptionOnTime` at 1, 4
  and 8 workers. `tools/check-determinism.ps1` runs headless through `lpf_bench --script`, whose replays match the
  sandbox's hash logs tick for tick.
