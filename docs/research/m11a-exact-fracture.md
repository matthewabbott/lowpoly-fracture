# Milestone 11a: exact fracture geometry

The running record of milestone 11a (roadmap §11a): fracture's float32 geometry replaced by exact integer geometry
(integer sites and bisector planes, exact classification, hulls from exact topology), with no plane-shift hack and no
tolerance flip left in fracture. Its exit: recorded blasts fracture exactly. The steps are C0 to C9; each lands as one
commit and adds its section here.

## C0: the recorder, the validators, the stats and the before-numbers (2026-10-07)

C0 measures the float engine as it stands. It changes no simulation: every `lpf_bench` hash is the same.

### What was built

- **Stats** (`lpFractureStats`, fracture.h; counted, never hashed, never branched on). The float clip is counted by
  caller (Voronoi, grain, radial, masonry, snap, chips) through `lpPoly_ClipCounted` (poly.h; `lpPoly_Clip` is
  unchanged): clips, plane shifts and the largest, tolerance outs (an answer only the tolerance decided: a vertex just
  outside kept whole, or just inside dropped) and failures. The pattern: sites drawn, slivers absorbed, cells dropped,
  cells out. The merge: pairs tried, too big, pre-rejected, quickhulls, accepted, faces retagged by the 0.9999 rule and
  bridge faces. Bonds by tag and by the `lpShape_Contact` fallback. Physics hulls built and failed (a cell with more
  edges than Box3D's 128, or degenerate: today either turns the cell to dust). Ghosts chipped and chips made. Stage
  times for the bonds and chips beside the pattern, merge and hulls. The output cells' face and vertex counts (the
  largest, and histograms). `lpStats` gains `planeShifts`, `cellBondCpuMs` and `chipCpuMs`, which the bench prints.
- **The job hook** (`lpWorld_SetFractureHook`, world.h): sees each job in phase 3, in job order, after it ran and
  before its cells become pieces. The job is const, and its output digest is checked unchanged after the hook in every
  build (a hard failure). The job's input is still intact there: `lpFracture_RunJob` reads the poly and the input and
  writes only cells (moved into the body frame), classes, hulls and bonds (a test writes the snapshot before and after a
  run and compares the bytes).
- **Snapshots and digests** (`src/fcheck.c`): `lpFractureJob_Write` and `_Read` carry everything the job reads (poly,
  input, tier thresholds, merge slack, chip splits, centre, local impact, and the impact for the integration) as
  little-endian bytes with exact float bits, a version and a checksum; a read refuses short, long, corrupt or out of
  range bytes. `lpFractureJob_Digest` hashes a finished job's output bit for bit in job order: cells (vertices, faces
  with planes, tags and materials, loops, mass), sites, classes, the physics hulls' points and planes, bonds.
- **Validators** (`src/fcheck.c`, pure, in double from the float cells; each check callable alone, for tests and C5):
  - a. tiling: the pattern cells' volumes (re-run from the snapshot) sum to the parent's (to 1e-9, C5's tolerance);
  - b. overlap: each pair of pattern cells whose bounds meet, one clipped by the other's face planes without tolerance;
  - c. siblings: every cut face of a pattern cell against the faces of the other cells on its opposite plane: *exact*
    (its twin: the exactly negated plane and the same vertices), *covered* (faces on the exactly negated plane that
    reach past it, or several of them, meet it at T-junctions: a masonry plate under its bricks, radial rings across a
    wedge's side), *near* (covered within 1 mm and 0.1% of its area, but not exactly) or *unmatched*;
  - d. validity of the output cells: a closed oriented 2-manifold, Euler 2, no degenerate face or repeated vertex, and
    convexity (the largest excess of a vertex over a face plane) and planarity;
  - e. merge containment: every keeper before the merge (re-run and classified as the job did) inside one cell after it;
  - f. chips (made again by the job's own code on the job run without chips) tile their ghost cell without overlap, and
    with the cells left whole are the job's own cells, digest for digest;
  - g. census: the largest coordinate in the object frame, vertices past 128 m, the shortest edge and smallest face,
    vertex pairs of one cell closer than 2^-16 m, distinct vertices of one cell on one 2^-16 m grid point. And the
    census of a scene's parts as built (`lpWorld_CensusParts`), for C3.
  Any nonzero error is a violation. Tests inject each fault (a gap, an overlapping pair, a moved vertex, a mismatched
  sibling face, a merged cell missing part of a source cell, an open cell, chips with a gap or an overlap) into an exact
  tiling of a cube into octants, which itself reports none.
- **lpf_bench** (`bench/fractures.c`): `--check-fractures` validates every job of each run and prints what the jobs
  did and what the checks found (with the worst job of each check and a breakdown by pattern), into `--json` too; it
  exits 5 on a violation and 6 when worker counts disagree. `--record-fractures path` writes every job's snapshot,
  digest and tick from the first worker count's run. `--replay-fractures path [--job k] [--repeat n]` runs recorded
  jobs alone, says same or differs per job, times each stage best of n, and lists the slowest jobs.
- **`tools/check-fractures.ps1`**: the four rungs (keep, barrage, siege, town) at 1 and 8 workers, two tables, JSON in
  `build/fractures/`; `-Record` keeps the recordings; `-ReportOnly` (the default until C5) fails only on a crash or a
  disagreement between worker counts.

### The gates

Written before the results; all pass. Timings are provisional (another agent may have been compiling on the laptop).

| gate | result |
|---|---|
| G1 | `tools/build.ps1 -Test`: 10 of 10 suites; `lpf_test --contract` passes; the catalogue check: 174 tests, 113 in the contract, no problems. |
| G2 | `tools/bench.ps1 -Repeat 3`: every rung's hash `same` against `bench/baseline.json` (12 rungs at 1 and 8 workers). |
| G3 | keep, barrage, siege and town at 1 and 8 workers: the world and solver hashes with `--check-fractures`, with `--record-fractures`, with neither, and from the unmodified build are all equal (keep `8253462fa0036e93`, barrage `dc3c85713d8fbf85`, siege `b00ae8d8bf643bd3`, town `d4e4fb0a443a1adf`). `check-fractures.ps1 -Record` (both at once) gives them too. |
| G4 | The counts (every field but the timings) are identical at 1 and 8 workers on the four rungs: the bench compares them (exit 6 otherwise) and the script compares them again. |
| G5 | All 6,946 jobs recorded on the four rungs (from msvc-release, 1 worker) replay `same` on msvc-release, on clang-cl (Windows, warnings not errors as in CI: the clang-release preset's `-Werror` stops at two warnings already on the branch, in `stress.c` and the tests), under the MSVC ASan build, and on the DGX Spark with gcc 13.3 and clang 18 (aarch64). On the Spark, `--check-fractures` gives the same counts and magnitudes as Windows on all four rungs at both worker counts, and `lpf_test fracture` passes. |
| G6 | `TestCheckCatchesFaults`: the octant tiling reports nothing; a gap, an overlapping pair, a moved vertex, a sibling face pulled 1/1024 m away, a merged cell missing half a source cell, an open cell, and chips with a gap or an overlap are each caught with their exact magnitude. |
| G7 | `tools/check-determinism.ps1`: 240 ticks identical at 1, 4 and 8 workers, and the per-tick log identical to the unmodified build's. |
| G8 | `tools/build.ps1 -Preset msvc-asan -Test`: 10 of 10 (598 s), and the fracture suite again after the last edits. Twelve truncated or corrupted recordings (cut mid-record, in a size, in a digest, in the header; empty; a flipped bit in a snapshot; a size past the end or negative; a header length past the end; garbage after the header; an index past the vertices under a valid checksum) all fail cleanly under the ASan bench: exit 1 with the record named, or exit 2 for a flipped digest, and no ASan report. Walls with `--check-fractures --record-fractures` and full replays with checks of keep, town and barrage run clean under ASan. |
| G9 | The ladder, best of 3, unmodified build against this one (ms per step at 1 / 8 workers): walls 0.75 / 0.48 against 0.75 / 0.49, town 4.07 / 2.44 against 4.03 / 2.29, pile 2.41 / 0.92 against 2.26 / 0.92, tower 3.77 / 1.60 against 3.71 / 1.56, keep 7.23 / 5.14 against 7.30 / 5.17, barrage 10.54 / 5.26 against 10.95 / 5.27, siege 19.25 / 10.80 against 19.31 / 10.55; the small rungs within 0.02 ms. A second pair on four rungs: barrage 11.28 / 5.43 against 11.92 / 5.80, town 4.57 / 2.41 against 4.07 / 2.29. Within the noise (both builds ran 10-40% slower than the baseline's day); for the lead to measure again on a quiet machine. |

### The before-numbers

Counts are the same at 1 and 8 workers on every rung (600 ticks; keep and town a blast every 12 ticks, barrage town
every 3, siege keep every 4). A job is a piece the impact reached; "checked" are those that split (two cells or more).

| rung | jobs (checked) | impact / grain / radial / masonry / snap | pattern cells | cells out | float clips | plane shifts (share, max) | tolerance outs | clip failures |
|---|---|---|---|---|---|---|---|---|
| keep | 626 (298) | 530 / 96 / 0 / 0 / 0 | 3,331 | 2,699 | 37,343 | 4 (0.011%, 0.040 mm) | 4 | 0 |
| barrage | 2,853 (1,837) | 1,340 / 635 / 277 / 470 / 131 | 16,442 | 17,798 | 123,132 | 23 (0.019%, 0.040 mm) | 54 | 0 |
| siege | 2,427 (973) | 2,228 / 199 / 0 / 0 / 0 | 9,602 | 7,742 | 98,505 | 31 (0.031%, 0.040 mm) | 6 | 0 |
| town | 1,040 (689) | 516 / 176 / 140 / 111 / 97 | 6,543 | 7,880 | 51,973 | 22 (0.042%, 0.040 mm) | 21 | 0 |

| rung | merges tried / too big / pre-rejected / quickhulls / accepted | faces retagged / bridges | bonds by tag / by contact | physics hulls (failed) | chips: ghosts into |
|---|---|---|---|---|---|
| keep | 10,399 / 0 / 3,016 / 7,383 / 641 | 5,181 / 3,623 | 8,411 / 0 | 2,681 (0) | 9 into 18 |
| barrage | 14,834 / 0 / 2,448 / 12,386 / 2,960 | 26,394 / 12,550 | 16,575 / 10,562 | 11,634 (0) | 1,527 into 5,855 |
| siege | 28,492 / 0 / 7,339 / 21,153 / 1,860 | 16,444 / 10,004 | 22,451 / 0 | 7,739 (0) | 0 |
| town | 5,294 / 0 / 1,142 / 4,152 / 1,037 | 8,861 / 4,536 | 6,854 / 4,633 | 4,508 (0) | 832 into 3,211 |

The checks (violations are counts of jobs, pairs, faces, cells, keepers or chip sets):

| rung | violations | tiling: jobs off (max) | overlap: pairs (max, total m^3) | sibling faces: exact / covered / near / unmatched | near: max distance | not convex (max excess) | containment (max excess) | chip sets off |
|---|---|---|---|---|---|---|---|---|
| keep | 29,665 | 284 (3.9e-6) | 6,508 of 12,161 (7.0e-7, 4.1e-6) | 64 / 1 / 18,175 / 4 | 0.099 mm | 2,503 (2.4 µm) | 2,182 of 2,220 (2.4 µm) | 9 of 9 |
| barrage | 84,700 | 1,468 (6.7e-5) | 17,485 of 36,809 (2.1e-5, 7.8e-5) | 7,296 / 7,237 / 43,386 / 14 | 0.48 mm | 14,443 (9.3 µm) | 6,380 of 7,834 (14 µm) | 1,524 of 1,527 |
| siege | 80,807 | 947 (1.3e-4) | 17,699 of 32,466 (1.9e-5, 8.2e-5) | 362 / 8 / 48,746 / 25 | 0.15 mm | 7,156 (3.6 µm) | 6,234 of 6,338 (3.6 µm) | none |
| town | 35,651 | 538 (7.7e-5) | 7,230 of 15,843 (1.1e-5, 3.4e-5) | 3,136 / 3,151 / 17,900 / 16 | 0.23 mm | 6,606 (12 µm) | 2,529 of 3,097 (20 µm) | 832 of 832 |

By pattern (barrage), the largest tiling error, overlap and near-sibling distance: impact 3.6e-5, 9.3e-6 m^3, 0.35 mm;
grain 6.7e-5, 2.1e-5 m^3, 0.48 mm; radial 1.1e-7, 1.8e-9 m^3, 0.36 µm; masonry 6.1e-7, 8.5e-9 m^3, 3.6 µm; snap
1.0e-7, 2.3e-8 m^3, and its two halves are exact twins. The patterns that shift planes (Voronoi and grain) are the
ones whose errors reach tenths of a millimetre.

No cell is topologically invalid and no face degenerate on any rung; no validator ran out of room.

The output cells (histograms by faces: up to 6, 8, 12, 16, 24, 32, 48, more; by vertices: up to 8, 12, 16, 24, 32, 48,
64, more):

| rung | faces: max, histogram | vertices: max, histogram |
|---|---|---|
| keep | 40: 230, 692, 1334, 259, 152, 28, 4, 0 | 38: 233, 719, 924, 768, 49, 6, 0, 0 |
| barrage | 55: 10019, 3737, 3020, 616, 283, 62, 55, 6 | 46: 10059, 3973, 2438, 1177, 118, 33, 0, 0 |
| siege | 51: 528, 1770, 3975, 826, 519, 100, 23, 1 | 46: 529, 1869, 2689, 2384, 238, 33, 0, 0 |
| town | 45: 4775, 1629, 1083, 239, 112, 31, 11, 0 | 41: 4791, 1700, 867, 462, 51, 9, 0, 0 |

The census (object frames):

| | parts at build (without the ground) | the ground | cells out: max coordinate | shortest edge | smallest face | vertex pairs under 2^-16 m | on one grid point |
|---|---|---|---|---|---|---|---|
| keep | 1,972 parts, within 13.9 m, edges from 6 cm, 6 faces and 8 vertices at most | 60 m | 13.9 m | 31 µm | 2.2e-8 m^2 | 0 | 0 |
| barrage | 1,149 parts, within 10.5 m, edges from 4 cm, up to 24 faces and 14 vertices | 120 m | 10.5 m | 16 µm | 3.0e-10 m^2 | 0 | 0 |
| siege | as keep | 60 m | 13.9 m | 12 µm | 5.2e-10 m^2 | 1 | 0 |
| town | as barrage | 120 m | 10.5 m | 10 µm | 6.7e-10 m^2 | 2 | 2 |

The jobs' stage times (CPU ms summed over a run's jobs, 1 worker, from the runs above; provisional, see G9), and the
replays' (each job alone, best of 3, summed; C2's go/no-go baseline):

| | jobs | pattern | merge | hulls | bonds | chips | total |
|---|---|---|---|---|---|---|---|
| keep, in the run | 626 | 22.6 | 116.3 | 21.7 | 1.5 | 0.0 | 162.1 |
| keep, replay MSVC / clang-cl | 626 | 20.7 / 16.0 | 115.1 / 101.8 | 21.3 / 18.8 | 1.3 / 1.0 | 0.0 / 0.0 | 158.4 / 137.7 |
| barrage, in the run | 2,853 | 74.7 | 176.2 | 67.0 | 8.4 | 7.8 | 334.2 |
| barrage, replay MSVC / clang-cl | 2,853 | 64.1 / 49.1 | 166.1 / 144.5 | 59.7 / 52.0 | 6.9 / 5.9 | 6.3 / 5.0 | 303.1 / 256.4 |
| siege, in the run | 2,427 | 74.8 | 354.5 | 66.6 | 4.1 | 0.1 | 500.1 |
| siege, replay MSVC / clang-cl | 2,427 | 68.6 / 53.9 | 348.3 / 306.5 | 62.4 / 54.2 | 3.8 / 2.8 | 0.0 / 0.0 | 483.1 / 417.5 |
| town, in the run | 1,040 | 28.6 | 58.2 | 25.1 | 3.6 | 4.2 | 119.7 |
| town, replay MSVC / clang-cl | 1,040 | 26.8 / 19.5 | 62.6 / 55.4 | 27.8 / 25.1 | 3.3 / 2.6 | 3.8 / 2.7 | 124.4 / 105.2 |

The merge is 50 to 73% of a job (its quickhull per tried pair), the pattern 13 to 22%, the physics hulls 13 to 22%;
bonds and chips are small. The slowest single jobs take 2 to 3.6 ms, nearly all of it merging 14 to 19 cells. The
recordings are in `build/fractures/` (`tools/check-fractures.ps1 -Record` writes them again, byte for byte).

### What was surprising

- **Plane shifts are rare.** 0.01 to 0.04% of clips push their plane, never past 0.04 mm (two steps of the shift);
  walls has none at all. Tolerance outs are as rare. The plane-shift hack is not why siblings disagree.
- **Twins are almost never exact anyway.** A Voronoi face and its twin lie on exactly negated planes (the bisector is
  computed the same way from either side), but each cell reaches its vertices through its own sequence of clips, so
  the rounding differs: 64 of 18,244 cut faces on keep are exact twins. Masonry's head joints are (both sides clip the
  same slab by the same plane), and T-junctions on exactly negated planes cover their faces as they should.
- **Shallow corners amplify.** A 0.04 mm shift in a grain cell moved a vertex at a shallow corner 0.48 mm (barrage job
  1681): the near-sibling distance is twelve times the shift. Impact cells reach 0.35 mm, likely the same way. The
  unmatched faces (4 to 25 a rung, 3.7 cm^2 at most in total) are, where looked at (job 1681), such twins slid sideways
  by more than 0.1% of their area.
- **Masonry can lose part of a wall.** On walls (not a rung), a masonry job hit the 128-cell limit and dropped cells
  without a word: job 7 lost 23% of the parent's volume (tiling error 0.229, 18 cut faces unmatched, 2.5 m^2). It is
  the only gap found (no rung has a job off by more than 1e-3).
- **Vertices closer than the grid.** Cells out have edges down to 10 µm (under 2^-16 m = 15 µm), faces down to
  3e-10 m^2, and two vertex pairs in town (one in siege) closer than 2^-16 m, which C5's grid will merge. Objects stay
  within 14 m; the grounds reach 60 and 120 m, inside ±128 m but close to it.
- **A third to half of the jobs split nothing.** On barrage 1,016 of 2,853 jobs give fewer than two cells (town 351 of
  1,040, keep 328 of 626, siege 1,454 of 2,427): their piece stays whole after paying for the attempt.
- **Faces:** the largest cells have 55 faces and 46 vertices; under 0.1% of cells have more than 48 faces. No physics
  hull failed on any rung.

## C1: 128-bit integers, integer planes and their predicates (2026-10-07)

Built beside the engine (nothing calls it yet; no simulation hash changes; the determinism self-test's hash does).

- `src/int128.h`: `lpI128`, two 64-bit words everywhere, so values hash and serialise alike. Paths: `__int128` on gcc
  and clang; `_umul128` / `_mul128` on MSVC x64, which clang-cl takes too (its `__int128` products could need
  compiler-rt); `__umulh` / `__mulh` on MSVC ARM64; the portable 32-bit-limb reference, always compiled, which every
  path is tested against and the whole build can run on (`-DLPF_INT128_PORTABLE=ON`).
- `src/geom.h/.c`: canonical `lpIPlane`s (n.x <= d, gcd 1, so equal planes are equal field by field and a negation
  stays canonical) from four range-checked constructors (bisector, grain's metric bisector, snapped normal through a
  grid point, plane about an integer axis), each returning a reason when it makes no plane; vertices of three planes
  (`lpIVertex`, homogeneous int128 by Cramer's rule, w > 0); classification in int128, and a grid point against a
  plane in int64.
- **The bit budget**, proved in geom.h's header with |n_i| <= A = 2^24 - 1 and grid points within P = 2^23 u: W <= 4A^3
  (< 2^74), numerators <= 12A^3P (< 2^98.6), the classification <= 48A^4P (< 2^124.6): two bits spare in a signed
  int128. The determinant bound (the largest determinant of a +-1 matrix: 4 for order 3, 16 for order 4) is what keeps
  them there; the triangle inequality alone gives 2^99.2. Every bound is reached by planes in a Hadamard sign pattern,
  and `TestPredicateBudget` checks them against a 256-bit reference.
- For C2 and C3: compare vertices by incidence, never by cross-multiplying (173 bits); a vertex is the same from its
  three planes in any order; the bisector seen from either site is the exact negation; an int64 orientation is exact
  only for points within about 2^20.3 u (20 m) of each other on every axis; a plane through three grid points more than
  about 4.4 cm apart can exceed A, so bridge and authored faces go through the snapped constructor.

Gates: the tests pass on msvc-release, clang-cl, a whole portable build, and the Spark's gcc 13.3 and clang 18 (aarch64,
the `__int128` path), with the same self-test hash everywhere; `clang-cl --target=aarch64-pc-windows-msvc` built the
MSVC ARM64 path (MSVC's own ARM64 tools are not installed; the CI leg builds it); the bench's hashes are the same.
Classification costs 9 ns on MSVC x64 (70 ns portable), 6 ns on clang-cl, 3-4 ns on the Spark; a vertex from three
planes 15-25 ns (laptop busy: provisional).

**Grain.** Wood's cells are longer along the grain: a metric Voronoi, K = sI - t g g^T up to scale, with the axis g a
small integer vector, c = 1 - 1/stretch^2 quantised as t/Q, and the job's sites on a lattice L so that bisector normals
2K(b - a)/L stay below A. Axis precision, stretch precision and the lattice share one budget (K grows with Q |g|^2, and
the lattice with K). As built in C1, |g_i| <= 8 and Q = 256: the axis up to 5.0 degrees off in 3D (the plan said 3.6,
true only within a coordinate plane), wood's 3.5 stored as 3.49, the worst axis's lattice 3.1 cm on a 4 m piece.
**The owner's decision (2026-10-07): |g_i| <= 16 with Q = 64** (to land at C5): the axis within 2.5 degrees, the same
lattice, wood's stretch stored as 3.58 (cells 2% longer; a material can be tuned to a stored value). The axis search
(about 18,000 candidates by brute force) runs once per piece when its object is made, is stored on the piece and
hashed, and fragments inherit it exactly; an assert build recomputes it, and a search over the rounding neighbours of
each largest component (about 64 candidates) is tested against the brute force. A load-dependent switch to the coarser
axis would save nothing per fracture once the axis is cached: noted with the real load-time knobs in roadmap §17.
