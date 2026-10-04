# The toy: running notes

The log of the toy's build (design in `DESIGN.md`): per step, what landed, what its gate measured (hashes, counts),
judgement calls, open issues, and what is next. Newest entry last. Agents append here before they stop.

## 2026-10-04: started

The GPU lab (E11 ported into `lab/gpu`) reproduces E11's hashes on Windows (MSVC, clang-cl) and on the Spark (gcc,
clang; GB10 and llvmpipe match their twin bit for bit in F, I32, I64 and V4). Findings that shape the toy:
- the GB10's driver (580.82) crashes compiling the F kernels when `RoundingModeRTE` is declared: F-plain only;
- NaN bits differ by machine (the lab's Fnocap): a NaN anywhere is a failure, not a value;
- unsequenced random draws made E11's corpus compiler-dependent: draw in separate statements.

## 2026-10-04: steps 1 and 2 (helpers and battery; layouts, scenes, quantisation, CPU stages, twin driver)

**What landed** (lines): `layout.h` 298 (dialect types and V4 formats, angular velocity Q11.20; hulls, bodies split
into state / pose / mass, `Aabb`, `Pair`, `Manifold` and `ManifoldPoint` for step 3, `Params`, `Push`, `Hash2`,
battery vectors), `num.slang` 748, `battery.slang` 166, `battery.c` 1,090, `battery_glue.cpp` 44, `scene.c/.h`
611/62, `quant.c/.h` 323/46, `stages.c/.h` 452/67, `kernels.slang` 176, `kernels_glue.cpp` 109, `toy.c` 757,
`rng.h` 102 (PCG32 and draws), `fpflags.h` 74 (the sentinel), `gen_toy.py` 70, `CMakeLists.txt` 50, `gate.py` 96 (both
gates in one command); about 5,300 in all. Hooks outside `toy/`: `lab.py gen` ends by calling `gen_toy.gen_toy()` when
`toy/gen_toy.py` exists (a full `lab.py gen` ran: E11's outputs byte-identical, the toy's added), and the lab's
`CMakeLists.txt` adds `toy/` when `gen/toy` exists. `gen_toy.py` also runs alone. Binaries: `battery_F/_V4/_D`,
`toy_F/_V4/_D` in `build/<compiler>/bin`, run from `lab/gpu`.

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1 and 2, logs in build/toy-scratch/gate (about 10 s)
build/msvc/bin/toy_V4.exe --scene pile200 --ticks 600 --threads 1,8 --pos-ref build/toy-scratch/gate/pile200.D.pos
```

**Gate 1: passed.** Each dialect's battery (16 helpers, 4,096 vectors each; lpDivQ31 79,976 including a sweep of
divisors near 1 and the limits against the same numerators at 10 shift classes, and a class of quotients just under
2^31 - 1 for every divisor size; divClamp 16,384) on the RTX 3060 (driver 0x988f0000) and the UHD 630 (0x194859),
F-plain: every output word identical to the twin in every helper, saturation counts too (V4: 725 in mul, 2,100 in
sum2/dif2, ..., 17,938 in lpDivQ31). Battery hashes, equal on MSVC 19.42.34435 and clang-cl 23.1.2: **F
2b11786b345dc01f, V4 50aaf74f7d01b6fe, D 5db9d655b22c9968** (`--n 4096 --seed 20261004`, the defaults).
`lpDivQ31` equals C's `/` on every input: 0 of 79,976, and 0 of 1,063,016 at `--n 65536 --seed 7` (twin and both
GPUs, battery hash 6894d90a8ee222b1 on both compilers). The twin's floating-point status is clean after every helper
except F's snap, whose subnormal inputs set the denormal flag by design. Accuracy against C's doubles (a report): F
recip 8.8e-8 relative, rsqrt 8.9e-8, |quatNormalize| - 1 1.3e-7, divClamp 1.3e-7; V4 rsqrt 2.7e-8, |quatNormalize| - 1
2e-9, recip and divClamp below 1 lsb (truncated); atan2 2.8e-5 rad in every dialect (lpAtan2's polynomial).

**Gate 2: passed.** 600 ticks, 1 and 8 threads, MSVC and clang-cl: per-tick hashes (bodies and stages) identical
across thread counts and compilers in every dialect, floating-point sentinel clean after every dispatch, saturations
0. Run hashes (an FNV of every tick's two hashes):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | 9fd7beb412959ce4 | a31b90a2beeb8c99 | d815f7db0352baea |
| pile200 | 83d78923a458f806 | ab414d9013c1c8df | c97b8c7ef3ea182d |
| pile200, gravity 0, body 112 pushed (sleep, wake) | 17884cfbe1e2b27a | 294119e0913ca824 | 6c7dbaf910c78405 |
| chip (10 m/s, 40 rad/s) | 8025ae0dc20ebfe9 | fb8d93f952c1ce6b | 8b9e728c30c9d31f |
| ramp | 10c613893e3d9b2a | e659316259462f98 | 59fb18a494cc885a |

Sensible counts: stack10 starts with 10 pairs (ground and the 9 neighbours), 2 colours, one island of 10; the stack
falls through the ground (pairs with it come and go) and ends with the 9 chain pairs. pile200 (205 bodies: floor, 4
walls, 160 boxes, 40 chunks of up to 14 vertices / 9 faces / 42 half-edges, 16 chunks redrawn) starts with 1 pair and
199 islands; as the layers pass the floor (the walls stay clear), pairs rise to 76 (tick 56) and colours to 2, then
fall back to 1. Without gravity,
197 bodies sleep at tick 31, the pushed body wakes bodies on its way (reset to rest, asleep again 30 ticks later), and
by tick 90 only it is awake. Free fall against D, dynamic bodies (stack10 and pile200 alike):

| | ticks 1-120: rms / max | ticks 1-600 (a 500 m drop): rms / max | rotation (chip), 600 ticks: rms / max |
|---|---|---|---|
| F | 9.9e-6 / 2.5e-5 m | 6.2e-4 / 1.4e-3 m | 7.8e-6 / 1.4e-5 rad |
| V4 | 1.7e-5 / 3.8e-5 m | 4.3e-4 / 9.5e-4 m | 1.2e-7 / 2.1e-7 rad |

F's error is its float positions (Box3D's); V4's is the quantised h g (7.9e-8 m/s too much per substep, so 9.5e-4 m
after 2,400 substeps, systematic). D's body lands where y0 - g h^2 n (n + 1) / 2 says, to the printed digit.

**Findings.**
- MSVC folds `x < 0 ? -x : x` into `fabs` in C (so -0 becomes +0) and clang does not: the battery's own input generator
  made different vectors on the two compilers (404 of them) until it used `fabs`. F's snap already sends -0 to +0; D's
  snap now does too (`x == 0 ? 0 : x`), so the D twins agree whatever a compiler folds. C code in the toy uses `fabs`.
- F's Newton steps written y (3/2 - x y^2 / 2) give rsqrt(1) = 1 - 2^-24, so renormalising a unit quaternion shrank it,
  and resting orientations drifted 1.4e-5 rad in 10 s against D. Written as a correction, y + y (1/2 - x y^2 / 2), they
  are within 1.7 ulp (from 2.4) and exact at powers of four; the reciprocal likewise (1.5 ulp from 2.5, exact at powers
  of two). The drift is gone (3.7e-8 rad, constant).
- lpDivQ31's estimate lands 0 to 13 below the quotient (a Python emulation over 157k inputs, half of them quotients
  near the limit); the correction has room for 31.

**Judgement calls and deviations from DESIGN.md.**
- V4 positions are two words (low, signed high) in `Pos3`, not an int64 field: no int64 anywhere in a kernel outside
  `mas` (the sum of up to three products, the rounding shift, the counted narrow), the same bytes as an int64. The
  element hash is two 32-bit murmur3 lanes for the same reason.
- lpDivQ31( n, d, k ) = n 2^k / d truncated toward zero (C's `/`), n and d in +-(2^31 - 1), k in [0, 32]; beyond +-(2^31
  - 1), or d = 0 with n != 0, it saturates and counts. The divisor is clz-normalised; y ~ 1/x in Q1.30 for x = (dn/2 +
  1)/2^30 (so y < 2 fits) from 48/17 - 32/17 x and three Newton steps; the estimate is biased 8 low; the remainder is
  exact in two words (mas's rounded high word less the low word's top bit); five binary steps finish. divClamp( n, d,
  k, lim ) gives +-lim without counting when |n| 2^k >= lim |d| (exact, two words). F's and D's divClamp ignore k.
- V4 rsqrt( u, su, sy ): clz to x in [0.5, 2) by an even shift, the line 1.5604 - 0.4714 x, four Newton steps; su must
  be even (every format is). F rsqrt: the 0x5f3759df seed and three steps; F recip: exponent bits, 48/17 - 32/17 m,
  three steps.
- The AABB is computed in prepareBodies (the brief), not also in finalize (DESIGN's step 7): prepare runs at the start of
  a tick over last tick's awake bodies (every body on the first tick). Cells are inclusive, [floor(p) + floor(lo),
  floor(p) + floor(hi) + 1] at 2^-10 m (conservative by up to a cell). The margin is `Params.aabbMargin`, 0.05 m (Box3D's
  largest), around a fresh box each tick (no fat AABBs kept).
- Sleep: a body is under the thresholds when |v|^2 + |maxExtent w|^2 <= 0.05^2 (Box3D: |v| + |arc| <= 0.05, plus a
  position term); each component is gated first, so nothing saturates. An island sleeps when its smallest counter is
  30. Waking resets the body (a sixth kernel, `wakeBodies`: Box3D's identity state on wake); a woken body's counter
  counts as 0 that tick (the CPU's copy is stale).
- Step 2 has no manifolds, so every broadphase pair counts as touching for waking, islands and colouring
  (`stages_tick`'s `prevTouching` is NULL); step 3 passes last tick's pointCount > 0 by prevIndex. Pairs with a
  sleeping body stay in the key list, inactive, so their manifolds can carry over; static-static pairs are skipped.
  Colours: 64, then an overflow colour.
- The convex clipper is a half-space intersection (vertices from plane triples, faces sorted by a pseudo-angle, twins
  adjacent as Box3D's); chunks with an edge under 2 cm or over the limits are drawn again. Mass properties from
  tetrahedra (a box's checks against V (b^2 + c^2) / 3). Face planes are recomputed from the quantised points (Newell),
  offsets rounded up.
- Scenes: stack10's cubes are 0.5 m on a side (density 1000); pile200 is 8 layers of 5 x 5 at 0.7 m from 3 m up,
  half extents 0.1 to 0.2 m, random orientations, every fifth a chunk; bounce, ramp, ratio, chip are simple versions
  (bodies placed, materials stored, no contacts yet). Gravity (0, -10, 0).
- F's Params follow Box3D's float arithmetic (h = dt / 4, inv_h = 4 inv_dt); the speed caps are E11's (no sqrt).

**Open issues.**
- F's first-120-tick free-fall error (9.9e-6 m rms) sits at DESIGN's 1e-5 bar, from float positions alone; if
  contacts add to it, F may want V4's two-word positions.
- V4's quantised gravity delta drifts 1 mm in 10 s of free fall; a finer format for h g (or v) would remove it.
- Only the battery has run on the GPUs; the kernels' SPIR-V is generated and passes spirv-val (Vulkan 1.3) and the
  NoContraction audit, but the Vulkan tick path is step 5.
- No UBSan build yet (DESIGN's gate); signed shifts of negative values are written through uint, right shifts of
  negative values are arithmetic (implementation-defined in C++17, every compiler here).
- Debug aids: `TOY_DUMP_AABBS=T` (toy, 1 thread) and `TOY_DUMP_INPUTS=FILE` (battery).

**Step 3 starts from:** `kernels.slang` (add `narrowSat` and `narrowClip` there; bindings 12 on for the pair list and
the two manifold buffers), `layout.h`'s `Manifold` / `ManifoldPoint` (defined, unused) and the hull buffers (points
Q8.24, faces with Q1.30 normals and Q8.24 offsets, packed half-edges), already uploaded at bindings 1 to 3 but read by
no kernel yet. The pairs come from `Stages` (`pairs`, `prevIndex`, `active`; the solver's lists in `colourStart` /
`colourList`); `toy.c`'s tick has one CPU point between the prepare dispatch and the rest, where narrowphase dispatches
over the active pairs go, and `stages_tick` should get the manifolds' pointCount > 0 (by prevIndex) as `prevTouching`.
Add the narrowphase helpers to `battery.slang` as they appear, and the 10k-pose corpus as a new runner beside
`battery.c`.

## 2026-10-04: step 3 (the narrowphase: narrowSat, narrowClip; the 10k-pose corpus)

**What landed** (lines, now): `narrow.slang` 1,303 (new: the narrowphase, included by `kernels.slang`: `narrowSat`,
`narrowClip`, `copyManifolds`, the SAT queries, incident face, clipping, reduction, edge contact, the diagnostics),
`narrow.c` 1,602 (new: the corpus runner, `narrow_F/_V4/_D`), `kernels.slang` 224 (bindings 12 to 17: pairs, manifolds,
prevManifolds, satAxes, narrowDiags, manifoldHashes; `hashManifolds`), `num.slang` 877 (the division fix; `posDelta`,
frames, `unit3`, `len3`), `layout.h` 371 (`SatAxis`, `NarrowDiag`, `AXIS_*`, `SAT_*`, `CLIP_MAX`, the narrowphase's
Params), `toy.c` 863 (the wiring), `battery.slang` 249 and `battery.c` 1,208 (three helpers), `kernels_glue.cpp` 131,
`scene.c/.h` 622/70 (`scene_box_hull`, `scene_chunk_hull` public, the draws unchanged), `quant.c` 332, `gen_toy.py` 71,
`CMakeLists.txt` 57, `gate.py` 143 (gate 3; `--gates`). About 3,700 lines added (2,900 of them the two new files).

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1, 2 and 3 (about 15 s); --gates 3 for the corpus alone
build/msvc/bin/narrow_D.exe --out build/toy-scratch/n.bin && build/msvc/bin/narrow_V4.exe --ref build/toy-scratch/n.bin --list 8
```

**Gate 3: passed.** The corpus: 10,000 pairs from the scene generator's hulls (4 floors and walls, 60 boxes with four
of stack10's cube, 60 chunks), box-box 4,066, box-chunk 2,963, chunk-chunk 2,971; faces aligned as in a stack 1,045,
faces twisted 1,949, tilted 1e-3 to 0.1 rad 1,574, within 1e-4 rad 963, edges crossing 1,997, a vertex on a face 1,484
(half exactly along the vertex's mean normal, half tipped off it), generic 988; separations deep (-0.15 to -0.04 m)
1,339, shallow 2,734, inside the speculative distance 2,255, at it (0.02 m +-2e-5) 423, just outside (to 0.03 m)
1,344, far 917. Pass 1 runs each pair fresh; pass 2 moves B by up to 0.5 mm and about 1 mrad and reads pass 1's
manifolds as last tick's, their impulses replaced by markers (so a carried impulse names the point it came from). D's
paths: pass 1, 6,976 touching and 3,017 separated; pass 2, about 6,530 keep the cached feature's contact and 2,960 stay
separated by the cached axis.

| | F | V4 |
|---|---|---|
| pass 1: axis ties / clip ties (of 10,000) | 477 / 336 | 477 / 336 |
| axis kind and index differ outside axis ties | **0** of 9,523 | **0** of 9,523 |
| point count or ids differ outside all ties | **0** of 9,187 | **0** of 9,187 |
| support vertex differs outside its own ties | 0 of 4,901 (3,762 ties) | 0 of 4,901 |
| pass 2 (pairs whose pass 1 agreed): ties | 8 + 22 of 9,518 | 9 + 20 of 9,525 |
| pass 2: axis, ids, warm-start markers differ outside ties | **0, 0, 0** | **0, 0, 0** |
| normal error rms / max | 2.1e-7 / 1.3e-5 | 1.9e-7 / 9.3e-6 |
| anchor error rms / max (pass 1) | 3.0e-7 / 2.7e-5 m | 1.4e-7 / 8.0e-6 m |
| separation error rms / max | 1.3e-7 / 4.7e-6 m | 9.6e-8 / 4.8e-7 m |
| SAT face separation error rms / max | 1.6e-7 / 3.3e-6 m | 6.2e-8 / 2.6e-7 m |
| both GPUs, both passes: SAT records, manifolds, diagnostics vs the twin | every word identical | every word identical |

GPUs: the RTX 3060 (driver 0x988f0000) and the UHD 630 (0x194859), F-plain and V4, saturations 0 as the twin's (a
preview of decision point 2: the narrowphase is bit-exact on both). Corpus hashes (manifolds and SAT records, both
passes), equal on MSVC and clang-cl: **D c99764ed09a826b7, F e9b4716289b5513d, V4 305aa1bd8412ad08**; the twins'
sentinel clean, saturations 0. The largest anchor errors are conditioning, not arithmetic: in pass 1 an edge contact
of nearly parallel edges (the closest point moves by the distance error over the sine), in pass 2 (1.0e-4 m F, 6.7e-5
V4) an aligned stack whose incident edge lies 2.3e-5 m from a side plane (the clip fraction's |d1| / (|d1| + |d2|)).

**Ties.** A tie is a decision whose closest call in D was within the tolerance: kernels record each decision's margin
in `NarrowDiag` (written when `Params.narrowDiag` is set: the corpus, not the toy). An *axis tie*: a SAT decision on
the axis's path (best against second-best face or edge pair, a separation against the speculative distance, B's face
against A's by faceTolerance, the edge against 0.9 x the clipped face plus faceTolerance, the Gauss-map and parallel
tests of the chosen pair, the cache's drift against linearSlop); or, when the axis hangs on a contact's outcome
(touching or not, the clipped separation), whether that contact touches (its clipped separation against the speculative
distance, a clip pass ending with 2 or 3 vertices, the edge contact's segment ends, a runner-up support vertex with
another incident face) or which incident face it clips. A *clip tie*: a clip distance, a kept separation or a reduction
score. The tolerance: 1e-5 m (areas 1e-5 m^2, cosines 1e-5). Why: the verdict is the same at 1e-4 and at 1e-6; at 1e-7
the first differences appear (three support vertices in F, one id set in V4, every one with D margins between 1e-7 and
1e-6 m), so the decision noise is about 1e-7 m and 1e-5 m is 100 times it, and over twice the largest error of any
decision quantity (F's SAT face separations, 3.3e-6 m). Who ties: aligned stacks (350 of 1,045, 276 to 285 of those with
other ids: every incident vertex sits on a side plane) and vertices exactly on their mean normal (333 of 1,484: a box
vertex's three faces tie for the incident face); every other pose about 1 to 2%.

**Gates 1 and 2: passed** (`gate.py`, MSVC 19.42.34435 and clang-cl 23.1.2). Battery: 19 helpers (new: `unit3/len3`,
`posDelta`, `frame`), both GPUs identical in F and V4; the 16 old helpers' outputs and hashes unchanged by the division
fix; battery hashes **F 3ad4fb70e1a79038, V4 cbbf92da69bcf218, D 1c2d51d801f3a44f**. |unit3| - 1: F 1.3e-7, V4
4.6e-9, D 3.3e-16. Toy, 600 ticks, 1 and 8 threads, both compilers, sentinel clean, saturations 0 (run hashes now cover
bodies, manifolds and stages):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | 8d3d7d843172e43a | 9cc14fa256e17b98 | dc7a6b74c85ce9af |
| pile200 | f9f1c5eec440458b | e6245cd2f41ee5fe | 617cd170f7f9802b |
| pile200, gravity 0, body 112 pushed | c18c498a624405fd | ff2bfe031741d08c | d2d3dd8ba3d76104 |
| chip | 030e4a59a2bbd406 | 01bbbad48632148b | 6c3bea66d00f7a13 |
| ramp | f085246842067e47 | d433941b6c223b9c | ce039119b913c349 |

With no solver, the bodies' per-tick hashes of stack10, pile200, chip and ramp equal step 2's in every dialect (the
scene refactor and the integration are untouched); the manifolds and stages are new. stack10 has 10 touching pairs of
4 points from tick 1 (from tick 2, when the stages see them: 2 colours, one island of 10) and peaks at 13 as the stack
sinks through the ground; pile200 peaks at 67 touching; without gravity 198 bodies sleep at tick 31 and the pushed body
wakes a sleeper through a real contact at tick 33.

**The fix from the Spark.** Every division and square root in D now divides (roots) a value selected safe before the
test that discards it (`ds = d != 0 ? d : 1`, then the select), so a compiler that evaluates it ahead of the select (clang
18 on aarch64 did, for `recipT`) raises no flag; F's software reciprocal and rsqrt get the same guard (rsqrtPos(0) would
overflow and then go NaN if speculated). The sentinel stays strict. Not yet rerun on the Spark.

**Judgement calls and deviations.**
- **Box3D's scalar path is stale.** The brief's `b3CollideHulls` at line 2534 sits under `#else` of
  `B3_SIMD_COLLIDE_HULLS 1` ("this code has gone stale, will be deleted soon"); the compiled one differs: face A unless
  B's separation is larger (no 0.5 linearSlop bias); the edge contact when the face's is empty or the edge beats the
  clipped separation by linearSlop (no 0.9 factor); a failed edge contact keeps the face's cache; the Gauss-map test
  takes products below -1e-4 m^2 (a scale-dependent tolerance); early exits at > rather than >= the speculative distance;
  bounds that skip candidates; the support vertex chosen with the index in the low mantissa bits. I followed the brief
  (the scalar path's structure and tolerances) and put every tolerance in Params (`faceTolerance`, `edgeRelTolerance`,
  `parallelTolerance`, `reduceBias`, `speculativeSq`, `cacheTolerance`), so step 4b can move to the compiled values if
  b3ref2 disagrees; the cache-clearing and the product tolerance would be code changes.
- **The cache as data.** The cache lives in the manifold (`axisType`, `axisA`, `axisB`, `axisSeparation`), carried by
  prevIndex for every pair, touching or not. `narrowSat` tests the cached axis (a face: its support, >=; an edge pair
  whose arcs still cross, >); when it still separates, the pair is done and the cache kept. Otherwise it also runs the
  full SAT and records both, and `narrowClip` tries the cached feature's contact first, kept while touching and within
  linearSlop of the cache's separation (Box3D does not refresh the cache then: the drift is measured from the last full
  SAT); only when that fails does it use the SAT's verdict. Box3D skips the SAT when the cached contact holds; the toy
  computes it anyway (a GPU cost, two dispatches instead of a dependent four; step 7's timing will say).
- **Edge contact** in unit directions: t1 = (w . (uB x m)) / sin, t2 = (w . (uA x m)) / sin, compared with the edges'
  lengths, instead of b3LineDistance's 2x2 system in |e|^2 units (whose m^4 determinant underflows V4's formats for 2 cm
  edges); the same points. The parallel-edge test unsquared (max(|cba|, |dba|) < tol |eA|, the same test, no Q11.20
  underflow); the Gauss-map signs by comparison, never by product.
- **Formats.** Separations, distances and points Q8.24 inside the narrowphase (lsb 6e-8 m); stored separations and the
  cache's Q9.22 (DESIGN), the cache's drift compared in Q9.22. Areas and squared distances Q11.20, cosines Q1.30. V4's
  `unit3` is scale-free: the largest component shifted into [2^28, 2^29), the rsqrt in Q3.28, 30 bits at any length.
- **Incident face:** b3FindIncidentFace from the support vertex's lowest-index outgoing half-edge (the layout has no
  vertex-to-edge table; Box3D's hull builder picks one); only ties can tell them apart.
- **The support vertex** stored beside a face axis is never read again (the cached path finds a fresh one), so the gate's
  "indices" are the axis's (the face, or the edge pair); the vertex is still checked, outside its own ties: 0 differ.
- **Pass 2** is compared only where pass 1 agreed on the axis and the ids (elsewhere the inputs differ).
- **Touching for the stages** is last tick's manifolds (DESIGN): a pair joins islands, waking and colouring the tick after
  its manifold first has points, and tick 1 has none. Inactive pairs (no awake body) carry their manifolds
  (`copyManifolds`); static-static pairs never appear.
- **Warm starts** as Box3D's b3UpdateContact: each new point takes the impulse of the first unclaimed old point with its
  feature id; the friction and twist impulses (and V4's eP) carry while the pair keeps touching, zero otherwise. Anchors
  are from each body's centre of mass (the toy's hulls are about it, so Box3D's later centre shift is folded in), normal
  and anchors in the world frame by A's rotation.
- **The tick** is now: prepare; CPU stages (with prevTouching); wake, `narrowSat` and `narrowClip` over the active pairs,
  `copyManifolds` over the inactive, the substeps, finalize, `hashElements` over the bodies, `hashManifolds` over every
  pair (category 2, each element seeded by its two bodies, every field and every point slot).
- **The corpus** is built in double by `narrow.c` from `scene_box_hull` / `scene_chunk_hull`, every draw in its own
  statement; its bodies are static (no masses needed) and quantised by `toy_quantize`, so the corpus hash equal on both
  compilers covers the generator too.
- `gate.py` now makes `--out` absolute: a relative one made the toy's `--pos-out` fail and crash (the binaries run from
  `lab/gpu`).

**Open issues.**
- The scalar-versus-compiled question above, for step 4b against b3ref2.
- The full SAT runs for every pair the cached axis does not separate; the clip buffers (2 x 32 vertices) sit in private
  memory; the margins are computed in every dialect even when not written (cheap; a define could drop them for timing).
- The narrowphase is bit-exact on both GPUs in the corpus, but the toy's tick path has not run on a GPU yet (step 5).
- Not yet rerun on the Spark (the division fix, the aarch64 twins, the GB10 and llvmpipe on the corpus).
- Still no UBSan build; V4's margins never subtract from their sentinel (2^31 - 1).
- `NarrowDiag` and `SatAxis` are per-pair scratch; `SatAxis` could shrink once step 5 measures bandwidth.

**Step 4 starts from:** `toy.c`'s tick, between `copyManifolds` and the substeps: `prepareContacts` over the coloured
pairs (`st.colourStart` / `st.colourList`; pairs touching last tick, so a contact that appears this tick waits a tick:
decide whether to colour by this tick's manifolds, one more CPU point and readback, or accept the delay) and
`prepareJoints`; then per substep: integrate velocities, warm start, solve (push) and relax per colour, integrate
positions; restitution; finalize. The manifold this tick writes is `mf[t & 1]` (binding 13), read by the solver and
written back with its impulses so the next tick's `narrowClip` carries them by feature id. In each `ManifoldPoint`,
narrowClip fills `anchorA`, `anchorB` (Q8.24, world, from each centre of mass), `separation` (Q9.22), `featureId` and the
carried `normalImpulse`, and zeroes `baseSeparation`, `totalNormalImpulse`, `normalMass`, `shN`: prepare fills those,
and V4's `eP` (carried today) and the impulse formats are still to be chosen; the normal is world, Q1.30, from A to B.
Params has no contact hertz, damping, restitution threshold or friction mixing yet; the bodies' world inverse inertia is
`prepareBodies`' `invIw` (mantissa with `eI`).

## 2026-10-04: step 4 (prepare, the soft-step solve, restitution, the body kernels, contact recycling, real scenes, trajectories)

**What landed** (lines, now): `solve.slang` 715 (new, included by `kernels.slang`: `prepareContacts`, `warmStart`,
`pushContacts`, `relaxContacts`, `restitution`, `storeImpulses`), `traj.c/.h` 174/52 (new: the LPTRAJ1 writer and
reader, dialect-free C17, in `toy_common` so b3ref2 can link it), `traj.py` 223 (new: reads LPTRAJ1, the per-scene
sanity summaries), `num.slang` 1,032 (V4's exponent helpers `msbT`, `fmtV4`, `shiftOf`; `recipE`, `sqrtT`,
`discScale` in every dialect; D's guards; F snaps in `quatNormalize` and `rotateSym`), `narrow.slang` 1,421 (Box3D's
contact recycling; the Intel workaround), `kernels.slang` 252 (binding 18, the constraints; Box3D's isFast in finalize;
the manifold hash covers the recycling cache), `layout.h` 476 (`Constraint`, `ConstraintPoint`, `Pair.flags`, the
material in `BodyMass`, `Hull.maxExtentV`, `STATE_FAST`, the manifold's recycling cache and flags, the solver's and
recycling's Params), `toy.c` 1,073 (the tick, `--traj`, `--traj-every`, `--recycle`, the solve summary,
`TOY_DUMP_CONTACTS=T`), `stages.c/.h` 460/69 (every active pair coloured; `PAIR_SOLVE_*`), `scene.c` 639 (ramp, ratio,
chip; `stackN`), `quant.c/.h` 425/50, `battery.slang` 278 and `battery.c` 1,330 (four helpers), `gate.py` 184 (gate 4;
bounce and ratio in gate 2; V4 saturations must be 0), `narrow.c` 1,621, `kernels_glue.cpp` 141, `gen_toy.py` 72,
`CMakeLists.txt` 72 (`traj.c`; the twins' `maytrap`). About 2,250 lines added (1,160 of them the new files).

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1 to 4 (about 3 minutes); --gates 4 for the sanity runs alone
build/msvc/bin/toy_V4.exe --scene stack10 --sleep 0 --ticks 3600 --threads 1,8 --traj build/toy-scratch/s.traj
python toy/traj.py build/toy-scratch/s.traj --scene stack10       # or --body 10 --every 30 for one body's track
```

**The tick** is now: prepare bodies; CPU stages; wake, `narrowSat`, `narrowClip` (active pairs), `copyManifolds`
(inactive); `prepareContacts` (active pairs); per substep `integrateVelocities`, `warmStart`, `pushContacts` per colour,
`integratePositions`, `relaxContacts` per colour; `restitution` per colour, twice (Box3D's `restitutionIterations`), only
when some body has restitution; `storeImpulses` (active pairs); finalize; the hashes. Overflow-colour pairs (none in
these scenes) get one dispatch each, before the colours, as Box3D solves its overflow first. Box3D's world defaults, in
Params: 60 Hz, 4 substeps, gravity -10, contact hertz 30 (min( 30, 0.125 inv_h )), damping ratio 10, the static softness
at twice the hertz and half the ratio (b3MakeSoft; F in float with B3_PI as Box3D computes it), contact speed 3 m/s,
restitution threshold 1 m/s, linear slop 5 mm, speculative distance 20 mm, sleep 0.05 m/s, contact recycling at 50 mm.

**Contact recycling (a deviation from the brief, and the key finding).** With the brief's solve alone, an exactly aligned
stack of cubes never comes to rest, in every dialect, D included: stack10 drifted 9 cm in 5 s, a 3-stack kept 0.6 mm/s.
The 1 and 2 stacks rested to 1e-12, and with each layer offset 1 cm the 10-stack rested too (6.7e-12 mm/s), so the solve
was right and the feature ids were not: in an aligned stack the clipper's ids flip tick to tick (an incident vertex on a
side plane is kept or clipped by a hair, Box3D's `b3ClipPolygon` alike), the warm start is lost every other tick, and
the oscillation feeds itself. A scratch Box3D program (not in the repository; the toy's scenes, 60 Hz, 4 substeps)
settled it: Box3D stands the aligned stack10 (drift 0.55 mm, sink 26.03 mm, last-second speed 0.0024 mm/s), and with
`b3World_SetContactRecycleDistance( 0 )` it drifts 46 mm at 51 mm/s, as the toy did; with layers offset 1 cm it gives
drift 4.54 mm, sink 24.58 mm, the D twin 4.64 mm, 24.59 mm. Box3D's default `b3Collide` recycles a contact while the
relative pose stays within 10 linearSlop of the one cached at the last full narrowphase ("keep anchors but update
separation, same as sub-stepping. This eliminates jitter."). The toy now does the same, in `narrowSat` (the test, in
Box3D's order: neither body `isFast`, a valid cache, each body turned under 10 degrees, the relative position moved
under the tolerance (50 mm touching, else min( 50 mm, the speculative distance )), plus twice the arc of the relative
turn at the larger `maxExtent` vector, `b3ModifiedCross`) and `narrowClip` (a recycled manifold: last tick's, its
separations the cached ones plus n . (dc + RB rB - RA rA)). Every difference is gated per component before squaring.
The manifold carries the cache (`flags`, both rotations, B's pose in A's frame) and `baseSeparation` is now Box3D's
cached separation; `isFast` (finalize: max( |dp| + 2 |arc of dq|, (|v| + |arc of w|) dt ) > 0.5 minExtent, the arcs in
the body's frame) lives in `BodyState.flags` and stops recycling. `--recycle 0` (Params' distance 0, like Box3D's
setter) turns it off; the corpus (gate 3) runs with it off, so its comparisons are unchanged. Without it, every dialect
jitters on the aligned stack10 as Box3D does (D: drift 54 mm, 68 mm/s; F 58 mm, 50 mm/s; V4 19 mm, 44 mm/s).

**Gate 4: passed** (`gate.py`, MSVC 19.42.34435 and clang-cl 23.1.2; gates 1 to 3 too): per-tick hashes identical at 1 and
8 threads and on both compilers in every dialect, the sentinel clean everywhere, V4's saturation counters 0 in every run.
- Gate 1: 23 helpers, both GPUs every word identical in F and V4 (V4's `shiftOf` counts its 1,298 out-of-range vectors
  identically on both). Battery hashes **F 5add7392dc064a30, V4 6cddc843b7bf4f72, D 17fdc03440fd6596**. New helpers'
  accuracy: `sqrtT` F 1.2e-7, V4 4.8e-7 (relative); `recipE` F 8.4e-8, V4 1.8e-9; `discScale` F 1.3e-7, V4 1.9e-3 at
  scales down to 2^-20 (the limit keeps 28 bits less the ratio's), exactly 1 inside the disc in every dialect.
- Gate 2 (600 ticks, run hashes):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | f215cccb206c916a | b090bbc24000a20a | ff90bc0f84bb5f3d |
| pile200 | e635bae0d2463a70 | 38c289f62a391e3e | a06b44300bf74f15 |
| pile200, gravity 0, body 112 pushed | 269dc387427aadb6 | fe1c75a7c754da4a | 9b493f468eb9d5c8 |
| chip | 78cdacdad5ba1611 | 16cbc151b1c07471 | 0b14d5bbcbe86ba7 |
| ramp | ad627f06abb55c70 | 9fa8eac59c80fee5 | bf6268179ea2dea9 |
| bounce | 3fd13858e9d1f9fa | 694b4d84adf7e4fa | 419876943b12a5bc |
| ratio | 4031f223785390f0 | 8771b095410fd3f2 | f381c5ba9d11ca6e |

  Position rms against D over ticks 1-120 (DESIGN's bar: 1e-5 m F, 1e-4 m V4): stack10 F 1.9e-7, V4 1.0e-5; bounce F
  1.3e-6, V4 4.5e-6; ramp F 1.3e-5, V4 8.9e-6; ratio F 1.4e-4, V4 3.0e-7; chip F 4.1e-5, V4 1.4e-4; pile200 F 0.10 m, V4
  0.12 m (the piles diverge at the first impacts, ticks 55-70: chaotic, as expected; 4b should judge them by outcome).
- Gate 3: unchanged results (0 differ outside ties, both passes; both GPUs every word identical in F and V4). Corpus
  hashes **D ec5972a1a82de840, F 2ea78ea1cf0bdb32, V4 e71df90861191eec** (new: the manifolds carry the recycling cache).

**Physical sanity** (gate 4: the twins, MSVC, 1 and 8 threads, through the trajectories):

| | F | V4 | D | Box3D (scratch) |
|---|---|---|---|---|
| stack10, sleep off, 3,600 ticks: top drift / sink | 0.225 / 26.03 mm | 0.155 / 26.04 mm | 0.226 / 26.03 mm | 0.55 / 26.03 mm |
| yaw / tilt | 6.6e-5 / 1e-5 deg | 6.4e-5 / 6.2e-4 deg | 6.6e-5 / 0 deg | 8e-5 deg |
| resting speed (largest, last second) | 0.0010 mm/s | 0.014 mm/s | 1.4e-12 mm/s | 0.0024 mm/s |
| stack10, sleep on: asleep from tick (stays asleep) | 42 | 42 | 42 | 41 |
| pile200, 8 seeds: every body asleep, ticks (median) | 236-295 (276) | 231-328 (251) | 211-332 (249) | 4b |
| pile200: deepest point at rest (the 60 ticks before sleep), worst seed | 1.9 cm | 2.3 cm | 1.7 cm | 4b |
| pile200: escapes | 0 | 0 | 0 | |
| bounce: lowest centre; first apex (analytic 0.6875 m) | 0.1875; 0.6574 m | 0.1875; 0.6574 m | 0.1875; 0.6574 m | 0.1875; 0.6574 m |
| ramp: the 0.6 box's travel; the 0.2 box's acceleration (analytic 1.0866) | 0.72 mm; 1.087 m/s^2 | 0.72 mm; 1.087 | 0.72 mm; 1.087 | 4b |
| ratio: the 3 t box on 1 kg; the plate on chips | crushes; sinks through | the same | the same | the same (chips at 0.0059 m) |
| chip: lowest centre; at rest; asleep | 0.0099 m; tick 40; 70 | the same | the same | 4b |

- stack10 sinks 26 mm at the top: Box3D's soft contacts sag by impulseScale x load / (normalMass x biasRate) per contact,
  0.56 mm per carried cube on each dynamic contact and 0.07 mm on the static one (the theory gives 25.2 + 0.7 mm; the 2-
  and 3-stacks match it within 2%). V4's top cube wanders 0.19 to 0.155 mm over 50 s at about 13 um/s (no creep);
  F and D sit still to 1 um. With sleep on all three sleep at tick 42 with 5.7 mm of drift (the stack settles for 0.7 s).
- pile200's deepest points over a run (0.17 to 0.21 m, ticks 60-69) are dynamic-dynamic impacts at up to 12 m/s: the
  narrowphase runs once a tick and sees a contact only within the speculative distance, and Box3D has no continuous
  collision between dynamic bodies either. At rest the points sit at up to 1.7 to 2.3 cm: soft stacks of mixed sizes
  (mass ratios to 8). In F and V4 one column of seed 1 stands as a tower (top at 2.35 m) where D's falls: chaos.
- bounce: the box falls 10 cm a tick at 6 m/s and the contact appears within 2 cm, so it goes 6.25 cm into the ground
  before the push; the apex is 4.4% under the analytic one (93.1% of the rise) in every dialect and in Box3D, to four
  digits.
- ratio is a crush in Box3D too: mass ratios of 3,000 and 2,000 with a soft contact. Same outcome class.

**Judgement calls and deviations.**
- **Colouring** as the lead decided: every active pair (24 to 26 colours in pile200, about 1,200 active pairs of which
  500 touch; at most 305 to 341 dispatches a tick, against DESIGN's estimate of 170). A static or sleeping body is solved
  as static (Box3D's null body: zero mass, identity state, never written; static softness) and takes no colour; a
  sleeping body under a new contact is woken the next tick (step 3's touching rule), where Box3D wakes it in the step.
- **V4's impulses:** one exponent per constraint, eP = 51 - eM of the lighter solved body (mu 2^eP in [2^19, 2^21], room
  for 1,024 m/s x mu), not E11's largest normal mass, so it is fixed while the bodies' solved set is; a carried
  impulse is rescaled when eP changes (sleep and wake). Effective masses: the sums in a format eK 8 bits under the larger
  inverse mass, each body's lever term r . (I r) in its own inertia format, then `recipE` (a normalised reciprocal and
  its exponent); the 2x2 tangent mass normalised so the larger diagonal has 31 bits, its inverse at a division shift of
  msb(det) - 1 so no entry can clamp; the twist mass as nA . IA nA + nB . IB nB, each in its own format (Box3D sums the
  matrices first: F rounds a little differently). Shifts are clamped to [1, 62] and counted (DESIGN: [1, 63]; mas's
  rounding half overflows at 63). Zero saturations in every scene.
- **Restitution** is the extern's current Box3D rule (b3ApplyRestitution_Convex: armed when the approach at prepare was
  faster than the threshold and the point carried a compression impulse; the bias restitution x that velocity; Poisson's
  limit restitution x (compression + approach) - restitution so far), not the older maxNormalImpulse rule the brief
  names; no restitution propagation (Box3D's default). The relative velocity is sampled only for contacts with
  restitution, as Box3D does.
- **Friction centre:** Box3D's weights clamp( 2 - s / speculative, minWeight, 1 ), divided by the largest first so V4
  keeps its bits (the same centre); every manifold point has s under the speculative distance, so the weights are 1 and
  the centre is the anchors' mean. minWeight is 2^-20 (Box3D's 1e-10 is under F's snap; never reached).
- **Materials** in `BodyMass` (`friction`, `restitution`, Q1.30), mixed in prepare: sqrt( fA fB ) through the dialect's
  `sqrtT`, the larger restitution.
- **Box3D's isFast** without its "not falling asleep" condition (a body under the sleep speed cannot move half its inner
  radius in a tick). No continuous collision, gyroscopic term or damping (integrate velocities is step 2's).
- **The trajectory** records tick 0 and every Nth tick; a body not stepped that tick (asleep, static) has zero
  velocities, as Box3D's `b3Body_GetLinearVelocity` reports them (the toy keeps the last ones until the wake).
- **Scenes.** ramp: 25 degrees, not 20: with Box3D's mixing the 0.2 box meets the ramp's 0.6 at sqrt(0.12) = 0.346,
  under tan 20 = 0.364 by too little; the ramp lifted to 2 m, the boxes 2.5 m up it (6.5 m to slide). ratio: plus a 100
  kg plate on four 0.05 kg chips. chip: (6, -8, 0) m/s, into the ground. `stackN` (1 to 40) for experiments. bounce
  unchanged (restitution 0.5 on the box, 0 on the ground).
- **Floating point.** F: `quatNormalize` snaps its input (dq + h w dq / 2 left components near 1e-21, whose squares were
  subnormal); `prepareBodies` snaps R and `rotateSym` R S (cancelled matrix entries times F's round-off inertia
  off-diagonals, near 1e-17, now snapped to 0 at quantisation); the solver snaps the normals in the delta frames; sums of
  squares of snapped values are left unsnapped (discScale's |n|^2 snapped to 0 under 2^-30 skipped the clamp; a heavy
  pair's tangent determinant likewise). D: every guarded division or root now takes `safePos( x )` (x, or the smallest
  normal double): clang-cl folded `x > 0 ? 1 / sqrt( x > 0 ? x : 1 ) : 0` into a speculated `1 / sqrt( x )`. Slang's
  unsuffixed floating literals are floats: `2.2250738585072014e-308` became 0 until it got the `l` suffix (MSVC had
  hidden that by sinking the division into the branch). The twins are compiled with `-ffp-exception-behavior=maytrap`
  (clang, clang-cl; gcc's default `-ftrapping-math` stated): clang-cl otherwise raised underflow flags in `relaxContacts`
  that the source never raises (speculation or a spare vector lane), on hashes identical to MSVC's.
- **The Intel UHD 630 (driver 0x194859)** misread `pm.points[j]` (last tick's manifold, a private copy, indexed by a
  variable) once the manifold grew by the recycling cache: in the corpus's pass 2 it carried impulses read from other
  fields (661 manifolds, F only; the RTX 3060 and V4 were exact). `narrowClip` now reads last tick's points from the
  buffer in that loop, and both GPUs are exact again. The solver's point loops are unrolled ([ForceUnroll]) so no
  private array is indexed by a variable; they have not run on a GPU yet (step 5).
- `Pair` is 24 bytes (`flags`, `pad`). The constraint (460 bytes in F) is a per-pair buffer at binding 18, written by
  prepare; `storeImpulses` writes the normal and total impulses, the normal mass and shN, the world friction impulse,
  the twist impulse and eP back into this tick's manifold.

**Open issues.**
- Decision point 1's inputs are ready but not judged: DESIGN's first-120-tick rms bar fails where impacts dominate
  (chip, F ratio, pile200), which says more about the bar than the dialects; 4b should set per-scene windows or judge by
  outcome. pile200's resting penetration reaches 2.3 cm in V4 (seeds 5 and 6) against the 2 cm bar: compare with
  Box3D first.
- Box3D also has continuous collision for isFast bodies against static ones, the gyroscopic term, and its own sleep
  test (velocity and position terms per body); the toy has none of them. The chip spins about a principal axis, so the
  gyroscopic term is quiet there, but tumbling pile bodies will differ.
- Dispatches: up to 341 a tick (colours of every active pair); step 5 or 7 may want fewer colours (touching or nearly)
  or one dispatch for several colours.
- Not run on a GPU: the solver kernels and the recycling pass spirv-val (Vulkan 1.3) and the F NoContraction audit
  (no FDiv, no extended instructions), but the tick path is step 5's. The Intel finding says private-array indexing is
  the thing to watch.
- Not rerun on the Spark (aarch64 gcc and clang twins: the maytrap flag, D's guards). Still no UBSan build.
- The narrowphase's scalar-versus-compiled question (step 3) stands; recycling now hides most of the narrowphase's
  per-tick work for resting pairs, as Box3D's does.
- Debug aids: `TOY_DUMP_CONTACTS=T` (1 thread: every solved pair's constraint after tick T), `--recycle 0`, `stackN`.

**Step 4b starts from:** `b3ref2.c` in `lab/gpu/src` beside `b3ref.c` (its CMake target links `box3d`; add `toy_common`
for `scene.c` and `traj.c`): build each scene from `scene_build` (hulls from `SceneHull.v` through Box3D's hull builder,
or `b3MakeBoxHull` for the boxes; mass and inertia from the scene (`b3Body_SetMassData`) so both sides integrate the
same bodies; friction and restitution into the shape's material; positions, rotations and velocities as given), with
`b3DefaultWorldDef` plus gravity -10, `enableSleep` per run and 4 substeps at 1/60, contact recycling at its default
(and `b3World_SetContactRecycleDistance( 0 )` against `--recycle 0`), and write LPTRAJ1 with `traj_open` /
`traj_write` / `traj_close` (awake from `b3Body_IsAwake`; Box3D already reports zero velocities when asleep). The
deepest contact needs Box3D's manifolds (`b3Body_GetContactData`): print it as the toy's `solve:` line does. Then the
metrics: `traj.py`'s summaries grown into DESIGN's acceptance table, toy against Box3D (drift, yaw and resting speed
ratios, sleep-time ratios and pile200's medians over the eight seeds, apex and slide within 5% and 2%, outcome classes
for ratio and chip), and decision point 1. The scratch programs' numbers above (stack10 with and without recycling,
ratio, bounce) are what b3ref2 should reproduce first.

## 2026-10-04: step 4b (b3ref2, the metrics, the acceptance table: decision point 1)

**What landed** (lines): `b3ref2.c` 493 (new: Box3D on the toy's scenes, from `scene_build`, writing LPTRAJ1 and the toy's
solve line), `metrics.py` 245 (new: the acceptance table's measurements from trajectories and logs), `accept.py` 422 (new:
runs everything, measures, prints the table and writes `results/decision1.md`), `results/decision1.md` (generated: the
table, the details, how each number is measured, the commands), `toy.c` 1,139 (+72: `--perturb EPS`, `--start-float`, a
`rest:` line, the solve line's at-rest point with its tick and bodies), `CMakeLists.txt` 79 (the `b3ref2` target:
`box3d` and `toy_common`). No kernel, layout, Params or scene changed: gates 1 to 4 give step 4's hashes.

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc
python lab/gpu/toy/accept.py      # 119 runs (8 at a time, about 25 s) into build/toy-scratch/4b, then the table (about 15 s)
build/msvc/bin/b3ref2.exe --scene pile200 --seed 5 --ticks 3600 --traj build/toy-scratch/b.traj   # from lab/gpu
python toy/metrics.py build/toy-scratch/b.traj --scene pile200
```

**b3ref2** builds each scene from the generator's doubles: boxes through `b3MakeBoxHull` (their half extents), the
chunks through `b3CreateHull` from the scene's points (vertex, face and half-edge counts equal the scene's on every hull
of every seed); poses, velocities, friction and restitution as given; mass: Box3D's own from the shapes (at the scene's
density) is within 3.2e-7 (mass) and 2e-6 (inertia, relative Frobenius) of the scene's, and the scene's values are set
with `b3Body_SetMassData` (centre at the origin), so both programs integrate the same bodies. Settings: 60 Hz, 4
substeps, gravity -10, hertz 30, damping 10, push 3 m/s, restitution threshold 1 m/s, two restitution iterations, max
speed 400 m/s, one worker, contact recycling at Box3D's default (50 mm; `--recycle 0` sets 0), continuous off
(`--continuous 1` for the chip). "Awake" in its trajectories and solve line means stepped this tick (awake before the
step or after it), the toy's meaning: Box3D sleeps an island at the end of the step that crossed the threshold, the toy
at the start of the next tick, so the two read the same tick (`box3d:` prints Box3D's own reading, one less). The
deepest point is Box3D's manifold separation (the narrowphase's, before the solve; `b3World_VisitContactState`, our
patch) among the contacts with a body stepped that tick; the `rest:` line, every touching contact after the last tick.
It reproduces step 4's scratch programs: stack10 sleep off, top drift 0.5516 mm, sink 26.03 mm, last second's largest
speed 0.00237 mm/s; sleep on, asleep after step 41 (Box3D's reading; tick 42 in the toy's, the toy's own 42); with
recycling 0, 45.9 mm at 51.5 mm/s (scratch: 46, 51), and the toy with `--recycle 0` slides likewise (D 54.1 mm at 85.8
mm/s, F 58.2 at 49.7, V4 18.8 at 47.3); bounce's apex 0.6574 m; ratio's chips at 0.0059 m.

**Decision point 1** (`results/decision1.md`, MSVC, one thread). Judged rows: **Box3D 16 of 16, D 16 of 16, F 18 of 19
(fails the ramp's 120-tick rms), V4 20 of 20.**

| criterion (threshold) | Box3D | D | F | V4 |
|---|---|---|---|---|
| stack10 off: top drift, tick 60 to 3,600 (<= 8.86 mm) | 4.43 mm | 2.89 | 2.89 | 2.82 |
| stack10 off: top drift, tick 600 to 3,600 (<= 2 mm) | 0.0081 mm | 0.0080 | 0.0091 | **0.093** |
| stack10 off: yaw at 60 s (<= 0.5 deg) | 8e-5 deg | 6.6e-5 | 6.6e-5 | 6.4e-5 |
| stack10 off: top's mean speed, last 10 s (<= 1 mm/s) | 0.0019 mm/s | 1.3e-11 | 0.00069 | 0.0057 |
| stack10 on: asleep from tick (<= 63, stays) | 42 | 42 | 42 | 42 |
| pile200: seeds asleep by 3,600; median (max) tick (<= 406) | 8; 271 (350) | 8; 249 (332) | 8; 276 (295) | 8; 251 (328) |
| pile200: escapes (0) | 0 | 0 | 0 | 0 |
| pile200: deepest at rest, every pair once all sleep, worst seed (<= 2 cm) | 1.13 cm | 1.70 | 1.03 | 1.24 |
| pile200: the solve line's 60 ticks before sleep (reported) | 2.07 cm | 1.70 | 1.91 | 2.32 |
| pile200: deepest during the drop (reported) | 22.5 cm | 18.9 | 20.7 | 18.6 |
| ratio: 3 t on 1 kg; plate on chips (Box3D's class) | crushed through; sinks through | same | same | same |
| bounce: first apex (within 5% of 0.6875) | 0.6574 (-4.4%) | same | same | same |
| ramp: 0.6 box travel by 600; creep after 60 (holds) | 0.48 mm; 0 | 0.72; 0 | 0.72; 0 | 0.72; 0 |
| ramp: 0.2 box at tick 180 (within 2% of Box3D's) | 4.926 m | +0.51% | +0.51% | +0.50% |
| chip: lowest; last centre (above the ground) | 0.00987; 0.00988 | 0.00988 | 0.00988 | 0.00988 |
| chip: asleep from (sleeps, stays) | 73 | 70 | 70 | 70 |
| V4 saturations, every run (0) | | | | 0 |
| rms vs D, ticks 1-120: stack10 / bounce / ramp (F 1e-5, V4 1e-4) | | | 2.5e-7 / 1.3e-6 / **1.34e-5** | 1.5e-5 / 4.5e-6 / 8.9e-6 |
| rms vs D: chip / ratio / pile200 pooled (by outcome) | | | 4.1e-5 / 1.4e-4 / 0.080 | 1.4e-4 / 3.0e-7 / 0.11 |

Chaos yardstick (D with every dynamic body moved 1e-9 m along x and y, against D, ticks 1-120): stack10 1.4e-6 (the
stack's first seconds' sway amplifies 1,400 times), bounce 1.2e-9, ramp 4.0e-8, chip 4.9e-8, ratio 6.9e-7, pile200 0.018
m pooled (chaos: the piles diverge at the first impacts). The chip, ratio and pile200 rms are judged by outcome (the
lead's note); V4's chip (1.4e-4) would miss the 1e-4 bar and F's ratio (1.4e-4) the 1e-5 one if they were judged.

**Each failure, diagnosed.**
- **F, the ramp's 120-tick rms, 1.34e-5 m against 1e-5.** Not the per-tick rounding of float positions (that, emulated on
  D's motion, gives 4.4e-7 for the sliding box). The sliding box's error is a velocity offset of 1.5e-5 m/s down the
  slope set in ticks 1 and 2, its landing (it starts 1 mm above the ramp), then held, so the error grows linearly (3.1e-5
  m at tick 120). D started from F's start (`--start-float`: every position, orientation and velocity rounded to float,
  what F stores; the boxes sit at 2 to 3.5 m, where a float's step is 2.4e-7 m) is 8.8e-6 from D, and F against that D is
  4.9e-6: two thirds of F's number is the scene's start, which F cannot represent, amplified about 70 times by the
  landing; F's arithmetic alone is under the bar. The same split explains F's ratio (1.43e-4, all start: 2.1e-7 against D
  from F's start); the chip's 4.1e-5 is F's arithmetic (4.05e-5 against D from F's start), amplified by the impacts. Not a
  solver bug; the lead's options: start the scenes on a grid every dialect holds exactly (positions rounded to float in
  the generator are exact in V4's 32 fractional bits and in D; every hash changes), or two-word positions for F (step 2's
  open issue), or accept it as F's cost.
- **The pile200 penetration bar on the solve line's window** (V4 2.32 cm on seed 5, 2.29 on 6; Box3D 2.07 on 5, 2.04 on
  3; D 1.70, F 1.91). The window is not rest: each pile sleeps as one island, so 195 to 199 of the 200 bodies are awake in
  it, and its deep points are late slides and falls (of the points over 1.5 cm but one, the faster body of the pair moves at 0.07
  to 1.2 m/s within 10 ticks; the four over 2 cm at 0.36 to 1.2 m/s: V4's seed 6 is body 161, which fell at 3.8 m/s at
  tick 191, landing on body 30 at about 1 m/s, seed 5 body 130 sliding at 0.48 m/s, Box3D's at 0.36 and 0.47 m/s). The
  one deep point there that is at rest is D's seed 6 (1.70 cm between bodies 22 and 47 at 0.9 mm/s), and it is still there
  in the final pile. So the bar is judged on the pile at rest (every touching pair once every body
  sleeps, the `rest:` line): worst Box3D 1.13 cm, D 1.70, F 1.03, V4 1.24, all under 2 cm; the window stays in the table,
  reported. Judged on the window, Box3D would fail too.

**V4's stack creeps (within the bar; diagnosed).** Between ticks 600 and 3,600 V4's top cube drifts 0.093 mm (Box3D
0.008, D 0.008, F 0.009, the bar 2 mm): the whole stack slides on the ground at 1 um/s in a fixed direction (the bottom
cube 50 um in 50 s, linear in time) and leans (the top's tilt 6.2e-4 deg; Box3D's 7e-6); D and Box3D stop after the
sway, F wanders a few um. The cause is `mas`'s rounding, half up for negative values too: the aligned stack's anchors are
exact binary values (+-0.25 m), so products such as r x P land on exact halves often, and -x rounds to a magnitude other
than x's, a bias of a fraction of an lsb with a sign the geometry fixes. An experiment (reverted: `num.slang`, `gen/toy`
and both builds are byte for byte step 4's again), `mas` rounding half away from zero: the bottom cube 0.7 um, the top 16.8
um over the same 50 s, tilt 1.6e-5 deg; the jitter unchanged (top's mean speed 0.0056 mm/s); stack10's 120-tick rms
against D 2.5e-6 (from 1.5e-5); pile200 still sleeps on every seed (median 249.5, max 320; at rest at most 1.42 cm;
saturations 0); the ramp unchanged (V4's slide 1.2e-4 m behind D at tick 180: a separate, small friction bias, not
diagnosed). Not kept: it changes every V4 hash and the battery's (mul64 relies on half up, but only for non-negative
products; lpDivQ31's exactness and gate 1 were not rerun under it). **Recommendation:** symmetric rounding in `mas` (half
away from zero, or half to even) before step 5's reference hashes. The kill-early note's clz-narrowed r x P was not
needed: V4's stacks rest and its piles sleep.

**Judgement calls.**
- `b3ref2.c` lives in `toy/` (the brief: edit only `toy/`), not `src/` as step 4's note proposed.
- stack10's drift: the brief's "from the settled position after the first second" is judged as written (tick 60), but the
  stack still sways there (up to 6 to 7 mm for about 5 s, every program), so that row measures the sway's return; a second
  judged row starts at tick 600, when the sway has died: the creep. Yaw: the top cube at tick 3,600 against the start.
  Resting speed: the top cube's mean linear speed over the last 600 ticks.
- pile200: the sleep tick per seed from the solve line (every tick; the trajectories keep every 10th); the median and
  max over the eight seeds; escapes, a dynamic body's centre outside |x|, |z| <= 2 m (the walls' inner faces) or below the
  floor's top in any record. Penetration as above.
- ratio's classes from the last record (metrics.py's `ratio`: rests, partly sunk, crushed through, pushed aside; the plate
  rests, partly sunk, sinks through, chips squeezed out). Every program: the 3 t box crushes through the 1 kg box (ends on
  the ground, the small box inside it) and the plate sinks through the chips.
- bounce: the apex height against the analytic (-4.4% in all four, Box3D identical to four digits). Read as the rise above
  the resting centre it is 93.1% of the analytic in all four, so "5% of the rise" would fail Box3D too (the box is 6.25
  cm into the ground before the contact pushes: the time step).
- ramp: holds = at most 1 cm down the slope by tick 600 and 1 mm after tick 60 (it starts 1 mm above the ramp); slides =
  the distance at tick 180 (3 s, still on the 6.5 m ramp) against Box3D's (4.926 m; the toy 4.951 m in every dialect;
  the acceleration 1.087 m/s^2 in both, the analytic 1.0866).
- chip: no tunnelling = its lowest and last centres above the ground. Box3D with continuous collision on: lowest 0.009878
  m, asleep from 79 (off: 73).
- The chaos yardstick moves the bodies along x and y: along x alone it is an exact symmetry of every scene but the ramp
  and the pile (the first runs gave 1e-9 flat on chip, ratio, bounce and stack10).
- The rms against D is computed from the trajectories (every tick to 120), as `--pos-ref` does; it equals gate 2's.

**Box3D beside the toy, beyond the table.** Every outcome agrees. Box3D's ratio sleeps at 79 (the toy 53) with its 1 kg
box 30 mm deeper; the chip at 73 (70); the 0.2 box slides 0.5% less far; pile200's sleep median 271 against 249 to 276
and its drop 22.5 cm deep against 18.6 to 20.7. bounce's face-on landing sends the box sideways (about 0.2 m/s) and
spinning in both, as mirror images (the four points' solve order differs with the hulls' vertex order), so Box3D's first-120-tick
rms against D is 0.18 m there; across programs the rms says nothing.

**Gates 1 to 4: passed** (`gate.py`, MSVC 19.42.34435 and clang-cl 23.1.2; logs in `build/toy-scratch/gate4b`): battery
hashes F 5add7392dc064a30, V4 6cddc843b7bf4f72, D 17fdc03440fd6596 and every gate 2 run hash as step 4's, the corpus
hashes as step 4's, the sentinel clean, V4's saturations 0.

**Open issues.**
- The two calls above for the lead: the scenes' starts (F's ramp and ratio rms) and V4's rounding.
- V4's small friction bias on the ramp (the sliding box 2.5e-5 slower, relative), not diagnosed; F's chip error (4e-5 in
  120 ticks, its arithmetic amplified by the impacts) not taken apart.
- DESIGN's arm row is step 6's (with the joint), grid:K step 7's.
- Box3D still differs in its gyroscopic term, continuous collision (the chip behaves the same with it), its sleep test
  (a position term) and waking in the step; none shows in the table.
- accept.py runs only the MSVC twins and one thread (gate 2 covers the rest); the Spark has not run 4b.

## 2026-10-04: step 5a (V4's rounding, float starts) and step 5 (the whole tick on the GPUs: decision point 2)

**Step 5a: the two corrections** (they change hashes, so they land before step 5's reference hashes).
1. **`mas` rounds halves away from zero** (`num.slang`): the offset is 2^(sh-1) for x >= 0 and 2^(sh-1) - 1 for x < 0 (0 for
   sh = 0), computed as `((1 << sh) + (x >> 63)) >> 1`, no select. **lpDivQ31 re-derived:** `mul64`'s product (q |d|, or
   lim |d| in divClamp) is never negative, and for x >= 0 halves away is halves up, so its high word is still mas less the
   low word's top bit (H + (L >> 31) - (L >> 31)); the estimate's mas can be negative but is clamped at 0, where a negative
   half's rounding cannot show (half up gives 0, half away -1, both 0 after the clamp). So lpDivQ31 gives the same words
   under either rounding, and the battery says so: the helper hashes of lpDivQ31, divClamp, recip, rsqrt, recipE, sqrtT,
   clz, atan2, quatNormalize, quatMul, rotate, pos/grid, posDelta, frame and msb/shiftOf are unchanged; mul, sum2/dif2,
   sum3, rescale, unit3/len3 and discScale changed (their vectors have negative exact halves), with the same saturation
   counts. lpDivQ31 equals C's `/` on 0 of 79,976 differing inputs, and 0 of 1,063,016 at `--n 65536 --seed 7`; both GPUs
   every word identical in every helper (F and V4). Battery hashes: **V4 6cddc843b7bf4f72 -> c45f361e4c33722e** (big run
   6894d90a8ee222b1 -> dce1bffc300d0d5b), F 5add7392dc064a30 and D 17fdc03440fd6596 unchanged.
2. **Starts every dialect holds exactly** (`scene.c`'s `scene_round_start`, called by `scene_build`, so b3ref2 gets it
   too): every body's position, orientation and velocities rounded to float (step 4b's `--start-float`, now the default and
   removed as an option), **then to V4's grid for that quantity** (positions 2^-32, orientations 2^-30, v 2^-22, w 2^-20).
   The check: float alone is exact in D always and in V4 for |x| >= 2^(23 - bits) (positions above 2 mm, quaternion
   components above 2^-7, v above 2 m/s or 0, w above 8 rad/s or 0). Every scene passes except pile200's random
   quaternions: 4 to 8 components per seed below 2^-7 (seed 1: 7, 2: 4, 3: 6, 4: 6, 5: 8, 6: 4, 7: 7, 8: 4) would have
   been rounded by V4's quantisation. The grid step moves exactly those, by under 2^-31, and leaves a float (at most 24
   significant bits), so now every start value is exact in F, V4 and D: the toy's new `start:` line checks it in every
   run (every scene, every dialect: exact). accept.py's "D from F's start" runs (Df) are gone (they equal D).

**Before and after** (gates 1 to 4 and `accept.py` rerun; `results/decision1.md` regenerated):

| | step 4b | step 5a |
|---|---|---|
| V4 stack10 sleep off, ticks 600 to 3,600: bottom cube's drift; top's drift | 50.5 um; 0.093 mm | **0.71 um; 0.0168 mm** (D 0.01 um; 0.008 mm, Box3D 0.97 um; 0.0081 mm) |
| V4 stack10: top's tilt at 60 s; mean speed, last 10 s | 6.2e-4 deg; 0.0057 mm/s | 1.6e-5 deg; 0.0056 mm/s |
| V4 stack10, recycling off: top drift; last second's speed | 18.8 mm; 47.3 mm/s | 46.5 mm; 51 mm/s (Box3D 45.9; 51.5) |
| rms vs D, ticks 1-120, stack10: F; V4 | 2.5e-7; 1.53e-5 | 2.5e-7; **2.54e-6** |
| bounce: F; V4 | 1.3e-6; 4.5e-6 | 1.3e-6; 4.7e-6 |
| ramp: F; V4 | **1.34e-5 (FAIL)**; 8.9e-6 | **4.9e-6**; 1.9e-5 |
| chip: F; V4 (reported) | 4.1e-5; 1.4e-4 | 4.05e-5; 1.5e-4 |
| ratio: F; V4 (reported) | 1.43e-4; 3.0e-7 | 2.1e-7; 1.43e-4 |
| pile200, 8 seeds pooled: F; V4 (reported) | 0.080; 0.11 | 0.077; 0.106 |
| pile200 sleep tick, median (max): Box3D; D; F; V4 | 271 (350); 249 (332); 276 (295); 251 (328) | 274 (622); 270 (351); 258 (288); 274 (330) |
| pile200 deepest at rest (judged, <= 2 cm), worst seed: Box3D; D; F; V4 | 1.13; 1.70; 1.03; 1.24 cm | 1.13; 1.23; 1.20; 1.80 cm |
| decision point 1, judged rows passed: Box3D; D; F; V4 | 16/16; 16/16; 18/19; 20/20 | 16/16; 16/16; **19/19**; 20/20 |

The creep is gone as step 4b's experiment said (to the digit: 0.7 and 16.8 um). F's ramp miss was its start (F against
D is now 4.9e-6, step 4b's "F against D from F's start"). ratio swapped its two numbers: it has two outcomes about 1e-4 m
apart (the same class) that a start difference of 1e-10 m picks; D from the unrounded start agreed with V4 before, D from
the float start agrees with F now, and V4 is 1.43e-4 m (max 1 mm) from it, F's old number to the digit. V4's ramp rms
doubled (the sliding box's small friction bias, step 4b's open issue). Box3D's pile200 seed 2 now sleeps at 622 (one
late body; its seeds 2, 4 and 5 moved with the 4 to 8 grid-moved quaternion components). The solve line's "at rest"
window (reported, not judged) reached 3.21 cm in D (seed 2) and 3.11 in V4 (seed 8): late impacts, as in step 4b.

Gates 1 to 4 after 5a: **passed** (MSVC 19.42.34435, clang-cl 23.1.2; the sentinel clean, saturations 0). Gate 2 run hashes
(600 ticks), unchanged ones marked =:

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | = f215cccb206c916a | = b090bbc24000a20a | 7b3114bdf3ab6d1a |
| pile200 | bc83fe37d85e9145 | 080a40a2a279debc | a51a7c89717698d6 |
| pile200, gravity 0, body 112 pushed | 553488d0c0eb1ea1 | cea8ea392605b5c4 | 0df970da7edaad96 |
| chip | 7e10e560cf31c9e0 | = 16cbc151b1c07471 | 2b4686ff5dc8fa82 |
| ramp | 7aa34cf634c31dc3 | = 9fa8eac59c80fee5 | 88a9bf253b0c28db |
| bounce | = 3fd13858e9d1f9fa | = 694b4d84adf7e4fa | b0af3f647a8ee84c |
| ratio | cd540394c778a0cc | = 8771b095410fd3f2 | 64b0b2b3a538be00 |

Gate 3: corpus hashes D ec5972a1a82de840 and F 2ea78ea1cf0bdb32 unchanged, **V4 222bdd55590f1dba** (was e71df90861191eec);
0 differ outside ties in both passes, both GPUs every word identical. Gate 4: every scene's numbers as in the table above.

**Step 5: what landed** (lines, now): `toy.c` 2,545 (was 1,139: the GPU tick path, the trace, the dispatch hashes, the
reference files, the field tables that name a word; DESIGN's estimate was 600), `gate.py` 272 (gate 5), `scene.c/.h` 677/82
(the start), `num.slang` 1,038 (mas), `accept.py` 418 (no Df), `results/ref/` (new: 42 reference files, 14 configs x F,
V4, D, about 10 KB each), `results/gate5-windows.txt` (new: gate 5's output on this machine).

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1 to 5 (about 15 minutes); --gates 5 for decision point 2 alone
python lab/gpu/toy/gate.py --gates 5 --write-ref     # (Windows, MSVC first) rewrite results/ref after a change meant to change hashes
build/msvc/bin/toy_V4.exe --scene pile200 --seed 3 --ticks 1200 --threads 1 --gpus 3 --ref toy/results/ref/pile200-s3.V4.txt   # from lab/gpu
build/msvc/bin/toy_F.exe --scene stack10 --ticks 600 --gpu 1 --trace 600        # every buffer after every tick; a difference to the word
python3 lab/gpu/toy/gate.py --compilers gcc,clang --gates 5                    # another machine, against the committed files
```

- **`toy.c` is one simulation (`Sim`: its buffers and CPU stages) driven two ways.** The twin's tick: bind, prepare,
  `sim_gather` (sleep counters and last tick's point counts from its own buffers), `sim_stages` (stages_tick, growth,
  lists, the dispatch list), the rest. A GPU's tick (`gpu_tick`): **submit 1** uploads the prepare list, runs
  `prepareBodies`, and reads back the stages' inputs (the AABBs whole; the sleep counters and last tick's point counts as
  4-byte copy regions, one per body and one per last tick's pair from the other manifold buffer: no new kernel, no
  readback of the manifolds); the CPU runs the same `sim_stages` on them; **submit 2** uploads the lists and the pairs,
  runs every dispatch of the list with a barrier and a timestamp after each, and reads back `hashElements`' and
  `hashManifolds`' arrays, summed on the CPU as the twin's. The manifolds are ping-ponged on the GPU: two buffers, two
  descriptor sets (tick parity p binds manifolds = mf[p], prevManifolds = mf[p ^ 1]). Per-pair buffers grow when the
  pairs outgrow them, the manifolds, SAT records, manifold hashes and constraints keeping their contents and the rest
  zero, on the GPU (fill, copy) and now on the twin too (its realloc's new parts are zeroed; never read, so no hash
  changed: gate 5's run hashes of bounce, ramp, ratio and chip equal gate 2's). So every byte of every buffer is
  deterministic and the whole buffers compare.
- `--gpus MASK` / `--gpu N`: each GPU runs the scene after the twins; per-tick hashes compared, the first differing tick
  printed with both triples; the saturation counter read back at the end against the twin's; the costs.
- `--trace T`: the twin (one thread) and the GPU in step; after every tick's prepare and after its rest every buffer the
  kernels write (state, pose, mass, AABBs, hashes, both manifold buffers, SAT records, manifold hashes, constraints) is
  read back and compared word for word. The first tick whose rest differs is stepped again from the prepare's state
  (the twin's snapshot, uploaded to the GPU), dispatch by dispatch, each followed by the whole comparison, and the first
  differing word is named: tick, dispatch, kernel, start, count, buffer, element, field (from offsetof tables:
  `manifolds[1][37].points[2].normalImpulse`), both values (float or integer, and hex), the element's words on both
  sides, the differing words per buffer. Tested with `TOY_TRACE_POKE=T:D` (flips a bit of the twin's first awake body's
  v.y after dispatch D of tick T): found at the right dispatch, body and field, in F and V4, stack10 and pile200 (a
  pile200 F poke that later rounding absorbed within the tick was rightly not reported).
- `--dispatch-hashes FILE`: an FNV over the same buffers after every dispatch of the first twin run (tick, dispatch, kernel,
  start, count, hash). pile200 seed 3, 120 ticks: 17,571 lines, identical on MSVC and clang-cl.
- `--ref-out FILE` / `--ref FILE`: the config line (scene, seed, dialect, ticks, gravity, sleep, recycle, push, perturb),
  the twin's identity, the three hashes every tick to 120 then every 60th and the last, and a run hash over every tick;
  `--ref` checks every twin run (1 and 8 threads) and every GPU run against it (the config must match).
- The toy's `start:` line (above), and the solve and pose reports split into functions; the per-tick table, the final
  line and every gate 2 and 4 output unchanged.

**Gate 5 = decision point 2: passed** (`gate.py`, `results/gate5-windows.txt`). RTX 3060 Laptop (driver 0x988f0000) and
UHD 630 (driver 0x194859), F-plain and V4: stack10 for 3,600 ticks with sleep off and on, pile200 seeds 1 to 8 for 1,200
ticks, bounce, ramp, ratio and chip for 600: **every tick's three hashes identical to the twin's on both GPUs in both
dialects, 14 of 14 runs each; saturations 0 on the GPUs and the twins.** The twins (F, V4, D; 1 and 8 threads) are
identical to the reference files on MSVC and clang-cl in all 42 configs (the files written by the MSVC twin, then
checked in a second full run of gates 1 to 5, which passed). **No difference was found, so nothing was fixed**: the
narrowphase, the solver (warm start, push, relax, restitution with their per-point loops unrolled), the speed caps,
finalize, the sleep counters, the hashes and the int64 `mas` gave the same bits on both drivers from tick 1 to the end;
the Intel driver's misread of a variably indexed private array (step 4's) did not reappear. The trace found nothing in
pile200 (V4, Intel, 200 ticks) or stack10 (V4, NVIDIA, 60 ticks).

**GPU costs** (informative; one dispatch per colour per substep, a barrier after every dispatch, two submits a tick). ms per
tick over the run (wall with the two submits, readbacks and CPU stages; kernels from timestamps) and the busiest tick
after the first:

| | RTX 3060 F | RTX 3060 V4 | UHD 630 F | UHD 630 V4 |
|---|---|---|---|---|
| stack10, sleep off (40 dispatches a tick, all awake) | 0.47 / 0.16 | 0.77 / 0.38 | 2.8 / 2.0 | 5.1 / 4.4 |
| pile200, 8 seeds, 1,200 ticks (F and V4 all asleep by tick 223 to 330) | 0.79-0.93 / 0.25-0.35 | 1.15-1.55 / 0.54-0.93 | 4.3-4.7 / 1.6-1.9 | 4.9-7.4 / 3.1-5.5 |
| pile200, the busiest tick (270-329 dispatches) | 3.2-4.5 / 2.1-3.0 | 5.3-7.9 / 4.0-4.9 | 13.8-16.9 / 10.7-13.7 | 23.7-29.9 / 21.6-27.9 |
| bounce, ramp, ratio, chip (28-76 dispatches) | 0.25-0.38 / 0.03-0.08 | 0.27-0.50 / 0.04-0.17 | 0.95-1.6 / 0.31-0.89 | 1.1-3.4 / 0.63-2.6 |

Every cell reads wall / kernels. The solver's per-colour dispatches (warm start, push, relax) take
74 to 81% of pile200's kernel time, the narrowphase 11 to 14%, the hashes 1 to 10%. Both GPUs are dispatch-bound: the UHD
630 costs about 50 us a dispatch with its barrier (stack10 F: 2.0 ms for 40), the RTX 3060 about 4 us; V4 costs 1.7 to 2.7
times F (the int64 `mas`). The twins for comparison (MSVC): stack10 0.18 / 0.22 ms a tick (F / V4, 1 thread), pile200
1.17 / 1.39 (1 thread), 0.75 / 0.87 (8 threads), averaged over the run.

**Judgement calls.**
- The starts go to V4's grid after float, so the three dialects (and Box3D) share every start bit; float alone left 4 to
  8 pile200 quaternion components per seed inexact in V4.
- The trace compares whole buffers at every tick's two points and replays only the first differing tick dispatch by
  dispatch (from the prepare's state, both sides restored from the twin's snapshot), rather than reading back every
  buffer after every one of up to 330 dispatches from tick 1: a difference that a later dispatch of the same tick fully
  overwrites is not reported (the end state equals); one that reaches the tick's end is, to the first dispatch and word.
  The replay also tells a non-reproducible difference apart ("differs at its end but not dispatch by dispatch").
- Two submits a tick (the brief's one readback for the CPU stages, plus the hashes'); the stages' inputs through copy
  regions rather than a gather kernel; the GPU runs after the twin rather than interleaved (the same comparison, and a
  divergent GPU cannot disturb the twin's run).
- Reference files for D too (the twins on other machines), and a run hash over every tick beside the kept lines.
- gate.py's gate 5 runs every GPU (`--gpus`, default 0xff) on the first compiler's build and traces a GPU that differs
  to the tick it first differs; `--write-ref` rewrites the files (from the first compiler, MSVC on Windows).
- The costs are averages over the run (sleeping ticks too) and the busiest tick after the first (tick 1 warms the driver
  up: up to 9 ms on the UHD).
- `posDeltaWord` (the two-word position difference, Q8.24) still rounds half up: it is not `mas` and the brief named only
  `mas`; b - a and a - b can differ by one lsb at exact halves (a quarter of all differences: positions move in Q5.26
  steps shifted by 6). No creep shows (the stack's bottom cube 0.7 um in 50 s), but it is the same kind of bias.

**Open issues.**
- Not run on the Spark (GB10, llvmpipe; gcc and clang twins): `python lab/gpu/lab.py remote consulear@spark-d683 --machine
  spark-gb10 --compiler gcc --twins clang --then "python3 toy/gate.py --compilers gcc,clang > results/spark-gb10/toy-gate.txt"`
  ships toy/ with results/ref and runs gates 1 to 5 there (llvmpipe's pile200 runs may be slow); then the Mac.
- `posDeltaWord`'s half-up rounding (above). V4's ramp friction bias (step 4b) and F's chip error not taken apart.
- Cost: up to 330 dispatches a tick, each with a full barrier; fewer colours (only touching pairs), several colours per
  dispatch, or a persistent solver kernel are step 7's questions. Bandwidth is not the limit yet.
- No UBSan build yet (DESIGN's gate).
- The joint (step 6) and its kernels are not in gate 5 yet; when they land, `--write-ref` and gate 5 again.

## 2026-10-04: step 6 (the revolute joint, the arm, its acceptance rows; gate 5 with the arm)

**What landed** (lines, now; about 1,750 added): `joint.slang` 527 (new, included by `kernels.slang`: `prepareJoints`,
`warmStartJoints`, `solveJoints`, `relaxJoints`, `hashJoints`), `num.slang` 1,092 (`twistAngle`, `invertSym`), `layout.h`
559 (`Joint`, `JointCommand`, `JOINT_*`, formats `S_JA` Q3.28 for angles, `S_TQ` Q15.16 for torques, `S_J` Q9.22 for the
point block; the joints' softness at the end of `Params`), `kernels.slang` 257 (bindings 19 to 21: joints, jointHashes,
jointCommands; `CAT_JOINT` 3), `stages.c/.h` 609/89 (`stages_set_joints`: the jointed-pair filter, waking and islands through
joints, joints coloured first), `scene.c/.h` 815/111 (`SceneJoint`, the arm, `grid:K` for step 7, `scene_servo_target`,
`scene_joint_text`), `quant.c` 481, `toy.c` 2,695 (the joints' buffers, lists, commands and dispatches; the fourth hash; the
reference files' joints column; the views and field table for the trace), `b3ref2.c` 538 (Box3D revolute joints, the servo
set before each step), `metrics.py` 354 (`parse_joints`, `joint_state`, `arm`, `angle_diff`), `accept.py` 455 (the arm's
rows), `gate.py` 285 (the arm in gates 2, 4 and 5), `battery.c/.slang` 1,497/305 (two helpers), `kernels_glue.cpp` 156,
`narrow.c` (22 bindings), `results/ref/arm*.txt` (6 new reference files).

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1 to 5 (about 9 minutes); the arm is in gates 2, 4 and 5
build/msvc/bin/toy_V4.exe --scene arm --ticks 1200 --threads 1,8 --gpus 3 --ref toy/results/ref/arm.V4.txt --traj build/toy-scratch/a.traj --log build/toy-scratch/a.txt
python toy/metrics.py build/toy-scratch/a.traj --scene arm --log build/toy-scratch/a.txt   # the joints' gap, overshoot, angles
```

**The joint**, after Box3D's `b3RevoluteJoint`, one struct per joint (`Joint`: the definition, the step's prepared data and
the impulses, which persist with the joint's fixed index: the warm start across ticks):
- `prepareJoints` (active joints): the world frames `q local` and anchors `rotate( q, local )` (the hulls are about the
  centre of mass, so Box3D's `localCenter` is 0), `deltaCenter = pB - pA` (`posDelta`), the hinge axis (frame A's z), the
  masses (zero for a body solved as static), V4's exponents as the contacts' (eP from the lighter solved body, the point
  block in eK, 8 bits under the larger inverse mass), the axial mass `1 / (z . (IA + IB) z)` (as the contact's twist mass),
  the 2x2 axis block on the perpendicular axes `0.5 rotate( qA, s e + v x e )` inverted as the contact's tangent mass, and
  the 3x3 point block `K = (mA + mB) E + sum (r x e_i) . I (r x e_j)` inverted by `invertSym`: Gaussian elimination with
  reciprocal pivots (`divClamp`), `K = L D L^T`, `K^-1 = L^-T D^-1 L^-1`. V4 normalises K so its largest diagonal entry is in
  [0.5, 1) (Q1.30); the reciprocal pivots, the multipliers and the inverse are Q9.22, so a pivot under 2^-9 of the largest
  diagonal (a condition near 512) saturates and counts. The twist angle (`twistAngle`: `2 atan2( z, s )` through the
  dialect's atan2, qs's sign taken out; V4's Q2.29 atan2 doubled is the same mantissa in Q3.28, so angles and their
  differences fit), and the motor's speed: the speed motor's, or the servo's `gain (target - angle)` capped at its largest
  speed, the target a per-tick command (`JointCommand.target`, the scene's rule quantised by the CPU and uploaded with the
  lists). The carried impulses rescaled to this step's eP.
- `warmStartJoints` per colour and substep: the linear impulse at the anchors (rotated by the bodies' delta rotations), the
  axis block's impulse on the last solve's perpendicular axes, the motor and limit impulses on the hinge axis.
- `solveJoints` (useBias) and `relaxJoints` per colour and substep, Box3D's order: the motor (its accumulated impulse capped
  at `maxMotorTorque h`), the lower and upper limits (speculative `c / h` while the margin is positive, soft with the bias
  when penetrating, rigid in the relax), the axis block (`C = (rel.x, rel.y)`, the perpendicular axes recomputed from the
  current relative rotation and kept for the warm start), the point block (`C = (dpB - dpA) + (rB - rA) + deltaCenter`).
  Box3D's joint softness (`b3MakeSoft( min( 60, 0.25 inv_h ), 2, h )`) in Params.
- `hashJoints`: every field, the joint's index seeding the element, category 3.
- Joints are coloured before the pairs, in index order; jointed bodies never make a broadphase pair (Box3D's
  `collideConnected` false); a joint joins its two awake bodies' islands and wakes a sleeping partner as a touching pair
  does; inactive joints (no awake body) keep their impulses. Per colour the joints' dispatch goes before the pairs' (Box3D's
  joint blocks before its contact blocks; within a colour they share no moving body).
- Scenes without joints are untouched: no joint dispatch, the stages hash and the run hash unchanged, the reference files'
  tick lines carry a fourth hash only when the scene has joints (and `--ref` checks that the file and the scene agree).

**The arm** (`scene.c`'s `build_arm`; bodies 0 ground, 1 base, 2 and 3 the links, 4 to 7 the stack): a static base (0.3 x 0.5
x 0.3 m); link 1 (1 m x 10 cm x 10 cm, 10 kg) hinged about the vertical 3 cm above the base, a servo (torque cap 150 N m, gain
8/s, at most 1.5 rad/s) whose target is a triangle wave of +-1 rad and 6 s (0 at tick 0, +1 at tick 90, -1 at 270); link 2
(0.8 m, 8 kg) hinged to link 1's end across a 4 cm gap, limits +-0.5 rad, no motor; both links cantilevered at 0.58 m. A
stack of four 0.2 m cubes (8 kg) at (1.2, 0.8) in x, z, each offset by up to 1.5 cm and turned by up to 3.4 degrees, in
link 2's path on the return sweep (first reached near tick 245). Link 2 swings onto its limits whenever link 1 speeds up,
slows or turns (135 to 146 of 1,200 records at a limit), and from tick 245 knocks the stack over and sweeps through the
boxes every 3 s.

**The acceptance rows** (`accept.py`, `results/decision1.md`; MSVC, one thread, 1,200 ticks, sleep on; every number from the
trajectories by `metrics.py`'s `arm`, Box3D's `b3RevoluteJoint_GetAngle` reproduced in Python):

| criterion (threshold) | Box3D | D | F | V4 |
|---|---|---|---|---|
| servo joint: angle against D's, ticks 1-120, largest (F, V4 <= 1 mrad) | 0.58 mrad | moved 1e-9 m: 2.0e-7 mrad | **0.00013** | **0.033** |
| limited joint: the same | 2.33 mrad | 6.1e-7 mrad | **0.00055** | **0.10** |
| hinge gap, largest over the run: servo; limited joint (<= 1 mm) | 0.62; 0.94 mm | 0.654; 0.953 | **0.655; 0.994** | **0.655; 0.961** |
| limited joint's overshoot beyond +-0.5 rad, largest (<= 10 mrad) | 2.35 mrad | 1.71 | **1.99** | **1.61** |
| the gap during the first sweep (ticks 30-90): servo; limited | 0.504; 0.543 mm | 0.503; 0.541 | 0.503; 0.541 | 0.503; 0.541 |
| the hinge axes' largest misalignment: servo; limited (reported) | 9.0; 8.3 mrad | 10.3; 9.4 | 10.5; 10.6 | 10.7; 11.4 |
| the servo's largest lag behind its target (reported) | 0.28 rad | 0.246 | 0.244 | 0.242 |

Decision point 1's table now: **Box3D 18 of 18, D 18 of 18, F 23 of 23, V4 24 of 24** (every earlier row unchanged). The
gap is mostly Box3D's soft joint sagging under the cantilevered links (about 0.5 mm in every program: load over the joint's
effective mass times (2 pi 60 Hz)^2), the rest the limit slams and the impacts; the 1 mm bar is about twice the sag, and F's
0.994 mm (tick 645, a sweep through the fallen boxes) is close to it, as Box3D's 0.94 is.

**Gate 5 with the arm: passed** (`gate.py --gates 5 --write-ref`, then gates 1 to 5 again). The arm for 1,200 ticks with sleep
on and off, F and V4, on the RTX 3060 (0x988f0000) and the UHD 630 (0x194859): **every tick's four hashes identical to the
twin's, saturations 0; 16 of 16 runs per GPU and dialect** (the 14 earlier configs and the two arm ones). The twins (F, V4, D;
1 and 8 threads; MSVC and clang-cl) identical to the six new reference files; the 42 existing files were rewritten byte for
byte by `--write-ref` (git sees no change). Run hashes (1,200 ticks): arm D 2561c12af76eb57f, F 3fc6a3ad9d2dbcd5, V4
0947f309d63f756a; arm with sleep off D 485ddf924b425db5, F 9784a8e41798d326, V4 1ec87cfc450c1f67. Gate 2's arm (600 ticks):
D c4d948574443a8bc, F eeeeb544ac37d298, V4 820498f21046b7d8, equal on both compilers and thread counts; every earlier gate 2
run hash is step 5a's. The arm's GPU cost (1,200 ticks, wall / kernels per tick): RTX 3060 F 0.79 / 0.33 ms, V4 1.02 / 0.59;
UHD 630 F 2.15 / 1.46, V4 3.58 / 2.95 (up to 114 dispatches a tick).

**Gate 1** (25 helpers): both GPUs every word identical in F and V4, both compilers equal. Battery hashes **F
f7006e1c8f347ba5, V4 b8cc94dd982a448d, D e0865a1f66f355ab**; the 23 earlier helpers' lines reproduce step 5a's hashes (F
5add7392dc064a30, V4 c45f361e4c33722e, D 17fdc03440fd6596) when hashed alone. New: `twistAngle` (unit quaternions, pure twists
of either sign of s, s = 0, negated, z = 0; and unnormalised pairs) within 5.4e-5 rad of 2 atan2 in every dialect (lpAtan2's
polynomial, doubled); `invertSym` (B^T B + lambda I, lambda from 2^-9 to 0.5 and one in sixteen at 1e-6, scaled into V4's
range): relative to the largest entry F 4.7e-4 (the near-singular ones), V4 3.6e-5 (3,997; 99 near Q9.22's limit left out),
D 2.5e-12; V4 counts 74 saturations (near-singular pivots), the same on both GPUs.

**Findings.**
- **Gauss-Seidel order matters for a chain.** With the joints given the lowest free colour, the base joint (static base, link
  1) was solved before the link joint, and a limit slam on the link joint reached link 1 after the base joint's rows: the
  servo hinge's gap peaked at 1.27 mm against Box3D's 0.62. Box3D colours a joint with a static body from the top
  (`b3AssignJointColor`: "higher priority than dyn-dyn constraints"), so it is solved last; the toy now does the same for
  joints (a joint with a body solved as static, static or asleep, takes the highest free colour, never 0), and the gap is
  0.654 mm. Contacts keep the lowest-colour rule (their reference hashes stand). The arm's colour count reads 64 (the base
  joint sits in colour 63; the empty colours cost no dispatch).
- **A stack's face-on contacts are ties from tick 1.** The arm first hit the stack in its first sweep, and D and F (or V4)
  then differed by 4 to 28 mrad in the joint angles, F and V4 agreeing with each other to 0.01 mrad, D perturbed by 1e-9 m
  agreeing with D to 1e-9: the cubes rest face on face with every clipped point at the same separation, exactly in D and
  within rounding in F and V4, so the reduction keeps different points from tick 1 (the perturbation moves every body alike
  and keeps the ties), the stacks settle a millimetre apart, and the arm meets different boxes. Offsetting and turning the
  cubes did not help (still parallel faces); the stack is now reached on the return sweep, after the 2 s over which the
  angles are compared, and the impacts count in the gap and overshoot rows.
- **V4 mass mantissas:** a 0.2 m cube at 1000 kg/m^3 has inverse mass 0.12499999999999997, whose mantissa at the exponent
  `exp_for` picks rounds to 2^31 (a range error, saturated). `quant.c` now takes one exponent less in that case; no earlier
  scene had one (their range errors were 0), so nothing else moved.

**Judgement calls and deviations from Box3D.**
- The point and axis blocks' masses are computed once per step at prepare (the brief's "inverted once per step"); Box3D
  recomputes both in every solve from the current anchors and axes. The Jacobians (anchors rotated by the delta rotations,
  the perpendicular axes from the current relative rotation) are current. Over the first 2 s Box3D's angles stay within 0.58
  mrad of D's on the servo joint and 2.3 mrad on the free link (its swings amplify), the gaps within 5% in the sweep.
- The servo's speed is set in `prepareJoints` from the angle at the start of the step, the rule b3ref2 applies between steps
  (Box3D's motor speed cannot change inside a step); the target is a command, so a game's input reaches it the same way.
- No spring, no fixed-rotation test, no joint breaking (Box3D's force thresholds): not in the brief.
- Joints filter their bodies' pairs in the broadphase (sorted keys, binary search); with no joints the broadphase is step 5's.
- The arm's stack placement (above), and 1,200 ticks for its gap and overshoot rows (20 s: four sweeps through the boxes).
- `twistAngle` and `invertSym` live in `num.slang` (the battery reaches them); the rest of the joint in `joint.slang`.

**Open issues.**
- The 1 mm gap bar sits at about twice Box3D's soft-joint sag for this arm; heavier outboard links would fail it in Box3D
  too. A stiffer joint (Box3D's `constraintHertz`) or a per-joint bar may be the better test.
- V4's point block supports a condition near 512 (Q9.22): a long light link with a far anchor would saturate (counted). The
  inverse's precision (2^-22 at values near 1) is ample for the arm; a per-joint exponent would remove the limit.
- `invertSym`'s F accuracy degrades as cond x 6e-8 (near-singular blocks); Box3D's adjugate form has the same property.
- Not yet on the Spark (the gcc and clang twins, the GB10 and llvmpipe: `gate.py --compilers gcc,clang` against the committed
  references, now 48 files).
- Box3D's servo here is per step, like the toy's; a per-substep servo (inside the solve) would track better and is a small
  change in `solveJoint`.

## 2026-10-04: step 7 (grid:K timing)

**What landed:** `grid.py` 187 (new: runs grid:K, reads the cost lines, writes `results/grid.md`), `scene.c`'s `grid:K` (K
copies of pile200's pit and bodies, the same draws, on a square grid 6.5 m apart on one floor), `toy.c` 2,797 (a `cost` line
per twin run and per GPU: per tick over ticks 2 to N, tick 1 warming the drivers up, the wall, the kernels by entry point, the
CPU stages, and on the GPUs the uploads before the dispatches and the readbacks after them, from timestamps; the twin times
each dispatch). No hash changed (gates 1 to 5 again, below).

```
python lab/gpu/toy/grid.py --ks 1,4,16,64 --ticks 300      # about 9 minutes; logs in build/toy-scratch/grid, table in results/grid.md
```

**The runs:** K = 1, 4, 16 and 64 (64 fits: 13,057 bodies, 56,000 pairs, the per-pair buffers at 131,072), sleep off so
every body is solved every tick, 300 ticks (the drop and the settled pile), F and V4, the MSVC twin at 1 and 8 threads, the
RTX 3060 and the UHD 630. **Every GPU run is bit-exact with the twin at every K** (each run compares every tick's hashes).
The dispatches hardly change with K (223 to 251 a tick: 17 to 20 colours, one dispatch per colour per stage and substep),
so K scales the work per dispatch, not the dispatch count.

Wall per tick, ms (the GPUs' kernels in brackets):

| K: bodies, pairs | twin 1 thread F / V4 | twin 8 threads F / V4 | RTX 3060 F / V4 | UHD 630 F / V4 |
|---|---|---|---|---|
| 1: 205, 903 | 4.08 / 4.77 | 2.60 / 2.91 | 2.36 (1.42) / 3.93 (2.93) | 10.6 (8.0) / 19.3 (17.3) |
| 4: 817, 3,540 | 15.5 / 18.6 | 4.84 / 5.78 | 3.22 (1.59) / 5.01 (3.33) | 21.7 (12.4) / 25.2 (19.5) |
| 16: 3,265, 14,100 | 63.6 / 78.6 | 15.5 / 18.8 | 7.40 (2.61) / 9.35 (4.41) | 52.1 (21.7) / 53.9 (31.8) |
| 64: 13,057, 56,300 | 275 / 335 | 70.2 / 81.6 | 32.0 (8.26) / 30.9 (9.30) | 165 (57.8) / 182 (90.9) |

At K = 64 by stage (ms per tick: narrowphase, prepare, solve, body kernels, hashes; the solve's cost per colour dispatch):
RTX 3060 F 1.15, 0.85, 5.94, 0.19, 0.14 (24 us per colour dispatch, 5.7 at K = 1); V4 1.75, 0.96, 6.30, 0.17, 0.12 (27 us);
UHD F 10.4, 4.1, 40.6, 0.97, 1.7 (165 us); V4 17.6, 13.8, 56.2, 1.7, 1.6 (243 us); twin at 8 threads F 10.6, 4.8, 35.3, 1.1,
2.1. Where the GPUs' wall goes at K = 64: RTX 3060 F 32.0 = kernels 8.3 + CPU stages 15.5 + readbacks 1.5 + uploads 0.6 + the
rest (two submits and waits, host copies) 6.2; UHD F 165 = 57.8 + 14.9 + **66** + 0.6 + 26.

**What it answers.**
- **Dispatch overhead dominates at pile200's size.** The kernels' time less the work (the work per body is the slope between
  K = 16 and 64, where the machines are busy) is the fixed cost: at K = 1 it is 92% of the RTX 3060's kernel time in F (6.0 us
  per dispatch with its barrier, about 230 dispatches), 97% in V4 (13 us), 91% and 93% on the UHD (34 and 78 us). At K = 64 it
  is 9% (F) and 30% (V4) on the 3060, 17% and 13% on the UHD. The work per body: 0.58 us (F) and 0.50 (V4) on the 3060, 3.7
  and 6.0 on the UHD, 4.2 and 5.1 on the twin at 8 threads, 20 and 25 at 1 thread.
- **The tick scales sublinearly on the GPUs until they fill:** 64 times the bodies cost the 3060's kernels 5.8 times (F) and
  3.2 times (V4); the twin at one thread scales linearly (67 and 70 times), at 8 threads 27 and 28 times.
- **V4 against F:** per body V4 costs the same as F on the 3060 (0.50 against 0.58 us) but twice per dispatch (longer
  dependent chains through the int64 `mas`), so it is 1.7 times F at K = 1 and level at K = 64 (30.9 against 32.0 ms wall); on
  the UHD V4 is 1.6 times F per body.
- **At scale the kernels stop being the tick.** The CPU stages (broadphase, islands, colouring, lists: integers, one thread)
  grow from 0.21 ms at K = 1 to 15 ms at K = 64, half the 3060's wall there. On the UHD the readback of the stages' inputs
  costs 1.2 ms at K = 1 and 66 ms at K = 64: step 5 reads the sleep counters and last tick's point counts back through one
  4-byte copy region per body and per pair (about 70,000 regions at K = 64), which the Intel driver executes one by one (the
  3060: 1.5 ms).
- **GPU against the twin:** at K = 1 the 8-thread twin (2.60 ms, F) is as fast as the 3060's wall (2.36); at K = 64 the 3060 is
  2.2 times the 8-thread twin in F and 2.6 times in V4; the UHD is slower than the 8-thread twin at every K.

**Judgement calls.**
- 300 ticks (the drop, ticks 1 to about 100, and the settled pile) with sleep off; costs averaged over ticks 2 to 300.
- The copies share one floor (overlapping floors would double the floor contacts) and one seed (the same draws per pit).
- The fixed cost from the slope between the two largest K rather than a least-squares fit of kernels = a dispatches + b bodies:
  the dispatch count barely moves with K, so that fit is ill-conditioned (it gave the 1-thread twin a negative cost per
  dispatch). The twin at one thread has no barrier and its fixed cost is within the noise (it scales slightly superlinearly,
  caches); at 8 threads its pool barrier costs about 1.5 ms a tick at K = 1 (60% of its kernels).
- Uploads and readbacks from timestamps at the start and end of each submit (around the copies), so they are GPU-side times;
  the rest of the wall is the two submits' fences, the host's copies and the driver.

**Open issues (the next cost work, in order of what it would buy).**
- The stages' inputs by one copy region per element: a gather kernel (or one copy of the state buffer and of the manifolds'
  point counts) would remove the UHD's 66 ms at K = 64 and most of its 1.2 ms at K = 1.
- The CPU stages are single-threaded and 15 ms at 13,000 bodies; the broadphase's sort-and-sweep along x, the islands and the
  colouring could move to the GPU (sort and sweep, union-find by pointer jumping) or run on the twin's pool.
- At pile200's size the GPU tick is about 90% fixed cost: fewer dispatches (several colours or stages per dispatch, a
  persistent solver kernel) would cut the 3060's 1.4 ms of kernels towards its 0.1 ms of work.
- Not run on the Spark or the Mac (DESIGN's step 7 also names them): `grid.py --compiler gcc` there after `gate.py`.

## 2026-10-04: the reviews applied (gates that prove what they claim, determinism hazards, the acceptance table as written)

Two reviews (Fable; GPT-5.6 Sol through Codex) found the GPU and twin results credible for the state that is hashed, but:
the acceptance table had been relaxed after the results were seen, some written state was hashed by nothing, and the
gates could pass vacuously. This entry applies their fourteen items. **Baseline first** (HEAD 3f27319, after steps 6
and 7): `lab.py gen`, both builds, `gate.py` gates 1 to 5 **PASS** (battery F f7006e1c8f347ba5, V4 b8cc94dd982a448d, D
e0865a1f66f355ab; corpus D ec5972a1a82de840, F 2ea78ea1cf0bdb32, V4 222bdd55590f1dba; 16 of 16 runs per GPU and dialect,
the 48 reference files identical on both compilers).

```
python lab/gpu/lab.py gen && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                                  # gates 1 to 5 (about 10 minutes); expects the RTX 3060 and the UHD 630 here
python lab/gpu/toy/gate.py --gates 5 --write-ref            # after a change meant to move hashes: two compilers must agree first
python lab/gpu/lab.py build --compiler clang-cl --ubsan     # into build/clang-cl-ubsan
python lab/gpu/toy/gate.py --compilers clang-cl-ubsan --gates 1,2,3,4 --no-gpu   # the twins under UBSan (about 15 minutes)
python lab/gpu/toy/accept.py                                # decision point 1's table, both headlines: results/decision1.md
```

**A. Gates that prove what they claim.**
1. **Every word a tick writes is hashed.** Two new categories, each an element hash per element summed order-free like
   the others, computed in the dispatches that already hash (no new dispatch): **bodyprep** (category 4, per body: the
   whole `BodyMass` with prepareBodies' world inertia `invIw`, and the whole `Aabb`, pads included; written by
   `hashElements` to binding 22) and **pairwork** (category 5, per pair, seeded by the pair's bodies: the whole `SatAxis`
   and `Constraint`, and the manifold's point pads, the one manifold word `hashManifolds` left out; written by
   `hashManifolds` to binding 23; an inactive pair's slot holds what an earlier tick wrote there, the same on every
   machine, and is hashed again). The joints' hash already covered every field. Not hashed, by design: the saturation
   counters (compared at the end of every run, as before) and `narrowDiags` (written only by the corpus, gate 3). The
   tick record, `run_hash`, the first-difference report, the tick table (two columns appended, so gate 2's parsing
   stands), the final line and the reference files carry the new categories; a reference file now opens with a
   `columns` line and an older file is refused by name. The trace's views include the two new hash buffers (so
   `--dispatch-hashes` values changed: they hash every view). **Mandatory full-buffer comparison in gate 5:** `toy
   --trace` (every buffer the kernels write, word by word, after every tick's prepare and after its end) over pile200
   seed 1 for 120 ticks and the arm for 300 (past its first contact with the stack near tick 245), on every GPU in F and
   V4, failing on any differing word: **no difference** on the RTX 3060 and the UHD 630 in either dialect (about 25 s in
   all). Cost of the wider hashes: on the UHD 630 `hashManifolds` went from 0.11 to 0.36 ms a tick (pile200 seed 1, F;
   V4 0.06 to 0.21), on the 3060 from 4 to 8 us; the kernels' totals moved within their noise.
2. **No vacuous pass.** `gate.py --expect-gpus` takes vendor names or a count; the default is `nvidia,intel` on this
   laptop (by host name, `EXPECTED_GPUS` in gate.py) and `1` elsewhere. Gate 1 (F and V4 batteries, every compiler) and
   gate 5 (every F and V4 run on the first compiler, and every trace) fail unless the expected GPUs ran. `--no-gpu` runs
   the twins alone (no GPU run, none expected; the UBSan gate uses it). `battery` now fails with "no GPU ran" when F or V4
   ran on no GPU unless given `--no-gpu` (D has no GPU kernels and says so). Also fixed: battery's `--gpus` used `atoi`,
   so `--gpus 0xff` meant 0 GPUs and a PASS; it takes hex now (strtol). Checked: `--gpus 0x1` (NVIDIA only) fails gate 1
   ("no intel GPU ran"), `battery_F --gpus 0` exits 1.
3. **Writing the reference files is separate from checking them.** `--write-ref` refuses with fewer than two compilers;
   it writes the first compiler's files into a scratch directory (`<out>/ref-new`), checks every other compiler's twins
   (1 and 8 threads) against them, and copies all 48 into `results/ref` only when every config of every compiler
   agreed; one disagreement writes nothing. A normal run only checks. The files were rewritten this way (MSVC
   19.42.34435 wrote, clang-cl 23.1.2 agreed on all 48), and a full gate run then checked them.

**B. Determinism hazards.**
4. **C++20 for every twin** (the lab's `CMAKE_CXX_STANDARD 20`, required: E11's and the toy's): a right shift of a
   negative value is arithmetic there and a left shift of one is defined. Built and run alone first (the old kernels,
   with items 6 and 12): **every hash unchanged** (battery, gate 2's run hashes, the corpus, gate 5's run hashes and all
   48 reference files, both compilers, both GPUs); E11's `lab.py run --machine win-quick --quick` (MSVC, then `--twins
   clang-cl twins`): every twin hash "ok" against `reference.json`, the threads agree, the controls differ; the GPU
   mismatches are E11's known ones (Fdisc, Fnocap, I64 on Intel). `results/win-quick` deleted. C code is not covered by
   C++20 (C17 and C23 leave a negative right shift to the implementation): the one in the C drivers, battery's input
   generator (`v->a >> 24`), is now `asr32`, a defined floor shift (the battery's input hashes are unchanged).
5. **UBSan:** `lab.py build --ubsan` (clang or clang-cl; CMake `LAB_UBSAN`) builds the toy's twins and C drivers
   (toy, battery, narrow, toy_common, b3ref2; not Box3D or vk_util) with `-fsanitize=undefined -fno-sanitize-recover=all`
   into `build/<compiler>-ubsan`; clang-cl links the runtime from the clang resource directory. `LAB_UBSAN_RECOVER`
   (CMake only) reports and continues, to list every report in one run. Gates 1 to 4 under it, twins only: **one finding**,
   V4's `invertSym` on the battery's near-singular matrices: `rp2 + M( m32, r.yz )` and `rp1 + sum2( ... )` overflowed
   int32 when a reciprocal pivot had already saturated at 2^31 - 1 (undefined in C++, a wrap on a GPU). Fixed with
   `addSat` / `subSat` (V4: `mas( a, 1, b, +-1, 0, 0, 0 )`, saturated and counted; F and D: plain + and -) for every sum and
   difference in `invertSym` that may hold a saturated operand. Only the overflowing vectors change (invertSym's
   battery hash and its saturations, 74 to 107, the same on both GPUs); no scene saturates, so no tick hash moved. The
   negation of INT_MIN the reviewers suspected (`x < 0 ? -x : x`) never ran: `mas` saturates at +-(2^31 - 1), so an
   INT_MIN can only come from an add that overflowed, which UBSan now watches; the C drivers, scenes, stages and the
   corpus raised nothing. After the fix, gates 1 to 4 under UBSan: **PASS**, no report (the battery, corpus and gate 2
   hashes equal MSVC's and clang-cl's; gate 4's 111 runs at 1 and 8 threads, Box3D's b3ref2 included).
6. **The sentinel's flushing modes** (`fpflags.h`): every twin (toy, battery, corpus) fails at start when the main thread
   flushes (x64 MXCSR FTZ, bit 15, or DAZ, bit 6; aarch64 FPCR FZ, bit 24, or FEAT_AFP's FIZ, bit 0), and `fp_flags`
   reports a flushing mode as `flush-mode`, so every dispatch checks the mode of every pool thread that ran it. On
   aarch64 (gcc or clang inline asm) the sentinel now reads FPSR itself as well as the C99 flags (IOC, DZC, OFC, UFC and
   IDC, cleared with the rest). **What an aarch64 sentinel can and cannot see with FZ off:** invalid, divide by zero,
   overflow, and underflow when a result is tiny and inexact (UFC); not a subnormal input (IDC is set only when an input
   is flushed, which FZ off never does) and not an exact subnormal result. x64 sees subnormal inputs too (MXCSR DE).
   The twins compute the same bits everywhere, so the x64 legs see what an aarch64 run would hide. Not yet built on
   aarch64 (the Spark).
7. **`posDeltaWord` rounds halves away from zero**, as `mas`: +128 at bit 7 for a difference >= 0, +127 for a negative
   one, so a - b is exactly -(b - a). V4 only: the battery's posDelta helper, the corpus and 13 of V4's 16 reference
   configs moved (below); stack10 (sleep on and off) and bounce did not.
8. **The NoContraction audit counts by result id** (`gen_toy.py`'s `audit`, `spirv-dis --raw-id`): every F `OpFAdd`,
   `OpFSub` and `OpFMul` result id must carry an `OpDecorate ... NoContraction`, or the generation fails naming the ids;
   F also fails on any `OpFDiv`, `OpFRem`, `OpFMod`, `OpDot`, vector or matrix product, or extended instruction but
   FindUMsb; V4 (and its battery) fails on any float add, subtract or multiply at all. Today: every F op decorated
   (battery 649, narrowClip 2,132, prepareContacts 1,130, ...; more decorations than ops, on other instructions), V4
   none.

**C. The acceptance table, honestly** (`accept.py`, `results/decision1.md`). Every rule DESIGN.md wrote is a judged row
in its own words; a judgement chosen after the results were seen is a separate row in **post hoc**, and the summary
counts the passes both ways. The rows: the 120-tick position rms against D on **every scene** (stack10, bounce, ramp,
chip, ratio, the arm, pile200 pooled over its seeds), and pile200's penetration on the **original window** (the solve
line's 60 ticks before every body sleeps); post hoc: chip, ratio and pile200 judged by their outcome rows in place of
their rms (each such row passes when every outcome row of its scene passes: chip's no tunnelling, sleeping and V4's
saturations; ratio's two classes; pile200's sleep, median and escapes), and the penetration after every body sleeps in
place of the original window. Gate 4 (below) prints the same counts.

**Decision point 1, the headline** (MSVC, one thread; the same at 1 and 8 threads in gate 4, and under UBSan):

| | Box3D | D | F | V4 |
|---|---|---|---|---|
| under DESIGN.md's rules as written | **17 of 18** | **17 of 18** | **24 of 27** | **24 of 28** |
| with the post-hoc rows (in place of the rows they relax) | 18 of 18 | 18 of 18 | 26 of 27 | 27 of 28 |
| the rows they fail as written | penetration, original window (2.04 cm, seed 3) | the same (3.21 cm, seed 2) | rms: chip 4.05e-5, arm 1.6e-3, pile200 0.077 m | rms: chip 1.5e-4, ratio 1.43e-4, arm 1.6e-3, pile200 0.105 m |

(Step 6's "Box3D 18 of 18, D 18 of 18, F 23 of 23, V4 24 of 24" counted the relaxed rules only, and no rms on the chip,
ratio, pile200 or the arm.) F's and V4's original-window penetration passes (F 1.91 cm, seed 7; V4 now 1.71 cm, seed 2:
item 7 moved V4's piles; it was 3.11 cm, seed 8). The arm's rms fails in both headlines: no post-hoc rule covers it.

- **chip (not chaotic: arithmetic).** D moved 1e-9 m moves 4.9e-8 m, but the yardstick moves positions only. Where the
  error comes from (per tick against D, every tick recorded): by the impact at tick 3 (12 m/s into the ground, 40 rad/s
  of spin) F carries its free fall's rounding, 2.0e-7 m and 2.5e-6 m/s (float positions near 0.3 m, the velocity's
  half-ulp per substep; V4 4.8e-8 m and 6.4e-7 m/s, its quantised h g). The impact takes the spin from 40 to 11.65
  rad/s through the friction impulse, and leaves F 1.7e-4 rad/s off D's spin (6e-6 of the change) and V4 6.9e-4 (2.4e-5);
  D with only its start velocity changed by 3e-6 m/s comes out 5e-5 off, so most of F's spin error, and nearly all of
  V4's, is the contact solve's own arithmetic at the impact (which of V4's formats, not taken apart). The chip then
  tumbles on its edge (contacts near ticks 7 and 18) and the spin error grows a hundredfold or more (F 3.1e-3 rad/s at
  tick 7, 5.0e-2 at tick 18; D with the velocity change 3.6e-3 at tick 18); it lands 1.9e-5 m (F) off, slides to rest
  at a velocity 1.5e-4 m/s off for 20 ticks, and rests 3.8e-5 m (F) and 1.5e-4 m (V4) from D's
  chip from tick 40 on, which is the rms. D with the start velocity changed by 3e-6 m/s alone: rms 4.3e-6 m. So: the
  impact scene amplifies velocity and spin errors about a hundredfold, and F's and V4's arithmetic at the impact feeds
  it. Not a solver bug: F against D differs by 6e-6 of the impact's change in spin, and the scene multiplies that.
- **ratio (not chaotic: arithmetic, V4 only).** D moved 1e-9, 1e-8, 1e-7 and 1e-6 m: rms 6.9e-7, 6.9e-7, 7.0e-7 and
  1.2e-6 m (the outcome never flips; step 5a's "a start difference of 1e-10 m picks the outcome" does not hold for these
  starts). F: 2.1e-7. V4's 1.43e-4 is born at tick 1 in the two chips solved last under the plate (bodies 5 and 6): after
  one tick their velocities are 6.9e-4 m/s off D's (F's 1e-7; chips 3 and 4 2e-5 and 7e-6), one chip moves 1 mm
  differently by tick 7, and the classes stay the same. Under a 2,000:1 mass ratio each chip's velocity is the small
  difference of the ground's and the plate's impulses (about 1 N s a substep each on 0.05 kg); which V4 format loses
  the bits is not taken apart.
- **arm (as written: fails in F and V4 alike).** Its links agree with D (rms 8.4e-8 m in F, 1.7e-5 in V4); its stack of
  four cubes, not reached before tick 245, parts from D's at tick 1 (6.8e-6 m on body 4, 4e-3 m on the top cube by tick
  25, F and V4 with the same numbers) on step 6's face-on ties: every clipped point of a face-on contact at the same
  separation, exactly in D, within rounding in F and V4, so the reduction keeps other points. D moved 1e-9 m moves every
  body alike, keeps the ties and stays within 1e-9 m: the yardstick cannot see it.
- **pile200 (chaotic).** D against itself moved 1e-9 m: 0.0165 m pooled (0.031 worst seed); F 0.077, V4 0.105.
- **penetration.** Original window (as written, judged): Box3D 2.04 cm (seed 3), D 3.21 (seed 2), F 1.91 (seed 7), V4
  1.71 (seed 2): Box3D and D fail. After every body sleeps (post hoc): Box3D 1.13, D 1.23, F 1.20, V4 1.46 cm.

**D. Simplicity.**
12. **One pool, one FNV** (`lab/gpu/src/pool.h`, `hash.h`, header-only): E11's harness and the toy's driver use
    `LabPool` (`lab_pool_start`, `lab_pool_run( fn, ctx )`, `lab_pool_stop`, `lab_pool_groups`); the toy keeps its
    per-thread floating-point flags beside it. `lab_fnv` (bytes) and `lab_fnv_words` (one step per word, the toy's
    buffer hashes) replace the copies in harness.c, rows.c, toy.c, battery.c, narrow.c and stages.c. No hash moved
    (item 4's run).
13. **Gate 4 is accept.py's runs** (`accept.gate`): every job of decision point 1 (111 runs: the twins and Box3D on every
    scene and seed, D moved 1e-9 m) on the first compiler's twins at **1 and 8 threads**, two runs at a time; it fails when
    a run exits badly or its log shows threads that disagree, a tripped sentinel or a saturation (`accept.health`), and
    prints both headlines and one line per scene and program (`metrics.summary`); it does not judge the table or write
    decision1.md (accept.py does, at one thread). **traj.py's per-scene analysers are folded into metrics.py**, each
    quantity computed once: stack (`spinmax1` added), pile, bounce (`impact_tick`, `vmin`), ramp (both boxes'
    accelerations by the least-squares parabola, `fit_accel`, moved here; `slide_end`), ratio (sinks, drifts), chip
    (`rest`), with `metrics.summary( scene, m )` for the one-line text; traj.py is the LPTRAJ1 reader, the shared
    quaternion helpers and the per-body CLI (`--scene` prints metrics' summary).
14. **`--pos-out` / `--pos-ref` are gone.** Gate 2 writes the first compiler's trajectories (`--traj`, every tick) and
    compares F and V4 with D through `metrics.rms` (positions: rms, max and where; orientations: rms and max, the same
    formula), over ticks 1-120, 121-600 and 1-600; the numbers are pos-ref's (chip F 4.05e-5, ramp F 4.9e-6, ...). The
    free-fall line went with `--pos-out` (with contacts it no longer measured free fall). The trajectories are deleted
    after each scene (pile200's are 14 MB).

**Every hash that changed, and why** (gates 1 to 5 PASS after; the per-category columns of the old files compared
tick by tick with the new):

| what | before | after | why |
|---|---|---|---|
| battery V4 | b8cc94dd982a448d | **081ff06910407f15** | posDelta 4767e08423e48a02 to 465c5d99d9b34de6 (item 7); invertSym 47bdaa241d9ec4da to 0d887c03aa7692fc, saturations 74 to 107 (item 5); the other 23 helpers unchanged |
| battery F, D | f7006e1c8f347ba5, e0865a1f66f355ab | unchanged | |
| battery V4, `--n 65536 --seed 7` | (not recorded at HEAD) | f8915578a21dbb3b | lpDivQ31 equal to C's `/` on 0 of 1,063,016 differing |
| corpus V4 | 222bdd55590f1dba | **c5d33b47944fc009** | posDelta in the narrowphase (item 7); 0 differ outside ties, both passes, both GPUs every word identical |
| corpus D, F | ec5972a1a82de840, 2ea78ea1cf0bdb32 | unchanged | |
| every run hash (gate 2, gate 5, the 48 reference files) | | new | the run hash covers the two new categories (item 1) |
| the old categories (bodies, manifolds, stages, joints) in the F and D reference files | | unchanged, every kept tick of all 32 | |
| the same in V4's | | unchanged in stack10, stack10 sleep 0 and bounce; changed in ratio (from tick 1), arm and arm sleep 0 (tick 2, bodies and joints: the joint's `deltaCenter`), chip (9), pile200 seeds 1 to 8 (46 to 47, manifolds), ramp (180) | item 7 (`addSat` / `subSat` change a value only on an overflow, which would count a saturation; no scene saturates) |
| `--dispatch-hashes` values | | new | they hash every view, now with the two new hash buffers |

Gate 2's run hashes (600 ticks; equal on MSVC, clang-cl and clang-cl under UBSan, 1 and 8 threads):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | 93e8a1b30e072c68 | 75fe3f09ef74fce1 | 99d818cb28a9505f |
| pile200 | 7977d22cb4b18c9f | 66ad7e5a215d6141 | 80016df1e5c89e4c |
| pile200, gravity 0, body 112 pushed | 63c7697a4ca9f492 | 452bd6380e46ee6f | 1802ea607b976402 |
| chip | 35cac36b6d7121e2 | 22308826aec57c83 | 19bd156226d3b1f4 |
| ramp | 4c94be0151e3537f | cc7d156d0e35c9f7 | 251330e7e0643146 |
| bounce | b1d611cc0f697635 | 5a472a3b03339b86 | bd0624b07e8cefaf |
| ratio | 67b7e59404194816 | c2bc875f4967a9ee | d0b69b314f19d00d |
| arm | 1a2a8b149d83d013 | 04bd8e67c8fa161a | ff9394053976a8db |

Gate 5's reference run hashes (1,200 ticks for the arm and pile200, 3,600 for stack10, 600 for the rest), D / F / V4:
stack10 036b2bfa93e1b928 / 15684d87ff1d68b1 / 3896097ca8de26ff; stack10 sleep 0 916691cd4af93abc / 36ba76c96aab06c4 /
6f69c34bd79e4213; arm d0aa4985d2d6f1a3 / 3aac45a38810f555 / ee7c37b7e15e2ef3; arm sleep 0 49ff930e829b5f61 /
2c318aad1b3f5233 / 45cfc897272f746e; the rest in the files. **Decision point 2 after all this: 16 of 16 runs per GPU and
dialect** (RTX 3060 0x988f0000, UHD 630 0x194859), every tick's hashes (five categories, six with joints) identical to
the twin's, saturations 0, and
the four full-buffer traces clean. (The reference reader's handling of an older file was tidied after that run; rebuilt,
gate 5 on the twins alone, `--no-gpu`, both compilers: all 48 configs identical, PASS.)

**What the reviews missed (found here).**
- **The 1e-9 m yardstick is blind twice.** It moves every body alike along x and y, so it keeps a stack's ties (the arm's
  stack parts from D's at tick 1 in F and V4 while D moved stays within 1e-9 m), and it moves positions only, while an
  impact amplifies velocity and spin errors (the chip: 1e-9 m moves 5e-8 m, 3e-6 m/s moves 4.3e-6 m). "Not chaotic" by it
  does not mean insensitive; a velocity perturbation of the dialects' own size would be the better yardstick.
- **The arm fails DESIGN's rms rule as written** (1.6e-3 m in F and V4: the stack's ties, the links agree), and its
  stack had already been moved after step 6's first results so that the impacts fall outside the 2 s angle window: a
  post-hoc change of the scene rather than of a predicate. Its rows are kept as they are (no new predicate).
- **Step 5a's ratio note is wrong** for these starts: D moved by up to 1e-6 m never flips the outcome; V4's 1.43e-4 is its
  own arithmetic at tick 1 (above).
- **C code is outside C++20's fix**: battery's input generator right-shifted a negative int32 (fixed, item 4).
- **battery's `--gpus` mask was parsed by `atoi`**: any hex mask meant no GPU and a pass (fixed, item 2).
- **Gate 3's GPU comparison is still "desired, not required"**: a machine without GPUs passes it. Not changed (outside
  the brief); `--expect-gpus` could cover it the same way.
- **The manifold hash skipped the point pads** (now in pairwork), and a reference file had no record of its columns (the
  reader matched lines by count; now a `columns` line, and an older file is refused).

**Open issues.**
- The Spark: the aarch64 sentinel (FPSR, FZ/FIZ) is unbuilt there, and gcc's and clang's twins are not yet checked against
  the rewritten references (`gate.py --compilers gcc,clang --gates 5`; `--expect-gpus` defaults to 1 there: `nvidia,llvmpipe`
  states the GB10 and llvmpipe); a clang UBSan build there too (`lab.py build --compiler clang --ubsan`).
- V4's ratio miss (which format loses the bits under the 2,000:1 ratio) and the chip's impact precision are diagnosed to the
  tick, not to the operation.
- A yardstick for the rms rule that perturbs velocities (or a per-scene bar) is the lead's call; no predicate was changed.
