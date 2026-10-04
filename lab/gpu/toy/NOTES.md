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
