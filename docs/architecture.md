# Architecture

```
app/sandbox  (C++20, sokol D3D11 + Dear ImGui)   tools, camera, record/replay, renderer, screenshots
scenes/      (C17)  procedural low-poly kit: walls, houses, trees, fences, tower, pile, lumber, ruins, yard; scripted bombardment
src/         (C17)  lpf: the destruction core               include/lpf/lpf.h is the whole public API
extern/box3d (C17)  physics, pinned (extern/box3d/PATCHES.md)
```

## The core in one paragraph

Every destructible object is a Box3D body whose shapes are convex hulls, one per **piece**. Pieces that share a
face are joined by **bonds**. An **impact** (tool, blast, detonating flask, or a hard collision found through Box3D
hit events) refractures the pieces it reaches into convex **cells** and damages bonds near it. A flood fill over
the surviving bonds splits the body; components without an anchored piece become dynamic **debris** bodies. A
**stress solve** finds the force in every bond under the structure's own weight and breaks the joints that cannot
hold it, so undermined structures crack, hinge and topple where they are weak. Objects can be joined by **links**
(Box3D welds, hinges, ball joints, ropes) that break under load or blasts and outlive the pieces they hang on. New fragments are sorted into cheap **debris tiers** by volume (below), resting debris freezes
into static **rubble** that wakes when something knocks it, and budgets move the oldest, smallest bodies down the
tiers instead of popping them.

## Debris tiers (`debris.c`)

"Cooking the books": the fracture is real, but most fragments get only as much simulation as they need to look right.

| tier | while flying | at rest | cost |
|---|---|---|---|
| puff | particles by material (dust, chip, splinter, leaf, glint) | gone | none in the core |
| ghost | no Box3D body: ballistic in the core, passes through everything | lands via ray cast as render-only **scrap** | one ray per 8 ticks |
| light | Box3D body that collides with static geometry only, pushes nothing | freezes fast into static rubble that collides with nothing | no contacts at rest |
| full | full physics | freezes into **fragile** rubble: static and solid, woken by any hit or shove | zero while static |

- Thresholds are per material (`particleVolume`, `ghostVolume`, `lightVolume`) times `lpWorldDef.debrisScale`.
  Ejecta (cells whose centroid is inside the break radius) are classified in the parallel phase, and puffs and
  ghosts skip Box3D hulls, shapes and bonds entirely.
- Collision filters are set once at shape creation: STATIC collides with everything, FULL with STATIC, FULL,
  VEHICLE and CHARACTER, LIGHT with static bodies only: full rubble keeps its FULL category once frozen, so a custom
  filter lets a moving light piece touch structures and rubble but never moving full debris.
- Ghost flight plans cast the chord of the next 8 ticks against static bodies (structures and rubble), capped per
  step. Loose ghosts and scrap live in a hashed 2 m grid for blasts, shoves and the blower.
- Rest and promotion: fast heavy full bodies shove light rubble and scrap aside one way (styrofoam) and wake full
  rubble so Box3D resolves the collision; hit events wake rubble above `wakeSpeed`; `lpWorld_Pull`,
  `lpWorld_PromoteBody` and blasts give pieces full physics back; `lpWorld_Blow` pushes rubble, scrap and ghosts
  (the sandbox's leaf blower).
- Budgets form a ladder: full over its cap becomes light, light becomes ghost, ghosts end as particles, rubble over
  its cap becomes scrap, and scrap over its cap sinks into the ground. Fracture jobs per step are capped too;
  overflow is deferred to the next step, nearest first.

## Geometry (`poly.c`, `fracture.c`)

- `lpPoly`: a convex polyhedron (planes + vertex loops, fixed capacity, no allocation). Faces carry a material and a
  tag: negative for authored surface, otherwise the id of the cut that made it.
- `lpPoly_Clip`: keep one side of a plane. Degenerate cases (vertices on the plane) are removed by pushing the plane
  outward by a sub-tolerance amount; intersection vertices are keyed by sorted edge so both faces sharing an edge
  get bit-identical points. Cap faces are chained from the cut edges.
- Patterns: **impact Voronoi** (sites dense near the hit, a ring at the damage radius, a few far sites for big
  plates), **grain Voronoi** (the same in space squashed along the grain, then unsquashed: long convex splinters;
  distances to the hit are still measured in real space, and an anchor site on each side keeps a log end whole),
  **radial** (wedges and concentric chords around the hit, for glass), **masonry** (brick: along the mortar of a
  course grid shared by the whole wall; bricks near the hit shatter, further ones come loose, the rest is cut into a
  plate above, a plate below and a run on each side of the hole per course, so holes are stair-stepped and the new
  bonds are mortar), and a **snap** (one tilted cut across an overloaded beam). Every cell is convex because it is an
  intersection of half-spaces, so it is directly a Box3D hull: no convex decomposition at runtime.
- Sliver absorption: tiny cells outside the damage radius have their sites dropped and the cells recomputed, which
  keeps the tiling exact.
- Keeper merging (`lpMergeCells`): cells that stay on the piece merge while the convex hull of the pair is within
  the material's `mergeSlack` of their true volume and covers no ejecta centroid. Fewer, chunkier pieces for the
  same look: a shot log leaves two ends of about 40 triangles each. Before any quickhull, an exact lower bound on
  the hull volume (the solid spanned by the other cell's most distant vertex) rejects pairs that cannot fit.
- Ejecta chipping (`lpChipCell`): ghost-sized ejecta are split by the material's `chipSplits` random planes into
  real chips (along the grain for wood, across the pane for glass), so a blast leaves a dirty spray of real
  fragments that land as scrap. Only chips smaller than about 1.5 cm become particles; a sliver left on the piece
  still turns to dust.
- Voronoi neighbours are visited nearest first and the search stops once the next site is farther than twice the
  cell radius.

## Pieces and bodies (`world.c`, `impact.c`, `split.c`, `step.c`, `world.h`)

- Piece geometry stays in its original **object frame** for its whole life. A split-off body is created at its
  parent's transform, so moving pieces between bodies never transforms geometry: no drift, cached Box3D hulls are
  reused, and procedural interior colours line up across cuts.
- Fracture of one impact runs in three phases: choose pieces and snapshot inputs (sequential), compute cells +
  hulls + sibling bonds (parallel, `tasks.c`), integrate in job order (sequential). Output never depends on the
  thread count.
- Sibling bonds of Voronoi cells come from face tags (a lookup, no polygon clipping). Bonds to former neighbours
  use 2D polygon overlap of coplanar opposing faces; parts that meet at an angle (roof on gable) are welded if they
  touch within 3 cm.
- Damage model: an impact of energy E and radius R delivers `E (1 - d/R)^2 / (pi R^2)` J/m^2 at distance d. A piece
  refractures above its material's `fractureEnergy`; a bond loses that much health and breaks at zero. Strengths
  are calibrated so a rifle chips brick and a grenade opens it (see the comment on the material table).
- Joints: every part meets its neighbours with a joint (`lpJointId`: mortar for masonry and plaster, nails for wood,
  dry, solid). A bond between parts takes the weaker joint; bonds between the cells of one broken piece are solid but
  start with the damage the impact did at their location, so cracks near a hit barely hold.

## Stress (`stress.c`)

- Quasi-static solve per structure: pieces are rigid nodes (6 degrees of freedom, anchored pieces fixed), bonds are
  short beams through their contact patch with axial, shear, bending and twist stiffness from its area and extents.
  Stiffness is normalized (only ratios share the load), so bond forces come out in newtons.
- K x = gravity by conjugate gradient with a block-Jacobi preconditioner (each piece's 6x6 block, Cholesky), warm
  started from the last solution. A solve continues across steps (residual and search direction live on the pieces)
  until the structure's topology stamp changes; a per-step work budget in bond-iterations slices big solves.
- Every structure updated in a step is checked together, after the splits (`lpCheckStructures`), in three phases:
  1. in queue order, the checks that need no solve finish (a creaking structure only adds strain; unchanged loads end
     the check), and each remaining structure reserves its share of the budget before anything is built: at most
     `maxStressStructureWork` bond-iterations for itself and `maxStressWork` for all. One that does not fit waits,
     having cost nothing, and goes first next step;
  2. the reserved structures build and solve in parallel, each job with its own scratch, writing only its own pieces;
  3. in queue order, each solution is judged: strain, breaks, slender pieces.
  The per-structure cap bounds the step's stress time on enough cores, the total bounds the CPU. Results do not
  depend on the worker count.
- Each bond's tension side (axial plus bending), compression side and shear (Coulomb: cohesion plus friction times
  compression) against its joint's limits, scaled by `stressScale` and the bond's health, give a utilization. Over
  1, strain accumulates each converged check (`strainRate`); at strain 1 the bond breaks, everything at twice its
  limit at once and otherwise the worst few per check, so failure cascades and a structure creaks before it gives.
- Nothing is judged on an unconverged solution. Dry and mortar joints carry little or no tension, so an overhang's
  moment opens the tension side, the part above loses its anchor, and Box3D topples it.
- New structures get one check when created; afterwards only topology changes (impacts, breaks) queue a solve.
- Fracture keeps only chunks on a structure: kept cells smaller than light debris fall, which keeps both the physics
  and the solve cheap (no crumbs hanging on tiny bonds).
- Invariants are checked every tick in tests by `lpWorld_Validate` (every piece on one live body with one shape,
  bonds only within a body, counts consistent).

## Links (`link.c`)

- A link joins two objects (or an object and a fixed point) with a Box3D weld, revolute, spherical or distance joint.
  Each end is a piece plus a frame in its body's frame. Body frames never change for a piece, so an end stays valid
  through splits, tier changes and fracture; only the Box3D joint is rebuilt when an end's body changes.
- Invariant (validated): a live link's ends are live pieces of live Box3D bodies, on two different bodies, and its
  joint is on exactly those bodies. It is kept in three ways:
  - an end whose piece leaves Box3D (freed, made a ghost or scrap) breaks the link at once;
  - a fractured piece hands each end to the kept cell holding its anchor, or the link breaks (the cell was blown
    out);
  - `lpSyncLinks`, just before the physics step (after every split and new body of the step), rebuilds joints whose
    ends changed body. A rebuilt link between two moving bodies tears if one is a chip (under 2% of the other's mass).
- Loads: `lpPollLinks`, just after the physics step, reads each awake link's constraint force and torque (Box3D's
  joint events miss joints in the overflow constraint colour and sleeping ones). Utilization, clamped to 3 and
  smoothed, strains the link above 1 (10% over lasts about a second) and breaks it at twice the limit or strain 1.
  A rebuilt joint starts cold, so it settles for 3 steps before it is judged. Nothing after the physics step touches
  a joint: state queries and the hash read cached fields.
- Blasts damage links like bonds, at the anchor or anywhere along a rope; `lpWorld_CastRay` hits ropes as thin
  capsules. Health scales the load a link can take.
- Structures carry what hangs on them: `lpSampleLoads` adds each link's force at its anchor, and a link whose pull
  changed by a quarter re-checks its structures (at most every 30 steps).
- Linked bodies, and bodies resting on moving linked ones (a crate in a cart), never freeze into rubble and are
  outside the debris budgets: frozen, they would hold an assembly rigid or jam it. `maxLinks` caps the count.
- Gravity scale ("fairy dust") lives on the body and passes to every body made from it (splits, ejecta, ghosts).

## Rendering (`facet.c`, `app/sandbox/renderer.cpp`)

- The core builds flat-shaded triangle meshes per piece (`lpWorld_BuildPieceMesh`): authored faces get their colour
  jittered per plane (so coplanar faces of neighbouring cells match and intact walls stay seamless); cut faces get
  the material's solid interior colour evaluated at the face centroid (wood rings along the grain, stone speckle).
- The renderer keeps all piece vertices in shared pages; each vertex carries its piece index. Two storage buffers
  map piece to body and body to transform, so moving a piece or a body never re-uploads vertices. The whole world
  draws in one call per 131k-vertex page, plus a shadow pass, instanced particles and a nearest-filtered blit
  (render scale < 1 gives a chunky retro look). Particles are one instanced cube scaled per kind: long splinters,
  flat leaves, thin glass shards that flash in the sun. The sandbox draws ropes the same way, as short knotted
  segments that sag when slack.

## Extension points already in the API

- `lpDetonatorDef` on an object: explodes when hit at `triggerSpeed` or caught in a blast (flasks, volatile cargo,
  chain reactions).
- `lpWorld_Pull`: spring pull on a piece with max acceleration and max liftable mass (grab tool, winch, crane hook).
- `lpMaterialDef` table (`lpSetMaterial`): density, strengths, fragment size, pattern, grain stretch, interior colour,
  tier thresholds, plate size, cells per fracture, particle kind, merge slack.
- `lpWorldDef`: fragment scale (main performance knob), debris scale (tier thresholds), per-tier caps, fracture
  jobs per step, stress scale and budgets, worker count, debug log (`LPF_DEBUG=1` in the sandbox).
- `lpWorld_Blow` (cone push) and `lpWorld_PromoteBody` (full physics for a thrown or launched piece).
- `lpCreateLink` (weld, hinge, ball, rope) with `lpWorld_GetLinkState` (force, utilization, strain, health) and
  `lpWorld_SetRopeLength` (winches, cranes); `lpObjectDef.gravityScale` and `lpWorld_SetGravityScale`.

## Borrowed from Nebenan

[Holz231/Nebenan](https://github.com/Holz231/Nebenan) (MIT) is an independent project with the same basic idea on
Box3D. It was read for ideas; no files were copied. Ideas taken from it, credited in code where they appear:

- pushing a clip plane outward to eliminate degenerate cases (`lpPoly_Clip`)
- visiting Voronoi neighbours nearest first with the "twice the cell radius" stop
- making every cell a pure function of the job so fracture can run on any thread count deterministically
- freezing settled debris into static rubble that wakes on nearby events
- the three-part site distribution (dense near the hit, a ring at the rim, far plates)

Not taken: its renderer, stress solver, scheduler, hull builder (we use Box3D's quickhull, cached per piece).

## Where to go next

See [roadmap.md](roadmap.md); open performance work is the backlog in [perf-log.md](perf-log.md).
