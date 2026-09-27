# Architecture

```
app/sandbox  (C++20, sokol D3D11 + Dear ImGui)   tools, camera, record/replay, renderer, screenshots
scenes/      (C17)  procedural low-poly kit: walls, houses, trees, fences, tower, pile, lumber; scripted bombardment
src/         (C17)  lpf: the destruction core               include/lpf/lpf.h is the whole public API
extern/box3d (C17)  physics, pinned (extern/box3d/PATCHES.md)
```

## The core in one paragraph

Every destructible object is a Box3D body whose shapes are convex hulls, one per **piece**. Pieces that share a
face are joined by **bonds**. An **impact** (tool, blast, detonating flask, or a hard collision found through Box3D
hit events) refractures the pieces it reaches into convex **cells** and damages bonds near it. A flood fill over
the surviving bonds splits the body; components without an anchored piece become dynamic **debris** bodies. A
**weight check** breaks bonds that carry more load than they can hold, so undermined structures come down over the
following steps. New fragments are sorted into cheap **debris tiers** by volume (below), resting debris freezes
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
  **radial** (wedges and concentric chords around the hit, for glass). Every cell is convex because it is an
  intersection of half-spaces, so it is directly a Box3D hull: no convex decomposition at runtime.
- Sliver absorption: tiny cells outside the damage radius have their sites dropped and the cells recomputed, which
  keeps the tiling exact.
- Keeper merging (`lpMergeCells`): cells that stay on the piece merge while the convex hull of the pair is within
  the material's `mergeSlack` of their true volume and covers no ejecta centroid. Fewer, chunkier pieces for the
  same look: a shot log leaves two ends of about 40 triangles each.
- Voronoi neighbours are visited nearest first and the search stops once the next site is farther than twice the
  cell radius.

## Pieces and bodies (`world.c`, `world.h`)

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
- Weight check: BFS depth from anchors, then loads flow from the top down, split over each piece's bonds toward the
  ground by area. A bond over `area * loadStrength * (health / strength)` breaks. This is a vertical-load model: it
  catches crushed or cut-away supports, not overturning moments.
- Invariants are checked every tick in tests by `lpWorld_Validate` (every piece on one live body with one shape,
  bonds only within a body, counts consistent).

## Rendering (`facet.c`, `app/sandbox/renderer.cpp`)

- The core builds flat-shaded triangle meshes per piece (`lpWorld_BuildPieceMesh`): authored faces get their colour
  jittered per plane (so coplanar faces of neighbouring cells match and intact walls stay seamless); cut faces get
  the material's solid interior colour evaluated at the face centroid (wood rings along the grain, stone speckle).
- The renderer keeps all piece vertices in shared pages; each vertex carries its piece index. Two storage buffers
  map piece to body and body to transform, so moving a piece or a body never re-uploads vertices. The whole world
  draws in one call per 131k-vertex page, plus a shadow pass, instanced particles and a nearest-filtered blit
  (render scale < 1 gives a chunky retro look). Particles are one instanced cube scaled per kind: long splinters,
  flat leaves, thin glass shards that flash in the sun.

## Extension points already in the API

- `lpDetonatorDef` on an object: explodes when hit at `triggerSpeed` or caught in a blast (flasks, volatile cargo,
  chain reactions).
- `lpWorld_Pull`: spring pull on a piece with max acceleration and max liftable mass (grab tool, winch, crane hook).
- `lpMaterialDef` table (`lpSetMaterial`): density, strengths, fragment size, pattern, grain stretch, interior colour,
  tier thresholds, plate size, cells per fracture, particle kind, merge slack.
- `lpWorldDef`: fragment scale (main performance knob), debris scale (tier thresholds), per-tier caps, fracture
  jobs per step, weight check scale, worker count, debug log (`LPF_DEBUG=1` in the sandbox).
- `lpWorld_Blow` (cone push) and `lpWorld_PromoteBody` (full physics for a thrown or launched piece).

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

See the "Next steps" section of [feasibility.md](feasibility.md).
