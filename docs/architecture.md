# Architecture

```
app/sandbox  (C++20, sokol D3D11 + Dear ImGui)   tools, camera, record/replay, renderer, screenshots
scenes/      (C17)  procedural low-poly kit: walls, houses, trees, fences, tower, pile, lumber, ruins, yard, keep, track, mech yard;
                    the car, crane and hexapod kits; scripted bombardment and drivers
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
tiers instead of popping them. Every input is a tick-stamped **command** applied as its step begins, and a **state
hash** kept per element covers everything, so machines in lockstep can check each other every tick and name what
differs.

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
  step. Loose ghosts and scrap live in a hashed 2 m grid for blasts and shoves.
- Rest and promotion: fast heavy full bodies shove light rubble and scrap aside one way (styrofoam) and wake full
  rubble so Box3D resolves the collision; hit events wake rubble above `wakeSpeed`; `lpWorld_Pull`,
  `lpWorld_PromoteBody` and blasts give pieces full physics back.
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
  touch within 3 cm. A new object's parts are paired by a sweep along x and bonded in the order of a double loop over
  its parts, so bond indices do not depend on the sweep.
- Damage model: an impact of energy E and radius R delivers `E (1 - d/R)^2 / (pi R^2)` J/m^2 at distance d. A piece
  refractures above its material's `fractureEnergy`; a bond loses that much health and breaks at zero. Strengths
  are calibrated so a rifle chips brick and a grenade opens it (see the comment on the material table).
- Joints: every part meets its neighbours with a joint (`lpJointId`: mortar for masonry and plaster, nails for wood,
  bolts for sheet metal, dry, mounts, weld, solid). A bond between parts takes the weaker joint; bonds between the cells
  of one broken piece are solid but start with the damage the impact did at their location, so cracks near a hit
  barely hold. Sheet metal cannot crack that way (a fracture ejects everything within 1.5 fragment sizes of the hit):
  a mech's femur is two halves welded together instead, and a blow under the metal's fracture energy cracks the weld.
- Identity: every piece keeps the object's `userId`, its part index and the part's system (`lpPartSystem`: a tag the
  core never reads, the channels it carries, sources and needs, and its share of its object's sources by volume).
  Fracture cells inherit all of it (the share split by volume); splits move pieces as they are.
- Detonators belong to parts (`w->detonators`, shared by every piece made from the part; an object's detonator is
  shared by all its parts without one of their own). A hit at trigger speed, or a blast of more than 150 J/m^2 at a
  piece, sets its detonator off: it disarms (a tank torn in two goes off once) and its pieces on that body go up with
  the blast at their centre next step. When they are the whole body it goes as before, the blast at its centre of
  mass; otherwise only they go and the rest is split. Chips thrown clear by a fracture are no longer volatile.
- Supply (`supply.c`): a bond between two pieces that carry a channel carries it, and so does a link that carries it
  (`lpLinkDef.carries`, a fuel hose) between two pieces that do. The carriers a channel connects form a group; its
  supply is the sum of its sources' feeds, capped at 1, where a source feeds its share times the worst supply among
  what it needs. Needs are only on lower channels than sources, so the channels are settled in order, 0 to 7, in one
  pass. It is recomputed once a step, after `lpSyncLinks`, and only when a carrier's connections changed (a bond or
  link between carriers made or broken, a carrier made or freed); only carriers are walked. Wheels drive as well as
  the worst of their `driveNeeds` is fed at their mount and steer at that rate for `steerNeeds` (unfed, they hold).
- The supply wave (`supplyHopsPerTick`, 16; milestone 10): what an update finds reaches each carrier
  `(d + 2 r) / H` steps later, where d is its distance (over carriers sharing a channel, and links carrying one) from
  the nearest place carriers' connections changed (`lpCarriersChanged` records them) and r how far that connected
  set's other change sites lie from its first; no site is further than d + 2 r, so nothing arrives before its cause.
  Until then the carrier keeps what it had; the change waits in `w->supplyWaves` (hashed with the world), applied at
  the start of the step it is due, in (tick, channel, piece) order. A wave already on its way with the same value keeps
  its arrival, and one a later update undoes is dropped. A pool's leak opens when the wave reaches the pool's source.
  A new object's supply comes at once (its own bonds change nothing that was there), and so does a change no site
  reaches. `TestSupplyCone`: the drop of a cut 60-box line runs exactly H boxes a step.
- Pools (`lpPartSystem.pool`, `seal`): a source part's fluid (hydraulics, fuel, blood), shared by every piece made from
  it (`w->pools`; cells kept on the body inherit it, chips thrown clear do not). Each supply update measures the
  carrier volume the pool's lowest channel reaches; when that drops, a leak opens that drains the share lost per second
  and closes over the seal time (0: it bleeds on). Pools drain before the supply update each step (`lpDrainPools`) and
  ask for one only when a level crosses a sixteenth or runs dry. A source feeds fully down to 30% of its pool, then in
  proportion (`lpSourceFeed`). A fracture that keeps every cell loses nothing.
- Crashes: a collision's energy is reduced by the more crushable material's `crush` (sheet metal 0.7, rubber 0.6:
  crumple zones soak up most of it until dents come), and a crushable hit is centred on the mean of the contact's
  manifold points instead of its first corner. Materials without crush hit as before.
- The car kit (`lpAddCar` in `scenes.c`): a sheet-metal floor pan carrying fuel, power and steering; an engine of
  three blocks (feeds power, needs fuel), a fuel tank (feeds fuel, a detonator), a steering box (feeds steering, needs
  power); hood, boot, doors, pillars and roof bolted on, glass, rubber bumpers; rear drive needs power, front steering
  needs steering. About 1.7 t in 19 pieces.

## Stress (`stress.c`, `solve.c`)

- Quasi-static solve per structure: pieces are rigid nodes (6 degrees of freedom, anchored pieces fixed), bonds are
  short beams through their contact patch with axial, shear, bending and twist stiffness from its area and extents.
  Stiffness is normalized (only ratios share the load), so bond forces come out in newtons.
- K x = gravity by conjugate gradient with a block-Jacobi preconditioner (each piece's 6x6 block, Cholesky), warm
  started from the last solution. `solve.c` holds the system and its math with no world in it (so it runs in parallel
  jobs and tests can build systems by hand); `stress.c` builds systems from structures and judges them. A structure
  keeps its system on its body: a solve continues across steps on it without a rebuild until the structure's topology
  stamp changes; a per-step work budget in bond-iterations slices big solves.
- Every structure updated in a step is checked together, after the splits (`lpCheckStructures`), in three phases:
  1. in queue order, the checks that need no solve finish (a creaking structure only adds strain; joints a blast
     weakened are judged again from the forces of the last solve, kept on the bonds; unchanged loads end the check),
     and each remaining structure reserves its share of the budget before anything is built: the step's
     `maxStressWork` shared by the structures wanting to solve, but at least `maxStressStructureWork` each (one
     solving alone may use all of it; a step continuing on its system is charged its iterations only, others two fine
     passes more for the build). One that does not fit waits, having cost nothing, and goes first next step;
  2. the reserved structures build (unless continuing on their system) and solve in parallel, each writing only its
     own pieces, bonds and system; a converged one also computes every joint's utilization, the force and moment it
     carries (kept on the bond), and its slender pieces' worst sections;
  3. in queue order, each solution is judged: strain, breaks, slender pieces.
  The per-structure cap bounds the step's stress time on enough cores, the total bounds the CPU. Results do not
  depend on the worker count.
- Each bond's tension side (axial plus bending), compression side and shear (Coulomb: cohesion plus friction times
  compression) against its joint's limits, scaled by `stressScale` and the bond's health, give a utilization. Over
  1, strain accumulates each converged check (`strainRate`); at strain 1 the bond breaks, everything at twice its
  limit at once and otherwise the worst few per check, so failure cascades and a structure creaks before it gives.
- Nothing is judged on an unconverged solution. Converged means the whole residual is within 0.1% of the whole load
  **and** every node balances to 5% of its own load plus the forces through it: a global norm alone let the residual
  gather on small fracture cells, whose joints then read absurd utilizations. After `stressPatience` steps without a
  judgement (restarts count too) both relax, 10 and 5 times. Dry and mortar joints carry little or no tension, so an
  overhang's moment opens the tension side, the part above loses its anchor, and Box3D topples it.
- A request for a new check (`lpRequestStressCheck`: a hit, a link's pull) samples the loads again: a solve in progress
  restarts with them from where it was, a settled structure solves again only if they changed. Kept fracture cells
  start from their parent's solution moved rigidly. Each piece carries change stamps (`changed`, `accepted`): what
  changed since its structure's last judgement is a seed of the next solve.
- New structures are **settled at load**: `lpWorld_SettleStructures` (called by `lpBuildScene`) solves every waiting
  structure to convergence with no per-step budget (up to `maxSettleIterations`), judges it, and repeats a few rounds
  while joints break, so a scene's first step solves nothing. Afterwards only topology changes (impacts, breaks),
  changed loads and weakened joints queue a check.
- **Rigid clusters and the delta form** (the load-bearing skeleton, milestone 4): pieces of a structure can move as
  rigid clusters (`lpPiece.cluster`). Its reduced system (`lpSystemReduce`) keeps each edge between two groups with
  its arms moved to the groups' centres of mass and drops the edges inside one: exactly `PᵀKP`, so the same kernels,
  preconditioner and conjugate gradient solve it. It is solved for the **correction** to the last solution, not the
  whole state: one pass over the fine edges gives `Pᵀ(f − K x_old)`, the reduced solve finds y from zero, and each
  member moves by its group's motion on top of x_old. Bonds inside a cluster keep their last forces, and the
  clusters bias only the redistribution. Forces are evaluated on the fine edges. `TestReducedAssembly` and
  `TestSolveSystem` check the algebra.
- **The clusters in play** (structures past `stressLargeNodes`, 512 pieces):
  - *Forming:* only after an exact solve, by union-find along bonds in edge order, at most 128 pieces within 12 mean
    piece sizes. Pieces never clustered: both ends of any joint loaded past `stressGlue` (0.3), and slender pieces
    loaded past it.
  - *Seeds:* at every check, the pieces changed since the last judgement (bond set, bond health, load, from their
    change stamps), and everything within 3 bonds of them, leave their clusters. A rigid cluster at the rim of a
    hole would carry the arching load stiffly and wrongly.
  - *What a correction solves:* the right-hand side subtracts the residual each piece's last judged solve was
    accepted with (`lpPiece.stressResidual`), so it is exactly zero where nothing changed and the correction stays
    local. A correction is written back to the pieces only once it has converged: a partial one moves clusters
    rigidly out of balance, and a restart would chase that everywhere.
  - *The residual meter:* on a converged correction it reads, per cluster, the load the correction brings each member
    through its bonds (`K (x − x_old)`). A cluster dissolves, and the solve runs again (at most twice), if that could
    carry a member's joints past 0.5, or if the cluster's load changed by more than a quarter of what its most loaded
    member carries (then it is carrying a redistribution stiffly and biasing the fine joints around it). If most
    clusters must go, the change is not local and they all go at once.
  - *Rounds:* at most two; if the meter still objects on the last one, every cluster goes and it is solved exactly.
  - *Provisional and audits:* a judgement on a reduced system is provisional. Provisional structures queue for an
    audit (`lpWorld.audits`), an exact solve on at most half the budget, one at a time, once the budget has been at
    most half used with nothing waiting for 30 steps, or once the oldest has waited 300 steps. An audit gives way to
    a change (it stays first in line), is judged as usual (a collapse a provisional judgement missed comes a beat
    late), and forms the clusters again. A hit on a provisional structure brings its audit to the front.
  - *Invariants:* any joint loaded past the glue share has both pieces unclustered (`lpWorld_Validate`).
  - *The oracle:* tests set `lpWorld.stressOracle` to check every judged correction against an exact fine solve of
    the same change, with the worst joint error and the number of strain decisions that differ.
    `TestKeepLocalHit`, `TestKeepBreach`, `TestKeepHole` and `TestDriftSmallStructures` hold these to bounds.
- **The speed of propagation** (`stressHopsPerTick`, 16; milestone 10): past settling, a structure's solve runs on a
  **region**, not the whole structure, and the region grows at most that many bonds a step.
  - *The region* (`lpStressFront`, phase 1): the region of the solve in progress, then everything within H bonds of
    the pieces changed since the last judgement (at a restart), then everything within H of the region when it must
    grow. Fixed pieces carry nothing across; a changed fixed piece seeds what it holds. On a reduced system a hop is a
    step from group to group (a cluster comes in whole), so a clustered structure's cone is H groups a step.
  - *The solve* (`lpSystemSolveFront`): conjugate gradient on the region's nodes and the edges with an end in it, the
    nodes just past it held where they were. A region that holds the whole system runs the plain solver, step for
    step the same. A region that takes in new pieces restarts the solve from where the solution is.
  - *Quiet:* a converged region is judged only if what its change puts on the held nodes just past it is within their
    equilibrium tolerance (`lpStressQuiet`; on the fine system counted from the residual their last judgement
    accepted). Otherwise it grows next step. Quiet, it is the whole structure's solve within tolerance.
  - *What it writes:* solutions, utilizations, strains and slender sections inside the region; residuals inside it and
    on the held nodes just past it. Nothing else changes, so after d steps a change has reached at most H d bonds
    (plus that boundary layer): `TestStressCone` checks the bound on a 60-block bridge, and that it is tight.
  - *Creaking is per joint:* while a region solves, every joint over its limit outside the region strains on each step
    (with breaks and slender pieces, as a settled structure's creaking does), so a change far away does not pause it
    (`TestCreakDuringFarSolve`); inside the region joints wait for the judgement, and a judgement strains every
    overloaded joint again.
  - *Audits* are region solves seeded by the pieces a reduced judgement left unaudited (`lpPiece.unaudited`).
  - *The drift guard:* a load is compared both with the last sample and with the load the last judgement solved for
    (`lpPiece.acceptedLoad`), so a load that creeps by less than the threshold at each look still seeds the region
    when the structure solves again (`TestStressDriftGuard`).
  - *Measured* (spike S1 and milestone 10's F1, perf-log): the keep's breach, hole and local hit decide on the same
    steps with the same breaks; the oracle sees no more error. A removed support needs almost the whole structure
    before the boundary is quiet: the quasi-static answer is global, and the speed only spreads it over steps.
- `lpWorld_HashStress` hashes the solver's state (solutions, loads, utilizations, strains, solves in progress); a
  refactor of the solver must keep it equal (`tools/bench.ps1 -StrictSolver`), not only the simulation hash.
- Fracture keeps only chunks on a structure: kept cells smaller than light debris fall, which keeps both the physics
  and the solve cheap (no crumbs hanging on tiny bonds).
- Invariants are checked every tick in tests by `lpWorld_Validate` (every piece on one live body with one shape,
  bonds only within a body, counts consistent).

### Stress on moving bodies (inertia relief)

- An object made with `solveStress` (the car kit; the hexapod's torso and leg segments) keeps a stress solve while it
  moves. It is checked only when asked:
  - a hard hit (at most every 10 steps). It jolts the moving bodies its links join to it too (a foot landing loads the
    femur through the knee): they are checked with it, and for 10 steps their links' loads are checked as they build,
    so a landing is judged at its peak, a step or two after the contact;
  - a wheel bottoming out hard;
  - a link's load changing by a quarter, at most every 30 steps per body whatever pulls on it (a walker's torso holds
    six legs whose loads swing every stride); a link held back keeps asking.
- Its solve is always exact (no clusters, no audits) and pinned at one piece: the one nearest where it was struck in
  the last step (the crash's force enters there; the struck piece itself may be broken by then), else the one nearest
  its centre of mass. Each piece carries its weight and its sampled loads (contacts, links, wheels) less what it takes
  to accelerate it with the body, m (a + alpha x r + omega x (omega x r)), and less its own small inertia times alpha.
- The acceleration is measured, the body's velocity change over the last step (`lpTrackMovingBodies` keeps the last
  two), because a crash's contact is gone by the time the body is checked: the impact that the hit made broke it.
  Whatever force was not sampled then enters at the pin. Without two consecutive velocities it is computed instead, as
  the acceleration that balances the sampled loads exactly. A rigid body stops in a step where a crumple zone takes
  several: the part of a crash's deceleration beyond gravity is scaled by 1 - the struck material's `crush`.
- It is judged once per check: a moment's load strains and breaks joints but does not creak on. What breaks splits as
  usual, and split-off bodies keep `solveStress`.

## Speeds of propagation and cones (milestone 10)

What one input can change in a step is bounded, so repair, rollback and zones can be bounded too:
- Between bodies, cause travels with matter: contacts, links and the ground under wheels and feet (the causal units,
  `units.c`).
- Within a structure, stress reaches `stressHopsPerTick` bonds further a step (the region solve, above).
- Along carriers, supply reaches `supplyHopsPerTick` carriers further a step (the supply wave, `supply.c`).
- Queries reach a bounded distance: an impact or a blast at most `maxImpactRadius` from its point, a command's ray
  `maxRayRange`. A command's cone is its ray's path, then the impact's radius, then the speeds above.
- Not bounded yet (they couple units unseen, measured by E3's cones): the world-wide budgets (fracture jobs, the
  stress share, debris ranks, freezes, free lists) and casts that only read what they hit. Milestones 12, 16 and 18
  take them on.

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
- Motors (`lpMotorDef` on hinges and ball joints: muscles, a crane's slew and luff): a velocity servo on Box3D's own
  joint motor. `lpDriveMotors`, after `lpSyncLinks` and the supply update, sets the speed from the error to the target
  (`lpWorld_SetLinkTarget`, `lpWorld_SetLinkTargetRotation`; a ball joint's error is a rotation vector) and the torque
  cap to `maxTorque` times health times how well its `needs` are fed at either end, plus `holdTorque` for the unfed
  share (a brake). It holds up to the cap and sags past it; that is the limp. Setters are called only when a value
  changed (and after a rebuild); a changed target or cap wakes the joint's bodies. The motor's own torque never counts
  against the link's `maxTorque`. A local patch fixes Box3D's revolute torque getter, which counted the axial torque
  twice (`extern/box3d/PATCHES.md`).
- A hinge servo adds a feedforward speed (`lpLink.feed`, set by rigs from the pose a step ahead) to its gain times the
  error. Jam (`lpMotorDef.jam`, opt-in): damage at a joint slows its top speed by jam times the damage and makes it
  stick, holding unfed with the larger of `holdTorque` and jam times `maxTorque`; a piece at its end breaking up knocks
  a quarter off its health. `lpLinkDef.tearRatio` sets when a rebuilt link tears off a chip (0 keeps 2%; the mech's
  leg hinges tear off a stub of a segment, 10%).

## Vehicles (`wheel.c`)

- A vehicle is a body on wheels, made by `lpCreateVehicle`. A wheel is a link (`lp_linkWheel`) with no Box3D joint:
  end 0 on the piece at its mount, end 1 on nothing. Everything links do comes free: it follows its piece through
  splits and fractures, takes blast damage (measured from the tyre), strains under its own load, and tears off a mount
  body lighter than a fifth of its share of the chassis (a spring on a chip would launch it). When a wheel's link
  breaks, the wheel comes off as an object of its own (a 12-sided cylinder with the hub's pose, velocity and spin) at
  the start of the next step. A vehicle holds its controls and its wheels' links, never a body: a chassis cut in two
  keeps on each half the wheels mounted there.
- Each step, after `lpSyncLinks` and before the physics step (`lpStepVehicles`):
  - steering turns toward the control at a limited rate;
  - each wheel on an awake chassis casts its tyre (8 rim points in the wheel's plane wrapped in half its width) from
    the mount down the suspension, through its own chassis, against static and full pieces only (light debris is not
    ground); casts per step are capped by `maxWheelCastsPerStep`, the others keep their last contact;
  - the suspension is a spring-damper along the contact normal, damped by the chassis's own motion, so a kerb lifts
    the car instead of jolting it;
  - grip is an impulse solve per chassis body (wheels grouped by body, in wheel order, 4 iterations) on a copy of its
    velocity after gravity and the springs: sideways and along the tyre within a friction circle (grip × the ground's
    friction, less while sliding), the drive a motor toward top speed with a capped force, brakes, the handbrake
    (locks, and halves sideways grip) and rolling resistance capped too, and a one-way stop when the suspension
    bottoms out. The result goes to Box3D as forces (the side force raised toward the centre of mass by `rollFactor`,
    so it rolls the body less), and the opposite onto a moving ground body.
- The load a wheel puts on its mount is exact, so it is judged unsmoothed: a hard landing breaks a wheel in one step.
  Structures carry the wheels standing on them (`lpSampleLoads`), and a wheel whose load changed by a quarter, or that
  moved onto another structure, re-checks them (at most every 30 steps).
- A parked chassis falls asleep (its forces never wake it); a changed control wakes it, and so does losing the ground
  under a wheel, which holds no Box3D contact to do it.
- Controls (`lpWorld_SetVehicleControl`) are persistent simulation state and hashed; so are the wheels' steering,
  suspension and contact state, only when vehicles exist. The sandbox turns keys into control commands when the
  controls change; scripted drivers (`lpSceneDrive`, the track's lap drivers) submit the scene's control commands from
  simulation state every tick, and leave alone a vehicle a player's command drives.

## Rigs (`rig.c`, `gait.c`)

- A rig makes no physics: it steers the servos of motorised hinges that already exist. `lpCreateRig` takes limbs, each
  an ordered chain of 1 to 3 hinge links from the torso outward and a foot point. It never names a body: the torso is
  the body holding the most root links (the lowest index on a tie). Controls (`lpWorld_SetRigControl`: forward, strafe,
  turn, crouch) and limb targets (`lpWorld_SetLimbTarget`) are persistent, hashed state; the state and each limb's
  capability come out (`lpWorld_GetRigState`, `lpWorld_GetLimbState`). `lpStepRigs` runs after the supply update and
  before `lpDriveMotors`, and costs about 12 us a rig awake, 2 asleep.
- Kinematics: a model built from each link's end frames and its measured angle (its sign from which end is proximal),
  the foot in the last segment's body frame. A piece's body frame never changes, so the model survives splits and joint
  rebuilds. IK is damped least squares on the chain as it is (6 iterations, damping 0.05, warm-started, limits less
  0.05 rad, b3 trig only), so stumps and strikes need nothing special. Box3D holds a joint only as stiffly as the lighter
  body's inertia allows: the kit's leg segments pad theirs (`lpObjectDef.inertiaRadius`), or the model is off by a
  quarter of a metre walking.
- Capability per limb, each step: attached, the intact chain, its strength (the weakest link's cap over its maximum,
  holds excluded), its foot (the def's, or after a break the far end of the last segment left, found again only when
  that segment's pieces change: a stump walks as a peg), reach, depth, and able (it can lift its foot, strength 0.2 or
  more, depth 0.6 of the stand height or more).
- The desired pose (`lpWalkRig`): the torso's pose moved one step by the controls, level, at most 5 cm ahead and 0.3 m
  behind, its heading at most 0.3 rad ahead. Its height over the planted feet climbs toward the goal at 0.5 m/s:
  the crouch, lowered by up to 30% as the weakest able leg weakens, no deeper than the shortest leg reaches, no lower
  than the belly. Falling (the torso sinking faster than 1 m/s) it follows the torso down, so a dropped mech is not
  flung back up. The servos get the IK of every foot from the desired pose, and as feedforward only the commanded
  motion (fed the pull toward the torso too, a torso running ahead would push itself on).
- A free gait: a foot is due when its drift along the motion passes 0.6 of the half-step (0.2 when told to stand
  still), or when it is overstretched any way. It lifts if its neighbours around the able ring are planted, if an even
  number of able legs keeps to every other one (a tripod on six), and if the centre of mass stays inside the other
  planted feet by the margin now and where the swing ends; the first that has to wait leans the body toward the
  others. That makes a tripod on six legs, a ripple on five and a wave on four with no replanning. Speeds follow the
  cadence (a half-step of min(0.7 stride, half the rate times the swing time)), and fewer legs are asked for less (five:
  half, four: a third).
- Swing and stance: a swing is aimed again every step from the torso's actual motion; footholds are sphere casts at
  liftoff and at two thirds (`maxFootCastsPerStep`); the arc is a polynomial. A foot that got where it was set down
  (3 cm, or 0.3 s) is held there, fixed in the world, eased toward the model's foot over 0.3 s (loaded joints give a
  centimetre), and held where it is if it slips 0.1 m. Structures underfoot are re-checked like wheels.
- After 30 calm steps the targets freeze (idle), and a sleeping torso with unchanged controls skips the step.
- Damage: fewer than four able legs, or stuck (told to move, making under a tenth of it for 1.5 s: one leg left on a
  side lifts nothing), it drops onto its belly and crawls; legs that cannot stand are tucked up. A weak leg lowers the
  body, a limp one (unfed) is carried, a cut hydraulic line drains the pool until its valves close.
- Reaching (`lpWorld_SetLimbTarget`): a limb leaves the gait once the others keep the centre of mass by the margin
  (crawling, its belly does); until then the body leans. IK drives its foot at the point with a strike feedforward (8/s
  times each joint's error, capped by its servo), so a weak limb hits softer. It reports the piece its foot touches (a
  contact of its tip within 0.35 m of the foot, something loose before something fixed), and a claw command
  (`lp_commandClaw`) welds the claw to it, or lets go; the grip is kept on the limb.
- The hexapod kit (`lpAddHexapod` in `scenes.c`): an armored torso (frame and belly skid, a reactor feeding power, a
  hydraulic reservoir with a pool of 100 and valves closing in 3 s, a computer feeding control, a deck), six sheet-metal
  legs (a hip block, a femur of two welded halves, a tibia with a welded rubber sole), every body `solveStress`; hinges
  capped at 10, 30 and 30 kN*m, needing hydraulics and control, the femur and knee jamming. It walks at 2.3 m/s on a
  0.6 stride and 0.35 s swings. The mech yard (`lp_sceneMech`) has a patrol driver (`lpSceneDrive`) and a bench rung.

## Rendering (`facet.c`, `app/sandbox/renderer.cpp`)

- The core builds flat-shaded triangle meshes per piece (`lpWorld_BuildPieceMesh`): authored faces get their colour
  jittered per plane (so coplanar faces of neighbouring cells match and intact walls stay seamless); cut faces get
  the material's solid interior colour evaluated at the face centroid (wood rings along the grain, stone speckle).
- The renderer keeps all piece vertices in shared pages; each vertex carries its piece index. Two storage buffers
  map piece to body and body to transform, so moving a piece or a body never re-uploads vertices. The whole world
  draws in one call per 131k-vertex page, plus a shadow pass, instanced particles and a nearest-filtered blit
  (render scale < 1 gives a chunky retro look). Particles are one instanced cube scaled per kind: long splinters,
  flat leaves, thin glass shards that flash in the sun. The sandbox draws ropes the same way, as short knotted
  segments that sag when slack. Wheels are no pieces: the renderer keeps a 12-sided tyre mesh per live wheel link in
  128 slots reserved at the start of the piece and body maps, posed each frame from `lpWorld_GetWheelState`.

## Lockstep co-op (`app/net`, outside the core)

- Every machine steps the same world on the same commands (milestone 7's choice: lockstep, the host keeps the clock).
  `lockstep.c` is the protocol over a line transport (`net.c`: TCP, or in memory for tests).
- A peer sends its session's description (`lpSceneDescribeSession`); the host compares it with its own and refuses
  it by the key that differs.
- The host closes tick T once every machine has stepped T - delay, by which time every player's commands for T (stamped
  `delay` ticks ahead, sent as script lines) have arrived, and sends them as one packet. Every machine, the host too,
  steps only on packets; each makes the scene's own commands from its own world. A command is applied as its line
  reads back, on the machine that sent it too, so a field the line does not carry cannot split them; a kind with no
  line is refused when given.
- Peers report their root hash every tick. The first that differs stops the clock; once all have stepped what was
  sent, the host follows the hash down over the wire (categories, buckets, elements) and names the element and the
  first tick.
- Runners: `lpf_bench --host` / `--join` (`bench/pair.c`) and the sandbox's co-op; `TestLockstepPair` in memory. Not
  yet: catch-up for a late joiner, host migration, the snapshot (milestones 14 and 16).
- When a machine leaves or the host stops, every machine ends with the host's own report ("done", "peer 2 left", a
  desync's element and first tick). Faults for tests (`net.h`'s `lpFault`): a link held back by a delay and jitter
  each way, in order, or stalled; never a lost line, since the protocol assumes a connection that delivers or closes.

## Agent-driven co-op tests (`app/sandbox/control.h`, `tools/coop.py`, `test/coop`)

Agents test the multiplayer side of the sandbox the way a person would, through its windows, but on a clock they
hold:
- **The control port.** `sandbox --control 0` listens on a loopback port for one request per line and answers each
  with one JSON line.
  - Requests press keys and the left button through the same handler as a person's input, and point the camera
    (which aims the tools and V).
  - They also step the clock or take turns, take screenshots (the scene, or the window with its panel), and read the
    state.
  - Real input is ignored, so typing elsewhere cannot change a test, and `--background` windows never take focus.
- **The clock is the agent's.** Held, a sandbox steps only what `step` (the host's clock, so every machine) or `turn`
  allow.
  - `turn PEER N` is a barrier: once every player has asked, the host steps the fewest ticks asked for. Several agents
    can each play one player, each seeing only its own window.
  - Held keys are read once per tick, and under control the camera and particles move per tick too. The same
    requests in the same order give the same session, hash for hash.
- **The director.** `tools/coop.py` (Python, standard library only) launches a session, one window per player, and
  sends requests by player. It also takes turns and side-by-side screenshots, slows a joiner's link, and runs the
  scenarios in `test/coop` (claim, turns, desync, latency, leave, late, reproducible). These run locally, since they
  need windows. CI runs the network faults headless with `lpf_bench --net-delay`, `--net-stall` and `--leave-at`.
- **What the scenarios found at once.**
  - Two players pressing V on one car in one tick: the second's window believed it drove the car the first got.
  - Getting out never released a car, so nobody else could take it.
  - A third machine waited for ever after a peer left.

  All three are fixed.

## Extension points already in the API

- `lpDetonatorDef` on an object: explodes when hit at `triggerSpeed` or caught in a blast (flasks, volatile cargo,
  chain reactions).
- `lpWorld_Pull`: spring pull on a piece with max acceleration and max liftable mass (grab tool, winch, crane hook).
- `lpMaterialDef` and `lpJointDef` tables, one copy per world (`lpWorldDef.materials` and `.joints`; NULL for the
  built-ins, `lpDefaultMaterials` and `lpDefaultJoints`; read back with `lpWorld_GetMaterial` and `lpWorld_GetJoint`):
  density, strengths, fragment size, pattern, grain stretch, interior colour, tier thresholds, plate size, cells per
  fracture, particle kind, merge slack, and each material's default joints.
- `lpWorldDef`: fragment scale (main performance knob), debris scale (tier thresholds), per-tier caps, fracture
  jobs per step, stress scale and budgets, worker count, debug log (`LPF_DEBUG=1` in the sandbox).
- `lpWorld_PromoteBody` (full physics for a thrown or launched piece).
- `lpCreateLink` (weld, hinge, ball, rope) with `lpWorld_GetLinkState` (force, utilization, strain, health) and
  `lpWorld_SetRopeLength` (winches, cranes); `lpObjectDef.gravityScale` and `lpWorld_SetGravityScale`.
- Motors on hinges and ball joints (`lpMotorDef`, `lpWorld_SetLinkTarget`, `lpWorld_SetLinkTargetRotation`), and rigs
  on them (`lpCreateRig`, `lpWorld_SetRigControl`, `lpWorld_SetLimbTarget`, the rig and limb states); pools
  (`lpPartSystem.pool`, `lpWorld_GetPiecePool`).
- `lpCreateVehicle` with `lpWorld_SetVehicleControl` and the vehicle and wheel states; `lpPartSystem` (tags,
  channels) with `lpWorld_GetPieceSupply` and `lpPieceInfo.supplied`; part detonators; `lpObjectDef.userId` and
  `solveStress`.

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
