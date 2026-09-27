# Roadmap

Principle for every milestone: **buy performance headroom first, then spend it.** A full game means a large map,
several destructible cars, and containers of sloshing volatile reagents, all at once. Every milestone logs its
before/after numbers in [perf-log.md](perf-log.md).

Order (one at a time):
1. Chunky fracture + debris tiers (done; polish and prune pass in progress)
2. Toppling and stress points
3. Breakable joints and assemblies
4. Destructible vehicles
5. Large-map physics zones
6. Networking
7. Art polish

## 1. Chunky fracture + debris tiers

"Cooking the books": the fracture detail is real, but most fragments become cheap objects. A log shot through leaves
cosmetic splinters and two rough ends of a handful of triangles each.

| Tier | Flying | At rest |
|---|---|---|
| Puff | particles (dust, chips, splinters, leaves, glints) | gone |
| Ghost | no physics body; falls like rock, passes through everything | render-only scrap (capped, oldest sink away) |
| Light (≈ baseball to brick) | collides with static geometry only; cannot push anything | light rubble: static, collides with nothing; moving things and tools shove it aside one-way ("styrofoam") |
| Full (≈ torso and up) | full physics | fragile rubble: static and solid, wakes when anything approaches or hits it (strong static friction, not cement) |

Grabbing or launching a piece promotes it to full. Over budget, debris is demoted down the ladder instead of
popping.

## 2. Toppling and stress points

The idea is snapshotted stress that is checked only where things break.

- **Local re-check.** After a break, re-check a neighbourhood only (k bond hops or radius R around broken bonds).
  Skip sleeping and frozen pieces.
- **Moment check at a cut.** For the part above a damaged layer, compare the weight N and the offset e of its
  centre of mass with the support polygon of the bonds crossing the cut. Masonry rule: once e leaves the polygon,
  break the tension-side bonds first. The upper block becomes a dynamic body resting on a hinge edge, and Box3D does
  the toppling.
- **Snapshots, recomputed lazily when topology changes near damage:**
  - a bridge / articulation-point index (Tarjan, O(V+E)); breaking a bond that is not a bridge can never split a
    body, so the flood fill is skipped
  - per-bond supported mass (dominators rooted at the ground), for O(1) load checks
- **Persistence.** A bond must stay overloaded for a few steps before it breaks. This reads as creaking and spreads
  the cost, as in Red Faction: Guerrilla (a layer fails when its strength is less than the weight above it), NVIDIA
  Blast (iterative solver carried across frames) and vox3D (strided analysis).
- **Sudden loads** (landing debris, a truck on a bridge) come from hit-event impulses.
- **Upgrade path:** a Blast-style iterative solver (compression, tension and shear per bond) on a reduced support
  graph.
- **Context:** Dennis Gustafsson has been prototyping "stress analysis" for Tuxedo Labs' new engine (Aug–Sep 2026,
  "one building at 60 fps", algorithm undisclosed; the solver is "inspired by Erin's work").
  https://x.com/voxagonlabs/status/2092992959507570923

## 3. Breakable joints and assemblies

Covers the rickety cart full of volatile crates and backpack contraptions.

- **`lpJointDef`:** weld, hinge, rope or wheel between pieces of two objects, with force and torque break
  thresholds. Box3D's `forceThreshold` / `torqueThreshold` emit joint events; `b3Joint_GetConstraintForce`
  confirms, and we destroy the joint. Nothing breaks automatically in Box3D.
- **Re-homing.** When a jointed piece fractures, the joint moves to the child cell containing the anchor, or breaks
  if that cell became debris.
- **Soft welds** (`linearHertz` / `angularHertz`) give wobbly assemblies. Chains of welds flex; that is a feature
  for rickety carts.
- **"Fairy dust":** per-object `b3Body_SetGravityScale` (cheap), later buoyancy. Joint forces tell you whether the
  contraption can be carried.

## 4. Destructible vehicles

- **Driving:** raycast suspension and our own tyre friction; stable at racing speed and deterministic. When a wheel
  comes off, a physical wheel body spawns. Box3D's wheel joint (steering servo, spin motor, friction-only grip) is
  the fallback.
- **Construction:** the chassis is convex parts (frame, floor, cabin panels, hood, doors, bumpers) plus hidden
  interior "vital" parts (engine block, fuel tank, battery, wheel mounts). This works like Teardown's vital tags,
  where destroying the engine area stops the car. Driving ability comes from which vital parts are still attached.
- **Crumpling:**
  - a cage lattice in car space (about 4×3×6 nodes) is dented by impacts, capped per node
  - render vertices interpolate from the lattice (per-vertex crush weight, like GTA's vertex-colour deformation)
  - the chassis hull is rebuilt from the lattice only occasionally (`b3Shape_SetHull` + `ApplyMassFromShapes`)
  - panels sit on breakable joints; parts fracture only past a threshold
- **Fuel tank:** a detonator part. Leaking comes later with fluids.
- **Materials:** sheet metal (dents), glass (shatters), rubber.

## 5. Large-map physics zones

- **Active cells** along the track, around "observers": players, AI cars, and flagged "chunk loaders" (armed
  contraptions, timers, detonators; anything the player sets up deliberately), like Minecraft's simulation distance.
- **Outside the active cells:**
  - resting islands sleep
  - loose debris is resolved by tier: snapped to the ground and frozen, or removed
  - unstable structures are settled with the stress check
  - flagged Rube Goldberg setups keep simulating
- **Determinism:** activity is derived from shared simulation state (player and car positions), never the camera.
- **Box3D:** prefer sleep over `b3Body_Disable` (re-enabling costs a spike). The static tree scales well; use
  `invokeContactCreation = false` for bulk static creation. Double precision is available beyond about 16 km.
- **Streaming:** add bodies in batches. Rendering LOD is a separate track.

## 6. Networking

Options:
- Teardown's approach: destruction is deterministic and sent as commands; bodies are server-synced with a
  priority queue.
  https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
- Full lockstep on Box3D's determinism, if the toolchain is pinned.

A two-process lockstep experiment comes first.

## 7. Art polish

Shaders, palette, sky, ambient occlusion, character pipeline (RetroDiffusion low-poly GLB with auto-rigging, Meshy
with target poly count, Blender cleanup via its MCP), point-filtered character textures. Essential for any public
demo; deliberately last.

Effects and mess, noted while tuning debris:

- **Dirt where debris lands:** decals or vertex-colour darkening on the ground and on walls around scrap and rubble,
  so a fight leaves the place dirty even after the pieces are cleared or sunk.
- **Smoke and fire:** particle smoke and volumetric fire (Teardown-style). Fire ties into material flammability
  ([materials.md](materials.md)).
- **Dust:** soft, fading dust clouds instead of solid motes once a transparent particle pass exists.
- **Toaster profile:** one switch that lowers caps (debris, ghosts, scrap), raises the fragment scale and render
  scale, and turns off shadows, so low-end machines still get the destruction.
