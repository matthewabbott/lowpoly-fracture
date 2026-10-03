# The outcome catalogue

What the engine owes its games ([goals.md](goals.md)), written down as outcomes, each pinned by a test with a tolerance
or by reference frames. This is the contract a redesign or a new physics core must meet (milestone 12 replaces Box3D
with an integer core whose numbers will differ): the outcomes, not the bench hashes. The bench hashes still pin exact
behaviour for refactors that mean to change nothing.

**The words are the contract; the tests are its gauge.** A test's tolerance is what a different but plausible
simulation should also meet. Where today's number is tighter than the outcome needs, the entry says so ("tight"), and
when a redesign or a new core fails such a test while the outcome in words still holds, the tolerance is re-judged
here, in the entry, not quietly in the test.

**Kinds.** Every test in `lpf_test` has one (test/test_macros.h):
- *outcome* and *determinism* tests are the contract, and every one is named below;
- *mechanism* tests pin today's implementation (unit geometry, the stress solver against its exact oracle, tooling,
  the physics engine's own numerics) and change with it;
- *timing* tests print costs.

```
lpf_test --contract                               # the contract only
lpf_test --list                                   # every test with its suite and kind
lpf_test --check-catalogue docs/catalogue.md      # every contract test is named here, and only those (CI runs it)
```

**Visual entries** have contact sheets in [catalogue/](catalogue/), made by `tools/catalogue-shots.ps1` from
`docs/catalogue/shots.txt`, and judged by eye against the entry's words. **References** to the behaviour an entry
aspires to (clips from other games, real footage) are in [references.md](references.md).

Ids are stable: a retired entry keeps its id, struck through, so the reference corpus and the reviews can cite them.

## Fracture and debris

Serves: "The fracture look", "Mess", "Everything is physical and breaks where it is weak".

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| F1 | An impact breaks an object locally: the cells tile it, smaller near the impact | `TestImpactPattern` | at least 8 cells tile the slab to 1e-3 of its volume; the cells nearest the impact smaller than the rest |
| F2 | Wood splinters along its grain | `TestGrainPattern` | splinters at least 1.5 times longer along the grain than across |
| F3 | Glass shatters radially | `TestRadialPattern` | at least 12 cells tile the pane |
| F4 | Brick breaks along its mortar | `TestMasonryGrid`, `TestMasonryWallHole` | every cut on a bed or head joint, 10 to 80 cells, volume conserved; after a grenade every bond left between bricks is mortar |
| F5 | No slivers: the small cells of an impact merge into their neighbours | `TestSliverAbsorption` | no small cell outside the impact's radius; volume conserved |
| F6 | A log shot through leaves two rough ends | `TestLogEnds` | exactly 2 ends of 1 piece each (the count is the outcome), under 60 triangles each |
| F7 | A wall under fire breaks up into pieces and debris | `TestWallDamage` | pieces more than double, over 10 fractures, over 20 debris bodies, no clip failures |
| F8 | Blasts at a house's corners knock a good part of it loose | `TestHouseCollapse` | over 2 m³ loose after two blasts |
| F9 | Fragments are carved from the object and coloured like it, never grey cubes | `TestFragmentColours`; sheet [F9](catalogue/F9.png) | after the walls are shot up and blasted, every face of every piece is the object's colour or its material's cut-face colour, shaded (the same hue within 2%) |
| F10 | Flying fragments land as scrap, or leave the world | `TestGhostLanding` | no ghost left after 5 s; over 5 scrap at a plausible height |
| F11 | Settled rubble is static but fragile: struck, it comes loose | `TestFragileRubble` | rubble within 1.5 s of landing (tight: the physics engine's sleep timer decides it), loose within 1 s of a hit |
| F12 | Rubble lingers after a fight: what was knocked loose stays, settled, not tidied away | `TestRubbleLingers` | after a 10 s barrage on the town and a minute of calm at the default budgets, at least 90% of the loose volume is still there (it grows today: late collapses), and under 1% of what settled is awake |
| F13 | Over budget, debris steps down the ladder (full, light, ghost, scrap) instead of popping out | `TestBudgetLadder` | demotions happen and every cap holds once the ladder has caught up |
| F14 | Light rubble does not slow a heavy crate sliding through it | `TestShove` | the crate ends within 2 cm of where it ends on clear ground (light debris cannot push dynamic bodies, by design) |
| F15 | "Fairy dust": a body's gravity scale, kept by what breaks off it | `TestGravityScale` | a quarter scale falls a quarter as far (±0.02 m); a weightless ghost holds its height; split halves keep the scale |
| F16 | A volatile flask goes off on impact and blows a hole | `TestDetonator` | disarmed, over 20 cells made |
| F17 | The pull tool lifts a loose piece to its target | `TestPull` | within 0.3 m of the target after 2 s |
| F18 | Pieces broken or split off a moving, spinning body leave with the motion it had at their centres | `TestBrokenPiecesKeepMotion` | within 0.5 m/s of the rigid motion, a step on (the fix of 2026-10-02: they once got the spin's velocity about the parent's frame origin again, and chips flew at hundreds of m/s) |

## Structures

Serves: "Breaking in the right place", "Weight and material", "Big events".

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| S1 | Every scene's structures stand under their own weight | `TestStructuresStand` | no volume lost, no late breaks, nothing left unsettled |
| S2 | A cantilever's root carries it: short stands, long snaps, long in low gravity stands | `TestCantileverRoot` | 0.8 m stands, 2 m snaps, 2 m at 0.1 g stands |
| S3 | A plank stands alone and snaps under a stone, which falls | `TestBeamMidspan` | as stated |
| S4 | A tower with a felling cut topples toward the cut | `TestTowerTopples` | the stone's centroid drifts over 1.5 m within 8 s; the same hash at 1 and 4 workers |
| S5 | A stone tower struck with a felling cut goes over toward the cut and comes down; nothing is left creeping or perched, and nothing is thrown faster than the blasts and the fall can throw it | `TestTowerFelled`; sheet [S5](catalogue/S5.png) | its stone's centroid moves over 2.5 m toward the cut within 10 s, less than 0.6 of that sideways; its top under 5 m (of 13); no body over 25 m/s; under 0.5 m/s in its 15th second. Dry-laid, it falls as a shower of blocks, not as one column |
| S6 | A wall shot up settles and stops creaking | `TestDamagedWallSettles` | no stress breaks and nothing unsettled over 2 s of calm |
| S7 | A dry arch stands by compression; without its keystone the span falls and the piers stay | `TestArchKeystone` | stands; under half left without the keystone |
| S8 | A colonnade drops the lintels a lost column carried, and only those | `TestColonnade` | what stands is 3 columns and 1 lintel (to 1e-3 of the volume) |
| S9 | A breach in the keep's wall: the masonry round it gives, the keep stands and settles | `TestKeepBreach` | over 95% stands; settled within 10 s; 1 to 24 joints break (12 today) |
| S10 | A cannon hole in the keep: the cells round it crumble, the keep holds | `TestKeepHole` | over 95% stands; decided within 1 s; the clustered solve flips under 1% of joints against the exact one |
| S11 | A keep under sustained fire is still judged, and comes down where it is hit | `TestKeepUnderFire` | judged at least 5 times; the same hash at 1 and 4 workers |
| S12 | A building that lost a support slumps and goes quiet, or gives out, without creaking forever | `TestBuildingGoesQuiet` | a house blasted at two corners: no joint breaks after 15 s, nothing unsettled and nothing moving (over 5 cm/s) after 20 s (today: 1.1 s and 12.8 s) |
| S13 | A body in free fall carries nothing: falling does not break it | `TestReliefFreeFall` | utilisation under 1e-3 |
| S14 | A beam landing across a ridge bends: a short drop holds, a long one snaps it | `TestReliefLandingSnaps` | 0.3 m holds, 4 m snaps |

## Links, motors and cranes

Serves: "links: ropes, hinges, welds, winches, motors, cranes" (the rigging-race and hauling games).

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| L1 | A sign on two ropes hangs still, each rope carrying half | `TestSignHangs` | at 2.5 m (±0.05); each rope half the weight (±5%) |
| L2 | A weld holds under its limit, creaks and gives a little over it, snaps far over it | `TestWeldOverload` | 1.5 times its limit gives in 6 to 90 steps; 3.5 times within 6 |
| L3 | A hinged door swings to its limit and stays on its hinge | `TestHingeDoorSwings` | swings 1.0 to 1.25 rad against a 1.2 rad limit (tight: the physics engine's limit overshoot); drifts under 2 cm |
| L4 | A hinge held at its stop carries the arm's weight | `TestHingeTorqueAtLimit` | the arm's weight moment (±10%) |
| L5 | A rope survives its piece splitting off, and still holds | `TestLinkSurvivesSplit` | the block holds at 3.0 m (±0.05) |
| L6 | A blast that blows out a rope's anchor breaks that rope; the other holds | `TestLinkBreaksWhenAnchorEjected` | as stated |
| L7 | A small end torn off by a weld to a heavy stone tears, rather than holding the stone | `TestTinyEndTears` | one break within 10 steps of the split (within 2 today) |
| L8 | One round cuts a rope; the sign swings on the other, a second round drops it | `TestRopeShotSnaps` | as stated |
| L9 | A rifle round 2 m from a metal hinge leaves it; a grenade beside it tears it off | `TestBlastBreaksHinge` | as stated |
| L10 | A winch lifts its load | `TestSetRopeLength` | to 2.5 m (±0.05) |
| L11 | A load hung on a cantilever pulls it down | `TestSignPullsBeam` | falls with the load, stands without |
| L12 | The yard at rest: every link holds, nothing strains | `TestYardAtRest` | no breaks; peak load under half of any limit |
| L13 | The yard's cart, its rope shot, rolls into the wall and its volatile crates go off | `TestYardCart` | rolls over 3 m; at least one crate goes off |
| L14 | A motor holds its target up to its torque, and sags past it | `TestMotorHoldsUntilCap` | holds within 0.03 rad; an undersized one sags |
| L15 | A damaged joint jams: it turns slower, and unpowered it sticks | `TestMotorJams` | 0.3 to 0.55 of the speed; holds unpowered |
| L16 | A motor goes limp when its power is cut; one with a brake holds | `TestMotorLimpUnsupplied` | as stated |
| L17 | A motor survives its end splitting off, rebuilt on the new body | `TestMotorSurvivesSplit` | holds |
| L18 | A ball joint's servo turns its arm to a target | `TestBallServo` | yaw 0.8 rad (±0.05) |
| L19 | A crane's load loads its tower; the slew turns the jib | `TestCraneLoadsTower` | slew 0.6 rad (±0.1); a heavier load gives over 1.5 times the footing load |

## Vehicles

Serves: "Vehicles. They crumple rather than shatter, and have innards" (the rigging-race game).

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| V1 | A parked car sits on its springs, a quarter of its weight on each, and sleeps | `TestWheelRestHeight` | sag ±5%, load ±30 N; asleep within 2 s (tight: the sleep timer) |
| V2 | At 40 m/s on a straight it stays upright, on course and at speed | `TestWheelHighSpeedStable` | upright; drift under 1 m; bounce under 5 cm |
| V3 | Full lock at 30 m/s slides instead of rolling over | `TestWheelSteerNoFlip` | stays upright (up·y over 0.7) |
| V4 | It climbs a 20 cm kerb at 15 m/s without being launched | `TestWheelKerb` | as stated |
| V5 | The handbrake holds it on a 15 degree slope; released, it rolls | `TestWheelParkedOnSlope` | creeps under 1 cm in 2 s |
| V6 | A 1 m drop keeps its wheels; an 8 m drop tears some off, as wheels | `TestWheelBreaksOnHardLanding` | as stated |
| V7 | A grenade at a wheel's mount: the wheel follows its piece or comes off | `TestWheelFollowsFracture` | at least 2 wheels still on; the world valid |
| V8 | A chassis cut in two: each half keeps its two wheels | `TestWheelSplitHalfRolls` | as stated |
| V9 | Wheels load what they stand on: a truck breaks a plank bridge a light car crosses | `TestWheelLoadsBridge` | the car breaks nothing; the truck breaks it |
| V10 | The track's drivers lap the ring, upright and whole | `TestTrackLap` | all three cars lap within 30 s, upright |
| V11 | The harder a car hits a wall, the worse for both; most of the car stays one piece | `TestCarWallCrash` | the volume it keeps falls with speed; power drops; at 30 m/s the wall is breached (over 0.3 m³) |
| V12 | A shot fuel tank goes off and the engine stops | `TestCarTankShot` | the tank goes off; power 0 |
| V13 | A blow knocks an engine block loose; the rest still drives the car at its share | `TestCarEngineShot` | power 0.5 to 0.9 |
| V14 | A crash tears the engine off only when it is hard enough | `TestCrashTearsEngine` | nothing off at 10 m/s, at least one engine piece at 30 m/s; the same hash at 1, 4 and 8 workers |

## Systems: parts, supply and pools

Serves: "Bodies are systems, not hit points" and the vehicles' innards.

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| Y1 | A part with its own detonator goes off alone; the rest of the object takes the blast | `TestPartDetonatorBlowsOnlyItsPart` | only the tank's pieces go |
| Y2 | A tank torn off its frame stays volatile and goes off once | `TestTankTornOffStaysVolatile` | as stated |
| Y11 | A charge with a fuse goes off a set time after what sets it off, wherever its pieces have gone | `TestDetonatorDelay` | a crate on a 1 s fuse, knocked by a cannonball: it goes off 59 to 61 steps after it is lit, having slid over 0.1 m, on the same tick at 1, 4 and 8 workers |
| Y3 | Cut a pipe and the far side goes dry: the drop runs down the line at `supplyHopsPerTick` carriers a step (16), so a line shorter than that goes dry in the same step | `TestSupplyCut` | a 7-box line: in the same step |
| Y4 | An engine feeds power only while fuel reaches it | `TestSupplyNeeds` | as stated |
| Y5 | Half an engine gives half the power | `TestSupplyShare` | 0.5 (±0.004) |
| Y6 | A hose between two objects carries fuel | `TestSupplyOverLink` | as stated |
| Y7 | Knock the engine off and the car coasts | `TestCutPowerCoasts` | power 0; it slows |
| Y8 | A cut line leaks its pool until its valves close | `TestPoolLeaksWhenCut` | leaks 3/7 of the pool a second (±0.01); the level within 1% of the model; at most 17 supply updates |
| Y9 | A ring cut once does not leak | `TestPoolRingDoesNotLeak` | as stated |
| Y10 | A chipped conduit leaks a little | `TestPoolChippedLineLeaksALittle` | most of the pool kept (0.8 to 1) |

## Rigs: creatures and mechs

Serves: "Creatures and machines. They adapt on their own" and "Bodies taken down by impairment".

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| R1 | A mech stands on its servos, level, and sleeps | `TestKitStands` | settles within 3 cm, tilt under 1 degree, speed under 1 mm/s, asleep within 4 s (tight: the sleep timer), servos under 70% |
| R2 | A crouch lowers the torso without moving the feet, then it settles | `TestRigCrouch` | drop as asked (±3 cm); feet still; then asleep |
| R3 | It walks straight at most of its top speed, level, its feet gripping | `TestRigWalksStraight` | at least 85% of top speed; tilt rms under 2 degrees; worst foot slip under 12 cm over 84 strides (tight) |
| R4 | It turns in place | `TestRigTurns` | at least 0.5 rad/s, moving under 1 m |
| R5 | It climbs a 15 degree slope | `TestRigClimbsSlope` | at least 60% of its flat speed |
| R6 | It steps up onto a 0.4 m step and down again | `TestRigStepsUpAndDown` | as stated |
| R7 | Stopped from full speed it pulls up, tidies its feet and sleeps | `TestRigStops` | within 1.5 m; asleep within 4 s (tight: the sleep timer) |
| R8 | The mech yard's patrol: round the loop, every leg on | `TestRigPatrols` | reaches the four corners in order; 6 legs able |
| R9 | It walks on five legs, four, and drags itself on three | `TestRigLosesLegs` | speed fractions per leg count; on three it crawls |
| R10 | A leg shot below the knee walks on its peg; one without a tibia is held up | `TestRigWalksOnAPeg` | the peg at least half speed, level |
| R11 | A weak leg walks lower; a limp leg hangs and the others walk on | `TestRigWeakAndLimpLegs` | as stated |
| R12 | A leg shot off leaks until its valves close; with them stuck open it bleeds dry and slumps | `TestRigBleedsOut` | fluid kept, or drained, slowed and sunk |
| R13 | A leg reaches for a point and steps back into the gait | `TestRigReaches` | within 5 cm in 1 s |
| R14 | A strike that would tip it over waits for balance | `TestRigStrikeWaitsForBalance` | it never reaches and stays up |
| R15 | A whole leg stomps harder than a damaged one | `TestRigStompsHarderWhole` | at least 1.5 times the pieces |
| R16 | A claw grabs a crate, lifts it, and drops it when the claw goes | `TestRigGrabs` | lifted over 0.4 m; dropped |
| R17 | Dropped on its feet, nothing breaks | `TestRigLandsWhole` | no breaks; rebound under 3.5 m/s (tight: the physics engine's contact push-out) |
| R18 | A cracked femur holds standing and snaps on landing, alone | `TestRigCrackedFemurSnaps` | cracked to 3 to 10% of its strength; one break within a quarter second of landing (within 5 steps today) |
| R19 | Walking does not break its bones, and their check is cheap | `TestRigWalkingBones` | no breaks in 10 s, the worst joint under 60%, at most 20 torso solves |
| R20 | The look of the gait, and of a crawl on three legs | sheets [R20-gait](catalogue/R20-gait.png) and [R20-crawl](catalogue/R20-crawl.png) (`scripts/mech_crawl.txt`: three legs shot through) | judged by eye: the legs swing in turn, the feet plant where they land, the body level; maimed, it drops onto its belly and drags itself on |
| R21 | A rig a game walks itself (no built-in walker) stands on its foot targets, lifts a foot where it is sent, and its legs push the torso after its pose | `TestRigNoWalker` | sinks under 3 cm with feet within 3 cm of their targets; a foot sent 0.25 m up gets within 4 cm while the others hold within 3 cm; pushed 0.1 m, the torso follows 0.07 to 0.13 m; the same at 1 and 8 workers |

## Far events

Serves: "Scale. Big maps, big events, and things left running far away that still happen."

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| E1 | A contraption left running goes off on time: the `contraption` scene, a spiral of giant dominoes set teetering, each frozen as it waits and woken as the run strikes it, ends in dominoes growing to 2.8 m whose tallest comes down on a volatile vial by a brick wall | `TestContraptionOnTime`; sheet [E1](catalogue/E1.png) | the vial goes off on the same tick at 1, 4 and 8 workers, within 10% of today's 49.8 s; every domino falls (none stalls asleep). The window is what physics left running far away, simplified or not, must meet (milestone 18). Today each frozen domino is pushed over from rest, so the run is slower than real dominoes |

## Determinism

Serves: "Shared by default" and "a simulation that is a pure function of the players' inputs"
([determinism-rules.md](determinism-rules.md)). The 14-leg CI also compares every bench rung across OSes, CPUs and
compilers.

| id | outcome | pinned by | tolerance |
|---|---|---|---|
| D1 | The machine's arithmetic is the one every peer shares | `TestDeterminismSelfTest` | known answers and one hash on every platform |
| D2 | A fracture is the same bytes for the same input | `TestFractureDeterminism` | exact |
| D3 | A bombarded wall is the same state at 1 and 4 workers, and when run again | `TestDeterminism` | every tick's hash |
| D4 | A library switching flush-to-zero on changes nothing | `TestFpGuard` | world and solver hashes; every repair counted |
| D5 | A replay script gives the same state at 1 and 8 workers | `TestScriptReplay` | every tick's hash |
| D6 | The keep's stress solve is the same at 1 and 4 workers | `TestKeepDeterminism` | world and solver hashes |
| D7 | Linked things under a blast | `TestLinkDeterminism` | every tick's hash, 1 and 4 workers |
| D8 | Vehicles driving and blown up | `TestVehicleDeterminism` | 1, 4 and 8 workers |
| D9 | Supply under fire | `TestSupplyDeterminism` | 1, 4 and 8 workers |
| D10 | Pools leaking | `TestPoolDeterminism` | 1 and 4 workers |
| D11 | A rig standing, and walking and turning | `TestRigDeterminism`, `TestRigWalkDeterminism` | 1, 4 and 8 workers |
| D12 | Two worlds side by side stay equal; a one-ulp desync injected into one is named (the body, its generation, its unit), and a repair of that unit stays repaired | `TestTwinWorlds`, `TestDesyncNamed` | every tick's hash; exact names |
| D13 | Two machines in lockstep stay in sync with one's player's commands relayed by the host; a one-ulp desync injected into the peer is named by the host (the body and the first tick) and stops both | `TestLockstepPair` | 240 ticks, every tick's hash; exact names. Over localhost in CI (one Linux leg) |
