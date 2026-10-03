# Box3D (vendored)

- Upstream: https://github.com/erincatto/box3d (MIT, see LICENSE)
- Pinned commit: `5643cd81ff07fd0497e3bfcdf04f6425cc8a2e7f` (2026-09-25, "Remove compound mesh child material limit. (#172)")
- Vendored subset: `include/`, `src/`, `CMakeLists.txt`, `LICENSE`, `README.md`. Samples, tests, benchmark,
  docs and data are not vendored; clone upstream at the pinned commit to run them.
- Upstream does not accept pull requests. Any local change must be listed below with its reason, so a future
  update can re-apply or drop it.

## Local patches

- `revolute_joint.c`, `b3GetRevoluteJointTorque`: the axial impulse (spring, motor and limits) was counted twice,
  once along `rotationAxisZ` (as the solver applies it) and again along the frame's z axis, so a hinge at its limit or
  driven by its motor reported twice its axial torque. The second term is dropped. Links judge a hinge's load from
  this getter, and motorised hinges read their drive from it. (The getter still writes `perpAxisX/Y`, which
  `b3PrepareRevoluteJoint` recomputes anyway.)

- `scheduler.c`, `b3SchedulerExecuteOne`: `b3FpGuard()` before each task puts back a floating-point control word
  that flushes subnormals or changes rounding (MXCSR on x86, FPCR on ARM64), as our own workers do (`lpFpGuard`,
  determinism rule 15). Scheduler threads inherit their creator's control word on POSIX, and milestone 7's experiment
  E9 showed flush-to-zero changes the stress solver's results. Drop it if upstream ever exposes a per-task hook.

- `physics_world.c`, `b3World_VisitContactState` (new, declared in `box3d.h`): one walk over the touching contacts,
  reporting what each carries into the next step (manifolds with their warm-start impulses and feature ids, the
  recycling cache, flags, which bodies are static) with its shapes' and bodies' user data, so the state hash (milestone 10)
  covers it.
  Reading the same through `b3Body_GetContactData` per body costs 0.5 to 2 ms at the bench peaks.
  `b3World_VisitContactStateRange` and `b3World_GetContactSlotCount` split the walk among threads.
- `body.c`, `b3Body_GetSleepTime` (new): the body's sleep timer, for the state hash. `b3Body_SetSleepTime` (new) sets
  it, for the two-world lab's repair (src/lab.c).
- `joint.c`, `b3Joint_GetImpulses` (new): the impulses a joint's next solve starts from (point, limit, motor and
  spring impulses, by joint type), for the state hash: a joint's warm start carries into the next step like a
  contact's.
- `physics_world.c`, `b3World_RestoreContactState` (new): writes a touching contact's manifolds and recycling caches
  back, by its slot, for the same lab (a repair that copies warm starts, and the injected warm-start desync).
- `body.c`, `b3Body_GetMotion` (new): the transform and both velocities in one lookup instead of three, for the state
  hash, which reads every body that moved each tick.

## Known issues at this commit (found in a code audit; not patched, avoided instead)

- `solver.c:443`: CCD calls `world->preSolveFcn` without a NULL check. Never enable `enablePreSolveEvents` on a
  shape unless a presolve callback is registered.
- `body.c:1571-1591`: `b3Body_SetType` sets `world->locked` and returns early without unlocking for bodies with
  compound or height-field shapes. We only change the type of hull-shape bodies.
- `shape.c:1402-1410`: `b3Shape_SetFilter` assigns the new filter before comparing category bits, so it always
  recreates the broadphase proxy. It is slow but safe; we set filters at shape creation instead.
- `contact.c:248, 322-330`: contact-event, presolve and rolling-resistance settings are captured when a contact is
  created; toggling them on a shape only affects future contacts.
- `joint.c:1432`: `b3Joint_GetAngularSeparation` asserts on wheel joints.
- `joint.c:1379-1398`, `b3Joint_GetAngularSeparation` on a revolute joint: it takes the bodies' relative rotation, not
  the joint frames', and removes its component about the bodies' z axis as "the hinge angle". A hinge whose axis is not
  its bodies' z (a mech's knee) reads its own angle as separation, and the limit test measures the twist about the
  wrong axis. Rigs measure their joints' give against their own model instead.
- `joint.c:1114`: the wheel joint reaction force is marked "todo probably wrong".
- `wheel_joint.c`, `b3GetWheelJointForce`: adds `lowerSuspensionLimit` (a length) to impulses, has the wrong sign on
  the upper limit and permutes the axes; `b3GetWheelJointTorque` returns only the spin impulse. Our wheels use no
  joint (`src/wheel.c`), so nothing reads them.
- `prismatic_joint.c`, `b3GetPrismaticJointForce`: adds the upper limit impulse where the solver subtracts it. No
  link uses a prismatic joint yet.
- The custom filter callback runs only at pair creation, and both callbacks run on worker threads (they must be
  pure).
