// SPDX-License-Identifier: MIT
// The two-world lab: causal units, injected desyncs, the hash followed down between two worlds, and repair by unit
// (src/units.c, src/lab.c). For experiments (lpf_bench --twin, the tests, a lockstep pair's injected desync): never in
// a game. A game reads lpf.h only.
#pragma once

#include "lpf/lpf.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- causal units ----
//
// The groups of bodies whose members can affect each other within a tick. Bodies are joined through touching contacts,
// links, a vehicle's wheels, a rig's limbs, the ground under a wheel or a planted foot, and pieces that share a
// detonator or a pool; an anchored piece and frozen rubble join nothing (within a tick they take no load and never
// move, so the ground does not make the world one unit). A unit holds its bodies' pieces, bonds and stress state.
// Casts that only read what they hit, and the world-wide budgets, couple units unseen. Computed on demand, not cheap.

// Numbers each body slot's unit into units (capacity at least lpWorld_GetBodyCapacity; -1 for a free slot), counted
// from 0 in the order of each unit's lowest slot. Returns the number of units, or -1 if capacity is too small.
int lpWorld_GetUnits( lpWorld* world, int* units, int capacity );
// The unit an element of the state hash belongs to (-1: the world category, or nothing in that slot). A pool's or a
// detonator's walks every piece (O(pieces)).
int lpWorld_GetElementUnit( const lpWorld* world, const int* units, int category, int slot );

// ---- two worlds ----
//
// Two worlds stepped on the same inputs: a desync injected on purpose, found by following the state hash down, and
// repaired by copying one world's state into the other, unit by unit.

// Nudges the largest component of a body's linear velocity by ulps units in the last place
void lpLab_NudgeVelocity( lpWorld* world, int body, int ulps );
// Nudges the warm start of a touching contact of the body by ulps (the physics engine's hidden state); false if it
// touches nothing
bool lpLab_NudgeWarmStart( lpWorld* world, int body, int ulps );

typedef struct lpLabDiff
{
	int category; // lpHashCategory
	int slot;
	uint32_t generation; // of the body, piece or link in a's slot (0 in other categories)
} lpLabDiff;

// The elements whose hashes differ between two worlds, in (category, slot) order, found as a host and a peer would:
// categories, then 64-slot buckets, then elements. Writes up to capacity; returns how many differ.
int lpLab_Diff( const lpWorld* a, const lpWorld* b, lpLabDiff* out, int capacity );

typedef enum lpLabRepair
{
	lp_labMotion = 1,	  // transforms and velocities (a ghost's or scrap's own motion and landing plan)
	lp_labWarmStarts = 2, // the contacts' manifolds: impulses and the feature ids that match them up
	lp_labSleep = 4,	  // sleep timers
} lpLabRepair;

// Copies a unit (lpWorld_GetUnits' numbering, in src) from src to dst: the parts asked for (lpLabRepair). State only:
// both worlds must hold the same bodies with the same pieces there. Returns the bytes a host would have sent.
int lpLab_RepairUnit( lpWorld* dst, const lpWorld* src, const int* units, int unit, int parts );

#ifdef __cplusplus
}
#endif
