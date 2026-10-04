// stages.h: the toy's CPU stages between the kernels (DESIGN.md "Pipeline per tick", step 2), C17 integers only,
// dialect-free: sort-and-sweep broadphase on the integer AABBs, radix-sorted pair keys (min id, max id), a merge-join
// with last tick's keys (prevIndex, so manifolds stay on the GPU), waking, union-find islands and sleep, greedy colouring
// (joints first, then pairs in key order). Static bodies never join islands or take colours.
#ifndef TOY_STAGES_H
#define TOY_STAGES_H

#define TOY_LAYOUT_NO_DIALECT
#include "layout.h"
#undef TOY_LAYOUT_NO_DIALECT

#include <stdint.h>

#define STAGE_MAX_COLOURS 64 // one bit per colour per body; pairs beyond go to the overflow colour (64)

typedef struct Stages
{
	int bodyCount;
	const uint8_t* isStatic; // per body (the caller's)

	// kept from tick to tick
	uint64_t* prevKeys;
	int prevCount;
	uint8_t* asleep;	  // per body
	int32_t* sleepIsland; // the island a sleeping body fell asleep in (its smallest body index); -1 awake

	// this tick
	uint64_t* keys; // sorted pair keys (min << 32 | max)
	Pair* pairs;	// in key order
	uint8_t* active; // per pair: at least one awake body (solved, coloured)
	int pairCount, activeCount;
	int colourCount, overflowCount;
	int colourStart[STAGE_MAX_COLOURS + 2]; // into colourList, per colour then the overflow
	int32_t* colourList;					// pair indices by colour, key order within a colour
	int32_t* islandOf;						// per body: its island (the smallest body index in it); -1 static or asleep
	int islandCount, largestIsland, sleptIslands, sleptBodies;
	int32_t* awake; // awake dynamic bodies, index order
	int awakeCount;
	int32_t* woken; // bodies woken this tick (their state is reset on the GPU)
	int wokenCount;

	// scratch
	int pairCap;
	uint64_t* sortA;
	uint64_t* sortB;
	uint32_t* counts;
	int32_t* order;
	int32_t* parent;
	int32_t* minTicks;
	uint64_t* used;
} Stages;

void stages_init( Stages* s, int bodyCount, const uint8_t* isStatic );
void stages_free( Stages* s );

// One tick. aabbs: every body's (the GPU's prepareBodies); sleepTicks: every body's counter (the GPU's finalize, last
// tick); prevTouching: per last tick's pair, whether its manifold had points (NULL: every pair touches, step 2).
void stages_tick( Stages* s, const Aabb* aabbs, const int32_t* sleepTicks, const uint8_t* prevTouching, int sleepTicksNeeded,
				  int enableSleep );

// Everything the stages decided this tick, hashed (keys, prevIndex, colours, islands, sleep, the awake list)
uint64_t stages_hash( const Stages* s );

// LSD radix sort of 64-bit keys, 16 bits a pass (passes where every key has the same digit are skipped)
void stages_radix64( uint64_t* keys, uint64_t* tmp, int n, uint32_t* counts65536 );

#endif
