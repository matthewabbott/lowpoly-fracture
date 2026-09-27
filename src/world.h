// SPDX-License-Identifier: MIT
// Internal layout of lpWorld, shared by world.c and the tests.

#pragma once

#include "core.h"
#include "fracture.h"
#include "poly.h"

typedef struct lpTaskPool lpTaskPool;

typedef enum lpBodyKind
{
	lp_kindStructure, // static until pieces break loose; anchored components stay
	lp_kindDebris,	  // dynamic
	lp_kindRubble,	  // debris that settled and was frozen static
} lpBodyKind;

typedef struct lpPiece
{
	lpShape* shape;	  // body frame; NULL for a free slot
	b3HullData* hull; // cached Box3D hull of the shape, reused when the piece changes body
	b3ShapeId shapeId;
	LP_ARRAY( int ) bonds;
	b3Plane anchorPlane;
	b3Vec3 axis; // grain axis (wood) or pane normal (glass), body frame
	uint32_t color;
	uint32_t seed;
	uint32_t generation;
	uint32_t meshVersion;
	int body;
	int nextFree;
	int mark;
	int supportBond; // stress pass: bond toward the ground
	int loadSlot;	 // stress pass: index into scratchLoad
	uint8_t material;
	uint8_t depth;
	bool anchored;
} lpPiece;

typedef struct lpBond
{
	int a, b; // a < b
	float area;
	float health;		// J/m^2 of damage left
	float strength;		// health when intact
	float loadStrength; // N/m^2 under the weight check, when intact
	b3Vec3 centroid; // body frame
	int nextFree;
	int stamp;
	bool alive;
} lpBond;

typedef struct lpBody
{
	b3BodyId id;
	LP_ARRAY( int ) pieces;
	uint64_t createdTick;
	float volume;
	uint32_t generation;
	int nextFree;
	int stamp;
	uint8_t kind;
	bool alive;
	bool dirty;
	bool freezePending;
	bool armed;
	lpDetonatorDef detonator;
} lpBody;

typedef struct lpPull
{
	int piece;
	b3Vec3 localPoint;
	b3Pos target;
	float maxAccel;
	float maxMass;
} lpPull;

typedef struct lpWake
{
	b3Vec3 center;
	float radius;
} lpWake;

typedef struct lpForce
{
	b3Pos point;
	b3Vec3 direction;
	float radius;
	float impulse;
	bool explosion;
} lpForce;

typedef struct lpComponent
{
	int first; // into scratchQueue
	int count;
	float volume;
	b3Vec3 centroid; // body frame, volume weighted
	bool anchored;
} lpComponent;

typedef struct lpHitCandidate
{
	float energy;
	b3Pos point;
	uint64_t key;
} lpHitCandidate;

#define LP_MAX_CELL_BONDS ( LP_MAX_SITES * 24 )

// One piece to fracture during an impact (see world.c)
typedef struct lpFractureJob
{
	int piece;
	b3Vec3 localImpact; // body frame
	b3Vec3 center;		// piece centroid; the fracture runs in a frame centered here
	lpPoly poly;
	lpFractureInput input;
	float minPieceVolume;

	int cellCount;
	lpShape* cells[LP_MAX_SITES];
	int cellSites[LP_MAX_SITES];
	b3HullData* hulls[LP_MAX_SITES];
	int bondCount;
	lpCellBond* bonds; // LP_MAX_CELL_BONDS
	lpFractureStats stats;
} lpFractureJob;

struct lpWorld
{
	lpWorldDef def;

	LP_ARRAY( lpPiece ) pieces;
	LP_ARRAY( lpBond ) bonds;
	LP_ARRAY( lpBody ) bodies;
	int freePiece;
	int freeBond;
	int freeBody;
	int bondCount;

	LP_ARRAY( lpImpactDef ) impacts;	 // to apply at the next step
	LP_ARRAY( lpImpactDef ) nextImpacts; // collision impacts found during a step
	LP_ARRAY( lpParticle ) particles;	 // emitted during the last step
	LP_ARRAY( lpWake ) pendingWakes;
	LP_ARRAY( lpForce ) forces;
	LP_ARRAY( int ) dirtyBodies;
	LP_ARRAY( int ) freezeCandidates;
	LP_ARRAY( int ) stressAgain; // structures that lost bonds to their own weight; re-checked next step
	LP_ARRAY( float ) scratchLoad;
	LP_ARRAY( int ) pendingDestroy; // detonated bodies, removed at the start of the next step
	LP_ARRAY( lpPull ) pulls;

	LP_ARRAY( int ) scratchPieces;
	LP_ARRAY( int ) scratchBodies;
	LP_ARRAY( int ) scratchQueue;
	LP_ARRAY( lpComponent ) scratchComponents;
	LP_ARRAY( lpHitCandidate ) scratchHits;

	lpTaskPool* tasks;
	lpFractureJob* jobs;
	int jobCount;
	int jobCapacity;

	uint64_t tick;
	uint64_t pieceSerial;
	int stamp;
	lpStats stats;
};

// Full invariant check, for tests. Returns false and prints the first violation.
bool lpWorld_Validate( const lpWorld* world );
