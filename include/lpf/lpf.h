// SPDX-License-Identifier: MIT
// lpf: low-poly polygonal destruction on Box3D.
//
// Objects are compounds of convex pieces. Impacts refracture the struck pieces into convex cells, damage the
// bonds between pieces, and pieces cut off from their anchors become Box3D debris bodies. Settled debris
// freezes into static rubble. Everything is deterministic: the same calls in the same order give
// bit-identical results (see docs/determinism-rules.md).

#pragma once

#include "box3d/box3d.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- materials ----

typedef enum lpMaterialId
{
	lp_wood,
	lp_stone,
	lp_brick,
	lp_plaster,
	lp_concrete,
	lp_glass,
	lp_metal,
	lp_ground,
	lp_foliage,
	lp_materialCount
} lpMaterialId;

typedef enum lpPatternId
{
	lp_breakImpact, // Voronoi dense at the impact: stone, brick, concrete, plaster
	lp_breakGrain,	// Voronoi stretched along the grain: wood splinters
	lp_breakRadial, // wedges and rings in the pane: glass
} lpPatternId;

typedef struct lpMaterialDef
{
	const char* name;
	float density;		  // kg/m^3
	float bondStrength;	  // energy density (J/m^2) that breaks a bond between pieces
	float fractureEnergy; // energy density (J/m^2) that refractures a piece
	float fragmentSize;	  // edge of the smallest fragments at the impact, meters
	float friction;
	float restitution;
	int pattern;		// lpPatternId
	float grainStretch; // wood: cells this many times longer along the grain
	uint32_t interiorColor; // 0xRRGGBB of freshly exposed faces
	bool breakable;
	float loadStrength; // N/m^2 a bond can carry under the weight check, scaled by its remaining health
} lpMaterialDef;

const lpMaterialDef* lpGetMaterial( int materialId );
void lpSetMaterial( int materialId, const lpMaterialDef* def );

// ---- world ----

typedef struct lpWorldDef
{
	b3WorldId physics;
	uint64_t seed;
	int maxDebrisBodies;   // cap on moving debris; smallest-oldest are removed first
	int maxRubblePieces;   // cap on frozen rubble pieces
	float fragmentScale;   // multiplies every material's fragment size (the main performance knob)
	bool freezeRubble;	   // debris that Box3D puts to sleep becomes static rubble (costs nothing to simulate)
	float minPieceVolume;  // cells smaller than this become cosmetic particles, m^3
	int maxDepth;		   // refracture depth limit per piece lineage
	int maxHitImpacts;	   // collision impacts processed per step
	float hitSpeed;		   // minimum approach speed for collision damage, m/s
	float killDepth;	   // bodies falling below this height are removed
	bool debugLog;		   // print every processed impact to stdout (for diagnosing tuning)
	int workerCount;	   // threads for fracture work, including the caller (results do not depend on it)
	float stressScale;	   // multiplies every material's loadStrength; 0 disables collapse under weight
} lpWorldDef;

lpWorldDef lpDefaultWorldDef( void );

typedef struct lpWorld lpWorld;

lpWorld* lpCreateWorld( const lpWorldDef* def );
void lpDestroyWorld( lpWorld* world );

// ---- objects ----

// One convex part of an object, in object space. A box when pointCount is zero, else the hull of points.
typedef struct lpPartDef
{
	b3Vec3 halfExtents;
	b3Transform transform;
	const b3Vec3* points;
	int pointCount;
	uint8_t material;
	uint32_t color;	  // exterior 0xRRGGBB
	b3Vec3 grainAxis; // object space; zero picks the longest box axis
	bool anchored;	  // rests on a foundation: bonded to the world through its bottom face
} lpPartDef;

// Makes an object explode: when it hits something at triggerSpeed or faster, or when a nearby blast reaches it.
// A thrown alchemical flask, a gas can, volatile cargo. Zero radius means inert.
typedef struct lpDetonatorDef
{
	float triggerSpeed; // m/s approach speed (at least lpWorldDef.hitSpeed, where collisions start to register)
	float radius;
	float energy;
	float speed; // blast push, m/s at the center
} lpDetonatorDef;

typedef struct lpObjectDef
{
	b3WorldTransform transform;
	bool isStatic; // structures stay static until pieces break loose
	const lpPartDef* parts;
	int partCount;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
	lpDetonatorDef detonator;
} lpObjectDef;

lpObjectDef lpDefaultObjectDef( void );
lpPartDef lpDefaultPartDef( void );

// Returns the body index of the object.
int lpCreateObject( lpWorld* world, const lpObjectDef* def );

// ---- impacts ----

typedef struct lpImpactDef
{
	b3Pos point;
	b3Vec3 direction; // unit; pushes loose pieces this way (zero for radial)
	float radius;
	float energy;  // joules
	float impulse; // directional: N*s given to loose pieces (capped at 12 m/s); explosion: outward speed at the center, m/s
	bool explosion;
} lpImpactDef;

// Queued; applied at the start of the next step, in call order.
void lpWorld_AddImpact( lpWorld* world, const lpImpactDef* impact );

// Pull a piece toward a target like a spring (grab tool, winch). Call every tick while pulling; applied at the next
// step. localPoint is in the piece's body frame (lpRayHit gives world points: convert with lpWorld_ToBodyFrame).
// Structures cannot be pulled; frozen rubble wakes up. The pull accelerates at most maxAccel and treats bodies
// heavier than maxMass as maxMass (so a crane can lift a beam but not a house).
void lpWorld_Pull( lpWorld* world, int piece, b3Vec3 localPoint, b3Pos target, float maxAccel, float maxMass );
b3Vec3 lpWorld_ToBodyFrame( const lpWorld* world, int piece, b3Pos worldPoint );
b3Pos lpWorld_ToWorldFrame( const lpWorld* world, int piece, b3Vec3 localPoint );

void lpWorld_Step( lpWorld* world, float timeStep, int subStepCount );

// Hash of the full simulation state (bodies, pieces, bonds). Equal hashes after the same inputs prove determinism.
uint64_t lpWorld_Hash( const lpWorld* world );

typedef struct lpStats
{
	int pieceCount;
	int bondCount;
	int structureBodies;
	int debrisBodies;
	int awakeDebris;
	int rubbleBodies;
	int impactsThisStep;
	int fracturesThisStep;
	int cellsThisStep;
	int splitsThisStep;
	int removedThisStep;
	int particlesThisStep;
	int clipFailures;
	float fractureMs;
	float physicsMs;
	float updateMs;

	// Breakdown of fractureMs for this step
	float cellMs;  // Voronoi cells (lpFracture)
	float hullMs;  // Box3D hulls for new pieces
	float shapeMs; // Box3D shape create/destroy
	float bondMs;  // contact areas for new bonds
	float splitMs; // connectivity and new bodies
	uint64_t tick;
} lpStats;

lpStats lpWorld_GetStats( const lpWorld* world );

// ---- queries ----

typedef struct lpRayHit
{
	b3Pos point;
	b3Vec3 normal;
	int piece;
	int body;
	bool hit;
} lpRayHit;

lpRayHit lpWorld_CastRay( const lpWorld* world, b3Pos origin, b3Vec3 translation );

// ---- rendering access ----

// Vertex of a piece's flat-shaded render mesh, in the body frame.
typedef struct lpVertex
{
	float position[3];
	int8_t normal[4];	// snorm8 face normal, w unused
	uint32_t color;		// 0xAABBGGRR (little-endian RGBA8)
} lpVertex;

typedef struct lpPieceInfo
{
	int body;			  // -1 when the slot is free
	uint32_t meshVersion; // changes whenever the render mesh must be rebuilt
	uint32_t generation;  // changes when the slot is reused
} lpPieceInfo;

int lpWorld_GetPieceCapacity( const lpWorld* world );
lpPieceInfo lpWorld_GetPieceInfo( const lpWorld* world, int piece );

int lpWorld_GetBodyCapacity( const lpWorld* world );
bool lpWorld_GetBodyTransform( const lpWorld* world, int body, b3WorldTransform* transform );
uint32_t lpWorld_GetBodyGeneration( const lpWorld* world, int body );

// Triangles (3 vertices each, no index buffer). Returns the vertex count, or -1 if capacity is too small.
int lpWorld_BuildPieceMesh( const lpWorld* world, int piece, lpVertex* vertices, int capacity );
int lpWorld_GetMaxPieceVertices( void );

// Cosmetic particles emitted during the last step (dust, tiny chips, despawned debris).
typedef struct lpParticle
{
	float position[3];
	float velocity[3];
	float size;
	uint32_t color; // 0xAABBGGRR
} lpParticle;

const lpParticle* lpWorld_GetParticles( const lpWorld* world, int* count );

#ifdef __cplusplus
}
#endif
