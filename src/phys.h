// SPDX-License-Identifier: MIT
// The physics interface: every rigid-body operation the core uses, on opaque handles. src/phys_box3d.c implements it
// on Box3D and is the only file that includes Box3D; the integer core (roadmap milestone 12) will be a second backend.
//
// Shapes carry a piece index and bodies a body index (-1 for none), and everything the backend reports (contacts,
// hits, moves, query and cast results) comes back in those terms and in a total order of our own, never the engine's
// traversal or report order (determinism rule 12).
//
// Threads: the hull operations (create, destroy, read, distance) are the only ones called from parallel jobs (fracture
// jobs build hulls); everything else is called on the stepping thread, between steps. Report and query results live
// in the backend's buffers until the next call of the same kind. Handles carry a generation: a handle to something
// destroyed never matches a later one, so they may be kept and compared across steps.

#pragma once

#include "lpf/lpmath.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct lpPhys lpPhys;

typedef struct lpPhysHull lpPhysHull;

// Handles: 0 is null
typedef struct lpPhysBody
{
	uint64_t handle;
} lpPhysBody;

typedef struct lpPhysShape
{
	uint64_t handle;
} lpPhysShape;

typedef struct lpPhysJoint
{
	uint64_t handle;
} lpPhysJoint;

#define LP_PHYS_NULL( h ) ( ( h ).handle == 0 )
#define LP_PHYS_EQUAL( a, b ) ( ( a ).handle == ( b ).handle )

static const lpPhysBody lp_nullPhysBody = { 0 };
static const lpPhysShape lp_nullPhysShape = { 0 };
static const lpPhysJoint lp_nullPhysJoint = { 0 };

#define LP_PHYS_MAX_POINTS 128 // of a hull, a cast shape, a distance query

// Collision filtering: two shapes touch when each one's category is in the other's mask; a query's likewise
typedef struct lpPhysFilter
{
	uint64_t category;
	uint64_t mask;
} lpPhysFilter;

static const lpPhysFilter lp_physQueryAll = { 0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull };

// ---- the world ----

// Called on the engine's worker threads when two shapes' bounds first overlap: false and the pair never collides.
// A GPU backend cannot call back: milestone 12 turns the rule into data (collision classes), as for cast filters.
typedef bool lpPhysPairFcn( int pieceA, int pieceB, void* context );

typedef struct lpPhysDef
{
	lpVec3 gravity;
	int workerCount;
	float hitSpeed; // contacts that start at this approach speed or faster are reported as hits
	lpPhysPairFcn* pairFilter; // for shapes created with customFilter
	void* context;
} lpPhysDef;

typedef struct lpPhysCounters
{
	int shapes;
	int contacts;
	int awakeContacts;
} lpPhysCounters;

lpPhys* lpPhys_Create( const lpPhysDef* def );
void lpPhys_Destroy( lpPhys* p );
void lpPhys_Step( lpPhys* p, float timeStep, int subStepCount );
lpPhysCounters lpPhys_GetCounters( const lpPhys* p );

// ---- bodies ----

typedef struct lpPhysBodyDef
{
	bool dynamic; // else static
	lpWorldTransform transform;
	lpVec3 linearVelocity;
	lpVec3 angularVelocity;
	float gravityScale;
	float sleepThreshold; // m/s; 0 keeps the engine's default
	int body;			  // its index, -1 for none
} lpPhysBodyDef;

lpPhysBodyDef lpPhys_DefaultBodyDef( void ); // static at the origin, gravity scale 1, no body index

lpPhysBody lpPhys_CreateBody( lpPhys* p, const lpPhysBodyDef* def );
void lpPhys_DestroyBody( lpPhys* p, lpPhysBody body ); // with its shapes and joints
bool lpPhys_IsValidBody( const lpPhys* p, lpPhysBody body );
int lpPhys_GetShapeCount( const lpPhys* p, lpPhysBody body );

bool lpPhys_IsDynamic( const lpPhys* p, lpPhysBody body );
void lpPhys_SetDynamic( lpPhys* p, lpPhysBody body, bool dynamic );

lpWorldTransform lpPhys_GetTransform( const lpPhys* p, lpPhysBody body );
lpPos lpPhys_GetWorldCenter( const lpPhys* p, lpPhysBody body ); // centre of mass
lpVec3 lpPhys_GetLocalCenter( const lpPhys* p, lpPhysBody body );
lpVec3 lpPhys_GetLinearVelocity( const lpPhys* p, lpPhysBody body );
lpVec3 lpPhys_GetAngularVelocity( const lpPhys* p, lpPhysBody body );
lpVec3 lpPhys_GetPointVelocity( const lpPhys* p, lpPhysBody body, lpPos point );
void lpPhys_SetLinearVelocity( lpPhys* p, lpPhysBody body, lpVec3 v );
void lpPhys_SetAngularVelocity( lpPhys* p, lpPhysBody body, lpVec3 omega );

float lpPhys_GetMass( const lpPhys* p, lpPhysBody body );
lpMatrix3 lpPhys_GetInvInertia( const lpPhys* p, lpPhysBody body ); // world frame
// Mass and inertia from the body's shapes, plus mass * radius^2 on the inertia's diagonal
void lpPhys_UpdateMass( lpPhys* p, lpPhysBody body, float inertiaRadius );

void lpPhys_ApplyForce( lpPhys* p, lpPhysBody body, lpVec3 force, lpPos point, bool wake );
void lpPhys_ApplyImpulse( lpPhys* p, lpPhysBody body, lpVec3 impulse, lpPos point, bool wake );

bool lpPhys_IsAwake( const lpPhys* p, lpPhysBody body );
// Its transform and velocities at once (cheaper than the three getters)
void lpPhys_GetMotion( const lpPhys* p, lpPhysBody body, lpWorldTransform* transform, lpVec3* linear, lpVec3* angular );
// How long the body has been still enough to sleep, s (the engine's hidden state, for the state hash)
float lpPhys_GetSleepTime( const lpPhys* p, lpPhysBody body );

// The two-world lab (lab.c) only: moving a body, setting its sleep timer, and reaching into the hidden contact state
void lpPhys_SetTransform( lpPhys* p, lpPhysBody body, lpWorldTransform transform );
void lpPhys_SetSleepTime( lpPhys* p, lpPhysBody body, float seconds );
// Nudges the first manifold point's normal impulse (its warm start) of a touching contact of the body by ulps; false
// if it has none
bool lpPhys_NudgeContact( lpPhys* p, int body, int ulps, int* other ); // other: the contact's other body (-1: static)
// Copies the manifolds (impulses, feature ids) and recycling caches of every touching contact in src with a body whose
// bit is set in bodies (by body index, bodyCount of them) to the contact of dst between the same two shapes, where it
// has the same manifold count; returns the bytes of what carries over (the fields the state hash covers). A contact
// one world has and the other lacks stays so: a state-only repair cannot make or end contacts.
int lpPhys_CopyContacts( lpPhys* dst, const lpPhys* src, const uint8_t* bodies, int bodyCount );
// The engine's hidden contact state (warm starts, the feature ids matching them, recycling caches): adds one hash per
// touching contact, sleeping ones too (they wake with their warm starts), to the sums of its non-static bodies (by body
// index; bodyCount entries), so the sums do not depend on the order contacts are kept in, and a static body's never
// changes. only (NULL: every body): the bodies to sum for, those with a bit of onlyBits set; the others' contacts are
// skipped before they are hashed, so a walk for a few marked bodies costs little more than the walk.
void lpPhys_HashContacts( lpPhys* p, const uint8_t* only, uint8_t onlyBits, uint64_t* sums, int bodyCount );
// The same contact hashes over a range of the engine's contact slots, one record per slot (bodies -1 where none),
// so they can be made in parallel and added up after
typedef struct lpPhysContactHash
{
	int a, b; // its bodies wanted (non-static, and in only), or -1
	uint64_t hash;
} lpPhysContactHash;
int lpPhys_GetContactSlotCount( const lpPhys* p );
void lpPhys_HashContactRange( const lpPhys* p, int begin, int end, const uint8_t* only, uint8_t onlyBits, int bodyCount,
							  lpPhysContactHash* records );
// The bodies (by index) whose engine state a call changed since lpPhys_ClearTouched: velocities, forces, type, mass,
// sleep, gravity, shapes or joints. With the step's moves, everything a step or a call can change; the incremental
// state hash rehashes them.
int lpPhys_GetTouched( const lpPhys* p, const int** bodies );
void lpPhys_ClearTouched( lpPhys* p );
void lpPhys_SetAwake( lpPhys* p, lpPhysBody body, bool awake );
void lpPhys_SetSleepThreshold( lpPhys* p, lpPhysBody body, float speed );
float lpPhys_GetGravityScale( const lpPhys* p, lpPhysBody body );
void lpPhys_SetGravityScale( lpPhys* p, lpPhysBody body, float scale );

lpAABB lpPhys_GetBounds( const lpPhys* p, lpPhysBody body ); // of its shapes
lpVec3 lpPhys_GetMaxExtent( const lpPhys* p, lpPhysBody body ); // from the centre of mass to the farthest shape point

// ---- hulls and shapes ----

// Quickhull. NULL on failure (degenerate, or more than maxVertices). Thread-safe.
lpPhysHull* lpPhys_CreateHull( const lpVec3* points, int count, int maxVertices );
void lpPhys_DestroyHull( lpPhysHull* hull );

int lpPhys_GetHullVertexCount( const lpPhysHull* hull );
int lpPhys_GetHullFaceCount( const lpPhysHull* hull );
lpVec3 lpPhys_GetHullPoint( const lpPhysHull* hull, int vertex );
lpPlane lpPhys_GetHullPlane( const lpPhysHull* hull, int face );
// A face's vertex loop (counter-clockwise from outside) into indices; -1 if it has more than capacity
int lpPhys_GetHullFace( const lpPhysHull* hull, int face, uint8_t* indices, int capacity );

// GJK distance between two convex point sets (at most LP_PHYS_MAX_POINTS each) in one frame
float lpPhys_HullDistance( const lpVec3* a, int countA, const lpVec3* b, int countB );

typedef struct lpPhysShapeDef
{
	float density;
	float friction;
	float restitution;
	int material; // reported by casts
	int piece; // its index, -1 for none
	lpPhysFilter filter;
	bool hitEvents;
	bool customFilter; // ask lpPhysDef.pairFilter
} lpPhysShapeDef;

// A hull shape; the body's mass is left alone (lpPhys_UpdateMass)
lpPhysShape lpPhys_CreateHullShape( lpPhys* p, lpPhysBody body, const lpPhysShapeDef* def, const lpPhysHull* hull );
void lpPhys_DestroyShape( lpPhys* p, lpPhysShape shape ); // the body's mass is left alone
bool lpPhys_IsValidShape( const lpPhys* p, lpPhysShape shape );
lpPhysBody lpPhys_GetShapeBody( const lpPhys* p, lpPhysShape shape );

// ---- joints ----

typedef enum lpPhysJointType
{
	lp_physWeld,
	lp_physHinge,
	lp_physBall,
	lp_physRope,
} lpPhysJointType;

typedef struct lpPhysJointDef
{
	lpPhysJointType type;
	lpPhysBody bodyA;
	lpPhysBody bodyB;
	lpTransform frameA; // the joint frame in each body's frame; a hinge turns about z
	lpTransform frameB;
	bool collideConnected;
	float hertz;		// weld: linear and angular stiffness, 0 for rigid
	float dampingRatio; // weld
	float lowerAngle;	// hinge: limited when lower < upper
	float upperAngle;
	float coneAngle;  // ball: limited when > 0
	bool enableMotor; // hinge, ball
	float length;	  // rope: the most it stretches to; slack below
} lpPhysJointDef;

lpPhysJoint lpPhys_CreateJoint( lpPhys* p, const lpPhysJointDef* def );
void lpPhys_DestroyJoint( lpPhys* p, lpPhysJoint joint, bool wakeBodies );
bool lpPhys_IsValidJoint( const lpPhys* p, lpPhysJoint joint );
void lpPhys_GetJointBodies( const lpPhys* p, lpPhysJoint joint, lpPhysBody* a, lpPhysBody* b );
void lpPhys_WakeJoint( lpPhys* p, lpPhysJoint joint );
// The force and torque the joint applied on body B at the last step (world frame)
void lpPhys_GetJointLoad( const lpPhys* p, lpPhysJoint joint, lpVec3* force, lpVec3* torque );
float lpPhys_GetJointSeparation( const lpPhys* p, lpPhysJoint joint ); // linear error, m
// The impulses its next solve starts from (the engine's hidden state, for the state hash): up to capacity, returns how
// many there are (at most LP_JOINT_IMPULSES)
#define LP_JOINT_IMPULSES 24
int lpPhys_GetJointImpulses( const lpPhys* p, lpPhysJoint joint, float* impulses, int capacity );
float lpPhys_GetHingeAngle( const lpPhys* p, lpPhysJoint joint );
void lpPhys_SetHingeMotor( lpPhys* p, lpPhysJoint joint, float speed );
void lpPhys_SetBallMotor( lpPhys* p, lpPhysJoint joint, lpVec3 omega );
void lpPhys_SetMotorMaxTorque( lpPhys* p, lpPhysJoint joint, float torque ); // hinge or ball
float lpPhys_GetHingeMotorTorque( const lpPhys* p, lpPhysJoint joint );
lpVec3 lpPhys_GetBallMotorTorque( const lpPhys* p, lpPhysJoint joint );
void lpPhys_SetRopeLength( lpPhys* p, lpPhysJoint joint, float length );

// ---- reports, in our own order ----

// A touching contact point of a body
typedef struct lpPhysContact
{
	int piece;		  // the body's own piece
	int other;		  // the piece it touches, -1 for a shape that is none
	lpVec3 normal;	  // unit: the way the contact pushes the piece
	lpPos point;	  // world
	float separation; // negative when overlapping
	float impulse;	  // total normal impulse over the last step, N s
} lpPhysContact;

// A body's contact points, sorted by (piece, other); within a contact, the engine's point order (a second backend
// orders its own points, so it agrees with this one within tolerances, not bit for bit)
int lpPhys_GetBodyContacts( lpPhys* p, lpPhysBody body, const lpPhysContact** contacts );

// A contact that began at hitSpeed or faster in the last step
typedef struct lpPhysHit
{
	uint64_t pair; // ((lower piece + 1) << 32) | (higher piece + 1); 0 for a shape that is no piece
	int pieceA;	   // -1 for none
	int pieceB;
	float speed; // approach speed
	lpPos point;
} lpPhysHit;

// The last step's hits, sorted by pair, then speed (fastest first), then point, then report order (identical hits)
int lpPhys_GetHits( lpPhys* p, const lpPhysHit** hits );
// The mean of the points of the contact a hit began (hit: its index in lpPhys_GetHits' array, until the next step);
// false if the contact is gone or has none
bool lpPhys_GetContactCentroid( const lpPhys* p, int hit, lpPos* point );

typedef struct lpPhysMove
{
	int body; // its index
	lpWorldTransform transform;
	bool fellAsleep;
} lpPhysMove;

// The bodies with a body index that the last step moved (or put to sleep), sorted by that index
int lpPhys_GetMoves( lpPhys* p, const lpPhysMove** moves );

// ---- queries ----

// The pieces whose shapes' (fattened) bounds overlap the box, sorted and unique
int lpPhys_OverlapBox( lpPhys* p, lpAABB box, lpPhysFilter filter, const int** pieces );

typedef struct lpPhysCastHit
{
	bool hit;
	float fraction; // of the translation
	lpPos point;
	lpVec3 normal; // zero when the cast starts inside the shape
	int piece;	   // -1 for a shape that is none
	int material;
} lpPhysCastHit;

// Whether a cast hit counts (false: the cast passes through that shape). Called back from the cast, so on the CPU:
// for a GPU backend these become data too (bodies to skip, start-inside hits ignored), milestone 12.
typedef bool lpPhysCastAcceptFcn( int piece, float fraction, void* context );

// The closest accepted hit; at equal fractions the lower piece wins, not the engine's traversal order. A NULL accept
// takes every hit.
lpPhysCastHit lpPhys_CastRay( const lpPhys* p, lpPos origin, lpVec3 translation, lpPhysFilter filter,
							  lpPhysCastAcceptFcn* accept, void* context );
// A convex shape (points around origin, rounded by radius) swept along translation; as lpPhys_CastRay
lpPhysCastHit lpPhys_CastShape( const lpPhys* p, lpPos origin, const lpVec3* points, int count, float radius,
								lpVec3 translation, lpPhysFilter filter, lpPhysCastAcceptFcn* accept, void* context );
