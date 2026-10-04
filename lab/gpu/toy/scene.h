// scene.h: the toy's scenes, in double with + - * / and sqrt only (DESIGN.md "Scenes and acceptance"), from fixed
// seeds with every random draw in its own statement. Hulls are Box3D-style half-edge hulls about the body's centre of
// mass; boxes and chunks (a box cut by one to three random planes) come from one half-space intersection. Shared by the
// twins and, later, by b3ref2.
#ifndef TOY_SCENE_H
#define TOY_SCENE_H

#include <stdint.h>

#define SCENE_MAX_VERTS 16
#define SCENE_MAX_FACES 24
#define SCENE_MAX_EDGES 48

typedef struct SceneHull
{
	int vertexCount, faceCount, edgeCount;
	double v[SCENE_MAX_VERTS][3];	   // about the centre of mass
	double normal[SCENE_MAX_FACES][3]; // face planes n . x = offset (outward)
	double offset[SCENE_MAX_FACES];
	int faceEdge[SCENE_MAX_FACES];			// a half-edge of each face
	uint8_t edge[SCENE_MAX_EDGES][4];		// next, twin, origin, face (twins adjacent: 2k, 2k + 1)
	double volume;							// m^3
	double inertia[6];						// per unit density, about the centre of mass: xx, xy, xz, yy, yz, zz
	double boundsCenter[3], boundsHalf[3];	// local AABB
	double maxExtent;						// farthest vertex from the centre of mass
	double minExtent;						// nearest face plane
} SceneHull;

typedef struct SceneBody
{
	int hull;
	int isStatic;
	double p[3]; // centre of mass
	double q[4]; // x, y, z, s
	double v[3], w[3];
	double mass, invMass;
	double invI[6]; // local inverse inertia about the centre of mass (xx, xy, xz, yy, yz, zz)
	double friction, restitution;
} SceneBody;

// A revolute joint (step 6), as Box3D's b3RevoluteJointDef: the hinge point and the joint frame in each body's frame (the
// frame's z axis is the hinge; the twist angle is B's rotation about it relative to A), limits, a speed motor with a
// torque cap, and a servo: the motor's speed each step is gain (target - angle), capped, with the target from the scene's
// rule (scene_servo_target). Jointed bodies do not collide (Box3D's collideConnected false).
typedef struct SceneJoint
{
	int bodyA, bodyB;
	double localAnchorA[3], localAnchorB[3]; // from each body's centre of mass
	double localFrameA[4], localFrameB[4];	 // x, y, z, s
	int enableLimit;
	double lowerAngle, upperAngle; // rad
	int enableMotor;
	double maxMotorTorque; // N m
	double motorSpeed;	   // rad/s (a speed motor, when not a servo)
	int servo;
	double servoGain;	   // 1/s
	double servoMaxSpeed;  // rad/s
	double servoAmplitude; // the target: a triangle wave of this amplitude (rad) and period (ticks), 0 at tick 0, rising
	int servoPeriod;
} SceneJoint;

typedef struct Scene
{
	char name[32];
	uint64_t seed;
	int bodyCount, hullCount, jointCount;
	SceneBody* bodies;
	SceneHull* hulls;
	SceneJoint* joints;
	int chunkCount;	 // irregular hulls
	int chunkRedraws; // chunks drawn again (too many features, or a feature too small)
	int startMoved;	  // start values the grid moved off their float (scene_round_start)
} Scene;

// stack10, pile200, bounce, ramp, ratio, chip, arm, grid:K (K copies of pile200 side by side). Returns 0 for an unknown
// name. The start (scene_round_start) is one every dialect holds exactly.
int scene_build( Scene* s, const char* name, uint64_t seed );

// A servo's target angle before step `tick` (1, 2, ...): the triangle wave, in double (+ - * / only)
double scene_servo_target( const SceneJoint* j, int tick );

// The joint's definition as one line, "joint K: bodies A B anchorA ... servo ..." (the toy and b3ref2 print it; metrics.py
// reads it to measure angles, gaps and overshoots from the trajectories)
void scene_joint_text( const SceneJoint* j, int k, char* out, int size );

// Every body's start (position, orientation, velocities) rounded to float, then to V4's grid for that quantity (layout.h:
// positions 2^-32, orientations 2^-30, linear velocities 2^-22, angular velocities 2^-20), so F, V4 and D (and Box3D)
// start from the same values: the grid moves only floats smaller than 2^(23 - bits), and what it gives is still a float.
// Returns the number of values the grid moved off their float (pile200's small quaternion components).
#define SCENE_GRID_P 32
#define SCENE_GRID_Q 30
#define SCENE_GRID_V 22
#define SCENE_GRID_W 20
int scene_round_start( Scene* s );
void scene_free( Scene* s );
const char* scene_names( void );

// A convex hull from half-spaces n . x <= d (n unit): vertices from plane triples, faces sorted counter-clockwise from
// outside, twin half-edges, mass properties; the points are moved so the centre of mass is the origin and the shift is
// returned in `center`. Returns 0 when the hull exceeds the limits, has an edge shorter than minEdge, or is degenerate.
int scene_hull_from_planes( SceneHull* h, const double ( *n )[3], const double* d, int planeCount, double minEdge, double center[3] );

// A box of half extents hx, hy, hz
void scene_box_hull( SceneHull* h, double hx, double hy, double hz );

// A box cut by one to three random planes through its inside (each keeps the side holding the box's centre), drawn
// again until the hull fits the limits and has no edge under 2 cm; returns the number of redraws
struct Pcg;
int scene_chunk_hull( SceneHull* h, struct Pcg* r, double hx, double hy, double hz );

#endif
