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

typedef struct Scene
{
	char name[32];
	uint64_t seed;
	int bodyCount, hullCount;
	SceneBody* bodies;
	SceneHull* hulls;
	int chunkCount;	 // irregular hulls
	int chunkRedraws; // chunks drawn again (too many features, or a feature too small)
} Scene;

// stack10, pile200, bounce, ramp, ratio, chip. Returns 0 for an unknown name.
int scene_build( Scene* s, const char* name, uint64_t seed );
void scene_free( Scene* s );
const char* scene_names( void );

// A convex hull from half-spaces n . x <= d (n unit): vertices from plane triples, faces sorted counter-clockwise from
// outside, twin half-edges, mass properties; the points are moved so the centre of mass is the origin and the shift is
// returned in `center`. Returns 0 when the hull exceeds the limits, has an edge shorter than minEdge, or is degenerate.
int scene_hull_from_planes( SceneHull* h, const double ( *n )[3], const double* d, int planeCount, double minEdge, double center[3] );

#endif
