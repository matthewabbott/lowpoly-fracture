// SPDX-License-Identifier: MIT
// The stress system of one structure and its solver. Nodes have 6 degrees of freedom (a small translation and
// rotation), edges are short beams between two nodes or a node and the world, and K x = f is solved by conjugate
// gradient with a block-Jacobi preconditioner. Pure math on the system's own arrays, with no world: it runs inside
// parallel jobs, and tests can build systems by hand. stress.c builds systems from structures and judges solutions.

#pragma once

#include "core.h"

// A 6-vector per node: force and torque, or translation and rotation
typedef struct lpVec6
{
	b3Vec3 f;
	b3Vec3 t;
} lpVec6;

// A bond as a short beam between two nodes (node -1: fixed to the world)
typedef struct lpStressEdge
{
	int a, b;
	int bond;
	b3Vec3 ra, rb; // contact centroid from each node's reference point
	b3Vec3 n, t1, t2;
	float kn, ks, kb1, kb2, kt; // axial, shear, bending about t1 and t2, twist
} lpStressEdge;

// One node's 6x6 block of the stiffness, then its Cholesky factor
typedef struct lpBlock6
{
	float m[6][6];
} lpBlock6;

// One structure's system. It lives on the structure's body and is rebuilt when the body's topology changes, so a solve
// that continues across steps does not rebuild it.
typedef struct lpStressSystem
{
	uint32_t topology; // of the body when built
	bool built;
	float forceScale;			 // loads and solutions are in units of this force
	LP_ARRAY( int ) nodes;		 // piece of each node
	LP_ARRAY( lpStressEdge ) edges;
	LP_ARRAY( lpVec6 ) vectors;	 // x, f, r, z, p, q: six per node
	LP_ARRAY( lpBlock6 ) blocks; // each node's factored block
	LP_ARRAY( int ) incidentStart; // the edges at node i are incident[incidentStart[i] .. incidentStart[i + 1]), in edge order
	LP_ARRAY( int ) incident;
	LP_ARRAY( float ) rho; // each edge's utilization at the last converged solve
} lpStressSystem;

// Relative motion of an edge's two sides at the contact, and the elastic force and moment it produces (on side b; side a
// gets the opposite)
void lpEdgeForce( const lpStressEdge* e, const lpVec6* x, b3Vec3* force, b3Vec3* moment );

// y = K x, matrix free, in edge order
void lpSystemApply( const lpStressSystem* s, const lpVec6* x, lpVec6* y );

// Sizes the vectors and blocks for the nodes; x and f are left for the caller to fill
void lpSystemResize( lpStressSystem* s );

// Each node's block of K from the edges, Cholesky-factored (the preconditioner), and the incident-edge lists
void lpSystemFactor( lpStressSystem* s );

typedef struct lpSolveState
{
	double rz;
	int iterations;
	bool converged;
} lpSolveState;

// Preconditioned conjugate gradient, at most `budget` iterations, until |r| <= tolerance |f|. A continuing solve picks
// up r, p (in the system) and rz (in the state) where the last call left them; otherwise it starts from x.
void lpSystemSolve( lpStressSystem* s, int budget, double tolerance, bool continuing, lpSolveState* state );

void lpSystemFree( lpStressSystem* s );
