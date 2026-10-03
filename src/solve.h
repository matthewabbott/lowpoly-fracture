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
	lpVec3 f;
	lpVec3 t;
} lpVec6;

// A bond as a short beam between two nodes (node -1: fixed to the world)
typedef struct lpStressEdge
{
	int a, b;
	int bond;
	lpVec3 ra, rb; // contact centroid from each node's reference point
	lpVec3 n, t1, t2;
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
	bool factored; // its blocks are factored (a system solved for a correction through a reduced one has none)
	float forceScale;			 // loads and solutions are in units of this force
	LP_ARRAY( int ) nodes;		 // piece of each node
	LP_ARRAY( lpStressEdge ) edges;
	LP_ARRAY( lpVec6 ) vectors;	 // x, f, r, z, p, q: six per node
	LP_ARRAY( lpBlock6 ) blocks; // each node's factored block
	LP_ARRAY( int ) incidentStart; // the edges at node i are incident[incidentStart[i] .. incidentStart[i + 1]), in edge order
	LP_ARRAY( int ) incident;
	LP_ARRAY( float ) rho; // each edge's utilization at the last converged solve
	// Optional (empty: not checked): each node's force scale (its load plus the forces through it) and arm (its size),
	// for the per-node equilibrium test
	LP_ARRAY( float ) nodeScale;
	LP_ARRAY( float ) nodeArm;
	double loadNorm2; // > 0: tolerances are relative to this squared load instead of |f|^2 (a correction's system)
	// Region solves (lpSystemSolveFront): each node's depth, 0 in the region (the caller fills it for every solve); the
	// region's nodes, the edges with an end in it and the held nodes just past it, found when a solve starts and kept
	// while it continues (a cache of depth: assert builds check that they agree)
	LP_ARRAY( int ) depth;
	LP_ARRAY( int ) activeNodes;
	LP_ARRAY( int ) activeEdges;
	LP_ARRAY( int ) boundary;
	int depthLimit;
} lpStressSystem;

// A partition of a system's nodes into groups that move rigidly. A group of one is its node; a bigger one is a rigid
// cluster, moving with its reference point: node i of group g translates by U + Theta x (nodeRef[i] - ref[g]) and
// turns by Theta. P maps group motions to node motions; P^T maps node forces to group resultants.
typedef struct lpPartition
{
	int groupCount;
	LP_ARRAY( int ) group;	 // group of each node
	LP_ARRAY( lpVec3 ) ref;	 // reference point of each group
	LP_ARRAY( int ) members; // nodes in each group (a group of one keeps its node's reference exactly)
} lpPartition;

// Relative motion of an edge's two sides at the contact, and the force and moment the bond carries: tension positive
// along n, so the bond pulls side b by -force (and -moment) and side a by +force
void lpEdgeForce( const lpStressEdge* e, const lpVec6* x, lpVec3* force, lpVec3* moment );

// y = K x, matrix free, in edge order
void lpSystemApply( const lpStressSystem* s, const lpVec6* x, lpVec6* y );

// Sizes the vectors for the nodes (lpSystemFactor sizes the blocks); x and f are left for the caller to fill
void lpSystemResize( lpStressSystem* s );

// Each node's block of K from the edges, Cholesky-factored (the preconditioner)
void lpSystemFactor( lpStressSystem* s );

// The edges at each node, in edge order
void lpSystemIncidence( lpStressSystem* s );

// The reduced system P^T K P of a fine one: an edge between two groups keeps its stiffness and axes with its arms moved
// to the groups' reference points; an edge inside a group cannot deform (the group is rigid) and is dropped. The
// reduced nodes are the groups (each lists its first node's piece). Sized and factored; x and f are the caller's. With
// every group of one node it is the fine system, bit for bit.
void lpSystemReduce( const lpStressSystem* fine, const lpVec3* nodeRef, const lpPartition* part, lpStressSystem* reduced );

// reduced = P^T fine: each group's resultant force, and torque about its reference point
void lpPartitionRestrict( const lpPartition* part, const lpVec3* nodeRef, const lpVec6* fine, int nodeCount, lpVec6* reduced );

// fine += P y: every node moves with its group
void lpPartitionProlong( const lpPartition* part, const lpVec3* nodeRef, const lpVec6* y, int nodeCount, lpVec6* fine );

void lpPartitionFree( lpPartition* part );

typedef struct lpSolveState
{
	double rz;
	int iterations;
	bool converged;
} lpSolveState;

// Preconditioned conjugate gradient, at most `budget` iterations, until |r| <= tolerance |f| and every node is in
// equilibrium to nodeTolerance of its scale: a global norm alone lets the residual gather on a few small nodes. A
// continuing solve picks up r, p (in the system) and rz (in the state) where the last call left them; otherwise it
// starts from x.
void lpSystemSolve( lpStressSystem* s, int budget, double tolerance, float nodeTolerance, bool continuing, lpSolveState* state );

// A region solve: only the nodes within `limit` bonds of the front (depth 0) move; those past it are held where they
// were. The caller judges it only once what the region's change puts on the held nodes is within tolerance.
void lpSystemSolveFront( lpStressSystem* s, int limit, int budget, double tolerance, float nodeTolerance, bool continuing,
						 lpSolveState* state );

// Each node's scale for the equilibrium test from the current x and f: its own load plus the magnitude of the edge
// forces on it (plus floor); arms are the caller's
void lpSystemNodeScales( lpStressSystem* s, float floor );

void lpSystemFree( lpStressSystem* s );
