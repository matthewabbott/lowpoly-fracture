// twin_glue.cpp: the CPU twin. Includes the C++ that slangc generated from solver.slang (same source
// as the GPU kernels) and exposes a C entry that runs one dispatch over a range of workgroups.
// Compiled without contraction (MSVC /fp:precise; gcc and clang -ffp-contract=off), except the
// positive-control builds (LAB_FMA).
#include TWIN_GEN
#include "twin_info.h"

static GlobalParams_0 g_base;

extern "C" void twin_bind( void* bodies, size_t nb, void* poses, size_t np, void* cons, size_t nc, void* imp, size_t ni, void* params,
						   void* counters )
{
	g_base.bodies_0.data = (Body_0*)bodies;
	g_base.bodies_0.count = nb;
	g_base.poses_0.data = (Pose_0*)poses;
	g_base.poses_0.count = np;
	g_base.constraints_0.data = (Constraint_0*)cons;
	g_base.constraints_0.count = nc;
	g_base.impulses_0.data = (Impulse_0*)imp;
	g_base.impulses_0.count = ni;
	g_base.params_0.data = (Params_0*)params;
	g_base.params_0.count = 1;
	g_base.counters_0.data = (uint32_t*)counters;
	g_base.counters_0.count = 4;
}

extern "C" const char* twin_info( void )
{
	return TWIN_INFO;
}

extern "C" size_t twin_sizeof( int which )
{
	switch ( which )
	{
		case 0: return sizeof( Body_0 );
		case 1: return sizeof( Pose_0 );
		case 2: return sizeof( Constraint_0 );
		case 3: return sizeof( Impulse_0 );
		case 4: return sizeof( Params_0 );
		default: return 0;
	}
}

// entry: 0 integrateVelocities, 1 warmStart, 2 push, 3 integratePositions, 4 relax, 5 finalizeBodies
extern "C" void twin_run( int entry, uint32_t start, uint32_t count, uint32_t g0, uint32_t g1 )
{
	GlobalParams_0 gp = g_base;
	Push_0 push = {};
	push.start_0 = start;
	push.count_0 = count;
	gp.pc_0 = &push;
	ComputeVaryingInput vi;
	vi.startGroupID.x = g0;
	vi.startGroupID.y = 0;
	vi.startGroupID.z = 0;
	vi.endGroupID.x = g1;
	vi.endGroupID.y = 1;
	vi.endGroupID.z = 1;
	switch ( entry )
	{
		case 0: integrateVelocities( &vi, nullptr, &gp ); break;
		case 1: warmStart( &vi, nullptr, &gp ); break;
		case 2: ::push( &vi, nullptr, &gp ); break;
		case 3: integratePositions( &vi, nullptr, &gp ); break;
		case 4: relax( &vi, nullptr, &gp ); break;
		case 5: finalizeBodies( &vi, nullptr, &gp ); break;
	}
}
