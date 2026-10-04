// kernels_glue.cpp: the toy's CPU twin. Includes the C++ that slangc generated from kernels.slang for one dialect
// (TWIN_GEN, set by CMake's wrapper file) and runs one dispatch over a range of workgroups. Each pool thread gets its
// own saturation counters (the twin has no atomics); the driver sums them.
#include TWIN_GEN
#include "twin_info.h"

static GlobalParams_0 g_base;

#define TWIN_MAX_THREADS 64
static uint32_t g_counters[TWIN_MAX_THREADS][4];

// The buffers, in kernels.slang's binding order 0..10 (hulls, hullPoints, hullFaces, hullEdges, bodyState, bodyPose,
// bodyMass, aabbs, lists, params, hashes) with their element counts
extern "C" void toy_bind( void* const* bufs, const size_t* counts )
{
	g_base.hulls_0.data = (Hull_0*)bufs[0];
	g_base.hulls_0.count = counts[0];
	g_base.hullPoints_0.data = (V3_0*)bufs[1];
	g_base.hullPoints_0.count = counts[1];
	g_base.hullFaces_0.data = (HullFace_0*)bufs[2];
	g_base.hullFaces_0.count = counts[2];
	g_base.hullEdges_0.data = (uint32_t*)bufs[3];
	g_base.hullEdges_0.count = counts[3];
	g_base.bodyState_0.data = (BodyState_0*)bufs[4];
	g_base.bodyState_0.count = counts[4];
	g_base.bodyPose_0.data = (BodyPose_0*)bufs[5];
	g_base.bodyPose_0.count = counts[5];
	g_base.bodyMass_0.data = (BodyMass_0*)bufs[6];
	g_base.bodyMass_0.count = counts[6];
	g_base.aabbs_0.data = (Aabb_0*)bufs[7];
	g_base.aabbs_0.count = counts[7];
	g_base.lists_0.data = (uint32_t*)bufs[8];
	g_base.lists_0.count = counts[8];
	g_base.params_0.data = (Params_0*)bufs[9];
	g_base.params_0.count = counts[9];
	g_base.hashes_0.data = (Hash2_0*)bufs[10];
	g_base.hashes_0.count = counts[10];
}

extern "C" const char* toy_twin_info( void )
{
	return TWIN_INFO;
}

// sizes of the twin's structs, against the C layout: Hull, HullFace, BodyState, BodyPose, BodyMass, Aabb, Params,
// Hash2, V3
extern "C" size_t toy_sizeof( int which )
{
	switch ( which )
	{
		case 0: return sizeof( Hull_0 );
		case 1: return sizeof( HullFace_0 );
		case 2: return sizeof( BodyState_0 );
		case 3: return sizeof( BodyPose_0 );
		case 4: return sizeof( BodyMass_0 );
		case 5: return sizeof( Aabb_0 );
		case 6: return sizeof( Params_0 );
		case 7: return sizeof( Hash2_0 );
		case 8: return sizeof( V3_0 );
		default: return 0;
	}
}

extern "C" uint32_t toy_saturations( void )
{
	uint32_t n = 0;
	for ( int t = 0; t < TWIN_MAX_THREADS; ++t )
	{
		n += g_counters[t][0];
	}
	return n;
}

extern "C" void toy_reset_saturations( void )
{
	for ( int t = 0; t < TWIN_MAX_THREADS; ++t )
	{
		g_counters[t][0] = 0;
	}
}

// entry: 0 prepareBodies, 1 integrateVelocities, 2 integratePositions, 3 finalizeBodies, 4 wakeBodies, 5 hashElements
extern "C" void toy_run( int entry, uint32_t start, uint32_t count, uint32_t aux, uint32_t g0, uint32_t g1, int thread )
{
	GlobalParams_0 gp = g_base;
	gp.counters_0.data = g_counters[thread];
	gp.counters_0.count = 4;
	Push_0 push = {};
	push.start_0 = start;
	push.count_0 = count;
	push.aux_0 = aux;
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
		case 0: prepareBodies( &vi, nullptr, &gp ); break;
		case 1: integrateVelocities( &vi, nullptr, &gp ); break;
		case 2: integratePositions( &vi, nullptr, &gp ); break;
		case 3: finalizeBodies( &vi, nullptr, &gp ); break;
		case 4: wakeBodies( &vi, nullptr, &gp ); break;
		case 5: hashElements( &vi, nullptr, &gp ); break;
	}
}
