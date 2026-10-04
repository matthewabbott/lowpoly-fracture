// rows_glue.cpp: the C++ twin of rows.slang (slangc -target cpp output included here).
#include TWIN_GEN
#include "twin_info.h"

static GlobalParams_0 g_base;

extern "C" void rtwin_bind( void* rows, size_t n, void* outs, void* counters )
{
	g_base.rows_0.data = (RRow_0*)rows;
	g_base.rows_0.count = n;
	g_base.outs_0.data = (ROut_0*)outs;
	g_base.outs_0.count = n;
	g_base.counters_0.data = (uint32_t*)counters;
	g_base.counters_0.count = 4;
}

extern "C" const char* rtwin_info( void )
{
	return TWIN_INFO;
}

extern "C" void rtwin_run( uint32_t count, uint32_t g0, uint32_t g1 )
{
	GlobalParams_0 gp = g_base;
	RPush_0 push = {};
	push.count_0 = count;
	gp.rp_0 = &push;
	ComputeVaryingInput vi;
	vi.startGroupID.x = g0;
	vi.startGroupID.y = 0;
	vi.startGroupID.z = 0;
	vi.endGroupID.x = g1;
	vi.endGroupID.y = 1;
	vi.endGroupID.z = 1;
	rowMain( &vi, nullptr, &gp );
}
