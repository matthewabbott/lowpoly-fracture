// battery_glue.cpp: the battery's CPU twin. Includes the C++ that slangc generated from battery.slang for one dialect
// (TWIN_GEN, set by CMake's wrapper file) and runs one dispatch over a range of workgroups.
#include TWIN_GEN
#include "twin_info.h"

static GlobalParams_0 g_base;

extern "C" void bat_bind( void* in, void* out, size_t n, uint32_t* counters )
{
	g_base.batIn_0.data = (BatIn_0*)in;
	g_base.batIn_0.count = n;
	g_base.batOut_0.data = (BatOut_0*)out;
	g_base.batOut_0.count = n;
	g_base.counters_0.data = counters;
	g_base.counters_0.count = 4;
}

extern "C" const char* bat_info( void )
{
	return TWIN_INFO;
}

extern "C" size_t bat_sizeof( int which )
{
	return which == 0 ? sizeof( BatIn_0 ) : sizeof( BatOut_0 );
}

extern "C" void bat_run( uint32_t start, uint32_t count, uint32_t helper, uint32_t g0, uint32_t g1 )
{
	GlobalParams_0 gp = g_base;
	Push_0 push = {};
	push.start_0 = start;
	push.count_0 = count;
	push.aux_0 = helper;
	gp.pc_0 = &push;
	ComputeVaryingInput vi;
	vi.startGroupID.x = g0;
	vi.startGroupID.y = 0;
	vi.startGroupID.z = 0;
	vi.endGroupID.x = g1;
	vi.endGroupID.y = 1;
	vi.endGroupID.z = 1;
	battery( &vi, nullptr, &gp );
}
