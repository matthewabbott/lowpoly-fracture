// toy.c: the toy's twin driver (DESIGN.md "Pipeline per tick", steps 2 to 4). Builds a scene (double), quantises it
// into the dialect, and runs N ticks: the GPU stages as the kernels' C++ twin on a thread pool (E11's), the CPU stages
// between them (stages.c, with last tick's touching pairs from the manifolds), the narrowphase over the active pairs
// (manifolds ping-ponged, an inactive pair's carried), the contact solve (prepare; per substep integrate velocities,
// warm start, push, integrate positions, relax, per colour; restitution; the impulses stored back), a per-tick hash
// (the bodies' and the manifolds' element hashes summed order-free per category, and the stages' decisions), the
// floating-point sentinel after every dispatch. Built once per dialect (toy_F, toy_V4, toy_D).
//
//   toy_F --scene pile200 --ticks 600 --threads 1,8 [--seed S] [--gravity G] [--sleep 0|1] [--recycle 0|1]
//         [--push BODY,VX,VY,VZ] [--perturb EPS] [--start-float]
//         [--pos-out FILE] [--pos-ref FILE] [--traj FILE [--traj-every N]] [--log FILE] [--quiet]
//
// --pos-out writes every body's pose per tick (double); --pos-ref reads another dialect's (D's) and reports the
// differences (rms, max) of the dynamic bodies' positions and orientations. --traj writes the first run's trajectory
// (traj.h's LPTRAJ1: tick 0, then every Nth tick). --perturb moves every dynamic body's start by EPS m along x and
// along y (in the scene's doubles, before quantisation: step 4b's measure of how much a scene amplifies a tiny
// difference); --start-float rounds every body's start (position, orientation, velocities) to float first, the start F
// stores (D from F's start). TOY_DUMP_AABBS=T prints every body's AABB at tick T (1-thread runs).
// The dispatch list is a plain array, so step 5 can replay it on a GPU.
#include "fpflags.h"
#include "layout.h"
#include "quant.h"
#include "scene.h"
#include "stages.h"
#include "traj.h"
#include "vk_util.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#endif

#if defined( DIALECT_F )
#define DIALECT_NAME "F"
#elif defined( DIALECT_V4 )
#define DIALECT_NAME "V4"
#else
#define DIALECT_NAME "D"
#endif

#define TOY_BUFFERS 19 // kernels.slang's bindings 0..18 (11, the counters, is the glue's)
void toy_bind( void* const* bufs, const size_t* counts );
const char* toy_twin_info( void );
size_t toy_sizeof( int which );
uint32_t toy_saturations( void );
void toy_reset_saturations( void );
void toy_run( int entry, uint32_t start, uint32_t count, uint32_t aux, uint32_t g0, uint32_t g1, int thread );

enum
{
	E_PREPARE,
	E_INTVEL,
	E_INTPOS,
	E_FINAL,
	E_WAKE,
	E_HASH,
	E_NSAT,
	E_NCLIP,
	E_NCOPY,
	E_MHASH,
	E_PREPC,
	E_WARM,
	E_PUSH,
	E_RELAX,
	E_REST,
	E_STORE,
	E_COUNT
};
static const char* g_entryNames[E_COUNT] = { "prepareBodies", "integrateVelocities", "integratePositions", "finalizeBodies",
											 "wakeBodies",	  "hashElements",		 "narrowSat",		   "narrowClip",
											 "copyManifolds", "hashManifolds",		 "prepareContacts",	   "warmStart",
											 "pushContacts",  "relaxContacts",		 "restitution",		   "storeImpulses" };

static FILE* g_log;
static int g_quiet;

static void say( const char* fmt, ... )
{
	va_list a;
	if ( !g_quiet )
	{
		va_start( a, fmt );
		vprintf( fmt, a );
		va_end( a );
	}
	if ( g_log )
	{
		va_start( a, fmt );
		vfprintf( g_log, fmt, a );
		va_end( a );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// The dispatch list and the twin's thread pool (E11's: one barrier per dispatch)
// ---------------------------------------------------------------------------------------------------------------------

typedef struct Dispatch
{
	int entry;
	uint32_t start, count, aux; // start and count into the list buffer
} Dispatch;

#if defined( _WIN32 )
typedef volatile LONG PoolWord;
#define POOL_LOAD( w ) ( *( w ) )
#define POOL_STORE( w, v ) InterlockedExchange( ( w ), ( v ) )
#define POOL_INC( w ) InterlockedIncrement( w )
#define POOL_PAUSE() YieldProcessor()
typedef HANDLE PoolThread;
#else
typedef atomic_long PoolWord;
#define POOL_LOAD( w ) atomic_load_explicit( ( w ), memory_order_acquire )
#define POOL_STORE( w, v ) atomic_store_explicit( ( w ), ( v ), memory_order_release )
#define POOL_INC( w ) atomic_fetch_add_explicit( ( w ), 1, memory_order_acq_rel )
#define POOL_PAUSE() sched_yield()
typedef pthread_t PoolThread;
#endif

#define MAX_THREADS 64

typedef struct Pool
{
	int threads;
	PoolThread handles[MAX_THREADS];
	PoolWord generation;
	PoolWord done;
	PoolWord quit;
	Dispatch job;
	int fpFlags[MAX_THREADS]; // each thread's floating-point status after its chunk
} Pool;

static Pool g_pool;

static void run_chunk( const Dispatch* d, int t, int threads )
{
	uint32_t groups = ( d->count + 63 ) / 64;
	uint32_t g0 = (uint32_t)( ( (uint64_t)groups * (uint64_t)t ) / (uint64_t)threads );
	uint32_t g1 = (uint32_t)( ( (uint64_t)groups * (uint64_t)( t + 1 ) ) / (uint64_t)threads );
	fp_clear();
	if ( g1 > g0 )
	{
		toy_run( d->entry, d->start, d->count, d->aux, g0, g1, t );
	}
	g_pool.fpFlags[t] = fp_flags();
}

static void pool_work( int t )
{
	long seen = 0;
	for ( ;; )
	{
		long g;
		while ( ( g = POOL_LOAD( &g_pool.generation ) ) == seen )
		{
			if ( POOL_LOAD( &g_pool.quit ) )
			{
				return;
			}
			POOL_PAUSE();
		}
		seen = g;
		Dispatch d = g_pool.job;
		run_chunk( &d, t, g_pool.threads );
		POOL_INC( &g_pool.done );
	}
}

#if defined( _WIN32 )
static DWORD WINAPI pool_worker( LPVOID arg )
{
	pool_work( (int)(intptr_t)arg );
	return 0;
}
#else
static void* pool_worker( void* arg )
{
	pool_work( (int)(intptr_t)arg );
	return NULL;
}
#endif

static void pool_start( int threads )
{
	memset( &g_pool, 0, sizeof( g_pool ) );
	g_pool.threads = threads;
	for ( int t = 1; t < threads; ++t )
	{
#if defined( _WIN32 )
		g_pool.handles[t] = CreateThread( NULL, 0, pool_worker, (LPVOID)(intptr_t)t, 0, NULL );
#else
		pthread_create( &g_pool.handles[t], NULL, pool_worker, (void*)(intptr_t)t );
#endif
	}
}

static void pool_stop( void )
{
	POOL_STORE( &g_pool.quit, 1 );
	for ( int t = 1; t < g_pool.threads; ++t )
	{
#if defined( _WIN32 )
		WaitForSingleObject( g_pool.handles[t], INFINITE );
		CloseHandle( g_pool.handles[t] );
#else
		pthread_join( g_pool.handles[t], NULL );
#endif
	}
}

// Runs one dispatch on every pool thread; returns the threads' floating-point status, OR-ed
static int pool_dispatch( const Dispatch* d )
{
	if ( g_pool.threads <= 1 )
	{
		run_chunk( d, 0, 1 );
		return g_pool.fpFlags[0];
	}
	g_pool.job = *d;
	POOL_STORE( &g_pool.done, 0 );
	POOL_INC( &g_pool.generation ); // releases the job and the reset count
	run_chunk( d, 0, g_pool.threads );
	while ( POOL_LOAD( &g_pool.done ) < g_pool.threads - 1 )
	{
		POOL_PAUSE();
	}
	int m = 0;
	for ( int t = 0; t < g_pool.threads; ++t )
	{
		m |= g_pool.fpFlags[t];
	}
	return m;
}

// ---------------------------------------------------------------------------------------------------------------------
// One run
// ---------------------------------------------------------------------------------------------------------------------

typedef struct TickRecord
{
	uint64_t bodies;	// the bodies' element hashes, summed
	uint64_t manifolds; // the manifolds' element hashes, summed
	uint64_t stages;	// the CPU stages' decisions
	int awake, pairs, active, colours, overflow, islands, largest, slept, woken, touching, points;
	double minSep; // the deepest manifold point this tick (the narrowphase's separation, before the solve), m
	int minSepA, minSepB;
} TickRecord;

typedef struct RunResult
{
	TickRecord* ticks; // [1, ticks]
	double* pos;	   // per tick, per body, xyz (when kept)
	double msPerTick;
	uint32_t saturations;
	int fpFailures;
	char firstFp[160];
	int dispatches; // the most in one tick
	// after the last tick, over every touching pair (sleeping ones too, their manifolds carried): the deepest point
	double finalSep;
	int finalA, finalB, finalTouching;
} RunResult;

static void push_dispatch( Dispatch* list, int* n, int entry, uint32_t start, uint32_t count )
{
	if ( count == 0 )
	{
		return;
	}
	list[*n].entry = entry;
	list[*n].start = start;
	list[*n].count = count;
	list[*n].aux = entry == E_HASH ? 1u : 0u;
	*n += 1;
}

// One solver stage over the coloured pairs (the lists at base, in colourList's order): the overflow colour's pairs one
// dispatch each (they may share bodies), first, as Box3D solves its overflow constraints before the colours; then one
// dispatch per colour
static void push_colours( Dispatch* list, int* n, int entry, const Stages* st, uint32_t base )
{
	for ( int i = st->colourStart[STAGE_MAX_COLOURS]; i < st->colourStart[STAGE_MAX_COLOURS + 1]; ++i )
	{
		push_dispatch( list, n, entry, base + (uint32_t)i, 1 );
	}
	for ( int c = 0; c < st->colourCount; ++c )
	{
		push_dispatch( list, n, entry, base + (uint32_t)st->colourStart[c], (uint32_t)( st->colourStart[c + 1] - st->colourStart[c] ) );
	}
}

static void run_list( const Dispatch* list, int n, int tick, RunResult* r )
{
	for ( int i = 0; i < n; ++i )
	{
		int m = pool_dispatch( list + i );
		if ( m )
		{
			if ( r->fpFailures == 0 )
			{
				char names[96];
				snprintf( r->firstFp, sizeof( r->firstFp ), "tick %d dispatch %d (%s start %u count %u): %s", tick, i, g_entryNames[list[i].entry],
						  list[i].start, list[i].count, fp_names( m, names, sizeof( names ) ) );
			}
			r->fpFailures += 1;
		}
	}
}

// The recorded pose per body per tick: position and quaternion (x, y, z, s), in double
#define POSW 7

static void record_poses( double* out, const BodyPose* pose, int n )
{
	for ( int i = 0; i < n; ++i )
	{
		double* p = out + (size_t)i * POSW;
		p[0] = toy_pos_x( &pose[i].p );
		p[1] = toy_pos_y( &pose[i].p );
		p[2] = toy_pos_z( &pose[i].p );
		p[3] = toy_val( pose[i].q.x, S_Q );
		p[4] = toy_val( pose[i].q.y, S_Q );
		p[5] = toy_val( pose[i].q.z, S_Q );
		p[6] = toy_val( pose[i].q.s, S_Q );
	}
}

// One trajectory record: every body's pose and velocities converted exactly to double, awake (stepped this tick), static.
// A body not stepped this tick (asleep, static) has zero velocities, as Box3D reports them (the toy's state keeps the
// last ones until the body wakes).
static void record_traj( TrajWriter* w, TrajBody* buf, int tick, const BodyState* state, const BodyPose* pose, const uint8_t* isStatic,
						 const uint8_t* awake, int n )
{
	for ( int i = 0; i < n; ++i )
	{
		TrajBody* b = buf + i;
		b->p[0] = toy_pos_x( &pose[i].p );
		b->p[1] = toy_pos_y( &pose[i].p );
		b->p[2] = toy_pos_z( &pose[i].p );
		b->q[0] = toy_val( pose[i].q.x, S_Q );
		b->q[1] = toy_val( pose[i].q.y, S_Q );
		b->q[2] = toy_val( pose[i].q.z, S_Q );
		b->q[3] = toy_val( pose[i].q.s, S_Q );
		b->v[0] = toy_val( state[i].v.x, S_V );
		b->v[1] = toy_val( state[i].v.y, S_V );
		b->v[2] = toy_val( state[i].v.z, S_V );
		b->w[0] = toy_val( state[i].w.x, S_W );
		b->w[1] = toy_val( state[i].w.y, S_W );
		b->w[2] = toy_val( state[i].w.z, S_W );
		if ( !awake[i] )
		{
			memset( b->v, 0, sizeof( b->v ) );
			memset( b->w, 0, sizeof( b->w ) );
		}
		b->awake = awake[i];
		b->flags = isStatic[i] ? TRAJ_STATIC : 0u;
	}
	traj_write( w, (uint32_t)tick, buf );
}

static RunResult run_toy( const ToyData* init, int ticks, int threads, int keepPos, TrajWriter* traj )
{
	RunResult r;
	memset( &r, 0, sizeof( r ) );
	int n = init->bodyCount;
	// this run's copies of the mutable buffers
	BodyState* state = (BodyState*)malloc( (size_t)n * sizeof( BodyState ) );
	BodyPose* pose = (BodyPose*)malloc( (size_t)n * sizeof( BodyPose ) );
	BodyMass* mass = (BodyMass*)malloc( (size_t)n * sizeof( BodyMass ) );
	memcpy( state, init->state, (size_t)n * sizeof( BodyState ) );
	memcpy( pose, init->pose, (size_t)n * sizeof( BodyPose ) );
	memcpy( mass, init->mass, (size_t)n * sizeof( BodyMass ) );
	Aabb* aabbs = (Aabb*)calloc( (size_t)n, sizeof( Aabb ) );
	Hash2* hashes = (Hash2*)calloc( (size_t)n, sizeof( Hash2 ) );
	// the list buffer: [prepare (last tick's awake)] [awake] [woken] [all] then, per pair, [active] [inactive] [all]
	// [by colour]
	int pcap = 256;
	uint32_t* lists = (uint32_t*)calloc( (size_t)( 4 * n + 4 * pcap ), sizeof( uint32_t ) );
	uint32_t L_PREP = 0, L_AWAKE = (uint32_t)n, L_WOKEN = (uint32_t)( 2 * n ), L_ALL = (uint32_t)( 3 * n );
	for ( int i = 0; i < n; ++i )
	{
		lists[L_ALL + i] = (uint32_t)i;
	}
	// per pair: the manifolds (ping-ponged: tick t writes mf[t & 1] and reads last tick's mf[(t & 1) ^ 1] by prevIndex),
	// the SAT records, the manifold hashes, the constraints; last tick's touching flags for the stages
	Manifold* mf[2];
	mf[0] = (Manifold*)calloc( (size_t)pcap, sizeof( Manifold ) );
	mf[1] = (Manifold*)calloc( (size_t)pcap, sizeof( Manifold ) );
	SatAxis* sat = (SatAxis*)calloc( (size_t)pcap, sizeof( SatAxis ) );
	Hash2* mhashes = (Hash2*)calloc( (size_t)pcap, sizeof( Hash2 ) );
	Constraint* cons = (Constraint*)calloc( (size_t)pcap, sizeof( Constraint ) );
	uint8_t* prevTouching = (uint8_t*)calloc( (size_t)pcap, 1 );
	NarrowDiag diagDummy; // Params.narrowDiag is 0: never written
	memset( &diagDummy, 0, sizeof( diagDummy ) );
	Params params = init->params;
	{
		// the bodies' buffers for the first prepare; every tick binds them all again after the stages (pairs move)
		void* bufs[TOY_BUFFERS] = { init->hulls, init->points, init->faces, init->edges, state, pose, mass, aabbs, lists, &params, hashes };
		size_t counts[TOY_BUFFERS] = { (size_t)init->hullCount, (size_t)init->pointCount, (size_t)init->faceCount, (size_t)init->edgeCount,
									   (size_t)n,
									   (size_t)n,
									   (size_t)n,
									   (size_t)n,
									   (size_t)( 4 * n + 4 * pcap ),
									   1,
									   (size_t)n };
		toy_bind( bufs, counts );
	}
	toy_reset_saturations();

	uint8_t* isStatic = (uint8_t*)malloc( (size_t)n );
	uint8_t* awakeNow = (uint8_t*)calloc( (size_t)n, 1 );
	int32_t* sleepTicks = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	int anyRestitution = 0; // the scene's materials: Box3D runs the restitution stage only when some contact has one
	for ( int i = 0; i < n; ++i )
	{
		isStatic[i] = ( mass[i].flags & BODY_STATIC ) ? 1 : 0;
		awakeNow[i] = isStatic[i] ? 0 : 1;
		anyRestitution |= mass[i].restitution > 0;
	}
	Stages st;
	stages_init( &st, n, isStatic );
	TrajBody* trajBuf = traj ? (TrajBody*)calloc( (size_t)n, sizeof( TrajBody ) ) : NULL;
	int trajEvery = traj ? (int)traj->every : 0;

	r.ticks = (TickRecord*)calloc( (size_t)ticks + 1, sizeof( TickRecord ) );
	if ( keepPos )
	{
		r.pos = (double*)malloc( (size_t)( ticks + 1 ) * (size_t)n * POSW * sizeof( double ) );
	}
	int listCap = 64;
	Dispatch* list = (Dispatch*)malloc( (size_t)listCap * sizeof( Dispatch ) );
	pool_start( threads );
	double total = 0.0;
	// the first prepare covers every body (static ones once, for good)
	uint32_t prepCount = (uint32_t)n;
	memcpy( lists + L_PREP, lists + L_ALL, (size_t)n * sizeof( uint32_t ) );
	if ( keepPos )
	{
		record_poses( r.pos, pose, n );
	}
	if ( traj )
	{
		record_traj( traj, trajBuf, 0, state, pose, isStatic, awakeNow, n );
	}
	for ( int t = 1; t <= ticks; ++t )
	{
		double t0 = vku_now_ms();
		// GPU: prepare the bodies that moved (AABBs, world inertia)
		int nd = 0;
		push_dispatch( list, &nd, E_PREPARE, L_PREP, prepCount );
		run_list( list, nd, t, &r );
		// CPU: last tick's touching pairs (their manifolds' point counts), broadphase, merge, wake, islands and sleep,
		// colouring
		for ( int i = 0; i < n; ++i )
		{
			sleepTicks[i] = state[i].sleepTicks;
		}
		Manifold* prevMf = mf[( t & 1 ) ^ 1];
		for ( int k = 0; k < st.prevCount; ++k )
		{
			prevTouching[k] = (uint8_t)( prevMf[k].pointCount > 0 );
		}
		stages_tick( &st, aabbs, sleepTicks, prevTouching, params.sleepTicks, params.enableSleep );
		if ( st.pairCount > pcap )
		{
			while ( pcap < st.pairCount )
			{
				pcap *= 2;
			}
			mf[0] = (Manifold*)realloc( mf[0], (size_t)pcap * sizeof( Manifold ) );
			mf[1] = (Manifold*)realloc( mf[1], (size_t)pcap * sizeof( Manifold ) );
			prevMf = mf[( t & 1 ) ^ 1];
			sat = (SatAxis*)realloc( sat, (size_t)pcap * sizeof( SatAxis ) );
			mhashes = (Hash2*)realloc( mhashes, (size_t)pcap * sizeof( Hash2 ) );
			cons = (Constraint*)realloc( cons, (size_t)pcap * sizeof( Constraint ) );
			prevTouching = (uint8_t*)realloc( prevTouching, (size_t)pcap );
			lists = (uint32_t*)realloc( lists, (size_t)( 4 * n + 4 * pcap ) * sizeof( uint32_t ) );
		}
		uint32_t L_APAIR = (uint32_t)( 4 * n ), L_IPAIR = L_APAIR + (uint32_t)pcap, L_PAIRS = L_IPAIR + (uint32_t)pcap;
		uint32_t L_COLOUR = L_PAIRS + (uint32_t)pcap;
		uint32_t activeCount = 0, inactiveCount = 0;
		for ( int k = 0; k < st.pairCount; ++k )
		{
			if ( st.active[k] )
			{
				lists[L_APAIR + activeCount++] = (uint32_t)k;
			}
			else
			{
				lists[L_IPAIR + inactiveCount++] = (uint32_t)k;
			}
			lists[L_PAIRS + k] = (uint32_t)k;
		}
		memcpy( lists + L_COLOUR, st.colourList, (size_t)st.colourStart[STAGE_MAX_COLOURS + 1] * sizeof( uint32_t ) );
		Manifold* curMf = mf[t & 1];
		void* bufs[TOY_BUFFERS] = { init->hulls, init->points, init->faces, init->edges, state, pose, mass, aabbs, lists, &params, hashes,
									NULL,		 st.pairs,	   curMf,		 prevMf,	   sat,	  &diagDummy, mhashes, cons };
		size_t counts[TOY_BUFFERS] = { (size_t)init->hullCount, (size_t)init->pointCount, (size_t)init->faceCount, (size_t)init->edgeCount,
									   (size_t)n,
									   (size_t)n,
									   (size_t)n,
									   (size_t)n,
									   (size_t)( 4 * n + 4 * pcap ),
									   1,
									   (size_t)n,
									   0,
									   (size_t)st.pairCount,
									   (size_t)pcap,
									   (size_t)pcap,
									   (size_t)pcap,
									   1,
									   (size_t)pcap,
									   (size_t)pcap };
		toy_bind( bufs, counts );
		if ( getenv( "TOY_DUMP_AABBS" ) != NULL && atoi( getenv( "TOY_DUMP_AABBS" ) ) == t && threads == 1 )
		{
			for ( int i = 0; i < n; ++i )
			{
				say( "  aabb %d: [%d %d %d] [%d %d %d]%s\n", i, aabbs[i].minx, aabbs[i].miny, aabbs[i].minz, aabbs[i].maxx, aabbs[i].maxy,
					 aabbs[i].maxz, isStatic[i] ? " static" : "" );
			}
		}
		memcpy( lists + L_AWAKE, st.awake, (size_t)st.awakeCount * sizeof( uint32_t ) );
		memcpy( lists + L_WOKEN, st.woken, (size_t)st.wokenCount * sizeof( uint32_t ) );
		// the dispatch list's size: the fixed stages, then per substep and restitution pass a dispatch per colour (and
		// per overflow pair)
		int perStage = st.colourCount + st.overflowCount;
		int need = 16 + ( 3 * params.substeps + params.restitutionIterations ) * perStage + 2 * params.substeps;
		if ( need > listCap )
		{
			listCap = need;
			list = (Dispatch*)realloc( list, (size_t)listCap * sizeof( Dispatch ) );
		}
		// GPU: wake; the narrowphase over the active pairs (an inactive pair's manifold carried); the contact solve
		// (Box3D's stage order); finalize, the hashes
		nd = 0;
		push_dispatch( list, &nd, E_WAKE, L_WOKEN, (uint32_t)st.wokenCount );
		push_dispatch( list, &nd, E_NSAT, L_APAIR, activeCount );
		push_dispatch( list, &nd, E_NCLIP, L_APAIR, activeCount );
		push_dispatch( list, &nd, E_NCOPY, L_IPAIR, inactiveCount );
		push_dispatch( list, &nd, E_PREPC, L_APAIR, activeCount );
		for ( int sub = 0; sub < params.substeps; ++sub )
		{
			push_dispatch( list, &nd, E_INTVEL, L_AWAKE, (uint32_t)st.awakeCount );
			push_colours( list, &nd, E_WARM, &st, L_COLOUR );
			push_colours( list, &nd, E_PUSH, &st, L_COLOUR );
			push_dispatch( list, &nd, E_INTPOS, L_AWAKE, (uint32_t)st.awakeCount );
			push_colours( list, &nd, E_RELAX, &st, L_COLOUR );
		}
		for ( int it = 0; anyRestitution && it < params.restitutionIterations; ++it )
		{
			push_colours( list, &nd, E_REST, &st, L_COLOUR );
		}
		push_dispatch( list, &nd, E_STORE, L_APAIR, activeCount );
		push_dispatch( list, &nd, E_FINAL, L_AWAKE, (uint32_t)st.awakeCount );
		push_dispatch( list, &nd, E_HASH, L_ALL, (uint32_t)n );
		push_dispatch( list, &nd, E_MHASH, L_PAIRS, (uint32_t)st.pairCount );
		r.dispatches = nd + 1 > r.dispatches ? nd + 1 : r.dispatches;
		run_list( list, nd, t, &r );
		total += vku_now_ms() - t0;
		if ( getenv( "TOY_DUMP_CONTACTS" ) != NULL && atoi( getenv( "TOY_DUMP_CONTACTS" ) ) == t && threads == 1 )
		{
			// the solved pairs' constraints after the tick (impulses after restitution), in double
			for ( int k = 0; k < st.pairCount; ++k )
			{
				const Constraint* c = cons + k;
				if ( !st.active[k] || c->pointCount == 0 )
				{
					continue;
				}
				say( "  pair %d (%d %d) colour %d flags %d eP %d: n (%.6f %.6f %.6f) friction %.4g restitution %.4g f (%.6g %.6g) twist %.6g centre "
					 "A (%.5f %.5f %.5f)\n",
					 k, c->bodyA, c->bodyB, st.pairs[k].colour, c->flags, c->eP, toy_val( c->normal.x, S_Q ), toy_val( c->normal.y, S_Q ),
					 toy_val( c->normal.z, S_Q ), toy_val( c->friction, S_MS ), toy_val( c->restitution, S_MS ), toy_val( c->frictionImpulse1, c->eP ),
					 toy_val( c->frictionImpulse2, c->eP ), toy_val( c->twistImpulse, c->eP ), toy_val( c->centerA.x, S_R ), toy_val( c->centerA.y, S_R ),
					 toy_val( c->centerA.z, S_R ) );
				for ( int i = 0; i < c->pointCount; ++i )
				{
					const ConstraintPoint* p = c->points + i;
					say( "    point %d id %08x: rA (%.5f %.5f %.5f) sep %.6g base %.6g impulse %.6g total %.6g mass %.6g lever %.4g vrel %.4g\n", i,
						 curMf[k].points[i].featureId, toy_val( p->anchorA.x, S_R ), toy_val( p->anchorA.y, S_R ), toy_val( p->anchorA.z, S_R ),
						 toy_val( curMf[k].points[i].separation, S_S ), toy_val( p->baseSeparation, S_S ), toy_val( p->normalImpulse, c->eP ),
						 toy_val( p->totalNormalImpulse, c->eP ), toy_val( p->normalMass, p->shN + c->eP - S_V ), toy_val( p->leverArm, S_R ),
						 toy_val( p->relativeVelocity, S_V ) );
				}
			}
		}

		TickRecord* rec = r.ticks + t;
		uint64_t sum = 0;
		for ( int i = 0; i < n; ++i )
		{
			sum += (uint64_t)hashes[i].lo | ( (uint64_t)hashes[i].hi << 32 );
		}
		rec->bodies = sum;
		sum = 0;
		rec->minSep = 1e30;
		rec->minSepA = -1;
		rec->minSepB = -1;
		for ( int k = 0; k < st.pairCount; ++k )
		{
			sum += (uint64_t)mhashes[k].lo | ( (uint64_t)mhashes[k].hi << 32 );
			rec->touching += curMf[k].pointCount > 0;
			rec->points += curMf[k].pointCount;
			for ( int i = 0; i < curMf[k].pointCount && st.active[k]; ++i )
			{
				double sep = toy_val( curMf[k].points[i].separation, S_S );
				if ( sep < rec->minSep )
				{
					rec->minSep = sep;
					rec->minSepA = curMf[k].bodyA;
					rec->minSepB = curMf[k].bodyB;
				}
			}
		}
		rec->manifolds = sum;
		rec->stages = stages_hash( &st );
		rec->awake = st.awakeCount;
		rec->pairs = st.pairCount;
		rec->active = st.activeCount;
		rec->colours = st.colourCount;
		rec->overflow = st.overflowCount;
		rec->islands = st.islandCount;
		rec->largest = st.largestIsland;
		rec->slept = st.sleptBodies;
		rec->woken = st.wokenCount;
		if ( keepPos )
		{
			record_poses( r.pos + (size_t)t * (size_t)n * POSW, pose, n );
		}
		if ( traj && t % trajEvery == 0 )
		{
			memset( awakeNow, 0, (size_t)n );
			for ( int i = 0; i < st.awakeCount; ++i )
			{
				awakeNow[st.awake[i]] = 1;
			}
			record_traj( traj, trajBuf, t, state, pose, isStatic, awakeNow, n );
		}
		// next tick prepares this tick's awake bodies
		memcpy( lists + L_PREP, st.awake, (size_t)st.awakeCount * sizeof( uint32_t ) );
		prepCount = (uint32_t)st.awakeCount;
	}
	pool_stop();
	r.msPerTick = total / ticks;
	r.saturations = toy_saturations();
	{
		const Manifold* last = mf[ticks & 1];
		r.finalSep = 1e30;
		r.finalA = -1;
		r.finalB = -1;
		for ( int k = 0; k < st.pairCount; ++k )
		{
			r.finalTouching += last[k].pointCount > 0;
			for ( int i = 0; i < last[k].pointCount; ++i )
			{
				double sep = toy_val( last[k].points[i].separation, S_S );
				if ( sep < r.finalSep )
				{
					r.finalSep = sep;
					r.finalA = last[k].bodyA;
					r.finalB = last[k].bodyB;
				}
			}
		}
	}
	stages_free( &st );
	free( list );
	free( isStatic );
	free( awakeNow );
	free( sleepTicks );
	free( state );
	free( pose );
	free( mass );
	free( aabbs );
	free( hashes );
	free( lists );
	free( mf[0] );
	free( mf[1] );
	free( sat );
	free( mhashes );
	free( cons );
	free( prevTouching );
	free( trajBuf );
	return r;
}

// ---------------------------------------------------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------------------------------------------------

static uint64_t fnv( uint64_t h, const void* p, size_t n )
{
	const uint8_t* b = (const uint8_t*)p;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= b[i];
		h *= 1099511628211ULL;
	}
	return h;
}

static int parse_list( const char* s, int* out, int max )
{
	int n = 0;
	while ( *s && n < max )
	{
		out[n++] = atoi( s );
		while ( *s && *s != ',' )
		{
			++s;
		}
		if ( *s == ',' )
		{
			++s;
		}
	}
	return n;
}

static void scene_summary( const Scene* sc, const ToyData* d )
{
	int statics = 0, maxV = 0, maxF = 0, maxE = 0;
	double massMin = 1e30, massMax = 0.0;
	for ( int i = 0; i < sc->bodyCount; ++i )
	{
		const SceneBody* b = sc->bodies + i;
		statics += b->isStatic;
		if ( !b->isStatic )
		{
			massMin = b->mass < massMin ? b->mass : massMin;
			massMax = b->mass > massMax ? b->mass : massMax;
		}
	}
	for ( int h = 0; h < sc->hullCount; ++h )
	{
		maxV = sc->hulls[h].vertexCount > maxV ? sc->hulls[h].vertexCount : maxV;
		maxF = sc->hulls[h].faceCount > maxF ? sc->hulls[h].faceCount : maxF;
		maxE = sc->hulls[h].edgeCount > maxE ? sc->hulls[h].edgeCount : maxE;
	}
	say( "scene %s (seed %llu): %d bodies (%d static), %d hulls (%d irregular chunks, %d redrawn), largest hull %d vertices %d faces "
		 "%d half-edges; mass %.4g to %.4g kg; quantisation range errors %d\n",
		 sc->name, (unsigned long long)sc->seed, sc->bodyCount, statics, sc->hullCount, sc->chunkCount, sc->chunkRedraws, maxV, maxF, maxE,
		 massMin, massMax, d->rangeErrors );
	// a check of the mass properties: a box's inertia per unit density is V (b^2 + c^2) / 3 from half extents
	for ( int h = 0; h < sc->hullCount; ++h )
	{
		const SceneHull* H = sc->hulls + h;
		if ( H->faceCount == 6 && H->vertexCount == 8 )
		{
			double a = H->boundsHalf[0], b = H->boundsHalf[1], c = H->boundsHalf[2];
			double want = H->volume * ( b * b + c * c ) / 3.0;
			say( "  hull %d (box %.3g x %.3g x %.3g m): volume %.6g (8abc %.6g), Ixx/density %.6g (V(b^2+c^2)/3 %.6g), Ixy %.3g\n", h, 2 * a, 2 * b,
				 2 * c, H->volume, 8.0 * a * b * c, H->inertia[0], want, H->inertia[1] );
			break;
		}
	}
}

int main( int argc, char** argv )
{
	const char* sceneName = "stack10";
	int ticks = 600;
	int threads[4] = { 1, 8 };
	int tCount = 2;
	uint64_t seed = 1;
	const char* posOut = NULL;
	const char* posRef = NULL;
	const char* trajPath = NULL;
	int trajEvery = 1;
	const char* logPath = NULL;
	ToySettings settings;
	toy_default_settings( &settings );
	int pushBody = -1;
	double pushV[3] = { 0.0, 0.0, 0.0 };
	double perturb = 0.0;
	int startFloat = 0;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--scene" ) == 0 )
			sceneName = v, ++i;
		else if ( strcmp( a, "--ticks" ) == 0 )
			ticks = atoi( v ), ++i;
		else if ( strcmp( a, "--threads" ) == 0 )
			tCount = parse_list( v, threads, 4 ), ++i;
		else if ( strcmp( a, "--seed" ) == 0 )
			seed = (uint64_t)strtoull( v, NULL, 10 ), ++i;
		else if ( strcmp( a, "--gravity" ) == 0 )
			settings.gravity = atof( v ), ++i;
		else if ( strcmp( a, "--sleep" ) == 0 )
			settings.enableSleep = atoi( v ), ++i;
		else if ( strcmp( a, "--recycle" ) == 0 )
			settings.recycle = atoi( v ), ++i;
		else if ( strcmp( a, "--push" ) == 0 )
		{
			pushBody = atoi( v );
			const char* c = strchr( v, ',' );
			for ( int k = 0; k < 3 && c; ++k )
			{
				pushV[k] = atof( c + 1 );
				c = strchr( c + 1, ',' );
			}
			++i;
		}
		else if ( strcmp( a, "--perturb" ) == 0 )
			perturb = atof( v ), ++i;
		else if ( strcmp( a, "--start-float" ) == 0 )
			startFloat = 1;
		else if ( strcmp( a, "--pos-out" ) == 0 )
			posOut = v, ++i;
		else if ( strcmp( a, "--pos-ref" ) == 0 )
			posRef = v, ++i;
		else if ( strcmp( a, "--traj" ) == 0 )
			trajPath = v, ++i;
		else if ( strcmp( a, "--traj-every" ) == 0 )
			trajEvery = atoi( v ) < 1 ? 1 : atoi( v ), ++i;
		else if ( strcmp( a, "--log" ) == 0 )
			logPath = v, ++i;
		else if ( strcmp( a, "--quiet" ) == 0 )
			g_quiet = 1;
		else
		{
			fprintf( stderr, "unknown option %s\n", a );
			return 1;
		}
	}
	if ( logPath )
	{
		g_log = fopen( logPath, "w" );
	}
	if ( tCount < 1 || threads[0] < 1 )
	{
		tCount = 1;
		threads[0] = 1;
	}
	for ( int i = 0; i < tCount; ++i )
	{
		threads[i] = threads[i] < 1 ? 1 : threads[i] > MAX_THREADS ? MAX_THREADS : threads[i];
	}

	Scene sc;
	if ( !scene_build( &sc, sceneName, seed ) )
	{
		fprintf( stderr, "unknown scene %s (%s)\n", sceneName, scene_names() );
		return 1;
	}
	if ( pushBody >= 0 && pushBody < sc.bodyCount )
	{
		sc.bodies[pushBody].v[0] = pushV[0];
		sc.bodies[pushBody].v[1] = pushV[1];
		sc.bodies[pushBody].v[2] = pushV[2];
	}
	for ( int i = 0; i < sc.bodyCount && perturb != 0.0; ++i )
	{
		sc.bodies[i].p[0] += sc.bodies[i].isStatic ? 0.0 : perturb;
		sc.bodies[i].p[1] += sc.bodies[i].isStatic ? 0.0 : perturb;
	}
	for ( int i = 0; i < sc.bodyCount && startFloat; ++i )
	{
		SceneBody* b = sc.bodies + i;
		for ( int k = 0; k < 3; ++k )
		{
			b->p[k] = (double)(float)b->p[k];
			b->v[k] = (double)(float)b->v[k];
			b->w[k] = (double)(float)b->w[k];
		}
		for ( int k = 0; k < 4; ++k )
		{
			b->q[k] = (double)(float)b->q[k];
		}
	}
	ToyData d;
	toy_quantize( &sc, &settings, &d );
	say( "toy %s, dialect %s, %d ticks, threads", sceneName, DIALECT_NAME, ticks );
	for ( int i = 0; i < tCount; ++i )
	{
		say( " %d", threads[i] );
	}
	say( "; gravity %g, sleep %s, contact recycling %s%s", settings.gravity, settings.enableSleep ? "on" : "off", settings.recycle ? "on" : "off",
		 pushBody >= 0 ? ", one body pushed" : "" );
	if ( perturb != 0.0 )
	{
		say( ", every dynamic body moved %g m along x and along y", perturb );
	}
	if ( startFloat )
	{
		say( ", the start rounded to float" );
	}
	say( "\n" );
	say( "twin: %s\n", toy_twin_info() );
	size_t cs[14] = { sizeof( Hull ),  sizeof( HullFace ), sizeof( BodyState ), sizeof( BodyPose ), sizeof( BodyMass ),
					  sizeof( Aabb ),  sizeof( Params ),   sizeof( Hash2 ),		sizeof( V3 ),		sizeof( Pair ),
					  sizeof( Manifold ), sizeof( SatAxis ), sizeof( NarrowDiag ), sizeof( Constraint ) };
	static const char* csn[14] = { "Hull", "HullFace", "BodyState", "BodyPose", "BodyMass", "Aabb",		  "Params",
								   "Hash2", "V3",		"Pair",		 "Manifold", "SatAxis", "NarrowDiag", "Constraint" };
	int layoutBad = 0;
	say( "layout sizes C/twin:" );
	for ( int i = 0; i < 14; ++i )
	{
		say( " %s %zu/%zu", csn[i], cs[i], toy_sizeof( i ) );
		layoutBad |= cs[i] != toy_sizeof( i );
	}
	say( "\n" );
	if ( layoutBad )
	{
		say( "FAIL: layout sizes differ\n" );
		return 2;
	}
	scene_summary( &sc, &d );

	RunResult runs[4];
	TrajWriter traj;
	int trajOk = trajPath ? traj_open( &traj, trajPath, (uint32_t)d.bodyCount, (uint32_t)trajEvery, 1.0 / 60.0 ) : 0;
	if ( trajPath && !trajOk )
	{
		say( "traj %s: cannot write\n", trajPath );
	}
	for ( int ti = 0; ti < tCount; ++ti )
	{
		runs[ti] = run_toy( &d, ticks, threads[ti], ti == 0 && ( posOut || posRef ), ti == 0 && trajOk ? &traj : NULL );
	}
	if ( trajOk )
	{
		uint32_t records = traj.records;
		trajOk = traj_close( &traj );
		say( "traj %s: %u records of %d bodies, every %d ticks%s\n", trajPath, records, d.bodyCount, trajEvery, trajOk ? "" : " (WRITE FAILED)" );
	}
	RunResult* r0 = runs;

	// per-tick hashes: every tick to 120, then every 60th
	say( "tick  bodies-hash       manifolds-hash    stages-hash       awake pairs active touching points colours(ovf) "
		 "islands(largest) slept woken\n" );
	for ( int t = 1; t <= ticks; ++t )
	{
		if ( t <= 120 || t % 60 == 0 || t == ticks )
		{
			const TickRecord* k = r0->ticks + t;
			say( "%4d  %016llx  %016llx  %016llx  %5d %5d %6d %8d %6d %4d(%d) %6d(%d) %5d %5d\n", t, (unsigned long long)k->bodies,
				 (unsigned long long)k->manifolds, (unsigned long long)k->stages, k->awake, k->pairs, k->active, k->touching, k->points, k->colours,
				 k->overflow, k->islands, k->largest, k->slept, k->woken );
		}
	}
	uint64_t runHash = 1469598103934665603ULL;
	for ( int t = 1; t <= ticks; ++t )
	{
		runHash = fnv( runHash, &r0->ticks[t].bodies, 8 );
		runHash = fnv( runHash, &r0->ticks[t].manifolds, 8 );
		runHash = fnv( runHash, &r0->ticks[t].stages, 8 );
	}
	int failures = 0;
	for ( int ti = 0; ti < tCount; ++ti )
	{
		int first = -1;
		for ( int t = 1; t <= ticks && first < 0; ++t )
		{
			if ( runs[ti].ticks[t].bodies != r0->ticks[t].bodies || runs[ti].ticks[t].manifolds != r0->ticks[t].manifolds ||
				 runs[ti].ticks[t].stages != r0->ticks[t].stages )
			{
				first = t;
			}
		}
		say( "threads %d: %.3f ms/tick, saturations %u, floating-point sentinel %s%s%s; per-tick hashes %s", threads[ti], runs[ti].msPerTick,
			 runs[ti].saturations, runs[ti].fpFailures ? "TRIPPED " : "clean", runs[ti].fpFailures ? "at " : "", runs[ti].firstFp,
			 ti == 0 ? "(reference)" : first < 0 ? "identical" : "DIFFER" );
		if ( first >= 0 )
		{
			say( " from tick %d", first );
		}
		say( "\n" );
		failures += first >= 0 || runs[ti].fpFailures > 0;
	}

	// the contact solve at a glance: when every dynamic body is asleep for good, the deepest manifold point (its
	// separation at the start of a tick, before the solve) over the run and at rest: the 60 ticks before everything
	// sleeps (the last 60 when it does not)
	{
		int asleepFrom = -1;
		for ( int t = ticks; t >= 1 && r0->ticks[t].awake == 0; --t )
		{
			asleepFrom = t;
		}
		int restEnd = asleepFrom > 0 ? asleepFrom - 1 : ticks;
		double deep = 1e30, deepLate = 1e30;
		int deepAt = 0, deepA = -1, deepB = -1, lateAt = 0, lateA = -1, lateB = -1;
		for ( int t = 1; t <= ticks; ++t )
		{
			const TickRecord* k = r0->ticks + t;
			if ( k->minSep < deep )
			{
				deep = k->minSep;
				deepAt = t;
				deepA = k->minSepA;
				deepB = k->minSepB;
			}
			if ( t > restEnd - 60 && t <= restEnd && k->minSep < deepLate )
			{
				deepLate = k->minSep;
				lateAt = t;
				lateA = k->minSepA;
				lateB = k->minSepB;
			}
		}
		if ( asleepFrom > 0 )
		{
			say( "solve: every dynamic body asleep from tick %d", asleepFrom );
		}
		else
		{
			say( "solve: %d bodies awake at the end", r0->ticks[ticks].awake );
		}
		say( "; deepest point %.6g m (tick %d, bodies %d %d), at rest (the 60 ticks before every body sleeps, or the last 60) %.6g m (tick %d, "
			 "bodies %d %d); at most %d dispatches a tick\n",
			 deep < 1e29 ? deep : 0.0, deepAt, deepA, deepB, deepLate < 1e29 ? deepLate : 0.0, lateAt, lateA, lateB, r0->dispatches );
		say( "rest: after tick %d, over every touching pair (sleeping ones too): deepest point %.6g m (bodies %d %d), %d touching pairs\n", ticks,
			 r0->finalSep < 1e29 ? r0->finalSep : 0.0, r0->finalA, r0->finalB, r0->finalTouching );
	}

	// positions: out, and against a reference (D)
	if ( posOut )
	{
		FILE* f = fopen( posOut, "wb" );
		int32_t hdr[3] = { d.bodyCount, ticks, POSW };
		fwrite( hdr, sizeof( hdr ), 1, f );
		fwrite( r0->pos, sizeof( double ), (size_t)( ticks + 1 ) * (size_t)d.bodyCount * POSW, f );
		fclose( f );
	}
	if ( posRef )
	{
		FILE* f = fopen( posRef, "rb" );
		int32_t hdr[3] = { 0, 0, 0 };
		if ( f == NULL || fread( hdr, sizeof( hdr ), 1, f ) != 1 || hdr[0] != d.bodyCount || hdr[1] < ticks || hdr[2] != POSW )
		{
			say( "pos-ref %s: missing or a different scene\n", posRef );
			failures += 1;
		}
		else
		{
			size_t cnt = (size_t)( hdr[1] + 1 ) * (size_t)hdr[0] * POSW;
			double* ref = (double*)malloc( cnt * sizeof( double ) );
			size_t got = fread( ref, sizeof( double ), cnt, f );
			(void)got;
			static const int windows[3][2] = { { 1, 120 }, { 121, 600 }, { 1, 600 } };
			for ( int w = 0; w < 3; ++w )
			{
				int a = windows[w][0], b = windows[w][1] < ticks ? windows[w][1] : ticks;
				if ( a > b )
				{
					continue;
				}
				double sum = 0.0, mx = 0.0, rsum = 0.0, rmx = 0.0;
				long long cntw = 0;
				int at = -1, atBody = -1;
				for ( int t = a; t <= b; ++t )
				{
					const double* p = r0->pos + (size_t)t * (size_t)d.bodyCount * POSW;
					const double* q = ref + (size_t)t * (size_t)d.bodyCount * POSW;
					for ( int i = 0; i < d.bodyCount; ++i )
					{
						if ( sc.bodies[i].isStatic )
						{
							continue;
						}
						const double* x = p + (size_t)i * POSW;
						const double* y = q + (size_t)i * POSW;
						double dx = x[0] - y[0], dy = x[1] - y[1], dz = x[2] - y[2];
						double e2 = dx * dx + dy * dy + dz * dz;
						sum += e2;
						cntw += 1;
						if ( e2 > mx )
						{
							mx = e2;
							at = t;
							atBody = i;
						}
						// the angle between the two orientations, 2 sqrt(1 - c^2) for small angles (c: the normalised dot)
						double dq = x[3] * y[3] + x[4] * y[4] + x[5] * y[5] + x[6] * y[6];
						double nx = x[3] * x[3] + x[4] * x[4] + x[5] * x[5] + x[6] * x[6];
						double ny = y[3] * y[3] + y[4] * y[4] + y[5] * y[5] + y[6] * y[6];
						double c2 = dq * dq / ( nx * ny );
						double ang2 = c2 < 1.0 ? 4.0 * ( 1.0 - c2 ) : 0.0;
						rsum += ang2;
						rmx = ang2 > rmx ? ang2 : rmx;
					}
				}
				say( "pose vs %s, ticks %d-%d: position rms %.3g m, max %.3g m (tick %d, body %d); rotation rms %.3g rad, max %.3g rad\n", posRef,
					 a, b, sqrt( sum / (double)( cntw ? cntw : 1 ) ), sqrt( mx ), at, atBody, sqrt( rsum / (double)( cntw ? cntw : 1 ) ), sqrt( rmx ) );
			}
			free( ref );
		}
		if ( f )
		{
			fclose( f );
		}
	}
	// where the first dynamic body is, as a sanity check of free fall (y0 - g h^2 n (n + 1) / 2 after n substeps)
	if ( r0->pos )
	{
		int body = -1;
		for ( int i = 0; i < d.bodyCount && body < 0; ++i )
		{
			body = sc.bodies[i].isStatic ? -1 : i;
		}
		if ( body >= 0 )
		{
			double y = r0->pos[( (size_t)ticks * (size_t)d.bodyCount + (size_t)body ) * POSW + 1];
			double nsub = 4.0 * ticks, h = 1.0 / 240.0;
			double want = sc.bodies[body].p[1] + sc.bodies[body].v[1] * h * nsub - settings.gravity * h * h * nsub * ( nsub + 1.0 ) / 2.0;
			say( "body %d at tick %d: y %.9g m (free fall gives %.9g)\n", body, ticks, y, want );
		}
	}
	say( "toy %s %s: run hash %016llx, final bodies %016llx manifolds %016llx stages %016llx, threads %s, saturations %u (twin: %s)\n",
		 DIALECT_NAME, sceneName, (unsigned long long)runHash, (unsigned long long)r0->ticks[ticks].bodies,
		 (unsigned long long)r0->ticks[ticks].manifolds, (unsigned long long)r0->ticks[ticks].stages,
		 failures ? "DISAGREE or sentinel tripped" : "agree", r0->saturations, toy_twin_info() );
	for ( int ti = 0; ti < tCount; ++ti )
	{
		free( runs[ti].ticks );
		free( runs[ti].pos );
	}
	toy_free_data( &d );
	scene_free( &sc );
	if ( g_log )
	{
		fclose( g_log );
	}
	return failures ? 1 : 0;
}
