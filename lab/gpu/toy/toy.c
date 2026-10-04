// toy.c: the toy's driver (DESIGN.md "Pipeline per tick" and "Comparing bits", steps 2 to 5). Builds a scene (double),
// quantises it into the dialect and runs N ticks: the GPU stages as the kernels' C++ twin on a thread pool (E11's) and,
// with --gpus, as their SPIR-V on each Vulkan GPU; the CPU stages between them (stages.c) on one readback (the AABBs, the
// sleep counters, last tick's touching pairs); the narrowphase over the active pairs (manifolds ping-ponged, an inactive
// pair's carried); the contact solve (prepare; per substep integrate velocities, warm start, push, integrate positions,
// relax, per colour; restitution; the impulses stored back); a per-tick hash (the bodies' and the manifolds' element
// hashes, read back and summed order-free per category, and the stages' decisions); on the twin, the floating-point
// sentinel after every dispatch. Built once per dialect (toy_F, toy_V4, toy_D; D has no GPU kernels).
//
//   toy_F --scene pile200 --ticks 600 --threads 1,8 [--seed S] [--gravity G] [--sleep 0|1] [--recycle 0|1]
//         [--push BODY,VX,VY,VZ] [--perturb EPS] [--gpus MASK | --gpu N] [--trace T] [--dispatch-hashes FILE]
//         [--ref FILE] [--ref-out FILE] [--pos-out FILE] [--pos-ref FILE] [--traj FILE [--traj-every N]] [--log FILE]
//         [--quiet]                                                                       (run from lab/gpu)
//
// --gpus MASK (or --gpu N) runs the scene on each Vulkan GPU in the mask beside the twin and compares the per-tick
// hashes; the first differing tick is reported, with the GPU's costs (per-stage time from timestamps, the tick's wall
// time with its readbacks). --trace T steps the twin and each GPU in the mask up to tick T, every buffer compared after
// every tick; the first tick that differs is stepped again dispatch by dispatch from its start, and the first differing
// word is named (dispatch, kernel, buffer, element, field, both values). --dispatch-hashes FILE writes a hash of the
// twin's buffers after every dispatch of the first run (to compare twins across machines). --ref-out FILE writes the
// reference hashes (the config, every tick to 120 then every 60th, the run hash over every tick); --ref FILE checks the
// twin's runs and every GPU's against them (so another machine is judged without Windows).
// --pos-out writes every body's pose per tick (double); --pos-ref reads another dialect's (D's) and reports the
// differences (rms, max) of the dynamic bodies' positions and orientations. --traj writes the first run's trajectory
// (traj.h's LPTRAJ1: tick 0, then every Nth tick). --perturb moves every dynamic body's start by EPS m along x and
// along y (in the scene's doubles, before quantisation: step 4b's measure of how much a scene amplifies a tiny
// difference; D keeps it, F and V4 round it away). Every scene starts from values every dialect holds exactly (scene.h's
// scene_round_start; the start line checks it). TOY_DUMP_AABBS=T prints every body's AABB at tick T (1-thread runs).
#include "fpflags.h"
#include "layout.h"
#include "quant.h"
#include "scene.h"
#include "stages.h"
#include "traj.h"
#include "vk_util.h"

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
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
#define HAS_GPU 1
#elif defined( DIALECT_V4 )
#define DIALECT_NAME "V4"
#define HAS_GPU 1
#else
#define DIALECT_NAME "D"
#define HAS_GPU 0
#endif

#define TOY_BUFFERS 22 // kernels.slang's bindings 0..21 (11, the counters, is the glue's)
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
	E_PREPJ,
	E_WARMJ,
	E_SOLVEJ,
	E_RELAXJ,
	E_JHASH,
	E_COUNT
};
static const char* g_entryNames[E_COUNT] = { "prepareBodies", "integrateVelocities", "integratePositions", "finalizeBodies",
											 "wakeBodies",	  "hashElements",		 "narrowSat",		   "narrowClip",
											 "copyManifolds", "hashManifolds",		 "prepareContacts",	   "warmStart",
											 "pushContacts",  "relaxContacts",		 "restitution",		   "storeImpulses",
											 "prepareJoints", "warmStartJoints",	 "solveJoints",		   "relaxJoints",
											 "hashJoints" };

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

// One solver stage over the coloured joints and pairs (the lists at jbase and base, in jointColourList's and
// colourList's order): the overflow colour's joints, then pairs, one dispatch each (they may share bodies), first, as
// Box3D solves its overflow constraints before the colours; then per colour its joints (jointEntry; -1: the stage has
// none, restitution) and its pairs. Without joints the dispatches are step 5's.
static void push_colours( Dispatch* list, int* n, int jointEntry, int entry, const Stages* st, uint32_t jbase, uint32_t base )
{
	for ( int i = st->jointColourStart[STAGE_MAX_COLOURS]; jointEntry >= 0 && i < st->jointColourStart[STAGE_MAX_COLOURS + 1]; ++i )
	{
		push_dispatch( list, n, jointEntry, jbase + (uint32_t)i, 1 );
	}
	for ( int i = st->colourStart[STAGE_MAX_COLOURS]; i < st->colourStart[STAGE_MAX_COLOURS + 1]; ++i )
	{
		push_dispatch( list, n, entry, base + (uint32_t)i, 1 );
	}
	for ( int c = 0; c < st->colourCount; ++c )
	{
		if ( jointEntry >= 0 )
		{
			push_dispatch( list, n, jointEntry, jbase + (uint32_t)st->jointColourStart[c],
						   (uint32_t)( st->jointColourStart[c + 1] - st->jointColourStart[c] ) );
		}
		push_dispatch( list, n, entry, base + (uint32_t)st->colourStart[c], (uint32_t)( st->colourStart[c + 1] - st->colourStart[c] ) );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// One simulation's buffers and CPU stages (the twin's buffers are the kernels'; a GPU run keeps its start, its
// readbacks and its uploads here)
// ---------------------------------------------------------------------------------------------------------------------

typedef struct TickRecord
{
	uint64_t bodies;	// the bodies' element hashes, summed
	uint64_t manifolds; // the manifolds' element hashes, summed
	uint64_t stages;	// the CPU stages' decisions
	uint64_t joints;	// the joints' element hashes, summed (0 without joints)
	int awake, pairs, active, colours, overflow, islands, largest, slept, woken, touching, points;
	double minSep; // the deepest manifold point this tick (the narrowphase's separation, before the solve), m
	int minSepA, minSepB;
} TickRecord;

typedef struct Sim
{
	const ToyData* init;
	int n;
	Params params;
	BodyState* state;
	BodyPose* pose;
	BodyMass* mass;
	Aabb* aabbs;
	Hash2* hashes;
	// the list buffer: [prepare (last tick's awake)] [awake] [woken] [all] then, per pair, [active] [inactive] [all]
	// [by colour], pcap entries each
	uint32_t* lists;
	int pcap;
	// per pair: the manifolds (ping-ponged: tick t writes mf[t & 1] and reads last tick's mf[(t & 1) ^ 1] by
	// prevIndex), the SAT records, the manifold hashes, the constraints; capacity pcap, zero beyond what was written
	Manifold* mf[2];
	SatAxis* sat;
	Hash2* mhashes;
	Constraint* cons;
	NarrowDiag diag; // Params.narrowDiag is 0: never written
	// the joints (fixed for the run): their state, hashes, this tick's commands, their bodies for the stages
	int jc;
	Joint* joints;
	Hash2* jhashes;
	JointCommand* jcmds;
	int32_t* jointA;
	int32_t* jointB;
	// the CPU stages' inputs (read back on a GPU) and state
	uint8_t* isStatic;
	int32_t* sleepTicks;
	uint8_t* prevTouching; // last tick's pairs: their manifold had points
	int anyRestitution;	   // the scene's materials: Box3D runs the restitution stage only when some contact has one
	Stages st;
	uint32_t prepCount;
	// this tick's dispatches after the stages
	Dispatch* list;
	int listCap, listCount, mostDispatches;
} Sim;

#define L_PREP 0u
#define L_AWAKE( s ) ( (uint32_t)( s )->n )
#define L_WOKEN( s ) ( (uint32_t)( 2 * ( s )->n ) )
#define L_ALL( s ) ( (uint32_t)( 3 * ( s )->n ) )
#define L_APAIR( s ) ( (uint32_t)( 4 * ( s )->n ) )
#define L_IPAIR( s ) ( L_APAIR( s ) + (uint32_t)( s )->pcap )
#define L_PAIRS( s ) ( L_IPAIR( s ) + (uint32_t)( s )->pcap )
#define L_COLOUR( s ) ( L_PAIRS( s ) + (uint32_t)( s )->pcap )
// then, per joint: [active] [by colour] [all]
#define L_JACTIVE( s ) ( L_COLOUR( s ) + (uint32_t)( s )->pcap )
#define L_JCOLOUR( s ) ( L_JACTIVE( s ) + (uint32_t)( s )->jc )
#define L_JALL( s ) ( L_JCOLOUR( s ) + (uint32_t)( s )->jc )
#define LIST_WORDS( s ) ( (size_t)( 4 * ( s )->n + 4 * ( s )->pcap + 3 * ( s )->jc ) )

static void sim_init( Sim* s, const ToyData* init )
{
	memset( s, 0, sizeof( *s ) );
	int n = init->bodyCount;
	s->init = init;
	s->n = n;
	s->params = init->params;
	s->state = (BodyState*)malloc( (size_t)n * sizeof( BodyState ) );
	s->pose = (BodyPose*)malloc( (size_t)n * sizeof( BodyPose ) );
	s->mass = (BodyMass*)malloc( (size_t)n * sizeof( BodyMass ) );
	memcpy( s->state, init->state, (size_t)n * sizeof( BodyState ) );
	memcpy( s->pose, init->pose, (size_t)n * sizeof( BodyPose ) );
	memcpy( s->mass, init->mass, (size_t)n * sizeof( BodyMass ) );
	s->aabbs = (Aabb*)calloc( (size_t)n, sizeof( Aabb ) );
	s->hashes = (Hash2*)calloc( (size_t)n, sizeof( Hash2 ) );
	s->jc = init->jointCount;
	size_t jn = (size_t)( s->jc > 0 ? s->jc : 1 );
	s->joints = (Joint*)calloc( jn, sizeof( Joint ) );
	memcpy( s->joints, init->joints, (size_t)s->jc * sizeof( Joint ) );
	s->jhashes = (Hash2*)calloc( jn, sizeof( Hash2 ) );
	s->jcmds = (JointCommand*)calloc( jn, sizeof( JointCommand ) );
	s->jointA = (int32_t*)calloc( jn, sizeof( int32_t ) );
	s->jointB = (int32_t*)calloc( jn, sizeof( int32_t ) );
	for ( int k = 0; k < s->jc; ++k )
	{
		s->jointA[k] = init->joints[k].bodyA;
		s->jointB[k] = init->joints[k].bodyB;
	}
	s->pcap = 256;
	s->lists = (uint32_t*)calloc( LIST_WORDS( s ), sizeof( uint32_t ) );
	for ( int i = 0; i < n; ++i )
	{
		s->lists[L_ALL( s ) + (uint32_t)i] = (uint32_t)i;
	}
	s->mf[0] = (Manifold*)calloc( (size_t)s->pcap, sizeof( Manifold ) );
	s->mf[1] = (Manifold*)calloc( (size_t)s->pcap, sizeof( Manifold ) );
	s->sat = (SatAxis*)calloc( (size_t)s->pcap, sizeof( SatAxis ) );
	s->mhashes = (Hash2*)calloc( (size_t)s->pcap, sizeof( Hash2 ) );
	s->cons = (Constraint*)calloc( (size_t)s->pcap, sizeof( Constraint ) );
	s->prevTouching = (uint8_t*)calloc( (size_t)s->pcap, 1 );
	s->isStatic = (uint8_t*)malloc( (size_t)n );
	s->sleepTicks = (int32_t*)calloc( (size_t)n, sizeof( int32_t ) );
	for ( int i = 0; i < n; ++i )
	{
		s->isStatic[i] = ( s->mass[i].flags & BODY_STATIC ) ? 1 : 0;
		s->anyRestitution |= s->mass[i].restitution > 0;
	}
	stages_init( &s->st, n, s->isStatic );
	if ( s->jc > 0 )
	{
		stages_set_joints( &s->st, s->jc, s->jointA, s->jointB );
	}
	// the first prepare covers every body (static ones once, for good)
	s->prepCount = (uint32_t)n;
	memcpy( s->lists + L_PREP, s->lists + L_ALL( s ), (size_t)n * sizeof( uint32_t ) );
	s->listCap = 64;
	s->list = (Dispatch*)malloc( (size_t)s->listCap * sizeof( Dispatch ) );
}

static void sim_free( Sim* s )
{
	stages_free( &s->st );
	free( s->state );
	free( s->pose );
	free( s->mass );
	free( s->aabbs );
	free( s->hashes );
	free( s->lists );
	free( s->mf[0] );
	free( s->mf[1] );
	free( s->sat );
	free( s->mhashes );
	free( s->cons );
	free( s->prevTouching );
	free( s->isStatic );
	free( s->sleepTicks );
	free( s->list );
	free( s->joints );
	free( s->jhashes );
	free( s->jcmds );
	free( s->jointA );
	free( s->jointB );
	memset( s, 0, sizeof( *s ) );
}

// The kernels' buffers for tick t, on the twin
static void sim_bind( Sim* s, int t )
{
	const ToyData* d = s->init;
	void* bufs[TOY_BUFFERS] = { d->hulls,	 d->points,	  d->faces,	   d->edges,	  s->state,				 s->pose,	s->mass,	s->aabbs,
								s->lists,	 &s->params,  s->hashes,   NULL,		  s->st.pairs,			 s->mf[t & 1], s->mf[( t & 1 ) ^ 1],
								s->sat,		 &s->diag,	  s->mhashes,  s->cons,		  s->joints,			 s->jhashes, s->jcmds };
	size_t counts[TOY_BUFFERS] = { (size_t)d->hullCount, (size_t)d->pointCount, (size_t)d->faceCount, (size_t)d->edgeCount, (size_t)s->n,
								   (size_t)s->n,		 (size_t)s->n,			(size_t)s->n,		  LIST_WORDS( s ),		1,
								   (size_t)s->n,		 0,						(size_t)s->st.pairCount, (size_t)s->pcap,	(size_t)s->pcap,
								   (size_t)s->pcap,		 1,						(size_t)s->pcap,	  (size_t)s->pcap,		(size_t)s->jc,
								   (size_t)s->jc,		 (size_t)s->jc };
	toy_bind( bufs, counts );
}

// The tick's first dispatch: prepare the bodies that moved (last tick's awake; every body on tick 1)
static int sim_list_prepare( const Sim* s, Dispatch* out )
{
	int nd = 0;
	push_dispatch( out, &nd, E_PREPARE, L_PREP, s->prepCount );
	return nd;
}

// The stages' inputs from the twin's buffers: the sleep counters, last tick's touching pairs (their point counts)
static void sim_gather( Sim* s, int t )
{
	for ( int i = 0; i < s->n; ++i )
	{
		s->sleepTicks[i] = s->state[i].sleepTicks;
	}
	const Manifold* prevMf = s->mf[( t & 1 ) ^ 1];
	for ( int k = 0; k < s->st.prevCount; ++k )
	{
		s->prevTouching[k] = (uint8_t)( prevMf[k].pointCount > 0 );
	}
}

static void* grow_zero( void* p, size_t oldCount, size_t newCount, size_t size )
{
	p = realloc( p, newCount * size );
	memset( (uint8_t*)p + oldCount * size, 0, ( newCount - oldCount ) * size );
	return p;
}

// The CPU stages (broadphase, merge, wake, islands and sleep, colouring), the per-pair buffers grown (the old contents
// kept, the rest zero), the lists, and the tick's dispatches after the stages. Returns the old pair capacity when it
// grew, else 0.
static int sim_stages( Sim* s, int t )
{
	Stages* st = &s->st;
	stages_tick( st, s->aabbs, s->sleepTicks, s->prevTouching, s->params.sleepTicks, s->params.enableSleep );
	int old = s->pcap;
	if ( st->pairCount > s->pcap )
	{
		int cap = s->pcap;
		while ( cap < st->pairCount )
		{
			cap *= 2;
		}
		s->mf[0] = (Manifold*)grow_zero( s->mf[0], (size_t)old, (size_t)cap, sizeof( Manifold ) );
		s->mf[1] = (Manifold*)grow_zero( s->mf[1], (size_t)old, (size_t)cap, sizeof( Manifold ) );
		s->sat = (SatAxis*)grow_zero( s->sat, (size_t)old, (size_t)cap, sizeof( SatAxis ) );
		s->mhashes = (Hash2*)grow_zero( s->mhashes, (size_t)old, (size_t)cap, sizeof( Hash2 ) );
		s->cons = (Constraint*)grow_zero( s->cons, (size_t)old, (size_t)cap, sizeof( Constraint ) );
		s->prevTouching = (uint8_t*)grow_zero( s->prevTouching, (size_t)old, (size_t)cap, 1 );
		size_t words = LIST_WORDS( s );
		s->pcap = cap;
		s->lists = (uint32_t*)grow_zero( s->lists, words, LIST_WORDS( s ), sizeof( uint32_t ) );
	}
	uint32_t activeCount = 0, inactiveCount = 0;
	for ( int k = 0; k < st->pairCount; ++k )
	{
		if ( st->active[k] )
		{
			s->lists[L_APAIR( s ) + activeCount++] = (uint32_t)k;
		}
		else
		{
			s->lists[L_IPAIR( s ) + inactiveCount++] = (uint32_t)k;
		}
		s->lists[L_PAIRS( s ) + (uint32_t)k] = (uint32_t)k;
	}
	memcpy( s->lists + L_COLOUR( s ), st->colourList, (size_t)st->colourStart[STAGE_MAX_COLOURS + 1] * sizeof( uint32_t ) );
	memcpy( s->lists + L_AWAKE( s ), st->awake, (size_t)st->awakeCount * sizeof( uint32_t ) );
	memcpy( s->lists + L_WOKEN( s ), st->woken, (size_t)st->wokenCount * sizeof( uint32_t ) );
	// the joints' lists and this tick's commands: the servo targets (the scene's rule, before step t) and the stages'
	// decisions
	for ( int k = 0; k < s->jc; ++k )
	{
		s->lists[L_JALL( s ) + (uint32_t)k] = (uint32_t)k;
		JointCommand* c = s->jcmds + k;
		c->target = toy_q( scene_servo_target( s->init->sceneJoints + k, t ), S_JA );
		c->pad = toy_q( 0.0, 0 );
		c->solve = st->jointActive[k] ? st->jointFlags[k] : 0;
		c->colour = st->jointColour[k];
	}
	memcpy( s->lists + L_JACTIVE( s ), st->jointActiveList, (size_t)st->jointActiveCount * sizeof( uint32_t ) );
	memcpy( s->lists + L_JCOLOUR( s ), st->jointColourList, (size_t)st->jointColourStart[STAGE_MAX_COLOURS + 1] * sizeof( uint32_t ) );
	// the dispatch list's size: the fixed stages, then per substep and restitution pass a dispatch per colour (and per
	// overflow pair), twice with joints
	const Params* P = &s->params;
	int perStage = ( s->jc > 0 ? 2 : 1 ) * st->colourCount + st->overflowCount + st->jointOverflow;
	int need = 24 + ( 3 * P->substeps + P->restitutionIterations ) * perStage + 2 * P->substeps;
	if ( need > s->listCap )
	{
		s->listCap = need;
		s->list = (Dispatch*)realloc( s->list, (size_t)s->listCap * sizeof( Dispatch ) );
	}
	// wake; the narrowphase over the active pairs (an inactive pair's manifold carried); the contact solve (Box3D's
	// stage order); finalize, the hashes
	Dispatch* list = s->list;
	int nd = 0;
	push_dispatch( list, &nd, E_WAKE, L_WOKEN( s ), (uint32_t)st->wokenCount );
	push_dispatch( list, &nd, E_NSAT, L_APAIR( s ), activeCount );
	push_dispatch( list, &nd, E_NCLIP, L_APAIR( s ), activeCount );
	push_dispatch( list, &nd, E_NCOPY, L_IPAIR( s ), inactiveCount );
	push_dispatch( list, &nd, E_PREPJ, L_JACTIVE( s ), (uint32_t)st->jointActiveCount );
	push_dispatch( list, &nd, E_PREPC, L_APAIR( s ), activeCount );
	for ( int sub = 0; sub < P->substeps; ++sub )
	{
		push_dispatch( list, &nd, E_INTVEL, L_AWAKE( s ), (uint32_t)st->awakeCount );
		push_colours( list, &nd, E_WARMJ, E_WARM, st, L_JCOLOUR( s ), L_COLOUR( s ) );
		push_colours( list, &nd, E_SOLVEJ, E_PUSH, st, L_JCOLOUR( s ), L_COLOUR( s ) );
		push_dispatch( list, &nd, E_INTPOS, L_AWAKE( s ), (uint32_t)st->awakeCount );
		push_colours( list, &nd, E_RELAXJ, E_RELAX, st, L_JCOLOUR( s ), L_COLOUR( s ) );
	}
	for ( int it = 0; s->anyRestitution && it < P->restitutionIterations; ++it )
	{
		push_colours( list, &nd, -1, E_REST, st, L_JCOLOUR( s ), L_COLOUR( s ) );
	}
	push_dispatch( list, &nd, E_STORE, L_APAIR( s ), activeCount );
	push_dispatch( list, &nd, E_FINAL, L_AWAKE( s ), (uint32_t)st->awakeCount );
	push_dispatch( list, &nd, E_HASH, L_ALL( s ), (uint32_t)s->n );
	push_dispatch( list, &nd, E_MHASH, L_PAIRS( s ), (uint32_t)st->pairCount );
	push_dispatch( list, &nd, E_JHASH, L_JALL( s ), (uint32_t)s->jc );
	s->listCount = nd;
	s->mostDispatches = nd + 1 > s->mostDispatches ? nd + 1 : s->mostDispatches;
	(void)t;
	return s->pcap != old ? old : 0;
}

// The tick's hashes (the element hashes summed per category, the stages'), and with `full` the manifolds' counts and
// deepest point (the twin's)
static void sim_record( const Sim* s, int t, TickRecord* rec, int full )
{
	const Stages* st = &s->st;
	uint64_t sum = 0;
	for ( int i = 0; i < s->n; ++i )
	{
		sum += (uint64_t)s->hashes[i].lo | ( (uint64_t)s->hashes[i].hi << 32 );
	}
	rec->bodies = sum;
	sum = 0;
	for ( int k = 0; k < st->pairCount; ++k )
	{
		sum += (uint64_t)s->mhashes[k].lo | ( (uint64_t)s->mhashes[k].hi << 32 );
	}
	rec->manifolds = sum;
	sum = 0;
	for ( int k = 0; k < s->jc; ++k )
	{
		sum += (uint64_t)s->jhashes[k].lo | ( (uint64_t)s->jhashes[k].hi << 32 );
	}
	rec->joints = sum;
	rec->minSep = 1e30;
	rec->minSepA = -1;
	rec->minSepB = -1;
	const Manifold* curMf = s->mf[t & 1];
	for ( int k = 0; k < st->pairCount && full; ++k )
	{
		rec->touching += curMf[k].pointCount > 0;
		rec->points += curMf[k].pointCount;
		for ( int i = 0; i < curMf[k].pointCount && st->active[k]; ++i )
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
	rec->stages = stages_hash( st );
	rec->awake = st->awakeCount;
	rec->pairs = st->pairCount;
	rec->active = st->activeCount;
	rec->colours = st->colourCount;
	rec->overflow = st->overflowCount;
	rec->islands = st->islandCount;
	rec->largest = st->largestIsland;
	rec->slept = st->sleptBodies;
	rec->woken = st->wokenCount;
}

// The next tick prepares this tick's awake bodies
static void sim_end_tick( Sim* s )
{
	memcpy( s->lists + L_PREP, s->st.awake, (size_t)s->st.awakeCount * sizeof( uint32_t ) );
	s->prepCount = (uint32_t)s->st.awakeCount;
}

// ---------------------------------------------------------------------------------------------------------------------
// The buffers the kernels write, as views (for the dispatch hashes and the trace's comparisons), and their fields
// ---------------------------------------------------------------------------------------------------------------------

enum
{
	V_STATE,
	V_POSE,
	V_MASS,
	V_AABB,
	V_HASH,
	V_MF0,
	V_MF1,
	V_SAT,
	V_MHASH,
	V_CONS,
	V_JOINT,
	V_JHASH,
	V_COUNT
};
static const int g_viewBinding[V_COUNT] = { 4, 5, 6, 7, 10, 13, 14, 15, 17, 18, 19, 20 }; // the GPU's buffer slots (13, 14: mf[0], mf[1])

typedef struct View
{
	void* p[V_COUNT];
	size_t bytes[V_COUNT];
} View;

static View sim_view( const Sim* s )
{
	View v;
	size_t n = (size_t)s->n, pc = (size_t)s->pcap, jc = (size_t)s->jc;
	void* p[V_COUNT] = { s->state, s->pose, s->mass, s->aabbs, s->hashes, s->mf[0], s->mf[1], s->sat, s->mhashes, s->cons, s->joints, s->jhashes };
	size_t b[V_COUNT] = { n * sizeof( BodyState ), n * sizeof( BodyPose ),	n * sizeof( BodyMass ), n * sizeof( Aabb ),	  n * sizeof( Hash2 ),
						  pc * sizeof( Manifold ), pc * sizeof( Manifold ), pc * sizeof( SatAxis ), pc * sizeof( Hash2 ), pc * sizeof( Constraint ),
						  jc * sizeof( Joint ),	   jc * sizeof( Hash2 ) };
	memcpy( v.p, p, sizeof( p ) );
	memcpy( v.bytes, b, sizeof( b ) );
	return v;
}

// A copy of a view's buffers (the trace's snapshots and downloads)
typedef struct Snap
{
	View v;
	size_t cap[V_COUNT];
} Snap;

static void snap_size( Snap* s, const View* like )
{
	for ( int i = 0; i < V_COUNT; ++i )
	{
		if ( s->cap[i] < like->bytes[i] || s->v.p[i] == NULL )
		{
			s->v.p[i] = realloc( s->v.p[i], like->bytes[i] > 16 ? like->bytes[i] : 16 );
			s->cap[i] = like->bytes[i] > 16 ? like->bytes[i] : 16;
		}
		s->v.bytes[i] = like->bytes[i];
	}
}

static void snap_take( Snap* s, const View* from )
{
	snap_size( s, from );
	for ( int i = 0; i < V_COUNT; ++i )
	{
		memcpy( s->v.p[i], from->p[i], from->bytes[i] );
	}
}

static void snap_free( Snap* s )
{
	for ( int i = 0; i < V_COUNT; ++i )
	{
		free( s->v.p[i] );
	}
	memset( s, 0, sizeof( *s ) );
}

// 64-bit FNV-1a over the views' 32-bit words (the dispatch hashes; every byte the kernels can write is deterministic:
// buffers start zeroed and grow zeroed, pads are written)
static uint64_t view_hash( const View* v )
{
	uint64_t h = 1469598103934665603ULL;
	for ( int i = 0; i < V_COUNT; ++i )
	{
		const uint32_t* w = (const uint32_t*)v->p[i];
		for ( size_t k = 0; k < v->bytes[i] / 4; ++k )
		{
			h ^= w[k];
			h *= 1099511628211ULL;
		}
	}
	return h;
}

// The fields of each buffer's element: name, byte offset, kind ('T' the dialect's scalar, 'i' int32, 'u' uint32)
typedef struct Field
{
	const char* name;
	uint32_t offset;
	char kind;
} Field;

#define FT( S, f ) { #f, (uint32_t)offsetof( S, f ), 'T' }
#define FI( S, f ) { #f, (uint32_t)offsetof( S, f ), 'i' }
#define FU( S, f ) { #f, (uint32_t)offsetof( S, f ), 'u' }
#define FV3( S, f ) FT( S, f.x ), FT( S, f.y ), FT( S, f.z )
#define FQ4( S, f ) FT( S, f.x ), FT( S, f.y ), FT( S, f.z ), FT( S, f.s )
#define FSYM( S, f ) FT( S, f.xx ), FT( S, f.xy ), FT( S, f.xz ), FT( S, f.yy ), FT( S, f.yz ), FT( S, f.zz )
#if defined( DIALECT_V4 )
#define FPOS( S, f ) FU( S, f.xlo ), FI( S, f.xhi ), FU( S, f.ylo ), FI( S, f.yhi ), FU( S, f.zlo ), FI( S, f.zhi )
#else
#define FPOS( S, f ) FV3( S, f )
#endif
#define FMP( i )                                                                                                       \
	FV3( Manifold, points[i].anchorA ), FV3( Manifold, points[i].anchorB ), FT( Manifold, points[i].separation ),       \
		FT( Manifold, points[i].baseSeparation ), FT( Manifold, points[i].normalImpulse ),                             \
		FT( Manifold, points[i].totalNormalImpulse ), FT( Manifold, points[i].normalMass ), FT( Manifold, points[i].pad ), \
		FU( Manifold, points[i].featureId ), FI( Manifold, points[i].shN )
#define FCP( i )                                                                                                       \
	FV3( Constraint, points[i].anchorA ), FV3( Constraint, points[i].anchorB ), FT( Constraint, points[i].baseSeparation ), \
		FT( Constraint, points[i].normalImpulse ), FT( Constraint, points[i].totalNormalImpulse ),                     \
		FT( Constraint, points[i].restitutionImpulse ), FT( Constraint, points[i].normalMass ),                        \
		FT( Constraint, points[i].leverArm ), FT( Constraint, points[i].relativeVelocity ), FT( Constraint, points[i].pad ), \
		FI( Constraint, points[i].shN ), FI( Constraint, points[i].pad1 )

static const Field g_fState[] = { FV3( BodyState, v ), FV3( BodyState, w ), FV3( BodyState, dp ), FQ4( BodyState, dq ),
								  FI( BodyState, sleepTicks ), FI( BodyState, flags ) };
static const Field g_fPose[] = { FPOS( BodyPose, p ), FQ4( BodyPose, q ) };
static const Field g_fMass[] = { FSYM( BodyMass, invIl ),	 FSYM( BodyMass, invIw ), FT( BodyMass, invMass ), FT( BodyMass, friction ),
								 FT( BodyMass, restitution ), FT( BodyMass, pad ),	  FI( BodyMass, eM ),	   FI( BodyMass, eI ),
								 FI( BodyMass, hull ),		  FI( BodyMass, flags ) };
static const Field g_fAabb[] = { FI( Aabb, minx ), FI( Aabb, miny ), FI( Aabb, minz ), FI( Aabb, maxx ),
								 FI( Aabb, maxy ), FI( Aabb, maxz ), FI( Aabb, pad0 ), FI( Aabb, pad1 ) };
static const Field g_fHash[] = { FU( Hash2, lo ), FU( Hash2, hi ) };
static const Field g_fManifold[] = { FMP( 0 ),
									 FMP( 1 ),
									 FMP( 2 ),
									 FMP( 3 ),
									 FV3( Manifold, normal ),
									 FV3( Manifold, frictionImpulse ),
									 FT( Manifold, twistImpulse ),
									 FT( Manifold, axisSeparation ),
									 FI( Manifold, pointCount ),
									 FI( Manifold, bodyA ),
									 FI( Manifold, bodyB ),
									 FI( Manifold, eP ),
									 FI( Manifold, axisType ),
									 FI( Manifold, axisA ),
									 FI( Manifold, axisB ),
									 FI( Manifold, flags ),
									 FQ4( Manifold, cachedRotationA ),
									 FQ4( Manifold, cachedRotationB ),
									 FV3( Manifold, cachedPoseP ),
									 FQ4( Manifold, cachedPoseQ ) };
static const Field g_fSat[] = { FT( SatAxis, faceASep ), FT( SatAxis, faceBSep ), FT( SatAxis, edgeSep ), FT( SatAxis, sepSep ),
								FI( SatAxis, kind ),	 FI( SatAxis, type ),	  FI( SatAxis, sepA ),	  FI( SatAxis, sepB ),
								FI( SatAxis, cacheType ), FI( SatAxis, cacheA ), FI( SatAxis, cacheB ), FI( SatAxis, faceA ),
								FI( SatAxis, vertexB ),	 FI( SatAxis, faceB ),	  FI( SatAxis, vertexA ), FI( SatAxis, edgeA ),
								FI( SatAxis, edgeB ),	 FI( SatAxis, pad ) };
static const Field g_fCons[] = { FCP( 0 ),
								 FCP( 1 ),
								 FCP( 2 ),
								 FCP( 3 ),
								 FV3( Constraint, normal ),
								 FV3( Constraint, tangent1 ),
								 FV3( Constraint, tangent2 ),
								 FV3( Constraint, centerA ),
								 FV3( Constraint, centerB ),
								 FSYM( Constraint, invIA ),
								 FSYM( Constraint, invIB ),
								 FT( Constraint, invMassA ),
								 FT( Constraint, invMassB ),
								 FT( Constraint, tangentMassXX ),
								 FT( Constraint, tangentMassXY ),
								 FT( Constraint, tangentMassYY ),
								 FT( Constraint, twistMass ),
								 FT( Constraint, frictionImpulse1 ),
								 FT( Constraint, frictionImpulse2 ),
								 FT( Constraint, twistImpulse ),
								 FT( Constraint, friction ),
								 FT( Constraint, restitution ),
								 FT( Constraint, pad ),
								 FI( Constraint, bodyA ),
								 FI( Constraint, bodyB ),
								 FI( Constraint, pointCount ),
								 FI( Constraint, flags ),
								 FI( Constraint, eP ),
								 FI( Constraint, shMA ),
								 FI( Constraint, shMB ),
								 FI( Constraint, shIA ),
								 FI( Constraint, shIB ),
								 FI( Constraint, shT ),
								 FI( Constraint, shTw ),
								 FI( Constraint, pad1 ) };
static const Field g_fJoint[] = { FV3( Joint, localAnchorA ), FV3( Joint, localAnchorB ), FQ4( Joint, localFrameA ), FQ4( Joint, localFrameB ),
								  FT( Joint, lowerAngle ),	  FT( Joint, upperAngle ),	 FT( Joint, maxMotorTorque ), FT( Joint, motorSpeed ),
								  FT( Joint, servoGain ),	  FT( Joint, servoMaxSpeed ), FQ4( Joint, frameAq ),	   FQ4( Joint, frameBq ),
								  FV3( Joint, frameAp ),	  FV3( Joint, frameBp ),	 FV3( Joint, deltaCenter ),   FV3( Joint, axisZ ),
								  FV3( Joint, perpAxisX ),	  FV3( Joint, perpAxisY ),	 FSYM( Joint, invIA ),		  FSYM( Joint, invIB ),
								  FSYM( Joint, pointMass ),	  FT( Joint, invMassA ),	 FT( Joint, invMassB ),		  FT( Joint, axisMassXX ),
								  FT( Joint, axisMassXY ),	  FT( Joint, axisMassYY ),	 FT( Joint, axialMass ),	  FT( Joint, angle ),
								  FT( Joint, speed ),		  FT( Joint, maxMotorImpulse ), FV3( Joint, linearImpulse ), FT( Joint, perpImpulseX ),
								  FT( Joint, perpImpulseY ),  FT( Joint, motorImpulse ), FT( Joint, lowerImpulse ),   FT( Joint, upperImpulse ),
								  FT( Joint, pad ),			  FI( Joint, bodyA ),		 FI( Joint, bodyB ),		  FI( Joint, flags ),
								  FI( Joint, solve ),		  FI( Joint, eP ),			 FI( Joint, shMA ),			  FI( Joint, shMB ),
								  FI( Joint, shIA ),		  FI( Joint, shIB ),		 FI( Joint, shK ),			  FI( Joint, shK2 ),
								  FI( Joint, shAx ) };

#define FIELDS( a ) a, (int)( sizeof( a ) / sizeof( a[0] ) )
static const struct
{
	const char* name;
	size_t elem;
	const Field* fields;
	int fieldCount;
} g_views[V_COUNT] = { { "bodyState", sizeof( BodyState ), FIELDS( g_fState ) },	{ "bodyPose", sizeof( BodyPose ), FIELDS( g_fPose ) },
					   { "bodyMass", sizeof( BodyMass ), FIELDS( g_fMass ) },		{ "aabbs", sizeof( Aabb ), FIELDS( g_fAabb ) },
					   { "hashes", sizeof( Hash2 ), FIELDS( g_fHash ) },			{ "manifolds[0]", sizeof( Manifold ), FIELDS( g_fManifold ) },
					   { "manifolds[1]", sizeof( Manifold ), FIELDS( g_fManifold ) }, { "satAxes", sizeof( SatAxis ), FIELDS( g_fSat ) },
					   { "manifoldHashes", sizeof( Hash2 ), FIELDS( g_fHash ) },	{ "constraints", sizeof( Constraint ), FIELDS( g_fCons ) },
					   { "joints", sizeof( Joint ), FIELDS( g_fJoint ) },			{ "jointHashes", sizeof( Hash2 ), FIELDS( g_fHash ) } };

static const Field* field_at( int view, size_t offset )
{
	const Field* best = NULL;
	for ( int i = 0; i < g_views[view].fieldCount; ++i )
	{
		if ( g_views[view].fields[i].offset <= offset )
		{
			best = g_views[view].fields + i;
		}
	}
	return best;
}

// One word as its field's kind
static void word_text( char* out, size_t size, char kind, uint32_t w )
{
	if ( kind == 'T' )
	{
#if defined( DIALECT_F )
		float f;
		memcpy( &f, &w, 4 );
		snprintf( out, size, "%.9g (%08x)", (double)f, w );
#else
		snprintf( out, size, "%d (%08x)", (int32_t)w, w );
#endif
	}
	else if ( kind == 'i' )
	{
		snprintf( out, size, "%d (%08x)", (int32_t)w, w );
	}
	else
	{
		snprintf( out, size, "%08x", w );
	}
}

// Compares two views word by word; prints the first differing word (buffer, element, field, both values), the
// element's words on both sides, and the differing words per buffer. Returns the number of differing words.
static long long view_compare( const View* a, const View* b, const char* labelA, const char* labelB, int print )
{
	long long total = 0;
	int shown = 0;
	for ( int i = 0; i < V_COUNT; ++i )
	{
		const uint32_t* x = (const uint32_t*)a->p[i];
		const uint32_t* y = (const uint32_t*)b->p[i];
		size_t words = ( a->bytes[i] < b->bytes[i] ? a->bytes[i] : b->bytes[i] ) / 4;
		long long bad = 0;
		size_t first = 0;
		for ( size_t k = 0; k < words; ++k )
		{
			if ( x[k] != y[k] )
			{
				first = bad == 0 ? k : first;
				bad += 1;
			}
		}
		if ( bad == 0 )
		{
			continue;
		}
		total += bad;
		if ( !print )
		{
			continue;
		}
		size_t elemWords = g_views[i].elem / 4;
		size_t elem = first / elemWords;
		size_t off = ( first % elemWords ) * 4;
		const Field* f = field_at( i, off );
		if ( shown == 0 )
		{
			char va[64], vb[64];
			word_text( va, sizeof( va ), f ? f->kind : 'u', x[first] );
			word_text( vb, sizeof( vb ), f ? f->kind : 'u', y[first] );
			say( "    first differing word: %s[%zu].%s: %s %s, %s %s\n", g_views[i].name, elem, f ? f->name : "?", labelA, va, labelB, vb );
			for ( int side = 0; side < 2; ++side )
			{
				const uint32_t* w = ( side ? y : x ) + elem * elemWords;
				say( "    %s[%zu] %-5s:", g_views[i].name, elem, side ? labelB : labelA );
				for ( size_t k = 0; k < elemWords; ++k )
				{
					say( " %08x", w[k] );
				}
				say( "\n" );
			}
		}
		say( "    %s: %lld words differ (first %s[%zu].%s)\n", g_views[i].name, bad, g_views[i].name, elem, f ? f->name : "?" );
		shown += 1;
	}
	return total;
}

// ---------------------------------------------------------------------------------------------------------------------
// The twin
// ---------------------------------------------------------------------------------------------------------------------

typedef struct RunResult
{
	TickRecord* ticks; // [1, ticks]
	double* pos;	   // per tick, per body, xyz (when kept)
	double msPerTick;
	uint32_t saturations;
	int fpFailures;
	char firstFp[160];
	int dispatches; // the most in one tick
	int joints;		// the scene's joint count (its hash joins the tick's three when there are joints)
	// after the last tick, over every touching pair (sleeping ones too, their manifolds carried): the deepest point
	double finalSep;
	int finalA, finalB, finalTouching;
} RunResult;

static FILE* g_dispatchHashes; // the first run's per-dispatch hashes

// Runs dispatches on the twin's pool (dispatch numbers from `first`: the prepare is 0, the rest 1 on); the sentinel
// after each
static void twin_list( Sim* s, const Dispatch* list, int n, int tick, int first, RunResult* r )
{
	for ( int i = 0; i < n; ++i )
	{
		int m = pool_dispatch( list + i );
		if ( m && r )
		{
			if ( r->fpFailures == 0 )
			{
				char names[96];
				snprintf( r->firstFp, sizeof( r->firstFp ), "tick %d dispatch %d (%s start %u count %u): %s", tick, first + i,
						  g_entryNames[list[i].entry], list[i].start, list[i].count, fp_names( m, names, sizeof( names ) ) );
			}
			r->fpFailures += 1;
		}
		if ( g_dispatchHashes )
		{
			View v = sim_view( s );
			fprintf( g_dispatchHashes, "%d %d %s %u %u %016llx\n", tick, first + i, g_entryNames[list[i].entry], list[i].start, list[i].count,
					 (unsigned long long)view_hash( &v ) );
		}
	}
}

// The twin's tick: prepare; the stages on its buffers; the rest
static void twin_tick( Sim* s, int t, RunResult* r )
{
	Dispatch prep;
	sim_bind( s, t );
	int np = sim_list_prepare( s, &prep );
	twin_list( s, &prep, np, t, 0, r );
	sim_gather( s, t );
	sim_stages( s, t );
	sim_bind( s, t );
	twin_list( s, s->list, s->listCount, t, 1, r );
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

static RunResult run_twin( const ToyData* init, int ticks, int threads, int keepPos, TrajWriter* traj )
{
	RunResult r;
	memset( &r, 0, sizeof( r ) );
	r.joints = init->jointCount;
	Sim s;
	sim_init( &s, init );
	int n = s.n;
	toy_reset_saturations();
	uint8_t* awakeNow = (uint8_t*)calloc( (size_t)n, 1 );
	for ( int i = 0; i < n; ++i )
	{
		awakeNow[i] = s.isStatic[i] ? 0 : 1;
	}
	TrajBody* trajBuf = traj ? (TrajBody*)calloc( (size_t)n, sizeof( TrajBody ) ) : NULL;
	int trajEvery = traj ? (int)traj->every : 0;
	r.ticks = (TickRecord*)calloc( (size_t)ticks + 1, sizeof( TickRecord ) );
	if ( keepPos )
	{
		r.pos = (double*)malloc( (size_t)( ticks + 1 ) * (size_t)n * POSW * sizeof( double ) );
		record_poses( r.pos, s.pose, n );
	}
	if ( traj )
	{
		record_traj( traj, trajBuf, 0, s.state, s.pose, s.isStatic, awakeNow, n );
	}
	pool_start( threads );
	double total = 0.0;
	for ( int t = 1; t <= ticks; ++t )
	{
		double t0 = vku_now_ms();
		twin_tick( &s, t, &r );
		total += vku_now_ms() - t0;
		if ( getenv( "TOY_DUMP_AABBS" ) != NULL && atoi( getenv( "TOY_DUMP_AABBS" ) ) == t && threads == 1 )
		{
			for ( int i = 0; i < n; ++i )
			{
				say( "  aabb %d: [%d %d %d] [%d %d %d]%s\n", i, s.aabbs[i].minx, s.aabbs[i].miny, s.aabbs[i].minz, s.aabbs[i].maxx,
					 s.aabbs[i].maxy, s.aabbs[i].maxz, s.isStatic[i] ? " static" : "" );
			}
		}
		if ( getenv( "TOY_DUMP_CONTACTS" ) != NULL && atoi( getenv( "TOY_DUMP_CONTACTS" ) ) == t && threads == 1 )
		{
			// the solved pairs' constraints after the tick (impulses after restitution), in double
			const Manifold* curMf = s.mf[t & 1];
			for ( int k = 0; k < s.st.pairCount; ++k )
			{
				const Constraint* c = s.cons + k;
				if ( !s.st.active[k] || c->pointCount == 0 )
				{
					continue;
				}
				say( "  pair %d (%d %d) colour %d flags %d eP %d: n (%.6f %.6f %.6f) friction %.4g restitution %.4g f (%.6g %.6g) twist %.6g centre "
					 "A (%.5f %.5f %.5f)\n",
					 k, c->bodyA, c->bodyB, s.st.pairs[k].colour, c->flags, c->eP, toy_val( c->normal.x, S_Q ), toy_val( c->normal.y, S_Q ),
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
		sim_record( &s, t, r.ticks + t, 1 );
		if ( keepPos )
		{
			record_poses( r.pos + (size_t)t * (size_t)n * POSW, s.pose, n );
		}
		if ( traj && t % trajEvery == 0 )
		{
			memset( awakeNow, 0, (size_t)n );
			for ( int i = 0; i < s.st.awakeCount; ++i )
			{
				awakeNow[s.st.awake[i]] = 1;
			}
			record_traj( traj, trajBuf, t, s.state, s.pose, s.isStatic, awakeNow, n );
		}
		sim_end_tick( &s );
	}
	pool_stop();
	r.msPerTick = total / ticks;
	r.saturations = toy_saturations();
	r.dispatches = s.mostDispatches;
	{
		const Manifold* last = s.mf[ticks & 1];
		r.finalSep = 1e30;
		r.finalA = -1;
		r.finalB = -1;
		for ( int k = 0; k < s.st.pairCount; ++k )
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
	free( awakeNow );
	free( trajBuf );
	sim_free( &s );
	return r;
}

static uint64_t run_hash( const RunResult* r, int ticks )
{
	uint64_t h = 1469598103934665603ULL;
	for ( int t = 1; t <= ticks; ++t )
	{
		h = fnv( h, &r->ticks[t].bodies, 8 );
		h = fnv( h, &r->ticks[t].manifolds, 8 );
		h = fnv( h, &r->ticks[t].stages, 8 );
		if ( r->joints > 0 ) // (a scene without joints hashes as before step 6)
		{
			h = fnv( h, &r->ticks[t].joints, 8 );
		}
	}
	return h;
}

// The first tick whose hashes differ (-1: none), and which
static int first_difference( const RunResult* a, const RunResult* b, int ticks, const char** what )
{
	for ( int t = 1; t <= ticks; ++t )
	{
		const TickRecord* x = a->ticks + t;
		const TickRecord* y = b->ticks + t;
		if ( x->bodies != y->bodies || x->manifolds != y->manifolds || x->stages != y->stages || x->joints != y->joints )
		{
			*what = x->bodies != y->bodies ? "bodies" : x->manifolds != y->manifolds ? "manifolds" : x->stages != y->stages ? "stages" : "joints";
			return t;
		}
	}
	*what = "";
	return -1;
}

// ---------------------------------------------------------------------------------------------------------------------
// Reference hash files: the config, the hashes every tick to 120 then every 60th (and the last), the run hash
// ---------------------------------------------------------------------------------------------------------------------

static int ref_kept( int t, int ticks )
{
	return t <= 120 || t % 60 == 0 || t == ticks;
}

static int ref_write( const char* path, const char* config, const RunResult* r, int ticks )
{
	FILE* f = fopen( path, "w" );
	if ( f == NULL )
	{
		return 0;
	}
	fprintf( f, "# toy reference hashes (toy.c --ref-out): bodies, manifolds and stages per tick to 120, then every 60th; the run hash "
				"covers every tick%s\n",
			 r->joints > 0 ? "; a scene with joints adds the joints' hash" : "" );
	fprintf( f, "config %s\n", config );
	fprintf( f, "twin %s\n", toy_twin_info() );
	for ( int t = 1; t <= ticks; ++t )
	{
		if ( ref_kept( t, ticks ) )
		{
			const TickRecord* k = r->ticks + t;
			fprintf( f, "tick %d %016llx %016llx %016llx", t, (unsigned long long)k->bodies, (unsigned long long)k->manifolds,
					 (unsigned long long)k->stages );
			if ( r->joints > 0 )
			{
				fprintf( f, " %016llx", (unsigned long long)k->joints );
			}
			fprintf( f, "\n" );
		}
	}
	fprintf( f, "run %016llx\n", (unsigned long long)run_hash( r, ticks ) );
	return fclose( f ) == 0;
}

typedef struct RefFile
{
	int ok;
	char config[256];
	char twin[160];
	int count;
	int* tick;
	uint64_t ( *h )[4];
	int hasJoints; // the tick lines carry the joints' hash
	uint64_t run;
} RefFile;

static RefFile ref_read( const char* path )
{
	RefFile ref;
	memset( &ref, 0, sizeof( ref ) );
	FILE* f = fopen( path, "r" );
	if ( f == NULL )
	{
		return ref;
	}
	char line[512];
	int cap = 0;
	while ( fgets( line, sizeof( line ), f ) )
	{
		line[strcspn( line, "\r\n" )] = 0;
		unsigned long long a, b, c, j = 0;
		int t;
		int got = 0;
		if ( strncmp( line, "config ", 7 ) == 0 )
		{
			snprintf( ref.config, sizeof( ref.config ), "%s", line + 7 );
		}
		else if ( strncmp( line, "twin ", 5 ) == 0 )
		{
			snprintf( ref.twin, sizeof( ref.twin ), "%s", line + 5 );
		}
		else if ( ( got = sscanf( line, "tick %d %llx %llx %llx %llx", &t, &a, &b, &c, &j ) ) >= 4 )
		{
			if ( ref.count == cap )
			{
				cap = cap ? 2 * cap : 256;
				ref.tick = (int*)realloc( ref.tick, (size_t)cap * sizeof( int ) );
				ref.h = (uint64_t( * )[4])realloc( ref.h, (size_t)cap * sizeof( *ref.h ) );
			}
			ref.tick[ref.count] = t;
			ref.h[ref.count][0] = a;
			ref.h[ref.count][1] = b;
			ref.h[ref.count][2] = c;
			ref.h[ref.count][3] = got == 5 ? j : 0;
			ref.hasJoints |= got == 5;
			ref.count += 1;
		}
		else if ( sscanf( line, "run %llx", &a ) == 1 )
		{
			ref.run = a;
			ref.ok = 1;
		}
	}
	fclose( f );
	return ref;
}

// One run against the reference: 1 when identical (the config, every kept tick, the run hash)
static int ref_check( const RefFile* ref, const char* config, const RunResult* r, int ticks, const char* who )
{
	if ( !ref->ok || strcmp( ref->config, config ) != 0 )
	{
		say( "ref: %s: the reference file is missing or of another config (file: %s)\n", who, ref->ok ? ref->config : "-" );
		return 0;
	}
	if ( ref->hasJoints != ( r->joints > 0 ) )
	{
		say( "ref: %s: the reference file %s the joints' hash, the scene has %d joints\n", who, ref->hasJoints ? "has" : "lacks", r->joints );
		return 0;
	}
	int lines = 0;
	for ( int i = 0; i < ref->count; ++i )
	{
		int t = ref->tick[i];
		if ( t < 1 || t > ticks )
		{
			say( "ref: %s: the file has tick %d, the run %d ticks\n", who, t, ticks );
			return 0;
		}
		const TickRecord* k = r->ticks + t;
		if ( k->bodies != ref->h[i][0] || k->manifolds != ref->h[i][1] || k->stages != ref->h[i][2] || k->joints != ref->h[i][3] )
		{
			say( "ref: %s DIFFERS from the reference at tick %d (%s)\n", who, t,
				 k->bodies != ref->h[i][0]		? "bodies"
				 : k->manifolds != ref->h[i][1] ? "manifolds"
				 : k->stages != ref->h[i][2]	? "stages"
												: "joints" );
			return 0;
		}
		lines += 1;
	}
	uint64_t h = run_hash( r, ticks );
	if ( h != ref->run )
	{
		say( "ref: %s DIFFERS from the reference: run hash %016llx, the file's %016llx (every kept tick equal)\n", who, (unsigned long long)h,
			 (unsigned long long)ref->run );
		return 0;
	}
	say( "ref: %s identical to the reference (%d tick lines and the run hash; the reference's twin: %s)\n", who, lines, ref->twin );
	return 1;
}

// ---------------------------------------------------------------------------------------------------------------------
// A GPU: the same tick on the kernels' SPIR-V. Two submits a tick: the prepare and the stages' readback (the AABBs, the
// sleep counters, last tick's point counts), then the lists and pairs uploaded, every dispatch with a barrier after
// it, and the hashes read back. Timestamps after every dispatch.
// ---------------------------------------------------------------------------------------------------------------------

#if HAS_GPU
typedef struct Gpu
{
	VkGpu g;
	int index;
	VkDescriptorSetLayout dsl;
	VkPipelineLayout layout;
	VkPipeline pipes[E_COUNT];
	VkBuf buf[TOY_BUFFERS]; // by binding; 13 and 14 hold mf[0] and mf[1] (the two sets swap them)
	size_t bytes[TOY_BUFFERS];
	VkBuf up, down, big; // staging: the tick's uploads, its readbacks, the trace's whole-buffer copies
	VkDescriptorPool dpool;
	VkDescriptorSet set[2]; // tick parity p: manifolds = mf[p], prevManifolds = mf[p ^ 1]
	VkQueryPool query;
	uint32_t queryCap;
	VkCommandBuffer cb;
	VkFence fence;
	VkBufferCopy* regions;
	int regionCap;
	// costs
	double stageMs[E_COUNT], dispatchCount[E_COUNT], kernelMs, wallMs, cpuMs;
	double peakWallMs, peakKernelMs; // the busiest tick's
	int ticks, peakTick;
} Gpu;

static int gpu_open( Gpu* G, VkInstance inst, VkPhysicalDevice phys, int index )
{
	memset( G, 0, sizeof( *G ) );
	G->index = index;
	vku_open_gpu( inst, phys, &G->g );
	VkDescriptorSetLayoutBinding bind[TOY_BUFFERS];
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		bind[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	}
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = TOY_BUFFERS;
	dci.pBindings = bind;
	VK_CHECK( vkCreateDescriptorSetLayout( G->g.device, &dci, NULL, &G->dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( Push ) };
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &G->dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VK_CHECK( vkCreatePipelineLayout( G->g.device, &lci, NULL, &G->layout ) );
	for ( int e = 0; e < E_COUNT; ++e )
	{
		char path[256];
		snprintf( path, sizeof( path ), "gen/toy/" DIALECT_NAME "/%s.spv", g_entryNames[e] );
		G->pipes[e] = vku_pipeline( &G->g, G->layout, path, "main" ); // slangc names the SPIR-V entry point main
		if ( G->pipes[e] == VK_NULL_HANDLE )
		{
			return 0;
		}
	}
	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 2 * TOY_BUFFERS };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 2;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VK_CHECK( vkCreateDescriptorPool( G->g.device, &pci, NULL, &G->dpool ) );
	VkDescriptorSetLayout layouts[2] = { G->dsl, G->dsl };
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = G->dpool;
	ai.descriptorSetCount = 2;
	ai.pSetLayouts = layouts;
	VK_CHECK( vkAllocateDescriptorSets( G->g.device, &ai, G->set ) );
	VkCommandBufferAllocateInfo cai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	cai.commandPool = G->g.cmdPool;
	cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cai.commandBufferCount = 1;
	VK_CHECK( vkAllocateCommandBuffers( G->g.device, &cai, &G->cb ) );
	VkFenceCreateInfo fci = { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	VK_CHECK( vkCreateFence( G->g.device, &fci, NULL, &G->fence ) );
	return 1;
}

static void gpu_close( Gpu* G )
{
	vkDeviceWaitIdle( G->g.device );
	for ( int e = 0; e < E_COUNT; ++e )
	{
		vkDestroyPipeline( G->g.device, G->pipes[e], NULL );
	}
	if ( G->query )
	{
		vkDestroyQueryPool( G->g.device, G->query, NULL );
	}
	vkDestroyFence( G->g.device, G->fence, NULL );
	vkFreeCommandBuffers( G->g.device, G->g.cmdPool, 1, &G->cb );
	vkDestroyDescriptorPool( G->g.device, G->dpool, NULL );
	vkDestroyPipelineLayout( G->g.device, G->layout, NULL );
	vkDestroyDescriptorSetLayout( G->g.device, G->dsl, NULL );
	free( G->regions );
	vku_close_gpu( &G->g );
}

static VkCommandBuffer gpu_begin( Gpu* G )
{
	VK_CHECK( vkResetCommandBuffer( G->cb, 0 ) );
	VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK( vkBeginCommandBuffer( G->cb, &bi ) );
	return G->cb;
}

// Ends, submits and waits; the transfers made visible to the host
static void gpu_submit( Gpu* G )
{
	VkMemoryBarrier mb = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT | VK_ACCESS_SHADER_WRITE_BIT;
	mb.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
	vkCmdPipelineBarrier( G->cb, VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &mb, 0,
						  NULL, 0, NULL );
	VK_CHECK( vkEndCommandBuffer( G->cb ) );
	VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.commandBufferCount = 1;
	si.pCommandBuffers = &G->cb;
	VK_CHECK( vkQueueSubmit( G->g.queue, 1, &si, G->fence ) );
	VK_CHECK( vkWaitForFences( G->g.device, 1, &G->fence, VK_TRUE, UINT64_MAX ) );
	VK_CHECK( vkResetFences( G->g.device, 1, &G->fence ) );
}

static void copy( VkCommandBuffer cb, VkBuf* src, size_t srcOff, VkBuf* dst, size_t dstOff, size_t bytes )
{
	if ( bytes > 0 )
	{
		VkBufferCopy c = { srcOff, dstOff, bytes };
		vkCmdCopyBuffer( cb, src->buffer, dst->buffer, 1, &c );
	}
}

static void ensure_staging( Gpu* G, VkBuf* b, size_t bytes )
{
	if ( b->buffer == VK_NULL_HANDLE || b->size < bytes )
	{
		vku_free( &G->g, b );
		*b = vku_buffer( &G->g, bytes < 256 ? 256 : bytes, true );
	}
}

static void ensure_query( Gpu* G, uint32_t count )
{
	if ( count > G->queryCap )
	{
		if ( G->query )
		{
			vkDestroyQueryPool( G->g.device, G->query, NULL );
		}
		G->queryCap = count + 64;
		VkQueryPoolCreateInfo qci = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
		qci.queryType = VK_QUERY_TYPE_TIMESTAMP;
		qci.queryCount = G->queryCap;
		VK_CHECK( vkCreateQueryPool( G->g.device, &qci, NULL, &G->query ) );
	}
}

static VkBufferCopy* ensure_regions( Gpu* G, int count )
{
	if ( count > G->regionCap )
	{
		G->regionCap = count + 256;
		G->regions = (VkBufferCopy*)realloc( G->regions, (size_t)G->regionCap * sizeof( VkBufferCopy ) );
	}
	return G->regions;
}

// The buffer sizes for a sim's capacities (16 bytes at least)
static void gpu_sizes( const Sim* s, size_t* b )
{
	const ToyData* d = s->init;
	size_t n = (size_t)s->n, pc = (size_t)s->pcap;
	size_t want[TOY_BUFFERS] = { (size_t)d->hullCount * sizeof( Hull ),
								 (size_t)d->pointCount * sizeof( V3 ),
								 (size_t)d->faceCount * sizeof( HullFace ),
								 (size_t)d->edgeCount * sizeof( uint32_t ),
								 n * sizeof( BodyState ),
								 n * sizeof( BodyPose ),
								 n * sizeof( BodyMass ),
								 n * sizeof( Aabb ),
								 LIST_WORDS( s ) * sizeof( uint32_t ),
								 sizeof( Params ),
								 n * sizeof( Hash2 ),
								 16,
								 pc * sizeof( Pair ),
								 pc * sizeof( Manifold ),
								 pc * sizeof( Manifold ),
								 pc * sizeof( SatAxis ),
								 sizeof( NarrowDiag ),
								 pc * sizeof( Hash2 ),
								 pc * sizeof( Constraint ),
								 (size_t)s->jc * sizeof( Joint ),
								 (size_t)s->jc * sizeof( Hash2 ),
								 (size_t)s->jc * sizeof( JointCommand ) };
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		b[i] = want[i] > 16 ? want[i] : 16;
	}
}

static void gpu_write_sets( Gpu* G )
{
	for ( int p = 0; p < 2; ++p )
	{
		VkDescriptorBufferInfo bi[TOY_BUFFERS];
		VkWriteDescriptorSet w[TOY_BUFFERS];
		for ( int i = 0; i < TOY_BUFFERS; ++i )
		{
			int slot = i == 13 ? 13 + p : i == 14 ? 13 + ( p ^ 1 ) : i;
			bi[i] = ( VkDescriptorBufferInfo ){ G->buf[slot].buffer, 0, VK_WHOLE_SIZE };
			w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
			w[i].dstSet = G->set[p];
			w[i].dstBinding = (uint32_t)i;
			w[i].descriptorCount = 1;
			w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
			w[i].pBufferInfo = &bi[i];
		}
		vkUpdateDescriptorSets( G->g.device, TOY_BUFFERS, w, 0, NULL );
	}
}

// Uploads host data into device buffers through the big staging buffer (one submit)
static void gpu_upload( Gpu* G, const int* slots, const void* const* src, const size_t* bytes, int count )
{
	size_t total = 0;
	for ( int i = 0; i < count; ++i )
	{
		total += ( bytes[i] + 15 ) & ~(size_t)15;
	}
	ensure_staging( G, &G->big, total );
	VkCommandBuffer cb = gpu_begin( G );
	size_t off = 0;
	for ( int i = 0; i < count; ++i )
	{
		memcpy( (uint8_t*)G->big.mapped + off, src[i], bytes[i] );
		copy( cb, &G->big, off, &G->buf[slots[i]], 0, bytes[i] );
		off += ( bytes[i] + 15 ) & ~(size_t)15;
	}
	vku_barrier( cb );
	gpu_submit( G );
}

// The buffers for a sim (its start uploaded, everything else zero), and the costs reset
static void gpu_start( Gpu* G, const Sim* s )
{
	gpu_sizes( s, G->bytes );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		G->buf[i] = vku_buffer( &G->g, G->bytes[i], false );
	}
	VkCommandBuffer cb = gpu_begin( G );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		vkCmdFillBuffer( cb, G->buf[i].buffer, 0, VK_WHOLE_SIZE, 0 );
	}
	vku_barrier( cb );
	gpu_submit( G );
	const ToyData* d = s->init;
	int slots[10] = { 0, 1, 2, 3, 4, 5, 6, 8, 9, 19 };
	const void* src[10] = { d->hulls, d->points, d->faces, d->edges, s->state, s->pose, s->mass, s->lists, &s->params, s->joints };
	size_t bytes[10] = { (size_t)d->hullCount * sizeof( Hull ),	 (size_t)d->pointCount * sizeof( V3 ), (size_t)d->faceCount * sizeof( HullFace ),
						 (size_t)d->edgeCount * sizeof( uint32_t ), (size_t)s->n * sizeof( BodyState ), (size_t)s->n * sizeof( BodyPose ),
						 (size_t)s->n * sizeof( BodyMass ),		 LIST_WORDS( s ) * sizeof( uint32_t ), sizeof( Params ),
						 (size_t)s->jc * sizeof( Joint ) };
	gpu_upload( G, slots, src, bytes, 10 );
	gpu_write_sets( G );
	memset( G->stageMs, 0, sizeof( G->stageMs ) );
	memset( G->dispatchCount, 0, sizeof( G->dispatchCount ) );
	G->kernelMs = G->wallMs = G->cpuMs = G->peakWallMs = G->peakKernelMs = 0.0;
	G->ticks = G->peakTick = 0;
}

static void gpu_stop( Gpu* G )
{
	vkDeviceWaitIdle( G->g.device );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		vku_free( &G->g, G->buf + i );
	}
	vku_free( &G->g, &G->up );
	vku_free( &G->g, &G->down );
	vku_free( &G->g, &G->big );
}

// The per-pair buffers (and the lists) for a grown capacity: the manifolds, SAT records, manifold hashes and
// constraints keep their contents, the rest zero (as the twin's)
static void gpu_grow( Gpu* G, const Sim* s )
{
	size_t want[TOY_BUFFERS];
	gpu_sizes( s, want );
	VkBuf old[TOY_BUFFERS];
	memset( old, 0, sizeof( old ) );
	VkCommandBuffer cb = gpu_begin( G );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		if ( want[i] == G->bytes[i] )
		{
			continue;
		}
		old[i] = G->buf[i];
		G->buf[i] = vku_buffer( &G->g, want[i], false );
		vkCmdFillBuffer( cb, G->buf[i].buffer, 0, VK_WHOLE_SIZE, 0 );
	}
	vku_barrier( cb );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		if ( old[i].buffer != VK_NULL_HANDLE && i != 8 && i != 12 ) // the lists and pairs are uploaded every tick
		{
			copy( cb, old + i, 0, G->buf + i, 0, G->bytes[i] );
		}
	}
	vku_barrier( cb );
	gpu_submit( G );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		if ( old[i].buffer != VK_NULL_HANDLE )
		{
			vku_free( &G->g, old + i );
			G->bytes[i] = want[i];
		}
	}
	gpu_write_sets( G );
}

static void record_dispatch( Gpu* G, VkCommandBuffer cb, const Dispatch* d, int* bound )
{
	if ( d->entry != *bound )
	{
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, G->pipes[d->entry] );
		*bound = d->entry;
	}
	Push push = { d->start, d->count, d->aux, 0 };
	vkCmdPushConstants( cb, G->layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( push ), &push );
	vkCmdDispatch( cb, ( d->count + 63 ) / 64, 1, 1 );
	vku_barrier( cb );
}

// Timestamps [0, count) into per-entry times for the dispatches between them
static void gpu_times( Gpu* G, const Dispatch* list, int nd )
{
	uint64_t ts[2048];
	if ( nd + 1 > 2048 )
	{
		return;
	}
	VK_CHECK( vkGetQueryPoolResults( G->g.device, G->query, 0, (uint32_t)( nd + 1 ), (size_t)( nd + 1 ) * 8, ts, 8,
									 VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
	uint64_t mask = G->g.timestampValidBits >= 64 ? ~0ULL : ( ( 1ULL << G->g.timestampValidBits ) - 1 );
	for ( int i = 0; i < nd; ++i )
	{
		double ms = (double)( ( ts[i + 1] - ts[i] ) & mask ) * (double)G->g.timestampPeriod * 1e-6;
		G->stageMs[list[i].entry] += ms;
		G->dispatchCount[list[i].entry] += 1.0;
		G->kernelMs += ms;
	}
}

// The tick's first submit: the prepare list uploaded, the prepare, the stages' readback into the sim
static void gpu_prepare( Gpu* G, Sim* s, int t )
{
	Dispatch prep;
	int np = sim_list_prepare( s, &prep );
	int prevCount = s->st.prevCount;
	size_t n = (size_t)s->n;
	size_t aOff = 0, sOff = n * sizeof( Aabb ), pOff = sOff + n * 4;
	ensure_staging( G, &G->up, LIST_WORDS( s ) * 4 + (size_t)s->pcap * sizeof( Pair ) );
	ensure_staging( G, &G->down, pOff + (size_t)( prevCount > 0 ? prevCount : 1 ) * 4 + n * 8 + (size_t)s->pcap * 8 );
	ensure_query( G, 2 );
	VkCommandBuffer cb = gpu_begin( G );
	memcpy( G->up.mapped, s->lists + L_PREP, (size_t)s->prepCount * 4 );
	copy( cb, &G->up, 0, &G->buf[8], 0, (size_t)s->prepCount * 4 );
	vku_barrier( cb );
	vkCmdResetQueryPool( cb, G->query, 0, 2 );
	vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, G->layout, 0, 1, &G->set[t & 1], 0, NULL );
	vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, G->query, 0 );
	int bound = -1;
	for ( int i = 0; i < np; ++i )
	{
		record_dispatch( G, cb, &prep, &bound );
	}
	vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, G->query, 1 );
	copy( cb, &G->buf[7], 0, &G->down, aOff, n * sizeof( Aabb ) );
	VkBufferCopy* r = ensure_regions( G, (int)n + prevCount );
	for ( size_t i = 0; i < n; ++i )
	{
		r[i] = ( VkBufferCopy ){ i * sizeof( BodyState ) + offsetof( BodyState, sleepTicks ), sOff + i * 4, 4 };
	}
	vkCmdCopyBuffer( cb, G->buf[4].buffer, G->down.buffer, (uint32_t)n, r );
	if ( prevCount > 0 )
	{
		for ( int k = 0; k < prevCount; ++k )
		{
			r[k] = ( VkBufferCopy ){ (size_t)k * sizeof( Manifold ) + offsetof( Manifold, pointCount ), pOff + (size_t)k * 4, 4 };
		}
		vkCmdCopyBuffer( cb, G->buf[13 + ( ( t & 1 ) ^ 1 )].buffer, G->down.buffer, (uint32_t)prevCount, r );
	}
	gpu_submit( G );
	const uint8_t* down = (const uint8_t*)G->down.mapped;
	memcpy( s->aabbs, down + aOff, n * sizeof( Aabb ) );
	memcpy( s->sleepTicks, down + sOff, n * 4 );
	for ( int k = 0; k < prevCount; ++k )
	{
		int32_t c;
		memcpy( &c, down + pOff + (size_t)k * 4, 4 );
		s->prevTouching[k] = (uint8_t)( c > 0 );
	}
	gpu_times( G, &prep, np );
}

// Dispatches [first, first + count) of the tick's list after the stages; with `full`, the lists and pairs uploaded first
// and the hashes read back after
static void gpu_rest( Gpu* G, Sim* s, int t, int first, int count, int full )
{
	size_t n = (size_t)s->n, hb = n * sizeof( Hash2 ), mb = (size_t)s->st.pairCount * sizeof( Hash2 );
	size_t lb = LIST_WORDS( s ) * 4, pb = (size_t)s->st.pairCount * sizeof( Pair );
	size_t jb = (size_t)s->jc * sizeof( JointCommand ), jhb = (size_t)s->jc * sizeof( Hash2 );
	size_t pcb = ( (size_t)s->pcap * sizeof( Pair ) + 15 ) & ~(size_t)15; // the commands after the pairs' room
	ensure_query( G, (uint32_t)count + 1 );
	if ( full )
	{
		ensure_staging( G, &G->up, lb + pcb + jb );
		ensure_staging( G, &G->down, hb + (size_t)s->pcap * sizeof( Hash2 ) + jhb );
	}
	VkCommandBuffer cb = gpu_begin( G );
	if ( full )
	{
		memcpy( G->up.mapped, s->lists, lb );
		memcpy( (uint8_t*)G->up.mapped + lb, s->st.pairs, pb );
		memcpy( (uint8_t*)G->up.mapped + lb + pcb, s->jcmds, jb );
		copy( cb, &G->up, 0, &G->buf[8], 0, lb );
		copy( cb, &G->up, lb, &G->buf[12], 0, pb );
		copy( cb, &G->up, lb + pcb, &G->buf[21], 0, jb );
		vku_barrier( cb );
	}
	vkCmdResetQueryPool( cb, G->query, 0, (uint32_t)count + 1 );
	vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, G->layout, 0, 1, &G->set[t & 1], 0, NULL );
	vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, G->query, 0 );
	int bound = -1;
	for ( int i = 0; i < count; ++i )
	{
		record_dispatch( G, cb, s->list + first + i, &bound );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, G->query, (uint32_t)( i + 1 ) );
	}
	if ( full )
	{
		copy( cb, &G->buf[10], 0, &G->down, 0, hb );
		copy( cb, &G->buf[17], 0, &G->down, hb, mb );
		copy( cb, &G->buf[20], 0, &G->down, hb + mb, jhb );
	}
	gpu_submit( G );
	if ( full )
	{
		memcpy( s->hashes, G->down.mapped, hb );
		memcpy( s->mhashes, (const uint8_t*)G->down.mapped + hb, mb );
		memcpy( s->jhashes, (const uint8_t*)G->down.mapped + hb + mb, jhb );
	}
	gpu_times( G, s->list + first, count );
}

// One tick on the GPU: the prepare and readback, the stages, the rest; the sim's hashes filled
static void gpu_tick( Gpu* G, Sim* s, int t )
{
	double t0 = vku_now_ms(), k0 = G->kernelMs;
	gpu_prepare( G, s, t );
	double c0 = vku_now_ms();
	int grown = sim_stages( s, t );
	G->cpuMs += vku_now_ms() - c0;
	if ( grown )
	{
		gpu_grow( G, s );
	}
	gpu_rest( G, s, t, 0, s->listCount, 1 );
	double wall = vku_now_ms() - t0;
	G->wallMs += wall;
	G->ticks += 1;
	if ( t > 1 && G->kernelMs - k0 > G->peakKernelMs ) // tick 1 warms the driver up
	{
		G->peakKernelMs = G->kernelMs - k0;
		G->peakWallMs = wall;
		G->peakTick = t;
	}
}

static uint32_t gpu_saturations( Gpu* G )
{
	ensure_staging( G, &G->down, 16 );
	VkCommandBuffer cb = gpu_begin( G );
	vku_barrier( cb );
	copy( cb, &G->buf[11], 0, &G->down, 0, 16 );
	gpu_submit( G );
	uint32_t c[4];
	memcpy( c, G->down.mapped, 16 );
	return c[0];
}

// Every buffer the kernels write, read back into a snapshot (the trace)
static void gpu_download( Gpu* G, const Sim* s, Snap* out )
{
	View like = sim_view( s );
	snap_size( out, &like );
	size_t total = 0;
	for ( int i = 0; i < V_COUNT; ++i )
	{
		total += ( like.bytes[i] + 15 ) & ~(size_t)15;
	}
	ensure_staging( G, &G->big, total );
	VkCommandBuffer cb = gpu_begin( G );
	vku_barrier( cb );
	size_t off = 0;
	for ( int i = 0; i < V_COUNT; ++i )
	{
		copy( cb, &G->buf[g_viewBinding[i]], 0, &G->big, off, like.bytes[i] );
		off += ( like.bytes[i] + 15 ) & ~(size_t)15;
	}
	gpu_submit( G );
	off = 0;
	for ( int i = 0; i < V_COUNT; ++i )
	{
		memcpy( out->v.p[i], (const uint8_t*)G->big.mapped + off, like.bytes[i] );
		off += ( like.bytes[i] + 15 ) & ~(size_t)15;
	}
}

// A snapshot's buffers into the GPU's (the trace's replay from a tick's start)
static void gpu_restore( Gpu* G, const Snap* from )
{
	int slots[V_COUNT];
	const void* src[V_COUNT];
	size_t bytes[V_COUNT];
	for ( int i = 0; i < V_COUNT; ++i )
	{
		slots[i] = g_viewBinding[i];
		src[i] = from->v.p[i];
		bytes[i] = from->v.bytes[i];
	}
	gpu_upload( G, slots, src, bytes, V_COUNT );
}

static RunResult run_gpu( Gpu* G, const ToyData* init, int ticks )
{
	RunResult r;
	memset( &r, 0, sizeof( r ) );
	r.joints = init->jointCount;
	Sim s;
	sim_init( &s, init );
	gpu_start( G, &s );
	r.ticks = (TickRecord*)calloc( (size_t)ticks + 1, sizeof( TickRecord ) );
	for ( int t = 1; t <= ticks; ++t )
	{
		gpu_tick( G, &s, t );
		sim_record( &s, t, r.ticks + t, 0 );
		sim_end_tick( &s );
	}
	r.saturations = gpu_saturations( G );
	r.msPerTick = G->wallMs / ticks;
	r.dispatches = s.mostDispatches;
	gpu_stop( G );
	sim_free( &s );
	return r;
}

static void gpu_costs( const Gpu* G, int dispatches )
{
	double k = G->ticks > 0 ? 1.0 / G->ticks : 0.0;
	say( "gpu %d costs: wall %.3f ms/tick (two submits, the readbacks and the CPU stages' %.3f ms), kernels %.3f ms/tick; the busiest "
		 "tick after the first (%d): kernels %.3f ms, wall %.3f ms; at most %d dispatches a tick; per stage (ms/tick, dispatches/tick):",
		 G->index, G->wallMs * k, G->cpuMs * k, G->kernelMs * k, G->peakTick, G->peakKernelMs, G->peakWallMs, dispatches );
	for ( int e = 0; e < E_COUNT; ++e )
	{
		if ( G->dispatchCount[e] > 0 )
		{
			say( " %s %.4f (%.1f)", g_entryNames[e], G->stageMs[e] * k, G->dispatchCount[e] * k );
		}
	}
	say( "\n" );
}

// TOY_TRACE_POKE=T:D tests the trace: after dispatch D of tick T the twin's first awake body has the low bit of its v.y
// flipped, as if that kernel had differed there
static int g_pokeTick = -1, g_pokeDispatch = -1;

static void trace_poke( Sim* s, int t, int dispatch )
{
	if ( t == g_pokeTick && dispatch == g_pokeDispatch && s->st.awakeCount > 0 )
	{
		BodyState* b = s->state + s->st.awake[0];
		uint32_t w;
		memcpy( &w, &b->v.y, 4 );
		w ^= 1u;
		memcpy( &b->v.y, &w, 4 );
	}
}

// --trace: the twin (one thread) and the GPU in step, every buffer compared after every tick's prepare and after its
// rest; the first tick whose rest differs is stepped again from the prepare's state, dispatch by dispatch, and the first
// differing word named. Returns 1 when no difference was found.
static int trace_gpu( Gpu* G, const ToyData* init, int T )
{
	Sim tw, gs;
	sim_init( &tw, init );
	sim_init( &gs, init );
	gpu_start( G, &gs );
	pool_start( 1 );
	if ( getenv( "TOY_TRACE_POKE" ) != NULL && sscanf( getenv( "TOY_TRACE_POKE" ), "%d:%d", &g_pokeTick, &g_pokeDispatch ) == 2 )
	{
		say( "trace gpu %d: TOY_TRACE_POKE: the twin's v.y flipped after tick %d dispatch %d (a test)\n", G->index, g_pokeTick, g_pokeDispatch );
	}
	Snap atPrep, got;
	memset( &atPrep, 0, sizeof( atPrep ) );
	memset( &got, 0, sizeof( got ) );
	int clean = 1;
	for ( int t = 1; t <= T && clean; ++t )
	{
		Dispatch prep;
		sim_bind( &tw, t );
		int np = sim_list_prepare( &tw, &prep );
		twin_list( &tw, &prep, np, t, 0, NULL );
		trace_poke( &tw, t, 0 );
		sim_gather( &tw, t );
		gpu_prepare( G, &gs, t );
		gpu_download( G, &gs, &got );
		View tv = sim_view( &tw );
		if ( view_compare( &tv, &got.v, "twin", "gpu", 0 ) > 0 )
		{
			say( "trace gpu %d: tick %d dispatch 0 (%s start 0 count %u) differs:\n", G->index, t, g_entryNames[E_PREPARE], tw.prepCount );
			view_compare( &tv, &got.v, "twin", "gpu", 1 );
			clean = 0;
			break;
		}
		sim_stages( &tw, t );
		sim_bind( &tw, t );
		if ( sim_stages( &gs, t ) )
		{
			gpu_grow( G, &gs );
		}
		if ( stages_hash( &tw.st ) != stages_hash( &gs.st ) )
		{
			say( "trace gpu %d: tick %d: the CPU stages differ on the GPU's readback (its buffers equal the twin's after the prepare)\n", G->index, t );
			clean = 0;
			break;
		}
		tv = sim_view( &tw );
		snap_take( &atPrep, &tv );
		for ( int i = 0; i < tw.listCount; ++i )
		{
			twin_list( &tw, tw.list + i, 1, t, 1 + i, NULL );
			trace_poke( &tw, t, 1 + i );
		}
		gpu_rest( G, &gs, t, 0, gs.listCount, 1 );
		gpu_download( G, &gs, &got );
		tv = sim_view( &tw );
		if ( view_compare( &tv, &got.v, "twin", "gpu", 0 ) == 0 )
		{
			sim_end_tick( &tw );
			sim_end_tick( &gs );
			continue;
		}
		// again from the prepare's state, dispatch by dispatch
		clean = 0;
		for ( int i = 0; i < V_COUNT; ++i )
		{
			memcpy( tv.p[i], atPrep.v.p[i], tv.bytes[i] );
		}
		gpu_restore( G, &atPrep );
		int found = 0;
		for ( int i = 0; i < tw.listCount && !found; ++i )
		{
			twin_list( &tw, tw.list + i, 1, t, 1 + i, NULL );
			trace_poke( &tw, t, 1 + i );
			gpu_rest( G, &gs, t, i, 1, 0 );
			gpu_download( G, &gs, &got );
			tv = sim_view( &tw );
			if ( view_compare( &tv, &got.v, "twin", "gpu", 0 ) > 0 )
			{
				const Dispatch* d = tw.list + i;
				say( "trace gpu %d: tick %d dispatch %d (%s start %u count %u) differs:\n", G->index, t, 1 + i, g_entryNames[d->entry], d->start,
					 d->count );
				view_compare( &tv, &got.v, "twin", "gpu", 1 );
				found = 1;
			}
		}
		if ( !found )
		{
			say( "trace gpu %d: tick %d differs at its end but not dispatch by dispatch from its prepare's state (not reproducible)\n", G->index, t );
			view_compare( &tv, &got.v, "twin", "gpu", 1 );
		}
	}
	if ( clean )
	{
		say( "trace gpu %d: no difference in %d ticks (every buffer after every tick)\n", G->index, T );
	}
	pool_stop();
	snap_free( &atPrep );
	snap_free( &got );
	gpu_stop( G );
	sim_free( &tw );
	sim_free( &gs );
	return clean;
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------------------------------------------------

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

// The start every dialect shares: each quantised position, orientation and velocity converted back to double against the
// scene's (scene_round_start makes them floats on V4's grids). Returns the number that differ.
static int start_check( const Scene* sc, const ToyData* d )
{
	int bad = 0, first = -1;
	for ( int i = 0; i < sc->bodyCount; ++i )
	{
		const SceneBody* b = sc->bodies + i;
		const BodyPose* P = d->pose + i;
		const BodyState* S = d->state + i;
		double got[13] = { toy_pos_x( &P->p ),		 toy_pos_y( &P->p ),		toy_pos_z( &P->p ),		  toy_val( P->q.x, S_Q ),
						   toy_val( P->q.y, S_Q ),	 toy_val( P->q.z, S_Q ),	toy_val( P->q.s, S_Q ),	  toy_val( S->v.x, S_V ),
						   toy_val( S->v.y, S_V ),	 toy_val( S->v.z, S_V ),	toy_val( S->w.x, S_W ),	  toy_val( S->w.y, S_W ),
						   toy_val( S->w.z, S_W ) };
		double want[13] = { b->p[0], b->p[1], b->p[2], b->q[0], b->q[1], b->q[2], b->q[3], b->v[0], b->v[1], b->v[2], b->w[0], b->w[1], b->w[2] };
		for ( int k = 0; k < 13; ++k )
		{
			if ( got[k] != want[k] )
			{
				first = first < 0 ? i : first;
				bad += 1;
			}
		}
	}
	say( "start: %s in %s (the grid moved %d values off their float)", bad ? "NOT EXACT" : "every position, orientation and velocity exact",
		 DIALECT_NAME, sc->startMoved );
	if ( bad )
	{
		say( ": %d values differ, the first in body %d", bad, first );
	}
	say( "\n" );
	return bad;
}

// The scene's joints, one line each in the scene's doubles (metrics.py reads them: the angles, gaps and overshoots are
// measured from the trajectories; b3ref2 prints the same lines)
static void joint_lines( const Scene* sc )
{
	for ( int k = 0; k < sc->jointCount; ++k )
	{
		char line[640];
		scene_joint_text( sc->joints + k, k, line, (int)sizeof( line ) );
		say( "%s\n", line );
	}
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

// Positions against a reference (D's): rms and max per window, dynamic bodies only
static int pos_report( const char* posRef, const RunResult* r0, const Scene* sc, int bodyCount, int ticks )
{
	FILE* f = fopen( posRef, "rb" );
	int32_t hdr[3] = { 0, 0, 0 };
	int failures = 0;
	if ( f == NULL || fread( hdr, sizeof( hdr ), 1, f ) != 1 || hdr[0] != bodyCount || hdr[1] < ticks || hdr[2] != POSW )
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
				const double* p = r0->pos + (size_t)t * (size_t)bodyCount * POSW;
				const double* q = ref + (size_t)t * (size_t)bodyCount * POSW;
				for ( int i = 0; i < bodyCount; ++i )
				{
					if ( sc->bodies[i].isStatic )
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
			say( "pose vs %s, ticks %d-%d: position rms %.3g m, max %.3g m (tick %d, body %d); rotation rms %.3g rad, max %.3g rad\n", posRef, a, b,
				 sqrt( sum / (double)( cntw ? cntw : 1 ) ), sqrt( mx ), at, atBody, sqrt( rsum / (double)( cntw ? cntw : 1 ) ), sqrt( rmx ) );
		}
		free( ref );
	}
	if ( f )
	{
		fclose( f );
	}
	return failures;
}

// The contact solve at a glance: when every dynamic body is asleep for good, the deepest manifold point (its separation
// at the start of a tick, before the solve) over the run and at rest: the 60 ticks before everything sleeps (the last
// 60 when it does not)
static void solve_report( const RunResult* r0, int ticks )
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
	const char* refPath = NULL;
	const char* refOut = NULL;
	const char* dhPath = NULL;
	int gpuMask = 0;
	int traceTicks = 0;
	ToySettings settings;
	toy_default_settings( &settings );
	int pushBody = -1;
	double pushV[3] = { 0.0, 0.0, 0.0 };
	double perturb = 0.0;
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
		else if ( strcmp( a, "--gpus" ) == 0 )
			gpuMask = (int)strtol( v, NULL, 0 ), ++i;
		else if ( strcmp( a, "--gpu" ) == 0 )
			gpuMask = 1 << atoi( v ), ++i;
		else if ( strcmp( a, "--trace" ) == 0 )
			traceTicks = atoi( v ), ++i;
		else if ( strcmp( a, "--dispatch-hashes" ) == 0 )
			dhPath = v, ++i;
		else if ( strcmp( a, "--ref" ) == 0 )
			refPath = v, ++i;
		else if ( strcmp( a, "--ref-out" ) == 0 )
			refOut = v, ++i;
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
	if ( traceTicks > 0 && gpuMask == 0 )
	{
		gpuMask = 0xff;
	}
	ticks = ticks < 1 ? 1 : ticks;

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
	ToyData d;
	toy_quantize( &sc, &settings, &d );
	// what the hashes depend on (the reference files' config line)
	char config[256];
	char pushText[96] = "-";
	if ( pushBody >= 0 )
	{
		snprintf( pushText, sizeof( pushText ), "%d,%g,%g,%g", pushBody, pushV[0], pushV[1], pushV[2] );
	}
	snprintf( config, sizeof( config ), "scene %s seed %llu dialect %s ticks %d gravity %g sleep %d recycle %d push %s perturb %g", sceneName,
			  (unsigned long long)seed, DIALECT_NAME, ticks, settings.gravity, settings.enableSleep, settings.recycle, pushText, perturb );
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
	say( "\n" );
	say( "twin: %s\n", toy_twin_info() );
	size_t cs[16] = { sizeof( Hull ),  sizeof( HullFace ), sizeof( BodyState ), sizeof( BodyPose ), sizeof( BodyMass ),
					  sizeof( Aabb ),  sizeof( Params ),   sizeof( Hash2 ),		sizeof( V3 ),		sizeof( Pair ),
					  sizeof( Manifold ), sizeof( SatAxis ), sizeof( NarrowDiag ), sizeof( Constraint ), sizeof( Joint ), sizeof( JointCommand ) };
	static const char* csn[16] = { "Hull",	   "HullFace", "BodyState", "BodyPose",	  "BodyMass",	"Aabb",	 "Params",		 "Hash2",
								   "V3",	   "Pair",	   "Manifold",	"SatAxis",	  "NarrowDiag", "Constraint", "Joint", "JointCommand" };
	int layoutBad = 0;
	say( "layout sizes C/twin:" );
	for ( int i = 0; i < 16; ++i )
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
	start_check( &sc, &d );
	joint_lines( &sc );

	RunResult runs[4];
	TrajWriter traj;
	int trajOk = trajPath ? traj_open( &traj, trajPath, (uint32_t)d.bodyCount, (uint32_t)trajEvery, 1.0 / 60.0 ) : 0;
	if ( trajPath && !trajOk )
	{
		say( "traj %s: cannot write\n", trajPath );
	}
	if ( dhPath )
	{
		g_dispatchHashes = fopen( dhPath, "w" );
		if ( g_dispatchHashes )
		{
			fprintf( g_dispatchHashes, "# toy dispatch hashes: tick, dispatch (0 the prepare), kernel, start, count, the hash of every buffer the "
									   "kernels write\n# config %s\n",
					 config );
		}
	}
	for ( int ti = 0; ti < tCount; ++ti )
	{
		runs[ti] = run_twin( &d, ticks, threads[ti], ti == 0 && ( posOut || posRef ), ti == 0 && trajOk ? &traj : NULL );
		if ( g_dispatchHashes )
		{
			fclose( g_dispatchHashes );
			g_dispatchHashes = NULL;
			say( "dispatch hashes: %s\n", dhPath );
		}
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
		 "islands(largest) slept woken%s\n",
		 d.jointCount > 0 ? " joints-hash" : "" );
	for ( int t = 1; t <= ticks; ++t )
	{
		if ( ref_kept( t, ticks ) )
		{
			const TickRecord* k = r0->ticks + t;
			say( "%4d  %016llx  %016llx  %016llx  %5d %5d %6d %8d %6d %4d(%d) %6d(%d) %5d %5d", t, (unsigned long long)k->bodies,
				 (unsigned long long)k->manifolds, (unsigned long long)k->stages, k->awake, k->pairs, k->active, k->touching, k->points, k->colours,
				 k->overflow, k->islands, k->largest, k->slept, k->woken );
			if ( d.jointCount > 0 )
			{
				say( " %016llx", (unsigned long long)k->joints );
			}
			say( "\n" );
		}
	}
	uint64_t runHash = run_hash( r0, ticks );
	int twinFailures = 0, failures = 0;
	for ( int ti = 0; ti < tCount; ++ti )
	{
		const char* what;
		int first = first_difference( runs + ti, r0, ticks, &what );
		say( "threads %d: %.3f ms/tick, saturations %u, floating-point sentinel %s%s%s; per-tick hashes %s", threads[ti], runs[ti].msPerTick,
			 runs[ti].saturations, runs[ti].fpFailures ? "TRIPPED " : "clean", runs[ti].fpFailures ? "at " : "", runs[ti].firstFp,
			 ti == 0 ? "(reference)" : first < 0 ? "identical" : "DIFFER" );
		if ( first >= 0 )
		{
			say( " from tick %d", first );
		}
		say( "\n" );
		twinFailures += first >= 0 || runs[ti].fpFailures > 0;
	}
	solve_report( r0, ticks );
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
		failures += pos_report( posRef, r0, &sc, d.bodyCount, ticks );
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

	// the reference hashes: written, or checked (the twin's runs; the GPUs' below)
	if ( refOut )
	{
		int ok = ref_write( refOut, config, r0, ticks );
		say( "ref-out %s: %s\n", refOut, ok ? "written" : "CANNOT WRITE" );
		failures += !ok;
	}
	RefFile ref;
	memset( &ref, 0, sizeof( ref ) );
	if ( refPath )
	{
		ref = ref_read( refPath );
		for ( int ti = 0; ti < tCount; ++ti )
		{
			char who[64];
			snprintf( who, sizeof( who ), "twin (threads %d)", threads[ti] );
			failures += !ref_check( &ref, config, runs + ti, ticks, who );
		}
	}

	// the GPUs: the same run beside the twin, per-tick hashes compared
#if HAS_GPU
	if ( gpuMask )
	{
		VkInstance inst = vku_create_instance();
		VkPhysicalDevice phys[8];
		int found = vku_list_gpus( inst, phys, 8 );
		for ( int gi = 0; gi < found; ++gi )
		{
			if ( !( gpuMask & ( 1 << gi ) ) )
			{
				continue;
			}
			Gpu G;
			if ( !gpu_open( &G, inst, phys[gi], gi ) )
			{
				say( "gpu %d: FAIL: no pipelines (gen/toy/%s)\n", gi, DIALECT_NAME );
				failures += 1;
				continue;
			}
			say( "gpu %d: %s (%s), driver 0x%x, api %u.%u.%u (F-plain: no float-control modes declared)\n", gi, G.g.props.deviceName, G.g.vendor,
				 G.g.props.driverVersion, VK_API_VERSION_MAJOR( G.g.props.apiVersion ), VK_API_VERSION_MINOR( G.g.props.apiVersion ),
				 VK_API_VERSION_PATCH( G.g.props.apiVersion ) );
			RunResult gr = run_gpu( &G, &d, ticks );
			const char* what;
			int first = first_difference( &gr, r0, ticks, &what );
			say( "gpu %d %s: per-tick hashes %s over %d ticks", gi, G.g.vendor, first < 0 ? "identical to the twin's" : "DIFFER from the twin's", ticks );
			if ( first >= 0 )
			{
				const TickRecord* a = gr.ticks + first;
				const TickRecord* b = r0->ticks + first;
				say( " from tick %d (%s; gpu %016llx %016llx %016llx, twin %016llx %016llx %016llx)", first, what, (unsigned long long)a->bodies,
					 (unsigned long long)a->manifolds, (unsigned long long)a->stages, (unsigned long long)b->bodies, (unsigned long long)b->manifolds,
					 (unsigned long long)b->stages );
			}
			say( "; run hash %016llx; saturations %u (twin %u)\n", (unsigned long long)run_hash( &gr, ticks ), gr.saturations, r0->saturations );
			failures += first >= 0 || gr.saturations != r0->saturations;
			if ( refPath )
			{
				char who[64];
				snprintf( who, sizeof( who ), "gpu %d %s", gi, G.g.vendor );
				failures += !ref_check( &ref, config, &gr, ticks, who );
			}
			gpu_costs( &G, gr.dispatches );
			free( gr.ticks );
			if ( traceTicks > 0 )
			{
				failures += !trace_gpu( &G, &d, traceTicks < ticks ? traceTicks : ticks );
			}
			gpu_close( &G );
		}
		vkDestroyInstance( inst, NULL );
	}
#else
	if ( gpuMask )
	{
		say( "gpus: dialect %s has no GPU kernels (twin only)\n", DIALECT_NAME );
	}
#endif
	free( ref.tick );
	free( ref.h );

	char jointText[48] = "";
	if ( d.jointCount > 0 )
	{
		snprintf( jointText, sizeof( jointText ), " joints %016llx", (unsigned long long)r0->ticks[ticks].joints );
	}
	say( "toy %s %s: run hash %016llx, final bodies %016llx manifolds %016llx stages %016llx%s, threads %s, saturations %u (twin: %s)\n",
		 DIALECT_NAME, sceneName, (unsigned long long)runHash, (unsigned long long)r0->ticks[ticks].bodies,
		 (unsigned long long)r0->ticks[ticks].manifolds, (unsigned long long)r0->ticks[ticks].stages, jointText,
		 twinFailures ? "DISAGREE or sentinel tripped" : "agree", r0->saturations, toy_twin_info() );
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
	return failures + twinFailures ? 1 : 0;
}
