// pool.h: the lab's thread pool for the CPU twins (E11's harness and the toy): the caller is thread 0, threads 1 to N - 1
// spin on a generation word; one job at a time, every thread runs its share, and the call returns when every thread is
// done (one barrier per dispatch, as a GPU's dispatch with a barrier after it). Header only (static functions).
//
//   LabPool pool;
//   lab_pool_start( &pool, threads );
//   lab_pool_run( &pool, fn, ctx );   // fn( ctx, t, threads ) on every thread t in [0, threads)
//   lab_pool_stop( &pool );
//
// The shared words: a generation the workers wait on, a count of finished workers, quit. Loads acquire and increments
// release, so a worker that sees a new generation sees its job, and the caller that sees every worker done sees their
// writes (x86 ordered these for free; ARM does not).
#ifndef LAB_POOL_H
#define LAB_POOL_H

#include <stdint.h>
#include <string.h>

#if defined( _WIN32 )
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
typedef volatile LONG LabPoolWord; // MSVC's volatile reads acquire and writes release on x64 (/volatile:ms)
#define LAB_POOL_LOAD( w ) ( *( w ) )
#define LAB_POOL_STORE( w, v ) InterlockedExchange( ( w ), ( v ) )
#define LAB_POOL_INC( w ) InterlockedIncrement( w )
#define LAB_POOL_PAUSE() YieldProcessor()
typedef HANDLE LabPoolThread;
#else
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
typedef atomic_long LabPoolWord;
#define LAB_POOL_LOAD( w ) atomic_load_explicit( ( w ), memory_order_acquire )
#define LAB_POOL_STORE( w, v ) atomic_store_explicit( ( w ), ( v ), memory_order_release )
#define LAB_POOL_INC( w ) atomic_fetch_add_explicit( ( w ), 1, memory_order_acq_rel )
#define LAB_POOL_PAUSE() sched_yield()
typedef pthread_t LabPoolThread;
#endif

#define LAB_POOL_MAX 64

typedef void ( *LabPoolFn )( void* ctx, int thread, int threads );

typedef struct LabPool LabPool;

typedef struct LabPoolArg
{
	LabPool* pool;
	int thread;
} LabPoolArg;

struct LabPool
{
	int threads;
	LabPoolThread handles[LAB_POOL_MAX];
	LabPoolArg args[LAB_POOL_MAX];
	LabPoolWord generation;
	LabPoolWord done;
	LabPoolWord quit;
	LabPoolFn fn; // the job: valid from the generation's release until every worker is done
	void* ctx;
};

// The workgroups [g0, g1) of a dispatch of `count` items (64 a group) that thread t of `threads` runs
static inline void lab_pool_groups( uint32_t count, int t, int threads, uint32_t* g0, uint32_t* g1 )
{
	uint32_t groups = ( count + 63 ) / 64;
	*g0 = (uint32_t)( ( (uint64_t)groups * (uint64_t)t ) / (uint64_t)threads );
	*g1 = (uint32_t)( ( (uint64_t)groups * (uint64_t)( t + 1 ) ) / (uint64_t)threads );
}

static inline void lab_pool_work( LabPool* p, int t )
{
	long seen = 0;
	for ( ;; )
	{
		long g;
		while ( ( g = LAB_POOL_LOAD( &p->generation ) ) == seen )
		{
			if ( LAB_POOL_LOAD( &p->quit ) )
			{
				return;
			}
			LAB_POOL_PAUSE();
		}
		seen = g;
		p->fn( p->ctx, t, p->threads );
		LAB_POOL_INC( &p->done );
	}
}

#if defined( _WIN32 )
static DWORD WINAPI lab_pool_worker( LPVOID arg )
{
	LabPoolArg* a = (LabPoolArg*)arg;
	lab_pool_work( a->pool, a->thread );
	return 0;
}
#else
static void* lab_pool_worker( void* arg )
{
	LabPoolArg* a = (LabPoolArg*)arg;
	lab_pool_work( a->pool, a->thread );
	return NULL;
}
#endif

static inline void lab_pool_start( LabPool* p, int threads )
{
	memset( p, 0, sizeof( *p ) );
	threads = threads < 1 ? 1 : threads > LAB_POOL_MAX ? LAB_POOL_MAX : threads;
	p->threads = threads;
	for ( int t = 1; t < threads; ++t )
	{
		p->args[t].pool = p;
		p->args[t].thread = t;
#if defined( _WIN32 )
		p->handles[t] = CreateThread( NULL, 0, lab_pool_worker, &p->args[t], 0, NULL );
#else
		pthread_create( &p->handles[t], NULL, lab_pool_worker, &p->args[t] );
#endif
	}
}

static inline void lab_pool_stop( LabPool* p )
{
	LAB_POOL_STORE( &p->quit, 1 );
	for ( int t = 1; t < p->threads; ++t )
	{
#if defined( _WIN32 )
		WaitForSingleObject( p->handles[t], INFINITE );
		CloseHandle( p->handles[t] );
#else
		pthread_join( p->handles[t], NULL );
#endif
	}
	p->threads = 0;
}

// Runs fn( ctx, t, threads ) on every thread (the caller is thread 0) and returns when all are done
static inline void lab_pool_run( LabPool* p, LabPoolFn fn, void* ctx )
{
	if ( p->threads <= 1 )
	{
		fn( ctx, 0, 1 );
		return;
	}
	p->fn = fn;
	p->ctx = ctx;
	LAB_POOL_STORE( &p->done, 0 );
	LAB_POOL_INC( &p->generation ); // releases the job and the reset count
	fn( ctx, 0, p->threads );
	while ( LAB_POOL_LOAD( &p->done ) < p->threads - 1 )
	{
		LAB_POOL_PAUSE();
	}
}

#endif
