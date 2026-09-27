// SPDX-License-Identifier: MIT

#include "tasks.h"

#include "core.h"

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
typedef SRWLOCK lpMutex;
typedef CONDITION_VARIABLE lpCond;
typedef HANDLE lpThread;
#define lpMutexInit( m ) InitializeSRWLock( m )
#define lpMutexLock( m ) AcquireSRWLockExclusive( m )
#define lpMutexUnlock( m ) ReleaseSRWLockExclusive( m )
#define lpCondInit( c ) InitializeConditionVariable( c )
#define lpCondWait( c, m ) SleepConditionVariableSRW( c, m, INFINITE, 0 )
#define lpCondBroadcast( c ) WakeAllConditionVariable( c )
#else
#include <pthread.h>
typedef pthread_mutex_t lpMutex;
typedef pthread_cond_t lpCond;
typedef pthread_t lpThread;
#define lpMutexInit( m ) pthread_mutex_init( m, NULL )
#define lpMutexLock( m ) pthread_mutex_lock( m )
#define lpMutexUnlock( m ) pthread_mutex_unlock( m )
#define lpCondInit( c ) pthread_cond_init( c, NULL )
#define lpCondWait( c, m ) pthread_cond_wait( c, m )
#define lpCondBroadcast( c ) pthread_cond_broadcast( c )
#endif

#define LP_MAX_WORKERS 32

struct lpTaskPool
{
	int workerCount;
	lpThread threads[LP_MAX_WORKERS];

	lpMutex mutex;
	lpCond wake; // workers wait for a new batch
	lpCond done; // the caller waits for the batch to finish

	// current batch, guarded by mutex
	lpTaskFcn* fcn;
	void* context;
	int count;
	int next;
	int finished;
	uint64_t batch;
	bool quit;
};

// Take items until the batch is exhausted. Called with the mutex held; returns with it held.
static void lpDrain( lpTaskPool* pool )
{
	while ( pool->next < pool->count )
	{
		int index = pool->next++;
		lpTaskFcn* fcn = pool->fcn;
		void* context = pool->context;
		lpMutexUnlock( &pool->mutex );
		fcn( index, context );
		lpMutexLock( &pool->mutex );
		pool->finished += 1;
		if ( pool->finished == pool->count )
		{
			lpCondBroadcast( &pool->done );
		}
	}
}

#if defined( _WIN32 )
static DWORD WINAPI lpWorkerMain( LPVOID param )
#else
static void* lpWorkerMain( void* param )
#endif
{
	lpTaskPool* pool = param;
	uint64_t seen = 0;
	lpMutexLock( &pool->mutex );
	for ( ;; )
	{
		while ( pool->quit == false && pool->batch == seen )
		{
			lpCondWait( &pool->wake, &pool->mutex );
		}
		if ( pool->quit )
		{
			break;
		}
		seen = pool->batch;
		lpDrain( pool );
	}
	lpMutexUnlock( &pool->mutex );
#if defined( _WIN32 )
	return 0;
#else
	return NULL;
#endif
}

lpTaskPool* lpTaskPool_Create( int workerCount )
{
	lpTaskPool* pool = lpAlloc( sizeof( lpTaskPool ) );
	memset( pool, 0, sizeof( lpTaskPool ) );
	workerCount = workerCount < 1 ? 1 : ( workerCount > LP_MAX_WORKERS ? LP_MAX_WORKERS : workerCount );
	pool->workerCount = workerCount;
	lpMutexInit( &pool->mutex );
	lpCondInit( &pool->wake );
	lpCondInit( &pool->done );
	for ( int i = 1; i < workerCount; ++i )
	{
#if defined( _WIN32 )
		pool->threads[i] = CreateThread( NULL, 0, lpWorkerMain, pool, 0, NULL );
#else
		pthread_create( pool->threads + i, NULL, lpWorkerMain, pool );
#endif
	}
	return pool;
}

void lpTaskPool_Destroy( lpTaskPool* pool )
{
	if ( pool == NULL )
	{
		return;
	}
	lpMutexLock( &pool->mutex );
	pool->quit = true;
	lpCondBroadcast( &pool->wake );
	lpMutexUnlock( &pool->mutex );
	for ( int i = 1; i < pool->workerCount; ++i )
	{
#if defined( _WIN32 )
		WaitForSingleObject( pool->threads[i], INFINITE );
		CloseHandle( pool->threads[i] );
#else
		pthread_join( pool->threads[i], NULL );
#endif
	}
	lpFree( pool );
}

void lpTaskPool_ParallelFor( lpTaskPool* pool, int count, lpTaskFcn* fcn, void* context )
{
	if ( count <= 0 )
	{
		return;
	}
	if ( pool == NULL || pool->workerCount == 1 || count == 1 )
	{
		for ( int i = 0; i < count; ++i )
		{
			fcn( i, context );
		}
		return;
	}

	lpMutexLock( &pool->mutex );
	pool->fcn = fcn;
	pool->context = context;
	pool->count = count;
	pool->next = 0;
	pool->finished = 0;
	pool->batch += 1;
	lpCondBroadcast( &pool->wake );
	lpDrain( pool );
	while ( pool->finished < pool->count )
	{
		lpCondWait( &pool->done, &pool->mutex );
	}
	pool->count = 0;
	lpMutexUnlock( &pool->mutex );
}

