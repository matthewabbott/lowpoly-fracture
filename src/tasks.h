// SPDX-License-Identifier: MIT
// A minimal thread pool with a blocking parallel-for. Work items are handed out one at a time under a lock, so
// it suits coarse items (a whole piece's fracture). Results must not depend on which worker ran an item.

#pragma once

#include <stdbool.h>

typedef struct lpTaskPool lpTaskPool;

// workerCount includes the calling thread; 1 creates no threads.
lpTaskPool* lpTaskPool_Create( int workerCount );
void lpTaskPool_Destroy( lpTaskPool* pool );

typedef void lpTaskFcn( int index, void* context );

// Runs fcn(i) for i in [0, count) across the pool and the caller, and returns when all are done.
void lpTaskPool_ParallelFor( lpTaskPool* pool, int count, lpTaskFcn* fcn, void* context );

// Floating-point control words the pool's threads found changed and put back (lpFpGuard), since the pool was made
int lpTaskPool_FpRepairs( lpTaskPool* pool );
