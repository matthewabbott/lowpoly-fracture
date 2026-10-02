// SPDX-License-Identifier: MIT
// The state of a world at a tick as JSON (numbers, not pictures), for agents, tests and bug reports: the hashes, the
// counters, every body, piece, bond, physics contact, link, vehicle and rig, in index order, floats exact (%.9g; null
// when not finite). Built on the public inspection queries only.

#pragma once

#include "lpf/lpf.h"

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void lpDumpWorld( FILE* file, const lpWorld* world );

// "tick:path": dumps after that tick's step. Returns false for a malformed argument.
bool lpParseDumpArg( const char* arg, int64_t* tick, const char** path );

#ifdef __cplusplus
}
#endif
