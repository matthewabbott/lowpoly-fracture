// SPDX-License-Identifier: MIT
// lpf_bench --twin (twin.c): two worlds, a desync injected into one, detection, the cone, repair
#pragma once

#include "scenes.h"
#include "script.h"

// inject: "velocity:tick[:ulps]" or "warm:tick[:ulps]"; repair: NULL to watch, else any of "motion,warm,sleep", with
// "@ticks" of delay; conePath: NULL, or a file for the per-tick spread. Returns 0, or 1 if nothing could be injected
// or it went unseen.
int lpBenchTwin( int scene, int period, int ticks, int workers, const lpScript* script, const char* inject,
				 const char* repair, const char* conePath );
