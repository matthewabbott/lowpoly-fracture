// SPDX-License-Identifier: MIT
// Headless benchmark: build a scene, bombard it on a fixed schedule, time every step.
//
//   lpf_bench --scene town --workers 1,4,8 --ticks 600 --period 12 --json bench.json
//
// Also checks determinism: the final state hash must be the same for every worker count.

#include "scenes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int CompareFloat( const void* a, const void* b )
{
	float x = *(const float*)a, y = *(const float*)b;
	return ( x > y ) - ( x < y );
}

typedef struct Summary
{
	float avg, p95, max;
} Summary;

static Summary Summarize( float* values, int count )
{
	Summary s = { 0 };
	double sum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		sum += values[i];
	}
	s.avg = (float)( sum / (double)count );
	qsort( values, (size_t)count, sizeof( float ), CompareFloat );
	s.p95 = values[(int)( 0.95f * (float)( count - 1 ) )];
	s.max = values[count - 1];
	return s;
}

typedef struct Result
{
	int workers;
	Summary total, fracture, physics, update;
	int maxPieces, maxBodies, maxAwakeDebris, maxRubble, impacts, fractures, cells;
	uint64_t hash, solverHash;
	float loadMs, settleMs; // building the scene, and settling its structures within that
	int settleIterations, pieces, bonds; // at load
	lpStats worst; // stats of the step with the largest fracture time
	double sumCell, sumHull, sumShape, sumBond, sumSplit;
	double sumVoronoiCpu, sumMergeCpu, sumHullCpu;
	double sumStress, maxStress;
	int stressIterations, stressBreaks, stressSolves, stressJudged, stressWaiting, stressReduced, stressAudits;
	int maxContacts, maxAwakeContacts, maxShapes;
	double sumAwakeContacts;
	int over16, over33; // ticks over a 60 Hz and a 30 Hz frame: the spikes a lockstep peer must absorb
} Result;

// Stress budgets from --stress-work (0: the world's defaults)
static int s_stressWork, s_stressStructureWork;

// --hash-log path: every tick's hashes go to path.w<workers>.txt (cross-platform checks diff them for the first
// differing tick)
static const char* s_hashLog;

// --tick-log path: every tick's step, fracture, physics and stress times go to path.w<workers>.txt (spikes over time)
static const char* s_tickLog;
static int s_physicsLag; // --physics-lag 1: milestone 8's experiment

static Result RunOnce( int scene, int workers, int ticks, int period, float fragmentScale, int maxDebris )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.physicsLag = s_physicsLag;
	ld.fragmentScale = fragmentScale;
	ld.maxFullDebris = maxDebris;
	ld.workerCount = workers;
	ld.maxStressWork = s_stressWork > 0 ? s_stressWork : ld.maxStressWork;
	ld.maxStressStructureWork = s_stressStructureWork > 0 ? s_stressStructureWork : ld.maxStressStructureWork;
	lpWorld* world = lpCreateWorld( &ld );
	Result r = { 0 };
	r.workers = workers;
	uint64_t loadTicks = lpGetTicks();
	lpBuildScene( world, scene );
	r.loadMs = lpGetMilliseconds( loadTicks );
	lpStats loaded = lpWorld_GetStats( world );
	r.settleMs = loaded.settleMs;
	r.settleIterations = loaded.settleIterations;
	r.bonds = loaded.bondCount;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( world ); ++i )
	{
		r.pieces += lpWorld_GetPieceInfo( world, i ).body >= 0 ? 1 : 0;
	}

	FILE* hashLog = NULL;
	if ( s_hashLog != NULL )
	{
		char path[512];
		snprintf( path, sizeof( path ), "%s.w%d.txt", s_hashLog, workers );
		hashLog = fopen( path, "w" );
		if ( hashLog != NULL )
		{
			fprintf( hashLog, "load %016llx %016llx\n", (unsigned long long)lpWorld_Hash( world ),
					 (unsigned long long)lpWorld_HashStress( world ) );
		}
	}

	FILE* tickLog = NULL;
	if ( s_tickLog != NULL )
	{
		char path[512];
		snprintf( path, sizeof( path ), "%s.w%d.txt", s_tickLog, workers );
		tickLog = fopen( path, "w" );
		if ( tickLog != NULL )
		{
			fprintf( tickLog, "tick stepMs fractureMs physicsMs stressMs pieces awakeDebris\n" );
		}
	}

	int samples = ticks > 0 ? ticks : 1;
	float* total = calloc( (size_t)samples, sizeof( float ) );
	float* fracture = calloc( (size_t)samples, sizeof( float ) );
	float* phys = calloc( (size_t)samples, sizeof( float ) );
	float* update = calloc( (size_t)samples, sizeof( float ) );

	for ( int tick = 0; tick < ticks; ++tick )
	{
		lpSceneBombard( world, scene, tick, period );
		lpSceneDrive( world, scene, tick, -1, -1 ); // the track's cars drive laps, the mech patrols
		uint64_t t0 = lpGetTicks();
		lpWorld_Step( world, 1.0f / 60.0f, 4 );
		total[tick] = lpGetMilliseconds( t0 );
		lpStats st = lpWorld_GetStats( world );
		fracture[tick] = st.fractureMs;
		if ( st.fractureMs > r.worst.fractureMs )
		{
			r.worst = st;
		}
		r.sumCell += st.cellMs;
		r.sumHull += st.hullMs;
		r.sumShape += st.shapeMs;
		r.sumBond += st.bondMs;
		r.sumSplit += st.splitMs;
		r.sumVoronoiCpu += st.voronoiCpuMs;
		r.sumStress += st.stressMs;
		r.maxStress = st.stressMs > r.maxStress ? st.stressMs : r.maxStress;
		r.stressIterations += st.stressIterations;
		r.stressBreaks += st.stressBreaks;
		r.stressSolves += st.stressSolves;
		r.stressJudged += st.stressJudged;
		r.stressReduced += st.stressReduced;
		r.stressAudits += st.stressAudits;
		r.stressWaiting += st.stressWaiting;
		r.sumMergeCpu += st.mergeCpuMs;
		r.sumHullCpu += st.hullCpuMs;
		phys[tick] = st.physicsMs;
		update[tick] = st.updateMs;
		int bodies = st.structureBodies + st.debrisBodies + st.rubbleBodies;
		r.maxPieces = st.pieceCount > r.maxPieces ? st.pieceCount : r.maxPieces;
		r.maxBodies = bodies > r.maxBodies ? bodies : r.maxBodies;
		r.maxAwakeDebris = st.awakeDebris > r.maxAwakeDebris ? st.awakeDebris : r.maxAwakeDebris;
		r.maxRubble = st.rubbleBodies > r.maxRubble ? st.rubbleBodies : r.maxRubble;
		r.impacts += st.impactsThisStep;
		r.fractures += st.fracturesThisStep;
		r.cells += st.cellsThisStep;
		r.maxContacts = st.contacts > r.maxContacts ? st.contacts : r.maxContacts;
		r.maxAwakeContacts = st.awakeContacts > r.maxAwakeContacts ? st.awakeContacts : r.maxAwakeContacts;
		r.maxShapes = st.shapes > r.maxShapes ? st.shapes : r.maxShapes;
		r.sumAwakeContacts += st.awakeContacts;
		r.over16 += total[tick] > 1000.0f / 60.0f ? 1 : 0;
		r.over33 += total[tick] > 1000.0f / 30.0f ? 1 : 0;
		if ( hashLog != NULL )
		{
			fprintf( hashLog, "%d %016llx %016llx\n", tick, (unsigned long long)lpWorld_Hash( world ),
					 (unsigned long long)lpWorld_HashStress( world ) );
		}
		if ( tickLog != NULL )
		{
			fprintf( tickLog, "%d %.3f %.3f %.3f %.3f %d %d\n", tick, (double)total[tick], (double)st.fractureMs,
					 (double)st.physicsMs, (double)st.stressMs, st.pieceCount, st.awakeDebris );
		}
	}
	if ( hashLog != NULL )
	{
		fclose( hashLog );
	}
	if ( tickLog != NULL )
	{
		fclose( tickLog );
	}
	r.sumAwakeContacts /= (double)samples;
	r.hash = lpWorld_Hash( world );
	r.solverHash = lpWorld_HashStress( world );
	r.total = Summarize( total, samples );
	r.fracture = Summarize( fracture, samples );
	r.physics = Summarize( phys, samples );
	r.update = Summarize( update, samples );

	free( total );
	free( fracture );
	free( phys );
	free( update );
	lpDestroyWorld( world );
	return r;
}


int main( int argc, char** argv )
{
	int scene = lp_sceneTown;
	int workers[8] = { 1, 4, 8 };
	int workerCount = 3;
	int ticks = 600;
	int period = 12;
	float fragmentScale = 1.0f;
	int maxDebris = 400;
	const char* jsonPath = NULL;

	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--scene" ) == 0 )
		{
			scene = lpSceneFromName( v );
			++i;
		}
		else if ( strcmp( a, "--workers" ) == 0 )
		{
			workerCount = 0;
			char buffer[64];
			strncpy( buffer, v, sizeof( buffer ) - 1 );
			buffer[sizeof( buffer ) - 1] = 0;
			for ( char* tok = strtok( buffer, "," ); tok != NULL && workerCount < 8; tok = strtok( NULL, "," ) )
			{
				workers[workerCount++] = atoi( tok );
			}
			++i;
		}
		else if ( strcmp( a, "--ticks" ) == 0 )
		{
			ticks = atoi( v );
			++i;
		}
		else if ( strcmp( a, "--period" ) == 0 )
		{
			period = atoi( v );
			++i;
		}
		else if ( strcmp( a, "--fragment-scale" ) == 0 )
		{
			fragmentScale = (float)atof( v );
			++i;
		}
		else if ( strcmp( a, "--max-debris" ) == 0 )
		{
			maxDebris = atoi( v );
			++i;
		}
		else if ( strcmp( a, "--stress-work" ) == 0 )
		{
			s_stressWork = atoi( v );
			const char* comma = strchr( v, ',' );
			s_stressStructureWork = comma != NULL ? atoi( comma + 1 ) : 0;
			++i;
		}
		else if ( strcmp( a, "--json" ) == 0 )
		{
			jsonPath = v;
			++i;
		}
		else if ( strcmp( a, "--hash-log" ) == 0 )
		{
			s_hashLog = v;
			++i;
		}
		else if ( strcmp( a, "--tick-log" ) == 0 )
		{
			s_tickLog = v;
			++i;
		}
		else if ( strcmp( a, "--physics-lag" ) == 0 )
		{
			s_physicsLag = atoi( v );
			++i;
		}
		else
		{
			printf( "usage: lpf_bench [--scene walls|house|town|tower|pile|lumber|ruins|yard|keep|track|mech] [--workers 1,4,8] [--ticks N]\n"
					"                 [--period N] [--fragment-scale F] [--max-debris N] [--stress-work total,perStructure] [--json path]\n"
					"                 [--hash-log path] [--tick-log path] [--physics-lag 0|1]\n" );
			return 1;
		}
	}

	printf( "scene %s, %d ticks at 60 Hz, a blast every %d ticks, fragment scale %.2f, debris cap %d\n",
			lpSceneName( scene ), ticks, period, (double)fragmentScale, maxDebris );
	printf( "workers | step avg   p95    max  | fracture avg  max | physics avg  p95 | update avg | max pieces bodies awake rubble | hash\n" );

	Result results[8];
	bool deterministic = true;
	for ( int w = 0; w < workerCount; ++w )
	{
		Result r = RunOnce( scene, workers[w], ticks, period, fragmentScale, maxDebris );
		results[w] = r;
		printf( "%7d | %6.2f %6.2f %6.2f | %8.2f %6.2f | %7.2f %6.2f | %6.2f     | %10d %6d %5d %6d | %016llx\n", r.workers,
				(double)r.total.avg, (double)r.total.p95, (double)r.total.max, (double)r.fracture.avg, (double)r.fracture.max,
				(double)r.physics.avg, (double)r.physics.p95, (double)r.update.avg, r.maxPieces, r.maxBodies, r.maxAwakeDebris,
				r.maxRubble, (unsigned long long)r.hash );
		deterministic = deterministic && r.hash == results[0].hash && r.solverHash == results[0].solverHash;
		printf( "        fracture total ms: cells %.0f hulls %.0f shapes %.0f bonds %.0f split %.0f | worst step %.1f ms: cells %.1f hulls %.1f "
				"shapes %.1f bonds %.1f split %.1f (%d fractures, %d cells)\n",
				r.sumCell, r.sumHull, r.sumShape, r.sumBond, r.sumSplit, (double)r.worst.fractureMs, (double)r.worst.cellMs,
				(double)r.worst.hullMs, (double)r.worst.shapeMs, (double)r.worst.bondMs, (double)r.worst.splitMs,
				r.worst.fracturesThisStep, r.worst.cellsThisStep );
		printf( "        box3d: shapes max %d, contacts max %d, awake contacts avg %.0f max %d\n", r.maxShapes, r.maxContacts,
				r.sumAwakeContacts, r.maxAwakeContacts );
		double jobCpu = r.sumVoronoiCpu + r.sumMergeCpu + r.sumHullCpu;
		printf( "        fracture job cpu ms: voronoi %.0f merge %.0f hulls %.0f (hulls %.0f%% of job time)\n", r.sumVoronoiCpu,
				r.sumMergeCpu, r.sumHullCpu, jobCpu > 0.0 ? 100.0 * r.sumHullCpu / jobCpu : 0.0 );
		printf( "        stress: avg %.3f ms, max %.2f ms, %d iterations, %d joints broke, %d solves (%d judged, %.1f steps each; "
				"%d on reduced systems, %d audits), %d waits for budget; solver %016llx\n",
				r.sumStress / (double)( ticks > 0 ? ticks : 1 ), r.maxStress, r.stressIterations, r.stressBreaks, r.stressSolves,
				r.stressJudged, r.stressJudged > 0 ? (double)r.stressSolves / (double)r.stressJudged : 0.0, r.stressReduced,
				r.stressAudits, r.stressWaiting,
				(unsigned long long)r.solverHash );
		printf( "        load: %.1f ms for %d pieces and %d bonds, settling %.1f ms of it (%d iterations)\n", (double)r.loadMs,
				r.pieces, r.bonds, (double)r.settleMs, r.settleIterations );
	}
	printf( "impacts %d, fractures %d, cells %d\n", results[0].impacts, results[0].fractures, results[0].cells );
	printf( "deterministic across worker counts: %s\n", deterministic ? "yes" : "NO" );

	if ( jsonPath != NULL )
	{
		FILE* f = fopen( jsonPath, "w" );
		if ( f != NULL )
		{
			fprintf( f, "{\n  \"scene\": \"%s\", \"ticks\": %d, \"period\": %d, \"fragmentScale\": %.3f, \"deterministic\": %s,\n  \"runs\": [\n",
					 lpSceneName( scene ), ticks, period, (double)fragmentScale, deterministic ? "true" : "false" );
			for ( int w = 0; w < workerCount; ++w )
			{
				Result r = results[w];
				fprintf( f,
						 "    {\"workers\": %d, \"stepAvgMs\": %.3f, \"stepP95Ms\": %.3f, \"stepMaxMs\": %.3f, \"fractureAvgMs\": %.3f, "
						 "\"fractureMaxMs\": %.3f, \"physicsAvgMs\": %.3f, \"physicsP95Ms\": %.3f, \"maxPieces\": %d, \"maxBodies\": %d, "
						 "\"maxAwakeDebris\": %d, \"maxRubble\": %d, \"impacts\": %d, \"fractures\": %d, \"cells\": %d, "
						 "\"awakeContactsAvg\": %.0f, \"maxContacts\": %d, \"voronoiCpuMs\": %.1f, \"mergeCpuMs\": %.1f, "
						 "\"hullCpuMs\": %.1f, \"stressAvgMs\": %.3f, \"stressMaxMs\": %.2f, \"stressSolves\": %d, \"stressJudged\": %d, "
						 "\"stressReduced\": %d, \"stressAudits\": %d, "
						 "\"stressWaits\": %d, \"loadMs\": %.1f, \"settleMs\": %.1f, \"settleIterations\": %d, \"pieces\": %d, "
						 "\"bonds\": %d, \"over16ms\": %d, \"over33ms\": %d, \"hash\": \"%016llx\", \"solverHash\": \"%016llx\"}%s\n",
						 r.workers, (double)r.total.avg, (double)r.total.p95, (double)r.total.max, (double)r.fracture.avg,
						 (double)r.fracture.max, (double)r.physics.avg, (double)r.physics.p95, r.maxPieces, r.maxBodies,
						 r.maxAwakeDebris, r.maxRubble, r.impacts, r.fractures, r.cells, r.sumAwakeContacts, r.maxContacts,
						 r.sumVoronoiCpu, r.sumMergeCpu, r.sumHullCpu, r.sumStress / (double)( ticks > 0 ? ticks : 1 ), r.maxStress,
						 r.stressSolves, r.stressJudged, r.stressReduced, r.stressAudits, r.stressWaiting, (double)r.loadMs,
						 (double)r.settleMs, r.settleIterations, r.pieces, r.bonds, r.over16, r.over33,
						 (unsigned long long)r.hash, (unsigned long long)r.solverHash, w + 1 < workerCount ? "," : "" );
			}
			fprintf( f, "  ]\n}\n" );
			fclose( f );
		}
	}
	return deterministic ? 0 : 2;
}
