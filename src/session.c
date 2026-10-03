// SPDX-License-Identifier: MIT
// Sessions (lpf.h): what machines that play together must agree on besides their commands, as "key value" lines, and
// the comparison that names the first key two machines differ in. Floats print as %.9g, which reads back to the same
// bits and prints the same characters on every platform we build for (the determinism self-test checks both), in the C
// locale: nothing in the engine or its apps calls setlocale.

#include "world.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef enum lpFieldType
{
	lp_fieldFloat,
	lp_fieldInt,
	lp_fieldBool,
	lp_fieldU64,
	lp_fieldVec3
} lpFieldType;

typedef struct lpSessionField
{
	const char* key;
	size_t offset;
	lpFieldType type;
} lpSessionField;

#define LP_DEF_FIELD( name, type ) { #name, offsetof( lpWorldDef, name ), type }

// lpWorldDef's simulation settings, a line each. Left out: materials and joints (a digest per entry instead), debugLog
// (it only prints) and workerCount (results do not depend on it).
static const lpSessionField lp_defFields[] = {
	LP_DEF_FIELD( gravity, lp_fieldVec3 ),
	LP_DEF_FIELD( seed, lp_fieldU64 ),
	LP_DEF_FIELD( maxFullDebris, lp_fieldInt ),
	LP_DEF_FIELD( maxLightDebris, lp_fieldInt ),
	LP_DEF_FIELD( maxGhosts, lp_fieldInt ),
	LP_DEF_FIELD( maxRubblePieces, lp_fieldInt ),
	LP_DEF_FIELD( maxScrapPieces, lp_fieldInt ),
	LP_DEF_FIELD( fragmentScale, lp_fieldFloat ),
	LP_DEF_FIELD( debrisScale, lp_fieldFloat ),
	LP_DEF_FIELD( freezeRubble, lp_fieldBool ),
	LP_DEF_FIELD( maxFractureJobsPerStep, lp_fieldInt ),
	LP_DEF_FIELD( maxFreezesPerStep, lp_fieldInt ),
	LP_DEF_FIELD( maxGhostCastsPerStep, lp_fieldInt ),
	LP_DEF_FIELD( wakeSpeed, lp_fieldFloat ),
	LP_DEF_FIELD( maxDepth, lp_fieldInt ),
	LP_DEF_FIELD( maxHitImpacts, lp_fieldInt ),
	LP_DEF_FIELD( hitSpeed, lp_fieldFloat ),
	LP_DEF_FIELD( hitEnergy, lp_fieldFloat ),
	LP_DEF_FIELD( hitRadiusScale, lp_fieldFloat ),
	LP_DEF_FIELD( hitRadiusMin, lp_fieldFloat ),
	LP_DEF_FIELD( hitRadiusMax, lp_fieldFloat ),
	LP_DEF_FIELD( pushSpeedCap, lp_fieldFloat ),
	LP_DEF_FIELD( freezeAgeFull, lp_fieldInt ),
	LP_DEF_FIELD( freezeAgeLight, lp_fieldInt ),
	LP_DEF_FIELD( freezeDriftAge, lp_fieldInt ),
	LP_DEF_FIELD( freezeDriftSpeed, lp_fieldFloat ),
	LP_DEF_FIELD( killDepth, lp_fieldFloat ),
	LP_DEF_FIELD( pullStiffness, lp_fieldFloat ),
	LP_DEF_FIELD( pullDamping, lp_fieldFloat ),
	LP_DEF_FIELD( pullSpinKeep, lp_fieldFloat ),
	LP_DEF_FIELD( stressScale, lp_fieldFloat ),
	LP_DEF_FIELD( maxStressWork, lp_fieldInt ),
	LP_DEF_FIELD( maxStressStructureWork, lp_fieldInt ),
	LP_DEF_FIELD( maxStressIterations, lp_fieldInt ),
	LP_DEF_FIELD( stressHopsPerTick, lp_fieldInt ),
	LP_DEF_FIELD( supplyHopsPerTick, lp_fieldInt ),
	LP_DEF_FIELD( maxImpactRadius, lp_fieldFloat ),
	LP_DEF_FIELD( maxRayRange, lp_fieldFloat ),
	LP_DEF_FIELD( maxSettleIterations, lp_fieldInt ),
	LP_DEF_FIELD( stressLargeNodes, lp_fieldInt ),
	LP_DEF_FIELD( stressGlue, lp_fieldFloat ),
	LP_DEF_FIELD( maxStressBreaks, lp_fieldInt ),
	LP_DEF_FIELD( stressPatience, lp_fieldInt ),
	LP_DEF_FIELD( strainRate, lp_fieldFloat ),
	LP_DEF_FIELD( maxLinks, lp_fieldInt ),
	LP_DEF_FIELD( maxWheelCastsPerStep, lp_fieldInt ),
	LP_DEF_FIELD( maxFootCastsPerStep, lp_fieldInt ),
};

// A field added to one of these defs must go into its line or digest below, or be left out with a reason; these sizes
// (on 64-bit targets) fail to compile until it is
_Static_assert( sizeof( void* ) != 8 || sizeof( lpWorldDef ) == 232, "lpWorldDef changed: describe the new field in lp_defFields" );
_Static_assert( sizeof( void* ) != 8 || sizeof( lpMaterialDef ) == 112, "lpMaterialDef changed: digest the new field" );
_Static_assert( sizeof( void* ) != 8 || sizeof( lpJointDef ) == 24, "lpJointDef changed: digest the new field" );
_Static_assert( sizeof( void* ) != 8 || sizeof( lpObjectDef ) == 104, "lpObjectDef changed: digest the new field" );
_Static_assert( sizeof( void* ) != 8 || sizeof( lpPartDef ) == 120, "lpPartDef changed: digest the new field" );

// A material's simulation fields (its name and interior colour are looks)
static uint64_t lpMaterialDigest( const lpMaterialDef* m )
{
	float f[18] = { m->density,		   m->bondStrength,	  m->fractureEnergy, m->fragmentSize,		 m->friction,
					m->restitution,	   m->grainStretch,	  m->particleVolume, m->ghostVolume,		 m->lightVolume,
					m->plateSize,	   m->mergeSlack,	  m->tensileStrength, m->compressiveStrength, m->shearStrength,
					m->courseHeight,   m->brickLength,	  m->crush };
	int32_t i[7] = { m->pattern, m->maxCells, m->particleKind, m->chipSplits, m->breakable ? 1 : 0, m->joint, m->cellJoint };
	uint64_t h = lpHashWords( LP_HASH_INIT, f, sizeof( f ) );
	return lpMix64( lpHashWords( h, i, sizeof( i ) ) );
}

static uint64_t lpJointDigest( const lpJointDef* j )
{
	float f[4] = { j->tensileStrength, j->compressiveStrength, j->shearStrength, j->friction };
	return lpMix64( lpHashWords( LP_HASH_INIT, f, sizeof( f ) ) );
}

static uint64_t lpDetonatorDigest( uint64_t h, const lpDetonatorDef* d )
{
	float f[5] = { d->triggerSpeed, d->radius, d->energy, d->speed, d->delay };
	return lpHashWords( h, f, sizeof( f ) );
}

// A template's parts and settings; its placement and velocities are each spawn's, and part colours are looks
static uint64_t lpTemplateDigest( const lpObjectDef* def )
{
	int32_t head[4] = { def->isStatic ? 1 : 0, def->partCount, (int32_t)def->userId, def->solveStress ? 1 : 0 };
	float scalars[2] = { def->gravityScale, def->inertiaRadius };
	uint64_t h = lpHashWords( LP_HASH_INIT, head, sizeof( head ) );
	h = lpHashWords( h, scalars, sizeof( scalars ) );
	h = lpDetonatorDigest( h, &def->detonator );
	for ( int k = 0; k < def->partCount; ++k )
	{
		const lpPartDef* p = def->parts + k;
		float geometry[13] = { p->halfExtents.x, p->halfExtents.y, p->halfExtents.z, p->transform.p.x, p->transform.p.y,
							   p->transform.p.z, p->transform.q.v.x, p->transform.q.v.y, p->transform.q.v.z, p->transform.q.s,
							   p->grainAxis.x,	 p->grainAxis.y,   p->grainAxis.z };
		const lpPartSystem* s = &p->system;
		int32_t ids[8] = { p->points != NULL ? p->pointCount : 0, p->material, p->anchored ? 1 : 0, p->joint,
						   s->tag,									 s->carries,  s->sources,		   s->needs };
		float system[4] = { s->pool, s->seal, s->leakRate, s->pressure };
		h = lpHashWords( h, geometry, sizeof( geometry ) );
		h = lpHashWords( h, ids, sizeof( ids ) );
		h = lpHashWords( h, system, sizeof( system ) );
		h = lpDetonatorDigest( h, &p->detonator );
		if ( p->points != NULL )
		{
			h = lpHashWords( h, p->points, sizeof( lpVec3 ) * (size_t)p->pointCount ); // floats only: no padding
		}
	}
	return lpMix64( h );
}

typedef struct lpText
{
	char* buffer;
	int size;
	int length; // needed, which may be more than fits
} lpText;

static void lpAppend( lpText* t, const char* line )
{
	int n = (int)strlen( line );
	if ( t->length < t->size - 1 )
	{
		int fit = lpMinInt( n, t->size - 1 - t->length );
		memcpy( t->buffer + t->length, line, (size_t)fit );
		t->buffer[t->length + fit] = 0;
	}
	t->length += n;
}

int lpWorld_DescribeSession( const lpWorld* w, char* buffer, int size )
{
	lpText t = { buffer, size, 0 };
	if ( size > 0 )
	{
		buffer[0] = 0;
	}
	char line[160];
	const char* def = (const char*)&w->def;
	for ( size_t i = 0; i < sizeof( lp_defFields ) / sizeof( lp_defFields[0] ); ++i )
	{
		const lpSessionField* f = lp_defFields + i;
		const char* at = def + f->offset;
		if ( f->type == lp_fieldFloat )
		{
			float v;
			memcpy( &v, at, sizeof( v ) );
			snprintf( line, sizeof( line ), "%s %.9g\n", f->key, (double)v );
		}
		else if ( f->type == lp_fieldInt )
		{
			int v;
			memcpy( &v, at, sizeof( v ) );
			snprintf( line, sizeof( line ), "%s %d\n", f->key, v );
		}
		else if ( f->type == lp_fieldBool )
		{
			bool v;
			memcpy( &v, at, sizeof( v ) );
			snprintf( line, sizeof( line ), "%s %d\n", f->key, v ? 1 : 0 );
		}
		else if ( f->type == lp_fieldU64 )
		{
			uint64_t v;
			memcpy( &v, at, sizeof( v ) );
			snprintf( line, sizeof( line ), "%s %llu\n", f->key, (unsigned long long)v );
		}
		else
		{
			lpVec3 v;
			memcpy( &v, at, sizeof( v ) );
			snprintf( line, sizeof( line ), "%s %.9g %.9g %.9g\n", f->key, (double)v.x, (double)v.y, (double)v.z );
		}
		lpAppend( &t, line );
	}
	for ( int i = 0; i < lp_materialCount; ++i )
	{
		snprintf( line, sizeof( line ), "material.%d %016llx\n", i, (unsigned long long)lpMaterialDigest( w->materials + i ) );
		lpAppend( &t, line );
	}
	for ( int i = 0; i < lp_jointCount; ++i )
	{
		snprintf( line, sizeof( line ), "joint.%d %016llx\n", i, (unsigned long long)lpJointDigest( w->joints + i ) );
		lpAppend( &t, line );
	}
	for ( int i = 0; i < w->templates.count; ++i )
	{
		snprintf( line, sizeof( line ), "template.%d %016llx\n", i, (unsigned long long)lpTemplateDigest( &w->templates.data[i].def ) );
		lpAppend( &t, line );
	}
	snprintf( line, sizeof( line ), "selftest %016llx\n", (unsigned long long)lpDeterminismSelfTest( NULL ) );
	lpAppend( &t, line );
	snprintf( line, sizeof( line ), "tick %llu\n", (unsigned long long)w->tick );
	lpAppend( &t, line );
	snprintf( line, sizeof( line ), "hash %016llx\n", (unsigned long long)lpWorld_Hash( w ) );
	lpAppend( &t, line );
	return t.length;
}

// One "key value" line: its key's length, and its value (without a trailing \r); returns where the next line starts
static const char* lpSplitLine( const char* line, int* keyLength, const char** value, int* valueLength )
{
	const char* end = strchr( line, '\n' );
	end = end != NULL ? end : line + strlen( line );
	const char* stop = end > line && end[-1] == '\r' ? end - 1 : end;
	const char* space = memchr( line, ' ', (size_t)( stop - line ) );
	*keyLength = (int)( ( space != NULL ? space : stop ) - line );
	*value = space != NULL ? space + 1 : stop;
	*valueLength = (int)( stop - *value );
	return *end != 0 ? end + 1 : end;
}

// The value of the first line of text with this key: true if there is one
static bool lpFindValue( const char* text, const char* key, int keyLength, const char** value, int* valueLength )
{
	for ( const char* line = text; *line != 0; )
	{
		int length;
		const char* next = lpSplitLine( line, &length, value, valueLength );
		if ( length == keyLength && memcmp( line, key, (size_t)keyLength ) == 0 )
		{
			return true;
		}
		line = next;
	}
	return false;
}

// Every key of a is in b, with the same value when values is set; else false, with the first key that is not
static bool lpCoveredBy( const char* a, const char* b, bool values, char* key, int keySize )
{
	for ( const char* line = a; *line != 0; )
	{
		int length, mineLength, theirsLength;
		const char *mine, *theirs;
		const char* next = lpSplitLine( line, &length, &mine, &mineLength );
		bool same = length == 0; // blank lines match
		if ( same == false && lpFindValue( b, line, length, &theirs, &theirsLength ) )
		{
			same = values == false || ( theirsLength == mineLength && memcmp( theirs, mine, (size_t)mineLength ) == 0 );
		}
		if ( same == false )
		{
			if ( keySize > 0 )
			{
				int fit = lpMinInt( length, keySize - 1 );
				memcpy( key, line, (size_t)fit );
				key[fit] = 0;
			}
			return false;
		}
		line = next;
	}
	return true;
}

bool lpSessionCompare( const char* a, const char* b, char* key, int keySize )
{
	if ( keySize > 0 )
	{
		key[0] = 0;
	}
	return lpCoveredBy( a, b, true, key, keySize ) && lpCoveredBy( b, a, false, key, keySize );
}
