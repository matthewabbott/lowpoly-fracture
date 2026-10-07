// SPDX-License-Identifier: MIT

#include "fcheck.h"

#include "world.h"

#include <float.h>
#include <math.h>

// ---- snapshots ----

typedef struct lpWriter
{
	uint8_t data[LP_JOB_SNAPSHOT_MAX];
	int size;
} lpWriter;

static void lpPutU8( lpWriter* w, uint32_t v )
{
	if ( w->size < LP_JOB_SNAPSHOT_MAX )
	{
		w->data[w->size] = (uint8_t)v;
	}
	w->size += 1;
}

static void lpPutU16( lpWriter* w, uint32_t v )
{
	lpPutU8( w, v & 0xFFu );
	lpPutU8( w, ( v >> 8 ) & 0xFFu );
}

static void lpPutU32( lpWriter* w, uint32_t v )
{
	for ( int k = 0; k < 4; ++k )
	{
		lpPutU8( w, ( v >> ( 8 * k ) ) & 0xFFu );
	}
}

static void lpPutU64( lpWriter* w, uint64_t v )
{
	lpPutU32( w, (uint32_t)( v & 0xFFFFFFFFu ) );
	lpPutU32( w, (uint32_t)( v >> 32 ) );
}

static uint32_t lpBitsOf( float x )
{
	uint32_t u;
	memcpy( &u, &x, 4 );
	return u;
}

static void lpPutF32( lpWriter* w, float x )
{
	lpPutU32( w, lpBitsOf( x ) );
}

static void lpPutVec3( lpWriter* w, lpVec3 v )
{
	lpPutF32( w, v.x );
	lpPutF32( w, v.y );
	lpPutF32( w, v.z );
}

static uint64_t lpChecksum( const uint8_t* data, int size )
{
	return lpMix64( lpHashBytes( LP_HASH_INIT, data, (size_t)size ) );
}

int lpFractureJob_Write( const lpFractureJob* job, uint8_t* bytes, int capacity )
{
	lpWriter* w = lpAlloc( sizeof( lpWriter ) );
	w->size = 0;
	lpPutU8( w, 'L' );
	lpPutU8( w, 'P' );
	lpPutU8( w, 'F' );
	lpPutU8( w, 'J' );
	lpPutU32( w, LP_JOB_SNAPSHOT_VERSION );
	lpPutU32( w, (uint32_t)job->piece );
	lpPutVec3( w, job->localImpact );
	lpPutVec3( w, job->center );

	lpPutVec3( w, job->impact.point );
	lpPutVec3( w, job->impact.direction );
	lpPutF32( w, job->impact.radius );
	lpPutF32( w, job->impact.energy );
	lpPutF32( w, job->impact.impulse );
	lpPutU8( w, job->impact.explosion ? 1u : 0u );

	lpPutF32( w, job->particleVolume );
	lpPutF32( w, job->ghostVolume );
	lpPutF32( w, job->lightVolume );
	lpPutF32( w, job->mergeSlack );
	lpPutU32( w, (uint32_t)job->chipSplits );

	const lpFractureInput* in = &job->input;
	lpPutVec3( w, in->impact );
	lpPutF32( w, in->radius );
	lpPutF32( w, in->fragmentSize );
	lpPutF32( w, in->plateSize );
	lpPutU32( w, (uint32_t)in->maxCells );
	lpPutF32( w, in->absorbVolume );
	lpPutU32( w, (uint32_t)in->pattern );
	lpPutVec3( w, in->axis );
	lpPutF32( w, in->stretch );
	lpPutU8( w, in->interiorMaterial );
	lpPutU64( w, in->seed );
	lpPutF32( w, in->tolerance );
	lpPutU8( w, in->snap ? 1u : 0u );
	lpPutF32( w, in->courseHeight );
	lpPutF32( w, in->brickLength );
	lpPutVec3( w, in->gridOrigin );

	const lpPoly* poly = &job->poly;
	lpPutU32( w, (uint32_t)poly->vertexCount );
	lpPutU32( w, (uint32_t)poly->faceCount );
	lpPutU32( w, (uint32_t)poly->indexCount );
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		lpPutVec3( w, poly->vertices[i] );
	}
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpFace* face = poly->faces + f;
		lpPutVec3( w, face->plane.normal );
		lpPutF32( w, face->plane.offset );
		lpPutU16( w, face->first );
		lpPutU8( w, face->count );
		lpPutU8( w, face->material );
		lpPutU32( w, (uint32_t)face->tag );
	}
	for ( int i = 0; i < poly->indexCount; ++i )
	{
		lpPutU8( w, poly->indices[i] );
	}
	LP_ASSERT( w->size + 8 <= LP_JOB_SNAPSHOT_MAX );
	lpPutU64( w, lpChecksum( w->data, w->size ) );

	int size = w->size;
	if ( bytes != NULL && size <= capacity )
	{
		memcpy( bytes, w->data, (size_t)size );
	}
	lpFree( w );
	return size;
}

typedef struct lpReader
{
	const uint8_t* data;
	int size;
	int at;
	bool bad;
} lpReader;

static uint32_t lpGetU8( lpReader* r )
{
	if ( r->at >= r->size )
	{
		r->bad = true;
		return 0;
	}
	return r->data[r->at++];
}

static uint32_t lpGetU16( lpReader* r )
{
	uint32_t lo = lpGetU8( r );
	return lo | ( lpGetU8( r ) << 8 );
}

static uint32_t lpGetU32( lpReader* r )
{
	uint32_t v = 0;
	for ( int k = 0; k < 4; ++k )
	{
		v |= lpGetU8( r ) << ( 8 * k );
	}
	return v;
}

static uint64_t lpGetU64( lpReader* r )
{
	uint64_t lo = lpGetU32( r );
	return lo | ( (uint64_t)lpGetU32( r ) << 32 );
}

// A finite float within +-limit, or the reader goes bad
static float lpGetF32( lpReader* r, float limit )
{
	uint32_t bits = lpGetU32( r );
	float x;
	memcpy( &x, &bits, 4 );
	if ( ( bits & 0x7F800000u ) == 0x7F800000u || ( fabsf( x ) <= limit ) == false )
	{
		r->bad = true;
		return 0.0f;
	}
	return x;
}

static lpVec3 lpGetVec3( lpReader* r, float limit )
{
	lpVec3 v;
	v.x = lpGetF32( r, limit );
	v.y = lpGetF32( r, limit );
	v.z = lpGetF32( r, limit );
	return v;
}

// An int in [lo, hi], or the reader goes bad
static int lpGetInt( lpReader* r, int lo, int hi )
{
	int32_t v = (int32_t)lpGetU32( r );
	if ( v < lo || v > hi )
	{
		r->bad = true;
		return lo;
	}
	return v;
}

static void lpCheckRange( lpReader* r, float x, float lo, float hi )
{
	r->bad = r->bad || ( x >= lo && x <= hi ) == false;
}

bool lpFractureJob_Read( lpFractureJob* job, const uint8_t* bytes, int size )
{
	lpCellBond* bonds = job->bonds;
	memset( job, 0, sizeof( *job ) );
	job->bonds = bonds;
	job->input.parent = &job->poly;
	if ( bytes == NULL || size < 16 || size > LP_JOB_SNAPSHOT_MAX )
	{
		return false;
	}
	uint64_t stored = 0;
	for ( int k = 0; k < 8; ++k )
	{
		stored |= (uint64_t)bytes[size - 8 + k] << ( 8 * k );
	}
	if ( stored != lpChecksum( bytes, size - 8 ) )
	{
		return false;
	}

	lpReader reader = { bytes, size - 8, 0, false };
	lpReader* r = &reader;
	const float far = 1e4f; // m: coordinates and lengths
	uint32_t magic = lpGetU32( r );
	uint32_t version = lpGetU32( r );
	if ( magic != ( 'L' | ( 'P' << 8 ) | ( 'F' << 16 ) | ( (uint32_t)'J' << 24 ) ) || version != LP_JOB_SNAPSHOT_VERSION )
	{
		return false;
	}
	job->piece = lpGetInt( r, -1, INT32_MAX );
	job->localImpact = lpGetVec3( r, far );
	job->center = lpGetVec3( r, far );

	job->impact.point = lpGetVec3( r, 1e7f );
	job->impact.direction = lpGetVec3( r, 1e3f );
	job->impact.radius = lpGetF32( r, 1e7f );
	job->impact.energy = lpGetF32( r, 1e30f );
	job->impact.impulse = lpGetF32( r, 1e30f );
	uint32_t explosion = lpGetU8( r );
	r->bad = r->bad || explosion > 1u;
	job->impact.explosion = explosion == 1u;

	job->particleVolume = lpGetF32( r, 1e9f );
	job->ghostVolume = lpGetF32( r, 1e9f );
	job->lightVolume = lpGetF32( r, 1e9f );
	job->mergeSlack = lpGetF32( r, 100.0f );
	job->chipSplits = lpGetInt( r, 0, 64 );

	lpFractureInput* in = &job->input;
	in->impact = lpGetVec3( r, far );
	in->radius = lpGetF32( r, far );
	in->fragmentSize = lpGetF32( r, far );
	in->plateSize = lpGetF32( r, far );
	in->maxCells = lpGetInt( r, 0, 1 << 20 );
	in->absorbVolume = lpGetF32( r, 1e9f );
	in->pattern = (lpPatternId)lpGetInt( r, lp_breakImpact, lp_breakMasonry );
	in->axis = lpGetVec3( r, 2.0f );
	in->stretch = lpGetF32( r, 1e3f );
	in->interiorMaterial = (uint8_t)lpGetU8( r );
	in->seed = lpGetU64( r );
	in->tolerance = lpGetF32( r, 1.0f );
	uint32_t snap = lpGetU8( r );
	r->bad = r->bad || snap > 1u;
	in->snap = snap == 1u;
	in->courseHeight = lpGetF32( r, far );
	in->brickLength = lpGetF32( r, far );
	in->gridOrigin = lpGetVec3( r, far );
	// What the patterns divide by or count with must be sane: a masonry grid finer than a millimetre would overflow the
	// course arithmetic, a fragment size of zero would ask for unbounded sites
	lpCheckRange( r, in->fragmentSize, 1e-4f, far );
	lpCheckRange( r, in->radius, 0.0f, far );
	lpCheckRange( r, in->tolerance, 1e-9f, 1.0f );
	r->bad = r->bad || ( in->courseHeight != 0.0f && ( in->courseHeight >= 1e-3f ) == false );
	r->bad = r->bad || ( in->brickLength != 0.0f && ( in->brickLength >= 1e-3f ) == false );
	r->bad = r->bad || in->interiorMaterial >= lp_materialCount;

	lpPoly* poly = &job->poly;
	poly->vertexCount = lpGetInt( r, 4, LP_POLY_MAX_VERTICES );
	poly->faceCount = lpGetInt( r, 4, LP_POLY_MAX_FACES );
	poly->indexCount = lpGetInt( r, 12, LP_POLY_MAX_INDICES );
	if ( r->bad )
	{
		memset( poly, 0, sizeof( *poly ) );
		return false;
	}
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		poly->vertices[i] = lpGetVec3( r, far );
	}
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		lpFace* face = poly->faces + f;
		face->plane.normal = lpGetVec3( r, 2.0f );
		face->plane.offset = lpGetF32( r, 2.0f * far );
		face->first = (uint16_t)lpGetU16( r );
		face->count = (uint8_t)lpGetU8( r );
		face->material = (uint8_t)lpGetU8( r );
		face->tag = (int32_t)lpGetU32( r );
		r->bad = r->bad || face->count < 3 || (int)face->first + (int)face->count > poly->indexCount;
	}
	for ( int i = 0; i < poly->indexCount; ++i )
	{
		poly->indices[i] = (uint8_t)lpGetU8( r );
		r->bad = r->bad || poly->indices[i] >= poly->vertexCount;
	}
	if ( r->bad || r->at != r->size )
	{
		memset( job, 0, sizeof( *job ) );
		job->bonds = bonds;
		job->input.parent = &job->poly;
		return false;
	}
	return true;
}

lpFractureJob* lpFractureJob_Create( void )
{
	lpFractureJob* job = lpAlloc( sizeof( lpFractureJob ) );
	memset( job, 0, sizeof( *job ) );
	job->bonds = lpAlloc( sizeof( lpCellBond ) * LP_MAX_CELL_BONDS );
	job->input.parent = &job->poly;
	return job;
}

void lpFractureJob_Destroy( lpFractureJob* job )
{
	if ( job == NULL )
	{
		return;
	}
	lpFracture_FreeJob( job );
	lpFree( job->bonds );
	lpFree( job );
}

// ---- digests ----

static uint64_t lpDigestU32( uint64_t h, uint32_t v )
{
	return lpHashWords( h, &v, sizeof( v ) );
}

static uint64_t lpDigestFloat( uint64_t h, float x )
{
	return lpDigestU32( h, lpBitsOf( x ) );
}

static uint64_t lpDigestVec3( uint64_t h, lpVec3 v )
{
	h = lpDigestFloat( h, v.x );
	h = lpDigestFloat( h, v.y );
	return lpDigestFloat( h, v.z );
}

static uint64_t lpDigestShape( uint64_t h, const lpShape* s )
{
	h = lpDigestU32( h, (uint32_t)s->vertexCount );
	h = lpDigestU32( h, (uint32_t)s->faceCount );
	h = lpDigestU32( h, (uint32_t)s->indexCount );
	for ( int i = 0; i < s->vertexCount; ++i )
	{
		h = lpDigestVec3( h, s->vertices[i] );
	}
	for ( int f = 0; f < s->faceCount; ++f )
	{
		const lpFace* face = s->faces + f;
		h = lpDigestVec3( h, face->plane.normal );
		h = lpDigestFloat( h, face->plane.offset );
		h = lpDigestU32( h, face->first );
		h = lpDigestU32( h, face->count );
		h = lpDigestU32( h, face->material );
		h = lpDigestU32( h, (uint32_t)face->tag );
	}
	h = lpHashWords( h, s->indices, (size_t)s->indexCount );
	h = lpDigestFloat( h, s->volume );
	h = lpDigestVec3( h, s->centroid );
	h = lpDigestFloat( h, s->radius );
	h = lpDigestVec3( h, s->bounds.lowerBound );
	return lpDigestVec3( h, s->bounds.upperBound );
}

uint64_t lpFractureJob_Digest( const lpFractureJob* job )
{
	uint64_t h = LP_HASH_INIT;
	h = lpDigestU32( h, (uint32_t)job->cellCount );
	h = lpDigestU32( h, (uint32_t)job->bondCount );
	bool classified = job->cellCount >= 2; // below that the job stopped after the pattern: classes and hulls are unset
	for ( int i = 0; i < job->cellCount; ++i )
	{
		h = lpDigestShape( h, job->cells[i] );
		h = lpDigestU32( h, (uint32_t)job->cellSites[i] );
		if ( classified == false )
		{
			continue;
		}
		h = lpDigestU32( h, job->cellClass[i] );
		const lpPhysHull* hull = job->hulls[i];
		if ( hull == NULL )
		{
			h = lpDigestU32( h, 0xFFFFFFFFu );
			continue;
		}
		int vertices = lpPhys_GetHullVertexCount( hull );
		int faces = lpPhys_GetHullFaceCount( hull );
		h = lpDigestU32( h, (uint32_t)vertices );
		h = lpDigestU32( h, (uint32_t)faces );
		for ( int k = 0; k < vertices; ++k )
		{
			h = lpDigestVec3( h, lpPhys_GetHullPoint( hull, k ) );
		}
		for ( int k = 0; k < faces; ++k )
		{
			lpPlane plane = lpPhys_GetHullPlane( hull, k );
			h = lpDigestVec3( h, plane.normal );
			h = lpDigestFloat( h, plane.offset );
		}
	}
	for ( int i = 0; i < job->bondCount; ++i )
	{
		const lpCellBond* b = job->bonds + i;
		h = lpDigestU32( h, (uint32_t)b->a );
		h = lpDigestU32( h, (uint32_t)b->b );
		h = lpDigestFloat( h, b->contact.area );
		h = lpDigestVec3( h, b->contact.centroid );
		h = lpDigestVec3( h, b->contact.normal );
		h = lpDigestFloat( h, b->contact.h1 );
		h = lpDigestFloat( h, b->contact.h2 );
	}
	return lpMix64( h );
}

static void lpAddClipStats( lpClipStats* total, const lpClipStats* one )
{
	total->clips += one->clips;
	total->shifts += one->shifts;
	total->maxShift = one->maxShift > total->maxShift ? one->maxShift : total->maxShift;
	total->toleranceOuts += one->toleranceOuts;
	total->failures += one->failures;
}

void lpFractureStats_Add( lpFractureStats* total, const lpFractureStats* one )
{
	total->failureCount += one->failureCount;
	for ( int k = 0; k < lp_clipCallerCount; ++k )
	{
		lpAddClipStats( total->clips + k, one->clips + k );
	}
	total->sitesDrawn += one->sitesDrawn;
	total->sliversAbsorbed += one->sliversAbsorbed;
	total->cellsDropped += one->cellsDropped;
	total->patternCells += one->patternCells;
	total->mergeTried += one->mergeTried;
	total->mergeTooBig += one->mergeTooBig;
	total->mergePrerejected += one->mergePrerejected;
	total->mergeHulls += one->mergeHulls;
	total->mergeAccepted += one->mergeAccepted;
	total->mergeRetagged += one->mergeRetagged;
	total->mergeBridges += one->mergeBridges;
	total->bondsByTag += one->bondsByTag;
	total->bondsByContact += one->bondsByContact;
	total->hullsBuilt += one->hullsBuilt;
	total->hullFailsLarge += one->hullFailsLarge;
	total->hullFailsOther += one->hullFailsOther;
	total->ghostsChipped += one->ghostsChipped;
	total->chipsMade += one->chipsMade;
	total->outputCells += one->outputCells;
	total->maxFaces = one->maxFaces > total->maxFaces ? one->maxFaces : total->maxFaces;
	total->maxVertices = one->maxVertices > total->maxVertices ? one->maxVertices : total->maxVertices;
	for ( int k = 0; k < LP_CELL_BINS; ++k )
	{
		total->faceBins[k] += one->faceBins[k];
		total->vertexBins[k] += one->vertexBins[k];
	}
	total->voronoiMs += one->voronoiMs;
	total->mergeMs += one->mergeMs;
	total->hullMs += one->hullMs;
	total->bondMs += one->bondMs;
	total->chipMs += one->chipMs;
}

// ---- geometry in double ----

typedef struct lpD3
{
	double x, y, z;
} lpD3;

static lpD3 lpD( lpVec3 v )
{
	return (lpD3){ v.x, v.y, v.z };
}

static lpD3 lpDSub( lpD3 a, lpD3 b )
{
	return (lpD3){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static lpD3 lpDAdd( lpD3 a, lpD3 b )
{
	return (lpD3){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static lpD3 lpDScale( double s, lpD3 a )
{
	return (lpD3){ s * a.x, s * a.y, s * a.z };
}

static double lpDDot( lpD3 a, lpD3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static lpD3 lpDCross( lpD3 a, lpD3 b )
{
	return (lpD3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static double lpDLength( lpD3 a )
{
	return sqrt( lpDDot( a, a ) );
}

// Six times the signed volume of a polyhedron given by its loops, against the reference point r
static double lpVolume6( const lpVec3* vertices, const lpFace* faces, int faceCount, const uint8_t* indices, lpD3 r )
{
	double v6 = 0.0;
	for ( int f = 0; f < faceCount; ++f )
	{
		const lpFace* face = faces + f;
		lpD3 a = lpDSub( lpD( vertices[indices[face->first]] ), r );
		for ( int k = 1; k + 1 < face->count; ++k )
		{
			lpD3 b = lpDSub( lpD( vertices[indices[face->first + k]] ), r );
			lpD3 c = lpDSub( lpD( vertices[indices[face->first + k + 1]] ), r );
			v6 += lpDDot( a, lpDCross( b, c ) );
		}
	}
	return v6;
}

static double lpShapeVolume( const lpShape* s, lpD3 r )
{
	return lpVolume6( s->vertices, s->faces, s->faceCount, s->indices, r ) / 6.0;
}

static double lpPolyVolume( const lpPoly* p, lpD3 r )
{
	return lpVolume6( p->vertices, p->faces, p->faceCount, p->indices, r ) / 6.0;
}

static lpD3 lpFaceNormal( const lpFace* face )
{
	return lpD( face->plane.normal );
}

// A face's area vector by Newell's method (its length is the area), in double
static lpD3 lpFaceAreaVector( const lpVec3* vertices, const uint8_t* indices, const lpFace* face )
{
	lpD3 n = { 0.0, 0.0, 0.0 };
	lpD3 a = lpD( vertices[indices[face->first]] );
	for ( int k = 1; k + 1 < face->count; ++k )
	{
		lpD3 b = lpD( vertices[indices[face->first + k]] );
		lpD3 c = lpD( vertices[indices[face->first + k + 1]] );
		n = lpDAdd( n, lpDCross( lpDSub( b, a ), lpDSub( c, a ) ) );
	}
	return lpDScale( 0.5, n );
}

// ---- a convex polyhedron in double, clipped without tolerance ----

enum
{
	lp_dMaxVertices = 512,
	lp_dMaxFaces = 192,
	lp_dMaxIndices = 2048,
	lp_dMaxCrossings = 256,
};

typedef struct lpDPoly
{
	int vertexCount;
	int faceCount;
	int indexCount;
	lpD3 v[lp_dMaxVertices];
	int first[lp_dMaxFaces];
	int count[lp_dMaxFaces];
	int indices[lp_dMaxIndices];
} lpDPoly;

static void lpDPoly_FromShape( lpDPoly* p, const lpShape* s )
{
	p->vertexCount = s->vertexCount;
	for ( int i = 0; i < s->vertexCount; ++i )
	{
		p->v[i] = lpD( s->vertices[i] );
	}
	p->faceCount = s->faceCount;
	p->indexCount = 0;
	for ( int f = 0; f < s->faceCount; ++f )
	{
		p->first[f] = p->indexCount;
		p->count[f] = s->faces[f].count;
		for ( int k = 0; k < s->faces[f].count; ++k )
		{
			p->indices[p->indexCount++] = s->indices[s->faces[f].first + k];
		}
	}
}

// Monotone in the angle of (x, y) over [0, 4): no trigonometry
static double lpPseudoAngle( double x, double y )
{
	if ( x == 0.0 && y == 0.0 )
	{
		return 0.0;
	}
	if ( y >= 0.0 )
	{
		return x >= 0.0 ? y / ( x + y ) : 1.0 - x / ( y - x );
	}
	return x < 0.0 ? 2.0 - y / ( -x - y ) : 3.0 + x / ( x - y );
}

typedef struct lpCapKey
{
	double angle;
	double distance;
	int vertex;
} lpCapKey;

static bool lpCapBefore( const lpCapKey* a, const lpCapKey* b )
{
	if ( a->angle != b->angle )
	{
		return a->angle < b->angle;
	}
	if ( a->distance != b->distance )
	{
		return a->distance < b->distance;
	}
	return a->vertex < b->vertex;
}

// Keeps n.x <= d. Returns 0 when nothing is cut away (out untouched), 1 when cut into out, 2 when nothing is left,
// -1 when out ran out of room. A vertex exactly on the plane stays; only strict crossings make new vertices.
static int lpDPoly_Clip( const lpDPoly* in, lpD3 n, double d, lpDPoly* out )
{
	double s[lp_dMaxVertices];
	double lo = DBL_MAX, hi = -DBL_MAX;
	for ( int i = 0; i < in->vertexCount; ++i )
	{
		s[i] = lpDDot( n, in->v[i] ) - d;
		lo = s[i] < lo ? s[i] : lo;
		hi = s[i] > hi ? s[i] : hi;
	}
	if ( hi <= 0.0 )
	{
		return 0;
	}
	if ( lo >= 0.0 )
	{
		return 2;
	}

	int map[lp_dMaxVertices];
	out->vertexCount = 0;
	for ( int i = 0; i < in->vertexCount; ++i )
	{
		map[i] = -1;
		if ( s[i] <= 0.0 )
		{
			map[i] = out->vertexCount;
			out->v[out->vertexCount++] = in->v[i];
		}
	}

	int crossA[lp_dMaxCrossings], crossB[lp_dMaxCrossings], crossV[lp_dMaxCrossings];
	int crossCount = 0;
	out->faceCount = 0;
	out->indexCount = 0;
	for ( int f = 0; f < in->faceCount; ++f )
	{
		int start = out->indexCount;
		int count = in->count[f];
		for ( int k = 0; k < count; ++k )
		{
			int cur = in->indices[in->first[f] + k];
			int next = in->indices[in->first[f] + ( k + 1 ) % count];
			if ( s[cur] <= 0.0 )
			{
				if ( out->indexCount >= lp_dMaxIndices )
				{
					return -1;
				}
				out->indices[out->indexCount++] = map[cur];
			}
			if ( ( s[cur] < 0.0 && s[next] > 0.0 ) || ( s[cur] > 0.0 && s[next] < 0.0 ) )
			{
				int a = cur < next ? cur : next;
				int b = cur < next ? next : cur;
				int v = -1;
				for ( int c = 0; c < crossCount && v < 0; ++c )
				{
					v = crossA[c] == a && crossB[c] == b ? crossV[c] : -1;
				}
				if ( v < 0 )
				{
					if ( crossCount >= lp_dMaxCrossings || out->vertexCount >= lp_dMaxVertices )
					{
						return -1;
					}
					double t = s[a] / ( s[a] - s[b] );
					v = out->vertexCount++;
					out->v[v] = lpDAdd( in->v[a], lpDScale( t, lpDSub( in->v[b], in->v[a] ) ) );
					crossA[crossCount] = a;
					crossB[crossCount] = b;
					crossV[crossCount] = v;
					crossCount += 1;
				}
				if ( out->indexCount >= lp_dMaxIndices )
				{
					return -1;
				}
				out->indices[out->indexCount++] = v;
			}
		}
		if ( out->indexCount - start < 3 )
		{
			out->indexCount = start; // a face that only touches the plane from outside
			continue;
		}
		if ( out->faceCount >= lp_dMaxFaces )
		{
			return -1;
		}
		out->first[out->faceCount] = start;
		out->count[out->faceCount] = out->indexCount - start;
		out->faceCount += 1;
	}

	// The cap: the crossings and the vertices on the plane, in order around their mean (counter-clockwise about n)
	lpCapKey keys[lp_dMaxVertices];
	int capCount = 0;
	for ( int c = 0; c < crossCount; ++c )
	{
		keys[capCount++].vertex = crossV[c];
	}
	for ( int i = 0; i < in->vertexCount; ++i )
	{
		if ( s[i] == 0.0 )
		{
			keys[capCount++].vertex = map[i];
		}
	}
	if ( capCount < 3 )
	{
		return 1;
	}
	lpD3 m = { 0.0, 0.0, 0.0 };
	for ( int k = 0; k < capCount; ++k )
	{
		m = lpDAdd( m, out->v[keys[k].vertex] );
	}
	m = lpDScale( 1.0 / (double)capCount, m );
	lpD3 t = fabs( n.x ) < 0.57 ? (lpD3){ 1.0, 0.0, 0.0 } : (lpD3){ 0.0, 1.0, 0.0 };
	lpD3 u = lpDCross( t, n );
	u = lpDScale( 1.0 / lpDLength( u ), u );
	lpD3 w = lpDCross( n, u );
	for ( int k = 0; k < capCount; ++k )
	{
		lpD3 p = lpDSub( out->v[keys[k].vertex], m );
		keys[k].angle = lpPseudoAngle( lpDDot( p, u ), lpDDot( p, w ) );
		keys[k].distance = lpDDot( p, p );
	}
	for ( int i = 1; i < capCount; ++i )
	{
		lpCapKey key = keys[i];
		int j = i - 1;
		while ( j >= 0 && lpCapBefore( &key, keys + j ) )
		{
			keys[j + 1] = keys[j];
			j -= 1;
		}
		keys[j + 1] = key;
	}
	if ( out->faceCount >= lp_dMaxFaces || out->indexCount + capCount > lp_dMaxIndices )
	{
		return -1;
	}
	out->first[out->faceCount] = out->indexCount;
	out->count[out->faceCount] = capCount;
	out->faceCount += 1;
	for ( int k = 0; k < capCount; ++k )
	{
		out->indices[out->indexCount++] = keys[k].vertex;
	}
	return 1;
}

static double lpDPoly_Volume( const lpDPoly* p )
{
	if ( p->vertexCount == 0 )
	{
		return 0.0;
	}
	lpD3 r = p->v[0];
	double v6 = 0.0;
	for ( int f = 0; f < p->faceCount; ++f )
	{
		lpD3 a = lpDSub( p->v[p->indices[p->first[f]]], r );
		for ( int k = 1; k + 1 < p->count[f]; ++k )
		{
			lpD3 b = lpDSub( p->v[p->indices[p->first[f] + k]], r );
			lpD3 c = lpDSub( p->v[p->indices[p->first[f] + k + 1]], r );
			v6 += lpDDot( a, lpDCross( b, c ) );
		}
	}
	return v6 / 6.0;
}

// True if one face plane of b has every vertex of a on it or outside: then they share no volume
static bool lpSeparatedBy( const lpShape* a, const lpShape* b )
{
	for ( int f = 0; f < b->faceCount; ++f )
	{
		lpD3 n = lpFaceNormal( b->faces + f );
		double d = b->faces[f].plane.offset;
		bool outside = true;
		for ( int i = 0; i < a->vertexCount && outside; ++i )
		{
			outside = lpDDot( n, lpD( a->vertices[i] ) ) - d >= 0.0;
		}
		if ( outside )
		{
			return true;
		}
	}
	return false;
}

// Volume of a clipped by b's face planes (b as the intersection of its half-spaces); -1 when the clip ran out of room
static double lpOverlapVolume( const lpShape* a, const lpShape* b, lpDPoly* scratch )
{
	if ( lpSeparatedBy( a, b ) || lpSeparatedBy( b, a ) )
	{
		return 0.0;
	}
	lpDPoly* cur = scratch;
	lpDPoly* next = scratch + 1;
	lpDPoly_FromShape( cur, a );
	for ( int f = 0; f < b->faceCount; ++f )
	{
		int result = lpDPoly_Clip( cur, lpFaceNormal( b->faces + f ), b->faces[f].plane.offset, next );
		if ( result == 2 )
		{
			return 0.0;
		}
		if ( result < 0 )
		{
			return -1.0;
		}
		if ( result == 1 )
		{
			lpDPoly* t = cur;
			cur = next;
			next = t;
		}
	}
	return lpDPoly_Volume( cur );
}

// ---- 2D, in a face's plane ----

enum
{
	lp_maxLoop2 = 600
};

typedef struct lpLoop2
{
	int count;
	double x[lp_maxLoop2];
	double y[lp_maxLoop2];
} lpLoop2;

static double lpLoop2Area( const lpLoop2* p )
{
	double a2 = 0.0;
	for ( int i = 0; i < p->count; ++i )
	{
		int j = ( i + 1 ) % p->count;
		a2 += p->x[i] * p->y[j] - p->x[j] * p->y[i];
	}
	return 0.5 * a2;
}

// The part of in on the left of a -> b
static void lpLoop2Clip( const lpLoop2* in, double ax, double ay, double bx, double by, lpLoop2* out )
{
	out->count = 0;
	for ( int i = 0; i < in->count; ++i )
	{
		int j = ( i + 1 ) % in->count;
		double sp = ( bx - ax ) * ( in->y[i] - ay ) - ( by - ay ) * ( in->x[i] - ax );
		double sq = ( bx - ax ) * ( in->y[j] - ay ) - ( by - ay ) * ( in->x[j] - ax );
		if ( sp >= 0.0 && out->count < lp_maxLoop2 )
		{
			out->x[out->count] = in->x[i];
			out->y[out->count++] = in->y[i];
		}
		if ( ( sp >= 0.0 ) != ( sq >= 0.0 ) && out->count < lp_maxLoop2 )
		{
			double t = sp / ( sp - sq );
			out->x[out->count] = in->x[i] + t * ( in->x[j] - in->x[i] );
			out->y[out->count++] = in->y[i] + t * ( in->y[j] - in->y[i] );
		}
	}
}

// Area of the intersection of two convex loops, a counter-clockwise; b either way. scratch holds two loops.
static double lpIntersectionArea( const lpLoop2* a, const lpLoop2* b, lpLoop2* scratch )
{
	bool ccw = lpLoop2Area( b ) >= 0.0;
	lpLoop2* src = scratch;
	lpLoop2* dst = scratch + 1;
	*src = *a;
	for ( int k = 0; k < b->count && src->count > 0; ++k )
	{
		int e0 = ccw ? k : ( k + 1 ) % b->count; // a clockwise loop's edges, each reversed
		int e1 = ccw ? ( k + 1 ) % b->count : k;
		lpLoop2Clip( src, b->x[e0], b->y[e0], b->x[e1], b->y[e1], dst );
		lpLoop2* t = src;
		src = dst;
		dst = t;
	}
	return src->count >= 3 ? fabs( lpLoop2Area( src ) ) : 0.0;
}

static void lpPlaneBasis( lpD3 n, lpD3* u, lpD3* w )
{
	lpD3 t = fabs( n.x ) < 0.57 ? (lpD3){ 1.0, 0.0, 0.0 } : (lpD3){ 0.0, 1.0, 0.0 };
	*u = lpDCross( t, n );
	*u = lpDScale( 1.0 / lpDLength( *u ), *u );
	*w = lpDCross( n, *u );
}

static void lpProjectFace( const lpShape* s, int f, lpD3 u, lpD3 w, lpLoop2* out )
{
	const lpFace* face = s->faces + f;
	out->count = face->count;
	for ( int k = 0; k < face->count; ++k )
	{
		lpD3 p = lpD( s->vertices[s->indices[face->first + k]] );
		out->x[k] = lpDDot( p, u );
		out->y[k] = lpDDot( p, w );
	}
}

static double lpSegmentDistanceD( lpD3 p, lpD3 a, lpD3 b )
{
	lpD3 ab = lpDSub( b, a );
	double l2 = lpDDot( ab, ab );
	double t = l2 > 0.0 ? lpDDot( lpDSub( p, a ), ab ) / l2 : 0.0;
	t = t < 0.0 ? 0.0 : ( t > 1.0 ? 1.0 : t );
	return lpDLength( lpDSub( p, lpDAdd( a, lpDScale( t, ab ) ) ) );
}

// Distance from p to face f of s (a convex polygon, counter-clockwise about its normal)
static double lpFaceDistance( lpD3 p, const lpShape* s, int f )
{
	const lpFace* face = s->faces + f;
	lpD3 n = lpFaceNormal( face );
	double h = lpDDot( n, p ) - face->plane.offset;
	lpD3 q = lpDSub( p, lpDScale( h, n ) );
	bool inside = true;
	double best = DBL_MAX;
	for ( int k = 0; k < face->count; ++k )
	{
		lpD3 a = lpD( s->vertices[s->indices[face->first + k]] );
		lpD3 b = lpD( s->vertices[s->indices[face->first + ( k + 1 ) % face->count]] );
		inside = inside && lpDDot( lpDCross( lpDSub( b, a ), lpDSub( q, a ) ), n ) >= 0.0;
		double e = lpSegmentDistanceD( p, a, b );
		best = e < best ? e : best;
	}
	return inside ? fabs( h ) : best;
}

// ---- the checks ----

void lpShapeCensus_Init( lpShapeCensus* census )
{
	memset( census, 0, sizeof( *census ) );
	census->minEdge = DBL_MAX;
	census->minFaceArea = DBL_MAX;
}

// The nearest grid point (exact: a float times a power of two, then floor)
static lpD3 lpGridRound( lpVec3 v )
{
	return (lpD3){ floor( (double)v.x / LP_CHECK_GRID + 0.5 ), floor( (double)v.y / LP_CHECK_GRID + 0.5 ),
				   floor( (double)v.z / LP_CHECK_GRID + 0.5 ) };
}

void lpShapeCensus_Add( lpShapeCensus* census, const lpShape* s )
{
	census->shapes += 1;
	census->maxFaces = s->faceCount > census->maxFaces ? s->faceCount : census->maxFaces;
	census->maxVertices = s->vertexCount > census->maxVertices ? s->vertexCount : census->maxVertices;
	lpD3 grid[LP_POLY_MAX_VERTICES];
	int count = s->vertexCount < LP_POLY_MAX_VERTICES ? s->vertexCount : LP_POLY_MAX_VERTICES;
	for ( int i = 0; i < count; ++i )
	{
		grid[i] = lpGridRound( s->vertices[i] );
	}
	for ( int i = 0; i < count; ++i )
	{
		lpVec3 v = s->vertices[i];
		double m = fabs( (double)v.x );
		m = fabs( (double)v.y ) > m ? fabs( (double)v.y ) : m;
		m = fabs( (double)v.z ) > m ? fabs( (double)v.z ) : m;
		census->maxCoordinate = m > census->maxCoordinate ? m : census->maxCoordinate;
		census->beyondRange += m > LP_CHECK_RANGE ? 1 : 0;
		for ( int j = i + 1; j < count; ++j )
		{
			lpVec3 o = s->vertices[j];
			lpD3 d = lpDSub( lpD( v ), lpD( o ) );
			census->closePairs += lpDDot( d, d ) < LP_CHECK_GRID * LP_CHECK_GRID ? 1 : 0;
			bool same = v.x == o.x && v.y == o.y && v.z == o.z;
			bool snapped = grid[i].x == grid[j].x && grid[i].y == grid[j].y && grid[i].z == grid[j].z;
			census->gridCollisions += snapped && same == false ? 1 : 0;
		}
	}
	for ( int f = 0; f < s->faceCount; ++f )
	{
		const lpFace* face = s->faces + f;
		for ( int k = 0; k < face->count; ++k )
		{
			lpD3 a = lpD( s->vertices[s->indices[face->first + k]] );
			lpD3 b = lpD( s->vertices[s->indices[face->first + ( k + 1 ) % face->count]] );
			double e = lpDLength( lpDSub( b, a ) );
			census->minEdge = e < census->minEdge ? e : census->minEdge;
		}
		double area = lpDLength( lpFaceAreaVector( s->vertices, s->indices, face ) );
		census->minFaceArea = area < census->minFaceArea ? area : census->minFaceArea;
	}
}

static void lpAddCensus( lpShapeCensus* total, const lpShapeCensus* one )
{
	total->shapes += one->shapes;
	total->maxCoordinate = one->maxCoordinate > total->maxCoordinate ? one->maxCoordinate : total->maxCoordinate;
	total->beyondRange += one->beyondRange;
	total->minEdge = one->minEdge < total->minEdge ? one->minEdge : total->minEdge;
	total->minFaceArea = one->minFaceArea < total->minFaceArea ? one->minFaceArea : total->minFaceArea;
	total->closePairs += one->closePairs;
	total->gridCollisions += one->gridCollisions;
	total->maxFaces = one->maxFaces > total->maxFaces ? one->maxFaces : total->maxFaces;
	total->maxVertices = one->maxVertices > total->maxVertices ? one->maxVertices : total->maxVertices;
}

void lpFractureCheck_Init( lpFractureCheck* check )
{
	memset( check, 0, sizeof( *check ) );
	lpShapeCensus_Init( &check->census );
}

static double lpMaxD( double a, double b )
{
	return a > b ? a : b;
}

void lpFractureCheck_Add( lpFractureCheck* t, const lpFractureCheck* o )
{
	t->jobs += o->jobs;
	t->patternCells += o->patternCells;
	t->outputCells += o->outputCells;
	t->tilingViolations += o->tilingViolations;
	t->tilingGaps += o->tilingGaps;
	t->tilingMaxError = lpMaxD( t->tilingMaxError, o->tilingMaxError );
	t->overlapPairs += o->overlapPairs;
	t->overlapViolations += o->overlapViolations;
	t->overlapMax = lpMaxD( t->overlapMax, o->overlapMax );
	t->overlapMaxRelative = lpMaxD( t->overlapMaxRelative, o->overlapMaxRelative );
	t->overlapTotal += o->overlapTotal;
	t->siblingFaces += o->siblingFaces;
	t->siblingExact += o->siblingExact;
	t->siblingCovered += o->siblingCovered;
	t->siblingNear += o->siblingNear;
	t->siblingUnmatched += o->siblingUnmatched;
	t->siblingMaxDistance = lpMaxD( t->siblingMaxDistance, o->siblingMaxDistance );
	t->siblingUnmatchedArea += o->siblingUnmatchedArea;
	t->siblingMaxUnmatchedArea = lpMaxD( t->siblingMaxUnmatchedArea, o->siblingMaxUnmatchedArea );
	t->invalidCells += o->invalidCells;
	t->degenerateFaces += o->degenerateFaces;
	t->convexViolations += o->convexViolations;
	t->maxConvexExcess = lpMaxD( t->maxConvexExcess, o->maxConvexExcess );
	t->maxPlanarity = lpMaxD( t->maxPlanarity, o->maxPlanarity );
	t->keepersChecked += o->keepersChecked;
	t->containViolations += o->containViolations;
	t->maxContainExcess = lpMaxD( t->maxContainExcess, o->maxContainExcess );
	t->chipSets += o->chipSets;
	t->chipViolations += o->chipViolations;
	t->chipMaxTilingError = lpMaxD( t->chipMaxTilingError, o->chipMaxTilingError );
	t->chipMaxOverlap = lpMaxD( t->chipMaxOverlap, o->chipMaxOverlap );
	t->chipMismatches += o->chipMismatches;
	lpAddCensus( &t->census, &o->census );
	t->validatorOverflows += o->validatorOverflows;
}

int lpFractureCheck_Violations( const lpFractureCheck* c )
{
	return c->tilingViolations + c->overlapViolations + c->siblingNear + c->siblingUnmatched + c->invalidCells +
		   c->convexViolations + c->containViolations + c->chipViolations + c->chipMismatches;
}

double lpFractureCheck_Tiling( lpFractureCheck* check, const lpPoly* parent, lpShape* const* cells, int count )
{
	lpD3 r = { 0.0, 0.0, 0.0 }; // one reference for all: the faces two cells share cancel
	double whole = lpPolyVolume( parent, r );
	double sum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		sum += lpShapeVolume( cells[i], r );
	}
	double error = whole > 0.0 ? fabs( sum - whole ) / whole : ( sum != 0.0 ? 1.0 : 0.0 );
	check->tilingViolations += error > LP_CHECK_TILING ? 1 : 0;
	check->tilingGaps += error > 1e-3 ? 1 : 0;
	check->tilingMaxError = lpMaxD( check->tilingMaxError, error );
	return error;
}

void lpFractureCheck_Overlap( lpFractureCheck* check, lpShape* const* cells, int count )
{
	lpDPoly* scratch = lpAlloc( 2 * sizeof( lpDPoly ) );
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = i + 1; j < count; ++j )
		{
			if ( lpBoxesTouch( cells[i]->bounds, cells[j]->bounds, 0.0f ) == false )
			{
				continue;
			}
			check->overlapPairs += 1;
			double v = lpOverlapVolume( cells[i], cells[j], scratch );
			if ( v < 0.0 )
			{
				check->validatorOverflows += 1;
				continue;
			}
			if ( v > 0.0 )
			{
				double smaller = cells[i]->volume < cells[j]->volume ? cells[i]->volume : cells[j]->volume;
				check->overlapViolations += 1;
				check->overlapTotal += v;
				check->overlapMax = lpMaxD( check->overlapMax, v );
				check->overlapMaxRelative = lpMaxD( check->overlapMaxRelative, smaller > 0.0 ? v / smaller : 1.0 );
			}
		}
	}
	lpFree( scratch );
}

// A face that is not on the parent's surface: a cut through it (a tag naming a site or a plane, or else a centroid
// inside the parent by more than 10 um: a grain cell's faces are re-planed, so a surface face only lies near its plane)
static bool lpIsCutFace( const lpPoly* parent, const lpShape* s, int f )
{
	const lpFace* face = s->faces + f;
	if ( face->tag >= 0 )
	{
		return true;
	}
	lpD3 c = { 0.0, 0.0, 0.0 };
	for ( int k = 0; k < face->count; ++k )
	{
		c = lpDAdd( c, lpD( s->vertices[s->indices[face->first + k]] ) );
	}
	c = lpDScale( 1.0 / (double)face->count, c );
	double inside = -DBL_MAX;
	for ( int g = 0; g < parent->faceCount; ++g )
	{
		double d = lpDDot( lpFaceNormal( parent->faces + g ), c ) - parent->faces[g].plane.offset;
		inside = d > inside ? d : inside;
	}
	return inside < -1e-5;
}

static bool lpSameLoop( const lpShape* a, const lpFace* fa, const lpShape* b, const lpFace* fb )
{
	if ( fa->count != fb->count )
	{
		return false;
	}
	for ( int k = 0; k < fa->count; ++k )
	{
		lpVec3 v = a->vertices[a->indices[fa->first + k]];
		bool found = false;
		for ( int j = 0; j < fb->count && found == false; ++j )
		{
			lpVec3 o = b->vertices[b->indices[fb->first + j]];
			found = v.x == o.x && v.y == o.y && v.z == o.z;
		}
		if ( found == false )
		{
			return false;
		}
	}
	return true;
}

void lpFractureCheck_Siblings( lpFractureCheck* check, const lpPoly* parent, lpShape* const* cells, int count )
{
	lpLoop2* loops = lpAlloc( 4 * sizeof( lpLoop2 ) ); // the face, a neighbour's face, and scratch
	lpLoop2* mine = loops;
	lpLoop2* theirs = loops + 1;
	double distance[256];
	for ( int a = 0; a < count; ++a )
	{
		const lpShape* sa = cells[a];
		for ( int f = 0; f < sa->faceCount; ++f )
		{
			if ( lpIsCutFace( parent, sa, f ) == false )
			{
				continue;
			}
			const lpFace* face = sa->faces + f;
			check->siblingFaces += 1;
			lpD3 n = lpFaceNormal( face );
			lpD3 u, w;
			lpPlaneBasis( n, &u, &w );
			lpProjectFace( sa, f, u, w, mine );
			double area = fabs( lpLoop2Area( mine ) );
			lpAABB box = { sa->vertices[sa->indices[face->first]], sa->vertices[sa->indices[face->first]] };
			for ( int k = 0; k < face->count; ++k )
			{
				box.lowerBound = lpMin( box.lowerBound, sa->vertices[sa->indices[face->first + k]] );
				box.upperBound = lpMax( box.upperBound, sa->vertices[sa->indices[face->first + k]] );
				distance[k] = DBL_MAX;
			}

			bool exact = false;
			bool exactPlanes = true;
			double covered = 0.0;
			int covers = 0;
			double coverSpill = 0.0; // how much of the one face covering it, when there is one, lies beyond it
			for ( int b = 0; b < count && exact == false; ++b )
			{
				const lpShape* sb = cells[b];
				if ( b == a || lpBoxesTouch( box, sb->bounds, 1e-3f ) == false )
				{
					continue;
				}
				for ( int g = 0; g < sb->faceCount && exact == false; ++g )
				{
					const lpFace* other = sb->faces + g;
					if ( lpDDot( n, lpFaceNormal( other ) ) >= -0.9999 ||
						 fabs( (double)face->plane.offset + (double)other->plane.offset ) >= 1e-3 )
					{
						continue;
					}
					lpPlane p = face->plane, q = other->plane;
					bool negated = q.normal.x == -p.normal.x && q.normal.y == -p.normal.y && q.normal.z == -p.normal.z &&
								   q.offset == -p.offset;
					if ( negated && lpSameLoop( sa, face, sb, other ) )
					{
						exact = true;
						break;
					}
					lpProjectFace( sb, g, u, w, theirs );
					double shared = lpIntersectionArea( mine, theirs, loops + 2 );
					if ( shared <= 0.0 )
					{
						continue;
					}
					covered += shared;
					covers += 1;
					coverSpill = fabs( lpLoop2Area( theirs ) ) - shared;
					exactPlanes = exactPlanes && negated;
					for ( int k = 0; k < face->count && k < 256; ++k )
					{
						double d = lpFaceDistance( lpD( sa->vertices[sa->indices[face->first + k]] ), sb, g );
						distance[k] = d < distance[k] ? d : distance[k];
					}
				}
			}
			if ( exact )
			{
				check->siblingExact += 1;
				continue;
			}
			// One face that lies within this one (up to 0.1% of it) was meant to be its twin (Voronoi siblings, a head
			// joint): anything but the same vertices is a miss. Faces that reach past it or several of them on the negated
			// plane meet it at T-junctions (a masonry plate under its bricks, a radial wedge's rings): covering it is all
			// they can do.
			double uncovered = area - covered > 0.0 ? area - covered : 0.0;
			bool twin = covers == 1 && coverSpill <= 1e-3 * area;
			if ( covers > 0 && exactPlanes && twin == false && uncovered <= 1e-6 * area )
			{
				check->siblingCovered += 1;
			}
			else if ( covers > 0 && ( uncovered <= 1e-3 * area || uncovered <= 1e-8 ) )
			{
				check->siblingNear += 1;
				for ( int k = 0; k < face->count && k < 256; ++k )
				{
					check->siblingMaxDistance = lpMaxD( check->siblingMaxDistance, distance[k] );
				}
			}
			else
			{
				check->siblingUnmatched += 1;
				check->siblingUnmatchedArea += uncovered;
				check->siblingMaxUnmatchedArea = lpMaxD( check->siblingMaxUnmatchedArea, uncovered );
			}
		}
	}
	lpFree( loops );
}

static int lpCompareU32( const void* a, const void* b )
{
	uint32_t x = *(const uint32_t*)a, y = *(const uint32_t*)b;
	return ( x > y ) - ( x < y );
}

// Topology and convexity of one cell
static void lpCheckCell( lpFractureCheck* check, const lpShape* s )
{
	int vertexCount = s->vertexCount;
	bool invalid = vertexCount < 4 || s->faceCount < 4 || vertexCount > 256;
	uint32_t* edges = lpAlloc( sizeof( uint32_t ) * (size_t)( s->indexCount > 0 ? s->indexCount : 1 ) );
	int edgeCount = 0;
	bool used[256] = { false };
	for ( int f = 0; f < s->faceCount && vertexCount <= 256; ++f )
	{
		const lpFace* face = s->faces + f;
		if ( face->count < 3 || face->first + face->count > s->indexCount )
		{
			check->degenerateFaces += 1;
			invalid = true;
			continue;
		}
		bool degenerate = false;
		for ( int k = 0; k < face->count; ++k )
		{
			int a = s->indices[face->first + k];
			int b = s->indices[face->first + ( k + 1 ) % face->count];
			if ( a >= vertexCount || b >= vertexCount )
			{
				invalid = true;
				continue;
			}
			for ( int j = 0; j < k; ++j )
			{
				degenerate = degenerate || s->indices[face->first + j] == a;
			}
			used[a] = true;
			edges[edgeCount++] = (uint32_t)a << 8 | (uint32_t)b;
		}
		degenerate = degenerate || lpDLength( lpFaceAreaVector( s->vertices, s->indices, face ) ) == 0.0;
		check->degenerateFaces += degenerate ? 1 : 0;
		invalid = invalid || degenerate;
	}

	// Closed and oriented: every directed edge once, and its reverse once
	qsort( edges, (size_t)edgeCount, sizeof( uint32_t ), lpCompareU32 );
	for ( int e = 0; e < edgeCount && invalid == false; ++e )
	{
		uint32_t reverse = ( edges[e] & 0xFFu ) << 8 | edges[e] >> 8;
		invalid = ( e > 0 && edges[e] == edges[e - 1] ) ||
				  bsearch( &reverse, edges, (size_t)edgeCount, sizeof( uint32_t ), lpCompareU32 ) == NULL;
	}
	for ( int i = 0; i < vertexCount && i < 256; ++i )
	{
		invalid = invalid || used[i] == false;
		for ( int j = i + 1; j < vertexCount; ++j )
		{
			lpVec3 a = s->vertices[i], b = s->vertices[j];
			invalid = invalid || ( a.x == b.x && a.y == b.y && a.z == b.z );
		}
	}
	invalid = invalid || vertexCount - edgeCount / 2 + s->faceCount != 2;
	lpFree( edges );

	// Convex: no vertex outside a face plane; planar: a face's own vertices on its plane
	double excess = 0.0, planarity = 0.0;
	for ( int f = 0; f < s->faceCount; ++f )
	{
		const lpFace* face = s->faces + f;
		lpD3 n = lpFaceNormal( face );
		for ( int i = 0; i < vertexCount; ++i )
		{
			double d = lpDDot( n, lpD( s->vertices[i] ) ) - face->plane.offset;
			excess = lpMaxD( excess, d );
		}
		for ( int k = 0; k < face->count && face->first + face->count <= s->indexCount; ++k )
		{
			int i = s->indices[face->first + k];
			if ( i < vertexCount )
			{
				planarity = lpMaxD( planarity, fabs( lpDDot( n, lpD( s->vertices[i] ) ) - face->plane.offset ) );
			}
		}
	}
	check->invalidCells += invalid ? 1 : 0;
	check->convexViolations += excess > 0.0 ? 1 : 0;
	check->maxConvexExcess = lpMaxD( check->maxConvexExcess, excess );
	check->maxPlanarity = lpMaxD( check->maxPlanarity, planarity );
}

void lpFractureCheck_Cells( lpFractureCheck* check, lpShape* const* cells, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		lpCheckCell( check, cells[i] );
		lpShapeCensus_Add( &check->census, cells[i] );
	}
}

// How far the vertices of inner reach outside outer's face planes (negative: strictly inside)
static double lpExcess( const lpShape* inner, const lpShape* outer )
{
	double excess = -DBL_MAX;
	for ( int f = 0; f < outer->faceCount; ++f )
	{
		lpD3 n = lpFaceNormal( outer->faces + f );
		for ( int i = 0; i < inner->vertexCount; ++i )
		{
			excess = lpMaxD( excess, lpDDot( n, lpD( inner->vertices[i] ) ) - outer->faces[f].plane.offset );
		}
	}
	return excess;
}

static bool lpBoundsInside( lpAABB inner, lpAABB outer, float margin )
{
	return inner.lowerBound.x >= outer.lowerBound.x - margin && inner.lowerBound.y >= outer.lowerBound.y - margin &&
		   inner.lowerBound.z >= outer.lowerBound.z - margin && inner.upperBound.x <= outer.upperBound.x + margin &&
		   inner.upperBound.y <= outer.upperBound.y + margin && inner.upperBound.z <= outer.upperBound.z + margin;
}

void lpFractureCheck_Containment( lpFractureCheck* check, lpShape* const* inner, int innerCount, lpShape* const* outer,
								  int outerCount )
{
	for ( int i = 0; i < innerCount; ++i )
	{
		double best = DBL_MAX;
		for ( int pass = 0; pass < 2 && best == DBL_MAX; ++pass )
		{
			for ( int o = 0; o < outerCount; ++o )
			{
				if ( pass == 0 && lpBoundsInside( inner[i]->bounds, outer[o]->bounds, 1e-3f ) == false )
				{
					continue; // the second pass tries every cell
				}
				double e = lpExcess( inner[i], outer[o] );
				best = e < best ? e : best;
			}
		}
		check->keepersChecked += 1;
		if ( best > 0.0 )
		{
			check->containViolations += 1;
			check->maxContainExcess = lpMaxD( check->maxContainExcess, best );
		}
	}
}

void lpFractureCheck_Chips( lpFractureCheck* check, const lpShape* cell, lpShape* const* chips, int count )
{
	lpD3 r = lpD( cell->centroid );
	double whole = lpShapeVolume( cell, r );
	double sum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		sum += lpShapeVolume( chips[i], r );
	}
	double error = whole > 0.0 ? fabs( sum - whole ) / whole : 1.0;
	double overlap = 0.0;
	lpDPoly* scratch = lpAlloc( 2 * sizeof( lpDPoly ) );
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = i + 1; j < count; ++j )
		{
			double v = lpOverlapVolume( chips[i], chips[j], scratch );
			check->validatorOverflows += v < 0.0 ? 1 : 0;
			overlap = lpMaxD( overlap, v );
		}
	}
	lpFree( scratch );
	check->chipSets += 1;
	check->chipViolations += error > LP_CHECK_TILING || overlap > 0.0 ? 1 : 0;
	check->chipMaxTilingError = lpMaxD( check->chipMaxTilingError, error );
	check->chipMaxOverlap = lpMaxD( check->chipMaxOverlap, overlap );
}

void lpFractureJob_Validate( const lpFractureJob* job, lpFractureCheck* check )
{
	if ( job->cellCount < 2 )
	{
		return; // the piece stayed whole
	}
	check->jobs += 1;
	check->outputCells += job->cellCount;

	// a to c: the pattern again (a pure function of the input), in the job's frame
	lpShape** pattern = lpAlloc( sizeof( lpShape* ) * LP_MAX_SITES );
	int* sites = lpAlloc( sizeof( int ) * LP_MAX_SITES );
	lpFractureInput input = job->input;
	input.parent = &job->poly;
	int count = lpFracture( &input, pattern, sites, LP_MAX_SITES, NULL );
	check->patternCells += count;
	lpFractureCheck_Tiling( check, &job->poly, pattern, count );
	lpFractureCheck_Overlap( check, pattern, count );
	lpFractureCheck_Siblings( check, &job->poly, pattern, count );

	// The job again without its chips, from its own snapshot: the cells after the merge and before the chips
	lpFractureJob* plain = NULL;
	const lpFractureJob* merged = job;
	if ( job->chipSplits > 0 )
	{
		uint8_t* bytes = lpAlloc( LP_JOB_SNAPSHOT_MAX );
		plain = lpFractureJob_Create();
		int size = lpFractureJob_Write( job, bytes, LP_JOB_SNAPSHOT_MAX );
		bool read = lpFractureJob_Read( plain, bytes, size );
		lpFree( bytes );
		if ( read )
		{
			plain->chipSplits = 0;
			lpFracture_RunJob( plain );
			plain->chipSplits = job->chipSplits;
			merged = plain;
		}
		else
		{
			check->chipMismatches += 1; // its own snapshot did not read back: never
			lpFractureJob_Destroy( plain );
			plain = NULL;
		}
	}

	// e: the keepers before the merge (moved into the body frame and classified as the job did) inside its cells after
	lpShape** keepers = lpAlloc( sizeof( lpShape* ) * LP_MAX_SITES );
	int keeperCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		lpShape_Translate( pattern[i], job->center );
		if ( lpFracture_ClassifyCell( job, pattern[i] ) == lp_cellKeep )
		{
			keepers[keeperCount++] = pattern[i];
		}
	}
	lpFractureCheck_Containment( check, keepers, keeperCount, merged->cells, merged->cellCount );
	lpFree( keepers );

	// f: the chips made again tile their ghost cell, and with the cells left whole they are the job's own, in its order
	if ( plain != NULL )
	{
		int cellCount = plain->cellCount;
		for ( int i = 0; i < plain->cellCount; ++i )
		{
			lpShape* chips[4];
			int chipCount = 0;
			if ( plain->cellClass[i] == lp_cellGhost )
			{
				chipCount = lpFracture_ChipGhost( plain, i, cellCount, chips, NULL );
			}
			const lpShape* first = chipCount > 0 ? chips[0] : plain->cells[i];
			bool same = i < job->cellCount && job->cells[i]->digest == first->digest;
			for ( int k = 1; k < chipCount; ++k )
			{
				int at = cellCount + k - 1;
				same = same && at < job->cellCount && job->cells[at]->digest == chips[k]->digest;
			}
			check->chipMismatches += same ? 0 : 1;
			if ( chipCount > 0 )
			{
				lpFractureCheck_Chips( check, plain->cells[i], chips, chipCount );
				cellCount += chipCount - 1;
			}
			for ( int k = 0; k < chipCount; ++k )
			{
				lpShape_Destroy( chips[k] );
			}
		}
		check->chipMismatches += cellCount == job->cellCount ? 0 : 1;
	}

	// d and g: the job's own cells
	lpFractureCheck_Cells( check, job->cells, job->cellCount );

	for ( int i = 0; i < count; ++i )
	{
		lpShape_Destroy( pattern[i] );
	}
	lpFree( pattern );
	lpFree( sites );
	lpFractureJob_Destroy( plain );
}

// ---- the parts a scene authored ----

void lpWorld_CensusParts( const lpWorld* world, lpPartCensus* census )
{
	memset( census, 0, sizeof( *census ) );
	lpShapeCensus_Init( &census->all );
	lpShapeCensus_Init( &census->noGround );
	for ( int i = 0; i < world->pieces.count; ++i )
	{
		const lpPiece* p = world->pieces.data + i;
		if ( p->body < 0 || p->shape == NULL )
		{
			continue;
		}
		census->parts += 1;
		lpShapeCensus_Add( &census->all, p->shape );
		if ( p->material == lp_ground )
		{
			census->groundParts += 1;
		}
		else
		{
			lpShapeCensus_Add( &census->noGround, p->shape );
		}
	}
}
