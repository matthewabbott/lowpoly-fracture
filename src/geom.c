// SPDX-License-Identifier: MIT

#include "geom.h"

#include <math.h>

static uint64_t lpAbs64( int64_t x )
{
	return x < 0 ? 0u - (uint64_t)x : (uint64_t)x;
}

static uint64_t lpGcd64( uint64_t a, uint64_t b )
{
	while ( b != 0 )
	{
		uint64_t r = a % b;
		a = b;
		b = r;
	}
	return a;
}

static bool lpGeomFail( lpGeomReject* why, lpGeomReject reason )
{
	if ( why != NULL )
	{
		*why = reason;
	}
	return false;
}

static bool lpGeomOk( lpGeomReject* why )
{
	if ( why != NULL )
	{
		*why = lp_geomOk;
	}
	return true;
}

int lpIPlane_Compare( const lpIPlane* a, const lpIPlane* b )
{
	for ( int i = 0; i < 3; ++i )
	{
		if ( a->n[i] != b->n[i] )
		{
			return a->n[i] < b->n[i] ? -1 : 1;
		}
	}
	return a->d < b->d ? -1 : ( a->d > b->d ? 1 : 0 );
}

bool lpIPlane_IsValid( const lpIPlane* plane )
{
	uint64_t g = lpAbs64( plane->d );
	int64_t l1 = 0;
	for ( int i = 0; i < 3; ++i )
	{
		int64_t n = plane->n[i];
		if ( n > LP_PLANE_NORMAL_MAX || n < -LP_PLANE_NORMAL_MAX )
		{
			return false;
		}
		l1 += n < 0 ? -n : n;
		g = lpGcd64( lpAbs64( n ), g );
	}
	return l1 > 0 && lpAbs64( plane->d ) <= (uint64_t)l1 * LP_GRID_RANGE && g == 1;
}

// The canonical half-space v.x <= v.m2 / 2 (a plane through the half-grid point m2 / 2, which lies within the grid's
// range): v divided by its gcd, then doubled only when the offset would be a half. Every constructor ends here.
static bool lpIPlane_Through( const int64_t v[3], const int64_t m2[3], lpIPlane* plane, lpGeomReject* why )
{
	uint64_t g = lpGcd64( lpGcd64( lpAbs64( v[0] ), lpAbs64( v[1] ) ), lpAbs64( v[2] ) );
	if ( g == 0 )
	{
		return lpGeomFail( why, lp_geomDegenerate );
	}
	int64_t n[3];
	for ( int i = 0; i < 3; ++i )
	{
		// |v_i| / g <= 2^63 here; the range check follows before any product
		uint64_t m = lpAbs64( v[i] ) / g;
		if ( m > LP_PLANE_NORMAL_MAX )
		{
			return lpGeomFail( why, lp_geomNormalRange );
		}
		n[i] = v[i] < 0 ? -(int64_t)m : (int64_t)m;
	}
	// |n_i| < 2^24 and |m2_i| <= 2^24: |d2| < 3 2^48
	int64_t d2 = n[0] * m2[0] + n[1] * m2[1] + n[2] * m2[2];
	if ( d2 % 2 == 0 )
	{
		// gcd(n) = 1, so gcd(n, d2 / 2) = 1
		*plane = (lpIPlane){ { (int32_t)n[0], (int32_t)n[1], (int32_t)n[2] }, d2 / 2 };
		return lpGeomOk( why );
	}
	for ( int i = 0; i < 3; ++i )
	{
		// gcd(2 n) = 2 and d2 is odd, so gcd(2 n, d2) = 1
		n[i] *= 2;
		if ( n[i] > LP_PLANE_NORMAL_MAX || n[i] < -LP_PLANE_NORMAL_MAX )
		{
			return lpGeomFail( why, lp_geomNormalRange );
		}
	}
	*plane = (lpIPlane){ { (int32_t)n[0], (int32_t)n[1], (int32_t)n[2] }, d2 };
	return lpGeomOk( why );
}

bool lpIPlane_MakeBisector( const int32_t a[3], const int32_t b[3], lpIPlane* plane, lpGeomReject* why )
{
	if ( lpGrid_Contains( a ) == false || lpGrid_Contains( b ) == false )
	{
		return lpGeomFail( why, lp_geomOffGrid );
	}
	// 2 (b - a).x <= (b - a).(a + b): halved, the plane through the midpoint with normal b - a
	int64_t v[3], m2[3];
	for ( int i = 0; i < 3; ++i )
	{
		v[i] = (int64_t)b[i] - a[i];
		m2[i] = (int64_t)a[i] + b[i];
	}
	return lpIPlane_Through( v, m2, plane, why );
}

// ---- snapping ----

// x rounded half away from zero (|x| < 2^31)
static int32_t lpRoundHalfAway( double x )
{
	double ax = x < 0.0 ? -x : x;
	double f = floor( ax );
	double r = ( ax - f >= 0.5 ? f + 1.0 : f ); // ax - f is exact
	return x < 0.0 ? -(int32_t)r : (int32_t)r;
}

bool lpGeom_SnapDirection( lpVec3 direction, int k, int32_t out[3] )
{
	if ( k < 0 || k > LP_SNAP_BITS_MAX )
	{
		return false;
	}
	double v[3] = { (double)direction.x, (double)direction.y, (double)direction.z };
	double m = 0.0;
	for ( int i = 0; i < 3; ++i )
	{
		if ( !( v[i] - v[i] == 0.0 ) )
		{
			return false; // NaN or infinite
		}
		double a = v[i] < 0.0 ? -v[i] : v[i];
		m = a > m ? a : m;
	}
	if ( m == 0.0 )
	{
		return false;
	}
	double scale = (double)( 1 << k );
	for ( int i = 0; i < 3; ++i )
	{
		// v_i / m is in [-1, 1] and exactly +-1 for the largest; times 2^k exactly
		out[i] = lpRoundHalfAway( v[i] / m * scale );
	}
	return true;
}

bool lpGeom_SnapAxis( lpVec3 axis, int32_t g[3] )
{
	double a[3] = { (double)axis.x, (double)axis.y, (double)axis.z };
	double length2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
	if ( !( length2 > 0.0 ) || !( length2 - length2 == 0.0 ) )
	{
		return false;
	}
	// Every candidate in a fixed order, scored by cos^2 of its angle to the axis times |axis|^2; the first best wins.
	// Doubles without contraction: the same choice everywhere.
	enum
	{
		lp_m = LP_GRAIN_AXIS_MAX
	};
	double best = -1.0;
	int32_t pick[3] = { 1, 0, 0 };
	for ( int x = 0; x <= lp_m; ++x )
	{
		for ( int y = -lp_m; y <= lp_m; ++y )
		{
			for ( int z = -lp_m; z <= lp_m; ++z )
			{
				int norm2 = x * x + y * y + z * z;
				if ( norm2 == 0 )
				{
					continue;
				}
				double dot = (double)x * a[0] + (double)y * a[1] + (double)z * a[2];
				double score = dot * dot / (double)norm2;
				if ( score > best )
				{
					best = score;
					pick[0] = x;
					pick[1] = y;
					pick[2] = z;
				}
			}
		}
	}
	uint64_t gcd = lpGcd64( lpGcd64( (uint64_t)pick[0], lpAbs64( pick[1] ) ), lpAbs64( pick[2] ) );
	int sign = pick[0] > 0 || ( pick[0] == 0 && ( pick[1] > 0 || ( pick[1] == 0 && pick[2] > 0 ) ) ) ? 1 : -1;
	for ( int i = 0; i < 3; ++i )
	{
		g[i] = sign * pick[i] / (int32_t)gcd;
	}
	return true;
}

bool lpIPlane_MakeSnapped( lpVec3 normal, int k, const int32_t p[3], lpIPlane* plane, lpGeomReject* why )
{
	if ( lpGrid_Contains( p ) == false )
	{
		return lpGeomFail( why, lp_geomOffGrid );
	}
	if ( k < 0 || k > LP_SNAP_BITS_MAX )
	{
		return lpGeomFail( why, lp_geomParamRange );
	}
	int32_t s[3];
	if ( lpGeom_SnapDirection( normal, k, s ) == false )
	{
		return lpGeomFail( why, lp_geomDegenerate );
	}
	int64_t v[3] = { s[0], s[1], s[2] };
	int64_t m2[3] = { 2 * (int64_t)p[0], 2 * (int64_t)p[1], 2 * (int64_t)p[2] };
	return lpIPlane_Through( v, m2, plane, why );
}

bool lpIPlane_MakeAxial( const int32_t g[3], const int32_t p[3], lpVec3 direction, int k, lpIPlane* plane,
						 lpGeomReject* why )
{
	if ( lpGrid_Contains( p ) == false )
	{
		return lpGeomFail( why, lp_geomOffGrid );
	}
	if ( k < 0 || k > LP_SNAP_BITS_MAX )
	{
		return lpGeomFail( why, lp_geomParamRange );
	}
	for ( int i = 0; i < 3; ++i )
	{
		if ( g[i] > LP_PLANE_NORMAL_MAX || g[i] < -LP_PLANE_NORMAL_MAX )
		{
			return lpGeomFail( why, lp_geomParamRange );
		}
	}
	int32_t s[3];
	if ( lpGeom_SnapDirection( direction, k, s ) == false )
	{
		return lpGeomFail( why, lp_geomDegenerate );
	}
	// |g_i| < 2^24, |s_i| <= 2^23: each product below 2^47
	int64_t v[3] = { (int64_t)g[1] * s[2] - (int64_t)g[2] * s[1], (int64_t)g[2] * s[0] - (int64_t)g[0] * s[2],
					 (int64_t)g[0] * s[1] - (int64_t)g[1] * s[0] };
	int64_t m2[3] = { 2 * (int64_t)p[0], 2 * (int64_t)p[1], 2 * (int64_t)p[2] };
	return lpIPlane_Through( v, m2, plane, why ); // v = 0 (s along g): degenerate
}

// ---- the grain's metric ----

bool lpIMetric_Make( lpVec3 axis, float stretch, int32_t extent, lpIMetric* metric, lpGeomReject* why )
{
	if ( !( stretch >= 1.0f ) || !( stretch - stretch == 0.0f ) || extent < 0 || extent > 2 * LP_GRID_RANGE )
	{
		return lpGeomFail( why, lp_geomParamRange );
	}
	int32_t g[3];
	if ( lpGeom_SnapAxis( axis, g ) == false )
	{
		return lpGeomFail( why, lp_geomDegenerate );
	}
	double sigma = (double)stretch;
	int32_t t = lpRoundHalfAway( (double)LP_GRAIN_Q * ( 1.0 - 1.0 / ( sigma * sigma ) ) );
	if ( t >= LP_GRAIN_Q )
	{
		return lpGeomFail( why, lp_geomParamRange ); // c would reach 1: stretch beyond about 22.6
	}
	int32_t norm2 = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
	int32_t s = LP_GRAIN_Q * norm2;
	int32_t h = (int32_t)lpGcd64( (uint64_t)s, (uint64_t)t ); // t = 0: s / s = 1
	s /= h;
	t /= h;
	// |K|_inf: the largest absolute row sum of s I - t g g^T (s > t g_i^2, so the diagonal stays positive)
	int64_t rowMax = 0;
	for ( int i = 0; i < 3; ++i )
	{
		int64_t row = 0;
		for ( int j = 0; j < 3; ++j )
		{
			int64_t k = ( i == j ? (int64_t)s : 0 ) - (int64_t)t * g[i] * g[j];
			row += k < 0 ? -k : k;
		}
		rowMax = row > rowMax ? row : rowMax;
	}
	int32_t span = (int32_t)( LP_PLANE_NORMAL_MAX / ( 2 * rowMax ) );
	int32_t lattice = 1;
	while ( (int64_t)span * lattice < extent )
	{
		lattice *= 2;
	}
	*metric = (lpIMetric){ { g[0], g[1], g[2] }, s, t, lattice, span };
	return lpGeomOk( why );
}

bool lpIPlane_MakeMetricBisector( const lpIMetric* metric, const int32_t a[3], const int32_t b[3], lpIPlane* plane,
								  lpGeomReject* why )
{
	if ( lpGrid_Contains( a ) == false || lpGrid_Contains( b ) == false )
	{
		return lpGeomFail( why, lp_geomOffGrid );
	}
	int64_t delta[3], m2[3];
	for ( int i = 0; i < 3; ++i )
	{
		int64_t step = (int64_t)b[i] - a[i];
		if ( step % metric->lattice != 0 )
		{
			return lpGeomFail( why, lp_geomOffLattice );
		}
		delta[i] = step / metric->lattice;
		m2[i] = (int64_t)a[i] + b[i];
	}
	// v = K delta: |delta_i| <= 2^24 and |K| < 2^17, far inside int64. The bisector is 2 v.x <= v.(a + b).
	const int32_t* g = metric->g;
	int64_t along = g[0] * delta[0] + g[1] * delta[1] + g[2] * delta[2];
	int64_t v[3];
	for ( int i = 0; i < 3; ++i )
	{
		v[i] = (int64_t)metric->s * delta[i] - (int64_t)metric->t * g[i] * along;
	}
	return lpIPlane_Through( v, m2, plane, why );
}

// ---- predicates ----

bool lpIVertex_FromPlanes( const lpIPlane* a, const lpIPlane* b, const lpIPlane* c, lpIVertex* vertex )
{
	// Cross products of the normals, in int64: each component below 2^49
	const int32_t* n1 = a->n;
	const int32_t* n2 = b->n;
	const int32_t* n3 = c->n;
	int64_t c23[3] = { (int64_t)n2[1] * n3[2] - (int64_t)n2[2] * n3[1], (int64_t)n2[2] * n3[0] - (int64_t)n2[0] * n3[2],
					   (int64_t)n2[0] * n3[1] - (int64_t)n2[1] * n3[0] };
	int64_t c31[3] = { (int64_t)n3[1] * n1[2] - (int64_t)n3[2] * n1[1], (int64_t)n3[2] * n1[0] - (int64_t)n3[0] * n1[2],
					   (int64_t)n3[0] * n1[1] - (int64_t)n3[1] * n1[0] };
	int64_t c12[3] = { (int64_t)n1[1] * n2[2] - (int64_t)n1[2] * n2[1], (int64_t)n1[2] * n2[0] - (int64_t)n1[0] * n2[2],
					   (int64_t)n1[0] * n2[1] - (int64_t)n1[1] * n2[0] };

	lpI128 w = lpI128_Add( lpI128_Add( lpI128_Mul64( n1[0], c23[0] ), lpI128_Mul64( n1[1], c23[1] ) ),
						   lpI128_Mul64( n1[2], c23[2] ) );
	int sign = lpI128_Sign( w );
	if ( sign == 0 )
	{
		return false;
	}
	lpI128 x[3];
	for ( int i = 0; i < 3; ++i )
	{
		x[i] = lpI128_Add( lpI128_Add( lpI128_Mul64( a->d, c23[i] ), lpI128_Mul64( b->d, c31[i] ) ),
						   lpI128_Mul64( c->d, c12[i] ) );
	}
	if ( sign < 0 )
	{
		w = lpI128_Neg( w );
		for ( int i = 0; i < 3; ++i )
		{
			x[i] = lpI128_Neg( x[i] );
		}
	}
	vertex->x = x[0];
	vertex->y = x[1];
	vertex->z = x[2];
	vertex->w = w;
	return true;
}

lpI128 lpIVertex_Evaluate( const lpIVertex* vertex, const lpIPlane* plane )
{
	lpI128 s = lpI128_Add( lpI128_Mul( vertex->x, plane->n[0] ), lpI128_Mul( vertex->y, plane->n[1] ) );
	s = lpI128_Add( s, lpI128_Mul( vertex->z, plane->n[2] ) );
	return lpI128_Sub( s, lpI128_Mul( vertex->w, plane->d ) );
}
