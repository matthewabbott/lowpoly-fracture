// quant.c: see quant.h.
#include "quant.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int g_range;

static int64_t round_scaled( double x, int s )
{
	return (int64_t)floor( ldexp( x, s ) + 0.5 );
}

// A double into T: F rounds to nearest, D keeps it, V4 rounds half up in format s (saturating, counted)
static T QT( double x, int s )
{
#if defined( DIALECT_F )
	(void)s;
	return (float)x;
#elif defined( DIALECT_D )
	(void)s;
	return x;
#else
	int64_t v = round_scaled( x, s );
	if ( v > 2147483647LL || v < -2147483647LL )
	{
		g_range += 1;
		v = v > 0 ? 2147483647LL : -2147483647LL;
	}
	return (T)v;
#endif
}

// The same, rounded toward +infinity (plane offsets: every quantised vertex stays on or inside)
static T QT_up( double x, int s )
{
#if defined( DIALECT_F )
	(void)s;
	float f = (float)x;
	return (double)f < x ? nextafterf( f, 3.0e38f ) : f;
#elif defined( DIALECT_D )
	(void)s;
	return x;
#else
	double y = ceil( ldexp( x, s ) );
	if ( y > 2147483647.0 || y < -2147483647.0 )
	{
		g_range += 1;
		y = y > 0.0 ? 2147483647.0 : -2147483647.0;
	}
	return (T)y;
#endif
}

double toy_val( T x, int s )
{
#if defined( DIALECT_V4 )
	return ldexp( (double)x, -s );
#else
	(void)s;
	return (double)x;
#endif
}

#if defined( DIALECT_V4 )
static double pos_word( uint32_t lo, int32_t hi )
{
	return (double)hi + ldexp( (double)lo, -32 );
}
double toy_pos_x( const Pos3* p )
{
	return pos_word( p->xlo, p->xhi );
}
double toy_pos_y( const Pos3* p )
{
	return pos_word( p->ylo, p->yhi );
}
double toy_pos_z( const Pos3* p )
{
	return pos_word( p->zlo, p->zhi );
}
static void set_pos( Pos3* p, const double x[3] )
{
	uint64_t u[3];
	for ( int i = 0; i < 3; ++i )
	{
		u[i] = (uint64_t)round_scaled( x[i], S_P );
	}
	p->xlo = (uint32_t)u[0];
	p->xhi = (int32_t)(uint32_t)( u[0] >> 32 );
	p->ylo = (uint32_t)u[1];
	p->yhi = (int32_t)(uint32_t)( u[1] >> 32 );
	p->zlo = (uint32_t)u[2];
	p->zhi = (int32_t)(uint32_t)( u[2] >> 32 );
}
#else
double toy_pos_x( const Pos3* p )
{
	return (double)p->x;
}
double toy_pos_y( const Pos3* p )
{
	return (double)p->y;
}
double toy_pos_z( const Pos3* p )
{
	return (double)p->z;
}
static void set_pos( Pos3* p, const double x[3] )
{
	p->x = (T)x[0];
	p->y = (T)x[1];
	p->z = (T)x[2];
}
#endif

// e so that maxAbs 2^e lies in [2^(bits-1), 2^bits) (V4 mantissas); 0 for zero
static int exp_for( double maxAbs, int bits )
{
	if ( !( maxAbs > 0.0 ) )
	{
		return 0;
	}
	int k;
	frexp( maxAbs, &k ); // maxAbs = m 2^k, m in [0.5, 1)
	return bits - k;
}

static V3 qv3( const double v[3], int s )
{
	V3 r = { QT( v[0], s ), QT( v[1], s ), QT( v[2], s ) };
	return r;
}

// A symmetric matrix; F sends entries under 2^-30 to +0 (the kernels' snap: an inertia's round-off off-diagonals, near
// 1e-17, would otherwise make subnormal products)
static Sym3 qsym( const double m[6], int s )
{
	double c[6];
	for ( int i = 0; i < 6; ++i )
	{
#if defined( DIALECT_F )
		c[i] = fabs( m[i] ) < ldexp( 1.0, -30 ) ? 0.0 : m[i];
#else
		c[i] = m[i];
#endif
	}
	Sym3 r = { QT( c[0], s ), QT( c[1], s ), QT( c[2], s ), QT( c[3], s ), QT( c[4], s ), QT( c[5], s ) };
	return r;
}

void toy_default_settings( ToySettings* s )
{
	s->gravity = 10.0;
	s->aabbMargin = 0.05;
	s->linearSlop = 0.005;
	s->sleepSpeed = 0.05;
	s->sleepTicks = 30;
	s->enableSleep = 1;
	s->maxLinearSpeed = 400.0;
	s->maxRotationPerStep = 0.25 * 3.14159265358979323846;
	s->contactSpeed = 3.0;
	s->restitutionThreshold = 1.0;
	s->restitutionIterations = 2;
	s->recycle = 1;
}

void toy_quantize( const Scene* sc, const ToySettings* st, ToyData* d )
{
	memset( d, 0, sizeof( *d ) );
	g_range = 0;

	// hulls: points, bounds, extents; faces recomputed from the quantised points; half-edges packed
	d->hullCount = sc->hullCount;
	for ( int h = 0; h < sc->hullCount; ++h )
	{
		d->pointCount += sc->hulls[h].vertexCount;
		d->faceCount += sc->hulls[h].faceCount;
		d->edgeCount += sc->hulls[h].edgeCount;
	}
	d->hulls = (Hull*)calloc( (size_t)( d->hullCount > 0 ? d->hullCount : 1 ), sizeof( Hull ) );
	d->points = (V3*)calloc( (size_t)( d->pointCount > 0 ? d->pointCount : 1 ), sizeof( V3 ) );
	d->faces = (HullFace*)calloc( (size_t)( d->faceCount > 0 ? d->faceCount : 1 ), sizeof( HullFace ) );
	d->edges = (uint32_t*)calloc( (size_t)( d->edgeCount > 0 ? d->edgeCount : 1 ), sizeof( uint32_t ) );
	int pv = 0, pf = 0, pe = 0;
	for ( int h = 0; h < sc->hullCount; ++h )
	{
		const SceneHull* sh = sc->hulls + h;
		Hull* H = d->hulls + h;
		H->boundsCenter = qv3( sh->boundsCenter, S_R );
		H->boundsHalf = qv3( sh->boundsHalf, S_R );
		H->maxExtent = QT( sh->maxExtent, S_R );
		H->minExtent = QT( sh->minExtent, S_R );
		H->vertexStart = pv;
		H->vertexCount = sh->vertexCount;
		H->faceStart = pf;
		H->faceCount = sh->faceCount;
		H->edgeStart = pe;
		H->edgeCount = sh->edgeCount;
		double qp[SCENE_MAX_VERTS][3]; // the quantised points, back in double (exact)
		for ( int k = 0; k < sh->vertexCount; ++k )
		{
			d->points[pv + k] = qv3( sh->v[k], S_R );
			qp[k][0] = toy_val( d->points[pv + k].x, S_R );
			qp[k][1] = toy_val( d->points[pv + k].y, S_R );
			qp[k][2] = toy_val( d->points[pv + k].z, S_R );
		}
		// Box3D's maxExtent vector: the largest |vertex| per axis (from the quantised points: exact)
		double ext[3] = { 0.0, 0.0, 0.0 };
		for ( int k = 0; k < sh->vertexCount; ++k )
		{
			for ( int i = 0; i < 3; ++i )
			{
				ext[i] = fabs( qp[k][i] ) > ext[i] ? fabs( qp[k][i] ) : ext[i];
			}
		}
		H->maxExtentV = qv3( ext, S_R );
		for ( int e = 0; e < sh->edgeCount; ++e )
		{
			d->edges[pe + e] = (uint32_t)sh->edge[e][0] | ( (uint32_t)sh->edge[e][1] << 8 ) | ( (uint32_t)sh->edge[e][2] << 16 ) |
							   ( (uint32_t)sh->edge[e][3] << 24 );
		}
		for ( int f = 0; f < sh->faceCount; ++f )
		{
			// Newell's normal over the face's loop of quantised points
			double n[3] = { 0.0, 0.0, 0.0 };
			int e0 = sh->faceEdge[f], e = e0, guard = 0;
			do
			{
				int a = sh->edge[e][2], b = sh->edge[sh->edge[e][0]][2];
				n[0] += ( qp[a][1] - qp[b][1] ) * ( qp[a][2] + qp[b][2] );
				n[1] += ( qp[a][2] - qp[b][2] ) * ( qp[a][0] + qp[b][0] );
				n[2] += ( qp[a][0] - qp[b][0] ) * ( qp[a][1] + qp[b][1] );
				e = sh->edge[e][0];
			} while ( e != e0 && ++guard < SCENE_MAX_EDGES );
			double l = sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2] );
			n[0] /= l;
			n[1] /= l;
			n[2] /= l;
			HullFace* F = d->faces + pf + f;
			F->normal = qv3( n, S_Q );
			double nq[3] = { toy_val( F->normal.x, S_Q ), toy_val( F->normal.y, S_Q ), toy_val( F->normal.z, S_Q ) };
			double off = -1e30;
			e = e0;
			guard = 0;
			do
			{
				int a = sh->edge[e][2];
				double s = nq[0] * qp[a][0] + nq[1] * qp[a][1] + nq[2] * qp[a][2];
				off = s > off ? s : off;
				e = sh->edge[e][0];
			} while ( e != e0 && ++guard < SCENE_MAX_EDGES );
			F->offset = QT_up( off, S_R );
			F->edge = sh->faceEdge[f];
		}
		pv += sh->vertexCount;
		pf += sh->faceCount;
		pe += sh->edgeCount;
	}

	// bodies
	d->bodyCount = sc->bodyCount;
	d->state = (BodyState*)calloc( (size_t)d->bodyCount, sizeof( BodyState ) );
	d->pose = (BodyPose*)calloc( (size_t)d->bodyCount, sizeof( BodyPose ) );
	d->mass = (BodyMass*)calloc( (size_t)d->bodyCount, sizeof( BodyMass ) );
	for ( int i = 0; i < sc->bodyCount; ++i )
	{
		const SceneBody* b = sc->bodies + i;
		BodyState* S = d->state + i;
		S->v = qv3( b->v, S_V );
		S->w = qv3( b->w, S_W );
		S->dq.s = QT( 1.0, S_Q );
		BodyPose* P = d->pose + i;
		set_pos( &P->p, b->p );
		P->q.x = QT( b->q[0], S_Q );
		P->q.y = QT( b->q[1], S_Q );
		P->q.z = QT( b->q[2], S_Q );
		P->q.s = QT( b->q[3], S_Q );
		BodyMass* M = d->mass + i;
		M->hull = b->hull;
		M->flags = b->isStatic ? BODY_STATIC : 0;
		double rowMax = 0.0;
		for ( int r = 0; r < 3; ++r )
		{
			static const int row[3][3] = { { 0, 1, 2 }, { 1, 3, 4 }, { 2, 4, 5 } };
			double sum = fabs( b->invI[row[r][0]] ) + fabs( b->invI[row[r][1]] ) + fabs( b->invI[row[r][2]] );
			rowMax = sum > rowMax ? sum : rowMax;
		}
#if defined( DIALECT_V4 )
		M->eM = exp_for( b->invMass, 31 ); // mantissa in [2^30, 2^31)
		M->eI = exp_for( rowMax, 30 );	   // row sums below 2^30, so R S R^T fits (its entries are bounded by them)
#endif
		M->invMass = QT( b->invMass, M->eM );
		M->invIl = qsym( b->invI, M->eI );
		M->friction = QT( b->friction, S_MS );
		M->restitution = QT( b->restitution, S_MS );
	}

	// Params: F computes in float as Box3D does (h = dt / 4); V4 rounds the doubles; D keeps them
	Params* P = &d->params;
	memset( P, 0, sizeof( *P ) );
	double maxAngularSpeed = st->maxRotationPerStep * 60.0;
#if defined( DIALECT_F )
	float dt = 1.0f / 60.0f;
	float h = dt / 4.0f;
	P->h = h;
	float inv_dt = 1.0f / dt;
	P->inv_h = 4.0f * inv_dt; // Box3D's subStepCount * inv_dt
	P->gravityDelta.x = 0.0f;
	P->gravityDelta.y = h * (float)( -st->gravity );
	P->gravityDelta.z = 0.0f;
#else
	double h = 1.0 / 240.0;
	P->h = QT( h, S_H );
	P->inv_h = QT( 240.0, S_IH );
	P->gravityDelta.x = 0;
	P->gravityDelta.y = QT( h * -st->gravity, S_V );
	P->gravityDelta.z = 0;
#endif
	P->aabbMargin = QT( st->aabbMargin, S_R );
	P->linearSlop = QT( st->linearSlop, S_R );
	P->speculativeDistance = QT( 4.0 * st->linearSlop, S_R );
	P->sleepV = QT( st->sleepSpeed, S_V );
	P->sleepW = QT( st->sleepSpeed, S_W );
	P->sleepSq = QT( st->sleepSpeed * st->sleepSpeed, S_VV );
	P->invMaxLinearSpeed = QT( 1.0 / st->maxLinearSpeed, S_MS );
	P->invMaxAngularSpeed = QT( 1.0 / maxAngularSpeed, S_MS );
	P->oneU = QT( 1.0, S_U );
	P->one = QT( 1.0, S_MS );
	P->half = QT( 0.5, S_MS );
	P->oneHalf = QT( 1.5, S_MS );
	// the narrowphase's tolerances: Box3D's scalar b3CollideHulls (0.5 linearSlop, 0.9), B3_PARALLEL_EDGE_TOL, the
	// reduction's bias
	P->faceTolerance = QT( 0.5 * st->linearSlop, S_R );
	P->edgeRelTolerance = QT( 0.9, S_MS );
	P->parallelTolerance = QT( 0.005, S_MS );
	P->reduceBias = QT( 0.95, S_MS );
	P->speculativeSq = QT( 16.0 * st->linearSlop * st->linearSlop, S_D2 );
	P->cacheTolerance = QT( st->linearSlop, S_S );
	// the contact solve: Box3D's b3MakeSoft at h for the contact hertz (min( 30, 0.125 inv_h )) and damping ratio 10, the
	// static softness at twice the hertz and half the ratio; F in float as Box3D computes them (B3_PI), V4 and D from
	// doubles. The push uses massScale x biasRate.
#if defined( DIALECT_F )
	{
		float hz = 30.0f < 0.125f * P->inv_h ? 30.0f : 0.125f * P->inv_h;
		float hertz[2] = { hz, 2.0f * hz }, zeta[2] = { 10.0f, 5.0f };
		float bias[2], mass[2], imp[2];
		for ( int i = 0; i < 2; ++i )
		{
			float omega = 2.0f * 3.14159265359f * hertz[i];
			float a1 = 2.0f * zeta[i] + h * omega;
			float a2 = h * omega * a1;
			float a3 = 1.0f / ( 1.0f + a2 );
			bias[i] = omega / a1;
			mass[i] = a2 * a3;
			imp[i] = a3;
		}
		P->dynBiasRate = mass[0] * bias[0];
		P->dynMassScale = mass[0];
		P->dynImpulseScale = imp[0];
		P->staBiasRate = mass[1] * bias[1];
		P->staMassScale = mass[1];
		P->staImpulseScale = imp[1];
	}
#else
	{
		double hz = 30.0 < 0.125 * 240.0 ? 30.0 : 0.125 * 240.0;
		double hertz[2] = { hz, 2.0 * hz }, zeta[2] = { 10.0, 5.0 };
		double bias[2], mass[2], imp[2];
		for ( int i = 0; i < 2; ++i )
		{
			double omega = 2.0 * 3.14159265358979323846 * hertz[i];
			double a1 = 2.0 * zeta[i] + h * omega;
			double a2 = h * omega * a1;
			double a3 = 1.0 / ( 1.0 + a2 );
			bias[i] = omega / a1;
			mass[i] = a2 * a3;
			imp[i] = a3;
		}
		P->dynBiasRate = QT( mass[0] * bias[0], S_BR );
		P->dynMassScale = QT( mass[0], S_MS );
		P->dynImpulseScale = QT( imp[0], S_MS );
		P->staBiasRate = QT( mass[1] * bias[1], S_BR );
		P->staMassScale = QT( mass[1], S_MS );
		P->staImpulseScale = QT( imp[1], S_MS );
	}
#endif
	P->negContactSpeed = QT( -st->contactSpeed, S_V );
	P->negRestitutionThreshold = QT( -st->restitutionThreshold, S_V );
	P->invSpeculative = QT( 1.0 / ( 4.0 * st->linearSlop ), S_BR );
	P->minFrictionWeight = QT( ldexp( 1.0, -20 ), S_S );
	P->oneV = QT( 1.0, S_S );
	P->twoV = QT( 2.0, S_S );
	// contact recycling: Box3D's B3_CONTACT_RECYCLE_DISTANCE (10 linearSlop) and angular threshold; isFast's safety factor
	double recycle = st->recycle ? 10.0 * st->linearSlop : 0.0;
	P->recycleDistance = QT( recycle, S_R );
	P->recycleNonTouching = QT( recycle < 4.0 * st->linearSlop ? recycle : 4.0 * st->linearSlop, S_R );
	P->recycleAngular = QT( 0.99240388, S_MS );
	P->fastSafety = QT( 0.5, S_MS );
#if defined( DIALECT_F )
	P->dt = dt;
#else
	P->dt = QT( 1.0 / 60.0, S_H );
#endif
	P->narrowDiag = 0;
	P->sleepTicks = st->sleepTicks;
	P->sleepCap = 1000;
	P->substeps = 4;
	P->enableSleep = st->enableSleep;
	P->restitutionIterations = st->restitutionIterations;
	d->rangeErrors = g_range;
}

void toy_free_data( ToyData* d )
{
	free( d->hulls );
	free( d->points );
	free( d->faces );
	free( d->edges );
	free( d->state );
	free( d->pose );
	free( d->mass );
	memset( d, 0, sizeof( *d ) );
}
