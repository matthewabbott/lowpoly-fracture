// scene.c: see scene.h. Double arithmetic with + - * / and sqrt only (and the exact floor and ldexp of the start's
// rounding); every random draw in its own statement.
#include "scene.h"

#include "rng.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------------------------------------------------
// Vectors
// ---------------------------------------------------------------------------------------------------------------------

static double dot3d( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3d( const double a[3], const double b[3], double out[3] )
{
	double x = a[1] * b[2] - a[2] * b[1];
	double y = a[2] * b[0] - a[0] * b[2];
	double z = a[0] * b[1] - a[1] * b[0];
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static void sub3d( const double a[3], const double b[3], double out[3] )
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

static double len3d( const double a[3] )
{
	return sqrt( dot3d( a, a ) );
}

// ---------------------------------------------------------------------------------------------------------------------
// Hulls from half-spaces
// ---------------------------------------------------------------------------------------------------------------------

#define MAX_PLANES 16
#define MAX_CAND 128

// An angle-like key in [0, 4), increasing counter-clockwise from +x, without trigonometry
static double pseudo_angle( double x, double y )
{
	if ( y >= 0.0 )
	{
		return x >= 0.0 ? y / ( x + y ) : 1.0 + ( -x ) / ( -x + y );
	}
	return x < 0.0 ? 2.0 + ( -y ) / ( -x - y ) : 3.0 + x / ( x - y );
}

int scene_hull_from_planes( SceneHull* h, const double ( *n )[3], const double* d, int planeCount, double minEdge, double center[3] )
{
	const double eps = 1e-9;
	memset( h, 0, sizeof( *h ) );
	if ( planeCount > MAX_PLANES )
	{
		return 0;
	}

	// vertices: every plane triple's intersection inside all half-spaces, deduplicated
	double cand[MAX_CAND][3];
	int vc = 0;
	for ( int a = 0; a < planeCount; ++a )
	{
		for ( int b = a + 1; b < planeCount; ++b )
		{
			for ( int c = b + 1; c < planeCount; ++c )
			{
				double bc[3], ca[3], ab[3];
				cross3d( n[b], n[c], bc );
				cross3d( n[c], n[a], ca );
				cross3d( n[a], n[b], ab );
				double det = dot3d( n[a], bc );
				if ( det < 1e-9 && det > -1e-9 )
				{
					continue;
				}
				double x[3];
				for ( int i = 0; i < 3; ++i )
				{
					x[i] = ( d[a] * bc[i] + d[b] * ca[i] + d[c] * ab[i] ) / det;
				}
				int inside = 1;
				for ( int k = 0; k < planeCount && inside; ++k )
				{
					inside = dot3d( n[k], x ) <= d[k] + eps;
				}
				if ( !inside )
				{
					continue;
				}
				int dup = 0;
				for ( int k = 0; k < vc && !dup; ++k )
				{
					double e[3];
					sub3d( cand[k], x, e );
					dup = dot3d( e, e ) < 1e-14;
				}
				if ( !dup )
				{
					if ( vc == MAX_CAND || vc == SCENE_MAX_VERTS )
					{
						return 0;
					}
					cand[vc][0] = x[0];
					cand[vc][1] = x[1];
					cand[vc][2] = x[2];
					++vc;
				}
			}
		}
	}
	if ( vc < 4 )
	{
		return 0;
	}

	// faces: the vertices on each plane, counter-clockwise seen from outside
	int faceVerts[SCENE_MAX_FACES][SCENE_MAX_VERTS];
	int faceCount[SCENE_MAX_FACES];
	int fc = 0;
	for ( int p = 0; p < planeCount; ++p )
	{
		int idx[SCENE_MAX_VERTS];
		int m = 0;
		for ( int k = 0; k < vc; ++k )
		{
			double s = dot3d( n[p], cand[k] ) - d[p];
			if ( s < 1e-7 && s > -1e-7 )
			{
				idx[m++] = k;
			}
		}
		if ( m < 3 )
		{
			continue;
		}
		if ( fc == SCENE_MAX_FACES )
		{
			return 0;
		}
		double c[3] = { 0.0, 0.0, 0.0 };
		for ( int k = 0; k < m; ++k )
		{
			c[0] += cand[idx[k]][0];
			c[1] += cand[idx[k]][1];
			c[2] += cand[idx[k]][2];
		}
		c[0] /= m;
		c[1] /= m;
		c[2] /= m;
		double u[3], w[3];
		sub3d( cand[idx[0]], c, u );
		double ul = len3d( u );
		u[0] /= ul;
		u[1] /= ul;
		u[2] /= ul;
		cross3d( n[p], u, w );
		double key[SCENE_MAX_VERTS];
		for ( int k = 0; k < m; ++k )
		{
			double e[3];
			sub3d( cand[idx[k]], c, e );
			key[k] = pseudo_angle( dot3d( e, u ), dot3d( e, w ) );
		}
		for ( int i = 1; i < m; ++i ) // insertion sort by (key, index): a total order
		{
			for ( int j = i; j > 0 && ( key[j] < key[j - 1] || ( key[j] == key[j - 1] && idx[j] < idx[j - 1] ) ); --j )
			{
				double tk = key[j];
				key[j] = key[j - 1];
				key[j - 1] = tk;
				int ti = idx[j];
				idx[j] = idx[j - 1];
				idx[j - 1] = ti;
			}
		}
		h->normal[fc][0] = n[p][0];
		h->normal[fc][1] = n[p][1];
		h->normal[fc][2] = n[p][2];
		h->offset[fc] = d[p];
		faceCount[fc] = m;
		memcpy( faceVerts[fc], idx, (size_t)m * sizeof( int ) );
		++fc;
	}

	// half-edges: an undirected edge per vertex pair, its two halves adjacent (2k from the first face that uses it)
	int ec = 0;
	int edgeA[SCENE_MAX_EDGES / 2], edgeB[SCENE_MAX_EDGES / 2];
	int used[SCENE_MAX_EDGES];
	memset( used, 0, sizeof( used ) );
	for ( int f = 0; f < fc; ++f )
	{
		int first = -1, prev = -1;
		for ( int k = 0; k < faceCount[f]; ++k )
		{
			int a = faceVerts[f][k], b = faceVerts[f][( k + 1 ) % faceCount[f]];
			int e = -1;
			for ( int p = 0; p < ec / 2 && e < 0; ++p )
			{
				if ( edgeA[p] == a && edgeB[p] == b )
				{
					e = 2 * p;
				}
				else if ( edgeA[p] == b && edgeB[p] == a )
				{
					e = 2 * p + 1;
				}
			}
			if ( e < 0 )
			{
				if ( ec + 2 > SCENE_MAX_EDGES )
				{
					return 0;
				}
				edgeA[ec / 2] = a;
				edgeB[ec / 2] = b;
				e = ec;
				ec += 2;
			}
			if ( used[e] )
			{
				return 0; // a directed edge twice: not a closed convex surface
			}
			used[e] = 1;
			h->edge[e][1] = (uint8_t)( e ^ 1 );
			h->edge[e][2] = (uint8_t)a;
			h->edge[e][3] = (uint8_t)f;
			if ( prev >= 0 )
			{
				h->edge[prev][0] = (uint8_t)e;
			}
			else
			{
				first = e;
			}
			prev = e;
		}
		h->edge[prev][0] = (uint8_t)first;
		h->faceEdge[f] = first;
	}
	for ( int e = 0; e < ec; ++e )
	{
		if ( !used[e] )
		{
			return 0; // an edge with one face
		}
	}
	if ( vc - ec / 2 + fc != 2 )
	{
		return 0; // Euler
	}
	for ( int p = 0; p < ec / 2; ++p )
	{
		double e[3];
		sub3d( cand[edgeA[p]], cand[edgeB[p]], e );
		if ( len3d( e ) < minEdge )
		{
			return 0;
		}
	}

	// mass properties: tetrahedra from the vertex average r to each face's fan
	double r[3] = { 0.0, 0.0, 0.0 };
	for ( int k = 0; k < vc; ++k )
	{
		r[0] += cand[k][0];
		r[1] += cand[k][1];
		r[2] += cand[k][2];
	}
	r[0] /= vc;
	r[1] /= vc;
	r[2] /= vc;
	double V = 0.0, S[3] = { 0.0, 0.0, 0.0 }, C[3][3] = { { 0.0 } };
	for ( int f = 0; f < fc; ++f )
	{
		double a[3];
		sub3d( cand[faceVerts[f][0]], r, a );
		for ( int k = 1; k + 1 < faceCount[f]; ++k )
		{
			double b[3], c[3], bc[3];
			sub3d( cand[faceVerts[f][k]], r, b );
			sub3d( cand[faceVerts[f][k + 1]], r, c );
			cross3d( b, c, bc );
			double D = dot3d( a, bc );
			V += D / 6.0;
			for ( int i = 0; i < 3; ++i )
			{
				S[i] += D / 24.0 * ( a[i] + b[i] + c[i] );
				for ( int j = 0; j < 3; ++j )
				{
					double t = 2.0 * ( a[i] * a[j] + b[i] * b[j] + c[i] * c[j] ) + a[i] * b[j] + a[j] * b[i] + a[i] * c[j] + a[j] * c[i] +
							   b[i] * c[j] + b[j] * c[i];
					C[i][j] += D / 120.0 * t;
				}
			}
		}
	}
	if ( !( V > 1e-9 ) )
	{
		return 0;
	}
	double g[3] = { S[0] / V, S[1] / V, S[2] / V };
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			C[i][j] -= V * g[i] * g[j];
		}
	}
	double tr = C[0][0] + C[1][1] + C[2][2];
	h->volume = V;
	h->inertia[0] = tr - C[0][0];
	h->inertia[1] = -C[0][1];
	h->inertia[2] = -C[0][2];
	h->inertia[3] = tr - C[1][1];
	h->inertia[4] = -C[1][2];
	h->inertia[5] = tr - C[2][2];

	// move to the centre of mass
	double cm[3] = { r[0] + g[0], r[1] + g[1], r[2] + g[2] };
	center[0] = cm[0];
	center[1] = cm[1];
	center[2] = cm[2];
	h->vertexCount = vc;
	h->faceCount = fc;
	h->edgeCount = ec;
	double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
	h->maxExtent = 0.0;
	for ( int k = 0; k < vc; ++k )
	{
		sub3d( cand[k], cm, h->v[k] );
		for ( int i = 0; i < 3; ++i )
		{
			lo[i] = h->v[k][i] < lo[i] ? h->v[k][i] : lo[i];
			hi[i] = h->v[k][i] > hi[i] ? h->v[k][i] : hi[i];
		}
		double l = len3d( h->v[k] );
		h->maxExtent = l > h->maxExtent ? l : h->maxExtent;
	}
	for ( int i = 0; i < 3; ++i )
	{
		h->boundsCenter[i] = 0.5 * ( lo[i] + hi[i] );
		h->boundsHalf[i] = 0.5 * ( hi[i] - lo[i] );
	}
	h->minExtent = 1e30;
	for ( int f = 0; f < fc; ++f )
	{
		h->offset[f] -= dot3d( h->normal[f], cm );
		h->minExtent = h->offset[f] < h->minExtent ? h->offset[f] : h->minExtent;
	}
	return 1;
}

// ---------------------------------------------------------------------------------------------------------------------
// Scenes
// ---------------------------------------------------------------------------------------------------------------------

static int add_hull( Scene* s, const SceneHull* h )
{
	s->hulls = (SceneHull*)realloc( s->hulls, (size_t)( s->hullCount + 1 ) * sizeof( SceneHull ) );
	s->hulls[s->hullCount] = *h;
	return s->hullCount++;
}

static void box_planes( double hx, double hy, double hz, double n[][3], double* d )
{
	static const double axes[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	double half[3] = { hx, hy, hz };
	for ( int i = 0; i < 6; ++i )
	{
		n[i][0] = axes[i][0];
		n[i][1] = axes[i][1];
		n[i][2] = axes[i][2];
		d[i] = half[i / 2];
	}
}

void scene_box_hull( SceneHull* h, double hx, double hy, double hz )
{
	double n[6][3], d[6], c[3];
	box_planes( hx, hy, hz, n, d );
	if ( !scene_hull_from_planes( h, (const double( * )[3])n, d, 6, 0.0, c ) )
	{
		fprintf( stderr, "box hull failed\n" );
		exit( 1 );
	}
}

int scene_chunk_hull( SceneHull* h, Pcg* r, double hx, double hy, double hz )
{
	int redraws = 0;
	for ( ;; )
	{
		double n[9][3], d[9], c[3];
		box_planes( hx, hy, hz, n, d );
		int cuts = pcg_int( r, 1, 3 );
		double hmin = hx < hy ? ( hx < hz ? hx : hz ) : ( hy < hz ? hy : hz );
		for ( int k = 0; k < cuts; ++k )
		{
			pcg_unit_vector( r, n[6 + k] );
			double t = pcg_range( r, 0.3, 0.9 );
			d[6 + k] = t * hmin;
		}
		if ( scene_hull_from_planes( h, (const double( * )[3])n, d, 6 + cuts, 0.02, c ) )
		{
			return redraws;
		}
		redraws += 1;
	}
}

static int add_box_hull( Scene* s, double hx, double hy, double hz )
{
	SceneHull h;
	scene_box_hull( &h, hx, hy, hz );
	return add_hull( s, &h );
}

// A box cut by one to three random planes (scene_chunk_hull)
static int add_chunk_hull( Scene* s, Pcg* r, double hx, double hy, double hz )
{
	SceneHull h;
	s->chunkRedraws += scene_chunk_hull( &h, r, hx, hy, hz );
	s->chunkCount += 1;
	return add_hull( s, &h );
}

static void invert_sym( const double m[6], double out[6] )
{
	// m = [a b c; b d e; c e f]
	double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5];
	double A = d * f - e * e, B = c * e - b * f, Cc = b * e - c * d;
	double det = a * A + b * B + c * Cc;
	out[0] = A / det;
	out[1] = B / det;
	out[2] = Cc / det;
	out[3] = ( a * f - c * c ) / det;
	out[4] = ( b * c - a * e ) / det;
	out[5] = ( a * d - b * b ) / det;
}

static SceneBody* add_body( Scene* s, int hull, int isStatic, double x, double y, double z, double density )
{
	s->bodies = (SceneBody*)realloc( s->bodies, (size_t)( s->bodyCount + 1 ) * sizeof( SceneBody ) );
	SceneBody* b = s->bodies + s->bodyCount++;
	memset( b, 0, sizeof( *b ) );
	b->hull = hull;
	b->isStatic = isStatic;
	b->p[0] = x;
	b->p[1] = y;
	b->p[2] = z;
	b->q[3] = 1.0;
	b->friction = 0.6;
	if ( !isStatic )
	{
		const SceneHull* h = s->hulls + hull;
		b->mass = density * h->volume;
		b->invMass = 1.0 / b->mass;
		double I[6];
		for ( int i = 0; i < 6; ++i )
		{
			I[i] = density * h->inertia[i];
		}
		invert_sym( I, b->invI );
	}
	return b;
}

static void set_quat( SceneBody* b, const double q[4] )
{
	b->q[0] = q[0];
	b->q[1] = q[1];
	b->q[2] = q[2];
	b->q[3] = q[3];
}

// A rotation about z by the angle whose half-angle tangent is t (no trigonometry)
static void quat_about_z( double t, double q[4] )
{
	double l = sqrt( 1.0 + t * t );
	q[0] = 0.0;
	q[1] = 0.0;
	q[2] = t / l;
	q[3] = 1.0 / l;
}

// stack10: ten 0.5 m cubes, touching, on a ground box whose top is y = 0 (stackN: N cubes, N in 1 to 40)
static void build_stack( Scene* s, int count )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	int cube = add_box_hull( s, 0.25, 0.25, 0.25 );
	for ( int i = 0; i < count; ++i )
	{
		add_body( s, cube, 0, 0.0, 0.25 + 0.5 * i, 0.0, 1000.0 );
	}
}

// pile200: a 4 m x 4 m pit (floor and four walls 2.5 m high); 200 bodies in eight layers of 5 x 5 at 0.7 m spacing,
// from 3 m up, random sizes (half extents 0.1 to 0.2 m) and orientations; every fifth an irregular chunk.
static void build_pile200( Scene* s, Pcg* r )
{
	int floorHull = add_box_hull( s, 6.0, 0.5, 6.0 );
	add_body( s, floorHull, 1, 0.0, -0.5, 0.0, 0.0 );
	int wallX = add_box_hull( s, 0.25, 1.25, 2.5 );
	int wallZ = add_box_hull( s, 2.5, 1.25, 0.25 );
	add_body( s, wallX, 1, 2.25, 1.25, 0.0, 0.0 );
	add_body( s, wallX, 1, -2.25, 1.25, 0.0, 0.0 );
	add_body( s, wallZ, 1, 0.0, 1.25, 2.25, 0.0 );
	add_body( s, wallZ, 1, 0.0, 1.25, -2.25, 0.0 );
	for ( int k = 0; k < 200; ++k )
	{
		int layer = k / 25, cell = k % 25;
		double x = -1.4 + 0.7 * ( cell % 5 );
		double z = -1.4 + 0.7 * ( cell / 5 );
		double y = 3.0 + 0.7 * layer;
		double hx = pcg_range( r, 0.1, 0.2 );
		double hy = pcg_range( r, 0.1, 0.2 );
		double hz = pcg_range( r, 0.1, 0.2 );
		int hull = k % 5 == 4 ? add_chunk_hull( s, r, hx, hy, hz ) : add_box_hull( s, hx, hy, hz );
		SceneBody* b = add_body( s, hull, 0, x, y, z, 1000.0 );
		double q[4];
		pcg_unit_quat( r, q );
		set_quat( b, q );
	}
}

// bounce: a 0.5 m box dropped from 2 m (its centre; 1.75 m to the ground), restitution 0.5 (the ground's 0: the pair
// takes the larger). The analytic first apex: the centre at 0.25 + 0.5^2 1.75 = 0.6875 m.
static void build_bounce( Scene* s )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	SceneBody* b = add_body( s, add_box_hull( s, 0.25, 0.25, 0.25 ), 0, 0.0, 2.0, 0.0, 1000.0 );
	b->restitution = 0.5;
}

// ramp: a static slope of 25 degrees (the rotation's half-angle tangent is tan 12.5 degrees; tan 25 degrees = 0.4663)
// and two 0.5 m boxes 2.5 m up it, friction 0.6 (body 2) and 0.2 (body 3). Mixed with the ramp's 0.6 (Box3D's sqrt):
// 0.6 holds, sqrt(0.12) = 0.3464 slides at g (sin 25 - 0.3464 cos 25) = 1.0866 m/s^2, 6.5 m to the ramp's lower end.
static void build_ramp( Scene* s )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	double q[4];
	quat_about_z( 0.22169466264293988, q );
	double cy = 2.0;
	SceneBody* ramp = add_body( s, add_box_hull( s, 4.0, 0.25, 2.0 ), 1, 0.0, cy, 0.0, 0.0 );
	set_quat( ramp, q );
	int box = add_box_hull( s, 0.25, 0.25, 0.25 );
	double along[3] = { 1.0 - 2.0 * q[2] * q[2], 2.0 * q[2] * q[3], 0.0 }; // the ramp's local x in the world (up the slope)
	double up[3] = { -2.0 * q[2] * q[3], 1.0 - 2.0 * q[2] * q[2], 0.0 }; // its local y
	for ( int i = 0; i < 2; ++i )
	{
		double off = 0.25 + 0.25 + 0.001;
		double z = i == 0 ? -0.8 : 0.8;
		SceneBody* b = add_body( s, box, 0, along[0] * 2.5 + up[0] * off, cy + along[1] * 2.5 + up[1] * off, z, 1000.0 );
		set_quat( b, q );
		b->friction = i == 0 ? 0.6 : 0.2;
	}
}

// ratio: 3 t resting on 1 kg (bodies 1, 2: a 1 m cube on a 0.5 m cube); a 100 kg plate (1 m x 5 cm x 1 m, body 7)
// resting on four 0.05 kg chips (10 x 2 x 10 cm, bodies 3 to 6) under its corners, 3 m away
static void build_ratio( Scene* s )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	int small = add_box_hull( s, 0.25, 0.25, 0.25 );
	add_body( s, small, 0, 0.0, 0.25, 0.0, 1.0 / 0.125 );
	int big = add_box_hull( s, 0.5, 0.5, 0.5 );
	add_body( s, big, 0, 0.0, 1.0, 0.0, 3000.0 );
	int chip = add_box_hull( s, 0.05, 0.01, 0.05 );
	for ( int k = 0; k < 4; ++k )
	{
		double x = 3.0 + ( ( k & 1 ) ? 0.4 : -0.4 );
		double z = ( k & 2 ) ? 0.4 : -0.4;
		add_body( s, chip, 0, x, 0.01, z, 0.05 / ( 0.1 * 0.02 * 0.1 ) );
	}
	int plate = add_box_hull( s, 0.5, 0.025, 0.5 );
	add_body( s, plate, 0, 3.0, 0.02 + 0.025, 0.0, 100.0 / ( 1.0 * 0.05 * 1.0 ) );
}

// chip: a 0.05 kg chip (10 x 2 x 10 cm) at 10 m/s, (6, -8, 0), spinning at 40 rad/s about z, 0.3 m above the ground
static void build_chip( Scene* s )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	int chip = add_box_hull( s, 0.05, 0.01, 0.05 );
	SceneBody* b = add_body( s, chip, 0, -3.0, 0.3, 0.0, 0.05 / ( 0.1 * 0.02 * 0.1 ) );
	b->v[0] = 6.0;
	b->v[1] = -8.0;
	b->w[2] = 40.0;
}

// The joint frame of both arm joints: a rotation of -90 degrees about x, so the frame's z axis (the hinge) is the
// world's y (b3RotateVector takes (0, 0, 1) to (0, 1, 0))
static void hinge_up( double q[4] )
{
	double h = sqrt( 0.5 );
	q[0] = -h;
	q[1] = 0.0;
	q[2] = 0.0;
	q[3] = h;
}

static SceneJoint* add_joint( Scene* s, int a, int b, const double anchorWorld[3] )
{
	s->joints = (SceneJoint*)realloc( s->joints, (size_t)( s->jointCount + 1 ) * sizeof( SceneJoint ) );
	SceneJoint* j = s->joints + s->jointCount++;
	memset( j, 0, sizeof( *j ) );
	j->bodyA = a;
	j->bodyB = b;
	for ( int i = 0; i < 3; ++i )
	{
		j->localAnchorA[i] = anchorWorld[i] - s->bodies[a].p[i]; // the bodies start unrotated
		j->localAnchorB[i] = anchorWorld[i] - s->bodies[b].p[i];
	}
	hinge_up( j->localFrameA );
	hinge_up( j->localFrameB );
	return j;
}

// arm: a static base (body 1, 0.3 x 0.5 x 0.3 m, its top at 0.5 m); link 1 (body 2: 1 m x 10 cm x 10 cm, 10 kg) hinged
// about the vertical on the base's axis, 3 cm above it, driven by a servo (torque cap 150 N m, gain 8/s, at most 1.5
// rad/s) whose target sweeps +-1 rad as a triangle wave of 6 s; link 2 (body 3: 0.8 m, 8 kg) hinged to link 1's end
// (a 4 cm gap) with limits +-0.5 rad and no motor. Both links lie along +x at y 0.58 m; positive angles turn toward -z.
// A stack of four 0.2 m cubes (bodies 4 to 7, 8 kg each) stands at (1.2, 0.8) in x, z (1.44 m out, at -0.59 rad), in
// link 2's path on the return sweep (reached near tick 245, after the first 2 s): the links span 0.53 to 0.63 m in
// height, the third and fourth cubes 0.4 to 0.8. Each cube is offset by up to 1.5 cm and turned by up to 3.4 degrees
// about y. (A stack's face-on-face contacts are ties from tick 1, its settling differs by dialect at the mm level: the
// stack is reached after the 2 s over which the joint's angle is compared with D's.)
static void build_arm( Scene* s )
{
	int ground = add_box_hull( s, 10.0, 0.5, 10.0 );
	add_body( s, ground, 1, 0.0, -0.5, 0.0, 0.0 );
	add_body( s, add_box_hull( s, 0.15, 0.25, 0.15 ), 1, 0.0, 0.25, 0.0, 0.0 );
	double y = 0.58;
	add_body( s, add_box_hull( s, 0.5, 0.05, 0.05 ), 0, 0.5, y, 0.0, 1000.0 );
	add_body( s, add_box_hull( s, 0.4, 0.05, 0.05 ), 0, 1.44, y, 0.0, 1000.0 );
	int cube = add_box_hull( s, 0.1, 0.1, 0.1 );
	static const double dx[4] = { 0.0, 0.012, -0.008, 0.006 }, dz[4] = { 0.0, -0.006, 0.01, -0.012 }, yaw[4] = { 0.0, 0.02, -0.015, 0.03 };
	for ( int i = 0; i < 4; ++i )
	{
		SceneBody* b = add_body( s, cube, 0, 1.2 + dx[i], 0.1 + 0.2 * i, 0.8 + dz[i], 1000.0 );
		double l = sqrt( 1.0 + yaw[i] * yaw[i] );
		double q[4] = { 0.0, yaw[i] / l, 0.0, 1.0 / l }; // about y by the angle whose half-angle tangent is yaw[i]
		set_quat( b, q );
	}
	double hinge1[3] = { 0.0, y, 0.0 };
	SceneJoint* j = add_joint( s, 1, 2, hinge1 );
	j->enableMotor = 1;
	j->maxMotorTorque = 150.0;
	j->servo = 1;
	j->servoGain = 8.0;
	j->servoMaxSpeed = 1.5;
	j->servoAmplitude = 1.0;
	j->servoPeriod = 360;
	double hinge2[3] = { 1.02, y, 0.0 };
	j = add_joint( s, 2, 3, hinge2 );
	j->enableLimit = 1;
	j->lowerAngle = -0.5;
	j->upperAngle = 0.5;
}

// grid:K: K copies of pile200's pit and bodies (the same seed, so the same draws) on a square grid 6.5 m apart, on one
// floor; sleep is the caller's (step 7 times it off)
static void build_grid( Scene* s, int K, uint64_t seed )
{
	int side = 1;
	while ( side * side < K )
	{
		++side;
	}
	double span = 6.5 * ( side - 1 );
	int floorHull = add_box_hull( s, span * 0.5 + 3.0, 0.5, span * 0.5 + 3.0 );
	add_body( s, floorHull, 1, span * 0.5, -0.5, span * 0.5, 0.0 );
	int wallX = add_box_hull( s, 0.25, 1.25, 2.5 );
	int wallZ = add_box_hull( s, 2.5, 1.25, 0.25 );
	for ( int c = 0; c < K; ++c )
	{
		double ox = 6.5 * ( c % side ), oz = 6.5 * ( c / side );
		add_body( s, wallX, 1, ox + 2.25, 1.25, oz, 0.0 );
		add_body( s, wallX, 1, ox - 2.25, 1.25, oz, 0.0 );
		add_body( s, wallZ, 1, ox, 1.25, oz + 2.25, 0.0 );
		add_body( s, wallZ, 1, ox, 1.25, oz - 2.25, 0.0 );
		Pcg r = pcg_seed( seed, 7 );
		for ( int k = 0; k < 200; ++k )
		{
			int layer = k / 25, cell = k % 25;
			double x = -1.4 + 0.7 * ( cell % 5 );
			double z = -1.4 + 0.7 * ( cell / 5 );
			double y = 3.0 + 0.7 * layer;
			double hx = pcg_range( &r, 0.1, 0.2 );
			double hy = pcg_range( &r, 0.1, 0.2 );
			double hz = pcg_range( &r, 0.1, 0.2 );
			int hull = k % 5 == 4 ? add_chunk_hull( s, &r, hx, hy, hz ) : add_box_hull( s, hx, hy, hz );
			SceneBody* b = add_body( s, hull, 0, ox + x, y, oz + z, 1000.0 );
			double q[4];
			pcg_unit_quat( &r, q );
			set_quat( b, q );
		}
	}
}

double scene_servo_target( const SceneJoint* j, int tick )
{
	if ( !j->servo || j->servoPeriod <= 0 )
	{
		return 0.0;
	}
	int t = tick % j->servoPeriod;
	double x = (double)t / (double)j->servoPeriod;
	double tri = x < 0.25 ? 4.0 * x : x < 0.75 ? 2.0 - 4.0 * x : 4.0 * x - 4.0;
	return j->servoAmplitude * tri;
}

void scene_joint_text( const SceneJoint* j, int k, char* out, int size )
{
	snprintf( out, (size_t)size,
			  "joint %d: bodies %d %d anchorA %.17g %.17g %.17g anchorB %.17g %.17g %.17g frameA %.17g %.17g %.17g %.17g frameB %.17g %.17g "
			  "%.17g %.17g limit %d %.17g %.17g motor %d %.17g %.17g servo %d %.17g %.17g %.17g %d",
			  k, j->bodyA, j->bodyB, j->localAnchorA[0], j->localAnchorA[1], j->localAnchorA[2], j->localAnchorB[0], j->localAnchorB[1],
			  j->localAnchorB[2], j->localFrameA[0], j->localFrameA[1], j->localFrameA[2], j->localFrameA[3], j->localFrameB[0], j->localFrameB[1],
			  j->localFrameB[2], j->localFrameB[3], j->enableLimit, j->lowerAngle, j->upperAngle, j->enableMotor, j->maxMotorTorque, j->motorSpeed,
			  j->servo, j->servoGain, j->servoMaxSpeed, j->servoAmplitude, j->servoPeriod );
}

const char* scene_names( void )
{
	return "stack10 (stackN) pile200 bounce ramp ratio chip arm grid:K";
}

// x rounded to float, then to the grid 2^-bits (to nearest, halves up): a value every dialect stores exactly. The float
// is already on the grid when |x| >= 2^(23 - bits); below that, the grid's value has at most 24 significant bits, so it
// is still a float (and a double).
static double start_value( double x, int bits )
{
	double f = (double)(float)x;
	return ldexp( floor( ldexp( f, bits ) + 0.5 ), -bits );
}

int scene_round_start( Scene* s )
{
	int moved = 0;
	for ( int i = 0; i < s->bodyCount; ++i )
	{
		SceneBody* b = s->bodies + i;
		for ( int k = 0; k < 3; ++k )
		{
			double p = start_value( b->p[k], SCENE_GRID_P );
			double v = start_value( b->v[k], SCENE_GRID_V );
			double w = start_value( b->w[k], SCENE_GRID_W );
			moved += ( p != (double)(float)b->p[k] ) + ( v != (double)(float)b->v[k] ) + ( w != (double)(float)b->w[k] );
			b->p[k] = p;
			b->v[k] = v;
			b->w[k] = w;
		}
		for ( int k = 0; k < 4; ++k )
		{
			double q = start_value( b->q[k], SCENE_GRID_Q );
			moved += q != (double)(float)b->q[k];
			b->q[k] = q;
		}
	}
	s->startMoved = moved;
	return moved;
}

int scene_build( Scene* s, const char* name, uint64_t seed )
{
	memset( s, 0, sizeof( *s ) );
	snprintf( s->name, sizeof( s->name ), "%s", name );
	s->seed = seed;
	Pcg r = pcg_seed( seed, 7 );
	int count = strncmp( name, "stack", 5 ) == 0 ? atoi( name + 5 ) : 0;
	if ( count >= 1 && count <= 40 )
		build_stack( s, count );
	else if ( strcmp( name, "pile200" ) == 0 )
		build_pile200( s, &r );
	else if ( strcmp( name, "bounce" ) == 0 )
		build_bounce( s );
	else if ( strcmp( name, "ramp" ) == 0 )
		build_ramp( s );
	else if ( strcmp( name, "ratio" ) == 0 )
		build_ratio( s );
	else if ( strcmp( name, "chip" ) == 0 )
		build_chip( s );
	else if ( strcmp( name, "arm" ) == 0 )
		build_arm( s );
	else if ( strncmp( name, "grid:", 5 ) == 0 && atoi( name + 5 ) >= 1 && atoi( name + 5 ) <= 256 )
		build_grid( s, atoi( name + 5 ), seed );
	else
		return 0;
	scene_round_start( s );
	return 1;
}

void scene_free( Scene* s )
{
	free( s->bodies );
	free( s->hulls );
	free( s->joints );
	memset( s, 0, sizeof( *s ) );
}
