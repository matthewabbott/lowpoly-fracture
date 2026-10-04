// rng.h: PCG32 (E11's) and a few draws built from it, for the scene generator and the battery. Every draw is one
// call in its own statement (docs/determinism-rules.md rule 13): never two draws in one expression.
#ifndef TOY_RNG_H
#define TOY_RNG_H

#include <math.h>
#include <stdint.h>

typedef struct Pcg
{
	uint64_t state, inc;
} Pcg;

static inline uint32_t pcg_next( Pcg* r )
{
	uint64_t old = r->state;
	r->state = old * 6364136223846793005ULL + r->inc;
	uint32_t xs = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
	uint32_t rot = (uint32_t)( old >> 59u );
	return ( xs >> rot ) | ( xs << ( ( 0u - rot ) & 31 ) );
}

static inline Pcg pcg_seed( uint64_t seed, uint64_t seq )
{
	Pcg r = { 0, ( seq << 1u ) | 1u };
	pcg_next( &r );
	r.state += seed;
	pcg_next( &r );
	return r;
}

// [0, 1) from 32 random bits
static inline double pcg_unit( Pcg* r )
{
	return (double)pcg_next( r ) * ( 1.0 / 4294967296.0 );
}

static inline double pcg_range( Pcg* r, double lo, double hi )
{
	double u = pcg_unit( r );
	return lo + ( hi - lo ) * u;
}

// an integer in [lo, hi]
static inline int pcg_int( Pcg* r, int lo, int hi )
{
	uint32_t u = pcg_next( r );
	return lo + (int)( u % (uint32_t)( hi - lo + 1 ) );
}

// +-m 2^e with m in [1, 2) and e in [elo, ehi] (ldexp is exact)
static inline double pcg_log( Pcg* r, int elo, int ehi )
{
	int e = pcg_int( r, elo, ehi );
	double m = 1.0 + pcg_unit( r );
	uint32_t s = pcg_next( r ) & 1u;
	return ldexp( s ? -m : m, e );
}

// a unit vector by rejection (no trigonometry)
static inline void pcg_unit_vector( Pcg* r, double out[3] )
{
	for ( ;; )
	{
		double x = pcg_range( r, -1.0, 1.0 );
		double y = pcg_range( r, -1.0, 1.0 );
		double z = pcg_range( r, -1.0, 1.0 );
		double l2 = x * x + y * y + z * z;
		if ( l2 > 1e-4 && l2 <= 1.0 )
		{
			double s = 1.0 / sqrt( l2 );
			out[0] = x * s;
			out[1] = y * s;
			out[2] = z * s;
			return;
		}
	}
}

// a unit quaternion (x, y, z, s) by rejection
static inline void pcg_unit_quat( Pcg* r, double q[4] )
{
	for ( ;; )
	{
		double a = pcg_range( r, -1.0, 1.0 );
		double b = pcg_range( r, -1.0, 1.0 );
		double c = pcg_range( r, -1.0, 1.0 );
		double d = pcg_range( r, -1.0, 1.0 );
		double l2 = a * a + b * b + c * c + d * d;
		if ( l2 > 1e-4 && l2 <= 1.0 )
		{
			double s = 1.0 / sqrt( l2 );
			q[0] = a * s;
			q[1] = b * s;
			q[2] = c * s;
			q[3] = d * s;
			return;
		}
	}
}

#endif
