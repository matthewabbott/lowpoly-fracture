// traj.c: see traj.h. Every scalar goes through bytes in little-endian order, so the files are the same on every machine.
#include "traj.h"

#include <stdlib.h>
#include <string.h>

#define TRAJ_HEADER 32
#define TRAJ_BODY ( 13 * 8 + 2 * 4 )

static void put_u32( uint8_t* b, uint32_t x )
{
	for ( int i = 0; i < 4; ++i )
	{
		b[i] = (uint8_t)( x >> ( 8 * i ) );
	}
}

static void put_f64( uint8_t* b, double x )
{
	uint64_t u;
	memcpy( &u, &x, 8 );
	for ( int i = 0; i < 8; ++i )
	{
		b[i] = (uint8_t)( u >> ( 8 * i ) );
	}
}

static uint32_t get_u32( const uint8_t* b )
{
	uint32_t x = 0;
	for ( int i = 0; i < 4; ++i )
	{
		x |= (uint32_t)b[i] << ( 8 * i );
	}
	return x;
}

static double get_f64( const uint8_t* b )
{
	uint64_t u = 0;
	for ( int i = 0; i < 8; ++i )
	{
		u |= (uint64_t)b[i] << ( 8 * i );
	}
	double x;
	memcpy( &x, &u, 8 );
	return x;
}

int traj_open( TrajWriter* w, const char* path, uint32_t bodyCount, uint32_t every, double dt )
{
	memset( w, 0, sizeof( *w ) );
	w->f = fopen( path, "wb" );
	if ( w->f == NULL )
	{
		return 0;
	}
	w->bodyCount = bodyCount;
	w->every = every;
	uint8_t h[TRAJ_HEADER];
	memset( h, 0, sizeof( h ) );
	memcpy( h, "LPTRAJ1", 7 );
	put_u32( h + 8, bodyCount );
	put_u32( h + 12, 0 );
	put_u32( h + 16, every );
	put_u32( h + 20, 0 );
	put_f64( h + 24, dt );
	return fwrite( h, sizeof( h ), 1, w->f ) == 1;
}

int traj_write( TrajWriter* w, uint32_t tick, const TrajBody* bodies )
{
	if ( w->f == NULL )
	{
		return 0;
	}
	uint8_t r[8];
	put_u32( r, tick );
	put_u32( r + 4, 0 );
	int ok = fwrite( r, sizeof( r ), 1, w->f ) == 1;
	for ( uint32_t i = 0; i < w->bodyCount && ok; ++i )
	{
		const TrajBody* b = bodies + i;
		uint8_t x[TRAJ_BODY];
		const double* d[4] = { b->p, b->q, b->v, b->w };
		const int n[4] = { 3, 4, 3, 3 };
		int o = 0;
		for ( int k = 0; k < 4; ++k )
		{
			for ( int j = 0; j < n[k]; ++j )
			{
				put_f64( x + o, d[k][j] );
				o += 8;
			}
		}
		put_u32( x + o, b->awake );
		put_u32( x + o + 4, b->flags );
		ok = fwrite( x, sizeof( x ), 1, w->f ) == 1;
	}
	w->records += ok ? 1u : 0u;
	return ok;
}

int traj_close( TrajWriter* w )
{
	if ( w->f == NULL )
	{
		return 0;
	}
	uint8_t c[4];
	put_u32( c, w->records );
	int ok = fseek( w->f, 12, SEEK_SET ) == 0 && fwrite( c, sizeof( c ), 1, w->f ) == 1;
	ok = fclose( w->f ) == 0 && ok;
	w->f = NULL;
	return ok;
}

int traj_read( Traj* t, const char* path )
{
	memset( t, 0, sizeof( *t ) );
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		return 0;
	}
	uint8_t h[TRAJ_HEADER];
	int ok = fread( h, sizeof( h ), 1, f ) == 1 && memcmp( h, "LPTRAJ1", 8 ) == 0;
	if ( ok )
	{
		t->bodyCount = get_u32( h + 8 );
		t->recordCount = get_u32( h + 12 );
		t->every = get_u32( h + 16 );
		t->dt = get_f64( h + 24 );
		t->ticks = (uint32_t*)calloc( (size_t)t->recordCount + 1, sizeof( uint32_t ) );
		t->bodies = (TrajBody*)calloc( (size_t)t->recordCount * t->bodyCount + 1, sizeof( TrajBody ) );
	}
	for ( uint32_t r = 0; r < t->recordCount && ok; ++r )
	{
		uint8_t x[TRAJ_BODY];
		ok = fread( x, 8, 1, f ) == 1;
		t->ticks[r] = get_u32( x );
		for ( uint32_t i = 0; i < t->bodyCount && ok; ++i )
		{
			ok = fread( x, sizeof( x ), 1, f ) == 1;
			TrajBody* b = t->bodies + (size_t)r * t->bodyCount + i;
			double* d[4] = { b->p, b->q, b->v, b->w };
			const int n[4] = { 3, 4, 3, 3 };
			int o = 0;
			for ( int k = 0; k < 4; ++k )
			{
				for ( int j = 0; j < n[k]; ++j )
				{
					d[k][j] = get_f64( x + o );
					o += 8;
				}
			}
			b->awake = get_u32( x + o );
			b->flags = get_u32( x + o + 4 );
		}
	}
	fclose( f );
	if ( !ok )
	{
		traj_free( t );
	}
	return ok;
}

void traj_free( Traj* t )
{
	free( t->ticks );
	free( t->bodies );
	memset( t, 0, sizeof( *t ) );
}
