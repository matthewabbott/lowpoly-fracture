// traj.h: trajectory files (LPTRAJ1) for step 4b's metrics and the Box3D reference program: a writer and a reader in
// dialect-free C17 (the caller converts its values to double exactly), so the toy and b3ref2 write the same format.
//
// Little-endian whatever the machine: a header char magic[8] = "LPTRAJ1" (NUL-padded), uint32 bodyCount, uint32
// recordCount (patched when the file closes), uint32 every, uint32 reserved, double dt; then per record uint32 tick,
// uint32 reserved, and per body in scene order (static bodies included) double px, py, pz, qx, qy, qz, qw, vx, vy, vz,
// wx, wy, wz, uint32 awake, uint32 flags (bit 0: static).
#ifndef TOY_TRAJ_H
#define TOY_TRAJ_H

#include <stdint.h>
#include <stdio.h>

#define TRAJ_STATIC 1u

typedef struct TrajBody
{
	double p[3];
	double q[4]; // x, y, z, w
	double v[3];
	double w[3];
	uint32_t awake;
	uint32_t flags; // TRAJ_STATIC
} TrajBody;

typedef struct TrajWriter
{
	FILE* f;
	uint32_t bodyCount;
	uint32_t records;
	uint32_t every;
} TrajWriter;

// 1 on success. traj_write takes bodyCount bodies; traj_close patches the record count and closes the file.
int traj_open( TrajWriter* w, const char* path, uint32_t bodyCount, uint32_t every, double dt );
int traj_write( TrajWriter* w, uint32_t tick, const TrajBody* bodies );
int traj_close( TrajWriter* w );

// A whole file in memory: ticks[r] and bodies[r * bodyCount + i]
typedef struct Traj
{
	uint32_t bodyCount, recordCount, every;
	double dt;
	uint32_t* ticks;
	TrajBody* bodies;
} Traj;

// 1 on success (0: missing, not LPTRAJ1, or short)
int traj_read( Traj* t, const char* path );
void traj_free( Traj* t );

#endif
