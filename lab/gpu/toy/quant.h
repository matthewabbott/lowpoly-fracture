// quant.h: a scene (double) quantised into one dialect's buffers (DESIGN.md "Layouts"): exact-rounded from the doubles
// into F (float), V4 (the formats of layout.h, round half up, with per-body mass and inertia exponents) and D (double).
// Face planes are recomputed from the quantised points. Compiled once per dialect.
#ifndef TOY_QUANT_H
#define TOY_QUANT_H

#include "layout.h"
#include "scene.h"

typedef struct ToyData
{
	int bodyCount, hullCount, pointCount, faceCount, edgeCount;
	Hull* hulls;
	V3* points;
	HullFace* faces;
	uint32_t* edges;
	BodyState* state;
	BodyPose* pose;
	BodyMass* mass;
	Params params;
	int rangeErrors; // values that did not fit their format (V4)
} ToyData;

typedef struct ToySettings
{
	double gravity;		 // m/s^2 along -y (10)
	double aabbMargin;	 // m (Box3D's 0.1)
	double linearSlop;	 // m (0.005)
	double sleepSpeed;	 // m/s (Box3D's sleep threshold 0.05)
	int sleepTicks;		 // 30 (0.5 s at 60 Hz)
	int enableSleep;
	double maxLinearSpeed;	 // m/s (Box3D's 4 m * 100)
	double maxRotationPerStep; // rad per step (Box3D's 0.25 pi)
} ToySettings;

void toy_default_settings( ToySettings* s );
void toy_quantize( const Scene* sc, const ToySettings* st, ToyData* d );
void toy_free_data( ToyData* d );

// A position component and a scalar of format s back in double (exact)
double toy_pos_x( const Pos3* p );
double toy_pos_y( const Pos3* p );
double toy_pos_z( const Pos3* p );
double toy_val( T x, int s );

#endif
