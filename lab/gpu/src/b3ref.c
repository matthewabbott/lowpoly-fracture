// b3ref.c: Box3D's own solver cost at a known contact count (approximate reference for E11).
// Stacks of 1 m boxes, 10 high, on a static ground; sleep off so every contact stays awake; dt 1/60
// with 4 substeps and the default soft contact settings. Reports b3Profile stage times per step.
//
//   b3ref.exe <columns> <workers> [layers]
#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv )
{
	int columns = argc > 1 ? atoi( argv[1] ) : 100;
	int workers = argc > 2 ? atoi( argv[2] ) : 1;
	int layers = argc > 3 ? atoi( argv[3] ) : 10;
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -10.0f, 0.0f };
	wd.enableSleep = false;
	wd.workerCount = (uint32_t)workers;
	b3WorldId world = b3CreateWorld( &wd );

	int side = 1;
	while ( side * side < columns )
		++side;
	float extent = 0.6f * (float)side + 2.0f;
	b3BodyDef gd = b3DefaultBodyDef();
	gd.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
	b3BodyId ground = b3CreateBody( world, &gd );
	b3BoxHull gh = b3MakeBoxHull( extent, 1.0f, extent );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3CreateHullShape( ground, &sd, &gh.base );

	b3BoxHull box = b3MakeCubeHull( 0.5f );
	int made = 0;
	for ( int c = 0; c < columns; ++c )
	{
		float x = 1.2f * (float)( c % side ) - 0.6f * (float)side;
		float z = 1.2f * (float)( c / side ) - 0.6f * (float)side;
		for ( int l = 0; l < layers; ++l )
		{
			b3BodyDef bd = b3DefaultBodyDef();
			bd.type = b3_dynamicBody;
			bd.position = ( b3Pos ){ x, 0.5f + 1.0f * (float)l, z };
			b3BodyId b = b3CreateBody( world, &bd );
			b3CreateHullShape( b, &sd, &box.base );
			++made;
		}
	}

	for ( int i = 0; i < 120; ++i )
		b3World_Step( world, 1.0f / 60.0f, 4 );

	double sum[10] = { 0 };
	int steps = 60;
	int contacts = 0, awake = 0;
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		b3Profile p = b3World_GetProfile( world );
		sum[0] += p.step;
		sum[1] += p.collide;
		sum[2] += p.solve;
		sum[3] += p.prepareConstraints;
		sum[4] += p.integrateVelocities;
		sum[5] += p.warmStart;
		sum[6] += p.solveImpulses;
		sum[7] += p.integratePositions;
		sum[8] += p.relaxImpulses;
		sum[9] += p.storeImpulses + p.restitution;
		b3Counters ct = b3World_GetCounters( world );
		contacts = ct.contactCount;
		awake = ct.awakeContactCount;
	}
	double stages = ( sum[4] + sum[5] + sum[6] + sum[7] + sum[8] ) / steps;
	printf( "box3d ref: %d bodies (%d columns x %d), workers %d, contacts %d (awake %d)\n", made, columns, layers, workers, contacts, awake );
	printf( "  per step ms: step %.3f collide %.3f solve %.3f | prepare %.3f intVel %.3f warm %.3f solve %.3f intPos %.3f relax %.3f store+rest %.3f"
			" | substep stages (intVel+warm+solve+intPos+relax) %.3f\n",
			sum[0] / steps, sum[1] / steps, sum[2] / steps, sum[3] / steps, sum[4] / steps, sum[5] / steps, sum[6] / steps, sum[7] / steps,
			sum[8] / steps, sum[9] / steps, stages );
	b3DestroyWorld( world );
	return 0;
}
