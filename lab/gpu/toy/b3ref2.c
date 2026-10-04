// b3ref2.c: the toy's scenes in Box3D (step 4b's reference, DESIGN.md "Scenes and acceptance"). Builds each scene from the
// toy's generator (scene.c: the same doubles for hull points, poses, velocities, masses, friction and restitution),
// runs it with the toy's settings (60 Hz, 4 substeps, gravity -10, contact hertz 30, damping ratio 10, push speed 3 m/s,
// restitution threshold 1 m/s, two restitution iterations, one worker; sleep and contact recycling as asked; continuous
// collision off unless asked) and writes the toy's trajectory format (traj.h's LPTRAJ1). The scene's joints (step 6's arm)
// are Box3D revolute joints with the same frames, limits and motor; a servo's motor speed is set before each step from
// the joint's angle then, by the toy's rule (gain (target - angle), capped), and each joint's line is printed as the toy
// prints it (metrics.py measures the angles, gaps and overshoots from the trajectory).
//
//   b3ref2 --scene pile200 [--seed S] [--ticks N] [--sleep 0|1] [--recycle 0|1] [--continuous 0|1] [--mass scene|shapes]
//          [--traj FILE [--traj-every N]] [--log FILE] [--quiet]
//
// Hulls: the scene's boxes through b3MakeBoxHull (its half extents), every other hull through b3CreateHull from the
// scene's points (about the centre of mass); the vertex and face counts are checked against the scene's. Mass: Box3D
// computes each body's mass, centre and inertia from its shape at the scene's density, those are compared with the
// scene's (printed), then the scene's values are set explicitly (b3Body_SetMassData, centre at the origin), so both
// programs integrate the same bodies (--mass shapes keeps Box3D's own).
//
// "Awake" in the trajectory and in the solve line means stepped this tick, as in the toy: awake before the step or after
// it (an island that falls asleep at the end of step t was stepped in t; the toy decides sleep at the start of a tick,
// so its body is not stepped in t + 1). Box3D's own reading (every body asleep after step t) is printed beside it. The
// solve line has the toy's format: the deepest manifold point (Box3D's separation from the narrowphase at the start of
// the step) over the run and over the 60 ticks before every body sleeps (or the last 60), among the contacts that have a
// body stepped this tick; the rest line, the deepest point over every touching contact after the last tick (sleeping
// ones too, their manifolds as they fell asleep), as the toy prints it.
#include "box3d/box3d.h"
#include "scene.h"
#include "traj.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static FILE* g_log;
static int g_quiet;

static void say( const char* fmt, ... )
{
	va_list a;
	if ( !g_quiet )
	{
		va_start( a, fmt );
		vprintf( fmt, a );
		va_end( a );
	}
	if ( g_log )
	{
		va_start( a, fmt );
		vfprintf( g_log, fmt, a );
		va_end( a );
	}
}

// A scene hull is a box when every face normal is a coordinate axis (scene_box_hull's planes)
static int is_box( const SceneHull* h )
{
	if ( h->vertexCount != 8 || h->faceCount != 6 )
	{
		return 0;
	}
	for ( int f = 0; f < h->faceCount; ++f )
	{
		int axis = 0;
		for ( int i = 0; i < 3; ++i )
		{
			axis += fabs( h->normal[f][i] ) == 1.0;
		}
		if ( axis != 1 )
		{
			return 0;
		}
	}
	return 1;
}

typedef struct ContactScan
{
	const uint8_t* stepped; // per body: stepped this tick (dynamic bodies only)
	double minSep;
	int minA, minB;
	int touching, points;
} ContactScan;

static void scan_contact( void* shapeUserDataA, void* shapeUserDataB, void* bodyUserDataA, void* bodyUserDataB, const b3ContactState* state,
						  void* context )
{
	(void)shapeUserDataA;
	(void)shapeUserDataB;
	ContactScan* c = (ContactScan*)context;
	int a = (int)(intptr_t)bodyUserDataA - 1;
	int b = (int)(intptr_t)bodyUserDataB - 1;
	int live = c->stepped == NULL || ( a >= 0 && c->stepped[a] ) || ( b >= 0 && c->stepped[b] );
	if ( !live )
	{
		return;
	}
	c->touching += 1;
	for ( int m = 0; m < state->manifoldCount; ++m )
	{
		const b3Manifold* mf = state->manifolds + m;
		c->points += mf->pointCount;
		for ( int k = 0; k < mf->pointCount; ++k )
		{
			double s = (double)mf->points[k].separation;
			if ( s < c->minSep )
			{
				c->minSep = s;
				c->minA = a < b ? a : b;
				c->minB = a < b ? b : a;
			}
		}
	}
}

typedef struct TickInfo
{
	int stepped;	 // dynamic bodies stepped this tick
	int awakeAfter; // dynamic bodies awake after the step (Box3D's own reading)
	int touching, points;
	double minSep;
	int minA, minB;
} TickInfo;

int main( int argc, char** argv )
{
	const char* sceneName = "stack10";
	uint64_t seed = 1;
	int ticks = 600;
	int sleep = 1, recycle = 1, continuous = 0, sceneMass = 1;
	const char* trajPath = NULL;
	int trajEvery = 1;
	const char* logPath = NULL;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--scene" ) == 0 )
			sceneName = v, ++i;
		else if ( strcmp( a, "--seed" ) == 0 )
			seed = (uint64_t)strtoull( v, NULL, 10 ), ++i;
		else if ( strcmp( a, "--ticks" ) == 0 )
			ticks = atoi( v ), ++i;
		else if ( strcmp( a, "--sleep" ) == 0 )
			sleep = atoi( v ), ++i;
		else if ( strcmp( a, "--recycle" ) == 0 )
			recycle = atoi( v ), ++i;
		else if ( strcmp( a, "--continuous" ) == 0 )
			continuous = atoi( v ), ++i;
		else if ( strcmp( a, "--mass" ) == 0 )
			sceneMass = strcmp( v, "shapes" ) != 0, ++i;
		else if ( strcmp( a, "--traj" ) == 0 )
			trajPath = v, ++i;
		else if ( strcmp( a, "--traj-every" ) == 0 )
			trajEvery = atoi( v ) < 1 ? 1 : atoi( v ), ++i;
		else if ( strcmp( a, "--log" ) == 0 )
			logPath = v, ++i;
		else if ( strcmp( a, "--quiet" ) == 0 )
			g_quiet = 1;
		else
		{
			fprintf( stderr, "unknown option %s\n", a );
			return 1;
		}
	}
	if ( logPath )
	{
		g_log = fopen( logPath, "w" );
	}
	if ( ticks < 1 )
	{
		ticks = 1;
	}

	Scene sc;
	if ( !scene_build( &sc, sceneName, seed ) )
	{
		fprintf( stderr, "unknown scene %s (%s)\n", sceneName, scene_names() );
		return 1;
	}
	const int n = sc.bodyCount;

	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -10.0f, 0.0f };
	wd.contactHertz = 30.0f;
	wd.contactDampingRatio = 10.0f;
	wd.contactSpeed = 3.0f;
	wd.restitutionThreshold = 1.0f;
	wd.restitutionIterations = 2;
	wd.maximumLinearSpeed = 400.0f;
	wd.enableSleep = sleep != 0;
	wd.enableContinuous = continuous != 0;
	wd.workerCount = 1;
	b3WorldId world = b3CreateWorld( &wd );
	if ( !recycle )
	{
		b3World_SetContactRecycleDistance( world, 0.0f );
	}
	say( "b3ref2 %s (seed %llu), %d ticks: Box3D, 60 Hz, 4 substeps, gravity -10, hertz %g, damping %g, push %g m/s, restitution threshold %g m/s, "
		 "sleep %s, contact recycling %s (%g m), continuous %s, 1 worker, mass from %s\n",
		 sceneName, (unsigned long long)seed, ticks, wd.contactHertz, wd.contactDampingRatio, wd.contactSpeed, wd.restitutionThreshold,
		 sleep ? "on" : "off", recycle ? "on" : "off", b3World_GetContactRecycleDistance( world ), continuous ? "on" : "off",
		 sceneMass ? "the scene (b3Body_SetMassData)" : "Box3D's shapes" );

	// hulls: boxes from their half extents, the rest from the scene's points
	b3BoxHull* boxes = (b3BoxHull*)calloc( (size_t)sc.hullCount, sizeof( b3BoxHull ) );
	b3HullData** made = (b3HullData**)calloc( (size_t)sc.hullCount, sizeof( b3HullData* ) );
	const b3HullData** hulls = (const b3HullData**)calloc( (size_t)sc.hullCount, sizeof( b3HullData* ) );
	int boxCount = 0, builtCount = 0, countDiffer = 0;
	double worstCenter = 0.0, worstVolume = 0.0;
	for ( int h = 0; h < sc.hullCount; ++h )
	{
		const SceneHull* H = sc.hulls + h;
		if ( is_box( H ) )
		{
			boxes[h] = b3MakeBoxHull( (float)H->boundsHalf[0], (float)H->boundsHalf[1], (float)H->boundsHalf[2] );
			hulls[h] = &boxes[h].base;
			boxCount += 1;
		}
		else
		{
			b3Vec3 pts[SCENE_MAX_VERTS];
			for ( int k = 0; k < H->vertexCount; ++k )
			{
				pts[k] = ( b3Vec3 ){ (float)H->v[k][0], (float)H->v[k][1], (float)H->v[k][2] };
			}
			made[h] = b3CreateHull( pts, H->vertexCount, B3_MAX_HULL_VERTICES );
			if ( made[h] == NULL )
			{
				fprintf( stderr, "hull %d: b3CreateHull failed\n", h );
				return 1;
			}
			hulls[h] = made[h];
			builtCount += 1;
		}
		const b3HullData* B = hulls[h];
		if ( B->vertexCount != H->vertexCount || B->faceCount != H->faceCount || B->edgeCount != H->edgeCount )
		{
			countDiffer += 1;
			say( "  hull %d: Box3D %d vertices %d faces %d half-edges, the scene %d %d %d\n", h, B->vertexCount, B->faceCount, B->edgeCount,
				 H->vertexCount, H->faceCount, H->edgeCount );
		}
		double c = sqrt( (double)B->center.x * B->center.x + (double)B->center.y * B->center.y + (double)B->center.z * B->center.z );
		worstCenter = c > worstCenter ? c : worstCenter;
		double dv = fabs( (double)B->volume - H->volume ) / H->volume;
		worstVolume = dv > worstVolume ? dv : worstVolume;
	}
	say( "hulls: %d (%d boxes through b3MakeBoxHull, %d through b3CreateHull), vertex/face/edge counts differ on %d; Box3D's centroid "
		 "at most %.3g m from the scene's centre of mass, volume within %.3g (relative)\n",
		 sc.hullCount, boxCount, builtCount, countDiffer, worstCenter, worstVolume );

	b3BodyId* ids = (b3BodyId*)calloc( (size_t)n, sizeof( b3BodyId ) );
	double worstMass = 0.0, worstInertia = 0.0, worstMassCenter = 0.0;
	int worstMassBody = -1, worstInertiaBody = -1;
	for ( int i = 0; i < n; ++i )
	{
		const SceneBody* b = sc.bodies + i;
		const SceneHull* H = sc.hulls + b->hull;
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b->isStatic ? b3_staticBody : b3_dynamicBody;
		bd.position = ( b3Pos ){ (float)b->p[0], (float)b->p[1], (float)b->p[2] };
		bd.rotation.v = ( b3Vec3 ){ (float)b->q[0], (float)b->q[1], (float)b->q[2] };
		bd.rotation.s = (float)b->q[3];
		bd.linearVelocity = ( b3Vec3 ){ (float)b->v[0], (float)b->v[1], (float)b->v[2] };
		bd.angularVelocity = ( b3Vec3 ){ (float)b->w[0], (float)b->w[1], (float)b->w[2] };
		bd.userData = (void*)(intptr_t)( i + 1 );
		ids[i] = b3CreateBody( world, &bd );
		b3ShapeDef sd = b3DefaultShapeDef();
		double density = b->isStatic ? 0.0 : b->mass / H->volume;
		sd.density = (float)density;
		sd.baseMaterial.friction = (float)b->friction;
		sd.baseMaterial.restitution = (float)b->restitution;
		b3CreateHullShape( ids[i], &sd, hulls[b->hull] );
		if ( b->isStatic )
		{
			continue;
		}
		// Box3D's mass from the shape against the scene's
		b3MassData md = b3Body_GetMassData( ids[i] );
		double dm = fabs( (double)md.mass - b->mass ) / b->mass;
		if ( dm > worstMass )
		{
			worstMass = dm;
			worstMassBody = i;
		}
		double cc = sqrt( (double)md.center.x * md.center.x + (double)md.center.y * md.center.y + (double)md.center.z * md.center.z );
		worstMassCenter = cc > worstMassCenter ? cc : worstMassCenter;
		double I[6];
		for ( int k = 0; k < 6; ++k )
		{
			I[k] = density * H->inertia[k];
		}
		double got[6] = { md.inertia.cx.x, md.inertia.cy.x, md.inertia.cz.x, md.inertia.cy.y, md.inertia.cz.y, md.inertia.cz.z };
		double num = 0.0, den = 0.0;
		for ( int k = 0; k < 6; ++k )
		{
			double w = ( k == 1 || k == 2 || k == 4 ) ? 2.0 : 1.0; // off-diagonals twice (the full matrix's Frobenius norm)
			num += w * ( got[k] - I[k] ) * ( got[k] - I[k] );
			den += w * I[k] * I[k];
		}
		double di = sqrt( num / den );
		if ( di > worstInertia )
		{
			worstInertia = di;
			worstInertiaBody = i;
		}
		if ( sceneMass )
		{
			b3MassData m;
			m.mass = (float)b->mass;
			m.center = ( b3Vec3 ){ 0.0f, 0.0f, 0.0f };
			m.inertia.cx = ( b3Vec3 ){ (float)I[0], (float)I[1], (float)I[2] };
			m.inertia.cy = ( b3Vec3 ){ (float)I[1], (float)I[3], (float)I[4] };
			m.inertia.cz = ( b3Vec3 ){ (float)I[2], (float)I[4], (float)I[5] };
			b3Body_SetMassData( ids[i], m );
		}
	}
	say( "mass from Box3D's shapes against the scene's: mass within %.3g (relative, body %d), centre within %.3g m, inertia within %.3g "
		 "(relative Frobenius, body %d)%s\n",
		 worstMass, worstMassBody, worstMassCenter, worstInertia, worstInertiaBody,
		 sceneMass ? "; the scene's values set" : "; Box3D's kept" );
	for ( int h = 0; h < sc.hullCount; ++h )
	{
		if ( made[h] )
		{
			b3DestroyHull( made[h] ); // the world keeps its own copy
		}
	}

	// the joints (step 6): revolute joints with the scene's frames (from each body's origin, its centre of mass here),
	// limits and motor; Box3D's defaults otherwise (constraint hertz 60, damping ratio 2, collideConnected false). A servo's
	// motor speed is set before every step from the angle then (b3RevoluteJoint_GetAngle) by the toy's rule.
	b3JointId* jids = (b3JointId*)calloc( (size_t)( sc.jointCount > 0 ? sc.jointCount : 1 ), sizeof( b3JointId ) );
	for ( int k = 0; k < sc.jointCount; ++k )
	{
		const SceneJoint* j = sc.joints + k;
		b3RevoluteJointDef jd = b3DefaultRevoluteJointDef();
		jd.base.bodyIdA = ids[j->bodyA];
		jd.base.bodyIdB = ids[j->bodyB];
		jd.base.localFrameA.p = ( b3Vec3 ){ (float)j->localAnchorA[0], (float)j->localAnchorA[1], (float)j->localAnchorA[2] };
		jd.base.localFrameA.q.v = ( b3Vec3 ){ (float)j->localFrameA[0], (float)j->localFrameA[1], (float)j->localFrameA[2] };
		jd.base.localFrameA.q.s = (float)j->localFrameA[3];
		jd.base.localFrameB.p = ( b3Vec3 ){ (float)j->localAnchorB[0], (float)j->localAnchorB[1], (float)j->localAnchorB[2] };
		jd.base.localFrameB.q.v = ( b3Vec3 ){ (float)j->localFrameB[0], (float)j->localFrameB[1], (float)j->localFrameB[2] };
		jd.base.localFrameB.q.s = (float)j->localFrameB[3];
		jd.enableLimit = j->enableLimit != 0;
		jd.lowerAngle = (float)j->lowerAngle;
		jd.upperAngle = (float)j->upperAngle;
		jd.enableMotor = j->enableMotor != 0 || j->servo != 0;
		jd.maxMotorTorque = (float)j->maxMotorTorque;
		jd.motorSpeed = (float)j->motorSpeed;
		jids[k] = b3CreateRevoluteJoint( world, &jd );
		char line[640];
		scene_joint_text( j, k, line, (int)sizeof( line ) );
		say( "%s\n", line );
	}

	TrajWriter traj;
	int trajOk = trajPath ? traj_open( &traj, trajPath, (uint32_t)n, (uint32_t)trajEvery, 1.0 / 60.0 ) : 0;
	if ( trajPath && !trajOk )
	{
		say( "traj %s: cannot write\n", trajPath );
	}
	TrajBody* tb = (TrajBody*)calloc( (size_t)n, sizeof( TrajBody ) );
	uint8_t* stepped = (uint8_t*)calloc( (size_t)n, 1 );
	TickInfo* info = (TickInfo*)calloc( (size_t)ticks + 1, sizeof( TickInfo ) );
	for ( int t = 0; t <= ticks; ++t )
	{
		if ( t > 0 )
		{
			for ( int i = 0; i < n; ++i )
			{
				stepped[i] = sc.bodies[i].isStatic ? 0 : (uint8_t)b3Body_IsAwake( ids[i] );
			}
			for ( int k = 0; k < sc.jointCount; ++k ) // the servos: gain (target - angle), capped, in float (the toy's F rule)
			{
				const SceneJoint* j = sc.joints + k;
				if ( j->servo )
				{
					float angle = b3RevoluteJoint_GetAngle( jids[k] );
					float err = (float)scene_servo_target( j, t ) - angle;
					float speed = (float)j->servoGain * err;
					float cap = (float)j->servoMaxSpeed;
					speed = speed < -cap ? -cap : ( speed > cap ? cap : speed );
					b3RevoluteJoint_SetMotorSpeed( jids[k], speed );
				}
			}
			b3World_Step( world, 1.0f / 60.0f, 4 );
			TickInfo* k = info + t;
			for ( int i = 0; i < n; ++i )
			{
				if ( sc.bodies[i].isStatic )
				{
					continue;
				}
				int after = b3Body_IsAwake( ids[i] ) ? 1 : 0;
				k->awakeAfter += after;
				stepped[i] = (uint8_t)( stepped[i] | after );
				k->stepped += stepped[i];
			}
			ContactScan cs = { stepped, 1e30, -1, -1, 0, 0 };
			b3World_VisitContactState( world, false, scan_contact, &cs );
			k->minSep = cs.minSep;
			k->minA = cs.minA;
			k->minB = cs.minB;
			k->touching = cs.touching;
			k->points = cs.points;
		}
		else
		{
			for ( int i = 0; i < n; ++i )
			{
				stepped[i] = sc.bodies[i].isStatic ? 0 : 1;
			}
		}
		if ( trajOk && t % trajEvery == 0 )
		{
			for ( int i = 0; i < n; ++i )
			{
				b3Pos p = b3Body_GetPosition( ids[i] );
				b3Quat q = b3Body_GetRotation( ids[i] );
				b3Vec3 v = b3Body_GetLinearVelocity( ids[i] );
				b3Vec3 w = b3Body_GetAngularVelocity( ids[i] );
				TrajBody* r = tb + i;
				r->p[0] = p.x;
				r->p[1] = p.y;
				r->p[2] = p.z;
				r->q[0] = q.v.x;
				r->q[1] = q.v.y;
				r->q[2] = q.v.z;
				r->q[3] = q.s;
				r->v[0] = v.x;
				r->v[1] = v.y;
				r->v[2] = v.z;
				r->w[0] = w.x;
				r->w[1] = w.y;
				r->w[2] = w.z;
				r->awake = stepped[i];
				r->flags = sc.bodies[i].isStatic ? TRAJ_STATIC : 0u;
			}
			traj_write( &traj, (uint32_t)t, tb );
		}
	}
	if ( trajOk )
	{
		uint32_t records = traj.records;
		trajOk = traj_close( &traj );
		say( "traj %s: %u records of %d bodies, every %d ticks%s\n", trajPath, records, n, trajEvery, trajOk ? "" : " (WRITE FAILED)" );
	}

	// per tick, the toy's columns where Box3D has them
	say( "tick  stepped awake-after touching points deepest\n" );
	for ( int t = 1; t <= ticks; ++t )
	{
		if ( t <= 120 || t % 60 == 0 || t == ticks )
		{
			const TickInfo* k = info + t;
			say( "%4d  %7d %11d %8d %6d %.6g\n", t, k->stepped, k->awakeAfter, k->touching, k->points, k->minSep < 1e29 ? k->minSep : 0.0 );
		}
	}

	// the solve line, in the toy's format and its reading of awake (stepped this tick)
	int asleepFrom = -1, box3dAsleepFrom = -1;
	for ( int t = ticks; t >= 1 && info[t].stepped == 0; --t )
	{
		asleepFrom = t;
	}
	for ( int t = ticks; t >= 1 && info[t].awakeAfter == 0; --t )
	{
		box3dAsleepFrom = t;
	}
	int restEnd = asleepFrom > 0 ? asleepFrom - 1 : ticks;
	double deep = 1e30, deepLate = 1e30;
	int deepAt = 0, deepA = -1, deepB = -1, lateAt = 0, lateA = -1, lateB = -1;
	for ( int t = 1; t <= ticks; ++t )
	{
		const TickInfo* k = info + t;
		if ( k->minSep < deep )
		{
			deep = k->minSep;
			deepAt = t;
			deepA = k->minA;
			deepB = k->minB;
		}
		if ( t > restEnd - 60 && t <= restEnd && k->minSep < deepLate )
		{
			deepLate = k->minSep;
			lateAt = t;
			lateA = k->minA;
			lateB = k->minB;
		}
	}
	if ( asleepFrom > 0 )
	{
		say( "solve: every dynamic body asleep from tick %d", asleepFrom );
	}
	else
	{
		say( "solve: %d bodies awake at the end", info[ticks].stepped );
	}
	say( "; deepest point %.6g m (tick %d, bodies %d %d), at rest (the 60 ticks before every body sleeps, or the last 60) %.6g m (tick %d, "
		 "bodies %d %d); Box3D\n",
		 deep < 1e29 ? deep : 0.0, deepAt, deepA, deepB, deepLate < 1e29 ? deepLate : 0.0, lateAt, lateA, lateB );
	{
		ContactScan all = { NULL, 1e30, -1, -1, 0, 0 };
		b3World_VisitContactState( world, false, scan_contact, &all );
		say( "rest: after tick %d, over every touching pair (sleeping ones too): deepest point %.6g m (bodies %d %d), %d touching pairs\n", ticks,
			 all.minSep < 1e29 ? all.minSep : 0.0, all.minA, all.minB, all.touching );
	}
	if ( box3dAsleepFrom > 0 )
	{
		say( "box3d: every dynamic body asleep after step %d (Box3D's reading: b3Body_IsAwake after the step)\n", box3dAsleepFrom );
	}
	else
	{
		say( "box3d: %d bodies awake after the last step\n", info[ticks].awakeAfter );
	}
	b3Counters ct = b3World_GetCounters( world );
	say( "b3ref2 %s: done, %d bodies, %d contacts (awake %d) at the end\n", sceneName, n, ct.contactCount, ct.awakeContactCount );

	free( tb );
	free( stepped );
	free( info );
	free( ids );
	free( jids );
	free( boxes );
	free( made );
	free( hulls );
	b3DestroyWorld( world );
	scene_free( &sc );
	if ( g_log )
	{
		fclose( g_log );
	}
	return 0;
}
