// SPDX-License-Identifier: MIT
// Gait: which feet step, where they land, and how the torso moves over them (rig.c has the limbs and their IK).
//
// A free gait, with no pattern tables. Each step, a planted foot steps once it has drifted behind its neutral point
// (where it rests under the torso) by most of its half-step, the most urgent first, if its nearest able neighbours around
// the body are down and the centre of mass stays inside the other planted feet by the margin, now and at the end of the
// swing. Feet that swing together are every other able leg around the body: on six legs the two tripods, and after
// damage whatever pattern the able legs allow, with nothing to replan. A foot that waits for balance leans the body
// toward the feet that hold it; a foot stretched past its stride slows the torso to a stop, so a mech that cannot step
// does not walk off its feet. Standing still, feet step back to their neutral points one at a time.
//
// A step's half-length is half the ground the torso covers in a swing (up to most of the stride), so the tripods take
// turns of the same length, and the controls are capped at the speed that cadence keeps up. A swing lands that far
// ahead of neutral, re-aimed across the ground every step from how the torso actually moves; the ground's height comes
// from a cast at liftoff and another two thirds through (a sole-sized sphere down from above the foothold, through the
// rig's own bodies, against static and full pieces). The foot lifts, crosses and comes down on a polynomial arc, and
// plants when its time is up.
//
// A planted foot is held where the model has it once it gets there (the model's feet, from the measured angles: a
// loaded joint gives a centimetre, and aiming at the bodies' feet would push them about), fixed in the world so the legs
// push the torso along, and easing over 0.3 s toward the legs' geometry so feet held a little apart from it do not push
// against each other through the ground. The torso is pushed toward a desired pose: moved and turned by the controls a
// step at a time, a little ahead of the torso at most (further behind, so a torso that runs on is held back), level, at
// its height over the feet. Each limb's targets are the IK of its foot (planted: its hold; swinging: on its arc) from
// that pose; its feedforward is the joint speeds the commanded motion and the arc ask for. Everything is in limb order
// with fixed counts.

#include "world.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define LP_GAIT_DUE 0.6f		// moving, a planted foot steps once it has drifted this share of its half-step behind neutral (and
								// its neighbours are down: a tripod lifts as the other lands, before it is left carrying nothing)
#define LP_GAIT_TIDY 0.2f		// standing still, feet further than this share step back to neutral, one at a time
#define LP_GAIT_LAND 0.7f		// a swing lands at most this share of the stride ahead of neutral
#define LP_GAIT_KEEP_UP 0.95f	// share of the cadence's top speed the controls may ask for
#define LP_GAIT_OVERREACH 0.35f // past its stride by this share, a planted foot stops the torso
#define LP_GAIT_SHIFT 0.5f		// m/s the body leans toward the feet that hold it while a foot waits for balance
#define LP_GAIT_CLEARANCE 0.8f	// m above the foothold the cast starts
#define LP_GAIT_DEPTH 1.2f		// m below it the cast looks
#define LP_GAIT_SOLE 0.1f		// the cast sphere's radius
#define LP_GAIT_LEAD 0.05f		// m the desired pose may run ahead of the torso (the servos' gains pull it along)
#define LP_GAIT_LAG 0.3f		// m it may fall behind (or away sideways): a torso that runs on is held back
#define LP_GAIT_LEAD_TURN 0.3f	// rad its heading may (it keeps the heading it was given, not the torso's)
#define LP_GAIT_CLIMB 0.5f		// m/s the desired height moves at
#define LP_GAIT_CALM_TICKS 30
#define LP_GAIT_CALM_HEIGHT 0.02f
#define LP_GAIT_CALM_TILT 0.0175f // rad
#define LP_GAIT_RECHECK_TICKS 30  // a structure a foot lands on or leaves is re-checked at most this often
#define LP_GAIT_ARRIVED 0.03f	  // m: a planted foot this near where it was set down has got there
#define LP_GAIT_ARRIVE_TIME 0.3f  // s it may take
#define LP_GAIT_MIN_STANCE 0.15f  // s a foot set down stays down
#define LP_GAIT_SLIPPED 0.1f	  // m: a held foot this far from its hold slipped or was knocked
#define LP_GAIT_EASE_TIME 0.3f	  // s a held foot's hold takes to ease toward where the legs' geometry has it

static bool lpControlStill( const lpRigControl* c )
{
	return c->forward == 0.0f && c->strafe == 0.0f && c->turn == 0.0f;
}

// Level, with the rig's forward along `heading` (horizontal) and its up along worldUp
static b3Quat lpLevelRotation( const lpRig* r, b3Vec3 heading, b3Vec3 worldUp )
{
	b3Matrix3 local = { b3Cross( r->up, r->forward ), r->up, r->forward };
	b3Matrix3 world = { b3Cross( worldUp, heading ), worldUp, heading };
	return b3MulQuat( b3MakeQuatFromMatrix( &world ), b3Conjugate( b3MakeQuatFromMatrix( &local ) ) );
}

static b3Vec3 lpFlatten( b3Vec3 v, b3Vec3 up )
{
	return b3Sub( v, b3MulSV( b3Dot( v, up ), up ) );
}

// ---- balance ----

// The rig's centre of mass: its torso and the bodies of its limbs' chains
static b3Pos lpRigCenter( const lpWorld* w, const lpRig* r )
{
	int bodies[1 + LP_MAX_RIG_LIMBS * LP_MAX_LIMB_JOINTS];
	int count = 0;
	bodies[count++] = r->body;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		const lpLimb* limb = r->limbs + i;
		for ( int k = 0; k < limb->joints && limb->attached; ++k )
		{
			const lpLink* l = w->links.data + limb->def.links[k];
			int body = w->pieces.data[l->ends[1 - limb->prox[k]].piece].body;
			bool seen = false;
			for ( int n = 0; n < count; ++n )
			{
				seen = seen || bodies[n] == body;
			}
			if ( seen == false )
			{
				bodies[count++] = body;
			}
		}
	}
	b3Pos origin = b3Body_GetWorldCenter( w->bodies.data[r->body].id );
	b3Vec3 sum = b3Vec3_zero;
	float mass = 0.0f;
	for ( int n = 0; n < count; ++n )
	{
		b3BodyId id = w->bodies.data[bodies[n]].id;
		float m = b3Body_GetMass( id );
		sum = b3MulAdd( sum, m, b3SubPos( b3Body_GetWorldCenter( id ), origin ) );
		mass += m;
	}
	return mass > 0.0f ? b3OffsetPos( origin, b3MulSV( 1.0f / mass, sum ) ) : origin;
}

// How far the point lies inside the convex hull of the feet, seen along up (negative outside; -FLT_MAX with fewer than
// three feet). Feet are 2D in the plane across up; the hull is a monotone chain over a total order.
static float lpSupportMargin( const b3Pos* feet, const bool* use, int count, b3Pos point, b3Vec3 up )
{
	b3Vec3 e1 = b3Abs( up ).x < 0.9f ? b3Normalize( b3Cross( up, (b3Vec3){ 1.0f, 0.0f, 0.0f } ) )
									 : b3Normalize( b3Cross( up, (b3Vec3){ 0.0f, 0.0f, 1.0f } ) );
	b3Vec3 e2 = b3Cross( up, e1 );
	float px[LP_MAX_RIG_LIMBS], py[LP_MAX_RIG_LIMBS];
	int order[LP_MAX_RIG_LIMBS];
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		if ( use[i] )
		{
			b3Vec3 d = b3SubPos( feet[i], point );
			px[n] = b3Dot( d, e1 );
			py[n] = b3Dot( d, e2 );
			order[n] = n;
			n += 1;
		}
	}
	if ( n < 3 )
	{
		return -FLT_MAX;
	}
	// Insertion sort by x, then y, then index
	for ( int i = 1; i < n; ++i )
	{
		int k = order[i];
		int j = i - 1;
		while ( j >= 0 && ( px[order[j]] > px[k] || ( px[order[j]] == px[k] && ( py[order[j]] > py[k] || ( py[order[j]] == py[k] && order[j] > k ) ) ) ) )
		{
			order[j + 1] = order[j];
			j -= 1;
		}
		order[j + 1] = k;
	}
	int hull[2 * LP_MAX_RIG_LIMBS];
	int h = 0;
	for ( int pass = 0; pass < 2; ++pass )
	{
		int start = h;
		for ( int s = 0; s < n; ++s )
		{
			int k = order[pass == 0 ? s : n - 1 - s];
			while ( h >= start + 2 )
			{
				int a = hull[h - 2], b = hull[h - 1];
				float cross = ( px[b] - px[a] ) * ( py[k] - py[a] ) - ( py[b] - py[a] ) * ( px[k] - px[a] );
				if ( cross > 0.0f )
				{
					break;
				}
				h -= 1;
			}
			hull[h++] = k;
		}
		h -= 1; // the last point of each pass starts the other
	}
	if ( h < 3 )
	{
		return -FLT_MAX;
	}
	// Counterclockwise: the point (the origin) is inside by its distance to the nearest edge
	float margin = FLT_MAX;
	for ( int e = 0; e < h; ++e )
	{
		int a = hull[e], b = hull[( e + 1 ) % h];
		float ex = px[b] - px[a], ey = py[b] - py[a];
		float length = sqrtf( ex * ex + ey * ey );
		if ( length > 1e-6f )
		{
			margin = b3MinFloat( margin, ( ex * ( -py[a] ) - ey * ( -px[a] ) ) / length );
		}
	}
	return margin;
}

// ---- footholds ----

typedef struct lpFootCast
{
	const lpWorld* world;
	const int* skip; // the rig's bodies
	int skipCount;
	float fraction;
	b3Pos point;
	int piece;
	bool hit;
} lpFootCast;

static float lpFootCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
							int triangleIndex, int childIndex, void* context )
{
	(void)normal;
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	lpFootCast* cast = context;
	intptr_t data = (intptr_t)b3Shape_GetUserData( shapeId );
	int piece = data > 0 ? (int)( data - 1 ) : -1;
	if ( piece >= 0 )
	{
		int body = cast->world->pieces.data[piece].body;
		for ( int n = 0; n < cast->skipCount; ++n )
		{
			if ( cast->skip[n] == body )
			{
				return -1.0f;
			}
		}
	}
	if ( fraction < cast->fraction )
	{
		cast->fraction = fraction;
		cast->point = point;
		cast->piece = piece;
		cast->hit = true;
	}
	return fraction;
}

// The ground under a planned foothold: its landing height is where a sole-sized sphere comes to rest on it
static void lpCastFoothold( lpWorld* w, const lpRig* r, lpLimb* limb, b3Vec3 up )
{
	int skip[1 + LP_MAX_RIG_LIMBS * LP_MAX_LIMB_JOINTS];
	int count = 0;
	skip[count++] = r->body;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		const lpLimb* other = r->limbs + i;
		for ( int k = 0; k < other->joints; ++k )
		{
			const lpLink* l = w->links.data + other->def.links[k];
			skip[count++] = w->pieces.data[l->ends[1 - other->prox[k]].piece].body;
		}
	}
	b3Vec3 center = b3Vec3_zero;
	b3ShapeProxy proxy = { &center, 1, LP_GAIT_SOLE };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = LP_CAT_VEHICLE;
	filter.maskBits = LP_CAT_STATIC | LP_CAT_FULL;
	b3Pos from = b3OffsetPos( limb->landing, b3MulSV( LP_GAIT_CLEARANCE + LP_GAIT_SOLE, up ) );
	lpFootCast cast = { w, skip, count, FLT_MAX, { 0 }, -1, false };
	b3World_CastShape( w->def.physics, from, &proxy, b3MulSV( -( LP_GAIT_CLEARANCE + LP_GAIT_DEPTH ), up ), filter,
					   lpFootCastFcn, &cast );
	w->stats.footCasts += 1;
	limb->grounded = cast.hit;
	limb->groundPiece = cast.hit ? cast.piece : -1;
	limb->groundGeneration = cast.piece >= 0 ? w->pieces.data[cast.piece].generation : 0;
	float drop = cast.hit ? cast.fraction * ( LP_GAIT_CLEARANCE + LP_GAIT_DEPTH ) : LP_GAIT_CLEARANCE + 0.3f;
	limb->landing = b3OffsetPos( from, b3MulSV( -( drop + LP_GAIT_SOLE ), up ) );
}

// A structure a foot lands on or leaves carries a changed load: check it again (at most every 30 steps per foot)
static void lpFootMoved( lpWorld* w, lpLimb* limb )
{
	int piece = limb->groundPiece;
	if ( piece < 0 || w->pieces.data[piece].generation != limb->groundGeneration || w->pieces.data[piece].body < 0 )
	{
		return;
	}
	int body = w->pieces.data[piece].body;
	if ( w->bodies.data[body].kind == lp_kindStructure && ( limb->recheckTick == 0 || w->tick + 1 >= limb->recheckTick + LP_GAIT_RECHECK_TICKS ) )
	{
		lpRequestStressCheck( w, body, false );
		limb->recheckTick = w->tick + 1;
	}
}

// ---- swings ----

static float lpSmooth( float s )
{
	s = b3ClampFloat( s, 0.0f, 1.0f );
	return s * s * ( 3.0f - 2.0f * s );
}

// The foot on its arc at s (0 liftoff, 1 landing): it lifts before it crosses and comes down after, over the higher of
// its ends by the step height
static b3Pos lpSwingPoint( const lpLimb* limb, float s, float stepHeight, b3Vec3 up )
{
	b3Vec3 span = b3SubPos( limb->landing, limb->liftoff );
	float rise = b3Dot( span, up );
	b3Vec3 across = lpFlatten( span, up );
	float top = b3MaxFloat( rise, 0.0f ) + stepHeight; // over the liftoff
	float height = s < 0.5f ? top * lpSmooth( 2.0f * s ) : top + ( rise - top ) * lpSmooth( 2.0f * s - 1.0f );
	return b3OffsetPos( limb->liftoff, b3MulAdd( b3MulSV( lpSmooth( ( s - 0.15f ) / 0.7f ), across ), height, up ) );
}

// Half a step's length at a foot drifting at `rate` under the torso: what the torso covers in one swing, so the feet that
// swing and the feet that carry take turns of the same length (a tripod's rhythm), up to most of the stride
static float lpHalfStep( const lpRig* r, float rate )
{
	return b3MinFloat( LP_GAIT_LAND * r->def.stride, 0.5f * rate * r->def.swingTime );
}

// Where a swinging foot lands, across the ground: ahead of its neutral point by most of the stride (along the way a
// planted foot drifts under the commanded motion), where the torso will be when it lands (moving and turning as it
// actually is, over the swing's time left). Its height is the ground's, from the casts.
static b3Pos lpAimLanding( const lpRig* r, const lpLimb* limb, b3WorldTransform pose, b3Vec3 heading, b3Vec3 side,
						   b3Vec3 velocity, float spin, b3Vec3 moving, float turning, float remaining, b3Vec3 up )
{
	b3CosSin turn = b3ComputeCosSin( turning * remaining );
	b3Vec3 f = b3Add( b3MulSV( turn.cosine, heading ), b3MulSV( turn.sine, side ) );
	b3WorldTransform then = { b3OffsetPos( pose.p, b3MulSV( remaining, moving ) ), lpLevelRotation( r, f, up ) };
	b3Pos rest = b3TransformWorldPoint( then, limb->neutral );
	b3Vec3 drift = b3Neg( b3Add( velocity, b3Cross( b3MulSV( spin, up ), b3SubPos( rest, then.p ) ) ) ); // a planted foot's
	float rate = b3Length( drift );
	b3Vec3 lead = rate > 1e-4f ? b3MulSV( -lpHalfStep( r, rate ) / rate, drift ) : b3Vec3_zero;
	return b3OffsetPos( rest, lpFlatten( lead, up ) );
}

// Lift a foot, aim it and cast for the ground there
static void lpLiftFoot( lpWorld* w, lpRig* r, int index, b3Pos foot, b3WorldTransform pose, b3Vec3 heading, b3Vec3 side,
						b3Vec3 velocity, float spin, b3Vec3 moving, float turning, b3Vec3 up )
{
	lpLimb* limb = r->limbs + index;
	limb->planted = false;
	limb->swinging = true;
	limb->swingClock = 0.0f;
	limb->castLate = false;
	limb->liftoff = foot;
	lpFootMoved( w, limb );
	limb->landing = lpAimLanding( r, limb, pose, heading, side, velocity, spin, moving, turning, r->def.swingTime, up );
	if ( w->stats.footCasts < w->def.maxFootCastsPerStep )
	{
		lpCastFoothold( w, r, limb, up );
	}
	else
	{
		limb->grounded = false;
	}
}

// ---- the step ----

void lpWalkRig( lpWorld* w, lpRig* r, float timeStep )
{
	const lpBody* torso = w->bodies.data + r->body;
	b3WorldTransform xf = lpGetTransform( torso );
	b3Vec3 up = lpRigWorldUp( w, r, xf.q );

	// Feet as the model has them at the measured angles, and the planted feet's height under the torso
	b3Pos feet[LP_MAX_RIG_LIMBS];
	bool planted[LP_MAX_RIG_LIMBS];
	float support = 0.0f;
	int plantedCount = 0;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->able == false )
		{
			limb->planted = false;
			limb->swinging = false;
		}
		planted[i] = limb->planted;
		if ( limb->attached == false )
		{
			continue;
		}
		b3Vec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
		for ( int k = 0; k < limb->joints; ++k )
		{
			limb->q[k] = w->links.data[limb->def.links[k]].angle;
		}
		feet[i] = b3TransformWorldPoint( xf, lpLimbForward( w, limb, limb->joints, limb->q, limb->foot, axes, origins ) );
		if ( limb->planted )
		{
			support += b3Dot( b3SubPos( feet[i], xf.p ), up );
			plantedCount += 1;
		}
	}
	if ( plantedCount == 0 )
	{
		r->height = 0.0f;
		return;
	}
	support /= (float)plantedCount; // the planted feet's mean height, from the torso's frame
	r->height = -support;
	b3Vec3 torsoUp = b3RotateVector( xf.q, r->up );
	float tilt = b3Atan2( b3Length( b3Cross( torsoUp, up ) ), b3Dot( torsoUp, up ) );
	float goal = r->def.standHeight * ( 1.0f - r->def.crouchDepth * b3ClampFloat( r->control.crouch, 0.0f, 1.0f ) );
	bool still = lpControlStill( &r->control );

	// Idle: the targets stay frozen until the controls change or something knocks it well off its stance
	if ( r->idle )
	{
		bool knocked = b3AbsFloat( r->height - goal ) > 2.5f * LP_GAIT_CALM_HEIGHT || tilt > 3.0f * LP_GAIT_CALM_TILT;
		if ( r->controlChanged == false && ( b3Body_IsAwake( torso->id ) == false || knocked == false ) )
		{
			return;
		}
		r->idle = false;
		r->calm = 0;
	}

	// The commanded motion: heading turned, velocity along it (no faster than the gait's cadence keeps up), both slowed
	// where a planted foot is overstretched
	b3WorldTransform old = r->desired;
	b3Vec3 heading = lpFlatten( b3RotateVector( old.q, r->forward ), up );
	heading = b3LengthSquared( heading ) > 1e-8f ? b3Normalize( heading ) : lpFlatten( b3RotateVector( xf.q, r->forward ), up );
	b3Vec3 side = b3Cross( up, heading ); // left
	float top = b3MinFloat( r->def.maxSpeed, LP_GAIT_KEEP_UP * 2.0f * LP_GAIT_LAND * r->def.stride / r->def.swingTime );
	b3Vec3 velocity = b3MulSV( top, b3Sub( b3MulSV( r->control.forward, heading ), b3MulSV( r->control.strafe, side ) ) );
	float spin = -r->control.turn * r->def.maxTurn; // rad/s about up, left for positive
	float stretch[LP_MAX_RIG_LIMBS]; // how far each foot has drifted from neutral, of the stride
	float urgency[LP_MAX_RIG_LIMBS]; // moving: how far behind it is along its drift, of its half-step; still: of the stride
	float worst = 0.0f;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		stretch[i] = 0.0f;
		urgency[i] = 0.0f;
		if ( limb->attached )
		{
			b3Vec3 off = lpFlatten( b3SubPos( feet[i], b3TransformWorldPoint( old, limb->neutral ) ), up );
			stretch[i] = b3Length( off ) / r->def.stride;
			b3Vec3 drift = b3Neg( b3Add( velocity, b3Cross( b3MulSV( spin, up ), b3SubPos( feet[i], old.p ) ) ) );
			float rate = b3Length( drift );
			float half = lpHalfStep( r, rate ); // it landed about this far ahead; it steps once about this far behind
			urgency[i] = rate > 0.05f * r->def.maxSpeed ? b3Dot( off, drift ) / ( rate * b3MaxFloat( half, 0.02f ) ) : stretch[i];
			if ( stretch[i] >= 1.0f )
			{
				urgency[i] = b3MaxFloat( urgency[i], 1.0f + stretch[i] ); // past its stride any way (a slip, a rock): due first
			}
			worst = limb->planted ? b3MaxFloat( worst, stretch[i] ) : worst;
		}
	}
	float pace = b3ClampFloat( 1.0f - ( worst - 1.0f ) / LP_GAIT_OVERREACH, 0.0f, 1.0f );
	velocity = b3MulSV( pace, velocity );
	spin *= pace;
	r->pace = pace;

	// Swings under way: their time, their aim (re-aimed across the ground as the torso goes; the ground's height from the
	// casts), the second cast, landing
	b3Vec3 moving = lpFlatten( b3Body_GetLinearVelocity( torso->id ), up ); // the torso, as it actually is
	float turning = b3Dot( b3Body_GetAngularVelocity( torso->id ), up );
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->swinging == false )
		{
			continue;
		}
		limb->swingClock += timeStep;
		float s = limb->swingClock / r->def.swingTime;
		float remaining = b3MaxFloat( r->def.swingTime - limb->swingClock, 0.0f );
		b3Pos aim = lpAimLanding( r, limb, old, heading, side, velocity, spin, moving, turning, remaining, up );
		limb->landing = b3OffsetPos( aim, b3MulSV( b3Dot( b3SubPos( limb->landing, aim ), up ), up ) );
		if ( s >= 0.66f && limb->castLate == false && w->stats.footCasts < w->def.maxFootCastsPerStep )
		{
			limb->castLate = true;
			lpCastFoothold( w, r, limb, up );
		}
		if ( s >= 1.0f )
		{
			// Down: it holds its landing point until the foot gets there (the servos lag the arc a little)
			limb->swinging = false;
			limb->planted = true;
			limb->hold = lpSwingPoint( limb, 1.0f, r->def.stepHeight, up );
			limb->holdClock = 0.0f;
			limb->arrived = false;
			planted[i] = true;
			lpFootMoved( w, limb );
		}
	}

	// A planted foot that has got where it was set down is held where the model has it then, fixed in the world until it
	// lifts (a loaded joint gives a centimetre, so aiming the model at the body's foot would push the foot about; and an
	// anchor that followed the model would slide whenever the joints give instead of moving the torso). One that cannot
	// get there in a moment is held where it got to.
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->planted )
		{
			limb->holdClock += timeStep;
			float off = b3Length( b3SubPos( feet[i], limb->hold ) );
			if ( limb->arrived == false && ( off < LP_GAIT_ARRIVED || limb->holdClock > LP_GAIT_ARRIVE_TIME ) )
			{
				limb->hold = feet[i];
				limb->arrived = true;
			}
			else if ( limb->arrived && off > LP_GAIT_SLIPPED )
			{
				limb->hold = feet[i]; // it slipped, or was knocked: held where it is now
			}
			else if ( limb->arrived )
			{
				// The servos hold their angles as stiffly as they can, so feet held a centimetre apart from where the legs'
				// geometry has them would push against each other through the ground: the hold eases toward the model's
				// foot, slowly enough to push the torso along
				limb->hold = b3OffsetPos( limb->hold, b3MulSV( timeStep / LP_GAIT_EASE_TIME, b3SubPos( feet[i], limb->hold ) ) );
			}
		}
	}

	// Liftoffs: the most stretched foot first; its nearest able neighbours around the body planted, and the centre of mass
	// inside the other planted feet by the margin, now and where the torso will be when it lands
	b3Pos center = lpRigCenter( w, r );
	b3Pos later = b3OffsetPos( center, b3MulSV( r->def.swingTime, moving ) );
	float threshold = still ? LP_GAIT_TIDY : LP_GAIT_DUE;
	b3Vec3 shift = b3Vec3_zero;
	bool waited = false;
	// Feet that swing together are every other able leg around the body (on six, the two tripods): each able limb's
	// place in that ring, and the parity of the ones swinging now (-1: none yet)
	int place[LP_MAX_RIG_LIMBS];
	int able = 0, group = -1;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		place[i] = r->limbs[i].able ? able++ : -1;
		group = r->limbs[i].swinging && place[i] >= 0 ? place[i] % 2 : group;
	}
	for ( ;; )
	{
		int pick = -1;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			const lpLimb* limb = r->limbs + i;
			bool inGroup = group < 0 || place[i] % 2 == group;
			bool settled = limb->holdClock >= LP_GAIT_MIN_STANCE; // just set down, it carries a moment first
			if ( limb->planted && settled && inGroup && urgency[i] >= threshold && ( pick < 0 || urgency[i] > urgency[pick] ) )
			{
				pick = i;
			}
		}
		if ( pick < 0 )
		{
			break;
		}
		urgency[pick] = -FLT_MAX; // considered
		bool neighbours = true;
		for ( int dir = -1; dir <= 1; dir += 2 )
		{
			for ( int step = 1; step < r->limbCount; ++step )
			{
				int j = ( pick + dir * step + r->limbCount ) % r->limbCount;
				if ( r->limbs[j].able )
				{
					neighbours = neighbours && r->limbs[j].planted;
					break;
				}
			}
		}
		bool busy = false; // standing still, one foot at a time
		for ( int i = 0; i < r->limbCount && still; ++i )
		{
			busy = busy || r->limbs[i].swinging;
		}
		if ( neighbours == false || busy )
		{
			continue;
		}
		planted[pick] = false;
		float margin = b3MinFloat( lpSupportMargin( feet, planted, r->limbCount, center, up ),
								   lpSupportMargin( feet, planted, r->limbCount, later, up ) );
		if ( margin < r->def.margin )
		{
			planted[pick] = true;
			if ( waited == false )
			{
				// The first foot that waits for balance leans the body toward the feet that would hold it
				waited = true;
				b3Vec3 sum = b3Vec3_zero;
				int n = 0;
				for ( int i = 0; i < r->limbCount; ++i )
				{
					if ( planted[i] && i != pick )
					{
						sum = b3Add( sum, b3SubPos( feet[i], center ) );
						n += 1;
					}
				}
				b3Vec3 toward = n > 0 ? lpFlatten( b3MulSV( 1.0f / (float)n, sum ), up ) : b3Vec3_zero;
				float distance = b3Length( toward );
				if ( distance > 1e-3f )
				{
					shift = b3MulSV( b3MinFloat( LP_GAIT_SHIFT, distance / timeStep ) / distance, toward );
				}
			}
			continue;
		}
		lpLiftFoot( w, r, pick, feet[pick], old, heading, side, velocity, spin, moving, turning, up );
		planted[pick] = false;
		group = place[pick] % 2;
	}

	// A limb able again (fed again, say) that is neither planted nor swinging swings to a foothold
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->able && limb->planted == false && limb->swinging == false )
		{
			lpLiftFoot( w, r, i, feet[i], old, heading, side, velocity, spin, moving, turning, up );
		}
	}

	r->waiting = waited;

	// The desired pose: turned and moved by the controls (and leaning for balance), never far ahead of the torso, level,
	// climbing toward its height over the planted feet
	b3Vec3 measured = lpFlatten( b3RotateVector( xf.q, r->forward ), up );
	measured = b3LengthSquared( measured ) > 1e-8f ? b3Normalize( measured ) : heading;
	float yawError = b3Atan2( b3Dot( b3Cross( measured, heading ), up ), b3Dot( measured, heading ) );
	float yaw = b3ClampFloat( yawError + spin * timeStep, -LP_GAIT_LEAD_TURN, LP_GAIT_LEAD_TURN ) - yawError;
	b3CosSin cs = b3ComputeCosSin( yaw );
	heading = b3Add( b3MulSV( cs.cosine, heading ), b3MulSV( cs.sine, side ) );
	b3Vec3 ahead = b3MulAdd( b3SubPos( old.p, xf.p ), timeStep, b3Add( velocity, shift ) ); // from the torso's frame
	// It may run a little ahead of the torso (the servos' gains pull the torso along), and further behind (a torso that
	// runs ahead is held back instead of dragging it along)
	b3Vec3 flat = lpFlatten( ahead, up );
	float lead = b3Length( flat );
	if ( lead > LP_GAIT_LAG )
	{
		flat = b3MulSV( LP_GAIT_LAG / lead, flat );
	}
	float pull = b3Length( velocity );
	float forward = pull > 1e-4f ? b3Dot( flat, velocity ) / pull : 0.0f;
	if ( forward > LP_GAIT_LEAD )
	{
		flat = b3MulAdd( flat, ( LP_GAIT_LEAD - forward ) / pull, velocity );
	}
	float height = b3Dot( b3SubPos( old.p, xf.p ), up ) - support; // the desired pose's, over the feet
	float climb = b3ClampFloat( goal - height, -LP_GAIT_CLIMB * timeStep, LP_GAIT_CLIMB * timeStep );
	height += climb;
	r->desired.p = b3OffsetPos( xf.p, b3MulAdd( flat, support + height, up ) );
	r->desired.q = lpLevelRotation( r, heading, up );
	// The feedforward is the commanded motion only: the pull that keeps the pose near the torso is left to the servos'
	// gains (fed forward, a torso running ahead would push itself on faster)
	b3Vec3 linear = b3MulAdd( b3Add( velocity, shift ), climb / timeStep, up );
	b3Vec3 angular = b3MulSV( spin, up );

	// Targets: each attached, able limb's foot from the desired pose (planted: where it is; swinging: on its arc, a step
	// on), with the joint speeds that keep it there as the pose moves
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->planted == false && limb->swinging == false )
		{
			continue;
		}
		b3Pos foot = limb->hold;
		b3Vec3 motion = b3Vec3_zero; // of the foot in the world
		if ( limb->swinging )
		{
			float s0 = limb->swingClock / r->def.swingTime;
			float s1 = ( limb->swingClock + timeStep ) / r->def.swingTime;
			foot = lpSwingPoint( limb, s1, r->def.stepHeight, up );
			motion = b3MulSV( 1.0f / timeStep, b3SubPos( foot, lpSwingPoint( limb, s0, r->def.stepHeight, up ) ) );
		}
		b3Vec3 target = b3InvTransformWorldPoint( r->desired, foot );
		limb->residual = lpLimbIK( w, limb, limb->joints, limb->foot, target, limb->q );
		b3Vec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
		b3Vec3 at = lpLimbForward( w, limb, limb->joints, limb->q, limb->foot, axes, origins );
		b3Vec3 arm = b3SubPos( foot, r->desired.p );
		b3Vec3 relative = b3InvRotateVector( r->desired.q, b3Sub( motion, b3Add( linear, b3Cross( angular, arm ) ) ) );
		float feed[LP_MAX_LIMB_JOINTS];
		lpLimbSpeeds( limb->joints, axes, origins, at, relative, feed );
		for ( int k = 0; k < limb->joints; ++k )
		{
			lpWorld_SetLinkTarget( w, limb->def.links[k], limb->q[k] );
			w->links.data[limb->def.links[k]].feed = feed[k];
		}
	}

	// Still, settled and every foot down for a moment: freeze the targets
	bool swinging = false;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		swinging = swinging || r->limbs[i].swinging;
	}
	bool settled = b3AbsFloat( height - goal ) < 0.001f && b3AbsFloat( r->height - goal ) < LP_GAIT_CALM_HEIGHT &&
				   tilt < LP_GAIT_CALM_TILT && swinging == false && waited == false;
	r->calm = still && settled ? r->calm + 1 : 0;
	if ( r->calm >= LP_GAIT_CALM_TICKS )
	{
		r->idle = true;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			for ( int k = 0; k < r->limbs[i].def.linkCount; ++k )
			{
				w->links.data[r->limbs[i].def.links[k]].feed = 0.0f;
			}
		}
	}
}
