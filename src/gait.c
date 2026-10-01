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
#define LP_GAIT_FALLING 1.0f	// m/s: a torso sinking faster than this is falling
#define LP_GAIT_CALM_TICKS 30
#define LP_GAIT_CALM_HEIGHT 0.02f
#define LP_GAIT_CALM_TILT 0.0175f // rad
#define LP_GAIT_RECHECK_TICKS 30  // a structure a foot lands on or leaves is re-checked at most this often
#define LP_GAIT_ARRIVED 0.03f	  // m: a planted foot this near where it was set down has got there
#define LP_GAIT_ARRIVE_TIME 0.3f  // s it may take
#define LP_GAIT_MIN_STANCE 0.15f  // s a foot set down stays down
#define LP_GAIT_SLIPPED 0.1f	  // m: a held foot this far from its hold slipped or was knocked
#define LP_GAIT_EASE_TIME 0.3f	  // s a held foot's hold takes to ease toward where the legs' geometry has it
#define LP_GAIT_WALKING_LEGS 4	  // fewer able limbs than this cannot lift one and stay up: it crawls on its belly
#define LP_GAIT_WEAK_SAG 0.3f	  // share of its height the torso drops as its weakest planted leg's strength goes to 0
#define LP_GAIT_REACH_SPARE 0.05f // m a leg keeps in hand below its deepest reach
#define LP_GAIT_CRAWL_SPEED 0.35f // share of its top speed it drags itself at
#define LP_GAIT_TUCK 0.4f		  // share of its stand height below the torso an unable limb's foot is held at
#define LP_GAIT_STALL_PACE 0.1f	  // told to move, making less than this share of the speed asked...
#define LP_GAIT_STALL_TICKS 90	  // ...for this long, it is stuck: it crawls
#define LP_GAIT_STRIKE 8.0f	  // 1/s: a reaching limb's joints go at full speed until this near their target (4 rad/s: 0.5 rad)

static bool lpControlStill( const lpRigControl* c )
{
	return c->forward == 0.0f && c->strafe == 0.0f && c->turn == 0.0f;
}

// Level, with the rig's forward along `heading` (horizontal) and its up along worldUp
static lpQuat lpLevelRotation( const lpRig* r, lpVec3 heading, lpVec3 worldUp )
{
	lpMatrix3 local = { lpCross( r->up, r->forward ), r->up, r->forward };
	lpMatrix3 world = { lpCross( worldUp, heading ), worldUp, heading };
	return lpMulQuat( lpMakeQuatFromMatrix( &world ), lpConjugate( lpMakeQuatFromMatrix( &local ) ) );
}

static lpVec3 lpFlatten( lpVec3 v, lpVec3 up )
{
	return lpSub( v, lpMulSV( lpDot( v, up ), up ) );
}

// ---- balance ----

// The rig's centre of mass: its torso and the bodies of its limbs' chains
static lpPos lpRigCenter( const lpWorld* w, const lpRig* r )
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
	lpPos origin = b3Body_GetWorldCenter( w->bodies.data[r->body].id );
	lpVec3 sum = lpVec3_zero;
	float mass = 0.0f;
	for ( int n = 0; n < count; ++n )
	{
		b3BodyId id = w->bodies.data[bodies[n]].id;
		float m = b3Body_GetMass( id );
		sum = lpMulAdd( sum, m, lpSubPos( b3Body_GetWorldCenter( id ), origin ) );
		mass += m;
	}
	return mass > 0.0f ? lpOffsetPos( origin, lpMulSV( 1.0f / mass, sum ) ) : origin;
}

// How far the point lies inside the convex hull of the feet, seen along up (negative outside; -FLT_MAX with fewer than
// three feet). Feet are 2D in the plane across up; the hull is a monotone chain over a total order.
static float lpSupportMargin( const lpPos* feet, const bool* use, int count, lpPos point, lpVec3 up )
{
	lpVec3 e1 = lpAbs( up ).x < 0.9f ? lpNormalize( lpCross( up, (lpVec3){ 1.0f, 0.0f, 0.0f } ) )
									 : lpNormalize( lpCross( up, (lpVec3){ 0.0f, 0.0f, 1.0f } ) );
	lpVec3 e2 = lpCross( up, e1 );
	float px[LP_MAX_RIG_LIMBS], py[LP_MAX_RIG_LIMBS];
	int order[LP_MAX_RIG_LIMBS];
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		if ( use[i] )
		{
			lpVec3 d = lpSubPos( feet[i], point );
			px[n] = lpDot( d, e1 );
			py[n] = lpDot( d, e2 );
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
			margin = lpMinFloat( margin, ( ex * ( -py[a] ) - ey * ( -px[a] ) ) / length );
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
	lpPos point;
	int piece;
	bool hit;
} lpFootCast;

static float lpFootCastFcn( b3ShapeId shapeId, lpPos point, lpVec3 normal, float fraction, uint64_t userMaterialId,
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
	if ( fraction < cast->fraction || ( cast->hit && fraction == cast->fraction && piece < cast->piece ) )
	{
		cast->fraction = fraction;
		cast->point = point;
		cast->piece = piece;
		cast->hit = true;
	}
	return lpNextUp( cast->fraction );
}

// The ground under a planned foothold: its landing height is where a sole-sized sphere comes to rest on it
static void lpCastFoothold( lpWorld* w, const lpRig* r, lpLimb* limb, lpVec3 up )
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
	lpVec3 center = lpVec3_zero;
	b3ShapeProxy proxy = { &center, 1, LP_GAIT_SOLE };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = LP_CAT_VEHICLE;
	filter.maskBits = LP_CAT_STATIC | LP_CAT_FULL;
	lpPos from = lpOffsetPos( limb->landing, lpMulSV( LP_GAIT_CLEARANCE + LP_GAIT_SOLE, up ) );
	lpFootCast cast = { w, skip, count, FLT_MAX, { 0 }, -1, false };
	b3World_CastShape( w->def.physics, from, &proxy, lpMulSV( -( LP_GAIT_CLEARANCE + LP_GAIT_DEPTH ), up ), filter,
					   lpFootCastFcn, &cast );
	w->stats.footCasts += 1;
	limb->grounded = cast.hit;
	limb->groundPiece = cast.hit ? cast.piece : -1;
	limb->groundGeneration = cast.piece >= 0 ? w->pieces.data[cast.piece].generation : 0;
	float drop = cast.hit ? cast.fraction * ( LP_GAIT_CLEARANCE + LP_GAIT_DEPTH ) : LP_GAIT_CLEARANCE + 0.3f;
	limb->landing = lpOffsetPos( from, lpMulSV( -( drop + LP_GAIT_SOLE ), up ) );
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
	s = lpClampFloat( s, 0.0f, 1.0f );
	return s * s * ( 3.0f - 2.0f * s );
}

// The foot on its arc at s (0 liftoff, 1 landing): it lifts before it crosses and comes down after, over the higher of
// its ends by the step height
static lpPos lpSwingPoint( const lpLimb* limb, float s, float stepHeight, lpVec3 up )
{
	lpVec3 span = lpSubPos( limb->landing, limb->liftoff );
	float rise = lpDot( span, up );
	lpVec3 across = lpFlatten( span, up );
	float top = lpMaxFloat( rise, 0.0f ) + stepHeight; // over the liftoff
	float height = s < 0.5f ? top * lpSmooth( 2.0f * s ) : top + ( rise - top ) * lpSmooth( 2.0f * s - 1.0f );
	return lpOffsetPos( limb->liftoff, lpMulAdd( lpMulSV( lpSmooth( ( s - 0.15f ) / 0.7f ), across ), height, up ) );
}

// Half a step's length at a foot drifting at `rate` under the torso: what the torso covers in one swing, so the feet that
// swing and the feet that carry take turns of the same length (a tripod's rhythm), up to most of the stride
static float lpHalfStep( const lpRig* r, float rate )
{
	return lpMinFloat( LP_GAIT_LAND * r->def.stride, 0.5f * rate * r->def.swingTime );
}

// Where a swinging foot lands, across the ground: ahead of its neutral point by most of the stride (along the way a
// planted foot drifts under the commanded motion), where the torso will be when it lands (moving and turning as it
// actually is, over the swing's time left). Its height is the ground's, from the casts.
static lpPos lpAimLanding( const lpRig* r, const lpLimb* limb, lpWorldTransform pose, lpVec3 heading, lpVec3 side,
						   lpVec3 velocity, float spin, lpVec3 moving, float turning, float remaining, lpVec3 up )
{
	lpCosSin turn = lpComputeCosSin( turning * remaining );
	lpVec3 f = lpAdd( lpMulSV( turn.cosine, heading ), lpMulSV( turn.sine, side ) );
	lpWorldTransform then = { lpOffsetPos( pose.p, lpMulSV( remaining, moving ) ), lpLevelRotation( r, f, up ) };
	lpPos rest = lpTransformWorldPoint( then, limb->neutral );
	lpVec3 drift = lpNeg( lpAdd( velocity, lpCross( lpMulSV( spin, up ), lpSubPos( rest, then.p ) ) ) ); // a planted foot's
	float rate = lpLength( drift );
	lpVec3 lead = rate > 1e-4f ? lpMulSV( -lpHalfStep( r, rate ) / rate, drift ) : lpVec3_zero;
	return lpOffsetPos( rest, lpFlatten( lead, up ) );
}

// Lift a foot, aim it and cast for the ground there
static void lpLiftFoot( lpWorld* w, lpRig* r, int index, lpPos foot, lpWorldTransform pose, lpVec3 heading, lpVec3 side,
						lpVec3 velocity, float spin, lpVec3 moving, float turning, lpVec3 up )
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

// The body leans toward the planted feet but `skip` (the one that waits to lift), to bring its centre of mass over them
static lpVec3 lpLean( const lpRig* r, const lpPos* feet, const bool* planted, int skip, lpPos center, lpVec3 up, float timeStep )
{
	lpVec3 sum = lpVec3_zero;
	int n = 0;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		if ( planted[i] && i != skip )
		{
			sum = lpAdd( sum, lpSubPos( feet[i], center ) );
			n += 1;
		}
	}
	lpVec3 toward = n > 0 ? lpFlatten( lpMulSV( 1.0f / (float)n, sum ), up ) : lpVec3_zero;
	float distance = lpLength( toward );
	return distance > 1e-3f ? lpMulSV( lpMinFloat( LP_GAIT_SHIFT, distance / timeStep ) / distance, toward ) : lpVec3_zero;
}

// ---- the step ----

void lpWalkRig( lpWorld* w, lpRig* r, float timeStep )
{
	const lpBody* torso = w->bodies.data + r->body;
	lpWorldTransform xf = lpGetTransform( torso );
	lpVec3 up = lpRigWorldUp( w, r, xf.q );

	// Feet as the model has them at the measured angles, and the planted feet's height under the torso
	lpPos feet[LP_MAX_RIG_LIMBS];
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
		lpVec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
		for ( int k = 0; k < limb->joints; ++k )
		{
			limb->q[k] = w->links.data[limb->def.links[k]].angle;
		}
		feet[i] = lpTransformWorldPoint( xf, lpLimbForward( w, limb, limb->joints, limb->q, limb->foot, axes, origins ) );
		if ( limb->planted )
		{
			support += lpDot( lpSubPos( feet[i], xf.p ), up );
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
	lpVec3 torsoUp = lpRotateVector( xf.q, r->up );
	float tilt = lpAtan2( lpLength( lpCross( torsoUp, up ) ), lpDot( torsoUp, up ) );
	// The height it stands at: crouched as asked, lower as its weakest planted leg weakens, no higher than its shortest able
	// leg reaches (a stump), and down on its belly with too few legs to walk
	int able = 0;
	float weakest = 1.0f, shortest = FLT_MAX;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		const lpLimb* limb = r->limbs + i;
		if ( limb->able && limb->reaching == false ) // a reaching limb is out of the gait
		{
			able += 1;
			shortest = lpMinFloat( shortest, limb->depth );
			weakest = lpMinFloat( weakest, limb->strength );
		}
	}
	// Stuck (no foot can lift without tipping it over, say one leg left on a side) it crawls too, until its legs change
	if ( able != r->ableSeen )
	{
		r->ableSeen = able;
		r->stuck = false;
		r->stall = 0;
	}
	r->crawling = able < LP_GAIT_WALKING_LEGS || r->stuck;
	float goal = r->def.standHeight * ( 1.0f - r->def.crouchDepth * lpClampFloat( r->control.crouch, 0.0f, 1.0f ) );
	goal *= 1.0f - LP_GAIT_WEAK_SAG * ( 1.0f - lpClampFloat( weakest, 0.0f, 1.0f ) );
	goal = lpMinFloat( goal, shortest - LP_GAIT_REACH_SPARE );
	goal = r->crawling ? r->def.bellyHeight : lpMaxFloat( goal, r->def.bellyHeight );
	bool still = lpControlStill( &r->control );

	// Idle: the targets stay frozen until the controls change or something knocks it well off its stance
	if ( r->idle )
	{
		bool knocked = lpAbsFloat( r->height - goal ) > 2.5f * LP_GAIT_CALM_HEIGHT || tilt > 3.0f * LP_GAIT_CALM_TILT;
		if ( r->controlChanged == false && ( b3Body_IsAwake( torso->id ) == false || knocked == false ) )
		{
			return;
		}
		r->idle = false;
		r->calm = 0;
	}

	// The commanded motion: heading turned, velocity along it (no faster than the gait's cadence keeps up), both slowed
	// where a planted foot is overstretched
	lpWorldTransform old = r->desired;
	lpVec3 heading = lpFlatten( lpRotateVector( old.q, r->forward ), up );
	heading = lpLengthSquared( heading ) > 1e-8f ? lpNormalize( heading ) : lpFlatten( lpRotateVector( xf.q, r->forward ), up );
	lpVec3 side = lpCross( up, heading ); // left
	float top = lpMinFloat( r->def.maxSpeed, LP_GAIT_KEEP_UP * 2.0f * LP_GAIT_LAND * r->def.stride / r->def.swingTime );
	// Fewer legs swing in more turns, each foot carrying longer: on five it keeps half its pace, on four a third
	top *= r->crawling ? LP_GAIT_CRAWL_SPEED : ( able >= 6 ? 1.0f : ( able == 5 ? 0.5f : 0.35f ) );
	lpVec3 velocity = lpMulSV( top, lpSub( lpMulSV( r->control.forward, heading ), lpMulSV( r->control.strafe, side ) ) );
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
			lpVec3 off = lpFlatten( lpSubPos( feet[i], lpTransformWorldPoint( old, limb->neutral ) ), up );
			stretch[i] = lpLength( off ) / r->def.stride;
			lpVec3 drift = lpNeg( lpAdd( velocity, lpCross( lpMulSV( spin, up ), lpSubPos( feet[i], old.p ) ) ) );
			float rate = lpLength( drift );
			float half = lpHalfStep( r, rate ); // it landed about this far ahead; it steps once about this far behind
			urgency[i] = rate > 0.05f * r->def.maxSpeed ? lpDot( off, drift ) / ( rate * lpMaxFloat( half, 0.02f ) ) : stretch[i];
			if ( stretch[i] >= 1.0f )
			{
				urgency[i] = lpMaxFloat( urgency[i], 1.0f + stretch[i] ); // past its stride any way (a slip, a rock): due first
			}
			worst = limb->planted ? lpMaxFloat( worst, stretch[i] ) : worst;
		}
	}
	float pace = lpClampFloat( 1.0f - ( worst - 1.0f ) / LP_GAIT_OVERREACH, 0.0f, 1.0f );
	lpVec3 wanted = velocity; // as commanded, before its feet held it back
	velocity = lpMulSV( pace, velocity );
	spin *= pace;
	r->pace = pace;

	// Swings under way: their time, their aim (re-aimed across the ground as the torso goes; the ground's height from the
	// casts), the second cast, landing
	lpVec3 moving = lpFlatten( b3Body_GetLinearVelocity( torso->id ), up ); // the torso, as it actually is
	float turning = lpDot( b3Body_GetAngularVelocity( torso->id ), up );
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->swinging == false )
		{
			continue;
		}
		limb->swingClock += timeStep;
		float s = limb->swingClock / r->def.swingTime;
		float remaining = lpMaxFloat( r->def.swingTime - limb->swingClock, 0.0f );
		lpPos aim = lpAimLanding( r, limb, old, heading, side, velocity, spin, moving, turning, remaining, up );
		limb->landing = lpOffsetPos( aim, lpMulSV( lpDot( lpSubPos( limb->landing, aim ), up ), up ) );
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
			float off = lpLength( lpSubPos( feet[i], limb->hold ) );
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
				limb->hold = lpOffsetPos( limb->hold, lpMulSV( timeStep / LP_GAIT_EASE_TIME, lpSubPos( feet[i], limb->hold ) ) );
			}
		}
	}

	// Liftoffs: the most stretched foot first; its nearest able neighbours around the body planted, and the centre of mass
	// inside the other planted feet by the margin, now and where the torso will be when it lands
	lpPos center = lpRigCenter( w, r );
	lpPos later = lpOffsetPos( center, lpMulSV( r->def.swingTime, moving ) );

	// Limbs told to reach leave the gait once the others hold the centre of mass by the margin (crawling, its belly does);
	// until then the body leans toward them. Told to stop, a limb steps back in (below: a limb neither planted nor
	// swinging swings to a foothold).
	lpVec3 reachLean = lpVec3_zero;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->reachWanted == false || limb->able == false )
		{
			limb->reaching = false;
			continue;
		}
		if ( limb->reaching )
		{
			continue;
		}
		if ( limb->planted )
		{
			planted[i] = false;
			bool steady = r->crawling || lpSupportMargin( feet, planted, r->limbCount, center, up ) >= r->def.margin;
			planted[i] = steady == false; // it stays down until the others can hold the body without it
			if ( steady == false )
			{
				reachLean = lpLean( r, feet, planted, i, center, up, timeStep );
				continue;
			}
			lpFootMoved( w, limb );
		}
		limb->planted = false;
		limb->swinging = false;
		limb->reaching = true;
	}
	float threshold = still ? LP_GAIT_TIDY : LP_GAIT_DUE;
	lpVec3 shift = lpVec3_zero;
	bool waited = false;
	// Feet that swing together are every other able leg around the body (on six, the two tripods; with an odd number
	// that does not go round, and only the neighbour rule holds): each able limb's place in that ring, and the parity of
	// the ones swinging now (-1: none yet)
	int place[LP_MAX_RIG_LIMBS];
	int ring = 0, group = -1;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		place[i] = r->limbs[i].able && r->limbs[i].reaching == false ? ring++ : -1;
		group = r->limbs[i].swinging && place[i] >= 0 ? place[i] % 2 : group;
	}
	for ( ;; )
	{
		int pick = -1;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			const lpLimb* limb = r->limbs + i;
			bool inGroup = group < 0 || ring % 2 == 1 || place[i] % 2 == group;
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
				if ( r->limbs[j].able && r->limbs[j].reaching == false )
				{
					neighbours = neighbours && r->limbs[j].planted;
					break;
				}
			}
		}
		bool busy = false; // standing still or crawling, one foot at a time
		for ( int i = 0; i < r->limbCount && ( still || r->crawling ); ++i )
		{
			busy = busy || r->limbs[i].swinging;
		}
		if ( neighbours == false || busy )
		{
			continue;
		}
		planted[pick] = false;
		float margin = lpMinFloat( lpSupportMargin( feet, planted, r->limbCount, center, up ),
								   lpSupportMargin( feet, planted, r->limbCount, later, up ) );
		if ( margin < r->def.margin && r->crawling == false ) // crawling, its belly holds it
		{
			planted[pick] = true;
			if ( waited == false )
			{
				// The first foot that waits for balance leans the body toward the feet that would hold it
				waited = true;
				shift = lpLean( r, feet, planted, pick, center, up, timeStep );
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
		if ( limb->able && limb->planted == false && limb->swinging == false && limb->reaching == false )
		{
			lpLiftFoot( w, r, i, feet[i], old, heading, side, velocity, spin, moving, turning, up );
		}
	}

	shift = lpLengthSquared( shift ) > 0.0f ? shift : reachLean;
	r->waiting = waited || lpLengthSquared( reachLean ) > 0.0f;
	// Told to move and getting nowhere (its feet may still step: one leg left on a side lifts nothing and the others
	// shuffle) for a while, it is stuck
	float asked = lpLength( wanted );
	float made = asked > 1e-3f ? lpDot( moving, wanted ) / asked : 0.0f;
	bool getting = asked < 1e-3f || made > LP_GAIT_STALL_PACE * asked; // turning on the spot asks for no speed
	r->stall = still == false && getting == false ? r->stall + 1 : 0;
	r->stuck = r->stuck || r->stall >= LP_GAIT_STALL_TICKS;

	// The desired pose: turned and moved by the controls (and leaning for balance), never far ahead of the torso, level,
	// climbing toward its height over the planted feet
	lpVec3 measured = lpFlatten( lpRotateVector( xf.q, r->forward ), up );
	measured = lpLengthSquared( measured ) > 1e-8f ? lpNormalize( measured ) : heading;
	float yawError = lpAtan2( lpDot( lpCross( measured, heading ), up ), lpDot( measured, heading ) );
	float yaw = lpClampFloat( yawError + spin * timeStep, -LP_GAIT_LEAD_TURN, LP_GAIT_LEAD_TURN ) - yawError;
	lpCosSin cs = lpComputeCosSin( yaw );
	heading = lpAdd( lpMulSV( cs.cosine, heading ), lpMulSV( cs.sine, side ) );
	lpVec3 ahead = lpMulAdd( lpSubPos( old.p, xf.p ), timeStep, lpAdd( velocity, shift ) ); // from the torso's frame
	// It may run a little ahead of the torso (the servos' gains pull the torso along), and further behind (a torso that
	// runs ahead is held back instead of dragging it along)
	lpVec3 flat = lpFlatten( ahead, up );
	float lead = lpLength( flat );
	if ( lead > LP_GAIT_LAG )
	{
		flat = lpMulSV( LP_GAIT_LAG / lead, flat );
	}
	float pull = lpLength( velocity );
	float forward = pull > 1e-4f ? lpDot( flat, velocity ) / pull : 0.0f;
	if ( forward > LP_GAIT_LEAD )
	{
		flat = lpMulAdd( flat, ( LP_GAIT_LEAD - forward ) / pull, velocity );
	}
	// Falling (dropped, or its legs knocked from under it), it follows the torso down: pushed back up to where it was, it
	// would land and spring up again
	float height = lpDot( lpSubPos( old.p, xf.p ), up ) - support; // the desired pose's, over the feet
	if ( lpDot( b3Body_GetLinearVelocity( torso->id ), up ) < -LP_GAIT_FALLING )
	{
		height = lpMinFloat( height, r->height + LP_GAIT_LEAD );
	}
	float climb = lpClampFloat( goal - height, -LP_GAIT_CLIMB * timeStep, LP_GAIT_CLIMB * timeStep );
	height += climb;
	r->desired.p = lpOffsetPos( xf.p, lpMulAdd( flat, support + height, up ) );
	r->desired.q = lpLevelRotation( r, heading, up );
	// The feedforward is the commanded motion only: the pull that keeps the pose near the torso is left to the servos'
	// gains (fed forward, a torso running ahead would push itself on faster)
	lpVec3 linear = lpMulAdd( lpAdd( velocity, shift ), climb / timeStep, up );
	lpVec3 angular = lpMulSV( spin, up );

	// An attached limb too weak or too short to stand is held clear of the ground, tucked under its hip
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->attached == false || limb->able || limb->joints < 2 || limb->strength <= 0.0f )
		{
			continue;
		}
		lpVec3 target = lpMulAdd( lpSub( limb->neutral, lpMulSV( lpDot( limb->neutral, r->up ), r->up ) ), -LP_GAIT_TUCK * r->def.standHeight,
								  r->up );
		target = lpLerp( lpMulSV( 0.6f, target ), target, 0.5f ); // drawn in toward the torso
		lpLimbIK( w, limb, limb->joints, limb->foot, target, limb->q );
		for ( int k = 0; k < limb->joints; ++k )
		{
			lpWorld_SetLinkTarget( w, limb->def.links[k], limb->q[k] );
			w->links.data[limb->def.links[k]].feed = 0.0f;
		}
	}

	// Targets: each attached, able limb's foot from the desired pose (planted: where it is; swinging: on its arc, a step
	// on), with the joint speeds that keep it there as the pose moves
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->planted == false && limb->swinging == false && limb->reaching == false )
		{
			continue;
		}
		lpPos foot = limb->reaching ? limb->reachPoint : limb->hold;
		lpVec3 motion = lpVec3_zero; // of the foot in the world
		if ( limb->swinging )
		{
			float s0 = limb->swingClock / r->def.swingTime;
			float s1 = ( limb->swingClock + timeStep ) / r->def.swingTime;
			foot = lpSwingPoint( limb, s1, r->def.stepHeight, up );
			motion = lpMulSV( 1.0f / timeStep, lpSubPos( foot, lpSwingPoint( limb, s0, r->def.stepHeight, up ) ) );
		}
		lpVec3 target = lpInvTransformWorldPoint( r->desired, foot );
		limb->residual = lpLimbIK( w, limb, limb->joints, limb->foot, target, limb->q );
		lpVec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
		lpVec3 at = lpLimbForward( w, limb, limb->joints, limb->q, limb->foot, axes, origins );
		lpVec3 arm = lpSubPos( foot, r->desired.p );
		lpVec3 relative = lpInvRotateVector( r->desired.q, lpSub( motion, lpAdd( linear, lpCross( angular, arm ) ) ) );
		float feed[LP_MAX_LIMB_JOINTS];
		lpLimbSpeeds( limb->joints, axes, origins, at, relative, feed );
		if ( limb->reaching )
		{
			// A strike: its joints go at full speed until they are nearly there (a servo alone slows as it closes in, and a
			// stomp would land too gently to break anything)
			for ( int k = 0; k < limb->joints; ++k )
			{
				const lpLink* l = w->links.data + limb->def.links[k];
				float fastest = l->def.motor.maxSpeed;
				feed[k] = lpClampFloat( LP_GAIT_STRIKE * ( limb->q[k] - l->angle ), -fastest, fastest );
			}
		}
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
		swinging = swinging || r->limbs[i].swinging || r->limbs[i].reaching;
	}
	bool settled = lpAbsFloat( height - goal ) < 0.001f && lpAbsFloat( r->height - goal ) < LP_GAIT_CALM_HEIGHT &&
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
