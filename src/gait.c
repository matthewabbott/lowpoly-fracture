// SPDX-License-Identifier: MIT
// Gait: the built-in walker (lp_walkerGait), tuned by lpRigDef.gait: which feet step, where they land, and how the torso
// moves over them. rig.c has the mechanism any walker uses: the limbs, their IK, capability, the rig's centre of mass,
// the support margin and the stress re-check of what a foot stands on; a game that walks its rigs itself drives each foot
// there (lp_walkerNone).
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

// ---- footholds ----

typedef struct lpFootSkip
{
	const lpWorld* world;
	const int* skip; // the rig's bodies
	int skipCount;
} lpFootSkip;

static bool lpFootAccept( int piece, float fraction, void* context )
{
	(void)fraction;
	const lpFootSkip* skip = context;
	if ( piece >= 0 )
	{
		int body = skip->world->pieces.data[piece].body;
		for ( int n = 0; n < skip->skipCount; ++n )
		{
			if ( skip->skip[n] == body )
			{
				return false;
			}
		}
	}
	return true;
}

// The ground under a planned foothold: its landing height is where a sole-sized sphere comes to rest on it
static void lpCastFoothold( lpWorld* w, const lpRig* r, lpLimb* limb, lpVec3 up )
{
	const lpGaitDef* g = &r->def.gait;
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
	lpPhysFilter filter = { LP_CAT_VEHICLE, LP_CAT_STATIC | LP_CAT_FULL };
	lpPos from = lpOffsetPos( limb->landing, lpMulSV( g->clearance + g->sole, up ) );
	lpFootSkip rig = { w, skip, count };
	lpPhysCastHit cast = lpPhys_CastShape( w->phys, from, &center, 1, g->sole,
										   lpMulSV( -( g->clearance + g->depth ), up ), filter, lpFootAccept, &rig );
	w->stats.footCasts += 1;
	limb->grounded = cast.hit;
	limb->groundPiece = cast.hit ? cast.piece : -1;
	limb->groundGeneration = cast.piece >= 0 ? w->pieces.data[cast.piece].generation : 0;
	float drop = cast.hit ? cast.fraction * ( g->clearance + g->depth ) : g->clearance + g->missDrop;
	limb->landing = lpOffsetPos( from, lpMulSV( -( drop + g->sole ), up ) );
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
	const lpGaitDef* g = &r->def.gait;
	return lpMinFloat( g->land * g->stride, 0.5f * rate * g->swingTime );
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
	const lpGaitDef* g = &r->def.gait;
	lpLimb* limb = r->limbs + index;
	limb->planted = false;
	limb->swinging = true;
	limb->swingClock = 0.0f;
	limb->castLate = false;
	limb->liftoff = foot;
	lpFootMoved( w, limb );
	limb->landing = lpAimLanding( r, limb, pose, heading, side, velocity, spin, moving, turning, g->swingTime, up );
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
	const lpGaitDef* g = &r->def.gait;
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
	return distance > 1e-3f ? lpMulSV( lpMinFloat( g->shift, distance / timeStep ) / distance, toward ) : lpVec3_zero;
}

// ---- the step ----

void lpWalkRig( lpWorld* w, lpRig* r, float timeStep )
{
	const lpGaitDef* g = &r->def.gait;
	const lpBody* torso = w->bodies.data + r->body;
	lpWorldTransform xf = lpGetTransform( w, torso );
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
	r->crawling = able < g->walkingLegs || r->stuck;
	float goal = r->def.standHeight * ( 1.0f - g->crouchDepth * lpClampFloat( r->control.crouch, 0.0f, 1.0f ) );
	goal *= 1.0f - g->weakSag * ( 1.0f - lpClampFloat( weakest, 0.0f, 1.0f ) );
	goal = lpMinFloat( goal, shortest - g->reachSpare );
	goal = r->crawling ? g->bellyHeight : lpMaxFloat( goal, g->bellyHeight );
	bool still = lpControlStill( &r->control );

	// Idle: the targets stay frozen until the controls change or something knocks it well off its stance
	if ( r->idle )
	{
		bool knocked = lpAbsFloat( r->height - goal ) > g->knockHeight * g->calmHeight || tilt > g->knockTilt * g->calmTilt;
		if ( r->controlChanged == false && ( lpPhys_IsAwake( w->phys, torso->id ) == false || knocked == false ) )
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
	float top = lpMinFloat( g->maxSpeed, g->keepUp * 2.0f * g->land * g->stride / g->swingTime );
	// Fewer legs swing in more turns, each foot carrying longer (by default: on five it keeps half its pace, on four a third)
	top *= r->crawling ? g->crawlSpeed : g->legPace[able];
	lpVec3 velocity = lpMulSV( top, lpSub( lpMulSV( r->control.forward, heading ), lpMulSV( r->control.strafe, side ) ) );
	float spin = -r->control.turn * g->maxTurn; // rad/s about up, left for positive
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
			stretch[i] = lpLength( off ) / g->stride;
			lpVec3 drift = lpNeg( lpAdd( velocity, lpCross( lpMulSV( spin, up ), lpSubPos( feet[i], old.p ) ) ) );
			float rate = lpLength( drift );
			float half = lpHalfStep( r, rate ); // it landed about this far ahead; it steps once about this far behind
			urgency[i] = rate > 0.05f * g->maxSpeed ? lpDot( off, drift ) / ( rate * lpMaxFloat( half, 0.02f ) ) : stretch[i];
			if ( stretch[i] >= 1.0f )
			{
				urgency[i] = lpMaxFloat( urgency[i], 1.0f + stretch[i] ); // past its stride any way (a slip, a rock): due first
			}
			worst = limb->planted ? lpMaxFloat( worst, stretch[i] ) : worst;
		}
	}
	float pace = lpClampFloat( 1.0f - ( worst - 1.0f ) / g->overreach, 0.0f, 1.0f );
	lpVec3 wanted = velocity; // as commanded, before its feet held it back
	velocity = lpMulSV( pace, velocity );
	spin *= pace;

	// Swings under way: their time, their aim (re-aimed across the ground as the torso goes; the ground's height from the
	// casts), the second cast, landing
	lpVec3 moving = lpFlatten( lpPhys_GetLinearVelocity( w->phys, torso->id ), up ); // the torso, as it actually is
	float turning = lpDot( lpPhys_GetAngularVelocity( w->phys, torso->id ), up );
	for ( int i = 0; i < r->limbCount; ++i )
	{
		lpLimb* limb = r->limbs + i;
		if ( limb->swinging == false )
		{
			continue;
		}
		limb->swingClock += timeStep;
		float s = limb->swingClock / g->swingTime;
		float remaining = lpMaxFloat( g->swingTime - limb->swingClock, 0.0f );
		lpPos aim = lpAimLanding( r, limb, old, heading, side, velocity, spin, moving, turning, remaining, up );
		limb->landing = lpOffsetPos( aim, lpMulSV( lpDot( lpSubPos( limb->landing, aim ), up ), up ) );
		if ( s >= g->lateCast && limb->castLate == false && w->stats.footCasts < w->def.maxFootCastsPerStep )
		{
			limb->castLate = true;
			lpCastFoothold( w, r, limb, up );
		}
		if ( s >= 1.0f )
		{
			// Down: it holds its landing point until the foot gets there (the servos lag the arc a little)
			limb->swinging = false;
			limb->planted = true;
			limb->hold = lpSwingPoint( limb, 1.0f, g->stepHeight, up );
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
			if ( limb->arrived == false && ( off < g->arrived || limb->holdClock > g->arriveTime ) )
			{
				limb->hold = feet[i];
				limb->arrived = true;
			}
			else if ( limb->arrived && off > g->slipped )
			{
				limb->hold = feet[i]; // it slipped, or was knocked: held where it is now
			}
			else if ( limb->arrived )
			{
				// The servos hold their angles as stiffly as they can, so feet held a centimetre apart from where the legs'
				// geometry has them would push against each other through the ground: the hold eases toward the model's
				// foot, slowly enough to push the torso along
				limb->hold = lpOffsetPos( limb->hold, lpMulSV( timeStep / g->easeTime, lpSubPos( feet[i], limb->hold ) ) );
			}
		}
	}

	// Liftoffs: the most stretched foot first; its nearest able neighbours around the body planted, and the centre of mass
	// inside the other planted feet by the margin, now and where the torso will be when it lands
	lpPos center = lpRigCenter( w, r );
	lpPos later = lpOffsetPos( center, lpMulSV( g->swingTime, moving ) );

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
			bool steady = r->crawling || lpSupportMargin( feet, planted, r->limbCount, center, up ) >= g->margin;
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
	float threshold = still ? g->tidy : g->due;
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
			bool settled = limb->holdClock >= g->minStance; // just set down, it carries a moment first
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
		if ( margin < g->margin && r->crawling == false ) // crawling, its belly holds it
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
	// Told to move and getting nowhere (its feet may still step: one leg left on a side lifts nothing and the others
	// shuffle) for a while, it is stuck
	float asked = lpLength( wanted );
	float made = asked > 1e-3f ? lpDot( moving, wanted ) / asked : 0.0f;
	bool getting = asked < 1e-3f || made > g->stallPace * asked; // turning on the spot asks for no speed
	r->stall = still == false && getting == false ? r->stall + 1 : 0;
	r->stuck = r->stuck || r->stall >= g->stallTicks;

	// The desired pose: turned and moved by the controls (and leaning for balance), never far ahead of the torso, level,
	// climbing toward its height over the planted feet
	lpVec3 measured = lpFlatten( lpRotateVector( xf.q, r->forward ), up );
	measured = lpLengthSquared( measured ) > 1e-8f ? lpNormalize( measured ) : heading;
	float yawError = lpAtan2( lpDot( lpCross( measured, heading ), up ), lpDot( measured, heading ) );
	float yaw = lpClampFloat( yawError + spin * timeStep, -g->leadTurn, g->leadTurn ) - yawError;
	lpCosSin cs = lpComputeCosSin( yaw );
	heading = lpAdd( lpMulSV( cs.cosine, heading ), lpMulSV( cs.sine, side ) );
	lpVec3 ahead = lpMulAdd( lpSubPos( old.p, xf.p ), timeStep, lpAdd( velocity, shift ) ); // from the torso's frame
	// It may run a little ahead of the torso (the servos' gains pull the torso along), and further behind (a torso that
	// runs ahead is held back instead of dragging it along)
	lpVec3 flat = lpFlatten( ahead, up );
	float lead = lpLength( flat );
	if ( lead > g->lag )
	{
		flat = lpMulSV( g->lag / lead, flat );
	}
	float pull = lpLength( velocity );
	float forward = pull > 1e-4f ? lpDot( flat, velocity ) / pull : 0.0f;
	if ( forward > g->lead )
	{
		flat = lpMulAdd( flat, ( g->lead - forward ) / pull, velocity );
	}
	// Falling (dropped, or its legs knocked from under it), it follows the torso down: pushed back up to where it was, it
	// would land and spring up again
	float height = lpDot( lpSubPos( old.p, xf.p ), up ) - support; // the desired pose's, over the feet
	if ( lpDot( lpPhys_GetLinearVelocity( w->phys, torso->id ), up ) < -g->falling )
	{
		height = lpMinFloat( height, r->height + g->lead );
	}
	float climb = lpClampFloat( goal - height, -g->climb * timeStep, g->climb * timeStep );
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
		lpVec3 target = lpMulAdd( lpSub( limb->neutral, lpMulSV( lpDot( limb->neutral, r->up ), r->up ) ), -g->tuck * r->def.standHeight,
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
			float s0 = limb->swingClock / g->swingTime;
			float s1 = ( limb->swingClock + timeStep ) / g->swingTime;
			foot = lpSwingPoint( limb, s1, g->stepHeight, up );
			motion = lpMulSV( 1.0f / timeStep, lpSubPos( foot, lpSwingPoint( limb, s0, g->stepHeight, up ) ) );
		}
		lpDriveFoot( w, r, limb, foot, motion, linear, angular, limb->reaching ? g->strike : 0.0f );
	}

	// Still, settled and every foot down for a moment: freeze the targets
	bool swinging = false;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		swinging = swinging || r->limbs[i].swinging || r->limbs[i].reaching;
	}
	bool settled = lpAbsFloat( height - goal ) < 0.001f && lpAbsFloat( r->height - goal ) < g->calmHeight &&
				   tilt < g->calmTilt && swinging == false && waited == false;
	r->calm = still && settled ? r->calm + 1 : 0;
	if ( r->calm >= g->calmTicks )
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
