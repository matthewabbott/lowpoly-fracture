// SPDX-License-Identifier: MIT
// Rigs: bodies that stand and walk on limbs.
//
// A rig makes no physics. Its limbs are chains of motorised hinge links from the torso outward, made by the game; the
// rig only steers their servos (targets and a feedforward speed). A link's inner end is on the body the link before it
// turns, and its hinge angle turns the outer end's frame about the inner end's z. A piece's body frame never changes,
// so the chain's kinematics (the end frames, composed with the measured angles) hold through splits and joint rebuilds.
//
// Each step, after the supply update and before the servos are driven (lpStepRigs):
// - capability: which links of each limb are still on in a chain from the torso, the weakest servo's share of its
//   torque (damage and supply), and the foot: as created while it is still on a piece, else the far end of the last
//   segment left along its axis (a stump stands on its end, as a peg);
// - the torso is the body holding most limbs' first links (a rig never names a body);
// - the gait (gait.c): which feet step and where, the torso's desired pose, and each limb's targets, the IK solution
//   that puts its foot where the gait wants it from that pose (damped least squares, a fixed number of iterations
//   warm-started from the measured angles, inside the hinges' limits), with feedforward speeds for the joint motion the
//   pose and the swing ask for, so servos with different gains or speed caps still move together.
// Everything is in index order with a fixed iteration count: it depends on nothing but the simulation state.

#include "world.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#define LP_RIG_IK_ITERATIONS 6
#define LP_RIG_IK_DAMPING 0.05f	 // m: damped least squares keeps a straight limb or an unreachable target calm
#define LP_RIG_LIMIT_MARGIN 0.05f // rad: targets stay inside a hinge's limits (pressed on a limit reads as load)
#define LP_RIG_WEAK 0.2f		  // a limb whose weakest servo has less of its torque than this cannot stand
#define LP_RIG_REACH 0.25f		 // the foot as created must be this close to a piece of its body
#define LP_RIG_STUMP 0.6f		 // a limb that reaches less than this share of the stand height below the torso cannot stand
#define LP_RIG_TOUCH 0.35f		 // m: a reaching foot touches what it is in contact with this near it

lpRigDef lpDefaultRigDef( void )
{
	lpRigDef def = { 0 };
	def.body = -1;
	def.forward = (lpVec3){ 0.0f, 0.0f, 1.0f };
	def.up = (lpVec3){ 0.0f, 1.0f, 0.0f };
	def.crouchDepth = 0.35f;
	def.stepHeight = 0.35f;
	def.stride = 0.5f;
	def.maxSpeed = 2.5f;
	def.maxTurn = 0.8f;
	def.swingTime = 0.4f;
	def.margin = 0.15f;
	return def;
}

static int lpEndBodyIndex( const lpWorld* w, const lpLink* l, int end )
{
	int piece = l->ends[end].piece;
	return piece >= 0 ? w->pieces.data[piece].body : -1;
}

static bool lpIsServo( const lpLink* l )
{
	return l->def.type == lp_linkHinge && ( l->def.motor.maxTorque > 0.0f || l->def.motor.holdTorque > 0.0f );
}

lpVec3 lpRigWorldUp( const lpWorld* w, const lpRig* r, lpQuat torso )
{
	lpVec3 g = lpPhys_GetGravity( w->phys );
	float length = lpLength( g );
	return length > 1e-6f ? lpMulSV( -1.0f / length, g ) : lpRotateVector( torso, r->up );
}

// ---- kinematics ----

// The foot of the limb's first `joints` links in the torso frame, with the joint angles q; each joint's axis (times its
// sign) and point go to axes and origins
lpVec3 lpLimbForward( const lpWorld* w, const lpLimb* limb, int joints, const float* q, lpVec3 foot, lpVec3* axes, lpVec3* origins )
{
	lpTransform x = lpTransform_identity;
	for ( int k = 0; k < joints; ++k )
	{
		const lpLink* l = w->links.data + limb->def.links[k];
		int prox = limb->prox[k];
		lpTransform inner = lpMulTransforms( x, l->ends[prox].frame );
		lpTransform outer = l->ends[1 - prox].frame;
		float sign = prox == 0 ? 1.0f : -1.0f;
		origins[k] = inner.p;
		axes[k] = lpMulSV( sign, lpRotateVector( inner.q, (lpVec3){ 0.0f, 0.0f, 1.0f } ) );
		lpCosSin cs = lpComputeCosSin( 0.5f * sign * q[k] );
		lpQuat turned = lpMulQuat( inner.q, (lpQuat){ { 0.0f, 0.0f, cs.sine }, cs.cosine } );
		// The outer body's frame: its end frame sits on the turned inner frame
		x.q = lpMulQuat( turned, lpConjugate( outer.q ) );
		x.p = lpSub( inner.p, lpRotateVector( x.q, outer.p ) );
	}
	return lpTransformPoint( x, foot );
}

void lpLimbSpeeds( int joints, const lpVec3* axes, const lpVec3* origins, lpVec3 foot, lpVec3 velocity, float* out )
{
	lpVec3 columns[LP_MAX_LIMB_JOINTS];
	float d = LP_RIG_IK_DAMPING * LP_RIG_IK_DAMPING;
	lpMatrix3 a = { { d, 0.0f, 0.0f }, { 0.0f, d, 0.0f }, { 0.0f, 0.0f, d } };
	for ( int k = 0; k < joints; ++k )
	{
		lpVec3 c = lpCross( axes[k], lpSub( foot, origins[k] ) );
		columns[k] = c;
		a.cx = lpMulAdd( a.cx, c.x, c );
		a.cy = lpMulAdd( a.cy, c.y, c );
		a.cz = lpMulAdd( a.cz, c.z, c );
	}
	lpVec3 y = lpSolve3( a, velocity );
	for ( int k = 0; k < joints; ++k )
	{
		out[k] = lpDot( columns[k], y );
	}
}

// IK: the angles (q, warm on entry) that put the foot at target (torso frame) within the limits; returns how far short
float lpLimbIK( const lpWorld* w, const lpLimb* limb, int joints, lpVec3 foot, lpVec3 target, float* q )
{
	lpVec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
	for ( int iteration = 0; iteration < LP_RIG_IK_ITERATIONS; ++iteration )
	{
		lpVec3 at = lpLimbForward( w, limb, joints, q, foot, axes, origins );
		float dq[LP_MAX_LIMB_JOINTS];
		lpLimbSpeeds( joints, axes, origins, at, lpSub( target, at ), dq );
		for ( int k = 0; k < joints; ++k )
		{
			q[k] = lpClampFloat( q[k] + dq[k], limb->lower[k], limb->upper[k] );
		}
	}
	return lpLength( lpSub( target, lpLimbForward( w, limb, joints, q, foot, axes, origins ) ) );
}

// ---- capability ----

// The foot on the limb's tip body: as created while it is still on a piece (and the chain whole), else the far end of
// the last segment along its axis, from its inner joint through its centroid (a stump stands on its end)
static void lpFindFoot( const lpWorld* w, lpLimb* limb )
{
	const lpBody* b = w->bodies.data + limb->tipBody;
	if ( limb->joints == limb->def.linkCount )
	{
		for ( int k = 0; k < b->pieces.count; ++k )
		{
			if ( lpShape_SignedDistance( w->pieces.data[b->pieces.data[k]].shape, limb->defFoot ) <= LP_RIG_REACH )
			{
				limb->foot = limb->defFoot;
				return;
			}
		}
	}
	const lpLink* last = w->links.data + limb->def.links[limb->joints - 1];
	lpVec3 joint = last->ends[1 - limb->prox[limb->joints - 1]].frame.p;
	lpVec3 centroid = lpVec3_zero;
	float volume = 0.0f;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpShape* shape = w->pieces.data[b->pieces.data[k]].shape;
		centroid = lpMulAdd( centroid, shape->volume, shape->centroid );
		volume += shape->volume;
	}
	lpVec3 axis = volume > 0.0f ? lpSub( lpMulSV( 1.0f / volume, centroid ), joint ) : lpVec3_zero;
	float length = lpLength( axis );
	if ( length < 1e-4f )
	{
		limb->foot = joint;
		return;
	}
	axis = lpMulSV( 1.0f / length, axis );
	float extent = 0.0f;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpShape* shape = w->pieces.data[b->pieces.data[k]].shape;
		for ( int v = 0; v < shape->vertexCount; ++v )
		{
			extent = lpMaxFloat( extent, lpDot( lpSub( shape->vertices[v], joint ), axis ) );
		}
	}
	limb->foot = lpMulAdd( joint, extent, axis );
}

// How far below the torso's frame (along its up) the limb's foot reaches at its neutral point: IK toward a point far
// below it, from where the joints are
static float lpLimbDepth( const lpWorld* w, const lpLimb* limb, lpVec3 up )
{
	float q[LP_MAX_LIMB_JOINTS];
	for ( int k = 0; k < limb->joints; ++k )
	{
		q[k] = w->links.data[limb->def.links[k]].angle;
	}
	lpVec3 target = lpMulAdd( lpSub( limb->neutral, lpMulSV( lpDot( limb->neutral, up ), up ) ), -4.0f, up );
	lpLimbIK( w, limb, limb->joints, limb->foot, target, q );
	lpVec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
	return -lpDot( lpLimbForward( w, limb, limb->joints, q, limb->foot, axes, origins ), up );
}

// Which links are on in a chain from the torso, the weakest servo, the foot (and how deep it reaches)
static void lpLimbCapability( const lpWorld* w, lpLimb* limb, lpVec3 up )
{
	int inner = -1;
	limb->joints = 0;
	limb->rootBody = -1;
	limb->strength = 1.0f;
	for ( int k = 0; k < limb->def.linkCount; ++k )
	{
		const lpLink* l = w->links.data + limb->def.links[k];
		if ( l->alive == false || l->generation != limb->gen[k] )
		{
			break;
		}
		int in = lpEndBodyIndex( w, l, limb->prox[k] );
		int out = lpEndBodyIndex( w, l, 1 - limb->prox[k] );
		if ( k > 0 && in != inner )
		{
			break; // the segment between this joint and the last came apart
		}
		if ( k == 0 )
		{
			limb->rootBody = in;
		}
		float cap = l->def.motor.maxTorque > 0.0f ? lpMotorDrive( w, l ) / l->def.motor.maxTorque : 0.0f; // not what brakes hold
		limb->strength = lpMinFloat( limb->strength, cap );
		inner = out;
		limb->joints = k + 1;
	}
	if ( limb->joints == 0 )
	{
		limb->tipBody = -1;
		limb->strength = 0.0f;
		return;
	}
	const lpBody* tip = w->bodies.data + inner;
	if ( inner != limb->tipBody || tip->generation != limb->tipGeneration || tip->topology != limb->tipTopology ||
		 limb->joints != limb->tipJoints )
	{
		limb->tipBody = inner;
		limb->tipGeneration = tip->generation;
		limb->tipTopology = tip->topology;
		limb->tipJoints = limb->joints;
		lpFindFoot( w, limb );
		limb->depth = lpLimbDepth( w, limb, up );
	}
}

// A limb can stand if some joint on it swings its foot up and down (not all about the vertical) and it is strong enough
static bool lpLimbCanLift( const lpWorld* w, const lpLimb* limb, lpVec3 up )
{
	if ( limb->strength < LP_RIG_WEAK )
	{
		return false;
	}
	for ( int k = 0; k < limb->joints; ++k )
	{
		const lpLink* l = w->links.data + limb->def.links[k];
		int body = lpEndBodyIndex( w, l, limb->prox[k] );
		lpQuat q = lpMulQuat( lpGetTransform( w, w->bodies.data + body ).q, l->ends[limb->prox[k]].frame.q );
		if ( lpAbsFloat( lpDot( lpRotateVector( q, (lpVec3){ 0.0f, 0.0f, 1.0f } ), up ) ) < 0.7f )
		{
			return true;
		}
	}
	return false;
}

lpPos lpFootWorld( const lpWorld* w, const lpLimb* limb )
{
	return lpTransformWorldPoint( lpGetTransform( w, w->bodies.data + limb->tipBody ), limb->foot );
}

// ---- creation ----

int lpCreateRig( lpWorld* w, const lpRigDef* def )
{
	lpGuardFp( w ); // computes in float between steps, on the caller's thread
	if ( def->body < 0 || def->body >= w->bodies.count || def->limbs == NULL || def->limbCount < 1 ||
		 def->limbCount > LP_MAX_RIG_LIMBS )
	{
		return -1;
	}
	const lpBody* torso = w->bodies.data + def->body;
	if ( torso->alive == false || LP_PHYS_NULL( torso->id ) || lpPhys_IsDynamic( w->phys, torso->id ) == false )
	{
		return -1;
	}
	lpRig r;
	memset( &r, 0, sizeof( r ) );
	for ( int i = 0; i < def->limbCount; ++i )
	{
		lpLimb* limb = r.limbs + i;
		limb->def = def->limbs[i];
		if ( limb->def.linkCount < 1 || limb->def.linkCount > LP_MAX_LIMB_JOINTS )
		{
			return -1;
		}
		int inner = def->body;
		for ( int k = 0; k < limb->def.linkCount; ++k )
		{
			int index = limb->def.links[k];
			if ( index < 0 || index >= w->links.count || w->links.data[index].alive == false || lpIsServo( w->links.data + index ) == false )
			{
				return -1;
			}
			const lpLink* l = w->links.data + index;
			if ( lpEndBodyIndex( w, l, 0 ) == inner )
			{
				limb->prox[k] = 0;
			}
			else if ( lpEndBodyIndex( w, l, 1 ) == inner )
			{
				limb->prox[k] = 1;
			}
			else
			{
				return -1;
			}
			inner = lpEndBodyIndex( w, l, 1 - limb->prox[k] );
			limb->gen[k] = l->generation;
			bool limited = l->def.lowerAngle < l->def.upperAngle;
			float mid = 0.5f * ( l->def.lowerAngle + l->def.upperAngle );
			limb->lower[k] = limited ? lpMinFloat( l->def.lowerAngle + LP_RIG_LIMIT_MARGIN, mid ) : -1e6f;
			limb->upper[k] = limited ? lpMaxFloat( l->def.upperAngle - LP_RIG_LIMIT_MARGIN, mid ) : 1e6f;
			limb->q[k] = l->target;
		}
		limb->defFoot = lpInvTransformWorldPoint( lpGetTransform( w, w->bodies.data + inner ), limb->def.foot );
		limb->tipBody = -1;
		lpLimbCapability( w, limb, lpInvRotateVector( lpPhys_GetTransform( w->phys, torso->id ).q, lpNormalize( def->up ) ) );
	}

	lpWorldTransform xf = lpPhys_GetTransform( w->phys, torso->id );
	lpVec3 forward = lpNormalize( def->forward );
	lpVec3 up = lpNormalize( lpSub( def->up, lpMulSV( lpDot( def->up, forward ), forward ) ) );
	r.def = *def;
	r.def.limbs = NULL;
	r.forward = lpInvRotateVector( xf.q, forward );
	r.up = lpInvRotateVector( xf.q, up );
	r.alive = true;
	r.body = def->body;
	r.limbCount = def->limbCount;
	r.desired = xf;
	for ( int i = 0; i < r.limbCount; ++i )
	{
		lpLimb* limb = r.limbs + i;
		limb->neutral = limb->joints > 0 ? lpInvTransformWorldPoint( xf, lpFootWorld( w, limb ) ) : lpVec3_zero;
		limb->depth = limb->joints > 0 ? lpLimbDepth( w, limb, r.up ) : 0.0f; // now that it has its neutral point
		limb->planted = limb->joints > 0; // standing as built
		limb->hold = limb->joints > 0 ? lpFootWorld( w, limb ) : xf.p;
		limb->holdClock = 0.0f; // built above the ground, it is held where it lands
		limb->arrived = false;
		limb->touching = -1;
		limb->groundPiece = -1;
	}
	if ( r.def.standHeight <= 0.0f )
	{
		// As created: the torso's frame above its feet
		lpVec3 worldUp = lpRigWorldUp( w, &r, xf.q );
		float sum = 0.0f;
		for ( int i = 0; i < r.limbCount; ++i )
		{
			sum += lpDot( lpSubPos( xf.p, lpFootWorld( w, r.limbs + i ) ), worldUp );
		}
		r.def.standHeight = sum / (float)r.limbCount;
	}
	if ( r.def.bellyHeight <= 0.0f )
	{
		r.def.bellyHeight = 0.2f * r.def.standHeight;
	}
	lpArray_Push( w->rigs, r );
	return w->rigs.count - 1;
}

// ---- the step ----

static int lpFindTouch( lpWorld* w, const lpRig* r, const lpLimb* limb );

void lpStepRigs( lpWorld* w, float timeStep )
{
	for ( int ri = 0; ri < w->rigs.count; ++ri )
	{
		lpRig* r = w->rigs.data + ri;
		if ( r->alive == false )
		{
			continue;
		}
		// Capability, then the torso: the body holding most first links (the lowest index on a tie)
		int best = 0;
		r->body = -1;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			lpLimbCapability( w, r->limbs + i, r->up );
		}
		for ( int i = 0; i < r->limbCount; ++i )
		{
			int body = r->limbs[i].rootBody;
			int held = 0;
			for ( int j = 0; j < r->limbCount && body >= 0; ++j )
			{
				held += r->limbs[j].rootBody == body ? 1 : 0;
			}
			if ( held > best || ( held == best && held > 0 && body < r->body ) )
			{
				best = held;
				r->body = body;
			}
		}
		if ( r->body < 0 || LP_PHYS_NULL( w->bodies.data[r->body].id ) )
		{
			r->body = -1;
			continue;
		}
		lpWorldTransform xf = lpGetTransform( w, w->bodies.data + r->body );
		lpVec3 worldUp = lpRigWorldUp( w, r, xf.q );
		for ( int i = 0; i < r->limbCount; ++i )
		{
			lpLimb* limb = r->limbs + i;
			limb->attached = limb->joints > 0 && limb->rootBody == r->body;
			// A stump too short to reach the ground from near its stance (a femur without its tibia) is held up instead
			limb->able = limb->attached && lpLimbCanLift( w, limb, worldUp ) && limb->depth >= LP_RIG_STUMP * r->def.standHeight;
			limb->reach = 0.0f;
			if ( limb->joints > 0 )
			{
				const lpLink* root = w->links.data + limb->def.links[0];
				lpWorldTransform rootXf = lpGetTransform( w, w->bodies.data + limb->rootBody );
				lpPos joint = lpTransformWorldPoint( rootXf, root->ends[limb->prox[0]].frame.p );
				limb->reach = lpLength( lpSubPos( lpFootWorld( w, limb ), joint ) );
			}
		}
		lpWalkRig( w, r, timeStep );
		r->controlChanged = false;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			lpLimb* limb = r->limbs + i;
			limb->touching = limb->reaching && limb->attached ? lpFindTouch( w, r, limb ) : -1;
		}
	}
}

// The piece a reaching limb's foot touches: a contact of its tip body within reach of the foot, on anything but the rig.
// Something loose comes before something fixed (a crate before the ground it rests on), then the nearest.
static int lpFindTouch( lpWorld* w, const lpRig* r, const lpLimb* limb )
{
	const lpPhysContact* contacts;
	int count = lpPhys_GetBodyContacts( w->phys, w->bodies.data[limb->tipBody].id, &contacts );
	lpPos foot = lpFootWorld( w, limb );
	int best = -1;
	bool bestFixed = true;
	float nearest = LP_RIG_TOUCH * LP_RIG_TOUCH;
	for ( int k = 0; k < count; ++k )
	{
		const lpPhysContact* c = contacts + k;
		int piece = c->other;
		if ( piece < 0 || w->pieces.data[piece].body == r->body )
		{
			continue;
		}
		bool ours = false;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			ours = ours || w->pieces.data[piece].body == r->limbs[i].tipBody || w->pieces.data[piece].body == r->limbs[i].rootBody;
		}
		if ( ours )
		{
			continue;
		}
		bool fixed = lpPhys_IsDynamic( w->phys, w->bodies.data[w->pieces.data[piece].body].id ) == false;
		lpVec3 d = lpSubPos( c->point, foot );
		float d2 = lpDot( d, d );
		bool closer = d2 < nearest || ( d2 == nearest && piece < best );
		bool better = best < 0 ? d2 < nearest : ( fixed != bestFixed ? bestFixed : closer );
		if ( c->separation < 0.05f && d2 < LP_RIG_TOUCH * LP_RIG_TOUCH && better )
		{
			nearest = d2;
			best = piece;
			bestFixed = fixed;
		}
	}
	return best;
}

// ---- API ----

void lpWorld_SetLimbTarget( lpWorld* w, int rig, int limb, bool active, lpPos point )
{
	if ( rig < 0 || rig >= w->rigs.count || limb < 0 || limb >= w->rigs.data[rig].limbCount )
	{
		return;
	}
	lpRig* r = w->rigs.data + rig;
	lpLimb* l = r->limbs + limb;
	if ( l->reachWanted != active || ( active && ( point.x != l->reachPoint.x || point.y != l->reachPoint.y || point.z != l->reachPoint.z ) ) )
	{
		l->reachWanted = active;
		l->reachPoint = active ? point : l->reachPoint;
		r->controlChanged = true; // wakes it from idle
	}
}

void lpWorld_SetRigControl( lpWorld* w, int rig, const lpRigControl* control )
{
	if ( rig < 0 || rig >= w->rigs.count )
	{
		return;
	}
	lpRig* r = w->rigs.data + rig;
	lpRigControl c = { lpClampFloat( control->forward, -1.0f, 1.0f ), lpClampFloat( control->strafe, -1.0f, 1.0f ),
					   lpClampFloat( control->turn, -1.0f, 1.0f ), lpClampFloat( control->crouch, 0.0f, 1.0f ) };
	if ( c.forward != r->control.forward || c.strafe != r->control.strafe || c.turn != r->control.turn || c.crouch != r->control.crouch )
	{
		r->control = c;
		r->controlChanged = true;
	}
}

lpRigState lpWorld_GetRigState( const lpWorld* w, int rig )
{
	lpRigState s = { 0 };
	s.body = -1;
	if ( rig < 0 || rig >= w->rigs.count )
	{
		return s;
	}
	const lpRig* r = w->rigs.data + rig;
	s.alive = r->alive;
	s.body = r->body;
	s.limbCount = r->limbCount;
	s.idle = r->idle;
	s.crawling = r->crawling;
	s.height = r->height;
	s.control = r->control;
	for ( int i = 0; i < r->limbCount; ++i )
	{
		s.attached += r->limbs[i].attached ? 1 : 0;
		s.able += r->limbs[i].able ? 1 : 0;
		s.planted += r->limbs[i].planted ? 1 : 0;
	}
	if ( s.body >= 0 )
	{
		lpPhysBody id = w->bodies.data[s.body].id;
		lpWorldTransform xf = lpPhys_GetTransform( w->phys, id );
		s.position = xf.p;
		s.forward = lpRotateVector( xf.q, r->forward );
		s.up = lpRotateVector( xf.q, r->up );
		s.speed = lpDot( lpPhys_GetLinearVelocity( w->phys, id ), s.forward );
	}
	return s;
}

int lpWorld_GetRigCapacity( const lpWorld* w )
{
	return w->rigs.count;
}

lpLimbState lpWorld_GetLimbState( const lpWorld* w, int rig, int limb )
{
	lpLimbState s = { 0 };
	s.footBody = -1;
	if ( rig < 0 || rig >= w->rigs.count || limb < 0 || limb >= w->rigs.data[rig].limbCount )
	{
		return s;
	}
	const lpLimb* l = w->rigs.data[rig].limbs + limb;
	s.attached = l->attached;
	s.able = l->able;
	s.planted = l->planted;
	s.swinging = l->swinging;
	s.joints = l->joints;
	s.strength = l->strength;
	s.reach = l->reach;
	s.depth = l->depth;
	s.reaching = l->reaching;
	s.touching = l->touching;
	if ( l->tipBody >= 0 && w->bodies.data[l->tipBody].alive )
	{
		s.footBody = l->tipBody;
		s.foot = lpFootWorld( w, l );
	}
	return s;
}

// ---- hash, validation ----

uint64_t lpHashRigs( const lpWorld* w, uint64_t h )
{
	for ( int ri = 0; ri < w->rigs.count; ++ri )
	{
		const lpRig* r = w->rigs.data + ri;
		// Field by field: no struct padding in the hash
		float control[4] = { r->control.forward, r->control.strafe, r->control.turn, r->control.crouch };
		float pose[4] = { r->desired.q.v.x, r->desired.q.v.y, r->desired.q.v.z, r->desired.q.s };
		uint8_t flags[4] = { r->alive ? 1 : 0, r->idle ? 1 : 0, r->crawling ? 1 : 0, r->stuck ? 1 : 0 };
		h = lpHashBytes( h, control, sizeof( control ) );
		h = lpHashBytes( h, &r->desired.p, sizeof( r->desired.p ) );
		h = lpHashBytes( h, pose, sizeof( pose ) );
		h = lpHashBytes( h, flags, sizeof( flags ) );
		h = lpHashBytes( h, &r->calm, sizeof( r->calm ) );
		int counters[2] = { r->stall, r->ableSeen };
		h = lpHashBytes( h, counters, sizeof( counters ) );
		h = lpHashBytes( h, &r->body, sizeof( r->body ) );
		for ( int i = 0; i < r->limbCount; ++i )
		{
			const lpLimb* limb = r->limbs + i;
			int ints[4] = { limb->joints, limb->tipBody, limb->groundPiece, limb->tipJoints };
			uint32_t tip[2] = { limb->tipGeneration, limb->tipTopology };
			uint8_t limbFlags[9] = { limb->attached ? 1 : 0, limb->able ? 1 : 0,	   limb->planted ? 1 : 0,
									 limb->swinging ? 1 : 0, limb->castLate ? 1 : 0, limb->grounded ? 1 : 0,
									 limb->arrived ? 1 : 0,	 limb->reachWanted ? 1 : 0,	   limb->reaching ? 1 : 0 };
			h = lpHashBytes( h, ints, sizeof( ints ) );
			h = lpHashBytes( h, tip, sizeof( tip ) );
			h = lpHashBytes( h, limbFlags, sizeof( limbFlags ) );
			h = lpHashBytes( h, &limb->depth, sizeof( limb->depth ) ); // gates able and the stand height
			h = lpHashBytes( h, &limb->neutral, sizeof( limb->neutral ) );
			h = lpHashBytes( h, &limb->foot, sizeof( limb->foot ) );
			h = lpHashBytes( h, limb->q, sizeof( limb->q ) );
			h = lpHashBytes( h, &limb->swingClock, sizeof( limb->swingClock ) );
			h = lpHashBytes( h, &limb->liftoff, sizeof( limb->liftoff ) );
			h = lpHashBytes( h, &limb->landing, sizeof( limb->landing ) );
			h = lpHashBytes( h, &limb->hold, sizeof( limb->hold ) );
			h = lpHashBytes( h, &limb->holdClock, sizeof( limb->holdClock ) );
			h = lpHashBytes( h, &limb->reachPoint, sizeof( limb->reachPoint ) );
			h = lpHashBytes( h, &limb->touching, sizeof( limb->touching ) );
		}
	}
	return h;
}

bool lpValidateRigs( const lpWorld* w )
{
	for ( int ri = 0; ri < w->rigs.count; ++ri )
	{
		const lpRig* r = w->rigs.data + ri;
		for ( int i = 0; i < r->limbCount; ++i )
		{
			const lpLimb* limb = r->limbs + i;
			// Capability is found at the start of a step; the body under a foot may have gone since
			if ( limb->joints < 0 || limb->joints > limb->def.linkCount || limb->tipBody >= w->bodies.count ||
				 ( limb->joints > 0 && limb->tipBody < 0 ) )
			{
				fprintf( stderr, "lpWorld_Validate: rig %d limb %d has %d joints, its foot on body %d\n", ri, i, limb->joints,
						 limb->tipBody );
				return false;
			}
		}
	}
	return true;
}

void lpFreeRigs( lpWorld* w )
{
	lpArray_Free( w->rigs );
}
