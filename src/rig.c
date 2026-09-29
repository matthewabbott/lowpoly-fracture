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

lpRigDef lpDefaultRigDef( void )
{
	lpRigDef def = { 0 };
	def.body = -1;
	def.forward = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	def.up = (b3Vec3){ 0.0f, 1.0f, 0.0f };
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

b3Vec3 lpRigWorldUp( const lpWorld* w, const lpRig* r, b3Quat torso )
{
	b3Vec3 g = b3World_GetGravity( w->def.physics );
	float length = b3Length( g );
	return length > 1e-6f ? b3MulSV( -1.0f / length, g ) : b3RotateVector( torso, r->up );
}

// ---- kinematics ----

// The foot of the limb's first `joints` links in the torso frame, with the joint angles q; each joint's axis (times its
// sign) and point go to axes and origins
b3Vec3 lpLimbForward( const lpWorld* w, const lpLimb* limb, int joints, const float* q, b3Vec3 foot, b3Vec3* axes, b3Vec3* origins )
{
	b3Transform x = b3Transform_identity;
	for ( int k = 0; k < joints; ++k )
	{
		const lpLink* l = w->links.data + limb->def.links[k];
		int prox = limb->prox[k];
		b3Transform inner = b3MulTransforms( x, l->ends[prox].frame );
		b3Transform outer = l->ends[1 - prox].frame;
		float sign = prox == 0 ? 1.0f : -1.0f;
		origins[k] = inner.p;
		axes[k] = b3MulSV( sign, b3RotateVector( inner.q, (b3Vec3){ 0.0f, 0.0f, 1.0f } ) );
		b3CosSin cs = b3ComputeCosSin( 0.5f * sign * q[k] );
		b3Quat turned = b3MulQuat( inner.q, (b3Quat){ { 0.0f, 0.0f, cs.sine }, cs.cosine } );
		// The outer body's frame: its end frame sits on the turned inner frame
		x.q = b3MulQuat( turned, b3Conjugate( outer.q ) );
		x.p = b3Sub( inner.p, b3RotateVector( x.q, outer.p ) );
	}
	return b3TransformPoint( x, foot );
}

void lpLimbSpeeds( int joints, const b3Vec3* axes, const b3Vec3* origins, b3Vec3 foot, b3Vec3 velocity, float* out )
{
	b3Vec3 columns[LP_MAX_LIMB_JOINTS];
	float d = LP_RIG_IK_DAMPING * LP_RIG_IK_DAMPING;
	b3Matrix3 a = { { d, 0.0f, 0.0f }, { 0.0f, d, 0.0f }, { 0.0f, 0.0f, d } };
	for ( int k = 0; k < joints; ++k )
	{
		b3Vec3 c = b3Cross( axes[k], b3Sub( foot, origins[k] ) );
		columns[k] = c;
		a.cx = b3MulAdd( a.cx, c.x, c );
		a.cy = b3MulAdd( a.cy, c.y, c );
		a.cz = b3MulAdd( a.cz, c.z, c );
	}
	b3Vec3 y = b3Solve3( a, velocity );
	for ( int k = 0; k < joints; ++k )
	{
		out[k] = b3Dot( columns[k], y );
	}
}

// IK: the angles (q, warm on entry) that put the foot at target (torso frame) within the limits; returns how far short
float lpLimbIK( const lpWorld* w, const lpLimb* limb, int joints, b3Vec3 foot, b3Vec3 target, float* q )
{
	b3Vec3 axes[LP_MAX_LIMB_JOINTS], origins[LP_MAX_LIMB_JOINTS];
	for ( int iteration = 0; iteration < LP_RIG_IK_ITERATIONS; ++iteration )
	{
		b3Vec3 at = lpLimbForward( w, limb, joints, q, foot, axes, origins );
		float dq[LP_MAX_LIMB_JOINTS];
		lpLimbSpeeds( joints, axes, origins, at, b3Sub( target, at ), dq );
		for ( int k = 0; k < joints; ++k )
		{
			q[k] = b3ClampFloat( q[k] + dq[k], limb->lower[k], limb->upper[k] );
		}
	}
	return b3Length( b3Sub( target, lpLimbForward( w, limb, joints, q, foot, axes, origins ) ) );
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
	b3Vec3 joint = last->ends[1 - limb->prox[limb->joints - 1]].frame.p;
	b3Vec3 centroid = b3Vec3_zero;
	float volume = 0.0f;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpShape* shape = w->pieces.data[b->pieces.data[k]].shape;
		centroid = b3MulAdd( centroid, shape->volume, shape->centroid );
		volume += shape->volume;
	}
	b3Vec3 axis = volume > 0.0f ? b3Sub( b3MulSV( 1.0f / volume, centroid ), joint ) : b3Vec3_zero;
	float length = b3Length( axis );
	if ( length < 1e-4f )
	{
		limb->foot = joint;
		return;
	}
	axis = b3MulSV( 1.0f / length, axis );
	float extent = 0.0f;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpShape* shape = w->pieces.data[b->pieces.data[k]].shape;
		for ( int v = 0; v < shape->vertexCount; ++v )
		{
			extent = b3MaxFloat( extent, b3Dot( b3Sub( shape->vertices[v], joint ), axis ) );
		}
	}
	limb->foot = b3MulAdd( joint, extent, axis );
}

// Which links are on in a chain from the torso, the weakest servo, the foot
static void lpLimbCapability( const lpWorld* w, lpLimb* limb )
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
		float cap = l->def.motor.maxTorque > 0.0f ? lpMotorCap( w, l ) / l->def.motor.maxTorque : 0.0f;
		limb->strength = b3MinFloat( limb->strength, cap );
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
	}
}

// A limb can stand if some joint on it swings its foot up and down (not all about the vertical) and it is strong enough
static bool lpLimbCanLift( const lpWorld* w, const lpLimb* limb, b3Vec3 up )
{
	if ( limb->strength < LP_RIG_WEAK )
	{
		return false;
	}
	for ( int k = 0; k < limb->joints; ++k )
	{
		const lpLink* l = w->links.data + limb->def.links[k];
		int body = lpEndBodyIndex( w, l, limb->prox[k] );
		b3Quat q = b3MulQuat( lpGetTransform( w->bodies.data + body ).q, l->ends[limb->prox[k]].frame.q );
		if ( b3AbsFloat( b3Dot( b3RotateVector( q, (b3Vec3){ 0.0f, 0.0f, 1.0f } ), up ) ) < 0.7f )
		{
			return true;
		}
	}
	return false;
}

b3Pos lpFootWorld( const lpWorld* w, const lpLimb* limb )
{
	return b3TransformWorldPoint( lpGetTransform( w->bodies.data + limb->tipBody ), limb->foot );
}

// ---- creation ----

int lpCreateRig( lpWorld* w, const lpRigDef* def )
{
	if ( def->body < 0 || def->body >= w->bodies.count || def->limbs == NULL || def->limbCount < 1 ||
		 def->limbCount > LP_MAX_RIG_LIMBS )
	{
		return -1;
	}
	const lpBody* torso = w->bodies.data + def->body;
	if ( torso->alive == false || B3_IS_NULL( torso->id ) || b3Body_GetType( torso->id ) != b3_dynamicBody )
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
			limb->lower[k] = limited ? b3MinFloat( l->def.lowerAngle + LP_RIG_LIMIT_MARGIN, mid ) : -1e6f;
			limb->upper[k] = limited ? b3MaxFloat( l->def.upperAngle - LP_RIG_LIMIT_MARGIN, mid ) : 1e6f;
			limb->q[k] = l->target;
		}
		limb->defFoot = b3InvTransformWorldPoint( lpGetTransform( w->bodies.data + inner ), limb->def.foot );
		limb->tipBody = -1;
		lpLimbCapability( w, limb );
	}

	b3WorldTransform xf = b3Body_GetTransform( torso->id );
	b3Vec3 forward = b3Normalize( def->forward );
	b3Vec3 up = b3Normalize( b3Sub( def->up, b3MulSV( b3Dot( def->up, forward ), forward ) ) );
	r.def = *def;
	r.def.limbs = NULL;
	r.forward = b3InvRotateVector( xf.q, forward );
	r.up = b3InvRotateVector( xf.q, up );
	r.alive = true;
	r.body = def->body;
	r.limbCount = def->limbCount;
	r.desired = xf;
	for ( int i = 0; i < r.limbCount; ++i )
	{
		lpLimb* limb = r.limbs + i;
		limb->neutral = limb->joints > 0 ? b3InvTransformWorldPoint( xf, lpFootWorld( w, limb ) ) : b3Vec3_zero;
		limb->planted = limb->joints > 0; // standing as built
		limb->hold = limb->joints > 0 ? lpFootWorld( w, limb ) : xf.p;
		limb->holdClock = 0.0f; // built above the ground, it is held where it lands
		limb->arrived = false;
		limb->groundPiece = -1;
	}
	if ( r.def.standHeight <= 0.0f )
	{
		// As created: the torso's frame above its feet
		b3Vec3 worldUp = lpRigWorldUp( w, &r, xf.q );
		float sum = 0.0f;
		for ( int i = 0; i < r.limbCount; ++i )
		{
			sum += b3Dot( b3SubPos( xf.p, lpFootWorld( w, r.limbs + i ) ), worldUp );
		}
		r.def.standHeight = sum / (float)r.limbCount;
	}
	lpArray_Push( w->rigs, r );
	return w->rigs.count - 1;
}

// ---- the step ----

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
			lpLimbCapability( w, r->limbs + i );
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
		if ( r->body < 0 || B3_IS_NULL( w->bodies.data[r->body].id ) )
		{
			r->body = -1;
			continue;
		}
		b3WorldTransform xf = lpGetTransform( w->bodies.data + r->body );
		b3Vec3 worldUp = lpRigWorldUp( w, r, xf.q );
		for ( int i = 0; i < r->limbCount; ++i )
		{
			lpLimb* limb = r->limbs + i;
			limb->attached = limb->joints > 0 && limb->rootBody == r->body;
			limb->able = limb->attached && lpLimbCanLift( w, limb, worldUp );
			limb->reach = 0.0f;
			if ( limb->joints > 0 )
			{
				const lpLink* root = w->links.data + limb->def.links[0];
				b3Pos joint = b3TransformWorldPoint( lpGetTransform( w->bodies.data + limb->rootBody ), root->ends[limb->prox[0]].frame.p );
				limb->reach = b3Length( b3SubPos( lpFootWorld( w, limb ), joint ) );
			}
		}
		lpWalkRig( w, r, timeStep );
		r->controlChanged = false;
	}
}

// ---- API ----

void lpWorld_SetRigControl( lpWorld* w, int rig, const lpRigControl* control )
{
	if ( rig < 0 || rig >= w->rigs.count )
	{
		return;
	}
	lpRig* r = w->rigs.data + rig;
	lpRigControl c = { b3ClampFloat( control->forward, -1.0f, 1.0f ), b3ClampFloat( control->strafe, -1.0f, 1.0f ),
					   b3ClampFloat( control->turn, -1.0f, 1.0f ), b3ClampFloat( control->crouch, 0.0f, 1.0f ) };
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
		b3BodyId id = w->bodies.data[s.body].id;
		b3WorldTransform xf = b3Body_GetTransform( id );
		s.position = xf.p;
		s.forward = b3RotateVector( xf.q, r->forward );
		s.up = b3RotateVector( xf.q, r->up );
		s.speed = b3Dot( b3Body_GetLinearVelocity( id ), s.forward );
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
		uint8_t flags[2] = { r->alive ? 1 : 0, r->idle ? 1 : 0 };
		h = lpHashBytes( h, control, sizeof( control ) );
		h = lpHashBytes( h, &r->desired.p, sizeof( r->desired.p ) );
		h = lpHashBytes( h, pose, sizeof( pose ) );
		h = lpHashBytes( h, flags, sizeof( flags ) );
		h = lpHashBytes( h, &r->calm, sizeof( r->calm ) );
		h = lpHashBytes( h, &r->body, sizeof( r->body ) );
		for ( int i = 0; i < r->limbCount; ++i )
		{
			const lpLimb* limb = r->limbs + i;
			int ints[3] = { limb->joints, limb->tipBody, limb->groundPiece };
			uint8_t limbFlags[7] = { limb->attached ? 1 : 0, limb->able ? 1 : 0,	limb->planted ? 1 : 0, limb->swinging ? 1 : 0,
									 limb->castLate ? 1 : 0, limb->grounded ? 1 : 0, limb->arrived ? 1 : 0 };
			h = lpHashBytes( h, ints, sizeof( ints ) );
			h = lpHashBytes( h, limbFlags, sizeof( limbFlags ) );
			h = lpHashBytes( h, &limb->foot, sizeof( limb->foot ) );
			h = lpHashBytes( h, limb->q, sizeof( limb->q ) );
			h = lpHashBytes( h, &limb->swingClock, sizeof( limb->swingClock ) );
			h = lpHashBytes( h, &limb->liftoff, sizeof( limb->liftoff ) );
			h = lpHashBytes( h, &limb->landing, sizeof( limb->landing ) );
			h = lpHashBytes( h, &limb->hold, sizeof( limb->hold ) );
			h = lpHashBytes( h, &limb->holdClock, sizeof( limb->holdClock ) );
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
