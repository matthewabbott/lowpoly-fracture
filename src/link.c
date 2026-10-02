// SPDX-License-Identifier: MIT
// Links: physics joints between objects (weld, hinge, ball, rope) that break under load and outlive the pieces they
// were made on.
//
// A link end is a piece and a frame in its body's frame. A piece's body frame never changes (new bodies are made at
// their parent's transform), so an end stays valid through splits and tier changes; only the physics joint has to be
// rebuilt when a piece's body changes, and lpSyncLinks does that just before the physics step, when every body of
// the step exists. When a piece fractures, each link end on it moves to the kept cell holding its anchor, and the link
// breaks if that cell was blown out. An end whose piece leaves the physics (freed, or made a ghost or scrap) breaks its
// link at once, so a live link always has both ends on live physics bodies. A rebuilt link between two moving bodies
// tears when one of them is a chip next to the other.
//
// Loads are polled after the physics step (Box3D's joint events miss joints in the overflow constraint colour and
// sleeping ones). A link over its limit strains and creaks; it breaks at strain 1, or at once when its smoothed load
// reaches twice its limit. A rebuilt joint starts cold, so loads are not judged for a few steps after a build.

#include "world.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#define LP_LINK_SETTLE 3	   // steps after a (re)build before the load is judged
#define LP_LINK_REACH 0.25f	   // an anchor must be this close to a piece of its body
#define LP_LINK_CLAMP 3.0f	   // utilization above this counts as this (one wild step cannot snap a link)
#define LP_LINK_STRAIN 10.0f   // strain per second per unit of overload: 10% over lasts about a second
#define LP_LINK_TEAR_RATIO 0.02f // a rebuilt link between two moving bodies tears when one is under 2% of the other

lpLinkDef lpDefaultLinkDef( int type )
{
	lpLinkDef def = { 0 };
	def.type = type;
	def.bodyA = -1;
	def.bodyB = -1;
	def.axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
	def.dampingRatio = 1.0f;
	def.motor.maxSpeed = 2.0f;
	def.motor.gain = 6.0f;
	def.motor.jamKnock = 0.25f;
	switch ( type )
	{
		case lp_linkRope:
			def.maxForce = 8000.0f; // a thick hemp rope
			def.strength = 3000.0f;
			def.collideConnected = true;
			break;
		case lp_linkHinge:
			def.maxForce = 40000.0f;
			def.maxTorque = 4000.0f;
			def.strength = 6000.0f;
			break;
		case lp_linkBall:
			def.maxForce = 30000.0f;
			def.maxTorque = 3000.0f;
			def.strength = 6000.0f;
			break;
		default:
			def.maxForce = 40000.0f;
			def.maxTorque = 8000.0f;
			def.strength = 8000.0f;
			break;
	}
	return def;
}

// A rotation taking z to the axis (hinge, spring and cone axes are the frames' z)
static lpQuat lpAxisRotation( lpVec3 axis )
{
	float length = lpLength( axis );
	lpVec3 z = length > 1e-6f ? lpMulSV( 1.0f / length, axis ) : (lpVec3){ 0.0f, 0.0f, 1.0f };
	return lpComputeQuatBetweenUnitVectors( (lpVec3){ 0.0f, 0.0f, 1.0f }, z );
}

// The piece of a body nearest a world point, and how far outside it the point is
static int lpNearestPiece( const lpWorld* w, int bodyIndex, lpPos point, float* distance )
{
	const lpBody* b = w->bodies.data + bodyIndex;
	lpVec3 local = lpInvTransformWorldPoint( lpGetTransform( w, b ), point );
	int best = -1;
	*distance = FLT_MAX;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		int pi = b->pieces.data[k];
		float d = lpShape_SignedDistance( w->pieces.data[pi].shape, local );
		if ( d < *distance )
		{
			*distance = d;
			best = pi;
		}
	}
	return best;
}

static lpPhysBody lpEndBody( const lpWorld* w, const lpLink* l, int k )
{
	int piece = l->ends[k].piece;
	return piece < 0 ? l->anchor[k] : w->bodies.data[w->pieces.data[piece].body].id;
}

static bool lpHasMotor( const lpLink* l )
{
	return ( l->def.type == lp_linkHinge || l->def.type == lp_linkBall ) &&
		   ( l->def.motor.maxTorque > 0.0f || l->def.motor.holdTorque > 0.0f );
}

static void lpBuildJoint( lpWorld* w, int index )
{
	lpLink* l = w->links.data + index;
	const lpLinkDef* d = &l->def;
	lpPhysBody a = lpEndBody( w, l, 0 );
	lpPhysBody b = lpEndBody( w, l, 1 );
	lpPhysJointDef jd = { 0 };
	jd.bodyA = a;
	jd.bodyB = b;
	jd.frameA = l->ends[0].frame;
	jd.frameB = l->ends[1].frame;
	jd.collideConnected = d->collideConnected;
	switch ( d->type )
	{
		case lp_linkWeld:
			jd.type = lp_physWeld;
			jd.hertz = d->hertz;
			jd.dampingRatio = d->dampingRatio;
			break;
		case lp_linkHinge:
			jd.type = lp_physHinge;
			jd.lowerAngle = d->lowerAngle;
			jd.upperAngle = d->upperAngle;
			jd.enableMotor = lpHasMotor( l ); // its speed and torque come from lpDriveMotors
			break;
		case lp_linkBall:
			jd.type = lp_physBall;
			jd.coneAngle = d->coneAngle;
			jd.enableMotor = lpHasMotor( l );
			break;
		default:
			jd.type = lp_physRope; // only an upper length limit, so it goes slack when pushed together
			jd.length = d->length;
			break;
	}
	l->joint = lpPhys_CreateJoint( w->phys, &jd );
	LP_ASSERT( LP_PHYS_NULL( l->joint ) == false );
	l->builtOn[0] = a;
	l->builtOn[1] = b;
	l->settle = LP_LINK_SETTLE;
	l->motorApplied = false; // a new joint gets its motor's speed and torque again
}

static int lpAllocLink( lpWorld* w )
{
	int index;
	if ( w->freeLink != -1 )
	{
		index = w->freeLink;
		w->freeLink = w->links.data[index].nextFree;
		uint32_t generation = w->links.data[index].generation;
		memset( w->links.data + index, 0, sizeof( lpLink ) );
		w->links.data[index].generation = generation + 1;
	}
	else
	{
		index = w->links.count;
		lpLink zero = { 0 };
		lpArray_Push( w->links, zero );
	}
	w->links.data[index].nextFree = -1;
	w->links.data[index].wheel = -1;
	w->links.data[index].targetRotation = lpQuat_identity;
	return index;
}

int lpCreateLink( lpWorld* w, const lpLinkDef* def )
{
	lpGuardFp( w ); // computes in float between steps, on the caller's thread
	if ( def->type < 0 || def->type >= lp_linkTypeCount || def->type == lp_linkWheel || def->bodyA == def->bodyB ||
		 w->linkCount >= w->def.maxLinks )
	{
		return -1;
	}
	int bodies[2] = { def->bodyA, def->bodyB };
	lpPos points[2] = { def->anchorA, def->type == lp_linkRope ? def->anchorB : def->anchorA };
	lpQuat q = lpAxisRotation( def->axis );
	lpLinkEnd ends[2];
	for ( int k = 0; k < 2; ++k )
	{
		if ( bodies[k] < 0 )
		{
			ends[k] = (lpLinkEnd){ -1, 0, { lpVec3_zero, q } }; // the anchor body sits at the point, unrotated
			continue;
		}
		if ( bodies[k] >= w->bodies.count || w->bodies.data[bodies[k]].alive == false ||
			 LP_PHYS_NULL( w->bodies.data[bodies[k]].id ) )
		{
			return -1; // gone, or a ghost or scrap body
		}
		float distance;
		int piece = lpNearestPiece( w, bodies[k], points[k], &distance );
		if ( piece < 0 || distance > LP_LINK_REACH )
		{
			return -1;
		}
		lpWorldTransform xf = lpGetTransform( w, w->bodies.data + bodies[k] );
		ends[k].piece = piece;
		ends[k].generation = w->pieces.data[piece].generation;
		ends[k].frame.p = lpInvTransformWorldPoint( xf, points[k] );
		ends[k].frame.q = lpInvMulQuat( xf.q, q );
	}
	for ( int k = 0; k < 2; ++k )
	{
		if ( bodies[k] >= 0 && w->bodies.data[bodies[k]].kind == lp_kindRubble )
		{
			lpWakeRubble( w, bodies[k] ); // a joint on frozen rubble would hold nothing
		}
	}

	int index = lpAllocLink( w );
	lpLink* l = w->links.data + index;
	l->def = *def;
	l->def.motor.jamKnock = l->def.motor.jamKnock > 0.0f ? l->def.motor.jamKnock : 0.25f;
	if ( def->type == lp_linkRope && l->def.length <= 0.0f )
	{
		l->def.length = lpLength( lpSubPos( points[1], points[0] ) );
	}
	l->def.length = lpMaxFloat( l->def.length, 0.01f );
	l->health = def->strength;
	l->alive = true;
	for ( int k = 0; k < 2; ++k )
	{
		l->ends[k] = ends[k];
		l->points[k] = points[k];
		if ( ends[k].piece >= 0 )
		{
			lpArray_Push( w->pieces.data[ends[k].piece].links, index );
		}
		else
		{
			lpPhysBodyDef bd = lpPhys_DefaultBodyDef();
			bd.transform.p = points[k];
			l->anchor[k] = lpPhys_CreateBody( w->phys, &bd );
		}
	}
	lpBuildJoint( w, index );
	w->linkCount += 1;
	lpCarriersChanged( w, def->carries );
	return index;
}

int lpCreateWheelLink( lpWorld* w, int body, lpPos mount, float maxForce, float strength, int wheel )
{
	float distance;
	int piece = lpNearestPiece( w, body, mount, &distance );
	if ( piece < 0 || distance > LP_LINK_REACH || w->linkCount >= w->def.maxLinks )
	{
		return -1;
	}
	lpWorldTransform xf = lpGetTransform( w, w->bodies.data + body );
	int index = lpAllocLink( w );
	lpLink* l = w->links.data + index;
	l->def = lpDefaultLinkDef( lp_linkWheel );
	l->def.bodyA = body;
	l->def.anchorA = mount;
	l->def.maxForce = maxForce;
	l->def.maxTorque = 0.0f;
	l->def.strength = strength;
	l->health = strength;
	l->alive = true;
	l->wheel = wheel;
	l->ends[0] = (lpLinkEnd){ piece, w->pieces.data[piece].generation, { lpInvTransformWorldPoint( xf, mount ), lpQuat_identity } };
	l->ends[1] = (lpLinkEnd){ -1, 0, lpTransform_identity };
	l->points[0] = mount;
	l->points[1] = mount;
	lpArray_Push( w->pieces.data[piece].links, index );
	w->linkCount += 1;
	return index;
}

static void lpRemoveLinkFromPiece( lpPiece* p, int index )
{
	for ( int i = 0; i < p->links.count; ++i )
	{
		if ( p->links.data[i] == index )
		{
			memmove( p->links.data + i, p->links.data + i + 1, sizeof( int ) * (size_t)( p->links.count - i - 1 ) );
			p->links.count -= 1;
			return;
		}
	}
	LP_ASSERT( false );
}

// A snapping link throws a little dust from both ends. Cosmetic only: the randomness is hashed from the tick and the
// link, never taken from simulation state.
static void lpLinkDust( lpWorld* w, const lpLink* l, int index, int motes )
{
	for ( int k = 0; k < 2; ++k )
	{
		int piece = l->ends[k].piece;
		if ( piece < 0 )
		{
			continue;
		}
		const lpPiece* p = w->pieces.data + piece;
		lpWorldTransform xf = lpGetTransform( w, w->bodies.data + p->body );
		uint64_t h = lpMix64( ( w->tick << 24 ) ^ (uint64_t)( 2 * index + k ) );
		for ( int m = 0; m < motes; ++m )
		{
			h = lpMix64( h + (uint64_t)m );
			float rx = (float)( h & 0xFFFF ) / 65535.0f - 0.5f;
			float rz = (float)( ( h >> 16 ) & 0xFFFF ) / 65535.0f - 0.5f;
			float rs = (float)( ( h >> 32 ) & 0xFFFF ) / 65535.0f;
			lpVec3 v = { 0.8f * rx, -0.2f - 0.4f * rs, 0.8f * rz };
			lpEmitParticle( w, xf, l->ends[k].frame.p, v, 0.02f + 0.02f * rs, p->material );
		}
	}
}

// A wheel that breaks comes off as a wheel of its own; one destroyed on purpose just goes
static void lpReleaseLink( lpWorld* w, int index, bool broken )
{
	lpLink* l = w->links.data + index;
	LP_ASSERT( l->alive );
	if ( l->wheel >= 0 )
	{
		lpReleaseWheel( w, l->wheel, broken );
		l->wheel = -1;
	}
	else if ( lpPhys_IsValidJoint( w->phys, l->joint ) )
	{
		lpPhys_DestroyJoint( w->phys, l->joint, true ); // what it held falls
	}
	for ( int k = 0; k < 2; ++k )
	{
		if ( l->ends[k].piece >= 0 )
		{
			lpPiece* p = w->pieces.data + l->ends[k].piece;
			lpRemoveLinkFromPiece( p, index );
			lpBody* b = w->bodies.data + p->body;
			if ( b->alive && b->kind == lp_kindStructure )
			{
				lpRequestStressCheck( w, p->body, true ); // its pull is gone (a link can break during the splits)
			}
		}
		else if ( LP_PHYS_NULL( l->anchor[k] ) == false )
		{
			lpPhys_DestroyBody( w->phys, l->anchor[k] );
		}
	}
	lpCarriersChanged( w, l->def.carries );
	l->alive = false;
	l->joint = lp_nullPhysJoint;
	l->nextFree = w->freeLink;
	w->freeLink = index;
	w->linkCount -= 1;
}

void lpBreakLink( lpWorld* w, int index, bool dust )
{
	if ( dust )
	{
		lpLinkDust( w, w->links.data + index, index, 4 );
	}
	lpReleaseLink( w, index, true );
	w->stats.linkBreaks += 1;
}

void lpDestroyLink( lpWorld* w, int link )
{
	if ( link >= 0 && link < w->links.count && w->links.data[link].alive )
	{
		lpReleaseLink( w, link, false );
	}
}

// The piece is leaving the physics (freed, or made a ghost or scrap): its links go with it
void lpBreakPieceLinks( lpWorld* w, int piece )
{
	lpPiece* p = w->pieces.data + piece;
	while ( p->links.count > 0 )
	{
		lpBreakLink( w, p->links.data[p->links.count - 1], false );
	}
}

void lpDetachLinks( lpWorld* w, int piece, lpShape* const* cells, int cellCount )
{
	lpPiece* p = w->pieces.data + piece;
	w->scratchLinkMoves.count = 0;
	for ( int n = 0; n < p->links.count; ++n )
	{
		int index = p->links.data[n];
		lpLink* l = w->links.data + index;
		int end = l->ends[0].piece == piece ? 0 : 1;
		int cell = -1;
		float nearest = FLT_MAX;
		for ( int i = 0; i < cellCount; ++i )
		{
			float d = lpShape_SignedDistance( cells[i], l->ends[end].frame.p );
			if ( d < nearest )
			{
				nearest = d;
				cell = i;
			}
		}
		l->ends[end].piece = -1; // detached until the cells are placed: on no piece and no anchor body
		lpLinkMove move = { index, end, cell };
		lpArray_Push( w->scratchLinkMoves, move );
	}
	p->links.count = 0; // so freeing the piece does not break them
}

void lpAttachLinks( lpWorld* w, const int* cellToPiece )
{
	for ( int n = 0; n < w->scratchLinkMoves.count; ++n )
	{
		lpLinkMove move = w->scratchLinkMoves.data[n];
		int child = move.cell >= 0 ? cellToPiece[move.cell] : -1;
		if ( child < 0 )
		{
			lpBreakLink( w, move.link, true ); // the cell holding the anchor was blown out
			continue;
		}
		// Kept cells stay on the same body, and the frame is in its frame: the joint holds on unchanged. A motor that jams
		// takes a knock from the piece breaking up around it.
		lpLink* l = w->links.data + move.link;
		if ( l->def.motor.jam > 0.0f && l->def.strength > 0.0f )
		{
			l->health = lpMaxFloat( l->health - l->def.motor.jamKnock * l->def.strength, 0.1f * l->def.strength );
		}
		lpLinkEnd* e = l->ends + move.end;
		e->piece = child;
		e->generation = w->pieces.data[child].generation;
		lpArray_Push( w->pieces.data[child].links, move.link );
	}
	w->scratchLinkMoves.count = 0;
}

// A chip left holding a much heavier body by a link would jitter on the joint (or blow it up): it tears off instead
static bool lpTearsOff( const lpWorld* w, lpPhysBody a, lpPhysBody b, float ratio )
{
	if ( lpPhys_IsDynamic( w->phys, a ) == false || lpPhys_IsDynamic( w->phys, b ) == false )
	{
		return false;
	}
	float ma = lpPhys_GetMass( w->phys, a );
	float mb = lpPhys_GetMass( w->phys, b );
	return lpMinFloat( ma, mb ) < ( ratio > 0.0f ? ratio : LP_LINK_TEAR_RATIO ) * lpMaxFloat( ma, mb );
}

void lpWorld_SetRopeLength( lpWorld* w, int link, float length )
{
	if ( link < 0 || link >= w->links.count || w->links.data[link].alive == false || w->links.data[link].def.type != lp_linkRope )
	{
		return;
	}
	lpLink* l = w->links.data + link;
	l->def.length = lpMaxFloat( length, 0.01f );
	lpPhys_SetRopeLength( w->phys, l->joint, l->def.length );
	lpPhys_WakeJoint( w->phys, l->joint );
}

void lpWorld_SetLinkTarget( lpWorld* w, int link, float angle )
{
	if ( link >= 0 && link < w->links.count && w->links.data[link].alive && w->links.data[link].target != angle )
	{
		w->links.data[link].target = angle;
		w->links.data[link].targetChanged = true;
	}
}

void lpWorld_SetLinkTargetRotation( lpWorld* w, int link, lpQuat rotation )
{
	if ( link < 0 || link >= w->links.count || w->links.data[link].alive == false )
	{
		return;
	}
	lpLink* l = w->links.data + link;
	lpQuat q = lpNormalizeQuat( rotation );
	if ( q.s != l->targetRotation.s || q.v.x != l->targetRotation.v.x || q.v.y != l->targetRotation.v.y ||
		 q.v.z != l->targetRotation.v.z )
	{
		l->targetRotation = q;
		l->targetChanged = true;
	}
}

// A joint frame of a link in the world
static lpQuat lpEndFrameRotation( const lpWorld* w, const lpLink* l, int k )
{
	return lpMulQuat( lpPhys_GetTransform( w->phys, lpEndBody( w, l, k ) ).q, l->ends[k].frame.q );
}

// How well what a motor needs is fed at the better of its ends
static float lpMotorFeed( const lpWorld* w, const lpLink* l )
{
	if ( l->def.motor.needs == 0 )
	{
		return 1.0f;
	}
	float fed = 0.0f;
	for ( int k = 0; k < 2; ++k )
	{
		if ( l->ends[k].piece >= 0 )
		{
			fed = lpMaxFloat( fed, lpSupplyOf( w->pieces.data + l->ends[k].piece, l->def.motor.needs ) );
		}
	}
	return fed;
}

// How jammed a motor is: its jam times the link's damage, 0 to 1
static float lpMotorJam( const lpLink* l )
{
	if ( l->def.motor.jam <= 0.0f || l->def.strength <= 0.0f )
	{
		return 0.0f;
	}
	return l->def.motor.jam * lpClampFloat( 1.0f - l->health / l->def.strength, 0.0f, 1.0f );
}

float lpMotorDrive( const lpWorld* w, const lpLink* l )
{
	float health = l->def.strength > 0.0f ? lpMaxFloat( l->health / l->def.strength, 0.1f ) : 1.0f;
	return l->def.motor.maxTorque * health * lpMotorFeed( w, l );
}

float lpMotorCap( const lpWorld* w, const lpLink* l )
{
	const lpMotorDef* m = &l->def.motor;
	float fed = lpMotorFeed( w, l );
	float hold = lpMaxFloat( m->holdTorque, lpMotorJam( l ) * m->maxTorque ); // a jammed joint sticks
	return lpMotorDrive( w, l ) + hold * ( 1.0f - fed );
}

// The servos: speed toward the target in proportion to how far off it is, torque up to the cap. Box3D's setters are
// called only when a value changed (and always after a rebuild); a changed target wakes the joint's bodies.
void lpDriveMotors( lpWorld* w )
{
	for ( int i = 0; i < w->links.count; ++i )
	{
		lpLink* l = w->links.data + i;
		if ( l->alive == false || lpHasMotor( l ) == false )
		{
			continue;
		}
		const lpMotorDef* m = &l->def.motor;
		float fed = lpMotorFeed( w, l );
		float cap = lpMotorCap( w, l );
		if ( l->def.type == lp_linkHinge )
		{
			float angle = lpPhys_GetHingeAngle( w->phys, l->joint );
			float speed = m->gain * ( l->target - angle );
			if ( l->feed != 0.0f )
			{
				speed += l->feed; // its rig's motion this step: servos with different gains or clamps still move together
			}
			float top = m->maxSpeed * ( 1.0f - lpMotorJam( l ) ); // a jammed joint turns slower
			speed = fed > 0.0f ? lpClampFloat( speed, -top, top ) : 0.0f;
			if ( l->motorApplied == false || speed != l->appliedSpeed )
			{
				lpPhys_SetHingeMotor( w->phys, l->joint, speed );
				l->appliedSpeed = speed;
				w->stats.motorSets += 1;
			}
		}
		else
		{
			// The rotation from where frame B is (relative to frame A) to where it should be, as a rotation vector
			lpQuat qA = lpEndFrameRotation( w, l, 0 );
			lpQuat relative = lpInvMulQuat( qA, lpEndFrameRotation( w, l, 1 ) );
			lpQuat error = lpMulQuat( l->targetRotation, lpConjugate( relative ) );
			if ( error.s < 0.0f )
			{
				error = (lpQuat){ lpNeg( error.v ), -error.s };
			}
			float sine = lpLength( error.v );
			lpVec3 omega = lpVec3_zero;
			if ( sine > 1e-6f && fed > 0.0f )
			{
				float angle = 2.0f * lpAtan2( sine, error.s );
				float speed = lpMinFloat( m->gain * angle, m->maxSpeed * ( 1.0f - lpMotorJam( l ) ) );
				omega = lpRotateVector( qA, lpMulSV( speed / sine, error.v ) );
			}
			if ( l->motorApplied == false || lpLength( lpSub( omega, l->appliedVelocity ) ) > 0.0f )
			{
				lpPhys_SetBallMotor( w->phys, l->joint, omega );
				l->appliedVelocity = omega;
				w->stats.motorSets += 1;
			}
		}
		if ( l->motorApplied == false || cap != l->appliedCap )
		{
			lpPhys_SetMotorMaxTorque( w->phys, l->joint, cap );
			if ( l->motorApplied )
			{
				lpPhys_WakeJoint( w->phys, l->joint ); // weaker or stronger (unfed, damaged, fed again): a sleeping limb must react
			}
			l->appliedCap = cap;
			w->stats.motorSets += 1;
		}
		l->motorApplied = true;
		l->motorCap = cap;
		if ( l->targetChanged )
		{
			lpPhys_WakeJoint( w->phys, l->joint );
			l->targetChanged = false;
		}
	}
}

void lpSyncLinks( lpWorld* w )
{
	uint64_t stamp = w->tick + 1;
	for ( int i = 0; i < w->links.count; ++i )
	{
		lpLink* l = w->links.data + i;
		if ( l->alive == false )
		{
			continue;
		}
		if ( l->wheel >= 0 )
		{
			// No joint to rebuild. A mount left on a chip would be launched by its spring: the wheel tears off instead.
			lpBody* mount = w->bodies.data + w->pieces.data[l->ends[0].piece].body;
			mount->linkStamp = stamp;
			if ( lpPhys_IsDynamic( w->phys, mount->id ) &&
				 lpPhys_GetMass( w->phys, mount->id ) < w->wheels.data[l->wheel].def.tearRatio * w->wheels.data[l->wheel].sprungMass )
			{
				lpBreakLink( w, i, true );
			}
			continue;
		}
		lpPhysBody bodies[2];
		for ( int k = 0; k < 2; ++k )
		{
			if ( l->ends[k].piece >= 0 )
			{
				w->bodies.data[w->pieces.data[l->ends[k].piece].body].linkStamp = stamp;
			}
			bodies[k] = lpEndBody( w, l, k );
		}
		if ( lpPhys_IsValidJoint( w->phys, l->joint ) && LP_PHYS_EQUAL( bodies[0], l->builtOn[0] ) &&
			 LP_PHYS_EQUAL( bodies[1], l->builtOn[1] ) )
		{
			continue;
		}
		// An end moved to a new body (a split, an ejected cell, a ghost made whole again), or its old body is gone
		if ( lpPhys_IsValidJoint( w->phys, l->joint ) )
		{
			lpPhys_DestroyJoint( w->phys, l->joint, false );
		}
		if ( lpTearsOff( w, bodies[0], bodies[1], l->def.tearRatio ) )
		{
			lpBreakLink( w, i, true );
			continue;
		}
		lpBuildJoint( w, i );
		w->stats.linkRebuilds += 1;
	}
}

float lpSegmentDistance( lpVec3 a, lpVec3 b )
{
	lpVec3 ab = lpSub( b, a );
	float length2 = lpDot( ab, ab );
	float t = length2 > 0.0f ? lpClampFloat( -lpDot( a, ab ) / length2, 0.0f, 1.0f ) : 0.0f;
	return lpLength( lpMulAdd( a, t, ab ) );
}

bool lpBodyLinked( const lpWorld* w, const lpBody* b )
{
	return b->linkStamp == w->tick + 1;
}

bool lpTouchesLinked( lpWorld* w, const lpBody* b )
{
	if ( w->linkCount == 0 )
	{
		return false; // free in a world without links
	}
	const lpPhysContact* contacts;
	int count = lpPhys_GetBodyContacts( w->phys, b->id, &contacts );
	for ( int k = 0; k < count; ++k )
	{
		int body = contacts[k].other >= 0 ? w->pieces.data[contacts[k].other].body : -1;
		if ( body >= 0 && w->bodies.data + body != b && w->bodies.data[body].kind == lp_kindDebris &&
			 lpBodyLinked( w, w->bodies.data + body ) )
		{
			return true;
		}
	}
	return false;
}

// A structure at either end carries the link's pull in its stress solve (sampled with its loads): check it again when
// its force or torque has changed by a quarter since (in direction too: a sign swung round pulls the other way), at
// most every 30 steps (a swinging load pulls back and forth). A moving body that solves its stress is checked this way
// at most every 30 steps whatever pulls on it (a walker's torso holds six legs); a link held back keeps asking, so a
// landing is checked with the loads it has then. Jolted by a hard hit, it is checked as the loads come, for 10 steps.
static void lpRecheckStructures( lpWorld* w, lpLink* l )
{
	float force = lpLength( lpSub( l->force, l->stressForce ) );
	float torque = lpLength( lpSub( l->torque, l->stressTorque ) );
	bool changed = force > 0.25f * lpMaxFloat( lpLength( l->force ), lpLength( l->stressForce ) ) + 10.0f ||
				   torque > 0.25f * lpMaxFloat( lpLength( l->torque ), lpLength( l->stressTorque ) ) + 10.0f;
	bool jolted[2] = { false, false };
	for ( int k = 0; k < 2; ++k )
	{
		int piece = l->ends[k].piece;
		const lpBody* b = piece >= 0 ? w->bodies.data + w->pieces.data[piece].body : NULL;
		jolted[k] = b != NULL && b->solveStress && b->joltTick != 0 && w->tick + 1 < b->joltTick + 10;
	}
	if ( changed == false || ( l->recheckTick != 0 && w->tick + 1 < l->recheckTick + 30 && jolted[0] == false && jolted[1] == false ) )
	{
		return;
	}
	bool heldBack = false;
	for ( int k = 0; k < 2; ++k )
	{
		int piece = l->ends[k].piece;
		int bodyIndex = piece >= 0 ? w->pieces.data[piece].body : -1;
		lpBody* b = bodyIndex >= 0 ? w->bodies.data + bodyIndex : NULL;
		bool moving = b != NULL && b->solveStress && b->kind == lp_kindDebris;
		if ( moving && jolted[k] == false && b->loadCheckTick != 0 && w->tick + 1 < b->loadCheckTick + 30 )
		{
			heldBack = true;
		}
		else if ( b != NULL && ( b->kind == lp_kindStructure || moving ) )
		{
			lpRequestStressCheck( w, bodyIndex, false );
			l->recheckTick = w->tick + 1;
			b->loadCheckTick = moving ? w->tick + 1 : b->loadCheckTick;
		}
	}
	if ( heldBack == false )
	{
		l->stressForce = l->force; // with no structure at either end there is nothing to check: stop asking too
		l->stressTorque = l->torque;
	}
}

// Load over limit, weakened by blast damage, clamped. The torque is what the joint holds against, without its motor's.
static float lpLinkLoad( const lpLink* l, lpVec3 torque )
{
	float u = 0.0f;
	if ( l->def.maxForce > 0.0f )
	{
		u = lpLength( l->force ) / l->def.maxForce;
	}
	if ( l->def.maxTorque > 0.0f )
	{
		u = lpMaxFloat( u, lpLength( torque ) / l->def.maxTorque );
	}
	if ( l->def.strength > 0.0f )
	{
		u /= lpMaxFloat( l->health / l->def.strength, 0.1f );
	}
	return lpMinFloat( u, LP_LINK_CLAMP );
}

// Strain and break an overloaded link; true when it broke
static bool lpStrainLink( lpWorld* w, int i, float timeStep )
{
	lpLink* l = w->links.data + i;
	if ( l->utilization >= 2.0f )
	{
		lpBreakLink( w, i, true );
		return true;
	}
	if ( l->utilization > 1.0f )
	{
		l->strain += ( l->utilization - 1.0f ) * w->def.strainRate * LP_LINK_STRAIN * timeStep;
		if ( l->strain >= 1.0f )
		{
			lpBreakLink( w, i, true );
			return true;
		}
		if ( ( w->tick + (uint64_t)i ) % 6 == 0 )
		{
			lpLinkDust( w, l, i, 1 ); // creaking
		}
	}
	return false;
}

void lpPollLinks( lpWorld* w, float timeStep )
{
	for ( int i = 0; i < w->links.count; ++i )
	{
		lpLink* l = w->links.data + i;
		if ( l->alive == false )
		{
			continue;
		}
		if ( l->wheel >= 0 )
		{
			// A wheel's load is its own force, exact and computed before the step (wheel.c): judged as it comes, so a
			// hard landing breaks it in one step
			lpPhysBody mount = w->bodies.data[w->pieces.data[l->ends[0].piece].body].id;
			if ( lpPhys_IsAwake( w->phys, mount ) )
			{
				l->utilization = lpLinkLoad( l, l->torque );
				if ( lpStrainLink( w, i, timeStep ) == false && l->utilization > 1.0f )
				{
					lpPhys_SetAwake( w->phys, mount, true );
				}
			}
			continue;
		}
		lpPhysBody a = l->builtOn[0];
		lpPhysBody b = l->builtOn[1];
		if ( lpPhys_IsAwake( w->phys, a ) == false && lpPhys_IsAwake( w->phys, b ) == false )
		{
			if ( l->settle == 0 )
			{
				lpRecheckStructures( w, l ); // one held back by the rate limit still gets through
			}
			continue; // asleep: nothing moved, nothing changed
		}
		l->points[0] = lpTransformWorldPoint( lpPhys_GetTransform( w->phys, a ), l->ends[0].frame.p );
		l->points[1] = lpTransformWorldPoint( lpPhys_GetTransform( w->phys, b ), l->ends[1].frame.p );
		lpPhys_GetJointLoad( w->phys, l->joint, &l->force, &l->torque );
		lpVec3 held = l->torque; // what the joint holds against: the motor's own torque is not a load on it
		if ( l->def.type == lp_linkHinge )
		{
			l->angle = lpPhys_GetHingeAngle( w->phys, l->joint );
		}
		if ( lpHasMotor( l ) && l->def.type == lp_linkHinge )
		{
			lpVec3 axis = lpRotateVector( lpEndFrameRotation( w, l, 0 ), (lpVec3){ 0.0f, 0.0f, 1.0f } );
			float drive = lpPhys_GetHingeMotorTorque( w->phys, l->joint );
			held = lpMulSub( held, drive, axis );
			l->motorTorque = lpAbsFloat( drive );
		}
		else if ( lpHasMotor( l ) )
		{
			lpVec3 drive = lpPhys_GetBallMotorTorque( w->phys, l->joint );
			held = lpSub( held, drive );
			l->motorTorque = lpLength( drive );
		}
		if ( l->settle > 0 )
		{
			l->settle -= 1;
			continue;
		}
		lpRecheckStructures( w, l );

		l->utilization += 0.5f * ( lpLinkLoad( l, held ) - l->utilization );
		if ( lpStrainLink( w, i, timeStep ) == false && l->utilization > 1.0f )
		{
			lpPhys_WakeJoint( w->phys, l->joint ); // keep straining until it holds or gives
		}
	}
}

lpLinkState lpWorld_GetLinkState( const lpWorld* w, int link )
{
	lpLinkState s = { 0 };
	s.bodyA = -1;
	s.bodyB = -1;
	if ( link < 0 || link >= w->links.count )
	{
		return s;
	}
	const lpLink* l = w->links.data + link;
	s.generation = l->generation;
	s.type = l->def.type;
	if ( l->alive == false )
	{
		return s;
	}
	s.alive = true;
	s.bodyA = l->ends[0].piece >= 0 ? w->pieces.data[l->ends[0].piece].body : -1;
	s.bodyB = l->ends[1].piece >= 0 ? w->pieces.data[l->ends[1].piece].body : -1;
	s.pointA = l->points[0];
	s.pointB = l->points[1];
	s.slack = l->def.type == lp_linkRope && lpLength( lpSubPos( s.pointB, s.pointA ) ) < l->def.length - 0.01f;
	s.force = l->force;
	s.torque = l->torque;
	s.utilization = l->utilization;
	s.strain = l->strain;
	s.health = l->health;
	s.length = l->def.length;
	for ( int k = 0; k < 2; ++k )
	{
		s.supplied |= l->ends[k].piece >= 0 ? lpSuppliedMask( w->pieces.data + l->ends[k].piece ) : (uint8_t)0;
	}
	s.userId = l->def.userId;
	s.angle = l->angle;
	s.motorTorque = l->motorTorque;
	s.motorCap = l->motorCap;
	return s;
}

int lpWorld_GetLinkCapacity( const lpWorld* w )
{
	return w->links.count;
}

uint64_t lpHashLinks( const lpWorld* w, uint64_t h )
{
	for ( int i = 0; i < w->links.count; ++i )
	{
		const lpLink* l = w->links.data + i;
		if ( l->alive == false )
		{
			continue;
		}
		h = lpHashBytes( h, &i, sizeof( i ) );
		h = lpHashBytes( h, &l->def.type, sizeof( l->def.type ) );
		for ( int k = 0; k < 2; ++k )
		{
			h = lpHashBytes( h, &l->ends[k].piece, sizeof( int ) );
			h = lpHashBytes( h, &l->ends[k].generation, sizeof( uint32_t ) );
		}
		h = lpHashBytes( h, &l->health, sizeof( float ) );
		h = lpHashBytes( h, &l->strain, sizeof( float ) );
		h = lpHashBytes( h, &l->utilization, sizeof( float ) );
		if ( lpHasMotor( l ) ) // only motorised links: old hashes stay valid
		{
			h = lpHashBytes( h, &l->target, sizeof( float ) );
			h = lpHashBytes( h, &l->targetRotation, sizeof( lpQuat ) );
			h = lpHashBytes( h, &l->motorCap, sizeof( float ) );
			if ( l->feed != 0.0f ) // only rigs set it: old hashes stay valid
			{
				h = lpHashBytes( h, &l->feed, sizeof( float ) );
			}
		}
	}
	h = w->vehicles.count > 0 ? lpHashVehicles( w, h ) : h; // only with vehicles: old hashes stay valid
	h = w->rigs.count > 0 ? lpHashRigs( w, h ) : h;
	return w->pools.count > 0 ? lpHashPools( w, h ) : h; // only with pools: old hashes stay valid
}

static bool lpLinkFail( const char* message, int a, int b )
{
	fprintf( stderr, "lpWorld_Validate: " );
	fprintf( stderr, message, a, b );
	fprintf( stderr, "\n" );
	return false;
}

bool lpValidateLinks( const lpWorld* w )
{
	int live = 0, ends = 0;
	for ( int i = 0; i < w->links.count; ++i )
	{
		const lpLink* l = w->links.data + i;
		if ( l->alive == false )
		{
			continue;
		}
		live += 1;
		if ( l->wheel >= 0 && lpValidateWheel( w, i ) == false )
		{
			return false;
		}
		int bodies[2] = { -1, -1 };
		for ( int k = 0; k < 2; ++k )
		{
			const lpLinkEnd* e = l->ends + k;
			if ( e->piece < 0 )
			{
				if ( LP_PHYS_NULL( l->anchor[k] ) && l->wheel < 0 )
				{
					return lpLinkFail( "link %d end %d is on the world but has no anchor body", i, k );
				}
				continue;
			}
			ends += 1;
			const lpPiece* p = w->pieces.data + e->piece;
			if ( p->shape == NULL || p->generation != e->generation || p->body < 0 )
			{
				return lpLinkFail( "link %d end %d is on a freed or reused piece", i, k );
			}
			const lpBody* b = w->bodies.data + p->body;
			if ( b->alive == false || LP_PHYS_NULL( b->id ) )
			{
				return lpLinkFail( "link %d end %d is on a body with no physics body", i, k );
			}
			bodies[k] = p->body;
			bool listed = false;
			for ( int n = 0; n < p->links.count; ++n )
			{
				listed = listed || p->links.data[n] == i;
			}
			if ( listed == false )
			{
				return lpLinkFail( "link %d is missing from the links of piece %d", i, e->piece );
			}
		}
		if ( bodies[0] >= 0 && bodies[0] == bodies[1] )
		{
			return lpLinkFail( "link %d has both ends on body %d", i, bodies[0] );
		}
		lpPhysBody jointA = lp_nullPhysBody;
		lpPhysBody jointB = lp_nullPhysBody;
		if ( l->wheel < 0 && lpPhys_IsValidJoint( w->phys, l->joint ) )
		{
			lpPhys_GetJointBodies( w->phys, l->joint, &jointA, &jointB );
		}
		if ( l->wheel < 0 && ( lpPhys_IsValidJoint( w->phys, l->joint ) == false ||
								LP_PHYS_EQUAL( jointA, lpEndBody( w, l, 0 ) ) == false ||
								LP_PHYS_EQUAL( jointB, lpEndBody( w, l, 1 ) ) == false ) )
		{
			return lpLinkFail( "link %d has a joint that is not on its ends' bodies (%d)", i, 0 );
		}
	}
	if ( live != w->linkCount )
	{
		return lpLinkFail( "link count %d, live %d", w->linkCount, live );
	}
	int refs = 0;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		refs += w->pieces.data[i].shape != NULL ? w->pieces.data[i].links.count : 0;
	}
	if ( refs != ends )
	{
		return lpLinkFail( "pieces list %d link ends, links have %d", refs, ends );
	}
	return true;
}
