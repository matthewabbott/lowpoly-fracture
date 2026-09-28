// SPDX-License-Identifier: MIT
// Links: Box3D joints between objects (weld, hinge, ball, rope) that break under load and outlive the pieces they
// were made on.
//
// A link end is a piece and a frame in its body's frame. A piece's body frame never changes (new bodies are made at
// their parent's transform), so an end stays valid through splits and tier changes; only the Box3D joint has to be
// rebuilt when a piece's body changes, and lpSyncLinks does that just before the physics step, when every body of
// the step exists. When a piece fractures, each link end on it moves to the kept cell holding its anchor, and the link
// breaks if that cell was blown out. An end whose piece leaves Box3D (freed, or made a ghost or scrap) breaks its link
// at once, so a live link always has both ends on live Box3D bodies. A rebuilt link between two moving bodies tears
// when one of them is a chip next to the other.
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
	def.axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	def.dampingRatio = 1.0f;
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
static b3Quat lpAxisRotation( b3Vec3 axis )
{
	float length = b3Length( axis );
	b3Vec3 z = length > 1e-6f ? b3MulSV( 1.0f / length, axis ) : (b3Vec3){ 0.0f, 0.0f, 1.0f };
	return b3ComputeQuatBetweenUnitVectors( (b3Vec3){ 0.0f, 0.0f, 1.0f }, z );
}

// The piece of a body nearest a world point, and how far outside it the point is
static int lpNearestPiece( const lpWorld* w, int bodyIndex, b3Pos point, float* distance )
{
	const lpBody* b = w->bodies.data + bodyIndex;
	b3Vec3 local = b3InvTransformWorldPoint( lpGetTransform( b ), point );
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

static b3BodyId lpEndBody( const lpWorld* w, const lpLink* l, int k )
{
	int piece = l->ends[k].piece;
	return piece < 0 ? l->anchor[k] : w->bodies.data[w->pieces.data[piece].body].id;
}

static void lpBuildJoint( lpWorld* w, int index )
{
	lpLink* l = w->links.data + index;
	const lpLinkDef* d = &l->def;
	b3BodyId a = lpEndBody( w, l, 0 );
	b3BodyId b = lpEndBody( w, l, 1 );
	b3JointDef base = b3DefaultWeldJointDef().base; // the common part is the same for every type
	base.bodyIdA = a;
	base.bodyIdB = b;
	base.localFrameA = l->ends[0].frame;
	base.localFrameB = l->ends[1].frame;
	base.collideConnected = d->collideConnected;
	base.userData = (void*)(intptr_t)( index + 1 );

	b3WorldId physics = w->def.physics;
	switch ( d->type )
	{
		case lp_linkWeld:
		{
			b3WeldJointDef jd = b3DefaultWeldJointDef();
			jd.base = base;
			jd.linearHertz = d->hertz;
			jd.angularHertz = d->hertz;
			jd.linearDampingRatio = d->dampingRatio;
			jd.angularDampingRatio = d->dampingRatio;
			l->joint = b3CreateWeldJoint( physics, &jd );
			break;
		}
		case lp_linkHinge:
		{
			b3RevoluteJointDef jd = b3DefaultRevoluteJointDef();
			jd.base = base;
			jd.enableLimit = d->lowerAngle < d->upperAngle;
			jd.lowerAngle = d->lowerAngle;
			jd.upperAngle = d->upperAngle;
			l->joint = b3CreateRevoluteJoint( physics, &jd );
			break;
		}
		case lp_linkBall:
		{
			b3SphericalJointDef jd = b3DefaultSphericalJointDef();
			jd.base = base;
			jd.enableConeLimit = d->coneAngle > 0.0f;
			jd.coneAngle = d->coneAngle;
			l->joint = b3CreateSphericalJoint( physics, &jd );
			break;
		}
		default:
		{
			// A rope: no spring force, only the upper length limit, so it goes slack when pushed together
			b3DistanceJointDef jd = b3DefaultDistanceJointDef();
			jd.base = base;
			jd.length = d->length;
			jd.enableSpring = true;
			jd.hertz = 0.0f;
			jd.enableLimit = true;
			jd.minLength = 0.0f;
			jd.maxLength = d->length;
			l->joint = b3CreateDistanceJoint( physics, &jd );
			break;
		}
	}
	LP_ASSERT( B3_IS_NON_NULL( l->joint ) );
	l->builtOn[0] = a;
	l->builtOn[1] = b;
	l->settle = LP_LINK_SETTLE;
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
	return index;
}

int lpCreateLink( lpWorld* w, const lpLinkDef* def )
{
	if ( def->type < 0 || def->type >= lp_linkTypeCount || def->bodyA == def->bodyB || w->linkCount >= w->def.maxLinks )
	{
		return -1;
	}
	int bodies[2] = { def->bodyA, def->bodyB };
	b3Pos points[2] = { def->anchorA, def->type == lp_linkRope ? def->anchorB : def->anchorA };
	b3Quat q = lpAxisRotation( def->axis );
	lpLinkEnd ends[2];
	for ( int k = 0; k < 2; ++k )
	{
		if ( bodies[k] < 0 )
		{
			ends[k] = (lpLinkEnd){ -1, 0, { b3Vec3_zero, q } }; // the anchor body sits at the point, unrotated
			continue;
		}
		if ( bodies[k] >= w->bodies.count || w->bodies.data[bodies[k]].alive == false ||
			 B3_IS_NULL( w->bodies.data[bodies[k]].id ) )
		{
			return -1; // gone, or a ghost or scrap body
		}
		float distance;
		int piece = lpNearestPiece( w, bodies[k], points[k], &distance );
		if ( piece < 0 || distance > LP_LINK_REACH )
		{
			return -1;
		}
		b3WorldTransform xf = lpGetTransform( w->bodies.data + bodies[k] );
		ends[k].piece = piece;
		ends[k].generation = w->pieces.data[piece].generation;
		ends[k].frame.p = b3InvTransformWorldPoint( xf, points[k] );
		ends[k].frame.q = b3InvMulQuat( xf.q, q );
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
	if ( def->type == lp_linkRope && l->def.length <= 0.0f )
	{
		l->def.length = b3Length( b3SubPos( points[1], points[0] ) );
	}
	l->def.length = b3MaxFloat( l->def.length, 0.01f );
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
			b3BodyDef bd = b3DefaultBodyDef();
			bd.type = b3_staticBody;
			bd.position = points[k];
			l->anchor[k] = b3CreateBody( w->def.physics, &bd );
		}
	}
	lpBuildJoint( w, index );
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
		b3WorldTransform xf = lpGetTransform( w->bodies.data + p->body );
		uint64_t h = lpMix64( ( w->tick << 24 ) ^ (uint64_t)( 2 * index + k ) );
		for ( int m = 0; m < motes; ++m )
		{
			h = lpMix64( h + (uint64_t)m );
			float rx = (float)( h & 0xFFFF ) / 65535.0f - 0.5f;
			float rz = (float)( ( h >> 16 ) & 0xFFFF ) / 65535.0f - 0.5f;
			float rs = (float)( ( h >> 32 ) & 0xFFFF ) / 65535.0f;
			b3Vec3 v = { 0.8f * rx, -0.2f - 0.4f * rs, 0.8f * rz };
			lpEmitParticle( w, xf, l->ends[k].frame.p, v, 0.02f + 0.02f * rs, p->material );
		}
	}
}

static void lpReleaseLink( lpWorld* w, int index )
{
	lpLink* l = w->links.data + index;
	LP_ASSERT( l->alive );
	if ( b3Joint_IsValid( l->joint ) )
	{
		b3DestroyJoint( l->joint, true ); // what it held falls
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
				// Its pull is gone: check the structure again with fresh loads. Queued, not marked dirty now, since a
				// link can break in the middle of the step's splits.
				b->solving = false;
				b->creaking = false;
				lpArray_Push( w->stressAgain, p->body );
			}
		}
		else if ( B3_IS_NON_NULL( l->anchor[k] ) )
		{
			b3DestroyBody( l->anchor[k] );
		}
	}
	l->alive = false;
	l->joint = b3_nullJointId;
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
	lpReleaseLink( w, index );
	w->stats.linkBreaks += 1;
}

void lpDestroyLink( lpWorld* w, int link )
{
	if ( link >= 0 && link < w->links.count && w->links.data[link].alive )
	{
		lpReleaseLink( w, link );
	}
}

// The piece is leaving Box3D (freed, or made a ghost or scrap): its links go with it
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
		// Kept cells stay on the same body, and the frame is in its frame: the joint holds on unchanged
		lpLinkEnd* e = w->links.data[move.link].ends + move.end;
		e->piece = child;
		e->generation = w->pieces.data[child].generation;
		lpArray_Push( w->pieces.data[child].links, move.link );
	}
	w->scratchLinkMoves.count = 0;
}

// A chip left holding a much heavier body by a link would jitter on the joint (or blow it up): it tears off instead
static bool lpTearsOff( b3BodyId a, b3BodyId b )
{
	if ( b3Body_GetType( a ) != b3_dynamicBody || b3Body_GetType( b ) != b3_dynamicBody )
	{
		return false;
	}
	float ma = b3Body_GetMass( a );
	float mb = b3Body_GetMass( b );
	return b3MinFloat( ma, mb ) < LP_LINK_TEAR_RATIO * b3MaxFloat( ma, mb );
}

void lpWorld_SetRopeLength( lpWorld* w, int link, float length )
{
	if ( link < 0 || link >= w->links.count || w->links.data[link].alive == false || w->links.data[link].def.type != lp_linkRope )
	{
		return;
	}
	lpLink* l = w->links.data + link;
	l->def.length = b3MaxFloat( length, 0.01f );
	b3DistanceJoint_SetLengthRange( l->joint, 0.0f, l->def.length );
	b3Joint_WakeBodies( l->joint );
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
		b3BodyId bodies[2];
		for ( int k = 0; k < 2; ++k )
		{
			if ( l->ends[k].piece >= 0 )
			{
				w->bodies.data[w->pieces.data[l->ends[k].piece].body].linkStamp = stamp;
			}
			bodies[k] = lpEndBody( w, l, k );
		}
		if ( b3Joint_IsValid( l->joint ) && B3_ID_EQUALS( bodies[0], l->builtOn[0] ) && B3_ID_EQUALS( bodies[1], l->builtOn[1] ) )
		{
			continue;
		}
		// An end moved to a new body (a split, an ejected cell, a ghost made whole again), or its old body is gone
		if ( b3Joint_IsValid( l->joint ) )
		{
			b3DestroyJoint( l->joint, false );
		}
		if ( lpTearsOff( bodies[0], bodies[1] ) )
		{
			lpBreakLink( w, i, true );
			continue;
		}
		lpBuildJoint( w, i );
		w->stats.linkRebuilds += 1;
	}
}

bool lpBodyLinked( const lpWorld* w, const lpBody* b )
{
	return b->linkStamp == w->tick + 1;
}

// A structure at either end carries the link's pull in its stress solve (sampled with its loads): check it again when
// the pull has changed by a quarter since, at most every 30 steps (a swinging load pulls back and forth)
static void lpRecheckStructures( lpWorld* w, lpLink* l )
{
	float pull = b3Length( l->force );
	if ( b3AbsFloat( pull - l->stressForce ) <= 0.25f * b3MaxFloat( pull, l->stressForce ) + 10.0f ||
		 ( l->recheckTick != 0 && w->tick + 1 < l->recheckTick + 30 ) )
	{
		return;
	}
	l->stressForce = pull; // with no structure at either end there is nothing to check: stop asking too
	for ( int k = 0; k < 2; ++k )
	{
		int piece = l->ends[k].piece;
		int bodyIndex = piece >= 0 ? w->pieces.data[piece].body : -1;
		if ( bodyIndex >= 0 && w->bodies.data[bodyIndex].kind == lp_kindStructure )
		{
			lpBody* b = w->bodies.data + bodyIndex;
			b->solving = false; // sample the new loads
			b->creaking = false;
			lpMarkDirty( w, bodyIndex );
			l->recheckTick = w->tick + 1;
		}
	}
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
		b3BodyId a = l->builtOn[0];
		b3BodyId b = l->builtOn[1];
		if ( b3Body_IsAwake( a ) == false && b3Body_IsAwake( b ) == false )
		{
			if ( l->settle == 0 )
			{
				lpRecheckStructures( w, l ); // one held back by the rate limit still gets through
			}
			continue; // asleep: nothing moved, nothing changed
		}
		l->points[0] = b3TransformWorldPoint( b3Body_GetTransform( a ), l->ends[0].frame.p );
		l->points[1] = b3TransformWorldPoint( b3Body_GetTransform( b ), l->ends[1].frame.p );
		l->force = b3Joint_GetConstraintForce( l->joint );
		l->torque = b3Joint_GetConstraintTorque( l->joint );
		if ( l->settle > 0 )
		{
			l->settle -= 1;
			continue;
		}
		lpRecheckStructures( w, l );

		float u = 0.0f;
		if ( l->def.maxForce > 0.0f )
		{
			u = b3Length( l->force ) / l->def.maxForce;
		}
		if ( l->def.maxTorque > 0.0f )
		{
			u = b3MaxFloat( u, b3Length( l->torque ) / l->def.maxTorque );
		}
		if ( l->def.strength > 0.0f )
		{
			u /= b3MaxFloat( l->health / l->def.strength, 0.1f );
		}
		u = b3MinFloat( u, LP_LINK_CLAMP );
		l->utilization += 0.5f * ( u - l->utilization );
		if ( l->utilization >= 2.0f )
		{
			lpBreakLink( w, i, true );
			continue;
		}
		if ( l->utilization > 1.0f )
		{
			l->strain += ( l->utilization - 1.0f ) * w->def.strainRate * LP_LINK_STRAIN * timeStep;
			if ( l->strain >= 1.0f )
			{
				lpBreakLink( w, i, true );
				continue;
			}
			if ( ( w->tick + (uint64_t)i ) % 6 == 0 )
			{
				lpLinkDust( w, l, i, 1 ); // creaking
			}
			b3Joint_WakeBodies( l->joint ); // keep straining until it holds or gives
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
	s.slack = l->def.type == lp_linkRope && b3Length( b3SubPos( s.pointB, s.pointA ) ) < l->def.length - 0.01f;
	s.force = l->force;
	s.torque = l->torque;
	s.utilization = l->utilization;
	s.strain = l->strain;
	s.health = l->health;
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
	}
	return h;
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
		int bodies[2] = { -1, -1 };
		for ( int k = 0; k < 2; ++k )
		{
			const lpLinkEnd* e = l->ends + k;
			if ( e->piece < 0 )
			{
				if ( B3_IS_NULL( l->anchor[k] ) )
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
			if ( b->alive == false || B3_IS_NULL( b->id ) )
			{
				return lpLinkFail( "link %d end %d is on a body with no Box3D body", i, k );
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
		if ( b3Joint_IsValid( l->joint ) == false || B3_ID_EQUALS( b3Joint_GetBodyA( l->joint ), lpEndBody( w, l, 0 ) ) == false ||
			 B3_ID_EQUALS( b3Joint_GetBodyB( l->joint ), lpEndBody( w, l, 1 ) ) == false )
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

void lpFreeLinks( lpWorld* w, bool physicsAlive )
{
	for ( int i = 0; i < w->links.count && physicsAlive; ++i )
	{
		const lpLink* l = w->links.data + i;
		for ( int k = 0; k < 2 && l->alive; ++k )
		{
			if ( l->ends[k].piece < 0 && B3_IS_NON_NULL( l->anchor[k] ) )
			{
				b3DestroyBody( l->anchor[k] ); // takes the joint with it
			}
		}
	}
	lpArray_Free( w->links );
}
