// SPDX-License-Identifier: MIT

#include "drive.h"

#include <stdio.h>

int Drive_Nearest( const lpWorld* world, V3 point, float reach )
{
	int best = -1;
	float nearest = reach * reach;
	for ( int v = 0; v < lpWorld_GetVehicleCapacity( world ); ++v )
	{
		lpVehicleState s = lpWorld_GetVehicleState( world, v );
		if ( s.alive == false || s.body < 0 )
		{
			continue;
		}
		V3 d = V3{ (float)s.position.x, (float)s.position.y, (float)s.position.z } - point;
		if ( Dot( d, d ) < nearest )
		{
			nearest = Dot( d, d );
			best = v;
		}
	}
	return best;
}

lpVehicleControl Drive_Control( const lpWorld* world, int vehicle, const DriveKeys& keys )
{
	lpVehicleState s = lpWorld_GetVehicleState( world, vehicle );
	lpVehicleControl c = {};
	if ( keys.forward )
	{
		c.throttle = 1.0f;
	}
	if ( keys.back )
	{
		if ( s.speed > 1.0f )
		{
			c.brake = 1.0f;
		}
		else
		{
			c.throttle = -0.6f;
		}
	}
	c.steer = ( keys.right ? 1.0f : 0.0f ) - ( keys.left ? 1.0f : 0.0f );
	c.handbrake = keys.handbrake;
	return c;
}

bool Drive_Same( const lpVehicleControl& a, const lpVehicleControl& b )
{
	return a.throttle == b.throttle && a.brake == b.brake && a.steer == b.steer && a.handbrake == b.handbrake;
}

void Drive_Camera( const lpWorld* world, int vehicle, float dt, V3* position, float* yaw, float* pitch )
{
	lpVehicleState s = lpWorld_GetVehicleState( world, vehicle );
	if ( s.body < 0 )
	{
		return;
	}
	V3 car = { (float)s.position.x, (float)s.position.y, (float)s.position.z };
	V3 flat = Normalize( V3{ s.forward.x, 0.0f, s.forward.z } );
	V3 want = car - 8.0f * flat + V3{ 0.0f, 3.2f, 0.0f };
	float ease = 1.0f - expf( -4.0f * dt );
	*position = *position + ease * ( want - *position );
	V3 look = car + 3.0f * flat + V3{ 0.0f, 0.8f, 0.0f } - *position;
	*yaw = atan2f( look.x, -look.z );
	*pitch = atan2f( look.y, sqrtf( look.x * look.x + look.z * look.z ) );
}

void Drive_Describe( const lpWorld* world, int vehicle, char* text, int size )
{
	lpVehicleState s = lpWorld_GetVehicleState( world, vehicle );
	snprintf( text, (size_t)size, "car %d: %.0f km/h  wheels %d on, %d down  power %.0f%%  throttle %.1f brake %.1f steer %.1f%s",
			  vehicle, 3.6f * s.speed, s.attached, s.grounded, 100.0f * s.power, s.control.throttle, s.control.brake,
			  s.control.steer, s.control.handbrake ? "  handbrake" : "" );
}

// ---- rigs ----

int Walk_Nearest( const lpWorld* world, V3 point, float reach, float* distance )
{
	int best = -1;
	float nearest = reach * reach;
	for ( int r = 0; r < lpWorld_GetRigCapacity( world ); ++r )
	{
		lpRigState s = lpWorld_GetRigState( world, r );
		if ( s.alive == false || s.body < 0 )
		{
			continue;
		}
		V3 d = V3{ (float)s.position.x, (float)s.position.y, (float)s.position.z } - point;
		if ( Dot( d, d ) < nearest )
		{
			nearest = Dot( d, d );
			best = r;
		}
	}
	*distance = sqrtf( nearest );
	return best;
}

lpRigControl Walk_Control( const WalkKeys& keys )
{
	lpRigControl c = {};
	c.forward = ( keys.forward ? 1.0f : 0.0f ) - ( keys.back ? 0.5f : 0.0f );
	c.turn = ( keys.right ? 1.0f : 0.0f ) - ( keys.left ? 1.0f : 0.0f );
	c.strafe = ( keys.strafeRight ? 0.6f : 0.0f ) - ( keys.strafeLeft ? 0.6f : 0.0f );
	c.crouch = keys.crouch ? 1.0f : 0.0f;
	return c;
}

bool Walk_Same( const lpRigControl& a, const lpRigControl& b )
{
	return a.forward == b.forward && a.strafe == b.strafe && a.turn == b.turn && a.crouch == b.crouch;
}

void Walk_Camera( const lpWorld* world, int rig, float dt, V3* position, float* yaw, float* pitch )
{
	lpRigState s = lpWorld_GetRigState( world, rig );
	if ( s.body < 0 )
	{
		return;
	}
	V3 torso = { (float)s.position.x, (float)s.position.y, (float)s.position.z };
	V3 flat = Normalize( V3{ s.forward.x, 0.0f, s.forward.z } );
	V3 want = torso - 11.0f * flat + V3{ 0.0f, 4.5f, 0.0f };
	float ease = 1.0f - expf( -3.0f * dt );
	*position = *position + ease * ( want - *position );
	V3 look = torso + 3.0f * flat - *position;
	*yaw = atan2f( look.x, -look.z );
	*pitch = atan2f( look.y, sqrtf( look.x * look.x + look.z * look.z ) );
}

void Walk_Describe( const lpWorld* world, int rig, char* text, int size )
{
	lpRigState s = lpWorld_GetRigState( world, rig );
	// The pool on its torso, if it has one (the mech's hydraulic reservoir)
	float fluid = -1.0f, leak = 0.0f;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( world ) && fluid < 0.0f && s.body >= 0; ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( world, i );
		fluid = info.body == s.body ? lpWorld_GetPiecePool( world, i, &leak ) : -1.0f;
	}
	char pool[48] = "";
	if ( fluid >= 0.0f )
	{
		snprintf( pool, sizeof( pool ), "  fluid %.0f%%%s", 100.0f * fluid, leak > 0.0f ? " (leaking)" : "" );
	}
	char arm[48] = "";
	for ( int i = 0; i < s.limbCount && arm[0] == 0; ++i )
	{
		lpLimbState l = lpWorld_GetLimbState( world, rig, i );
		if ( l.reaching )
		{
			snprintf( arm, sizeof( arm ), "  leg %d striking%s", i, l.touching >= 0 ? ", touching" : "" );
		}
	}
	snprintf( text, (size_t)size, "mech %d: %.1f m/s  legs %d on, %d able, %d down  height %.2f m%s%s%s  forward %.1f turn %.1f strafe %.1f%s%s",
			  rig, s.speed, s.attached, s.able, s.planted, s.height, s.idle ? " (idle)" : "", s.crawling ? " crawling" : "", pool,
			  s.control.forward, s.control.turn, s.control.strafe, s.control.crouch > 0.0f ? "  crouched" : "", arm );
}

// The rig's own bodies: its torso, its feet, and what its hinges join to them (a grip is a weld: a held crate is not)
static bool Walk_Own( const lpWorld* world, int rig, int body )
{
	lpRigState s = lpWorld_GetRigState( world, rig );
	int own[1 + LP_MAX_RIG_LIMBS];
	int count = 0;
	own[count++] = s.body;
	for ( int i = 0; i < s.limbCount; ++i )
	{
		own[count++] = lpWorld_GetLimbState( world, rig, i ).footBody;
	}
	for ( int k = 0; k < count; ++k )
	{
		if ( own[k] >= 0 && own[k] == body )
		{
			return true;
		}
	}
	for ( int l = 0; l < lpWorld_GetLinkCapacity( world ); ++l )
	{
		lpLinkState link = lpWorld_GetLinkState( world, l );
		if ( link.alive == false || link.type != lp_linkHinge || ( link.bodyA != body && link.bodyB != body ) )
		{
			continue;
		}
		int other = link.bodyA == body ? link.bodyB : link.bodyA;
		for ( int k = 0; k < count; ++k )
		{
			if ( own[k] >= 0 && own[k] == other )
			{
				return true;
			}
		}
	}
	return false;
}

// A ray that passes through the rig
static lpRayHit Walk_Cast( const lpWorld* world, int rig, V3 origin, V3 dir, float range )
{
	lpRayHit hit = {};
	for ( int k = 0; k < 8 && range > 0.0f; ++k )
	{
		hit = lpWorld_CastRay( world, b3Pos{ origin.x, origin.y, origin.z }, b3Vec3{ range * dir.x, range * dir.y, range * dir.z } );
		if ( hit.hit == false || hit.body < 0 || Walk_Own( world, rig, hit.body ) == false )
		{
			return hit;
		}
		V3 p = { (float)hit.point.x, (float)hit.point.y, (float)hit.point.z };
		range -= Dot( p - origin, dir ) + 0.02f;
		origin = p + 0.02f * dir;
	}
	hit.hit = false;
	return hit;
}

bool Walk_Aim( const lpWorld* world, int rig, V3 origin, V3 dir, int* limb, V3* point )
{
	const float strikeReach = 1.0f; // m ahead of where the foot stands
	const float through = 0.15f;	// m past the surface: a stomp strikes through it, a claw rests against it
	lpRayHit hit = Walk_Cast( world, rig, origin, dir, 60.0f );
	if ( hit.hit == false )
	{
		return false;
	}
	V3 target = V3{ (float)hit.point.x, (float)hit.point.y, (float)hit.point.z } + through * dir;
	lpRigState s = lpWorld_GetRigState( world, rig );
	*limb = -1;
	float nearest = 0.0f;
	V3 foot = {};
	for ( int i = 0; i < s.limbCount; ++i )
	{
		lpLimbState l = lpWorld_GetLimbState( world, rig, i );
		V3 f = { (float)l.foot.x, (float)l.foot.y, (float)l.foot.z };
		float d = Dot( f - target, f - target );
		if ( l.able && ( *limb < 0 || d < nearest ) )
		{
			*limb = i;
			nearest = d;
			foot = f;
		}
	}
	if ( *limb < 0 )
	{
		return false;
	}
	V3 away = { target.x - foot.x, 0.0f, target.z - foot.z };
	float far = sqrtf( Dot( away, away ) );
	if ( far > strikeReach )
	{
		V3 above = foot + ( strikeReach / far ) * away + V3{ 0.0f, 2.5f, 0.0f };
		lpRayHit ground = Walk_Cast( world, rig, above, V3{ 0.0f, -1.0f, 0.0f }, 5.0f );
		if ( ground.hit == false )
		{
			return false;
		}
		target = V3{ (float)ground.point.x, (float)ground.point.y - through, (float)ground.point.z };
	}
	*point = target;
	return true;
}
