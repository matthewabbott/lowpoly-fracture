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
