// SPDX-License-Identifier: MIT
// Driving a vehicle from the keyboard, and a camera that chases it. The controls become recorded drive events (the
// simulation sees only those); the camera is render-only.
#pragma once

#include "math3d.h"

#include "lpf/lpf.h"

struct DriveKeys
{
	bool forward, back, left, right, handbrake;
};

// The vehicle whose body's centre of mass is nearest the point, within reach; -1 if none
int Drive_Nearest( const lpWorld* world, V3 point, float reach );

// W drives; S brakes while rolling forward, then reverses; A and D steer; space pulls the handbrake
lpVehicleControl Drive_Control( const lpWorld* world, int vehicle, const DriveKeys& keys );

bool Drive_Same( const lpVehicleControl& a, const lpVehicleControl& b );

// Eases the camera toward a point behind and above the vehicle and aims it there. Leaves it alone once the vehicle
// has no wheels left.
void Drive_Camera( const lpWorld* world, int vehicle, float dt, V3* position, float* yaw, float* pitch );

// One HUD line: speed, wheels, controls
void Drive_Describe( const lpWorld* world, int vehicle, char* text, int size );
