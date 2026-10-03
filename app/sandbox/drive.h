// SPDX-License-Identifier: MIT
// Driving a vehicle or walking a rig from the keyboard, and a camera that chases it. The controls become recorded drive
// and walk events (the simulation sees only those); the camera is render-only.
#pragma once

#include "math3d.h"

#include "lpf/lpf.h"

struct DriveKeys
{
	bool forward, back, left, right, handbrake;
};

// The vehicle whose body's centre of mass is nearest the point, within reach, free or this peer's (another player's is
// not); -1 if none
int Drive_Nearest( const lpWorld* world, V3 point, float reach, int peer );

// W drives; S brakes while rolling forward, then reverses; A and D steer; space pulls the handbrake
lpVehicleControl Drive_Control( const lpWorld* world, int vehicle, const DriveKeys& keys );

bool Drive_Same( const lpVehicleControl& a, const lpVehicleControl& b );

// Eases the camera toward a point behind and above the vehicle and aims it there. Leaves it alone once the vehicle
// has no wheels left.
void Drive_Camera( const lpWorld* world, int vehicle, float dt, V3* position, float* yaw, float* pitch );

// One HUD line: speed, wheels, controls
void Drive_Describe( const lpWorld* world, int vehicle, char* text, int size );

// ---- rigs ----

struct WalkKeys
{
	bool forward, back, left, right, strafeLeft, strafeRight, crouch;
};

// The rig whose torso is nearest the point, within reach, free or this peer's; -1 if none
int Walk_Nearest( const lpWorld* world, V3 point, float reach, int peer, float* distance );

// W walks, S backs off at half speed, A and D turn, Q and E step sideways, C crouches
lpRigControl Walk_Control( const WalkKeys& keys );

bool Walk_Same( const lpRigControl& a, const lpRigControl& b );

// Eases the camera toward a point behind and above the rig and aims it there
void Walk_Camera( const lpWorld* world, int rig, float dt, V3* position, float* yaw, float* pitch );

// One HUD line: speed, legs, height, controls, a leg that strikes or grips
void Walk_Describe( const lpWorld* world, int rig, char* text, int size );

// F: where the rig strikes, and with which leg. The crosshair ray hits something that is not the rig itself; the able
// leg whose foot is nearest strikes it, a little past its surface. Out of that leg's reach, it strikes the ground (or
// what is on it) as far toward it as it can. False if the ray hits nothing or no leg is able.
bool Walk_Aim( const lpWorld* world, int rig, V3 origin, V3 dir, int* limb, V3* point );
