// SPDX-License-Identifier: MIT
// lowpoly-fracture sandbox: smash, blow up and grab a low-poly world.
//
// Interactive:  sandbox --scene town
// Automation:   sandbox --scene walls --script scripts/walls_demo.txt --frames 300 --screenshot shot.png --hash-log h.txt
//               (--screenshot-at 60,120 saves shot_0060.png and shot_0120.png; --dump 120:state.json the state as JSON)
//
// Co-op:        sandbox --scene track --host 7777     and   sandbox --scene track --join 192.168.1.20:7777
//               (lockstep, app/net: the host keeps the clock; each player's commands apply --input-delay ticks later,
//               4 by default; V takes a free car or mech, or leaves it; a desync stops both and names what differs)
//
// The simulation runs at a fixed 60 Hz. With --frames it advances exactly one tick per rendered frame, so scripted
// runs are independent of machine speed and their hash logs are comparable across runs. In co-op, --frames N runs
// until the host has sent N ticks and both have stepped them.

#include "drive.h"
#include "math3d.h"
#include "renderer.h"

#include "dump.h"
#include "lockstep.h"
#include "scenes.h"
#include "script.h"

#include "imgui.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_imgui.h"
#include "sokol_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

namespace
{

// The tools are the replay scripts' (scenes/script.h), in their order, then the grab
enum Tool
{
	ToolRifle = lp_toolRifle,
	ToolGrenade = lp_toolGrenade,
	ToolCannon = lp_toolCannon,
	ToolHammer = lp_toolHammer,
	ToolBall = lp_toolBall,
	ToolFlask = lp_toolFlask,
	ToolPull = lp_toolCount,
	ToolCount
};

const char* kToolNames[ToolCount] = { "Rifle", "Grenade", "Cannon blast", "Sledgehammer", "Cannonball", "Volatile flask",
									  "Grab / pull" };

lpVec3 ToLp( V3 v )
{
	return lpVec3{ v.x, v.y, v.z };
}

struct Options
{
	int scene = lp_sceneWall;
	int workers = 4;
	int frames = 0; // 0 = interactive
	std::string screenshot;
	std::vector<int> shotsAt; // --screenshot-at: frames to save (numbered), instead of only the last
	std::vector<std::pair<int64_t, std::string>> dumps; // --dump tick:path, the state after that tick's step as JSON
	std::string script;
	std::string record;
	std::string hashLog;
	int bombard = 0;
	float fragmentScale = 1.0f;
	int maxDebris = 400;
	float renderScale = 1.0f;
	bool vsync = true;
	bool hideUi = false;
	int inputDelay = -1; // ticks every command of this player's waits before it applies (-1: 0 alone, 4 in co-op)
	int hostPort = -1;	 // co-op: host on this port (0: any free one)
	bool hostLoopback = false; // --host 127.0.0.1:port: listen on loopback only
	std::string joinHost; // co-op: join this host
	int joinPort = 7777;
	int peers = 1; // co-op: the players to wait for (the host's)
	bool follow = false; // the camera chases the vehicle or the rig this player's commands drive (else the scene's first rig)
	bool haveCamera = false;
	float camera[5] = {};
	int width = 1600;
	int height = 900;
};

struct App
{
	Options opt;
	lpWorld* world = nullptr;
	int64_t tick = 0;
	double accumulator = 0.0;
	bool paused = false;

	V3 camPos = { 0.0f, 2.0f, 8.0f };
	float yaw = 0.0f;
	float pitch = 0.0f;
	bool looking = false;
	bool keys[512] = {};
	float moveSpeed = 8.0f;

	int tool = ToolGrenade;
	bool firing = false;
	int fireCooldown = 0;

	// the script's next command to submit, and the number of this player's next live command (after any script's)
	int scriptNext = 0;
	uint32_t seq = 0x80000000u;
	// driving: the vehicle the keys drive here
	int driving = -1;
	lpVehicleControl sent = {};
	// walking: likewise for a rig
	int walking = -1;
	lpRigControl walkSent = {};
	// its arms: the leg F strikes with (-1: none)
	int striking = -1;

	// grab tool
	int grabPiece = -1;
	uint32_t grabGeneration = 0;
	V3 grabLocal = {};
	float grabDistance = 5.0f;

	lpScript script = {};
	FILE* recordFile = nullptr;
	FILE* hashFile = nullptr;

	// co-op (app/net): the session, its sockets, and this machine's peer (0: the host, or playing alone)
	lpLockstep* lockstep = nullptr;
	lpTcp* listener = nullptr;
	std::vector<lpTcp*> links;
	int peer = 0;

	std::vector<Particle> particles;
	std::vector<Particle> drawn; // particles plus rope segments, rebuilt each frame
	RenderSettings rs;
	bool showUi = true;
	bool showLinks = false; // overlay: every link, coloured by how close it is to its limit
	lpMat4 viewProj = {};
	int frame = 0;
	bool wantScreenshot = false;
	int screenshotCounter = 0;

	float stepMs = 0.0f;
	float frameMs = 0.0f;
	float renderMs = 0.0f;
	lpStats last = {};
	uint64_t lastHash = 0;
	bool quitting = false;
	double sumFrameMs = 0.0;
	double sumStepMs = 0.0;
	double sumRenderMs = 0.0;
	float maxFrameMs = 0.0f;
};

App app;

V3 Forward()
{
	return { sinf( app.yaw ) * cosf( app.pitch ), sinf( app.pitch ), -cosf( app.yaw ) * cosf( app.pitch ) };
}

void SetSceneCamera( int scene )
{
	app.pitch = -0.1f;
	app.yaw = 0.0f;
	switch ( scene )
	{
		case lp_sceneWall:
			app.camPos = { 0.0f, 2.2f, 8.0f };
			break;
		case lp_sceneHouse:
			app.camPos = { 2.0f, 3.2f, 8.0f };
			app.pitch = -0.15f;
			break;
		case lp_sceneTown:
			app.camPos = { -40.0f, 6.0f, 2.0f };
			app.yaw = 0.5f * 3.14159265f;
			app.pitch = -0.12f;
			break;
		case lp_sceneTower:
			app.camPos = { 0.0f, 4.0f, 12.0f };
			app.pitch = 0.2f;
			break;
		case lp_scenePile:
			app.camPos = { 0.0f, 6.0f, 10.0f };
			app.pitch = -0.3f;
			break;
		case lp_sceneLumber:
			app.camPos = { 0.0f, 2.0f, 3.0f };
			app.pitch = -0.12f;
			break;
		case lp_sceneRuins:
			app.camPos = { 2.0f, 3.0f, 8.0f };
			app.pitch = -0.08f;
			break;
		case lp_sceneYard:
			app.camPos = { -1.3f, 2.8f, 3.5f };
			app.pitch = -0.1f;
			break;
		case lp_sceneKeep:
			app.camPos = { 0.0f, 7.0f, 19.0f };
			app.pitch = -0.02f;
			break;
		case lp_sceneTrack:
			app.camPos = { 0.0f, 22.0f, 78.0f };
			app.pitch = -0.3f;
			break;
		case lp_sceneMech:
			app.camPos = { -9.0f, 6.0f, -32.0f };
			app.yaw = 0.95f * 3.14159265f;
			app.pitch = -0.2f;
			break;
		case lp_sceneContraption:
			app.camPos = { 0.0f, 7.5f, 9.0f };
			app.pitch = -0.65f;
			break;
		default:
			break;
	}
	if ( app.opt.haveCamera )
	{
		app.camPos = { app.opt.camera[0], app.opt.camera[1], app.opt.camera[2] };
		app.yaw = app.opt.camera[3] * 3.14159265f / 180.0f;
		app.pitch = app.opt.camera[4] * 3.14159265f / 180.0f;
	}
}

void DestroyWorld()
{
	if ( app.world != nullptr )
	{
		lpDestroyWorld( app.world );
		app.world = nullptr;
	}
}

void LoadScene( int scene )
{
	DestroyWorld();
	lpWorldDef ld = lpDefaultWorldDef();
	ld.fragmentScale = app.opt.fragmentScale;
	ld.maxFullDebris = app.opt.maxDebris;
	ld.workerCount = app.opt.workers;
	ld.debugLog = getenv( "LPF_DEBUG" ) != nullptr;
	app.world = lpCreateWorld( &ld );
	lpBuildScene( app.world, scene );

	app.opt.scene = scene;
	app.tick = 0;
	app.accumulator = 0.0;
	app.scriptNext = 0;
	app.driving = -1;
	app.sent = {};
	app.walking = -1;
	app.walkSent = {};
	app.striking = -1;
	app.particles.clear();
	Renderer_Reset();
	SetSceneCamera( scene );
}

void LoadScript( const std::string& path )
{
	if ( lpScriptLoad( &app.script, path.c_str() ) )
	{
		printf( "script: %d commands from %s\n", app.script.count, path.c_str() );
	}
}

// A command of this player's: it applies after the input delay (a recording writes what the world applied). In
// co-op it goes through the host, stamped there with this machine's peer.
void Submit( lpCommand c )
{
	if ( app.lockstep != nullptr )
	{
		lpLockstep_Command( app.lockstep, &c );
		return;
	}
	c.tick = app.tick + app.opt.inputDelay;
	c.peer = 0;
	c.seq = app.seq++;
	lpWorld_Submit( app.world, &c );
}

// The vehicle and the rig this player's commands drive (-1: none): the scene's drivers leave them alone
int ControlledVehicle()
{
	for ( int v = 0; v < lpWorld_GetVehicleCapacity( app.world ); ++v )
	{
		if ( lpWorld_GetVehicleState( app.world, v ).controller == app.peer )
		{
			return v;
		}
	}
	return -1;
}

int ControlledRig()
{
	for ( int r = 0; r < lpWorld_GetRigCapacity( app.world ); ++r )
	{
		if ( lpWorld_GetRigState( app.world, r ).controller == app.peer )
		{
			return r;
		}
	}
	return -1;
}

// The limb of the walked rig whose claw holds something (-1: none)
int GripLimb()
{
	int limbs = app.walking >= 0 ? lpWorld_GetRigState( app.world, app.walking ).limbCount : 0;
	for ( int k = 0; k < limbs; ++k )
	{
		if ( lpWorld_GetLimbState( app.world, app.walking, k ).grip >= 0 )
		{
			return k;
		}
	}
	return -1;
}

// The keys drive `app.driving`: a control command whenever the controls change
void QueueDrive( const lpVehicleControl& control )
{
	lpCommand c = {};
	c.kind = lp_commandVehicleControl;
	c.vehicleControl.vehicle = app.driving;
	c.vehicleControl.control = control;
	Submit( c );
	app.sent = control;
}

// The keys walk `app.walking`: a control command whenever the controls change
void QueueWalk( const lpRigControl& control )
{
	lpCommand c = {};
	c.kind = lp_commandRigControl;
	c.rigControl.rig = app.walking;
	c.rigControl.control = control;
	Submit( c );
	app.walkSent = control;
}

void StrikeKey( bool down );

// V: get into the nearest vehicle or rig, or out of the one being driven (a car brakes and parks, a rig stands)
void ToggleDriving()
{
	if ( app.driving >= 0 )
	{
		lpVehicleControl park = {};
		park.brake = 1.0f;
		park.handbrake = true;
		QueueDrive( park );
		app.driving = -1;
		return;
	}
	if ( app.walking >= 0 )
	{
		QueueWalk( lpRigControl{} );
		StrikeKey( false ); // a striking leg steps back; a claw keeps its grip
		app.walking = -1;
		return;
	}
	float rigDistance = 0.0f;
	int rig = Walk_Nearest( app.world, app.camPos, 25.0f, app.peer, &rigDistance ); // free, or this player's already
	int car = Drive_Nearest( app.world, app.camPos, rig >= 0 ? rigDistance : 25.0f, app.peer );
	if ( car >= 0 )
	{
		app.driving = car;
		QueueDrive( lpVehicleControl{} );
	}
	else if ( rig >= 0 )
	{
		app.walking = rig;
		QueueWalk( lpRigControl{} );
	}
}

void QueueReach( int limb, bool active, V3 point )
{
	lpCommand c = {};
	c.kind = lp_commandLimbTarget;
	c.limbTarget.rig = app.walking;
	c.limbTarget.limb = limb;
	c.limbTarget.active = active;
	c.limbTarget.point = ToLp( point );
	Submit( c );
}

// F held: the walked rig strikes at what the crosshair is on, with the leg nearest it; let go, the leg steps back into
// the gait (unless its claw holds something: G lets go)
void StrikeKey( bool down )
{
	if ( app.walking < 0 )
	{
		return;
	}
	int limb = -1;
	V3 point = {};
	int grip = GripLimb();
	if ( down && Walk_Aim( app.world, app.walking, app.camPos, Forward(), &limb, &point ) )
	{
		if ( app.striking >= 0 && app.striking != limb && grip < 0 )
		{
			QueueReach( app.striking, false, V3{} );
		}
		app.striking = grip >= 0 ? grip : limb;
		QueueReach( app.striking, true, point );
	}
	else if ( down == false && app.striking >= 0 && grip < 0 )
	{
		QueueReach( app.striking, false, V3{} );
		app.striking = -1;
	}
}

// G: the striking leg's claw grabs what it touches and lifts it; G again lets go, and the leg steps back into the gait
void ClawKey()
{
	if ( app.walking < 0 )
	{
		return;
	}
	lpCommand c = {};
	c.kind = lp_commandClaw;
	c.claw.rig = app.walking;
	c.claw.maxForce = 40000.0f; // the hexapod's claw
	c.claw.maxTorque = 15000.0f;
	c.claw.strength = 5000.0f;
	int grip = GripLimb();
	if ( grip >= 0 )
	{
		c.claw.limb = grip;
		c.claw.mode = lp_clawRelease;
		Submit( c );
		QueueReach( grip, false, V3{} );
		app.striking = -1;
		return;
	}
	lpLimbState st = app.striking >= 0 ? lpWorld_GetLimbState( app.world, app.walking, app.striking ) : lpLimbState{};
	if ( app.striking < 0 || st.touching < 0 )
	{
		return;
	}
	c.claw.limb = app.striking;
	c.claw.mode = lp_clawGrab;
	Submit( c );
	QueueReach( app.striking, true, V3{ (float)st.foot.x, (float)st.foot.y + 0.8f, (float)st.foot.z } );
}

void QueueFire()
{
	Submit( lpScriptToolCommand( app.tool, ToLp( app.camPos ), ToLp( Forward() ) ) );
}

// Start holding whatever loose piece is under the crosshair
void BeginGrab()
{
	V3 f = Forward();
	lpVec3 origin = { app.camPos.x, app.camPos.y, app.camPos.z };
	lpRayHit hit = lpWorld_CastRay( app.world, origin, lpVec3{ 40.0f * f.x, 40.0f * f.y, 40.0f * f.z } );
	app.grabPiece = -1;
	if ( hit.hit && hit.piece >= 0 )
	{
		app.grabPiece = hit.piece;
		app.grabGeneration = hit.pieceGeneration;
		lpVec3 local = lpWorld_ToBodyFrame( app.world, hit.piece, hit.point );
		app.grabLocal = { local.x, local.y, local.z };
		V3 p = { (float)hit.point.x, (float)hit.point.y, (float)hit.point.z };
		app.grabDistance = sqrtf( Dot( p - app.camPos, p - app.camPos ) );
	}
}

// While held, one pull command per tick toward the point in front of the camera
void QueueGrab()
{
	if ( app.grabPiece < 0 )
	{
		return;
	}
	lpPieceInfo info = lpWorld_GetPieceInfo( app.world, app.grabPiece );
	if ( info.body < 0 || info.generation != app.grabGeneration )
	{
		app.grabPiece = -1;
		return;
	}
	V3 target = app.camPos + app.grabDistance * Forward();
	lpCommand c = {};
	c.kind = lp_commandPull;
	c.pull.piece = app.grabPiece;
	c.pull.generation = app.grabGeneration;
	c.pull.localPoint = ToLp( app.grabLocal );
	c.pull.target = ToLp( target );
	c.pull.maxAccel = 40.0f;
	c.pull.maxMass = 400.0f;
	Submit( c );
}

// Commands for this tick: the grab's, the script's (this player's) and the scene's (live ones were submitted as they
// came). In co-op the player's go through the host, and the scene's are made here from this machine's world.
void PreStep( int64_t tick )
{
	if ( app.firing && app.tool == ToolPull )
	{
		QueueGrab();
	}
	if ( app.lockstep != nullptr )
	{
		while ( app.scriptNext < app.script.count && app.script.commands[app.scriptNext].tick <= tick )
		{
			lpLockstep_Command( app.lockstep, app.script.commands + app.scriptNext );
			app.scriptNext += 1;
		}
	}
	else
	{
		app.scriptNext = lpScriptPlay( app.world, &app.script, app.scriptNext );
	}
	if ( app.opt.bombard > 0 )
	{
		lpSceneBombard( app.world, app.opt.scene, (int)tick, app.opt.bombard );
	}
	lpSceneDrive( app.world, app.opt.scene, (int)tick );
}

void PostStep();

void StepSimulation()
{
	PreStep( app.tick );
	uint64_t t0 = lpGetTicks();
	lpWorld_Step( app.world, 1.0f / 60.0f, 4 );
	app.stepMs = lpGetMilliseconds( t0 );
	PostStep();
}

void NetBefore( void* context, lpLockstep* lockstep, int64_t tick )
{
	(void)context;
	(void)lockstep;
	PreStep( tick );
}

// Co-op: the host closes up to `close` ticks (its clock), and every tick a packet has come for is stepped (a peer
// catches up a few a frame). An automated host stops the session once it has sent and confirmed its frames.
void NetStep( int close )
{
	if ( app.listener != nullptr )
	{
		lpTcp* c = lpTcp_Accept( app.listener );
		if ( c != nullptr )
		{
			app.links.push_back( c );
			lpLockstep_AddPeer( app.lockstep, lpTcp_Transport( c ) );
		}
	}
	bool host = app.listener != nullptr;
	if ( host && app.opt.frames > 0 && lpLockstep_GetClosed( app.lockstep ) >= app.opt.frames )
	{
		close = 0;
	}
	uint64_t t0 = lpGetTicks();
	int steps = 0;
	for ( int i = 0; i < ( host ? close : 8 ) || i == 0; ++i )
	{
		if ( lpLockstep_Pump( app.lockstep, NetBefore, nullptr, host ? ( close > 0 ? 1 : 0 ) : 0, 1 ) == 0 )
		{
			break;
		}
		PostStep();
		app.fireCooldown -= 1;
		steps += 1;
	}
	app.stepMs = steps > 0 ? lpGetMilliseconds( t0 ) / (float)steps : app.stepMs;
	app.peer = lpLockstep_GetPeer( app.lockstep );
	if ( host && app.opt.frames > 0 && lpLockstep_GetState( app.lockstep ) == lp_lockstepRunning && app.tick >= app.opt.frames &&
		 lpLockstep_GetConfirmed( app.lockstep ) >= app.opt.frames - 1 )
	{
		lpLockstep_Stop( app.lockstep, "done" );
	}
}

void PostStep()
{
	app.last = lpWorld_GetStats( app.world );
	if ( app.recordFile != nullptr )
	{
		// What the step applied, as it applied it (the scene's own commands are made again on replay)
		int count = 0;
		const lpCommand* applied = lpWorld_GetAppliedCommands( app.world, &count );
		for ( int i = 0; i < count; ++i )
		{
			lpScriptWrite( app.recordFile, applied + i );
		}
	}

	int count = 0;
	const lpParticle* emitted = lpWorld_GetParticles( app.world, &count );
	for ( int i = 0; i < count && app.particles.size() < 16000; ++i )
	{
		const lpParticle& src = emitted[i];
		Particle p = {};
		p.position = { src.position[0], src.position[1], src.position[2] };
		p.velocity = { src.velocity[0], src.velocity[1] + 1.5f, src.velocity[2] };
		p.size = src.size < 0.03f ? 0.03f : ( src.size > 0.2f ? 0.2f : src.size );
		p.life = 1.5f + 0.5f * (float)( i % 5 ) / 5.0f;
		p.spin = (float)i;
		p.color = src.color;
		p.kind = src.kind;
		// A spin axis and a shape seed per particle, hashed from its tick and index
		uint32_t h = (uint32_t)( app.tick * 2654435761u ) ^ (uint32_t)( ( i + 1 ) * 2246822519u );
		V3 axis = { (float)( h & 255 ) - 127.5f, (float)( ( h >> 8 ) & 255 ) - 127.5f, (float)( ( h >> 16 ) & 255 ) - 127.5f };
		p.axis = Normalize( axis );
		p.seed = (float)( h >> 24 ) / 255.0f;
		app.particles.push_back( p );
	}

	if ( app.hashFile != nullptr )
	{
		app.lastHash = lpWorld_Hash( app.world );
		fprintf( app.hashFile, "%lld %016llx %016llx\n", (long long)app.tick, (unsigned long long)app.lastHash,
				 (unsigned long long)lpWorld_HashStress( app.world ) );
	}
	for ( const auto& dump : app.opt.dumps )
	{
		FILE* f = dump.first == app.tick ? fopen( dump.second.c_str(), "w" ) : nullptr;
		if ( f != nullptr )
		{
			lpDumpWorld( f, app.world );
			fclose( f );
			printf( "dump %s\n", dump.second.c_str() );
		}
	}
	app.tick += 1;
}

// Ropes, drawn as short knotted segments of the particle shape, sagging when slack. Render only.
void AppendRopes( std::vector<Particle>& out )
{
	int capacity = lpWorld_GetLinkCapacity( app.world );
	for ( int i = 0; i < capacity; ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( app.world, i );
		if ( st.alive == false || st.type != lp_linkRope )
		{
			continue;
		}
		V3 a = { (float)st.pointA.x, (float)st.pointA.y, (float)st.pointA.z };
		V3 b = { (float)st.pointB.x, (float)st.pointB.y, (float)st.pointB.z };
		V3 ab = b - a;
		float span = sqrtf( Dot( ab, ab ) );
		// A slack rope as a parabola whose arc is about the rope's length
		float sag = st.length > span ? sqrtf( 3.0f * span * ( st.length - span ) / 8.0f ) : 0.0f;
		int n = (int)ceilf( ( st.length > span ? st.length : span ) / 0.12f );
		n = n < 1 ? 1 : ( n > 64 ? 64 : n );
		const float thickness = 0.035f;
		V3 prev = a;
		for ( int k = 1; k <= n; ++k )
		{
			float t = (float)k / (float)n;
			V3 p = a + t * ab;
			p.y -= 4.0f * sag * t * ( 1.0f - t );
			V3 seg = p - prev;
			float len = sqrtf( Dot( seg, seg ) );
			V3 dir = len > 1e-6f ? ( 1.0f / len ) * seg : V3{ 0.0f, 0.0f, 1.0f };
			V3 axis = Cross( V3{ 0.0f, 0.0f, 1.0f }, dir );
			float axisLength = sqrtf( Dot( axis, axis ) );
			Particle q = {};
			q.position = 0.5f * ( prev + p );
			q.size = thickness;
			q.color = 0xFF5E8AAEu; // hemp
			q.kind = 5;
			q.axis = axisLength > 1e-6f ? ( 1.0f / axisLength ) * axis : V3{ 1.0f, 0.0f, 0.0f };
			q.spin = atan2f( axisLength, dir.z );
			q.seed = fmodf( 0.618034f * (float)( 37 * i + k ), 1.0f );
			q.stretch = 1.3f * len / thickness; // overlapping, so the lumps never open a gap
			q.life = 1.0f;
			out.push_back( q );
			prev = p;
		}
	}
}

// Overlay: every link from end to end, green when easy, red at its limit
void DrawLinks( ImDrawList* dl )
{
	ImVec2 size = ImGui::GetIO().DisplaySize;
	auto project = [&]( lpPos w, ImVec2* out ) {
		const float* m = app.viewProj.m;
		float x = (float)w.x, y = (float)w.y, z = (float)w.z;
		float cx = m[0] * x + m[4] * y + m[8] * z + m[12];
		float cy = m[1] * x + m[5] * y + m[9] * z + m[13];
		float cw = m[3] * x + m[7] * y + m[11] * z + m[15];
		if ( cw <= 0.01f )
		{
			return false;
		}
		*out = ImVec2( ( 0.5f + 0.5f * cx / cw ) * size.x, ( 0.5f - 0.5f * cy / cw ) * size.y );
		return true;
	};
	int capacity = lpWorld_GetLinkCapacity( app.world );
	for ( int i = 0; i < capacity; ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( app.world, i );
		ImVec2 a, b;
		if ( st.alive == false || project( st.pointA, &a ) == false || project( st.pointB, &b ) == false )
		{
			continue;
		}
		float u = st.utilization < 1.0f ? st.utilization : 1.0f;
		ImU32 col = IM_COL32( (int)( 255.0f * fminf( 1.0f, 2.0f * u ) ), (int)( 255.0f * fminf( 1.0f, 2.0f - 2.0f * u ) ), 40, 230 );
		dl->AddLine( a, b, col, 2.0f );
		dl->AddCircle( a, 4.0f, col, 8, 2.0f );
	}
}

void UpdateParticles( float dt )
{
	size_t k = 0;
	for ( size_t i = 0; i < app.particles.size(); ++i )
	{
		Particle p = app.particles[i];
		p.life -= dt;
		if ( p.life <= 0.0f )
		{
			continue;
		}
		p.velocity.y -= 9.8f * dt;
		if ( p.kind == lp_particleLeaf || p.kind == lp_particleDust )
		{
			// Air drag: leaves flutter down, dust hangs
			float drag = p.kind == lp_particleLeaf ? 3.0f : 1.5f;
			p.velocity = ( 1.0f - drag * dt ) * p.velocity;
		}
		p.position = p.position + dt * p.velocity;
		if ( p.position.y < 0.5f * p.size )
		{
			p.position.y = 0.5f * p.size;
			p.velocity = 0.3f * p.velocity;
			p.velocity.y = -p.velocity.y;
		}
		p.spin += 6.0f * dt;
		float shrink = p.life < 0.4f ? p.life / 0.4f : 1.0f;
		p.size *= shrink < 1.0f ? ( 0.97f ) : 1.0f;
		app.particles[k++] = p;
	}
	app.particles.resize( k );
}

void UpdateCamera( float dt )
{
	int controlledVehicle = ControlledVehicle();
	int controlledRig = ControlledRig();
	int chase = app.driving >= 0 ? app.driving : ( app.opt.follow ? controlledVehicle : -1 );
	if ( chase >= 0 )
	{
		Drive_Camera( app.world, chase, dt, &app.camPos, &app.yaw, &app.pitch );
		return; // the keys drive
	}
	// Following with no rig walked by commands, the scene's first rig (the mech on patrol)
	int followed = controlledRig >= 0 ? controlledRig : ( controlledVehicle < 0 && lpWorld_GetRigCapacity( app.world ) > 0 ? 0 : -1 );
	int walker = app.walking >= 0 ? app.walking : ( app.opt.follow ? followed : -1 );
	if ( walker >= 0 )
	{
		Walk_Camera( app.world, walker, dt, &app.camPos, &app.yaw, &app.pitch );
		return; // the keys walk
	}
	V3 f = Forward();
	V3 flat = Normalize( V3{ f.x, 0.0f, f.z } );
	V3 right = Normalize( Cross( flat, V3{ 0.0f, 1.0f, 0.0f } ) );
	float speed = app.moveSpeed * ( app.keys[SAPP_KEYCODE_LEFT_SHIFT] ? 3.0f : 1.0f );
	V3 move = { 0.0f, 0.0f, 0.0f };
	if ( app.keys[SAPP_KEYCODE_W] )
		move = move + f;
	if ( app.keys[SAPP_KEYCODE_S] )
		move = move - f;
	if ( app.keys[SAPP_KEYCODE_D] )
		move = move + right;
	if ( app.keys[SAPP_KEYCODE_A] )
		move = move - right;
	if ( app.keys[SAPP_KEYCODE_E] )
		move.y += 1.0f;
	if ( app.keys[SAPP_KEYCODE_Q] )
		move.y -= 1.0f;
	app.camPos = app.camPos + ( speed * dt ) * move;
}

void DrawUi()
{
	const RenderStats& r = Renderer_GetStats();
	ImGui::SetNextWindowPos( ImVec2( 10, 10 ), ImGuiCond_FirstUseEver );
	ImGui::SetNextWindowBgAlpha( 0.8f );
	ImGui::Begin( "lowpoly-fracture", nullptr, ImGuiWindowFlags_AlwaysAutoResize );
	ImGui::Text( "frame %.2f ms (%.0f fps)  step %.2f ms  render %.2f ms", app.frameMs, app.frameMs > 0.0f ? 1000.0f / app.frameMs : 0.0f,
				 app.stepMs, app.renderMs );
	ImGui::Text( "fracture %.2f  physics %.2f  update %.2f ms", app.last.fractureMs, app.last.physicsMs, app.last.updateMs );
	ImGui::Text( "pieces %d  bonds %d  structures %d", app.last.pieceCount, app.last.bondCount, app.last.structureBodies );
	ImGui::Text( "debris %d (awake %d)  rubble %d  particles %d", app.last.debrisBodies, app.last.awakeDebris, app.last.rubbleBodies,
				 (int)app.particles.size() );
	ImGui::Text( "stress %.2f ms  %d solves (%d waiting)  %d iterations  unsettled %d  joints broke %d", app.last.stressMs,
				 app.last.stressSolves, app.last.stressWaiting, app.last.stressIterations, app.last.unsettledStructures,
				 app.last.stressBreaks );
	ImGui::Text( "clustered pieces %d  provisional %d (audits queued %d)  meter dissolved %d", app.last.clusteredPieces,
				 app.last.provisionalStructures, app.last.auditBacklog, app.last.stressDissolved );
	float peak = 0.0f;
	for ( int i = 0; i < lpWorld_GetLinkCapacity( app.world ); ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( app.world, i );
		peak = st.alive && st.utilization > peak ? st.utilization : peak;
	}
	ImGui::Text( "links %d  broke %d  rebuilt %d  peak load %.0f%% of limit (L: show)", app.last.linkCount, app.last.linkBreaks,
				 app.last.linkRebuilds, 100.0f * peak );
	ImGui::Text( "tiers: full %d  light %d  ghosts %d  scrap %d", app.last.fullDebris, app.last.lightDebris, app.last.ghostBodies,
				 app.last.scrapBodies );
	ImGui::Text( "deferred jobs %d  demotions %d  ghost casts %d", app.last.deferredJobs, app.last.demotionsThisStep, app.last.ghostCasts );
	ImGui::Text( "triangles %d  draws %d  pages %d  upload %d KB", r.triangles, r.drawCalls, r.pages, r.uploadKB );
	ImGui::Text( "vehicles %.2f ms  wheel casts %d", app.last.vehicleMs, app.last.wheelCasts );
	int shown = app.driving >= 0 ? app.driving : ControlledVehicle();
	if ( shown >= 0 )
	{
		char line[160];
		Drive_Describe( app.world, shown, line, (int)sizeof( line ) );
		ImGui::Text( "%s%s", app.driving >= 0 ? "driving " : "", line );
	}
	if ( lpWorld_GetRigCapacity( app.world ) > 0 )
	{
		int controlled = ControlledRig();
		int walker = app.walking >= 0 ? app.walking : ( controlled >= 0 ? controlled : 0 );
		char line[256];
		Walk_Describe( app.world, walker, line, (int)sizeof( line ) );
		ImGui::Text( "rigs %.2f ms  foot casts %d", app.last.rigMs, app.last.footCasts );
		ImGui::Text( "%s%s%s", app.walking >= 0 ? "walking " : "", line, GripLimb() >= 0 ? "  gripping" : "" );
	}
	ImGui::Text( "tick %lld", (long long)app.tick );
	if ( app.lockstep != nullptr )
	{
		lpLockstepState state = lpLockstep_GetState( app.lockstep );
		bool host = app.listener != nullptr;
		int64_t ahead = lpLockstep_GetClosed( app.lockstep ) - app.tick;
		if ( state == lp_lockstepJoining )
		{
			ImGui::Text( host ? "co-op: waiting for %d player(s)" : "co-op: joining", app.opt.peers - lpLockstep_GetPeerCount( app.lockstep ) );
		}
		else if ( state == lp_lockstepRunning )
		{
			ImGui::Text( "co-op: %s%d, %d player(s), input delay %d, %lld tick(s) %s", host ? "host, peer " : "peer ", app.peer,
						 host ? lpLockstep_GetPeerCount( app.lockstep ) + 1 : 0, app.opt.inputDelay, (long long)ahead,
						 host ? "sent ahead" : "received ahead" );
		}
		else
		{
			ImGui::TextColored( ImVec4( 1.0f, 0.35f, 0.3f, 1.0f ), "co-op stopped: %s", lpLockstep_GetReport( app.lockstep ) );
		}
	}
	ImGui::Separator();

	for ( int t = 0; t < ToolCount; ++t )
	{
		char label[64];
		snprintf( label, sizeof( label ), "%d %s", t + 1, kToolNames[t] );
		if ( ImGui::RadioButton( label, app.tool == t ) )
		{
			app.tool = t;
		}
	}
	ImGui::Separator();
	for ( int sc = 0; sc < lp_sceneCount && app.lockstep == nullptr; ++sc ) // in co-op the session's scene stays
	{
		if ( sc > 0 )
		{
			ImGui::SameLine();
		}
		if ( ImGui::Button( lpSceneName( sc ) ) )
		{
			LoadScene( sc );
		}
	}
	if ( app.lockstep == nullptr )
	{
		ImGui::SliderFloat( "fragment scale", &app.opt.fragmentScale, 0.5f, 4.0f, "%.2f" );
		ImGui::SliderInt( "debris cap", &app.opt.maxDebris, 100, 5000 );
		ImGui::SliderInt( "bombard period", &app.opt.bombard, 0, 60 );
		ImGui::Text( "(fragment scale / debris cap apply on reload: R)" );
	}
	ImGui::Separator();
	ImGui::SliderFloat( "render scale", &app.rs.renderScale, 0.25f, 1.0f, "%.2f" );
	ImGui::Checkbox( "shadows", &app.rs.shadows );
	ImGui::SliderFloat( "fog", &app.rs.fogDensity, 0.0f, 0.04f, "%.3f" );
	ImGui::Checkbox( "paused (P)", &app.paused );
	ImGui::TextDisabled( "RMB look, WASD/QE move, shift fast, LMB fire, 1-7 tools" );
	ImGui::TextDisabled( "grab: hold LMB, wheel changes grab distance" );
	ImGui::TextDisabled( "R reload, B bombard, L links, F1 ui, F12 screenshot" );
	ImGui::TextDisabled( "V drive the nearest car (WASD, space handbrake) or mech (WASD, QE sideways, C crouch," );
	ImGui::TextDisabled( "  F strike at the crosshair, G grab and lift what the claw touches / let go), V again to get out" );
	ImGui::End();

	// Crosshair
	ImDrawList* dl = ImGui::GetForegroundDrawList();
	if ( app.showLinks )
	{
		DrawLinks( dl );
	}
	ImVec2 c = ImVec2( 0.5f * ImGui::GetIO().DisplaySize.x, 0.5f * ImGui::GetIO().DisplaySize.y );
	ImU32 col = app.grabPiece >= 0 ? IM_COL32( 255, 220, 80, 230 ) : IM_COL32( 255, 255, 255, 200 );
	dl->AddLine( ImVec2( c.x - 8, c.y ), ImVec2( c.x - 3, c.y ), col, 2.0f );
	dl->AddLine( ImVec2( c.x + 3, c.y ), ImVec2( c.x + 8, c.y ), col, 2.0f );
	dl->AddLine( ImVec2( c.x, c.y - 8 ), ImVec2( c.x, c.y - 3 ), col, 2.0f );
	dl->AddLine( ImVec2( c.x, c.y + 3 ), ImVec2( c.x, c.y + 8 ), col, 2.0f );
}

void Init()
{
	sg_desc desc = {};
	desc.environment = sglue_environment();
	desc.logger.func = slog_func;
	desc.buffer_pool_size = 1024;
	desc.view_pool_size = 1024;
	sg_setup( &desc );

	simgui_desc_t sd = {};
	sd.logger.func = slog_func;
	simgui_setup( &sd );

	Renderer_Init();
	app.rs.renderScale = app.opt.renderScale;
	app.showUi = !app.opt.hideUi;

	if ( !app.opt.script.empty() )
	{
		LoadScript( app.opt.script );
	}
	if ( !app.opt.record.empty() )
	{
		app.recordFile = fopen( app.opt.record.c_str(), "w" );
		if ( app.recordFile != nullptr )
		{
			fprintf( app.recordFile, "# the commands applied (scenes/script.h); scene %s\n", lpSceneName( app.opt.scene ) );
		}
	}
	if ( !app.opt.hashLog.empty() )
	{
		app.hashFile = fopen( app.opt.hashLog.c_str(), "w" );
	}
	LoadScene( app.opt.scene );

	// Co-op: host or join (the description must match the host's: scene, bombardment, settings, build)
	bool coop = app.opt.hostPort >= 0 || !app.opt.joinHost.empty();
	app.opt.inputDelay = app.opt.inputDelay >= 0 ? app.opt.inputDelay : ( coop ? 4 : 0 );
	if ( coop )
	{
		static char session[8192];
		lpSceneDescribeSession( app.world, app.opt.scene, app.opt.bombard, 1.0f / 60.0f, 4, session, (int)sizeof( session ) );
		lpLockstepDef def = { app.world, session, app.opt.inputDelay > 0 ? app.opt.inputDelay : 1, app.opt.peers, 1.0f / 60.0f, 4 };
		if ( app.opt.hostPort >= 0 )
		{
			app.listener = lpTcp_Listen( app.opt.hostPort, app.opt.hostLoopback );
			app.lockstep = app.listener != nullptr ? lpLockstep_CreateHost( &def ) : nullptr;
			printf( "co-op: hosting %s on port %d for %d player(s)\n", lpSceneName( app.opt.scene ),
					app.listener != nullptr ? lpTcp_Port( app.listener ) : app.opt.hostPort, app.opt.peers );
		}
		else
		{
			lpTcp* host = lpTcp_Connect( app.opt.joinHost.c_str(), app.opt.joinPort, 10000 );
			if ( host != nullptr )
			{
				app.links.push_back( host );
				app.lockstep = lpLockstep_CreatePeer( &def, lpTcp_Transport( host ) );
				printf( "co-op: joined %s:%d\n", app.opt.joinHost.c_str(), app.opt.joinPort );
			}
		}
		if ( app.lockstep == nullptr )
		{
			printf( "co-op: no session; playing alone\n" );
		}
	}
}

void Frame()
{
	if ( app.quitting )
	{
		return;
	}
	uint64_t frameStart = lpGetTicks();
	float dt = (float)sapp_frame_duration();
	dt = dt > 0.1f ? 0.1f : dt;
	bool automated = app.opt.frames > 0;

	UpdateCamera( automated ? 1.0f / 60.0f : dt );
	if ( app.driving >= 0 )
	{
		DriveKeys keys = { app.keys[SAPP_KEYCODE_W], app.keys[SAPP_KEYCODE_S], app.keys[SAPP_KEYCODE_A], app.keys[SAPP_KEYCODE_D],
						   app.keys[SAPP_KEYCODE_SPACE] };
		lpVehicleControl control = Drive_Control( app.world, app.driving, keys );
		if ( Drive_Same( control, app.sent ) == false )
		{
			QueueDrive( control );
		}
	}
	if ( app.walking >= 0 )
	{
		WalkKeys keys = { app.keys[SAPP_KEYCODE_W], app.keys[SAPP_KEYCODE_S], app.keys[SAPP_KEYCODE_A], app.keys[SAPP_KEYCODE_D],
						  app.keys[SAPP_KEYCODE_Q], app.keys[SAPP_KEYCODE_E], app.keys[SAPP_KEYCODE_C] };
		lpRigControl control = Walk_Control( keys );
		if ( Walk_Same( control, app.walkSent ) == false )
		{
			QueueWalk( control );
		}
	}

	if ( app.firing && app.tool == ToolRifle )
	{
		if ( app.fireCooldown <= 0 )
		{
			QueueFire();
			app.fireCooldown = 6;
		}
	}

	if ( !app.paused || app.lockstep != nullptr )
	{
		int steps;
		if ( automated )
		{
			steps = 1;
		}
		else
		{
			app.accumulator += dt;
			steps = (int)( app.accumulator * 60.0 );
			steps = steps > 3 ? 3 : steps;
			app.accumulator -= (double)steps / 60.0;
			app.accumulator = app.accumulator > 0.1 ? 0.1 : app.accumulator;
		}
		if ( app.lockstep != nullptr )
		{
			NetStep( steps );
		}
		for ( int i = 0; i < steps && app.lockstep == nullptr; ++i )
		{
			StepSimulation();
			app.fireCooldown -= 1;
		}
	}
	UpdateParticles( automated ? 1.0f / 60.0f : dt );

	uint64_t renderStart = lpGetTicks();
	Renderer_Sync( app.world );
	app.drawn = app.particles;
	AppendRopes( app.drawn );
	Renderer_SetParticles( app.drawn.data(), (int)app.drawn.size() );

	int width = sapp_width();
	int height = sapp_height();
	V3 f = Forward();
	lpMat4 view = LookAt( app.camPos, app.camPos + f, V3{ 0.0f, 1.0f, 0.0f } );
	lpMat4 proj = Perspective( 1.05f, (float)width / (float)( height > 0 ? height : 1 ), 0.1f, 400.0f );
	app.viewProj = Mul( proj, view );

	simgui_frame_desc_t fd = {};
	fd.width = width;
	fd.height = height;
	fd.delta_time = sapp_frame_duration();
	fd.dpi_scale = sapp_dpi_scale();
	simgui_new_frame( &fd );
	if ( app.showUi )
	{
		DrawUi();
	}

	Renderer_BeginFrame( view, proj, app.camPos, f, width, height, app.rs );
	simgui_render();
	Renderer_EndFrame();
	app.renderMs = lpGetMilliseconds( renderStart );

	app.frame += 1;
	bool last = automated && app.frame >= app.opt.frames && app.lockstep == nullptr;
	if ( app.lockstep != nullptr && lpLockstep_GetState( app.lockstep ) > lp_lockstepRunning )
	{
		// Co-op: the session ended (the host's "done", a desync, a refusal): an automated run ends with it
		static bool told = false;
		if ( told == false )
		{
			printf( "co-op: %s\n", lpLockstep_GetReport( app.lockstep ) );
			told = true;
		}
		last = last || automated;
	}
	last = last || ( automated && app.frame >= 20 * app.opt.frames + 600 ); // a session that never ends
	bool listed = false;
	for ( int at : app.opt.shotsAt )
	{
		listed = listed || ( automated && at == app.frame );
	}
	bool shoot = app.opt.shotsAt.empty() ? last : listed;
	if ( app.wantScreenshot || ( shoot && !app.opt.screenshot.empty() ) )
	{
		std::string path = app.opt.screenshot;
		if ( app.wantScreenshot || path.empty() )
		{
			char name[64];
			snprintf( name, sizeof( name ), "screenshot_%03d.png", app.screenshotCounter++ );
			path = name;
		}
		else if ( listed )
		{
			// shot.png: shot_0120.png at frame 120
			size_t dot = path.rfind( '.' );
			char number[16];
			snprintf( number, sizeof( number ), "_%04d", app.frame );
			path.insert( dot == std::string::npos ? path.size() : dot, number );
		}
		bool ok = Renderer_Screenshot( path.c_str() );
		printf( "screenshot %s: %s\n", path.c_str(), ok ? "ok" : "FAILED" );
		app.wantScreenshot = false;
	}
	app.frameMs = lpGetMilliseconds( frameStart );
	app.sumFrameMs += app.frameMs;
	app.sumStepMs += app.stepMs;
	app.sumRenderMs += app.renderMs;
	app.maxFrameMs = app.frameMs > app.maxFrameMs ? app.frameMs : app.maxFrameMs;

	if ( last )
	{
		lpStats st = lpWorld_GetStats( app.world );
		const RenderStats& rs = Renderer_GetStats();
		double n = (double)app.frame;
		printf( "done: %d frames, tick %lld, pieces %d, debris %d, rubble %d, hash %016llx\n", app.frame, (long long)app.tick,
				st.pieceCount, st.debrisBodies, st.rubbleBodies, (unsigned long long)lpWorld_Hash( app.world ) );
		printf( "timing: frame avg %.2f ms (max %.2f), step avg %.2f ms, render+sync avg %.2f ms; last frame %d triangles, %d draws\n",
				app.sumFrameMs / n, app.maxFrameMs, app.sumStepMs / n, app.sumRenderMs / n, rs.triangles, rs.drawCalls );
		app.quitting = true;
		sapp_request_quit();
	}
}

void Event_( const sapp_event* ev )
{
	if ( simgui_handle_event( ev ) && ( ev->type == SAPP_EVENTTYPE_MOUSE_DOWN || ev->type == SAPP_EVENTTYPE_MOUSE_SCROLL ||
										ev->type == SAPP_EVENTTYPE_KEY_DOWN || ev->type == SAPP_EVENTTYPE_CHAR ) )
	{
		if ( ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard )
		{
			return;
		}
	}

	switch ( ev->type )
	{
		case SAPP_EVENTTYPE_KEY_DOWN:
			if ( ev->key_code >= 0 && ev->key_code < 512 )
			{
				app.keys[ev->key_code] = true;
			}
			if ( ev->key_code >= SAPP_KEYCODE_1 && ev->key_code < SAPP_KEYCODE_1 + ToolCount )
			{
				app.tool = ev->key_code - SAPP_KEYCODE_1;
			}
			if ( ev->key_code == SAPP_KEYCODE_R && app.lockstep == nullptr ) // the session's world and settings stay in co-op
			{
				LoadScene( app.opt.scene );
			}
			if ( ev->key_code == SAPP_KEYCODE_P && app.lockstep == nullptr )
			{
				app.paused = !app.paused;
			}
			if ( ev->key_code == SAPP_KEYCODE_B && app.lockstep == nullptr )
			{
				app.opt.bombard = app.opt.bombard > 0 ? 0 : 20;
			}
			if ( ev->key_code == SAPP_KEYCODE_F1 )
			{
				app.showUi = !app.showUi;
			}
			if ( ev->key_code == SAPP_KEYCODE_L )
			{
				app.showLinks = !app.showLinks;
			}
			if ( ev->key_code == SAPP_KEYCODE_V )
			{
				ToggleDriving();
			}
			if ( ev->key_code == SAPP_KEYCODE_F && ev->key_repeat == false )
			{
				StrikeKey( true );
			}
			if ( ev->key_code == SAPP_KEYCODE_G && ev->key_repeat == false )
			{
				ClawKey();
			}
			if ( ev->key_code == SAPP_KEYCODE_F12 )
			{
				app.wantScreenshot = true;
			}
			if ( ev->key_code == SAPP_KEYCODE_ESCAPE )
			{
				sapp_request_quit();
			}
			break;
		case SAPP_EVENTTYPE_KEY_UP:
			if ( ev->key_code >= 0 && ev->key_code < 512 )
			{
				app.keys[ev->key_code] = false;
			}
			if ( ev->key_code == SAPP_KEYCODE_F )
			{
				StrikeKey( false );
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_DOWN:
			if ( ev->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
			{
				app.looking = true;
				sapp_lock_mouse( true );
			}
			if ( ev->mouse_button == SAPP_MOUSEBUTTON_LEFT )
			{
				app.firing = true;
				if ( app.tool == ToolPull )
				{
					BeginGrab();
				}
				else if ( app.tool != ToolRifle )
				{
					QueueFire();
				}
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_UP:
			if ( ev->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
			{
				app.looking = false;
				sapp_lock_mouse( false );
			}
			if ( ev->mouse_button == SAPP_MOUSEBUTTON_LEFT )
			{
				app.firing = false;
				app.grabPiece = -1;
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_MOVE:
			if ( app.looking )
			{
				app.yaw += 0.0025f * ev->mouse_dx;
				app.pitch -= 0.0025f * ev->mouse_dy;
				app.pitch = app.pitch > 1.5f ? 1.5f : ( app.pitch < -1.5f ? -1.5f : app.pitch );
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_SCROLL:
			if ( app.grabPiece >= 0 )
			{
				app.grabDistance *= ev->scroll_y > 0.0f ? 1.1f : 1.0f / 1.1f;
			}
			else
			{
				app.moveSpeed *= ev->scroll_y > 0.0f ? 1.2f : 1.0f / 1.2f;
			}
			break;
		default:
			break;
	}
}

void Cleanup()
{
	if ( app.recordFile != nullptr )
	{
		fclose( app.recordFile );
	}
	if ( app.hashFile != nullptr )
	{
		fclose( app.hashFile );
	}
	lpScriptFree( &app.script );
	lpLockstep_Destroy( app.lockstep );
	for ( lpTcp* link : app.links )
	{
		lpTcp_Close( link );
	}
	lpTcp_Close( app.listener );
	DestroyWorld();
	Renderer_Shutdown();
	simgui_shutdown();
	sg_shutdown();
}


} // namespace

int main( int argc, char** argv )
{
	Options& o = app.opt;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		bool takes = true;
		if ( strcmp( a, "--scene" ) == 0 )
			o.scene = lpSceneFromName( v );
		else if ( strcmp( a, "--workers" ) == 0 )
			o.workers = atoi( v );
		else if ( strcmp( a, "--input-delay" ) == 0 )
			o.inputDelay = atoi( v ) < 0 ? 0 : atoi( v );
		else if ( strcmp( a, "--host" ) == 0 )
		{
			const char* colon = strrchr( v, ':' ); // 127.0.0.1:port listens on loopback only
			o.hostPort = atoi( colon != nullptr ? colon + 1 : v );
			o.hostLoopback = colon != nullptr;
		}
		else if ( strcmp( a, "--join" ) == 0 )
		{
			o.joinHost = v;
			size_t colon = o.joinHost.rfind( ':' );
			if ( colon != std::string::npos )
			{
				o.joinPort = atoi( o.joinHost.c_str() + colon + 1 );
				o.joinHost.resize( colon );
			}
		}
		else if ( strcmp( a, "--peers" ) == 0 )
			o.peers = atoi( v );
		else if ( strcmp( a, "--frames" ) == 0 )
			o.frames = atoi( v );
		else if ( strcmp( a, "--screenshot" ) == 0 )
			o.screenshot = v;
		else if ( strcmp( a, "--screenshot-at" ) == 0 )
		{
			for ( const char* c = v; *c != 0; )
			{
				o.shotsAt.push_back( atoi( c ) );
				const char* comma = strchr( c, ',' );
				c = comma != nullptr ? comma + 1 : c + strlen( c );
			}
		}
		else if ( strcmp( a, "--dump" ) == 0 )
		{
			int64_t tick = 0;
			const char* path = nullptr;
			if ( lpParseDumpArg( v, &tick, &path ) == false )
			{
				printf( "--dump wants tick:path\n" );
				return 1;
			}
			o.dumps.push_back( { tick, path } );
		}
		else if ( strcmp( a, "--script" ) == 0 )
			o.script = v;
		else if ( strcmp( a, "--record" ) == 0 )
			o.record = v;
		else if ( strcmp( a, "--hash-log" ) == 0 )
			o.hashLog = v;
		else if ( strcmp( a, "--bombard" ) == 0 )
			o.bombard = atoi( v );
		else if ( strcmp( a, "--fragment-scale" ) == 0 )
			o.fragmentScale = (float)atof( v );
		else if ( strcmp( a, "--max-debris" ) == 0 )
			o.maxDebris = atoi( v );
		else if ( strcmp( a, "--render-scale" ) == 0 )
			o.renderScale = (float)atof( v );
		else if ( strcmp( a, "--vsync" ) == 0 )
			o.vsync = atoi( v ) != 0;
		else if ( strcmp( a, "--width" ) == 0 )
			o.width = atoi( v );
		else if ( strcmp( a, "--height" ) == 0 )
			o.height = atoi( v );
		else if ( strcmp( a, "--camera" ) == 0 )
		{
			o.haveCamera = sscanf( v, "%f,%f,%f,%f,%f", &o.camera[0], &o.camera[1], &o.camera[2], &o.camera[3], &o.camera[4] ) == 5;
		}
		else if ( strcmp( a, "--hide-ui" ) == 0 )
		{
			o.hideUi = true;
			takes = false;
		}
		else if ( strcmp( a, "--follow" ) == 0 )
		{
			o.follow = true;
			takes = false;
		}
		else
		{
			printf( "usage: sandbox [--scene walls|house|town|tower|pile|lumber|ruins|yard|keep|track|mech] [--workers N] [--frames N] [--screenshot out.png]\n"
					"               [--screenshot-at f1,f2,...] [--dump tick:path.json]\n"
					"               [--script file] [--record file] [--hash-log file] [--bombard period] [--fragment-scale F]\n"
					"               [--max-debris N] [--render-scale F] [--vsync 0|1] [--camera x,y,z,yawDeg,pitchDeg] [--hide-ui] [--follow]\n"
					"               [--input-delay ticks] [--host port [--peers N] | --join host:port]\n" );
			return 1;
		}
		if ( takes )
		{
			++i;
		}
	}

	sapp_desc desc = {};
	desc.init_cb = Init;
	desc.frame_cb = Frame;
	desc.cleanup_cb = Cleanup;
	desc.event_cb = Event_;
	desc.width = o.width;
	desc.height = o.height;
	desc.window_title = "lowpoly-fracture sandbox";
	desc.swap_interval = o.vsync ? 1 : 0;
	desc.logger.func = slog_func;
	sapp_run( &desc );
	return 0;
}
