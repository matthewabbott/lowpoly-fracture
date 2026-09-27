// SPDX-License-Identifier: MIT
// lowpoly-fracture sandbox: smash, blow up and grab a low-poly world.
//
// Interactive:  sandbox --scene town
// Automation:   sandbox --scene walls --script scripts/walls_demo.txt --frames 300 --screenshot shot.png --hash-log h.txt
//
// The simulation runs at a fixed 60 Hz. With --frames it advances exactly one tick per rendered frame, so scripted
// runs are independent of machine speed and their hash logs are comparable across runs.

#include "math3d.h"
#include "renderer.h"

#include "scenes.h"

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

enum Tool
{
	ToolRifle,
	ToolGrenade,
	ToolCannon,
	ToolHammer,
	ToolBall,
	ToolFlask,
	ToolPull,
	ToolBlow,
	ToolCount
};

const char* kToolNames[ToolCount] = { "Rifle", "Grenade", "Cannon blast", "Sledgehammer", "Cannonball", "Volatile flask",
									  "Grab / pull", "Leaf blower" };

// A sim input: applied at the start of `tick`, before the step. Recorded and replayed as text.
struct Event
{
	int64_t tick;
	int tool;
	V3 origin; // pull: target point
	V3 dir;	   // pull: grabbed point in the body frame
	int piece; // pull only
};

struct Options
{
	int scene = lp_sceneWall;
	int workers = 4;
	int frames = 0; // 0 = interactive
	std::string screenshot;
	std::string script;
	std::string record;
	std::string hashLog;
	int bombard = 0;
	float fragmentScale = 1.0f;
	int maxDebris = 400;
	float renderScale = 1.0f;
	bool vsync = true;
	bool hideUi = false;
	bool haveCamera = false;
	float camera[5] = {};
	int width = 1600;
	int height = 900;
};

struct App
{
	Options opt;
	b3WorldId physics = b3_nullWorldId;
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

	// grab tool
	int grabPiece = -1;
	uint32_t grabGeneration = 0;
	V3 grabLocal = {};
	float grabDistance = 5.0f;

	std::vector<Event> script;
	size_t nextScript = 0;
	std::vector<Event> live;
	FILE* recordFile = nullptr;
	FILE* hashFile = nullptr;

	std::vector<Particle> particles;
	RenderSettings rs;
	bool showUi = true;
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
		b3DestroyWorld( app.physics );
		app.world = nullptr;
	}
}

void LoadScene( int scene )
{
	DestroyWorld();
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = { 0.0f, -10.0f, 0.0f };
	wd.workerCount = (uint32_t)app.opt.workers;
	app.physics = b3CreateWorld( &wd );

	lpWorldDef ld = lpDefaultWorldDef();
	ld.physics = app.physics;
	ld.fragmentScale = app.opt.fragmentScale;
	ld.maxFullDebris = app.opt.maxDebris;
	ld.workerCount = app.opt.workers;
	ld.debugLog = getenv( "LPF_DEBUG" ) != nullptr;
	app.world = lpCreateWorld( &ld );
	lpBuildScene( app.world, scene );

	app.opt.scene = scene;
	app.tick = 0;
	app.accumulator = 0.0;
	app.nextScript = 0;
	app.live.clear();
	app.particles.clear();
	Renderer_Reset();
	SetSceneCamera( scene );
}

const char* ToolToken( int tool )
{
	static const char* tokens[ToolCount] = { "rifle", "grenade", "cannon", "hammer", "ball", "flask", "pull", "blow" };
	return tokens[tool];
}

void LoadScript( const std::string& path )
{
	FILE* f = fopen( path.c_str(), "r" );
	if ( f == nullptr )
	{
		fprintf( stderr, "cannot open script %s\n", path.c_str() );
		return;
	}
	char line[256];
	while ( fgets( line, sizeof( line ), f ) )
	{
		if ( line[0] == '#' || line[0] == '\n' )
		{
			continue;
		}
		Event e = {};
		long long t = 0;
		char name[32] = {};
		e.piece = -1;
		if ( sscanf( line, "%lld %31s %f %f %f %f %f %f %d", &t, name, &e.origin.x, &e.origin.y, &e.origin.z, &e.dir.x, &e.dir.y,
					 &e.dir.z, &e.piece ) >= 8 )
		{
			e.tick = t;
			e.tool = -1;
			for ( int k = 0; k < ToolCount; ++k )
			{
				if ( _stricmp( name, ToolToken( k ) ) == 0 )
				{
					e.tool = k;
				}
			}
			if ( e.tool < 0 )
			{
				fprintf( stderr, "script: unknown tool '%s' (expected rifle, grenade, cannon, hammer, ball, flask, pull, blow)\n", name );
				continue;
			}
			if ( e.tool != ToolPull )
			{
				e.dir = Normalize( e.dir );
			}
			// A held blower: the last field is how many ticks it stays on
			int repeat = e.tool == ToolBlow && e.piece > 1 ? e.piece : 1;
			for ( int k = 0; k < repeat; ++k )
			{
				Event copy = e;
				copy.tick = e.tick + k;
				copy.piece = -1;
				app.script.push_back( copy );
			}
		}
	}
	fclose( f );
	printf( "script: %d events from %s\n", (int)app.script.size(), path.c_str() );
}

// Deterministic: everything below depends only on the event and the world state at this tick.
void ApplyEvent( const Event& e )
{
	b3Vec3 origin = { e.origin.x, e.origin.y, e.origin.z };
	b3Vec3 dir = { e.dir.x, e.dir.y, e.dir.z };

	if ( e.tool == ToolPull )
	{
		// origin = target, dir = grabbed point in the body frame
		lpWorld_Pull( app.world, e.piece, dir, origin, 40.0f, 400.0f );
		return;
	}

	if ( e.tool == ToolBlow )
	{
		// 8 m cone of air: wakes and pushes rubble, scrap and ghosts, so a road can be cleared. Gentle enough that
		// blown rubble does not smash into what it lands against (damage starts at 4 m/s).
		lpWorld_Blow( app.world, origin, dir, 8.0f, 0.35f, 4.5f );
		return;
	}

	if ( e.tool == ToolFlask )
	{
		// A chunky hexagonal bottle with a neck
		b3Vec3 points[15];
		for ( int i = 0; i < 6; ++i )
		{
			b3CosSin cs = b3ComputeCosSin( 1.0471976f * (float)i );
			points[i] = { 0.09f * cs.cosine, -0.12f, 0.09f * cs.sine };
			points[6 + i] = { 0.09f * cs.cosine, 0.06f, 0.09f * cs.sine };
		}
		points[12] = { 0.04f, 0.2f, 0.0f };
		points[13] = { -0.03f, 0.2f, 0.035f };
		points[14] = { -0.03f, 0.2f, -0.035f };
		lpPartDef part = lpDefaultPartDef();
		part.points = points;
		part.pointCount = 15;
		part.material = lp_glass;
		part.color = 0x6FD68Au;
		lpObjectDef def = lpDefaultObjectDef();
		def.isStatic = false;
		def.transform.p = b3MulAdd( origin, 0.8f, dir );
		def.parts = &part;
		def.partCount = 1;
		def.linearVelocity = b3Add( b3MulSV( 16.0f, dir ), b3Vec3{ 0.0f, 2.5f, 0.0f } );
		def.angularVelocity = { 4.0f, 1.0f, 7.0f };
		def.detonator.triggerSpeed = 4.5f;
		def.detonator.radius = 1.8f;
		def.detonator.energy = 120000.0f;
		def.detonator.speed = 12.0f;
		lpCreateObject( app.world, &def );
		return;
	}

	if ( e.tool == ToolBall )
	{
		b3Vec3 points[20];
		for ( int i = 0; i < 20; ++i )
		{
			// golden-spiral points on a sphere: a chunky low-poly ball
			float y = 1.0f - 2.0f * ( (float)i + 0.5f ) / 20.0f;
			float r = sqrtf( 1.0f - y * y );
			float a = 2.39996323f * (float)i;
			b3CosSin cs = b3ComputeCosSin( a );
			points[i] = { 0.3f * r * cs.cosine, 0.3f * y, 0.3f * r * cs.sine };
		}
		lpPartDef part = lpDefaultPartDef();
		part.points = points;
		part.pointCount = 20;
		part.material = lp_metal;
		part.color = 0x3A3D42u;
		lpObjectDef def = lpDefaultObjectDef();
		def.isStatic = false;
		def.transform.p = b3MulAdd( origin, 1.0f, dir );
		def.parts = &part;
		def.partCount = 1;
		def.linearVelocity = b3MulSV( 45.0f, dir );
		lpCreateObject( app.world, &def );
		return;
	}

	float range = e.tool == ToolHammer ? 4.0f : 250.0f;
	lpRayHit hit = lpWorld_CastRay( app.world, origin, b3MulSV( range, dir ) );
	if ( hit.hit == false )
	{
		return;
	}

	lpImpactDef im = {};
	im.point = hit.point;
	im.direction = dir;
	switch ( e.tool )
	{
		case ToolRifle:
			im.radius = 0.35f;
			im.energy = 4000.0f;
			im.impulse = 20.0f;
			break;
		case ToolGrenade:
			im.radius = 1.4f;
			im.energy = 80000.0f;
			im.impulse = 12.0f;
			im.explosion = true;
			break;
		case ToolCannon:
			im.radius = 2.3f;
			im.energy = 350000.0f;
			im.impulse = 18.0f;
			im.explosion = true;
			break;
		case ToolHammer:
			im.radius = 0.6f;
			im.energy = 14000.0f;
			im.impulse = 60.0f;
			break;
		default:
			return;
	}
	lpWorld_AddImpact( app.world, &im ); // dust comes from the core, in the colour of what broke
}

void RecordAndQueue( const Event& e )
{
	app.live.push_back( e );
	if ( app.recordFile != nullptr )
	{
		fprintf( app.recordFile, "%lld %s %.6f %.6f %.6f %.6f %.6f %.6f %d\n", (long long)e.tick, ToolToken( e.tool ), e.origin.x,
				 e.origin.y, e.origin.z, e.dir.x, e.dir.y, e.dir.z, e.piece );
		fflush( app.recordFile );
	}
}

void QueueFire()
{
	V3 f = Forward();
	Event e = { app.tick, app.tool, app.camPos, f, -1 };
	RecordAndQueue( e );
}

// Start holding whatever loose piece is under the crosshair
void BeginGrab()
{
	V3 f = Forward();
	b3Vec3 origin = { app.camPos.x, app.camPos.y, app.camPos.z };
	lpRayHit hit = lpWorld_CastRay( app.world, origin, b3Vec3{ 40.0f * f.x, 40.0f * f.y, 40.0f * f.z } );
	app.grabPiece = -1;
	if ( hit.hit && hit.piece >= 0 )
	{
		app.grabPiece = hit.piece;
		app.grabGeneration = lpWorld_GetPieceInfo( app.world, hit.piece ).generation;
		b3Vec3 local = lpWorld_ToBodyFrame( app.world, hit.piece, hit.point );
		app.grabLocal = { local.x, local.y, local.z };
		V3 p = { (float)hit.point.x, (float)hit.point.y, (float)hit.point.z };
		app.grabDistance = sqrtf( Dot( p - app.camPos, p - app.camPos ) );
	}
}

// While held, one pull event per tick toward the point in front of the camera
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
	Event e = { app.tick, ToolPull, target, app.grabLocal, app.grabPiece };
	RecordAndQueue( e );
}

void StepSimulation()
{
	// Script and live events for this tick, in file order then input order
	while ( app.nextScript < app.script.size() && app.script[app.nextScript].tick <= app.tick )
	{
		ApplyEvent( app.script[app.nextScript] );
		app.nextScript += 1;
	}
	for ( const Event& e : app.live )
	{
		ApplyEvent( e );
	}
	app.live.clear();

	if ( app.opt.bombard > 0 )
	{
		lpSceneBombard( app.world, app.opt.scene, (int)app.tick, app.opt.bombard );
	}

	uint64_t t0 = b3GetTicks();
	lpWorld_Step( app.world, 1.0f / 60.0f, 4 );
	app.stepMs = b3GetMilliseconds( t0 );
	app.last = lpWorld_GetStats( app.world );

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
		fprintf( app.hashFile, "%lld %016llx\n", (long long)app.tick, (unsigned long long)app.lastHash );
	}
	app.tick += 1;
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
	ImGui::Text( "stress %.2f ms  %d iterations  unsettled %d  joints broke %d", app.last.stressMs, app.last.stressIterations,
				 app.last.unsettledStructures, app.last.stressBreaks );
	ImGui::Text( "tiers: full %d  light %d  ghosts %d  scrap %d", app.last.fullDebris, app.last.lightDebris, app.last.ghostBodies,
				 app.last.scrapBodies );
	ImGui::Text( "deferred jobs %d  demotions %d  ghost casts %d", app.last.deferredJobs, app.last.demotionsThisStep, app.last.ghostCasts );
	ImGui::Text( "triangles %d  draws %d  pages %d  upload %d KB", r.triangles, r.drawCalls, r.pages, r.uploadKB );
	ImGui::Text( "tick %lld", (long long)app.tick );
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
	for ( int sc = 0; sc < lp_sceneCount; ++sc )
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
	ImGui::SliderFloat( "fragment scale", &app.opt.fragmentScale, 0.5f, 4.0f, "%.2f" );
	ImGui::SliderInt( "debris cap", &app.opt.maxDebris, 100, 5000 );
	ImGui::SliderInt( "bombard period", &app.opt.bombard, 0, 60 );
	ImGui::Text( "(fragment scale / debris cap apply on reload: R)" );
	ImGui::Separator();
	ImGui::SliderFloat( "render scale", &app.rs.renderScale, 0.25f, 1.0f, "%.2f" );
	ImGui::Checkbox( "shadows", &app.rs.shadows );
	ImGui::SliderFloat( "fog", &app.rs.fogDensity, 0.0f, 0.04f, "%.3f" );
	ImGui::Checkbox( "paused (P)", &app.paused );
	ImGui::TextDisabled( "RMB look, WASD/QE move, shift fast, LMB fire, 1-8 tools" );
	ImGui::TextDisabled( "grab / blower: hold LMB, wheel changes grab distance" );
	ImGui::TextDisabled( "R reload, B bombard, F1 ui, F12 screenshot" );
	ImGui::End();

	// Crosshair
	ImDrawList* dl = ImGui::GetForegroundDrawList();
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
			fprintf( app.recordFile, "# tick tool origin(xyz) direction(xyz); scene %s\n", lpSceneName( app.opt.scene ) );
		}
	}
	if ( !app.opt.hashLog.empty() )
	{
		app.hashFile = fopen( app.opt.hashLog.c_str(), "w" );
	}
	LoadScene( app.opt.scene );
}

void Frame()
{
	if ( app.quitting )
	{
		return;
	}
	uint64_t frameStart = b3GetTicks();
	float dt = (float)sapp_frame_duration();
	dt = dt > 0.1f ? 0.1f : dt;
	bool automated = app.opt.frames > 0;

	UpdateCamera( automated ? 1.0f / 60.0f : dt );

	if ( app.firing && app.tool == ToolRifle )
	{
		if ( app.fireCooldown <= 0 )
		{
			QueueFire();
			app.fireCooldown = 6;
		}
	}

	if ( !app.paused )
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
		for ( int i = 0; i < steps; ++i )
		{
			if ( app.firing && app.tool == ToolPull )
			{
				QueueGrab();
			}
			if ( app.firing && app.tool == ToolBlow )
			{
				QueueFire(); // one blow per tick while held
			}
			StepSimulation();
			app.fireCooldown -= 1;
		}
	}
	UpdateParticles( automated ? 1.0f / 60.0f : dt );

	uint64_t renderStart = b3GetTicks();
	Renderer_Sync( app.world );
	Renderer_SetParticles( app.particles.data(), (int)app.particles.size() );

	int width = sapp_width();
	int height = sapp_height();
	V3 f = Forward();
	lpMat4 view = LookAt( app.camPos, app.camPos + f, V3{ 0.0f, 1.0f, 0.0f } );
	lpMat4 proj = Perspective( 1.05f, (float)width / (float)( height > 0 ? height : 1 ), 0.1f, 400.0f );

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
	app.renderMs = b3GetMilliseconds( renderStart );

	app.frame += 1;
	bool last = automated && app.frame >= app.opt.frames;
	if ( app.wantScreenshot || ( last && !app.opt.screenshot.empty() ) )
	{
		std::string path = app.opt.screenshot;
		if ( app.wantScreenshot || path.empty() )
		{
			char name[64];
			snprintf( name, sizeof( name ), "screenshot_%03d.png", app.screenshotCounter++ );
			path = name;
		}
		bool ok = Renderer_Screenshot( path.c_str() );
		printf( "screenshot %s: %s\n", path.c_str(), ok ? "ok" : "FAILED" );
		app.wantScreenshot = false;
	}
	app.frameMs = b3GetMilliseconds( frameStart );
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
			if ( ev->key_code == SAPP_KEYCODE_R )
			{
				LoadScene( app.opt.scene );
			}
			if ( ev->key_code == SAPP_KEYCODE_P )
			{
				app.paused = !app.paused;
			}
			if ( ev->key_code == SAPP_KEYCODE_B )
			{
				app.opt.bombard = app.opt.bombard > 0 ? 0 : 20;
			}
			if ( ev->key_code == SAPP_KEYCODE_F1 )
			{
				app.showUi = !app.showUi;
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
				else if ( app.tool != ToolRifle && app.tool != ToolBlow )
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
		else if ( strcmp( a, "--frames" ) == 0 )
			o.frames = atoi( v );
		else if ( strcmp( a, "--screenshot" ) == 0 )
			o.screenshot = v;
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
		else
		{
			printf( "usage: sandbox [--scene walls|house|town|tower|pile|lumber] [--workers N] [--frames N] [--screenshot out.png]\n"
					"               [--script file] [--record file] [--hash-log file] [--bombard period] [--fragment-scale F]\n"
					"               [--max-debris N] [--render-scale F] [--vsync 0|1] [--camera x,y,z,yawDeg,pitchDeg] [--hide-ui]\n" );
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
