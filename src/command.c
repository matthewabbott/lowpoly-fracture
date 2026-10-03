// SPDX-License-Identifier: MIT
// Commands (lpf.h): everything from outside the simulation, queued by the tick it applies at and applied as that
// tick's step begins, in (peer, sequence) order. Templates: objects registered once and spawned by command.

#include "world.h"

static int lpCompareKey( const lpCommand* a, const lpCommand* b )
{
	if ( a->tick != b->tick )
	{
		return a->tick < b->tick ? -1 : 1;
	}
	if ( a->peer != b->peer )
	{
		return a->peer < b->peer ? -1 : 1;
	}
	return ( a->seq > b->seq ) - ( a->seq < b->seq );
}

bool lpWorld_Submit( lpWorld* w, const lpCommand* submitted )
{
	if ( submitted->tick < (int64_t)w->tick || submitted->kind >= lp_commandKindCount )
	{
		return false;
	}
	lpCommand numbered = *submitted;
	if ( numbered.peer == LP_PEER_SCENE )
	{
		numbered.seq = w->sceneSeq++; // the scene's own commands, in the order it submits them
	}
	const lpCommand* command = &numbered;
	// The queue stays sorted by (tick, peer, seq): find the first entry not before this one
	int lo = 0, hi = w->commands.count;
	while ( lo < hi )
	{
		int mid = ( lo + hi ) / 2;
		if ( lpCompareKey( w->commands.data + mid, command ) < 0 )
		{
			lo = mid + 1;
		}
		else
		{
			hi = mid;
		}
	}
	if ( lo < w->commands.count && lpCompareKey( w->commands.data + lo, command ) == 0 )
	{
		return false; // that peer already sent that sequence number for that tick
	}
	lpCommand zero = { 0 };
	lpArray_Push( w->commands, zero );
	memmove( w->commands.data + lo + 1, w->commands.data + lo, sizeof( lpCommand ) * (size_t)( w->commands.count - 1 - lo ) );
	w->commands.data[lo] = *command;
	w->commands.data[lo].result = -1;
	return true;
}

const lpCommand* lpWorld_GetAppliedCommands( const lpWorld* w, int* count )
{
	*count = w->applied.count;
	return w->applied.data;
}

int lpWorld_AddTemplate( lpWorld* w, const lpObjectDef* def )
{
	lpTemplate t = { 0 };
	t.def = *def;
	int points = 0;
	for ( int i = 0; i < def->partCount; ++i )
	{
		points += def->parts[i].points != NULL ? def->parts[i].pointCount : 0;
	}
	t.parts = lpAlloc( sizeof( lpPartDef ) * (size_t)( def->partCount > 0 ? def->partCount : 1 ) );
	t.points = lpAlloc( sizeof( lpVec3 ) * (size_t)( points > 0 ? points : 1 ) );
	int at = 0;
	for ( int i = 0; i < def->partCount; ++i )
	{
		t.parts[i] = def->parts[i];
		if ( def->parts[i].points != NULL )
		{
			memcpy( t.points + at, def->parts[i].points, sizeof( lpVec3 ) * (size_t)def->parts[i].pointCount );
			t.parts[i].points = t.points + at;
			at += def->parts[i].pointCount;
		}
	}
	t.def.parts = t.parts;
	lpArray_Push( w->templates, t );
	return w->templates.count - 1;
}

void lpFreeTemplates( lpWorld* w )
{
	for ( int i = 0; i < w->templates.count; ++i )
	{
		lpFree( w->templates.data[i].parts );
		lpFree( w->templates.data[i].points );
	}
	lpArray_Free( w->templates );
}

static bool lpFresh( uint32_t generation, uint32_t current )
{
	return generation == LP_ANY_GENERATION || generation == current;
}

static bool lpBodyNamed( const lpWorld* w, int body, uint32_t generation )
{
	return body >= 0 && body < w->bodies.count && w->bodies.data[body].alive && lpFresh( generation, w->bodies.data[body].generation );
}

static bool lpLinkNamed( const lpWorld* w, int link, uint32_t generation )
{
	return link >= 0 && link < w->links.count && w->links.data[link].alive && lpFresh( generation, w->links.data[link].generation );
}

// Who drives a vehicle or rig: a player's command takes it over; the scene's drivers keep off it until released
static bool lpTakeControl( uint8_t* controller, uint8_t peer )
{
	if ( peer == LP_PEER_SCENE )
	{
		return *controller == 0;
	}
	*controller = (uint8_t)( peer + 1 );
	return true;
}

static lpRig* lpCommandRig( lpWorld* w, int rig )
{
	return rig >= 0 && rig < w->rigs.count && w->rigs.data[rig].alive ? w->rigs.data + rig : NULL;
}

// Applies one command; false if it was dropped (a stale reference, or a scene's control of what a player drives)
static bool lpApplyCommand( lpWorld* w, lpCommand* c )
{
	switch ( c->kind )
	{
		case lp_commandImpact:
		{
			lpImpactDef def = c->impact.def;
			if ( c->impact.range > 0.0f )
			{
				float range = lpMinFloat( c->impact.range, w->def.maxRayRange ); // how far one step's ray reaches
				lpRayHit hit = lpWorld_CastRay( w, c->impact.origin, lpMulSV( range, def.direction ) );
				if ( hit.hit == false || ( c->impact.piecesOnly && hit.piece < 0 ) )
				{
					return true; // it missed
				}
				def.point = hit.point;
			}
			lpWorld_AddImpact( w, &def );
			return true;
		}
		case lp_commandPull:
		{
			int piece = c->pull.piece;
			if ( piece < 0 || piece >= w->pieces.count || w->pieces.data[piece].body < 0 ||
				 lpFresh( c->pull.generation, w->pieces.data[piece].generation ) == false )
			{
				return false;
			}
			lpWorld_Pull( w, piece, c->pull.localPoint, c->pull.target, c->pull.maxAccel, c->pull.maxMass );
			return true;
		}
		case lp_commandSpawn:
		{
			if ( c->spawn.templateIndex < 0 || c->spawn.templateIndex >= w->templates.count )
			{
				return false;
			}
			lpObjectDef def = w->templates.data[c->spawn.templateIndex].def;
			def.transform = c->spawn.transform;
			def.linearVelocity = c->spawn.linearVelocity;
			def.angularVelocity = c->spawn.angularVelocity;
			c->result = lpCreateObject( w, &def );
			return true;
		}
		case lp_commandVehicleControl:
		{
			int v = c->vehicleControl.vehicle;
			if ( v < 0 || v >= w->vehicles.count || w->vehicles.data[v].alive == false ||
				 lpTakeControl( &w->vehicles.data[v].controller, c->peer ) == false )
			{
				return false;
			}
			lpWorld_SetVehicleControl( w, v, &c->vehicleControl.control );
			return true;
		}
		case lp_commandRigControl:
		{
			lpRig* r = lpCommandRig( w, c->rigControl.rig );
			if ( r == NULL || lpTakeControl( &r->controller, c->peer ) == false )
			{
				return false;
			}
			lpWorld_SetRigControl( w, c->rigControl.rig, &c->rigControl.control );
			return true;
		}
		case lp_commandLimbTarget:
		{
			lpRig* r = lpCommandRig( w, c->limbTarget.rig );
			if ( r == NULL || lpTakeControl( &r->controller, c->peer ) == false )
			{
				return false;
			}
			lpWorld_SetLimbTarget( w, c->limbTarget.rig, c->limbTarget.limb, c->limbTarget.active, c->limbTarget.point );
			return true;
		}
		case lp_commandFootTarget:
		{
			lpRig* r = lpCommandRig( w, c->footTarget.rig );
			if ( r == NULL || lpTakeControl( &r->controller, c->peer ) == false )
			{
				return false;
			}
			lpWorld_SetFootTarget( w, c->footTarget.rig, c->footTarget.limb, &c->footTarget.target );
			return true;
		}
		case lp_commandRigPose:
		{
			lpRig* r = lpCommandRig( w, c->rigPose.rig );
			if ( r == NULL || lpTakeControl( &r->controller, c->peer ) == false )
			{
				return false;
			}
			lpWorld_SetRigPose( w, c->rigPose.rig, c->rigPose.pose, c->rigPose.linear, c->rigPose.angular );
			return true;
		}
		case lp_commandRelease:
		{
			int v = c->release.vehicle;
			lpRig* r = lpCommandRig( w, c->release.rig );
			if ( v >= 0 && v < w->vehicles.count && w->vehicles.data[v].controller == c->peer + 1 )
			{
				w->vehicles.data[v].controller = 0;
				return true;
			}
			if ( r != NULL && r->controller == c->peer + 1 )
			{
				r->controller = 0;
				return true;
			}
			return false; // not this peer's to let go
		}
		case lp_commandClaw:
		{
			lpRig* r = lpCommandRig( w, c->claw.rig );
			if ( r == NULL || c->claw.limb < 0 || c->claw.limb >= r->limbCount || lpTakeControl( &r->controller, c->peer ) == false )
			{
				return false;
			}
			c->result = lpApplyClaw( w, c->claw.rig, c->claw.limb, c->claw.mode, c->claw.maxForce, c->claw.maxTorque, c->claw.strength );
			return true;
		}
		case lp_commandLinkTarget:
		case lp_commandLinkRotation:
		case lp_commandRopeLength:
		case lp_commandDestroyLink:
		{
			if ( lpLinkNamed( w, c->link.link, c->link.generation ) == false )
			{
				return false;
			}
			if ( c->kind == lp_commandLinkTarget )
			{
				lpWorld_SetLinkTarget( w, c->link.link, c->link.value );
			}
			else if ( c->kind == lp_commandLinkRotation )
			{
				lpWorld_SetLinkTargetRotation( w, c->link.link, c->link.rotation );
			}
			else if ( c->kind == lp_commandRopeLength )
			{
				lpWorld_SetRopeLength( w, c->link.link, c->link.value );
			}
			else
			{
				lpDestroyLink( w, c->link.link );
			}
			return true;
		}
		case lp_commandCreateLink:
		{
			const lpLinkDef* def = &c->createLink.def;
			if ( ( def->bodyA >= 0 && lpBodyNamed( w, def->bodyA, c->createLink.generationA ) == false ) ||
				 ( def->bodyB >= 0 && lpBodyNamed( w, def->bodyB, c->createLink.generationB ) == false ) )
			{
				return false;
			}
			c->result = lpCreateLink( w, def );
			return true;
		}
		case lp_commandGravityScale:
		case lp_commandPromote:
		{
			if ( lpBodyNamed( w, c->body.body, c->body.generation ) == false )
			{
				return false;
			}
			if ( c->kind == lp_commandGravityScale )
			{
				lpWorld_SetGravityScale( w, c->body.body, c->body.scale );
			}
			else
			{
				lpWorld_PromoteBody( w, c->body.body );
			}
			return true;
		}
		default:
			return false;
	}
}

void lpApplyCommands( lpWorld* w )
{
	w->applied.count = 0;
	int due = 0;
	while ( due < w->commands.count && w->commands.data[due].tick == (int64_t)w->tick )
	{
		due += 1;
	}
	for ( int i = 0; i < due; ++i )
	{
		lpCommand* c = w->commands.data + i;
		c->dropped = lpApplyCommand( w, c ) == false;
		w->stats.commandsApplied += c->dropped ? 0 : 1;
		w->stats.commandsDropped += c->dropped ? 1 : 0;
		lpArray_Push( w->applied, *c );
	}
	memmove( w->commands.data, w->commands.data + due, sizeof( lpCommand ) * (size_t)( w->commands.count - due ) );
	w->commands.count -= due;
}
