// SPDX-License-Identifier: MIT
// The control port's connections and replies (control.h); the requests themselves are main.cpp's.

#include "control.h"

#include "net.h"

#include <math.h>
#include <stdio.h>

namespace
{

struct Connection
{
	lpTcp* tcp;
	lpTransport transport;
	int waiting; // the id of its request not yet answered (0: none)
	std::vector<std::string> early; // requests sent before the last was answered, in order
};

lpTcp* g_listener = nullptr;
std::vector<Connection> g_connections;
int g_nextId = 1;

Connection* Find( int id )
{
	for ( Connection& c : g_connections )
	{
		if ( c.waiting == id )
		{
			return &c;
		}
	}
	return nullptr;
}

} // namespace

bool Control_Start( int port )
{
	g_listener = lpTcp_Listen( port, true );
	if ( g_listener == nullptr )
	{
		return false;
	}
	printf( "control 127.0.0.1:%d\n", lpTcp_Port( g_listener ) );
	return true;
}

std::vector<ControlRequest> Control_Poll()
{
	std::vector<ControlRequest> requests;
	if ( g_listener == nullptr )
	{
		return requests;
	}
	for ( lpTcp* tcp = lpTcp_Accept( g_listener ); tcp != nullptr; tcp = lpTcp_Accept( g_listener ) )
	{
		g_connections.push_back( Connection{ tcp, lpTcp_Transport( tcp ), 0, {} } );
	}
	for ( size_t i = 0; i < g_connections.size(); )
	{
		Connection& c = g_connections[i];
		const char* line = c.waiting == 0 && c.early.empty() ? c.transport.receive( c.transport.context ) : nullptr;
		if ( c.waiting == 0 && ( line != nullptr || c.early.empty() == false ) )
		{
			c.waiting = g_nextId++;
			requests.push_back( ControlRequest{ c.waiting, line != nullptr ? std::string( line ) : c.early.front() } );
			if ( line == nullptr )
			{
				c.early.erase( c.early.begin() );
			}
		}
		if ( c.waiting == 0 && line == nullptr && c.transport.closed( c.transport.context ) )
		{
			lpTcp_Close( c.tcp );
			g_connections.erase( g_connections.begin() + (long long)i );
			continue;
		}
		++i;
	}
	return requests;
}

void Control_Reply( int id, const std::string& json )
{
	Connection* c = Find( id );
	if ( c == nullptr )
	{
		return;
	}
	c->transport.send( c->transport.context, json.c_str() );
	c->transport.flush( c->transport.context );
	c->waiting = 0;
}

bool Control_Open( int id )
{
	Connection* c = Find( id );
	if ( c == nullptr )
	{
		return false;
	}
	// Reading notices a closed connection; a request sent early waits its turn
	for ( const char* line = c->transport.receive( c->transport.context ); line != nullptr; line = c->transport.receive( c->transport.context ) )
	{
		c->early.push_back( line );
	}
	return c->transport.closed( c->transport.context ) == false;
}

void Control_Stop()
{
	for ( Connection& c : g_connections )
	{
		lpTcp_Close( c.tcp );
	}
	g_connections.clear();
	lpTcp_Close( g_listener );
	g_listener = nullptr;
}

// ---- JSON ----

void Json::Key( const char* key )
{
	text += text.size() > 1 ? ",\"" : "\"";
	text += key;
	text += "\":";
}

Json& Json::Int( const char* key, long long value )
{
	Key( key );
	text += std::to_string( value );
	return *this;
}

Json& Json::Num( const char* key, double value )
{
	Key( key );
	text += JsonNumber( value );
	return *this;
}

std::string JsonNumber( double value )
{
	char number[32];
	snprintf( number, sizeof( number ), "%.9g", value );
	return isfinite( value ) ? number : "null";
}

Json& Json::Bool( const char* key, bool value )
{
	Key( key );
	text += value ? "true" : "false";
	return *this;
}

Json& Json::Str( const char* key, const char* value )
{
	Key( key );
	text += '"';
	for ( const char* p = value; *p != 0; ++p )
	{
		unsigned char c = (unsigned char)*p;
		if ( c == '"' || c == '\\' )
		{
			text += '\\';
			text += (char)c;
		}
		else if ( c < 0x20 )
		{
			char escape[8];
			snprintf( escape, sizeof( escape ), "\\u%04x", c );
			text += escape;
		}
		else
		{
			text += (char)c;
		}
	}
	text += '"';
	return *this;
}

Json& Json::Hex( const char* key, uint64_t value )
{
	char hex[24];
	snprintf( hex, sizeof( hex ), "%016llx", (unsigned long long)value );
	return Str( key, hex );
}

Json& Json::Raw( const char* key, const std::string& json )
{
	Key( key );
	text += json;
	return *this;
}

std::string Json::Done()
{
	return text + "}";
}

std::string JsonArray( const std::vector<std::string>& items )
{
	std::string text = "[";
	for ( size_t i = 0; i < items.size(); ++i )
	{
		text += i > 0 ? "," : "";
		text += items[i];
	}
	return text + "]";
}

std::string JsonError( const char* error )
{
	return Json().Bool( "ok", false ).Str( "error", error ).Done();
}
