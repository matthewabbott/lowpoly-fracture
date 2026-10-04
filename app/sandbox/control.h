// SPDX-License-Identifier: MIT
// Agent control of the sandbox (--control PORT): a loopback TCP port that takes one request per line and answers
// each with one JSON line, {"ok":true,...} or {"ok":false,"error":"..."}, so an agent (tools/coop.py) can drive a
// window: press its keys, step its clock, take screenshots, read its state. A connection sends one request at a time;
// several connections may be open. Real keyboard and mouse input is ignored unless --allow-input, so nothing typed
// elsewhere can change a test.
//
//   state                      tick, role, peer, the session, the clock, what this player controls, the camera, the
//                              tool, held keys, the root hash, and every vehicle and rig
//   key NAME down|up|tap       a key, as if pressed (A-Z, 0-9, SPACE, SHIFT, F1, F12). tap: down and up at once, for
//                              keys that act when pressed (V, G, the tools' digits); hold W with down, step, then up
//   mouse down|up              the left button: fires the tool at the crosshair, or grabs
//   camera x y z [yaw pitch]   the free camera (degrees, as --camera); "camera x y z at x y z" looks at a point. The
//                              crosshair is the screen's centre, so this aims the tools and picks what V takes
//   cmd LINE                   this player's command, as a script line without its tick (scenes/script.h)
//   pause | run                hold the clock, or let it run at 60 Hz
//   step N                     N more ticks, then hold (alone, or the host: the host keeps the clock). Answered once
//                              every machine has stepped them
//   wait T                     answered once this machine has stepped to tick T
//   turn PEER N                (the host) player PEER is ready for N more ticks. Once every player is, the host
//                              steps the fewest asked for and answers them all: agents playing one player each
//   shot PATH [ui]             a PNG of the next frame: the scene, or with ui the whole window
//   dump PATH                  the world as JSON (scenes/dump.h)
//   inject                     a moving body's velocity nudged by one ulp, on this machine only: a desync to find
//   net delay MS [JITTER]      (a joiner) its link's lines held back, each way (app/net/net.h's faults)
//   net stall MS               (a joiner) nothing passes either way for a while
//   quit
//
// While the clock is held, the camera and the particles move once per stepped tick, not per frame, and the held keys
// are read once per tick: the same requests in the same order give the same session, hash for hash.
// A request still waiting (step, wait, turn, shot) fails if the session stops first, with the session's report.
#pragma once

#include <stdint.h>
#include <string>
#include <vector>

struct ControlRequest
{
	int id;
	std::string line;
};

// Listens on 127.0.0.1:port (0: any free port) and prints "control 127.0.0.1:N" for the launcher; false if it cannot
bool Control_Start( int port );
// The requests that arrived since the last call, at most one per connection
std::vector<ControlRequest> Control_Poll();
// Answers a request with one JSON line; its connection may then send the next
void Control_Reply( int id, const std::string& json );
// Whether the request's connection is still open (a waiting request whose agent left is dropped)
bool Control_Open( int id );
void Control_Stop();

// A JSON object written field by field
struct Json
{
	std::string text = "{";

	Json& Int( const char* key, long long value );
	Json& Num( const char* key, double value ); // %.9g; null when not finite
	Json& Bool( const char* key, bool value );
	Json& Str( const char* key, const char* value );
	Json& Hex( const char* key, uint64_t value ); // a hash, as 16 hex digits in a string
	Json& Raw( const char* key, const std::string& json ); // a nested object or array, already written
	std::string Done();

private:
	void Key( const char* key );
};

// A JSON array of written values
std::string JsonArray( const std::vector<std::string>& items );
// A number (%.9g), or null when not finite
std::string JsonNumber( double value );
// {"ok":false,"error":...}
std::string JsonError( const char* error );
