// SPDX-License-Identifier: MIT
// Single translation unit for the sokol implementations (D3D11 on Windows).

#define SOKOL_IMPL
#define SOKOL_NO_ENTRY

#include "sokol_gfx.h"
#include "sokol_app.h"
#include "sokol_glue.h"
#include "sokol_log.h"

// The window moved to (x, y) on the desktop without taking focus (agent-driven tests tile their windows). Here
// because this file already sees windows.h.
void Sandbox_PlaceWindow( int x, int y )
{
#if defined( _WIN32 )
	SetWindowPos( (HWND)sapp_win32_get_hwnd(), NULL, x, y, 0, 0, SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE );
#else
	(void)x;
	(void)y;
#endif
}
