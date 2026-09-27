// SPDX-License-Identifier: MIT
#pragma once

#include "math3d.h"

#include "lpf/lpf.h"

#include <stdint.h>

struct RenderSettings
{
	float renderScale = 1.0f; // < 1 renders at lower resolution and upscales with nearest filtering (retro look)
	bool shadows = true;
	float fogDensity = 0.008f;
	V3 sunDir = { 0.45f, 0.8f, 0.35f };
	V3 sunColor = { 1.0f, 0.93f, 0.8f };
	V3 skyColor = { 0.55f, 0.62f, 0.72f };
	V3 groundColor = { 0.33f, 0.3f, 0.26f };
	V3 fogColor = { 0.72f, 0.8f, 0.88f };
	float shadowStrength = 0.75f;
};

struct Particle
{
	V3 position;
	V3 velocity;
	float size;
	float life;
	float spin;
	uint32_t color; // 0xAABBGGRR
	int kind;		// lpParticleKind: the shape it is drawn with
	V3 axis;		// spin axis, unit length
	float seed;		// 0..1, picks the particle's lumpy shape and colour jitter
};

struct RenderStats
{
	int triangles = 0;
	int pages = 0;
	int drawCalls = 0;
	int uploadKB = 0;
};

void Renderer_Init();
void Renderer_Shutdown();

// Forget all piece meshes (call when the world is replaced).
void Renderer_Reset();

// Mirror the world: build meshes of new or changed pieces, update the piece -> body map and body transforms.
void Renderer_Sync( const lpWorld* world );
void Renderer_SetParticles( const Particle* particles, int count );

// Shadow pass, scene pass into the offscreen target, then blit to the swapchain. Leaves the swapchain pass open so
// the caller can draw UI; the caller ends it with Renderer_EndFrame.
void Renderer_BeginFrame( const lpMat4& view, const lpMat4& proj, V3 cameraPos, V3 cameraForward, int width, int height,
						  const RenderSettings& settings );
void Renderer_EndFrame();

// Save the last rendered scene (without UI) as PNG. Call after Renderer_EndFrame.
bool Renderer_Screenshot( const char* path );

const RenderStats& Renderer_GetStats();
