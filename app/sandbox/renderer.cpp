// SPDX-License-Identifier: MIT

#include "renderer.h"

#include "png.h"

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"

#include "shaders/generated/scene.glsl.h"

#include <d3d11.h>

#include <stdio.h>
#include <string.h>
#include <vector>

namespace
{

// GPU vertex: the core's lpVertex plus the piece index for vertex pulling
struct RVertex
{
	float pos[3];
	int8_t normal[4];
	uint32_t color;
	uint32_t piece; // map slot = piece index + 1; 0 = hidden
};
static_assert( sizeof( RVertex ) == 24, "vertex layout" );

constexpr int kPageVertices = 1 << 17;
constexpr int kShadowSize = 2048;
constexpr int kMaxParticles = 16384;
// Vehicles' wheels are no pieces: each drawn wheel gets a mesh of its own and a body slot for its hub, both in the
// first kWheelSlots entries of the piece and body maps (pieces and bodies come after them)
constexpr int kWheelSlots = 128;

struct Page
{
	sg_buffer buffer = {};
	std::vector<RVertex> cpu;
	int used = 0;
	int live = 0;
	bool dirty = false;
};

struct PieceSlot
{
	uint32_t generation = 0;
	int page = -1;
	int offset = 0;
	int count = 0;
};

struct StorageBuffer
{
	sg_buffer buffer = {};
	sg_view view = {};
	int capacityBytes = 0;
};

struct State
{
	std::vector<Page> pages;
	std::vector<PieceSlot> slots;
	std::vector<PieceSlot> wheelSlots = std::vector<PieceSlot>( kWheelSlots );
	std::vector<int> wheelLink = std::vector<int>( kWheelSlots, -1 ); // the link each wheel slot draws
	std::vector<uint32_t> wheelGeneration = std::vector<uint32_t>( kWheelSlots, 0u );
	std::vector<uint32_t> pieceMap;			 // [piece + 1] -> body slot
	std::vector<scene_body_xf_t> bodyXf;	 // [body + 1]
	std::vector<scene_particle_inst_t> particleInst;
	std::vector<lpVertex> scratch;

	StorageBuffer pieceBuffer, bodyBuffer, particleBuffer;

	sg_pipeline piecePipe = {}, shadowPipe = {}, particlePipe = {}, blitPipe = {};
	sg_buffer cubeVertices = {}, cubeIndices = {};

	sg_image shadowImage = {};
	sg_view shadowAttachment = {}, shadowTexture = {};
	sg_sampler shadowSampler = {};

	sg_image colorImage = {}, depthImage = {};
	sg_view colorAttachment = {}, depthAttachment = {}, colorTexture = {};
	sg_sampler blitSampler = {};
	int targetWidth = 0, targetHeight = 0;

	int particleCount = 0;
	RenderStats stats;
	int uploadBytes = 0;
};

State s;

void UploadStorage( StorageBuffer& sb, const void* data, int bytes, const char* label )
{
	bytes = bytes < 16 ? 16 : bytes;
	if ( bytes > sb.capacityBytes )
	{
		if ( sb.buffer.id != SG_INVALID_ID )
		{
			sg_destroy_view( sb.view );
			sg_destroy_buffer( sb.buffer );
		}
		int capacity = sb.capacityBytes > 0 ? sb.capacityBytes : 4096;
		while ( capacity < bytes )
		{
			capacity *= 2;
		}
		sg_buffer_desc bd = {};
		bd.size = (size_t)capacity;
		bd.usage.storage_buffer = true;
		bd.usage.vertex_buffer = false;
		bd.usage.stream_update = true;
		bd.label = label;
		sb.buffer = sg_make_buffer( &bd );
		sg_view_desc vd = {};
		vd.storage_buffer.buffer = sb.buffer;
		sb.view = sg_make_view( &vd );
		sb.capacityBytes = capacity;
	}
	sg_update_buffer( sb.buffer, sg_range{ data, (size_t)bytes } );
	s.uploadBytes += bytes;
}

Page& NewPage()
{
	Page page;
	sg_buffer_desc bd = {};
	bd.size = sizeof( RVertex ) * (size_t)kPageVertices;
	bd.usage.vertex_buffer = true;
	bd.usage.dynamic_update = true;
	bd.label = "piece-page";
	page.buffer = sg_make_buffer( &bd );
	page.cpu.resize( kPageVertices );
	s.pages.push_back( std::move( page ) );
	return s.pages.back();
}

void FreeSlotOf( PieceSlot& slot )
{
	if ( slot.page < 0 )
	{
		return;
	}
	Page& page = s.pages[slot.page];
	for ( int i = 0; i < slot.count; ++i )
	{
		page.cpu[slot.offset + i].piece = 0; // hidden until the page is compacted
	}
	page.live -= slot.count;
	page.dirty = true;
	slot.page = -1;
}

void FreeSlot( int piece )
{
	FreeSlotOf( s.slots[piece] );
}

void CompactPage( int pageIndex )
{
	Page& page = s.pages[pageIndex];
	std::vector<RVertex> fresh( kPageVertices );
	int used = 0;
	for ( std::vector<PieceSlot>* slots : { &s.slots, &s.wheelSlots } )
	{
		for ( PieceSlot& slot : *slots )
		{
			if ( slot.page != pageIndex )
			{
				continue;
			}
			memcpy( fresh.data() + used, page.cpu.data() + slot.offset, sizeof( RVertex ) * (size_t)slot.count );
			slot.offset = used;
			used += slot.count;
		}
	}
	page.cpu.swap( fresh );
	page.used = used;
	page.live = used;
	page.dirty = true;
}

// Find room for `count` vertices: the last page, else the emptiest compacted page, else a new page.
int Allocate( int count, int* offset )
{
	for ( int attempt = 0; attempt < 2; ++attempt )
	{
		for ( int p = (int)s.pages.size() - 1; p >= 0; --p )
		{
			Page& page = s.pages[p];
			if ( page.used + count <= kPageVertices )
			{
				*offset = page.used;
				page.used += count;
				page.live += count;
				page.dirty = true;
				return p;
			}
		}
		// Compact pages that are mostly garbage, then try again
		bool compacted = false;
		for ( int p = 0; p < (int)s.pages.size(); ++p )
		{
			Page& page = s.pages[p];
			if ( page.used - page.live > kPageVertices / 4 )
			{
				CompactPage( p );
				compacted = true;
			}
		}
		if ( compacted == false )
		{
			break;
		}
	}
	NewPage();
	int p = (int)s.pages.size() - 1;
	*offset = 0;
	s.pages[p].used = count;
	s.pages[p].live = count;
	s.pages[p].dirty = true;
	return p;
}

void MakeTargets( int width, int height )
{
	if ( width == s.targetWidth && height == s.targetHeight )
	{
		return;
	}
	if ( s.colorImage.id != SG_INVALID_ID )
	{
		sg_destroy_view( s.colorAttachment );
		sg_destroy_view( s.colorTexture );
		sg_destroy_view( s.depthAttachment );
		sg_destroy_image( s.colorImage );
		sg_destroy_image( s.depthImage );
	}
	sg_image_desc cd = {};
	cd.width = width;
	cd.height = height;
	cd.pixel_format = SG_PIXELFORMAT_RGBA8;
	cd.usage.color_attachment = true;
	cd.label = "scene-color";
	s.colorImage = sg_make_image( &cd );

	sg_image_desc dd = {};
	dd.width = width;
	dd.height = height;
	dd.pixel_format = SG_PIXELFORMAT_DEPTH;
	dd.usage.depth_stencil_attachment = true;
	dd.label = "scene-depth";
	s.depthImage = sg_make_image( &dd );

	sg_view_desc v = {};
	v.color_attachment.image = s.colorImage;
	s.colorAttachment = sg_make_view( &v );
	v = {};
	v.texture.image = s.colorImage;
	s.colorTexture = sg_make_view( &v );
	v = {};
	v.depth_stencil_attachment.image = s.depthImage;
	s.depthAttachment = sg_make_view( &v );

	s.targetWidth = width;
	s.targetHeight = height;
}

void MakeCube()
{
	// 24 vertices: position + normal, 4 per face
	float v[24 * 6];
	int k = 0;
	const float n[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	for ( int f = 0; f < 6; ++f )
	{
		float nx = n[f][0], ny = n[f][1], nz = n[f][2];
		// two tangents
		float ux = ny != 0.0f ? 1.0f : 0.0f, uy = 0.0f, uz = ny != 0.0f ? 0.0f : ( nx != 0.0f ? 0.0f : 1.0f );
		if ( nz != 0.0f )
		{
			ux = 1.0f;
			uz = 0.0f;
		}
		if ( nx != 0.0f )
		{
			ux = 0.0f;
			uz = 1.0f;
		}
		float wx = ny * uz - nz * uy, wy = nz * ux - nx * uz, wz = nx * uy - ny * ux;
		const float c[4][2] = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
		for ( int i = 0; i < 4; ++i )
		{
			v[k++] = 0.5f * ( nx + c[i][0] * ux + c[i][1] * wx );
			v[k++] = 0.5f * ( ny + c[i][0] * uy + c[i][1] * wy );
			v[k++] = 0.5f * ( nz + c[i][0] * uz + c[i][1] * wz );
			v[k++] = nx;
			v[k++] = ny;
			v[k++] = nz;
		}
	}
	uint16_t idx[36];
	for ( int f = 0; f < 6; ++f )
	{
		uint16_t b = (uint16_t)( 4 * f );
		uint16_t q[6] = { b, (uint16_t)( b + 1 ), (uint16_t)( b + 2 ), b, (uint16_t)( b + 2 ), (uint16_t)( b + 3 ) };
		memcpy( idx + 6 * f, q, sizeof( q ) );
	}
	sg_buffer_desc bd = {};
	bd.data = SG_RANGE( v );
	bd.label = "cube-vertices";
	s.cubeVertices = sg_make_buffer( &bd );
	bd = {};
	bd.usage.index_buffer = true;
	bd.usage.vertex_buffer = false;
	bd.data = SG_RANGE( idx );
	bd.label = "cube-indices";
	s.cubeIndices = sg_make_buffer( &bd );
}

void SetPieceLayout( sg_pipeline_desc& pd, int posAttr, int normalAttr, int colorAttr, int pieceAttr )
{
	pd.layout.buffers[0].stride = sizeof( RVertex );
	pd.layout.attrs[posAttr].format = SG_VERTEXFORMAT_FLOAT3;
	pd.layout.attrs[posAttr].offset = offsetof( RVertex, pos );
	pd.layout.attrs[normalAttr].format = SG_VERTEXFORMAT_BYTE4N;
	pd.layout.attrs[normalAttr].offset = offsetof( RVertex, normal );
	pd.layout.attrs[colorAttr].format = SG_VERTEXFORMAT_UBYTE4N;
	pd.layout.attrs[colorAttr].offset = offsetof( RVertex, color );
	pd.layout.attrs[pieceAttr].format = SG_VERTEXFORMAT_UINT;
	pd.layout.attrs[pieceAttr].offset = offsetof( RVertex, piece );
}

} // namespace

void Renderer_Init()
{
	sg_backend backend = sg_query_backend();

	sg_pipeline_desc pd = {};
	pd.shader = sg_make_shader( scene_piece_shader_desc( backend ) );
	SetPieceLayout( pd, ATTR_scene_piece_in_pos, ATTR_scene_piece_in_normal, ATTR_scene_piece_in_color, ATTR_scene_piece_in_piece );
	pd.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
	pd.depth.write_enabled = true;
	pd.depth.pixel_format = SG_PIXELFORMAT_DEPTH;
	pd.colors[0].pixel_format = SG_PIXELFORMAT_RGBA8;
	pd.cull_mode = SG_CULLMODE_BACK;
	pd.face_winding = SG_FACEWINDING_CCW;
	pd.sample_count = 1;
	pd.label = "piece";
	s.piecePipe = sg_make_pipeline( &pd );

	pd = {};
	pd.shader = sg_make_shader( scene_shadow_shader_desc( backend ) );
	SetPieceLayout( pd, ATTR_scene_shadow_in_pos, ATTR_scene_shadow_in_normal, ATTR_scene_shadow_in_color, ATTR_scene_shadow_in_piece );
	pd.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
	pd.depth.write_enabled = true;
	pd.depth.pixel_format = SG_PIXELFORMAT_DEPTH;
	pd.depth.bias = 1.0f;
	pd.depth.bias_slope_scale = 1.5f;
	pd.color_count = 0;
	pd.cull_mode = SG_CULLMODE_NONE;
	pd.sample_count = 1;
	pd.label = "shadow";
	s.shadowPipe = sg_make_pipeline( &pd );

	pd = {};
	pd.shader = sg_make_shader( scene_particle_shader_desc( backend ) );
	pd.layout.buffers[0].stride = 6 * sizeof( float );
	pd.layout.attrs[ATTR_scene_particle_in_corner].format = SG_VERTEXFORMAT_FLOAT3;
	pd.layout.attrs[ATTR_scene_particle_in_cnormal].format = SG_VERTEXFORMAT_FLOAT3;
	pd.layout.attrs[ATTR_scene_particle_in_cnormal].offset = 3 * sizeof( float );
	pd.index_type = SG_INDEXTYPE_UINT16;
	pd.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
	pd.depth.write_enabled = true;
	pd.depth.pixel_format = SG_PIXELFORMAT_DEPTH;
	pd.colors[0].pixel_format = SG_PIXELFORMAT_RGBA8;
	pd.cull_mode = SG_CULLMODE_BACK;
	pd.face_winding = SG_FACEWINDING_CCW;
	pd.sample_count = 1;
	pd.label = "particle";
	s.particlePipe = sg_make_pipeline( &pd );

	pd = {};
	pd.shader = sg_make_shader( scene_blit_shader_desc( backend ) );
	pd.label = "blit";
	s.blitPipe = sg_make_pipeline( &pd );

	// Shadow map with a comparison sampler
	sg_image_desc sd = {};
	sd.width = kShadowSize;
	sd.height = kShadowSize;
	sd.pixel_format = SG_PIXELFORMAT_DEPTH;
	sd.usage.depth_stencil_attachment = true;
	sd.label = "shadow-map";
	s.shadowImage = sg_make_image( &sd );
	sg_view_desc vd = {};
	vd.depth_stencil_attachment.image = s.shadowImage;
	s.shadowAttachment = sg_make_view( &vd );
	vd = {};
	vd.texture.image = s.shadowImage;
	s.shadowTexture = sg_make_view( &vd );

	sg_sampler_desc smp = {};
	smp.min_filter = SG_FILTER_LINEAR;
	smp.mag_filter = SG_FILTER_LINEAR;
	smp.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	smp.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	smp.compare = SG_COMPAREFUNC_LESS_EQUAL;
	s.shadowSampler = sg_make_sampler( &smp );

	smp = {};
	smp.min_filter = SG_FILTER_NEAREST;
	smp.mag_filter = SG_FILTER_NEAREST;
	smp.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	smp.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	s.blitSampler = sg_make_sampler( &smp );

	MakeCube();
	s.pieceMap.assign( 1, 0u );
	s.bodyXf.assign( 1, scene_body_xf_t{} );
}

void Renderer_Shutdown()
{
	s = State();
}

void Renderer_Reset()
{
	for ( PieceSlot& slot : s.slots )
	{
		slot = PieceSlot();
	}
	for ( int k = 0; k < kWheelSlots; ++k )
	{
		s.wheelSlots[k] = PieceSlot();
		s.wheelLink[k] = -1;
	}
	for ( Page& page : s.pages )
	{
		page.used = 0;
		page.live = 0;
		page.dirty = false;
	}
	s.pieceMap.assign( 1 + kWheelSlots, 0u );
	s.bodyXf.assign( 1 + kWheelSlots, scene_body_xf_t{} );
	s.particleCount = 0;
}

void SetBodyXf( scene_body_xf_t& dst, const b3WorldTransform& xf )
{
	dst.pos[0] = (float)xf.p.x;
	dst.pos[1] = (float)xf.p.y;
	dst.pos[2] = (float)xf.p.z;
	dst.pos[3] = 1.0f;
	dst.rot[0] = xf.q.v.x;
	dst.rot[1] = xf.q.v.y;
	dst.rot[2] = xf.q.v.z;
	dst.rot[3] = xf.q.s;
}

constexpr int kWheelSides = 12;
constexpr int kWheelVertices = kWheelSides * 12;

// A 12-sided tyre along x, flat shaded: dark tread, grey hubs
void BuildWheelMesh( float radius, float width, uint32_t piece, RVertex* out )
{
	const uint32_t tread = 0xFF2A2A2Au, hub = 0xFF9A9A9Au;
	float h = 0.5f * width;
	int n = 0;
	auto put = [&]( float x, float y, float z, float nx, float ny, float nz, uint32_t color ) {
		RVertex& v = out[n++];
		v.pos[0] = x;
		v.pos[1] = y;
		v.pos[2] = z;
		v.normal[0] = (int8_t)( 127.0f * nx );
		v.normal[1] = (int8_t)( 127.0f * ny );
		v.normal[2] = (int8_t)( 127.0f * nz );
		v.normal[3] = 0;
		v.color = color;
		v.piece = piece;
	};
	for ( int k = 0; k < kWheelSides; ++k )
	{
		float a0 = 6.2831853f * (float)k / (float)kWheelSides, a1 = 6.2831853f * (float)( k + 1 ) / (float)kWheelSides;
		float am = 0.5f * ( a0 + a1 );
		float y0 = radius * cosf( a0 ), z0 = radius * sinf( a0 ), y1 = radius * cosf( a1 ), z1 = radius * sinf( a1 );
		float ny = cosf( am ), nz = sinf( am );
		put( -h, y0, z0, 0.0f, ny, nz, tread );
		put( h, y1, z1, 0.0f, ny, nz, tread );
		put( h, y0, z0, 0.0f, ny, nz, tread );
		put( -h, y0, z0, 0.0f, ny, nz, tread );
		put( -h, y1, z1, 0.0f, ny, nz, tread );
		put( h, y1, z1, 0.0f, ny, nz, tread );
		put( h, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, hub );
		put( h, y0, z0, 1.0f, 0.0f, 0.0f, hub );
		put( h, y1, z1, 1.0f, 0.0f, 0.0f, hub );
		put( -h, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, hub );
		put( -h, y1, z1, -1.0f, 0.0f, 0.0f, hub );
		put( -h, y0, z0, -1.0f, 0.0f, 0.0f, hub );
	}
}

// Wheel slots follow the live wheel links: a slot keeps its link (and its mesh) while the link lives. Returns the
// triangles drawn.
int SyncWheels( const lpWorld* world )
{
	int links = lpWorld_GetLinkCapacity( world );
	std::vector<char> drawn( (size_t)links, 0 );
	for ( int k = 0; k < kWheelSlots; ++k )
	{
		int link = s.wheelLink[k];
		if ( link < 0 )
		{
			continue;
		}
		bool live = link < links && lpWorld_GetWheelState( world, link ).alive &&
					lpWorld_GetLinkState( world, link ).generation == s.wheelGeneration[k];
		if ( live )
		{
			drawn[(size_t)link] = 1;
		}
		else
		{
			FreeSlotOf( s.wheelSlots[k] );
			s.wheelLink[k] = -1;
		}
	}
	int next = 0;
	for ( int link = 0; link < links; ++link )
	{
		lpWheelState ws = lpWorld_GetWheelState( world, link );
		if ( ws.alive == false || drawn[(size_t)link] )
		{
			continue;
		}
		while ( next < kWheelSlots && s.wheelLink[next] >= 0 )
		{
			next += 1;
		}
		if ( next == kWheelSlots )
		{
			break; // more wheels than slots: the rest go undrawn
		}
		int offset = 0;
		int page = Allocate( kWheelVertices, &offset );
		BuildWheelMesh( ws.radius, ws.width, (uint32_t)( next + 1 ), s.pages[page].cpu.data() + offset );
		s.wheelSlots[next] = PieceSlot{ 0, page, offset, kWheelVertices };
		s.wheelLink[next] = link;
		s.wheelGeneration[next] = lpWorld_GetLinkState( world, link ).generation;
	}
	int triangles = 0;
	for ( int k = 0; k < kWheelSlots; ++k )
	{
		bool live = s.wheelLink[k] >= 0;
		s.pieceMap[k + 1] = live ? (uint32_t)( k + 1 ) : 0u;
		if ( live )
		{
			SetBodyXf( s.bodyXf[k + 1], lpWorld_GetWheelState( world, s.wheelLink[k] ).hub );
			triangles += kWheelVertices / 3;
		}
	}
	return triangles;
}

void Renderer_Sync( const lpWorld* world )
{
	s.uploadBytes = 0;

	int capacity = lpWorld_GetPieceCapacity( world );
	if ( (int)s.slots.size() < capacity )
	{
		s.slots.resize( (size_t)capacity );
	}
	s.pieceMap.resize( (size_t)capacity + 1 + kWheelSlots );
	s.pieceMap[0] = 0;
	if ( s.scratch.empty() )
	{
		s.scratch.resize( (size_t)lpWorld_GetMaxPieceVertices() );
	}

	int triangles = 0;
	for ( int i = 0; i < capacity; ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( world, i );
		PieceSlot& slot = s.slots[i];
		if ( info.body < 0 )
		{
			FreeSlot( i );
			s.pieceMap[i + 1 + kWheelSlots] = 0;
			continue;
		}

		if ( slot.page < 0 || slot.generation != info.generation )
		{
			FreeSlot( i );
			int count = lpWorld_BuildPieceMesh( world, i, s.scratch.data(), (int)s.scratch.size() );
			if ( count > 0 )
			{
				int offset = 0;
				int page = Allocate( count, &offset );
				RVertex* dst = s.pages[page].cpu.data() + offset;
				for ( int k = 0; k < count; ++k )
				{
					const lpVertex& src = s.scratch[k];
					memcpy( dst[k].pos, src.position, sizeof( dst[k].pos ) );
					memcpy( dst[k].normal, src.normal, 4 );
					dst[k].color = src.color;
					dst[k].piece = (uint32_t)( i + 1 + kWheelSlots );
				}
				slot.page = page;
				slot.offset = offset;
				slot.count = count;
			}
			slot.generation = info.generation;
		}
		s.pieceMap[i + 1 + kWheelSlots] = (uint32_t)( info.body + 1 + kWheelSlots );
		triangles += slot.count / 3;
	}

	int bodyCapacity = lpWorld_GetBodyCapacity( world );
	s.bodyXf.resize( (size_t)bodyCapacity + 1 + kWheelSlots );
	s.bodyXf[0] = scene_body_xf_t{};
	for ( int b = 0; b < bodyCapacity; ++b )
	{
		b3WorldTransform xf;
		if ( lpWorld_GetBodyTransform( world, b, &xf ) )
		{
			SetBodyXf( s.bodyXf[b + 1 + kWheelSlots], xf );
		}
		else
		{
			s.bodyXf[b + 1 + kWheelSlots] = scene_body_xf_t{};
			s.bodyXf[b + 1 + kWheelSlots].rot[3] = 1.0f;
		}
	}
	triangles += SyncWheels( world );

	for ( Page& page : s.pages )
	{
		if ( page.dirty && page.used > 0 )
		{
			sg_update_buffer( page.buffer, sg_range{ page.cpu.data(), sizeof( RVertex ) * (size_t)page.used } );
			s.uploadBytes += (int)( sizeof( RVertex ) * (size_t)page.used );
		}
		page.dirty = false;
	}

	s.stats.triangles = triangles;
	s.stats.pages = (int)s.pages.size();
}

void Renderer_SetParticles( const Particle* particles, int count )
{
	count = count < kMaxParticles ? count : kMaxParticles;
	s.particleInst.resize( (size_t)count );
	for ( int i = 0; i < count; ++i )
	{
		const Particle& p = particles[i];
		scene_particle_inst_t& d = s.particleInst[i];
		d.pos_size[0] = p.position.x;
		d.pos_size[1] = p.position.y;
		d.pos_size[2] = p.position.z;
		d.pos_size[3] = p.size;
		d.color[0] = (float)( p.color & 0xFF ) / 255.0f;
		d.color[1] = (float)( ( p.color >> 8 ) & 0xFF ) / 255.0f;
		d.color[2] = (float)( ( p.color >> 16 ) & 0xFF ) / 255.0f;
		d.color[3] = p.seed;
		float half = 0.5f * p.spin;
		float sn = sinf( half ), cs = cosf( half );
		d.rot[0] = p.axis.x * sn;
		d.rot[1] = p.axis.y * sn;
		d.rot[2] = p.axis.z * sn;
		d.rot[3] = cs;

		// Shape by kind: a dust cube, a squat chip, a long splinter, a flat leaf, a thin glinting shard
		static const float kShapes[5][4] = {
			{ 1.0f, 1.0f, 1.0f, 0.0f }, { 1.0f, 0.6f, 0.8f, 0.0f }, { 0.3f, 0.3f, 2.4f, 0.0f },
			{ 1.3f, 0.1f, 0.9f, 0.0f }, { 1.1f, 0.08f, 0.8f, 1.0f },
		};
		int kind = p.kind >= 0 && p.kind < 5 ? p.kind : 0;
		d.shape[0] = kShapes[kind][0];
		d.shape[1] = kShapes[kind][1];
		d.shape[2] = p.kind == 5 ? p.stretch : kShapes[kind][2]; // a rope segment, knotted by the lumps
		d.shape[3] = kShapes[kind][3];
	}
	s.particleCount = count;
}

void Renderer_BeginFrame( const lpMat4& view, const lpMat4& proj, V3 cameraPos, V3 cameraForward, int width, int height,
						  const RenderSettings& settings )
{
	s.stats.drawCalls = 0;

	UploadStorage( s.pieceBuffer, s.pieceMap.data(), (int)( sizeof( uint32_t ) * s.pieceMap.size() ), "piece-map" );
	UploadStorage( s.bodyBuffer, s.bodyXf.data(), (int)( sizeof( scene_body_xf_t ) * s.bodyXf.size() ), "body-xf" );
	if ( s.particleCount > 0 )
	{
		UploadStorage( s.particleBuffer, s.particleInst.data(), (int)( sizeof( scene_particle_inst_t ) * (size_t)s.particleCount ),
					   "particles" );
	}
	s.stats.uploadKB = s.uploadBytes / 1024;

	lpMat4 viewProj = Mul( proj, view );

	// Sun shadow: an orthographic box around the area in front of the camera
	V3 sun = Normalize( settings.sunDir );
	V3 focus = cameraPos + 25.0f * Normalize( V3{ cameraForward.x, 0.0f, cameraForward.z } );
	focus.y = 0.0f;
	float extent = 45.0f;
	// Snap the focus to shadow texels to avoid shimmering
	float texel = 2.0f * extent / (float)kShadowSize;
	focus.x = floorf( focus.x / texel ) * texel;
	focus.z = floorf( focus.z / texel ) * texel;
	lpMat4 lightView = LookAt( focus + 120.0f * sun, focus, V3{ 0.0f, 1.0f, 0.0f } );
	lpMat4 lightProj = Ortho( -extent, extent, -extent, extent, 1.0f, 260.0f );
	lpMat4 lightViewProj = Mul( lightProj, lightView );

	if ( settings.shadows )
	{
		sg_pass pass = {};
		pass.action.depth.load_action = SG_LOADACTION_CLEAR;
		pass.action.depth.store_action = SG_STOREACTION_STORE;
		pass.action.depth.clear_value = 1.0f;
		pass.attachments.depth_stencil = s.shadowAttachment;
		sg_begin_pass( &pass );
		sg_apply_pipeline( s.shadowPipe );
		scene_shadow_params_vs_t su = {};
		su.light_view_proj_s = lightViewProj;
		sg_apply_uniforms( UB_scene_shadow_params_vs, SG_RANGE( su ) );
		for ( Page& page : s.pages )
		{
			if ( page.used == 0 )
			{
				continue;
			}
			sg_bindings b = {};
			b.vertex_buffers[0] = page.buffer;
			b.views[VIEW_scene_bodies] = s.bodyBuffer.view;
			b.views[VIEW_scene_pieces] = s.pieceBuffer.view;
			sg_apply_bindings( &b );
			sg_draw( 0, page.used, 1 );
			s.stats.drawCalls += 1;
		}
		sg_end_pass();
	}

	int rw = (int)( (float)width * settings.renderScale );
	int rh = (int)( (float)height * settings.renderScale );
	rw = rw < 16 ? 16 : rw;
	rh = rh < 16 ? 16 : rh;
	MakeTargets( rw, rh );

	sg_pass pass = {};
	pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
	pass.action.colors[0].clear_value = { settings.fogColor.x, settings.fogColor.y, settings.fogColor.z, 1.0f };
	pass.action.depth.load_action = SG_LOADACTION_CLEAR;
	pass.action.depth.clear_value = 1.0f;
	pass.attachments.colors[0] = s.colorAttachment;
	pass.attachments.depth_stencil = s.depthAttachment;
	sg_begin_pass( &pass );

	sg_apply_pipeline( s.piecePipe );
	scene_vs_params_t vu = {};
	vu.view_proj = viewProj;
	vu.light_view_proj = lightViewProj;
	sg_apply_uniforms( UB_scene_vs_params, SG_RANGE( vu ) );

	scene_fs_params_t fu = {};
	fu.sun_dir[0] = sun.x;
	fu.sun_dir[1] = sun.y;
	fu.sun_dir[2] = sun.z;
	fu.sun_color[0] = settings.sunColor.x;
	fu.sun_color[1] = settings.sunColor.y;
	fu.sun_color[2] = settings.sunColor.z;
	fu.sun_color[3] = settings.shadows ? settings.shadowStrength : 0.0f;
	fu.sky_color[0] = settings.skyColor.x;
	fu.sky_color[1] = settings.skyColor.y;
	fu.sky_color[2] = settings.skyColor.z;
	fu.ground_color[0] = settings.groundColor.x;
	fu.ground_color[1] = settings.groundColor.y;
	fu.ground_color[2] = settings.groundColor.z;
	fu.fog_color[0] = settings.fogColor.x;
	fu.fog_color[1] = settings.fogColor.y;
	fu.fog_color[2] = settings.fogColor.z;
	fu.fog_color[3] = settings.fogDensity;
	fu.camera_pos[0] = cameraPos.x;
	fu.camera_pos[1] = cameraPos.y;
	fu.camera_pos[2] = cameraPos.z;
	fu.shadow_params[0] = 1.0f / (float)kShadowSize;
	fu.shadow_params[1] = 0.0008f;
	fu.shadow_params[2] = settings.shadows ? 1.0f : 0.0f;
	sg_apply_uniforms( UB_scene_fs_params, SG_RANGE( fu ) );

	for ( Page& page : s.pages )
	{
		if ( page.used == 0 )
		{
			continue;
		}
		sg_bindings b = {};
		b.vertex_buffers[0] = page.buffer;
		b.views[VIEW_scene_bodies] = s.bodyBuffer.view;
		b.views[VIEW_scene_pieces] = s.pieceBuffer.view;
		b.views[VIEW_scene_shadow_map] = s.shadowTexture;
		b.samplers[SMP_scene_shadow_smp] = s.shadowSampler;
		sg_apply_bindings( &b );
		sg_draw( 0, page.used, 1 );
		s.stats.drawCalls += 1;
	}

	if ( s.particleCount > 0 )
	{
		sg_apply_pipeline( s.particlePipe );
		scene_particle_vs_t pu = {};
		pu.view_proj_p = viewProj;
		sg_apply_uniforms( UB_scene_particle_vs, SG_RANGE( pu ) );
		sg_bindings b = {};
		b.vertex_buffers[0] = s.cubeVertices;
		b.index_buffer = s.cubeIndices;
		b.views[VIEW_scene_particles] = s.particleBuffer.view;
		sg_apply_bindings( &b );
		sg_draw( 0, 36, s.particleCount );
		s.stats.drawCalls += 1;
	}
	sg_end_pass();

	// Swapchain: upscale the scene with nearest filtering
	sg_pass swap = {};
	swap.action.colors[0].load_action = SG_LOADACTION_DONTCARE;
	swap.swapchain = sglue_swapchain();
	sg_begin_pass( &swap );
	sg_apply_pipeline( s.blitPipe );
	sg_bindings b = {};
	b.views[VIEW_scene_scene_tex] = s.colorTexture;
	b.samplers[SMP_scene_scene_smp] = s.blitSampler;
	sg_apply_bindings( &b );
	sg_draw( 0, 3, 1 );
	s.stats.drawCalls += 1;
}

void Renderer_EndFrame()
{
	sg_end_pass();
	sg_commit();
}

bool Renderer_Screenshot( const char* path )
{
	if ( s.colorImage.id == SG_INVALID_ID )
	{
		return false;
	}
	sg_d3d11_image_info info = sg_d3d11_query_image_info( s.colorImage );
	ID3D11Texture2D* tex = (ID3D11Texture2D*)info.tex2d;
	sapp_environment env = sapp_get_environment();
	ID3D11Device* device = (ID3D11Device*)env.d3d11.device;
	ID3D11DeviceContext* context = (ID3D11DeviceContext*)env.d3d11.device_context;
	if ( tex == nullptr || device == nullptr || context == nullptr )
	{
		return false;
	}

	D3D11_TEXTURE2D_DESC desc;
	tex->GetDesc( &desc );
	desc.Usage = D3D11_USAGE_STAGING;
	desc.BindFlags = 0;
	desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	desc.MiscFlags = 0;
	ID3D11Texture2D* staging = nullptr;
	if ( FAILED( device->CreateTexture2D( &desc, nullptr, &staging ) ) )
	{
		return false;
	}
	context->CopyResource( staging, tex );
	D3D11_MAPPED_SUBRESOURCE mapped;
	bool ok = false;
	if ( SUCCEEDED( context->Map( staging, 0, D3D11_MAP_READ, 0, &mapped ) ) )
	{
		std::vector<uint8_t> rgb( (size_t)desc.Width * desc.Height * 3 );
		for ( UINT y = 0; y < desc.Height; ++y )
		{
			const uint8_t* row = (const uint8_t*)mapped.pData + (size_t)y * mapped.RowPitch;
			for ( UINT x = 0; x < desc.Width; ++x )
			{
				uint8_t* d = rgb.data() + ( (size_t)y * desc.Width + x ) * 3;
				d[0] = row[4 * x + 0];
				d[1] = row[4 * x + 1];
				d[2] = row[4 * x + 2];
			}
		}
		context->Unmap( staging, 0 );
		ok = WritePng( path, rgb.data(), (int)desc.Width, (int)desc.Height );
	}
	staging->Release();
	return ok;
}

const RenderStats& Renderer_GetStats()
{
	return s.stats;
}
