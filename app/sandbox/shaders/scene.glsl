// SPDX-License-Identifier: MIT
// Low-poly scene shaders. Compile with tools/build.ps1 -Shaders (sokol-shdc, hlsl5).
//
// Pieces live in shared vertex pages in their body frame. Each vertex carries its piece index; a storage buffer maps
// piece -> body slot and another holds body transforms (position + quaternion), so moving a piece to another body
// or moving a body never touches vertex data. Slot 0 is the hidden body: its w = 0 collapses the triangle.
//
// Every face has one normal and one color, so shading is flat: the classic low-poly facet look.

#pragma sokol @module scene
#pragma sokol @ctype mat4 lpMat4

// ---------------------------------------------------------------- pieces

#pragma sokol @block rotate
vec3 rotate_vector( vec4 q, vec3 v )
{
	vec3 t = 2.0 * cross( q.xyz, v );
	return v + q.w * t + cross( q.xyz, t );
}
#pragma sokol @end

#pragma sokol @block piece_pull
struct body_xf
{
	vec4 pos; // .w = 1 visible, 0 hidden
	vec4 rot;
};

layout( binding = 0 ) readonly buffer bodies
{
	body_xf body[];
};

struct piece_ref
{
	uint body;
};

layout( binding = 1 ) readonly buffer pieces
{
	piece_ref piece[];
};
#pragma sokol @end

#pragma sokol @vs vs_piece
#pragma sokol @include_block rotate
#pragma sokol @include_block piece_pull

layout( binding = 0 ) uniform vs_params
{
	mat4 view_proj;
	mat4 light_view_proj;
};

in vec3 in_pos;
in vec4 in_normal;
in vec4 in_color;
in uint in_piece;

out vec3 v_color;
out vec3 v_normal;
out vec3 v_world;
out vec4 v_light;

void main()
{
	body_xf xf = body[piece[in_piece].body];
	vec3 wp = rotate_vector( xf.rot, in_pos * xf.pos.w ) + xf.pos.xyz;
	v_normal = rotate_vector( xf.rot, in_normal.xyz );
	v_color = in_color.rgb;
	v_world = wp;
	v_light = light_view_proj * vec4( wp, 1.0 );
	gl_Position = view_proj * vec4( wp, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs_piece
layout( binding = 1 ) uniform fs_params
{
	vec4 sun_dir;	   // xyz toward the sun
	vec4 sun_color;	   // rgb, a = shadow strength
	vec4 sky_color;
	vec4 ground_color;
	vec4 fog_color;	   // rgb, a = fog density
	vec4 camera_pos;   // xyz
	vec4 shadow_params; // x = texel size, y = bias, z = enabled
};

layout( binding = 3 ) uniform texture2D shadow_map;
layout( binding = 0 ) uniform sampler shadow_smp;

in vec3 v_color;
in vec3 v_normal;
in vec3 v_world;
in vec4 v_light;

out vec4 frag_color;

float shadow_factor( vec3 n )
{
	if ( shadow_params.z < 0.5 )
	{
		return 1.0;
	}
	vec3 p = v_light.xyz / v_light.w;
	vec2 uv = p.xy * 0.5 + 0.5;
	uv.y = 1.0 - uv.y;
	if ( uv.x <= 0.0 || uv.x >= 1.0 || uv.y <= 0.0 || uv.y >= 1.0 || p.z >= 1.0 )
	{
		return 1.0;
	}
	float ndl = max( dot( n, sun_dir.xyz ), 0.0 );
	float bias = shadow_params.y * ( 1.0 + 2.0 * ( 1.0 - ndl ) );
	float depth = p.z - bias;
	float t = shadow_params.x;
	float sum = texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv, depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( -t, -t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( t, -t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( -t, t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( t, t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( 0.0, -t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( 0.0, t ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( -t, 0.0 ), depth ) );
	sum += texture( sampler2DShadow( shadow_map, shadow_smp ), vec3( uv + vec2( t, 0.0 ), depth ) );
	return sum / 9.0;
}

void main()
{
	vec3 n = normalize( v_normal );
	float ndl = max( dot( n, sun_dir.xyz ), 0.0 );
	float s = mix( 1.0, shadow_factor( n ), sun_color.a );
	vec3 hemi = mix( ground_color.rgb, sky_color.rgb, n.y * 0.5 + 0.5 );
	vec3 lit = v_color * ( hemi + sun_color.rgb * ndl * s );
	float dist = length( v_world - camera_pos.xyz );
	float fog = 1.0 - exp( -dist * fog_color.a );
	frag_color = vec4( mix( lit, fog_color.rgb, fog ), 1.0 );
}
#pragma sokol @end

#pragma sokol @program piece vs_piece fs_piece

// ---------------------------------------------------------------- shadow depth

#pragma sokol @vs vs_shadow
#pragma sokol @include_block rotate
#pragma sokol @include_block piece_pull

layout( binding = 0 ) uniform shadow_params_vs
{
	mat4 light_view_proj_s;
};

in vec3 in_pos;
in vec4 in_normal;
in vec4 in_color;
in uint in_piece;

void main()
{
	body_xf xf = body[piece[in_piece].body];
	vec3 wp = rotate_vector( xf.rot, in_pos * xf.pos.w ) + xf.pos.xyz;
	gl_Position = light_view_proj_s * vec4( wp, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs_shadow
void main()
{
}
#pragma sokol @end

#pragma sokol @program shadow vs_shadow fs_shadow

// ---------------------------------------------------------------- particles (instanced lumpy solids)

#pragma sokol @vs vs_particle
layout( binding = 0 ) uniform particle_vs
{
	mat4 view_proj_p;
};

struct particle_inst
{
	vec4 pos_size; // xyz, size
	vec4 color;	   // rgb, a: shape seed in [0, 1]
	vec4 rot;	   // quaternion
	vec4 shape;	   // xyz: per-axis scale (a splinter is long, a leaf flat), w: glint strength
};

layout( binding = 2 ) readonly buffer particles
{
	particle_inst inst[];
};

in vec3 in_corner;
in vec3 in_cnormal;

out vec3 v_pcolor;
out vec3 v_pwpos;
out vec3 v_pnormal;
out float v_glint;

vec3 qrot( vec4 q, vec3 v )
{
	vec3 t = 2.0 * cross( q.xyz, v );
	return v + q.w * t + cross( q.xyz, t );
}

// Cosmetic only (never feeds the simulation), so a sin hash is fine here
float hash1( float n )
{
	return fract( sin( n ) * 43758.5453 );
}

void main()
{
	particle_inst p = inst[gl_InstanceIndex];
	// A lumpy low-poly solid per particle, never a cube: each corner is pushed by a hash of (seed, corner). The three
	// faces meeting at a corner move it the same way, so the solid stays closed.
	float corner = dot( step( vec3( 0.0 ), in_corner ), vec3( 1.0, 2.0, 4.0 ) );
	float key = p.color.a * 97.0 + corner * 7.31;
	vec3 lump = vec3( hash1( key ), hash1( key + 17.13 ), hash1( key + 31.71 ) ) - 0.5;
	vec3 local = ( in_corner + 0.45 * lump ) * p.shape.xyz * p.pos_size.w;
	vec3 wp = qrot( p.rot, local ) + p.pos_size.xyz;
	v_pwpos = wp;
	v_pnormal = qrot( p.rot, in_cnormal / p.shape.xyz );
	v_pcolor = p.color.rgb * ( 0.88 + 0.24 * hash1( p.color.a * 53.0 + 3.7 ) );
	v_glint = p.shape.w;
	gl_Position = view_proj_p * vec4( wp, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs_particle
in vec3 v_pcolor;
in vec3 v_pwpos;
in vec3 v_pnormal;
in float v_glint;
out vec4 frag_color;
void main()
{
	// Flat facets: the triangle normal from screen-space derivatives, turned to agree with the face's rough normal
	vec3 n = normalize( cross( dFdx( v_pwpos ), dFdy( v_pwpos ) ) );
	n = dot( n, v_pnormal ) < 0.0 ? -n : n;
	float sun = max( dot( n, normalize( vec3( 0.4, 0.8, 0.3 ) ) ), 0.0 );
	float light = 0.55 + 0.25 * n.y + 0.3 * sun;
	// Glass shards flash as they tumble through the sun direction
	light += v_glint * 2.5 * pow( sun, 12.0 );
	frag_color = vec4( v_pcolor * light, 1.0 );
}
#pragma sokol @end

#pragma sokol @program particle vs_particle fs_particle

// ---------------------------------------------------------------- composite (offscreen -> swapchain)

#pragma sokol @vs vs_blit
out vec2 v_uv;
void main()
{
	vec2 p = vec2( ( gl_VertexIndex << 1 ) & 2, gl_VertexIndex & 2 );
	v_uv = vec2( p.x, 1.0 - p.y );
	gl_Position = vec4( p * 2.0 - 1.0, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs_blit
layout( binding = 4 ) uniform texture2D scene_tex;
layout( binding = 1 ) uniform sampler scene_smp;
in vec2 v_uv;
out vec4 frag_color;
void main()
{
	frag_color = texture( sampler2D( scene_tex, scene_smp ), v_uv );
}
#pragma sokol @end

#pragma sokol @program blit vs_blit fs_blit
