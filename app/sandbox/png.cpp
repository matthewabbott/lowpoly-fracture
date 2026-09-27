// SPDX-License-Identifier: MIT

#include "png.h"

#include <stdio.h>
#include <vector>

namespace
{

uint32_t Crc( const uint8_t* data, size_t size, uint32_t crc = 0xFFFFFFFFu )
{
	static uint32_t table[256];
	static bool ready = false;
	if ( !ready )
	{
		for ( uint32_t n = 0; n < 256; ++n )
		{
			uint32_t c = n;
			for ( int k = 0; k < 8; ++k )
			{
				c = ( c & 1 ) ? 0xEDB88320u ^ ( c >> 1 ) : c >> 1;
			}
			table[n] = c;
		}
		ready = true;
	}
	for ( size_t i = 0; i < size; ++i )
	{
		crc = table[( crc ^ data[i] ) & 0xFF] ^ ( crc >> 8 );
	}
	return crc;
}

void Put32( std::vector<uint8_t>& v, uint32_t x )
{
	v.push_back( (uint8_t)( x >> 24 ) );
	v.push_back( (uint8_t)( x >> 16 ) );
	v.push_back( (uint8_t)( x >> 8 ) );
	v.push_back( (uint8_t)x );
}

void Chunk( std::vector<uint8_t>& out, const char* type, const std::vector<uint8_t>& data )
{
	Put32( out, (uint32_t)data.size() );
	size_t start = out.size();
	out.insert( out.end(), type, type + 4 );
	out.insert( out.end(), data.begin(), data.end() );
	uint32_t crc = Crc( out.data() + start, out.size() - start ) ^ 0xFFFFFFFFu;
	Put32( out, crc );
}

} // namespace

bool WritePng( const char* path, const uint8_t* rgb, int width, int height )
{
	// Raw scanlines with filter byte 0
	std::vector<uint8_t> raw;
	size_t stride = (size_t)width * 3;
	raw.reserve( ( stride + 1 ) * (size_t)height );
	for ( int y = 0; y < height; ++y )
	{
		raw.push_back( 0 );
		raw.insert( raw.end(), rgb + (size_t)y * stride, rgb + (size_t)( y + 1 ) * stride );
	}

	// zlib stream of stored blocks
	std::vector<uint8_t> z;
	z.push_back( 0x78 );
	z.push_back( 0x01 );
	size_t pos = 0;
	while ( pos < raw.size() || raw.empty() )
	{
		size_t n = raw.size() - pos;
		n = n > 65535 ? 65535 : n;
		bool last = pos + n == raw.size();
		z.push_back( last ? 1 : 0 );
		z.push_back( (uint8_t)( n & 0xFF ) );
		z.push_back( (uint8_t)( n >> 8 ) );
		z.push_back( (uint8_t)( ~n & 0xFF ) );
		z.push_back( (uint8_t)( ( ~n >> 8 ) & 0xFF ) );
		z.insert( z.end(), raw.begin() + (ptrdiff_t)pos, raw.begin() + (ptrdiff_t)( pos + n ) );
		pos += n;
		if ( raw.empty() )
		{
			break;
		}
	}
	uint32_t a = 1, b = 0;
	for ( uint8_t c : raw )
	{
		a = ( a + c ) % 65521u;
		b = ( b + a ) % 65521u;
	}
	Put32( z, ( b << 16 ) | a );

	std::vector<uint8_t> out = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n' };
	std::vector<uint8_t> ihdr;
	Put32( ihdr, (uint32_t)width );
	Put32( ihdr, (uint32_t)height );
	ihdr.push_back( 8 ); // bit depth
	ihdr.push_back( 2 ); // RGB
	ihdr.push_back( 0 );
	ihdr.push_back( 0 );
	ihdr.push_back( 0 );
	Chunk( out, "IHDR", ihdr );
	Chunk( out, "IDAT", z );
	Chunk( out, "IEND", {} );

	FILE* f = fopen( path, "wb" );
	if ( f == nullptr )
	{
		return false;
	}
	bool ok = fwrite( out.data(), 1, out.size(), f ) == out.size();
	fclose( f );
	return ok;
}
