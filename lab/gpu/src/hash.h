// hash.h: the lab's one CPU hash, 64-bit FNV-1a (E11's harness and rows, the toy's driver, battery, corpus and CPU
// stages): over bytes, or over 32-bit words (the toy's buffer hashes: one step per word). Header only.
#ifndef LAB_HASH_H
#define LAB_HASH_H

#include <stddef.h>
#include <stdint.h>

#define LAB_FNV0 1469598103934665603ULL
#define LAB_FNV_PRIME 1099511628211ULL

// FNV-1a over n bytes from h (start from LAB_FNV0)
static inline uint64_t lab_fnv( uint64_t h, const void* p, size_t n )
{
	const uint8_t* b = (const uint8_t*)p;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= b[i];
		h *= LAB_FNV_PRIME;
	}
	return h;
}

// FNV-1a with one step per 32-bit word: n words from h
static inline uint64_t lab_fnv_words( uint64_t h, const uint32_t* w, size_t n )
{
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= w[i];
		h *= LAB_FNV_PRIME;
	}
	return h;
}

#endif
