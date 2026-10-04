// stages.c: see stages.h. Integers only; every loop in index or key order; every sort a total order.
#include "stages.h"

#include <stdlib.h>
#include <string.h>

void stages_radix64( uint64_t* keys, uint64_t* tmp, int n, uint32_t* counts )
{
	uint64_t* src = keys;
	uint64_t* dst = tmp;
	for ( int pass = 0; pass < 4; ++pass )
	{
		int shift = 16 * pass;
		memset( counts, 0, 65536 * sizeof( uint32_t ) );
		for ( int i = 0; i < n; ++i )
		{
			counts[( src[i] >> shift ) & 0xffffu] += 1;
		}
		if ( n == 0 || counts[( src[0] >> shift ) & 0xffffu] == (uint32_t)n )
		{
			continue;
		}
		uint32_t sum = 0;
		for ( int d = 0; d < 65536; ++d )
		{
			uint32_t c = counts[d];
			counts[d] = sum;
			sum += c;
		}
		for ( int i = 0; i < n; ++i )
		{
			dst[counts[( src[i] >> shift ) & 0xffffu]++] = src[i];
		}
		uint64_t* t = src;
		src = dst;
		dst = t;
	}
	if ( src != keys )
	{
		memcpy( keys, src, (size_t)n * sizeof( uint64_t ) );
	}
}

void stages_init( Stages* s, int n, const uint8_t* isStatic )
{
	memset( s, 0, sizeof( *s ) );
	s->bodyCount = n;
	s->isStatic = isStatic;
	s->asleep = (uint8_t*)calloc( (size_t)n, 1 );
	s->sleepIsland = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	for ( int i = 0; i < n; ++i )
	{
		s->sleepIsland[i] = -1;
	}
	s->islandOf = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->awake = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->woken = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->order = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->parent = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->minTicks = (int32_t*)malloc( (size_t)n * sizeof( int32_t ) );
	s->used = (uint64_t*)malloc( (size_t)n * sizeof( uint64_t ) );
	s->counts = (uint32_t*)malloc( 65536 * sizeof( uint32_t ) );
	s->sortA = (uint64_t*)malloc( (size_t)n * sizeof( uint64_t ) );
	s->sortB = (uint64_t*)malloc( (size_t)n * sizeof( uint64_t ) );
}

void stages_free( Stages* s )
{
	free( s->prevKeys );
	free( s->asleep );
	free( s->sleepIsland );
	free( s->keys );
	free( s->pairs );
	free( s->active );
	free( s->colourList );
	free( s->islandOf );
	free( s->awake );
	free( s->woken );
	free( s->order );
	free( s->parent );
	free( s->minTicks );
	free( s->used );
	free( s->counts );
	free( s->sortA );
	free( s->sortB );
	free( s->jointKeys );
	free( s->jointActive );
	free( s->jointColour );
	free( s->jointFlags );
	free( s->jointActiveList );
	free( s->jointColourList );
	memset( s, 0, sizeof( *s ) );
}

void stages_set_joints( Stages* s, int count, const int32_t* a, const int32_t* b )
{
	s->jointCount = count;
	s->jointA = a;
	s->jointB = b;
	size_t n = (size_t)( count > 0 ? count : 1 );
	s->jointKeys = (uint64_t*)malloc( n * sizeof( uint64_t ) );
	s->jointActive = (uint8_t*)calloc( n, 1 );
	s->jointColour = (int32_t*)malloc( n * sizeof( int32_t ) );
	s->jointFlags = (int32_t*)calloc( n, sizeof( int32_t ) );
	s->jointActiveList = (int32_t*)malloc( n * sizeof( int32_t ) );
	s->jointColourList = (int32_t*)malloc( n * sizeof( int32_t ) );
	// the filter's keys: sorted (insertion: a scene has few joints), duplicates dropped
	int kc = 0;
	for ( int k = 0; k < count; ++k )
	{
		s->jointColour[k] = -1;
		uint32_t lo = (uint32_t)( a[k] < b[k] ? a[k] : b[k] ), hi = (uint32_t)( a[k] < b[k] ? b[k] : a[k] );
		uint64_t key = ( (uint64_t)lo << 32 ) | hi;
		int i = kc;
		while ( i > 0 && s->jointKeys[i - 1] > key )
		{
			s->jointKeys[i] = s->jointKeys[i - 1];
			--i;
		}
		if ( i > 0 && s->jointKeys[i - 1] == key )
		{
			for ( int m = i; m < kc; ++m ) // undo the shift: a duplicate
			{
				s->jointKeys[m] = s->jointKeys[m + 1];
			}
			continue;
		}
		s->jointKeys[i] = key;
		kc += 1;
	}
	s->jointKeyCount = kc;
}

static int jointed( const Stages* s, uint64_t key )
{
	int lo = 0, hi = s->jointKeyCount;
	while ( lo < hi )
	{
		int mid = ( lo + hi ) / 2;
		if ( s->jointKeys[mid] < key )
		{
			lo = mid + 1;
		}
		else
		{
			hi = mid;
		}
	}
	return lo < s->jointKeyCount && s->jointKeys[lo] == key;
}

static void grow_pairs( Stages* s, int need )
{
	if ( need <= s->pairCap )
	{
		return;
	}
	int cap = s->pairCap ? s->pairCap : 256;
	while ( cap < need )
	{
		cap *= 2;
	}
	s->keys = (uint64_t*)realloc( s->keys, (size_t)cap * sizeof( uint64_t ) );
	s->pairs = (Pair*)realloc( s->pairs, (size_t)cap * sizeof( Pair ) );
	s->active = (uint8_t*)realloc( s->active, (size_t)cap );
	s->colourList = (int32_t*)realloc( s->colourList, (size_t)cap * sizeof( int32_t ) );
	s->sortB = (uint64_t*)realloc( s->sortB, (size_t)( cap > s->bodyCount ? cap : s->bodyCount ) * sizeof( uint64_t ) );
	s->pairCap = cap;
}

static int is_awake( const Stages* s, int i )
{
	return !s->isStatic[i] && !s->asleep[i];
}

static int find_root( int32_t* parent, int i )
{
	while ( parent[i] != i )
	{
		parent[i] = parent[parent[i]]; // path halving
		i = parent[i];
	}
	return i;
}

// Sort-and-sweep: bodies by (min x, index), each against the following ones until their min x passes its max x;
// pairs of two static bodies are skipped. The keys come out sorted.
static void broadphase( Stages* s, const Aabb* b )
{
	int n = s->bodyCount;
	for ( int i = 0; i < n; ++i )
	{
		s->sortA[i] = ( (uint64_t)( (uint32_t)b[i].minx ^ 0x80000000u ) << 32 ) | (uint32_t)i;
	}
	stages_radix64( s->sortA, s->sortB, n, s->counts );
	for ( int i = 0; i < n; ++i )
	{
		s->order[i] = (int32_t)( s->sortA[i] & 0xffffffffu );
	}
	int count = 0;
	for ( int a = 0; a < n; ++a )
	{
		int i = s->order[a];
		for ( int c = a + 1; c < n; ++c )
		{
			int j = s->order[c];
			if ( b[j].minx > b[i].maxx )
			{
				break;
			}
			if ( b[j].miny > b[i].maxy || b[i].miny > b[j].maxy || b[j].minz > b[i].maxz || b[i].minz > b[j].maxz )
			{
				continue;
			}
			if ( s->isStatic[i] && s->isStatic[j] )
			{
				continue;
			}
			uint32_t lo = (uint32_t)( i < j ? i : j ), hi = (uint32_t)( i < j ? j : i );
			uint64_t key = ( (uint64_t)lo << 32 ) | hi;
			if ( s->jointKeyCount > 0 && jointed( s, key ) )
			{
				continue;
			}
			grow_pairs( s, count + 1 );
			s->keys[count++] = key;
		}
	}
	grow_pairs( s, count + 1 );
	stages_radix64( s->keys, s->sortB, count, s->counts );
	s->pairCount = count;
}

// The merge-join with last tick's sorted keys
static void merge_prev( Stages* s )
{
	int p = 0;
	for ( int k = 0; k < s->pairCount; ++k )
	{
		uint64_t key = s->keys[k];
		while ( p < s->prevCount && s->prevKeys[p] < key )
		{
			++p;
		}
		Pair* pr = s->pairs + k;
		pr->bodyA = (int32_t)( key >> 32 );
		pr->bodyB = (int32_t)( key & 0xffffffffu );
		pr->prevIndex = ( p < s->prevCount && s->prevKeys[p] == key ) ? p : -1;
		pr->colour = -1;
		pr->flags = 0;
		pr->pad = 0;
	}
}

static int touching( const Stages* s, const uint8_t* prevTouching, int k )
{
	return prevTouching == NULL || ( s->pairs[k].prevIndex >= 0 && prevTouching[s->pairs[k].prevIndex] );
}

static void set_active( Stages* s )
{
	s->activeCount = 0;
	for ( int k = 0; k < s->pairCount; ++k )
	{
		s->active[k] = (uint8_t)( is_awake( s, s->pairs[k].bodyA ) || is_awake( s, s->pairs[k].bodyB ) );
		s->activeCount += s->active[k];
	}
	s->jointActiveCount = 0;
	for ( int k = 0; k < s->jointCount; ++k )
	{
		s->jointActive[k] = (uint8_t)( is_awake( s, s->jointA[k] ) || is_awake( s, s->jointB[k] ) );
		if ( s->jointActive[k] )
		{
			s->jointActiveList[s->jointActiveCount++] = k;
		}
	}
}

// A touching pair (or a joint) between an awake body and a sleeping one wakes the sleeping one's island (repeated until
// nothing more wakes); the joints after the pairs
static void wake( Stages* s, const uint8_t* prevTouching )
{
	s->wokenCount = 0;
	int changed = 1;
	while ( changed )
	{
		changed = 0;
		for ( int k = 0; k < s->pairCount + s->jointCount; ++k )
		{
			int isJoint = k >= s->pairCount;
			int a = isJoint ? s->jointA[k - s->pairCount] : s->pairs[k].bodyA;
			int b = isJoint ? s->jointB[k - s->pairCount] : s->pairs[k].bodyB;
			int sleeper = -1;
			if ( is_awake( s, a ) && !s->isStatic[b] && s->asleep[b] )
			{
				sleeper = b;
			}
			else if ( is_awake( s, b ) && !s->isStatic[a] && s->asleep[a] )
			{
				sleeper = a;
			}
			if ( sleeper < 0 || ( !isJoint && !touching( s, prevTouching, k ) ) )
			{
				continue;
			}
			int island = s->sleepIsland[sleeper];
			for ( int i = 0; i < s->bodyCount; ++i )
			{
				if ( s->asleep[i] && s->sleepIsland[i] == island )
				{
					s->asleep[i] = 0;
					s->sleepIsland[i] = -1;
					s->woken[s->wokenCount++] = i;
				}
			}
			changed = 1;
		}
	}
	// index order (several islands may wake in one tick)
	for ( int i = 1; i < s->wokenCount; ++i )
	{
		for ( int j = i; j > 0 && s->woken[j] < s->woken[j - 1]; --j )
		{
			int32_t t = s->woken[j];
			s->woken[j] = s->woken[j - 1];
			s->woken[j - 1] = t;
		}
	}
}

// Union-find over the awake bodies, joined by touching pairs between two of them (key order); an island's id is its
// smallest body index. An island whose every body has been under the thresholds for sleepTicksNeeded ticks sleeps.
static void islands( Stages* s, const int32_t* sleepTicks, const uint8_t* prevTouching, int needed, int enableSleep )
{
	int n = s->bodyCount;
	for ( int i = 0; i < n; ++i )
	{
		s->parent[i] = i;
	}
	for ( int k = 0; k < s->pairCount + s->jointCount; ++k ) // the joints after the pairs (the roots are the minima either way)
	{
		int isJoint = k >= s->pairCount;
		int a = isJoint ? s->jointA[k - s->pairCount] : s->pairs[k].bodyA;
		int b = isJoint ? s->jointB[k - s->pairCount] : s->pairs[k].bodyB;
		if ( !is_awake( s, a ) || !is_awake( s, b ) || ( !isJoint && !touching( s, prevTouching, k ) ) )
		{
			continue;
		}
		int ra = find_root( s->parent, a ), rb = find_root( s->parent, b );
		if ( ra != rb )
		{
			if ( ra < rb )
				s->parent[rb] = ra;
			else
				s->parent[ra] = rb;
		}
	}
	// the counter of a body woken this tick is stale on the CPU (the GPU resets it): it counts as 0
	for ( int i = 0; i < n; ++i )
	{
		s->minTicks[i] = 0x7fffffff;
	}
	for ( int w = 0; w < s->wokenCount; ++w )
	{
		s->minTicks[s->woken[w]] = -1;
	}
	s->islandCount = 0;
	s->largestIsland = 0;
	for ( int i = 0; i < n; ++i )
	{
		s->islandOf[i] = -1;
		if ( !is_awake( s, i ) )
		{
			continue;
		}
		int r = find_root( s->parent, i );
		s->islandOf[i] = r;
		int t = s->minTicks[i] < 0 ? 0 : sleepTicks[i];
		s->minTicks[r] = t < s->minTicks[r] ? t : s->minTicks[r];
		if ( r == i )
		{
			s->islandCount += 1;
		}
	}
	// island sizes (for the report)
	for ( int i = 0; i < n; ++i )
	{
		s->used[i] = 0;
	}
	for ( int i = 0; i < n; ++i )
	{
		if ( s->islandOf[i] >= 0 )
		{
			s->used[s->islandOf[i]] += 1;
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		s->largestIsland = (int)s->used[i] > s->largestIsland ? (int)s->used[i] : s->largestIsland;
	}
	s->sleptIslands = 0;
	s->sleptBodies = 0;
	if ( !enableSleep )
	{
		return;
	}
	for ( int i = 0; i < n; ++i )
	{
		int r = s->islandOf[i];
		if ( r >= 0 && s->minTicks[r] >= needed )
		{
			s->sleptIslands += r == i;
			s->sleptBodies += 1;
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		int r = s->islandOf[i];
		if ( r >= 0 && s->minTicks[r] >= needed )
		{
			s->asleep[i] = 1;
			s->sleepIsland[i] = r;
			s->islandOf[i] = -1;
		}
	}
	s->islandCount -= s->sleptIslands;
}

// Greedy colouring of every active pair in key order (joints first, when there are any): the pairs the narrowphase runs
// on this tick, touching or not, so a contact is solved the tick it appears (prepare and the solve skip a pair whose
// manifold has no points). The lowest colour neither moving body has used; a static or sleeping body is solved as
// static (PAIR_SOLVE_*: never written) and takes no colour. Beyond STAGE_MAX_COLOURS: the overflow colour.
// The lowest colour neither moving body of (a, b) has used (STAGE_MAX_COLOURS: the overflow), taken; flags gets which
// bodies move
static int colour_one( Stages* s, int a, int b, int32_t* flags )
{
	int da = is_awake( s, a ), db = is_awake( s, b );
	*flags = ( da ? PAIR_SOLVE_A : 0 ) | ( db ? PAIR_SOLVE_B : 0 );
	uint64_t mask = ( da ? s->used[a] : 0 ) | ( db ? s->used[b] : 0 );
	int c = 0;
	while ( c < STAGE_MAX_COLOURS && ( mask & ( 1ULL << c ) ) )
	{
		++c;
	}
	if ( c < STAGE_MAX_COLOURS )
	{
		if ( da )
			s->used[a] |= 1ULL << c;
		if ( db )
			s->used[b] |= 1ULL << c;
		s->colourCount = c + 1 > s->colourCount ? c + 1 : s->colourCount;
	}
	return c;
}

static void colour( Stages* s )
{
	int n = s->bodyCount;
	for ( int i = 0; i < n; ++i )
	{
		s->used[i] = 0;
	}
	s->colourCount = 0;
	// the joints first, in index order. Box3D's b3AssignJointColor: a joint between two moving bodies takes the lowest
	// free colour; one with a body solved as static takes the highest free colour from the top down (never colour 0), so
	// it is solved after the dynamic ones ("higher priority than dyn-dyn constraints": a chain's anchor corrects last)
	int jcounts[STAGE_MAX_COLOURS + 1];
	memset( jcounts, 0, sizeof( jcounts ) );
	for ( int k = 0; k < s->jointCount; ++k )
	{
		s->jointColour[k] = -1;
		s->jointFlags[k] = 0;
		if ( !s->jointActive[k] )
		{
			continue;
		}
		int a = s->jointA[k], b = s->jointB[k];
		int c = STAGE_MAX_COLOURS;
		if ( is_awake( s, a ) && is_awake( s, b ) )
		{
			c = colour_one( s, a, b, s->jointFlags + k );
		}
		else
		{
			int m = is_awake( s, a ) ? a : b;
			s->jointFlags[k] = m == a ? PAIR_SOLVE_A : PAIR_SOLVE_B;
			for ( int i = STAGE_MAX_COLOURS - 1; i >= 1 && c == STAGE_MAX_COLOURS; --i )
			{
				if ( !( s->used[m] & ( 1ULL << i ) ) )
				{
					s->used[m] |= 1ULL << i;
					c = i;
					s->colourCount = c + 1 > s->colourCount ? c + 1 : s->colourCount;
				}
			}
		}
		s->jointColour[k] = c;
		jcounts[c] += 1;
	}
	s->jointOverflow = jcounts[STAGE_MAX_COLOURS];
	s->jointColourStart[0] = 0;
	for ( int c = 0; c <= STAGE_MAX_COLOURS; ++c )
	{
		s->jointColourStart[c + 1] = s->jointColourStart[c] + jcounts[c];
	}
	{
		int fill[STAGE_MAX_COLOURS + 1];
		memcpy( fill, s->jointColourStart, sizeof( fill ) );
		for ( int k = 0; k < s->jointCount; ++k )
		{
			if ( s->jointColour[k] >= 0 )
			{
				s->jointColourList[fill[s->jointColour[k]]++] = k;
			}
		}
	}
	// then the pairs, in key order
	int counts[STAGE_MAX_COLOURS + 1];
	memset( counts, 0, sizeof( counts ) );
	for ( int k = 0; k < s->pairCount; ++k )
	{
		Pair* p = s->pairs + k;
		p->colour = -1;
		p->flags = 0;
		if ( !s->active[k] )
		{
			continue;
		}
		int c = colour_one( s, p->bodyA, p->bodyB, &p->flags );
		p->colour = c;
		counts[c] += 1;
	}
	s->overflowCount = counts[STAGE_MAX_COLOURS];
	s->colourStart[0] = 0;
	for ( int c = 0; c <= STAGE_MAX_COLOURS; ++c )
	{
		s->colourStart[c + 1] = s->colourStart[c] + counts[c];
	}
	int fill[STAGE_MAX_COLOURS + 1];
	memcpy( fill, s->colourStart, sizeof( fill ) );
	for ( int k = 0; k < s->pairCount; ++k )
	{
		if ( s->pairs[k].colour >= 0 )
		{
			s->colourList[fill[s->pairs[k].colour]++] = k;
		}
	}
}

void stages_tick( Stages* s, const Aabb* aabbs, const int32_t* sleepTicks, const uint8_t* prevTouching, int needed, int enableSleep )
{
	broadphase( s, aabbs );
	merge_prev( s );
	wake( s, prevTouching );
	islands( s, sleepTicks, prevTouching, needed, enableSleep );
	set_active( s );
	colour( s );
	s->awakeCount = 0;
	for ( int i = 0; i < s->bodyCount; ++i )
	{
		if ( is_awake( s, i ) )
		{
			s->awake[s->awakeCount++] = i;
		}
	}
	s->prevKeys = (uint64_t*)realloc( s->prevKeys, (size_t)( s->pairCount > 0 ? s->pairCount : 1 ) * sizeof( uint64_t ) );
	memcpy( s->prevKeys, s->keys, (size_t)s->pairCount * sizeof( uint64_t ) );
	s->prevCount = s->pairCount;
}

static uint64_t fnv( uint64_t h, const void* p, size_t n )
{
	const uint8_t* b = (const uint8_t*)p;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= b[i];
		h *= 1099511628211ULL;
	}
	return h;
}

uint64_t stages_hash( const Stages* s )
{
	uint64_t h = 1469598103934665603ULL;
	h = fnv( h, &s->pairCount, sizeof( int ) );
	for ( int k = 0; k < s->pairCount; ++k )
	{
		h = fnv( h, s->keys + k, 8 );
		h = fnv( h, &s->pairs[k].prevIndex, 4 );
		h = fnv( h, &s->pairs[k].colour, 4 );
		h = fnv( h, &s->pairs[k].flags, 4 );
		h = fnv( h, s->active + k, 1 );
	}
	h = fnv( h, s->islandOf, (size_t)s->bodyCount * sizeof( int32_t ) );
	h = fnv( h, s->asleep, (size_t)s->bodyCount );
	h = fnv( h, s->sleepIsland, (size_t)s->bodyCount * sizeof( int32_t ) );
	h = fnv( h, &s->awakeCount, sizeof( int ) );
	h = fnv( h, s->awake, (size_t)s->awakeCount * sizeof( int32_t ) );
	h = fnv( h, &s->wokenCount, sizeof( int ) );
	h = fnv( h, s->woken, (size_t)s->wokenCount * sizeof( int32_t ) );
	if ( s->jointCount > 0 ) // (so a scene without joints hashes as before step 6)
	{
		h = fnv( h, &s->jointCount, sizeof( int ) );
		h = fnv( h, s->jointActive, (size_t)s->jointCount );
		h = fnv( h, s->jointColour, (size_t)s->jointCount * sizeof( int32_t ) );
		h = fnv( h, s->jointFlags, (size_t)s->jointCount * sizeof( int32_t ) );
	}
	return h;
}
