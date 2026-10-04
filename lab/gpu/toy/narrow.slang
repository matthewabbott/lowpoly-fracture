// narrow.slang: the narrowphase (DESIGN.md "Pipeline per tick" 3 and 4), included by kernels.slang after its bindings.
// Box3D's hull versus hull, convex_manifold.c's scalar b3CollideHulls (b3QueryFaceDirections, b3QueryEdgeDirections;
// b3BuildFaceAContact, b3BuildFaceBContact, b3BuildEdgeContact, b3ClipPolygon, b3FindIncidentFace,
// b3ReduceManifoldPoints) and contact.c's b3ComputeConvexManifold (anchors, warm starts by feature id), in the three
// dialects, with the SAT cache made data: it lives in the manifold, which the next tick reads by prevIndex.
//
//   narrowSat     per active pair: last tick's cached axis first; when it still separates by the speculative distance,
//                 nothing more. Otherwise the full SAT (the face query of A, of B, the edge pairs whose Gauss-map arcs
//                 cross), all recorded in a SatAxis.
//   narrowClip    per active pair: Box3D's decision. The cached feature's contact, kept while its separation stays
//                 within linearSlop of the cache's; else the separating axis (no contact), or the face contact (the
//                 face of B when it beats A's by faceTolerance) and the edge contact when the face's is empty or the
//                 edge beats 0.9 times its clipped separation plus faceTolerance. Then the manifold in the world frame.
//   copyManifolds per inactive pair (both bodies asleep or static): last tick's manifold carried to this tick's slot.
//
// Everything is computed in A's local frame (B's points and normals through a Frame), as Box3D does; a reference face
// on B is clipped in B's frame and its points brought back. V4 formats: points, distances and separations Q8.24 (S_R;
// stored separations Q9.22), unit vectors and cosines Q1.30, areas and squared distances Q11.20. F snaps every value
// that is stored, selected or multiplied again, so nothing subnormal is ever formed.
//
// Diagnostics (Params.narrowDiag): each decision's closest call goes to NarrowDiag, so the corpus can tell a tie (two
// candidates within its tolerance in D) from a disagreement.

// ---------------------------------------------------------------------------------------------------------------------
// Hulls, feature ids, diagnostics
// ---------------------------------------------------------------------------------------------------------------------

uint edgeWord( Hull h, int e ) { return hullEdges[h.edgeStart + e]; }
int edgeNext( uint w ) { return int( w & 0xffu ); }
int edgeTwin( uint w ) { return int( ( w >> 8u ) & 0xffu ); }
int edgeOrigin( uint w ) { return int( ( w >> 16u ) & 0xffu ); }
int edgeFace( uint w ) { return int( w >> 24u ); }
V3 hullPoint( Hull h, int v ) { return hullPoints[h.vertexStart + v]; }
HullFace hullFace( Hull h, int f ) { return hullFaces[h.faceStart + f]; }

// b3MakeFeatureId( b3MakeFeaturePair( owner1, index1, owner2, index2 ) ); owner 0 is A, 1 is B
uint featureId( int owner1, int index1, int owner2, int index2 )
{
	return ( uint( owner1 ) << 24u ) | ( uint( index1 ) << 16u ) | ( uint( owner2 ) << 8u ) | uint( index2 );
}

// b3FlipPair: the owners swapped and inverted, the indices swapped (so a reference face on A or on B gives one id)
uint flipFeature( uint id )
{
	uint o1 = id >> 24u;
	uint i1 = ( id >> 16u ) & 0xffu;
	uint o2 = ( id >> 8u ) & 0xffu;
	uint i2 = id & 0xffu;
	return featureId( int( 1u - o2 ), int( i2 ), int( 1u - o1 ), int( i1 ) );
}

T bigT() { return KT( 1.0e6f, 1.0e6l, 2147483647 ); }

// m = min( m, |x| ): the closest call so far
void closest( inout T m, T x ) { m = minT( m, absT( x ) ); }

NarrowDiag diagInit()
{
	NarrowDiag d;
	d.faceAGap = bigT();
	d.faceASpec = bigT();
	d.faceBGap = bigT();
	d.faceBSpec = bigT();
	d.edgeGap = bigT();
	d.edgeSpec = bigT();
	d.edgeTest = bigT();
	d.cacheSpec = bigT();
	d.axisLen = bigT();
	d.clipLen = bigT();
	d.clipArea = bigT();
	d.clipCos = bigT();
	d.touchLen = bigT();
	d.supportA = bigT();
	d.supportB = bigT();
	d.supportGap = bigT();
	return d;
}

V3 neg3( V3 a ) { return snap3( v3( -a.x, -a.y, -a.z ) ); }
V3 rotFrameM( M3 R, V3 x ) { return snap3( mulM3V( R, x ) ); }

// ---------------------------------------------------------------------------------------------------------------------
// The SAT: face queries, edge queries
// ---------------------------------------------------------------------------------------------------------------------

// b3FindHullSupportVertex in direction -n (n in the hull's frame): the smallest n . v, the first on ties; gap
// (diagnostics): the second smallest minus the smallest
int supportMin( Hull h, V3 n, out T gap )
{
	int best = 0;
	T bestDot = zero();
	T second = bigT();
	for ( int v = 0; v < h.vertexCount; ++v )
	{
		T d = snap( dot3( n, hullPoint( h, v ), S_Q ) );
		if ( v == 0 || d < bestDot )
		{
			second = v == 0 ? second : bestDot;
			best = v;
			bestDot = d;
		}
		else if ( d < second )
		{
			second = d;
		}
	}
	bool one = !( second < bigT() );
	T sc = one ? bestDot : second; // (V4: never subtract from the sentinel)
	gap = selectT( one, bigT(), sc - bestDot );
	return best;
}

// Diagnostics: the vertex after `best` in supportMin's order (the smallest n . v of the others, the first on ties)
int supportRunnerUp( Hull h, V3 n, int best )
{
	int second = -1;
	T secondDot = zero();
	for ( int v = 0; v < h.vertexCount; ++v )
	{
		T d = snap( dot3( n, hullPoint( h, v ), S_Q ) );
		if ( v != best && ( second < 0 || d < secondDot ) )
		{
			second = v;
			secondDot = d;
		}
	}
	return second;
}

struct FaceQuery
{
	T sep;		// the best face's separation
	int face;	// of the reference hull
	int vertex; // the other hull's support vertex
	T gap;		// diagnostics: best minus second best
	T spec;		// diagnostics: the closest |separation - speculative distance|
	T support;	// diagnostics: the best face's support vertex against the next one along its normal
};

// b3QueryFaceDirections: the faces of `ref` against the points of `inc` (given in ref's frame by f), the largest
// separation, the first face on ties; stops at the first face whose separation reaches the speculative distance.
FaceQuery queryFaces( Hull rh, Hull ih, Frame f, T spec )
{
	FaceQuery q;
	q.sep = zero();
	q.face = -1;
	q.vertex = -1;
	q.gap = bigT();
	q.spec = bigT();
	q.support = bigT();
	T second = zero();
	bool haveSecond = false;
	for ( int i = 0; i < rh.faceCount; ++i )
	{
		HullFace F = hullFace( rh, i );
		T supGap = zero();
		int v = supportMin( ih, unrotFrame( f, F.normal ), supGap );
		T sep = snap( dot3( F.normal, toFrame( f, hullPoint( ih, v ) ), S_Q ) - F.offset );
		closest( q.spec, sep - spec );
		if ( q.face < 0 || sep > q.sep )
		{
			second = q.sep;
			haveSecond = q.face >= 0;
			q.sep = sep;
			q.face = i;
			q.vertex = v;
			q.support = supGap;
			if ( sep >= spec )
			{
				break;
			}
		}
		else if ( !haveSecond || sep > second )
		{
			second = sep;
			haveSecond = true;
		}
	}
	q.gap = haveSecond ? q.sep - second : bigT();
	return q;
}

// An edge of B in A's frame: its ends (the origins of the half-edge and of its twin), the edge vector and the normals
// of its two faces (b3QueryEdgeDirections' per-edge transforms; ib is even, its twin ib + 1)
struct EdgeB
{
	V3 pB;
	V3 qB;
	V3 eB;
	V3 uB;
	V3 vB;
};

EdgeB edgeOfB( Hull hB, int ib, Frame f )
{
	uint w = edgeWord( hB, ib );
	uint t = edgeWord( hB, ib + 1 );
	V3 pl = hullPoint( hB, edgeOrigin( w ) );
	V3 ql = hullPoint( hB, edgeOrigin( t ) );
	EdgeB e;
	e.pB = toFrame( f, pl );
	e.qB = toFrame( f, ql );
	e.eB = rotFrame( f, sub3( ql, pl ) );
	e.uB = rotFrame( f, hullFace( hB, edgeFace( w ) ).normal );
	e.vB = rotFrame( f, hullFace( hB, edgeFace( t ) ).normal );
	return e;
}

// Signs by comparison, never by a product (exact, and no product of two small values in F)
bool oppositeSigns( T a, T b ) { return ( a < zero() && b > zero() ) || ( a > zero() && b < zero() ); }
bool sameSigns( T a, T b ) { return ( a < zero() && b < zero() ) || ( a > zero() && b > zero() ); }

struct EdgeTest
{
	bool pass; // the arcs cross on the Gauss map and the edges are not parallel
	T sep;	   // along the axis
	V3 axis;   // unit, from B to A (Box3D's; the contact normal is its negation)
	T test;	   // diagnostics: the closest test value
};

// The Minkowski-face test of A's edge ia against B's edge e and the separation along their axis (b3QueryEdgeDirections'
// inner loop): the signed volumes cba, dba, adc, bdc; the parallel tolerance; the arc of B cut by A's plane at
// t = cba / (cba - dba) = |cba| / (|cba| + |dba|), the signs being opposite.
EdgeTest edgeTest( Hull hA, int ia, EdgeB e, Params P )
{
	uint w = edgeWord( hA, ia );
	uint t = edgeWord( hA, ia + 1 );
	V3 qA = hullPoint( hA, edgeOrigin( t ) );
	V3 eA = sub3( qA, hullPoint( hA, edgeOrigin( w ) ) );
	V3 uA = hullFace( hA, edgeFace( w ) ).normal;
	V3 vA = hullFace( hA, edgeFace( t ) ).normal;
	EdgeTest r;
	r.pass = false;
	r.sep = zero();
	r.axis = zero3();
	r.test = bigT();
	T cba = snap( dot3( e.uB, eA, S_Q ) );
	T dba = snap( dot3( e.vB, eA, S_Q ) );
	T adc = snap( -dot3( uA, e.eB, S_Q ) );
	T bdc = snap( -dot3( vA, e.eB, S_Q ) );
	if ( oppositeSigns( cba, dba ) && oppositeSigns( adc, bdc ) && sameSigns( cba, bdc ) )
	{
		T acba = absT( cba );
		T adba = absT( dba );
		// Box3D skips max(cba^2, dba^2) < tol^2 |eA|^2; here unsquared, the same test without squares that would
		// underflow Q11.20 for a 2 cm edge
		T par = snap( maxT( acba, adba ) - M( P.parallelTolerance, len3( eA ), S_Q ) );
		r.test = minT( minT( minT( acba, adba ), minT( absT( adc ), absT( bdc ) ) ), absT( par ) );
		if ( par >= zero() )
		{
			T s = divClamp( acba, acba + adba, S_Q, P.one );
			T s1 = P.one - s;
			V3 lv = v3( sum2( s1, e.uB.x, s, e.vB.x, S_Q ), sum2( s1, e.uB.y, s, e.vB.y, S_Q ), sum2( s1, e.uB.z, s, e.vB.z, S_Q ) );
			r.axis = unit3( snap3( lv ) );
			r.sep = snap( dot3( r.axis, sub3( qA, e.qB ), S_Q ) );
			r.pass = true;
		}
	}
	return r;
}

struct EdgeQuery
{
	T sep;
	int a; // half-edge of A (even), -1 when no pair passes
	int b; // half-edge of B
	T gap;
	T spec;
	T test;
};

// b3QueryEdgeDirections: B's edges outer, A's inner; the largest separation, the first pair on ties; stops at the first
// pair whose separation reaches the speculative distance
EdgeQuery queryEdges( Hull hA, Hull hB, Frame f, Params P )
{
	EdgeQuery q;
	q.sep = zero();
	q.a = -1;
	q.b = -1;
	q.gap = bigT();
	q.spec = bigT();
	q.test = bigT();
	T second = zero();
	bool haveSecond = false;
	bool stop = false;
	for ( int ib = 0; ib < hB.edgeCount && !stop; ib += 2 )
	{
		EdgeB e = edgeOfB( hB, ib, f );
		for ( int ia = 0; ia < hA.edgeCount; ia += 2 )
		{
			EdgeTest t = edgeTest( hA, ia, e, P );
			if ( !t.pass )
			{
				continue;
			}
			closest( q.spec, t.sep - P.speculativeDistance );
			if ( q.a < 0 || t.sep > q.sep )
			{
				second = q.sep;
				haveSecond = q.a >= 0;
				q.sep = t.sep;
				q.a = ia;
				q.b = ib;
				q.test = t.test;
				if ( t.sep >= P.speculativeDistance )
				{
					stop = true;
					break;
				}
			}
			else if ( !haveSecond || t.sep > second )
			{
				second = t.sep;
				haveSecond = true;
			}
		}
	}
	q.gap = haveSecond ? q.sep - second : bigT();
	return q;
}

// ---------------------------------------------------------------------------------------------------------------------
// Contacts: the face contact (incident face, clipping, reduction) and the edge contact
// ---------------------------------------------------------------------------------------------------------------------

// The face of edge e (or of its twin) most anti-parallel to n; margin: the two faces' cosines apart
int facePick( Hull h, int e, V3 n, out T margin )
{
	uint w = edgeWord( h, e );
	int f1 = edgeFace( w );
	int f2 = edgeFace( edgeWord( h, edgeTwin( w ) ) );
	T d1 = snap( dot3( hullFace( h, f1 ).normal, n, S_Q ) );
	T d2 = snap( dot3( hullFace( h, f2 ).normal, n, S_Q ) );
	margin = d1 - d2;
	return d1 < d2 ? f1 : f2;
}

// b3FindIncidentFace: from the support vertex, the edge most perpendicular to the reference normal n (in the hull's
// frame), then that edge's face most anti-parallel to n. The vertex's first half-edge is the lowest-index one leaving it.
int incidentFace( Hull h, V3 n, int vertex, inout NarrowDiag dg )
{
	int first = 0;
	for ( int e = h.edgeCount - 1; e >= 0; --e )
	{
		if ( edgeOrigin( edgeWord( h, e ) ) == vertex )
		{
			first = e;
		}
	}
	V3 o = hullPoint( h, vertex );
	int minEdge = first;
	int secondEdge = -1;
	T minProj = zero();
	T secondProj = zero();
	int e = first;
	for ( int g = 0; g < HULL_MAX_EDGES; ++g )
	{
		uint w = edgeWord( h, e );
		uint tw = edgeWord( h, edgeTwin( w ) );
		V3 axis = unit3( sub3( hullPoint( h, edgeOrigin( tw ) ), o ) );
		T proj = absT( snap( dot3( axis, n, S_Q ) ) );
		if ( g == 0 || proj < minProj )
		{
			secondEdge = g == 0 ? -1 : minEdge;
			secondProj = minProj;
			minEdge = e;
			minProj = proj;
		}
		else if ( secondEdge < 0 || proj < secondProj )
		{
			secondEdge = e;
			secondProj = proj;
		}
		e = edgeNext( tw );
		if ( e == first )
		{
			break;
		}
	}
	T faceMargin = zero();
	int face = facePick( h, minEdge, n, faceMargin );
	closest( dg.clipCos, faceMargin );
	if ( secondEdge >= 0 )
	{
		T unused = zero();
		if ( facePick( h, secondEdge, n, unused ) != face )
		{
			closest( dg.clipCos, secondProj - minProj ); // a near tie between edges that lead to different faces
		}
	}
	return face;
}

struct ClipVertex
{
	V3 pos; // in the reference frame
	T sep;	// to the reference plane
	uint id;
};

struct LocalPoint
{
	V3 pos; // in the reference frame (then A's), halfway between the faces
	T sep;	  // Q8.24
	uint id;
};

LocalPoint noPoint()
{
	LocalPoint p;
	p.pos = zero3();
	p.sep = zero();
	p.id = 0u;
	return p;
}

// b3ClipPolygon's new vertex where the segment a b crosses the side plane (da, db: their distances, of opposite sides)
ClipVertex clipCross( ClipVertex a, ClipVertex b, T da, T db, V3 n, T d, Params P )
{
	T ada = absT( da );
	T s = divClamp( ada, ada + absT( db ), S_Q, P.one ); // Box3D's d1 / (d1 - d2)
	ClipVertex c;
	c.pos = snap3( mulAddSV( a.pos, s, sub3( b.pos, a.pos ), S_Q ) );
	c.sep = snap( dot3( n, c.pos, S_Q ) - d );
	c.id = 0u;
	return c;
}

// b3ArbitraryPerp: a unit vector perpendicular to the unit n, from n's largest component's branch
V3 arbitraryPerp( V3 n )
{
	T a = KT( 0.67f, 0.67l, 719407022 );
	T b = KT( -0.42f, -0.42l, -450971566 );
	T h = KT( 0.5f, 0.5l, 536870912 );
	V3 p;
	if ( n.x < -h || h < n.x )
	{
		p = v3( sum2( a, n.y, b, n.z, S_Q ), M( -a, n.x, S_Q ), M( -b, n.x, S_Q ) );
	}
	else if ( n.y < -h || h < n.y )
	{
		p = v3( M( a, n.y, S_Q ), sum2( -a, n.x, b, n.z, S_Q ), M( -b, n.y, S_Q ) );
	}
	else
	{
		p = v3( M( a, n.z, S_Q ), M( b, n.z, S_Q ), dif2( -a, n.x, b, n.y, S_Q ) );
	}
	return unit3( snap3( p ) );
}

// b3ReduceManifoldPoints (the 4-point version): the deepest extreme point along an arbitrary tangent, the farthest
// from it, the largest triangle, the largest area outside it, each with Box3D's 0.95 pecking-order bias; at most four
// points pass through untouched (even beyond the speculative distance, as Box3D's). pts is consumed.
int reducePoints( inout LocalPoint pts[CLIP_MAX], int count, V3 n, Params P, inout LocalPoint outPts[MANIFOLD_POINTS], inout NarrowDiag dg )
{
	if ( count <= MANIFOLD_POINTS )
	{
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			if ( i < count )
			{
				outPts[i] = pts[i];
			}
		}
		return count;
	}
	T spec = P.speculativeDistance;
	T tolSq = P.speculativeSq;
	T bias = P.reduceBias;

	// 1: the extreme touching point (the deeper the better)
	V3 dir = arbitraryPerp( n );
	int best = -1;
	T bestScore = zero();
	for ( int i = 0; i < count; ++i )
	{
		closest( dg.clipLen, pts[i].sep - spec );
		if ( pts[i].sep > spec )
		{
			continue;
		}
		T score = snap( dot3( dir, pts[i].pos, S_Q ) - pts[i].sep );
		T bs = M( bias, score, S_Q );
		if ( best >= 0 )
		{
			closest( dg.clipLen, bs - bestScore );
		}
		if ( best < 0 || bs > bestScore )
		{
			best = i;
			bestScore = score;
		}
	}
	if ( best < 0 )
	{
		return 0;
	}
	outPts[0] = pts[best];
	pts[best] = pts[count - 1];
	count -= 1;
	V3 a = outPts[0].pos;

	// 2: the farthest in the plane (and deep)
	best = -1;
	bestScore = zero();
	for ( int i = 0; i < count; ++i )
	{
		V3 dd = sub3( pts[i].pos, a );
		V3 v = snap3( mulSubSV( dd, snap( dot3( dd, n, S_Q ) ), n, S_Q ) );
		T s = maxT( zero(), -pts[i].sep );
		T s2 = s + s;
		T score = snap( dot3( v, v, 2 * S_R - S_D2 ) + M( s2, s2, 2 * S_R - S_D2 ) ); // |v|^2 + 4 s^2
		T bs = M( bias, score, S_Q );
		closest( dg.clipArea, bs - bestScore );
		if ( bs > bestScore )
		{
			bestScore = score;
			best = i;
		}
	}
	closest( dg.clipArea, bestScore - tolSq );
	if ( bestScore < tolSq )
	{
		return 1;
	}
	outPts[1] = pts[best];
	pts[best] = pts[count - 1];
	count -= 1;
	V3 b = outPts[1].pos;

	// 3: the largest triangle
	best = -1;
	bestScore = tolSq;
	T bestArea = zero();
	V3 ba = sub3( b, a );
	for ( int i = 0; i < count; ++i )
	{
		T area = snap( dot3( n, snap3( cross3( ba, sub3( pts[i].pos, a ), 2 * S_R - S_D2 ) ), S_Q ) );
		T score = absT( area );
		T bs = M( bias, score, S_Q );
		closest( dg.clipArea, bs - bestScore );
		if ( bs >= bestScore )
		{
			bestScore = score;
			best = i;
			bestArea = area;
		}
	}
	if ( best < 0 )
	{
		return 2;
	}
	outPts[2] = pts[best];
	pts[best] = pts[count - 1];
	count -= 1;
	V3 c = outPts[2].pos;

	// 4: the most area added outside the triangle
	best = -1;
	bestScore = tolSq;
	bool flip = bestArea < zero();
	V3 cb = sub3( c, b );
	V3 ac = sub3( a, c );
	for ( int i = 0; i < count; ++i )
	{
		V3 p = pts[i].pos;
		T u1 = snap( dot3( n, snap3( cross3( sub3( p, a ), ba, 2 * S_R - S_D2 ) ), S_Q ) );
		T u2 = snap( dot3( n, snap3( cross3( sub3( p, b ), cb, 2 * S_R - S_D2 ) ), S_Q ) );
		T u3 = snap( dot3( n, snap3( cross3( sub3( p, c ), ac, 2 * S_R - S_D2 ) ), S_Q ) );
		u1 = selectT( flip, -u1, u1 );
		u2 = selectT( flip, -u2, u2 );
		u3 = selectT( flip, -u3, u3 );
		T score = maxT( u1, maxT( u2, u3 ) );
		T bs = M( bias, score, S_Q );
		closest( dg.clipArea, bs - bestScore );
		if ( bs > bestScore )
		{
			bestScore = score;
			best = i;
		}
	}
	if ( best < 0 )
	{
		return 3;
	}
	outPts[3] = pts[best];
	return 4;
}

struct FaceContact
{
	bool touching;
	int count;
	LocalPoint pts[MANIFOLD_POINTS]; // in the reference frame; ids with the incident hull as owner 1 (Box3D's)
	T minSep;						 // the cache's separation
};

// b3BuildFaceAContact: reference face refFace of `ref`, the incident hull `inc` given in ref's frame by f, vertex its
// support vertex. b3BuildPolygon (the incident face's vertices, each the head of a half-edge: ids (B, edge, B, next)),
// Sutherland-Hodgman against the reference face's side planes (b3ClipPolygon: a crossing takes the reference edge as
// its incoming or outgoing feature), the points halfway between the faces, none touching beyond the speculative
// distance, the reduction.
FaceContact faceContact( Hull rh, int refFace, Hull ih, int vertex, Frame f, Params P, inout NarrowDiag dg )
{
	FaceContact r;
	r.touching = false;
	r.count = 0;
	r.minSep = zero();
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		r.pts[i] = noPoint();
	}
	HullFace F = hullFace( rh, refFace );
	V3 n = F.normal;
	T d = F.offset;
	V3 nInc = unrotFrame( f, n );
	int incFace = incidentFace( ih, nInc, vertex, dg );
	if ( P.narrowDiag != 0 )
	{
		// diagnostics: a near tie for the support vertex (the SAT's, in the same direction) that would start the search
		// from a vertex with another incident face decides whether, and how, the contact touches
		int v2 = supportRunnerUp( ih, nInc, vertex );
		if ( v2 >= 0 )
		{
			NarrowDiag scratch = diagInit();
			if ( incidentFace( ih, nInc, v2, scratch ) != incFace )
			{
				T d1 = snap( dot3( nInc, hullPoint( ih, vertex ), S_Q ) );
				T d2 = snap( dot3( nInc, hullPoint( ih, v2 ), S_Q ) );
				closest( dg.touchLen, d2 - d1 );
			}
		}
	}

	ClipVertex poly[2 * CLIP_MAX];
	int cnt = 0;
	int e0 = hullFace( ih, incFace ).edge;
	int e = e0;
	for ( int g = 0; g < HULL_MAX_EDGES && cnt < CLIP_MAX; ++g )
	{
		int nx = edgeNext( edgeWord( ih, e ) );
		ClipVertex v;
		v.pos = toFrame( f, hullPoint( ih, edgeOrigin( edgeWord( ih, nx ) ) ) );
		v.sep = snap( dot3( n, v.pos, S_Q ) - d );
		v.id = featureId( 1, e, 1, nx );
		poly[cnt] = v;
		cnt += 1;
		e = nx;
		if ( e == e0 )
		{
			break;
		}
	}

	int src = 0;
	int dst = CLIP_MAX;
	e = F.edge;
	for ( int g = 0; g < HULL_MAX_EDGES; ++g )
	{
		uint w = edgeWord( rh, e );
		int nx = edgeNext( w );
		V3 v1 = hullPoint( rh, edgeOrigin( w ) );
		V3 v2 = hullPoint( rh, edgeOrigin( edgeWord( rh, nx ) ) );
		V3 bn = snap3( cross3( unit3( sub3( v2, v1 ) ), n, S_Q ) );
		T off = snap( dot3( bn, v1, S_Q ) );
		int no = 0;
		T passMin = bigT(); // this pass's closest distance: it decides the count when the count ends at 2 or 3
		ClipVertex a = poly[src + cnt - 1];
		T da = snap( dot3( bn, a.pos, S_Q ) - off );
		for ( int i = 0; i < cnt; ++i )
		{
			ClipVertex b = poly[src + i];
			T db = snap( dot3( bn, b.pos, S_Q ) - off );
			closest( dg.clipLen, db );
			closest( passMin, db );
			if ( da <= zero() && db <= zero() )
			{
				if ( no < CLIP_MAX )
				{
					poly[dst + no] = b;
					no += 1;
				}
			}
			else if ( da <= zero() && db > zero() )
			{
				ClipVertex c = clipCross( a, b, da, db, n, d, P );
				c.id = ( b.id & 0xffff0000u ) | uint( e ); // the outgoing feature: A's edge
				if ( no < CLIP_MAX )
				{
					poly[dst + no] = c;
					no += 1;
				}
			}
			else if ( db <= zero() && da > zero() )
			{
				ClipVertex c = clipCross( a, b, da, db, n, d, P );
				c.id = ( a.id & 0x0000ffffu ) | ( uint( e ) << 16u ); // the incoming feature: A's edge
				if ( no < CLIP_MAX )
				{
					poly[dst + no] = c;
					no += 1;
				}
				if ( no < CLIP_MAX )
				{
					poly[dst + no] = b;
					no += 1;
				}
			}
			a = b;
			da = db;
		}
		cnt = no;
		if ( cnt == 2 || cnt == 3 )
		{
			closest( dg.touchLen, passMin );
		}
		int t = src;
		src = dst;
		dst = t;
		if ( cnt < 3 )
		{
			return r;
		}
		e = nx;
		if ( e == F.edge )
		{
			break;
		}
	}

	LocalPoint pts[CLIP_MAX];
	T minSep = zero();
	for ( int i = 0; i < cnt; ++i )
	{
		ClipVertex c = poly[src + i];
		LocalPoint p;
		p.pos = snap3( mulSubSV( c.pos, snap( M( P.half, c.sep, S_MS ) ), n, S_Q ) ); // halfway: pos - sep n / 2
		p.sep = c.sep;
		p.id = c.id;
		pts[i] = p;
		minSep = ( i == 0 || c.sep < minSep ) ? c.sep : minSep;
	}
	closest( dg.touchLen, minSep - P.speculativeDistance );
	if ( minSep >= P.speculativeDistance )
	{
		return r;
	}
	r.count = reducePoints( pts, cnt, n, P, r.pts, dg );
	r.touching = true;
	r.minSep = minSep;
	return r;
}

struct EdgeContact
{
	bool touching;
	LocalPoint pt; // in A's frame
};

// b3BuildEdgeContact: the closest points of the two edges' lines, none when either falls outside its segment. Box3D
// solves b3LineDistance's 2x2 system in |e|^2 units; here along unit directions instead, t1 = (w . (uB x m)) / sin and
// t2 = (w . (uA x m)) / sin, with m the unit common normal, sin = |uA x uB| and w = pB - pA: the same points, with no
// m^4 determinant to lose V4's bits for short edges. normal: from A to B.
EdgeContact edgeContact( Hull hA, int ia, Hull hB, int ib, Frame f, V3 normal, Params P, inout NarrowDiag dg )
{
	uint w = edgeWord( hA, ia );
	uint t = edgeWord( hA, edgeTwin( w ) );
	V3 pA = hullPoint( hA, edgeOrigin( w ) );
	V3 eA = sub3( hullPoint( hA, edgeOrigin( t ) ), pA );
	EdgeB e = edgeOfB( hB, ib, f );
	V3 uA = unit3( eA );
	V3 uB = unit3( e.eB );
	V3 c = snap3( cross3( uA, uB, S_Q ) );
	V3 m = unit3( c );
	T sinT = snap( dot3( m, c, S_Q ) );
	V3 wv = sub3( e.pB, pA );
	T n1 = snap( dot3( wv, snap3( cross3( uB, m, S_Q ) ), S_Q ) );
	T n2 = snap( dot3( wv, snap3( cross3( uA, m, S_Q ) ), S_Q ) );
	T lim = KT( 64.0f, 64.0l, 1073741824 ); // 64 m: far beyond any edge
	T t1 = divClamp( n1, sinT, S_Q, lim );
	T t2 = divClamp( n2, sinT, S_Q, lim );
	T lenA = snap( dot3( eA, uA, S_Q ) );
	T lenB = snap( dot3( e.eB, uB, S_Q ) );
	closest( dg.touchLen, t1 );
	closest( dg.touchLen, lenA - t1 );
	closest( dg.touchLen, t2 );
	closest( dg.touchLen, lenB - t2 );
	EdgeContact r;
	r.touching = sinT > zero() && t1 >= zero() && t1 <= lenA && t2 >= zero() && t2 <= lenB;
	V3 p1 = snap3( mulAddSV( pA, t1, uA, S_Q ) );
	V3 p2 = snap3( mulAddSV( e.pB, t2, uB, S_Q ) );
	V3 dp = sub3( p2, p1 );
	r.pt.sep = snap( dot3( normal, dp, S_Q ) );
	r.pt.pos = snap3( mulAddSV( p1, P.half, dp, S_MS ) ); // (p1 + p2) / 2
	r.pt.id = featureId( 0, ia, 1, ib );
	return r;
}

// A local manifold in A's frame
struct Contact
{
	int count;
	V3 normal; // from A to B, Q1.30
	LocalPoint pts[MANIFOLD_POINTS];
};

Contact noContact()
{
	Contact c;
	c.count = 0;
	c.normal = zero3();
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		c.pts[i] = noPoint();
	}
	return c;
}

// A face contact with the reference face on A or on B (b3BuildFaceBContact: clipped in B's frame, the points brought
// into A's, the normal negated so it points from A to B, the ids flipped)
Contact faceContactAB( bool onB, int face, int vertex, Hull hA, Hull hB, Frame fBA, Frame fAB, Params P, inout NarrowDiag dg,
					   out bool touching, out T minSep )
{
	Hull rh = hA;
	Hull ih = hB;
	Frame f = fBA;
	if ( onB )
	{
		rh = hB;
		ih = hA;
		f = fAB;
	}
	FaceContact fc = faceContact( rh, face, ih, vertex, f, P, dg );
	Contact c = noContact();
	c.count = fc.count;
	if ( onB )
	{
		c.normal = neg3( rotFrame( fBA, hullFace( hB, face ).normal ) );
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			LocalPoint p = fc.pts[i];
			if ( i < fc.count )
			{
				p.pos = toFrame( fBA, p.pos );
				p.id = flipFeature( p.id );
			}
			c.pts[i] = p;
		}
	}
	else
	{
		c.normal = hullFace( hA, face ).normal;
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			c.pts[i] = fc.pts[i];
		}
	}
	touching = fc.touching;
	minSep = fc.minSep;
	return c;
}

// The edge contact of A's edge ia and B's edge ib (their axis recomputed: the same arithmetic as the query's)
Contact edgeContactAB( int ia, int ib, Hull hA, Hull hB, Frame fBA, Params P, inout NarrowDiag dg, out bool touching )
{
	EdgeTest t = edgeTest( hA, ia, edgeOfB( hB, ib, fBA ), P );
	V3 normal = neg3( t.axis );
	EdgeContact ec = edgeContact( hA, ia, hB, ib, fBA, normal, P, dg );
	Contact c = noContact();
	touching = ec.touching;
	if ( ec.touching )
	{
		c.count = 1;
		c.normal = normal;
		c.pts[0] = ec.pt;
	}
	return c;
}

Manifold emptyManifold( int bodyA, int bodyB )
{
	Manifold m;
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		ManifoldPoint p;
		p.anchorA = zero3();
		p.anchorB = zero3();
		p.separation = zero();
		p.baseSeparation = zero();
		p.normalImpulse = zero();
		p.totalNormalImpulse = zero();
		p.normalMass = zero();
		p.pad = zero();
		p.featureId = 0u;
		p.shN = 0;
		m.points[i] = p;
	}
	m.normal = zero3();
	m.frictionImpulse = zero3();
	m.twistImpulse = zero();
	m.axisSeparation = zero();
	m.pointCount = 0;
	m.bodyA = bodyA;
	m.bodyB = bodyB;
	m.eP = 0;
	m.axisType = AXIS_NONE;
	m.axisA = 0;
	m.axisB = 0;
	m.pad = 0;
	return m;
}

SatAxis satInit()
{
	SatAxis s;
	s.faceASep = zero();
	s.faceBSep = zero();
	s.edgeSep = zero();
	s.sepSep = zero();
	s.kind = SAT_OVERLAP;
	s.type = AXIS_NONE;
	s.sepA = 0;
	s.sepB = 0;
	s.cacheType = AXIS_NONE;
	s.cacheA = 0;
	s.cacheB = 0;
	s.faceA = -1;
	s.vertexB = -1;
	s.faceB = -1;
	s.vertexA = -1;
	s.edgeA = -1;
	s.edgeB = -1;
	s.pad = 0;
	return s;
}

// ---------------------------------------------------------------------------------------------------------------------
// Kernels
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void narrowSat( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	T spec = P.speculativeDistance;
	Pair pr = pairs[k];
	Hull hA = hulls[bodyMass[pr.bodyA].hull];
	Hull hB = hulls[bodyMass[pr.bodyB].hull];
	Frame fBA = frameBtoA( bodyPose[pr.bodyA], bodyPose[pr.bodyB], P.one );
	Frame fAB = invFrame( fBA );
	SatAxis s = satInit();
	NarrowDiag dg = diagInit();
	int cType = AXIS_NONE;
	int cA = 0;
	int cB = 0;
	if ( pr.prevIndex >= 0 )
	{
		Manifold pm = prevManifolds[pr.prevIndex];
		cType = pm.axisType;
		cA = pm.axisA;
		cB = pm.axisB;
	}

	// the cached axis: still separating? (Box3D: >= for a face, > for an edge pair whose arcs still cross)
	bool stop = false;
	if ( cType == AXIS_FACE_A || cType == AXIS_FACE_B )
	{
		bool onB = cType == AXIS_FACE_B;
		int face = onB ? cB : cA;
		Hull rh = hA;
		Hull ih = hB;
		Frame f = fBA;
		if ( onB )
		{
			rh = hB;
			ih = hA;
			f = fAB;
		}
		HullFace F = hullFace( rh, face );
		T supGap = zero();
		int v = supportMin( ih, unrotFrame( f, F.normal ), supGap );
		T sep = snap( dot3( F.normal, toFrame( f, hullPoint( ih, v ) ), S_Q ) - F.offset );
		closest( dg.cacheSpec, sep - spec );
		dg.supportGap = supGap;
		if ( sep >= spec )
		{
			s.kind = SAT_CACHE_SEPARATED;
			stop = true;
		}
		else
		{
			s.cacheType = cType;
			s.cacheA = onB ? v : face;
			s.cacheB = onB ? face : v;
		}
	}
	else if ( cType == AXIS_EDGE )
	{
		EdgeTest t = edgeTest( hA, cA, edgeOfB( hB, cB, fBA ), P );
		if ( t.pass )
		{
			closest( dg.cacheSpec, t.sep - spec );
			if ( t.sep > spec )
			{
				s.kind = SAT_CACHE_SEPARATED;
				stop = true;
			}
			else
			{
				s.cacheType = AXIS_EDGE;
				s.cacheA = cA;
				s.cacheB = cB;
			}
		}
	}

	// the full SAT (narrowClip uses it when the cached feature's contact fails or there is none)
	if ( !stop )
	{
		FaceQuery qa = queryFaces( hA, hB, fBA, spec );
		s.faceASep = qa.sep;
		s.faceA = qa.face;
		s.vertexB = qa.vertex;
		dg.faceAGap = qa.gap;
		dg.faceASpec = qa.spec;
		dg.supportA = qa.support;
		if ( qa.sep > spec )
		{
			s.kind = SAT_SEPARATED;
			s.type = AXIS_FACE_A;
			s.sepA = qa.face;
			s.sepB = qa.vertex;
			s.sepSep = qa.sep;
		}
		else
		{
			FaceQuery qb = queryFaces( hB, hA, fAB, spec );
			s.faceBSep = qb.sep;
			s.faceB = qb.face;
			s.vertexA = qb.vertex;
			dg.faceBGap = qb.gap;
			dg.faceBSpec = qb.spec;
			dg.supportB = qb.support;
			if ( qb.sep > spec )
			{
				s.kind = SAT_SEPARATED;
				s.type = AXIS_FACE_B;
				s.sepA = qb.vertex;
				s.sepB = qb.face;
				s.sepSep = qb.sep;
			}
			else
			{
				EdgeQuery qe = queryEdges( hA, hB, fBA, P );
				s.edgeSep = qe.sep;
				s.edgeA = qe.a;
				s.edgeB = qe.b;
				dg.edgeGap = qe.gap;
				dg.edgeSpec = qe.spec;
				dg.edgeTest = qe.test;
				if ( qe.a >= 0 && qe.sep > spec )
				{
					s.kind = SAT_SEPARATED;
					s.type = AXIS_EDGE;
					s.sepA = qe.a;
					s.sepB = qe.b;
					s.sepSep = qe.sep;
				}
			}
		}
	}
	satAxes[k] = s;
	if ( P.narrowDiag != 0 )
	{
		narrowDiags[k] = dg;
	}
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void narrowClip( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Pair pr = pairs[k];
	BodyPose poseA = bodyPose[pr.bodyA];
	BodyPose poseB = bodyPose[pr.bodyB];
	Hull hA = hulls[bodyMass[pr.bodyA].hull];
	Hull hB = hulls[bodyMass[pr.bodyB].hull];
	Frame fBA = frameBtoA( poseA, poseB, P.one );
	Frame fAB = invFrame( fBA );
	SatAxis s = satAxes[k];
	NarrowDiag dg = diagInit();
	if ( P.narrowDiag != 0 )
	{
		dg = narrowDiags[k];
	}
	bool hasPrev = pr.prevIndex >= 0;
	Manifold pm = emptyManifold( pr.bodyA, pr.bodyB );
	if ( hasPrev )
	{
		pm = prevManifolds[pr.prevIndex];
	}

	// the cache (Box3D's b3SATCache) stays unless the full SAT runs
	int cType = pm.axisType;
	int cA = pm.axisA;
	int cB = pm.axisB;
	T cSep = pm.axisSeparation;
	Contact c = noContact();
	T axisM = bigT();
	if ( s.kind == SAT_CACHE_SEPARATED )
	{
		closest( axisM, dg.cacheSpec );
	}
	else
	{
		bool done = false;
		if ( s.cacheType != AXIS_NONE )
		{
			// the cached feature's contact, kept while touching and within linearSlop of the cache's separation
			bool touching = false;
			T sep = zero();
			if ( s.cacheType == AXIS_EDGE )
			{
				c = edgeContactAB( s.cacheA, s.cacheB, hA, hB, fBA, P, dg, touching );
				sep = c.pts[0].sep;
			}
			else
			{
				bool onB = s.cacheType == AXIS_FACE_B;
				c = faceContactAB( onB, onB ? s.cacheB : s.cacheA, onB ? s.cacheA : s.cacheB, hA, hB, fBA, fAB, P, dg, touching, sep );
			}
			T drift = absT( cSep - rescale( sep, S_R - S_S ) ) - P.cacheTolerance;
			T oneM = KT( 1.0f, 1.0l, 4194304 ); // 1 m in Q9.22: the margin's range, so V4's rescale cannot saturate
			closest( axisM, dg.cacheSpec );
			closest( axisM, rescale( clampT( drift, -oneM, oneM ), S_S - S_R ) );
			done = touching && drift < zero();
			if ( !done )
			{
				c = noContact();
			}
		}
		if ( !done )
		{
			cType = AXIS_NONE;
			cA = 0;
			cB = 0;
			cSep = zero();
			if ( s.kind == SAT_SEPARATED )
			{
				cType = s.type;
				cA = s.sepA;
				cB = s.sepB;
				cSep = rescale( s.sepSep, S_R - S_S );
				closest( axisM, dg.faceASpec );
				if ( s.type != AXIS_FACE_A )
				{
					closest( axisM, dg.faceBSpec );
				}
				if ( s.type == AXIS_EDGE )
				{
					closest( axisM, dg.edgeSpec );
					closest( axisM, dg.edgeTest );
				}
			}
			else
			{
				// the face contact: B's face when it beats A's by faceTolerance
				bool onB = s.faceBSep > s.faceASep + P.faceTolerance;
				closest( axisM, s.faceBSep - s.faceASep - P.faceTolerance );
				closest( axisM, dg.faceASpec );
				closest( axisM, dg.faceBSpec );
				closest( axisM, onB ? dg.faceBGap : dg.faceAGap );
				if ( s.edgeA >= 0 )
				{
					closest( axisM, dg.edgeSpec );
				}
				bool touching = false;
				T minSep = zero();
				c = faceContactAB( onB, onB ? s.faceB : s.faceA, onB ? s.vertexA : s.vertexB, hA, hB, fBA, fAB, P, dg, touching, minSep );
				if ( touching )
				{
					cType = onB ? AXIS_FACE_B : AXIS_FACE_A;
					cA = onB ? s.vertexA : s.faceA;
					cB = onB ? s.faceB : s.vertexB;
					cSep = rescale( minSep, S_R - S_S );
				}
				// the edge contact when the face's is empty or the edge is better (Box3D's clipped face separation is
				// the cache's: 0 when the face contact failed)
				if ( s.edgeA >= 0 )
				{
					T clipped = touching ? minSep : zero();
					T bar = M( P.edgeRelTolerance, clipped, S_Q ) + P.faceTolerance;
					if ( c.count > 0 )
					{
						closest( axisM, s.edgeSep - bar );
					}
					if ( c.count == 0 || s.edgeSep > bar )
					{
						closest( axisM, dg.edgeGap );
						closest( axisM, dg.edgeTest );
						bool edgeTouching = false;
						Contact ce = edgeContactAB( s.edgeA, s.edgeB, hA, hB, fBA, P, dg, edgeTouching );
						if ( edgeTouching )
						{
							c = ce;
							cType = AXIS_EDGE;
							cA = s.edgeA;
							cB = s.edgeB;
							cSep = rescale( ce.pts[0].sep, S_R - S_S );
						}
						else
						{
							// b3BuildEdgeContact clears the cache when the closest points fall outside the edges
							cType = AXIS_NONE;
							cA = 0;
							cB = 0;
							cSep = zero();
						}
					}
				}
			}
		}
	}
	dg.axisLen = axisM;

	// b3ComputeConvexManifold: the normal and the anchors in the world frame (from each body's centre of mass), the
	// separations, the ids; each point takes the impulse of the first of last tick's points with its id (each claimed
	// once); the friction and twist impulses carry while the pair keeps touching
	Manifold m = emptyManifold( pr.bodyA, pr.bodyB );
	m.axisType = cType;
	m.axisA = cA;
	m.axisB = cB;
	m.axisSeparation = cSep;
	m.pointCount = c.count;
	if ( c.count > 0 )
	{
		M3 RA = snapM3( matrixFromQuat( poseA.q, P.one ) );
		m.normal = rotFrameM( RA, c.normal );
		V3 d = posDelta( poseA.p, poseB.p );
		bool carry = hasPrev && pm.pointCount > 0;
		uint claimed = 0u;
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			if ( i < c.count )
			{
				LocalPoint lp = c.pts[i];
				ManifoldPoint mp = m.points[i];
				mp.anchorA = rotFrameM( RA, lp.pos );
				mp.anchorB = snap3( sub3( mp.anchorA, d ) );
				mp.separation = rescale( lp.sep, S_R - S_S );
				mp.featureId = lp.id;
				for ( int j = 0; j < MANIFOLD_POINTS; ++j )
				{
					bool avail = carry && j < pm.pointCount && ( claimed & ( 1u << uint( j ) ) ) == 0u;
					if ( avail && pm.points[j].featureId == lp.id )
					{
						mp.normalImpulse = pm.points[j].normalImpulse;
						claimed = claimed | ( 1u << uint( j ) );
						break;
					}
				}
				m.points[i] = mp;
			}
		}
		if ( carry )
		{
			m.frictionImpulse = pm.frictionImpulse;
			m.twistImpulse = pm.twistImpulse;
			m.eP = pm.eP;
		}
	}
	manifolds[k] = m;
	if ( P.narrowDiag != 0 )
	{
		narrowDiags[k] = dg;
	}
}

// Last tick's manifold into this tick's slot, for a pair whose bodies are asleep or static (its cache and impulses
// wait for the wake); a new pair starts empty
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void copyManifolds( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Pair pr = pairs[k];
	Manifold m = emptyManifold( pr.bodyA, pr.bodyB );
	if ( pr.prevIndex >= 0 )
	{
		m = prevManifolds[pr.prevIndex];
	}
	manifolds[k] = m;
}
