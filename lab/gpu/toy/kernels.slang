// kernels.slang: the toy's GPU stages so far (DESIGN.md "Pipeline per tick"), one source for the three dialects
// through num.slang; the narrowphase is in narrow.slang, the contact solve in solve.slang and the revolute joint in
// joint.slang (included below), the body kernels here. Every kernel takes an index list: pc.start
// and pc.count into `lists`, never "the world", so a unit can later be stepped alone. No atomics or groupshared memory
// (the saturation counter aside).
#define TOY_COUNTERS_BINDING 11
#include "num.slang"

[[vk::binding( 0, 0 )]] StructuredBuffer<Hull> hulls;
[[vk::binding( 1, 0 )]] StructuredBuffer<V3> hullPoints;
[[vk::binding( 2, 0 )]] StructuredBuffer<HullFace> hullFaces;
[[vk::binding( 3, 0 )]] StructuredBuffer<uint> hullEdges;
[[vk::binding( 4, 0 )]] RWStructuredBuffer<BodyState> bodyState;
[[vk::binding( 5, 0 )]] RWStructuredBuffer<BodyPose> bodyPose;
[[vk::binding( 6, 0 )]] RWStructuredBuffer<BodyMass> bodyMass;
[[vk::binding( 7, 0 )]] RWStructuredBuffer<Aabb> aabbs;
[[vk::binding( 8, 0 )]] StructuredBuffer<uint> lists;
[[vk::binding( 9, 0 )]] StructuredBuffer<Params> params;
[[vk::binding( 10, 0 )]] RWStructuredBuffer<Hash2> hashes;
// 11: the saturation counters (num.slang)
[[vk::binding( 12, 0 )]] StructuredBuffer<Pair> pairs;				 // this tick's, in key order
[[vk::binding( 13, 0 )]] RWStructuredBuffer<Manifold> manifolds;	 // this tick's, by pair
[[vk::binding( 14, 0 )]] StructuredBuffer<Manifold> prevManifolds;	 // last tick's (the ping-pong's other half), by prevIndex
[[vk::binding( 15, 0 )]] RWStructuredBuffer<SatAxis> satAxes;		 // narrowSat to narrowClip, by pair
[[vk::binding( 16, 0 )]] RWStructuredBuffer<NarrowDiag> narrowDiags; // diagnostics (Params.narrowDiag), by pair
[[vk::binding( 17, 0 )]] RWStructuredBuffer<Hash2> manifoldHashes;	 // by pair
[[vk::binding( 18, 0 )]] RWStructuredBuffer<Constraint> constraints; // the contact solve's, by pair (prepareContacts)
[[vk::binding( 19, 0 )]] RWStructuredBuffer<Joint> joints;			 // the revolute joints (step 6), by joint
[[vk::binding( 20, 0 )]] RWStructuredBuffer<Hash2> jointHashes;		 // by joint
[[vk::binding( 21, 0 )]] StructuredBuffer<JointCommand> jointCommands; // this tick's targets and decisions, by joint
[[vk::binding( 22, 0 )]] RWStructuredBuffer<Hash2> bodyPrepHashes;	// by body: its mass (the world inertia) and AABB
[[vk::binding( 23, 0 )]] RWStructuredBuffer<Hash2> pairWorkHashes;	// by pair: its SAT record and constraint
[[vk::push_constant]] ConstantBuffer<Push> pc;

// hash categories: every word a tick writes is in one (the saturation counters and the corpus's diagnostics aside)
#define CAT_BODY 1u		// state and pose
#define CAT_MANIFOLD 2u // the manifold, its pads aside
#define CAT_JOINT 3u	// every field
#define CAT_BODYPREP 4u // the mass (with prepareBodies' world inertia) and the AABB, every field
#define CAT_PAIRWORK 5u // the SAT record and the constraint, every field, and the manifold's pads

#include "narrow.slang"
#include "solve.slang"
#include "joint.slang"

// World inverse inertia (R invIl R^T, the mantissa keeping eI) and the integer AABB: the local bounds' centre rotated
// and the half extents through |R|, plus the margin, around the position, on the 2^-10 m grid.
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void prepareBodies( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	Params P = params[0];
	BodyPose pose = bodyPose[i];
	BodyMass m = bodyMass[i];
	Hull h = hulls[m.hull];
	M3 R = snapM3( matrixFromQuat( pose.q, P.one ) );
	m.invIw = rotateSym( R, m.invIl );
	bodyMass[i] = m;
	V3 c = mulM3V( R, h.boundsCenter );
	V3 e = absM3V( R, h.boundsHalf );
	e = add3( e, v3( P.aabbMargin, P.aabbMargin, P.aabbMargin ) );
	int2 x = gridSpanX( pose.p, c.x - e.x, c.x + e.x );
	int2 y = gridSpanY( pose.p, c.y - e.y, c.y + e.y );
	int2 z = gridSpanZ( pose.p, c.z - e.z, c.z + e.z );
	Aabb b;
	b.minx = x.x;
	b.miny = y.x;
	b.minz = z.x;
	b.maxx = x.y;
	b.maxy = y.y;
	b.maxz = z.y;
	b.pad0 = 0;
	b.pad1 = 0;
	aabbs[i] = b;
}

// b3IntegrateVelocitiesTask: gravity only (no damping, forces, torques or gyroscopic term yet).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void integrateVelocities( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	Params P = params[0];
	BodyState s = bodyState[i];
	s.v = snap3( add3( P.gravityDelta, s.v ) ); // b3MulAdd( delta, damping = 1, v )
	bodyState[i] = s;
}

// u = |a / maxSpeed|^2 and, above 1, a / sqrt(u): Box3D's speed caps without sqrt or division
V3 capSpeed( V3 a, T invMax, int s, T oneU )
{
	V3 an = mulSV( invMax, a, S_MS );
	T u = dot3( an, an, 2 * s - S_U );
	V3 r = a;
	if ( u > oneU )
	{
		T y = rsqrtT( u, S_U, S_MS );
		r = snap3( mulSV( y, a, S_MS ) );
	}
	return r;
}

// b3IntegratePositionsTask: the linear and angular speed caps, dp += h v, dq = b3IntegrateRotation( dq, h w ).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void integratePositions( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	Params P = params[0];
	BodyState s = bodyState[i];
	s.v = capSpeed( s.v, P.invMaxLinearSpeed, S_V, P.oneU );
	s.w = capSpeed( s.w, P.invMaxAngularSpeed, S_W, P.oneU );
	s.dp = snap3( mulAddSV( s.dp, P.h, s.v, S_H + S_V - S_DP ) );
	// b3IntegrateRotation: qd = (h w / 2, 0) dq; dq = normalize( dq + qd )
	V3 hw = mulSV( P.h, s.w, S_H + S_W - S_Q );
	V3 hh = mulSV( P.half, hw, S_MS );
	Q4 qd = quatMul( q4( hh.x, hh.y, hh.z, zero() ), s.dq );
	s.dq = quatNormalize( q4( s.dq.x + qd.x, s.dq.y + qd.y, s.dq.z + qd.z, qd.s + s.dq.s ), P.one );
	bodyState[i] = s;
}

// End of step: the pose from the deltas (b3FinalizeBodiesTask), the deltas reset, the sleep counter: a body is under
// the thresholds when |v|^2 + |maxExtent w|^2 <= sleep^2 (each component gated first, so nothing saturates).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void finalizeBodies( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	Params P = params[0];
	BodyState s = bodyState[i];
	BodyPose pose = bodyPose[i];
	Hull h = hulls[bodyMass[i].hull];
	// Box3D's isFast (b3FinalizeBodiesTask; it stops contact recycling): the farthest point's motion, max( |dp| + 2
	// |arc of dq|, (|v| + |arc of w|) dt ), above half the inner radius; the arcs b3ModifiedCross( |local turn|,
	// maxExtent ) in the body's frame before this step's turn. (Box3D also asks for a body that is not falling asleep;
	// one under the sleep speed cannot move that far.)
	{
		V3 lw = invRotate( pose.q, s.w );							// Q11.20
		V3 ldq = invRotate( pose.q, v3( s.dq.x, s.dq.y, s.dq.z ) ); // Q1.30
		V3 e = h.maxExtentV;
		V3 aw = v3( absT( lw.x ), absT( lw.y ), absT( lw.z ) );
		V3 aq = v3( absT( ldq.x ), absT( ldq.y ), absT( ldq.z ) );
		V3 velArc = snap3( v3( sum2( aw.y, e.z, aw.z, e.y, S_W + S_R - S_V ), sum2( aw.z, e.x, aw.x, e.z, S_W + S_R - S_V ),
							   sum2( aw.x, e.y, aw.y, e.x, S_W + S_R - S_V ) ) ); // Q9.22
		V3 rotArc = snap3( v3( sum2( aq.y, e.z, aq.z, e.y, S_Q ), sum2( aq.z, e.x, aq.x, e.z, S_Q ), sum2( aq.x, e.y, aq.y, e.x, S_Q ) ) ); // Q8.24
		T maxVelocity = snap( len3( s.v ) + len3( velArc ) ); // Q9.22
		T rotLen = len3( rotArc );
		T maxDelta = snap( rescale( len3( s.dp ), S_DP - S_R ) + ( rotLen + rotLen ) ); // Q8.24
		T maxMotion = maxT( maxDelta, M( maxVelocity, P.dt, S_V + S_H - S_R ) );
		s.flags = maxMotion > M( P.fastSafety, h.minExtent, S_MS ) ? STATE_FAST : 0;
	}
	pose.p = posAdd( pose.p, s.dp );
	pose.q = quatNormalize( quatMul( s.dq, pose.q ), P.one );
	s.dp = zero3();
	s.dq = quatIdentity( P.one );
	V3 arc = mulSV( h.maxExtent, s.w, S_R ); // maxExtent w, Q11.20
	bool sleepy = P.enableSleep != 0;
	sleepy = sleepy && absT( s.v.x ) <= P.sleepV && absT( s.v.y ) <= P.sleepV && absT( s.v.z ) <= P.sleepV;
	sleepy = sleepy && absT( arc.x ) <= P.sleepW && absT( arc.y ) <= P.sleepW && absT( arc.z ) <= P.sleepW;
	if ( sleepy )
	{
		T vv = dot3( s.v, s.v, 2 * S_V - S_VV ) + dot3( arc, arc, 2 * S_W - S_VV );
		sleepy = vv <= P.sleepSq;
	}
	s.sleepTicks = sleepy ? ( s.sleepTicks < P.sleepCap ? s.sleepTicks + 1 : s.sleepTicks ) : 0;
	bodyPose[i] = pose;
	bodyState[i] = s;
}

// A woken body starts from rest (Box3D's b3WakeSolverSet: an identity body state).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void wakeBodies( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	Params P = params[0];
	BodyState s = bodyState[i];
	s.v = zero3();
	s.w = zero3();
	s.dp = zero3();
	s.dq = quatIdentity( P.one );
	s.sleepTicks = 0;
	s.flags = 0;
	bodyState[i] = s;
}

// One 64-bit hash per element, no atomics; the CPU sums them per category, order-free (as src/hash.c does).
// Bodies: state and pose, field by field (category 1); and the body's prepared data, its mass with the world inertia
// and its AABB, every field (category 4: a GPU that rounds an AABB or a world inertia differently fails here even when
// nothing downstream moves).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void hashElements( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = lists[pc.start + tid.x];
	BodyState s = bodyState[i];
	BodyPose pose = bodyPose[i];
	H2 h = hashStart( CAT_BODY, i );
	h = hashV3( h, s.v );
	h = hashV3( h, s.w );
	h = hashV3( h, s.dp );
	h = hashQ4( h, s.dq );
	h = hashWord( h, uint( s.sleepTicks ) );
	h = hashWord( h, uint( s.flags ) );
	h = hashPos( h, pose.p );
	h = hashQ4( h, pose.q );
	hashes[i] = hashEnd( h );
	BodyMass m = bodyMass[i];
	Aabb b = aabbs[i];
	H2 g = hashStart( CAT_BODYPREP, i );
	g = hashSym( g, m.invIl );
	g = hashSym( g, m.invIw );
	g = hashT( hashT( hashT( hashT( g, m.invMass ), m.friction ), m.restitution ), m.pad );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( m.eM ) ), uint( m.eI ) ), uint( m.hull ) ), uint( m.flags ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( b.minx ) ), uint( b.miny ) ), uint( b.minz ) ), uint( b.maxx ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( b.maxy ) ), uint( b.maxz ) ), uint( b.pad0 ) ), uint( b.pad1 ) );
	bodyPrepHashes[i] = hashEnd( g );
}

// Manifolds: every field (unused point slots are zero), the pair's bodies seeding the element (order-free sums tell
// pairs apart by their bodies, not by their slot)
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void hashManifolds( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Manifold m = manifolds[k];
	H2 h = hashStart( CAT_MANIFOLD, uint( m.bodyA ) );
	h = hashWord( h, uint( m.bodyB ) );
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		ManifoldPoint p = m.points[i];
		h = hashV3( h, p.anchorA );
		h = hashV3( h, p.anchorB );
		h = hashT( h, p.separation );
		h = hashT( h, p.baseSeparation );
		h = hashT( h, p.normalImpulse );
		h = hashT( h, p.totalNormalImpulse );
		h = hashT( h, p.normalMass );
		h = hashWord( h, p.featureId );
		h = hashWord( h, uint( p.shN ) );
	}
	h = hashV3( h, m.normal );
	h = hashV3( h, m.frictionImpulse );
	h = hashT( h, m.twistImpulse );
	h = hashT( h, m.axisSeparation );
	h = hashWord( h, uint( m.pointCount ) );
	h = hashWord( h, uint( m.eP ) );
	h = hashWord( h, uint( m.axisType ) );
	h = hashWord( h, uint( m.axisA ) );
	h = hashWord( h, uint( m.axisB ) );
	h = hashWord( h, uint( m.flags ) );
	h = hashQ4( h, m.cachedRotationA );
	h = hashQ4( h, m.cachedRotationB );
	h = hashV3( h, m.cachedPoseP );
	h = hashQ4( h, m.cachedPoseQ );
	manifoldHashes[k] = hashEnd( h );
	// the pair's working records (category 5): the SAT record and the constraint, every field (an inactive pair's slot
	// keeps what an earlier tick wrote there: hashed again, the same on every machine), and the manifold's pads
	SatAxis a = satAxes[k];
	Constraint c = constraints[k];
	H2 g = hashStart( CAT_PAIRWORK, uint( m.bodyA ) );
	g = hashWord( g, uint( m.bodyB ) );
	g = hashT( hashT( hashT( hashT( g, a.faceASep ), a.faceBSep ), a.edgeSep ), a.sepSep );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( a.kind ) ), uint( a.type ) ), uint( a.sepA ) ), uint( a.sepB ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( a.cacheType ) ), uint( a.cacheA ) ), uint( a.cacheB ) ), uint( a.faceA ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( a.vertexB ) ), uint( a.faceB ) ), uint( a.vertexA ) ), uint( a.edgeA ) );
	g = hashWord( hashWord( g, uint( a.edgeB ) ), uint( a.pad ) );
	[ForceUnroll]
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		ConstraintPoint p = c.points[i];
		g = hashV3( g, p.anchorA );
		g = hashV3( g, p.anchorB );
		g = hashT( hashT( hashT( hashT( g, p.baseSeparation ), p.normalImpulse ), p.totalNormalImpulse ), p.restitutionImpulse );
		g = hashT( hashT( hashT( hashT( g, p.normalMass ), p.leverArm ), p.relativeVelocity ), p.pad );
		g = hashWord( hashWord( g, uint( p.shN ) ), uint( p.pad1 ) );
		g = hashT( g, m.points[i].pad );
	}
	g = hashV3( hashV3( hashV3( g, c.normal ), c.tangent1 ), c.tangent2 );
	g = hashV3( hashV3( g, c.centerA ), c.centerB );
	g = hashSym( hashSym( g, c.invIA ), c.invIB );
	g = hashT( hashT( g, c.invMassA ), c.invMassB );
	g = hashT( hashT( hashT( hashT( g, c.tangentMassXX ), c.tangentMassXY ), c.tangentMassYY ), c.twistMass );
	g = hashT( hashT( hashT( g, c.frictionImpulse1 ), c.frictionImpulse2 ), c.twistImpulse );
	g = hashT( hashT( hashT( g, c.friction ), c.restitution ), c.pad );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( c.bodyA ) ), uint( c.bodyB ) ), uint( c.pointCount ) ), uint( c.flags ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( c.eP ) ), uint( c.shMA ) ), uint( c.shMB ) ), uint( c.shIA ) );
	g = hashWord( hashWord( hashWord( hashWord( g, uint( c.shIB ) ), uint( c.shT ) ), uint( c.shTw ) ), uint( c.pad1 ) );
	pairWorkHashes[k] = hashEnd( g );
}
