// kernels.slang: the toy's GPU stages so far (DESIGN.md "Pipeline per tick"), one source for the three dialects
// through num.slang. Every kernel takes an index list: pc.start and pc.count into `lists`, never "the world", so a unit
// can later be stepped alone. No atomics or groupshared memory (the saturation counter aside).
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
[[vk::push_constant]] ConstantBuffer<Push> pc;

// hash categories (hashElements' pc.aux)
#define CAT_BODY 1u

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
	M3 R = matrixFromQuat( pose.q, P.one );
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
	bodyState[i] = s;
}

// One 64-bit hash per element, no atomics; the CPU sums them per category, order-free (as src/hash.c does).
// Bodies: state and pose, field by field.
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
}
