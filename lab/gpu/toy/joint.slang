// joint.slang: the revolute joint (step 6; DESIGN.md "The joint"), included by kernels.slang after solve.slang. Box3D's
// b3RevoluteJoint (revolute_joint.c: b3PrepareRevoluteJoint, b3WarmStartRevoluteJoint, b3SolveRevoluteJoint, its soft
// step with useBias in the solve and without it in the relax), one joint per thread, in the three dialects:
//
//   prepareJoints     per active joint: the world frames and anchors, the hinge axis, the masses, the 3x3 point block
//                     inverted once per step by Gaussian elimination with reciprocal pivots (num.slang's invertSym), the
//                     2x2 axis block inverted (as the contact's tangent mass), the axial mass; the twist angle; the
//                     servo's motor speed from this tick's target (a JointCommand), capped; the carried impulses
//                     rescaled to this step's exponent
//   warmStartJoints   per colour, per substep: the carried impulses applied
//   solveJoints       per colour, per substep (useBias): the motor (speed, torque cap), the speculative soft limits, the
//                     axis block (the hinge axes kept aligned), the point block (the anchors kept together)
//   relaxJoints       the same without the bias (Box3D's relax: rigid, speculative limits only)
//   hashJoints        one element hash per joint (every field), its own category
//
// Deviations from Box3D (NOTES.md, step 6): the point and axis blocks' masses are computed once per step at prepare
// (Box3D recomputes both in every solve from the current anchors and axes; the Jacobians here are current, the masses
// the step's); no spring and no fixed-rotation test. The servo is Box3D's motor with its speed set from the angle at the
// start of the step, as b3ref2 sets it between steps. A body the joint does not solve (static, asleep) is Box3D's null
// body (solve.slang's loadBody). F snaps every value it multiplies again.

// The world joint frame's rotation and the relative rotation between them with Box3D's polarity (b3DotQuat < 0: qB
// negated, so the twist stays in [-pi, pi])
Q4 relativeRotation( Q4 qA, inout Q4 qB )
{
	T d = snap( sum3( qA.x, qB.x, qA.y, qB.y, qA.z, qB.z, S_Q ) + M( qA.s, qB.s, S_Q ) ); // b3DotQuat's order
	if ( d < zero() )
	{
		qB = q4( -qB.x, -qB.y, -qB.z, -qB.s );
	}
	Q4 r = quatMul( quatConj( qA ), qB ); // b3InvMulQuat( qA, qB ), the same operations
	return q4( snap( r.x ), snap( r.y ), snap( r.z ), snap( r.s ) );
}

// Box3D's perpendicular axes of the collinearity block: 0.5 rotate( qA, s e + v x e ) for e = x, y
V3 perpAxis( Q4 qA, Q4 rel, bool y, T half )
{
	V3 local = y ? v3( -rel.z, rel.s, rel.x ) : v3( rel.s, rel.z, -rel.y );
	return snap3( mulSV( half, rotate( qA, local ), S_MS ) );
}

// One body's share of the point block: (r x e_i) . I (r x e_j) for the lever r (Q8.24), in format ek
Sym3 leverBlock( Sym3 invI, V3 r, int eI, int ek )
{
	V3 u0 = v3( zero(), r.z, -r.y );
	V3 u1 = v3( -r.z, zero(), r.x );
	V3 u2 = v3( r.y, -r.x, zero() );
	V3 w0 = snap3( mulMV( invI, u0, S_R ) ); // format eI
	V3 w1 = snap3( mulMV( invI, u1, S_R ) );
	V3 w2 = snap3( mulMV( invI, u2, S_R ) );
	int sh = shiftOf( S_R + eI - ek );
	Sym3 k;
	k.xx = sdot( u0, w0, sh );
	k.xy = sdot( u0, w1, sh );
	k.xz = sdot( u0, w2, sh );
	k.yy = sdot( u1, w1, sh );
	k.yz = sdot( u1, w2, sh );
	k.zz = sdot( u2, w2, sh );
	return k;
}

// a . I a for an axis a (Q1.30), in format et (the body's inertia in eI)
T axisInertia( Sym3 invI, V3 a, int eI, int et ) { return sdot( a, snap3( mulMV( invI, a, S_Q ) ), shiftOf( S_Q + eI - et ) ); }

// a . I b
T axisInertia2( Sym3 invI, V3 a, V3 b, int eI, int et ) { return sdot( a, snap3( mulMV( invI, b, S_Q ) ), shiftOf( S_Q + eI - et ) ); }

// ---------------------------------------------------------------------------------------------------------------------
// prepareJoints (b3PrepareJoint: the softness is Params', b3PrepareRevoluteJoint)
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void prepareJoints( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Joint j = joints[k];
	JointCommand cmd = jointCommands[k];
	bool dynA = ( cmd.solve & PAIR_SOLVE_A ) != 0;
	bool dynB = ( cmd.solve & PAIR_SOLVE_B ) != 0;
	BodyMass mA = bodyMass[j.bodyA];
	BodyMass mB = bodyMass[j.bodyB];
	BodyPose pA = bodyPose[j.bodyA];
	BodyPose pB = bodyPose[j.bodyB];
	j.solve = cmd.solve & ( PAIR_SOLVE_A | PAIR_SOLVE_B );

	// masses: zero for a body solved as static
	j.invMassA = selectT( dynA, mA.invMass, zero() );
	j.invMassB = selectT( dynB, mB.invMass, zero() );
	j.invIA = zeroSym();
	j.invIB = zeroSym();
	if ( dynA )
	{
		j.invIA = mA.invIw;
	}
	if ( dynB )
	{
		j.invIB = mB.invIw;
	}

	// V4's exponents, as the contacts': the impulses' eP from the lighter solved body, the point block in eK
	int eMmin = dynA ? ( dynB ? ( mA.eM < mB.eM ? mA.eM : mB.eM ) : mA.eM ) : mB.eM;
	int eP = fmtV4( 51 - eMmin );
	int eK = fmtV4( eMmin - 8 );
	int ePold = j.eP;
	j.eP = eP;
	j.shMA = dynA ? shiftOf( mA.eM + eP - S_V ) : 0;
	j.shMB = dynB ? shiftOf( mB.eM + eP - S_V ) : 0;
	j.shIA = dynA ? shiftOf( mA.eI + eP - S_W ) : 0;
	j.shIB = dynB ? shiftOf( mB.eI + eP - S_W ) : 0;

	// the frames (Box3D: frame.q = q local.q, frame.p = rotate( q, local.p - localCenter ); the toy's hulls are about the
	// centre of mass), the centres' difference, the hinge axis
	j.frameAq = quatMul( pA.q, j.localFrameA );
	j.frameBq = quatMul( pB.q, j.localFrameB );
	j.frameAq = q4( snap( j.frameAq.x ), snap( j.frameAq.y ), snap( j.frameAq.z ), snap( j.frameAq.s ) );
	j.frameBq = q4( snap( j.frameBq.x ), snap( j.frameBq.y ), snap( j.frameBq.z ), snap( j.frameBq.s ) );
	j.frameAp = snap3( rotate( pA.q, j.localAnchorA ) );
	j.frameBp = snap3( rotate( pB.q, j.localAnchorB ) );
	j.deltaCenter = snap3( posDelta( pA.p, pB.p ) );
	j.axisZ = snap3( rotate( j.frameAq, v3( zero(), zero(), P.one ) ) );

	// the twist angle now, and the motor's speed: the servo's gain (target - angle) capped (Q3.28 differences fit)
	Q4 qB = j.frameBq;
	Q4 rel = relativeRotation( j.frameAq, qB );
	j.angle = twistAngle( rel );
	j.speed = j.motorSpeed;
	if ( ( j.flags & JOINT_SERVO ) != 0 )
	{
		T err = snap( cmd.target - j.angle );
		j.speed = clampT( M( j.servoGain, err, S_BR + S_JA - S_W ), -j.servoMaxSpeed, j.servoMaxSpeed );
	}
	j.maxMotorImpulse = snap( M( j.maxMotorTorque, P.h, S_TQ + S_H - eP ) );

	// the axis block's Jacobians at prepare (the warm start's; the solve recomputes them)
	j.perpAxisX = perpAxis( j.frameAq, rel, false, P.half );
	j.perpAxisY = perpAxis( j.frameAq, rel, true, P.half );

	// the angular blocks in eT, the larger inertia's format less a bit (as the contact's twist mass)
	int eIA = mA.eI;
	int eIB = mB.eI;
	int eT = dynA ? ( dynB ? ( eIA < eIB ? eIA : eIB ) : eIA ) : eIB;
	eT = fmtV4( eT - 1 );

	// the axial mass 1 / (z . (IA + IB) z)
	{
		T tA = dynA ? axisInertia( j.invIA, j.axisZ, eIA, eT ) : zero();
		T tB = dynB ? axisInertia( j.invIB, j.axisZ, eIB, eT ) : zero();
		int eAx = 0;
		j.axialMass = recipE( snap( tA + tB ), eT, eAx );
		j.shAx = shiftOf( fmtV4( eAx ) + S_W - eP );
	}

	// the 2x2 axis block [px.I px, px.I py; py.I px, py.I py] (I = IA + IB), inverted as the contact's tangent mass:
	// normalised so the larger diagonal has its top bit at 30, the adjugate over the determinant at msb(det) - 1
	{
		V3 px = j.perpAxisX;
		V3 py = j.perpAxisY;
		T kxx = zero();
		T kxy = zero();
		T kyy = zero();
		if ( dynA )
		{
			kxx = kxx + axisInertia( j.invIA, px, eIA, eT );
			kxy = kxy + axisInertia2( j.invIA, px, py, eIA, eT );
			kyy = kyy + axisInertia( j.invIA, py, eIA, eT );
		}
		if ( dynB )
		{
			kxx = kxx + axisInertia( j.invIB, px, eIB, eT );
			kxy = kxy + axisInertia2( j.invIB, px, py, eIB, eT );
			kyy = kyy + axisInertia( j.invIB, py, eIB, eT );
		}
		kxx = snap( kxx );
		kxy = snap( kxy );
		kyy = snap( kyy );
		T kmax = maxT( kxx, kyy );
		int s = kmax > zero() ? fmtV4( 30 - msbT( kmax ) ) : 0;
		s = s < 0 ? 0 : s;
		T xx = rescale( kxx, -s );
		T yy = rescale( kyy, -s );
		T xy = rescale( kxy, -s );
		T det = dif2( xx, yy, xy, xy, 31 );
		int kd = msbT( det ) - 1;
		kd = kd < 0 ? 0 : ( kd > 32 ? 32 : kd );
		j.axisMassXX = zero();
		j.axisMassXY = zero();
		j.axisMassYY = zero();
		if ( det > zero() )
		{
			j.axisMassXX = divClamp( yy, det, kd, hugeT() );
			j.axisMassYY = divClamp( xx, det, kd, hugeT() );
			j.axisMassXY = snap( -divClamp( xy, det, kd, hugeT() ) );
		}
		j.shK2 = shiftOf( fmtV4( 31 - eT - s + kd ) + S_W - eP );
	}

	// the 3x3 point block K = (mA + mB) E + sum (r x e_i) . I (r x e_j), in eK (8 bits under the larger inverse mass);
	// V4 normalises it so the largest diagonal entry is in [0.5, 1) (Q1.30) and inverts it (Q9.22): K^-1 in format
	// S_J + 30 - eK - s
	{
		T mAK = dynA ? rescale( mA.invMass, shiftOf( mA.eM - eK ) ) : zero();
		T mBK = dynB ? rescale( mB.invMass, shiftOf( mB.eM - eK ) ) : zero();
		Sym3 kA = zeroSym();
		Sym3 kB = zeroSym();
		if ( dynA )
		{
			kA = leverBlock( j.invIA, j.frameAp, eIA, eK );
		}
		if ( dynB )
		{
			kB = leverBlock( j.invIB, j.frameBp, eIB, eK );
		}
		T m = snap( mAK + mBK );
		Sym3 K;
		K.xx = snap( ( m + kA.xx ) + kB.xx );
		K.xy = snap( kA.xy + kB.xy );
		K.xz = snap( kA.xz + kB.xz );
		K.yy = snap( ( m + kA.yy ) + kB.yy );
		K.yz = snap( kA.yz + kB.yz );
		K.zz = snap( ( m + kA.zz ) + kB.zz );
		T kmax = maxT( maxT( K.xx, K.yy ), K.zz );
		int s = kmax > zero() ? fmtV4( 29 - msbT( kmax ) ) : 0;
		Sym3 A;
		A.xx = rescale( K.xx, -s );
		A.xy = rescale( K.xy, -s );
		A.xz = rescale( K.xz, -s );
		A.yy = rescale( K.yy, -s );
		A.yz = rescale( K.yz, -s );
		A.zz = rescale( K.zz, -s );
		j.pointMass = invertSym( A, P.one );
		j.shK = shiftOf( fmtV4( S_J + 30 - eK - s ) + S_V - eP );
	}

	// the carried impulses (Box3D's warm start keeps them whole), from last step's exponent to this one's. A joint
	// carried nothing when its eP is 0 (every impulse zero): no shift. rescale shifts left by at most 30.
	int shP = ePold == 0 ? 0 : ePold - eP;
	shP = shP < -30 ? -30 : shP;
	j.linearImpulse = v3( rescale( j.linearImpulse.x, shP ), rescale( j.linearImpulse.y, shP ), rescale( j.linearImpulse.z, shP ) );
	j.perpImpulseX = rescale( j.perpImpulseX, shP );
	j.perpImpulseY = rescale( j.perpImpulseY, shP );
	j.motorImpulse = rescale( j.motorImpulse, shP );
	j.lowerImpulse = rescale( j.lowerImpulse, shP );
	j.upperImpulse = rescale( j.upperImpulse, shP );
	if ( ( j.flags & JOINT_MOTOR ) == 0 )
	{
		j.motorImpulse = zero();
	}
	if ( ( j.flags & JOINT_LIMIT ) == 0 )
	{
		j.lowerImpulse = zero();
		j.upperImpulse = zero();
	}
	joints[k] = j;
}

// ---------------------------------------------------------------------------------------------------------------------
// warmStartJoints (b3WarmStartRevoluteJoint)
// ---------------------------------------------------------------------------------------------------------------------

void storeJointBodies( Joint j, bool dynA, bool dynB, BodyState bA, BodyState bB )
{
	if ( dynA )
	{
		bodyState[j.bodyA] = bA;
	}
	if ( dynB )
	{
		bodyState[j.bodyB] = bB;
	}
}

// A linear impulse P (eP) at the anchors and an angular impulse L (eP): v -= mA P, w -= IA (rA x P + L); B the other way
void applyJointImpulse( inout BodyState bA, inout BodyState bB, Joint j, bool dynA, bool dynB, V3 rA, V3 rB, V3 Pl, V3 L )
{
	if ( dynA )
	{
		V3 a = snap3( add3( scross( rA, Pl, S_R ), L ) );
		bA.w = snap3( sub3( bA.w, mulMV( j.invIA, a, j.shIA ) ) );
		bA.v = snap3( mulSubSV( bA.v, j.invMassA, Pl, j.shMA ) );
	}
	if ( dynB )
	{
		V3 b = snap3( add3( scross( rB, Pl, S_R ), L ) );
		bB.w = snap3( add3( bB.w, mulMV( j.invIB, b, j.shIB ) ) );
		bB.v = snap3( mulAddSV( bB.v, j.invMassB, Pl, j.shMB ) );
	}
}

// An angular impulse along an axis: w -= IA (d a), w += IB (d a)
void applyAxial( inout BodyState bA, inout BodyState bB, Joint j, bool dynA, bool dynB, T d, V3 a )
{
	V3 L = snap3( mulSV( d, a, S_Q ) );
	if ( dynA )
	{
		bA.w = snap3( sub3( bA.w, mulMV( j.invIA, L, j.shIA ) ) );
	}
	if ( dynB )
	{
		bB.w = snap3( add3( bB.w, mulMV( j.invIB, L, j.shIB ) ) );
	}
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void warmStartJoints( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Joint j = joints[k];
	bool dynA = ( j.solve & PAIR_SOLVE_A ) != 0;
	bool dynB = ( j.solve & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( j.bodyA, dynA, P.one );
	BodyState bB = loadBody( j.bodyB, dynB, P.one );
	V3 rA = snap3( rotate( bA.dq, j.frameAp ) );
	V3 rB = snap3( rotate( bB.dq, j.frameBp ) );
	T axial = snap( ( j.motorImpulse + j.lowerImpulse ) - j.upperImpulse );
	V3 L = snap3( mulAddSV( mulSV( j.perpImpulseX, j.perpAxisX, S_Q ), j.perpImpulseY, j.perpAxisY, S_Q ) );
	L = snap3( mulAddSV( L, axial, j.axisZ, S_Q ) );
	applyJointImpulse( bA, bB, j, dynA, dynB, rA, rB, j.linearImpulse, L );
	storeJointBodies( j, dynA, dynB, bA, bB );
}

// ---------------------------------------------------------------------------------------------------------------------
// solveJoints and relaxJoints (b3SolveRevoluteJoint with useBias true and false)
// ---------------------------------------------------------------------------------------------------------------------

// One limit row (Box3D's lower and upper limits): c the margin (Q3.28), cdot the closing speed along the axis (Q11.20);
// speculative while c > 0 (the bias c / h), soft with the bias when penetrating, rigid in the relax. Returns the
// accumulated impulse; d gets its change.
T limitRow( Joint j, Params P, bool useBias, T c, T cdot, T old, out T d )
{
	T bias = zero();
	T massScale = P.one;
	T impulseScale = zero();
	if ( c > zero() )
	{
		bias = M( c, P.inv_h, S_JA + S_IH - S_W );
	}
	else if ( useBias )
	{
		bias = M( P.jointBiasRate, c, S_BR + S_JA - S_W );
		massScale = P.jointMassScale;
		impulseScale = P.jointImpulseScale;
	}
	T x = snap( cdot + bias );
	T negImpulse = snap( M( massScale, snap( M( j.axialMass, x, j.shAx ) ), S_MS ) + M( impulseScale, old, S_MS ) );
	T nw = maxT( old - negImpulse, zero() );
	d = snap( nw - old );
	return nw;
}

void solveJoint( uint k, bool useBias )
{
	Params P = params[0];
	Joint j = joints[k];
	bool dynA = ( j.solve & PAIR_SOLVE_A ) != 0;
	bool dynB = ( j.solve & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( j.bodyA, dynA, P.one );
	BodyState bB = loadBody( j.bodyB, dynB, P.one );

	Q4 qA = quatMul( bA.dq, j.frameAq );
	Q4 qB = quatMul( bB.dq, j.frameBq );
	qA = q4( snap( qA.x ), snap( qA.y ), snap( qA.z ), snap( qA.s ) );
	qB = q4( snap( qB.x ), snap( qB.y ), snap( qB.z ), snap( qB.s ) );
	Q4 rel = relativeRotation( qA, qB );
	V3 axis = j.axisZ;

	// the motor: the speed, the accumulated impulse capped at maxMotorTorque h
	if ( ( j.flags & JOINT_MOTOR ) != 0 )
	{
		T cdot = snap( sdot( sub3( bB.w, bA.w ), axis, S_Q ) - j.speed );
		T impulse = snap( -M( j.axialMass, cdot, j.shAx ) );
		T old = j.motorImpulse;
		j.motorImpulse = clampT( old + impulse, -j.maxMotorImpulse, j.maxMotorImpulse );
		applyAxial( bA, bB, j, dynA, dynB, snap( j.motorImpulse - old ), axis );
	}

	// the limits, from the current twist angle
	if ( ( j.flags & JOINT_LIMIT ) != 0 )
	{
		T angle = twistAngle( rel );
		T d = zero();
		T cdot = sdot( sub3( bB.w, bA.w ), axis, S_Q );
		j.lowerImpulse = limitRow( j, P, useBias, snap( angle - j.lowerAngle ), cdot, j.lowerImpulse, d );
		applyAxial( bA, bB, j, dynA, dynB, d, axis );
		cdot = sdot( sub3( bA.w, bB.w ), axis, S_Q ); // the upper's sign flipped
		j.upperImpulse = limitRow( j, P, useBias, snap( j.upperAngle - angle ), cdot, j.upperImpulse, d );
		applyAxial( bA, bB, j, dynA, dynB, snap( -d ), axis );
	}

	// the axis block: the hinge axes kept aligned, C = (rel.x, rel.y), with the step's 2x2 inverse
	{
		T bx = zero();
		T by = zero();
		T massScale = P.one;
		T impulseScale = zero();
		if ( useBias )
		{
			bx = M( P.jointBiasRate, rel.x, S_BR + S_Q - S_W );
			by = M( P.jointBiasRate, rel.y, S_BR + S_Q - S_W );
			massScale = P.jointMassScale;
			impulseScale = P.jointImpulseScale;
		}
		V3 px = perpAxis( qA, rel, false, P.half );
		V3 py = perpAxis( qA, rel, true, P.half );
		j.perpAxisX = px;
		j.perpAxisY = py;
		V3 wRel = snap3( sub3( bB.w, bA.w ) );
		T cx = snap( sdot( wRel, px, S_Q ) + bx );
		T cy = snap( sdot( wRel, py, S_Q ) + by );
		T solx = snap( sum2( j.axisMassXX, cx, j.axisMassXY, cy, j.shK2 ) );
		T soly = snap( sum2( j.axisMassXY, cx, j.axisMassYY, cy, j.shK2 ) );
		T dx = snap( -( M( massScale, solx, S_MS ) + M( impulseScale, j.perpImpulseX, S_MS ) ) );
		T dy = snap( -( M( massScale, soly, S_MS ) + M( impulseScale, j.perpImpulseY, S_MS ) ) );
		j.perpImpulseX = snap( j.perpImpulseX + dx );
		j.perpImpulseY = snap( j.perpImpulseY + dy );
		V3 L = snap3( mulAddSV( mulSV( dx, px, S_Q ), dy, py, S_Q ) );
		if ( dynA )
		{
			bA.w = snap3( sub3( bA.w, mulMV( j.invIA, L, j.shIA ) ) );
		}
		if ( dynB )
		{
			bB.w = snap3( add3( bB.w, mulMV( j.invIB, L, j.shIB ) ) );
		}
	}

	// the point block: the anchors kept together, with the step's 3x3 inverse
	{
		V3 rA = snap3( rotate( bA.dq, j.frameAp ) );
		V3 rB = snap3( rotate( bB.dq, j.frameBp ) );
		V3 vrA = add3( bA.v, cross3( bA.w, rA, S_W + S_R - S_V ) );
		V3 vrB = add3( bB.v, cross3( bB.w, rB, S_W + S_R - S_V ) );
		V3 cdot = snap3( sub3( vrB, vrA ) );
		V3 bias = zero3();
		T massScale = P.one;
		T impulseScale = zero();
		if ( useBias )
		{
			// separation = (dcB - dcA) + (rB - rA) + deltaCenter, Q8.24
			V3 dc = sub3( bB.dp, bA.dp ); // Q5.26
			V3 sep = add3( add3( v3( rescale( dc.x, S_DP - S_R ), rescale( dc.y, S_DP - S_R ), rescale( dc.z, S_DP - S_R ) ), sub3( rB, rA ) ),
						   j.deltaCenter );
			bias = snap3( mulSV( P.jointBiasRate, snap3( sep ), S_BR + S_R - S_V ) );
			massScale = P.jointMassScale;
			impulseScale = P.jointImpulseScale;
		}
		V3 rhs = snap3( add3( cdot, bias ) );
		V3 sol = snap3( mulMV( j.pointMass, rhs, j.shK ) );
		V3 impulse = snap3( v3( -( M( massScale, sol.x, S_MS ) + M( impulseScale, j.linearImpulse.x, S_MS ) ),
								-( M( massScale, sol.y, S_MS ) + M( impulseScale, j.linearImpulse.y, S_MS ) ),
								-( M( massScale, sol.z, S_MS ) + M( impulseScale, j.linearImpulse.z, S_MS ) ) ) );
		j.linearImpulse = snap3( add3( j.linearImpulse, impulse ) );
		applyJointImpulse( bA, bB, j, dynA, dynB, rA, rB, impulse, zero3() );
	}

	joints[k] = j;
	storeJointBodies( j, dynA, dynB, bA, bB );
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void solveJoints( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	solveJoint( lists[pc.start + tid.x], true );
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void relaxJoints( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	solveJoint( lists[pc.start + tid.x], false );
}

// ---------------------------------------------------------------------------------------------------------------------
// hashJoints: every field of the joint, the joint's index seeding the element (category 3)
// ---------------------------------------------------------------------------------------------------------------------

H2 hashSym( H2 h, Sym3 s ) { return hashT( hashT( hashT( hashT( hashT( hashT( h, s.xx ), s.xy ), s.xz ), s.yy ), s.yz ), s.zz ); }

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void hashJoints( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Joint j = joints[k];
	H2 h = hashStart( CAT_JOINT, k );
	h = hashV3( h, j.localAnchorA );
	h = hashV3( h, j.localAnchorB );
	h = hashQ4( h, j.localFrameA );
	h = hashQ4( h, j.localFrameB );
	h = hashT( hashT( hashT( h, j.lowerAngle ), j.upperAngle ), j.maxMotorTorque );
	h = hashT( hashT( hashT( h, j.motorSpeed ), j.servoGain ), j.servoMaxSpeed );
	h = hashQ4( h, j.frameAq );
	h = hashQ4( h, j.frameBq );
	h = hashV3( h, j.frameAp );
	h = hashV3( h, j.frameBp );
	h = hashV3( h, j.deltaCenter );
	h = hashV3( h, j.axisZ );
	h = hashV3( h, j.perpAxisX );
	h = hashV3( h, j.perpAxisY );
	h = hashSym( h, j.invIA );
	h = hashSym( h, j.invIB );
	h = hashSym( h, j.pointMass );
	h = hashT( hashT( h, j.invMassA ), j.invMassB );
	h = hashT( hashT( hashT( h, j.axisMassXX ), j.axisMassXY ), j.axisMassYY );
	h = hashT( hashT( hashT( hashT( h, j.axialMass ), j.angle ), j.speed ), j.maxMotorImpulse );
	h = hashV3( h, j.linearImpulse );
	h = hashT( hashT( h, j.perpImpulseX ), j.perpImpulseY );
	h = hashT( hashT( hashT( hashT( h, j.motorImpulse ), j.lowerImpulse ), j.upperImpulse ), j.pad );
	h = hashWord( hashWord( hashWord( hashWord( h, uint( j.bodyA ) ), uint( j.bodyB ) ), uint( j.flags ) ), uint( j.solve ) );
	h = hashWord( hashWord( hashWord( hashWord( h, uint( j.eP ) ), uint( j.shMA ) ), uint( j.shMB ) ), uint( j.shIA ) );
	h = hashWord( hashWord( hashWord( hashWord( h, uint( j.shIB ) ), uint( j.shK ) ), uint( j.shK2 ) ), uint( j.shAx ) );
	jointHashes[k] = hashEnd( h );
}
