// solve.slang: the contact solve (DESIGN.md "Pipeline per tick" 5 and 6), included by kernels.slang after its bindings
// and narrow.slang. Box3D's soft step, convex path (contact_solver.c's b3PrepareContacts_Convex,
// b3WarmStartContacts_Convex, b3PushContacts_Convex, b3SolveContacts_Convex, b3ApplyRestitution_Convex and
// b3StoreImpulses_Convex, one lane at a time), in the three dialects:
//
//   prepareContacts per active pair (every pair the narrowphase ran on is coloured): the constraint from this tick's
//                   manifold (pointCount 0: nothing to solve), its V4 exponents and shifts; the warm-start impulses
//                   carried by narrowClip, rescaled to this tick's exponent
//   warmStart       per colour, per substep: the carried impulses applied (normal, friction at the centre, twist)
//   pushContacts    per colour, per substep: normal rows with the soft bias (speculative points: s / h when separated)
//   relaxContacts   per colour, per substep: rigid normal rows, then twist friction, then central friction with the disc
//                   clamp through the software rsqrt
//   restitution     per colour, after the substeps (Box3D's two iterations): the armed points' bounce, Poisson's limit
//   storeImpulses   per active pair: the impulses back into this tick's manifold, so the next narrowClip carries them by
//                   feature id
//
// The separation at a point follows the bodies' deltas through the fixed anchors (Box3D's anchor update, E11's
// separationAt). A body the pair does not solve (static, or asleep: Pair.flags) is Box3D's null body: zero mass, the
// identity state, never written; one of those makes the contact static-soft. F snaps every value it multiplies again,
// so no chain of products reaches a subnormal. The point loops are unrolled, so no private array is indexed by a
// variable (the Intel UHD 630's compiler misread such an index into a large private manifold: narrow.slang).

T sdot( V3 a, V3 b, int sh ) { return snap( dot3( a, b, sh ) ); }
V3 scross( V3 a, V3 b, int sh ) { return snap3( cross3( a, b, sh ) ); }

Sym3 zeroSym()
{
	Sym3 s;
	s.xx = zero();
	s.xy = zero();
	s.xz = zero();
	s.yy = zero();
	s.yz = zero();
	s.zz = zero();
	return s;
}

// A large clamp for the solver's divisions (finite times any divisor here)
T hugeT() { return KT( 1.0e20f, 1.0e20l, 2147483647 ); }

// A solved body's state, or Box3D's identity state for a body solved as static
BodyState loadBody( int index, bool dyn, T one )
{
	BodyState s;
	s.v = zero3();
	s.w = zero3();
	s.dp = zero3();
	s.dq = quatIdentity( one );
	s.sleepTicks = 0;
	s.flags = 0;
	if ( dyn )
	{
		s = bodyState[index];
	}
	return s;
}

// b3PerpW: a unit vector perpendicular to n
V3 perp( V3 n, T half )
{
	bool big = n.x < -half || n.x > half;
	V3 p = big ? v3( n.y, -n.x, zero() ) : v3( zero(), n.z, -n.y );
	return unit3( p );
}

// A body's share of an effective mass: r . (invI r) for the lever r (Q8.24), in format ek (invI's mantissa in eI)
T leverMass( Sym3 invI, V3 r, int eI, int ek )
{
	V3 u = snap3( mulMV( invI, r, S_R ) ); // format eI
	return sdot( r, u, shiftOf( S_R + eI - ek ) );
}

// The current separation at a point: base + n . (dpB - dpA) + (n_B . rB - n_A . rA), n_X the normal in X's delta frame
T separationAt( ConstraintPoint cp, T normalSeparation, V3 normalA, V3 normalB )
{
	T sb = dot3( normalB, cp.anchorB, S_Q + S_R - S_S );
	T sa = dot3( normalA, cp.anchorA, S_Q + S_R - S_S );
	return snap( ( normalSeparation + ( sb - sa ) ) + cp.baseSeparation );
}

T normalVelocity( BodyState bA, BodyState bB, V3 rA, V3 rB, V3 n )
{
	V3 vrA = add3( bA.v, cross3( bA.w, rA, S_W + S_R - S_V ) );
	V3 vrB = add3( bB.v, cross3( bB.w, rB, S_W + S_R - S_V ) );
	return sdot( sub3( vrB, vrA ), n, S_Q );
}

// An impulse P (eP) at the anchors rA, rB: Box3D's v -= mA P, w -= IA (rA x P) and the same, added, for B
void applyImpulse( inout BodyState bA, inout BodyState bB, Constraint c, bool dynA, bool dynB, V3 rA, V3 rB, V3 P )
{
	if ( dynA )
	{
		bA.w = snap3( sub3( bA.w, mulMV( c.invIA, scross( rA, P, S_R ), c.shIA ) ) );
		bA.v = snap3( mulSubSV( bA.v, c.invMassA, P, c.shMA ) );
	}
	if ( dynB )
	{
		bB.w = snap3( add3( bB.w, mulMV( c.invIB, scross( rB, P, S_R ), c.shIB ) ) );
		bB.v = snap3( mulAddSV( bB.v, c.invMassB, P, c.shMB ) );
	}
}

// An angular impulse L (eP): w -= IA L, w += IB L
void applyAngular( inout BodyState bA, inout BodyState bB, Constraint c, bool dynA, bool dynB, V3 L )
{
	if ( dynA )
	{
		bA.w = snap3( sub3( bA.w, mulMV( c.invIA, L, c.shIA ) ) );
	}
	if ( dynB )
	{
		bB.w = snap3( add3( bB.w, mulMV( c.invIB, L, c.shIB ) ) );
	}
}

void storeBodies( Constraint c, bool dynA, bool dynB, BodyState bA, BodyState bB )
{
	if ( dynA )
	{
		bodyState[c.bodyA] = bA;
	}
	if ( dynB )
	{
		bodyState[c.bodyB] = bB;
	}
}

ConstraintPoint emptyPoint()
{
	ConstraintPoint p;
	p.anchorA = zero3();
	p.anchorB = zero3();
	p.baseSeparation = zero();
	p.normalImpulse = zero();
	p.totalNormalImpulse = zero();
	p.restitutionImpulse = zero();
	p.normalMass = zero();
	p.leverArm = zero();
	p.relativeVelocity = zero();
	p.pad = zero();
	p.shN = 0;
	p.pad1 = 0;
	return p;
}

Constraint emptyConstraint( Pair pr )
{
	Constraint c;
	[ForceUnroll]
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		c.points[i] = emptyPoint();
	}
	c.normal = zero3();
	c.tangent1 = zero3();
	c.tangent2 = zero3();
	c.centerA = zero3();
	c.centerB = zero3();
	c.invIA = zeroSym();
	c.invIB = zeroSym();
	c.invMassA = zero();
	c.invMassB = zero();
	c.tangentMassXX = zero();
	c.tangentMassXY = zero();
	c.tangentMassYY = zero();
	c.twistMass = zero();
	c.frictionImpulse1 = zero();
	c.frictionImpulse2 = zero();
	c.twistImpulse = zero();
	c.friction = zero();
	c.restitution = zero();
	c.pad = zero();
	c.bodyA = pr.bodyA;
	c.bodyB = pr.bodyB;
	c.pointCount = 0;
	c.flags = pr.flags & ( PAIR_SOLVE_A | PAIR_SOLVE_B );
	c.eP = 0;
	c.shMA = 0;
	c.shMB = 0;
	c.shIA = 0;
	c.shIB = 0;
	c.shT = 0;
	c.shTw = 0;
	c.pad1 = 0;
	return c;
}

// ---------------------------------------------------------------------------------------------------------------------
// prepareContacts (b3PrepareContacts_Convex)
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void prepareContacts( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Pair pr = pairs[k];
	Manifold m = manifolds[k];
	Constraint c = emptyConstraint( pr );
	bool dynA = ( pr.flags & PAIR_SOLVE_A ) != 0;
	bool dynB = ( pr.flags & PAIR_SOLVE_B ) != 0;
	if ( m.pointCount > 0 && ( dynA || dynB ) )
	{
		BodyMass mA = bodyMass[pr.bodyA];
		BodyMass mB = bodyMass[pr.bodyB];
		c.pointCount = m.pointCount;
		c.flags = c.flags | ( dynA && dynB ? 0 : CON_SOFT );

		// the material: Box3D's default mixing (friction sqrt( fA fB ), restitution the larger)
		c.friction = sqrtT( M( mA.friction, mB.friction, S_MS ), S_MS );
		c.restitution = maxT( mA.restitution, mB.restitution );

		// masses: zero for a body solved as static
		c.invMassA = selectT( dynA, mA.invMass, zero() );
		c.invMassB = selectT( dynB, mB.invMass, zero() );
		c.invIA = zeroSym();
		c.invIB = zeroSym();
		if ( dynA )
		{
			c.invIA = mA.invIw;
		}
		if ( dynB )
		{
			c.invIB = mB.invIw;
		}

		// V4's exponents: the impulses' eP from the lighter solved body (the smaller eM), the effective-mass sums in
		// eK, 8 bits under the larger inverse mass (headroom for the angular terms)
		int eMmin = dynA ? ( dynB ? ( mA.eM < mB.eM ? mA.eM : mB.eM ) : mA.eM ) : mB.eM;
		int eP = fmtV4( 51 - eMmin );
		int eK = fmtV4( eMmin - 8 );
		int ePold = m.eP;
		c.eP = eP;
		c.shMA = dynA ? shiftOf( mA.eM + eP - S_V ) : 0;
		c.shMB = dynB ? shiftOf( mB.eM + eP - S_V ) : 0;
		c.shIA = dynA ? shiftOf( mA.eI + eP - S_W ) : 0;
		c.shIB = dynB ? shiftOf( mB.eI + eP - S_W ) : 0;
		T mAK = dynA ? rescale( mA.invMass, shiftOf( mA.eM - eK ) ) : zero(); // the inverse masses in eK
		T mBK = dynB ? rescale( mB.invMass, shiftOf( mB.eM - eK ) ) : zero();

		// the normal and the tangents
		V3 n = m.normal;
		c.normal = n;
		c.tangent1 = perp( n, P.half );
		c.tangent2 = scross( c.tangent1, n, S_Q );

		// the warm-start impulses (Box3D's warmStartScale 1), from last tick's exponent to this tick's. A manifold carried
		// nothing when its eP is 0 (its impulses are zero): no shift. rescale shifts left by at most 30.
		int shP = ePold == 0 ? 0 : ePold - eP;
		shP = shP < -30 ? -30 : shP;
		c.frictionImpulse1 = rescale( sdot( m.frictionImpulse, c.tangent1, S_Q ), shP );
		c.frictionImpulse2 = rescale( sdot( m.frictionImpulse, c.tangent2, S_Q ), shP );
		c.twistImpulse = rescale( m.twistImpulse, shP );

		BodyState bA = loadBody( pr.bodyA, dynA, P.one );
		BodyState bB = loadBody( pr.bodyB, dynB, P.one );
		bool sample = c.restitution > zero(); // Box3D samples the normal velocity only for restitution

		// the points; the friction centre's weights clamp( 2 - s / speculativeDistance, minWeight, 1 ), Q9.22
		T weight[MANIFOLD_POINTS];
		T wmax = zero();
		[ForceUnroll]
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			weight[i] = zero();
			if ( i < m.pointCount )
			{
				ManifoldPoint mp = m.points[i];
				ConstraintPoint cp = emptyPoint();
				V3 rA = mp.anchorA;
				V3 rB = mp.anchorB;
				cp.anchorA = rA;
				cp.anchorB = rB;
				T s = mp.separation;
				cp.baseSeparation = snap( s - sdot( sub3( rB, rA ), n, S_Q + S_R - S_S ) );
				cp.normalImpulse = rescale( mp.normalImpulse, shP );

				T x = M( s, P.invSpeculative, S_BR );
				T w = clampT( P.twoV - x, P.minFrictionWeight, P.oneV );
				weight[i] = w;
				wmax = maxT( wmax, w );

				// kNormal = mA + mB + rnA . IA rnA + rnB . IB rnB (format eK), normalMass = 1 / kNormal
				V3 rnA = scross( rA, n, S_Q );
				V3 rnB = scross( rB, n, S_Q );
				T kA = dynA ? leverMass( c.invIA, rnA, mA.eI, eK ) : zero();
				T kB = dynB ? leverMass( c.invIB, rnB, mB.eI, eK ) : zero();
				T kNormal = snap( ( ( mAK + mBK ) + kA ) + kB );
				int eN = 0;
				cp.normalMass = recipE( kNormal, eK, eN );
				cp.shN = shiftOf( fmtV4( eN ) + S_V - eP );

				if ( sample )
				{
					cp.relativeVelocity = normalVelocity( bA, bB, rA, rB, n );
				}
				c.points[i] = cp;
			}
		}

		// the friction centre: the anchors' mean weighted by w / max w (the same centre as Box3D's, with weights that
		// keep V4's bits; the weights are 1 whenever s < speculativeDistance, as every manifold point's is)
		V3 sumA = zero3();
		V3 sumB = zero3();
		T sumW = zero();
		[ForceUnroll]
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			if ( i < m.pointCount )
			{
				T wn = divClamp( weight[i], wmax, S_MS, P.one ); // Q1.30, 1 for the largest
				sumA = mulAddSV( sumA, wn, c.points[i].anchorA, S_MS );
				sumB = mulAddSV( sumB, wn, c.points[i].anchorB, S_MS );
				sumW = sumW + rescale( wn, S_MS - S_S );
			}
		}
		sumA = snap3( sumA );
		sumB = snap3( sumB );
		c.centerA = v3( divClamp( sumA.x, sumW, S_S, hugeT() ), divClamp( sumA.y, sumW, S_S, hugeT() ), divClamp( sumA.z, sumW, S_S, hugeT() ) );
		c.centerB = v3( divClamp( sumB.x, sumW, S_S, hugeT() ), divClamp( sumB.y, sumW, S_S, hugeT() ), divClamp( sumB.z, sumW, S_S, hugeT() ) );
		[ForceUnroll]
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			if ( i < m.pointCount )
			{
				c.points[i].leverArm = len3( snap3( sub3( c.points[i].anchorA, c.centerA ) ) );
			}
		}

		// the tangent mass: K = [kxx kxy; kxy kyy] at the centres (format eK), inverted. V4 shifts the three together
		// so the larger diagonal entry has its top bit at 30 (kxy^2 <= kxx kyy: it is never larger), so the determinant
		// keeps up to 30 bits; the adjugate over the determinant then goes to a format where every entry of the inverse
		// (at most kmax / det) stays below 2^30: the division's shift is msb(det) - 1.
		{
			V3 rtA1 = scross( c.centerA, c.tangent1, S_Q );
			V3 rtA2 = scross( c.centerA, c.tangent2, S_Q );
			V3 rtB1 = scross( c.centerB, c.tangent1, S_Q );
			V3 rtB2 = scross( c.centerB, c.tangent2, S_Q );
			T kxx = mAK + mBK;
			T kyy = mAK + mBK;
			T kxy = zero();
			if ( dynA )
			{
				V3 u1 = snap3( mulMV( c.invIA, rtA1, S_R ) );
				V3 u2 = snap3( mulMV( c.invIA, rtA2, S_R ) );
				int sh = shiftOf( S_R + mA.eI - eK );
				kxx = kxx + sdot( rtA1, u1, sh );
				kyy = kyy + sdot( rtA2, u2, sh );
				kxy = kxy + sdot( rtA1, u2, sh );
			}
			if ( dynB )
			{
				V3 u1 = snap3( mulMV( c.invIB, rtB1, S_R ) );
				V3 u2 = snap3( mulMV( c.invIB, rtB2, S_R ) );
				int sh = shiftOf( S_R + mB.eI - eK );
				kxx = kxx + sdot( rtB1, u1, sh );
				kyy = kyy + sdot( rtB2, u2, sh );
				kxy = kxy + sdot( rtB1, u2, sh );
			}
			kxx = snap( kxx );
			kyy = snap( kyy );
			kxy = snap( kxy );
			T kmax = maxT( kxx, kyy );
			int s = kmax > zero() ? fmtV4( 30 - msbT( kmax ) ) : 0;
			s = s < 0 ? 0 : s;
			T xx = rescale( kxx, -s );
			T yy = rescale( kyy, -s );
			T xy = rescale( kxy, -s );
			T det = dif2( xx, yy, xy, xy, 31 ); // format 2 (eK + s) - 31 (F: not snapped, a heavy pair's is small)
			int kd = msbT( det ) - 1;
			kd = kd < 0 ? 0 : ( kd > 32 ? 32 : kd );
			if ( det > zero() )
			{
				c.tangentMassXX = divClamp( yy, det, kd, hugeT() );
				c.tangentMassYY = divClamp( xx, det, kd, hugeT() );
				c.tangentMassXY = snap( -divClamp( xy, det, kd, hugeT() ) );
			}
			// adj(K) / det: the format (eK + s) - (2 (eK + s) - 31) + kd
			c.shT = shiftOf( fmtV4( 31 - eK - s + kd ) + S_V - eP );
		}

		// the twist mass 1 / (n . (IA + IB) n): each body's share in its own format, then together in the larger
		// inertia's format less a bit
		{
			int eIA = mA.eI;
			int eIB = mB.eI;
			int eT = dynA ? ( dynB ? ( eIA < eIB ? eIA : eIB ) : eIA ) : eIB;
			eT = eT - 1;
			T tA = zero();
			T tB = zero();
			if ( dynA )
			{
				tA = rescale( sdot( n, snap3( mulMV( c.invIA, n, S_Q ) ), S_Q ), shiftOf( fmtV4( eIA - eT ) ) );
			}
			if ( dynB )
			{
				tB = rescale( sdot( n, snap3( mulMV( c.invIB, n, S_Q ) ), S_Q ), shiftOf( fmtV4( eIB - eT ) ) );
			}
			int eTw = 0;
			c.twistMass = recipE( snap( tA + tB ), fmtV4( eT ), eTw );
			c.shTw = shiftOf( fmtV4( eTw ) + S_W - eP );
		}
	}
	constraints[k] = c;
}

// ---------------------------------------------------------------------------------------------------------------------
// warmStart (b3WarmStartContacts_Convex)
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void warmStart( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Constraint c = constraints[k];
	if ( c.pointCount == 0 )
		return;
	bool dynA = ( c.flags & PAIR_SOLVE_A ) != 0;
	bool dynB = ( c.flags & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( c.bodyA, dynA, P.one );
	BodyState bB = loadBody( c.bodyB, dynB, P.one );

	T totalNormalImpulse = zero();
	V3 momentA = zero3();
	V3 momentB = zero3();
	[ForceUnroll]
	for ( int j = 0; j < MANIFOLD_POINTS; ++j )
	{
		if ( j < c.pointCount )
		{
			ConstraintPoint cp = c.points[j];
			totalNormalImpulse = totalNormalImpulse + cp.normalImpulse;
			momentA = mulAddSV( momentA, cp.normalImpulse, cp.anchorA, S_R );
			momentB = mulAddSV( momentB, cp.normalImpulse, cp.anchorB, S_R );
		}
	}
	totalNormalImpulse = snap( totalNormalImpulse );
	momentA = snap3( momentA );
	momentB = snap3( momentB );

	V3 n = c.normal;
	V3 frictionImpulse = snap3( mulAddSV( mulSV( c.frictionImpulse1, c.tangent1, S_Q ), c.frictionImpulse2, c.tangent2, S_Q ) );
	V3 linearImpulse = snap3( mulAddSV( frictionImpulse, totalNormalImpulse, n, S_Q ) );
	V3 twistImpulse = mulSV( c.twistImpulse, n, S_Q );
	V3 angularA = snap3( add3( add3( cross3( momentA, n, S_Q ), cross3( c.centerA, frictionImpulse, S_R ) ), twistImpulse ) );
	V3 angularB = snap3( add3( add3( cross3( momentB, n, S_Q ), cross3( c.centerB, frictionImpulse, S_R ) ), twistImpulse ) );

	if ( dynA )
	{
		bA.w = snap3( sub3( bA.w, mulMV( c.invIA, angularA, c.shIA ) ) );
		bA.v = snap3( mulSubSV( bA.v, c.invMassA, linearImpulse, c.shMA ) );
	}
	if ( dynB )
	{
		bB.w = snap3( add3( bB.w, mulMV( c.invIB, angularB, c.shIB ) ) );
		bB.v = snap3( mulAddSV( bB.v, c.invMassB, linearImpulse, c.shMB ) );
	}
	storeBodies( c, dynA, dynB, bA, bB );
}

// ---------------------------------------------------------------------------------------------------------------------
// pushContacts (b3PushContacts_Convex): the normal rows with the soft bias, no friction
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void pushContacts( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Constraint c = constraints[k];
	if ( c.pointCount == 0 )
		return;
	bool dynA = ( c.flags & PAIR_SOLVE_A ) != 0;
	bool dynB = ( c.flags & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( c.bodyA, dynA, P.one );
	BodyState bB = loadBody( c.bodyB, dynB, P.one );

	bool soft = ( c.flags & CON_SOFT ) != 0;
	T biasRate = selectT( soft, P.staBiasRate, P.dynBiasRate );
	T massScale = selectT( soft, P.staMassScale, P.dynMassScale );
	T impulseScale = selectT( soft, P.staImpulseScale, P.dynImpulseScale );

	V3 n = c.normal;
	V3 dp = sub3( bB.dp, bA.dp );
	T normalSeparation = sdot( n, dp, S_Q + S_DP - S_S );
	V3 normalA = snap3( invRotate( bA.dq, n ) );
	V3 normalB = snap3( invRotate( bB.dq, n ) );

	[ForceUnroll]
	for ( int j = 0; j < MANIFOLD_POINTS; ++j )
	{
		if ( j < c.pointCount )
		{
			ConstraintPoint cp = c.points[j];
			V3 rA = cp.anchorA;
			V3 rB = cp.anchorB;
			T s = separationAt( cp, normalSeparation, normalA, normalB );
			bool separated = s > zero();
			T specBias = M( s, P.inv_h, S_S + S_IH - S_V );
			T overlapBias = maxT( M( biasRate, s, S_BR + S_S - S_V ), P.negContactSpeed );
			T velocityBias = selectT( separated, specBias, overlapBias );
			T pointMassScale = selectT( separated, P.one, massScale );
			T pointImpulseScale = selectT( separated, zero(), impulseScale );

			T vn = normalVelocity( bA, bB, rA, rB, n );
			T x = snap( M( pointMassScale, vn, S_MS ) + velocityBias );
			T negImpulse = snap( M( cp.normalMass, x, cp.shN ) + M( pointImpulseScale, cp.normalImpulse, S_MS ) );
			T newImpulse = maxT( cp.normalImpulse - negImpulse, zero() );
			T deltaImpulse = snap( newImpulse - cp.normalImpulse );
			c.points[j].normalImpulse = newImpulse;
			applyImpulse( bA, bB, c, dynA, dynB, rA, rB, snap3( mulSV( deltaImpulse, n, S_Q ) ) );
		}
	}
	constraints[k] = c;
	storeBodies( c, dynA, dynB, bA, bB );
}

// ---------------------------------------------------------------------------------------------------------------------
// relaxContacts (b3SolveContacts_Convex): rigid normal rows, twist friction, central friction
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void relaxContacts( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Constraint c = constraints[k];
	if ( c.pointCount == 0 )
		return;
	bool dynA = ( c.flags & PAIR_SOLVE_A ) != 0;
	bool dynB = ( c.flags & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( c.bodyA, dynA, P.one );
	BodyState bB = loadBody( c.bodyB, dynB, P.one );

	V3 n = c.normal;
	V3 dp = sub3( bB.dp, bA.dp );
	T normalSeparation = sdot( n, dp, S_Q + S_DP - S_S );
	V3 normalA = snap3( invRotate( bA.dq, n ) );
	V3 normalB = snap3( invRotate( bB.dq, n ) );

	T totalNormalImpulse = zero();
	T totalTwistLimit = zero();
	[ForceUnroll]
	for ( int j = 0; j < MANIFOLD_POINTS; ++j )
	{
		if ( j < c.pointCount )
		{
			ConstraintPoint cp = c.points[j];
			V3 rA = cp.anchorA;
			V3 rB = cp.anchorB;
			T s = separationAt( cp, normalSeparation, normalA, normalB );
			T velocityBias = maxT( zero(), M( s, P.inv_h, S_S + S_IH - S_V ) );
			T vn = normalVelocity( bA, bB, rA, rB, n );
			T negImpulse = snap( M( cp.normalMass, snap( vn + velocityBias ), cp.shN ) );
			T newImpulse = maxT( cp.normalImpulse - negImpulse, zero() );
			T deltaImpulse = snap( newImpulse - cp.normalImpulse );
			c.points[j].normalImpulse = newImpulse;
			c.points[j].totalNormalImpulse = snap( cp.totalNormalImpulse + newImpulse );
			totalNormalImpulse = snap( totalNormalImpulse + newImpulse );
			totalTwistLimit = snap( totalTwistLimit + M( cp.leverArm, newImpulse, S_R ) );
			applyImpulse( bA, bB, c, dynA, dynB, rA, rB, snap3( mulSV( deltaImpulse, n, S_Q ) ) );
		}
	}

	// central twist friction
	{
		T twistSpeed = sdot( n, sub3( bB.w, bA.w ), S_Q );
		T maxLambda = snap( M( c.friction, totalTwistLimit, S_MS ) );
		T deltaImpulse = snap( -M( c.twistMass, twistSpeed, c.shTw ) );
		T oldImpulse = c.twistImpulse;
		c.twistImpulse = clampT( oldImpulse + deltaImpulse, -maxLambda, maxLambda );
		deltaImpulse = snap( c.twistImpulse - oldImpulse );
		applyAngular( bA, bB, c, dynA, dynB, snap3( mulSV( deltaImpulse, n, S_Q ) ) );
	}

	// central friction at the friction centre: the 2x2 tangent mass, the disc clamp
	{
		V3 rA = c.centerA;
		V3 rB = c.centerB;
		V3 vrA = add3( bA.v, cross3( bA.w, rA, S_W + S_R - S_V ) );
		V3 vrB = add3( bB.v, cross3( bB.w, rB, S_W + S_R - S_V ) );
		V3 vr = snap3( sub3( vrB, vrA ) );
		T vt1 = sdot( vr, c.tangent1, S_Q );
		T vt2 = sdot( vr, c.tangent2, S_Q );
		T d1 = sum2( c.tangentMassXX, vt1, c.tangentMassXY, vt2, c.shT );
		T d2 = sum2( c.tangentMassXY, vt1, c.tangentMassYY, vt2, c.shT );
		T new1 = snap( c.frictionImpulse1 + snap( -d1 ) );
		T new2 = snap( c.frictionImpulse2 + snap( -d2 ) );
		T maxImpulse = snap( M( c.friction, totalNormalImpulse, S_MS ) );
		T scale = discScale( new1, new2, maxImpulse, P.one );
		new1 = snap( M( scale, new1, S_MS ) );
		new2 = snap( M( scale, new2, S_MS ) );
		T dd1 = snap( new1 - c.frictionImpulse1 );
		T dd2 = snap( new2 - c.frictionImpulse2 );
		c.frictionImpulse1 = new1;
		c.frictionImpulse2 = new2;
		V3 Pf = snap3( add3( mulSV( dd1, c.tangent1, S_Q ), mulSV( dd2, c.tangent2, S_Q ) ) );
		applyImpulse( bA, bB, c, dynA, dynB, rA, rB, Pf );
	}

	constraints[k] = c;
	storeBodies( c, dynA, dynB, bA, bB );
}

// ---------------------------------------------------------------------------------------------------------------------
// restitution (b3ApplyRestitution_Convex): after the substeps, Params.restitutionIterations times. A point is armed when
// its approach (the normal velocity at prepare) was faster than the threshold and it carried a compression impulse; its
// bias is then restitution x that velocity, and Poisson's limit caps the bounce at restitution x the compression
// impulse (no energy gain). Others keep the speculative bias, as the relax pass.
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void restitution( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Params P = params[0];
	Constraint c = constraints[k];
	if ( c.pointCount == 0 || !( c.restitution > zero() ) ) // Box3D without restitution propagation
		return;
	bool dynA = ( c.flags & PAIR_SOLVE_A ) != 0;
	bool dynB = ( c.flags & PAIR_SOLVE_B ) != 0;
	BodyState bA = loadBody( c.bodyA, dynA, P.one );
	BodyState bB = loadBody( c.bodyB, dynB, P.one );

	V3 n = c.normal;
	V3 dp = sub3( bB.dp, bA.dp );
	T normalSeparation = sdot( n, dp, S_Q + S_DP - S_S );
	V3 normalA = snap3( invRotate( bA.dq, n ) );
	V3 normalB = snap3( invRotate( bB.dq, n ) );

	[ForceUnroll]
	for ( int j = 0; j < MANIFOLD_POINTS; ++j )
	{
		if ( j < c.pointCount )
		{
			ConstraintPoint cp = c.points[j];
			V3 rA = cp.anchorA;
			V3 rB = cp.anchorB;
			T compressionImpulse = snap( cp.totalNormalImpulse - cp.restitutionImpulse );
			bool armed = cp.relativeVelocity < P.negRestitutionThreshold && compressionImpulse > zero();

			T s = separationAt( cp, normalSeparation, normalA, normalB );
			T specBias = maxT( zero(), M( s, P.inv_h, S_S + S_IH - S_V ) );
			T velocityBias = selectT( armed, M( c.restitution, cp.relativeVelocity, S_MS ), specBias );

			T vn = normalVelocity( bA, bB, rA, rB, n );
			T negImpulse = snap( M( cp.normalMass, snap( vn + velocityBias ), cp.shN ) );
			T newImpulse = maxT( cp.normalImpulse - negImpulse, zero() );
			T impulse = snap( newImpulse - cp.normalImpulse );

			T approachImpulse = minT( maxT( snap( -M( cp.normalMass, vn, cp.shN ) ), zero() ), maxT( impulse, zero() ) );
			T allowance = snap( M( c.restitution, snap( compressionImpulse + approachImpulse ), S_MS ) - cp.restitutionImpulse );
			T maxImpulse = snap( approachImpulse + maxT( allowance, zero() ) );
			impulse = selectT( armed, minT( impulse, maxImpulse ), impulse );

			c.points[j].normalImpulse = snap( cp.normalImpulse + impulse );
			c.points[j].restitutionImpulse = snap( cp.restitutionImpulse + snap( impulse - approachImpulse ) );
			c.points[j].totalNormalImpulse = snap( cp.totalNormalImpulse + impulse );
			applyImpulse( bA, bB, c, dynA, dynB, rA, rB, snap3( mulSV( impulse, n, S_Q ) ) );
		}
	}
	constraints[k] = c;
	storeBodies( c, dynA, dynB, bA, bB );
}

// ---------------------------------------------------------------------------------------------------------------------
// storeImpulses (b3StoreImpulses_Convex): into this tick's manifold, read by the next tick's narrowClip
// ---------------------------------------------------------------------------------------------------------------------

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void storeImpulses( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint k = lists[pc.start + tid.x];
	Constraint c = constraints[k];
	if ( c.pointCount == 0 )
		return;
	Manifold m = manifolds[k];
	// the friction impulse as a world vector (Box3D: f1 t1 + f2 t2 per component), so a new tangent basis reads it
	m.frictionImpulse = v3( sum2( c.frictionImpulse1, c.tangent1.x, c.frictionImpulse2, c.tangent2.x, S_Q ),
							sum2( c.frictionImpulse1, c.tangent1.y, c.frictionImpulse2, c.tangent2.y, S_Q ),
							sum2( c.frictionImpulse1, c.tangent1.z, c.frictionImpulse2, c.tangent2.z, S_Q ) );
	m.frictionImpulse = snap3( m.frictionImpulse );
	m.twistImpulse = c.twistImpulse;
	m.eP = c.eP;
	[ForceUnroll]
	for ( int i = 0; i < MANIFOLD_POINTS; ++i )
	{
		if ( i < c.pointCount )
		{
			ConstraintPoint cp = c.points[i];
			m.points[i].normalImpulse = cp.normalImpulse;
			m.points[i].totalNormalImpulse = cp.totalNormalImpulse;
			m.points[i].normalMass = cp.normalMass;
			m.points[i].shN = cp.shN;
		}
	}
	manifolds[k] = m;
}
