/* gcc 13.3 -O2 (aarch64 and x86-64): returns -1, should return 1. -fno-ssa-backprop, -fno-tree-vrp or -O1 give 1;
   clang gives 1. backprop sees s2 used only as s2 * s2, strips the negation in s's definition (s = PHI <0, -x>
   becomes PHI <0, x>), but leaves s2's range info ([0, +Inf] from evrp) on s2 = s * 2, whose sign has now flipped;
   VRP then intersects [-Inf, 0] with the stale [0, +Inf] and, on the threaded path x <= -2^-30, gets an empty range
   ("NaN only"), so score > best folds to false. */
#include <stdio.h>
static inline float snap(float x) { return (x < 0x1p-30f && x > -0x1p-30f) ? 0.0f : x; }
__attribute__((noinline)) int pick(const float *x, int n)
{
	int best = -1;
	float bestScore = 0.0f;
	for (int i = 0; i < n; ++i) {
		float t = -x[i];
		float s = snap(0.0f > t ? 0.0f : t);  /* max(0, -x) */
		float s2 = s + s;
		float score = snap(s2 * s2);
		if (score > bestScore) { bestScore = score; best = i; }
	}
	return best;
}
int main(void)
{
	float x[3] = { 0.01f, -0.02f, -0.01f };
	int b = pick(x, 3);
	printf("best %d (expect 1)\n", b);
	return b != 1;
}
