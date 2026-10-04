#!/usr/bin/env python3
"""traj.py: read the toy's trajectory files (traj.h's LPTRAJ1, written by the toy and by b3ref2), and the quaternion
helpers the measurements share. The measurements themselves are metrics.py's (each computed once there).

  python lab/gpu/toy/traj.py FILE [--body I] [--every N] [--scene S]

Without options: a summary per body (start, end, the largest speed). --body I prints one body's track (every Nth
record). --scene S prints metrics.py's one-line summary of scene S (stack10, pile200, bounce, ramp, ratio, chip).
"""
import argparse
import math
import struct
import sys

BODY = struct.Struct("<13d2I")


class Traj:
    def __init__(self, path):
        with open(path, "rb") as f:
            data = f.read()
        if data[:7] != b"LPTRAJ1":
            raise SystemExit(f"{path}: not an LPTRAJ1 file")
        self.bodies, self.records, self.every, _, self.dt = struct.unpack_from("<4Id", data, 8)
        self.ticks = []
        self.rec = []  # per record, per body: (p, q, v, w, awake, flags)
        o = 32
        for _ in range(self.records):
            tick, _ = struct.unpack_from("<2I", data, o)
            o += 8
            row = []
            for _ in range(self.bodies):
                x = BODY.unpack_from(data, o)
                o += BODY.size
                row.append((x[0:3], x[3:7], x[7:10], x[10:13], x[13], x[14]))
            self.ticks.append(tick)
            self.rec.append(row)

    def track(self, i):
        return [r[i] for r in self.rec]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def qmul(a, b):  # (x, y, z, w)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rel_angles(q0, q1):
    """The rotation from q0 to q1 (world frame): its yaw about y and its tilt (the angle of the body's y axis), rad."""
    d = qmul(q1, qconj(q0))
    if d[3] < 0:
        d = tuple(-x for x in d)
    yaw = 2.0 * math.atan2(d[1], d[3])
    x, y, z, w = d
    up = (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w))  # d applied to (0, 1, 0)
    tilt = math.acos(max(-1.0, min(1.0, up[1])))
    return yaw, tilt


def dynamic(t):
    return [i for i in range(t.bodies) if not (t.rec[0][i][5] & 1)]


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("file")
    a.add_argument("--scene")
    a.add_argument("--body", type=int)
    a.add_argument("--every", type=int, default=1)
    a = a.parse_args()
    t = Traj(a.file)
    if a.body is not None:
        for k in range(0, len(t.ticks), a.every):
            p, q, v, w, aw, fl = t.rec[k][a.body]
            print(f"{t.ticks[k]:5d}  p {p[0]: .6f} {p[1]: .6f} {p[2]: .6f}  q {q[0]: .5f} {q[1]: .5f} {q[2]: .5f} {q[3]: .5f}  "
                  f"v {v[0]: .4f} {v[1]: .4f} {v[2]: .4f}  w {w[0]: .3f} {w[1]: .3f} {w[2]: .3f}  {'awake' if aw else 'asleep'}")
        return
    if a.scene:
        import metrics  # noqa: E402  (metrics imports this module)
        print(f"{a.scene}: {metrics.summary(a.scene, metrics.FNS[a.scene](t))}")
        return
    print(f"{a.file}: {t.bodies} bodies, {t.records} records every {t.every} ticks, dt {t.dt}")
    for i in range(t.bodies):
        s = t.track(i)
        vmax = max(norm(r[2]) for r in s)
        print(f"  body {i}{' static' if s[0][5] & 1 else ''}: {tuple(round(x, 4) for x in s[0][0])} -> {tuple(round(x, 4) for x in s[-1][0])}, "
              f"largest speed {vmax:.3g} m/s, {'awake' if s[-1][4] else 'asleep'} at the end")


if __name__ == "__main__":
    sys.exit(main())
