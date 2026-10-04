#!/usr/bin/env python3
"""traj.py: read the toy's trajectory files (traj.h's LPTRAJ1) and print a scene's sanity numbers (NOTES.md, step 4).

  python lab/gpu/toy/traj.py FILE [--scene stack10|pile200|bounce|ramp|ratio|chip] [--body I] [--every N]

Without --scene: a summary per body (start, end, the largest speed). --body I prints one body's track (every Nth
record). The scene summaries: stack10, the top cube's drift, sink, yaw and tilt and its resting speed over the last
second; pile200, when every body sleeps and how many left the pit; bounce, the first apex against the analytic one;
ramp, each box's travel along the slope and the sliding one's acceleration against g (sin - mu cos); ratio, the boxes'
and the plate's sink; chip, its lowest point, when it stops and sleeps.
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


def asleep_from(t, bodies):
    """The first tick from which every listed body is asleep to the end (None: never)."""
    first = None
    for k in range(len(t.ticks) - 1, -1, -1):
        if all(t.rec[k][i][4] == 0 for i in bodies):
            first = t.ticks[k]
        else:
            break
    return first


def dynamic(t):
    return [i for i in range(t.bodies) if not (t.rec[0][i][5] & 1)]


def last_second(t):
    n = max(1, int(round(1.0 / (t.dt * t.every))))
    return t.rec[-n:]


def stack10(t):
    top = t.bodies - 1
    s = t.track(top)
    p0, q0 = s[0][0], s[0][1]
    p1, q1 = s[-1][0], s[-1][1]
    drift = math.hypot(p1[0] - p0[0], p1[2] - p0[2])
    yaw, tilt = rel_angles(q0, q1)
    rest = max(norm(r[top][2]) for r in last_second(t))
    restw = max(norm(r[top][3]) for r in last_second(t))
    sl = asleep_from(t, dynamic(t))
    print(f"stack10: top cube (body {top}) after tick {t.ticks[-1]}: drift {drift * 1e3:.4g} mm, sink {(p0[1] - p1[1]) * 1e3:.4g} mm, "
          f"yaw {math.degrees(yaw):.4g} deg, tilt {math.degrees(tilt):.4g} deg; last second's largest speed {rest * 1e3:.4g} mm/s, "
          f"spin {restw:.3g} rad/s; asleep {('from tick ' + str(sl)) if sl is not None else 'never'}")


def pile200(t):
    dyn = dynamic(t)
    sl = asleep_from(t, dyn)
    awake = sum(1 for i in dyn if t.rec[-1][i][4])
    out = [i for i in dyn if abs(t.rec[-1][i][0][0]) > 2.0 or abs(t.rec[-1][i][0][2]) > 2.0 or t.rec[-1][i][0][1] < 0.0]
    top = max(t.rec[-1][i][0][1] for i in dyn)
    print(f"pile200: every body asleep {('from tick ' + str(sl)) if sl is not None else 'never'} ({awake} awake at tick {t.ticks[-1]}); "
          f"escapes {len(out)}{' ' + str(out[:8]) if out else ''}; highest centre {top:.3f} m")


def bounce(t):
    s = t.track(1)
    ys = [r[0][1] for r in s]
    k = min(range(len(ys)), key=lambda i: ys[i])  # the impact
    apex = max(ys[k:]) if k < len(ys) else ys[-1]
    at = t.ticks[k + ys[k:].index(apex)]
    want = 0.25 + 0.25 * 1.75
    vmin = min(r[2][1] for r in s)
    print(f"bounce: impact near tick {t.ticks[k]} (lowest centre {ys[k]:.4f} m, fastest fall {vmin:.4f} m/s; analytic -5.916), first apex "
          f"{apex:.4f} m at tick {at} (analytic 0.6875 m: {100.0 * (apex - 0.25) / (want - 0.25):.1f}% of the rise)")


def ramp(t):
    th = math.radians(25.0)
    down = (-math.cos(th), -math.sin(th), 0.0)
    out = []
    for i, f in ((2, 0.6), (3, 0.2)):
        s = t.track(i)
        d = [sum((r[0][j] - s[0][0][j]) * down[j] for j in range(3)) for r in s]
        # the acceleration down the slope: a least-squares parabola over ticks 30-150 (on the ramp, after settling)
        pts = [(t.ticks[k] * t.dt, d[k]) for k in range(len(d)) if 30 <= t.ticks[k] <= 150]
        acc = fit_accel(pts) if len(pts) > 5 else float("nan")
        out.append(f"friction {f} (body {i}): travel {d[-1]:.4g} m by tick {t.ticks[-1]}, at tick 150 {d[min(len(d) - 1, 150 // t.every)]:.4g} m, "
                   f"acceleration (ticks 30-150) {acc:.4g} m/s^2")
    mu = math.sqrt(0.12)
    print("ramp: " + "; ".join(out) + f" (sliding analytic {10.0 * (math.sin(th) - mu * math.cos(th)):.4g}, holding 0)")


def fit_accel(pts):
    # d = a + b t + c t^2, acceleration 2c
    n = len(pts)
    S = [[0.0] * 3 for _ in range(3)]
    r = [0.0] * 3
    for x, y in pts:
        b = (1.0, x, x * x)
        for i in range(3):
            r[i] += b[i] * y
            for j in range(3):
                S[i][j] += b[i] * b[j]
    # Gaussian elimination
    for i in range(3):
        p = S[i][i]
        for j in range(i + 1, 3):
            f = S[j][i] / p
            for k in range(3):
                S[j][k] -= f * S[i][k]
            r[j] -= f * r[i]
    c = [0.0] * 3
    for i in (2, 1, 0):
        c[i] = (r[i] - sum(S[i][k] * c[k] for k in range(i + 1, 3))) / S[i][i]
    return 2.0 * c[2]


def ratio(t):
    names = {1: "1 kg box", 2: "3 t box", 7: "plate"}
    out = []
    for i, nm in names.items():
        s = t.track(i)
        out.append(f"{nm} sink {(s[0][0][1] - s[-1][0][1]) * 1e3:.4g} mm, drift {math.hypot(s[-1][0][0] - s[0][0][0], s[-1][0][2] - s[0][0][2]) * 1e3:.4g} mm")
    chips = [t.track(i)[-1][0][1] for i in range(3, 7)]
    sl = asleep_from(t, dynamic(t))
    print("ratio: " + "; ".join(out) + f"; chips' centres {', '.join(f'{y:.4f}' for y in chips)} m (start 0.01); asleep "
          f"{('from tick ' + str(sl)) if sl is not None else 'never'}")


def chip(t):
    s = t.track(1)
    low = min(r[0][1] for r in s)
    stop = None
    for k in range(len(s)):
        if norm(s[k][2]) < 0.05 and stop is None:
            stop = t.ticks[k]
        elif norm(s[k][2]) >= 0.05:
            stop = None
    sl = asleep_from(t, [1])
    p = s[-1][0]
    print(f"chip: lowest centre {low:.4f} m (half thickness 0.01), at rest from tick {stop}, asleep {('from tick ' + str(sl)) if sl is not None else 'never'}, "
          f"ends at ({p[0]:.3f}, {p[1]:.4f}, {p[2]:.3f})")


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
        {"stack10": stack10, "pile200": pile200, "bounce": bounce, "ramp": ramp, "ratio": ratio, "chip": chip}[a.scene](t)
        return
    print(f"{a.file}: {t.bodies} bodies, {t.records} records every {t.every} ticks, dt {t.dt}")
    for i in range(t.bodies):
        s = t.track(i)
        vmax = max(norm(r[2]) for r in s)
        print(f"  body {i}{' static' if s[0][5] & 1 else ''}: {tuple(round(x, 4) for x in s[0][0])} -> {tuple(round(x, 4) for x in s[-1][0])}, "
              f"largest speed {vmax:.3g} m/s, {'awake' if s[-1][4] else 'asleep'} at the end")


if __name__ == "__main__":
    sys.exit(main())
