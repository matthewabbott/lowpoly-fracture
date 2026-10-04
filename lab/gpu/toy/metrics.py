#!/usr/bin/env python3
"""metrics.py: the toy's measurements, each computed once (DESIGN.md "Scenes and acceptance"), from trajectory files
(traj.h's LPTRAJ1, written by the toy and by b3ref2; traj.py reads them) and from their logs' solve and rest lines.
accept.py builds decision point 1's table from them (gate.py's gate 4 runs it), gate 2 compares the dialects with
`rms`, and `summary` prints a scene's numbers in one line.

  python lab/gpu/toy/metrics.py FILE --scene stack10|pile200|bounce|ramp|ratio|chip|arm [--ref D.traj] [--log LOG]

Each function takes a traj.Traj and returns a dict:
- stack: the top cube's drift (horizontal, from its position at tick 60 to the end, as the brief has it; from tick 600,
  once the first seconds' sway (`sway`, the largest excursion over the first 10 s) has died; from the start), sink,
  yaw (the end against the start, and the largest over the run), tilt, its mean speed over the last 10 s and its
  largest speed and spin in the last second; the cubes' mean speed over the last 10 s; when every cube sleeps and
  whether it stays asleep.
- pile: escapes (a dynamic body's centre outside the pit, |x| or |z| over 2 m (the walls' inner faces), or below the
  floor's top, in any record), when every body sleeps, the bodies awake and the highest centre at the end.
- ratio: the outcome class of each half (the 3 t box on the 1 kg box; the plate on the chips); the boxes' and the
  plate's sink and drift.
- bounce: the impact (the lowest centre, the fastest fall) and the first apex against the analytic 0.6875 m.
- ramp: the 0.6 box's travel down the slope (from the start; its creep from tick 60) and its speed at the end; the 0.2
  box's distance down the slope at tick 180 (3 s, still on the ramp) and at the end; both boxes' accelerations down the
  slope (a least-squares parabola over ticks 30 to 150; the sliding one's analytic g (sin - mu cos)).
- chip: its lowest and last centre, when it stops (under 0.05 m/s from then on), when it sleeps and whether it stays
  asleep.
- arm: per joint, the largest gap, overshoot, axis misalignment and servo tracking error, and the angle track
  (angle_diff compares two runs').
- rms: position rms against a reference trajectory over ticks lo to hi (dynamic bodies), the largest difference and
  where, and the orientations' rms and largest angle (gate 2's dialects against D).
- parse_log: the solve line (when every body sleeps, the deepest point over the run and at rest) and the rest line (the
  deepest point over every touching pair after the last tick), the toy's saturations, threads and sentinel.
"""
import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj import Traj, dynamic, norm, qconj, qmul, rel_angles  # noqa: E402

PIT = 2.0  # the walls' inner faces (pile200: walls at +-2.25, 0.25 thick)
SLOPE = math.radians(25.0)
DOWN = (-math.cos(SLOPE), -math.sin(SLOPE), 0.0)  # down the ramp (scene.c's ramp rises along +x)
MU_SLIDE = math.sqrt(0.12)  # the 0.2 box on the 0.6 ramp, Box3D's mixing sqrt( fA fB )


def index_at(t, tick):
    """The first record at or after tick (the last one if none)."""
    for k, x in enumerate(t.ticks):
        if x >= tick:
            return k
    return len(t.ticks) - 1


def sleep_info(t, bodies):
    """(first tick every listed body is asleep, the tick from which they stay asleep to the end, stays asleep)."""
    first = None
    for k in range(len(t.ticks)):
        if all(t.rec[k][i][4] == 0 for i in bodies):
            first = t.ticks[k]
            break
    stay = None
    for k in range(len(t.ticks) - 1, -1, -1):
        if all(t.rec[k][i][4] == 0 for i in bodies):
            stay = t.ticks[k]
        else:
            break
    return first, stay, first is not None and first == stay


def stack(t):
    top = t.bodies - 1
    dyn = dynamic(t)
    s = t.track(top)
    k60, k600 = index_at(t, 60), index_at(t, 600)
    p0, q0 = s[0][0], s[0][1]
    p60, p600 = s[k60][0], s[k600][0]
    p1, q1 = s[-1][0], s[-1][1]
    yaw, tilt = rel_angles(q0, q1)
    yawmax = max(abs(rel_angles(q0, r[1])[0]) for r in s)
    end = t.ticks[-1]
    last10 = [k for k in range(len(t.ticks)) if t.ticks[k] > end - 600]
    last1 = [k for k in range(len(t.ticks)) if t.ticks[k] > end - 60]
    first, stay, stays = sleep_info(t, dyn)
    return {
        "top": top,
        "end": end,
        "drift": math.hypot(p1[0] - p60[0], p1[2] - p60[2]),
        "drift0": math.hypot(p1[0] - p0[0], p1[2] - p0[2]),
        "drift600": math.hypot(p1[0] - p600[0], p1[2] - p600[2]),
        "sway": max(math.hypot(r[0][0] - p0[0], r[0][2] - p0[2]) for r in s[:k600 + 1]),
        "sink": p0[1] - p1[1],
        "yaw": math.degrees(yaw),
        "yawmax": math.degrees(yawmax),
        "tilt": math.degrees(tilt),
        "speed": sum(norm(t.rec[k][top][2]) for k in last10) / max(1, len(last10)),
        "speedall": sum(norm(t.rec[k][i][2]) for k in last10 for i in dyn) / max(1, len(last10) * len(dyn)),
        "speedmax1": max(norm(t.rec[k][top][2]) for k in last1),
        "spinmax1": max(norm(t.rec[k][top][3]) for k in last1),
        "asleep": first,
        "asleepstay": stay,
        "stays": stays,
    }


def pile(t):
    dyn = dynamic(t)
    out = set()
    for row in t.rec:
        for i in dyn:
            p = row[i][0]
            if abs(p[0]) > PIT or abs(p[2]) > PIT or p[1] < 0.0:
                out.add(i)
    first, stay, stays = sleep_info(t, dyn)
    return {"escapes": sorted(out), "asleep": first, "asleepstay": stay, "stays": stays, "end": t.ticks[-1],
            "awake_end": sum(1 for i in dyn if t.rec[-1][i][4]), "top": max(t.rec[-1][i][0][1] for i in dyn)}


def ratio(t):
    """Classes from the last record. The 3 t box (body 2, 1 m cube, starts at y 1.0 on the 0.5 m cube of 1 kg, body 1):
    'rests' when its centre stays at 0.95 m or more (at most 5 cm into the small box), 'crushed through' when it ends on
    the ground (0.55 m or less) with the small box still under its footprint (the two interpenetrate), 'pushed aside'
    when it ends on the ground beside the small box, 'partly sunk' between. The plate (body 7, 5 cm thick, its centre at
    0.045 m on four 2 cm chips, bodies 3 to 6): 'rests' at 0.04 m or more (at most 5 mm into the chips), 'sinks through'
    at 0.03 m or less (its bottom within 5 mm of the ground), 'partly sunk' between; 'chips squeezed out' when a chip
    moved over 10 cm sideways."""
    last = t.rec[-1]
    small, big, plate = last[1][0], last[2][0], last[7][0]
    if big[1] >= 0.95:
        a = "rests"
    elif big[1] <= 0.55:
        under = abs(small[0] - big[0]) < 0.75 and abs(small[2] - big[2]) < 0.75
        a = "crushed through" if under else "pushed aside"
    else:
        a = "partly sunk"
    moved = [i for i in range(3, 7) if math.hypot(last[i][0][0] - t.rec[0][i][0][0], last[i][0][2] - t.rec[0][i][0][2]) > 0.1]
    if moved:
        b = "chips squeezed out"
    elif plate[1] >= 0.04:
        b = "rests"
    elif plate[1] <= 0.03:
        b = "sinks through"
    else:
        b = "partly sunk"
    first, stay, stays = sleep_info(t, dynamic(t))
    sink = {i: t.rec[0][i][0][1] - last[i][0][1] for i in (1, 2, 7)}
    drift = {i: math.hypot(last[i][0][0] - t.rec[0][i][0][0], last[i][0][2] - t.rec[0][i][0][2]) for i in (1, 2, 7)}
    return {"heavy": a, "plate": b, "big_y": big[1], "small_y": small[1], "plate_y": plate[1],
            "chips_y": [last[i][0][1] for i in range(3, 7)], "asleep": stay, "sink": sink, "drift": drift}


def bounce(t):
    s = t.track(1)
    ys = [r[0][1] for r in s]
    k = min(range(len(ys)), key=lambda i: ys[i])
    apex = max(ys[k:])
    want = 0.25 + 0.25 * 1.75
    return {"apex": apex, "analytic": want, "error": (apex - want) / want, "rise": (apex - 0.25) / (want - 0.25),
            "lowest": ys[k], "impact_tick": t.ticks[k], "vmin": min(r[2][1] for r in s), "apex_tick": t.ticks[k + ys[k:].index(apex)]}


def fit_accel(pts):
    """d = a + b t + c t^2 by least squares; the acceleration 2c."""
    S = [[0.0] * 3 for _ in range(3)]
    r = [0.0] * 3
    for x, y in pts:
        b = (1.0, x, x * x)
        for i in range(3):
            r[i] += b[i] * y
            for j in range(3):
                S[i][j] += b[i] * b[j]
    for i in range(3):  # Gaussian elimination
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


def ramp(t, at=180):
    def along(i, k):
        s = t.track(i)
        return sum((s[k][0][j] - s[0][0][j]) * DOWN[j] for j in range(3))

    def accel(i):
        pts = [(t.ticks[k] * t.dt, along(i, k)) for k in range(len(t.ticks)) if 30 <= t.ticks[k] <= 150]
        return fit_accel(pts) if len(pts) > 5 else float("nan")
    k60, kat, end = index_at(t, 60), index_at(t, at), len(t.ticks) - 1
    last1 = [k for k in range(len(t.ticks)) if t.ticks[k] > t.ticks[-1] - 60]
    return {"hold_travel": along(2, end), "hold_creep": along(2, end) - along(2, k60),
            "hold_speed": max(norm(t.rec[k][2][2]) for k in last1), "hold_accel": accel(2),
            "slide": along(3, kat), "slide_tick": t.ticks[kat], "slide_end": along(3, end), "slide_accel": accel(3),
            "slide_analytic": 10.0 * (math.sin(SLOPE) - MU_SLIDE * math.cos(SLOPE)), "end": t.ticks[-1]}


def chip(t):
    s = t.track(1)
    first, stay, stays = sleep_info(t, [1])
    rest = None
    for k in range(len(s)):
        if norm(s[k][2]) < 0.05:
            rest = t.ticks[k] if rest is None else rest
        else:
            rest = None
    return {"lowest": min(r[0][1] for r in s), "final_y": s[-1][0][1], "asleep": first, "asleepstay": stay, "stays": stays,
            "end": s[-1][0], "rest": rest}


JOINT = re.compile(r"^joint (\d+): bodies (\d+) (\d+) anchorA (\S+) (\S+) (\S+) anchorB (\S+) (\S+) (\S+) frameA (\S+) (\S+) (\S+) (\S+) "
                   r"frameB (\S+) (\S+) (\S+) (\S+) limit (\d) (\S+) (\S+) motor (\d) (\S+) (\S+) servo (\d) (\S+) (\S+) (\S+) (\d+)", re.M)


def parse_joints(text):
    """The scene's joints from a log's joint lines (the toy and b3ref2 print the same, from the scene's doubles)."""
    out = []
    for m in JOINT.finditer(text):
        g = m.groups()
        f = [float(x) for x in g[3:17]]
        out.append({"a": int(g[1]), "b": int(g[2]), "anchorA": f[0:3], "anchorB": f[3:6], "frameA": f[6:10], "frameB": f[10:14],
                    "limit": g[17] == "1", "lower": float(g[18]), "upper": float(g[19]), "motor": g[20] == "1",
                    "servo": g[23] == "1", "gain": float(g[24]), "maxSpeed": float(g[25]), "amplitude": float(g[26]), "period": int(g[27])})
    return out


def qrot(q, v):
    """b3RotateVector: v + 2 q.v x (q.v x v + q.s v), q = (x, y, z, s)."""
    x, y, z, s = q
    t = (y * v[2] - z * v[1] + s * v[0], z * v[0] - x * v[2] + s * v[1], x * v[1] - y * v[0] + s * v[2])
    c = (y * t[2] - z * t[1], z * t[0] - x * t[2], x * t[1] - y * t[0])
    return (v[0] + 2 * c[0], v[1] + 2 * c[1], v[2] + 2 * c[2])


def servo_target(j, tick):
    """scene.c's scene_servo_target: the triangle wave before step tick."""
    if not j["servo"] or j["period"] <= 0:
        return 0.0
    x = (tick % j["period"]) / j["period"]
    tri = 4 * x if x < 0.25 else (2 - 4 * x if x < 0.75 else 4 * x - 4)
    return j["amplitude"] * tri


def joint_state(rec, j):
    """One record's joint: (twist angle about the hinge in rad, the anchors' gap in m, the hinge axes' misalignment in rad),
    as Box3D's b3RevoluteJoint_GetAngle (polarity, b3GetTwistAngle)."""
    (pA, qA), (pB, qB) = (rec[j["a"]][0], rec[j["a"]][1]), (rec[j["b"]][0], rec[j["b"]][1])
    rA, rB = qrot(qA, j["anchorA"]), qrot(qB, j["anchorB"])
    gap = math.sqrt(sum((pB[i] + rB[i] - pA[i] - rA[i]) ** 2 for i in range(3)))
    fA, fB = qmul(qA, j["frameA"]), qmul(qB, j["frameB"])
    if sum(fA[i] * fB[i] for i in range(4)) < 0:
        fB = tuple(-x for x in fB)
    r = qmul(qconj(fA), fB)
    twist = 2 * (math.atan2(-r[2], -r[3]) if r[3] < 0 else math.atan2(r[2], r[3]))
    n = math.sqrt(sum(x * x for x in r))
    swing = 2 * math.asin(min(1.0, math.sqrt(r[0] ** 2 + r[1] ** 2) / n))
    return twist, gap, swing


def arm(t, joints, lo=1, hi=None):
    """Per joint over the records (ticks lo..hi): the largest gap, limit overshoot (beyond lower or upper), axis
    misalignment, the servo's largest tracking error against its target, and the angle track (for the comparison against
    D); the limited joint's ticks at a limit (within 1 mrad, or beyond)."""
    out = []
    for k, j in enumerate(joints):
        gap = over = swing = track = gapSweep = 0.0
        gapAt = overAt = None
        atLimit = 0
        angles = {}
        for i, tick in enumerate(t.ticks):
            if tick < lo or (hi is not None and tick > hi):
                continue
            a, g, s = joint_state(t.rec[i], j)
            angles[tick] = a
            if g > gap:
                gap, gapAt = g, tick
            if 30 <= tick <= 90:
                gapSweep = max(gapSweep, g)
            swing = max(swing, s)
            if j["limit"]:
                o = max(j["lower"] - a, a - j["upper"], 0.0)
                if o > over:
                    over, overAt = o, tick
                atLimit += a <= j["lower"] + 1e-3 or a >= j["upper"] - 1e-3
            if j["servo"] and tick >= 1:
                track = max(track, abs(a - servo_target(j, tick)))
        out.append({"gap": gap, "gapAt": gapAt, "gapSweep": gapSweep, "overshoot": over, "overAt": overAt, "swing": swing, "track": track,
                    "angles": angles,
                    "atLimit": atLimit, "limit": j["limit"], "servo": j["servo"],
                    "range": (min(angles.values()), max(angles.values())) if angles else (0, 0)})
    return out


def angle_diff(x, ref, lo=1, hi=120):
    """The largest difference of a joint's angle against a reference run's over ticks lo..hi (both recorded)."""
    d, at = 0.0, None
    for tick in range(lo, hi + 1):
        if tick in x["angles"] and tick in ref["angles"]:
            e = abs(x["angles"][tick] - ref["angles"][tick])
            if e > d:
                d, at = e, tick
    return d, at


def rms(t, ref, lo=1, hi=120, bodies=None):
    """Over ticks lo..hi (both trajectories must record them), the dynamic bodies (or the listed ones): the position rms
    (m), the largest position difference and its tick and body, and the orientations' rms and largest angle (rad; the
    angle between two unit quaternions, 2 sqrt(1 - c^2) for small angles, c their normalised dot)."""
    a = {x: k for k, x in enumerate(t.ticks)}
    b = {x: k for k, x in enumerate(ref.ticks)}
    dyn = dynamic(t) if bodies is None else bodies
    s, n, mx, at, atBody, rs, rmx = 0.0, 0, 0.0, None, None, 0.0, 0.0
    for tick in range(lo, hi + 1):
        if tick not in a or tick not in b:
            raise ValueError(f"tick {tick} is not recorded in both trajectories")
        ra, rb = t.rec[a[tick]], ref.rec[b[tick]]
        for i in dyn:
            e = sum((ra[i][0][j] - rb[i][0][j]) ** 2 for j in range(3))
            s += e
            n += 1
            if e > mx:
                mx, at, atBody = e, tick, i
            x, y = ra[i][1], rb[i][1]
            dq = sum(x[j] * y[j] for j in range(4))
            c2 = dq * dq / (sum(v * v for v in x) * sum(v * v for v in y))
            ang2 = 4.0 * (1.0 - c2) if c2 < 1.0 else 0.0
            rs += ang2
            rmx = max(rmx, ang2)
    return {"rms": math.sqrt(s / max(1, n)), "max": math.sqrt(mx), "at": at, "body": atBody, "n": n, "sq": s,
            "rot_rms": math.sqrt(rs / max(1, n)), "rot_max": math.sqrt(rmx)}


SOLVE = re.compile(r"^solve: (?:every dynamic body asleep from tick (\d+)|(\d+) bodies awake at the end); deepest point (\S+) m "
                   r"\(tick (\d+), bodies (-?\d+) (-?\d+)\), at rest \(.*?\) (\S+) m(?: \(tick (\d+), bodies (-?\d+) (-?\d+)\))?", re.M)
REST = re.compile(r"^rest: after tick \d+, over every touching pair \(sleeping ones too\): deepest point (\S+) m \(bodies (-?\d+) (-?\d+)\), "
                  r"(\d+) touching", re.M)


def parse_log(text):
    out = {}
    m = SOLVE.search(text)
    if m:
        out["asleep"] = int(m.group(1)) if m.group(1) else None
        out["awake_end"] = int(m.group(2)) if m.group(2) else 0
        out["deep"] = float(m.group(3))
        out["deep_tick"] = int(m.group(4))
        out["deep_bodies"] = (int(m.group(5)), int(m.group(6)))
        out["deep_rest"] = float(m.group(7))
        if m.group(8):
            out["deep_rest_tick"] = int(m.group(8))
            out["deep_rest_bodies"] = (int(m.group(9)), int(m.group(10)))
    m = REST.search(text)
    if m:
        out["final"] = float(m.group(1))
        out["final_bodies"] = (int(m.group(2)), int(m.group(3)))
        out["final_touching"] = int(m.group(4))
    m = re.search(r"threads (\w+), saturations (\d+) \(twin", text)
    if m:
        out["threads"] = m.group(1)
        out["saturations"] = int(m.group(2))
    out["sentinel"] = "TRIPPED" not in text and "flushes subnormals" not in text
    m = re.search(r"^box3d: every dynamic body asleep after step (\d+)", text, re.M)
    if m:
        out["box3d_asleep"] = int(m.group(1))
    m = re.search(r"^mass from Box3D's shapes against the scene's: mass within (\S+) .*?inertia within (\S+) ", text, re.M)
    if m:
        out["mass_check"] = (float(m.group(1)), float(m.group(2)))
    m = re.search(r"^hulls: .*?counts differ on (\d+)", text, re.M)
    if m:
        out["hull_differ"] = int(m.group(1))
    return out


def tick_text(x):
    return "never" if x is None else f"tick {x}"


def summary(scene, m):
    """One line of a scene's numbers from its measurement dict (for arm: the list of joints)."""
    if scene == "stack10":
        return (f"top cube (body {m['top']}) after tick {m['end']}: drift {m['drift0'] * 1e3:.4g} mm (from tick 600 {m['drift600'] * 1e3:.3g}), "
                f"sink {m['sink'] * 1e3:.4g} mm, yaw {m['yaw']:.4g} deg, tilt {m['tilt']:.4g} deg; the last second's largest speed "
                f"{m['speedmax1'] * 1e3:.4g} mm/s, spin {m['spinmax1']:.3g} rad/s; asleep from {tick_text(m['asleepstay'])}")
    if scene == "pile200":
        return (f"every body asleep from {tick_text(m['asleepstay'])} ({m['awake_end']} awake at tick {m['end']}); escapes "
                f"{len(m['escapes'])}{' ' + str(m['escapes'][:8]) if m['escapes'] else ''}; highest centre {m['top']:.3f} m")
    if scene == "bounce":
        return (f"impact near tick {m['impact_tick']} (lowest centre {m['lowest']:.4f} m, fastest fall {m['vmin']:.4f} m/s; analytic -5.916), "
                f"first apex {m['apex']:.4f} m at tick {m['apex_tick']} (analytic 0.6875 m: {100.0 * m['rise']:.1f}% of the rise)")
    if scene == "ramp":
        return (f"friction 0.6 (body 2): travel {m['hold_travel']:.4g} m by tick {m['end']}, acceleration (ticks 30-150) "
                f"{m['hold_accel']:.4g} m/s^2; friction 0.2 (body 3): {m['slide']:.4g} m at tick {m['slide_tick']}, {m['slide_end']:.4g} m by "
                f"tick {m['end']}, acceleration {m['slide_accel']:.4g} m/s^2 (analytic {m['slide_analytic']:.4g})")
    if scene == "ratio":
        names = {1: "1 kg box", 2: "3 t box", 7: "plate"}
        return ("; ".join(f"{names[i]} sink {m['sink'][i] * 1e3:.4g} mm, drift {m['drift'][i] * 1e3:.4g} mm" for i in (1, 2, 7)) +
                f"; chips' centres {', '.join(f'{y:.4f}' for y in m['chips_y'])} m (start 0.01); {m['heavy']}, {m['plate']}; asleep from "
                f"{tick_text(m['asleep'])}")
    if scene == "chip":
        p = m["end"]
        return (f"lowest centre {m['lowest']:.4f} m (half thickness 0.01), at rest from {tick_text(m['rest'])}, asleep from "
                f"{tick_text(m['asleepstay'])}, ends at ({p[0]:.3f}, {p[1]:.4f}, {p[2]:.3f})")
    if scene == "arm":
        return "; ".join(f"joint {k}: gap {r['gap'] * 1e3:.3g} mm, overshoot {r['overshoot'] * 1e3:.3g} mrad, axis error "
                         f"{r['swing'] * 1e3:.3g} mrad, angles {r['range'][0]:.3f} to {r['range'][1]:.3f}" for k, r in enumerate(m))
    raise ValueError(f"no summary for scene {scene}")


FNS = {"stack10": stack, "pile200": pile, "bounce": bounce, "ramp": ramp, "ratio": ratio, "chip": chip}


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("file")
    a.add_argument("--scene", required=True)
    a.add_argument("--ref")
    a.add_argument("--log")
    a = a.parse_args()
    t = Traj(a.file)
    if a.scene == "arm":
        if not a.log:
            raise SystemExit("arm: --log (the run's log, for its joint lines)")
        joints = parse_joints(open(a.log, errors="replace").read())
        res = arm(t, joints)
        ref = arm(Traj(a.ref), joints) if a.ref else None
        for k, r in enumerate(res):
            line = (f"joint {k}: gap {r['gap'] * 1e3:.4g} mm (tick {r['gapAt']}), overshoot {r['overshoot'] * 1e3:.4g} mrad (tick {r['overAt']}), "
                    f"axis error {r['swing'] * 1e3:.4g} mrad, "
                    f"servo tracking {r['track']:.4g} rad, angles {r['range'][0]:.4f} to {r['range'][1]:.4f}, records at a limit {r['atLimit']}")
            if ref:
                d, at = angle_diff(r, ref[k])
                line += f"; angle vs ref, ticks 1-120: {d * 1e3:.4g} mrad (tick {at})"
            print(line)
        return
    m = FNS[a.scene](t)
    print(f"{a.scene}: {summary(a.scene, m)}")
    for k, v in m.items():
        print(f"{k}: {v}")
    if a.ref:
        r = rms(t, Traj(a.ref))
        print(f"position rms against {a.ref}, ticks 1-120: {r['rms']:.3g} m (max {r['max']:.3g} m, tick {r['at']}, body {r['body']}); "
              f"rotation rms {r['rot_rms']:.3g} rad, max {r['rot_max']:.3g} rad")
    if a.log:
        for k, v in parse_log(open(a.log, errors="replace").read()).items():
            print(f"{k}: {v}")


if __name__ == "__main__":
    main()
