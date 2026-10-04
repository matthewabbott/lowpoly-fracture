#!/usr/bin/env python3
"""accept.py: decision point 1 (DESIGN.md "Scenes and acceptance"): runs the toy's twins (D, F, V4; one thread) and
b3ref2 (Box3D) on every scene and seed, measures each trajectory (metrics.py) and prints the acceptance table: one row
per criterion with the values of Box3D, D, F and V4, and PASS or FAIL against DESIGN.md's thresholds (for every
program; the decision is F's and V4's). Writes the table to lab/gpu/toy/results/decision1.md.

  python lab/gpu/toy/accept.py [--compiler msvc] [--out lab/gpu/build/toy-scratch/4b] [--jobs 8] [--no-run]

Runs (trajectories and logs in --out; --no-run measures what is there): stack10 for 3,600 ticks with sleep off, with
sleep on, and with sleep and contact recycling off; pile200 for 3,600 ticks over seeds 1 to 8 (every 10th tick
recorded) and for 120 ticks (every tick); bounce 300, ramp 600, ratio 1,200 and chip 600 ticks; Box3D's chip again
with continuous collision on; D with every dynamic body moved 1e-9 m along x and y, the first 120 ticks of each scene
(how much the scene amplifies a tiny difference). Every program starts from the same values (scene_round_start: floats
on V4's grids, step 5a), so no difference is the start's.
"""
import argparse
import concurrent.futures
import math
import os
import statistics
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics  # noqa: E402
from traj import Traj  # noqa: E402

TOY = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.dirname(TOY)
EXE = ".exe" if sys.platform == "win32" else ""
PROGS = ("Box3D", "D", "F", "V4")
SEEDS = range(1, 9)
PERTURB = "1e-9"
REST_LOOK = 0.015  # the solve line's "at rest" points deeper than this are looked at closely (the details)

# name, scene, ticks, extra arguments, record every Nth tick
JOBS = [("stack-off", "stack10", 3600, ["--sleep", "0"], 1),
        ("stack-on", "stack10", 3600, ["--sleep", "1"], 1),
        ("stack-r0", "stack10", 3600, ["--sleep", "0", "--recycle", "0"], 1),
        ("bounce", "bounce", 300, [], 1),
        ("ramp", "ramp", 600, [], 1),
        ("ratio", "ratio", 1200, [], 1),
        ("chip", "chip", 600, [], 1),
        ("arm", "arm", 1200, [], 1)] + \
       [(f"pile-s{k}", "pile200", 3600, ["--seed", str(k)], 10) for k in SEEDS] + \
       [(f"pilehead-s{k}", "pile200", 120, ["--seed", str(k)], 1) for k in SEEDS]
# D perturbed (Dp: every dynamic body moved 1e-9 m along x and y), the first 120 ticks, against D
PERTURBED = [("stack-off", "stack10", ["--sleep", "0"]), ("bounce", "bounce", []), ("ramp", "ramp", []), ("ratio", "ratio", []),
             ("chip", "chip", []), ("arm", "arm", [])] + [(f"pilehead-s{k}", "pile200", ["--seed", str(k)]) for k in SEEDS]


def exe(comp, prog):
    name = "b3ref2" if prog == "Box3D" else f"toy_{prog}"
    return os.path.join(LAB, "build", comp, "bin", name + EXE)


def paths(out, name, prog):
    base = os.path.join(out, f"{name}.{prog}")
    return base + ".traj", base + ".txt"


def run_all(a):
    tasks = []
    for name, scene, ticks, extra, every in JOBS:
        for prog in PROGS:
            tasks.append((name, prog, scene, ticks, extra, every))
    tasks.append(("chip-ccd", "Box3D", "chip", 600, ["--continuous", "1"], 1))
    for name, scene, extra in PERTURBED:
        tasks.append((name, "Dp", scene, 120, extra + ["--perturb", PERTURB], 1))

    def one(task):
        name, prog, scene, ticks, extra, every = task
        tr, log = paths(a.out, name, prog)
        args = ["--scene", scene, "--ticks", str(ticks), "--quiet", "--traj", tr, "--traj-every", str(every), "--log", log, *extra]
        if prog != "Box3D":
            args += ["--threads", "1"]
        r = subprocess.run([exe(a.compiler, "D" if prog == "Dp" else prog), *args], cwd=LAB, capture_output=True, text=True)
        return task, r.returncode

    bad = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for task, code in pool.map(one, tasks):
            if code != 0:
                bad.append(f"{task[0]} {task[1]}: exit {code}")
    print(f"ran {len(tasks)} runs in {a.out}" + ("" if not bad else "; FAILED: " + ", ".join(bad)))
    return bad


# ----------------------------------------------------------------------------------------------------------------------
# measuring
# ----------------------------------------------------------------------------------------------------------------------

def measure(a):
    M = {}  # M[prog][name] = {"m": metrics, "log": parsed log, "traj": Traj (kept for the 120-tick ones)}
    fns = {"stack10": metrics.stack, "pile200": metrics.pile, "bounce": metrics.bounce, "ramp": metrics.ramp, "ratio": metrics.ratio,
           "chip": metrics.chip}
    runs = [(n, s, p) for n, s, _, _, _ in JOBS for p in PROGS] + [("chip-ccd", "chip", "Box3D")] + [(n, s, "Dp") for n, s, _ in PERTURBED]
    for name, scene, prog in runs:
        tr, log = paths(a.out, name, prog)
        t = Traj(tr)
        text = open(log, errors="replace").read()
        entry = {"log": metrics.parse_log(text)}
        if scene == "arm":  # the joints' measures (Dp's too: its angles against D's)
            entry["m"] = metrics.arm(t, metrics.parse_joints(text))
        elif prog != "Dp" and not name.startswith("pilehead"):
            entry["m"] = fns[scene](t)
        lg = entry["log"]
        if name.startswith("pile-s") and -lg.get("deep_rest", 0.0) > REST_LOOK and "deep_rest_tick" in lg:
            # the solve line's "at rest" point: the run again to 10 ticks past it, every tick recorded (the same run: the
            # programs are deterministic), for its two bodies' largest speed within 10 ticks and the bodies awake at it
            T, (A, B) = lg["deep_rest_tick"], lg["deep_rest_bodies"]
            short = os.path.join(a.out, f"{name}.{prog}.near.traj")
            args = ["--scene", scene, "--seed", name.split("-s")[1], "--ticks", str(T + 10), "--quiet", "--traj", short]
            subprocess.run([exe(a.compiler, prog), *args, *(["--threads", "1"] if prog != "Box3D" else [])], cwd=LAB, capture_output=True)
            n = Traj(short)
            near = [k for k, x in enumerate(n.ticks) if abs(x - T) <= 10]
            lg["rest_speed"] = max(metrics.norm(n.rec[k][i][2]) for k in near for i in (A, B) if i >= 0)
            lg["rest_awake"] = sum(1 for i in metrics.dynamic(n) if n.rec[T][i][4])
            os.remove(short)
        if t.every == 1:
            entry["traj"] = t  # for the 120-tick rms
        M.setdefault(prog, {})[name] = entry
    return M


def rms_of(M, prog, name, ref="D"):
    return metrics.rms(M[prog][name]["traj"], M[ref][name]["traj"])


def pooled_rms(M, prog, ref="D"):
    s, n, worst, worstSeed = 0.0, 0, 0.0, None
    for k in SEEDS:
        r = rms_of(M, prog, f"pilehead-s{k}", ref)
        s += r["sq"]
        n += r["n"]
        if r["rms"] > worst:
            worst, worstSeed = r["rms"], k
    return math.sqrt(s / n), worst, worstSeed


# ----------------------------------------------------------------------------------------------------------------------
# the table
# ----------------------------------------------------------------------------------------------------------------------

def fmt(x, digits=4):
    if x is None:
        return "-"
    if isinstance(x, str):
        return x
    if x == 0:
        return "0"
    return f"{x:.{digits}g}"


def verdict(ok):
    return "-" if ok is None else ("PASS" if ok else "**FAIL**")


def build_table(M):
    rows = []  # (group, criterion, threshold, {prog: value text}, {prog: ok or None})

    def add(group, crit, thr, vals, oks):
        rows.append((group, crit, thr, vals, oks))

    P = PROGS
    # stack10, sleep off
    m = {p: M[p]["stack-off"]["m"] for p in P}
    bd = m["Box3D"]["drift"]
    lim = max(2.0 * bd, 0.002)
    add("stack10, sleep off, 3,600 ticks", "top cube's drift, tick 60 to 3,600 (mm; the stack still sways at tick 60)",
        f"<= max(2 x Box3D's, 2 mm) = {lim * 1e3:.3g} mm", {p: fmt(m[p]["drift"] * 1e3) for p in P}, {p: m[p]["drift"] <= lim for p in P})
    bd = m["Box3D"]["drift600"]
    lim = max(2.0 * bd, 0.002)
    add("", "top cube's drift, tick 600 to 3,600 (mm; the sway has died: the creep)", f"<= max(2 x Box3D's, 2 mm) = {lim * 1e3:.3g} mm",
        {p: fmt(m[p]["drift600"] * 1e3, 3) for p in P}, {p: m[p]["drift600"] <= lim for p in P})
    add("", "top cube's yaw at 60 s (deg)", "<= 0.5 deg", {p: fmt(m[p]["yaw"]) for p in P}, {p: abs(m[p]["yaw"]) <= 0.5 for p in P})
    bs = m["Box3D"]["speed"]
    lim = max(3.0 * bs, 0.001)
    add("", "top cube's mean speed, last 10 s (mm/s)", f"<= max(3 x Box3D's, 1 mm/s) = {lim * 1e3:.3g} mm/s",
        {p: fmt(m[p]["speed"] * 1e3) for p in P}, {p: m[p]["speed"] <= lim for p in P})
    # stack10, sleep on
    m = {p: M[p]["stack-on"]["m"] for p in P}
    bt = m["Box3D"]["asleepstay"]
    lim = 1.5 * bt if bt else None
    add("stack10, sleep on", "every cube asleep from tick (stays asleep)", f"<= 1.5 x Box3D's = {fmt(lim)}, and stays",
        {p: f"{fmt(m[p]['asleep'])} ({'stays' if m[p]['stays'] else 'wakes'})" for p in P},
        {p: lim is not None and m[p]["asleep"] is not None and m[p]["asleep"] <= lim and m[p]["stays"] for p in P})
    # pile200
    pl = {p: [M[p][f"pile-s{k}"] for k in SEEDS] for p in P}
    asl = {p: [e["log"].get("asleep") for e in pl[p]] for p in P}
    add("pile200, 8 seeds, sleep on, 3,600 ticks", "seeds with every body asleep by tick 3,600", "8 of 8",
        {p: f"{sum(x is not None for x in asl[p])} of 8" for p in P}, {p: all(x is not None for x in asl[p]) for p in P})
    med = {p: statistics.median([x if x is not None else 3601 for x in asl[p]]) for p in P}
    lim = 1.5 * med["Box3D"]
    add("", "tick every body asleep: median (max)", f"median <= 1.5 x Box3D's = {fmt(lim)}",
        {p: f"{fmt(med[p])} ({fmt(max(x if x is not None else 3601 for x in asl[p]))})" for p in P}, {p: med[p] <= lim for p in P})
    esc = {p: sum(len(e["m"]["escapes"]) for e in pl[p]) for p in P}
    add("", "escapes (bodies outside the pit, any record)", "0", {p: str(esc[p]) for p in P}, {p: esc[p] == 0 for p in P})
    fin = {p: min(e["log"]["final"] for e in pl[p]) for p in P}
    finSeed = {p: min(SEEDS, key=lambda k: pl[p][k - 1]["log"]["final"]) for p in P}
    add("", "deepest point at rest: every touching pair once every body sleeps (tick 3,600), worst seed (cm)", "<= 2 cm",
        {p: f"{fmt(-fin[p] * 100, 3)} (seed {finSeed[p]})" for p in P}, {p: -fin[p] <= 0.02 for p in P})
    rest = {p: min(e["log"]["deep_rest"] for e in pl[p]) for p in P}
    restSeed = {p: min(SEEDS, key=lambda k: pl[p][k - 1]["log"]["deep_rest"]) for p in P}
    add("", "deepest point in the 60 ticks before every body sleeps (the solve line's \"at rest\"), worst seed (cm)",
        "reported: not at rest (late impacts, see the details); 2 cm would fail Box3D",
        {p: f"{fmt(-rest[p] * 100, 3)} (seed {restSeed[p]})" for p in P}, {p: None for p in P})
    drop = {p: min(e["log"]["deep"] for e in pl[p]) for p in P}
    add("", "deepest point during the drop, worst seed (cm)", "(reported)", {p: fmt(-drop[p] * 100, 3) for p in P}, {p: None for p in P})
    # ratio
    m = {p: M[p]["ratio"]["m"] for p in P}
    for key, crit in (("heavy", "3 t box on the 1 kg box: outcome"), ("plate", "100 kg plate on four 0.05 kg chips: outcome")):
        add("ratio" if key == "heavy" else "", crit, "Box3D's class", {p: m[p][key] for p in P}, {p: m[p][key] == m["Box3D"][key] for p in P})
    # bounce
    m = {p: M[p]["bounce"]["m"] for p in P}
    add("bounce", "first apex (m), analytic 0.6875", "within 5% of the analytic",
        {p: f"{fmt(m[p]['apex'])} ({100 * m[p]['error']:+.1f}%)" for p in P}, {p: abs(m[p]["error"]) <= 0.05 for p in P})
    # ramp
    m = {p: M[p]["ramp"]["m"] for p in P}
    add("ramp", "friction 0.6 box: travel down the slope by tick 600 (mm); creep after tick 60 (mm)",
        "holds: travel <= 10 mm, creep <= 1 mm",
        {p: f"{fmt(m[p]['hold_travel'] * 1e3, 3)}; {fmt(m[p]['hold_creep'] * 1e3, 3)}" for p in P},
        {p: m[p]["hold_travel"] <= 0.01 and abs(m[p]["hold_creep"]) <= 0.001 for p in P})
    bsl = m["Box3D"]["slide"]
    add("", f"friction 0.2 box: distance down the slope at tick {m['Box3D']['slide_tick']} (m)", "within 2% of Box3D's",
        {p: f"{fmt(m[p]['slide'])} ({100 * (m[p]['slide'] - bsl) / bsl:+.2f}%)" for p in P},
        {p: abs(m[p]["slide"] - bsl) <= 0.02 * abs(bsl) for p in P})
    # chip
    m = {p: M[p]["chip"]["m"] for p in P}
    add("chip", "lowest centre; last centre (m), half thickness 0.01", "no tunnelling: both above the ground",
        {p: f"{fmt(m[p]['lowest'])}; {fmt(m[p]['final_y'])}" for p in P},
        {p: m[p]["lowest"] > 0.0 and m[p]["final_y"] > 0.005 for p in P})
    add("", "asleep from tick (stays asleep)", "sleeps and stays asleep",
        {p: f"{fmt(m[p]['asleep'])} ({'stays' if m[p]['stays'] else 'wakes'})" for p in P}, {p: m[p]["asleep"] is not None and m[p]["stays"] for p in P})
    # arm (step 6): the servo joint (0) and the limited joint (1)
    m = {p: M[p]["arm"]["m"] for p in P}
    dp = M["Dp"]["arm"]["m"]
    for k, label in ((0, "servo joint (0)"), (1, "limited joint (1)")):
        d = {p: metrics.angle_diff(m[p][k], m["D"][k])[0] for p in P}
        dpk = metrics.angle_diff(dp[k], m["D"][k])[0]
        add("arm, 1,200 ticks" if k == 0 else "", f"{label}: angle against D's, ticks 1-120, largest (mrad)", "F, V4: <= 1 mrad",
            {"Box3D": fmt(d["Box3D"] * 1e3, 3), "D": f"moved 1e-9 m: {fmt(dpk * 1e3, 3)}", "F": fmt(d["F"] * 1e3, 3), "V4": fmt(d["V4"] * 1e3, 3)},
            {"Box3D": None, "D": None, "F": d["F"] <= 1e-3, "V4": d["V4"] <= 1e-3})
    add("", "hinge gap, largest over the run: servo joint; limited joint (mm)", "<= 1 mm",
        {p: f"{fmt(m[p][0]['gap'] * 1e3, 3)}; {fmt(m[p][1]['gap'] * 1e3, 3)}" for p in P},
        {p: max(m[p][0]["gap"], m[p][1]["gap"]) <= 1e-3 for p in P})
    add("", "limited joint: overshoot beyond its limits (+-0.5 rad), largest over the run (mrad)", "<= 10 mrad (0.01 rad)",
        {p: fmt(m[p][1]["overshoot"] * 1e3, 3) for p in P}, {p: m[p][1]["overshoot"] <= 0.01 for p in P})
    sat = {p: sum(e["log"].get("saturations", 0) for e in M[p].values()) for p in ("D", "F", "V4")}
    add("every run", "saturation counters, summed over every run", "V4: 0", {"Box3D": "-", **{p: str(sat[p]) for p in sat}},
        {"Box3D": None, "D": None, "F": None, "V4": sat["V4"] == 0})
    # the first 120 ticks against D
    for name, label, judged in (("stack-off", "stack10 (sleep off)", True), ("bounce", "bounce", True), ("ramp", "ramp", True),
                                ("chip", "chip", False), ("ratio", "ratio", False)):
        r = {p: rms_of(M, p, name)["rms"] for p in ("Box3D", "F", "V4", "Dp")}
        thr = "F <= 1e-5 m, V4 <= 1e-4 m" if judged else "reported (impacts: judged by outcome)"
        add("position rms against D, ticks 1-120 (m)" if name == "stack-off" else "", label, thr,
            {"Box3D": fmt(r["Box3D"], 3), "D": f"moved 1e-9 m: {fmt(r['Dp'], 3)}", "F": fmt(r["F"], 3), "V4": fmt(r["V4"], 3)},
            {"Box3D": None, "D": None, "F": r["F"] <= 1e-5 if judged else None, "V4": r["V4"] <= 1e-4 if judged else None})
    pr = {p: pooled_rms(M, p) for p in ("Box3D", "F", "V4", "Dp")}
    add("", "pile200, 8 seeds pooled (worst seed)", "reported (impacts: judged by outcome)",
        {"Box3D": f"{fmt(pr['Box3D'][0], 3)} ({fmt(pr['Box3D'][1], 3)})",
         "D": f"moved 1e-9 m: {fmt(pr['Dp'][0], 3)} ({fmt(pr['Dp'][1], 3)})",
         "F": f"{fmt(pr['F'][0], 3)} ({fmt(pr['F'][1], 3)})",
         "V4": f"{fmt(pr['V4'][0], 3)} ({fmt(pr['V4'][1], 3)})"},
        {p: None for p in P})
    return rows


def table_md(rows):
    out = ["| scene | criterion | threshold | Box3D | D | F | V4 | Box3D: verdict | D: verdict | F: verdict | V4: verdict |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for group, crit, thr, vals, oks in rows:
        out.append(f"| {group} | {crit} | {thr} | " + " | ".join(vals[p] for p in PROGS) + " | " +
                   " | ".join(verdict(oks[p]) for p in PROGS) + " |")
    return "\n".join(out)


def summary(rows):
    lines = []
    groups, g = [], ""
    for row in rows:
        g = row[0] or g
        groups.append(g)
    for p in PROGS:
        judged = [(g, c, oks[p]) for g, (_, c, _, _, oks) in zip(groups, rows) if oks[p] is not None]
        fails = [f"{g}: {c}" for g, c, ok in judged if not ok]
        lines.append(f"- **{p}**: {len(judged) - len(fails)} of {len(judged)} pass" + (f"; fails: {'; '.join(fails)}" if fails else ""))
    return "\n".join(lines)


def details_md(M):
    P = PROGS
    out = []
    # stack10 extras
    out.append("| stack10 | Box3D | D | F | V4 |\n|---|---|---|---|---|")
    m = {p: M[p]["stack-off"]["m"] for p in P}
    out.append("| sleep off: top drift from the start (mm), sink (mm); the sway, its largest excursion in the first 10 s (mm) | " + " | ".join(
        f"{fmt(m[p]['drift0'] * 1e3)}, {fmt(m[p]['sink'] * 1e3)}; {fmt(m[p]['sway'] * 1e3, 3)}" for p in P) + " |")
    out.append("| sleep off: top yaw, largest over the run (deg); tilt at the end (deg) | " + " | ".join(
        f"{fmt(m[p]['yawmax'], 3)}; {fmt(m[p]['tilt'], 3)}" for p in P) + " |")
    out.append("| sleep off: top cube's largest speed in the last second (mm/s); every cube's mean speed, last 10 s (mm/s) | " + " | ".join(
        f"{fmt(m[p]['speedmax1'] * 1e3, 3)}; {fmt(m[p]['speedall'] * 1e3, 3)}" for p in P) + " |")
    m = {p: M[p]["stack-off"]["log"] for p in P}
    out.append("| sleep off: deepest point over the run; over the last 60 ticks (mm) | " + " | ".join(
        f"{fmt(-m[p]['deep'] * 1e3, 3)}; {fmt(-m[p]['deep_rest'] * 1e3, 3)}" for p in P) + " |")
    m = {p: M[p]["stack-r0"]["m"] for p in P}
    out.append("| sleep off, contact recycling off: top drift from the start (mm); largest speed in the last second (mm/s) | " + " | ".join(
        f"{fmt(m[p]['drift0'] * 1e3, 3)}; {fmt(m[p]['speedmax1'] * 1e3, 3)}" for p in P) + " |")
    m = {p: M[p]["stack-on"]["m"] for p in P}
    out.append("| sleep on: top drift from the start (mm) when asleep | " + " | ".join(fmt(m[p]['drift0'] * 1e3, 3) for p in P) + " |")
    b = M["Box3D"]["stack-on"]["log"].get("box3d_asleep")
    out.append(f"| sleep on: Box3D's own reading (every body asleep after step N) | {fmt(b)} | | | |")
    out.append("")
    # pile200 per seed
    out.append("| pile200 seed | every body asleep from tick: Box3D, D, F, V4 | deepest at rest, every pair at the end (cm) | "
               "deepest in the 60 ticks before every body sleeps (cm) | deepest during the drop (cm) | escapes |\n|---|---|---|---|---|---|")
    for k in SEEDS:
        e = {p: M[p][f"pile-s{k}"] for p in P}
        out.append(f"| {k} | " + ", ".join(fmt(e[p]["log"].get("asleep")) for p in P) + " | " +
                   ", ".join(fmt(-e[p]["log"]["final"] * 100, 3) for p in P) + " | " +
                   ", ".join(fmt(-e[p]["log"]["deep_rest"] * 100, 3) for p in P) + " | " +
                   ", ".join(fmt(-e[p]["log"]["deep"] * 100, 3) for p in P) + " | " +
                   ", ".join(str(len(e[p]["m"]["escapes"])) for p in P) + " |")
    out.append("")
    # what the solve line's "at rest" window catches: the points over 1.5 cm
    out.append("The solve line's \"at rest\" window (the 60 ticks before every body sleeps), its points deeper than 1.5 cm: the "
               "tick, the pair, the faster body's largest speed within 10 ticks of it and the bodies awake at that tick (of 200).\n")
    out.append("| program, seed | deepest (cm) | tick | bodies | speed (m/s) | awake |\n|---|---|---|---|---|---|")
    for p in P:
        for k in SEEDS:
            lg = M[p][f"pile-s{k}"]["log"]
            if -lg["deep_rest"] > 0.015 and "rest_speed" in lg:
                out.append(f"| {p}, {k} | {fmt(-lg['deep_rest'] * 100, 3)} | {lg['deep_rest_tick']} | {lg['deep_rest_bodies'][0]} "
                           f"{lg['deep_rest_bodies'][1]} | {fmt(lg['rest_speed'], 2)} | {lg['rest_awake']} |")
    out.append("")
    # the other scenes
    m = {p: M[p]["ratio"]["m"] for p in P}
    out.append("| other scenes | Box3D | D | F | V4 |\n|---|---|---|---|---|")
    out.append("| ratio: 3 t box's centre, 1 kg box's centre, plate's centre, a chip's centre (m) at tick 1,200 | " + " | ".join(
        f"{fmt(m[p]['big_y'])}, {fmt(m[p]['small_y'])}, {fmt(m[p]['plate_y'])}, {fmt(m[p]['chips_y'][0])}" for p in P) + " |")
    out.append("| ratio: every body asleep from tick | " + " | ".join(fmt(M[p]["ratio"]["log"].get("asleep")) for p in P) + " |")
    m = {p: M[p]["bounce"]["m"] for p in P}
    out.append("| bounce: lowest centre (m); the rise to the apex against the analytic | " + " | ".join(
        f"{fmt(m[p]['lowest'])}; {100 * m[p]['rise']:.1f}%" for p in P) + " |")
    m = {p: M[p]["ramp"]["m"] for p in P}
    out.append("| ramp: friction 0.6 box's speed in the last second (mm/s) | " + " | ".join(fmt(m[p]["hold_speed"] * 1e3, 3) for p in P) + " |")
    c = M["Box3D"]["chip-ccd"]["m"]
    out.append(f"| chip, Box3D with continuous collision on: lowest centre; last centre (m); asleep from tick | "
               f"{fmt(c['lowest'])}; {fmt(c['final_y'])}; {fmt(c['asleepstay'])} | | | |")
    m = {p: M[p]["chip"]["log"] for p in P}
    out.append("| chip: deepest point over the run (mm) | " + " | ".join(fmt(-m[p]["deep"] * 1e3, 3) for p in P) + " |")
    m = {p: M[p]["arm"]["m"] for p in P}
    out.append("| arm: the largest gap's tick, servo joint; limited joint | " + " | ".join(
        f"{m[p][0]['gapAt']}; {m[p][1]['gapAt']}" for p in P) + " |")
    out.append("| arm: the hinge axes' largest misalignment, servo joint; limited joint (mrad) | " + " | ".join(
        f"{fmt(m[p][0]['swing'] * 1e3, 3)}; {fmt(m[p][1]['swing'] * 1e3, 3)}" for p in P) + " |")
    out.append("| arm: the servo's largest lag behind its target (rad) | " + " | ".join(fmt(m[p][0]["track"], 3) for p in P) + " |")
    out.append("| arm: the limited joint's records at a limit (within 1 mrad or beyond), of 1,200; its angles' range (rad) | " + " | ".join(
        f"{m[p][1]['atLimit']}; {fmt(m[p][1]['range'][0], 4)} to {fmt(m[p][1]['range'][1], 4)}" for p in P) + " |")
    out.append("| arm: the gap over ticks 30 to 90 (the sweep, the limited link resting on a limit, before the arm turns), largest (mm): "
               "servo joint; limited joint | " + " | ".join(
                   f"{fmt(m[p][0]['gapSweep'] * 1e3, 3)}; {fmt(m[p][1]['gapSweep'] * 1e3, 3)}" for p in P) + " |")
    out.append("")
    # health of the runs
    hulls = {M["Box3D"][n]["log"].get("hull_differ", 0) for n in M["Box3D"]}
    mass = max(M["Box3D"][n]["log"].get("mass_check", (0, 0))[0] for n in M["Box3D"])
    inertia = max(M["Box3D"][n]["log"].get("mass_check", (0, 0))[1] for n in M["Box3D"])
    sent = all(e["log"]["sentinel"] for p in ("D", "F", "V4", "Dp") for e in M[p].values())
    out.append(f"Box3D's hulls: vertex, face and half-edge counts differ from the scene's on {max(hulls)} hulls in any run. Box3D's own mass "
               f"from the shapes against the scene's: within {mass:.2g} (mass) and {inertia:.2g} (inertia, relative); every run sets "
               f"the scene's values. The twins' floating-point sentinel: {'clean in every run' if sent else 'TRIPPED in some run'}.")
    return "\n".join(out)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--compiler", default="msvc")
    p.add_argument("--out", default=os.path.join(LAB, "build", "toy-scratch", "4b"))
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--no-run", action="store_true")
    p.add_argument("--md", default=os.path.join(TOY, "results", "decision1.md"))
    a = p.parse_args()
    a.out = os.path.abspath(a.out)
    os.makedirs(a.out, exist_ok=True)
    if not a.no_run:
        bad = run_all(a)
        if bad:
            sys.exit(1)
    M = measure(a)
    rows = build_table(M)
    text = ["# Decision point 1: the acceptance table", "",
            "Generated by `lab/gpu/toy/accept.py` (step 4b, rerun after step 5a's two corrections; DESIGN.md \"Scenes and acceptance\", NOTES.md steps 4b and 5). Values: Box3D "
            "(`b3ref2`, Box3D's own solver on the toy's scenes), the toy's twins D (double), F (float) and V4 (block-scaled int32), "
            f"{a.compiler}, one thread. The last four columns: PASS or FAIL of each program against the threshold (the decision "
            "is F's and V4's; Box3D's and D's verdicts say whether the bar is fair). Each FAIL's diagnosis, the judgement calls "
            "behind the measurements and V4's rounding: NOTES.md, steps 4b and 5. Step 5a: V4's mas rounds halves away from "
            "zero, and every scene starts from values every program holds exactly (floats on V4's grids).", "",
            "Reproduce (Windows, from the repository's root):", "",
            "```",
            "python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc",
            "python lab/gpu/toy/accept.py        # about a minute; trajectories and logs in lab/gpu/build/toy-scratch/4b",
            "```", "",
            "## Summary", "", summary(rows), "",
            "## The table", "", table_md(rows), "",
            "## Details", "", details_md(M), "",
            "## How each number is measured", "",
            "- Trajectories (traj.h's LPTRAJ1) record every tick (every 10th for pile200's 3,600-tick runs); \"asleep\" means not "
            "stepped this tick, in both programs (Box3D: asleep before the step and after it; an island Box3D puts to sleep at the "
            "end of step t was stepped in t, the toy's is not stepped in t + 1, so both read t + 1). The pile200 sleep ticks come "
            "from each program's solve line (every tick).",
            "- stack10: the top cube (body 10). Drift: horizontal distance between its position at tick 60 (the brief's \"settled "
            "position after the first second\"; the stack still sways there, up to 6 to 7 mm, for about 5 s) or at tick 600 (the "
            "sway gone: what is left is creep) and at tick 3,600. Yaw: its rotation about y at tick 3,600 against the start. "
            "Resting speed: the mean of its linear speed over the last 600 ticks.",
            "- pile200: escapes are dynamic bodies whose centre is ever outside |x|, |z| <= 2 m (the walls' inner faces) or below "
            "the floor's top (y < 0) in any record. Penetration: the deepest manifold point (the narrowphase's separation at the "
            "start of a tick, before the solve; the same quantity in both programs). At rest (judged): over every touching pair "
            "after tick 3,600, when every body sleeps (the sleeping pairs' manifolds as they fell asleep; the rest line). The "
            "solve line's two (reported): among the pairs with a body stepped that tick, over the run (\"during the drop\") and "
            "over the 60 ticks before every body sleeps; that window is not rest (the pile sleeps as one island, so 195 to 198 of "
            "200 bodies are still awake in it, and its deepest points are late slides and falls at 0.1 to 4 m/s).",
            "- ratio: outcome classes from the last record (metrics.py's `ratio`): the 3 t box rests (centre >= 0.95 m), is partly "
            "sunk, or ends on the ground (<= 0.55 m) crushed through the 1 kg box (still under it) or having pushed it aside; the "
            "plate rests (centre >= 0.04 m), is partly sunk, or sinks through the chips (<= 0.03 m), or the chips are squeezed out.",
            "- bounce: the highest centre after the lowest one, against 0.25 + 0.5^2 x 1.75 = 0.6875 m.",
            "- ramp: travel along the slope (25 degrees). The 0.6 box holds when it travels at most 1 cm by tick 600 and at most "
            "1 mm after tick 60 (it starts 1 mm above the ramp); the 0.2 box's distance at tick 180 (3 s, still on the 6.5 m "
            "ramp) against Box3D's.",
            "- chip: no tunnelling when its lowest centre over the run and its last centre are above the ground (y > 0 and "
            "> 5 mm); it sleeps when every record from some tick on is asleep and it never wakes after first sleeping.",
            "- Position rms against D over ticks 1 to 120: the dynamic bodies' centres, as toy.c's --pos-ref. D moved 1e-9 m: D "
            "with every dynamic body's start moved 1e-9 m along x and along y (`--perturb`), against D: how much the scene "
            "amplifies a tiny difference. Every program starts from the same values (step 5a), so F's and V4's differences are "
            "their arithmetic's.",
            ""]
    md = "\n".join(text)
    os.makedirs(os.path.dirname(a.md), exist_ok=True)
    with open(a.md, "w", newline="\n") as f:
        f.write(md)
    print(md)
    print(f"\nwritten {a.md}")


if __name__ == "__main__":
    main()
