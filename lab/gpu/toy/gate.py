#!/usr/bin/env python3
"""gate.py: the toy's gates 1 to 5 (NOTES.md) on this machine, from the binaries of `lab.py build`.

  python lab/gpu/toy/gate.py [--compilers msvc,clang-cl] [--ticks 600] [--out lab/gpu/build/toy-scratch/gate] [--gates 1,2,3,4,5]
                             [--gpus MASK] [--expect-gpus nvidia,intel | N] [--no-gpu] [--write-ref] [--jobs 2]
  python3 lab/gpu/toy/gate.py --compilers gcc,clang --gates 5       # another machine, against the committed reference files
  python lab/gpu/toy/gate.py --compilers clang-cl-ubsan --gates 1,2,3,4 --no-gpu   # the twins under UBSan (lab.py build --ubsan)

Gate 1: each dialect's battery on every GPU and the twin of each compiler: every word identical, lpDivQ31 equal to C's
division, the battery hashes equal across compilers. Gate 2: stack10 and pile200 (and pile200 without gravity, one body
pushed: sleep and wake), chip, ramp, bounce, ratio and arm (step 6's joints) per dialect and compiler at 1 and 8 threads:
per-tick hashes (every category) equal across thread counts and compilers, the floating-point sentinel clean, V4's
saturation counters at 0; F's and V4's positions and orientations against D's (metrics.py's rms on the first
compiler's trajectories). Gate 3: the narrowphase's corpus (narrow_<d>): D's results as the reference, F and V4 pick D's
axis and feature ids on every pair outside ties (both passes: fresh, and cached with warm starts), the corpus hashes
equal across compilers; the corpus on every GPU against the twin is reported (desired, not required).
Gate 4: accept.py's runs (decision point 1's: every scene and seed, Box3D beside the twins) on the first compiler's twins
at 1 and 8 threads: every run must exit cleanly (threads agree, the sentinel clean), saturations 0; the acceptance
table's counts and each scene's numbers are printed (accept.py writes results/decision1.md; the gate does not judge it).
Gate 5 (decision point 2): the whole tick on every GPU in --gpus beside the twin of the first compiler, F and V4:
stack10 for 3,600 ticks with sleep off and on, pile200 over seeds 1 to 8 for 1,200 ticks, bounce, ramp, ratio and chip
for 600, the arm for 1,200 with sleep on and off: every tick's hashes (every category: bodies, manifolds, stages, the
bodies' prepared data, the pairs' working records, joints) identical to the twin's, saturations 0 (as the twin's); every
compiler's twins (F, V4 and D, 1 and 8 threads) identical to the reference files in toy/results/ref; and the full-buffer
trace (toy --trace: every buffer the kernels write, word by word, after every tick) over pile200 seed 1 for 120 ticks
and the arm for 300 on every GPU in both dialects, failing on any differing word. A GPU that differs in a run is traced
to its first differing word. The GPUs' costs are reported.

No vacuous pass: gates 1 and 5 fail unless the GPUs --expect-gpus names ran (vendor names, or a count; the default is
nvidia,intel on the Windows laptop of NOTES.md and 1 elsewhere); --no-gpu runs the twins alone (and expects none).
--write-ref (gate 5) rewrites the reference files: it needs two compilers or more, writes the first's into a scratch
directory, checks every other compiler's twins against them, and copies them into toy/results/ref only when every config
of every compiler agrees. Without it, gate 5 only checks.
"""
import argparse
import os
import platform
import re
import shutil
import subprocess
import sys

TOY = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.dirname(TOY)
EXE = ".exe" if sys.platform == "win32" else ""
SCENES = [("stack10", []), ("pile200", []), ("pile200-wake", ["--gravity", "0", "--push", "112,1.5,0,0.5"]), ("chip", []), ("ramp", []),
          ("bounce", []), ("ratio", []), ("arm", [])]
# gate 5: name (the reference file's), scene, ticks, extra arguments
GATE5 = [("stack10-sleep0", "stack10", 3600, ["--sleep", "0"]), ("stack10", "stack10", 3600, [])] + \
        [(f"pile200-s{k}", "pile200", 1200, ["--seed", str(k)]) for k in range(1, 9)] + \
        [(s, s, 600, []) for s in ("bounce", "ramp", "ratio", "chip")] + \
        [("arm", "arm", 1200, []), ("arm-sleep0", "arm", 1200, ["--sleep", "0"])]
# gate 5's mandatory full-buffer traces: name, scene, ticks, extra arguments
TRACES = [("pile200-s1", "pile200", 120, ["--seed", "1"]), ("arm", "arm", 300, [])]
REF = os.path.join(TOY, "results", "ref")
# the final line's hashes (a scene with joints adds its joints' hash)
FINAL = (r"run hash ([0-9a-f]+), final bodies ([0-9a-f]+) manifolds ([0-9a-f]+) stages ([0-9a-f]+) bodyprep ([0-9a-f]+) pairwork ([0-9a-f]+)"
         r"(?: joints [0-9a-f]+)?, threads (\w+)")
# the GPUs each machine must run (gates 1 and 5), by host name; elsewhere at least one
EXPECTED_GPUS = {"desktop-j0pimak": "nvidia,intel"}  # the Windows laptop: RTX 3060 Laptop and UHD 630


def run(exe, args, log):
    r = subprocess.run([exe, *args, "--log", log], cwd=LAB, capture_output=True, text=True)
    return r.returncode, open(log, errors="replace").read() if os.path.exists(log) else r.stdout + r.stderr


def gpu_vendors(text):
    """The GPUs a log says ran: (index, vendor) from its 'gpu N: name (vendor), driver' lines."""
    return re.findall(r"^gpu (\d+): .*? \((\w+)\), driver", text, re.M)


def expect_problem(a, ran, what):
    """None when the GPUs that ran (vendor names) meet --expect-gpus, else the problem."""
    if a.no_gpu:
        return f"{what}: GPUs ran under --no-gpu" if ran else None
    spec = a.expect_gpus.strip().lower()
    if spec.isdigit():
        return None if len(ran) >= int(spec) else f"{what}: {len(ran)} GPU(s) ran, expected at least {spec}"
    missing = [v for v in spec.split(",") if v and v not in [x.lower() for x in ran]]
    return f"{what}: no {', '.join(missing)} GPU ran (ran: {', '.join(ran) or 'none'})" if missing else None


def gpu_args(a):
    return ["--gpus", "0"] if a.no_gpu else ["--gpus", a.gpus]


def gate1(a, comps, bad):
    print("gate 1: the battery")
    hashes = {}
    for d in ("F", "V4", "D"):
        for c in comps:
            exe = os.path.join(LAB, "build", c, "bin", f"battery_{d}{EXE}")
            log = os.path.join(a.out, f"battery_{d}.{c}.txt")
            code, text = run(exe, ["--no-gpu"] if a.no_gpu else ["--gpus", a.gpus], log)
            h = re.search(r"^battery \S+ hash ([0-9a-f]+)", text, re.M)
            helpers = re.findall(r"^(helper .*)$", text, re.M)
            div = re.search(r"^lpDivQ31 vs C's '/': (\d+) inputs, (\d+) differ", text, re.M)
            ran = gpu_vendors(text)
            hashes.setdefault(d, {})[c] = (h.group(1) if h else None, [re.sub(r"\s+\| .*", "", x) for x in helpers])
            line = f"  battery {d:2} {c:9} hash {h.group(1) if h else '?'}  exit {code}  GPUs {', '.join(v for _, v in ran) or '-'}"
            if div:
                line += f"; lpDivQ31 vs C: {div.group(2)} of {div.group(1)} differ"
            print(line)
            if code != 0 or not h:
                bad.append(f"battery {d} {c}: exit {code}")
            if d != "D":
                p = expect_problem(a, [v for _, v in ran], f"battery {d} {c}")
                if p:
                    bad.append(p)
        hs = {x[0] for x in hashes[d].values()}
        hl = {tuple(x[1]) for x in hashes[d].values()}
        if len(hs) != 1 or len(hl) != 1:
            bad.append(f"battery {d}: compilers differ")
        print(f"  battery {d}: {'same hash and helper lines on ' + ', '.join(comps) if len(hs) == 1 and len(hl) == 1 else 'COMPILERS DIFFER'}")


def gate2(a, comps, bad):
    print("gate 2: the twin driver")
    import metrics  # noqa: E402
    from traj import Traj  # noqa: E402
    ticks = int(a.ticks)
    windows = [(lo, min(hi, ticks)) for lo, hi in ((1, 120), (121, 600), (1, 600)) if lo <= min(hi, ticks)]
    for name, extra in SCENES:
        scene = name.split("-")[0]
        runs = {}
        trajs = {}
        for d in ("D", "F", "V4"):
            for c in comps:
                exe = os.path.join(LAB, "build", c, "bin", f"toy_{d}{EXE}")
                log = os.path.join(a.out, f"toy_{d}.{name}.{c}.txt")
                args = ["--scene", scene, "--ticks", a.ticks, "--threads", "1,8", "--quiet", *extra]
                if c == comps[0]:  # the first compiler's trajectories (every tick) for the dialects' comparison
                    trajs[d] = os.path.join(a.out, f"{name}.{d}.traj")
                    args += ["--traj", trajs[d]]
                code, text = run(exe, args, log)
                m = re.search(FINAL, text)
                ticklines = re.findall(r"^ *\d+  [0-9a-f]{16}  [0-9a-f]{16}  [0-9a-f]{16} .*$", text, re.M)
                runs[(d, c)] = (m.group(1) if m else None, ticklines)
                last = ticklines[-1].split() if ticklines else []
                touching = max((int(t.split()[7]) for t in ticklines), default=0)
                line = (f"  {name:12} {d:2} {c:9} run hash {m.group(1) if m else '?'} threads {m.group(7) if m else '?'} exit {code}; "
                        f"last tick: awake {last[4] if last else '?'} pairs {last[5] if last else '?'} touching {last[7] if last else '?'}"
                        f" (most {touching})")
                if c == comps[0] and d != "D" and os.path.exists(trajs[d]) and os.path.exists(trajs["D"]):
                    t, ref = Traj(trajs[d]), Traj(trajs["D"])
                    for lo, hi in windows:
                        r = metrics.rms(t, ref, lo, hi)
                        line += (f"; vs D {lo}-{hi}: pos rms {r['rms']:.3g} max {r['max']:.3g} (tick {r['at']}, body {r['body']}), "
                                 f"rot rms {r['rot_rms']:.3g} max {r['rot_max']:.3g}")
                print(line)
                if code != 0 or not m or m.group(7) != "agree":
                    bad.append(f"toy {d} {name} {c}: exit {code}")
                sat = re.search(r"saturations (\d+) \(twin", text)
                if not sat or int(sat.group(1)) != 0:
                    bad.append(f"toy {d} {name} {c}: saturations {sat.group(1) if sat else '?'}")
            same = len({runs[(d, c)][0] for c in comps}) == 1 and len({tuple(runs[(d, c)][1]) for c in comps}) == 1
            print(f"  {name:12} {d:2} per-tick hashes {'identical on ' + ', '.join(comps) if same else 'DIFFER between compilers'}")
            if not same:
                bad.append(f"toy {d} {name}: compilers differ")
        for f in trajs.values():  # 600 ticks of pile200 every tick: 14 MB each
            if os.path.exists(f):
                os.remove(f)


def gate3(a, comps, bad):
    print("gate 3: the narrowphase corpus")
    ref = os.path.join(a.out, "narrow.D.bin")
    hashes = {}
    for d in ("D", "F", "V4"):
        for c in comps:
            exe = os.path.join(LAB, "build", c, "bin", f"narrow_{d}{EXE}")
            log = os.path.join(a.out, f"narrow_{d}.{c}.txt")
            args = ["--out", ref] if d == "D" and c == comps[0] else ["--ref", ref] if d != "D" else []
            if c != comps[0] or a.no_gpu:
                args += ["--gpus", "0"]  # the GPUs once, against the first compiler's twin (the twins agree)
            code, text = run(exe, args, log)
            h = re.search(r"^narrow \S+ corpus hash ([0-9a-f]+)", text, re.M)
            hashes.setdefault(d, set()).add(h.group(1) if h else None)
            sentinel = re.findall(r"^twin pass \d: saturations (\d+), floating-point sentinel (\S+)", text, re.M)
            print(f"  narrow {d:2} {c:9} corpus hash {h.group(1) if h else '?'} exit {code}; twin passes: "
                  + ", ".join(f"saturations {s} sentinel {f}" for s, f in sentinel))
            if c == comps[0]:  # the other compilers' twins give the same words (the corpus hash)
                for line in re.findall(r"^(  pass \d: .*|    ties .*|    GATE .*|    errors .*)$", text, re.M):
                    print("    " + line.strip())
                for line in re.findall(r"^(  \S+ pass \d: SAT records .*)$", text, re.M):
                    print("    gpu " + line.strip())
            if code != 0 or not h:
                bad.append(f"narrow {d} {c}: exit {code}")
        same = len(hashes[d]) == 1
        print(f"  narrow {d}: {'same corpus hash on ' + ', '.join(comps) if same else 'COMPILERS DIFFER'}")
        if not same:
            bad.append(f"narrow {d}: compilers differ")


def gate4(a, comps, bad):
    print("gate 4: decision point 1's runs (accept.py) on the twins of " + comps[0] + ", 1 and 8 threads")
    import accept  # noqa: E402
    problems, rows, M = accept.gate(comps[0], os.path.join(a.out, "accept"), a.jobs, "1,8")
    print(accept.summary(rows))
    for line in accept.scene_lines(M):
        print("  " + line)
    for p in problems:
        print("  PROBLEM: " + p)
    bad += [f"gate 4: {p}" for p in problems]


def gate5_run(a, c, d, name, scene, ticks, extra, refArgs, gpus):
    exe = os.path.join(LAB, "build", c, "bin", f"toy_{d}{EXE}")
    log = os.path.join(a.out, f"g5_{d}.{name}.{c}.txt")
    args = ["--scene", scene, "--ticks", str(ticks), "--threads", "1,8", "--quiet", *extra, *refArgs]
    if gpus and d != "D":
        args += gpu_args(a)
    return exe, run(exe, args, log)


def gate5(a, comps, bad):
    print("gate 5: decision point 2 (the whole tick on the GPUs against the twin; the twins against the reference files; the full-buffer traces)")
    newRef = os.path.join(a.out, "ref-new")
    if a.write_ref:
        if len(comps) < 2:
            bad.append("gate 5 --write-ref: needs two compilers or more (the second's twins must agree before a file is written)")
            print("  --write-ref needs two compilers or more: nothing run")
            return
        os.makedirs(newRef, exist_ok=True)
    gpu = {}  # (gpu name, dialect) -> [identical, runs]
    costs = []
    verified = []  # --write-ref: (new file, reference file) once every other compiler agreed
    refBad = 0
    for name, scene, ticks, extra in GATE5:
        for d in ("F", "V4", "D"):
            ref = os.path.join(REF, f"{name}.{d}.txt")
            new = os.path.join(newRef, f"{name}.{d}.txt")
            agreed = True
            for c in comps:
                first = c == comps[0]
                refArgs = (["--ref-out", new] if first else ["--ref", new]) if a.write_ref else ["--ref", ref]
                exe, (code, text) = gate5_run(a, c, d, name, scene, ticks, extra, refArgs, first)
                m = re.search(r"run hash ([0-9a-f]+).*?threads (\w+), saturations (\d+)", text)
                refs = re.findall(r"^ref: (.*?) (identical to the reference.*|DIFFERS.*|the reference file .*)$", text, re.M)
                gpus = re.findall(r"^gpu (\d+) (\w+): per-tick hashes (identical to the twin's|DIFFER from the twin's)(.*?); run hash \w+; "
                                  r"saturations (\d+) \(twin (\d+)\)$", text, re.M)
                written = re.search(r"^ref-out \S+: (.*)$", text, re.M)
                line = f"  {name:14} {d:2} {c:9} run hash {m.group(1) if m else '?'} threads {m.group(2) if m else '?'} saturations " \
                       f"{m.group(3) if m else '?'} exit {code}"
                if written:
                    line += f"; new reference {written.group(1)}"
                    agreed &= written.group(1) == "written"
                twinRefs = [r for r in refs if r[0].startswith("twin")]
                if twinRefs:
                    ok = all(r[1].startswith("identical") for r in twinRefs)
                    line += f"; twins vs {'the first compiler' if a.write_ref else 'reference'}: {'identical' if ok else 'DIFFER'}"
                    agreed &= ok
                    if not ok:
                        refBad += 1
                        bad.append(f"gate 5 {name} {d} {c}: a twin differs from the {'first compiler' if a.write_ref else 'reference'} "
                                   f"({'; '.join(r[1] for r in twinRefs if not r[1].startswith('identical'))})")
                elif not written:
                    agreed = False
                    bad.append(f"gate 5 {name} {d} {c}: no reference check")
                for gi, vendor, verdict, where, sat, twinSat in gpus:
                    same = verdict.startswith("identical") and sat == twinSat == "0"
                    gref = [r for r in refs if r[0].startswith(f"gpu {gi}")]
                    line += f"; gpu {gi} {vendor}: {'identical' if same else 'DIFFERS' + where}" + (f", saturations {sat}" if sat != "0" else "")
                    g = gpu.setdefault((f"{gi} {vendor}", d), [0, 0])
                    g[0] += same
                    g[1] += 1
                    if not same:
                        bad.append(f"gate 5 {name} {d} gpu {gi} {vendor}: {verdict}{where}, saturations {sat} (twin {twinSat})")
                        t = re.search(r"from tick (\d+)", where)
                        if t:  # trace it to the first differing word
                            tlog = os.path.join(a.out, f"g5_{d}.{name}.gpu{gi}.trace.txt")
                            targs = ["--scene", scene, "--ticks", t.group(1), "--threads", "1", "--quiet", *extra, "--gpu", gi,
                                     "--trace", t.group(1)]
                            _, ttext = run(exe, targs, tlog)
                            for tl in re.findall(r"^(trace .*|    .*)$", ttext, re.M)[:12]:
                                print("      " + tl.strip())
                    if gref and not all(r[1].startswith("identical") for r in gref):
                        bad.append(f"gate 5 {name} {d} gpu {gi} {vendor}: differs from the reference")
                if first and d != "D":
                    p = expect_problem(a, [v for _, v, _, _, _, _ in gpus], f"gate 5 {name} {d}")
                    if p:
                        bad.append(p)
                cm = re.findall(r"^gpu (\d+) costs: wall (\S+) ms/tick .*?kernels (\S+) ms/tick; the busiest tick(?: after the first)? \((\d+)\): kernels (\S+) ms, "
                                r"wall (\S+) ms; at most (\d+) dispatches", text, re.M)
                for gi, wall, kern, pt, pk, pw, nd in cm:
                    costs.append((name, d, gi, wall, kern, pt, pk, pw, nd))
                print(line)
                if code != 0 or not m or m.group(2) != "agree" or m.group(3) != "0":
                    agreed = False
                    bad.append(f"gate 5 {name} {d} {c}: exit {code}, threads {m.group(2) if m else '?'}, saturations {m.group(3) if m else '?'}")
            if a.write_ref and agreed:
                verified.append((new, ref))
    if a.write_ref:
        total = len(GATE5) * 3
        if len(verified) == total and refBad == 0:
            os.makedirs(REF, exist_ok=True)
            for new, ref in verified:
                shutil.copyfile(new, ref)
            print(f"  --write-ref: {total} reference files written to {os.path.relpath(REF, LAB)} (the first compiler's, every other compiler's "
                  f"twins identical)")
        else:
            bad.append(f"gate 5 --write-ref: {total - len(verified)} of {total} configs not verified on every compiler: no reference file written")
            print(f"  --write-ref: NOTHING WRITTEN ({total - len(verified)} of {total} configs not verified on every compiler)")
    print("  decision point 2, per GPU and dialect: runs whose every tick's hashes equal the twin's (saturations 0)")
    for (g, d), (same, total) in sorted(gpu.items()):
        print(f"    gpu {g:12} {d:2}: {same} of {total}")
    # the full-buffer traces: every buffer the kernels write, word by word, after every tick (toy --trace)
    if a.no_gpu:
        print("  full-buffer traces: skipped (--no-gpu)")
    else:
        print("  full-buffer traces (every buffer the kernels write, word by word, after every tick's prepare and after its end):")
        for name, scene, ticks, extra in TRACES:
            for d in ("F", "V4"):
                exe = os.path.join(LAB, "build", comps[0], "bin", f"toy_{d}{EXE}")
                log = os.path.join(a.out, f"trace_{d}.{name}.txt")
                code, text = run(exe, ["--scene", scene, "--ticks", str(ticks), "--threads", "1", "--quiet", *extra, *gpu_args(a),
                                       "--trace", str(ticks)], log)
                ran = gpu_vendors(text)
                clean = dict(re.findall(r"^trace gpu (\d+): no difference in (\d+) ticks", text, re.M))
                verdicts = []
                for gi, vendor in ran:
                    ok = clean.get(gi) == str(ticks)
                    verdicts.append(f"gpu {gi} {vendor}: {'no difference in ' + str(ticks) + ' ticks' if ok else 'DIFFERS'}")
                    if not ok:
                        bad.append(f"gate 5 trace {name} {d} gpu {gi} {vendor}: a buffer differs ({os.path.relpath(log, LAB)})")
                        for tl in re.findall(r"^(trace .*|    .*)$", text, re.M)[:12]:
                            print("      " + tl.strip())
                print(f"    {name:12} {d:2} {ticks} ticks, exit {code}: " + ("; ".join(verdicts) or "no GPU ran"))
                if code != 0:
                    bad.append(f"gate 5 trace {name} {d}: exit {code}")
                p = expect_problem(a, [v for _, v in ran], f"gate 5 trace {name} {d}")
                if p:
                    bad.append(p)
    if costs:
        print("  GPU costs (informative): scene, dialect, gpu: wall and kernel ms per tick over the run; the busiest tick after the first: tick, kernels, "
              "wall (ms); the most dispatches in a tick")
        for name, d, gi, wall, kern, pt, pk, pw, nd in costs:
            print(f"    {name:14} {d:2} gpu {gi}: {wall:>7} {kern:>7}; tick {pt:>4}: {pk:>7} {pw:>7}; {nd}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--compilers", default="msvc,clang-cl")
    p.add_argument("--ticks", default="600")
    p.add_argument("--out", default=os.path.join(LAB, "build", "toy-scratch", "gate"))
    p.add_argument("--gates", default="1,2,3,4,5")
    p.add_argument("--gpus", default="0xff", help="gates 1, 3 and 5: the GPUs to run (a mask)")
    p.add_argument("--expect-gpus", default=EXPECTED_GPUS.get(platform.node().split(".")[0].lower(), "1"),
                   help="gates 1 and 5 fail unless these GPUs ran: vendor names (nvidia,intel) or a count (default: this machine's)")
    p.add_argument("--no-gpu", action="store_true", help="the twins alone: no GPU runs, none expected")
    p.add_argument("--write-ref", action="store_true", help="gate 5: rewrite the reference files (two compilers or more must agree)")
    p.add_argument("--jobs", type=int, default=2, help="gate 4: runs at a time (each run's twin uses 8 threads)")
    a = p.parse_args()
    a.out = os.path.abspath(a.out)  # the binaries run from lab/gpu
    os.makedirs(a.out, exist_ok=True)
    sys.path.insert(0, TOY)
    comps = a.compilers.split(",")
    gates = a.gates.split(",")
    print(f"gate.py: compilers {', '.join(comps)}; GPUs {'none (--no-gpu)' if a.no_gpu else 'mask ' + a.gpus + ', expected ' + a.expect_gpus}", flush=True)
    bad = []
    if "1" in gates:
        gate1(a, comps, bad)
    if "2" in gates:
        gate2(a, comps, bad)
    if "3" in gates:
        gate3(a, comps, bad)
    if "4" in gates:
        gate4(a, comps, bad)
    if "5" in gates:
        gate5(a, comps, bad)
    print("\n" + ("PASS" if not bad else "FAIL:\n  " + "\n  ".join(bad)))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
