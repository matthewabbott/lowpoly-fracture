#!/usr/bin/env python3
"""gate.py: the toy's gates 1 to 5 (NOTES.md) on this machine, from the binaries of `lab.py build`.

  python lab/gpu/toy/gate.py [--compilers msvc,clang-cl] [--ticks 600] [--out lab/gpu/build/toy-scratch/gate] [--gates 1,2,3,4,5]
                             [--gpus MASK] [--write-ref]
  python3 lab/gpu/toy/gate.py --compilers gcc,clang --gates 5       # another machine, against the committed reference files

Gate 1: each dialect's battery on every GPU and the twin of each compiler: every word identical, lpDivQ31 equal to C's
division, the battery hashes equal across compilers. Gate 2: stack10 and pile200 (and pile200 without gravity, one body
pushed: sleep and wake), chip, ramp, bounce and ratio per dialect and compiler at 1 and 8 threads: per-tick hashes
(bodies, manifolds, stages) equal across thread counts and compilers, the floating-point sentinel clean, V4's saturation
counters at 0; F and V4 positions against D's.
Gate 3: the narrowphase's corpus (narrow_<d>): D's results as the reference, F and V4 pick D's axis and feature ids on
every pair outside ties (both passes: fresh, and cached with warm starts), the corpus hashes equal across compilers;
the corpus on every GPU against the twin is reported (desired, not required).
Gate 4: the contact solve's physical sanity on the twins of the first compiler, every dialect, 1 and 8 threads (the
threads must agree, the sentinel stay clean and V4's saturations at 0; the numbers are reported, step 4b judges them
against Box3D): stack10 for 3,600 ticks with sleep off and on, pile200 over eight seeds, bounce, ramp, ratio and chip,
each through its trajectory (traj.py's summaries).
Gate 5 (decision point 2): the whole tick on every GPU in --gpus (all by default) beside the twin of the first compiler,
F and V4: stack10 for 3,600 ticks with sleep off and on, pile200 over seeds 1 to 8 for 1,200 ticks, bounce, ramp, ratio
and chip for 600: every tick's hashes identical to the twin's, saturations 0 (as the twin's); and every compiler's twins
(F, V4 and D, 1 and 8 threads) identical to the reference files in toy/results/ref (written by the Windows MSVC twin:
--write-ref, first compiler). A GPU that differs is traced (toy --trace) to its first differing word. The GPUs' costs
are reported.
"""
import argparse
import os
import re
import subprocess
import sys

LAB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = ".exe" if sys.platform == "win32" else ""
SCENES = [("stack10", []), ("pile200", []), ("pile200-wake", ["--gravity", "0", "--push", "112,1.5,0,0.5"]), ("chip", []), ("ramp", []),
          ("bounce", []), ("ratio", [])]
# gate 5: name (the reference file's), scene, ticks, extra arguments
GATE5 = [("stack10-sleep0", "stack10", 3600, ["--sleep", "0"]), ("stack10", "stack10", 3600, [])] + \
        [(f"pile200-s{k}", "pile200", 1200, ["--seed", str(k)]) for k in range(1, 9)] + \
        [(s, s, 600, []) for s in ("bounce", "ramp", "ratio", "chip")]
REF = os.path.join(LAB, "toy", "results", "ref")
# gate 4: name, scene, ticks, extra arguments
SANITY = [("stack10-awake", "stack10", 3600, ["--sleep", "0"]), ("stack10-sleep", "stack10", 3600, [])] + \
         [(f"pile200-s{k}", "pile200", 3600, ["--seed", str(k)]) for k in range(1, 9)] + \
         [("bounce", "bounce", 300, []), ("ramp", "ramp", 600, []), ("ratio", "ratio", 1200, []), ("chip", "chip", 600, [])]


def run(exe, args, log):
    r = subprocess.run([exe, *args, "--log", log], cwd=LAB, capture_output=True, text=True)
    return r.returncode, open(log, errors="replace").read() if os.path.exists(log) else r.stdout + r.stderr


def gate1(a, comps, bad):
    print("gate 1: the battery")
    hashes = {}
    for d in ("F", "V4", "D"):
        for c in comps:
            exe = os.path.join(LAB, "build", c, "bin", f"battery_{d}{EXE}")
            log = os.path.join(a.out, f"battery_{d}.{c}.txt")
            code, text = run(exe, [], log)
            h = re.search(r"^battery \S+ hash ([0-9a-f]+)", text, re.M)
            helpers = re.findall(r"^(helper .*)$", text, re.M)
            div = re.search(r"^lpDivQ31 vs C's '/': (\d+) inputs, (\d+) differ", text, re.M)
            gpus = re.findall(r"^gpu \d+: (.*?) \(", text, re.M)
            hashes.setdefault(d, {})[c] = (h.group(1) if h else None, [re.sub(r"\s+\| .*", "", x) for x in helpers])
            line = f"  battery {d:2} {c:9} hash {h.group(1) if h else '?'}  exit {code}  GPUs {', '.join(gpus) or '-'}"
            if div:
                line += f"; lpDivQ31 vs C: {div.group(2)} of {div.group(1)} differ"
            print(line)
            if code != 0 or not h:
                bad.append(f"battery {d} {c}: exit {code}")
        hs = {x[0] for x in hashes[d].values()}
        hl = {tuple(x[1]) for x in hashes[d].values()}
        if len(hs) != 1 or len(hl) != 1:
            bad.append(f"battery {d}: compilers differ")
        print(f"  battery {d}: {'same hash and helper lines on ' + ', '.join(comps) if len(hs) == 1 and len(hl) == 1 else 'COMPILERS DIFFER'}")


def gate2(a, comps, bad):
    print("gate 2: the twin driver")
    for name, extra in SCENES:
        scene = name.split("-")[0]
        ref = os.path.join(a.out, f"{name}.D.pos")
        runs = {}
        for d in ("D", "F", "V4"):
            for c in comps:
                exe = os.path.join(LAB, "build", c, "bin", f"toy_{d}{EXE}")
                log = os.path.join(a.out, f"toy_{d}.{name}.{c}.txt")
                args = ["--scene", scene, "--ticks", a.ticks, "--threads", "1,8", "--quiet", *extra]
                if d == "D" and c == comps[0]:
                    args += ["--pos-out", ref]
                elif d != "D":
                    args += ["--pos-ref", ref]
                code, text = run(exe, args, log)
                m = re.search(r"run hash ([0-9a-f]+), final bodies ([0-9a-f]+) manifolds ([0-9a-f]+) stages ([0-9a-f]+), threads (\w+)", text)
                ticks = re.findall(r"^ *\d+  [0-9a-f]{16}  [0-9a-f]{16}  [0-9a-f]{16} .*$", text, re.M)
                runs[(d, c)] = (m.group(1) if m else None, ticks)
                pos = re.findall(r"pose vs \S+, ticks (\d+-\d+): position rms (\S+) m, max (\S+) m.*?rotation rms (\S+) rad, max (\S+) rad", text)
                last = ticks[-1].split() if ticks else []
                touching = max((int(t.split()[7]) for t in ticks), default=0)
                print(f"  {name:12} {d:2} {c:9} run hash {m.group(1) if m else '?'} threads {m.group(5) if m else '?'} exit {code}; "
                      f"last tick: awake {last[4] if last else '?'} pairs {last[5] if last else '?'} touching {last[7] if last else '?'}"
                      f" (most {touching})"
                      + "".join(f"; vs D {w}: pos rms {r} max {x}, rot rms {rr} max {rx}" for w, r, x, rr, rx in pos))
                if code != 0 or not m or m.group(5) != "agree":
                    bad.append(f"toy {d} {name} {c}: exit {code}")
                sat = re.search(r"saturations (\d+) \(twin", text)
                if not sat or int(sat.group(1)) != 0:
                    bad.append(f"toy {d} {name} {c}: saturations {sat.group(1) if sat else '?'}")
            same = len({runs[(d, c)][0] for c in comps}) == 1 and len({tuple(runs[(d, c)][1]) for c in comps}) == 1
            print(f"  {name:12} {d:2} per-tick hashes {'identical on ' + ', '.join(comps) if same else 'DIFFER between compilers'}")
            if not same:
                bad.append(f"toy {d} {name}: compilers differ")


def gate3(a, comps, bad):
    print("gate 3: the narrowphase corpus")
    ref = os.path.join(a.out, "narrow.D.bin")
    hashes = {}
    for d in ("D", "F", "V4"):
        for c in comps:
            exe = os.path.join(LAB, "build", c, "bin", f"narrow_{d}{EXE}")
            log = os.path.join(a.out, f"narrow_{d}.{c}.txt")
            args = ["--out", ref] if d == "D" and c == comps[0] else ["--ref", ref] if d != "D" else []
            if c != comps[0]:
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
    print("gate 4: the contact solve's sanity (twins, " + comps[0] + ")")
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import traj  # noqa: E402
    import contextlib
    import io
    for name, scene, ticks, extra in SANITY:
        for d in ("F", "V4", "D"):
            exe = os.path.join(LAB, "build", comps[0], "bin", f"toy_{d}{EXE}")
            log = os.path.join(a.out, f"sanity_{d}.{name}.txt")
            tr = os.path.join(a.out, f"sanity_{d}.{name}.traj")
            args = ["--scene", scene, "--ticks", str(ticks), "--threads", "1,8", "--quiet", "--traj", tr, *extra]
            code, text = run(exe, args, log)
            m = re.search(r"run hash ([0-9a-f]+).*?threads (\w+), saturations (\d+)", text)
            solve = re.search(r"^solve: (.*?); at most", text, re.M)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                getattr(traj, scene)(traj.Traj(tr))
            summary = buf.getvalue().strip().split(": ", 1)[-1]
            print(f"  {name:14} {d:2} {summary}")
            print(f"  {'':14} {'':2} toy: {solve.group(1) if solve else '?'}; threads {m.group(2) if m else '?'}, saturations "
                  f"{m.group(3) if m else '?'}")
            if code != 0 or not m or m.group(2) != "agree" or m.group(3) != "0":
                bad.append(f"sanity {d} {name}: exit {code}, threads {m.group(2) if m else '?'}, saturations {m.group(3) if m else '?'}")


def gate5(a, comps, bad):
    print("gate 5: decision point 2 (the whole tick on the GPUs against the twin; the twins against the reference files)")
    if a.write_ref:
        os.makedirs(REF, exist_ok=True)
    gpu = {}  # (gpu name, dialect) -> [identical, runs]
    costs = []
    for name, scene, ticks, extra in GATE5:
        for d in ("F", "V4", "D"):
            ref = os.path.join(REF, f"{name}.{d}.txt")
            for c in comps:
                exe = os.path.join(LAB, "build", c, "bin", f"toy_{d}{EXE}")
                log = os.path.join(a.out, f"g5_{d}.{name}.{c}.txt")
                args = ["--scene", scene, "--ticks", str(ticks), "--threads", "1,8", "--quiet", *extra]
                args += ["--ref-out", ref] if a.write_ref and c == comps[0] else ["--ref", ref]
                if d != "D" and c == comps[0] and a.gpus not in ("0", ""):
                    args += ["--gpus", a.gpus]
                code, text = run(exe, args, log)
                m = re.search(r"run hash ([0-9a-f]+).*?threads (\w+), saturations (\d+)", text)
                refs = re.findall(r"^ref: (.*?) (identical to the reference.*|DIFFERS.*|the reference file is missing.*)$", text, re.M)
                gpus = re.findall(r"^gpu (\d+) (\w+): per-tick hashes (identical to the twin's|DIFFER from the twin's)(.*?); run hash \w+; "
                                  r"saturations (\d+) \(twin (\d+)\)$", text, re.M)
                written = re.search(r"^ref-out \S+: (.*)$", text, re.M)
                line = f"  {name:14} {d:2} {c:9} run hash {m.group(1) if m else '?'} threads {m.group(2) if m else '?'} saturations " \
                       f"{m.group(3) if m else '?'} exit {code}"
                if written:
                    line += f"; reference {written.group(1)}"
                twinRefs = [r for r in refs if r[0].startswith("twin")]
                if twinRefs:
                    ok = all(r[1].startswith("identical") for r in twinRefs)
                    line += f"; twins vs reference: {'identical' if ok else 'DIFFER'}"
                    if not ok:
                        bad.append(f"gate 5 {name} {d} {c}: a twin differs from the reference "
                                   f"({'; '.join(r[1] for r in twinRefs if not r[1].startswith('identical'))})")
                elif not written:
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
                cm = re.findall(r"^gpu (\d+) costs: wall (\S+) ms/tick .*?kernels (\S+) ms/tick; the busiest tick(?: after the first)? \((\d+)\): kernels (\S+) ms, "
                                r"wall (\S+) ms; at most (\d+) dispatches", text, re.M)
                for gi, wall, kern, pt, pk, pw, nd in cm:
                    costs.append((name, d, gi, wall, kern, pt, pk, pw, nd))
                print(line)
                if code != 0 or not m or m.group(2) != "agree" or m.group(3) != "0":
                    bad.append(f"gate 5 {name} {d} {c}: exit {code}, threads {m.group(2) if m else '?'}, saturations {m.group(3) if m else '?'}")
    print("  decision point 2, per GPU and dialect: runs whose every tick's hashes equal the twin's (saturations 0)")
    for (g, d), (same, total) in sorted(gpu.items()):
        print(f"    gpu {g:12} {d:2}: {same} of {total}")
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
    p.add_argument("--gpus", default="0xff", help="gate 5: the GPUs to run (a mask; 0: none)")
    p.add_argument("--write-ref", action="store_true", help="gate 5: write the reference files from the first compiler's twins")
    a = p.parse_args()
    a.out = os.path.abspath(a.out)  # the binaries run from lab/gpu
    os.makedirs(a.out, exist_ok=True)
    comps = a.compilers.split(",")
    gates = a.gates.split(",")
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
