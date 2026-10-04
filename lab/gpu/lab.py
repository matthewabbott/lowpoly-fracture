#!/usr/bin/env python3
"""lab.py: the GPU lab's one tool (lab/gpu/README.md). Standard library only.

  python lab/gpu/lab.py gen                       # Windows + Vulkan SDK: SPIR-V kernels, C++ twins, gen/variants.cmake
  python lab/gpu/lab.py build [--compiler C]      # CMake into lab/gpu/build/C; C is msvc or clang-cl (Windows), gcc or clang
  python lab/gpu/lab.py run [--machine M] [--compiler C] [--twins C2,...] [--quick] [suite ...]
                                                  # suites: probe solve rows controls mulbench twins (default: all)
  python lab/gpu/lab.py check [results/M]         # hashes against reference.json; writes results/M/summary.md
  python lab/gpu/lab.py remote HOST [--dir D] [--machine M] [--compiler gcc] [--twins clang] [--quick] [suite ...]
                                                  # copy src and gen to HOST, build and run there, fetch results/M

Kernels and twins are generated once, on Windows, and shipped: every machine then tests its own C compiler and GPU
driver, never a different slangc. Runs write their logs to lab/gpu/results/<machine>/.
"""
import argparse
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile

LAB = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(LAB))
GEN = os.path.join(LAB, "gen")
WINDOWS = sys.platform == "win32"

# ----------------------------------------------------------------------------------------------------------------------
# gen: kernels and twins (Windows, Vulkan SDK)
# ----------------------------------------------------------------------------------------------------------------------

ENTRIES = ["integrateVelocities", "warmStart", "push", "integratePositions", "relax", "finalizeBodies"]
# name, defines, float? (float kernels also get the float-control builds)
SOLVE = [
    ("F", ["VAR_F"], True),
    ("Fdisc", ["VAR_F", "FRICTION_DISC"], True),
    ("Fnocap", ["VAR_F", "NO_SPEED_CAP"], True),
    ("I64", ["VAR_I64"], False),
    ("I64w", ["VAR_I64", "INTEL_I64_WORKAROUND"], False),
    ("I64diag", ["VAR_I64", "DIAG_NO_FRICTION_MUL"], False),
    ("I32", ["VAR_I32"], False),
    ("I32chk", ["VAR_I32", "CHECK_RANGE"], False),
    ("V4", ["VAR_V4"], False),
    ("V4w20", ["VAR_V4", "V4_SW=20"], False),
    ("V4v20", ["VAR_V4", "V4_SV=20"], False),
]
ROWS = [("V1", "VAR_F"), ("V2", "VAR_I64"), ("V3", "VAR_V3"), ("V4", "VAR_V4"), ("V4b", "VAR_V4B")]
# SPIR-V float-control execution modes, by file tag: added by slangc (preserve, ftz) or patched in (the rest)
PATCHED = {".rte": ["rte"], ".szinp": ["szinp"], ".rtesz": ["rte", "szinp"], ".pszinp": ["preserve", "szinp"]}


def sdk_bin():
    sdk = os.environ.get("VULKAN_SDK", r"C:\VulkanSDK\1.4.357.0")
    b = os.path.join(sdk, "Bin" if WINDOWS else "bin")
    if not os.path.isdir(b):
        raise SystemExit(f"no Vulkan SDK at {sdk} (set VULKAN_SDK)")
    return b


def tool(name):
    return os.path.join(sdk_bin(), name + (".exe" if WINDOWS else ""))


def run(args, **kw):
    r = subprocess.run(args, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit(f"failed: {' '.join(args)}")
    return r.stdout


def slangc(src, out, defs, *extra):
    args = [tool("slangc"), "src/" + src]  # relative, so the twins' #line paths are the same on every checkout
    for d in defs:
        args += ["-D", d]
    run(args + list(extra) + ["-o", out], cwd=LAB)


def patch_spv(src, dst, modes):
    """Adds float-control capabilities and execution modes to a slangc SPIR-V module (disassemble, insert,
    reassemble, validate for Vulkan 1.3)."""
    caps = {
        "szinp": ("SignedZeroInfNanPreserve", "SignedZeroInfNanPreserve 32"),
        "rte": ("RoundingModeRTE", "RoundingModeRTE 32"),
        "preserve": ("DenormPreserve", "DenormPreserve 32"),
        "ftz": ("DenormFlushToZero", "DenormFlushToZero 32"),
    }
    lines = run([tool("spirv-dis"), "--raw-id", src]).splitlines()
    entry = next(l.split()[2] for l in lines if l.strip().startswith("OpEntryPoint"))
    out, have_ext, done_caps, done_modes = [], any('"SPV_KHR_float_controls"' in l for l in lines), False, False
    for l in lines:
        s = l.strip()
        if not done_caps and s.startswith("OpCapability"):
            out += [f"OpCapability {caps[m][0]}" for m in modes if f"OpCapability {caps[m][0]}" not in (x.strip() for x in lines)]
            done_caps = True
        if ("OpExtInstImport" in s or s.startswith("OpMemoryModel")) and not have_ext:
            out.append('OpExtension "SPV_KHR_float_controls"')
            have_ext = True
        if not done_modes and s.startswith("OpExecutionMode"):
            out += [f"OpExecutionMode {entry} {caps[m][1]}" for m in modes if not any(caps[m][1] in x for x in lines)]
            done_modes = True
        out.append(l)
    asm = dst + ".spvasm"
    with open(asm, "w") as f:
        f.write("\n".join(out) + "\n")
    run([tool("spirv-as"), "--target-env", "vulkan1.3", "--preserve-numeric-ids", asm, "-o", dst])
    run([tool("spirv-val"), "--target-env", "vulkan1.3", dst])
    os.remove(asm)


def float_builds(src, outdir, base, defs, entry):
    """The plain build plus every float-control build of one float entry point."""
    common = ["-target", "spirv", "-fp-mode", "precise", "-entry", entry, "-stage", "compute"]
    plain = os.path.join(outdir, base + ".spv")
    slangc(src, plain, defs, *common)
    slangc(src, os.path.join(outdir, base + ".preserve.spv"), defs, *common, "-denorm-mode-fp32", "preserve")
    slangc(src, os.path.join(outdir, base + ".ftz.spv"), defs, *common, "-denorm-mode-fp32", "ftz")
    for tag, modes in PATCHED.items():
        patch_spv(plain, os.path.join(outdir, base + tag + ".spv"), modes)


def cmd_gen(a):
    if not WINDOWS:
        raise SystemExit("gen runs on Windows (the Vulkan SDK's slangc); copy gen/ to other machines (lab.py remote)")
    os.makedirs(GEN, exist_ok=True)
    for name, defs, is_float in SOLVE:
        out = os.path.join(GEN, name)
        os.makedirs(out, exist_ok=True)
        for e in ENTRIES:
            if is_float:
                float_builds("solver.slang", out, e, defs, e)
            else:
                slangc("solver.slang", os.path.join(out, e + ".spv"), defs, "-target", "spirv", "-fp-mode", "precise", "-entry", e,
                       "-stage", "compute")
        slangc("solver.slang", os.path.join(out, "twin.cpp"), defs + ["TWIN_CPP"], "-target", "cpp", "-fp-mode", "precise")
        print("gen", name)
    # the GPU positive control: F with fast math (contraction allowed), run against F's own twin
    out = os.path.join(GEN, "F-fast")
    os.makedirs(out, exist_ok=True)
    for e in ENTRIES:
        slangc("solver.slang", os.path.join(out, e + ".spv"), ["VAR_F"], "-target", "spirv", "-fp-mode", "fast", "-entry", e, "-stage",
               "compute")
    print("gen F-fast")
    # NoContraction audit of the float kernels
    for name in ("F", "Fdisc", "F-fast"):
        for e in ("push", "relax"):
            asm = run([tool("spirv-dis"), os.path.join(GEN, name, e + ".spv")])
            ops = len(re.findall(r"= Op(FMul|FAdd|FSub|FDiv)\b", asm))
            print(f"  {name}/{e}: float mul/add/sub/div {ops}, NoContraction {asm.count('NoContraction')}")

    out = os.path.join(GEN, "probe")
    os.makedirs(out, exist_ok=True)
    float_builds("probe.slang", out, "probe", [], "main")
    print("gen probe")

    out = os.path.join(GEN, "rows")
    os.makedirs(out, exist_ok=True)
    common = ["-target", "spirv", "-fp-mode", "precise", "-entry", "rowMain", "-stage", "compute"]
    for name, d in ROWS:
        spv = os.path.join(out, name + ".spv")
        slangc("rows.slang", spv, [d], *common)
        slangc("rows.slang", os.path.join(out, name + ".cpp"), [d, "TWIN_CPP"], "-target", "cpp", "-fp-mode", "precise")
    for tag in (".rtesz", ".pszinp"):
        patch_spv(os.path.join(out, "V1.spv"), os.path.join(out, "V1" + tag + ".spv"), PATCHED[tag])
    slangc("rows.slang", os.path.join(out, "V1.fast.spv"), ["VAR_F"], "-target", "spirv", "-fp-mode", "fast", "-entry", "rowMain",
           "-stage", "compute")
    # V4 through two more front ends: Slang -> HLSL -> dxc, Slang -> GLSL -> glslang
    hlsl = os.path.join(out, "V4.hlsl")
    slangc("rows.slang", hlsl, ["VAR_V4"], "-target", "hlsl", "-entry", "rowMain", "-stage", "compute")
    s = open(hlsl).read()  # Slang's HLSL has D3D registers only: give it Vulkan bindings and a push constant
    s = s.replace("StructuredBuffer<RRow_0 > rows_0 : register(t0);", "[[vk::binding(0, 0)]] StructuredBuffer<RRow_0 > rows_0;")
    s = s.replace("RWStructuredBuffer<ROut_0 > outs_0 : register(u0);", "[[vk::binding(1, 0)]] RWStructuredBuffer<ROut_0 > outs_0;")
    s = s.replace("RWStructuredBuffer<uint > counters_0 : register(u1);", "[[vk::binding(2, 0)]] RWStructuredBuffer<uint > counters_0;")
    s = re.sub(r"cbuffer rp_0 : register\(b0\)\s*\{\s*RPush_0 rp_0;\s*\}", "[[vk::push_constant]] RPush_0 rp_0;", s)
    open(os.path.join(out, "V4.vk.hlsl"), "w").write(s)
    run([tool("dxc"), "-spirv", "-T", "cs_6_2", "-E", "rowMain", "-fspv-target-env=vulkan1.3", "-fspv-entrypoint-name=main",
         os.path.join(out, "V4.vk.hlsl"), "-Fo", os.path.join(out, "V4.dxc.spv")])
    glsl = os.path.join(out, "V4.glsl")
    slangc("rows.slang", glsl, ["VAR_V4"], "-target", "glsl", "-entry", "rowMain", "-stage", "compute")
    run([tool("glslangValidator"), "-V", "--target-env", "vulkan1.3", "-S", "comp", glsl, "-o", os.path.join(out, "V4.glslang.spv")])
    print("gen rows")

    out = os.path.join(GEN, "mulbench")
    os.makedirs(out, exist_ok=True)
    for m in range(8):
        run([tool("glslangValidator"), "-V", "--target-env", "vulkan1.3", "-S", "comp", f"-DMODE={m}",
             os.path.join(LAB, "src", "mulbench.comp"), "-o", os.path.join(out, f"mode{m}.spv")])
    print("gen mulbench")

    with open(os.path.join(GEN, "variants.cmake"), "w") as f:
        f.write("# written by lab.py gen\n")
        f.write("set(LAB_SOLVE_VARIANTS " + " ".join(n for n, _, _ in SOLVE) + ")\n")
        for n, defs, _ in SOLVE:
            f.write(f"set(LAB_SOLVE_DEFS_{n} " + " ".join(f'"{d}"' for d in defs) + ")\n")
        f.write("set(LAB_ROWS_VARIANTS " + " ".join(n for n, _ in ROWS) + ")\n")
        for n, d in ROWS:
            f.write(f"set(LAB_ROWS_DEF_{n} {d})\n")
    r = subprocess.run([tool("slangc"), "-version"], capture_output=True, text=True)
    version = (r.stdout + r.stderr).strip() or "?"
    with open(os.path.join(GEN, "manifest.json"), "w") as f:
        json.dump({"slangc": version, "sdk": sdk_bin()}, f, indent=1)
    print("gen done: slangc", version)
    # the dual-dialect toy (toy/DESIGN.md): its kernels and twins into gen/toy, when toy/ is here
    if os.path.exists(os.path.join(LAB, "toy", "gen_toy.py")):
        sys.path.insert(0, os.path.join(LAB, "toy"))
        import gen_toy
        gen_toy.gen_toy(LAB, tool, run)


# ----------------------------------------------------------------------------------------------------------------------
# build
# ----------------------------------------------------------------------------------------------------------------------

def default_compiler():
    return "msvc" if WINDOWS else "gcc"


def cmd_build(a):
    comp = a.compiler or default_compiler()
    bdir = os.path.join(LAB, "build", comp)
    gen = ["-G", "Ninja"] if shutil.which("ninja") or WINDOWS else []
    cc = {"msvc": ("cl", "cl"), "clang-cl": ("clang-cl", "clang-cl"), "gcc": ("gcc", "g++"), "clang": ("clang", "clang++")}[comp]
    cfg = ["cmake", "-S", LAB, "-B", bdir, *gen, "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_C_COMPILER={cc[0]}",
           f"-DCMAKE_CXX_COMPILER={cc[1]}"]
    bld = ["cmake", "--build", bdir, "--parallel"]
    if WINDOWS:  # the MSVC environment comes from the repository's devenv.ps1
        q = lambda args: " ".join("'" + x.replace("'", "''") + "'" for x in args)
        script = f". '{os.path.join(REPO, 'tools', 'devenv.ps1')}' | Out-Null; & {q(cfg)}; if ($LASTEXITCODE) {{ exit 1 }}; & {q(bld)}; exit $LASTEXITCODE"
        r = subprocess.run(["pwsh", "-NoProfile", "-Command", script])
    else:
        r = subprocess.run(cfg)
        if r.returncode == 0:
            r = subprocess.run(bld)
    if r.returncode != 0:
        raise SystemExit(f"build failed ({comp})")
    print(f"built {bdir}")


# ----------------------------------------------------------------------------------------------------------------------
# run
# ----------------------------------------------------------------------------------------------------------------------

def jobs(quick):
    n = "1000,10000" if quick else "1000,10000,100000"
    rows_n = ["--n", "65536"] if quick else []
    reps = ["--reps", "1"] if quick else []
    ieee = ["--tag", "nvidia=.rtesz", "--tag", "intel=.pszinp"]
    return {
        "probe": [("probe", "probe", ["--n", "65536" if quick else "1048576"])],
        "solve": [
            ("solve_F", "harness_F", ["--n", n, "--label", "F", *reps]),
            ("solve_F_ieee", "harness_F", ["--n", n, "--label", "F-ieee", *ieee, *reps]),
            ("solve_F_any", "harness_F", ["--n", n, "--label", "F-any", "--tag", "intel=", "--threads", "1", *reps]),
            ("solve_Fdisc", "harness_Fdisc", ["--n", n, "--label", "Fdisc", "--spv", "gen/Fdisc", "--threads", "1", *reps]),
            ("solve_Fdisc_ieee", "harness_Fdisc", ["--n", n, "--label", "Fdisc-ieee", "--spv", "gen/Fdisc", *ieee, "--threads", "1", *reps]),
            ("solve_Fnocap", "harness_Fnocap", ["--n", "1000,10000", "--label", "Fnocap", "--spv", "gen/Fnocap", "--threads", "1", *reps]),
            ("solve_I64", "harness_I64", ["--n", n, "--label", "I64", *reps]),
            ("solve_I32", "harness_I32", ["--n", n, "--label", "I32", *reps]),
            ("solve_V4", "harness_V4", ["--n", n, "--label", "V4", *reps]),
        ],
        "rows": [(f"rows_{v}", f"rows_{v}", [*rows_n, *reps]) for v, _ in ROWS] + [
            ("rows_V1_rtesz", "rows_V1", ["--spv", "gen/rows/V1.rtesz.spv", "--label", "V1-rtesz", "--tag", ".rtesz", *rows_n, *reps]),
            ("rows_V1_pszinp", "rows_V1", ["--spv", "gen/rows/V1.pszinp.spv", "--label", "V1-pszinp", "--tag", ".pszinp", *rows_n, *reps]),
            ("rows_V4_dxc", "rows_V4", ["--spv", "gen/rows/V4.dxc.spv", "--label", "V4-dxc", *rows_n, *reps]),
            ("rows_V4_glslang", "rows_V4", ["--spv", "gen/rows/V4.glslang.spv", "--label", "V4-glslang", *rows_n, *reps]),
        ],
        # Positive controls: each must show a difference, which proves the harness can see one
        "controls": [
            ("control_F_fma", "harness_F-fma", ["--n", "10000", "--label", "F-fma", "--threads", "1", "--gpus", "0"]),
            # fast math on the GPU at every size: what the float dialect's NoContraction costs
            ("control_F_fast", "harness_F", ["--n", n, "--label", "F-fast", "--spv", "gen/F-fast", "--tag", "intel=", "--threads", "1", *reps]),
            ("control_rows_V1_fma", "rows_V1-fma", ["--label", "V1-fma", "--gpus", "0", *rows_n]),
            ("control_rows_V1_fast", "rows_V1", ["--spv", "gen/rows/V1.fast.spv", "--label", "V1-fast", *rows_n, "--reps", "1"]),
        ],
        "mulbench": [("mulbench", "mulbench", [])],
    }


def twin_jobs(comp, quick):
    """Twins only (no GPU), for a second compiler on the same machine."""
    n = "1000,10000" if quick else "1000,10000,100000"
    out = [(f"twin_{v}.{comp}", f"harness_{v}", ["--n", n, "--label", f"{v}-{comp}", "--gpus", "0"]) for v in ("F", "Fdisc", "I64", "I32", "V4")]
    out += [(f"twin_rows_{v}.{comp}", f"rows_{v}", ["--label", f"{v}-{comp}", "--gpus", "0"] + (["--n", "65536"] if quick else [])) for v, _ in ROWS]
    return out


def machine_name():
    return re.sub(r"[^a-z0-9-]", "-", platform.node().split(".")[0].lower()) or "local"


def cmd_run(a):
    machine = a.machine or machine_name()
    comp = a.compiler or default_compiler()
    res = os.path.join(LAB, "results", machine)
    os.makedirs(res, exist_ok=True)
    all_jobs = jobs(a.quick)
    suites = a.suites or ["probe", "solve", "rows", "controls", "mulbench", "twins"]
    plan = []
    for s in suites:
        if s == "twins":
            for c in (a.twins.split(",") if a.twins else []):
                plan += [(name, os.path.join(LAB, "build", c, "bin"), exe, args) for name, exe, args in twin_jobs(c, a.quick)]
        elif s in all_jobs:
            plan += [(name, os.path.join(LAB, "build", comp, "bin"), exe, args) for name, exe, args in all_jobs[s]]
        else:
            raise SystemExit(f"unknown suite {s}")
    keep = a.keep_csv or bool(a.suites)  # a partial run adds to the machine's results
    for f in ("solve.csv", "rows.csv"):
        if os.path.exists(os.path.join(res, f)) and not keep:
            os.remove(os.path.join(res, f))
    with open(os.path.join(res, "machine.txt"), "a" if keep else "w") as f:
        f.write(f"machine {machine}\nplatform {platform.platform()}\nprocessor {platform.machine()}\nprimary compiler {comp}\n")
        f.write(f"twin compilers {a.twins or '-'}\nquick {a.quick}\n")
    exits = {}  # runs that crashed or failed, by log name (a driver's compiler crashing is a result)
    if keep and os.path.exists(os.path.join(res, "exits.json")):
        exits = json.load(open(os.path.join(res, "exits.json")))
    for name, bindir, exe, args in plan:
        log = os.path.join(res, name + ".txt")
        exits.pop(name, None)
        path = os.path.join(bindir, exe + (".exe" if WINDOWS else ""))
        if not os.path.exists(path):
            print(f"  {name}: missing {path} (lab.py build)")
            continue
        if exe == "mulbench":
            cmd = [path, log]
        elif exe == "probe":
            cmd = [path, *args, "--log", log]
        else:
            cmd = [path, *args, "--log", log, "--csv", os.path.join(res, "rows.csv" if exe.startswith("rows") else "solve.csv")]
        print(f"  {name}: {' '.join(os.path.relpath(c, LAB) if os.path.isabs(c) else c for c in cmd)}", flush=True)
        r = subprocess.run(cmd, cwd=LAB, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"    exit {r.returncode}: {(r.stdout + r.stderr).strip()[-400:]}")
            exits[name] = r.returncode
    with open(os.path.join(res, "exits.json"), "w") as f:
        json.dump(exits, f, indent=1)
    check(res)


# ----------------------------------------------------------------------------------------------------------------------
# check: logs against reference.json
# ----------------------------------------------------------------------------------------------------------------------

def parse_solve(text):
    """A harness log: label, GPUs, and per N the twin hashes, the comparisons, GPU costs."""
    out = {"label": None, "gpus": [], "skipped": [], "n": {}}
    m = re.search(r"\(label ([^)]*)\)", text)
    out["label"] = m.group(1) if m else None
    m = re.search(r"^twin: (.*)$", text, re.M)
    out["twin"] = m.group(1) if m else "?"
    out["gpus"] = re.findall(r"^gpu \d+: (.*?) \((\w+)\), driver (0x[0-9a-f]+)", text, re.M)
    out["skipped"] = re.findall(r"skipped: (.*)$", text, re.M)
    for sec in re.split(r"^=== ", text, flags=re.M)[1:]:
        n = int(re.match(r"N (\d+)", sec).group(1))
        d = {"twin": {}, "compare": {}, "gpu": {}}
        for t, ms, h in re.findall(r"cpu twin (\d+) thread\(s\): ([0-9.]+) ms/step, final hash ([0-9a-f]+)", sec):
            d["twin"][int(t)] = (h, float(ms))
        for lab, mism in re.findall(r"compare (.+?)\s+words \d+ mismatches (\d+)", sec):
            d["compare"][lab.strip()] = int(mism)
        for v, ms, step in re.findall(r"gpu (\w+): ([0-9.]+) ms/step .*?first divergent step (-?\d+)", sec):
            d["gpu"][v] = (float(ms), int(step))
        out["n"][n] = d
    return out


def parse_rows(text):
    out = {"label": None, "twin": None, "gpu": {}}
    m = re.search(r"\(label ([^)]*)\)", text)
    out["label"] = m.group(1) if m else None
    m = re.search(r"^twin \((.*?); 1 thread\): ([0-9.]+) ns/row, hash ([0-9a-f]+)", text, re.M)
    if m:
        out["twin"] = (m.group(3), float(m.group(2)), m.group(1))
    for v, ns, h, eq, diff in re.findall(r"^gpu (\w+): ([0-9.]+) ns/row .*?hash ([0-9a-f]+) (==|!=) twin, rows differing (\d+)", text, re.M):
        out["gpu"][v] = (float(ns), h, eq == "==", int(diff))
    return out


def ref_key(label):
    base = re.split(r"-(ieee|any|fma|fast|gcc|clang|clang-cl|msvc|rtesz|pszinp|dxc|glslang)$", label or "")[0]
    return base


def check(res):
    ref = json.load(open(os.path.join(LAB, "reference.json")))
    machine_txt = open(os.path.join(res, "machine.txt")).read() if os.path.exists(os.path.join(res, "machine.txt")) else ""
    rows_ref = ref["rows_quick"] if "quick True" in machine_txt else ref["rows"]
    exits = json.load(open(os.path.join(res, "exits.json"))) if os.path.exists(os.path.join(res, "exits.json")) else {}
    lines, failures = [], []
    lines.append(f"# GPU lab results: {os.path.basename(res)}\n")
    if machine_txt:
        lines.append("```\n" + machine_txt.strip() + "\n```\n")
    lines.append("Each twin's hash against reference.json (E11's Windows hashes); each GPU's mismatching words against the twin of")
    lines.append("its own run. Positive controls must differ. ms/step at the largest N.\n")
    lines.append("| log | twin | twin vs reference | threads agree | GPUs (mismatching words; ms/step) |")
    lines.append("|---|---|---|---|---|")
    for f in sorted(os.listdir(res)):
        if not f.endswith(".txt") or f in ("machine.txt",) or f.startswith(("probe", "mulbench")) or ".mismatch" in f:
            continue
        text = open(os.path.join(res, f), errors="replace").read()
        name = f[:-4]
        control = name.startswith("control_")
        if "harness" in text[:200] or "E11 harness" in text:
            p = parse_solve(text)
            key = ref_key(p["label"])
            exp = ref["solve"].get(key, {})
            refcells, agree, gcells = [], True, {}
            for n, d in sorted(p["n"].items()):
                hashes = {h for h, _ in d["twin"].values()}
                agree &= len(hashes) <= 1
                h = next(iter(hashes)) if hashes else None
                want = exp.get(str(n))
                if want is None or h is None:
                    refcells.append(f"{n}: {h or '-'} (no ref)")
                elif (h == want) != (name == "control_F_fma"):
                    refcells.append(f"{n}: ok" if h == want else f"{n}: differs (control ok)")
                elif key in ref.get("nan_variants", {}):
                    refcells.append(f"{n}: differs (NaN bits, expected off x64)")
                else:
                    refcells.append(f"{n}: **{'DIFFERS' if h != want else 'SAME (control failed)'}**")
                    failures.append(f"{name} N={n}: twin {h}, reference {want}")
                for lab, mism in d["compare"].items():
                    if lab.startswith("cpu vs "):
                        v = lab[7:]
                        ms = d["gpu"].get(v, (0.0, 0))[0]
                        gcells.setdefault(v, []).append((n, mism, ms))
                        if name == "control_F_fast" and mism == 0:
                            failures.append(f"{name} N={n}: {v} matched the twin under fast math (control failed)")
                    elif lab.startswith("cpu1 vs") and mism:
                        agree = False
            gpu_txt = "; ".join(
                f"{v}: " + "/".join(str(m) for _, m, _ in cells) + f" ({cells[-1][2]:.2f} ms)" for v, cells in gcells.items())
            if p["skipped"]:
                gpu_txt += "; skipped: " + "; ".join(s[:60] for s in p["skipped"])
            if name in exits:
                gpu_txt += f"; **exit {exits[name]}** after GPU {len(p['gpus'])} ({p['gpus'][-1][0] if p['gpus'] else '-'})"
            if not agree:
                failures.append(f"{name}: thread counts disagree")
            lines.append(f"| {name} | {p['twin'][:40]} | {', '.join(refcells)} | {'yes' if agree else '**NO**'} | {gpu_txt or '-'} |")
        elif text.startswith("E11 rows"):
            p = parse_rows(text)
            key = ref_key(p["label"])
            want = rows_ref.get(key)
            if p["twin"] is None:
                lines.append(f"| {name} | ? | no twin line | | |")
                continue
            h = p["twin"][0]
            is_fma = name == "control_rows_V1_fma"
            if want is None:
                cell = f"{h} (no ref)"
            elif (h == want) != is_fma:
                cell = "ok" if h == want else "differs (control ok)"
            else:
                cell = "**DIFFERS**" if h != want else "**SAME (control failed)**"
                failures.append(f"{name}: twin {h}, reference {want}")
            gpu_txt = "; ".join(f"{v}: {diff} rows ({ns:.2f} ns/row)" for v, (ns, gh, eq, diff) in p["gpu"].items())
            if name in exits:
                gpu_txt += f"; **exit {exits[name]}**"
            if name == "control_rows_V1_fast":
                for v, (ns, gh, eq, diff) in p["gpu"].items():
                    if eq:
                        failures.append(f"{name}: {v} matched the twin under fast math (control failed)")
            lines.append(f"| {name} | {p['twin'][2][:40]} | {cell} | | {gpu_txt or '-'} |")
    lines.append("")
    lines.append("**Failures:** " + ("none" if not failures else ""))
    lines += [f"- {x}" for x in failures]
    summary = "\n".join(lines) + "\n"
    with open(os.path.join(res, "summary.md"), "w") as f:
        f.write(summary)
    print(summary)
    return not failures


def cmd_check(a):
    res = a.results or os.path.join(LAB, "results", machine_name())
    sys.exit(0 if check(os.path.abspath(res)) else 1)


# ----------------------------------------------------------------------------------------------------------------------
# remote: copy, build and run on another machine over ssh, fetch the results
# ----------------------------------------------------------------------------------------------------------------------

def cmd_remote(a):
    machine = a.machine or a.host.split("@")[-1].split(".")[0]
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as t:
        skip = lambda ti: None if ti.name.endswith((".spvasm", ".hlsl", ".glsl")) or "/build/" in ti.name else ti
        for item in ("CMakeLists.txt", "lab.py", "reference.json", "src", "gen", "toy"):
            if os.path.exists(os.path.join(LAB, item)):
                t.add(os.path.join(LAB, item), arcname=f"lab/gpu/{item}", filter=skip)
        for item in ("CMakeLists.txt", "include", "src"):  # Box3D, for the reference programs
            t.add(os.path.join(REPO, "extern", "box3d", item), arcname=f"extern/box3d/{item}", filter=skip)
    rdir = a.dir
    print(f"copying {len(buf.getvalue()) // 1024} KiB to {a.host}:{rdir}")
    r = subprocess.run(["ssh", a.host, f"mkdir -p {rdir} && rm -rf {rdir}/lab/gpu/src {rdir}/lab/gpu/gen {rdir}/lab/gpu/toy && tar xzf - -C {rdir}"],
                       input=buf.getvalue())
    if r.returncode:
        raise SystemExit("copy failed")
    comps = [a.compiler] + (a.twins.split(",") if a.twins else [])
    steps = [f"python3 lab.py build --compiler {c}" for c in comps]
    run_args = f"--machine {machine} --compiler {a.compiler}" + (f" --twins {a.twins}" if a.twins else "") + (" --quick" if a.quick else "")
    steps.append(f"python3 lab.py run {run_args} {' '.join(a.suites)}")
    remote_cmd = f"cd {rdir}/lab/gpu && " + " && ".join(steps)
    print(f"ssh {a.host}: {remote_cmd}")
    r = subprocess.run(["ssh", a.host, remote_cmd])
    if r.returncode:
        print(f"remote run exited {r.returncode}; fetching what there is")
    r = subprocess.run(["ssh", a.host, f"cd {rdir}/lab/gpu && tar czf - results/{machine}"], capture_output=True)
    if r.returncode == 0 and r.stdout:
        with tarfile.open(fileobj=io.BytesIO(r.stdout), mode="r:gz") as t:
            t.extractall(LAB)
        print(f"fetched results/{machine}")
        check(os.path.join(LAB, "results", machine))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("gen")
    b = sub.add_parser("build")
    b.add_argument("--compiler", choices=["msvc", "clang-cl", "gcc", "clang"])
    r = sub.add_parser("run")
    r.add_argument("--machine")
    r.add_argument("--compiler", choices=["msvc", "clang-cl", "gcc", "clang"])
    r.add_argument("--twins", help="more compilers whose twins run without GPUs (built with lab.py build --compiler)")
    r.add_argument("--quick", action="store_true", help="small sizes, one rep: a smoke test")
    r.add_argument("--keep-csv", action="store_true")
    r.add_argument("suites", nargs="*")
    c = sub.add_parser("check")
    c.add_argument("results", nargs="?")
    m = sub.add_parser("remote")
    m.add_argument("host")
    m.add_argument("--dir", default="~/lpf-m11")
    m.add_argument("--machine")
    m.add_argument("--compiler", default="gcc", choices=["gcc", "clang"])
    m.add_argument("--twins")
    m.add_argument("--quick", action="store_true")
    m.add_argument("suites", nargs="*")
    a = p.parse_args()
    {"gen": cmd_gen, "build": cmd_build, "run": cmd_run, "check": cmd_check, "remote": cmd_remote}[a.cmd](a)


if __name__ == "__main__":
    main()
