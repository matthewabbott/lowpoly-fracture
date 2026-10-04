#!/usr/bin/env python3
"""gen_toy.py: the toy's kernels and twins (toy/DESIGN.md), into gen/toy/<dialect>/.

  python lab/gpu/toy/gen_toy.py      # alone (Windows, Vulkan SDK); `python lab/gpu/lab.py gen` also calls gen_toy()

Per dialect: SPIR-V per entry point for F and V4 (plain -fp-mode precise: no float-control execution modes, F-plain),
and the C++ twins of battery.slang and kernels.slang (-target cpp, TWIN_CPP) for F, V4 and D. slangc runs from the
lab directory with relative source paths, so the twins' #line paths are the same on every checkout.
"""
import os
import re
import sys

DIALECTS = [("F", True), ("V4", True), ("D", False)]  # name, has GPU kernels
SOURCES = {"battery": ["battery"], "kernels": []}  # source -> SPIR-V entry points (kernels' from KERNELS)
KERNELS = ["prepareBodies", "integrateVelocities", "integratePositions", "finalizeBodies", "wakeBodies", "hashElements",
           "narrowSat", "narrowClip", "copyManifolds", "hashManifolds", "prepareContacts", "warmStart", "pushContacts",
           "relaxContacts", "restitution", "storeImpulses", "prepareJoints", "warmStartJoints", "solveJoints", "relaxJoints",
           "hashJoints"]
FLOAT_MODES = ("DenormPreserve", "DenormFlushToZero", "RoundingModeRTE", "RoundingModeRTZ", "SignedZeroInfNanPreserve")


def gen_toy(lab, tool, run):
    toy = os.path.join(lab, "toy")
    if not os.path.isdir(toy):
        return
    out_root = os.path.join(lab, "gen", "toy")
    os.makedirs(out_root, exist_ok=True)

    def slangc(src, out, defs, *extra):
        args = [tool("slangc"), "toy/" + src]
        for d in defs:
            args += ["-D", d]
        run(args + list(extra) + ["-o", out], cwd=lab)

    for name, gpu in DIALECTS:
        out = os.path.join(out_root, name)
        os.makedirs(out, exist_ok=True)
        defs = ["DIALECT_" + name]
        for src, entries in (("battery", SOURCES["battery"]), ("kernels", KERNELS)):
            if not os.path.exists(os.path.join(toy, src + ".slang")):
                continue
            if gpu:
                for e in entries:
                    spv = os.path.join(out, e + ".spv")
                    slangc(src + ".slang", spv, defs, "-target", "spirv", "-fp-mode", "precise", "-entry", e, "-stage", "compute")
                    run([tool("spirv-val"), "--target-env", "vulkan1.3", spv])
                    asm = run([tool("spirv-dis"), spv])
                    modes = [m for m in FLOAT_MODES if m in asm]
                    if modes:  # F-plain: no float-control modes in the decision rule (DESIGN.md)
                        raise SystemExit(f"{spv}: float-control execution modes {modes}")
            slangc(src + ".slang", os.path.join(out, src + ".cpp"), defs + ["TWIN_CPP"], "-target", "cpp", "-fp-mode", "precise")
        if gpu:
            audit(name, out, ["battery"] + KERNELS, tool, run)
        print("gen toy", name)


# Float arithmetic a driver could contract or that is not + - * (DESIGN.md: F uses only add, subtract and multiply on
# the GPU, each decorated NoContraction; V4 uses none)
FLOAT_OPS = ("FMul", "FAdd", "FSub")
FORBIDDEN = ("FDiv", "FRem", "FMod", "Dot", "VectorTimesScalar", "MatrixTimesScalar", "VectorTimesMatrix", "MatrixTimesVector",
             "MatrixTimesMatrix", "OuterProduct")


def audit(name, out, entries, tool, run):
    """The NoContraction audit, by result id: every F float add, subtract and multiply must carry the decoration (a total
    that matches could hide an undecorated op behind a decorated non-arithmetic one); no float division, no matrix or
    vector product and no extended instruction but FindUMsb (clz) in F; no float arithmetic at all in V4. Any miss fails
    the generation."""
    for e in entries:
        spv = os.path.join(out, e + ".spv")
        if not os.path.exists(spv):
            continue
        asm = run([tool("spirv-dis"), "--raw-id", spv])
        decorated = set(re.findall(r"OpDecorate (%\d+) NoContraction\b", asm))
        ops = re.findall(r"^\s*(%\d+) = Op(" + "|".join(FLOAT_OPS) + r")\b", asm, re.M)
        missing = [rid for rid, _ in ops if rid not in decorated]
        bad = sorted(set(re.findall(r"= Op(" + "|".join(FORBIDDEN) + r")\b", asm)))
        ext = sorted(set(re.findall(r"OpExtInst %\d+ %\d+ (\w+)", asm)))
        if name == "F":
            print(f"  toy F/{e}: float mul/add/sub {len(ops)}, each NoContraction ({len(ops) - len(missing)} decorated, "
                  f"{len(decorated)} decorations), forbidden {','.join(bad) or '-'}, extended {','.join(ext) or '-'}")
            if missing:
                raise SystemExit(f"{spv}: {len(missing)} float add/sub/mul without NoContraction (result ids {', '.join(missing[:8])})")
            if bad or any(x != "FindUMsb" for x in ext):  # only + - * on floats; FindUMsb is clz
                raise SystemExit(f"{spv}: forbidden float operations {bad} or extended instructions {ext}")
        elif ops or bad:
            raise SystemExit(f"{spv}: dialect {name} has float arithmetic ({len(ops)} add/sub/mul, {bad})")


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import lab  # noqa: E402  (lab.py: the SDK's tools and the subprocess helper)

    gen_toy(lab.LAB, lab.tool, lab.run)
