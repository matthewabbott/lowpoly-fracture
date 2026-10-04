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
           "relaxContacts", "restitution", "storeImpulses"]
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
        if name == "F":  # NoContraction audit of the float kernels
            for e in ["battery"] + KERNELS:
                spv = os.path.join(out, e + ".spv")
                if not os.path.exists(spv):
                    continue
                asm = run([tool("spirv-dis"), spv])
                ops = len(re.findall(r"= Op(FMul|FAdd|FSub)\b", asm))
                fdiv = len(re.findall(r"= OpFDiv\b", asm))
                ext = sorted(set(re.findall(r"OpExtInst %\w+ %\w+ (\w+)", asm)))
                print(f"  toy F/{e}: float mul/add/sub {ops}, NoContraction {asm.count('NoContraction')}, FDiv {fdiv}, "
                      f"extended {','.join(ext) or '-'}")
                if fdiv or any(x != "FindUMsb" for x in ext):  # only + - * on floats; FindUMsb is clz
                    raise SystemExit(f"{spv}: float division or an extended instruction")
        print("gen toy", name)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import lab  # noqa: E402  (lab.py: the SDK's tools and the subprocess helper)

    gen_toy(lab.LAB, lab.tool, lab.run)
