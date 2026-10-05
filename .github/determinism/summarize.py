#!/usr/bin/env python3
"""Diffs the determinism legs' outputs into a Markdown table.

    summarize.py <artifacts dir> <summary.md>

Each leg is a directory det-<leg> holding a default/ ladder (hashes.txt, ticks/<rung>.w<N>.txt),
probe/*.bin (libm results), probe.txt, info.txt and test.txt. The reference leg is windows-x64-msvc.

Exits 1, after writing the summary, when the verdict fails: a leg that must match the reference (every leg but the
control and the informational ones) differs on a row or lacks one, or a leg's worker counts disagree, or the control
matches a row (contraction no longer shows in the hashes)."""
import os
import sys
from array import array

REFERENCE = "windows-x64-msvc"
CONTROL = "linux-arm64-gcc-contract"  # FMA contraction on: must differ from the reference on every row
INFORMATIONAL = {"box64-default"}  # box64's default dynarec flags differed on one transient line (determinism-rules.md)
RUNGS = ["walls", "town", "pile", "lumber", "tower", "ruins", "yard", "keep", "barrage", "siege", "track", "mech"]
PROBES = ["cbrtf", "sqrtf", "sinf", "cosf", "atan2f"]


def read_lines(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return [line.rstrip("\n") for line in f]
    except OSError:
        return None


def read_hashes(leg_dir, variant):
    table = {}
    for line in read_lines(os.path.join(leg_dir, variant, "hashes.txt")) or []:
        parts = line.split()
        if len(parts) >= 5 and parts[1] != "-":
            table[(parts[0], parts[1])] = (parts[2], parts[3], parts[4])
    return table


def first_diff(ref_dir, leg_dir, variant, rung, workers):
    name = f"{rung}.w{workers}.txt"
    a = read_lines(os.path.join(ref_dir, variant, "ticks", name))
    b = read_lines(os.path.join(leg_dir, variant, "ticks", name))
    if a is None or b is None:
        return "?"
    for x, y in zip(a, b):
        if x != y:
            xs, ys = x.split(), y.split()
            which = "" if xs[1] != ys[1] else " solver"
            return f"{xs[0]}{which}"
    return "len" if len(a) != len(b) else None


def cell(ref, got, ref_dir, leg_dir, variant, rung, workers):
    if got is None:
        return "-"
    mark = "" if got[2] == "exit=0" else f" ({got[2]})"
    if ref is None:
        return got[0][:8] + mark
    if got[0] == ref[0] and got[1] == ref[1]:
        return "=" + mark
    where = first_diff(ref_dir, leg_dir, variant, rung, workers)
    if got[0] == ref[0]:
        return f"solver differs @{where}{mark}"
    return f"DIFF @{where}{mark}"


def verdict(ref_leg, ref_hashes, leg_hashes):
    """What breaks the contract, one line each (empty: the run passes)."""
    rows = [(rung, workers) for rung in RUNGS for workers in ("1", "8")]
    problems = [] if ref_leg == REFERENCE else [f"the reference leg {REFERENCE} is missing"]
    for leg, hashes in leg_hashes.items():
        if leg in INFORMATIONAL:
            continue
        split = sum(1 for k in rows if k in hashes and hashes[k][2] != "exit=0")
        if split:
            problems.append(f"{leg}: its worker counts disagree on {split} rows (lpf_bench exit 2)")
        if leg == ref_leg:
            missing = sum(1 for k in rows if k not in hashes)
            if missing:
                problems.append(f"{leg} (the reference) lacks {missing} of {len(rows)} rows")
            continue
        same = sum(1 for k in rows if k in hashes and k in ref_hashes and hashes[k][:2] == ref_hashes[k][:2])
        if leg == CONTROL:
            if same:
                problems.append(f"{leg} (the control) matches {same}/{len(rows)} rows: "
                                "contraction no longer shows in the hashes")
        elif same < len(rows):
            problems.append(f"{leg} matches the reference on {same}/{len(rows)} rows")
    return problems


def main():
    root, out_path = sys.argv[1], sys.argv[2]
    legs = sorted(d[4:] for d in os.listdir(root) if d.startswith("det-") and os.path.isdir(os.path.join(root, d)))
    ref_leg = REFERENCE if REFERENCE in legs else (legs[0] if legs else None)
    legs = [ref_leg] + [leg for leg in legs if leg != ref_leg] if ref_leg else []
    ref_dir = os.path.join(root, f"det-{ref_leg}") if ref_leg else ""
    out = []
    out.append("# Cross-platform determinism\n")
    out.append(f"Reference leg: `{ref_leg}`. A cell is `=` when the final world and solver hashes equal the reference; "
               "otherwise the first differing tick (`load` = right after the scene was built). An exit code other "
               "than 0 means the leg's own worker counts disagreed (lpf_bench exit 2).\n")

    for variant, title in (("default", "The build (portable maths: no C-library roots or trig in the core and scenes)"),):
        ref_hashes = read_hashes(ref_dir, variant)
        leg_hashes = {leg: read_hashes(os.path.join(root, f"det-{leg}"), variant) for leg in legs}
        out.append(f"\n## {title}\n")
        out.append("| rung | w | reference | " + " | ".join(legs[1:]) + " |")
        out.append("|---|---|---|" + "---|" * (len(legs) - 1))
        for rung in RUNGS:
            for workers in ("1", "8"):
                ref = ref_hashes.get((rung, workers))
                row = [rung, workers, ref[0] if ref else "-"]
                for leg in legs[1:]:
                    got = leg_hashes[leg].get((rung, workers))
                    row.append(cell(ref, got, ref_dir, os.path.join(root, f"det-{leg}"), variant, rung, workers))
                out.append("| " + " | ".join(row) + " |")
        same = []
        for leg in legs[1:]:
            n = sum(1 for k, v in leg_hashes[leg].items() if ref_hashes.get(k) and v[:2] == ref_hashes[k][:2])
            same.append(f"{leg} {n}/{len(ref_hashes)}")
        out.append("\nMatching the reference: " + ", ".join(same) + "\n")
        problems = verdict(ref_leg, ref_hashes, leg_hashes)
        rule = (f"every leg but `{CONTROL}` (the control, which must differ on every row) and "
                + ", ".join(f"`{leg}`" for leg in sorted(INFORMATIONAL)) + " (informational) {} the reference on all "
                + f"{2 * len(RUNGS)} rows, with its worker counts agreeing")
        if problems:
            lines = [f"**FAIL**: {rule.format('must match')}.\n"] + [f"- {p}" for p in problems]
        else:
            lines = [f"**PASS**: {rule.format('matches')}."]
        out[2:2] = ["## Verdict\n"] + lines + [""]

    out.append("\n## C library probe\n")
    out.append("Values that differ from the reference leg's libm, of the probe's sweep (cbrtf and sqrtf: 2^-30..2^30; "
               "sinf, cosf: +-2^-12..64; atan2f: 2^18 pairs in [-60, 60]^2), and in brackets how many of the leg's own "
               "results differ from the double function rounded to float.\n")
    out.append("| function | " + " | ".join(legs) + " |")
    out.append("|---|" + "---|" * len(legs))

    def load(leg, fn):
        path = os.path.join(root, f"det-{leg}", "probe", f"{fn}.bin")
        try:
            data = array("I")
            with open(path, "rb") as f:
                data.frombytes(f.read())
            return data
        except OSError:
            return None

    def not_rounded(leg, fn):
        for line in read_lines(os.path.join(root, f"det-{leg}", "probe.txt")) or []:
            parts = line.split()
            if parts and parts[0] == fn:
                for i, p in enumerate(parts):
                    if p == "differ" and i > 0:
                        return parts[i - 1]
        return "?"

    for fn in PROBES:
        ref = load(ref_leg, fn) if ref_leg else None
        row = [fn]
        for leg in legs:
            data = load(leg, fn)
            if data is None or ref is None:
                row.append("-")
                continue
            diff = sum(1 for x, y in zip(ref, data) if x != y) + abs(len(ref) - len(data))
            row.append(f"{diff} ({not_rounded(leg, fn)})")
        out.append("| " + " | ".join(row) + " |")

    def canary(leg):
        # out/canary.txt: the gcc 13 backprop reproducer's answer (1 is right) at -O2 and with the engine's flags
        got = {}
        for line in read_lines(os.path.join(root, f"det-{leg}", "canary.txt")) or []:
            key = "-O2" if line.startswith("-O2:") else "engine" if line.startswith("engine flags") else None
            if key and " best " in line:
                got[key] = line.split(" best ")[1].split()[0]
        return f"{got.get('-O2', '?')}, {got.get('engine', '?')}" if got else "-"

    out.append("\n## Legs\n")
    out.append("The backprop canary is `lab/gpu/repro/gcc13-backprop.c`'s answer (1 is right) at -O2, then with the "
               "engine's flags.\n")
    out.append("| leg | toolchain | lpf_test | backprop canary |")
    out.append("|---|---|---|---|")
    for leg in legs:
        info = " ".join((read_lines(os.path.join(root, f"det-{leg}", "info.txt")) or ["?"])[:2]).replace("|", "/")
        test = " ".join(read_lines(os.path.join(root, f"det-{leg}", "test.txt")) or ["not run"]).replace("|", "/")
        out.append(f"| {leg} | {info[:160]} | {test[:60]} | {canary(leg)} |")

    text = "\n".join(out) + "\n"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    for p in problems:
        print(f"::error title=determinism::{p}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
