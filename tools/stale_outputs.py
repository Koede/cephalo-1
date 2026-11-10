#!/usr/bin/env python3
"""Find committed output files that no script in the repo appears to generate, and stale duplicate parts.

  python3 tools/stale_outputs.py                     # report only, never deletes
For each committed file under arm-pin-hinge/out*/, spec-sheet/ and joint-study/results/, looks for the file's name
(minus xN counts and digits) in the .py sources (also f-string prefixes such as "assembly_" in "assembly_{nm}.stl").
Reports: files with no generator found, name_xN.stl families with more than one N, and output folders whose
generating command is not written in README.md or DESIGN.md. Heuristic: a hit means "check by hand".
"""
import argparse, os, re, subprocess, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, Checks


def main():
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    r = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True); tracked = r.stdout.split()
    py = "\n".join(open(os.path.join(ROOT, f), encoding="utf8", errors="ignore").read() for f in tracked if f.endswith(".py"))
    docs = "".join(open(os.path.join(ROOT, f), encoding="utf8", errors="ignore").read() for f in ("README.md", "DESIGN.md") if os.path.exists(os.path.join(ROOT, f)))
    C = Checks("committed outputs vs generators")
    outs = [f for f in tracked if re.match(r"(arm-pin-hinge/out|spec-sheet/|joint-study/results/|arm-canted-hinge/out)", f) and not f.endswith((".py", ".md", ".html", ".csv", ".gz", ".log", ".json")) or f.endswith(("render.png", "root_gear_check.png"))]
    orphans = []
    for f in outs:
        base = os.path.basename(f); stem = re.sub(r"_x\d+(?=\.)|\d+", "", os.path.splitext(base)[0]); stem = stem.strip("_")
        if stem and (stem in py or base in py): continue
        parts = re.split(r"[_]", os.path.splitext(base)[0])
        if any(len(p) > 3 and p in py for p in parts if not p.isdigit()) and base.endswith(".stl"): continue
        orphans.append(f)
    C.add("every image/svg/dxf/stl has a generator", not orphans, "none missing" if not orphans else f"{len(orphans)} without a visible generator, e.g. " + ", ".join(orphans[:4]), warn=True)
    for o in orphans[:12]: print("  no generator found:", o)
    fam = defaultdict(list)
    for f in tracked:
        m = re.match(r"(.*)_x(\d+)\.stl$", f)
        if m: fam[m.group(1)].append(int(m.group(2)))
    dup = {k: v for k, v in fam.items() if len(v) > 1}
    C.add("no stale name_xN.stl duplicates", not dup, str(dup) if dup else "none")
    dirs = sorted({os.path.dirname(f).split("/")[0] + "/" + (os.path.dirname(f).split("/")[1] if "/" in os.path.dirname(f) else "") for f in tracked if "/out" in f or f.startswith("spec-sheet/cad")})
    for d in dirs:
        name = d.rstrip("/").split("/")[-1]
        if name and not re.search(re.escape(name), docs): C.add(f"folder {d} mentioned in README/DESIGN", False, "no recorded generating command", warn=True)
    C.exit()


if __name__ == "__main__":
    main()
