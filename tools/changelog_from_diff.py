#!/usr/bin/env python3
"""Summarise a branch's changes against a base for a PR description (markdown to stdout).

  python3 tools/changelog_from_diff.py                 # vs origin/main (or main)
  python3 tools/changelog_from_diff.py --base v1.0
Groups changed files (generators, tools/tests, CAD outputs, docs), lists STL size changes, and for changed JSON reports
shows every number that moved (relative change > 1e-6), so you can see gate impact (fit.json, root_gear.json ...).
Needs git. Never writes.
"""
import argparse, json, os, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT
from golden_diff import cmp


def git(*a):
    r = subprocess.run(["git", *a], cwd=ROOT, capture_output=True); return r.returncode, r.stdout.decode("utf8", "replace")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=None); a = ap.parse_args()
    base = a.base or next((b for b in ("origin/main", "main") if git("rev-parse", "--verify", b)[0] == 0), None)
    if not base: sys.exit("no base branch found; pass --base")
    rc, names = git("diff", "--name-status", f"{base}...HEAD")
    if rc: sys.exit("git diff failed")
    ch = [l.split("\t") for l in names.splitlines() if l.strip()]
    grp = {"Generators (CAD/sim scripts)": [], "Tools, tests, skills": [], "CAD outputs (STL/JSON/reports)": [], "Docs": [], "Other": []}
    for st, *p in ch:
        f = p[-1]
        k = ("Tools, tests, skills" if f.startswith(("tools/", "tests/", ".claude/")) else "Generators (CAD/sim scripts)" if f.endswith(".py")
             else "CAD outputs (STL/JSON/reports)" if f.endswith((".stl", ".json", ".dxf", ".svg", ".png")) or "/out" in f else "Docs" if f.endswith(".md") else "Other")
        grp[k].append((st, f))
    print(f"# Changes vs {base}\n")
    rc, stat = git("diff", "--shortstat", f"{base}...HEAD"); print(stat.strip() + "\n")
    for k, v in grp.items():
        if v: print(f"## {k} ({len(v)})"); [print(f"- `{f}` ({st})") for st, f in v[:40]]; print("..." if len(v) > 40 else "")
    moved = []
    for st, f in grp["CAD outputs (STL/JSON/reports)"]:
        if f.endswith(".json") and st == "M" and "preview" not in f:
            _, old = git("show", f"{base}:{f}"); new = open(os.path.join(ROOT, f), encoding="utf8").read()
            try: d = cmp(json.loads(new), json.loads(old), 1e-6, 1e-9)
            except Exception: continue
            if d: moved.append((f, d))
    if moved:
        print("## Numbers that moved (new vs base)")
        for f, d in moved: print(f"- `{f}`: " + "; ".join(d[:6]) + (f" ... (+{len(d)-6})" if len(d) > 6 else ""))
    stl = [f for st, f in grp["CAD outputs (STL/JSON/reports)"] if f.endswith(".stl") and st in ("M", "A")]
    if stl: print("\n## STL files to reprint / re-slice\n" + "\n".join(f"- `{f}`" for f in stl[:30]))
    print("\n## Verification\nRun `python3 tools/check_all.py` and paste the table here.")


if __name__ == "__main__":
    main()
