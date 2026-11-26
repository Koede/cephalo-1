#!/usr/bin/env python3
"""Health report for this repo on this machine: python, dependencies, git, committed outputs, quick checks.

  python3 tools/doctor.py            # about 5 s
  python3 tools/doctor.py --fast     # skip running the unit tests and gate checks
Prints OK / WARN / FAIL lines and the next actions. Works on Linux, macOS and Windows (python 3.9+).
"""
import argparse, importlib, os, platform, re, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, PIN_DIR, MAIN_OUT, SIM_DIR, Checks


def sh(*cmd, cwd=ROOT):
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=300); return r.returncode, (r.stdout + r.stderr).strip()
    except Exception as e:
        return 1, str(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args(); C = Checks(f"cephalo-1 doctor ({platform.system()} {platform.machine()}, python {platform.python_version()})")
    C.add("python >= 3.9", sys.version_info >= (3, 9), platform.python_version())
    need = {"numpy": "CAD + tools", "manifold3d": "CAD booleans", "trimesh": "STL I/O", "scipy": "tools (clearance, study)", "matplotlib": "plots",
            "networkx": "trimesh split", "pytest": "unit tests"}
    opt = {"numba": "joint study", "mujoco": "mujoco_sanity + study", "pandas": "joint study"}
    for m, why in need.items():
        try: v = importlib.import_module(m).__version__ if hasattr(importlib.import_module(m), "__version__") else ""; C.add(f"import {m}", True, f"{v} ({why})")
        except Exception: C.add(f"import {m}", False, f"missing: pip install -r requirements.txt ({why})")
    for m, why in opt.items():
        try: importlib.import_module(m); C.add(f"import {m}", True, why)
        except Exception: C.add(f"import {m}", False, f"optional, needed for {why}: pip install -r joint-study/sim/requirements.txt", warn=True)
    rc, out = sh("git", "status", "-sb")
    if rc == 0:
        lines = out.splitlines(); dirty = [l for l in lines[1:] if l.strip()]
        C.add("git working tree", True, f"{lines[0][3:]}; {len(dirty)} changed file(s)" if lines else "clean")
    else: C.add("git", False, "not a git checkout or git missing", warn=True)
    for rel in ("fit.json", "root_gear.json", "fit_report.md", "stl"):
        C.add(f"committed main design: {rel}", os.path.exists(os.path.join(MAIN_OUT, rel)), os.path.relpath(MAIN_OUT, ROOT))
    for d in ("tools", "tests", ".claude/skills"):
        C.add(f"{d}/ present", os.path.isdir(os.path.join(ROOT, d)))
    bad = []
    for fn in sorted(os.listdir(SIM_DIR)):
        if fn.endswith(".py"):
            t = open(os.path.join(SIM_DIR, fn), encoding="utf8", errors="ignore").read()
            if "/mnt/project-files" in t: bad.append(f"{fn}: hard-coded /mnt/project-files")
            if re.search(r'makedirs\("results"\)', t): bad.append(f"{fn}: cwd-relative results")
    C.add("joint-study/sim path bugs", not bad, "; ".join(bad) or "none", warn=True)
    if not a.fast and not C.failed:
        rc, out = sh(sys.executable, "-m", "pytest", "-q", "-x", "tests"); C.add("unit tests", rc == 0, out.splitlines()[-1] if out else "")
        rc, out = sh(sys.executable, os.path.join("tools", "check_gates.py")); C.add("design gates (committed outputs)", rc == 0, out.splitlines()[-1] if out else "")
    C.report()
    print("\nnext: " + ("fix the FAIL lines above (pip install -r requirements.txt)" if C.failed else ("review the WARN lines" if C.warned else "all good; try tools/check_all.py")))
    sys.exit(1 if C.failed else 0)


if __name__ == "__main__":
    main()
