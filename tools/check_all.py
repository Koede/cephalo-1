#!/usr/bin/env python3
"""One command: the whole verification chain with a single summary table.

  python3 tools/check_all.py               # unit tests, regen, gates, collision, STL, golden, DOF, symmetry, mass, docs (~40 s)
  python3 tools/check_all.py --quick       # no regeneration: tests + gates + collision(static,pairs) (~8 s)
  python3 tools/check_all.py --full        # also clearance_map, singularity scan, mujoco_sanity, study_report all
  python3 tools/check_all.py --keep-going  # (default) run every step even after a failure
Each step runs as its own process, so a crash in one does not hide the others. Exit code 1 if any step FAILS;
steps that only WARN (known open items: horn-bearing T/W, mantle flange overlap, bearing non-manifold edges) pass.
"""
import argparse, os, subprocess, sys, time, shutil
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, SCRATCH, table

PY = sys.executable
T = lambda name, *args: [PY, os.path.join(ROOT, "tools", name), *args]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true"); ap.add_argument("--full", action="store_true")
    a = ap.parse_args()
    out = os.path.join(SCRATCH, "check_all"); steps = [("unit tests", [PY, "-m", "pytest", "-q", "tests"])]
    if a.quick:
        steps += [("design gates (committed)", T("check_gates.py")), ("collision static+pairs", T("collision.py", "--level", "static,pairs"))]
    else:
        steps += [("regenerate main design", T("regen.py", "--out", out)), ("design gates (regenerated)", T("check_gates.py", out)),
                  ("collision", T("collision.py", "--level", "static,sweeps,pairs,mc", "--samples", "100")),
                  ("STL integrity", T("stl_integrity.py", out)), ("golden diff vs committed", T("golden_diff.py", out)),
                  ("joint range (all)", T("joint_range.py", "--step", "5", "--out", os.path.join(SCRATCH, "joint_range.json"))),
                  ("DOF / Jacobian rank", T("joint_dof.py", "--samples", "200")), ("8-arm symmetry", T("symmetry_8arm.py")),
                  ("mass budget", T("mass_budget.py")), ("gear check", T("gear_check.py")), ("fasteners", T("fastener_check.py")),
                  ("docs vs reports", T("design_doc_sync.py"))]
    if a.full:
        steps += [("clearance map", T("clearance_map.py")), ("singularity scan", T("singularity_scan.py", "--samples", "2000")),
                  ("MuJoCo cross-check", T("mujoco_sanity.py")), ("study results", T("study_report.py", "all"))]
    rows, bad = [], 0
    for name, cmd in steps:
        t = time.time(); r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True); dt = time.time() - t
        lines = [l for l in (r.stdout + r.stderr).strip().splitlines() if l.strip()]; last = lines[-1][:80] if lines else ""
        warn = sum(1 for l in lines if l.startswith("WARN"))
        status = "FAIL" if r.returncode else ("WARN" if warn or "warning" in last else "PASS"); bad += bool(r.returncode)
        rows.append([status, name, f"{dt:.1f}s", last])
        if r.returncode: print(f"--- {name} failed:\n" + "\n".join(lines[-12:]) + "\n")
    print(table(rows, ["", "step", "time", "result"]))
    print(f"\n{len(rows)-bad}/{len(rows)} steps ok" + (f", {bad} FAILED" if bad else ""))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
