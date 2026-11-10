#!/usr/bin/env python3
"""Compare a regenerated design directory with the committed one (golden regression check).

  python3 tools/golden_diff.py /tmp/cephalo_regen/main            # vs arm-pin-hinge/out_1m_13
  python3 tools/golden_diff.py NEW --golden OLD --rtol 1e-6

STL: same file set; per file volume and bbox equal within --rtol (byte equality reported too).
JSON: every number equal within --rtol (float noise like 1e-12 ignored below --atol).
Lists files only in one side. Exit 1 on any difference.
"""
import argparse, glob, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, MAIN_OUT
import trimesh


def cmp(a, b, rtol, atol, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b: out.append(f"{path}/{k}: only in {'new' if k in a else 'golden'}")
            else: out += cmp(a[k], b[k], rtol, atol, f"{path}/{k}")
        return out
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b): return [f"{path}: length {len(a)} vs {len(b)}"]
        return [d for i, (x, y) in enumerate(zip(a, b)) for d in cmp(x, y, rtol, atol, f"{path}[{i}]")]
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return [] if abs(a - b) <= atol + rtol * max(abs(a), abs(b)) else [f"{path}: {a:.6g} vs {b:.6g}"]
    return [] if a == b else [f"{path}: {str(a)[:40]} vs {str(b)[:40]}"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("new"); ap.add_argument("--golden", default=MAIN_OUT)
    ap.add_argument("--rtol", type=float, default=1e-6); ap.add_argument("--atol", type=float, default=1e-6)
    ap.add_argument("--only", default=None, help="substring filter on relative paths")
    a = ap.parse_args()
    rel = lambda root, pat: {os.path.relpath(f, root) for f in glob.glob(os.path.join(root, "**", pat), recursive=True)}
    C = Checks(f"golden diff: {a.new}  vs  {a.golden}")
    for pat in ("*.stl", "*.json"):
        n, g = rel(a.new, pat), rel(a.golden, pat)
        # horn_bearing/fan_pod subassemblies are only in golden unless regenerated: report as warnings
        for f in sorted(g - n): C.add(f"{f} present", False, "missing from new", warn=True)
        for f in sorted(n - g): C.add(f"{f} present", False, "not in golden", warn=True)
        for f in sorted(n & g):
            if a.only and a.only not in f: continue
            p1, p2 = os.path.join(a.new, f), os.path.join(a.golden, f)
            if f.endswith(".stl"):
                if open(p1, "rb").read() == open(p2, "rb").read(): C.add(f, True, "byte-identical"); continue
                m1, m2 = trimesh.load(p1), trimesh.load(p2)
                dv = abs(m1.volume - m2.volume) if m1.is_watertight and m2.is_watertight else float("nan")
                db = float(np.abs(m1.bounds - m2.bounds).max())
                ok = (dv <= a.rtol * max(abs(m2.volume), 1) + 1e-3 and db < 1e-3)
                C.add(f, ok, f"volume diff {dv:.3g} mm^3, bbox diff {db:.3g} mm")
            else:
                if f == "preview.json": continue
                d = cmp(json.load(open(p1)), json.load(open(p2)), a.rtol, a.atol)
                C.add(f, not d, "equal" if not d else f"{len(d)} differences, first: {d[0]}")
    C.exit()


if __name__ == "__main__":
    main()
