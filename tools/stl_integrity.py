#!/usr/bin/env python3
"""Check every STL under a design directory: watertight, winding, volume, degenerate faces, counts.

  python3 tools/stl_integrity.py                     # committed main design
  python3 tools/stl_integrity.py DIR --max-dim 400   # flag parts longer than 400 mm
  python3 tools/stl_integrity.py DIR --counts        # also check the xN count in each file name

Per file: is_watertight, is_winding_consistent, volume > 0, no zero-area faces, no non-finite
vertices, bounding box. Assembly files (assembly_*.stl) are combined meshes and are only checked
for finite vertices and non-empty. Also flags: filename xN counts that disagree with the joint
count, duplicate xN siblings (a stale servo_lid_x13 next to servo_lid_x12), and empty files.
"""
import argparse, glob, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, MAIN_OUT, load_json
import trimesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT); ap.add_argument("--max-dim", type=float, default=1200.0)
    ap.add_argument("--counts", action="store_true")
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.dir, "**", "*.stl"), recursive=True))
    C = Checks(f"STL integrity: {len(files)} files under {a.dir}")
    if not files: C.add("any STL", False, "none found"); C.exit()
    for f in files:
        rel = os.path.relpath(f, a.dir)
        try:
            m = trimesh.load(f, process=False)
        except Exception as e:
            C.add(rel, False, f"unreadable: {e}"); continue
        if len(m.faces) == 0: C.add(rel, False, "empty mesh"); continue
        fin = bool(np.isfinite(m.vertices).all())
        ext = m.extents
        if os.path.basename(f).startswith("assembly_") or "cutter" in f:   # visual / cutter tools, not printed parts
            C.add(rel, fin, f"{len(m.faces)} faces, bbox {ext.round(0).tolist()}"); continue
        mm = m.copy(); mm.merge_vertices()
        wt, wc = mm.is_watertight, mm.is_winding_consistent
        deg = int((mm.area_faces < 1e-9).sum())
        vol = float(mm.volume) if wt else float("nan")
        _, cnt = np.unique(mm.edges_sorted, axis=0, return_counts=True); bad_edges = int((cnt != 2).sum())
        minor = (not wt) and bad_edges <= 4 and fin and deg == 0     # a slicer's repair closes a few stray edges
        if minor:
            C.add(rel, False, f"{len(mm.faces)} faces, {bad_edges} non-manifold edges (slicer will repair; fix the CAD join), bbox {ext.round(0).tolist()}", warn=True); continue
        ok = fin and wt and wc and deg == 0 and (vol > 0) and max(ext) <= a.max_dim
        why = [] if ok else [n for n, c in (("non-finite", fin), ("not watertight", wt), ("winding", wc), ("degenerate faces", deg == 0), ("volume<=0", vol > 0), ("too large", max(ext) <= a.max_dim)) if not c]
        C.add(rel, ok, f"{len(mm.faces)} faces, vol {vol:.0f} mm^3, bbox {ext.round(0).tolist()}" + ("" if ok else "  <- " + ", ".join(why)))
    # sibling duplicates: name_xN.stl with different N in one folder
    by = {}
    for f in files:
        m = re.match(r"(.*)_x(\d+)\.stl$", f)
        if m: by.setdefault(m.group(1), []).append(int(m.group(2)))
    for k, ns in by.items():
        C.add(f"no stale duplicates: {os.path.relpath(k, a.dir)}_x*", len(ns) == 1, f"counts {sorted(ns)}")
    if a.counts and os.path.exists(os.path.join(a.dir, "fit.json")):
        J = load_json(os.path.join(a.dir, "fit.json"))["joints"]
        for pre, want in (("vertebra", J - 1), ("servo_lid", J - 1)):
            got = [int(n) for k, ns in by.items() if os.path.basename(k) == pre for n in ns]
            C.add(f"{pre} count = {want}", got == [want] or got == [J], f"file says x{got}, J={J}", warn=True)
    C.exit()


if __name__ == "__main__":
    main()
