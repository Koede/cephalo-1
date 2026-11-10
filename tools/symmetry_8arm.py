#!/usr/bin/env python3
"""8-arm layout check: mantle port cutters evenly spaced, equal, flanges clear at the hub radius.

  python3 tools/symmetry_8arm.py [DIR]        # DIR defaults to the committed arm-pin-hinge/out_1m_13
Reads mantle_port_cutters_x8_r<R>.stl (8 bodies) and mantle_ports.json: body count, equal volumes,
centroid radius spread, 45 deg angular spacing, mutual overlap, and the recorded flange clearance
(hub_radius_used vs min_hub_radius_flanges_clear). The flange overlap is a WARN: it is an open design item.
"""
import argparse, glob, math, os, sys, warnings
warnings.filterwarnings("ignore")
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, MAIN_OUT, load_json
import trimesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT); ap.add_argument("--tol-deg", type=float, default=0.5); ap.add_argument("--tol-mm", type=float, default=0.5)
    a = ap.parse_args()
    C = Checks(f"8-arm symmetry ({a.dir})")
    fs = glob.glob(os.path.join(a.dir, "mantle_port_cutters_x8_r*.stl"))
    if not fs: C.add("cutter STL", False, "mantle_port_cutters_x8_r*.stl not found (run arm-pin-hinge/mantle_notch.py)"); C.exit()
    parts = trimesh.load(fs[0]).split(only_watertight=False)
    nb = len(parts)                                        # openings plus M3 hole bodies (STL has no port grouping)
    parts = sorted(parts, key=lambda p: -abs(p.volume))[:8]       # the 8 openings = the 8 largest bodies
    big = [abs(p.volume) for p in parts]
    C.add("8 port openings found", nb >= 8 and min(big) > 100, f"{nb} bodies in {os.path.basename(fs[0])}; 8 largest {min(big):.0f}..{max(big):.0f} mm^3")
    if len(parts) == 8:
        c = np.array([p.bounds.mean(axis=0) for p in parts]); v = np.array([abs(p.volume) for p in parts])
        r = np.hypot(c[:, 0], c[:, 1]); ang = np.sort(np.degrees(np.arctan2(c[:, 1], c[:, 0])) % 360)
        gaps = np.diff(np.append(ang, ang[0] + 360))
        C.add("equal cutter volumes", (v.max() - v.min()) / v.mean() < 0.01, f"spread {(v.max()-v.min())/v.mean()*100:.2f}%")
        C.add("equal radius from the hub axis", r.max() - r.min() < a.tol_mm, f"{r.min():.2f}..{r.max():.2f} mm")
        C.add("45 deg spacing", np.abs(gaps - 45).max() < a.tol_deg, f"gaps {np.round(gaps, 2).tolist()}")
    if os.path.exists(os.path.join(a.dir, "mantle_ports.json")):
        m = load_json(os.path.join(a.dir, "mantle_ports.json"))
        h = np.array(m["holes"]); C.add("8 M3 holes per port", len(h) == 8, f"{len(h)} holes")
        C.add("hole edge to opening > 3 mm", min(m["hole_edge_to_opening_mm"]) > 3.0, f"min {min(m['hole_edge_to_opening_mm'])} mm", warn=True)
        C.add("flanges clear at the hub radius used", m["flange_overlap_at_hub_radius_mm3"] <= 0.5,
              f"overlap {m['flange_overlap_at_hub_radius_mm3']:.0f} mm^3 at R={m['hub_radius_used']:g}; clear from R={m['min_hub_radius_flanges_clear']:g} (flange face radius there {m['flange_face_radius_at_min']:g} mm)", warn=True)
    C.exit()


if __name__ == "__main__":
    main()
