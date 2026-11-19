"""Range boxes of hand3 hub E's static parts (frame, servos, spools, idlers, tie ring, yokes, ...) for coupler_ranges.py.
Each part is cut into 2 mm height bands; each band becomes one ring box from its innermost material radius out.  Hand3
frame -> connector frame: hand3 is turned end for end about the y axis (a rotation, so the hand is not mirrored):
z_conn = z_flange_bottom - z_h3, azimuth -> 180 - azimuth.  Writes coupler_hubE_ranges.json (cached; rerun after hub E changes)."""
import os, sys, json
import numpy as np
H3E = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e")


def build():
    sys.path.insert(0, H3E); cwd = os.getcwd(); os.chdir(H3E)
    import hand3_hubc as HC
    os.chdir(cwd)
    A = HC.assembly()
    out = []
    for name, (m, rotating) in A.items():
        if rotating or name.startswith(("cord", "rcord")):
            continue
        v = np.asarray(m.to_mesh().vert_properties)[:, :3]
        z0, z1 = float(v[:, 2].min()), float(v[:, 2].max())
        if z0 > 70:
            continue
        for zb in np.arange(np.floor(z0), min(z1, 70.0), 2.0):
            sl = m.slice(float(zb) + 1.0).to_polygons()
            pts = [np.asarray(p) for p in sl if len(p)]
            if not pts:
                continue
            allp = np.vstack(pts); r = np.hypot(allp[:, 0], allp[:, 1])
            # innermost material radius of this band: the smallest r on any loop that is an outer boundary or a hole edge
            out.append(dict(name=f"hub E {name}", r0=float(r.min()), r1=float(r.max()), z_h3=(float(zb), float(zb) + 2.0)))
    return out


if __name__ == "__main__":
    B = build()
    json.dump(B, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "coupler_hubE_ranges.json"), "w"), indent=0)
    print(len(B), "bands;", "innermost r per part:", {b["name"]: round(min(x["r0"] for x in B if x["name"] == b["name"]), 1) for b in B})
