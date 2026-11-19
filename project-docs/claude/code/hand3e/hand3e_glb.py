"""Final hand3 (hub C2) as one GLB: assembled (left) + exploded (right), every part a named, coloured node."""
import numpy as np, trimesh, sys
sys.path.insert(0, "/home/claude/octodrone/hand3e")
import hand3_hubc as HC
from hand3_finger import to_trimesh
a = HC.assembly()
COL = dict(alu=(190, 195, 205), pa12=(225, 222, 210), cf=(55, 58, 64), resin=(120, 130, 145), tpu=(200, 60, 55), steel=(150, 150, 155),
           servo=(40, 90, 170), motor=(25, 25, 28), carbon=(30, 30, 30), cord=(240, 200, 40), brass=(200, 160, 60), link=(230, 110, 40), spring=(90, 160, 90))
def mat(n):
    if n.startswith(("grip", "hub_")): return "alu"
    if n.startswith(("ph",)): return "pa12"
    if n.startswith(("pad", "heel")): return "tpu"
    if n.startswith(("rlink",)): return "link"
    if n.startswith(("cord", "rcord")): return "cord"
    if n.startswith(("stray", "slid")): return "spring"
    if n.startswith(("tube", "hw_rscrew", "hw_rnut", "hw_sscrew", "hw_idler")): return "steel"
    if n.startswith("hw_servo"): return "servo"
    if n.startswith("hw_motor"): return "motor"
    if n.startswith("hw_rod"): return "carbon"
    if n.startswith(("frame", "cage", "ring_", "yoke", "tie_ring", "hw_base_spool", "hw_curl_spool")): return "cf"
    return "resin"
def explode(n, c):
    r = np.r_[c[:2], 0.0]; rn = np.linalg.norm(r); u = r / rn if rn > 1e-6 else np.zeros(3); z = np.array([0, 0, 1.0])
    if n == "frame": return -45 * z
    if n.startswith(("hw_servo", "hw_base_spool", "hw_curl_spool", "hw_idler", "cord_")): return -45 * z + 45 * np.sign(c[1] or 1) * np.array([0, 1, 0])
    if n.startswith("hw_rod"): return -20 * z
    if n == "tie_ring": return 15 * z
    if n.startswith("yoke"): return 40 * u - 10 * z
    if n == "ring_out": return -12 * z
    if n == "ring_in": return -28 * z
    if n.startswith(("rlink", "hw_rscrew", "hw_rnut", "rcordlo")): return 30 * u - 20 * z
    if n == "cage": return 0 * z
    if n.startswith("hw_motor"): return 0 * z
    if n == "hub_lower": return 30 * z
    if n.startswith(("tube", "rcordup")): return 42 * z
    if n.startswith("grip"): return 30 * u + 42 * z
    if n.startswith(("ph", "pad")):
        j = int(n[-1]); base = (30 + 18 * j) * u + 42 * z
        return base + (8 * z if n.startswith("pad") else 0)
    if n == "hub_upper": return 58 * z
    if n.startswith("stray1"): return 72 * z + 15 * u
    if n.startswith("stray2"): return 82 * z + 15 * u
    if n.startswith(("slid", "hw_sscrew")): return 92 * z + 15 * u
    if n == "heel": return 105 * z
    return 0 * z
scene = trimesh.Scene(); tot = 0
for view, dx in (("assembled", -150.0), ("exploded", 150.0)):
    for n, (m, rot) in sorted(a.items()):
        if m.is_empty(): continue
        t = to_trimesh(m); t.merge_vertices(); t.fix_normals()
        c = t.bounds.mean(0)
        off = np.array([dx, 0, 0]) + (explode(n, c) if view == "exploded" else 0)
        t.apply_translation(off - [0, 0, 60])
        t.apply_scale(0.001)                                    # glTF is in metres
        t.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(name=mat(n), baseColorFactor=list(COL[mat(n)]) + [255],
                                                 metallicFactor=0.35 if mat(n) in ("alu", "steel") else 0.0, roughnessFactor=0.55, doubleSided=True))
        scene.add_geometry(t, node_name=f"{view}/{n}", geom_name=f"{view}_{n}")
        tot += 1
scene.export("/home/claude/octodrone/hand3_final_breakdown.glb")
import os; print(tot, "nodes", round(os.path.getsize("/home/claude/octodrone/hand3_final_breakdown.glb") / 1e6, 1), "MB")
