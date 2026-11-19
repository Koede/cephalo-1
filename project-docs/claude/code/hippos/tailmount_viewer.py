"""Rotatable 3D viewer for the Hippos-1 tail mount.  python3 tailmount_viewer.py -> tailmount_viewer.html
Meshes from tailmount.py / sff_core.py; numbers from tailmount_checks.json."""
import json, base64, numpy as np
import sff_core as H, tailmount as M
from sff_core import P

def pack(m, col, name, group):
    t = H.to_trimesh(m)
    t.merge_vertices()
    v = np.asarray(t.vertices, np.float32)[:, [0, 2, 1]] * np.array([1, 1, -1], np.float32)   # model z -> three y
    f = np.asarray(t.faces, np.uint32)                       # (x, z, -y) is a proper rotation: winding kept
    return dict(name=name, group=group, color=col, v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode())

def box(x, y, z0, z1, cx=0, cy=0):
    return H.mf.Manifold.cube([x, y, z1 - z0]).translate([cx - x / 2, cy - y / 2, z0])

parts = [
    pack(M.rump_socket(), "#e0a030", "rump socket + sleeve", "fixed"),
    pack(H.pin_board(), "#2e7d32", "pin board (5 spring pins)", "fixed"),
    pack(H.hand_base(), "#2f6db3", "standard SFF base", "tail"),
    pack(H.pad_board(), "#c9a227", "pad board (ring pads)", "tail"),
    pack(M.tail_adapter(), "#5aa0e6", "tail adapter (skirt)", "tail"),
]
# context, not CAD: rump bulkhead and the tail's root servo + first link
bulk = box(90, 90, 10.9, 14.9) - H.cyl(5, 9, 16)
parts.append(pack(bulk, "#c9ced6", "rump bulkhead (context)", "fixed"))
servo = box(25, 45, -72, -37) + box(56, 56, -37, -35) - H.cyl(5, -40, -34)
parts.append(pack(servo, "#3a3a3a", "root STS3250 + bracket (context)", "tail"))
link = H.cyl(9, -150, -72, 48) + box(28, 50, -160, -150)
parts.append(pack(link, "#9aa3ad", "tail link 1 (context)", "tail"))
for p in parts:
    p["ghost"] = "context" in p["name"]

ck = json.load(open("tailmount_checks.json"))
ramp = [[float(th), H.ramp_dz(th)] for th in np.arange(-60, 0.01, 1.0)]
data = dict(parts=parts, ramp=ramp, ck=dict(
    rock=ck["rocking"]["worst_deg"], tip=ck["rocking"]["tip_wobble_mm"], plain_tip=ck["rocking"]["plain_tip_mm"],
    defl=ck["twist"]["max_latch_defl"], mass=ck["mass_g"], loads=ck["loads"], print=ck["print"]))

html = open("tailmount_viewer_template.html").read().replace("/*DATA*/", json.dumps(data))
open("tailmount_viewer.html", "w").write(html)
print("written", len(html) / 1e6, "MB")
