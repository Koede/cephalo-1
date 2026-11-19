"""Writes coupler_site/: a three.js viewer of connector rev B.1 (hand/arm coupler) with the insert-and-lock sequence,
a section cut, an explode slider and three hands (bare base, hand3 base, wheel hand).  Meshes come straight from the
CAD (handmount_b.py, coupler_hand3_fit.py, wheelhand.py) and are packed as base64 Float32/Uint32 in coupler_data.js.
python3 coupler_viewer.py"""
import os, json, base64
import numpy as np
import manifold3d as mf
import handmount_b as HB
import coupler_hand3_fit as F
import wheelhand as WH

def pack(m):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3].astype(np.float32); t = np.asarray(me.tri_verts).astype(np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), t=base64.b64encode(t.tobytes()).decode(), nt=int(len(t)))

U = lambda L: mf.Manifold.batch_boolean(L, mf.OpType.Add)
P = HB.P
parts = {}
# arm side (fixed); 'g' = group: arm (static), collar (turns), hand (slides)
tip = F.C3.cyl(20.0, P["z_back"], 46.0, n=64) - F.C3.cyl(P["pinboard_r"], P["z_back"] - 1, 47, n=48)
parts["arm tip bay (sketch)"] = dict(m=tip, g="arm", c="#6b7079", o=0.35)
parts["socket body"] = dict(m=HB.arm_socket(), g="arm", c="#2b2d31")
parts["nose ring"] = dict(m=HB.nose_ring(), g="arm", c="#3d4046")
parts["female bullets (Castle 4 mm)"] = dict(m=U(HB.female_bullets()), g="arm", c="#c9a227")
parts["3 x M3x20 DIN 7984"] = dict(m=U(HB.screws()), g="arm", c="#b8bcc4")
parts["detent ball 3 mm"] = dict(m=HB.detent_ball(), g="arm", c="#e8e8f0")
parts["locking collar"] = dict(m=HB.collar(0), g="collar", c="#d2542d")
# hands
parts["hand base"] = dict(m=HB.hand_base(), g="hand", c="#30343a", h="all")
parts["male bullets"] = dict(m=U(HB.male_bullets()), g="hand", c="#c9a227", h="all")
pf = F.pack_frame_b() - F.holes(0.0)
parts["hand3 pack frame (lower)"] = dict(m=F.to_conn(pf.trim_by_plane([0, 0, -1], -40.0), 0.0), g="hand", c="#e9e9e4", h="hand3")
hw = F.C3.hardware()
sv = U([m for k, (m, _, _) in hw.items() if k.startswith("servo")]).trim_by_plane([0, 0, -1], -40.0)
parts["hand3 servos (tier 1)"] = dict(m=F.to_conn(sv, 0.0), g="hand", c="#1f4e8c", h="hand3")
parts["hand3 base screws M3x8 DIN 7991"] = dict(m=F.to_conn(F.hand_screws_h3(0.0), 0.0), g="hand", c="#b8bcc4", h="hand3")
W = WH.parts()
for k, c in (("housing", "#3a3f46"), ("wheel_hub", "#e9e9e4"), ("tyre", "#202022"), ("motor", "#8a1d1d"), ("carrier2", "#9aa0a8")):
    parts["wheel hand: " + k] = dict(m=W[k], g="hand", c=c, h="wheel", o=0.5 if k == "housing" else 1.0)
os.makedirs("coupler_site", exist_ok=True)
data = {k: dict(pack(v["m"]), g=v["g"], c=v["c"], h=v.get("h", ""), o=v.get("o", 1.0)) for k, v in parts.items()}
open("coupler_site/coupler_data.js", "w").write("window.CPL=" + json.dumps(data) + ";\n")
print({k: v["nt"] for k, v in data.items()}, os.path.getsize("coupler_site/coupler_data.js") / 1e6, "MB")
html = open("coupler_viewer_template.html").read().replace("__TWIST__", str(P["twist"]))
open("coupler_site/index.html", "w").write(html)
