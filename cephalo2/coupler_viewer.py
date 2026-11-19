"""Writes coupler_site/: a three.js viewer of connector rev B.7 (hand/arm coupler) with the insert-and-lock sequence,
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

def _capsules(pts, r):
    """Tube through 3D points as hulls of consecutive spheres (for wire leads)."""
    sp = [mf.Manifold.sphere(r, 16).translate([float(a) for a in p]) for p in pts]
    return mf.Manifold.batch_boolean([mf.Manifold.batch_hull([sp[i], sp[i + 1]]) for i in range(len(sp) - 1)], mf.OpType.Add)


def phase_leads():
    """ESC phase leads on the arm side, 14 AWG silicone (OD 3.5): ESC -> female cup tail in the tip bay.  The hand side
    (rev B.7) comes from hubE_leads: exit slot, dive, back face of hub E's servo stack inside the cover rail."""
    P = HB.P; rw = P["wire_od"] / 2; arm = []
    for az in P["az_bullet"]:
        u = np.array([np.cos(np.radians(az)), np.sin(np.radians(az))]); top = P["fem_recess"] + P["bul_len"]
        arm.append(_capsules([(*(u * P["r_bullet"]), top + rw + 0.3), (*(u * P["r_bullet"]), top + 6.0), (*(u * 9.0), top + 14.0)], rw))
    return mf.Manifold.batch_boolean(arm, mf.OpType.Add)

parts = {}
# arm side (fixed); 'g' = group: arm (static), collar (turns), hand (slides)
tip = F.C3.cyl(20.0, P["z_back"], 46.0, n=64) - F.C3.cyl(18.0, P["z_back"] + 2.5, 47, n=64) - F.C3.cyl(P["pinboard_r"], P["z_back"] - 1, 47, n=48)
for _az in P["az_bullet"]:                    # B.6: end face holes for the female bullet tails + ESC leads
    tip -= F.C3.cyl(3.0, P["z_back"] - 1, P["z_back"] + 3.0, c=(*HB.at(P["r_bullet"], _az), 0), n=24)
parts["arm tip bay (sketch)"] = dict(m=tip, g="arm", c="#6b7079", o=0.35)
parts["socket body"] = dict(m=HB.arm_socket(), g="arm", c="#2b2d31")
parts["nose ring"] = dict(m=HB.nose_ring(), g="arm", c="#3d4046")
parts["female bullets (Castle 4 mm)"] = dict(m=U(HB.female_bullets()), g="arm", c="#c9a227")
_al = phase_leads()
parts["ESC phase leads 14 AWG (to the tip-bay ESC)"] = dict(m=_al, g="arm", c="#b0302a")
parts["3 x M3x20 DIN 7984"] = dict(m=U(HB.screws()), g="arm", c="#b8bcc4")
parts["detent ball 3 mm"] = dict(m=HB.detent_ball(), g="arm", c="#e8e8f0")
parts["O-ring 21 x 1.0 NBR 70A (on Ø22, 3 x 0.05 shims)"] = dict(m=HB.oring(), g="arm", c="#1a7f37")
parts["locking collar"] = dict(m=HB.collar(0), g="collar", c="#d2542d")
# hands
parts["hand base"] = dict(m=HB.hand_base(), g="hand", c="#30343a", h="all")
parts["male bullets"] = dict(m=U(HB.male_bullets()), g="hand", c="#c9a227", h="all")
import sys
sys.path.insert(0, "hand3e"); _cwd = os.getcwd(); os.chdir("hand3e")
import hand3_hubc as HC
os.chdir(_cwd)
import hubE_leads as HL
A = HL.hand3_to_conn_matrix(0.0)[:3, :4]; LM = HL.lead_meshes("conn")
parts["motor phase leads 14 AWG"] = dict(m=U([LM[f"lead {k}"] for k in "BAC"]), g="hand", c="#b0302a", h="hand3")
parts["solder joints (heat shrink)"] = dict(m=U([LM[f"joint {k}"] for k in "BAC"]), g="hand", c="#111111", h="hand3")
parts["motor stubs 18 AWG"] = dict(m=U([LM[f"stub {k}"] for k in "BAC"]), g="hand", c="#d2452d", h="hand3")
parts["signal wires (5 x 26 AWG)"] = dict(m=LM["sig"], g="hand", c="#e6c229", h="hand3")
hw = HC.hardware()
parts["hand3 hub E frame"] = dict(m=HC.frame().transform(A), g="hand", c="#e9e9e4", h="hand3")
parts["hand3 servos 2 x STS3215"] = dict(m=U([m for k, m in hw.items() if k.startswith("servo")]).transform(A), g="hand", c="#1f4e8c", h="hand3")
parts["hand3 spools, idlers, rods"] = dict(m=U([m for k, m in hw.items() if not k.startswith(("servo", "motor"))]).transform(A), g="hand", c="#9aa1aa", h="hand3")
parts["hand3 motor base (SE 2816)"] = dict(m=hw["motor_base"].transform(A), g="hand", c="#555a63", h="hand3")
parts["lead cover rail"] = dict(m=HL.rail().transform(A), g="hand", c="#f08a24", h="hand3", o=0.6)
parts["rail screws 2 x M2x6"] = dict(m=U(HL.rail_screws()).transform(A), g="hand", c="#b8bcc4", h="hand3")
W = WH.parts()
for k, c in (("housing", "#3a3f46"), ("wheel_hub", "#e9e9e4"), ("tyre", "#202022"), ("motor", "#8a1d1d"), ("carrier2", "#9aa0a8")):
    parts["wheel hand: " + k] = dict(m=W[k], g="hand", c=c, h="wheel", o=0.5 if k == "housing" else 1.0)
os.makedirs("coupler_site", exist_ok=True)
data = {k: dict(pack(v["m"]), g=v["g"], c=v["c"], h=v.get("h", ""), o=v.get("o", 1.0)) for k, v in parts.items()}
open("coupler_site/coupler_data.js", "w").write("window.CPL=" + json.dumps(data) + ";\n")
print({k: v["nt"] for k, v in data.items()}, os.path.getsize("coupler_site/coupler_data.js") / 1e6, "MB")
html = open("coupler_viewer_template.html").read().replace("__TWIST__", str(P["twist"]))
open("coupler_site/index.html", "w").write(html)
