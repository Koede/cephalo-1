"""Writes coupler_site/: a three.js viewer of connector rev B.6 (hand/arm coupler) with the insert-and-lock sequence,
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
    """Rev B.6 motor phase leads, 14 AWG silicone (OD 3.5): ESC -> female cup tail (tip bay) and male cup -> groove ->
    90 deg bend R 3 -> down hand3's r < 7 centre channel toward the motor."""
    P = HB.P; rw = P["wire_od"] / 2; fz0 = P["z_flange"][0]; zc = fz0 + 0.15 + rw
    hand, arm = [], []
    for az in P["az_bullet"]:
        u = np.array([np.cos(np.radians(az)), np.sin(np.radians(az))])
        pts = [(*(u * P["r_bullet"]), -P["male_body"] + 1.0), (*(u * (P["r_bullet"] - 2.5)), zc)]
        pts += [(*(u * (P["lead_rv"] + P["lead_bend_r"])), zc)]
        for q in np.linspace(0, 90, 7)[1:]:
            rr = P["lead_rv"] + P["lead_bend_r"] * np.cos(np.radians(q)); zz = zc - P["lead_bend_r"] * np.sin(np.radians(q))
            pts.append((*(u * rr), zz))
        pts.append((*(u * P["lead_rv"]), fz0 - 30.0))
        hand.append(_capsules(pts, rw))
        top = P["fem_recess"] + P["bul_len"]
        arm.append(_capsules([(*(u * P["r_bullet"]), top - 2.0), (*(u * P["r_bullet"]), top + 6.0), (*(u * 9.0), top + 14.0)], rw))
    U_ = lambda L: mf.Manifold.batch_boolean(L, mf.OpType.Add)
    return U_(arm), U_(hand)

parts = {}
# arm side (fixed); 'g' = group: arm (static), collar (turns), hand (slides)
tip = F.C3.cyl(20.0, P["z_back"], 46.0, n=64) - F.C3.cyl(18.0, P["z_back"] + 2.5, 47, n=64) - F.C3.cyl(P["pinboard_r"], P["z_back"] - 1, 47, n=48)
for _az in P["az_bullet"]:                    # B.6: end face holes for the female bullet tails + ESC leads
    tip -= F.C3.cyl(3.0, P["z_back"] - 1, P["z_back"] + 3.0, c=(*HB.at(P["r_bullet"], _az), 0), n=24)
parts["arm tip bay (sketch)"] = dict(m=tip, g="arm", c="#6b7079", o=0.35)
parts["socket body"] = dict(m=HB.arm_socket(), g="arm", c="#2b2d31")
parts["nose ring"] = dict(m=HB.nose_ring(), g="arm", c="#3d4046")
parts["female bullets (Castle 4 mm)"] = dict(m=U(HB.female_bullets()), g="arm", c="#c9a227")
_al, _hl = phase_leads()
parts["ESC phase leads 14 AWG (to the tip-bay ESC)"] = dict(m=_al, g="arm", c="#b0302a")
parts["3 x M3x20 DIN 7984"] = dict(m=U(HB.screws()), g="arm", c="#b8bcc4")
parts["detent ball 3 mm"] = dict(m=HB.detent_ball(), g="arm", c="#e8e8f0")
parts["O-ring 21 x 1.0 NBR 70A (on Ø22, 3 x 0.05 shims)"] = dict(m=HB.oring(), g="arm", c="#1a7f37")
parts["locking collar"] = dict(m=HB.collar(0), g="collar", c="#d2542d")
# hands
parts["hand base"] = dict(m=HB.hand_base(), g="hand", c="#30343a", h="all")
parts["male bullets"] = dict(m=U(HB.male_bullets()), g="hand", c="#c9a227", h="all")
parts["motor phase leads 14 AWG (to the hand motor)"] = dict(m=_hl, g="hand", c="#b0302a", h="all")
pf = F.pack_frame_b() - F.holes(0.0)
parts["hand3 pack frame (lower)"] = dict(m=F.to_conn(pf.trim_by_plane([0, 0, -1], -40.0), 0.0), g="hand", c="#e9e9e4", h="hand3")
hw = F.C3.hardware()
sv = U([m for k, (m, _, _) in hw.items() if k.startswith("servo")]).trim_by_plane([0, 0, -1], -40.0)
parts["hand3 servos (tier 1)"] = dict(m=F.to_conn(sv, 0.0), g="hand", c="#1f4e8c", h="hand3")
parts["hand3 base screws M3x8 DIN 7991 (3, B.6)"] = dict(m=F.to_conn(F.hand_screws_h3(0.0), 0.0), g="hand", c="#b8bcc4", h="hand3")
W = WH.parts()
for k, c in (("housing", "#3a3f46"), ("wheel_hub", "#e9e9e4"), ("tyre", "#202022"), ("motor", "#8a1d1d"), ("carrier2", "#9aa0a8")):
    parts["wheel hand: " + k] = dict(m=W[k], g="hand", c=c, h="wheel", o=0.5 if k == "housing" else 1.0)
os.makedirs("coupler_site", exist_ok=True)
data = {k: dict(pack(v["m"]), g=v["g"], c=v["c"], h=v.get("h", ""), o=v.get("o", 1.0)) for k, v in parts.items()}
open("coupler_site/coupler_data.js", "w").write("window.CPL=" + json.dumps(data) + ";\n")
print({k: v["nt"] for k, v in data.items()}, os.path.getsize("coupler_site/coupler_data.js") / 1e6, "MB")
html = open("coupler_viewer_template.html").read().replace("__TWIST__", str(P["twist"]))
open("coupler_site/index.html", "w").write(html)
