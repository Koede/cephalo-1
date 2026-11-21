"""Checks for hand3 finger rotation S3, cam sleeve (2026-11-20).  Writes hand3_cam_s3_check.json.
1) r-z sweeps of everything that spins (roots at any angle, sleeve at both ends of its stroke, bracket, screw, nut) against the
   static parts, at lift 0 (flight) and lift 2.0 (rotation stage).
2) Along the whole sleeve stroke (0.1 mm steps): every root at its slot angle with the sleeve where it is - exact booleans of
   roots x sleeve (pin excluded), pins x sleeve (pin must sit in its slot), roots x bracket, roots x each other, roots x counterweight.
   Same with all four fingers curled 30 % (the regrip).
3) Sweep of roots vs base disc / top disc / cage (sliding faces).
4) One finger curling alone at the flight cross (5 % steps).
5) Stage 2 at each layout: curl drivers up under the parked roots; the sleeve driver's key in its screw at stage 1 and 2.
6) Static parts among themselves.  7) Numbers."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hub_v3 as V3, hand3_hubc as HC, hand3_multidigit as MD
import hand3_cam_s3 as S
from scipy.spatial import cKDTree

res = {}


def rz(m, n=6):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, n)])
    return np.c_[np.hypot(e[:, 0], e[:, 1]), e[:, 2]]


def U(L):
    out = None
    for m in L: out = m if out is None else out + m
    return out


fing = U(list(V3.finger(0).values()))
ROOT = S.root_s3(0); PIN = S.pin()
SPIN = dict(root=ROOT, screw=MD.screw(), slider_top=MD.slider(0.0), slider_bottom=MD.slider(MD.STROKE), pin=PIN, finger=fing,
            sleeve_low=S.sleeve(), sleeve_high=S.sleeve().translate([0, 0, S.STROKE]), bracket=S.bracket(), screw9=S.screw9(),
            nut9_low=S.nut9(), nut9_high=S.nut9().translate([0, 0, S.STROKE]), counterweight=S.counterweight())
PTS = {n: rz(m) for n, m in SPIN.items()}

res["static_sweep_gap_mm"] = {}
for lift in (0.0, S.STAGE1):
    st = {"curl drivers": MD.drivers(False).translate([0, 0, lift]), "sleeve driver": S.driver9(S.driver9_dz(lift)),
          "lift ring": S.lift_ring_s3().translate([0, 0, lift]), "frame": S.frame_s3(), "motor base": HC.hardware()["motor_base"]}
    out = {}
    for sn, sm in st.items():
        tr = cKDTree(rz(sm)); worst = (1e9, "")
        for rn, p in PTS.items():
            if lift > 0 and rn not in ("root", "screw", "slider_top", "slider_bottom", "pin", "finger"): continue   # rotor parked: only roots move
            d, _ = tr.query(p); worst = min(worst, (float(d.min()), rn))
        out[sn] = dict(gap=round(worst[0], 2), nearest=worst[1])
    res["static_sweep_gap_mm"][f"lift {lift}"] = out
print("static", json.dumps(res["static_sweep_gap_mm"]), flush=True)

# 2) along the stroke
def finger_world(az, c):
    fr = C.finger_frames(0, 0.0, tuple(f * c for f in H.FOLD_MAX)); R = C.parts()
    Rz = np.eye(4); a = np.radians(az - C.FAZ[0]); Rz[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return U([V3.B.phalanx_v3(j).transform((Rz @ np.asarray(fr[j + 1], float))[:3, :4]) + R[f"pad{j + 1}"].transform((Rz @ np.asarray(fr[j + 1], float))[:3, :4]) for j in range(3)])


SL = S.sleeve(); BR = S.bracket(); CW = S.counterweight(); S9 = S.screw9()
res["stroke"] = {}
for c in (0.0, 0.3):
    worst = {}
    for u in np.arange(S.PRE_U[0], S.PRE_U[-1] + 1e-6, 0.1):
        th = S.theta_at(u); dz = S.sleeve_dz(u); sl = SL.translate([0, 0, dz])
        bodies = []
        for k in range(4):
            b = S.root_s3(th[k]) + MD.screw().rotate([0, 0, th[k]]) + MD.slider(MD.STROKE * c).rotate([0, 0, th[k]])
            if c > 0: b += finger_world(th[k], c)
            bodies.append(b)
            pins = PIN.rotate([0, 0, th[k]])
            for nm, v in (("root x sleeve", (b ^ sl).volume()), ("pin x sleeve", (pins ^ sl).volume()), ("root x bracket", (b ^ (BR + S9)).volume()),
                          ("root x counterweight", (b ^ CW).volume())):
                if v > worst.get(nm, (0, 0))[0]: worst[nm] = (round(v, 4), round(float(u), 2))
        for i in range(4):
            for j in range(i + 1, 4):
                v = (bodies[i] ^ bodies[j]).volume()
                if v > worst.get("root x root", (0, 0))[0]: worst["root x root"] = (round(v, 4), round(float(u), 2))
    res["stroke"][f"curl {int(c * 100)} %"] = worst or "no contact"
    print("stroke", c, worst or "no contact", flush=True)

# 3) sliding faces
res["rotor_sweep_gap_mm"] = {}
for sn, sm in {"base disc": S.base_disc_s3(), "top disc": V3.top_disc(), "cage": V3.cage_v3()}.items():
    tr = cKDTree(rz(sm)); worst = (1e9, "")
    for rn in ("root", "pin", "screw", "slider_bottom"):
        d, _ = tr.query(PTS[rn]); worst = min(worst, (float(d.min()), rn))
    res["rotor_sweep_gap_mm"][sn] = dict(gap=round(worst[0], 2), nearest=worst[1])
tr = cKDTree(rz(V3.cage_v3()))
for nm in ("sleeve_low", "sleeve_high"):
    d, _ = tr.query(PTS[nm]); res["rotor_sweep_gap_mm"][f"cage x {nm}"] = round(float(d.min()), 2)
print("rotor", res["rotor_sweep_gap_mm"], flush=True)

# 4) solo curl at the flight cross
cross_u = S.PRE_U[1]; th = S.theta_at(cross_u)
others = U([S.root_s3(a) + finger_world(a, 0.0) for a in th[1:]]) + S.base_disc_s3() + V3.top_disc() + C.parts()["heel"] + SL.translate([0, 0, S.sleeve_dz(cross_u)])
first = None
for c in np.arange(0.05, 1.001, 0.05):
    v = (finger_world(th[0], c) ^ (others + S.root_s3(th[0]) - S.root_s3(th[0]))).volume()
    if v > 0.01: first = round(float(c), 2); res["solo_curl_touch"] = dict(curl=first, mm3=round(v, 3)); break
if first is None: res["solo_curl_touch"] = "none to 100 %"
print("solo", res["solo_curl_touch"], flush=True)

# 5) stage 2 at each layout
up = MD.drivers(False).translate([0, 0, MD.KEY["lift"]]) + S.lift_ring_s3().translate([0, 0, MD.KEY["lift"]])
d9 = S.driver9(S.driver9_dz(MD.KEY["lift"]))
res["stage2"] = {}
for name, az in S.LAYOUTS:
    u = S.PRE_U[[l[0] for l in S.LAYOUTS].index(name)]; sl = SL.translate([0, 0, S.sleeve_dz(u)])
    r = U([S.root_s3(a) + MD.screw().rotate([0, 0, a]) + MD.slider(MD.STROKE).rotate([0, 0, a]) for a in az])
    missing = [a for a in az if a not in MD.DRIVER_AZ]
    res["stage2"][name] = dict(drivers_x_roots=round((up ^ r).volume(), 3), sleeve_driver_x_screw=round((d9 ^ (S9 + sl + BR)).volume(), 3),
                               roots_off_drivers=missing)
res["stage1_key9"] = dict(key_top=MD.KEY["top_retracted"] + S.STAGE1, slot=(S.SCREW9["z"][0], S.SCREW9["z"][0] + S.SCREW9["slot"][1]),
                          flight_gap=round(S.SCREW9["z"][0] - MD.KEY["top_retracted"], 2), curl_key_top_stage1=MD.KEY["top_retracted"] + S.STAGE1,
                          leg_bottom=MD.LEG["z0"])
print("stage2", res["stage2"], res["stage1_key9"], flush=True)

# 6) static-static
st6 = {}
for lift in (0.0, S.STAGE1, MD.KEY["lift"]):
    dr = MD.drivers(False).translate([0, 0, lift]); d9 = S.driver9(S.driver9_dz(lift)); li = S.lift_ring_s3().translate([0, 0, lift]); fr = S.frame_s3()
    st6[f"lift {lift}"] = {"drivers x frame": round((dr ^ fr).volume(), 3), "sleeve driver x frame": round((d9 ^ fr).volume(), 3),
                           "sleeve driver x drivers": round((d9 ^ dr).volume(), 3), "lift ring x frame": round((li ^ fr).volume(), 3),
                           "sleeve driver x lift ring": round((d9 ^ li).volume(), 3)}
res["static_static_mm3"] = st6
print("static-static", st6, flush=True)

res["numbers"] = S.numbers()
json.dump(res, open(os.path.join(HERE, "hand3_cam_s3_check.json"), "w"), indent=1, default=float)
print(json.dumps(res["numbers"], default=float))
