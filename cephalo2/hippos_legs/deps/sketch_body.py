"""Forest's sketch (2026-11-15): the body sweeps up into a forward-leaning neck; the collar is the top of the neck.
Traced from the photo (pixel coords), scaled so the collar is 236 mm.  A = as drawn, B = same shape, body scaled down
around a fixed collar, C = B with the forelegs moved forward under the neck base.  Masses and torques are estimates.
python3 sketch_body.py -> sketch_body.json, sheet_sketch_body.png"""
import sys, json

import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon, Circle, Ellipse
import sk3d, centaur_study as CS
G = 9.81
# ---------------------------------------------------------------- traced sketch (px, y down)
OUT = [(165,900),(205,872),(300,840),(420,820),(560,815),(640,815),(700,800),(745,770),(780,728),(918,770),
       (905,800),(885,860),(860,920),(840,980),(830,1010),(800,1040),(760,1060),(700,1095),(600,1092),(480,1090),
       (370,1088),(320,1072),(290,1030),(275,985),(245,945),(205,912)]
CL, CR = 8, 9                              # collar ends in OUT
HIP_R, HIP_F = (360,1012), (735,1028)
LEG_F = [(735,1028),(765,1325),(648,1525),(708,1596)]         # shoulder, carpus, fetlock, hoof (drawn)
GROUND = 1600
K = 236.0 / np.hypot(918-780, 770-728)     # mm per px from the collar
def mm(p): return np.array([(p[0]-715)*K, (GROUND-p[1])*K])   # x forward from the drawn hoof, z up
out0 = np.array([mm(p) for p in OUT]); cl, cr = out0[CL], out0[CR]
cc = (cl + cr) / 2; band = (cr - cl) / np.linalg.norm(cr - cl)
n_up = np.array([-band[1], band[0]]);  n_up = n_up if n_up[1] > 0 else -n_up
TILT = np.degrees(np.arctan2(n_up[0], n_up[1]))               # neck axis, forward of vertical

def build(f, dx_front=0.0, m_scale=None):
    """f: scale of everything except the collar, about the collar centre's foot point.  dx_front: forelegs forward (mm)."""
    o = cc.copy(); o[1] = 0.0
    out = (out0 - o) * f + o
    # keep the collar at 236: re-seat the scaled collar centre at the top of the scaled neck
    c_new = (out[CL] + out[CR]) / 2
    out[CL] = c_new - 118 * band; out[CR] = c_new + 118 * band
    if f < 1:                                     # smooth the back of the neck into the wider collar
        out[7] = 0.5 * (out[6] + out[CL]) + [0, 10]; out[10] = out[CR] + 0.35 * (out[11] - out[CR])
    if dx_front:                                  # chest (brisket) grows forward with the forelegs
        for i, w in zip(range(12, 19), (0.35, 0.7, 1.0, 1.0, 1.0, 0.7, 0.35)):
            out[i, 0] += w * dx_front
    hipF = (mm(HIP_F) - o) * f + o + [dx_front, 0]; hipR = (mm(HIP_R) - o) * f + o
    legF = [(mm(p) - o) * f + o + [dx_front, 0] for p in LEG_F]
    legF[-1][1] = 0.0
    legR = [hipR, hipR + (legF[1]-legF[0]) * [-1, 1] + [0.15*(legF[1][0]-legF[0][0]), 0], None, None]
    legR[2] = hipR + (legF[2]-legF[0]) * [-1, 1]; legR[3] = np.array([hipR[0] - (legF[3][0]-legF[0][0]), 0.0])
    return dict(f=f, out=out, cc=c_new, hipF=hipF, hipR=hipR, legF=legF, legR=legR, dx=dx_front)

def analyse(b, name):
    out = b["out"]; x, z = out[:, 0], out[:, 1]
    A = 0.5 * np.sum(x * np.roll(z, -1) - np.roll(x, -1) * z)
    cx = np.sum((x + np.roll(x, -1)) * (x * np.roll(z, -1) - np.roll(x, -1) * z)) / (6 * A)
    cz = np.sum((z + np.roll(z, -1)) * (x * np.roll(z, -1) - np.roll(x, -1) * z)) / (6 * A)
    leg = np.linalg.norm(b["legF"][0] - b["legF"][1]) + np.linalg.norm(b["legF"][1] - b["legF"][2]) + np.linalg.norm(b["legF"][2] - b["legF"][3])
    s = b["hipF"][1] / 361.0                                     # size against the study horse (hip height 361)
    m_h = 11.8 * s ** 1.15                                       # est., same scaling as centaur_bigbody.py
    oc = b["cc"] + n_up * CS.COM_H * 1e3                         # octopus CoM, 60 mm up the tilted axis
    xf, xr = b["legF"][3][0], b["legR"][3][0]
    m = m_h + CS.OCTO_M
    xc = (m_h * cx + CS.OCTO_M * oc[0]) / m; zc = (m_h * cz + CS.OCTO_M * oc[1]) / m
    share = (xc - xr) / (xf - xr)
    k = (m * share) / (21.4 * 0.62) * s
    sheath_end = b["cc"] - n_up * 228
    inside = Path(out).contains_point(sheath_end)
    top = b["cc"] + n_up * 224 + band * 0
    return dict(name=name, f=b["f"], dx=b["dx"], length=x.max()-x.min(), depth=None, height_plate=b["cc"][1], height_top=top[1] + 100,
                shoulder_h=b["hipF"][1], hip_span=b["hipF"][0]-b["hipR"][0], m_h=m_h, m=m, share=share,
                collar_ahead_of_front_feet=b["cc"][0]-xf, tip_fwd=np.degrees(np.arctan2(xf - xc, zc)), tip_back=np.degrees(np.arctan2(xc - xr, zc)),
                tip_side=np.degrees(np.arctan2(max(0.17 * s * 1e3, 140), zc)), knee_rms=5.51 * k, knee_peak=10.74 * k,
                sheath_inside=bool(inside), seat_shear=CS.OCTO_M * G * np.sin(np.radians(TILT)), leg=leg, s=s, com=(xc, zc), ocom=oc)

# B: shrink until the shoulder is at the M-size hip height (~490)
fB = 490.0 / mm(HIP_F)[1]
bA, bB = build(1.0), build(fB)
rA, rB = analyse(bA, "A as drawn"), analyse(bB, "B compact")
# C: forelegs as far forward as the chest allows (shoulder 45 mm inside the outline), packs at the croup on the sled
def inside_by(out, pt, d):
    P = Path(out); return all(P.contains_point(pt + d * np.array([np.cos(a), np.sin(a)])) for a in np.linspace(0, 2*np.pi, 16, endpoint=False))
def with_sled(dx):
    b = build(fB, dx); r = analyse(b, "C forelegs under the neck")
    xc, zc = r["com"]; m = r["m"]
    xr_body = b["out"][:, 0].min() + 120
    xc2 = xc + 1.62 * (xr_body - xc) / m
    xf, xr = b["legF"][3][0], b["legR"][3][0]
    r["share_sled"] = (xc2 - xr) / (xf - xr); r["tip_fwd_sled"] = np.degrees(np.arctan2(xf - xc2, zc)); r["com"] = (xc2, zc)
    k = (m * r["share_sled"]) / (21.4 * 0.62) * r["s"]; r["knee_rms"], r["knee_peak"] = 5.51 * k, 10.74 * k
    r["collar_ahead_of_front_feet"] = b["cc"][0] - xf
    return b, r
for dx in np.arange(0, 400, 2.0):
    bC, rC = with_sled(dx)
    if rC["share_sled"] <= 0.62: break
rC["share"], rC["tip_fwd"] = rC["share_sled"], rC["tip_fwd_sled"]
R = [rA, rB, rC]; B = [bA, bB, bC]
json.dump(dict(tilt=TILT, k=K, rows=R), open("sketch_body.json", "w"), indent=1, default=lambda v: v.tolist() if hasattr(v, "tolist") else float(v))
print(f"neck axis {TILT:.1f} deg forward of vertical; scale {K:.3f} mm/px")
for r in R:
    print(f"{r['name']:38s} len {r['length']:.0f} shoulder {r['shoulder_h']:.0f} plate {r['height_plate']:.0f} top {r['height_top']:.0f} span {r['hip_span']:.0f} "
          f"| horse {r['m_h']:.1f} centaur {r['m']:.1f} kg | front {r['share']*100:.0f} % collar ahead of front feet {r['collar_ahead_of_front_feet']:.0f} "
          f"| tip fwd {r['tip_fwd']:.1f} back {r['tip_back']:.1f} side {r['tip_side']:.1f} | knee {r['knee_rms']:.1f}/{r['knee_peak']:.1f} | sheath inside {r['sheath_inside']} | shear {r['seat_shear']:.0f} N | forelegs +{r['dx']:.0f}")

# ---------------------------------------------------------------- sheet
C = sk3d.COL
fig = sk3d.figure(16, 9, f"Your sketch at true scale: the collar (236 mm) is the top of a neck leaning {TILT:.0f} deg forward")
def draw(ax, b, r):
    ax.add_patch(Polygon(b["out"], closed=True, fc="#b9c8d8", ec="#5a4a42", lw=2))
    for h in (b["hipF"], b["hipR"]): ax.add_patch(Circle(h, 42 * max(b["f"], 0.7), fc="none", ec=C["horse"], lw=2))
    for L in (b["legF"], b["legR"]):
        P = np.array(L); ax.plot(P[:, 0], P[:, 1], color=C["horse"], lw=5, solid_capstyle="round"); ax.plot(*P[-1], "o", color=C["tpu"], ms=9)
    # octopus, tilted with the neck
    ang = np.radians(TILT); R2 = np.array([[np.cos(ang), np.sin(ang)], [-np.sin(ang), np.cos(ang)]])
    def T(pts): return np.array(pts) @ R2.T + b["cc"]
    ax.add_patch(Polygon(T([(-118,-6),(118,-6),(118,0),(-118,0)]), fc=C["latch"], ec="none"))        # collar band
    ax.add_patch(Polygon(T([(-125,0),(125,0),(125,6),(-125,6)]), fc=C["frame"], ec="none"))
    th = np.linspace(0, np.pi, 30); ax.add_patch(Polygon(T(np.c_[125*np.cos(th), 6+150*np.sin(th)]), fc=C["octo"], ec="none"))
    ax.add_patch(Polygon(T([(-90,100),(90,100),(90,224),(-90,224)]), fc=C["octo2"], ec="none"))
    for xb in (-49.5, 49.5): ax.plot(*T([(xb, 0), (xb, -194)]).T, color=C["bar"], lw=3)              # bars down the neck
    ax.plot(*T([(0, 0), (0, -228)]).T, ls=":", color="k", lw=1)
    ax.plot(*r["com"], "P", color=C["hi"], ms=12)
    ax.axhline(0, color="#999", lw=1.5); ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-1100, 450); ax.set_ylim(-40, 1720)
for i, (b, r) in enumerate(zip(B, R)):
    ax = fig.add_axes([0.01 + 0.33 * i, 0.30, 0.32, 0.62]); draw(ax, b, r)
    ax.set_title(r["name"], fontsize=14, fontweight="bold", loc="left")
    sk3d.panel_text(fig, 0.02 + 0.33 * i, 0.285, [
        f"shoulder {r['shoulder_h']:.0f}, top of octopus {r['height_top']:.0f} mm",
        f"body {r['length']:.0f} long, horse {r['m_h']:.0f} kg, centaur {r['m']:.0f} kg",
        f"front legs carry {r['share']*100:.0f} %, tips forward at {r['tip_fwd']:.0f} deg",
        f"front knee, trot: {r['knee_rms']:.0f} rms / {r['knee_peak']:.0f} peak N m",
        (f"forelegs {r['dx']:.0f} mm forward, packs at the croup" if r["dx"] else f"collar {r['collar_ahead_of_front_feet']:.0f} mm ahead of the front hooves"),
    ], fs=12)
sk3d.panel_text(fig, 0.02, 0.075, [
    f"Red cross: centre of mass.  Black bars: the octopus bars slide {194} mm down the neck axis; the neck holds the whole sheath, so nothing pierces the body between the hips any more.",
    f"The octopus rides tilted {TILT:.0f} deg forward with the neck: {rA['seat_shear']:.0f} N of its weight pushes along the seat, taken by the bars.  Arm props can still point straight up (tentacle gimbals).",
], fs=12)
sk3d.save(fig, "sheet_sketch_body.png")
