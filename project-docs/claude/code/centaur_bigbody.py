"""Bigger horse so the collar (dia 236) reads as the horse's neck.  Hand sizing + sketch (est.).  python3 bigbody.py"""
import sys, json; sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, sk3d, centaur_study as CS
C = sk3d.COL; G = 9.81
COLLAR_D = 0.236
# name: scale on leg/hip-spacing, barrel width, barrel depth, neck height above the back, horse mass (est.)
OPT = {
 "now (study)":      dict(s=1.00, w=0.150, d=0.100, neck=0.124, m_h=11.8),
 "M  neck = 2/3 of barrel": dict(s=1.35, w=0.340, d=0.300, neck=0.150, m_h=16.5),
 "L  neck = 1/2 of barrel": dict(s=1.70, w=0.460, d=0.400, neck=0.180, m_h=22.0),
}
def size(o):
    s = o["s"]; L = 0.380 * s; hip = 0.95 * L; Lh = 0.440 * s
    z1 = hip + 0.05 * s + (o["d"] - 0.100) / 2; z0 = z1 - o["d"]; plate = z1 + o["neck"]
    m = CS.OCTO_M + o["m_h"]
    xn = 0.110 * s; xh = 0.250 * s
    xc = (CS.OCTO_M * xn + o["m_h"] * xh) / m; share = 1 - xc / Lh
    zc = (CS.OCTO_M * (plate + CS.COM_H) + o["m_h"] * (hip - 0.03)) / m
    yf = max(0.170 * s, o["w"] / 2 + 0.07)
    # knee torque: scale the leg study's front-knee trot numbers (5.51 rms / 10.74 peak at 21.4 kg, share 0.62, s = 1)
    k = (m * share) / (21.4 * 0.62) * s
    return dict(leg=L, hip=hip, Lh=Lh, z0=z0, z1=z1, plate=plate, top=plate + CS.TURRET_TOP, m=m, share=share, zc=zc, yf=yf,
                tip_side=np.degrees(np.arctan(yf / zc)), tip_fwd=np.degrees(np.arctan(xc / zc)),
                foot=share * m * G, knee_rms=5.51 * k, knee_peak=10.74 * k, getup=10.12 * k, neck_ratio=COLLAR_D / o["w"], xn=xn)
R = {n: size(o) for n, o in OPT.items()}
json.dump(R, open("/home/claude/octodrone/centaur/bigbody.json", "w"), indent=1, default=float)
for n, r in R.items():
    print(f"{n:26s} leg {r['leg']*1e3:.0f} hip {r['hip']*1e3:.0f} hips {r['Lh']*1e3:.0f} barrel {OPT[n]['w']*1e3:.0f}x{OPT[n]['d']*1e3:.0f} back {r['z1']*1e3:.0f} plate {r['plate']*1e3:.0f} top {r['top']*1e3:.0f} | m {r['m']:.1f} share {r['share']*100:.0f}% foot {r['foot']:.0f} N knee {r['knee_rms']:.1f}/{r['knee_peak']:.1f} getup {r['getup']:.1f} | tip side {r['tip_side']:.1f} fwd {r['tip_fwd']:.1f} | collar/barrel {r['neck_ratio']:.2f}")
fig = sk3d.figure(16, 9, "Bigger horse: the collar becomes the neck (same octopus, same scale in all three)")
for i, (n, o) in enumerate(OPT.items()):
    r = R[n]; sc = sk3d.Scene(fig, [0.0 + 0.335 * i, 0.24, 0.33, 0.70], elev=10, azim=-68)
    sc.ground((-1.0, -0.5), (0.6, 0.5))
    xm = -r["Lh"] / 2
    sc.box((xm, 0, (r["z0"] + r["z1"]) / 2), (r["Lh"] + 0.12 * o["s"], o["w"], o["d"]), fc=C["horse"])
    sc.ring((-r["xn"], 0), 0.100, 0.118, r["z1"], r["plate"], C["frame"])
    for x in (0.0, -r["Lh"]):
        for sgn in (1, -1):
            hip = np.array([x, sgn * (o["w"] / 2 + 0.03), r["hip"]]); foot = np.array([x, sgn * r["yf"], 0.02])
            f = 1 if x == 0 else -1
            knee = hip + [0.10 * o["s"] * f, 0.005 * sgn, -0.42 * r["hip"]]; hock = hip + [0.0, 0.01 * sgn, -0.78 * r["hip"]]
            sc.cyl(hip - [0, 0.02 * sgn, 0], hip + [0, 0.02 * sgn, 0], 0.045, C["act"])
            sc.tube([hip, knee, hock, foot], 0.012 + 0.004 * o["s"], C["horse"]); sc.sphere(foot, 0.025, C["tpu"])
    sc.octopus(base=(-r["xn"], 0, r["plate"]), yaw=45, bars=0.194, arms="centaur")
    sc.done((-1.0, -0.5, 0), (0.6, 0.5, 1.25))
    sk3d.panel_text(fig, 0.02 + 0.335 * i, 0.25, [
        f"barrel {o['w']*1e3:.0f} wide: collar is {r['neck_ratio']*100:.0f} % of it",
        f"withers {r['z1']*1e3:.0f}, top of octopus {r['top']*1e3:.0f} mm",
        f"legs {r['leg']*1e3:.0f}, hips {r['Lh']*1e3:.0f} apart",
        f"horse {o['m_h']:.1f} kg, centaur {r['m']:.1f} kg (est.)",
        f"front knee, trot: {r['knee_rms']:.0f} rms / {r['knee_peak']:.0f} peak N m",
        f"tips sideways at {r['tip_side']:.0f} deg"], fs=13, title=n)
sk3d.save(fig, "/home/claude/octodrone/centaur/sheet_bigbody.png")
