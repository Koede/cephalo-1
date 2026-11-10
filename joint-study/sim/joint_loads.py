"""Joints 1-4 side-load check for the Cephalo-1 main arm (1 m, 13 joints, pin hinge 60 deg helix).

Same load model as bearing_loads.py (gravity of the outboard arm + claw payload + claw fan push, straight, curled,
coiled, walking and 60 random poses), applied at joints 1-4. These joints have no gear: the servo holds the moment
about the hinge, and the fork's two ears carry everything else:
  horn side : fork horn ear (alpha +24.5 mm) -> 4 x M3 -> servo horn -> STS3250 output spline and its ball bearing
  idler side: fork idler ear (alpha -24.5 mm) -> printed 8 mm stub axle on the back of the cradle floor
Each load case keeps only the poses the servo can hold (|moment about the hinge| <= 4.9 N.m, STS3250 stall).

For every joint and case it reports the radial force on each ear, the axial force, and stresses in the printed
stub, the ears (in-plane bending where they meet the ring) and the ear bores. Writes results/joint_loads.json.

  python3 joint_loads.py                 # design as drawn and the fixes
"""
import os, sys, math, json
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import config as C, bearing_loads as B

STALL = 4.9                    # N.m, STS3250 at 12 V (joints 1-4)
A_H, A_I = 24.5, -24.5         # mm, ear mid-planes along the hinge axis (pin_cad.py stack)
EAR_W = 21.0                   # mm, ear width (2 x BOSS_R)
EAR_L = 20.8                   # mm, hinge axis to the ring start (pin_cad.py ring_start)
T_H, T_I = 4.0, 3.2            # mm, horn ear and idler ear thickness
STUB_D = 8.0; STUB_LEVER = 2.1 # mm, stub diameter; cradle floor to the idler ear mid-plane
Z_STUB = math.pi * STUB_D ** 3 / 32
A_STUB = math.pi * STUB_D ** 2 / 4
# printed PETG, layers normal to the arm axis [A]: about 45 MPa along the layers, 25 MPa across them (short-term).
PETG_ALONG, PETG_ACROSS = 45.0, 25.0
SF = C.SF_REQUIRED             # Forest's rule: 2x on load calcs
# Reference only: ROBOTIS XM430 (82 g, 4.1 N.m) allows 40 N radial 10 mm from the horn, 20 N axial (e-manual).
# Feetech publishes no side-load limit for the STS3215/STS3250 (ball-bearing output, 25T 5.9 mm spline).
XM430_RADIAL, XM430_AXIAL = 40.0, 20.0

def split(Mx, Fx, a1=A_H, a2=A_I):
    """support reactions (beta, zeta components, N) at a1 and a2 (mm along the hinge axis) for wrench (Mx, Fx)"""
    a1 *= 1e-3; a2 *= 1e-3
    Mrem = -np.asarray(Mx); Frem = -np.asarray(Fx)
    A = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, -a1, 0, -a2], [a1, 0, a2, 0]], float)
    s = np.linalg.solve(A, [Frem[1], Frem[2], Mrem[1], Mrem[2]])
    return s[0:2], s[2:4], float(Frem[0])

def stresses(Rh, Ri, axial, t_h=T_H, t_i=T_I, stub="printed"):
    """stress (MPa) in the printed parts and loads (N) on the servo for one load state"""
    rh, ri = float(np.hypot(*Rh)), float(np.hypot(*Ri))
    ear = lambda R, t: 6 * abs(R[0]) * EAR_L / (t * EAR_W ** 2) + abs(R[1]) / (t * EAR_W)   # beta bends it in-plane, zeta pulls
    out = dict(horn_N=round(rh), idler_N=round(ri), axial_N=round(abs(axial)),
               horn_ear_MPa=round(ear(Rh, t_h), 1), idler_ear_MPa=round(ear(Ri, t_i), 1),
               idler_bore_MPa=round(ri / (STUB_D * t_i), 1))
    if stub == "printed":
        out["stub_bend_MPa"] = round(ri * STUB_LEVER / Z_STUB, 1); out["stub_shear_MPa"] = round(ri / A_STUB, 1)
    else:
        out["stub_bend_MPa"] = round(ri * STUB_LEVER / Z_STUB, 1); out["stub_shear_MPa"] = round(ri / A_STUB, 1)
    return out

def cases(P, st):
    up = lambda f: np.array([0, 0, f]); dirs = B.sphere(400)[::4]; fly = [(pn, q) for pn, q in P.items() if not pn.startswith("walking")]
    tilt = lambda f, t: [(pn, q, f * np.array([math.sin(t) * c, math.sin(t) * s_, math.cos(t)])) for pn, q in fly
                         for c, s_ in ((1, 0), (-1, 0), (0, 1), (0, -1), (.7071, .7071), (-.7071, .7071), (.7071, -.7071), (-.7071, -.7071))]
    fw = B.ROBOT_KG * C.G / C.FEET_ON_GROUND; fl = 2 * B.ROBOT_KG * C.G / C.LANDING_FEET
    return {
        "Hover, any pose": [(pn, q, up(B.FAN["hover"])) for pn, q in fly],
        "Rated thrust within 30 deg of straight up": tilt(B.FAN["rated"], math.radians(30)),
        "Full throttle within 30 deg of straight up": tilt(B.FAN["full"], math.radians(30)),
        "Rated thrust within 60 deg of straight up": tilt(B.FAN["rated"], math.radians(60)),
        "Rated thrust, any direction": [(pn, q, B.FAN["rated"] * d) for pn, q in fly for d in dirs],
        "Full throttle, any direction": [(pn, q, B.FAN["full"] * d) for pn, q in fly for d in dirs],
        "Walking stance": [(f"walking stance {i+1}", q, up(fw)) for i, q in enumerate(st)],
        "Hard landing": [(f"walking stance {i+1}", q, up(fl)) for i, q in enumerate(st)],
    }

def run():
    ch, masses = B.build(); P, st = B.poses(ch); CS = cases(P, st)
    res = {}
    for j in range(1, 5):
        rj = {}
        for cn, items in CS.items():
            worst = None; held = 0; total = 0
            for pn, q, F in items:
                total += 1
                Mx, Fx = B.wrench(ch, q, masses, F, j)
                if abs(Mx[0]) > STALL * 1.0001: continue
                held += 1
                Rh, Ri, ax = split(Mx, Fx)
                key = max(np.hypot(*Rh), np.hypot(*Ri))
                if worst is None or key > worst[0]:
                    worst = (key, dict(pose=pn, F=[round(float(x), 2) for x in F], M_hinge=round(float(Mx[0]), 2),
                                       M_oop=round(float(np.hypot(Mx[1], Mx[2])), 2), Rh=Rh.tolist(), Ri=Ri.tolist(), axial=ax))
            if worst is None: rj[cn] = dict(held=0, total=total); continue
            w = worst[1]; w.update(held=held, total=total); rj[cn] = w
        res[f"joint {j}"] = rj
    return res

def rate_all(res, **kw):
    out = {}
    for jn, rj in res.items():
        out[jn] = {}
        for cn, w in rj.items():
            if "Rh" not in w: continue
            out[jn][cn] = stresses(np.array(w["Rh"]), np.array(w["Ri"]), w["axial"], **kw)
    return out

if __name__ == "__main__":
    res = run()
    rated = rate_all(res)
    for jn in res:
        print("==", jn)
        for cn, w in res[jn].items():
            if "Rh" not in w: print(f"  {cn:44s} no pose held"); continue
            r = rated[jn][cn]
            print(f"  {cn:44s} held {w['held']:4d}/{w['total']:4d}  Moop {w['M_oop']:5.2f}  horn {r['horn_N']:4d} N idler {r['idler_N']:4d} N ax {r['axial_N']:3d}"
                  f" | stub {r['stub_bend_MPa']:5.1f} MPa  hornEar {r['horn_ear_MPa']:5.1f} idlerEar {r['idler_ear_MPa']:5.1f} bore {r['idler_bore_MPa']:4.1f}")
    json.dump(dict(loads=res, rated=rated, limits=dict(petg_along=PETG_ALONG, petg_across=PETG_ACROSS, sf=SF,
                                                      xm430_radial_ref=XM430_RADIAL, xm430_axial_ref=XM430_AXIAL)),
              open(os.path.join(HERE, "..", "results", "joint_loads.json"), "w"), indent=1, default=float)
