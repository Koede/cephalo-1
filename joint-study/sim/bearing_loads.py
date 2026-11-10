"""Joint-0 root bearing check for the Cephalo-1 main arm (1 m, 13 joints, pin hinge 60 deg helix, 2:1 gear on joint 0).

Static load cases on the main-design chain (same kinematics, masses and tip payload as the study, more_thrust scenario).
For each case the loads the arm puts on the root socket (gravity + tip force) are split into
  - the hinge-axis moment, carried by the gear mesh (tooth force at the mesh point, 20 deg pressure angle),
  - the out-of-plane moment and the force across the axis, carried by the two radial supports:
      gear side : 6800 bearing on the gear shaft
      idler side: the fork's idler ear on the socket's stub axle
Writes results/bearing_loads.json.  Support positions along the hinge axis are passed in mm (joint frame alpha).

  python3 bearing_loads.py            # both designs: the socket as first drawn and after the bearing check
  python3 bearing_loads.py --a1 6.5 --a2 -24.5 --tag _x   # one support layout only
"""
import os, sys, math, json, argparse
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import config as C, chain as CH, metrics as M, eval_main as E

ROBOT_KG = 15.077          # main design with the root gear (root_gear.json)
GEAR_KG = 0.0453           # root gear stage, CAD
FAN = dict(hover=13.2, rated=16.0, full=21.4)
GEAR_R = 0.020; PA = math.radians(20.0); A_MESH = 14.5e-3
STALL_AXIS = 9.8           # N.m at the joint: STS3250 4.9 N.m stall x 2

def build():
    nm = E.MAIN_LABEL_KEY if hasattr(E, "MAIN_LABEL_KEY") else "STS3250 to 35 %, STS3215 beyond"
    C.ARM_LENGTH = 1.0; v = E.FITS[nm]
    cfg = dict(family="pin_hinge", variant="M3_bolt_PETG", range_deg=90, arrangement="helix60", n_nodes=13, **v)
    ch = CH.build_chain(cfg); masses = M.node_masses(ch).copy(); masses[0] += GEAR_KG
    return ch, masses

def wrench(ch, q, masses, F_tip, j=0):
    """loads the arm (outboard of joint j) puts on the part before it, about the joint origin, in that part's frame"""
    m, eo, ea, eang, R = CH.run_fk(ch, q)
    es = np.where(ch.et == 0)[0]; e = es[j]; k = ch.enode[e]; o = eo[e]
    cen = 0.5 * (m[:-1] + m[1:]); g = np.array([0, 0, -C.G])
    Mx = np.cross(cen[k:] - o, masses[k:, None] * g).sum(0) + np.cross(m[-1] - o, C.TIP_PAYLOAD_MASS * g + F_tip)
    Fx = masses[k:].sum() * g + C.TIP_PAYLOAD_MASS * g + F_tip
    a = ea[e] / np.linalg.norm(ea[e])
    # arm direction at the joint with the joint at zero: the parent's z axis. For joint 0 that is base_R[:,2].
    if j == 0: z = ch.base_R[:, 2]
    else:
        z = (m[k + 1] - m[k]) if k + 1 < len(m) else (m[k] - m[k - 1])
    z = z - a * (a @ z); z /= np.linalg.norm(z); b = np.cross(z, a)
    Rf = np.stack([a, b, z])            # rows: alpha, beta, zeta
    return Rf @ Mx, Rf @ Fx

def supports(Mx, Fx, a1, a2, geared=True):
    """reactions (N) at the gear-side bearing (a1) and the idler support (a2), alpha in mm; tooth force on the gear"""
    a1 *= 1e-3; a2 *= 1e-3
    Ma, Mb, Mz = Mx; Fa, Fb, Fz = Fx
    if geared:
        t = -Ma / GEAR_R; s = abs(t) * math.tan(PA)        # tooth force on the gear: t along beta at zeta -20 mm, s along +zeta
        T = np.array([0.0, t, s]); rT = np.array([A_MESH, 0.0, -GEAR_R])
        Mrem = -(np.array(Mx) + np.cross(rT, T)); Frem = -(np.array(Fx) + T)
    else:
        T = np.zeros(3); Mrem = -np.array(Mx); Frem = -np.array(Fx)
    # R1, R2 in (beta, zeta) at alpha a1, a2:  R1 + R2 = Frem_perp ; moments about the origin: a_i * (alpha x R_i)
    # alpha x (0, Rb, Rz) = (0, -Rz, Rb)  ->  sum a_i*(-Rz_i) = Mrem_beta ; sum a_i*Rb_i = Mrem_zeta
    A = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, -a1, 0, -a2], [a1, 0, a2, 0]], float)
    sol = np.linalg.solve(A, [Frem[1], Frem[2], Mrem[1], Mrem[2]])
    R1 = np.hypot(sol[0], sol[1]); R2 = np.hypot(sol[2], sol[3])
    return dict(R1=R1, R2=R2, axial=abs(Fa), tooth=float(np.hypot(T[1], T[2])), M_axis=Ma, M_oop=float(np.hypot(Mb, Mz)))

def sphere(n):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)

def poses(ch):
    x0 = M._x0(ch); P = {"straight out": x0}
    try: P["full curl"] = M.fit_shape(ch, M.target_shapes(ch)["arc_curl"])[0]
    except Exception: pass
    for nm in ("helix_coil", "side_arc", "s_curve"):
        try: P[nm.replace("_", " ")] = M.fit_shape(ch, M.target_shapes(ch)[nm])[0]
        except Exception: pass
    st = []
    for f in M.FEET:
        st.append(M.ik_foot(ch, f, st[-1] if st else x0)[0])
    for i, q in enumerate(st): P[f"walking stance {i+1}"] = q
    rng = np.random.default_rng(1)
    hi = np.minimum(ch.hi, math.radians(45)); lo = np.maximum(ch.lo, -math.radians(45))
    for i in range(60): P[f"random {i}"] = lo + (hi - lo) * rng.random(ch.nv)
    return P, st

# the two root layouts. Pinion: tooth force at alpha 14.5 between the servo spline (horn centre, alpha 6.0 [A]) and the cap bearing.
DESIGNS = {
    "as drawn": dict(a1=6.5, a2=-24.5, gear_brg=("6800", 1720, 830), pin_brg=("685", 681, 280), pin_brg_a=23.7,
                     idler="printed PETG stub, 8 mm", idler_Z=47.1, idler_allow=40.0),
    "after the bearing check": dict(a1=12.0, a2=-24.5, gear_brg=("61803", 2030, 1270), pin_brg=("695", 1025, 410), pin_brg_a=24.2,
                     idler="7075 pin, 8 mm", idler_Z=49.8, idler_allow=500.0),
}
EAR_AREA = 8.0 * 3.2          # fork idler ear on the 8 mm stub or pin, projected, mm^2 (PETG ear, unchanged)
IDLER_LEVER = 2.1             # mm, cheek face to the ear's mid-plane

def rate(d, c):
    """safety factors for one load case on one design"""
    P0 = max(c["R1"], 0.6 * c["R1"] + 0.5 * c["axial"])
    tooth = c["tooth"]; Rp = tooth * (14.5 - 6.0) / (d["pin_brg_a"] - 6.0); Rs = tooth - Rp
    return dict(gear_brg_N=round(c["R1"]), gear_brg_s0=round(d["gear_brg"][2] / P0, 2),
                pin_brg_N=round(Rp), pin_brg_s0=round(d["pin_brg"][2] / max(Rp, 1e-9), 2), servo_spline_N=round(Rs),
                idler_N=round(c["R2"]), ear_MPa=round(c["R2"] / EAR_AREA, 1),
                idler_bend_MPa=round(c["R2"] * IDLER_LEVER / d["idler_Z"], 1))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--a1", type=float); ap.add_argument("--a2", type=float, default=-24.5)
    ap.add_argument("--tag", default=""); a = ap.parse_args()
    if a.a1 is None:
        res = {}
        for nm, d in DESIGNS.items():
            o = run(d["a1"], d["a2"])
            o["design"] = {k: v for k, v in d.items()}
            o["rating"] = {k: rate(d, c) for k, c in o["cases"].items()}
            res[nm] = o
            print(f"== {nm}: {d['gear_brg'][0]} at alpha {d['a1']}, {d['pin_brg'][0]} on the pinion, {d['idler']}")
            for k, r in o["rating"].items():
                print(f"  {k:36s} {d['gear_brg'][0]:>6s} {r['gear_brg_N']:5d} N s0 {r['gear_brg_s0']:4.2f} | {d['pin_brg'][0]} {r['pin_brg_N']:4d} N s0 {r['pin_brg_s0']:4.2f}"
                      f" | spline {r['servo_spline_N']:4d} N | idler {r['idler_N']:4d} N ear {r['ear_MPa']:4.1f} MPa bend {r['idler_bend_MPa']:4.1f} MPa")
        print("joints 1-4, rated thrust, worst out-of-plane N.m:", res["as drawn"]["joints_1_4_rated_oop_Nm"])
        with open(os.path.join(HERE, "results", "bearing_loads.json"), "w") as f: json.dump(res, f, indent=1, default=float)
        return
    o = run(a.a1, a.a2)
    with open(os.path.join(HERE, "results", f"bearing_loads{a.tag}.json"), "w") as f: json.dump(o, f, indent=1, default=float)

def run(a1, a2):
    class A: pass
    a = A(); a.a1 = a1; a.a2 = a2
    ch, masses = build(); P, st = poses(ch)
    dirs = sphere(400); out = {"a1_mm": a.a1, "a2_mm": a.a2, "cases": {}}
    def worst(name, items, note):
        best = None
        for (pn, q, F) in items:
            Mx, Fx = wrench(ch, q, masses, F)
            if abs(Mx[0]) > STALL_AXIS * 1.0001: continue          # the gear stage cannot hold it: not a static pose
            r = supports(Mx, Fx, a.a1, a.a2)
            if best is None or max(r["R1"], r["R2"]) > max(best["R1"], best["R2"]):
                best = dict(r, pose=pn, F=[round(float(x), 2) for x in F])
        out["cases"][name] = dict(best, note=note); return best
    up = lambda f: np.array([0, 0, f])
    worst("Hover, straight out", [("straight out", P["straight out"], up(FAN["hover"]))], "13.2 N up at the tip, arm level")
    worst("Hover, any pose", [(pn, q, up(FAN["hover"])) for pn, q in P.items() if not pn.startswith("walking")],
          "13.2 N up, curled, coiled and random poses up to +-45 deg per joint")
    tilt = lambda f, t: [(pn, q, f * np.array([math.sin(t) * c, math.sin(t) * s_, math.cos(t)])) for pn, q in P.items() if not pn.startswith("walking")
                         for c, s_ in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    worst("Flight, rated thrust tilted 30 deg", tilt(FAN["rated"], math.radians(30)), "16 N tilted 30 deg any way, any pose (the study's flight stress case)")
    worst("Rated thrust, any direction", [(pn, q, FAN["rated"] * d) for pn, q in P.items() if not pn.startswith("walking") for d in dirs[::4]],
          "16 N pointed any way by the wrist gimbal, any pose, hinge moment within the gear's 9.8 N.m")
    worst("Full throttle, any direction", [(pn, q, FAN["full"] * d) for pn, q in P.items() if not pn.startswith("walking") for d in dirs[::4]],
          "21.4 N pointed any way, any pose, hinge moment within the gear's 9.8 N.m")
    fw = ROBOT_KG * C.G / C.FEET_ON_GROUND
    worst("Walking stance", [(f"walking stance {i+1}", q, up(fw)) for i, q in enumerate(st)], f"{fw:.1f} N up at the foot (7 feet share {ROBOT_KG} kg)")
    fl = 2 * ROBOT_KG * C.G / C.LANDING_FEET
    worst("Hard landing", [(f"walking stance {i+1}", q, up(fl)) for i, q in enumerate(st)], f"{fl:.0f} N: 4 feet share {ROBOT_KG} kg, x2 for impact [A]")
    # stall: the most the gear can load the 6800, with the hinge moment at stall and the rest from the hover case
    Mx, Fx = wrench(ch, P["straight out"], masses, up(FAN["hover"])); Mx = Mx.copy(); Mx[0] = STALL_AXIS
    out["cases"]["Gear at stall"] = dict(supports(Mx, Fx, a.a1, a.a2), pose="straight out", note="hinge moment at the gear's 9.8 N.m stall, hover loads otherwise")
    # joints 1-4 for context (servo output + idler stub, no gear): worst out-of-plane moment in the rated-thrust case
    ctx = {}
    for j in range(1, 5):
        mo = 0.0
        for pn, q in P.items():
            if pn.startswith("walking"): continue
            for d in dirs[::8]:
                Mx, Fx = wrench(ch, q, masses, FAN["rated"] * d, j)
                if abs(Mx[0]) <= 4.9: mo = max(mo, float(np.hypot(Mx[1], Mx[2])))
        ctx[f"joint {j}"] = round(mo, 2)
    out["joints_1_4_rated_oop_Nm"] = ctx
    return out

if __name__ == "__main__":
    main()
