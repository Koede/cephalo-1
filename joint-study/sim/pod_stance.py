"""Torque-aware walking stance with the folded fan pod as the foot (Forest, Nov 10: "Folded pod as foot").

The foot is the fixed finger tip of arm-pin-hinge/fan_pod.py: 42.5 mm off the arm axis (tip frame -y) and 78 mm out
along it, with the pod folded 88 deg to +y and the stopped prop parked blades along tip-frame x. For a foot position
the 13 joint angles are chosen to minimise the worst joint's need/stall, keeping: the finger on the ground, the arm
tip within TILT of straight down (so the parked blades stay level), the arm above the ground, clear of the central
17 in disc under the dome, and the parked blade tips above the ground. Writes results/pod_stance.json.
Run with the eval_main stand-in:  python3 <scratch>/stub/run.py pod_stance.py
"""
import os, sys, json, math, time
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
from scipy.optimize import minimize
import config as C, chain as CH, bearing_loads as B, metrics as M, central_prop as CP

ROBOT = 18.95; TIP = 0.2525
FOOT_OFF = np.array([0.0, -0.0425, 0.078])                    # finger tip in the tip frame, m
HUB_OFF = np.array([0.0, 0.068, 0.036])                       # folded prop hub in the tip frame [fan_pod.py at 88 deg, 21 mm adapter]
BLADE_R = 0.2032; BLADE_MARGIN = 0.015; MOTOR_R = 0.0278
TILT = math.radians(10.0)
DISC_R, DISC_Z = 17 * 0.0254 / 2, CP.DOME_BOTTOM - 0.025
CAPS = {"STS3215": 2.942, "STS3250": 4.903}

def caps_for(up):            # up: set of joint indices given STS3250
    c = [9.8] + [4.903] * 4 + [2.942] * 8
    for j in up: c[j] = max(c[j], 4.903)
    return np.array(c)

class Arm:
    def __init__(s, tip=TIP, robot=ROBOT, feet=C.FEET_ON_GROUND, lift=0.0, extra=None):
        C.TIP_PAYLOAD_MASS = tip; B.ROBOT_KG = robot
        s.ch, s.masses = B.build()
        if extra:
            for k, kg in extra.items(): s.masses[k] += kg
        s.tip = tip; s.fw = (robot * C.G - lift) / feet
        s.es = np.where(s.ch.et == 0)[0]; s.g = np.array([0, 0, -C.G])
    def fk(s, q):
        m, eo, ea, eang, R = CH.run_fk(s.ch, q); return m, eo, ea, R
    def torques(s, q, H, fw=None):
        fw = s.fw if fw is None else fw
        m, eo, ea, R = s.fk(q); cen = 0.5 * (m[:-1] + m[1:]); p = m[-1] + R @ FOOT_OFF; F = np.array([0, 0, fw])
        out = np.zeros(13)
        for j in range(13):
            e = s.es[j]; k = s.ch.enode[e]; o = eo[e]; a = ea[e] / np.linalg.norm(ea[e])
            Mv = np.cross(cen[k:] - o, s.masses[k:, None] * s.g).sum(0) + np.cross(m[-1] - o, s.tip * s.g) + np.cross(p - o, F)
            out[j] = a @ Mv
        return out
    def geom(s, q):
        m, eo, ea, R = s.fk(q)
        p = m[-1] + R @ FOOT_OFF; hub = m[-1] + R @ HUB_OFF
        blades = [hub + R @ np.array([sg * BLADE_R, 0, 0]) for sg in (1, -1)]
        return m, R, p, blades

    def motor(s, q):
        m, eo, ea, R = s.fk(q); return m[-1] + R @ HUB_OFF

def rot_z(pts, a):
    c, s_ = math.cos(a), math.sin(a); return pts @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]]).T

def neighbour_gap(m, bl, seg_r):
    """smallest gap (m) from this arm and its parked blade to the two neighbour arms in the same pose, 45 deg away"""
    g = 1.0
    for a in (math.pi / 4, -math.pi / 4):
        n = rot_z(m, a)
        for i in range(1, len(m) - 1):
            for j in range(1, len(n) - 1):
                g = min(g, CH.seg_dist(m[i], m[i + 1], n[j], n[j + 1]) - seg_r[i] - seg_r[j])
            g = min(g, CH.seg_dist(bl[0], bl[1], n[i], n[i + 1]) - seg_r[i] - 0.010)
    for a in (math.pi / 4, -math.pi / 4):        # parked blade against the neighbours' parked blades
        nb = rot_z(np.array(bl), a); g = min(g, CH.seg_dist(bl[0], bl[1], nb[0], nb[1]) - 0.020)
    return g

def blade_disc_gap(bl):
    s = np.linspace(0, 1, 11)[:, None]; pts = bl[0] + s * (bl[1] - bl[0])
    return CP.disc_gap(np.c_[np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]], DISC_R, DISC_Z) + CP.ARM_R - 0.005 - CP.MARGIN

def arm_pts(m, n=6):
    s = np.linspace(0, 1, n)[:, None]
    return np.vstack([m[i] + s * (m[i + 1] - m[i]) for i in range(len(m) - 1)])

def solve(A, q0, H, caps, foot_xy=None, r_min=0.25):
    """minimise the worst need/stall; foot_xy fixes the foot, else it is free with radius >= r_min"""
    ch = A.ch; nv = ch.nv
    def need(q): return np.abs(A.torques(q, H)) / caps
    def obj(x): return x[-1]
    cons = []
    def c_need(x): return x[-1] - need(x[:-1])
    def c_geo(x):
        q = x[:-1]; m, R, p, bl = A.geom(q)
        pts = arm_pts(m[1:]); rz = np.c_[np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]]
        gap = CP.disc_gap(rz, DISC_R, DISC_Z) - CP.MARGIN
        ground = pts[:, 2] - (-H + 0.010 + 0.03)              # arm above the ground (tube radius about 30 mm)
        tilt = R[:, 2] @ np.array([0, 0, -1.0]) - math.cos(TILT)
        blade = np.array([b[2] - (-H) - BLADE_MARGIN for b in bl])
        mot = A.motor(q)[2] - (-H) - (MOTOR_R + 0.005)
        c = [[gap * 10], np.sort(ground)[:5] * 10, [tilt * 10], blade * 10, [neighbour_gap(m, bl, ch.seg_r) * 10],
             [blade_disc_gap(bl) * 10], [mot * 10]]
        if foot_xy is None: c.append([math.hypot(p[0], p[1]) - r_min])
        return np.concatenate(c)
    def c_eq(x):
        q = x[:-1]; m, R, p, bl = A.geom(q)
        e = [p[2] + H]
        if foot_xy is not None: e += [p[0] - foot_xy[0], p[1] - foot_xy[1]]
        return np.array(e) * 10
    x0 = np.r_[q0, need(q0).max()]
    bnds = list(zip(ch.lo, ch.hi)) + [(0, 5)]
    r = minimize(obj, x0, method="SLSQP", bounds=bnds, constraints=[dict(type="ineq", fun=c_need), dict(type="ineq", fun=c_geo), dict(type="eq", fun=c_eq)],
                 options=dict(maxiter=400, ftol=1e-6))
    q = r.x[:-1]; m, R, p, bl = A.geom(q)
    feas = (np.all(c_geo(r.x) > -1e-3) and np.all(np.abs(c_eq(r.x)) < 2e-2))
    pen = CH.self_penetration(m, ch.seg_r, 0.9) + CH.hub_penetration(m, ch.seg_r, M.HUB_R_COLL)
    return dict(q=q, need=need(q), worst=float(need(q).max()), feasible=bool(feas and pen < 1e-3), foot=p, pen=float(pen),
                tilt_deg=float(math.degrees(math.acos(np.clip(R[:, 2] @ [0, 0, -1], -1, 1)))), blade_h=[float(b[2] + H) for b in bl], ok=r.success)

def starts(A, H, n_rand=6, seed=0):
    ch = A.ch; P, st = B.poses(ch); out = list(st)
    rng = np.random.default_rng(seed)
    for i in range(n_rand):
        out.append(np.clip(st[i % len(st)] + rng.normal(0, 0.25, ch.nv), ch.lo + 1e-6, ch.hi - 1e-6))
    return out

def best(A, H, caps, foot_xy=None, q_seed=None, n_rand=6):
    S = [q_seed] if q_seed is not None else []
    S += starts(A, H, n_rand)
    res = [solve(A, q, H, caps, foot_xy) for q in S]
    ok = [r for r in res if r["feasible"]]
    return min(ok, key=lambda r: r["worst"]) if ok else min(res, key=lambda r: r["worst"])

def gait(A, H, caps, q_nom, foot_nom, step=0.045):
    """the nominal foot and the study's step pattern round it (±45 mm sideways, 34 mm in)"""
    rad = foot_nom[:2] / np.linalg.norm(foot_nom[:2]); tan = np.array([-rad[1], rad[0]])
    pts = [foot_nom[:2] + d for d in (step * tan, -step * tan, -0.034 * rad)]
    rows = [solve(A, q_nom, H, caps, f) for f in pts]
    for i, r in enumerate(rows):
        if not r["feasible"] or r["worst"] > 1.5:
            r2 = best(A, H, caps, pts[i], q_seed=q_nom, n_rand=3)
            if r2["feasible"] and (not r["feasible"] or r2["worst"] < r["worst"]): rows[i] = r2
    return rows

def summ(r):
    return dict(worst=round(r["worst"], 2), feasible=r["feasible"], need=[round(float(x), 2) for x in r["need"]],
                foot_mm=[round(float(v) * 1e3) for v in r["foot"]], foot_radius_mm=round(float(np.hypot(*r["foot"][:2])) * 1e3),
                tilt_deg=round(r["tilt_deg"], 1), blade_tips_above_ground_mm=[round(b * 1e3) for b in r["blade_h"]])

def main():
    t0 = time.time(); out = dict(assumptions=dict(robot_kg=ROBOT, tip_kg=TIP, feet_on_ground=C.FEET_ON_GROUND, foot_offset_mm=(FOOT_OFF * 1e3).tolist(),
                                                 tilt_limit_deg=math.degrees(TILT), blade_margin_mm=BLADE_MARGIN * 1e3, central_disc=dict(R=DISC_R, z=DISC_Z)), cases={})
    A = Arm(); caps0 = caps_for(set())
    # 1) the study's stance poses with the finger as the foot, as they stand (no torque tuning): the starting point
    P, st = B.poses(A.ch)
    base = [np.abs(A.torques(q, C.BODY_HEIGHT)) / caps0 for q in st]
    out["study_stance_as_is_on_tip"] = [round(float(x), 2) for x in np.max(base, 0)]
    for H in (0.20, 0.25):
        caps = caps0; r = best(A, H, caps)
        g = gait(A, H, caps, r["q"], r["foot"]) if r["feasible"] else []
        rows = [r] + [x for x in g]
        two = [float(np.max(np.abs(A.torques(x["q"], H, fw=2 * A.fw)) / caps)) for x in rows]
        worst_g = max(x["worst"] for x in rows)
        out["cases"][f"hub {int(H*1e3)} mm above ground"] = dict(nominal=summ(r), gait=[summ(x) for x in g], worst_over_gait=round(worst_g, 2),
            worst_over_gait_at_2x_foot_load=round(max(two), 2), all_feasible=bool(all(x["feasible"] for x in rows)),
            q_deg=[[round(float(v), 1) for v in np.degrees(x["q"])] for x in rows])
        print(f"H {H}: nominal {r['worst']:.2f} foot {np.round(r['foot']*1e3)} | gait {[round(x['worst'],2) for x in g]} feas {[x['feasible'] for x in rows]}"
              f" | 2x load {np.round(two,2)}  t={time.time()-t0:.0f}s", flush=True)
    json.dump(out, open(os.path.join(HERE, "..", "results", "pod_stance.json"), "w"), indent=1, default=float)

if __name__ == "__main__":
    main()
