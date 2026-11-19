"""Rev 4 poses (stand, walk envelope, flight 'tulip') and the static load table for every mode.
Run: python3 poses4.py  -> prints tables, writes poses4.json"""
import json, numpy as np
from scipy.optimize import minimize
import cephalo4 as C

D = C.DEG
EDF_WALK = 30.0          # N of EDF assist used while walking/standing (continuous case); peak case = fans off


def pose_from_pitch(p):
    q = np.zeros(C.NJ); q[[0, 2, 4, 6]] = p; return q


def stand_eval(x, verbose=False):
    p0, p2, p4 = x; p6 = 90 * D - p0 - p2 - p4
    q = pose_from_pitch([p0, p2, p4, p6])
    if np.any(q < C.LIM[:, 0]) or np.any(q > C.LIM[:, 1]):
        return 1e3, None
    f, u = C.tip_point(0, q, np.zeros(3))
    h = -f[2]                                  # plate height above the floor
    body_p = np.array([0, 0, h])
    foot = f + body_p
    rf = np.hypot(foot[0], foot[1])
    P, r, _ = C.arm_capsules(0, q, body_p)
    clr = C.body_clearance(P, r, body_p).min()
    out = {}
    worst = 0
    for tag, Fz, n in (("cont", C.W_ROBOT - EDF_WALK, 7), ("peak", C.W_ROBOT, 7)):
        F = np.array([0, 0, Fz / n])
        tau = np.abs(C.joint_torques(0, q, body_p, np.eye(3), F, foot))
        util = tau / C.RATED if tag == "cont" else 1.25 * tau / C.STALL
        out[tag] = (tau, util); worst = max(worst, util.max())
    cost = max(worst, 0.5) + 2 * abs(h - 0.30) + 2 * max(0, 0.36 - rf) + 20 * max(0, 0.015 - clr)
    return cost, dict(q=q, h=h, rf=rf, clr=clr, out=out, worst=worst)


def flight_eval(x):
    p0, p2, p4 = x; p6 = -90 * D - p0 - p2 - p4
    q = pose_from_pitch([p0, p2, p4, p6])
    if np.any(q < C.LIM[:, 0]) or np.any(q > C.LIM[:, 1]):
        return 1e3, None
    body_p = np.zeros(3)
    _, (R, p) = C.arm_fk(0, q, body_p)
    hub = p + R[:, 0] * C.BLADE_OFF
    rf = np.hypot(hub[0], hub[1])
    P, r, _ = C.arm_capsules(0, q, body_p, s_min=0.15)
    clr = C.body_clearance(P, r, body_p).min()
    # prop disc clearance to the dome (ring of blade tips) and to the neighbour's disc
    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    e1 = np.cross(R[:, 0], [0, 0, 1.0]); e1 = e1 / (np.linalg.norm(e1) + 1e-9) if np.linalg.norm(e1) > 1e-6 else np.array([1.0, 0, 0])
    e2 = np.cross(R[:, 0], e1)
    ring = hub + C.CLAW_R * (np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2)
    dclr = C.body_clearance(ring, np.full(len(ring), 0.003), body_p).min()
    gap_nb = 2 * rf * np.sin(22.5 * D) - 2 * C.CLAW_R
    loads = []
    T = (C.W_ROBOT - 0.8 * C.EDF_MAX) / 8
    for Tf, vec in ((T, 10), (T, -10), (C.FAN_MAX["opt"], 20), (C.FAN_MAX["opt"], -20)):
        d = np.cos(vec * D) * R[:, 0] + np.sin(vec * D) * R[:, 1]
        loads.append(np.abs(C.joint_torques(0, q, body_p, np.eye(3), Tf * d, hub)))
    cont = np.max(loads[:2], 0) / C.RATED; peak = 1.25 * np.max(loads[2:], 0) / C.STALL
    worst = max(cont.max(), peak.max())
    cost = worst + 3 * max(0, 0.36 - rf) + 30 * max(0, 0.10 - clr) + 30 * max(0, 0.03 - dclr) + 30 * max(0, 0.02 - gap_nb) \
        + 2 * max(0, hub[2] - 0.45) + 0.05 * np.sum(q ** 2)
    return cost, dict(q=q, rf=rf, hub_z=hub[2], clr=clr, disc_clr=dclr, gap_nb=gap_nb, cont=cont, peak=peak, worst=worst,
                      hover_N=T)


def root_limits():
    """j0 pitch range with the rest of the arm straight / curled: first contact with the mantle (12 mm margin)."""
    res = {}
    for name, rest in (("straight", np.zeros(C.NJ)), ("curled up", pose_from_pitch([0, -60 * D, -60 * D, -30 * D]))):
        lim = []
        for sgn in (-1, 1):
            a = 0.0
            while abs(a) < 120 * D:
                q = rest.copy(); q[0] = a + sgn * D
                P, r, _ = C.arm_capsules(0, q, np.zeros(3))
                if C.body_clearance(P, r, np.zeros(3)).min() < 0.012:
                    break
                a += sgn * D
            lim.append(np.degrees(a))
        res[name] = lim
    return res


def best(fn, starts):
    bst = None
    for s in starts:
        r = minimize(lambda x: fn(x)[0], np.radians(s), method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-5, maxiter=3000))
        if bst is None or r.fun < bst.fun:
            bst = r
    return fn(bst.x)[1]


if __name__ == "__main__":
    print("j0 limits (deg):", root_limits())
    st = best(stand_eval, [(10, 40, 30), (0, 45, 45), (20, 30, 20), (-10, 60, 30), (5, 60, 15)])
    print("\nSTAND  q =", np.round(np.degrees(st["q"]), 1), f"plate h {st['h']*1000:.0f} mm, foot r {st['rf']*1000:.0f} mm, "
          f"body clearance {st['clr']*1000:.0f} mm")
    for tag in ("cont", "peak"):
        tau, u = st["out"][tag]
        print(f"   {tag}: tau {np.round(tau, 2)}  util {np.round(u, 2)}")
    fl = best(flight_eval, [(-30, -30, -30), (-40, -35, -15), (-20, -40, -20), (-45, -20, -20), (-35, -35, 0)])
    print("\nFLIGHT q =", np.round(np.degrees(fl["q"]), 1), f"hub r {fl['rf']*1000:.0f} mm, hub z {fl['hub_z']*1000:.0f} mm, "
          f"arm-body clr {fl['clr']*1000:.0f} mm, disc clr {fl['disc_clr']*1000:.0f}, neighbour gap {fl['gap_nb']*1000:.0f} mm")
    print("   cont util", np.round(fl["cont"], 2), " peak util", np.round(fl["peak"], 2), f" hover {fl['hover_N']:.2f} N/fan")
    json.dump(dict(stand=np.degrees(st["q"]).tolist(), stand_h=st["h"], stand_rf=st["rf"], flight=np.degrees(fl["q"]).tolist(),
                   flight_rf=fl["rf"], flight_hub_z=fl["hub_z"]), open("poses4.json", "w"), indent=1)
