import os
"""Cephalo-1 docked on Chilo-1's face: 4 octopus arms planted as front legs, posed with the real rev 4 arm and
checked against reach, joint limits and servo load.  No centipede head legs.
Arm (cephalo1-rev4-integrated.md): root at wall face r 104, z 39 above the plate underside; 8 joints p-y-p-y-p-y-p-y at
s = 50..442 mm, spigot 478; j0 STS3250, j1-j7 STS3215; limits j0 -59/+55, j1 +-30, j2-j7 +-100 deg.
Tip = hand3 folded (396 g, contact 100 mm past the spigot, est.).  Docked octopus: plate on the face collar, axis
horizontal pointing forward.  Feet carry a share of the octopus weight (the rest goes through the hub).
python3 chilo_arm_legs.py -> arm_legs.json + arm_legs.png"""
import json
import numpy as np
from scipy.optimize import minimize

S = np.array([50, 106, 162, 218, 274, 330, 386, 442]) / 1000.0
SPIGOT = 0.478
# Cephalo-2 arm (cephalo2-baseline.md, Forest's choice 2026-11-15): rev 4 tentacle, current build (j0 + j1 STS3250,
# cephalo4 CEPH_J1), bayonet connector end-mounted on the tip (hand base 9.7 + 0.2 gap = 9.9 mm), hand3 hub C2 on every arm.
# Foot = hand3 folded into a fist (fingers at fold 90 / 90 / 85 round the heel): the knuckles (phalanx-2 backs) reach
# 127.9 mm past the hand-base face (CAD, hand3_mesh.py).  Standing on the heel itself needs the hand axis within ~6 deg of
# vertical (flat fingers are only 18 mm above the heel plane): tried, the planted arms then miss their targets by up to 155 mm.
BAYONET = 0.0099
FIST = 0.1279
FOOT_R = 0.020                                    # contact sphere (knuckle pad) radius in the sims
HAND = BAYONET + FIST - FOOT_R                    # spigot -> centre of the contact sphere, 117.8 mm
SERVO = ["STS3250", "STS3250"] + ["STS3215"] * 6
SV = {"STS3215": (0.981, 2.94, 0.055), "STS3250": (1.63, 4.90, 0.0745)}     # rated, stall (N m), mass (kg)
LIM = np.radians([[-59, 55], [-30, 30]] + [[-100, 100]] * 6)
MARGIN = 0.70                                     # use at most 70 % of each joint's range
REACH_MAX = 0.80                                  # foot at most 80 % of the arm's full length from its root
L_FULL = SPIGOT + HAND + FOOT_R
RATED = np.array([SV[n][0] for n in SERVO]); STALL = np.array([SV[n][1] for n in SERVO])
# per joint (cephalo4 mass table): servo + shells/bosses/pin 12 g + clutch (j0 38 g, j1 rigid, j2-j7 23.8 g) + 1/8 of the link
# structure (70 g/m x 478 mm) and line/spring/pins (13 g)
CLUTCH_M = [0.038, 0.0] + [0.0238] * 6
M_JOINT = np.array([SV[n][2] + 0.012 + c + (0.070 * 0.478 + 0.013) / 8 for n, c in zip(SERVO, CLUTCH_M)])
# tip masses (offset along the arm axis from the spigot, kg): tip bay (XIAO node + camera, IMU, ESC), bayonet set, hand3
# (hub C2: 396 g, COM 58.4 mm from its base face, hand3_hubc.json)
TIP_MASSES = [(-0.012, 0.024), (0.003, 0.026), (BAYONET + 0.0584, 0.396)]
M_TIP = sum(m for _, m in TIP_MASSES)
G = 9.81
M_OCTO = 14.81                                    # Cephalo-2, hand3 on all 8, bayonets (cephalo2-baseline.md)

HC = float(os.environ.get("CHILO_HC", 0.38))       # centipede body axis height
FACE_X = 0.0 + 0.28                               # head segment front face (chain(): head 280 mm long, starts at x 0.035 -> see note)
P_PLATE = None
Xw, Yw, Zw = np.eye(3)
import os
ROLL = float(os.environ.get("OCTO_ROLL", 0.0))     # deg: octopus rolled about its axis when docked (hub orientation on the face)


def Ry(q):
    c, s = np.cos(q), np.sin(q); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(q):
    c, s = np.cos(q), np.sin(q); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def arm_frame(i, plate):
    """Octopus axis = world +x.  Octopus x_o = world -z (down), y_o = world +y; arm i at azimuth 22.5 + 45 i from x_o."""
    a = np.radians(22.5 + 45 * i + ROLL)
    e = np.cos(a) * (-Zw) + np.sin(a) * Yw
    Z0 = Xw; Y0 = np.cross(Z0, e)
    root = plate + 0.104 * e + 0.039 * Z0
    return root, np.column_stack([e, Y0, Z0])


def fk(q, root, R0):
    R = R0.copy(); p = root.copy(); s0 = 0.0; P, A = [], []
    for k in range(8):
        p = p + R[:, 0] * (S[k] - s0); s0 = S[k]
        ax = R[:, 1] if k % 2 == 0 else R[:, 2]
        P.append(p.copy()); A.append(ax.copy())
        R = R @ (Ry(q[k]) if k % 2 == 0 else Rz(q[k]))
    spig = p + R[:, 0] * (SPIGOT - S[-1]); tip = spig + R[:, 0] * HAND
    return np.array(P), np.array(A), spig, tip


def torques(q, root, R0, Ffoot):
    P, A, spig, tip = fk(q, root, R0)
    F = np.array([0, 0, Ffoot])
    tau = np.zeros(8)
    for k in range(8):
        t = np.cross(tip - P[k], F)                                  # floor reaction at the foot
        for j in range(k + 1, 8):
            t += np.cross(P[j] - P[k], np.array([0, 0, -M_JOINT[j] * G]))
        u = (tip - spig) / np.linalg.norm(tip - spig)
        for off, mt in TIP_MASSES:
            t += np.cross(spig + u * off - P[k], np.array([0, 0, -mt * G]))
        tau[k] = t @ A[k]
    return tau


def head_radius_at(x):
    return 0.125


def solve(i, plate, foot_xy, Ffoot, q0):
    root, R0 = arm_frame(i, plate)
    lo, hi = LIM[:, 0] * MARGIN, LIM[:, 1] * MARGIN

    def obj(q):
        tau = torques(q, root, R0, Ffoot)
        return float(np.sum((tau / RATED) ** 2)) + 0.02 * float(np.sum(q ** 2))

    cons = [{"type": "eq", "fun": lambda q: fk(q, root, R0)[3] - np.array([foot_xy[0], foot_xy[1], FOOT_R])},
            {"type": "ineq", "fun": lambda q: REACH_MAX * L_FULL - np.linalg.norm(fk(q, root, R0)[3] - root)},
            # clearance: every joint and the spigot stay >= 25 mm in front of the face or outside the head segment
            {"type": "ineq", "fun": lambda q: np.array([max(p[0] - (plate[0] + 0.025), np.hypot(p[1], p[2] - HC) - 0.150)
                                                         for p in list(fk(q, root, R0)[0]) + [fk(q, root, R0)[2]]])},
            # every joint above the floor
            {"type": "ineq", "fun": lambda q: fk(q, root, R0)[0][:, 2] - 0.04}]
    best = None
    rng = np.random.default_rng(i)
    for trial in range(5):
        x0 = q0 if trial == 0 else rng.uniform(lo, hi) * 0.6
        r = minimize(obj, x0, method="SLSQP", bounds=list(zip(lo, hi)), constraints=cons, options=dict(maxiter=400, ftol=1e-9))
        if r.success and (best is None or r.fun < best.fun):
            best = r
    return best, root, R0


CASES = {
    "4 arms, no assist": ([0, 7, 1, 6], 0.0),
    "6 arms, no assist": ([0, 7, 1, 6, 2, 5], 0.0),
    "4 arms + 2 free hands as props (40 N lift)": ([0, 7, 1, 6], 40.0),
    "6 arms + 2 free hands as props (40 N lift)": ([0, 7, 1, 6, 2, 5], 40.0),
}


def foot_for(i, plate):
    s = 1 if np.sin(np.radians(22.5 + 45 * i)) > 0 else -1
    return {0: (plate[0] + 0.19, 0.17), 7: (plate[0] + 0.19, -0.17), 1: (plate[0] + 0.07, 0.32), 6: (plate[0] + 0.07, -0.32),
            2: (plate[0] - 0.08, 0.36), 5: (plate[0] - 0.08, -0.36)}[i]


if __name__ == "__main__":
    plate = np.array([0.315 + 0.012, 0, HC])
    out = {"assumptions": dict(margin_of_joint_range=MARGIN, max_reach_fraction=REACH_MAX, arm_full_length_mm=round(L_FULL * 1e3),
                               hand="hand3 folded, 396 g, contact 100 mm past the spigot (est.)", octopus_kg=M_OCTO,
                               load="the planted arms carry the whole octopus weight minus prop lift (a limp head-spine joint "
                                    "puts ~90 % on the front feet; holding more in the spine needs ~29 N m there)"), "cases": {}}
    for name, (arms, lift) in CASES.items():
        n = len(arms); Ff = (M_OCTO * G - lift) / n; Ff1 = (M_OCTO * G - lift) / (n - 1)
        res = {}
        for i in arms:
            fx0, fy0 = foot_for(i, plate); best = None
            for dx in (-0.10, -0.05, 0.0, 0.05, 0.10, 0.15):
                for dy in (-0.06, 0.0, 0.06):
                    fy = fy0 + np.sign(fy0) * dy
                    rr, root, R0 = solve(i, plate, (fx0 + dx, fy), Ff, np.radians([10, 0, 30, 0, 40, 0, 20, 0]))
                    if rr is None: continue
                    w = np.max(np.abs(torques(rr.x, root, R0, Ff)) / RATED)
                    if best is None or w < best[0]: best = (w, rr, root, R0)
            r, root, R0 = best[1:]
            q = r.x; tau = torques(q, root, R0, Ff); tau1 = torques(q, root, R0, Ff1)
            P, A, spig, tip = fk(q, root, R0)
            res[i] = dict(q_deg=np.degrees(q).round(1).tolist(), joints=np.round(P, 4).tolist(), spigot=np.round(spig, 4).tolist(), foot=np.round(tip, 4).tolist(),
                          reach=round(float(np.linalg.norm(tip - root) / L_FULL), 2),
                          joint_range=round(float(np.max(np.where(q >= 0, q / LIM[:, 1], q / LIM[:, 0]))), 2),
                          x_rated=round(float(np.max(np.abs(tau) / RATED)), 2), x_stall=round(float(np.max(np.abs(tau) / STALL)), 2),
                          x_stall_one_lifted=round(float(np.max(np.abs(tau1) / STALL)), 2), worst_joint=int(np.argmax(np.abs(tau) / RATED)), ok=bool(r.success))
        out["cases"][name] = dict(foot_N=round(Ff, 1), foot_N_one_lifted=round(Ff1, 1), arms=res)
        w = max(v["x_rated"] for v in res.values()); ws = max(v["x_stall_one_lifted"] for v in res.values())
        print(f"== {name}: {Ff:.1f} N/foot ({Ff1:.1f} with one lifted) -> worst {w:.2f} x rated, {ws:.2f} x stall with one lifted")
        for i, v in res.items():
            print(f"   arm {i}: reach {v['reach']:.2f}  joint range {v['joint_range']:.2f}  x rated {v['x_rated']:.2f} (j{v['worst_joint']})  ok {v['ok']}")
    json.dump(out, open("arm_legs.json", "w"), indent=1)
