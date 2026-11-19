"""Arm-vs-arm check for Cephalo-1 on the horse (arms unchanged: cephalo4 rev 4 geometry), and a guard pose for the six
non-working arms that keeps every arm, prop disc and the octopus body apart."""
import numpy as np, itertools
import arm_scale_study as A
CLAW_R, BLADE_OFF = 0.0762, 0.036

def seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c = d1 @ r; b = d1 @ d2; den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = np.clip((b * s + f) / e, 0, 1)
    s = np.clip((b * t - c) / a, 0, 1)
    return np.linalg.norm(p1 + d1 * s - (p2 + d2 * t))

def arm_geo(az, q, bp, bR):
    pts, hand, R = A.arm_fk(az, q, bp, bR)
    P, r = A.capsules(pts, n=20)
    return P, r, pts[-1] + R[:, 0] * BLADE_OFF, R[:, 0]

def arm_arm(g1, g2):
    (P1, r1, *_), (P2, r2, *_) = g1, g2
    m = 9
    for i in range(len(P1) - 1):
        for j in range(len(P2) - 1):
            m = min(m, seg_seg(P1[i], P1[i + 1], P2[j], P2[j + 1]) - max(r1[i], r1[i + 1]) - max(r2[j], r2[j + 1]))
    return m

def disc_disc(g1, g2):
    """approx: prop discs (r 76) as flat discs; distance between centres minus 2 r when nearly coplanar, else centre gap"""
    c1, n1, c2, n2 = g1[2], g1[3], g2[2], g2[3]
    return np.linalg.norm(c1 - c2) - 2 * CLAW_R

def body_clear(g, bp, bR):
    return float(np.min(body_d(g[0], bp, bR) - g[1]))

def body_d(P, bp, bR):
    """per-point distance to the octopus body: dome (upper half-ellipsoid), plate disc, EDF can, turret"""
    q = (P - bp) @ bR
    e = np.sqrt((q[:, 0] ** 2 + q[:, 1] ** 2) / 0.125 ** 2 + np.clip(q[:, 2] - 0.006, 0, None) ** 2 / 0.150 ** 2)
    rr = np.hypot(q[:, 0], q[:, 1]); d_t = np.where((q[:, 2] > 0.1) & (q[:, 2] < 0.224), rr - 0.09, 1.0)
    # dome = upper half-ellipsoid over the plate (z >= 6 mm); below it: plate disc r125 (z -4..10 mm) and EDF can r56 (95 mm deep)
    d_dome = np.where(q[:, 2] >= 0.006, (e - 1) * 0.125, np.hypot(np.clip(rr - 0.125, 0, None), 0.006 - q[:, 2]))
    d_plate = np.where((q[:, 2] > -0.004) & (q[:, 2] < 0.010), rr - 0.125, np.hypot(np.clip(rr - 0.125, 0, None), np.minimum(np.abs(q[:, 2] - 0.003) - 0.007, 1)))
    d_edf = np.where((q[:, 2] < 0) & (q[:, 2] > -0.095), rr - 0.056, np.hypot(np.clip(rr - 0.056, 0, None), np.clip(-0.095 - q[:, 2], 0, None)) + 9 * (q[:, 2] >= 0))
    return np.minimum.reduce([d_dome, d_plate, d_edf, d_t])

def report(geos, bp, bR, props):
    worst_arm = min(arm_arm(geos[i], geos[j]) for i, j in itertools.combinations(range(8), 2))
    worst_disc = min((disc_disc(geos[i], geos[j]) for i, j in itertools.combinations(props, 2)), default=9)
    worst_body = min(body_clear(g, bp, bR) for g in geos)
    return worst_arm, worst_disc, worst_body

def guard_pose(bp, bR, front_q=None, H=None):
    """search a symmetric guard pose (same q on the six raised arms): arms up and splayed out; maximise the worst of
    arm-arm, disc-disc and arm-body clearance (+ arm-horse), prefer tips high."""
    best = None
    for j0 in range(-59, 1, 6):
        for j2 in range(-60, 61, 15):
            for j4 in range(-60, 61, 15):
                for j6 in (-30, 0, 30):
                    q = np.radians([j0, 0, j2, 0, j4, 0, j6, 0])
                    geos = []
                    if front_q is not None:
                        geos.append(arm_geo(A.AZ_R, front_q, bp, bR))
                        geos.append(arm_geo(A.AZ_L, front_q * np.array([1, -1, 1, -1, 1, -1, 1, -1]), bp, bR))
                    for i in range(1, 7):                              # the six others: 67.5 ... 292.5 deg
                        geos.append(arm_geo(A.AZ_R + 45 * i, q, bp, bR))
                    # quick rejects
                    tips_z = np.array([g[2][2] for g in geos[2:]]) - bp[2]
                    if tips_z.min() < 0.10: continue
                    wa, wd, wb = report(geos, bp, bR, range(2, 8))
                    wh = 9.0
                    if H is not None:
                        for g in geos[2:]:
                            wh = min(wh, float((A.shell_dist(H, g[0]) - g[1]).min()))
                    score = min(wa, wd - 0.01, wb, wh - 0.01) + 0.05 * tips_z.mean()
                    if best is None or score > best[0]:
                        best = (score, q, dict(arm_arm=wa, disc_disc=wd, arm_body=wb, arm_horse=wh, tip_z=float(tips_z.mean())))
    return best

if __name__ == "__main__":
    import sys
    s = 0.75; H = A.horse(s); bp, bR = H["oct_p"], H["oct_R"]
    T, _ = A.tasks(s)
    q4 = T["T4 hands meet"]["q"]
    g0 = arm_geo(A.AZ_R, q4, bp, bR); g1 = arm_geo(A.AZ_L, q4 * np.array([1, -1, 1, -1, 1, -1, 1, -1]), bp, bR)
    print("front pair (hands meet) arm-arm clearance mm:", round(1e3 * arm_arm(g0, g1), 1))
    old = np.radians([-55, 0, -35, 0, -15, 0, 0, 0])
    geos = [g0, g1] + [arm_geo(A.AZ_R + 45 * i, old, bp, bR) for i in range(1, 7)]
    print("old guard pose: arm-arm / disc-disc / arm-body mm:", [round(1e3 * v, 1) for v in report(geos, bp, bR, range(2, 8))])
    b = guard_pose(bp, bR, q4, H)
    print("new guard q deg", np.degrees(b[1]).round(), {k: round(1e3 * v, 1) for k, v in b[2].items()})
