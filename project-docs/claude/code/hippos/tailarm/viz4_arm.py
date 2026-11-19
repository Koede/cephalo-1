"""Rev 4 arm mesh (project's claude/code/viz4.py arm_mesh, verbatim arm body: tapered shell, servos in their cradles,
clutch cams + rockers, idler bosses, driven-link forks, tip bay).  The pod is NOT drawn here: Cephalo-2 puts the
connector (socket + hand base) on the spigot and the pod / hand LEN_BASE further out (cephalo2_model.py)."""
import numpy as np
import cephalo4 as C
from geom import frustum, box

WHITE = [0.92, 0.92, 0.90]; GREY = [0.55, 0.57, 0.60]; DARK = [0.20, 0.21, 0.23]; ORANGE = [0.95, 0.55, 0.15]
BLUE = [0.22, 0.42, 0.78]; STEEL = [0.70, 0.72, 0.75]
SCOL = {"STS3215": BLUE, "STS3250": [0.28, 0.29, 0.32]}


def arm_body(S, i, p, R, q, c=None):
    c = np.zeros(C.NJ) if c is None else c
    segs, (Rt, pt) = C.arm_fk(i, q, p, R, clutch=c)
    jf = C.joint_frames(segs)
    ss = np.linspace(0.0, C.SPIGOT, 34)
    P = [C.at_s(segs, s)[0] for s in ss]
    for k in range(len(P) - 1):
        r = float(np.interp(ss[k], [0, C.SPIGOT], [0.013, 0.008]))
        S.add(frustum(P[k], P[k + 1], r, r, WHITE, n=16), same=k > 0)
    for k in range(C.NJ):
        n = C.SERVO[k]
        L, Wd, H = np.array(C.SV[n][3:]) * 1e-3
        sc = C.S_J[k] - (L / 2 - 0.0114)
        cpos, Rl = C.at_s(segs, sc)
        ax = Rl[:, 1] if C.TYP[k] == "p" else Rl[:, 2]
        o = np.cross(ax, Rl[:, 0])
        S.add(box(cpos, np.stack([Rl[:, 0], o, ax], 1), [L, Wd, H * 0.92], SCOL[n]))
        for sg in (-1, 1):
            S.add(box(cpos + o * sg * (Wd / 2 + 0.0025) - Rl[:, 0] * 0.004, np.stack([Rl[:, 0], o, ax], 1), [L + 0.006, 0.004, H * 0.80], WHITE), same=sg > 0)
        cj, axj, Rc = jf[k]
        hw = H / 2
        if C.CLUTCH_FRAC[k] > 0:
            rc = 0.019 if n == "STS3215" else 0.025
            S.add(frustum(cj + axj * (hw + 0.001), cj + axj * (hw + 0.007), rc, rc, ORANGE, n=28, c2=[0.75, 0.40, 0.10], stripes=4, ref=Rc[:, 0]))
            pd, Rd = C.at_s(segs, C.S_J[k] + 0.026)
            S.add(box(pd + axj * (hw + 0.010), np.stack([Rd[:, 0], np.cross(axj, Rd[:, 0]), axj], 1), [0.050, 0.010, 0.006], [0.80, 0.45, 0.12]))
            S.add(frustum(pd + Rd[:, 0] * 0.022 + axj * (hw + 0.007), pd + Rd[:, 0] * 0.022 + axj * (hw + 0.014), 0.005, 0.005, STEEL, n=12))
        else:
            S.add(frustum(cj + axj * (hw + 0.001), cj + axj * (hw + 0.005), 0.012, 0.012, DARK, n=20))
        S.add(frustum(cj - axj * (hw + 0.006), cj - axj * (hw + 0.001), 0.011, 0.011, GREY, n=20))
        pd2, Rd2 = C.at_s(segs, C.S_J[k] + 0.012)
        od = np.cross(axj, Rd2[:, 0])
        for sg in (-1, 1):
            S.add(box(pd2 + axj * sg * (hw + 0.0035 + (0.008 if sg > 0 and C.CLUTCH_FRAC[k] > 0 else 0)), np.stack([Rd2[:, 0], od, axj], 1),
                      [0.034, 0.020, 0.003], WHITE), same=sg > 0)
    u = Rt[:, 0]
    S.add(box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], [0.35, 0.36, 0.40]))
    return segs, (Rt, pt)
