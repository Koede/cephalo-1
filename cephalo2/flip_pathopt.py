"""Roll path for the mid-air flip (python3 flip_pathopt.py -> flip4_path_opt.npy).
Refines flip4.roll_path() step by step along the roll (body at the reference attitude): flyers keep their claw axes
vertical; joint loads above 0.6 x stall are penalised (thrust = weight / active claw-props); the thrust centroid of the
active claw-props is kept over the robot's centre of mass (else the props cannot hold the attitude); arms 5-7 (props
stopped mid-roll) may move off their interpolation to help that balance; body, arm-arm and prop-disc clearances;
continuity with the previous step."""
import numpy as np, time
from scipy.optimize import least_squares
import flip4 as F4, cephalo4 as C
c = np.zeros(3); RP = 0.0953
P0 = F4.roll_path(); N = len(P0) - 1
M_PACK = 0.81
W = (C.M_ROBOT) * C.G


def arm_pts(i, q, R, n=14):
    segs, tip = C.arm_fk(i, q, c, R); return np.array([C.at_s(segs, s)[0] for s in np.linspace(0.07, C.SPIGOT, n)])


def arm_mass(i, q, R):
    segs, tip = C.arm_fk(i, q, c, R); P, m, _ = C.arm_points(segs, tip); return (P * m[:, None]).sum(0), m.sum()


def active_w(th):
    """Thrust share of arms 5-7 (they stop as their claw axes pass 40 deg and restart near the end)."""
    d = np.degrees(th)
    return float(np.clip((40 - d) / 15, 0, 1) + np.clip((d - 165) / 10, 0, 1))


def resid(q, i, R, qprev, qnom, T, others, hubs_o, ctx):
    r = []
    h, a = F4.hub(i, q, c, R)
    flyer = i in F4.FLYERS
    tau = C.joint_torques(i, q, c, R, (T if flyer else 0.0) * np.array([0, 0, 1.]), h) / C.STALL
    if flyer:
        r.append(8 * (a - np.array([0, 0, 1.])))
    r.append(4.0 * np.maximum(np.abs(tau) - 0.6, 0))
    r.append(1.0 * (q - qprev)); r.append(0.4 * (q - qnom))
    P = arm_pts(i, q, R)
    bc = C.body_clearance((P - c) @ R, np.full(len(P), 0.024), np.zeros(3))
    r.append(20 * np.maximum(0.015 - bc, 0))
    for Po in others:
        r.append([20 * max(0.03 - (np.min(np.linalg.norm(P[:, None] - Po[None], axis=2)) - 0.048), 0)])
    if flyer:
        for ho in hubs_o:
            g = np.hypot(*(h[:2] - ho[:2])) - 2 * RP
            r.append([20 * max(0.03 - g, 0) * (abs(h[2] - ho[2]) < 0.1)])
    # balance: weighted hub centroid over the CoM
    mp, mm = arm_mass(i, q, R)
    com = (ctx["mp"] + mp) / (ctx["mm"] + mm)
    wi = ctx["w"][i]
    cen = (ctx["hw"] + wi * h) / max(ctx["ws"] + wi, 1e-6)
    r.append(6.0 * (cen[:2] - com[:2]))
    return np.concatenate([np.atleast_1d(x) for x in r])


def smooth(Q, sigma=4.0):
    """Gaussian smoothing along the path (removes the optimiser's step jumps), ends kept."""
    from scipy.ndimage import gaussian_filter1d
    S = gaussian_filter1d(Q, sigma, axis=0, mode="nearest")
    w = np.clip(np.minimum(np.arange(len(Q)), np.arange(len(Q))[::-1]) / (3 * sigma), 0, 1)[:, None, None]
    return Q * (1 - w) + S * w


if __name__ == "__main__":
    Q = P0.copy(); t0 = time.time()
    for n in range(1, N + 1):
        s = n / N; th = np.pi * F4.mj(s); R = F4.Rax(th)
        aw = active_w(th)
        w = np.array([1.0 if i in F4.FLYERS else aw for i in range(8)])
        T = W / w.sum()
        Q[n] = Q[n - 1] + (P0[n] - P0[n - 1])
        for sweep in range(2):
            for i in list(F4.FLYERS) + list(F4.UNDER):
                others = [arm_pts(j, Q[n, j], R) for j in range(8) if j != i]
                hubs = {j: F4.hub(j, Q[n, j], c, R)[0] for j in range(8)}
                hubs_o = [hubs[j] for j in F4.FLYERS if j != i]
                mp = M_PACK * (R @ np.array([0, 0, 0.03])) + C.M_BODY * (R @ np.array([0, 0, 0.05])); mm = C.M_BODY + M_PACK
                for j in range(8):
                    if j != i:
                        a_, b_ = arm_mass(j, Q[n, j], R); mp = mp + a_; mm += b_
                hw = sum(w[j] * hubs[j] for j in range(8) if j != i); ws = sum(w[j] for j in range(8) if j != i)
                ctx = dict(mp=mp, mm=mm, hw=hw, ws=ws, w=w)
                q0 = np.clip(Q[n, i], C.LIM[:, 0] + 1e-6, C.LIM[:, 1] - 1e-6)
                sol = least_squares(resid, q0, bounds=(C.LIM[:, 0], C.LIM[:, 1]), args=(i, R, Q[n - 1, i], P0[n, i], T, others, hubs_o, ctx), max_nfev=60)
                Q[n, i] = sol.x
        if n % 20 == 0: print(n, f"{time.time() - t0:.0f} s", flush=True)
    np.save("flip4_path_raw.npy", Q); Q = smooth(Q, 4.0); np.save("flip4_path_opt.npy", Q)
    # report: loads, axis, balance (bounded thrust solve) along the path
    from scipy.optimize import lsq_linear
    print("  s    theta | worst load | thrust needed (max per prop, N) | CoM-centroid (mm)")
    for n in range(0, N + 1, 16):
        s = n / N; th = np.pi * F4.mj(s); R = F4.Rax(th); aw = active_w(th)
        on = [i for i in range(8) if (i in F4.FLYERS or aw > 0.5)]
        mp = M_PACK * (R @ np.array([0, 0, 0.03])) + C.M_BODY * (R @ np.array([0, 0, 0.05])); mm = C.M_BODY + M_PACK
        for j in range(8):
            a_, b_ = arm_mass(j, Q[n, j], R); mp = mp + a_; mm += b_
        com = mp / mm
        H = np.array([F4.hub(i, Q[n, i], c, R)[0] for i in on])
        A = np.vstack([np.ones(len(on)), (H[:, 1] - com[1]), -(H[:, 0] - com[0])])
        sol = lsq_linear(A, np.array([W, 0, 0]), bounds=(0, 60))
        Tm = sol.x
        L = max((np.abs(C.joint_torques(i, Q[n, i], c, R, (Tm[on.index(i)] if i in on else 0) * np.array([0, 0, 1.]), F4.hub(i, Q[n, i], c, R)[0])) / C.STALL).max() for i in range(8))
        cen = (H * Tm[:, None]).sum(0) / Tm.sum()
        print(f"{s:5.2f} {np.degrees(th):6.1f} | {L:.2f} | {Tm.round(1)} | {np.linalg.norm(cen[:2]-com[:2])*1000:.0f}")
    print("max step deg", np.degrees(np.abs(np.diff(Q, axis=0)).max()))
