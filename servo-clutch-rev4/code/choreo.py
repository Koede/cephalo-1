"""Shared motion scripts.  The SAME commands go to both wrist variants; only the IK reach (wrist lead) and the
wrist's own clamp differ.  Output: per-frame state arrays saved to states_<video>_<variant>.npz"""
import numpy as np
from robot import arm_fk, arm_ik, STAND, foot_drop, J_LIM
from wrists import BallDrive, OmniWrist

FPS = 30
Z = np.array([0, 0, 1.0])
FOLD_WALK = np.radians(60)
FLIGHT = np.radians([-20, 0, -20, 0, 0, 0, 0, 0, 0])        # wide flight pose (wrist turns the fans up)
ORDER = [0, 4, 2, 6, 1, 5, 3, 7]                            # ripple gait
SWING, STRIDE, LIFT = 0.25, 0.09, 0.045
CYCLE = SWING * 8
V = STRIDE / CYCLE


def ss(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def slerp_via(a, b, mid, s):
    """a -> mid -> b on the sphere (two great-circle legs)."""
    def sl(p, q, f):
        om = np.arccos(np.clip(np.dot(p, q), -1, 1))
        if om < 1e-6:
            return p
        return (np.sin((1 - f) * om) * p + np.sin(f * om) * q) / np.sin(om)
    return sl(a, mid, s * 2) if s < 0.5 else sl(mid, b, s * 2 - 1)


def stand_geometry(wrist):
    """Body height and nominal footholds (relative to body xy) for the stand pose with this wrist."""
    feet, h = [], None
    for i in range(8):
        _, (R, p) = arm_fk(i, STAND)
        c = p + R[:, 0] * wrist.lead()
        foot = c - Z * (wrist.c_to_stalk() + foot_drop(FOLD_WALK))
        feet.append(foot)
    feet = np.array(feet)
    h = -feet[:, 2].mean()
    feet[:, 2] = 0
    return h, feet


def walk_takeoff(wrist):
    """Video 1: ripple walk -> EDF spin-up -> legs up in pairs (approved take-off order) -> rise -> hover."""
    T_WALK, T_IDLE, T_PAIR, T_RISE, T_HOVER = 5.0, 1.5, 0.8, 1.5, 3.0
    groups = [(2, 6), (1, 5), (3, 7), (4,), (0,)]
    T = T_WALK + T_IDLE + T_PAIR * len(groups) + T_RISE + T_HOVER
    N = int(round(T * FPS))
    h, feet0 = stand_geometry(wrist)
    lead, c2s = wrist.lead(), wrist.c_to_stalk()
    q = np.tile(STAND, (8, 1))
    out = dict(body=[], q=[], u=[], fold=[], prop=[], edf=[], cap=[], cam=[], tgt=[], flight=[])
    up_start = {}
    t_up0 = T_WALK + T_IDLE
    for gi, g in enumerate(groups):
        for leg in g:
            up_start[leg] = t_up0 + gi * T_PAIR
    q_lift = {}
    for f in range(N):
        t = f / FPS
        xb = V * min(t, T_WALK)
        zb = h
        if t > t_up0 + T_PAIR * len(groups):
            zb = h + 0.15 * ss((t - t_up0 - T_PAIR * len(groups)) / T_RISE)
            zb += 0.006 * np.sin(2 * np.pi * (t - 12) / 1.7) * ss((t - 12.5))
        body = np.array([xb, 0, zb])
        u_all, fold_all, prop_all = [], [], []
        for i in range(8):
            slot = ORDER.index(i)
            # foothold from the gait clock (frozen once walking ends)
            tw = min(t, T_WALK)
            ph = (tw / CYCLE - slot / 8) % 1.0
            n_cyc = np.floor(tw / CYCLE - slot / 8)
            t_land_prev = (n_cyc + slot / 8) * CYCLE + SWING
            def land(tl):
                return feet0[i] + [V * tl + 0.039, 0, 0]
            swinging = ph < SWING / CYCLE and t < T_WALK
            if swinging:
                s = ph * CYCLE / SWING
                a = land(t_land_prev - CYCLE); b = land(t_land_prev)
                foot = a + (b - a) * ss(s) + Z * LIFT * np.sin(np.pi * s)
            else:
                foot = land(t_land_prev if ph >= SWING / CYCLE else t_land_prev - CYCLE)
            if i in up_start and t >= up_start[i]:
                s = ss((t - up_start[i]) / T_PAIR)
                if i not in q_lift:
                    q_lift[i] = q[i].copy()
                q[i] = q_lift[i] + (FLIGHT - q_lift[i]) * s
                R0 = arm_fk(i, q[i], body)[1][0]
                out_dir = np.array([np.cos(np.radians(22.5 + 45 * i)), np.sin(np.radians(22.5 + 45 * i)), 0])
                u_all.append(slerp_via(-Z, Z, out_dir, s)); fold_all.append(FOLD_WALK * (1 - s)); prop_all.append(s > 0.02)
            else:
                c_target = foot + Z * (c2s + foot_drop(FOLD_WALK))
                q[i] = arm_ik(i, c_target, lead, q[i], STAND, body)
                u_all.append(-Z); fold_all.append(FOLD_WALK); prop_all.append(False)
        flying = t > t_up0
        if t < T_WALK:
            cap = "Walking: ripple gait, one leg in swing, fans off"
        elif t < t_up0:
            cap = "Standing: central EDF spins up"
        elif t < t_up0 + T_PAIR * len(groups):
            cap = "Take-off: legs swing up in pairs 2+6, 1+5, 3+7, then 4, then 0; wrists turn the fans up"
        elif t < t_up0 + T_PAIR * len(groups) + T_RISE:
            cap = "Lift-off: fans pull up, body rises 150 mm"
        else:
            cap = "Hover: arms spread wide, every fan held vertical by its wrist"
        az = np.radians(-70 + 35 * t / T)
        tgt = np.array([xb, 0, h * 0.62 + (zb - h)])
        eye = tgt + 2.65 * np.array([np.cos(az) * np.cos(0.33), np.sin(az) * np.cos(0.33), np.sin(0.33)])
        out["body"].append(body); out["q"].append(q.copy()); out["u"].append(np.array(u_all)); out["fold"].append(fold_all)
        out["prop"].append(prop_all); out["edf"].append(min(max((t - T_WALK) / T_IDLE, 0), 1)); out["cap"].append(cap)
        out["cam"].append(eye); out["tgt"].append(tgt); out["flight"].append(flying)
    return {k: np.array(v) for k, v in out.items()}, N


def arms_moving(wrist, h_hover):
    """Video 2: hover with waving arms -> whole-arm droop sweep -> close-up wrist range sweep on arm 0."""
    TA, TB, TC = 9.0, 6.0, 7.0
    T = TA + TB + TC
    N = int(round(T * FPS))
    out = dict(body=[], q=[], u=[], fold=[], prop=[], edf=[], cap=[], cam=[], tgt=[], flight=[], roll=[], trail=[])
    w = 2 * np.pi / 3.0
    for f in range(N):
        t = f / FPS
        body = np.array([0, 0, h_hover + 0.006 * np.sin(2 * np.pi * t / 1.7)])
        q = np.tile(FLIGHT, (8, 1))
        u = np.tile(Z, (8, 1))
        roll = 0.0; trail = False
        if t < TA:
            env = ss(t / 1.0) * ss((TA - t) / 1.0)
            for i in range(8):
                ph = i * np.pi / 4
                d = np.radians([20 * np.sin(w * t + ph), 15 * np.sin(0.7 * w * t + ph), 35 * np.sin(w * t + ph + 0.6), 0,
                                30 * np.sin(w * t + ph + 1.2), 0, 25 * np.sin(w * t + ph + 1.8), 0, 20 * np.sin(w * t + ph + 2.4)])
                q[i] = np.clip(FLIGHT + env * d, J_LIM[:, 0], J_LIM[:, 1])
            cap = "Arms waving (travelling wave around the body); each wrist tries to keep its fan vertical"
            az = np.radians(-60 + 25 * t / TA); dist = 2.45; tgt = body * [1, 1, 1] + [0, 0, 0.12]
        elif t < TA + TB:
            s = (t - TA) / TB
            k = np.sin(np.pi * s) ** 2                          # 0 -> 1 -> 0
            droop = np.radians([-40, 0, -35, 0, -15, 0, 0, 0, 0]) * (1 - 2 * k) + np.radians([35, 0, 40, 0, 20, 0, 10, 0, 0]) * 0
            pose_up = np.radians([-35, 0, -30, 0, -10, 0, 0, 0, 0])
            pose_dn = np.radians([30, 0, 30, 0, 15, 0, 0, 0, 0])
            for i in range(8):
                q[i] = pose_up + (pose_dn - pose_up) * k
            cap = "Arms sweep from curled up to splayed down; each fan stays vertical until its wrist runs out of range"
            az = np.radians(-35 + 20 * s); dist = 2.45; tgt = body + [0, 0, 0.05]
        else:
            s = (t - TA - TB) / TC
            q[0] = np.radians([0, 0, 0, 0, 0, 0, 0, 0, 0])
            ss_ = ss(s / 0.1)
            # spiral: polar angle from arm axis 0 -> 180 deg, 5 turns
            th = np.pi * ss(s); phi = 2 * np.pi * 5 * s
            R0 = arm_fk(0, q[0], body)[1][0]
            tdir = R0[:, 0]; e1 = R0[:, 2]; e2 = np.cross(tdir, e1)
            ucmd = np.cos(th) * tdir + np.sin(th) * (np.cos(phi) * e1 + np.sin(phi) * e2)
            u[0] = Z + (ucmd - Z) * ss_ if ss_ < 1 else ucmd
            u[0] /= np.linalg.norm(u[0])
            roll = 3.0; trail = True
            cap = "Range sweep on one arm: the fan is commanded around the whole sphere; dots show where it actually points"
            tip = arm_fk(0, q[0], body)[1][1]
            tgt = tip + tdir * 0.08
            az = np.radians(22.5 - 95 + 15 * s); dist = 0.62
        el = 0.30 if t < TA + TB else 0.30
        eye = tgt + dist * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
        out["body"].append(body); out["q"].append(q.copy()); out["u"].append(u.copy()); out["fold"].append(np.zeros(8))
        out["prop"].append(np.ones(8, bool)); out["edf"].append(1.0); out["cap"].append(cap)
        out["cam"].append(eye); out["tgt"].append(tgt); out["flight"].append(True); out["roll"].append(roll); out["trail"].append(trail)
    return {k: np.array(v) for k, v in out.items()}, N


if __name__ == "__main__":
    import time
    for W in (BallDrive, OmniWrist):
        t0 = time.time()
        st, N = walk_takeoff(W())
        h = st["body"][-1][2]
        np.savez(f"states_walk_{W.__name__}.npz", **st)
        st2, N2 = arms_moving(W(), h)
        np.savez(f"states_arms_{W.__name__}.npz", **st2)
        print(W.__name__, N, N2, "hover h", round(h, 3), "time", round(time.time() - t0, 1))
