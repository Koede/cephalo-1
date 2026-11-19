"""Chilo-1 rev 2 + Cephalo-2 (Forest's arm choice) as GLB models for the rotatable viewer.

Octopus = Cephalo-2: rev 4 tentacles (8 servo-in-link joints, j0 + j1 STS3250, j2-j7 STS3215, detent clutches, tip bay
with camera), bayonet connector end-mounted on every tip (socket + shared hand base, 9.9 mm), hand3 hub C2 on all eight
arms (rotor parts from the project CAD, drive as its envelope: hand3_mesh.py).  Planted arms 0/7 + 1/6: hand folded into
a fist, standing on its knuckles.  Free arms 2-5: fingers flat (rotor), axis up (assist props).
Centipede and arm-leg joint angles: MuJoCo state of chilo2_walk_assist4.npz.
python3 chilo_c2_model.py -> chilo_c2_stand.glb, chilo_c2_walk.glb, chilo_c2_model.json"""
import json
import numpy as np
import trimesh
import mujoco
from scipy.optimize import least_squares
from geom import frustum, sphere, box, torus, merge, Scene
import chilo2 as C, chilo_arm_legs as A, chilo_face_dock as FD
import hand3_mesh as HM

WHITE = (0.93, 0.93, 0.91); GREY = (0.55, 0.57, 0.60); DARK = (0.20, 0.21, 0.23); ORANGE = (0.95, 0.55, 0.15)
BLUE = (0.22, 0.42, 0.78); STEEL = (0.70, 0.72, 0.75); SVC = {"STS3215": BLUE, "STS3250": (0.28, 0.29, 0.32)}
LEGC = (0.95, 0.62, 0.27); FOOT = (0.12, 0.12, 0.13); HUBC = (0.45, 0.47, 0.50)
SV_DIM = (0.0452, 0.0247, 0.035)                                   # STS3215 / STS3250 case L W H (m)
S_J = A.S
FIST = (90.0, 90.0, 85.0)


def Ry(q):
    c, s = np.cos(q), np.sin(q); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(q):
    c, s = np.cos(q), np.sin(q); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def arm_frames(q, root, R0):
    """Per joint k: (centre, R before, R after); spigot frame (p, R)."""
    R = R0.copy(); p = root.copy(); s0 = 0.0; out = []
    for k in range(8):
        p = p + R[:, 0] * (S_J[k] - s0); s0 = S_J[k]
        Rb = R.copy(); R = R @ (Ry(q[k]) if k % 2 == 0 else Rz(q[k])); out.append((p.copy(), Rb, R.copy()))
    return out, (p + R[:, 0] * (A.SPIGOT - S_J[-1]), R)


def arm_mesh(sc, q, root, R0):
    """viz4.arm_mesh layout on the Cephalo-2 arm (servo boxes in their cradles, clutch cams + rockers, idler bosses,
    driven-link forks, tip bay with camera), then socket + hand base.  Returns the spigot frame."""
    fr, (pt, Rt) = arm_frames(q, root, R0)
    pts = [root] + [f[0] for f in fr] + [pt]
    ss = np.concatenate([[0], S_J, [A.SPIGOT]])
    sp = []
    for k in range(len(pts) - 1):
        r0 = float(np.interp(ss[k], [0, A.SPIGOT], [0.013, 0.008])); r1 = float(np.interp(ss[k + 1], [0, A.SPIGOT], [0.013, 0.008]))
        sp.append(frustum(pts[k], pts[k + 1], r0, r1, WHITE, n=14))
    sc.add(merge(sp))
    for k, (cj, Rb, Ra) in enumerate(fr):
        n = A.SERVO[k]; L, Wd, H = SV_DIM
        ax = Rb[:, 1] if k % 2 == 0 else Rb[:, 2]; o = np.cross(ax, Rb[:, 0])
        cpos = cj - Rb[:, 0] * (L / 2 - 0.0114)
        Rs = np.stack([Rb[:, 0], o, ax], 1)
        parts = [box(cpos, Rs, [L, Wd, H * 0.92], SVC[n])]
        for sg in (-1, 1):
            parts.append(box(cpos + o * sg * (Wd / 2 + 0.0025) - Rb[:, 0] * 0.004, Rs, [L + 0.006, 0.004, H * 0.80], WHITE))
        hw = H / 2
        if A.CLUTCH_M[k] > 0:
            rc = 0.019 if n == "STS3215" else 0.025
            parts.append(frustum(cj + ax * (hw + 0.001), cj + ax * (hw + 0.007), rc, rc, ORANGE, n=22))
            pd = cj + Ra[:, 0] * 0.026; Rd = np.stack([Ra[:, 0], np.cross(ax, Ra[:, 0]), ax], 1)
            parts.append(box(pd + ax * (hw + 0.010), Rd, [0.050, 0.010, 0.006], (0.80, 0.45, 0.12)))
            parts.append(frustum(pd + Ra[:, 0] * 0.022 + ax * (hw + 0.007), pd + Ra[:, 0] * 0.022 + ax * (hw + 0.014), 0.005, 0.005, STEEL, n=12))
        else:
            parts.append(frustum(cj + ax * (hw + 0.001), cj + ax * (hw + 0.005), 0.012, 0.012, DARK, n=16))
        parts.append(frustum(cj - ax * (hw + 0.006), cj - ax * (hw + 0.001), 0.011, 0.011, GREY, n=16))
        pd2 = cj + Ra[:, 0] * 0.012; od = np.cross(ax, Ra[:, 0])
        for sg in (-1, 1):
            parts.append(box(pd2 + ax * sg * (hw + 0.0035 + (0.008 if sg > 0 and A.CLUTCH_M[k] > 0 else 0)), np.stack([Ra[:, 0], od, ax], 1),
                             [0.034, 0.020, 0.003], WHITE))
        sc.add(merge(parts))
    u = Rt[:, 0]
    parts = [box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], (0.35, 0.36, 0.40))]
    pc = pt - u * 0.012 - Rt[:, 2] * 0.010                                         # tip camera on the ventral face
    parts.append(box(pc, Rt, [0.020, 0.018, 0.006], (0.10, 0.10, 0.12)))
    parts.append(frustum(pc - Rt[:, 2] * 0.003, pc - Rt[:, 2] * 0.007, 0.005, 0.006, (0.15, 0.62, 0.72), n=12))
    parts.append(frustum(pt - u * 0.0109, pt, 0.016, 0.016, ORANGE, n=32))           # arm socket (32 mm ring, end-mounted)
    parts.append(frustum(pt + u * 0.0002, pt + u * A.BAYONET, 0.016, 0.016, (0.30, 0.31, 0.34), n=32))   # hand base
    sc.add(merge(parts))
    return pt, Rt


def hand_meshes(pt, Rt, fold, clock=HM.CLOCK, spin=False):
    u = Rt[:, 0]; ex = Rt[:, 1]; Rh = np.column_stack([ex, np.cross(u, ex), u])
    A4 = np.eye(4); A4[:3, :3] = Rh; A4[:3, 3] = pt + u * A.BAYONET
    out = []
    for m, col in HM.hand(0.0, fold, clock=clock):
        m = m.copy(); m.apply_scale(1e-3); m.apply_transform(A4); out.append((m, col))
    if spin:                                                   # spinning rotor: thin tip-path ring at the blade plane
        ring = trimesh.creation.annulus(0.110, 0.1143, height=0.0015, sections=64)
        T4 = np.eye(4); T4[:3, 3] = [0, 0, 0.096]; ring.apply_transform(A4 @ T4); out.append((ring, (0.62, 0.66, 0.72)))
    return out


def octopus_body(sc, plate):
    """Cephalo-2 mantle on the face, axis = world +x: plate r 125, dome a 125 / c 150, turret r 92 with 4 side camera
    windows + 1 roof window (turretcams), arm portholes.  Belly rotor docked inside the face hub (not drawn)."""
    R = np.column_stack([[0, 0, -1.0], [0, 1.0, 0], [1.0, 0, 0]])        # octopus x down, y = y, z (up) = world +x
    T = lambda v: plate + R @ np.asarray(v, float)
    sc.add(frustum(T([0, 0, 0]), T([0, 0, 0.006]), 0.125, 0.125, GREY, n=64))
    V, F, Cc = sphere(np.zeros(3), 1.0, WHITE, n=48, m=24)
    V[:, 2] = np.maximum(V[:, 2], 0.0)
    V = (V * [0.125, 0.125, 0.150] + [0, 0, 0.006]) @ R.T + plate
    sc.add((V, F, Cc))
    parts = [frustum(T([0, 0, 0.12]), T([0, 0, 0.224]), 0.092, 0.092, WHITE, n=48), frustum(T([0, 0, 0.224]), T([0, 0, 0.228]), 0.092, 0.06, WHITE, n=48)]
    for k in range(4):
        a = np.radians(45 + 90 * k); d = np.array([np.cos(a), np.sin(a), 0])
        parts.append(box(T(d * 0.091 + [0, 0, 0.19]), R @ np.column_stack([d, [-d[1], d[0], 0], [0, 0, 1]]), [0.006, 0.03, 0.022], (0.55, 0.78, 0.95)))
    parts.append(frustum(T([0, 0, 0.226]), T([0, 0, 0.230]), 0.016, 0.016, (0.55, 0.78, 0.95), n=24))
    sc.add(merge(parts))
    pp = []
    for i in range(8):
        root, R0 = A.arm_frame(i, plate); e = R0[:, 0]
        pp.append(frustum(root - e * 0.004, root + e * 0.006, 0.031, 0.031, DARK, n=24))
    sc.add(merge(pp))


def free_hand_pose(i, plate, goal=None):
    """Hand arms 2-5: hand axis up (rotor lifting), tip clear of the mantle, joints <= 60 % of range, mid-range."""
    root, R0 = A.arm_frame(i, plate)
    lo, hi = A.LIM[:, 0] * 0.6, A.LIM[:, 1] * 0.6
    q0 = np.radians([-30, 0, -45, 0, -35, 0, -20, 0])
    side = np.sign(root[1]) if abs(root[1]) > 1e-3 else 1.0

    def res(q):
        _, _, sp, tp = A.fk(q, root, R0); u = (tp - sp) / np.linalg.norm(tp - sp)
        g = plate + np.array([0.05, side * 0.42, 0.20 if i in (3, 4) else 0.05]) if goal is None else goal     # hands out to the sides, rotors clear
        return np.concatenate([(u - [0, 0, 1.0]) * 1.0, (sp - g) * 2.0, 0.05 * q])
    r = least_squares(res, q0, bounds=(lo, hi))
    return r.x, root, R0


def scene_groups(prims):
    out = {}
    for V, F, Cc, _ in prims:
        cols = np.round(np.asarray(Cc, float), 3)
        for c in np.unique(cols, axis=0):
            m = np.all(cols == c, axis=1); Ff = F[m]; used = np.unique(Ff)
            remap = -np.ones(len(V), int); remap[used] = np.arange(len(used))
            out.setdefault(tuple(c), []).append(trimesh.Trimesh(V[used], remap[Ff], process=False))
    return out


LEG_CLOCK = 0.0          # leg rotors park as an X at 45 deg to the body axis (rotor_leg_check.py: >= 32 mm to the neighbours)
LEGS = [f"{'L' if s > 0 else 'R'}{seg}" for seg in range(1, 6) for s in (1, -1)]


def draw(m, d, tag, fist_arms, free, spin=False, fold=FIST):
    """Scene of the current MuJoCo state.  fist_arms: octopus arms drawn from the state with a fist; free: {arm: q}
    drawn in the head frame with flat fingers."""
    sc = Scene()
    hb = m.body("seg0").id; Rh = d.xmat[hb].reshape(3, 3); ph = d.xpos[hb]; h0 = np.array([C.SEG_X[0], 0, C.HC])
    for i in range(6):
        b = m.body(f"seg{i}").id; R = d.xmat[b].reshape(3, 3); p = d.xpos[b]; x = R[:, 0]
        FD.seg(sc, p - x * C.SEG_L[i] / 2, p + x * C.SEG_L[i] / 2, C.SEG_R[i])
        if i: sc.add(sphere(p + x * (C.PIVOT_X[i] - C.SEG_X[i]), 0.04, STEEL, n=12, m=8))
    sc.add(torus(ph + Rh @ (np.array([C.FACE_X + 0.004, 0, C.HC]) - h0), Rh[:, 0], 0.109, 0.009, HUBC, n=48, m=10))
    hands = []; info = {}
    for n in LEGS:
        pts = [d.xpos[m.body(f"{n}_hip").id]] + [d.xpos[m.body(f"{n}_l{k}").id] for k in range(1, 5)] + [d.site_xpos[m.site(f"{n}_ft").id]]
        parts = [frustum(a, b, 0.017, 0.016, LEGC, n=10) for a, b in zip(pts[:-1], pts[1:])]
        for k, p in enumerate(pts[:-1]):
            parts.append(sphere(p, 0.026 if k else 0.03, BLUE, n=10, m=6))
            if 0 < k < 4: parts.append(sphere(0.5 * (p + pts[k + 1]), 0.02, BLUE, n=8, m=6))
        Rt = d.xmat[m.body(f"{n}_l4").id].reshape(3, 3); u = Rt[:, 0]; pt = pts[-1]
        parts.append(box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], (0.35, 0.36, 0.40)))          # hand electronics (ESC + node)
        parts.append(frustum(pt - u * 0.0109, pt, 0.016, 0.016, ORANGE, n=32))                     # socket
        parts.append(frustum(pt + u * 0.0002, pt + u * A.BAYONET, 0.016, 0.016, (0.30, 0.31, 0.34), n=32))   # hand base
        sc.add(merge(parts))
        hands += hand_meshes(pt, Rt, (0, 0, 0), clock=LEG_CLOCK, spin=spin)
    tmp = Scene(); octopus_body(tmp, C.PLATE)
    for V, F, Cc, _ in tmp.prims: sc.add(((V - h0) @ Rh.T + ph, F, Cc))
    for i in fist_arms:
        rb = m.body(f"arm{i}_root").id
        q = np.array([d.qpos[m.jnt_qposadr[m.joint(f"arm{i}_q{k}").id]] for k in range(8)])
        pt, Rt = arm_mesh(sc, q, d.xpos[rb], d.xmat[rb].reshape(3, 3))
        hands += hand_meshes(pt, Rt, fold, spin=spin)
        info[i] = dict(role="leg" if fold == FIST else "prop", q_deg=np.degrees(q).round(1).tolist(), hand_axis_from_down_deg=round(float(np.degrees(np.arccos(-Rt[2, 0]))), 1))
    for i, q in free.items():
        root, R0 = A.arm_frame(i, C.PLATE)
        tmp = Scene(); pt, Rt = arm_mesh(tmp, q, root, R0)
        for V, F, Cc, _ in tmp.prims: sc.add(((V - h0) @ Rh.T + ph, F, Cc))
        pw = (pt - h0) @ Rh.T + ph; Rw = Rh @ Rt
        hands += hand_meshes(pw, Rw, (0, 0, 0), spin=spin)
        info[i] = dict(role="hand / prop", q_deg=np.degrees(q).round(1).tolist(), hand_axis_from_up_deg=round(float(np.degrees(np.arccos(Rw[2, 0]))), 1))
    gl = trimesh.Scene()
    for c, lst in scene_groups(sc.prims).items():
        g = trimesh.util.concatenate(lst); g.visual = trimesh.visual.ColorVisuals(g, face_colors=np.tile(np.r_[np.array(c) * 255, 255], (len(g.faces), 1)).astype(np.uint8))
        gl.add_geometry(g)
    byc = {}
    for mm, c in hands: byc.setdefault(tuple(np.round(c, 3)), []).append(mm)
    for c, lst in byc.items():
        g = trimesh.util.concatenate(lst); g.visual = trimesh.visual.ColorVisuals(g, face_colors=np.tile(np.r_[np.array(c) * 255, 255], (len(g.faces), 1)).astype(np.uint8))
        gl.add_geometry(g)
    Tm = np.eye(4); Tm[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    ctr = np.array([d.subtree_com[1][0], d.subtree_com[1][1], 0.0]); Tm[:3, 3] = -Tm[:3, :3] @ ctr
    gl.apply_transform(Tm)
    gl.export(f"chilo_c2_{tag}.glb")
    return dict(arms=info, faces=int(sum(len(g.faces) for g in gl.geometry.values())))


def build(t, tag, npz="chilo2_walk_assist4.npz"):
    m = C.build(); d = mujoco.MjData(m)
    z = np.load(npz); f = int(np.argmin(np.abs(z["t"] - t)))
    d.qpos[:] = z["qpos"][f]; mujoco.mj_kinematics(m, d)
    free = {i: free_hand_pose(i, C.PLATE)[0] for i in C.ARM_HANDS}
    out = draw(m, d, tag, C.ARM_LEGS, free); out["t"] = float(z["t"][f]); return out


# ------------------------------------------------------------------ flight: every rotor up
FLY_Z = 0.62                      # body axis height in the flight pose (m)
LEG_UP = {1: 0.045, 2: 0.0, 3: 0.045, 4: 0.0, 5: 0.045}      # leg spigot height above its hip: neighbours staggered 45 mm
                                                             # so their spinning discs (Ø229, 235 mm apart) do not cross
ARM_GOALS = {0: (0.20, 0.13, -0.30), 7: (0.20, -0.13, -0.30), 1: (0.02, 0.45, 0.02), 6: (0.02, -0.45, 0.02),
             2: (-0.04, 0.40, 0.26), 5: (-0.04, -0.40, 0.26), 3: (0.25, 0.14, 0.36), 4: (0.25, -0.14, 0.36)}   # spigot - plate (m)


def leg_up_q(r, z):
    """Leg swung up: spigot at (r outward, z up) in the leg plane, shank pointing straight up (hand3 rotor up)."""
    def res(q4):
        return np.r_[(C.leg_fk_planar(q4)[-1] - [r, z]) * 10, (C.leg_shank_angle(q4) + np.pi / 2) * 0.5, 0.01 * q4]
    best = None
    for q0 in ([-0.8, -0.8, -0.8, -0.8], [-1.2, -0.6, -0.6, -0.8], [-0.4, -1.0, -1.0, -0.6]):
        s = least_squares(res, q0, bounds=(C.LEG_LIM[1:, 0] * 0.9, C.LEG_LIM[1:, 1] * 0.9))
        if best is None or s.cost < best.cost: best = s
    return np.r_[0.0, best.x]


def flight_pose():
    m = C.build(); d = mujoco.MjData(m)
    d.qpos[:] = 0; d.qpos[0:3] = [C.SEG_X[0], 0, FLY_Z]; d.qpos[3] = 1.0
    for seg in range(1, 6):
        for s in (1, -1):
            n = f"{'L' if s > 0 else 'R'}{seg}"
            p = np.array([0.0, 0.17, LEG_UP[seg]])
            q = leg_up_q(p[1], p[2]); q[0] *= -s
            for k in range(5): d.qpos[m.jnt_qposadr[m.joint(f"{n}_q{k}").id]] = q[k]
    arms = {i: free_hand_pose(i, C.PLATE, C.PLATE + np.array(ARM_GOALS[i]))[0] for i in range(8)}
    for i in C.ARM_LEGS:
        for k in range(8): d.qpos[m.jnt_qposadr[m.joint(f"arm{i}_q{k}").id]] = arms[i][k]
    mujoco.mj_kinematics(m, d)
    return m, d, arms


def rotor_discs(m, d, arms):
    """(centre, axis) of all 18 hand3 rotor discs (blade plane, world)."""
    hb = m.body("seg0").id; Rh = d.xmat[hb].reshape(3, 3); ph = d.xpos[hb]; h0 = np.array([C.SEG_X[0], 0, C.HC])
    out = {}
    for n in LEGS:
        u = d.xmat[m.body(f"{n}_l4").id].reshape(3, 3)[:, 0]; sp = d.site_xpos[m.site(f"{n}_ft").id]
        out[n] = (sp + u * (A.BAYONET + 0.096), u)
    for i, q in arms.items():
        root, R0 = A.arm_frame(i, C.PLATE); _, _, sp, tp = A.fk(q, root, R0); u = (tp - sp) / np.linalg.norm(tp - sp)
        out[f"arm{i}"] = ((sp + u * (A.BAYONET + 0.096) - h0) @ Rh.T + ph, Rh @ u)
    return out


def disc_gaps(discs, R=0.1143, half_t=0.004):
    """Smallest gap between any two spinning discs (sampled), and the robot body (segment shells as capsules)."""
    pts = {}
    for k, (c, u) in discs.items():
        e1 = np.cross(u, [0.31, 0.57, 0.76]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
        rr, aa = np.meshgrid(np.linspace(0, R, 7), np.linspace(0, 2 * np.pi, 48, endpoint=False))
        pts[k] = c + (rr * np.cos(aa)).reshape(-1, 1) * e1 + (rr * np.sin(aa)).reshape(-1, 1) * e2
    keys = list(pts); worst = (1.0, None)
    for a in range(len(keys)):
        for b in range(a + 1, len(keys)):
            g = np.min(np.linalg.norm(pts[keys[a]][:, None] - pts[keys[b]][None], axis=2)) - 2 * half_t
            if g < worst[0]: worst = (g, (keys[a], keys[b]))
    return worst


def disc_body_gap(m, d, discs, R=0.1143):
    """Smallest gap from the spinning discs to the centipede shells and the octopus mantle / turret (approx. solids)."""
    hb = m.body("seg0").id; Rh = d.xmat[hb].reshape(3, 3); ph = d.xpos[hb]; h0 = np.array([C.SEG_X[0], 0, C.HC])
    caps = []
    for i in range(6):
        b = m.body(f"seg{i}").id; Rb = d.xmat[b].reshape(3, 3); p = d.xpos[b]; x = Rb[:, 0]
        caps.append((p - x * (C.SEG_L[i] / 2 - C.SEG_R[i] * 0.5), p + x * (C.SEG_L[i] / 2 - C.SEG_R[i] * 0.5), C.SEG_R[i]))
    pl = C.PLATE - h0
    caps.append((ph + Rh @ (pl + [0.0, 0, 0]), ph + Rh @ (pl + [0.10, 0, 0]), 0.125))     # mantle + dome (as a capsule)
    caps.append((ph + Rh @ (pl + [0.10, 0, 0]), ph + Rh @ (pl + [0.228, 0, 0]), 0.092))  # turret
    worst = (1.0, None)
    for k, (c, u) in discs.items():
        e1 = np.cross(u, [0.31, 0.57, 0.76]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
        rr, aa = np.meshgrid(np.linspace(0, R, 8), np.linspace(0, 2 * np.pi, 60, endpoint=False))
        P = c + (rr * np.cos(aa)).reshape(-1, 1) * e1 + (rr * np.sin(aa)).reshape(-1, 1) * e2
        for j, (a, b, r) in enumerate(caps):
            t = np.clip((P - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
            g = np.min(np.linalg.norm(P - (a + t[:, None] * (b - a)), axis=1)) - r - 0.004
            if g < worst[0]: worst = (g, (k, "seg%d" % j if j < 6 else ("mantle" if j == 6 else "turret")))
    return worst


if __name__ == "__main__":
    out = {"stand": build(2.0, "stand"), "walk": build(7.3, "walk")}
    m, d, arms = flight_pose()
    g, pair = disc_gaps(rotor_discs(m, d, arms))
    out["fly"] = draw(m, d, "fly", [i for i in C.ARM_LEGS], {i: arms[i] for i in C.ARM_HANDS}, spin=True, fold=(0, 0, 0))
    out["fly"]["rotor_disc_gap_min_mm"] = round(g * 1e3, 1); out["fly"]["closest_pair"] = pair
    gb, pb = disc_body_gap(m, d, rotor_discs(m, d, arms)); out["fly"]["rotor_body_gap_min_mm"] = round(gb * 1e3, 1); out["fly"]["closest_body"] = pb
    json.dump(out, open("chilo_c2_model.json", "w"), indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "arms"} for k, v in out.items()}, indent=1))
    print({i: v.get("hand_axis_from_up_deg") for i, v in out["fly"]["arms"].items()})


