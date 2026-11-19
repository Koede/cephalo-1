"""Flip and walk upside down: Cephalo-1 rolls over in mid-air, lands on its feet dome-down and walks inverted.

Uses the body4 robot (full MuJoCo model, battery sag, prop reaction torque, spinal tracker, clutches) with the XL
claw-props (7.5 in, CEPH_PROP=xl), ONE pack (CEPH_PACKS=1: with two the roll runs out of thrust margin) and an STS3250
at the root yaw j1 (CEPH_J1=STS3250: the STS3215 there reaches 1.2 x stall mid-roll).  Sequence:
  1  hover upright (tulip pose, 8 claw-props + EDF);
  2  EDF spools down: the claw-props take the whole weight (the EDF cannot help once the body tilts);
  3  gimballed roll, 180 deg in 4 s about the horizontal axis through arms 0 and 4: the body turns over while the
     arms keep their claw-prop axes vertical (tentacle gimbal), on a path solved offline (flip_pathopt.py: axes
     vertical, joint loads, thrust centroid over the CoM, clearances).  Flyers 0, 4 (on the axis: the bend passes
     from the pitch to the yaw joints and back), 1, 2, 3 (over the top).  Arms 5, 6, 7 swing under the body, stop
     their claw-props while the axes are past 40 deg and turn them back up from 62 % of the roll.  Yaw is held by
     leaning the claw axes tangentially (5 props cannot balance the rotor torques by thrust alone);
  4  inverted hover: the stand pose is the inverted flight pose (claw axes point away from the dome = up);
  5  inverted landing: descend, then opposite leg pairs swing down to the floor one after another (0+4, 2+6, 1+5,
     3+7), each landing before the next starts; the planted legs hold the planned body pose and take 16 N each from
     the claw-props; props spin down;
  6  inverted walk: ripple gait (body4 Walk) in the "spider" stance (knees up, plate 0.32 m above the floor, feet
     at r 0.30 m, dome 16 cm clear).  A landed foot is latched where it actually touched (otherwise the IK drags it
     toward the planned spot and opposite legs squeeze each other through toe friction); the load governor is off
     for this gait (its slow-downs made the stiff legs rock the body).
python3 flip4.py   ->  flip4.npz / flip4.json      (FLIP_START=stand: bench start in the inverted stand)
"""
import os, sys, json, time
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "1"); os.environ.setdefault("CEPH_J1", "STS3250")
import numpy as np, mujoco
from scipy.optimize import lsq_linear
import cephalo4 as C
import sim4 as S4
# dome crown as a contact sphere (inside the real dome; the rest of the dome never touches anything here)
S4.TORSO_XML = '<geom name="dome" type="sphere" pos="0 0 0.066" size="0.090" mass="0" contype="1" conaffinity="1"/>'
import body4 as B4
import power4 as PW

D = C.DEG
mj = B4.mj
Z = np.array([0, 0, 1.0])
RX = np.diag([1.0, -1.0, -1.0])                     # inverted body: Rz(yaw) @ RX
AX = np.radians(22.5)
U = np.array([np.cos(AX), np.sin(AX), 0.0])
SPIN = B4.SPIN
Q_FLY, Q_STAND = B4.Q_FLY, B4.Q_STAND
FLYERS, UNDER = [0, 1, 2, 3, 4], [5, 6, 7]
PAIRS = [(0, 4), (2, 6), (1, 5), (3, 7)]
H_INV, RF_INV = float(os.environ.get("FLIP_H", 0.32)), float(os.environ.get("FLIP_RF", 0.30))                          # inverted stand: plate height, foot radius
Z0 = 1.00
T_EDF0, T_EDF = 1.0, 1.0
T_R0, T_R = 2.4, 4.0
T_DES0, T_DES = T_R0 + T_R + 1.2, 3.0
T_LAND0 = T_DES0 + T_DES + 0.4
T_SW, T_PAIR = 1.7, 1.45
F_FOOT = 16.0                                       # load a planted foot takes over from the claw-props (N)
WALK = tuple(float(x) for x in os.environ.get("FLIP_WALK", "0.18,0,20").split(","))
S_REJOIN = float(os.environ.get("FLIP_REJOIN", 0.62))   # roll fraction where arms 5-7 start turning their claw axes back up
GOVERNOR = bool(int(os.environ.get("FLIP_GOV", "0")))   # the load governor destabilises the inverted ripple gait (see doc)
K_SHARE = float(os.environ.get("FLIP_KSHARE", 0.0))   # load-sharing reflex gain (m per N s)
I_ROLL = 0.5                                        # roll inertia for the feed-forward (kg m^2, est.)
T_END = float(os.environ.get("FLIP_T", "40"))


def Rax(th):
    K = np.array([[0, -U[2], U[1]], [U[2], 0, -U[0]], [-U[1], U[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def hub(i, q, bp, bR):
    _, _, R, p = B4.fk(i, q, bp, bR)
    return p + R[:, 0] * C.BLADE_OFF, R[:, 0]


def roll_s(t):
    return float(np.clip((t - T_R0) / T_R, 0, 1))


def roll_theta(t):
    u = roll_s(t)
    return np.pi * mj(u), np.pi * 30 * u * u * (1 - u) ** 2 / T_R


# ---------------------------------------------------------------- kinematic roll path (body frame joint angles)
def roll_path(N=160, bump=0.08):
    """Arm poses along the roll, solved with the body at the reference attitude: flyers keep their claw axes vertical,
    hubs of 0 / 4 follow a path around the roll axis (pushed 8 cm out along it mid-roll so they clear arms 1 / 3),
    arm 2's pitch joints counter-rotate exactly, arms 1 / 3 are the same blend refined by IK; 5, 6, 7 interpolate."""
    c = np.zeros(3)
    V = np.cross(Z, U)
    def cyl(h): return h @ U, np.hypot(h @ V, h @ Z), np.arctan2(h @ Z, h @ V)
    def fromcyl(x, r, f): return x * U + r * np.cos(f) * V + r * np.sin(f) * Z
    H0 = [hub(i, Q_FLY, c, np.eye(3))[0] for i in range(8)]
    H1 = [hub(i, Q_STAND, c, Rax(np.pi))[0] for i in range(8)]
    q = np.tile(Q_FLY, (8, 1)); out = [q.copy()]
    for n in range(1, N + 1):
        s = n / N; th = np.pi * mj(s); R = Rax(th); u = th / np.pi
        qn = Q_FLY + (Q_STAND - Q_FLY) * u
        for i in range(8):
            if i in (0, 4):
                x0, r0, f0 = cyl(H0[i]); x1, r1, f1 = cyl(H1[i])
                tgt = fromcyl(x0 + (x1 - x0) * s + np.sign(x0) * bump * np.sin(np.pi * s), r0 + (r1 - r0) * s, f0 + (f1 - f0) * s)
                nom = Q_FLY + (Q_STAND - Q_FLY) * mj(s); qq = q[i].copy()
                for _ in range(12): qq = B4.ik(i, tgt, Z, qq, nom, c, R, lead=C.BLADE_OFF, w_ax=6.0, w_reg=0.05)
            elif i in (1, 3):
                tgt = hub(i, qn, c, R)[0]; qq = q[i].copy()
                for _ in range(12): qq = B4.ik(i, tgt, Z, qq, qn, c, R, lead=C.BLADE_OFF, w_ax=30.0, w_reg=1.0)
            else:
                qq = qn.copy()
            q[i] = qq
        out.append(q.copy())
    return np.array(out)


def inv_pose(H=H_INV, rf=RF_INV):
    bp = np.array([0, 0, H]); Q = np.zeros((8, 8))
    qs = np.radians([45, 0, -65, 0, -80, 0, 10, 0])
    for i in range(8):
        a = -np.radians(22.5 + 45 * i)
        tgt = np.array([rf * np.cos(a), rf * np.sin(a), 0.0]); q = qs.copy()
        for _ in range(20): q = B4.ik(i, tgt, B4.DOWN, q, qs, bp, RX)
        Q[i] = q
    return Q


CACHE = f"flip4_path_{H_INV:.2f}_{RF_INV:.2f}.npz"
if os.path.exists(CACHE):
    _z = np.load(CACHE); PATH, Q_INV = _z["path"], _z["qinv"]
else:
    PATH, Q_INV = roll_path(), inv_pose()
    np.savez(CACHE, path=PATH, qinv=Q_INV)
if os.path.exists("flip4_path_opt.npy"):        # load-aware refinement of the flyers' path (flip_pathopt.py)
    PATH = np.load("flip4_path_opt.npy")
F_INV = np.array([B4.foot(i, Q_INV[i], np.array([0, 0, H_INV]), RX)[0] for i in range(8)])


def path_at(s):
    x = s * (len(PATH) - 1); k = int(min(np.floor(x), len(PATH) - 2)); f = x - k
    return PATH[k] * (1 - f) + PATH[k + 1] * f


# ---------------------------------------------------------------- the robot
class Flip(B4.Body):
    def __init__(self):
        super().__init__([], start="stand")
        m, d = self.m, self.d
        for i in range(8):
            g = m.geom(f"prop{i}").id; m.geom_contype[g] = 0; m.geom_conaffinity[g] = 0
        self.belly_ids |= {m.geom("dome").id}
        d.qpos[:] = 0; d.qvel[:] = 0; d.qpos[3] = 1.0; d.qpos[2] = Z0
        d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]; d.qpos[self.imp_q + 3] = 1
        for i in range(8):
            for k in range(8):
                d.qpos[self.ja[i, k]] = Q_FLY[k] / 2; d.qpos[self.jb[i, k]] = Q_FLY[k] / 2
        q = np.tile(Q_FLY, (8, 1))
        self.q_cmd = q.copy(); self.goal = q.copy(); self.goal_prev = q.copy(); self.q_ik = q.copy(); self.qfly = q.copy()
        d.ctrl[:] = q.ravel(); mujoco.mj_forward(m, d)
        self.bat = PW.Battery(B4.PACKS, float(os.environ.get("CEPH_SOC", "1.0")))
        self.edf = 34.0; self.edf_target = 34.0
        self.fan = np.full(8, (C.W_ROBOT - 34.0) / 8); self.fans_on = True
        self.gate = np.ones(8); self.gate_s = np.ones(8); self.t_in = [-1.0] * 8
        p = d.subtree_com[self.torso][:2].copy()
        self.xyplan = B4.Quintic(0, 1e-3, p, p); self.zplan = B4.Quintic(0, 1e-3, Z0, Z0)
        self.T_line = C.T_LINE["firm"]; self.winch_target = self.T_line
        self.iatt = np.zeros(3); self.Mz_dem = 0.0; self.axy = np.zeros(2)
        self.phase = "hover"; self.label = "HOVER upright (8 claw-props + EDF)"
        self.stance = np.zeros(8, bool); self.spots = np.zeros((8, 3)); self.u_sw = np.zeros(8); self.q_sw0 = None
        self.prop_scale = 1.0; self.walk = None; self.M = dict(att=0.0, exy=0.0, servo={}, slip={}, sat={}, fan=0.0, vmin=99.0, dome=0.0, ez=0.0)
        self.R_ref = np.eye(3); self.w_ref = np.zeros(3); self.dz = np.zeros(8)
        self.att_err = 0.0

    # ---- inverted ground helpers (body4 Walk drives these)
    def ground_ik(self, arms):
        bR = B4.Rz(self.byaw) @ RX
        # load-sharing reflex (walk/stand): a planted foot that carries more than the average is raised a little, one
        # that carries less is lowered (integral, +-15 mm); stiff position-controlled legs otherwise rock the body
        st = [i for i in range(8) if self.swing_foot.get(i) is None and self.foot_f[i] > 0.5]
        if self.phase in ("walk", "done") and len(st) >= 5:
            Fm = float(np.mean(self.foot_f[st]))
            for i in range(8):
                if i in st: self.dz[i] = np.clip(self.dz[i] + K_SHARE * (self.foot_f[i] - Fm) * B4.CTRL_DT, -0.015, 0.015)
                elif self.swing_foot.get(i) is not None: self.dz[i] *= 0.9
        for i in arms:
            tgt = self.swing_foot.get(i)
            if tgt is None: tgt = self.feet[i] + np.array([0, 0, self.dz[i]])
            self.q_ik[i] = B4.ik(i, tgt, B4.DOWN, self.q_ik[i], Q_INV[i], self.bp, bR)
            self.goal[i] = self.q_ik[i]

    # ---- flight arms: claw axes vertical (+ lean for position and yaw), poses from the roll path
    def gimbal(self, t, arms, qnom, wax=30.0):
        d = self.d
        p = d.xpos[self.torso].copy(); R = d.xmat[self.torso].reshape(3, 3); com = d.subtree_com[self.torso]
        f = np.maximum(self.fan, 2.0)
        hubs = {i: hub(i, self.q_cmd[i], p, R)[0] for i in arms}
        rr = {i: hubs[i][:2] - com[:2] for i in arms}
        den = max(sum(f[i] * np.linalg.norm(rr[i]) for i in arms), 1e-6)
        for i in arms:
            tn = np.array([-rr[i][1], rr[i][0]]) / max(np.linalg.norm(rr[i]), 1e-6)
            lean = self.axy / C.G + tn * np.clip(self.Mz_dem, -4, 4) / den * 1.0
            ln = np.linalg.norm(lean)
            if ln > np.sin(25 * D): lean *= np.sin(25 * D) / ln
            ax = np.array([lean[0], lean[1], np.sqrt(max(1 - lean @ lean, 0.0))])
            tgt = hub(i, qnom[i], p, R)[0]
            q = self.goal[i].copy()
            for _ in range(3): q = B4.ik(i, tgt, ax, q, qnom[i], p, R, lead=C.BLADE_OFF, w_ax=wax, w_reg=1.0, iters=1)
            self.goal[i] = q

    def xy_accel(self, t):
        d = self.d; com = d.subtree_com[self.torso]
        p, v, a = self.xyplan.at(t)
        e = p - com[:2]
        acc = a + 3.0 * e + 3.5 * (v - d.qvel[0:2])
        n = np.linalg.norm(acc)
        if n > 1.5: acc *= 1.5 / n
        self.axy = acc
        self.M["exy"] = max(self.M["exy"], float(np.linalg.norm(e))) if self.phase in ("roll", "inverted", "hover", "edf") else self.M["exy"]

    # ---- per control tick
    def control(self, t, dt):
        d = self.d
        self.sense()
        self.bat.step(self.edf, self.fan, self.load, dt, k_claw=B4.PROP_P["K"])
        self.fmax_eff = PW.claw_max(B4.PROP, self.bat.V); self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())
        th, thd = roll_theta(t)
        self.R_ref = Rax(th); self.w_ref = U * thd
        self.xy_accel(t)
        if self.phase in ("land", "stand", "walk", "done"):
            self.land_or_walk(t, dt)
        elif t < T_EDF0:
            self.phase = "hover"; self.label = "HOVER upright: 8 claw-props + EDF"
            self.gimbal(t, range(8), self.qfly)
        elif t < T_R0:
            self.phase = "edf"; self.label = "EDF spools down: claw-props take the weight"
            self.edf_target = 34.0 * (1 - mj((t - T_EDF0) / T_EDF))
            self.gimbal(t, range(8), self.qfly)
        elif t < T_R0 + T_R + 0.6:
            s = roll_s(t); self.phase = "roll"
            self.winch_target = C.T_LINE["tight"] if s < 0.98 else C.T_LINE["firm"]   # clutches must not slip under 25+ N of thrust
            self.label = f"FLIP: body rolls {np.degrees(th):3.0f} deg, arms keep the claw-props level"
            self.edf_target = 0.0
            qp = path_at(s)
            self.gate[:] = 1.0                       # arms 5-7 thrust as long as their claw axes stay within 40 deg
            for i in UNDER: self.goal[i] = qp[i]
            self.gimbal(t, FLYERS, qp)
            if s < 0.45: self.gimbal(t, UNDER, qp, wax=30.0 * (1 - s / 0.45))
            elif s >= S_REJOIN: self.gimbal(t, UNDER, qp, wax=30.0 * min(1.0, (s - S_REJOIN) / 0.15))
        elif t < T_LAND0:
            self.phase = "inverted"
            if t < T_DES0:
                self.label = "INVERTED HOVER: dome down, stand pose = flight pose"
            else:
                if self.zplan.done(t) and not getattr(self, "des_set", False):
                    z = d.xpos[self.torso][2]
                    self.zplan = B4.Quintic(t, T_DES, z, H_INV + 0.03, d.qvel[2], 0.0); self.des_set = True
                self.label = "INVERTED: descend"
            self.gimbal(t, range(8), np.tile(Q_STAND, (8, 1)))
        else:
            self.land_or_walk(t, dt)
        if self.phase == "walk" and GOVERNOR:
            self.reflexes(dt)
        self.spine(dt)
        d.ctrl[:] = self.q_cmd.ravel()
        self.T_line += (self.winch_target - self.T_line) * min(1, dt / 0.15)

    def land_or_walk(self, t, dt):
        d = self.d
        p = d.xpos[self.torso].copy(); R = d.xmat[self.torso].reshape(3, 3)
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        if self.phase == "inverted":
            self.phase = "land"; self.q_sw0 = self.goal.copy(); self.t_l = t
            self.yaw_l = yaw; self.xy_l = d.subtree_com[self.torso][:2].copy(); self.p_l = p.copy()
            Rz = B4.Rz(yaw)
            for i in range(8):
                self.spots[i] = np.array([p[0], p[1], 0.0]) + Rz @ np.array([F_INV[i, 0], F_INV[i, 1], 0.0])
        if self.phase == "land":
            zr = self.zplan.at(t)[0]
            bp = np.array([self.p_l[0], self.p_l[1], zr]); bR = B4.Rz(self.yaw_l) @ RX     # planted legs hold the planned body pose
            flying = []
            for k, (a, b) in enumerate(PAIRS):
                for i in (a, b):
                    u = (t - self.t_l - k * T_PAIR) / T_SW
                    if u <= 0:
                        flying.append(i); continue
                    self.gate[i] = 0.0
                    u = min(u, 1.0); self.u_sw[i] = u
                    qp = self.q_sw0[i] + (Q_INV[i] - self.q_sw0[i]) * mj(u / 0.85)
                    if u >= 0.6:
                        if not getattr(self, "ik_on", np.zeros(8, bool))[i]:
                            if not hasattr(self, "ik_on"): self.ik_on = np.zeros(8, bool)
                            self.ik_on[i] = True; self.q_ik[i] = qp.copy()
                        tgt = self.spots[i] + np.array([0, 0, 0.05 * (1 - mj((u - 0.6) / 0.4)) - 0.006])
                        self.q_ik[i] = B4.ik(i, tgt, B4.DOWN, self.q_ik[i], Q_INV[i], bp, bR)
                        w = mj((u - 0.6) / 0.3)
                        self.goal[i] = qp * (1 - w) + self.q_ik[i] * w
                    else:
                        self.goal[i] = qp
                    if not self.stance[i] and (self.foot_f[i] > B4.FOOT_N or u >= 1.0) and u > 0.7:
                        self.stance[i] = True
            if flying: self.gimbal(t, flying, np.tile(Q_STAND, (8, 1)))
            n_dn = int(self.stance.sum())
            self.label = f"INVERTED LANDING: leg pairs swing down ({n_dn} feet planted)"
            if n_dn == 8 and not hasattr(self, "t_all"):
                self.t_all = t
            if hasattr(self, "t_all"):
                e = t - self.t_all
                self.prop_scale = 1 - mj(e / 1.0)
                self.winch_target = C.T_LINE["firm"] + (C.T_LINE["tight"] - C.T_LINE["firm"]) * mj(e / 0.8)
                self.label = "INVERTED: weight onto the feet, claw-props spin down"
                if e >= 1.1:
                    self.fans_on = False; self.fan[:] = 0
                    self.phase = "stand"; self.t_st = t
                    self.bp = p.copy(); self.byaw = yaw; self.feet = self.measured_feet(); self.gz = float(np.median(self.feet[:, 2]))
                    self.feet[:, 2] = self.gz; self.q_ik = self.goal.copy(); self.swing_foot = {}; self.z_st = p[2]
            return
        if self.phase == "stand":
            p = d.xpos[self.torso].copy()
            e = t - self.t_st
            self.bp[2] = self.z_st + (self.gz + H_INV - self.z_st) * mj(e / 0.8)
            self.ground_ik(range(8))
            self.label = "INVERTED STAND: dome down, knees up"
            if e >= 1.4:
                self.phase = "walk"
                B4.F_STAND = F_INV                          # body4 Walk places feet from the inverted stand
                self.walk = B4.Walk(*WALK); self.walk.start(self, t)
            return
        if self.phase == "walk":
            was = {i for i, v in self.swing_foot.items() if v is not None}
            self.walk.step(self, t, B4.CTRL_DT)
            # touchdown latch: a foot that just landed stays where it actually touched (the IK no longer drags it toward
            # the planned spot, which squeezed opposite legs against each other through toe friction)
            now = {i for i, v in self.swing_foot.items() if v is not None}
            fm = None
            for i in was - now:
                if fm is None: fm = self.measured_feet()
                self.feet[i][:2] = fm[i][:2]
            if was - now: self.ground_ik(list(was - now))
            self.label = "WALK UPSIDE DOWN: " + self.walk.label(self)[6:]
            if self.walk.done:
                self.phase = "done"; self.anchor_ground(t); self.t_done = t
            return
        if self.phase == "done":
            self.ground_ik(range(8)); self.label = "INVERTED STAND"

    # ---- thrust (every physics step): claw-props + EDF, attitude to the reference (roll), altitude
    def flip_thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; com = d.subtree_com[self.torso].copy()
        wW = Rb @ d.qvel[3:6]
        self.edf += (min(self.edf_target, self.edf_cap) - self.edf) * min(1, B4.DT / 0.08); edf = self.edf
        d.xfrc_applied[:] = 0
        ez = Rb[:, 2]
        if edf > 1e-3:
            d.xfrc_applied[self.torso, :3] += edf * ez
            d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)
        if not self.fans_on:
            self.sat = 0
            return
        Re = self.R_ref.T @ Rb
        eR = 0.5 * vee(Re - Re.T)                                   # body frame
        eW = Rb @ eR
        self.att_err = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))))
        cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
        w_ax = np.clip((cz - np.cos(40 * D)) / (np.cos(25 * D) - np.cos(40 * D)), 0, 1)
        self.gate_s += (self.gate - self.gate_s) * min(1, B4.DT / 0.12)
        wgt = w_ax * self.gate_s
        for i in range(8):
            if wgt[i] > 1e-3:
                if self.t_in[i] is None: self.t_in[i] = t
                wgt[i] *= mj((t - self.t_in[i]) / 0.3)
            else:
                self.t_in[i] = None
        on = [i for i in range(8) if wgt[i] > 1e-3]
        cmd = np.zeros(8)
        if on:
            sat = self.sat > 0
            if not sat: self.iatt = np.clip(self.iatt + 6.0 * eW * B4.DT, -2.5, 2.5)
            u = roll_s(t)
            thdd = np.pi * (60 * u - 180 * u * u + 120 * u ** 3) / T_R ** 2 if 0 < u < 1 else 0.0
            Mw = -40.0 * eW - 7.0 * (wW - self.w_ref) - self.iatt + I_ROLL * thdd * U
            self.Mz_dem = float(Mw[2])
            zr, vzr, azr = self.zplan.at(t)
            M = C.M_ROBOT
            Fz = M * (C.G + azr + 6.0 * (zr - pb[2]) + 3.5 * (vzr - d.qvel[2])) - edf * ez[2]
            if self.phase == "land":
                Fz -= F_FOOT * self.stance.sum()
                Fz = max(Fz, 3.0 * len(on))
            Fz *= self.prop_scale
            M_edf = np.cross(pb - com, edf * ez)
            A = np.zeros((4, len(on))); fmax = self.fmax_eff * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax) - B4.PROP_P["q_per_T"] * SPIN[i] * ax
                A[:, j] = [ax[2], mo[0], mo[1], mo[2]]
            b = np.array([Fz, (Mw[0] - M_edf[0]) * self.prop_scale, (Mw[1] - M_edf[1]) * self.prop_scale, Mw[2] * self.prop_scale])
            Wr = np.array([1.0, 3.0, 3.0, 0.15])
            AA = np.vstack([Wr[:, None] * A, 0.02 * np.eye(len(on))]); bb = np.concatenate([Wr * b, np.zeros(len(on))])
            F = lsq_linear(AA, bb, bounds=(np.zeros(len(on)), np.maximum(fmax, 1e-6)), method="bvls").x
            for j, i in enumerate(on): cmd[i] = F[j]
        self.fan += (cmd - self.fan) * min(1.0, B4.DT / 0.04)
        self.sat = int(np.sum((cmd >= self.fmax_eff - 1e-6) & (wgt > 0.99)))
        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
                d.xfrc_applied[self.pod[i], 3:] -= B4.PROP_P["q_per_T"] * SPIN[i] * self.fan[i] * R[:, 0]
        v = d.qvel[0:3]; drag = -0.5 * B4.RHO * B4.CDA * np.linalg.norm(v) * v
        d.xfrc_applied[self.torso, :3] += drag

    def start_inverted(self):
        """Bench start (FLIP_START=stand): standing inverted at the walk start, props off."""
        m, d = self.m, self.d
        R = B4.Rz(np.radians(45 + float(os.environ.get("FLIP_YAW0", 0)))) @ RX; q4 = np.zeros(4); mujoco.mju_mat2Quat(q4, R.ravel())
        d.qpos[:3] = [0, 0, H_INV + 0.003]; d.qpos[3:7] = q4; d.qvel[:] = 0
        for i in range(8):
            for k in range(8):
                d.qpos[self.ja[i, k]] = Q_INV[i, k] / 2; d.qpos[self.jb[i, k]] = Q_INV[i, k] / 2
        self.q_cmd = Q_INV.copy(); self.goal = Q_INV.copy(); self.goal_prev = Q_INV.copy(); self.q_ik = Q_INV.copy(); self.qd_cmd[:] = 0
        d.ctrl[:] = Q_INV.ravel(); mujoco.mj_forward(m, d)
        self.fans_on = False; self.fan[:] = 0; self.edf = 0; self.edf_target = 0; self.T_line = self.winch_target = C.T_LINE["tight"]
        self.phase = "stand"; self.t_st = 0.0; self.stance[:] = True
        p = d.xpos[self.torso].copy(); self.bp = p.copy(); self.byaw = np.radians(45 + float(os.environ.get("FLIP_YAW0", 0))); self.feet = np.array([B4.foot(i, Q_INV[i], p, R)[0] for i in range(8)])
        self.gz = 0.0; self.feet[:, 2] = 0.0; self.swing_foot = {}; self.z_st = p[2]

    def run_flip(self):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_ctrl = -1.0; M = self.M
        while d.time < T_END:
            t = d.time
            if t - t_ctrl >= B4.CTRL_DT - 1e-9:
                self.control(t, B4.CTRL_DT); t_ctrl = t
                if self.phase == "done" and t - self.t_done > 1.0: break
            d.qfrc_applied[:] = 0
            self.clutch(self.T_line)
            self.flip_thrust(t)
            d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G
            mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all():
                print("NaN at", t); break
            if t >= nxt - 1e-9:
                Rb = d.xmat[self.torso].reshape(3, 3)
                slip = np.zeros(8)
                for (i, k), qa in self.cq.items():
                    slip[i] = max(slip[i], abs(np.degrees(d.qpos[qa])))
                ph = self.phase
                M["servo"][ph] = max(M["servo"].get(ph, 0), float(self.load.max()))
                M["slip"][ph] = max(M["slip"].get(ph, 0), float(slip.max()))
                M["sat"][ph] = M["sat"].get(ph, 0) + int(self.sat > 0)
                if ph in ("roll", "inverted", "edf", "hover"): M["att"] = max(M["att"], self.att_err)
                M["fan"] = max(M["fan"], float(self.fan.max())); M["vmin"] = min(M["vmin"], self.bat.V); M["dome"] = max(M["dome"], self.belly_f)
                up = float(np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1))))
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo_arm=self.load.max(1), slip_arm=slip,
                                   up=up, att=self.att_err, foot=self.foot_f.copy(), V=self.bat.V, I=self.bat.I, soc=self.bat.soc,
                                   fmax=self.fmax_eff, com=d.subtree_com[self.torso].copy(), label=self.label, phase=ph,
                                   stance=self.stance.copy(), sat=self.sat, dome=self.belly_f, rate=self.rate,
                                   load=self.load.copy(), qcmd=self.q_cmd.copy()))
                nxt += 1 / 30
        return frames


if __name__ == "__main__":
    t0 = time.time(); F = Flip()
    if os.environ.get("FLIP_START") == "stand": F.start_inverted()
    fr = F.run_flip()
    print(f"sim {F.d.time:.1f} s in {time.time() - t0:.0f} s wall | phase {F.phase}")
    M = F.M
    print("max attitude error (flight) %.1f deg | max xy error %.1f cm | max claw-prop %.1f N | min pack %.1f V | dome contact %.1f N" %
          (M["att"], M["exy"] * 100, M["fan"], M["vmin"], M["dome"]))
    for ph in M["servo"]:
        print(f"  {ph:9s} servo {M['servo'][ph]:.2f} x stall | slip {M['slip'][ph]:4.1f} deg | saturated frames {M['sat'][ph]}")
    keys = [k for k in fr[0] if k not in ("label", "phase")]
    np.savez_compressed("flip4.npz", **{k: np.array([f[k] for f in fr]) for k in keys}, label=np.array([f["label"] for f in fr]),
                        phase=np.array([f["phase"] for f in fr]))
    json.dump(dict(M=M, end_phase=F.phase), open("flip4.json", "w"), indent=1)
