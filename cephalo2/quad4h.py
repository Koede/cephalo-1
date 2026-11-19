"""Quad mode: Cephalo-1 flies on 4 claw-props (arms 0, 2, 4, 6 in a near-vertical 'candle' pose) + the EDF while the
other 4 arms work: arms 1 + 3 dig with a shovel, arms 5 + 7 wash a pan with a brush, both at the same time.
Built on body4 (same robot model, power model with battery sag, prop reaction torque, spinal layer, thrust law).
Requires the claw-prop overhaul (CEPH_PROP=xl: 7.5 in Snap-Digit on a 28xx motor) and 2 packs (CEPH_PACKS=2).

Tools are inertial free bodies held by spring-damper grasp ties (as chores4).  Shovel 0.40 kg, 0.80 m; it digs into a
sand box (loose sand: up to 8 N vertical and 6 N drag on the blade, est.), lifts a 0.20 kg scoop, swings it aside and tips it.
Pan 0.25 kg held by its handle; brush 0.05 kg pressed on it with 2.5 N and moved in circles (1.2 N friction).
Controller gains are trained generation by generation (CMA-ES) from a hand ansatz (steer4/chores4 values).

python3 quad4.py run [gains.json]     one episode, saves quad4_<tag>.npz/.json
python3 quad4.py train <gens> <pop>   CMA-ES; quad4_train.json (best gains, history)
"""
import os, sys, json, time
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "2")
import numpy as np, mujoco
import cephalo4 as C
import sim4 as S4
import body4h as B4
import power4 as PW

D = C.DEG
FLY, WORK = [0, 2, 4, 6], [1, 3, 5, 7]
QP = json.load(open("quad_pose.json")); Q_CANDLE = np.radians(QP["q_candle"])
Z0 = 0.80
T_EP = float(os.environ.get("QUAD_T", "13.0"))
SOC = float(os.environ.get("CEPH_SOC", "1.0"))
HAND = C.foot_drop()                                     # hand point: the folded fingertips, as the foot point
SAND_Z, SAND_C, SAND_HALF = 0.24, None, 0.25
M_SCOOP = 0.20
# tools: mass, inertia, tie gains (as chores4)
TOOLS = dict(shovel=dict(m=0.40, I=(0.0005, 0.024, 0.024)), pan=dict(m=0.25, I=(0.0012, 0.0012, 0.0022)),
             brush=dict(m=0.05, I=(0.00017, 0.00017, 0.00004)))
KP_T, CP_T, KR_T, CR_T = 900.0, 25.0, 6.0, 0.15
S4.EXTRA_XML = "".join(f'<body name="{n}" pos="0 0 -6"><freejoint name="{n}j"/><inertial pos="0 0 0" mass="{o["m"]}" '
                       f'diaginertia="{o["I"][0]} {o["I"][1]} {o["I"][2]}"/></body>' for n, o in TOOLS.items())


def az(a):
    return np.array([np.cos(np.radians(a)), np.sin(np.radians(a)), 0.0])


def mj5(u):
    u = np.clip(u, 0, 1); return u ** 3 * (10 - 15 * u + 6 * u * u)


def kf(t, keys):
    if t <= keys[0][0]: return np.asarray(keys[0][1], float)
    for (t0, v0), (t1, v1) in zip(keys[:-1], keys[1:]):
        if t <= t1:
            return np.asarray(v0, float) + (np.asarray(v1, float) - np.asarray(v0, float)) * mj5((t - t0) / (t1 - t0))
    return np.asarray(keys[-1][1], float)


# ---------------------------------------------------------------- task geometry (body frame, origin plate centre)
G_UP0 = 0.17 * az(145) + np.array([0, 0, -0.15])         # hand 3: upper grip
G_LO0 = 0.21 * az(75) + np.array([0, 0, -0.33])          # hand 1: lower grip
S_UP, S_LO, S_TIP = 0.0, float(np.linalg.norm(G_LO0 - G_UP0)), float(np.linalg.norm(G_LO0 - G_UP0)) + 0.38
P_GRIP0 = 0.21 * az(262) + np.array([0, 0, -0.34])       # hand 5: pan handle
PAN_DIR = az(300)
PAN_C0 = P_GRIP0 + 0.10 * PAN_DIR                         # pan centre, held 0.10 m from the grip (short handle)


def dig(t):
    """Shovel offset (body frame) through a dig cycle of 4.2 s: down into the sand, drag out, lift, swing aside, tip."""
    c = (t - 1.0) % 4.2 if t > 1.0 else 0.0
    n = int((t - 1.0) // 4.2) if t > 1.0 else -1
    down = np.array([0, 0, -0.08]); out = 0.05 * az(95); up = np.array([0, 0, 0.06]); side = 0.08 * az(185)
    keys = [(0.0, np.zeros(3)), (1.2, down), (2.0, down + out), (2.9, up + out), (3.5, up + side), (4.2, np.zeros(3))]
    scoop = n >= 0 and 2.0 <= c < 3.6
    return kf(c, keys), scoop, c


def scrub(t):
    s = max(t - 0.8, 0.0)
    a = 2 * np.pi * 1.2 * s
    amp = 0.035 * mj5(s / 0.6)
    return amp * (np.cos(a) * PAN_DIR + np.sin(a) * np.cross([0, 0, 1], PAN_DIR)), a


class Quad(B4.Body):
    def __init__(self):
        super().__init__([], start="stand")
        m, d = self.m, self.d
        self.bat = PW.Battery(B4.PACKS, SOC)
        for i in range(8):                      # prop discs only exist for the impactor tests; worker claws are folded
            g = m.geom(f"prop{i}").id; m.geom_contype[g] = 0; m.geom_conaffinity[g] = 0
        self.tid = {n: m.body(n).id for n in TOOLS}
        self.tq = {n: m.joint(f"{n}j").qposadr[0] for n in TOOLS}
        self.td = {n: m.joint(f"{n}j").dofadr[0] for n in TOOLS}
        # start: hovering in quad mode, tools in hand
        d.qpos[:] = 0; d.qvel[:] = 0; d.qpos[3] = 1.0; d.qpos[2] = Z0; d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]; d.qpos[self.imp_q + 3] = 1
        self.q_base = np.tile(Q_FLY_ := B4.Q_FLY, (8, 1))
        q = np.tile(Q_CANDLE, (8, 1))
        self.targets0 = self.hand_targets(0.0)[0]
        for i in WORK:
            qi = np.radians([-40, 0, 40, 0, 20, 0, 0, 0])
            for _ in range(12):
                qi = B4.ik(i, self.targets0[i] + [0, 0, Z0], None, qi, qi, np.array([0, 0, Z0]), np.eye(3))
            q[i] = qi
        for i in range(8):
            for k in range(8):
                d.qpos[self.ja[i, k]] = q[i, k] / 2; d.qpos[self.jb[i, k]] = q[i, k] / 2
        self.q_base = np.tile(Q_CANDLE, (8, 1))
        self.q_cmd = q.copy(); self.goal = q.copy(); self.goal_prev = q.copy(); self.q_ik = q.copy(); self.q_geo = q.copy(); self.dq_air = q - self.q_base
        d.ctrl[:] = q.ravel()
        mujoco.mj_forward(m, d)
        # tools placed so their grip points sit in the hands
        self.place_tools()
        mujoco.mj_forward(m, d)
        self.payload = sum(o["m"] for o in TOOLS.values())
        hover = (C.W_ROBOT + self.payload * C.G - 34.0) / 4
        self.fan = np.zeros(8); self.fan[FLY] = hover
        self.edf = 34.0; self.edf_target = C.EDF_MAX; self.edf_cap = C.EDF_MAX
        self.fans_on = True; self.air = 1.0; self.airborne = True; self.steering = True; self.steer_w = 1.0; self.t_steer = -10
        self.gate = np.zeros(8); self.gate[FLY] = 1; self.gate_s = self.gate.copy()
        self.t_in = [(-1.0 if i in FLY else None) for i in range(8)]
        p = d.subtree_com[self.torso][:2].copy()
        self.xyplan = B4.Quintic(0, 1e-3, p, p); self.zplan = B4.Quintic(0, 1e-3, Z0, Z0); self.yawplan = B4.Quintic(0, 1e-3, 0.0, 0.0)
        self.T_line = C.T_LINE["firm"]; self.winch_target = self.T_line
        self.g.update(lean_max=np.radians(25), a_max=1.2, ki_att=12.0)          # chores4 ansatz
        self.scoop_on = False; self.metrics = dict(exy=[], ez=[], tilt=[], yaw=[], sat=0, n=0, servo=0.0, slip=0.0, tie=0.0, vmin=99.0)

    # ---- task targets (body frame offsets from the plate centre)
    def hand_targets(self, t):
        off, scoop, c = dig(t)
        gu, gl = G_UP0 + off, G_LO0 + off
        sc, a = scrub(t)
        T = {3: gu, 1: gl, 5: P_GRIP0.copy(), 7: PAN_C0 + sc + np.array([0, 0, 0.045])}
        return T, scoop, a

    def body_ref(self):
        p = np.array([self.xyplan.at(0)[0][0], self.xyplan.at(0)[0][1], Z0])
        return p

    def place_tools(self):
        d = self.d
        P0 = np.array([self.d.qpos[0], self.d.qpos[1], Z0])
        T = self.targets0
        gu, gl = T[3] + P0, T[1] + P0
        ex = (gl - gu) / np.linalg.norm(gl - gu)
        ey = np.cross([0, 0, 1], ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
        self.R_sh = np.stack([ex, ey, ez], 1)
        self.set_body("shovel", gu + ex * (S_LO / 2), self.R_sh)
        self.off = dict(shovel={3: np.array([-S_LO / 2, 0, 0]), 1: np.array([S_LO / 2, 0, 0])})
        pc = PAN_C0 + P0
        ey = np.cross([0, 0, 1], PAN_DIR)
        self.set_body("pan", pc, np.stack([PAN_DIR, ey, [0, 0, 1]], 1))
        self.off["pan"] = {5: np.array([-0.10, 0, 0])}
        self.set_body("brush", T[7] + P0, np.eye(3))
        self.off["brush"] = {7: np.zeros(3)}
        # orientation ties: each tool keeps its relative orientation to one hand
        self.rrel = {}
        for n, h in (("shovel", 3), ("pan", 5), ("brush", 7)):
            Rh = d.xmat[self.pod[h]].reshape(3, 3); Ro = self.body_R(n)
            self.rrel[n] = (h, Rh.T @ Ro)
        SAND = gl + ex * (S_TIP - S_LO)
        self.sand_xy = SAND[:2].copy(); self.sand_z = SAND[2] - 0.05         # sand surface 5 cm under the resting blade tip

    def set_body(self, n, p, R):
        qa = self.tq[n]; q = np.zeros(4); mujoco.mju_mat2Quat(q, R.ravel())
        self.d.qpos[qa:qa + 3] = p; self.d.qpos[qa + 3:qa + 7] = q

    def body_R(self, n):
        return self.d.xmat[self.tid[n]].reshape(3, 3)

    def hand_point(self, h):
        R = self.d.xmat[self.pod[h]].reshape(3, 3)
        return self.d.xpos[self.pod[h]] + R[:, 0] * HAND, R

    # ---- worker arms: hand IK on the reference body pose (body-frame task targets)
    def work_arms(self, t):
        T, scoop, _ = self.hand_targets(t)
        self.scoop_on = scoop
        p = self.d.xpos[self.torso].copy(); R = self.d.xmat[self.torso].reshape(3, 3)
        for i in WORK:
            tgt = p + R @ T[i]
            self.q_ik[i] = B4.ik(i, tgt, None, self.q_ik[i], self.q_ik[i], p, R)
            self.override[i] = self.q_ik[i]

    # ---- tool forces (every physics step)
    def tool_forces(self, t):
        m, d = self.m, self.d
        tie_max = 0.0
        for n, hs in self.off.items():
            oid = self.tid[n]; po = d.xpos[oid]; Ro = self.body_R(n)
            vo = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, oid, vo, 0)
            for h, off in hs.items():
                g, Rh = self.hand_point(h)
                vh = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.pod[h], vh, 0)
                pt = po + Ro @ off
                vg = vh[3:] + np.cross(vh[:3], g - d.xpos[self.pod[h]]); vp = vo[3:] + np.cross(vo[:3], pt - po)
                mt = TOOLS[n]["m"]; wt = 60.0                                     # translational tie ~60 rad/s, damped 0.7
                F = mt * wt * wt * (g - pt) + 2 * 0.7 * mt * wt * (vg - vp)
                d.xfrc_applied[oid, :3] += F; d.xfrc_applied[oid, 3:] += np.cross(pt - d.xipos[oid], F)
                d.xfrc_applied[self.pod[h], :3] -= F; d.xfrc_applied[self.pod[h], 3:] -= np.cross(g - d.xipos[self.pod[h]], F)
                tie_max = max(tie_max, float(np.linalg.norm(g - pt)))
            h, Rrel = self.rrel[n]
            Rh = d.xmat[self.pod[h]].reshape(3, 3)
            Re = (Rh @ Rrel) @ Ro.T
            w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)); sn = np.linalg.norm(w)
            rv = w * (ang / sn) if sn > 1e-6 else w
            vh = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.pod[h], vh, 0)
            Id = np.array(TOOLS[n]["I"]); wr = 40.0                         # rotational tie ~40 rad/s per tool axis, damped 0.7
            if n == "shovel": Id = Id * np.array([1.0, 0.0, 0.0])        # two hands fix the shaft; the claw only stops it rolling
            tau = Ro @ (Id * (wr * wr * (Ro.T @ rv) + 2 * 0.7 * wr * (Ro.T @ (vh[:3] - vo[:3]))))
            d.xfrc_applied[oid, 3:] += tau; d.xfrc_applied[self.pod[h], 3:] -= tau
        # sand on the shovel blade; the scoop rides on the blade while lifted
        oid = self.tid["shovel"]; po = d.xpos[oid]; Ro = self.body_R("shovel")
        tip = po + Ro[:, 0] * (S_TIP - S_LO / 2)
        vo = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, oid, vo, 0)
        vt = vo[3:] + np.cross(vo[:3], tip - po)
        F = np.zeros(3)
        if np.all(np.abs(tip[:2] - self.sand_xy) < SAND_HALF) and tip[2] < self.sand_z:
            F[2] = float(np.clip(1500.0 * (self.sand_z - tip[2]) - 30.0 * vt[2], 0, 8.0))      # loose dry sand (est.)
            Fh = -40.0 * vt[:2]; nh = np.linalg.norm(Fh)
            F[:2] = Fh * min(1, 6.0 / max(nh, 1e-9))
        if self.scoop_on:
            F[2] -= M_SCOOP * C.G
        d.xfrc_applied[oid, :3] += F; d.xfrc_applied[oid, 3:] += np.cross(tip - d.xipos[oid], F)
        # brush on the pan: 2.5 N press + 1.2 N friction along the scrub
        _, a = scrub(t)[0], scrub(t)[1]
        if t > 0.8:
            pid, bid = self.tid["pan"], self.tid["brush"]
            dirv = -np.sin(a) * PAN_DIR + np.cos(a) * np.cross([0, 0, 1], PAN_DIR)
            Fb = np.array([0, 0, 2.5]) - 1.2 * dirv                      # on the brush: pushed up, dragged back
            d.xfrc_applied[bid, :3] += Fb; d.xfrc_applied[pid, :3] -= Fb
        self.payload = sum(o["m"] for o in TOOLS.values()) + (M_SCOOP if self.scoop_on else 0)
        return tie_max

    def clutch_arms(self):
        """Per-arm winch: working arms tight (150 N line, clutches do not slip under the tools), flying arms firm."""
        m, d = self.m, self.d
        for (i, k), qa in self.cq.items():
            T = C.T_LINE["tight"] if i in WORK else C.T_LINE["firm"]
            tb = C.CLUTCH_FRAC[k] * C.STALL[k] * T / C.T_FIRM
            kc = tb / (0.3 * D)
            m.jnt_stiffness[self.cj_ids[i, k]] = kc
            phi = d.qpos[qa]
            if kc * abs(phi) > tb:
                d.qfrc_applied[self.cd[i, k]] += np.sign(phi) * (kc * abs(phi) - tb)

    def episode(self, gains=None, log=False):
        if gains: self.g.update(gains)
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_ctrl = -1.0; M = self.metrics
        lab = "QUAD MODE: shovel + dishes"
        while d.time < T_EP:
            t = d.time
            if t - t_ctrl >= B4.CTRL_DT - 1e-9:
                self.sense()
                self.bat.step(self.edf, self.fan * self.k_ratio, self.load, B4.CTRL_DT, k_claw=B4.PROP_P["K"])
                self.fmax_eff = PW.claw_max(B4.PROP, self.bat.V); self.fmax_arm = np.array([B4.HAND[i]["T"] * min(1.0, (self.bat.V / B4.V_REF_H) ** 1.5) if i in B4.HAND else self.fmax_eff for i in range(8)]); self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())
                self.gate = np.zeros(8); self.gate[FLY] = 1
                self.work_arms(t)
                self.fly_arms(t, B4.CTRL_DT)
                self.spine(B4.CTRL_DT)
                d.ctrl[:] = self.q_cmd.ravel()
                t_ctrl = t
            d.qfrc_applied[:] = 0
            self.clutch_arms()
            self.thrust(t)
            tie = self.tool_forces(t)
            d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G
            mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all() or d.qpos[2] < 0.25:
                M["crash"] = float(t); break
            if t >= nxt - 1e-9:
                Rb = d.xmat[self.torso].reshape(3, 3)
                tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1))); yaw = np.degrees(np.arctan2(Rb[1, 0], Rb[0, 0]))
                com = d.subtree_com[self.torso]; ref = self.xyplan.at(t)[0]
                M["exy"].append(float(np.linalg.norm(com[:2] - ref))); M["ez"].append(float(d.qpos[2] - Z0))
                M["tilt"].append(tilt); M["yaw"].append(abs(yaw)); M["sat"] += self.sat > 0; M["n"] += 1
                M["servo"] = max(M["servo"], float(self.load.max())); M["tie"] = max(M["tie"], tie); M["vmin"] = min(M["vmin"], self.bat.V)
                if log:
                    slip = np.zeros(8)
                    for (i, k), qa in self.cq.items():
                        slip[i] = max(slip[i], abs(np.degrees(d.qpos[qa])))
                    M["slip"] = max(M["slip"], float(slip.max()))
                    frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, T=self.T_line, servo_arm=self.load.max(1),
                                       slip_arm=slip, tilt=tilt, yaw=yaw, foot=self.foot_f.copy(), rate=1.0, air=1.0, label=lab,
                                       V=self.bat.V, I=self.bat.I, soc=self.bat.soc, fmax=self.fmax_eff, com=com.copy(), xyref=ref.copy(),
                                       scoop=self.scoop_on, load=self.load.copy()))
                nxt += 1 / 30
        return frames

    def fitness(self):
        M = self.metrics
        if not M["n"]: return 1e3, {}
        exy = np.array(M["exy"]) * 100; ez = np.abs(M["ez"]) * 100
        r = dict(exy_mean=float(exy.mean()), exy_max=float(exy.max()), ez_max=float(ez.max()), tilt_max=float(np.max(M["tilt"])),
                 yaw_max=float(np.max(M["yaw"])), sat_frac=M["sat"] / M["n"], servo=M["servo"], tie_mm=M["tie"] * 1000, vmin=M["vmin"],
                 crash=M.get("crash", 0.0))
        J = (r["exy_mean"] + 0.5 * r["exy_max"] + 0.3 * r["ez_max"] + 0.4 * r["tilt_max"] + 0.2 * r["yaw_max"] + 20 * r["sat_frac"]
             + 30 * max(r["servo"] - 0.9, 0) + 0.05 * max(r["tie_mm"] - 20, 0))
        if r["crash"]: J += 500 + 50 * (T_EP - r["crash"])
        r["J"] = float(J)
        return J, r


# ---------------------------------------------------------------- training
NAMES = ["kp_xy", "kd_xy", "ki_xy", "a_max", "lean_max", "kp_att", "kd_att", "ki_att", "kp_z", "kd_z", "kp_psi", "kd_psi", "tau_scale", "mz_alloc"]
ANSATZ = dict(kp_xy=4.0, kd_xy=3.6, ki_xy=1.0, a_max=1.2, lean_max=np.radians(25), kp_att=14.0, kd_att=4.5, ki_att=12.0,
              kp_z=4.0, kd_z=3.5, kp_psi=6.0, kd_psi=3.0, tau_scale=1.0, mz_alloc=0.5)


def decode(x):
    g = {}
    for n, xi in zip(NAMES, x):
        if n == "mz_alloc": g[n] = float(1 / (1 + np.exp(-(np.log(ANSATZ[n] / (1 - ANSATZ[n])) + xi))))
        else: g[n] = float(ANSATZ[n] * np.exp(0.5 * xi))
    g["lean_max"] = min(g["lean_max"], np.radians(40))
    return g


def evaluate(x):
    Q = Quad(); Q.episode(decode(x)); return Q.fitness()


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "run"
    if cmd == "run":
        g = json.load(open(sys.argv[2]))["best_gains"] if len(sys.argv) > 2 else decode(np.zeros(len(NAMES)))
        tag = os.environ.get("QUAD_TAG", "ansatz")
        t0 = time.time(); Q = Quad(); fr = Q.episode(g, log=True); J, r = Q.fitness()
        print(f"{tag}: {time.time() - t0:.0f} s wall | " + " ".join(f"{k} {v:.2f}" for k, v in r.items()) + f" | slip {Q.metrics.get('slip', 0):.1f}")
        keys = [k for k in fr[0] if k != "label"]
        np.savez_compressed(f"quad4_{tag}.npz", **{k: np.array([f[k] for f in fr]) for k in keys}, label=np.array([f["label"] for f in fr]))
        json.dump(dict(gains=g, result=r, slip=Q.metrics.get("slip", 0)), open(f"quad4_{tag}.json", "w"), indent=1)
    elif cmd == "train":
        import cma
        from multiprocessing import Pool
        gens, pop = int(sys.argv[2]), int(sys.argv[3])
        es = cma.CMAEvolutionStrategy(np.zeros(len(NAMES)), 0.5, dict(popsize=pop, seed=1, verbose=-9))
        hist = []
        J0, r0 = evaluate(np.zeros(len(NAMES))); hist.append(dict(gen=0, best=J0, mean=J0, r=r0))
        print(f"gen 0 (ansatz): J {J0:.1f} | " + " ".join(f"{k} {v:.2f}" for k, v in r0.items()), flush=True)
        best = (J0, np.zeros(len(NAMES)), r0)
        with Pool(int(os.environ.get("QUAD_PROCS", "2"))) as P:
            for gen in range(1, gens + 1):
                X = es.ask(); res = P.map(evaluate, X)
                Js = [j for j, _ in res]; es.tell(X, Js)
                k = int(np.argmin(Js))
                if Js[k] < best[0]: best = (Js[k], np.array(X[k]), res[k][1])
                hist.append(dict(gen=gen, best=float(min(Js)), mean=float(np.mean(Js)), r=res[k][1]))
                print(f"gen {gen}: best {min(Js):.1f} mean {np.mean(Js):.1f} | overall best {best[0]:.1f} | " +
                      " ".join(f"{k2} {v:.2f}" for k2, v in res[k][1].items()), flush=True)
                json.dump(dict(best_J=best[0], best_gains=decode(best[1]), best_result=best[2], history=hist), open("quad4_train.json", "w"), indent=1)
