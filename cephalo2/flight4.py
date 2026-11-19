"""Organic mid-air correction for Cephalo-1 rev 4 (hover test bench, full-robot MuJoCo model from sim4.py).

Two flight controllers on the same robot, same disturbances:
  baseline : the current mission controller (sim4): roll/pitch by differential claw-prop thrust, position held by
             tilting the whole body (max 0.10 rad), no yaw control (prop reaction torques are not modelled).
  organic  : "tentacle vectoring".  The body stays level.  Horizontal force and yaw moment come from tilting each
             claw-prop's thrust axis by up to 18 deg, and every arm makes its tilt by curling like a tentacle:
               * the axis tilt each arm needs (min-norm over the 8 claw-props: shared push + tangential twist for yaw)
                 is spread over the six outer joints j2..j7 with tip-heavy weights (weighted least squares on the
                 claw-axis Jacobian at the flight pose);
               * the tip joints answer first (tau 0.06 s) and the bend flows root-ward (j4/j5 0.12 s, j2/j3 0.20 s),
                 so a correction starts at the claw and settles into a smooth curve along the arm;
               * a slow "breathing" wave (3-4 deg, 3.2 s period) travels from root to tip and around the body; it is
                 projected into the null space of the claw-axis Jacobian, so it moves the arms but not the thrust axes.
             Roll/pitch stay on differential thrust (fast); xy and yaw go through the arms (servo-limited, 180 deg/s).
             The 25 deg world-frame fan cut-out becomes a 28 -> 32 deg fade for vectoring claw-props.
Scenario (16 s, hover at plate z = 0.80 m, EDF 80 %):
  2.0-3.5 s  gust: 8 N sideways push on the body (+x, smooth 0.2 s edges)
  5.5 s      yaw kick: swirling gust, tangential pushes on the 8 claw-props totalling 2 N m for 0.25 s
  8.5 s      side hit: 0.2 kg ball at 6 m/s into arm 2's claw-prop (tangential), as in the mission
  11.0 s     move 0.6 m along +y (minimum-jerk reference over 2.0 s)
Run: python3 flight4.py baseline|organic [mid|pess]  -> flight4_<mode>_<case>.npz/.json
"""
import os, sys, json, time, numpy as np, mujoco
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4

D = C.DEG
MODE = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_FLIGHT", "organic")
CASE = sys.argv[2] if (__name__ == "__main__" and len(sys.argv) > 2) else os.environ.get("CEPH_CASE", "mid")
F_MAX = C.FAN_MAX[CASE]
Q_FLY = S4.Q_FLY
Z0 = 0.80                      # hover height of the plate underside (m)
EDF_HOVER = 0.8 * C.EDF_MAX
T_END = 16.0
T_GUST, GUST_DUR, F_GUST = 2.0, 1.5, 8.0
T_YAW, YAW_DUR, M_YAW = 5.5, 0.25, 2.0
T_HIT = 8.5
T_MOVE, MOVE_DUR, MOVE_Y = 11.0, 2.0, 0.6
EVENTS = [("gust", T_GUST, T_YAW), ("yaw kick", T_YAW, T_HIT - 0.05), ("side hit", T_HIT - 0.05, T_MOVE), ("move 0.6 m", T_MOVE, T_END)]

# organic controller parameters
DMAX = 18 * D                                           # max claw-axis tilt per arm
W_J = np.array([0, 0, 0.35, 0.35, 0.65, 0.65, 1.0, 1.0])  # tentacle bend weights (root -> tip)
TAU_J = np.array([1, 1, 0.20, 0.20, 0.12, 0.12, 0.06, 0.06])
V_ARM = 180.0 * D                                       # airborne joint speed limit (rad/s), as take-off v7
KP_XY, KD_XY, KI_XY, A_MAX = 4.0, 3.6, 1.5, 2.2         # m/s^2 per m, per m/s, per m s; accel limit
I_MAX = 1.0                                             # integrator clamp (m/s^2): cancels a steady gust
KP_PSI, KD_PSI, MZ_MAX = 6.0, 3.0, 6.0                  # N m / rad, N m s / rad
BREATH_AMP = np.radians([0, 0, 3, 4, 3, 4, 3, 4]); BREATH_T = 3.2


def smooth(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)


def axis_jacobians():
    """Claw-axis (pod x axis, body frame) Jacobian of each arm at the flight pose; x/y rows only."""
    P, N, A0 = [], [], []
    Wd = np.diag(W_J)
    for i in range(8):
        a0 = C.arm_fk(i, Q_FLY)[1][0][:, 0]
        J = np.zeros((3, 8))
        for k in range(8):
            dq = np.zeros(8); dq[k] = 1e-5
            J[:, k] = (C.arm_fk(i, Q_FLY + dq)[1][0][:, 0] - a0) / 1e-5
        J2 = J[:2]
        Pi = Wd @ J2.T @ np.linalg.inv(J2 @ Wd @ J2.T)           # 8x2: axis-tilt request -> joint offsets
        P.append(Pi); N.append(np.eye(8) - Pi @ J2); A0.append(a0)
    return np.array(P), np.array(N), np.array(A0)


class Flight(S4.Sim):
    def __init__(self, mode=MODE):
        self.mode_ = mode
        self.m = S4.build(); self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        self.ja = np.array([[m.joint(f"q{i}_{k}a").qposadr[0] for k in range(C.NJ)] for i in range(8)])
        self.jb = np.array([[m.joint(f"q{i}_{k}b").qposadr[0] for k in range(C.NJ)] for i in range(8)])
        self.da = np.array([[m.joint(f"q{i}_{k}a").dofadr[0] for k in range(C.NJ)] for i in range(8)])
        self.cq = {}; self.cd = {}
        for i in range(8):
            for k in range(C.NJ):
                if C.CLUTCH_FRAC[k] > 0:
                    self.cq[i, k] = m.joint(f"c{i}_{k}").qposadr[0]; self.cd[i, k] = m.joint(f"c{i}_{k}").dofadr[0]
        self.cj_ids = {(i, k): m.joint(f"c{i}_{k}").id for (i, k) in self.cq}
        self.pod = [m.body(f"pod{i}").id for i in range(8)]
        self.torso = m.body("torso").id
        self.imp_q = m.joint("impj").qposadr[0]; self.imp_d = m.joint("impj").dofadr[0]
        for i in range(8):
            for k in range(C.NJ):
                d.qpos[self.ja[i, k]] = Q_FLY[k] / 2; d.qpos[self.jb[i, k]] = Q_FLY[k] / 2
        d.qpos[2] = Z0
        d.ctrl[:] = np.tile(Q_FLY, 8)
        mujoco.mj_forward(m, d)
        self.fan = np.full(8, (C.W_ROBOT - EDF_HOVER) / 8)
        self.edf = EDF_HOVER
        self.P, self.N, self.A0 = axis_jacobians()
        self.dq = np.zeros((8, 8)); self.q_cmd = np.tile(Q_FLY, (8, 1))
        self.xy0 = d.subtree_com[self.torso][:2].copy()
        self.hit_done = False; self.sat = 0
        self.delta = np.zeros((8, 2)); self.Fh = np.zeros(2); self.Mz = 0.0; self.ixy = np.zeros(2)

    # ---- references
    def ref(self, t):
        y = MOVE_Y * S4.mj((t - T_MOVE) / MOVE_DUR)
        return np.array([self.xy0[0], self.xy0[1] + y]), Z0

    @staticmethod
    def yaw_of(Rb):
        return float(np.arctan2(Rb[1, 0], Rb[0, 0]))

    # ---- organic arm targets (100 Hz)
    def arms(self, t, dt):
        d = self.d
        Rb = d.xmat[self.torso].reshape(3, 3); com = d.subtree_com[self.torso]
        ref_xy, _ = self.ref(t)
        u = (t - T_MOVE) / MOVE_DUR                                     # reference velocity / acceleration feed-forward
        if 0 < u < 1:
            v_ref = np.array([0, MOVE_Y * 30 * u * u * (1 - u) ** 2 / MOVE_DUR])
            a_ref = np.array([0, MOVE_Y * 60 * u * (1 - u) * (1 - 2 * u) / MOVE_DUR ** 2])
        else:
            v_ref = a_ref = np.zeros(2)
        e = ref_xy - com[:2]
        self.ixy = np.clip(self.ixy + KI_XY * e * dt, -I_MAX, I_MAX)
        a = a_ref + KP_XY * e + KD_XY * (v_ref - d.qvel[0:2]) + self.ixy
        n = np.linalg.norm(a)
        if n > A_MAX: a *= A_MAX / n
        Fh = C.M_ROBOT * a
        wW = Rb @ d.qvel[3:6]
        Mz = float(np.clip(-KP_PSI * self.yaw_of(Rb) - KD_PSI * wW[2], -MZ_MAX, MZ_MAX))
        hub = np.array([d.xpos[self.pod[i]] + d.xmat[self.pod[i]].reshape(3, 3)[:, 0] * C.BLADE_OFF for i in range(8)])
        r = hub[:, :2] - com[:2]; rn = np.linalg.norm(r, axis=1)
        tan = np.stack([-r[:, 1], r[:, 0]], 1) / rn[:, None]
        f = np.maximum(self.fan, 2.0)
        delta = Fh[None, :] / f.sum() + tan * Mz / (f * rn).sum()        # world-horizontal axis tilt per arm (rad)
        dn = np.linalg.norm(delta, axis=1)
        delta *= np.minimum(1, DMAX / np.maximum(dn, 1e-9))[:, None]
        self.delta, self.Fh, self.Mz = delta, Fh, Mz
        ph = 2 * np.pi * t / BREATH_T
        for i in range(8):
            db = Rb.T @ np.array([delta[i, 0], delta[i, 1], 0.0])            # body frame
            tgt = self.P[i] @ db[:2]
            br = BREATH_AMP * np.sin(ph - 2 * np.pi * i / 8 - 0.7 * np.arange(8))
            tgt = tgt + self.N[i] @ br
            self.dq[i] += (tgt - self.dq[i]) * np.minimum(1, dt / TAU_J)
        q_new = np.tile(Q_FLY, (8, 1)) + self.dq
        step = V_ARM * dt
        self.q_cmd = self.q_cmd + np.clip(q_new - self.q_cmd, -step, step)
        lo, hi = C.LIM[:, 0], C.LIM[:, 1]
        self.q_cmd = np.clip(self.q_cmd, lo, hi)
        return self.q_cmd

    # ---- thrust (every physics step)
    def thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; w = d.qvel[3:6]
        com = d.subtree_com[self.torso].copy()
        edf = EDF_HOVER; self.edf = edf
        ez = Rb[:, 2]; roll_err = -ez[1]; pitch_err = ez[0]
        ref_xy, zr = self.ref(t)
        vz = d.qvel[2]
        if self.mode_ == "baseline":
            exy = ref_xy - com[:2]; vxy = d.qvel[0:2]
            tx = np.clip(0.30 * exy[0] - 0.45 * vxy[0], -0.10, 0.10); ty = np.clip(0.30 * exy[1] - 0.45 * vxy[1], -0.10, 0.10)
        else:
            tx = ty = 0.0
        F_need = C.M_ROBOT * (C.G + 4.0 * (zr - pb[2]) - 3.5 * vz) / max(ez[2], 0.7) - edf * ez[2]
        wW = Rb @ w
        kp, kd = 14.0, 4.5
        Mx = -kp * (roll_err + ty) - kd * wW[0]
        My = -kp * (pitch_err - tx) - kd * wW[1]
        M_edf = np.cross(pb - com, edf * ez)
        cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
        if self.mode_ == "baseline":
            wgt = (cz > np.cos(25 * D)).astype(float)
        else:
            wgt = np.clip((cz - np.cos(32 * D)) / (np.cos(28 * D) - np.cos(32 * D)), 0, 1)
        on = [i for i in range(8) if wgt[i] > 1e-3]
        cmd = np.zeros(8)
        if on:
            n = len(on)
            A = np.zeros((3, n)); fmax = F_MAX * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax); A[:, j] = [ax[2], mo[0], mo[1]]
            b = np.array([F_need, Mx - M_edf[0], My - M_edf[1]])
            F = self.allocate(A, b, wgt[on], fmax)
            for j, i in enumerate(on):
                cmd[i] = F[j]
        self.fan += (cmd - self.fan) * min(1.0, S4.DT / 0.04)
        self.sat = int(np.sum(cmd >= F_MAX - 1e-6))
        d.xfrc_applied[:] = 0
        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
        d.xfrc_applied[self.torso, :3] += edf * ez
        d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)
        # disturbances
        g = smooth((t - T_GUST) / 0.2) * (1 - smooth((t - T_GUST - GUST_DUR) / 0.2))
        self.gust = F_GUST * g
        d.xfrc_applied[self.torso, 0] += self.gust
        y = M_YAW * smooth((t - T_YAW) / 0.05) * (1 - smooth((t - T_YAW - YAW_DUR) / 0.05))
        self.yawkick = y
        if y > 0:      # swirling gust: tangential push on the 8 claw-props adding up to M_YAW about the COM
            for i in range(8):
                r = d.xpos[self.pod[i]] - com; r[2] = 0; rn = np.linalg.norm(r)
                ft = y / (8 * rn) * np.array([-r[1], r[0], 0]) / rn
                d.xfrc_applied[self.pod[i], :3] += ft
                d.xfrc_applied[self.pod[i], 3:] += np.cross(d.xpos[self.pod[i]] - d.xipos[self.pod[i]], ft)

    def impactor(self, t):
        d = self.d
        if t >= T_HIT - 0.03 and not self.hit_done:
            arm = 2
            pc = d.geom(f"prop{arm}").xpos.copy()
            az = np.radians(22.5 + 45 * arm) + self.yaw_of(d.xmat[self.torso].reshape(3, 3))
            rad = np.array([np.cos(az), np.sin(az), 0]); tan = np.array([-np.sin(az), np.cos(az), 0])
            d.qpos[self.imp_q:self.imp_q + 3] = pc - tan * 0.18 + rad * 0.6 * C.CLAW_R
            d.qpos[self.imp_q + 3:self.imp_q + 7] = [1, 0, 0, 0]
            d.qvel[self.imp_d:self.imp_d + 3] = tan * 6.0; d.qvel[self.imp_d + 3:self.imp_d + 6] = 0
            self.hit_done = True
        if self.hit_done and t > T_HIT + 0.25:
            d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]; d.qvel[self.imp_d:self.imp_d + 6] = 0
        d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_arm = -1.0
        dq_prev = d.qvel.copy()
        ev = {e[0]: dict(xy=0.0, tilt=0.0, yaw=0.0, servo=0.0, slip=0.0, sat=0, dmax=0.0, end_xy=0.0, end_yaw=0.0) for e in EVENTS}
        hov = dict(xy=0.0, tilt=0.0, yaw=0.0, servo=0.0)
        rms = np.zeros((8, C.NJ)); nrm = 0
        t0 = time.time()
        while d.time < T_END:
            t = d.time
            if t - t_arm >= 0.01 - 1e-9:
                if self.mode_ == "organic":
                    d.ctrl[:] = self.arms(t, 0.01 if t_arm >= 0 else 0.01).ravel()
                t_arm = t
            d.qfrc_applied[:] = 0
            self.clutch(C.T_LINE["firm"])
            self.thrust(t)
            self.impactor(t)
            mujoco.mj_step(m, d)
            qacc = (d.qvel - dq_prev) / S4.DT; dq_prev = d.qvel.copy()
            frc = np.abs(d.actuator_force.reshape(8, C.NJ)); servo = frc / C.STALL
            rms += (frc / C.RATED) ** 2; nrm += 1
            slip = np.zeros((8, C.NJ))
            for (i, k), qa in self.cq.items():
                slip[i, k] = abs(d.qpos[qa])
            Rb = d.xmat[self.torso].reshape(3, 3)
            tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
            yaw = np.degrees(self.yaw_of(Rb))
            com = d.subtree_com[self.torso]
            ref_xy, _ = self.ref(t)
            exy = float(np.linalg.norm(ref_xy - com[:2])) * 100
            cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
            dax = np.degrees(np.arccos(np.clip(cz, -1, 1)))
            for nm, a, b in EVENTS:
                if a <= t < b:
                    e = ev[nm]
                    e["xy"] = max(e["xy"], exy); e["tilt"] = max(e["tilt"], tilt); e["yaw"] = max(e["yaw"], abs(yaw))
                    e["servo"] = max(e["servo"], float(servo.max())); e["slip"] = max(e["slip"], float(np.degrees(slip.max())))
                    e["sat"] = max(e["sat"], self.sat); e["dmax"] = max(e["dmax"], float(dax.max()))
                    e["end_xy"], e["end_yaw"] = exy, abs(yaw)
            if 0.5 <= t < T_GUST:
                hov["xy"] = max(hov["xy"], exy); hov["tilt"] = max(hov["tilt"], tilt); hov["yaw"] = max(hov["yaw"], abs(yaw))
                hov["servo"] = max(hov["servo"], float(servo.max()))
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo=servo.max(),
                                   servo_arm=servo.max(1), slip=np.degrees(slip.max()), slip_arm=np.degrees(slip.max(1)),
                                   tilt=tilt, yaw=yaw, exy=exy, com=com.copy(), ref=ref_xy.copy(), dax=dax,
                                   gust=self.gust, yawkick=self.yawkick, sat=self.sat))
                nxt += 1 / fps
        print(f"{self.mode_} {CASE}: sim {T_END:.0f} s in {time.time() - t0:.0f} s wall")
        rms = float(np.sqrt(rms / nrm).max())
        return frames, ev, hov, rms


if __name__ == "__main__":
    F = Flight(MODE)
    frames, ev, hov, rms = F.run()
    print(f"hover (0.5-2 s): xy {hov['xy']:.1f} cm, tilt {hov['tilt']:.2f} deg, yaw {hov['yaw']:.2f} deg, servo {hov['servo']:.2f}")
    for nm, e in ev.items():
        print(f"{nm:11s} xy max {e['xy']:5.1f} cm end {e['end_xy']:5.1f} | tilt {e['tilt']:4.1f} | yaw max {e['yaw']:6.1f} end {e['end_yaw']:6.1f} deg"
              f" | servo {e['servo']:.2f} | slip {e['slip']:4.1f} | sat {e['sat']} | claw axis max {e['dmax']:4.1f} deg")
    print(f"rms servo / rated over the run: {rms:.2f}")
    keys = ["t", "qpos", "fan", "edf", "servo", "servo_arm", "slip", "slip_arm", "tilt", "yaw", "exy", "com", "ref", "dax", "gust", "yawkick", "sat"]
    np.savez_compressed(f"flight4_{MODE}_{CASE}.npz", **{k: np.array([f[k] for f in frames]) for k in keys})
    json.dump(dict(mode=MODE, case=CASE, events=ev, hover=hov, rms_rated=rms, fan_max=F_MAX), open(f"flight4_{MODE}_{CASE}.json", "w"), indent=1)
