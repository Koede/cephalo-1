"""Tentacle steering for Cephalo-1 rev 4: fly a path by leaning SOME tentacles, body level (full-robot MuJoCo).

To fly north, the arms on the trailing (south) side lean their claw-props north; the leading arms stay upright and
carry the weight.  Braking or turning hands the job to a different set of arms.  Compared with the conventional way:
tilt the whole robot like a quadcopter.

  tilt  : quadcopter-style.  Acceleration demand -> body tilt (atan(a/g), max 15 deg), roll/pitch by differential
          claw-prop thrust; arms stay in the flight pose.
  steer : tentacle steering.  Body level.  Horizontal force F_h is shared among the arms on the side opposite to
          F_h, with weight w_i = max(0, -cos(arm azimuth - force azimuth))^1.5 + 0.05 (the arm straight behind
          leans most, its neighbours less, the leading arms almost not at all).  Each arm's claw-axis tilt
          (up to 40 deg) is reached by one damped Gauss-Newton step per 10 ms on the claw-axis Jacobian of the
          CURRENT arm shape, spread over j2..j7 with tip-heavy weights; the joints follow with tip-first lags
          (0.06 / 0.12 / 0.20 s), so a lean starts at the claw and curls down the arm.  The null-space breathing
          wave of flight4.py runs on top.  Heading held by tangential lean of all arms.  Vectoring claw-props are
          faded out only beyond 42-48 deg from vertical.
Path (heading held, north = +y): hover 1 s -> 2.5 m north (3.0 s) -> hold 1 s -> 2.5 m east (3.0 s) -> hold 1 s ->
home, 3.54 m south-west (3.5 s) -> hold.  Minimum-jerk reference with velocity/acceleration feed-forward.
Air drag on the robot: 0.5 rho CdA |v| v, CdA = 0.10 m^2 (est.).
Run: python3 steer4.py tilt|steer [mid|pess] -> steer4_<mode>_<case>.npz/.json
"""
import os, sys, json, time, numpy as np, mujoco
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4
import flight4 as FL

D = C.DEG
MODE = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_STEER", "steer")
CASE = sys.argv[2] if (__name__ == "__main__" and len(sys.argv) > 2) else os.environ.get("CEPH_CASE", "mid")
F_MAX = C.FAN_MAX[CASE]
CDA, RHO = 0.10, 1.2
TILT_MAX = 15 * D                 # tilt mode: max body tilt
LEAN_MAX = 40 * D                 # steer mode: max claw-axis lean
KP, KD, KI, A_MAX = 4.0, 3.6, 1.0, 3.0
# legs: (start time, duration, target xy)
LEGS = [(1.0, 3.0, (0.0, 2.5), "FLY NORTH 2.5 m"), (5.0, 3.0, (2.5, 2.5), "FLY EAST 2.5 m"),
        (9.0, 3.5, (0.0, 0.0), "RETURN HOME (south-west, 3.5 m)")]
T_END = 15.0


def mj5(u):
    u = np.clip(u, 0, 1); return u ** 3 * (10 - 15 * u + 6 * u * u)


def reference(t):
    """Position, velocity, acceleration of the path (minimum jerk per leg)."""
    p = np.zeros(2); v = np.zeros(2); a = np.zeros(2); prev = np.zeros(2)
    for t0, T, tgt, _ in LEGS:
        tgt = np.array(tgt, float); dlt = tgt - prev
        u = (t - t0) / T
        if u >= 1:
            p = tgt.copy()
        elif u > 0:
            p = prev + dlt * mj5(u)
            v = dlt * 30 * u * u * (1 - u) ** 2 / T
            a = dlt * 60 * u * (1 - u) * (1 - 2 * u) / T ** 2
            break
        else:
            break
        prev = tgt
    return p, v, a


def leg_label(t):
    for t0, T, _, lab in LEGS:
        if t0 <= t < t0 + T + 0.3:
            return lab
    return "HOLD / HOVER"


class Steer(FL.Flight):
    def __init__(self, mode=MODE):
        super().__init__("organic" if mode == "steer" else "baseline")
        self.smode = mode
        self.Wd = np.diag(FL.W_J)
        self.I = np.zeros(2); self.lean = np.zeros(8); self.wts = np.zeros(8)
        self.q_geo = np.tile(FL.Q_FLY, (8, 1))          # shape targets before the tip-first lag
        self.adem = np.zeros(2)
        self.gate = np.ones(8)                          # per-arm claw-prop permission (1 = flying arm)
        self.flare_amp = 0.0                            # null-space hub flare (m, + = out); set by subclasses
        self.breath = 1.0                               # breathing wave amplitude (0..1)
        self.lean_max, self.a_max = LEAN_MAX, A_MAX
        self.att_ki, self.iatt = 0.0, np.zeros(2)       # roll/pitch integral (N m per rad s); off by default, used with payloads

    def ref2(self, t):
        return reference(t)

    def zref(self, t):
        return FL.Z0, 0.0, 0.0

    def edf_at(self, t):
        return FL.EDF_HOVER

    def pos_xy(self):
        return self.d.subtree_com[self.torso][:2]

    def arm_override(self, i, t):
        return None

    def demand(self, t, dt):
        d = self.d
        com = self.pos_xy()
        p, v, a = self.ref2(t)
        e = p - com[:2]
        self.I = np.clip(self.I + KI * e * dt, -1.0, 1.0)
        vel = d.qvel[0:2]
        acc = a + KP * e + KD * (v - vel) + self.I
        acc = acc + 0.5 * RHO * CDA * np.linalg.norm(vel) * vel / C.M_ROBOT      # drag feed-forward
        n = np.linalg.norm(acc)
        if n > self.a_max: acc *= self.a_max / n
        self.adem = acc
        return acc

    def axis_jac(self, i, q):
        R0, p0 = C.arm_fk(i, q)[1]
        a0 = R0[:, 0]; h0 = p0 + a0 * C.BLADE_OFF
        J = np.zeros((3, 8)); Jp = np.zeros((3, 8))
        for k in range(2, 8):
            dq = q.copy(); dq[k] += 1e-4
            R1, p1 = C.arm_fk(i, dq)[1]
            J[:, k] = (R1[:, 0] - a0) / 1e-4; Jp[:, k] = (p1 + R1[:, 0] * C.BLADE_OFF - h0) / 1e-4
        self._Jp, self._h0 = Jp, h0
        return a0, J

    def arms(self, t, dt):
        d = self.d
        Rb = d.xmat[self.torso].reshape(3, 3); com = d.subtree_com[self.torso]
        acc = self.demand(t, dt)
        Fh = C.M_ROBOT * acc
        wW = Rb @ d.qvel[3:6]
        Mz = float(np.clip(-FL.KP_PSI * self.yaw_of(Rb) - FL.KD_PSI * wW[2], -FL.MZ_MAX, FL.MZ_MAX))
        hub = np.array([d.xpos[self.pod[i]] + d.xmat[self.pod[i]].reshape(3, 3)[:, 0] * C.BLADE_OFF for i in range(8)])
        r = hub[:, :2] - com[:2]; rn = np.linalg.norm(r, axis=1); rh = r / rn[:, None]
        tan = np.stack([-rh[:, 1], rh[:, 0]], 1)
        f = np.maximum(self.fan, 2.0)
        Fn = np.linalg.norm(Fh)
        u = Fh / Fn if Fn > 1e-6 else np.zeros(2)
        w = (np.clip(-(rh @ u), 0, 1) ** 1.5 + 0.05) * (self.gate > 0.5)
        self.wts = w
        lean = (w / (f * w).sum())[:, None] * Fh[None, :]                    # sin(lean) vector per arm
        lean = lean + (tan * Mz / (f * rn * (self.gate > 0.5)).sum()) * (self.gate > 0.5)[:, None]
        ln = np.linalg.norm(lean, axis=1)
        lean *= (np.minimum(ln, np.sin(self.lean_max)) / np.maximum(ln, 1e-9))[:, None]
        self.lean = np.degrees(np.arcsin(np.minimum(np.linalg.norm(lean, axis=1), 1)))
        ph = 2 * np.pi * t / FL.BREATH_T
        for i in range(8):
            lb = Rb.T @ np.array([lean[i, 0], lean[i, 1], 0.0])
            s = np.linalg.norm(lb[:2])
            a_des = np.array([lb[0], lb[1], np.sqrt(max(1 - s * s, 0))])
            q = self.q_geo[i]
            a0, J = self.axis_jac(i, q)
            err = a_des - a0
            JW = J @ self.Wd
            step = self.Wd @ J.T @ np.linalg.solve(JW @ J.T + 1e-3 * np.eye(3), err)
            br = self.breath * FL.BREATH_AMP * np.sin(ph - 2 * np.pi * i / 8 - 0.7 * np.arange(8))
            Pn = self.Wd @ J.T @ np.linalg.pinv(JW @ J.T, rcond=1e-4)
            Nn = np.eye(8) - Pn @ J
            nb = Nn @ br
            fa = float(np.broadcast_to(self.flare_amp, (8,))[i])
            if abs(fa) > 1e-4:                      # move the hub in/out along the arm azimuth, axes unchanged
                rb = np.array([np.cos(np.radians(22.5 + 45 * i)), np.sin(np.radians(22.5 + 45 * i)), 0.0])
                jr = rb @ self._Jp @ Nn @ self.Wd
                nb = nb + Nn @ self.Wd @ jr * (fa / max(jr @ jr, 1e-9))
            self.q_geo[i] = np.clip(q + np.clip(step, -0.15, 0.15), C.LIM[:, 0], C.LIM[:, 1])
            tgt = self.q_geo[i] + nb
            ov = self.arm_override(i, t)
            if ov is not None:
                tgt = ov; self.q_geo[i] = ov
            self.dq[i] += (tgt - FL.Q_FLY - self.dq[i]) * np.minimum(1, dt / FL.TAU_J)
        q_new = np.tile(FL.Q_FLY, (8, 1)) + self.dq
        stp = FL.V_ARM * dt
        self.q_cmd = np.clip(self.q_cmd + np.clip(q_new - self.q_cmd, -stp, stp), C.LIM[:, 0], C.LIM[:, 1])
        return self.q_cmd

    def thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; w = d.qvel[3:6]
        com = d.subtree_com[self.torso].copy()
        edf = self.edf_at(t); self.edf = edf
        ez = Rb[:, 2]; roll_err = -ez[1]; pitch_err = ez[0]
        zr, vzr, azr = self.zref(t)
        if self.smode == "tilt":
            acc = self.demand(t, S4.DT)
            th = np.arctan2(acc, C.G)
            n = np.linalg.norm(th)
            if n > TILT_MAX: th *= TILT_MAX / n
            tx, ty = th
        else:
            tx = ty = 0.0
        F_need = (C.M_ROBOT + self.payload_m()) * (C.G + azr + 4.0 * (zr - pb[2]) + 3.5 * (vzr - d.qvel[2])) / max(ez[2], 0.7) - edf * ez[2]
        wW = Rb @ w
        kp, kd = 14.0, 4.5
        Mx = -kp * (roll_err + ty) - kd * wW[0]
        My = -kp * (pitch_err - tx) - kd * wW[1]
        if self.att_ki > 0:               # trims out the steady tilt from an off-centre payload (frozen while a fan is saturated)
            if getattr(self, 'sat', 0) == 0:
                self.iatt = np.clip(self.iatt + self.att_ki * np.array([roll_err + ty, pitch_err - tx]) * S4.DT, -2.5, 2.5)
            Mx -= self.iatt[0]; My -= self.iatt[1]
        M_edf = np.cross(pb - com, edf * ez)
        cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
        if self.smode == "tilt":
            wgt = (cz > np.cos(25 * D)).astype(float)
        else:
            wgt = np.clip((cz - np.cos(48 * D)) / (np.cos(42 * D) - np.cos(48 * D)), 0, 1)
        wgt = wgt * self.gate
        on = [i for i in range(8) if wgt[i] > 1e-3]
        cmd = np.zeros(8)
        A = np.zeros((3, len(on))); fmax = F_MAX * wgt[on]
        for j, i in enumerate(on):
            ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
            mo = np.cross(r, ax); A[:, j] = [ax[2], mo[0], mo[1]]
        F = self.allocate(A, np.array([F_need, Mx - M_edf[0], My - M_edf[1]]), wgt[on], fmax)
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
        v = d.qvel[0:3]
        drag = -0.5 * RHO * CDA * np.linalg.norm(v) * v
        d.xfrc_applied[self.torso, :3] += drag
        d.xfrc_applied[self.torso, 3:] += np.cross(com - d.xipos[self.torso], drag)
        self.gust = 0.0; self.yawkick = 0.0

    def payload_m(self):
        return 0.0

    def impactor(self, t):
        self.d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_arm = -1.0
        st = dict(err=0.0, err_rms=0.0, tilt=0.0, yaw=0.0, servo=0.0, slip=0.0, sat=0, lean=0.0, vmax=0.0, fan=0.0, final=0.0)
        n_e = 0; rms = np.zeros((8, C.NJ)); nrm = 0; t0 = time.time()
        while d.time < T_END:
            t = d.time
            if t - t_arm >= 0.01 - 1e-9:
                if self.smode == "steer":
                    d.ctrl[:] = self.arms(t, 0.01).ravel()
                t_arm = t
            d.qfrc_applied[:] = 0
            self.clutch(C.T_LINE["firm"])
            self.thrust(t)
            self.impactor(t)
            mujoco.mj_step(m, d)
            frc = np.abs(d.actuator_force.reshape(8, C.NJ)); servo = frc / C.STALL
            rms += (frc / C.RATED) ** 2; nrm += 1
            slip = max(abs(d.qpos[qa]) for qa in self.cq.values())
            Rb = d.xmat[self.torso].reshape(3, 3)
            tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1))); yaw = np.degrees(self.yaw_of(Rb))
            com = d.subtree_com[self.torso].copy()
            p, v, a = self.ref2(t)
            err = float(np.linalg.norm(p - com[:2])) * 100
            cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
            dax = np.degrees(np.arccos(np.clip(cz, -1, 1)))
            if t > 0.5:
                st["err"] = max(st["err"], err); st["err_rms"] += err ** 2; n_e += 1
                st["tilt"] = max(st["tilt"], tilt); st["yaw"] = max(st["yaw"], abs(yaw))
                st["servo"] = max(st["servo"], float(servo.max())); st["slip"] = max(st["slip"], float(np.degrees(slip)))
                st["sat"] = max(st["sat"], self.sat); st["lean"] = max(st["lean"], float(dax.max()))
                st["vmax"] = max(st["vmax"], float(np.linalg.norm(d.qvel[0:2]))); st["fan"] = max(st["fan"], float(self.fan.max()))
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo=servo.max(),
                                   servo_arm=servo.max(1), tilt=tilt, yaw=yaw, err=err, com=com, ref=p.copy(),
                                   vel=d.qvel[0:2].copy(), dax=dax, wts=self.wts.copy(), adem=self.adem.copy(), sat=self.sat))
                nxt += 1 / fps
        st["err_rms"] = float(np.sqrt(st["err_rms"] / max(n_e, 1)))
        st["final"] = err
        st["rms_rated"] = float(np.sqrt(rms / nrm).max())
        print(f"{self.smode} {CASE}: sim {T_END:.0f} s in {time.time() - t0:.0f} s wall")
        return frames, st


if __name__ == "__main__":
    S = Steer(MODE)
    frames, st = S.run()
    print(" | ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in st.items()))
    keys = ["t", "qpos", "fan", "edf", "servo", "servo_arm", "tilt", "yaw", "err", "com", "ref", "vel", "dax", "wts", "adem", "sat"]
    np.savez_compressed(f"steer4_{MODE}_{CASE}.npz", **{k: np.array([f[k] for f in frames]) for k in keys})
    json.dump(dict(mode=MODE, case=CASE, stats=st, fan_max=F_MAX), open(f"steer4_{MODE}_{CASE}.json", "w"), indent=1)
