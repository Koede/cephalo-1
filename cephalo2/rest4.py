"""Cephalo-1 rev 4: lie down to rest (full-robot MuJoCo, on the ground, fans off).

From the stand (plate 300 mm up, 8 feet at r 360 mm):
  1.0-4.0 s  crouch: the feet stay where they are, the arms arch up and the body sinks until the EDF can sits on the
             floor (plate 94 mm up).  A printed TPU bumper ring on the EDF outlet is assumed (est.).
  4.0-7.6 s  lie down: each arm lifts its foot, swings it outward and is laid flat radially (spigot 45 mm above the
             floor, claw pointing outward, folded fingers resting on the floor); the wave runs around the body,
             0.12 s per arm.
  7.6 s on   winch to soft (25 N); servo holding loads are reported (they could be powered down).
Run: python3 rest4.py -> rest4.npz/.json
"""
import os, sys, json, time, numpy as np, mujoco
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4
import flight4 as FL

H_REST = 0.094
T_CROUCH, T_LIE, T_SOFT, T_END = (1.0, 4.0), (4.0, 7.6), 7.6, 9.6
LABELS = [(0, 1.0, "STAND"), (1.0, 4.0, "CROUCH: body sinks onto the EDF bumper"), (4.0, 7.6, "LIE DOWN: arms lift, swing out and roll flat"),
          (7.6, 99, "REST: winch soft, servos idle")]


def label(t):
    for a, b, lab in LABELS:
        if a <= t < b: return lab


def plan():
    feet = [C.tip_point(i, S4.Q_STAND, np.array([0, 0, S4.H_STAND]))[0] for i in range(8)]
    hs = np.linspace(S4.H_STAND, H_REST, 31)
    Qc = np.zeros((31, 8, 8)); q = np.tile(S4.Q_STAND, (8, 1))
    for k, h in enumerate(hs):
        for i in range(8):
            q[i] = C.ik(i, feet[i], np.array([0, 0, -1.0]), q[i], q[i], np.array([0, 0, h]))
            Qc[k, i] = q[i]
    err = max(np.linalg.norm(C.tip_point(i, Qc[-1, i], np.array([0, 0, H_REST]))[0] - feet[i]) for i in range(8)) * 1000
    # lying pose: arm straight, sloping 15.5 deg down from its root (133 mm up) so the folded fingers touch the floor
    # 0.57 m out; mid pose: foot lifted and swung outward first (checked: nothing dips below the floor on the way)
    Qs = np.tile(np.radians([15.5, 0, 0, 0, 0, 0, -15.5, 0]), (8, 1))
    Qm = np.tile(np.radians([-45, 0, 0, 0, 40, 0, 10, 0]), (8, 1))
    f, u = C.tip_point(0, Qs[0], np.array([0, 0, H_REST]), lead=0.0)
    print(f"plan: crouch foot error {err:.1f} mm | sprawl spigot z {f[2] * 1000:.0f} mm, axis z {u[2]:.2f} | "
          f"crouch pose {np.degrees(Qc[-1, 0]).round(0)} | sprawl pose {np.degrees(Qs[0]).round(0)}")
    return hs, Qc, Qs, Qm


class Rest(FL.Flight):
    def __init__(self):
        super().__init__("baseline")
        m, d = self.m, self.d
        d.qpos[:] = 0; d.qvel[:] = 0
        d.qpos[3] = 1.0; d.qpos[2] = S4.H_STAND + 0.004
        d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]; d.qpos[self.imp_q + 3] = 1.0
        for i in range(8):
            for k in range(C.NJ):
                d.qpos[self.ja[i, k]] = S4.Q_STAND[k] / 2; d.qpos[self.jb[i, k]] = S4.Q_STAND[k] / 2
        d.ctrl[:] = np.tile(S4.Q_STAND, 8)
        mujoco.mj_forward(m, d)
        self.hs, self.Qc, self.Qs, self.Qm = plan()

    def target(self, t):
        if t < T_CROUCH[0]: return np.tile(S4.Q_STAND, (8, 1))
        if t < T_CROUCH[1]:
            x = S4.mj((t - T_CROUCH[0]) / (T_CROUCH[1] - T_CROUCH[0])) * 30; j = min(int(x), 29); a = x - j
            return self.Qc[j] * (1 - a) + self.Qc[j + 1] * a
        Q = np.zeros((8, 8))
        for i in range(8):
            u = S4.trap((t - T_LIE[0] - 0.12 * i) / (T_LIE[1] - T_LIE[0] - 0.84))
            if u < 0.5: Q[i] = self.Qc[-1, i] + (self.Qm[i] - self.Qc[-1, i]) * S4.mj(u / 0.5)
            else: Q[i] = self.Qm[i] + (self.Qs[i] - self.Qm[i]) * S4.mj((u - 0.5) / 0.5)
        return Q

    def winch(self, t):
        Tt, Tf, Ts = C.T_LINE["tight"], C.T_LINE["firm"], C.T_LINE["soft"]
        if t < T_LIE[0]: return Tt
        if t < T_SOFT: return Tt + (Tf - Tt) * S4.mj((t - T_LIE[0]) / 1.0)
        return Tf + (Ts - Tf) * S4.mj((t - T_SOFT) / 0.8)

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; st = {}
        while d.time < T_END:
            t = d.time
            d.ctrl[:] = self.target(t).ravel()
            d.qfrc_applied[:] = 0; d.xfrc_applied[:] = 0
            self.clutch(self.winch(t))
            d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G
            mujoco.mj_step(m, d)
            frc = np.abs(d.actuator_force.reshape(8, C.NJ)); servo = frc / C.STALL
            Rb = d.xmat[self.torso].reshape(3, 3); tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
            slip = np.degrees(max(abs(d.qpos[qa]) for qa in self.cq.values()))
            s = st.setdefault(label(t), dict(servo=0.0, servo_at="", tilt=0.0, slip=0.0, rms=np.zeros((8, 8)), n=0, z_end=0.0))
            if servo.max() > s["servo"]:
                i, k = np.unravel_index(servo.argmax(), servo.shape); s["servo_at"] = f"arm{i} j{k}"
            s["servo"] = max(s["servo"], float(servo.max())); s["tilt"] = max(s["tilt"], tilt); s["slip"] = max(s["slip"], slip)
            s["rms"] += (frc / C.RATED) ** 2; s["n"] += 1; s["z_end"] = float(d.qpos[2])
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=np.zeros(8), edf=0.0, servo=servo.max(), servo_arm=servo.max(1),
                                   tilt=tilt, T=self.winch(t), pos=d.xpos[self.torso].copy()))
                nxt += 1 / fps
        for s in st.values():
            s["rms"] = float(np.sqrt(s["rms"] / s["n"]).max()); del s["n"]
        return frames, st


if __name__ == "__main__":
    R = Rest()
    frames, st = R.run()
    for lab, s in st.items():
        print(f"{lab:42s} servo {s['servo']:.2f} x stall ({s['servo_at']}) | rms {s['rms']:.2f} x rated | tilt {s['tilt']:.1f} | "
              f"slip {s['slip']:.1f} | plate height at end {s['z_end'] * 1000:.0f} mm")
    keys = list(frames[0].keys())
    np.savez_compressed("rest4.npz", **{k: np.array([f[k] for f in frames]) for k in keys})
    json.dump(st, open("rest4.json", "w"), indent=1)
