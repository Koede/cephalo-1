"""Cephalo-1 rev 4 organic flight mission: tentacle steering + climbing/descending + grabbing an object in flight.

Built on steer4.Steer (mode "steer": body level, trailing-side arms lean their claw-props).
  * Climb / descend: collective with reference feed-forward.  The arms also change shape in the null space of the
    claw-axis Jacobian, so no thrust axis tilts: hubs pulled 5 cm in while climbing (streamlined), flared 5 cm out
    while descending (umbrella).
  * Grab (arm 2): spin down its claw-prop (0.4 s), swing the arm down into a reach pose (claw pointing down, IK,
    fingers open to 20 deg), lower the body 7 cm so the open fingers surround the object, close the three
    finger-blades to 72 deg, then lift and curl the arm into a carry pose under the body and fly home on 7 claw-props
    + EDF at 100 %.  Object: 0.30 kg ball, 70 mm diameter (est.), waiting on a stand at z 0.40 m.
    Grasp model: if the finger centre is within 3 cm of the object centre when the fingers close (open tip radius 71 mm
    vs object radius 35 mm, so the closing fingers would centre it), a stiff spring-damper (4000 N/m,
    60 N s/m) ties the object to the finger centre.  Finger contact forces are NOT simulated; the object's weight and
    swing do load the arm, the servos and the flight controller.
Route (north = +y, heading held): north 2.5 m -> climb 0.8 m -> east 2.5 m -> descend onto the object -> grab ->
lift 0.5 m -> fly home with it (3.5 m) -> descend.
Run: python3 flymission4.py [mid|pess] -> flymission4_<case>.npz/.json
"""
import os, sys, json, time, numpy as np, mujoco
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4
import flight4 as FL
import steer4 as ST

CASE = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_CASE", "mid")
ST.F_MAX = float(os.environ["CEPH_FMAX"]) if os.environ.get("CEPH_FMAX") else C.FAN_MAX[CASE]
if os.environ.get("CEPH_FMAX"): CASE = "f" + os.environ["CEPH_FMAX"]
GA = 2                                    # grabbing arm
AZ = np.radians(22.5 + 45 * GA)
R_G, Z_G = 0.30, -0.30                    # grasp point in the body frame at the grab (radius, height below the plate)
LEAD = 0.090                              # spigot -> centre of the closed fingers along the claw axis
OBJ_R, OBJ_M = 0.035, 0.30
Z_OBJ = 0.40
Z0, Z_UP = FL.Z0, 1.60
Z_GRAB = Z_OBJ - Z_G                      # plate height with the fingers around the object
Z_PRE = Z_GRAB + 0.07
HOME_XY = np.array([0.0, 0.0]); SITE = np.array([2.5, 2.5])
OBJ = np.array([SITE[0] + R_G * np.cos(AZ), SITE[1] + R_G * np.sin(AZ), Z_OBJ])
# (t0, T, target xyz, label)
SEGS = [(1.0, 3.0, (0.0, 2.5, Z0), "FLY NORTH 2.5 m: south arms lean"),
        (4.3, 2.0, (0.0, 2.5, Z_UP), "CLIMB 0.8 m: arms tuck in"),
        (6.6, 3.0, (2.5, 2.5, Z_UP), "FLY EAST 2.5 m: west arms lean"),
        (9.9, 2.5, (2.5, 2.5, Z_PRE), "DESCEND to the object: arms flare"),
        (14.2, 0.6, (2.5, 2.5, Z_GRAB), "GRAB: lower onto the object"),
        (16.2, 3.0, (2.5, 2.5, 1.05), "LIFT the object"),
        (19.5, 5.5, (0.0, 0.0, 1.05), "FLY HOME carrying it (7 claw-props)"),
        (25.3, 2.0, (0.0, 0.0, Z0), "DESCEND at home")]
T_FANOFF, T_REACH, REACH_DUR = 12.5, 12.8, 1.3
BAL = 0.05                                # balance reshape amplitude (m)
T_CLOSE, CLOSE_DUR = 15.7, 0.35
T_CARRY, CARRY_DUR = 16.3, 2.2
T_END = 28.0
T_MEAS = (14.15, 14.85, 15.3)                   # one-shot finger-to-object measurements (tip camera / proximity sensor, est.)
ST.T_END = T_END


def mj5(u):
    u = np.clip(u, 0, 1); return u ** 3 * (10 - 15 * u + 6 * u * u)


def ref3(t):
    p = np.array([0.0, 0.0, Z0]); v = np.zeros(3); a = np.zeros(3)
    for t0, T, tgt, _ in SEGS:
        tgt = np.array(tgt, float); dlt = tgt - p
        u = (t - t0) / T
        if u >= 1:
            p = tgt
            continue
        if u > 0:
            p = p + dlt * mj5(u)
            v = dlt * 30 * u * u * (1 - u) ** 2 / T
            a = dlt * 60 * u * (1 - u) * (1 - 2 * u) / T ** 2
        break
    return p, v, a


def label(t):
    if T_FANOFF <= t < T_REACH + REACH_DUR:
        return "GRAB: arm 2 stops its prop and reaches down"
    if T_CLOSE - 0.1 <= t < T_CLOSE + CLOSE_DUR + 0.3:
        return "GRAB: fingers close"
    if T_MEAS[1] - 0.1 <= t < T_CLOSE - 0.1:
        return "GRAB: line up the fingers (tip sensor)"
    for t0, T, _, lab in SEGS:
        if t0 - 0.2 <= t < t0 + T + 0.3:
            return lab
    return "HOVER"


# arm 2 key poses (body frame, heading 0)
_q0 = FL.Q_FLY.copy()
Q_REACH = C.ik(GA, np.array([R_G * np.cos(AZ), R_G * np.sin(AZ), Z_G]), np.array([0, 0, -1.0]), np.radians([30, 0, 40, 0, 40, 0, 20, 0]),
               np.radians([30, 0, 40, 0, 40, 0, 20, 0]), np.zeros(3), lead=LEAD)
R_C, Z_C = 0.26, -0.22
Q_CARRY = C.ik(GA, np.array([R_C * np.cos(AZ), R_C * np.sin(AZ), Z_C]), np.array([0, 0, -1.0]), Q_REACH, Q_REACH, np.zeros(3), lead=LEAD)


class Mission(ST.Steer):
    def __init__(self):
        super().__init__("steer")
        m, d = self.m, self.d
        self.ob = m.body("imp").id; gid = m.geom("imp").id
        m.body_mass[self.ob] = OBJ_M; m.body_inertia[self.ob] = 0.4 * OBJ_M * OBJ_R ** 2
        m.geom_size[gid, 0] = OBJ_R; m.geom_contype[gid] = 0; m.geom_conaffinity[gid] = 0
        self.latched = False; self.t_latch = None; self.miss = None
        self.fold_ga = 0.0; self.grip_force = 0.0; self.corr = np.zeros(3); self.corr_tgt = np.zeros(3); self.align_err = None; self.n_meas = 0

    def ref2(self, t):
        p, v, a = ref3(t); return p[:2] + self.corr[:2], v[:2], a[:2]

    def zref(self, t):
        p, v, a = ref3(t); return p[2] + self.corr[2], v[2], a[2]

    def pos_xy(self):
        return self.d.xpos[self.torso][:2]            # track the plate centre (the arm reach shifts the COM)

    def edf_at(self, t):
        return C.EDF_MAX * (0.8 + 0.2 * mj5((t - 11.5) / 1.0))

    def payload_m(self):
        return OBJ_M if self.latched else 0.0

    def arm_override(self, i, t):
        if i != GA or t < T_REACH:
            return None
        u = (t - T_REACH) / REACH_DUR
        if t < T_CARRY:
            return FL.Q_FLY + (Q_REACH - FL.Q_FLY) * S4.trap(u)
        return Q_REACH + (Q_CARRY - Q_REACH) * S4.trap((t - T_CARRY) / CARRY_DUR)

    def arms(self, t, dt):
        _, vz, _ = self.zref(t)
        # uniform: tuck in while climbing, flare out while descending.  Balance: while arm 2's claw-prop is off, the arms
        # beside the gap move their hubs out and the opposite arm moves its hub in (null space, axes stay vertical), so
        # the seven running claw-props share the load more evenly (static optimum 10.05 -> 9.5 N per prop)
        dphi = np.radians(45 * (np.arange(8) - GA))
        bal = BAL * (1 - self.gate[GA]) * np.cos(dphi); bal[GA] = 0.0
        self.flare_amp = -0.05 * float(np.clip(vz / 0.6, -1, 1)) + bal
        self.gate[GA] = 1.0 - S4.mj((t - T_FANOFF) / 0.4)
        self.breath = 1.0 - S4.mj((t - 11.9) / 0.6) + S4.mj((t - T_CARRY) / 1.0)   # arms hold still for the grab
        if self.latched:                       # gentle with the payload on seven claw-props
            self.lean_max, self.a_max = np.radians(25), 1.2
        if self.n_meas < len(T_MEAS) and t >= T_MEAS[self.n_meas]:
            # fine alignment: the tip node measures where the object sits relative to the open fingers and shifts the
            # body reference once (before the final lowering, then again after it); the shift is blended in smoothly
            g, _ = self.grasp_pt()
            still_to_lower = np.array([0, 0, (Z_PRE - Z_GRAB) if self.n_meas == 0 else 0.0])
            self.corr_tgt = self.corr_tgt + (OBJ + still_to_lower - g)
            self.n_meas += 1
        self.corr += (self.corr_tgt - self.corr) * min(1.0, dt / 0.15)
        if T_CLOSE <= t < T_CLOSE + dt + 1e-9:
            g, _ = self.grasp_pt(); self.align_err = float(np.linalg.norm(OBJ - g))
        if t < T_FANOFF: self.fold_ga = 0.0
        elif t < T_CLOSE: self.fold_ga = np.radians(20) * S4.mj((t - T_FANOFF - 0.3) / 0.6)
        else: self.fold_ga = np.radians(20 + 52 * S4.mj((t - T_CLOSE) / CLOSE_DUR))
        return super().arms(t, dt)

    def grasp_pt(self):
        d = self.d
        R = d.xmat[self.pod[GA]].reshape(3, 3)
        return d.xpos[self.pod[GA]] + R[:, 0] * LEAD, R

    def impactor(self, t):
        d = self.d; q, v = self.imp_q, self.imp_d
        g, R = self.grasp_pt()
        if not self.latched:
            d.qpos[q:q + 3] = OBJ; d.qpos[q + 3:q + 7] = [1, 0, 0, 0]; d.qvel[v:v + 6] = 0
            d.qfrc_applied[v + 2] += OBJ_M * C.G                       # resting on its stand
            if T_CLOSE <= t <= T_CLOSE + CLOSE_DUR:
                dist = float(np.linalg.norm(g - OBJ))
                self.miss = dist if self.miss is None else min(self.miss, dist)
                if dist < 0.030:          # closing fingers (tip radius 71 mm open) centre a 35 mm-radius ball
                    self.latched = True; self.t_latch = t
            self.grip_force = 0.0
            return
        # stiff spring-damper between the closed fingers and the object
        gv = np.zeros(6); mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, self.pod[GA], gv, 0)
        vg = gv[3:] + np.cross(gv[:3], R[:, 0] * LEAD)
        xo = d.qpos[q:q + 3]; vo = d.qvel[v:v + 3]
        F = (4000.0 * (g - xo) + 60.0 * (vg - vo)) * S4.mj((t - self.t_latch) / 0.15)
        self.grip_force = float(np.linalg.norm(F))
        d.xfrc_applied[self.ob, :3] += F
        d.xfrc_applied[self.pod[GA], :3] -= F
        d.xfrc_applied[self.pod[GA], 3:] -= np.cross(g - d.xipos[self.pod[GA]], F)

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_arm = -1.0
        segst = {}
        rms = np.zeros((8, C.NJ)); nrm = 0; t0 = time.time()
        while d.time < T_END:
            t = d.time
            if t - t_arm >= 0.01 - 1e-9:
                d.ctrl[:] = self.arms(t, 0.01).ravel(); t_arm = t
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
            p, v, a = ref3(t)
            pt = d.xpos[self.torso].copy()
            err = float(np.linalg.norm(p - pt)) * 100
            cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
            dax = np.degrees(np.arccos(np.clip(cz, -1, 1)))
            lab = label(t)
            ss = segst.setdefault(lab, dict(err=0.0, tilt=0.0, yaw=0.0, servo=0.0, servo_at="", slip=0.0, sat=0, lean=0.0, fan=0.0, grip=0.0))
            if servo.max() > ss["servo"]:
                i, k = np.unravel_index(servo.argmax(), servo.shape); ss["servo_at"] = f"arm{i} j{k}"
            ss["err"] = max(ss["err"], err); ss["tilt"] = max(ss["tilt"], tilt); ss["yaw"] = max(ss["yaw"], abs(yaw))
            ss["servo"] = max(ss["servo"], float(servo.max())); ss["slip"] = max(ss["slip"], float(np.degrees(slip)))
            ss["sat"] = max(ss["sat"], self.sat); ss["lean"] = max(ss["lean"], float(np.delete(dax, GA).max()))
            ss["fan"] = max(ss["fan"], float(self.fan.max())); ss["grip"] = max(ss["grip"], self.grip_force)
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo=servo.max(),
                                   servo_arm=servo.max(1), tilt=tilt, yaw=yaw, err=err, pos=pt, ref=p.copy(),
                                   vel=d.qvel[0:3].copy(), dax=dax, wts=self.wts.copy(), adem=self.adem.copy(), sat=self.sat,
                                   fold_ga=self.fold_ga, latched=float(self.latched), gate=self.gate.copy(),
                                   obj=d.qpos[self.imp_q:self.imp_q + 3].copy(), grip=self.grip_force))
                nxt += 1 / fps
        print(f"mission {CASE}: sim {T_END:.1f} s in {time.time() - t0:.0f} s wall | latched {self.latched} at "
              f"{self.t_latch} | miss {self.miss} | finger-object offset at close {self.align_err}")
        return frames, segst, float(np.sqrt(rms / nrm).max())


if __name__ == "__main__":
    print("Q_REACH", np.degrees(Q_REACH).round(1), "Q_CARRY", np.degrees(Q_CARRY).round(1))
    for nm, q, tgt in (("reach", Q_REACH, [R_G * np.cos(AZ), R_G * np.sin(AZ), Z_G]), ("carry", Q_CARRY, [R_C * np.cos(AZ), R_C * np.sin(AZ), Z_C])):
        f, u = C.tip_point(GA, q, np.zeros(3), lead=LEAD)
        print(nm, "residual mm", (np.linalg.norm(f - tgt) * 1000).round(1), "axis z", u[2].round(3))
    M = Mission()
    frames, segst, rms = M.run()
    for lab, s in segst.items():
        print(f"{lab:44s} err {s['err']:5.1f} cm | tilt {s['tilt']:4.1f} | yaw {s['yaw']:4.1f} | servo {s['servo']:.2f} ({s['servo_at']}) | "
              f"slip {s['slip']:4.1f} | sat {s['sat']} | lean {s['lean']:4.1f} | fan {s['fan']:4.1f} | grip {s['grip']:5.1f} N")
    print("rms/rated", round(rms, 2))
    keys = list(frames[0].keys())
    np.savez_compressed(f"flymission4_{CASE}.npz", **{k: np.array([f[k] for f in frames]) for k in keys})
    json.dump(dict(case=CASE, segments=segst, rms_rated=rms, latched=M.latched, t_latch=M.t_latch, miss=M.miss,
                   obj=OBJ.tolist()), open(f"flymission4_{CASE}.json", "w"), indent=1)
