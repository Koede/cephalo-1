"""Cephalo-1 rev 4 in-flight chores (full-robot MuJoCo): two opposite arms (2 and 6) work as hands while the other six
claw-props + EDF fly the robot (tentacle steering, steer4.py).

Why arms 2 and 6: with two claw-props stopped, only an OPPOSITE pair leaves the other six balanced
(10.2 N per prop with a 0.4 kg load; a 135 deg pair needs 11.9 N, a 90 deg pair 13.9 N, of 12.6 N mid thrust).

Scenarios
  shovel : pick a 0.40 kg, 0.80 m shovel (est.) off the ground with arm 2 at its balance point, carry it 1.2 m to a
           sandbox, arm 6 takes the top of the handle (two-handed digging grip: arm 6 steers, arm 2 lifts), then: tilt
           the blade down 18 deg, push in, lever a 0.2 kg scoop out, fly it 0.35 m aside, tip it off, fly back, stab the
           shovel back into the sand and let go.  (One claw cannot hold the shovel away from its balance point: that
           needs ~1.7 N m of grip torque, so the hand-over adds the second hand instead of swapping hands.)
           Hand workspace under the body (claw down): 0.18-0.38 m out, 0.16-0.38 m below the plate.
  dishes : hover over a sink; arm 2 takes a dish brush from its rest, arm 6 holds a 0.25 kg pan by its handle just above the sink grid,
           arm 2 scrubs three circles inside the pan, arm 6 lifts the pan onto the drying rack, the brush goes back.
Hand motion: planned in the body frame (dense IK along the path, warm-started), plus a small correction from the tip
sensor (est.) that lines the open fingers up on the object before each grasp.
Grasp model: if the finger centre is within 3 cm of the grip point when the fingers close, the object is tied to the
hand by a spring-damper (1500 N/m, 40 N s/m for the shovel; softer for the pan and brush) and a soft rotational spring (3 N m/rad for the shovel, less for the light tools = grip friction, est.).  Finger
contact, bristle contact and water are NOT simulated; the reported grip force/torque is what the fingers must hold.
Sand: support 1500 N/m (max 15 N) + drag 40 N s/m (max 10 N) on the blade tip (est.).
Run: python3 chores4.py shovel|dishes [mid|pess] -> chores4_<scn>_<case>.npz/.json     (plan only: add 'plan')
"""
import os, sys, json, time, numpy as np, mujoco
from scipy.optimize import least_squares
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4

SCN = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_SCN", "shovel")
CASE = sys.argv[2] if (__name__ == "__main__" and len(sys.argv) > 2 and sys.argv[2] != "plan") else os.environ.get("CEPH_CASE", "mid")
HA, HB = 2, 6                                   # the two hands (opposite arms)
AZ = np.radians(22.5 + 45 * HA)
E = np.array([np.cos(AZ), np.sin(AZ), 0.0]); TV = np.array([-np.sin(AZ), np.cos(AZ), 0.0]); UZ = np.array([0, 0, 1.0])
LEAD = 0.090
DOWN = -UZ


def BF(x, y, z):
    """Body-frame point: x toward arm 2, y tangential, z up."""
    return x * E + y * TV + z * UZ


def mj5(u):
    u = np.clip(u, 0, 1); return u ** 3 * (10 - 15 * u + 6 * u * u)


def kf(t, keys):
    """Minimum-jerk interpolation through keyframes [(t, value)]; clamped outside."""
    if t <= keys[0][0]: return np.asarray(keys[0][1], float)
    for (t0, v0), (t1, v1) in zip(keys[:-1], keys[1:]):
        if t <= t1:
            return np.asarray(v0, float) + (np.asarray(v1, float) - np.asarray(v0, float)) * mj5((t - t0) / (t1 - t0))
    return np.asarray(keys[-1][1], float)


def obj_xml(name, mass, I):
    return (f'<body name="{name}" pos="0 0 -6"><freejoint name="{name}j"/>'
            f'<inertial pos="0 0 0" mass="{mass}" diaginertia="{I[0]:.6f} {I[1]:.6f} {I[2]:.6f}"/></body>')


# ====================================================================== scenario definitions
if SCN == "shovel":
    Z0 = 0.80
    H2 = np.array([1.2, 0.0])                                   # hover point at the sandbox
    H3 = H2 + 0.05 * E[:2]
    H4 = H3 + 0.35 * TV[:2]                                     # where the scoop is tipped off, beside the sandbox
    Z_LINE, Z_PUSH, Z_LEV = 0.796, 0.726, 0.95
    SEGS = [(0.8, 1.8, (0, 0, 0.485), "DESCEND over the shovel"),
            (4.6, 0.6, (0, 0, 0.415), "PICK UP: lower onto the shovel"),
            (6.2, 2.4, (0, 0, 1.00), "LIFT the shovel (arm 2, balance point)"),
            (8.8, 3.0, (1.2, 0, 1.00), "CARRY it to the sandbox (7 claw-props)"),
            (14.6, 1.6, (1.2, 0, Z_LINE), "TWO HANDS: tilt the blade down, line up"),
            (16.6, 1.0, (H3[0], H3[1], Z_PUSH), "DIG: push the blade in"),
            (17.8, 1.4, (H3[0], H3[1], Z_LEV), "DIG: lever the scoop out"),
            (19.4, 1.8, (H4[0], H4[1], Z_LEV), "DIG: carry the scoop aside"),
            (22.4, 1.8, (H3[0], H3[1], Z_LEV), "Back over the sandbox"),
            (24.3, 1.2, (H3[0], H3[1], Z_PUSH), "STAB the shovel back into the sand"),
            (27.7, 1.5, (H3[0], H3[1], 1.00), "FLY ON (8 claw-props)")]
    LABELS = [(2.5, 4.6, "PICK UP: arm 2 stops its prop, reaches down"), (5.2, 6.2, "PICK UP: fingers close"),
              (11.9, 14.6, "HAND OVER: arm 6 takes the handle top"), (21.2, 22.4, "DIG: tip the sand off"),
              (25.6, 27.7, "LET GO: both hands release, props back on")]
    T_END = 30.0
    SH_L, SH_M = 0.80, 0.40                                     # shovel (est.): COM 0.50 m from the handle end
    G_MID, G_TOP, S_TIP, S_SCOOP = 0.0, -0.43, 0.30, 0.20
    OBJS = {"shovel": dict(m=SH_M, I=(0.0004, SH_M * SH_L ** 2 / 12, SH_M * SH_L ** 2 / 12), kr=3.0, cr=0.08, kp=1500.0, cp=40.0,
                           pos=BF(0.26, 0, 0) + np.array([0, 0, 0.045]), R=np.stack([E, TV, UZ], 1))}
    SAND_C = np.array([H2[0], H2[1], 0]) + BF(0.50, 0, 0); SAND_Z, SAND_HALF, M_SCOOP = 0.30, 0.35, 0.20
    T_SAND_ON, T_SAND_OFF = 18.1, 21.9
    T2 = 14.6
    Tx = [(T2, -0.17), (16.2, -0.21)]
    Tz = [(T2, -0.30), (16.2, -0.24)]
    TH = [(T2, 0), (16.2, 18), (17.8, 18), (19.2, 5), (21.3, 5), (22.1, 18)]
    HAk = [(26.1, 0), (26.3, 0.05)]
    HBk = [(13.3, 0.07), (13.8, 0.0), (26.1, 0), (26.3, 0.05)]

    def shovel_pose(t):
        th = np.radians(kf(t, TH)); T = BF(kf(t, Tx), 0.0, kf(t, Tz))
        dr = np.cos(th) * E - np.sin(th) * UZ; n = -np.sin(th) * E - np.cos(th) * UZ
        return T, dr, n

    def hand_pose(h, t):
        T, dr, n = shovel_pose(t)
        if h == HA:
            if t < T2:
                return kf(t, [(4.2, BF(0.26, 0, -0.37)), (5.9, BF(0.26, 0, -0.37)), (7.5, BF(0.26, 0, -0.30))]), DOWN
            return T + 0.43 * dr - kf(t, HAk) * n, n
        return T - kf(t, HBk) * n, n

    EPIS = {HA: (4.2, 26.3), HB: (13.3, 26.3)}                  # first / last key time of each hand
    GATE = {HA: (2.5, 27.7), HB: (11.9, 27.7)}                  # claw-prop off / on
    FOLD = {HA: [(2.6, 0), (3.4, 20), (5.4, 20), (5.75, 72), (25.8, 72), (26.1, 20), (27.7, 20), (28.1, 0)],
            HB: [(12.0, 0), (12.7, 20), (13.8, 20), (14.15, 72), (25.8, 72), (26.1, 20), (27.7, 20), (28.1, 0)]}
    # (time, hand, object, grip point in the object frame, correction window start, z-correction start)
    LATCH = [(5.75, HA, "shovel", (G_MID, 0, 0), 4.2, 5.2), (14.15, HB, "shovel", (G_TOP, 0, 0), 13.3, 13.3)]
    UNLATCH = [(25.95, HA, "shovel"), (25.95, HB, "shovel")]
    PIN = [(26.0, "shovel")]
else:
    Z0 = 1.22
    SEGS = [(0.0, 0.1, (0, 0, Z0), "HOVER over the sink")]
    LABELS = [(0.4, 3.0, "Arms 2 and 6 stop their props and reach down"), (3.0, 3.6, "GRAB: brush (arm 2), pan handle (arm 6)"),
              (3.6, 5.8, "Bring the brush to the pan"), (5.8, 10.4, "SCRUB: three circles inside the pan"),
              (10.4, 13.6, "Pan onto the drying rack, brush back to its rest"), (13.6, 14.4, "LET GO"),
              (14.4, 17.5, "Arms back to flying, props on")]
    T_END = 17.5
    PAN_M, BR_M = 0.25, 0.06                                    # small skillet and dish brush (est.)
    Z_GRID, Z_COUNTER = 0.82, 0.90
    OBJS = {"pan": dict(m=PAN_M, I=(0.0008, 0.0030, 0.0036), kr=3.0, cr=0.08, kp=1200.0, cp=30.0, pos=BF(-0.01, 0, 0) + np.array([0, 0, Z_GRID + 0.02]),
                        R=np.stack([-E, -TV, UZ], 1)),
            "brush": dict(m=BR_M, I=(0.00003, 0.0004, 0.0004), kr=0.3, cr=0.003, kp=400.0, cp=8.0, pos=BF(0.21, 0, 0) + np.array([0, 0, 0.94]),
                          R=np.stack([-E, -TV, UZ], 1))}
    PAN_GRIP, BR_GRIP, BR_HEAD = (0.17, 0, 0.02), (-0.09, 0, 0), (0.10, 0, -0.065)
    T_SCRUB = (5.8, 10.3)

    def hand_pose(h, t):
        if h == HA:
            p = kf(t, [(2.2, (0.30, 0, -0.21)), (2.3, (0.30, 0, -0.21)), (2.8, (0.30, 0, -0.28)), (3.5, (0.30, 0, -0.28)),
                       (4.0, (0.30, 0, -0.23)), (5.2, (0.25, 0, -0.27)), (5.7, (0.25, 0, -0.33)), (5.8, (0.25, 0, -0.33))])
            if T_SCRUB[0] < t:
                ph = 2 * np.pi * 3 * mj5((min(t, T_SCRUB[1]) - T_SCRUB[0]) / (T_SCRUB[1] - T_SCRUB[0]))
                p = np.array([0.21 + 0.04 * np.cos(ph), 0.04 * np.sin(ph), -0.33])
            if t > T_SCRUB[1]:
                p = kf(t, [(10.4, (0.25, 0, -0.33)), (10.9, (0.25, 0, -0.25)), (12.0, (0.25, 0, -0.25)), (13.2, (0.30, 0, -0.21)),
                           (13.3, (0.30, 0, -0.21)), (13.8, (0.30, 0, -0.28)), (14.25, (0.30, 0, -0.28)), (14.4, (0.30, 0, -0.23))])
            return BF(*p), DOWN
        p = kf(t, [(2.2, (-0.18, 0, -0.29)), (2.3, (-0.18, 0, -0.29)), (2.8, (-0.18, 0, -0.36)), (11.0, (-0.18, 0, -0.36)),
                   (11.8, (-0.18, 0, -0.22)), (13.0, (-0.38, 0, -0.22)), (13.5, (-0.38, 0, -0.26)), (14.25, (-0.38, 0, -0.26)),
                   (14.4, (-0.38, 0, -0.21))])
        return BF(*p), DOWN

    EPIS = {HA: (2.2, 14.4), HB: (2.2, 14.4)}
    GATE = {HA: (0.5, 15.8), HB: (0.5, 15.8)}
    FOLD = {h: [(0.6, 0), (1.4, 20), (3.0, 20), (3.35, 72), (13.9, 72), (14.2, 20), (15.8, 20), (16.2, 0)] for h in (HA, HB)}
    LATCH = [(3.35, HA, "brush", BR_GRIP, 2.2, 2.6), (3.35, HB, "pan", PAN_GRIP, 2.2, 2.6)]
    UNLATCH = [(14.1, HA, "brush"), (14.1, HB, "pan")]
    PIN = [(14.15, "brush"), (14.15, "pan")]

S4.EXTRA_XML = "".join(obj_xml(n, o["m"], o["I"]) for n, o in OBJS.items())
import flight4 as FL
import steer4 as ST
ST.F_MAX = float(os.environ["CEPH_FMAX"]) if os.environ.get("CEPH_FMAX") else C.FAN_MAX[CASE]
ST.T_END = T_END
BLEND = 1.3
Q_NOM = np.radians([-14.7, 0, 60.6, 0, 64.3, 0, -19.7, 0])


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
    for a, b, lab in LABELS:
        if a <= t < b: return lab
    for t0, T, _, lab in SEGS:
        if t0 - 0.1 <= t < t0 + T + 0.3: return lab
    return "HOVER"


def ik_clear(h, p, n, q0, reg=0.25):
    """IK for the grasp point + claw axis that also keeps the arm >= 25 mm clear of the mantle / plate / EDF."""
    lo, hi = C.LIM.T
    def res(q):
        f, u = C.tip_point(h, q, np.zeros(3), lead=LEAD)
        P, r, _ = C.arm_capsules(h, q, np.zeros(3), n=16)
        clr = C.body_clearance(P, r, np.zeros(3))
        return np.concatenate([(f - p) * 10, (u - n) * 1.0, reg * (q - q0), 0.05 * q[1::2], np.clip(0.025 - clr, 0, None) * 30,
                               np.clip(np.radians(6) - (q - lo), 0, None) * 2, np.clip(np.radians(6) - (hi - q), 0, None) * 2])
    return least_squares(res, np.clip(q0, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=200).x


def plan(verbose=False):
    """Dense IK along each hand's path (body frame).  Returns {hand: (times, Q)} and prints the checks."""
    out = {}
    for h, (ta, tb) in EPIS.items():
        ts = np.arange(ta, tb + 1e-9, 0.05); Q = []; q = Q_NOM.copy(); worst = [0, 0, 9, 9]
        for t in ts:
            p, n = hand_pose(h, t)
            q = ik_clear(h, p, n, q, 0.03 if t == ts[0] else 0.25)
            f, u = C.tip_point(h, q, np.zeros(3), lead=LEAD)
            P, r, _ = C.arm_capsules(h, q, np.zeros(3))
            worst[0] = max(worst[0], np.linalg.norm(f - p) * 1000); worst[1] = max(worst[1], np.degrees(np.arccos(np.clip(u @ n, -1, 1))))
            worst[2] = min(worst[2], C.body_clearance(P, r, np.zeros(3)).min() * 1000)
            worst[3] = min(worst[3], np.degrees(np.minimum(q - C.LIM[:, 0], C.LIM[:, 1] - q)).min())
            Q.append(q.copy())
            if verbose and (np.linalg.norm(f - p) > 0.004 or C.body_clearance(P, r, np.zeros(3)).min() < 0.010):
                print(f"   t {t:5.2f} arm {h}: err {np.linalg.norm(f - p) * 1000:5.1f} mm clr {C.body_clearance(P, r, np.zeros(3)).min() * 1000:5.0f} mm "
                      f"target x {p @ E:+.2f} y {p @ TV:+.2f} z {p[2]:+.2f} q {np.degrees(q).round(0)}")
        Q = np.array(Q)
        v = np.abs(np.gradient(np.degrees(Q), 0.05, axis=0)).max()
        print(f"plan arm {h}: max IK error {worst[0]:.1f} mm, axis {worst[1]:.1f} deg | min body clearance {worst[2]:.0f} mm | "
              f"min joint-limit margin {worst[3]:.1f} deg | peak joint speed {v:.0f} deg/s")
        out[h] = (ts, Q)
    return out


class Chores(ST.Steer):
    def __init__(self):
        super().__init__("steer")
        m, d = self.m, self.d
        self.traj = plan()
        self.ob = {}
        for n, o in OBJS.items():
            q = np.zeros(4); mujoco.mju_mat2Quat(q, o["R"].ravel())
            self.ob[n] = dict(id=m.body(n).id, qa=m.joint(n + "j").qposadr[0], da=m.joint(n + "j").dofadr[0], m=o["m"], kr=o["kr"], cr=o["cr"], kp=o["kp"], cp=o["cp"],
                              pinned=True, pose=(o["pos"].copy(), q))
        self.ties = []                                  # dict(h, obj, off, Rrel, t0)
        self.corr = {HA: np.zeros(3), HB: np.zeros(3)}
        self.fold = {HA: 0.0, HB: 0.0}
        self.grip = {HA: [0.0, 0.0], HB: [0.0, 0.0]}    # tie force (N), torque (N m)
        self.done_l, self.done_u, self.done_p = set(), set(), set()
        self.log = []
        self.sand_on = False
        self.att_ki = 12.0
        d.qpos[2] = Z0; self.impactor(0.0); mujoco.mj_forward(m, d)

    # ---- references
    def ref2(self, t):
        p, v, a = ref3(t); return p[:2], v[:2], a[:2]

    def zref(self, t):
        p, v, a = ref3(t); return p[2], v[2], a[2]

    def pos_xy(self):
        return self.d.xpos[self.torso][:2]

    def edf_at(self, t):
        return C.EDF_MAX * (0.8 + 0.2 * mj5(t / 1.0))

    def payload_m(self):
        return sum(self.ob[n]["m"] for n in {tie["obj"] for tie in self.ties}) + (M_SCOOP if self.sand_on else 0.0)

    # ---- hands
    def hand_frame(self, h):
        d = self.d; R = d.xmat[self.pod[h]].reshape(3, 3)
        return d.xpos[self.pod[h]] + R[:, 0] * LEAD, R

    def obj_frame(self, n):
        d = self.d; o = self.ob[n]
        return d.xpos[o["id"]].copy(), d.xmat[o["id"]].reshape(3, 3).copy()

    def q_hand(self, h, t):
        ta, tb = EPIS[h]; ts, Q = self.traj[h]
        if t <= ta - BLEND or t >= tb + BLEND: return None
        if t < ta: return FL.Q_FLY + (Q[0] - FL.Q_FLY) * S4.trap((t - (ta - BLEND)) / BLEND)
        if t > tb: return Q[-1] + (FL.Q_FLY - Q[-1]) * S4.trap((t - tb) / BLEND)
        x = (t - ta) / 0.05; j = min(int(x), len(Q) - 2); a = x - j
        return Q[j] * (1 - a) + Q[j + 1] * a

    def arms(self, t, dt):
        d = self.d
        Rb = d.xmat[self.torso].reshape(3, 3)
        for h in (HA, HB):
            self.gate[h] = 1.0 - S4.mj((t - GATE[h][0]) / 0.4) + S4.mj((t - GATE[h][1]) / 0.4)
            self.fold[h] = np.radians(float(kf(t, FOLD[h])))
        phi = np.radians(45 * np.arange(8))
        bal = np.zeros(8)
        for h in (HA, HB):
            bal += ST_BAL * (1 - self.gate[h]) * np.cos(phi - phi[h])
        bal[[HA, HB]] = 0.0
        _, vz, _ = self.zref(t)
        self.flare_amp = -0.05 * float(np.clip(vz / 0.6, -1, 1)) + bal
        hands_busy = min(self.gate[HA], self.gate[HB]) < 0.5
        self.breath = 0.0 if hands_busy else 1.0
        self.lean_max, self.a_max = (np.radians(25), 1.2) if self.ties else (ST.LEAN_MAX, ST.A_MAX)
        # tip-sensor alignment before each grasp
        for (tl, h, n, off, tc, tz) in LATCH:
            if tc <= t < tl - 0.3 and (tl, h) not in self.done_l:
                po, Ro = self.obj_frame(n); g, _ = self.hand_frame(h)
                e = Rb.T @ (po + Ro @ np.array(off) - g)
                if t < tz: e[2] = 0.0
                c = self.corr[h] + 3.0 * e * dt
                nrm = np.linalg.norm(c)
                self.corr[h] = c * min(1.0, 0.06 / max(nrm, 1e-9))
        q_all = super().arms(t, dt)
        stp = FL.V_ARM * dt
        for h in (HA, HB):
            q = self.q_hand(h, t)
            if q is None: continue
            if np.linalg.norm(self.corr[h]) > 1e-5:
                f0 = C.tip_point(h, q, np.zeros(3), lead=LEAD)[0]; Jp = np.zeros((3, 8))
                for k in range(8):
                    dq = q.copy(); dq[k] += 1e-4
                    Jp[:, k] = (C.tip_point(h, dq, np.zeros(3), lead=LEAD)[0] - f0) / 1e-4
                q = q + Jp.T @ np.linalg.solve(Jp @ Jp.T + 1e-4 * np.eye(3), self.corr[h])
            q = np.clip(q, C.LIM[:, 0], C.LIM[:, 1])
            self.q_cmd[h] = self._prev[h] + np.clip(q - self._prev[h], -stp, stp)
            self.dq[h] = self.q_cmd[h] - FL.Q_FLY; self.q_geo[h] = self.q_cmd[h].copy()
        self._prev = self.q_cmd.copy()
        return self.q_cmd

    _prev = np.tile(FL.Q_FLY, (8, 1))

    def arm_override(self, i, t):
        return self.q_hand(i, t) if i in (HA, HB) else None

    # ---- objects, ties, sand (called every physics step, after thrust())
    def impactor(self, t):
        m, d = self.m, self.d
        d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G                     # the unused impactor stays parked
        for (tl, h, n, off, tc, tz) in LATCH:
            if t >= tl and (tl, h) not in self.done_l:
                self.done_l.add((tl, h))
                po, Ro = self.obj_frame(n); g, Rh = self.hand_frame(h)
                dist = float(np.linalg.norm(po + Ro @ np.array(off) - g))
                ok = dist < 0.03
                self.log.append(dict(t=round(t, 2), hand=h, obj=n, offset_mm=round(dist * 1000, 1), grasped=ok))
                if ok:
                    self.ties.append(dict(h=h, obj=n, off=np.array(off, float), Rrel=Rh.T @ Ro, t0=t))
                    self.ob[n]["pinned"] = False
        for (tu, h, n) in UNLATCH:
            if t >= tu and (tu, h) not in self.done_u:
                self.done_u.add((tu, h)); self.ties = [x for x in self.ties if not (x["h"] == h and x["obj"] == n)]
        for (tp, n) in PIN:
            if t >= tp and (tp, n) not in self.done_p:
                self.done_p.add((tp, n)); o = self.ob[n]
                o["pinned"] = True; o["pose"] = (d.qpos[o["qa"]:o["qa"] + 3].copy(), d.qpos[o["qa"] + 3:o["qa"] + 7].copy())
        for n, o in self.ob.items():
            if o["pinned"]:
                d.qpos[o["qa"]:o["qa"] + 3] = o["pose"][0]; d.qpos[o["qa"] + 3:o["qa"] + 7] = o["pose"][1]
                d.qvel[o["da"]:o["da"] + 6] = 0; d.qfrc_applied[o["da"] + 2] += o["m"] * C.G
        self.grip = {HA: [0.0, 0.0], HB: [0.0, 0.0]}
        for tie in self.ties:
            h, o = tie["h"], self.ob[tie["obj"]]
            g, Rh = self.hand_frame(h); po, Ro = self.obj_frame(tie["obj"])
            vh = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.pod[h], vh, 0)
            vo = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, o["id"], vo, 0)
            p = po + Ro @ tie["off"]
            vg = vh[3:] + np.cross(vh[:3], g - d.xpos[self.pod[h]]); vp = vo[3:] + np.cross(vo[:3], p - po)
            ramp = S4.mj((t - tie["t0"]) / 0.15)
            F = (o["kp"] * (g - p) + o["cp"] * (vg - vp)) * ramp
            Re = (Rh @ tie["Rrel"]) @ Ro.T
            w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)); sn = np.linalg.norm(w)
            rv = w * (ang / sn) if sn > 1e-6 else w
            tau = (o["kr"] * rv + o["cr"] * (vh[:3] - vo[:3])) * ramp
            d.xfrc_applied[o["id"], :3] += F; d.xfrc_applied[o["id"], 3:] += np.cross(p - d.xipos[o["id"]], F) + tau
            d.xfrc_applied[self.pod[h], :3] -= F
            d.xfrc_applied[self.pod[h], 3:] -= np.cross(g - d.xipos[self.pod[h]], F) + tau
            self.grip[h] = [float(np.linalg.norm(F)), float(np.linalg.norm(tau))]
        if SCN == "shovel":
            o = self.ob["shovel"]
            if not o["pinned"]:
                po, Ro = self.obj_frame("shovel"); tip = po + Ro[:, 0] * S_TIP
                vo = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, o["id"], vo, 0)
                vt = vo[3:] + np.cross(vo[:3], tip - po)
                if np.all(np.abs(tip[:2] - SAND_C[:2]) < SAND_HALF) and tip[2] < SAND_Z:
                    Fz = float(np.clip(1500.0 * (SAND_Z - tip[2]) - 30.0 * vt[2], 0, 15.0))
                    Fh = -40.0 * vt[:2]; nh = np.linalg.norm(Fh)
                    if nh > 10.0: Fh *= 10.0 / nh
                    Fs = np.array([Fh[0], Fh[1], Fz])
                    d.xfrc_applied[o["id"], :3] += Fs; d.xfrc_applied[o["id"], 3:] += np.cross(tip - d.xipos[o["id"]], Fs)
                self.sand_on = T_SAND_ON <= t < T_SAND_OFF
                if self.sand_on:      # the scoop's weight comes on over 0.3 s and slides off over the last 0.6 s
                    ps = po + Ro[:, 0] * S_SCOOP
                    Fw = np.array([0, 0, -M_SCOOP * C.G]) * S4.mj((t - T_SAND_ON) / 0.3) * (1 - S4.mj((t - T_SAND_OFF + 0.6) / 0.6))
                    d.xfrc_applied[o["id"], :3] += Fw; d.xfrc_applied[o["id"], 3:] += np.cross(ps - d.xipos[o["id"]], Fw)
        else:
            o = self.ob["pan"]
            if not o["pinned"] and not self.ties and t < 3.4:    # the pan rests on the sink grid until arm 6 holds it
                z = d.qpos[o["qa"] + 2]; vz = d.qvel[o["da"] + 2]
                if z < Z_GRID + 0.02:
                    d.qfrc_applied[o["da"] + 2] += float(np.clip(4000.0 * (Z_GRID + 0.02 - z) - 60.0 * vz, 0, 30.0))
                    d.qfrc_applied[o["da"]:o["da"] + 2] += -20.0 * d.qvel[o["da"]:o["da"] + 2]

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_arm = -1.0; segst = {}
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
            Rb = d.xmat[self.torso].reshape(3, 3)
            tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1))); yaw = np.degrees(self.yaw_of(Rb))
            p, v, a = ref3(t); pt = d.xpos[self.torso].copy()
            err = float(np.linalg.norm(p - pt)) * 100
            cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
            dax = np.degrees(np.arccos(np.clip(cz, -1, 1)))
            lab = label(t)
            ss = segst.setdefault(lab, dict(err=0.0, tilt=0.0, yaw=0.0, servo=0.0, servo_at="", sat=0, fan=0.0, gripF=0.0, gripT=0.0, slip=0.0))
            if servo.max() > ss["servo"]:
                i, k = np.unravel_index(servo.argmax(), servo.shape); ss["servo_at"] = f"arm{i} j{k}"
            slip = max(abs(d.qpos[qa]) for qa in self.cq.values())
            ss["err"] = max(ss["err"], err); ss["tilt"] = max(ss["tilt"], tilt); ss["yaw"] = max(ss["yaw"], abs(yaw))
            ss["servo"] = max(ss["servo"], float(servo.max())); ss["sat"] = max(ss["sat"], self.sat)
            ss["fan"] = max(ss["fan"], float(self.fan.max())); ss["slip"] = max(ss["slip"], float(np.degrees(slip)))
            ss["gripF"] = max(ss["gripF"], self.grip[HA][0], self.grip[HB][0]); ss["gripT"] = max(ss["gripT"], self.grip[HA][1], self.grip[HB][1])
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo=servo.max(), servo_arm=servo.max(1),
                                   tilt=tilt, yaw=yaw, err=err, pos=pt, ref=p.copy(), vel=d.qvel[0:3].copy(), dax=dax,
                                   adem=self.adem.copy(), sat=self.sat, gate=self.gate.copy(),
                                   fold=np.array([self.fold[HA], self.fold[HB]]), held=np.array([float(any(x["h"] == h for x in self.ties)) for h in (HA, HB)]),
                                   grip=np.array(self.grip[HA] + self.grip[HB]), sand=float(self.sand_on), payload=self.payload_m()))
                nxt += 1 / fps
        print(f"chores {SCN} {CASE}: sim {T_END:.1f} s in {time.time() - t0:.0f} s wall")
        return frames, segst, float(np.sqrt(rms / nrm).max())


ST_BAL = 0.05

if __name__ == "__main__":
    if "plan" in sys.argv:
        plan(verbose=True); sys.exit()
    M = Chores()
    frames, segst, rms = M.run()
    for g in M.log: print("grasp:", g)
    for lab, s in segst.items():
        print(f"{lab:50s} err {s['err']:5.1f} cm | tilt {s['tilt']:4.1f} | yaw {s['yaw']:4.1f} | servo {s['servo']:.2f} ({s['servo_at']}) | "
              f"slip {s['slip']:4.1f} | sat {s['sat']} | fan {s['fan']:4.1f} | grip {s['gripF']:5.1f} N {s['gripT']:.2f} N m")
    print("rms/rated", round(rms, 2))
    keys = list(frames[0].keys())
    np.savez_compressed(f"chores4_{SCN}_{CASE}.npz", **{k: np.array([f[k] for f in frames]) for k in keys})
    json.dump(dict(scn=SCN, case=CASE, segments=segst, rms_rated=rms, grasps=M.log), open(f"chores4_{SCN}_{CASE}.json", "w"), indent=1)
