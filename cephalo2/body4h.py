"""Cephalo-1 rev 4 as one body: an intent-level, event-driven whole-body controller (full-robot MuJoCo model of sim4).

The mission scripts so far each ran their own timeline from their own start state.  Here one controller runs the robot
continuously and is driven by intents ("get up", "walk 0.2 m and turn 30 deg", "take off to 0.8 m", "fly north 1.5 m",
"land", "lie down").  Layers, from the top:
  intent   a queue of skills; each skill starts from wherever the body actually is (pose, velocity, which feet are
           down) and ends on a sensed condition, not a clock.
  skills   get_up, idle, walk (any direction + turn), wave, take_off, fly_to (x, y, z, heading), land, lie_down.
           Ground skills drive the feet through a fast whole-arm IK (feet in world, body pose planned and re-anchored to
           the measured pose at every skill start).  Flight uses tentacle steering (steer4) with a heading reference.
  reflexes * touchdown is sensed: the reaching feet see the ground coming (downward ToF range per tip bay, VL53L4CD
             class; a MuJoCo ray here) and below 10 cm legs + body are re-timed to arrive together; the pair wave starts
             when foot force is felt (toe contact force; real robot: servo load + tip IMU); the touching feet become
             stance feet where they landed and a higher ground is absorbed by lifting the body to stand height over
             them; if the floor is not there the body keeps sinking at 5 cm/s, feeling for it;
           * a load governor slows every arm motion (skill phase rate 1 -> 0.45) when any servo passes 0.80 x stall
             or a claw-prop saturates, and speeds back up when the load drops;
           * lift-off is sensed: air control blends in when the planned stance feet are actually unloaded.
  spinal   every joint command passes a critically damped 2nd-order tracker (16 rad/s) with goal-velocity
           feed-forward and per-arm speed/acceleration limits (120 deg/s, 800 deg/s^2 for a loaded leg, 180 / 1200 in
           the air), so a skill switch can never step or kink a joint command.
  thrust   one law for ground, transition and flight: ground collective (fans carry what the planned feet should not)
           blends into the altitude loop with quintic feed-forward; attitude gains blend 60/8 -> 14/4.5; claw-props
           fade in/out by their claw angle and spin-up time; attitude-first mixing (sim4.allocate).
Senses logged every frame: body pose/velocity, joint angles, servo load per joint, clutch slip, foot force per arm,
belly (EDF bumper) force, fan thrusts and saturation, thrust margin, governor rate.
Run: python3 body4.py [mid|pess] [program]  -> body4_<case>.npz/.json   (program: default 'day')
"""
import os, sys, json, time, numpy as np, mujoco
os.environ.setdefault("CEPH_TAKEOFF", "v7")
import cephalo4 as C
import sim4 as S4
import power4 as PW

D = C.DEG
CASE = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_CASE", "mid")
F_MAX = float(os.environ["CEPH_FMAX"]) if os.environ.get("CEPH_FMAX") else C.FAN_MAX[CASE]
# power system (power4.py): battery sag sets the EDF and claw-prop thrust ceilings.  CEPH_POWER=0 = ideal supply
# (the earlier runs).  Packs beyond the first one (already inside the body mass) add 0.81 kg each.
POWER = bool(int(os.environ.get("CEPH_POWER", "1")))
PACKS = int(os.environ.get("CEPH_PACKS", "1")); SOC0 = float(os.environ.get("CEPH_SOC", "1.0"))
XMASS = (PACKS - 1) * PW.M_PACK + float(os.environ.get("CEPH_XMASS", "0"))
if XMASS:
    S4.M_TORSO += XMASS; C.M_ROBOT += XMASS; C.W_ROBOT = C.M_ROBOT * C.G
# claw-prop variant (power4.PROPS): "std" 6 in on a 2207, "xl" 7.5 in on a 28xx (heavier pod, CEPH_PROP=xl)
PROP = os.environ.get("CEPH_PROP", "std")
PROP_P = PW.PROPS[PROP]
if PROP_P["dM_pod"]:
    C.M_POD += PROP_P["dM_pod"]; C.M_ROBOT += 8 * PROP_P["dM_pod"]; C.W_ROBOT = C.M_ROBOT * C.G
if PROP != "std":
    F_MAX = PW.claw_max(PROP, PW.V_REF)
# prop reaction torque about each claw axis (on by default for xl): the spin pattern balances 8-prop flight and the
# 4-prop quad mode on arms 0/2/4/6 alike
PROP_TORQUE = bool(int(os.environ.get("CEPH_PROPTORQUE", "1" if PROP != "std" else "0")))
SPIN = np.array([1, -1, -1, 1, 1, -1, -1, 1], float)
SIX_FAN_LOAD = (C.W_ROBOT - C.EDF_MAX) / (6 * F_MAX)
DT = S4.DT
CTRL_DT = 0.01
Q_STAND, Q_FLY, H_STAND = S4.Q_STAND, S4.Q_FLY, S4.H_STAND
H_REST = 0.094
SIT = 0.015                                      # crouch target this far below the bumper height, so the feet unload
Q_LIE = np.radians([15.5, 0, 0, 0, 0, 0, -15.5, 0])           # rest4 lying pose (arm straight, sloping to the floor)
Q_MID = np.radians([-45, 0, 0, 0, 40, 0, 10, 0])              # rest4 mid pose (foot lifted and swung out)
LEAD = C.foot_drop()
BO = [C.BLADE_OFF] * 8                           # per-arm blade-plane offset from the spigot (hand arms differ)
HAND = {}                                        # arm -> dict(T=ceiling at V_REF_H, K=W/N^1.5); filled by hand3_fly.py
V_REF_H = 22.2
QT = [None] * 8                                  # per-arm rotor torque / thrust (m); None = the XL pod value
DOWN = np.array([0, 0, -1.0])
YAW_YIELD = os.environ.get("H3_YAWYIELD", "1") != "0"
assert C.RG < 1e-9

# limits (deg -> rad)
V_LOAD, A_LOAD = 120 * D, 800 * D
V_FREE, A_FREE = 180 * D, 1200 * D
W_SPINE = 16.0
# real servo torque-speed line (12 V datasheet): STS3250 (j0) 0.133 s/60 deg = 451 deg/s, STS3215 0.222 s/60 deg = 270 deg/s;
# available speed = no-load speed x (1 - load / stall).  Checked after the run by servo_envelope.py; enforcing it here
# (CEPH_ENVLIMIT=1, ENV_MARGIN of the line) is off by default because clipped stance legs lag their IK and load up.
W_NOLOAD = np.radians([451.0] + [270.0] * 7)
ENV_MARGIN = 0.85
ENV_LIMIT = bool(int(os.environ.get("CEPH_ENVLIMIT", "0")))
FOOT_N = 3.0                                     # foot contact threshold (N)
FAN_BASE = 6.0
FOOT_LOAD = 10.0                                 # target load per planted foot while fans still carry (N)
# flight (steer4 / flight4 values)
KP, KD, KI, A_MAX = 4.0, 3.6, 1.0, 3.0
CDA, RHO = 0.10, 1.2
LEAN_MAX = 40 * D
W_J = np.array([0, 0, 0.35, 0.35, 0.65, 0.65, 1.0, 1.0])
TAU_J = np.array([1, 1, 0.20, 0.20, 0.12, 0.12, 0.06, 0.06])
BREATH_AMP = np.radians([0, 0, 3, 4, 3, 4, 3, 4]); BREATH_T = 3.2
KP_PSI, KD_PSI, MZ_MAX = 6.0, 3.0, 6.0


def mj(u):
    u = np.clip(u, 0, 1); return u ** 3 * (10 - 15 * u + 6 * u * u)


def sstep(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Quintic:
    """x(t) from (x0, v0, a0) at t0 to (x1, 0, 0) at t0 + T; vector-valued.  Starting from the current reference state
    keeps position, velocity and acceleration continuous when a new intent replaces an old one mid-motion."""

    def __init__(self, t0, T, x0, x1, v0=None, a0=None):
        self.scalar = np.ndim(x1) == 0
        x0 = np.atleast_1d(np.asarray(x0, float)); x1 = np.atleast_1d(np.asarray(x1, float))
        v0 = np.zeros_like(x0) if v0 is None else np.atleast_1d(np.asarray(v0, float))
        a0 = np.zeros_like(x0) if a0 is None else np.atleast_1d(np.asarray(a0, float))
        self.t0, self.T = t0, max(T, 1e-3)
        T = self.T
        M = np.array([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4], [6 * T, 12 * T ** 2, 20 * T ** 3]])
        rhs = np.stack([x1 - x0 - v0 * T - a0 * T * T / 2, -v0 - a0 * T, -a0])
        c345 = np.linalg.solve(M, rhs)
        self.c = np.vstack([x0, v0, a0 / 2, c345])

    def at(self, t):
        s = np.clip(t - self.t0, 0, self.T)
        c = self.c
        x = c[0] + c[1] * s + c[2] * s ** 2 + c[3] * s ** 3 + c[4] * s ** 4 + c[5] * s ** 5
        v = c[1] + 2 * c[2] * s + 3 * c[3] * s ** 2 + 4 * c[4] * s ** 3 + 5 * c[5] * s ** 4
        a = 2 * c[2] + 6 * c[3] * s + 12 * c[4] * s ** 2 + 20 * c[5] * s ** 3
        if t - self.t0 >= self.T:
            v = np.zeros_like(v); a = np.zeros_like(a)
        if self.scalar: return float(x[0]), float(v[0]), float(a[0])
        return x, v, a

    def done(self, t):
        return t - self.t0 >= self.T


def mj_time(dist, a_peak):
    """Duration of a minimum-jerk move of length dist with peak acceleration a_peak (peak = 5.77 d / T^2)."""
    return float(np.sqrt(5.7735 * abs(dist) / a_peak))


# ================================================================== fast kinematics (analytic Jacobian)
ROOT = [C.arm_base(i) for i in range(8)]
_PITCH = np.array([t == "p" for t in C.TYP])


def fk(i, q, bp, bR):
    R0, p0 = ROOT[i]
    R = bR @ R0; p = bp + bR @ p0
    cs = np.zeros((8, 3)); ax = np.zeros((8, 3)); s = 0.0
    for k in range(8):
        p = p + R[:, 0] * (C.S_J[k] - s); s = C.S_J[k]
        cs[k] = p; ax[k] = R[:, 1] if _PITCH[k] else R[:, 2]
        c, sn = np.cos(q[k]), np.sin(q[k])
        Rk = np.array([[c, 0, sn], [0, 1, 0], [-sn, 0, c]]) if _PITCH[k] else np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]])
        R = R @ Rk
    p = p + R[:, 0] * (C.SPIGOT - s)
    return cs, ax, R, p


def foot(i, q, bp, bR, lead=LEAD):
    _, _, R, p = fk(i, q, bp, bR)
    return p + R[:, 0] * lead, R[:, 0]


def ik(i, target, axis, q0, q_nom, bp, bR, lead=LEAD, iters=3, w_ax=0.3, w_reg=0.02):
    """Damped Gauss-Newton on the same residual as cephalo4.ik (foot x10, claw axis x w_ax, regularisation), warm-started;
    ~0.2 ms per arm instead of 7 ms."""
    q = q0.copy(); lo, hi = C.LIM[:, 0], C.LIM[:, 1]
    for _ in range(iters):
        cs, ax, R, p = fk(i, q, bp, bR)
        u = R[:, 0]; f = p + u * lead
        Jf = np.cross(ax, f - cs).T                       # 3 x 8 (point moves with the joint about its axis)
        Ju = np.cross(ax, u).T
        rows = [10 * (f - target)]; Js = [10 * Jf]
        if axis is not None:
            rows.append(w_ax * (u - axis)); Js.append(w_ax * Ju)
        rows.append(w_reg * (q - q_nom)); Js.append(w_reg * np.eye(8))
        r = np.concatenate(rows); J = np.vstack(Js)
        dq = -np.linalg.solve(J.T @ J + 1e-6 * np.eye(8), J.T @ r)
        q = np.clip(q + np.clip(dq, -0.3, 0.3), lo, hi)
    return q


F_STAND = np.array([foot(i, Q_STAND, np.array([0, 0, H_STAND]), np.eye(3))[0] for i in range(8)])   # body frame at 0


def stand_feet(xy, yaw, gz=0.0):
    R = Rz(yaw); out = F_STAND.copy(); out[:, 2] += gz
    out[:, :2] = (R[:2, :2] @ F_STAND[:, :2].T).T + xy
    return out


# ================================================================== skills
class Skill:
    name = "skill"

    def __init__(self, *a, **k):
        self.args = a; self.kw = k; self.done = False; self.s = 0.0

    def start(self, B, t): pass

    def step(self, B, t, dt): pass

    def label(self, B): return self.name.upper()


class Idle(Skill):
    """Stand and breathe: the body rises/falls 4 mm and sways 5 mm in a slow circle on fixed feet (or hovers in place)."""
    name = "idle"

    def __init__(self, dur=1.0): super().__init__(); self.dur = dur

    def start(self, B, t):
        self.t0 = t
        if not B.airborne:
            B.anchor_ground(t)
            self.p0 = B.bp.copy()

    def step(self, B, t, dt):
        if B.airborne:
            B.fly_arms(t, dt)
        else:
            ph = 2 * np.pi * (t - self.t0)
            ramp = sstep((t - self.t0) / 0.8)
            B.bp = self.p0 + ramp * np.array([0.005 * np.sin(ph / 5.0), 0.005 * (1 - np.cos(ph / 5.0)), 0.004 * np.sin(ph / 4.2)])
            B.ground_ik(range(8))
        self.done = t - self.t0 >= self.dur

    def label(self, B): return "HOVER" if B.airborne else ("REST" if B.lying else "STAND (breathing)")


class Rest(Skill):
    name = "rest"

    def __init__(self, dur=1.0): super().__init__(); self.dur = dur

    def start(self, B, t): self.t0 = t

    def step(self, B, t, dt):
        self.done = t - self.t0 >= self.dur

    def label(self, B): return "REST (lying on the EDF bumper)"


class GetUp(Skill):
    """From lying: arms roll in, lift and place their feet in a wave around the body (lie-down reversed), then the body
    rises on its eight feet."""
    name = "get up"
    TA, TB = 3.6, 2.6

    def start(self, B, t):
        B.lying = False
        B.anchor_ground(t, feet=False)
        B.feet = stand_feet(B.bp[:2], B.byaw)
        self.q0 = B.q_cmd.copy()
        bpc = np.array([B.bp[0], B.bp[1], H_REST - SIT]); bR = Rz(B.byaw)   # feet placed just touching, unloaded
        self.Qc = np.zeros((8, 8))
        for i in range(8):     # crouch pose: foot at its stand spot with the body still on the bumper
            q = Q_MID.copy()
            for _ in range(8):
                q = ik(i, B.feet[i], DOWN, q, Q_STAND, bpc, bR)
            self.Qc[i] = q
        self.phase = 0; self.sB = 0.0; self.sA = 0.0

    def step(self, B, t, dt):
        if self.phase == 0:
            self.sA += dt * B.rate
            for i in range(8):
                u = S4.trap((self.sA - 0.4 - 0.12 * (7 - i)) / (self.TA - 1.24))
                if u < 0.5: B.goal[i] = self.q0[i] + (Q_MID - self.q0[i]) * mj(u / 0.5)
                else: B.goal[i] = Q_MID + (self.Qc[i] - Q_MID) * mj((u - 0.5) / 0.5)
            Ts, Tf, Tt = C.T_LINE["soft"], C.T_LINE["firm"], C.T_LINE["tight"]
            B.winch_target = Ts + (Tf - Ts) * mj(self.sA / 0.4) + (Tt - Tf) * mj((self.sA - 0.4) / (self.TA - 0.4))
            if self.sA >= self.TA:
                self.phase = 1; B.bp[2] = H_REST - SIT
                B.q_ik = self.Qc.copy()
        else:
            self.sB += dt * B.rate
            B.bp[2] = H_REST - SIT + (H_STAND - H_REST + SIT) * mj(self.sB / self.TB)
            B.ground_ik(range(8))
            if self.sB >= self.TB + 0.3:
                self.done = True

    def label(self, B): return "GET UP: arms roll in, feet placed" if self.phase == 0 else "GET UP: body rises on 8 feet"


class LieDown(Skill):
    """Crouch onto the EDF bumper with the feet where they are, then lay the arms out flat in a wave (rest4)."""
    name = "lie down"
    TA, TB = 3.0, 3.6

    def start(self, B, t):
        B.anchor_ground(t)
        self.z0 = B.bp[2]; self.phase = 0; self.sA = 0.0; self.sB = 0.0

    def step(self, B, t, dt):
        if self.phase == 0:
            self.sA += dt * B.rate
            B.bp[2] = self.z0 + (B.gz + H_REST - SIT - self.z0) * mj(self.sA / self.TA)
            B.ground_ik(range(8))
            if self.sA >= self.TA:
                self.phase = 1; self.q_up = B.goal.copy(); self.f0 = B.feet.copy()
        else:
            self.sB += dt * B.rate
            for i in range(8):
                u = S4.trap((self.sB - 0.12 * i) / (self.TB - 0.84))
                if u < 0.2:                       # foot straight up first (no dragging), then out and down flat
                    B.swing_foot[i] = self.f0[i] + [0, 0, 0.06 * mj(u / 0.2)]; B.ground_ik([i]); self.q_up[i] = B.goal[i].copy()
                elif u < 0.6: B.goal[i] = self.q_up[i] + (Q_MID - self.q_up[i]) * mj((u - 0.2) / 0.4)
                else: B.goal[i] = Q_MID + (Q_LIE - Q_MID) * mj((u - 0.6) / 0.4)
            B.winch_target = C.T_LINE["tight"] + (C.T_LINE["soft"] - C.T_LINE["tight"]) * mj(self.sB / self.TB)
            if self.sB >= self.TB + 0.4:
                self.done = True; B.lying = True

    def label(self, B): return "LIE DOWN: crouch onto the EDF bumper" if self.phase == 0 else "LIE DOWN: arms roll out flat"


class Walk(Skill):
    """Ripple gait in any direction with a turn: body moves on a smooth trapezoid, each foot steps once per cycle to
    where it should be half a cycle later (one foot in the air at a time); a last cycle tidies feet that are > 15 mm
    off their stand spot."""
    name = "walk"
    STRIDE, LIFT, SWING, SLOT = 0.090, 0.045, 0.30, 0.36
    ORDER = [0, 4, 2, 6, 1, 5, 3, 7]

    def __init__(self, fwd=0.0, left=0.0, turn_deg=0.0):
        super().__init__(); self.fwd, self.left, self.turn = fwd, left, np.radians(turn_deg)

    def start(self, B, t):
        B.anchor_ground(t)
        self.p0 = B.bp.copy(); self.y0 = B.byaw
        d_body = Rz(self.y0)[:2, :2] @ np.array([self.fwd, self.left])
        self.dxy = d_body
        dist = np.linalg.norm(d_body)
        self.ncyc = int(max(1, np.ceil(max(dist / self.STRIDE, abs(self.turn) / np.radians(12)))))
        self.Tc = 8 * self.SLOT
        self.Tbody = self.ncyc * self.Tc
        self.Ttot = (self.ncyc + 1) * self.Tc
        self.tau = 0.0
        self.lift = {}                                  # (arm, cycle) -> (foot start world, foot end world) or None
        self.active = [None] * 8

    def body_at(self, tau):
        u = S4.trap(np.clip(tau / self.Tbody, 0, 1), 0.2)
        return self.p0[:2] + self.dxy * u, self.y0 + self.turn * u

    def step(self, B, t, dt):
        self.tau += dt * B.rate
        xy, yaw = self.body_at(self.tau)
        B.bp = np.array([xy[0], xy[1], self.p0[2]]); B.byaw = yaw
        swing = []
        for i in range(8):
            cyc = int(self.tau // self.Tc)
            ts = self.tau - cyc * self.Tc - self.ORDER.index(i) * self.SLOT
            if cyc <= self.ncyc and 0 <= ts < self.SWING:
                key = (i, cyc)
                if key not in self.lift:
                    xy_l, yaw_l = self.body_at(cyc * self.Tc + self.ORDER.index(i) * self.SLOT + 4 * self.SLOT)
                    tgt = stand_feet(xy_l, yaw_l, B.gz)[i]
                    if np.linalg.norm(tgt[:2] - B.feet[i][:2]) < 0.015:
                        self.lift[key] = None
                    else:
                        self.lift[key] = (B.feet[i].copy(), tgt)
                pl = self.lift[key]
                if pl is not None:
                    u = ts / self.SWING
                    f = pl[0] + (pl[1] - pl[0]) * sstep(u); f[2] += self.LIFT * np.sin(np.pi * u)
                    B.swing_foot[i] = f; swing.append(i); self.active[i] = pl
            else:
                if self.active[i] is not None:
                    B.feet[i] = self.active[i][1]; self.active[i] = None
                B.swing_foot[i] = None
        B.ground_ik(range(8))
        if self.tau >= self.Ttot:
            self.done = True

    def label(self, B):
        parts = []
        if abs(self.fwd) > 1e-3: parts.append(f"{abs(self.fwd) * 100:.0f} cm {'forward' if self.fwd > 0 else 'back'}")
        if abs(self.left) > 1e-3: parts.append(f"{abs(self.left) * 100:.0f} cm {'left' if self.left > 0 else 'right'}")
        if abs(self.turn) > 1e-3: parts.append(f"turn {np.degrees(self.turn):+.0f} deg")
        return "WALK: " + ", ".join(parts)


class Wave(Skill):
    """Lift one leg (7 feet stay down), raise the arm and wave it like a tentacle (a travelling yaw/pitch wave from
    root to tip), then put the foot back where it was."""
    name = "wave"
    Q_UP = np.radians([-50, 0, -20, 0, -15, 0, -10, 0])

    def __init__(self, arm=1, dur=3.0): super().__init__(); self.arm, self.dur = arm, dur

    def start(self, B, t):
        B.anchor_ground(t)
        self.tau = 0.0; self.f0 = B.feet[self.arm].copy(); self.p0 = B.bp.copy()
        a = np.radians(22.5 + 45 * self.arm) + B.byaw
        self.shift = -0.025 * np.array([np.cos(a), np.sin(a), 0])      # lean the body away from the waving arm
        self.T_sh, self.T_lift, self.T_up, self.T_dn = 0.5, 0.35, 1.4, 1.6

    def step(self, B, t, dt):
        i = self.arm
        self.tau += dt * B.rate
        T1 = self.T_sh; T2 = T1 + self.T_lift; T3 = T2 + self.T_up; T4 = T3 + self.dur; T5 = T4 + self.T_dn; T6 = T5 + self.T_lift; T7 = T6 + self.T_sh
        tau = self.tau
        sh = mj(tau / T1) * (1 - mj((tau - T6) / self.T_sh))
        B.bp = self.p0 + self.shift * sh
        others = [k for k in range(8) if k != i]
        B.ground_ik(others)
        bR = Rz(B.byaw)
        if tau < T1 or tau >= T6:                                   # weight shift (foot planted)
            B.swing_foot[i] = None; B.ground_ik([i])
        elif tau < T2:                                              # foot straight up 6 cm
            B.swing_foot[i] = self.f0 + [0, 0, 0.06 * mj((tau - T1) / self.T_lift)]; B.ground_ik([i])
            self.q_lift = B.goal[i].copy()
        elif tau < T3:
            B.goal[i] = self.q_lift + (self.Q_UP - self.q_lift) * mj((tau - T2) / self.T_up)
        elif tau < T4:
            s = tau - T3; Tw = self.dur
            env = sstep(s / 0.5) * sstep((Tw - s) / 0.5)
            k = np.arange(8)
            # 0.8 Hz: peak joint speed 2 pi f A = 110 deg/s on j7, under 0.85 x the STS3215 line at the ~0.35 x stall it carries
            yawj = np.radians([0, 0, 0, 14, 0, 18, 0, 22]) * np.sin(2 * np.pi * 0.8 * s - 0.9 * k)
            pitj = np.radians([0, 0, 4, 0, 6, 0, 8, 0]) * np.sin(2 * np.pi * 0.8 * s - 0.9 * k - 1.2)
            B.goal[i] = self.Q_UP + env * (yawj + pitj)
            B.q_ik[i] = ik(i, self.f0 + [0, 0, 0.06], DOWN, self.q_lift, Q_STAND, B.bp, bR)
        elif tau < T5:
            B.goal[i] = self.Q_UP + (B.q_ik[i] - self.Q_UP) * mj((tau - T4) / self.T_dn)
        else:                                                       # foot down onto its old spot
            B.swing_foot[i] = self.f0 + [0, 0, 0.06 * (1 - mj((tau - T5) / self.T_lift))]; B.ground_ik([i])
        if tau >= T7:
            B.swing_foot[i] = None
            self.done = True

    def label(self, B): return f"WAVE: arm {self.arm} up, tentacle wave"


class TakeOff(Skill):
    """Take-off v7, event-driven: EDF spools; once it carries ~70 % the leg pairs unfurl in a rotating wave (each from
    its actual start pose); the fans that are up carry what the planned stance feet should not; lift-off when the
    last pair is half way and the stance feet are sensed unloaded; legs 0 + 4 unfurl together (opposite arms: their
    mass shifts cancel, so no differential thrust is needed; the one-after-another fallback is only used above a
    97 % six-claw-prop load, CEPH_FASTLIM); climb to the requested height; tentacle steering takes over."""
    name = "take off"
    PAIRS = [(1, 5), (2, 6), (3, 7)]

    def __init__(self, h=0.80): super().__init__(); self.h = h

    def start(self, B, t):
        B.anchor_ground(t)
        self.t0 = t; self.tau = 0.0
        self.q0 = B.goal.copy()
        # self-check before committing: thrust ceilings at the voltage the battery will sag to under take-off load
        six, eight = B.load_fraction(6), B.load_fraction(8)
        self.refused = eight > 0.92 or six > 1.0
        self.check = (six, eight)
        if self.refused:
            self.done = True; B.refused = True
            print(f"TAKE OFF REFUSED: hover on 8 claw-props + EDF would need {eight * 100:.0f} %, the 6-claw-prop moments {six * 100:.0f} %")
            return
        self.u = np.zeros(8); self.t_h = None; self.fast = six <= float(os.environ.get("CEPH_FASTLIM", 0.97))
        self.sw = S4.T_SW
        if self.fast: self.sw04 = S4.T_SW
        else:
            uu = np.linspace(0, 1, 2001); qq = np.degrees(np.array([S4._path_v7(u) for u in uu]))
            dq = np.gradient(qq, uu, axis=0); ddq = np.gradient(dq, uu, axis=0)
            self.sw04 = float(np.ceil(20 * max(np.abs(dq).max() / 120, np.sqrt(np.abs(ddq).max() / 800))) / 20)
        self.start_pair = [None] * 3; self.t04 = None
        B.ground_collective = True; B.fans_on = True; B.unload = 0.0
        self.xy_hold = B.bp[:2].copy()

    def path(self, i, u):
        return S4._path_v7(u) + (self.q0[i] - Q_STAND) * (1 - mj(u / 0.3))

    def step(self, B, t, dt):
        el = t - self.t0
        B.edf_target = C.EDF_MAX * mj(el / 0.6)
        B.winch_target = C.T_LINE["tight"] + (C.T_LINE["firm"] - C.T_LINE["tight"]) * mj(el / 0.5)
        self.tau += dt * B.rate
        # pairs: first once the EDF carries ~70 %, then 0.4 s apart (in governed time)
        if B.steering:
            B.fly_arms(t, dt)
            if B.zplan.done(t):
                B.ground_collective = False; B.unload = 0.0; B.n_down_plan = None
                self.done = True
            return
        for p, (a, b) in enumerate(self.PAIRS):
            if self.start_pair[p] is None and el >= 0.45 and (p == 0 or (self.start_pair[p - 1] is not None and self.tau - self.start_pair[p - 1] >= 0.4)):
                self.start_pair[p] = self.tau
            if self.start_pair[p] is not None:
                for i in (a, b):
                    self.u[i] = min((self.tau - self.start_pair[p]) / self.sw, 1.0)
                    B.goal[i] = self.path(i, self.u[i])
        stance = [i for i in range(8) if self.u[i] < 0.15]
        B.n_down_plan = len(stance)
        sw_now = [i for i in range(8) if 0 < self.u[i] < 1]
        B.ground_ik([i for i in (0, 4) if self.u[i] == 0])
        # lift-off when the last pair is 45 % through, after a 0.6 s unload (sensed: the stance feet carry < 25 N)
        if self.start_pair[2] is not None and self.t_h is None:
            if self.u[3] >= 0.45:
                if not hasattr(self, "t_un"): self.t_un = t
                B.unload = mj((t - self.t_un) / 0.6)
                fz = B.foot_f[[0, 4]].sum()
                if t - self.t_un >= 0.6 and fz < 25.0:
                    self.t_h = t; self.z_h = B.d.xpos[B.torso][2]
                    B.air_from(t, 0.4)
        if self.t_h is not None:
            th = t - self.t_h
            if self.t04 is None and th >= (0.3 if self.fast else 0.5):
                self.t04 = t
            if self.t04 is not None:
                # airborne legs run in real time, not governed: finishing the unfurl is what brings the missing
                # claw-props back, so slowing it would only stretch the short-handed phase
                for i, delay in ((4, 0.0), (0, 0.0 if self.fast else 1.2)):
                    self.u[i] = float(np.clip((t - self.t04 - delay) / self.sw04, 0, 1))
                    if self.u[i] > 0: B.goal[i] = self.path(i, self.u[i])
                    else: B.goal[i] = B.q_ik[i]
            if self.t04 is not None and not hasattr(self, "zplan_set") and self.u[0] >= 0.35:
                z = B.d.xpos[B.torso][2]
                B.zplan = Quintic(t, max(2.0, mj_time(self.h - z, 0.9)), z, self.h, B.zref_state(t)[1], B.zref_state(t)[2])
                self.zplan_set = True
            elif not hasattr(self, "zplan_set"):
                B.zplan = Quintic(self.t_h, 0.6, self.z_h, self.z_h + 0.04)
            if self.u.min() >= 1.0 and hasattr(self, "zplan_set"):
                B.start_steering(t)

    def label(self, B):
        if getattr(self, "refused", False): return "TAKE OFF REFUSED: not enough thrust"
        if self.t_h is None: return "TAKE OFF: arms unfurl in a spiral wave"
        if self.u.min() < 1: return "TAKE OFF: lift-off, last arms unfurl"
        return f"TAKE OFF: climb to {self.h:.2f} m"


class FlyTo(Skill):
    """Fly to (x, y, z) with heading psi (deg); relative by default.  Quintic references starting from the current
    reference state; the trailing arms lean their claw-props (tentacle steering), the body stays level."""
    name = "fly"

    def __init__(self, dx=0.0, dy=0.0, dz=0.0, dpsi=0.0, text=None):
        super().__init__(); self.dx, self.dy, self.dz, self.dpsi, self.text = dx, dy, dz, np.radians(dpsi), text

    def start(self, B, t):
        if not B.airborne:
            self.done = True; self.t1 = t; return
        p, v, a = B.xyplan.at(t); z, vz, az = B.zplan.at(t); y, vy, ay = B.yawplan.at(t)
        p1 = p + np.array([self.dx, self.dy]); z1 = z + self.dz; y1 = y + self.dpsi
        T = max(1.6, mj_time(np.hypot(self.dx, self.dy), 1.4), mj_time(self.dz, 0.8), mj_time(self.dpsi, 0.9))
        B.xyplan = Quintic(t, T, p, p1, v, a); B.zplan = Quintic(t, T, z, z1, vz, az); B.yawplan = Quintic(t, T, y, y1, vy, ay)
        B.flare = -0.05 * np.sign(self.dz) if abs(self.dz) > 0.05 else 0.0
        self.t1 = t + T + 0.4

    def step(self, B, t, dt):
        if not B.airborne: self.done = True; return
        B.fly_arms(t, dt)
        if t > self.t1 - 0.4: B.flare = 0.0
        self.done = t >= self.t1

    def label(self, B): return "FLY: " + (self.text or f"{self.dx:+.1f} / {self.dy:+.1f} / {self.dz:+.1f} m")


class Land(Skill):
    """Organic landing with a sensed touchdown: one quintic descent toward 2 cm below the stand datum (thin thrust
    margin: sink on 8 claw-props to 15 cm above it first); legs 0 + 4 reach down (take-off unfurl reversed) while the
    body sinks; their ToF ranges trigger a final approach that re-times legs and body to arrive together; on felt
    contact those feet become stance feet, the three pairs follow in a rotating wave (3+7 -> 2+6 -> 1+5, 0.4 s apart)
    guided onto the real ground by IK for the last quarter, the claw-props still up carry what the landed feet
    (latched, not raw contact) should not, flight hands over to ground over 1.2 s, the body stands up to full height
    and the EDF spools down.  If the floor is not where expected the body keeps sinking at 5 cm/s until it is felt."""
    name = "land"
    PAIRS = [(3, 7), (2, 6), (1, 5)]

    def start(self, B, t):
        self.t0 = t
        if not B.airborne:
            self.done = True; self.skip = True; return
        z, vz, az = B.zplan.at(t)
        self.zt = H_STAND + 0.05 - 0.02
        self.thin = B.load_fraction(6) > 0.85
        if not self.thin:          # one descent; legs 0 + 4 reach down during it
            self.Td = 2.75; self.lead = 0.3
            B.zplan = Quintic(t, self.Td, z, self.zt, vz, az)
        else:                      # thin margin: sink on 8 claw-props to 15 cm above touch, then reach and settle slowly
            T1 = max(1.5, mj_time(z - self.zt - 0.15, 0.6))
            self.z1, self.T1 = self.zt + 0.15, T1
            B.zplan = Quintic(t, T1, z, self.z1, vz, az)
            self.lead = T1; self.Td = T1 + 2.6
        p, v, a = B.xyplan.at(t)
        B.xyplan = Quintic(t, 1.0, p, p + v * 0.3, v, a)
        self.u = np.zeros(8); self.t_touch = None; self.tau = 0.0; self.t_all = None; self.search = False
        self.sw04 = 2.2
        self.q_air = None; self.q_air0 = B.goal.copy()
        B.edf_target = C.EDF_MAX

    def step(self, B, t, dt):
        if getattr(self, "skip", False): self.done = True; return
        el = t - self.t0
        B.edf_target = C.EDF_MAX
        self.tau += dt * (B.rate if self.t_touch is not None else 1.0)
        # final approach: the reaching feet see the ground coming (ToF); legs and body are re-timed so the feet arrive
        # together with the body at a gentle press, wherever the ground actually is
        if self.t_touch is None and not getattr(self, "final", False) and self.u[0] > 0.3:
            h = min(B.foot_range(0), B.foot_range(4))
            self.h_seen = h
            if h < 0.10:
                self.final = True; self.t_f = t; self.u0 = self.u[0]
                fz = lambda q: foot(0, q, np.zeros(3), np.eye(3))[0][2]
                ext = fz(S4._path_v7(1 - self.u0)) - fz(S4._path_v7(0.0))          # foot travel the leg path still adds
                drop = max(h - ext + 0.008, 0.0)
                z, vz, az = B.zplan.at(t)
                zm = B.d.xpos[B.torso][2]
                self.T_f = float(np.clip(1.4 * (1 - self.u0) * self.sw04, 0.6, 1.4))
                B.zplan = Quintic(t, self.T_f, zm, zm - drop, B.d.qvel[2], 0.0)
        if self.thin and self.t_touch is None and el >= self.T1 and not getattr(self, "stage2", False) and not getattr(self, "final", False):
            self.stage2 = True
            z, vz, az = B.zplan.at(t)
            B.zplan = Quintic(t, 2.6, z, self.zt, vz, az)
        for i in (0, 4):
            if getattr(self, "final", False) and self.t_touch is None:
                self.u[i] = self.u0 + (1 - self.u0) * mj((t - self.t_f) / self.T_f)
            elif self.t_touch is None:
                self.u[i] = float(np.clip((el - self.lead) / self.sw04, 0, 1))
            if self.u[i] > 0 and self.t_touch is None:
                B.override[i] = S4._path_v7(1 - self.u[i]) + (self.q_air0[i] - Q_FLY) * (1 - mj(self.u[i] / 0.3))
                B.gate[i] = 0.0 if self.u[i] > 0.5 else 1.0
        # touchdown reflex: legs 0 + 4 feel the floor
        if self.t_touch is None:
            touch = (B.foot_f[0] > FOOT_N or B.foot_f[4] > FOOT_N) and self.u[0] > 0.6
            if touch:
                self.t_touch = t; self.tau = 0.0
                z = B.d.xpos[B.torso][2]
                B.zplan = Quintic(t, 0.4, z, z - 0.02, B.d.qvel[2], 0.0)    # altitude loop keeps flying, pressing 2 cm
                # the touching feet become stance feet where they actually are; the body settles to stand height
                # above them (a higher or lower floor than expected is absorbed here, not by the leg path)
                B.anchor_ground(t, feet=True)
                fm = B.measured_feet(); B.gz = float(fm[[0, 4], 2].mean()); B.feet[[0, 4]] = fm[[0, 4]]
                # if it touched low (ground higher than expected) the fans lift the body to stand height over the
                # touching feet while the pairs come down; altitude reference 2 cm under the leg plan = gentle press
                if z < B.gz + H_STAND - 0.01:
                    zt = B.gz + H_STAND - 0.005
                    self.zb = Quintic(t, 1.2, z, zt, B.d.qvel[2], 0.0)
                    B.zplan = Quintic(t, 1.2, z, zt - 0.02, B.d.qvel[2], 0.0)
                else:
                    self.zb = Quintic(t, 0.5, z, z, B.d.qvel[2], 0.0)
                self.ik_on = [False] * 8
                self.air_off = False
                B.ground_collective = True; B.n_down_plan = 2         # feet that have landed (latched, not raw contact: no bounce)
                B.stop_steering(t)
                self.q_air = B.goal.copy()
            elif B.zplan.done(t) and (el > self.Td + 0.4 or getattr(self, "final", False)):   # floor not there yet: keep sinking, feeling for it
                self.search = True
                z = B.d.xpos[B.torso][2]
                B.zplan = Quintic(t, 0.2, z, z - 0.01, -0.05, 0.0)
            if not B.steering:
                pass
            else:
                B.fly_arms(t, dt)
            return
        # after touchdown: the pair wave; each foot is guided onto the actual ground for the last quarter of its path
        if self.t_all is None:
            B.bp[2] = self.zb.at(t)[0]
        if not self.air_off and self.u[3] >= 0.5:      # flight -> ground hand-off while the pairs come down
            B.air_to(t, 0.0, 1.2); self.air_off = True
        B.ground_ik([0, 4])
        bR = Rz(B.byaw); spots = stand_feet(B.bp[:2], B.byaw, B.gz)
        for p, (a, b) in enumerate(self.PAIRS):
            for i in (a, b):
                self.u[i] = float(np.clip((self.tau - 0.4 * p) / S4.T_SW, 0, 1))
                qp = (S4._path_v7(1 - self.u[i]) + (self.q_air[i] - Q_FLY) * (1 - mj(self.u[i] / 0.3))) if self.u[i] > 0 else self.q_air[i]
                if self.u[i] >= 0.75:
                    if not self.ik_on[i]: B.q_ik[i] = qp.copy(); self.ik_on[i] = True; B.feet[i] = spots[i]
                    B.q_ik[i] = ik(i, spots[i], DOWN, B.q_ik[i], Q_STAND, B.bp, bR)
                    wv = mj((self.u[i] - 0.75) / 0.25)
                    B.goal[i] = qp * (1 - wv) + B.q_ik[i] * wv
                else:
                    B.goal[i] = qp
        B.override = [None] * 8
        B.n_down_plan = 2 + int(sum(1 for i in range(8) if i not in (0, 4) and self.u[i] > 0.85))
        if min(self.u[i] for i in range(8) if i not in (0, 4)) >= 1.0 and self.t_all is None:
            self.t_all = t
            zb = B.bp[2]; B.anchor_ground(t, feet=True); B.bp[2] = zb
        if self.t_all is not None:          # all eight feet down: stand up to full height, EDF spools down
            e = t - self.t_all
            if not hasattr(self, "zr2"): self.zr2 = Quintic(t, 1.0, B.bp[2], B.gz + H_STAND)
            B.bp[2] = self.zr2.at(t)[0]
            B.edf_target = C.EDF_MAX * (1 - mj((e - 0.4) / 1.0))
            B.winch_target = C.T_LINE["firm"] + (C.T_LINE["tight"] - C.T_LINE["firm"]) * mj((e - 0.6) / 0.5)
            if e >= 1.4: B.fans_on = False; B.ground_collective = False
            B.ground_ik(range(8))
            if e >= 1.6:
                self.done = True

    def label(self, B):
        if getattr(self, "skip", False): return "LAND: (on the ground)"
        if self.t_touch is None:
            if self.search: return "LAND: searching for the floor"
            if getattr(self, "final", False): return "LAND: feet see the ground, final approach"
            return "LAND: sink, legs 0 + 4 reach down"
        if self.t_all is None: return "LAND: touch felt, legs settle in a wave"
        return "LAND: EDF spools down"


SKILLS = dict(idle=Idle, rest=Rest, get_up=GetUp, lie_down=LieDown, walk=Walk, wave=Wave, take_off=TakeOff, fly=FlyTo, land=Land)

PROGRAMS = {
    "day": [("rest", 1.0), ("get_up",), ("idle", 1.2), ("walk", 0.18, 0.0, 30.0), ("wave", 1, 2.6), ("idle", 0.6),
            ("take_off", 0.80), ("fly", 0.0, 1.5, 0.3, 0.0, "north 1.5 m, climb 0.3 m"),
            ("fly", 1.5, 0.0, -0.3, 90.0, "east 1.5 m, descend 0.3 m, turn 90 deg"), ("idle", 0.8), ("land",),
            ("idle", 1.0), ("lie_down",), ("rest", 1.0)],
    "step": [("idle", 0.5), ("take_off", 0.80), ("fly", 0.0, 1.6, 0.1, 0.0, "north 1.6 m, over a 12 cm step"), ("land",), ("idle", 1.5)],
    "liftoff": [("idle", 0.5), ("take_off", 0.80), ("idle", 2.0), ("land",), ("idle", 1.0)],
    "short": [("idle", 0.5), ("take_off", 0.80), ("fly", 0.0, 0.8, 0.0, 0.0, "north 0.8 m"), ("land",), ("idle", 1.0)],
    "ground": [("rest", 0.5), ("get_up",), ("idle", 1.0), ("walk", 0.18, 0.0, 30.0), ("wave", 1, 2.6), ("idle", 0.5),
               ("lie_down",), ("rest", 0.5)],
}


# ================================================================== the body
class Body(S4.Sim):
    def __init__(self, program, start="lying"):
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
        self.toe = {m.geom(f"toe{i}_{f}").id: i for i in range(8) for f in range(3)}
        self.floor = m.geom("floor").id
        self.ground_ids = {self.floor} | {g for g in range(m.ngeom) if (m.geom(g).name or "").startswith("ground")}
        self.gz = 0.0                                    # ground height under the feet (estimated at each anchor)
        for g in range(m.ngeom):                         # robot geoms in group 3 so the foot ToF rays only see the world
            if m.geom_bodyid[g] != 0: m.geom_group[g] = 3
        self.ray_groups = np.array([1, 1, 1, 0, 0, 0], np.uint8)
        self.belly_ids = {m.geom("edf").id, m.geom("plate").id}
        q0 = Q_LIE if start == "lying" else Q_STAND
        z0 = H_REST + 0.004 if start == "lying" else H_STAND + 0.004
        for i in range(8):
            for k in range(C.NJ):
                d.qpos[self.ja[i, k]] = q0[k] / 2; d.qpos[self.jb[i, k]] = q0[k] / 2
        d.qpos[2] = z0; d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]
        d.ctrl[:8 * C.NJ] = np.tile(q0, 8)
        mujoco.mj_forward(m, d)
        # motor layer
        self.q_cmd = np.tile(q0, (8, 1)); self.qd_cmd = np.zeros((8, 8)); self.goal = self.q_cmd.copy(); self.goal_prev = self.goal.copy()
        self.q_ik = self.q_cmd.copy(); self.env_clip = 0.0
        # body plan / ground
        self.bp = np.array([0, 0, z0]); self.byaw = 0.0; self.feet = stand_feet(np.zeros(2), 0.0); self.swing_foot = {}
        self.lying = start == "lying"
        # thrust / flight state
        self.fan = np.zeros(8); self.edf = 0.0; self.edf_target = 0.0
        self.winch_target = C.T_LINE["soft"] if self.lying else C.T_LINE["tight"]; self.T_line = self.winch_target
        self.air = 0.0; self.air_plan = None; self.airborne = False; self.steering = False; self.steer_w = 0.0
        self.fans_on = False; self.ground_collective = False; self.unload = 0.0; self.n_down_plan = None
        self.gate = np.ones(8); self.gate_s = np.ones(8); self.override = [None] * 8; self.flare = 0.0
        self.zplan = Quintic(0, 1e-3, z0, z0); self.xyplan = Quintic(0, 1e-3, np.zeros(2), np.zeros(2)); self.yawplan = Quintic(0, 1e-3, 0.0, 0.0)
        self.F_prev = None; self.t_in = [None] * 8; self.sat = 0; self.wgt = np.zeros(8)
        self.I = np.zeros(2); self.q_geo = np.tile(Q_FLY, (8, 1)); self.dq_air = np.zeros((8, 8))
        self.Wd = np.diag(W_J)
        # controller gains (trainable; defaults = the hand-tuned steer4 / sim4 values)
        self.g = dict(kp_xy=KP, kd_xy=KD, ki_xy=KI, a_max=A_MAX, lean_max=LEAN_MAX, kp_psi=KP_PSI, kd_psi=KD_PSI,
                      kp_att=14.0, kd_att=4.5, ki_att=0.0, kp_z=4.0, kd_z=3.5, tau_scale=1.0, mz_alloc=1.0)
        self.iatt = np.zeros(2); self.payload = 0.0
        # senses
        self.foot_f = np.zeros(8); self.belly_f = 0.0; self.load = np.zeros((8, 8)); self.load_f = 0.0; self.rate = 1.0
        # power
        self.bat = PW.Battery(PACKS, SOC0) if POWER else None
        self.fmax_eff = F_MAX; self.edf_cap = C.EDF_MAX; self.refused = False
        self.fmax_arm = np.full(8, F_MAX); self.k_ratio = np.array([(HAND[i]["K"] / PROP_P["K"]) ** (2 / 3) if i in HAND else 1.0 for i in range(8)])
        # intent queue
        self.program = [SKILLS[c[0]](*c[1:]) for c in program]
        self.cur = None; self.log_skills = []

    # ---------------------------------------------------------------- power self-model
    def load_fraction(self, n_fans):
        """Fraction of the summed thrust ceilings needed to hover on n claw-props (6: arms 1,2,3,5,6,7) + EDF, at the
        voltage the battery would sag to (per-arm ceilings: hand arms differ from the XL pods)."""
        arms = list(range(8)) if n_fans == 8 else [1, 2, 3, 5, 6, 7]
        kr = np.array([self.k_ratio[i] for i in arms])
        b = PW.Battery(self.bat.n, self.bat.soc)
        f = (C.W_ROBOT - C.EDF_MAX) / n_fans; e = C.EDF_MAX
        for _ in range(20):
            b.step(e, np.full(n_fans, f) * kr, np.full((8, 8), 0.25), 0.0, k_claw=PROP_P["K"])
            e = min(C.EDF_MAX, b.edf_max())
            cap = sum(HAND[i]["T"] * min(1.0, (b.V / V_REF_H) ** 1.5) if i in HAND else PW.claw_max(PROP, b.V) for i in arms)
            f = (C.W_ROBOT - e) / n_fans
        return (C.W_ROBOT - e) / cap

    # ---------------------------------------------------------------- senses
    def sense(self):
        m, d = self.m, self.d
        f = np.zeros(8); belly = 0.0; out = np.zeros(6)
        for j in range(d.ncon):
            c = d.contact[j]
            g1, g2 = c.geom1, c.geom2
            if g1 in self.ground_ids: g = g2
            elif g2 in self.ground_ids: g = g1
            else: continue
            if g in self.toe or g in self.belly_ids:
                mujoco.mj_contactForce(m, d, j, out)
                if g in self.toe: f[self.toe[g]] += abs(out[0])
                else: belly += abs(out[0])
        self.foot_f = f; self.belly_f = belly
        self.load = np.abs(d.actuator_force[:8 * C.NJ].reshape(8, C.NJ)) / C.STALL
        self.q_meas = 2 * d.qpos[self.ja]

    def foot_range(self, i):
        """Downward time-of-flight range from foot i to whatever is below it (a VL53L4CD-class ToF sensor in the tip
        bay, looking down past the folded fingers; modelled as a MuJoCo ray, 2 cm below the foot point)."""
        p, yaw, R = self.body_pose()
        f, _ = foot(i, self.q_meas[i], p, R)
        gid = np.zeros(1, np.int32)
        r = mujoco.mj_ray(self.m, self.d, f - [0, 0, 0.02], np.array([0, 0, -1.0]), self.ray_groups, 1, -1, gid)
        return np.inf if r < 0 else r + 0.02

    def body_pose(self):
        d = self.d
        R = d.xmat[self.torso].reshape(3, 3)
        return d.xpos[self.torso].copy(), float(np.arctan2(R[1, 0], R[0, 0])), R

    def measured_feet(self):
        p, yaw, R = self.body_pose()
        return np.array([foot(i, self.q_meas[i], p, R)[0] for i in range(8)])

    # ---------------------------------------------------------------- ground helpers
    def anchor_ground(self, t, feet=True):
        """Re-anchor the ground plan on the measured body pose (and the measured feet): a new skill starts exactly where
        the body is, so its first IK solution equals the current joint command."""
        p, yaw, R = self.body_pose()
        self.bp = p.copy(); self.byaw = yaw
        if feet:                       # every foot near the floor is taken where it actually is
            fm = self.measured_feet()
            down = self.foot_f > FOOT_N
            if down.sum() >= 3: self.gz = float(np.median(fm[down, 2]))
            near = (fm[:, 2] < self.gz + 0.03) | down
            self.feet[near] = fm[near]
        self.q_ik = self.goal.copy()
        self.swing_foot = {}

    def ground_ik(self, arms):
        bR = Rz(self.byaw)
        for i in arms:
            tgt = self.swing_foot.get(i)
            if tgt is None: tgt = self.feet[i]
            self.q_ik[i] = ik(i, tgt, DOWN, self.q_ik[i], Q_STAND, self.bp, bR)
            self.goal[i] = self.q_ik[i]

    # ---------------------------------------------------------------- flight helpers
    def air_from(self, t, T):
        self.air_plan = (t, T, self.air, 1.0); self.airborne = True
        z = self.d.xpos[self.torso][2]
        self.zplan = Quintic(t, 1e-3, z, z)
        p = self.d.subtree_com[self.torso][:2].copy()
        self.xyplan = Quintic(t, 1e-3, p, p); y = self.body_pose()[1]; self.yawplan = Quintic(t, 1e-3, y, y)

    def air_to(self, t, a1, T):
        self.air_plan = (t, T, self.air, a1)

    def zref_state(self, t):
        return self.zplan.at(t)

    def start_steering(self, t):
        self.steering = True; self.t_steer = t
        self.q_geo = self.goal.copy(); self.dq_air = self.goal - Q_FLY
        p = self.d.subtree_com[self.torso][:2].copy()
        xy, _, _ = self.xyplan.at(t)
        self.xyplan = Quintic(t, 1.2, p, xy)            # take over gently from where the body drifted to

    def stop_steering(self, t):
        self.steering = False

    def axis_jac(self, i, q):
        cs, ax, R, p = fk(i, q, np.zeros(3), np.eye(3))
        a0 = R[:, 0]; h0 = p + a0 * BO[i]
        J = np.cross(ax, a0).T; Jp = np.cross(ax, h0 - cs).T
        J[:, :2] = 0; Jp[:, :2] = 0
        return a0, J, Jp

    def fly_arms(self, t, dt):
        """Tentacle steering (steer4) with a heading reference and per-arm overrides/gates."""
        if not self.steering:
            return
        d = self.d
        Rb = d.xmat[self.torso].reshape(3, 3); com = d.subtree_com[self.torso]
        p, v, a = self.xyplan.at(t)
        e = p - com[:2]
        g_ = self.g
        self.I = np.clip(self.I + g_["ki_xy"] * e * dt, -1.0, 1.0)
        vel = d.qvel[0:2]
        acc = a + g_["kp_xy"] * e + g_["kd_xy"] * (v - vel) + self.I + 0.5 * RHO * CDA * np.linalg.norm(vel) * vel / C.M_ROBOT
        n = np.linalg.norm(acc)
        if n > g_["a_max"]: acc *= g_["a_max"] / n
        Fh = (C.M_ROBOT + self.payload) * acc
        wW = Rb @ d.qvel[3:6]
        yr, vyr, _ = self.yawplan.at(t)
        yaw = float(np.arctan2(Rb[1, 0], Rb[0, 0]))
        Mz = float(np.clip(g_["kp_psi"] * wrap(yr - yaw) + g_["kd_psi"] * (vyr - wW[2]), -MZ_MAX, MZ_MAX))
        self.Mz_dem = Mz
        hub = np.array([d.xpos[self.pod[i]] + d.xmat[self.pod[i]].reshape(3, 3)[:, 0] * BO[i] for i in range(8)])
        r = hub[:, :2] - com[:2]; rn = np.linalg.norm(r, axis=1); rh = r / rn[:, None]
        tan = np.stack([-rh[:, 1], rh[:, 0]], 1)
        f = np.maximum(self.fan, 2.0)
        g = (self.gate > 0.5) & np.array([o is None for o in self.override])
        if os.environ.get("H3_HANDLEAN", "0") == "0": g = g & np.array([i not in HAND for i in range(8)])      # hand arms stay upright (heavy pods): the other six lean
        Fn = np.linalg.norm(Fh)
        u = Fh / Fn if Fn > 1e-6 else np.zeros(2)
        w = (np.clip(-(rh @ u), 0, 1) ** 1.5 + 0.05) * g
        lean = (w / max((f * w).sum(), 1e-6))[:, None] * Fh[None, :]
        Mz_lean = Mz * (1 - g_["mz_alloc"]) if PROP_TORQUE else Mz     # with prop torque, part of yaw goes to differential thrust
        lean = lean + (tan * Mz_lean / max((f * rn * g).sum(), 1e-6)) * g[:, None]
        ln = np.linalg.norm(lean, axis=1)
        lean *= (np.minimum(ln, np.sin(g_["lean_max"])) / np.maximum(ln, 1e-9))[:, None]
        self.lean = np.degrees(np.arcsin(np.minimum(np.linalg.norm(lean, axis=1), 1)))
        ph = 2 * np.pi * t / BREATH_T
        ramp = sstep((t - self.t_steer) / 0.8)
        for i in range(8):
            if self.override[i] is not None:
                self.q_geo[i] = self.override[i]; self.dq_air[i] = self.override[i] - Q_FLY
                self.goal[i] = self.override[i]
                continue
            lb = Rb.T @ np.array([lean[i, 0], lean[i, 1], 0.0]) * ramp
            s = np.linalg.norm(lb[:2])
            a_des = np.array([lb[0], lb[1], np.sqrt(max(1 - s * s, 0))])
            q = self.q_geo[i]
            a0, J, Jp = self.axis_jac(i, q)
            JW = J @ self.Wd
            step = self.Wd @ J.T @ np.linalg.solve(JW @ J.T + 1e-3 * np.eye(3), a_des - a0)
            br = ramp * BREATH_AMP * np.sin(ph - 2 * np.pi * i / 8 - 0.7 * np.arange(8))
            Pn = self.Wd @ J.T @ np.linalg.pinv(JW @ J.T, rcond=1e-4)
            Nn = np.eye(8) - Pn @ J
            nb = Nn @ br
            if abs(self.flare) > 1e-4:
                rb = np.array([np.cos(np.radians(22.5 + 45 * i)), np.sin(np.radians(22.5 + 45 * i)), 0.0])
                jr = rb @ Jp @ Nn @ self.Wd
                nb = nb + Nn @ self.Wd @ jr * (self.flare / max(jr @ jr, 1e-9))
            self.q_geo[i] = np.clip(q + np.clip(step, -0.15, 0.15), C.LIM[:, 0], C.LIM[:, 1])
            tgt = self.q_geo[i] + nb
            q_base = getattr(self, "q_base", None)
            qb = Q_FLY if q_base is None else q_base[i]
            self.dq_air[i] += (tgt - qb - self.dq_air[i]) * np.minimum(1, dt / (TAU_J * g_["tau_scale"]))
            self.goal[i] = qb + self.dq_air[i]

    @staticmethod
    def allocate_n(A, b, w, fmax):
        """sim4.allocate (attitude first, collective gives way) generalised to any number of moment rows."""
        Wd = np.diag(np.maximum(w, 1e-3))
        P = Wd @ np.linalg.pinv(A @ Wd)
        e1 = np.zeros(len(b)); e1[0] = 1.0
        bm = b.copy(); bm[0] = 0.0
        u = P @ e1; Fm = P @ bm
        def c_range(al):
            lo, hi = -np.inf, np.inf
            for ui, fi, mx in zip(u, al * Fm, fmax):
                if abs(ui) < 1e-9:
                    if fi < -1e-9 or fi > mx + 1e-9: return None
                    continue
                a1, a2 = (0 - fi) / ui, (mx - fi) / ui
                lo, hi = max(lo, min(a1, a2)), min(hi, max(a1, a2))
            return (lo, hi) if lo <= hi else None
        al, r = 1.0, c_range(1.0)
        if r is None:
            a_lo, a_hi = 0.0, 1.0
            for _ in range(20):
                mm = 0.5 * (a_lo + a_hi)
                if c_range(mm) is None: a_hi = mm
                else: a_lo = mm
            al, r = a_lo, c_range(a_lo)
        if r is None:
            return np.clip(b[0] * u, 0, fmax)
        c = float(np.clip(b[0], r[0], r[1]))
        return np.clip(c * u + al * Fm, 0, fmax)

    # ---------------------------------------------------------------- reflexes
    def reflexes(self, dt):
        """Load governor: skill phases slow down (to 0.45) while any servo is above 0.80 x stall or a claw-prop is
        saturated, and recover at 0.8 /s."""
        pk = float(self.load.max())
        self.load_f += (pk - self.load_f) * min(1, dt / 0.05)
        target = 1.0 - 2.75 * max(self.load_f - 0.80, 0) - 0.25 * min(self.sat, 2)
        target = float(np.clip(target, 0.45, 1.0))
        self.rate = target if target < self.rate else min(target, self.rate + 0.8 * dt)

    # ---------------------------------------------------------------- spinal tracker
    def spine(self, dt):
        self.gjump = max(getattr(self, "gjump", 0.0), float(np.abs(self.goal - self.goal_prev).max()))   # largest raw step since last frame
        gd = (self.goal - self.goal_prev) / dt
        gd = np.clip(gd, -V_FREE * 1.5, V_FREE * 1.5)
        self.goal_prev = self.goal.copy()
        loaded = self.foot_f > FOOT_N
        V = np.where(loaded, V_LOAD, V_FREE)[:, None]; A = np.where(loaded, A_LOAD, A_FREE)[:, None]
        V_env = np.maximum(ENV_MARGIN * W_NOLOAD[None, :] * (1 - np.clip(self.load, 0, 1)), 10 * D)
        if ENV_LIMIT: V = np.minimum(V, V_env)       # off by default: clipping stance legs makes them lag their IK and raises loads
        self.env_clip = float((np.abs(self.qd_cmd) >= V - 1e-9).any())
        acc = W_SPINE ** 2 * (self.goal - self.q_cmd) + 2 * W_SPINE * (gd - self.qd_cmd)
        acc = np.clip(acc, -A, A)
        self.qd_cmd = np.clip(self.qd_cmd + acc * dt, -V, V)
        self.q_cmd = np.clip(self.q_cmd + self.qd_cmd * dt, C.LIM[:, 0], C.LIM[:, 1])
        self.acc_cmd = acc

    # ---------------------------------------------------------------- thrust (every physics step)
    def thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; w = d.qvel[3:6]
        com = d.subtree_com[self.torso].copy()
        self.edf += (min(self.edf_target, self.edf_cap) - self.edf) * min(1, DT / 0.05); edf = self.edf
        if self.air_plan is not None:
            t0, T, a0, a1 = self.air_plan
            self.air = a0 + (a1 - a0) * mj((t - t0) / T)
            if a1 == 0 and t - t0 >= T: self.airborne = False
        self.steer_w += ((1.0 if self.steering else 0.0) - self.steer_w) * min(1, DT / 0.3)
        air = self.air
        ez = Rb[:, 2]; roll_err = -ez[1]; pitch_err = ez[0]
        d.xfrc_applied[:] = 0
        wW = Rb @ w
        if not self.fans_on:
            self.fan[:] = 0; self.sat = 0; self.wgt = np.zeros(8)
            if edf > 1e-3:
                d.xfrc_applied[self.torso, :3] += edf * ez
                d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)
            return
        # claw-prop permission: angle to the body axis (25 -> 12 deg on/near the ground, 48 -> 42 deg world when
        # steering), a 0.4 s spin-up fade, the per-arm gate
        cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
        cb = np.array([d.xmat[self.pod[i]].reshape(3, 3)[:, 0] @ ez for i in range(8)])
        w_ground = sstep((cb - np.cos(25 * D)) / (np.cos(12 * D) - np.cos(25 * D))) * (cz > np.cos(25 * D))
        w_steer = np.clip((cz - np.cos(48 * D)) / (np.cos(42 * D) - np.cos(48 * D)), 0, 1)
        wgt = (1 - self.steer_w) * w_ground + self.steer_w * w_steer
        for i in range(8):
            if cz[i] > np.cos(25 * D) or (self.steer_w > 0.5 and wgt[i] > 0):
                if self.t_in[i] is None: self.t_in[i] = t
                wgt[i] *= mj((t - self.t_in[i]) / 0.4)
            else:
                self.t_in[i] = None
        self.gate_s += (self.gate - self.gate_s) * min(1, DT / 0.15)
        wgt *= self.gate_s * (1.0 if self.fans_on else 0.0)
        self.wgt = wgt
        on = [i for i in range(8) if wgt[i] > 1e-3]
        cmd = np.zeros(8)
        if on:
            n_eff = float(wgt[on].sum())
            # ground collective: the fans carry what the planted feet should not
            if self.ground_collective:
                n_dn = self.n_down_plan if self.n_down_plan is not None else int((self.foot_f > FOOT_N).sum())
                Fg = max(n_eff * FAN_BASE, C.W_ROBOT - edf - FOOT_LOAD * n_dn)
                light = C.W_ROBOT - edf - FOOT_LOAD
                Fg = Fg + (light - Fg) * self.unload if self.unload > 0 else Fg
            else:
                Fg = n_eff * FAN_BASE
            zr, vzr, azr = self.zplan.at(t)
            g_ = self.g
            Fa = (C.M_ROBOT + self.payload) * (C.G + azr + g_["kp_z"] * (zr - pb[2]) + g_["kd_z"] * (vzr - d.qvel[2])) / max(ez[2], 0.7) - edf * ez[2]
            F_need = (1 - air) * Fg + air * Fa
            if air < 0.999:
                if self.F_prev is not None:
                    F_need = self.F_prev + np.clip(F_need - self.F_prev, -40.0 * DT, 40.0 * DT)
                self.F_prev = F_need
            else:
                self.F_prev = F_need
            # attitude: ground gains blend into flight gains; weak body-tilt xy hold only in transitional flight
            if air > 0 and self.steer_w < 0.5:
                p, _, _ = self.xyplan.at(t)
                exy = p - com[:2]; vxy = d.qvel[0:2]
                hold = air * (1 - 2 * self.steer_w)
                tx = hold * np.clip(0.30 * exy[0] - 0.45 * vxy[0], -0.10, 0.10); ty = hold * np.clip(0.30 * exy[1] - 0.45 * vxy[1], -0.10, 0.10)
            else:
                tx = ty = 0.0
            kp, kd = 60.0 + (g_["kp_att"] - 60.0) * air, 8.0 + (g_["kd_att"] - 8.0) * air
            Mx = -kp * (roll_err + ty) - kd * wW[0]
            My = -kp * (pitch_err - tx) - kd * wW[1]
            if g_["ki_att"] > 0 and air > 0.99:      # trims a steady tilt from off-centre payloads (frozen while saturated)
                if self.sat == 0:
                    self.iatt = np.clip(self.iatt + g_["ki_att"] * np.array([roll_err + ty, pitch_err - tx]) * DT, -3.0, 3.0)
                Mx -= self.iatt[0]; My -= self.iatt[1]
            M_edf = np.cross(pb - com, edf * ez)
            nr = 4 if PROP_TORQUE else 3
            A = np.zeros((nr, len(on))); fmax = self.fmax_arm[on] * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * BO[i] - com
                mo = np.cross(r, ax) - ((QT[i] or PROP_P["q_per_T"]) * SPIN[i] * ax if PROP_TORQUE else 0)
                A[:, j] = [ax[2], mo[0], mo[1], mo[2]][:nr]
            b = [F_need, Mx - M_edf[0], My - M_edf[1]]
            if PROP_TORQUE: b.append(getattr(self, "Mz_dem", 0.0) * self.g["mz_alloc"])
            F = self.allocate_n(A, np.array(b), wgt[on], fmax)
            # yaw gives way (hand3 final flight fix, tests/body4h_yawyield.diff): if any prop would pass 0.9 of its ceiling while
            # steering in the air, halve the yaw demand (up to 4x).  Inert in normal flight.  H3_YAWYIELD=0 disables.
            if PROP_TORQUE and YAW_YIELD and self.steer_w > 0.5 and air > 0.99:
                bb = np.array(b, float); kk = 0
                while kk < 4 and np.any(F >= 0.9 * fmax - 1e-9):
                    bb[3] *= 0.5; F = self.allocate_n(A, bb, wgt[on], fmax); kk += 1
            for j, i in enumerate(on):
                cmd[i] = F[j]
        self.fan += (cmd - self.fan) * min(1.0, DT / 0.04)
        self.sat = int(np.sum((cmd >= self.fmax_arm - 1e-6) & (wgt > 0.99)))
        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * BO[i] + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
                if PROP_TORQUE:                      # rotor reaction torque on the pod, opposite to the spin
                    d.xfrc_applied[self.pod[i], 3:] -= (QT[i] or PROP_P["q_per_T"]) * SPIN[i] * self.fan[i] * R[:, 0]
        d.xfrc_applied[self.torso, :3] += edf * ez
        d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)
        v = d.qvel[0:3]
        drag = -0.5 * RHO * CDA * np.linalg.norm(v) * v
        d.xfrc_applied[self.torso, :3] += drag
        d.xfrc_applied[self.torso, 3:] += np.cross(com - d.xipos[self.torso], drag)

    # ---------------------------------------------------------------- main loop
    def next_skill(self, t):
        if self.program:
            self.cur = self.program.pop(0)
        else:
            self.cur = Idle(1e9) if not self.lying else Rest(1e9)
        self.cur.start(self, t)
        self.log_skills.append((t, self.cur.name)); self.cur.key = f"{len(self.log_skills):02d} {self.cur.name}"

    def run(self, t_max=120.0, fps=30):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_ctrl = -1.0
        t0w = time.time()
        stats = {}
        prev_q = self.q_cmd.copy(); prev_qd = self.qd_cmd.copy()
        dq_jump = 0.0
        while d.time < t_max:
            t = d.time
            if t - t_ctrl >= CTRL_DT - 1e-9:
                self.sense()
                if self.bat is not None:
                    self.bat.step(self.edf, self.fan * self.k_ratio, self.load, CTRL_DT, k_claw=PROP_P["K"])
                    self.fmax_eff = F_MAX * self.bat.claw_scale() if PROP == "std" else PW.claw_max(PROP, self.bat.V)
                    self.fmax_arm = np.array([HAND[i]["T"] * min(1.0, (self.bat.V / V_REF_H) ** 1.5) if i in HAND else self.fmax_eff for i in range(8)])
                    self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())
                self.reflexes(CTRL_DT)
                if self.cur is None or self.cur.done:
                    if self.cur is not None and not self.program:
                        break
                    self.next_skill(t)
                self.gate = np.ones(8)
                if not isinstance(self.cur, Land): self.override = [None] * 8
                self.cur.step(self, t, CTRL_DT)
                self.spine(CTRL_DT)
                dq_jump = max(dq_jump, float(np.abs(self.qd_cmd - prev_qd).max()) / CTRL_DT)
                prev_qd = self.qd_cmd.copy()
                d.ctrl[:8 * C.NJ] = self.q_cmd.ravel()
                self.T_line += (self.winch_target - self.T_line) * min(1, CTRL_DT / 0.15)
                t_ctrl = t
                lab = self.cur.label(self)
            d.qfrc_applied[:] = 0
            self.clutch(self.T_line)
            self.thrust(t)
            d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G
            mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all():
                print("NaN at", t); break
            if t >= nxt - 1e-9:
                Rb = d.xmat[self.torso].reshape(3, 3)
                tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
                slip = np.zeros(8)
                for (i, k), qa in self.cq.items():
                    slip[i] = max(slip[i], abs(np.degrees(d.qpos[qa])))
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, T=self.T_line,
                                   servo_arm=self.load.max(1), slip_arm=slip, tilt=tilt, foot=self.foot_f.copy(), belly=self.belly_f,
                                   rate=self.rate, air=self.air, steer=self.steer_w, sat=self.sat, skill=self.cur.name, label=lab, sidx=len(self.log_skills) - 1,
                                   qcmd=self.q_cmd.copy(), qdcmd=self.qd_cmd.copy(), load=self.load.copy(), qdm=2 * d.qvel[self.da].copy(), goal=self.goal.copy(), gjump=self.gjump, env_clip=self.env_clip, com=d.subtree_com[self.torso].copy(),
                                   zref=float(self.zplan.at(t)[0]), xyref=self.xyplan.at(t)[0].copy(),
                                   yaw=float(np.degrees(np.arctan2(Rb[1, 0], Rb[0, 0]))),
                                   V=self.bat.V if self.bat else 25.2, I=self.bat.I if self.bat else 0.0, soc=self.bat.soc if self.bat else 1.0,
                                   fmax=self.fmax_eff, edfcap=self.edf_cap))
                s = stats.setdefault(self.cur.key, dict(servo=0.0, tilt=0.0, slip=0.0, sat=0, rate_min=1.0, t0=t, t1=t, fan=0.0))
                if self.load.max() > s["servo"]:
                    ii, kk = np.unravel_index(self.load.argmax(), self.load.shape); s["at"] = f"arm{ii} j{kk} t{t:.2f}"
                s["servo"] = max(s["servo"], float(self.load.max())); s["tilt"] = max(s["tilt"], tilt); s["slip"] = max(s["slip"], float(slip.max()))
                s["sat"] = max(s["sat"], self.sat); s["rate_min"] = min(s["rate_min"], self.rate); s["t1"] = t; s["fan"] = max(s["fan"], float(self.fan.max()))
                nxt += 1 / fps
                self.gjump = 0.0
        print(f"{CASE}: sim {d.time:.1f} s in {time.time() - t0w:.0f} s wall")
        if self.bat is not None:
            print(f"battery {PACKS} pack(s), start {SOC0 * 100:.0f} %: end {self.bat.soc * 100:.0f} %, used {self.bat.ah:.2f} Ah")
        return frames, stats


def save(frames, stats, tag):
    keys = [k for k in frames[0] if k not in ("skill", "label")]
    np.savez_compressed(f"body4_{tag}.npz", **{k: np.array([f[k] for f in frames]) for k in keys},
                        skill=np.array([f["skill"] for f in frames]), label=np.array([f["label"] for f in frames]))
    json.dump(dict(case=CASE, fan_max=F_MAX, stats=stats), open(f"body4_{tag}.json", "w"), indent=1)


if __name__ == "__main__":
    prog = sys.argv[2] if len(sys.argv) > 2 else "day"
    P = PROGRAMS[prog]
    start = "lying" if P[0][0] in ("rest", "get_up") else "stand"
    if prog == "step":          # a 12 cm step 1.6 m north, 1.0 x 1.0 m: the floor is not where the landing expects it
        S4.EXTRA_XML = '<geom name="ground_step" type="box" pos="0 1.6 0.06" size="0.5 0.5 0.06" rgba="0.75 0.7 0.6 1" contype="1" conaffinity="1" friction="1.0 0.01 0.001"/>'
    B = Body(P, start)
    frames, stats = B.run()
    for k, s in stats.items():
        print(f"{k:9s} {s['t0']:5.1f}-{s['t1']:5.1f} s | servo {s['servo']:.2f} x stall | tilt {s['tilt']:4.1f} | slip {s['slip']:4.1f} | "
              f"fan max {s['fan']:4.1f} N, sat {s['sat']} | governor min {s['rate_min']:.2f} | {s.get('at','')}")
    tag = f"{CASE}_{prog}" + (f"_p{PACKS}s{int(SOC0 * 100)}" if os.environ.get("CEPH_PACKS") or os.environ.get("CEPH_SOC") else "")
    save(frames, stats, tag)
