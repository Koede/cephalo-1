"""Full-robot MuJoCo model of Cephalo-1 rev 4 and the 'everything' mission:
stand -> ripple walk -> take-off (v5 sequence) -> hover -> side hit + radial hit -> landing (take-off reversed) -> stand.

Model (assumptions listed, all estimates unless noted):
  * free body 2.81 kg (mantle + EDF + battery + 8 winches), 8 arms x 8 servo joints.  Each joint is written as two
    coupled half-joints A and B (equality B = A) so the same code also models a double hinge (cephalo4.RG > 0); rev 4
    has RG = 0, i.e. both halves on one axis.  Servo: position actuator on A with transmission gear 2 (acts on the full
    joint angle), kp = stall / 2 deg, force-limited at stall.  Clutch dof on the servo side (detent: implicit spring
    capped at the slip torque C_k * stall * T_line / 60 N).
  * reflected rotor inertia (armature) per servo: 0.02 / 0.012 kg m^2 (est.); gear-load metric = |servo torque| +
    armature * |joint acc|, against an assumed 3 x stall gear limit.
  * fingertips: 3 x r 4 mm spheres per pod at 60 deg fold, friction 1.0; prop disc collides with the impactor only.
  * thrust = force along each pod axis (puller), fail-safe: a fan only runs when its axis is within 25 deg of up.
    EDF = force along the body +z.  Fan max per case in cephalo4.FAN_MAX ('mid' by default).
  * flight controller: PD attitude + altitude + weak xy hold on the 8 fans (+ EDF collective); no rotor dynamics
    except a 40 ms first-order lag on each fan.
Run: python3 sim4.py [mid|pess]  -> sim4_<case>.npz (30 fps states, 300 fps around the hits, + metrics) and a summary.
     CEPH_FMAX=<N> overrides the claw-prop max thrust (sensitivity runs).
"""
import os, sys, json, time, numpy as np, mujoco
import cephalo4 as C
if os.environ.get("CEPH_J1CLUTCH"):
    C.CLUTCH_FRAC[1] = float(os.environ["CEPH_J1CLUTCH"]); C.CLUTCH_TRAVEL[1] = 25 * C.DEG

D = C.DEG
PZ = json.load(open("poses4.json"))
Q_STAND = np.radians(PZ["stand"]); Q_FLY = np.radians(PZ["flight"]); H_STAND = PZ["stand_h"]
CASE = sys.argv[1] if (__name__ == "__main__" and len(sys.argv) > 1) else os.environ.get("CEPH_CASE", "mid")
F_MAX = float(os.environ["CEPH_FMAX"]) if os.environ.get("CEPH_FMAX") else C.FAN_MAX[CASE]
M_TORSO = C.M_BODY + 8 * C.M_WINCH
DT = 0.0004
CAPSULE_R = lambda s: float(np.interp(s, [0, 0.2, C.SPIGOT], [0.024, 0.022, 0.017]))


# ================================================================== model
def v2s(v):
    return " ".join(f"{x:.6f}" for x in v)


def q2s(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel()); return v2s(q)


def arm_xml(i):
    R0, p0 = C.arm_base(i)
    ax = lambda k: "0 1 0" if C.TYP[k] == "p" else "0 0 1"
    x = [f'<body name="arm{i}" pos="{v2s(p0)}" quat="{q2s(R0)}">']
    # root stub (fixed to the mantle): servo 0 + root clutch hardware
    sa0 = C.S_J[0] - C.RG
    x.append(f'<geom type="capsule" fromto="0 0 0 {sa0:.4f} 0 0" size="0.022" mass="{C.SV[C.SERVO[0]][2] + C.M_CLUTCH[C.SERVO[0]]:.4f}" '
             f'contype="0" conaffinity="0"/>')
    depth = 1; s_prev = 0.0
    for k in range(C.NJ):
        sa = C.S_J[k] - C.RG
        lim = np.degrees(C.LIM[k]) / 2
        x.append(f'<body name="a{i}_{k}" pos="{sa - s_prev:.5f} 0 0">'
                 f'<joint name="q{i}_{k}a" axis="{ax(k)}" range="{lim[0] - 1:.2f} {lim[1] + 1:.2f}" damping="0.03" armature="{4 * C.ARMATURE[k]:.4f}"/>'
                 f'<geom type="sphere" size="0.006" mass="0.002" contype="0" conaffinity="0"/>')
        depth += 1
        if C.CLUTCH_FRAC[k] > 0:
            tr = np.degrees(C.CLUTCH_TRAVEL[k])
            x.append(f'<body name="c{i}_{k}"><joint name="c{i}_{k}" axis="{ax(k)}" range="{-tr:.1f} {tr:.1f}" damping="0.02" '
                     f'armature="0.0005" stiffness="0"/>')
            depth += 1
        x.append(f'<geom type="capsule" fromto="0 0 0 {2 * C.RG:.4f} 0 0" size="0.012" mass="{C.M_HINGE_J[k] / 2:.4f}" contype="0" conaffinity="0"/>'
                 if C.RG > 1e-4 else f'<geom type="sphere" size="0.012" mass="{C.M_HINGE_J[k] / 2:.4f}" contype="0" conaffinity="0"/>')
        x.append(f'<body name="b{i}_{k}" pos="{2 * C.RG:.5f} 0 0"><joint name="q{i}_{k}b" axis="{ax(k)}" range="{lim[0] - 1:.2f} {lim[1] + 1:.2f}" damping="0.01"/>')
        depth += 1
        s0 = C.S_J[k] + C.RG
        s1 = C.S_J[k + 1] - C.RG if k + 1 < C.NJ else C.SPIGOT
        L = s1 - s0
        m_link = C.M_HINGE_J[k] / 2 + (C.LAM * C.SPIGOT + C.M_LINE) * L / C.SPIGOT
        if k + 1 < C.NJ:
            n = C.SERVO[k + 1]
            m_link += C.SV[n][2] + (C.M_CLUTCH[n] if C.CLUTCH_FRAC[k + 1] > 0 else 0)
        x.append(f'<geom type="capsule" fromto="0 0 0 {L:.4f} 0 0" size="{CAPSULE_R(s0):.4f}" mass="{m_link:.4f}" contype="0" conaffinity="0"/>')
        s_prev = s0
    # tip bay + pod (fixed fold); fingertips at 60 deg fold
    x.append(f'<body name="pod{i}" pos="{C.SPIGOT - s_prev:.5f} 0 0">'
             f'<geom type="box" pos="-0.012 0 0" size="0.012 0.012 0.008" mass="{C.M_TIPBAY:.4f}" contype="0" conaffinity="0"/>'
             f'<geom type="cylinder" fromto="0 0 0 0.030 0 0" size="0.0145" mass="{C.M_POD * 0.8:.4f}" contype="0" conaffinity="0"/>'
             f'<geom name="prop{i}" type="cylinder" fromto="{C.BLADE_OFF - 0.002:.4f} 0 0 {C.BLADE_OFF + 0.002:.4f} 0 0" size="{C.CLAW_R:.4f}" '
             f'mass="{C.M_POD * 0.2:.4f}" contype="2" conaffinity="2"/>')
    for f in range(3):
        a = f * 2 * np.pi / 3
        rr = C.CLAW_R * np.cos(C.FOLD_STAND); xx = C.BLADE_OFF + C.CLAW_R * np.sin(C.FOLD_STAND)
        x.append(f'<geom name="toe{i}_{f}" type="sphere" pos="{xx:.4f} {rr * np.cos(a):.4f} {rr * np.sin(a):.4f}" size="0.004" '
                 f'mass="0.0001" contype="1" conaffinity="1" friction="1.0 0.01 0.001"/>')
    x.append('<site name="tip%d" pos="0 0 0"/></body>' % i)
    x += ["</body>"] * depth
    return "\n".join(x)


def build(case_imp=True):
    arms = "\n".join(arm_xml(i) for i in range(8))
    eq = "".join(f'<joint joint1="q{i}_{k}b" joint2="q{i}_{k}a" polycoef="0 1 0 0 0" solref="0.002 1"/>' for i in range(8) for k in range(C.NJ))
    act = "".join(f'<position name="s{i}_{k}" joint="q{i}_{k}a" gear="2" kp="{C.STALL[k] / (2 * D):.2f}" kv="{C.STALL[k] / (2 * D) * 0.02:.3f}" '
                  f'forcerange="{-C.STALL[k]} {C.STALL[k]}" forcelimited="true" ctrlrange="-3.2 3.2"/>' for i in range(8) for k in range(C.NJ))
    xml = f"""<mujoco model="cephalo1_rev4">
<option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001" friction="0.8 0.01 0.001"/><joint limited="true"/></default>
<worldbody>
<geom name="floor" type="plane" size="20 20 0.1" contype="1" conaffinity="1" friction="1.0 0.01 0.001"/>
<body name="torso" pos="0 0 {H_STAND:.4f}"><freejoint name="root"/>
<inertial pos="0 0 0.05" mass="{M_TORSO:.4f}" diaginertia="0.020 0.020 0.022"/>
<geom name="edf" type="cylinder" fromto="0 0 -0.004 0 0 -0.092" size="0.055" mass="0" contype="1" conaffinity="1"/>
<geom name="plate" type="cylinder" fromto="0 0 0 0 0 0.006" size="0.125" mass="0" contype="1" conaffinity="1"/>
{arms}
</body>
<body name="imp" pos="0 0 -5"><freejoint name="impj"/>
<geom name="imp" type="sphere" size="0.02" mass="0.2" contype="2" conaffinity="2" rgba="0.85 0.15 0.1 1"/></body>
</worldbody>
<equality>{eq}</equality>
<actuator>{act}</actuator>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


# ================================================================== mission plan (kinematic targets)
def smooth(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)


ORDER = [0, 4, 2, 6, 1, 5, 3, 7]
STRIDE, LIFT, SWING, SLOT = 0.090, 0.045, 0.30, 0.36
T_WALK0, N_CYC = 1.5, 2
T_WALK1 = T_WALK0 + N_CYC * 8 * SLOT
T_TO = T_WALK1 + 0.8                              # take-off start
PAIRS = [(2, 6), (1, 5), (3, 7)]
T_SW = 1.8                                        # leg swing time stand <-> flight (firmware: <=120 deg/s, 800 deg/s^2)
T_UNLOAD = T_TO + 0.6 + 3 * T_SW                  # collective ramps so feet 0 + 4 keep only ~10 N
T_HOP = T_UNLOAD + 0.6                            # airborne control on: lift 4 cm off feet 0 + 4
T_L4 = T_HOP + 0.5                                # leg 4 swings up in the air (6 fans + EDF hold)
T_LEG0 = (T_L4 + T_SW, T_L4 + 2 * T_SW)           # then leg 0 (7 fans + EDF hold)
Z_HOVER_UP = 0.45                                 # climb (m) above the stand plate height
T_HIT1, T_HIT2 = T_LEG0[1] + 2.4, T_LEG0[1] + 4.2
T_DESC = T_HIT2 + 1.6
T_LAND0 = T_DESC + 2.4                            # leg 0 down (in the air)
T_LAND4 = T_LAND0 + T_SW                          # leg 4 down (in the air)
T_TOUCH = T_LAND4 + T_SW + 0.9                    # feet 0 + 4 on the floor, airborne control off
T_LAND_PAIRS = T_TOUCH + 0.8
T_END = T_LAND_PAIRS + 3 * T_SW + 2.2
FAN_BASE = 6.0                                    # N per fan while the robot still stands on legs


_F0 = C.tip_point(0, Q_STAND, np.array([0, 0, H_STAND]))[0]
Q_LIFT = C.ik(0, _F0 + [0, 0, 0.07], np.array([0, 0, -1.0]), Q_STAND, Q_STAND, np.array([0, 0, H_STAND]))
Q_TUCK = np.radians([-40, 0, 40, 0, -60, 0, -20, 0])


def tuck_path(u):
    """Stand -> flight joint path (u 0..1): straight lift of the foot by 70 mm, tuck, then unfold to the tulip pose."""
    if u < 0.2:
        return Q_STAND + (Q_LIFT - Q_STAND) * smooth(u / 0.2)
    if u < 0.55:
        return Q_LIFT + (Q_TUCK - Q_LIFT) * smooth((u - 0.2) / 0.35)
    return Q_TUCK + (Q_FLY - Q_TUCK) * smooth((u - 0.55) / 0.45)


class Plan:
    """Precomputes joint targets for all arms on a 100 Hz grid with IK for stance legs during the walk."""

    def __init__(self):
        self.t = np.arange(0, T_END + 0.01, 0.01)
        n = len(self.t)
        self.Q = np.zeros((n, 8, C.NJ)); self.body = np.zeros((n, 3))
        feet0 = []
        for i in range(8):
            f, _ = C.tip_point(i, Q_STAND, np.array([0, 0, H_STAND])); feet0.append(f)
        feet0 = np.array(feet0)
        qprev = np.tile(Q_STAND, (8, 1))
        v = STRIDE / (8 * SLOT)
        for j, t in enumerate(self.t):
            # ---- body plan (only meaningful on the ground)
            if T_WALK0 <= t < T_WALK1:
                bx = v * (t - T_WALK0)
            else:
                bx = v * (T_WALK1 - T_WALK0) if t >= T_WALK1 else 0.0
            bz = H_STAND
            self.body[j] = [bx, 0, bz]
            for i in range(8):
                if t < T_WALK1 + 0.4:
                    # walking: feet in world; swing in slots
                    tc = t - T_WALK0
                    foot = feet0[i].copy()
                    if tc > 0:
                        cyc = min(int(tc // (8 * SLOT)), N_CYC)
                        slot = ORDER.index(i)
                        ts = tc - cyc * 8 * SLOT - slot * SLOT
                        done = cyc + (1 if (cyc < N_CYC and ts >= SWING) else 0)
                        foot[0] += STRIDE * done
                        if cyc < N_CYC and 0 <= ts < SWING:
                            u = ts / SWING
                            foot[0] += STRIDE * smooth(u); foot[2] += LIFT * np.sin(np.pi * u)
                    q = C.ik(i, foot, np.array([0, 0, -1.0]), qprev[i], Q_STAND, np.array([bx, 0, bz]))
                    self.Q[j, i] = q; qprev[i] = q
                else:
                    self.Q[j, i] = self.arm_target(i, t, qprev[i])

    def arm_target(self, i, t, q_walk_end):
        if i == 0:
            t_up, t_dn = T_LEG0[0], T_LAND0
        elif i == 4:
            t_up, t_dn = T_L4, T_LAND4
        else:
            p = [k for k, pr in enumerate(PAIRS) if i in pr][0]
            t_up, t_dn = T_TO + 0.6 + p * T_SW, T_LAND_PAIRS + (2 - p) * T_SW
        u_up = (t - t_up) / T_SW; u_dn = (t - t_dn) / T_SW
        if u_dn > 0:
            return tuck_path(1 - min(u_dn, 1.0))
        if u_up <= 0:
            return q_walk_end
        return tuck_path(min(u_up, 1.0))

    def at(self, t):
        j = min(int(t / 0.01), len(self.t) - 2); a = (t - self.t[j]) / 0.01
        return self.Q[j] * (1 - a) + self.Q[j + 1] * a


def winch(t):
    """Line tension: tight on the ground, firm for take-off/flight, soft for the airborne approach (legs 0 + 4 meet the
    floor compliant), back to firm at touch-down and tight once all feet are down.  The N20 worm winch ramps in ~0.5 s."""
    Tt, Tf, Ts = C.T_LINE["tight"], C.T_LINE["firm"], C.T_LINE["soft"]
    if t < T_TO: return Tt
    if t < T_TO + 0.5: return Tt + (Tf - Tt) * smooth((t - T_TO) / 0.5)
    if t < T_LAND0: return Tf
    if t < T_LAND0 + 0.5: return Tf + (Ts - Tf) * smooth((t - T_LAND0) / 0.5)
    if t < T_TOUCH - 0.1: return Ts
    if t < T_TOUCH + 0.4: return Ts + (Tf - Ts) * smooth((t - T_TOUCH + 0.1) / 0.5)
    if t < T_END - 1.6: return Tf
    return Tf + (Tt - Tf) * smooth((t - T_END + 1.6) / 0.5)


def mode(t):
    if t < T_WALK0: return "STAND"
    if t < T_WALK1: return "WALK (ripple gait, 7 feet down)"
    if t < T_TO: return "STAND, EDF spool-up"
    if t < T_UNLOAD: return "TAKE-OFF: legs up in pairs"
    if t < T_HOP: return "TAKE-OFF: unload feet 0 + 4"
    if t < T_LEG0[1]: return "TAKE-OFF: airborne, legs 4 + 0 up"
    if t < T_HIT1 - 0.3: return "HOVER / CLIMB"
    if t < T_HIT2 + 0.8: return "HOVER: HIT TESTS"
    if t < T_LAND0: return "DESCEND"
    if t < T_TOUCH: return "LANDING: legs 0 + 4 down, touch"
    if t < T_END - 1.6: return "LANDING: legs down in pairs"
    return "STAND"


# ================================================================== simulation
class Sim:
    def __init__(self):
        self.m = build(); self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        self.plan = Plan()
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
                d.qpos[self.ja[i, k]] = Q_STAND[k] / 2; d.qpos[self.jb[i, k]] = Q_STAND[k] / 2
        d.qpos[2] = H_STAND + 0.004
        d.ctrl[:] = np.tile(Q_STAND, 8)
        mujoco.mj_forward(m, d)
        self.fan = np.zeros(8); self.edf = 0.0
        self.z_ref = None; self.xy_ref = None
        self.hits = []
        self.prev_dq = np.zeros(m.nv)

    # ---- clutch: implicit spring (joint stiffness) capped at the slip torque
    def clutch(self, T):
        m, d = self.m, self.d
        for (i, k), qa in self.cq.items():
            tb = C.CLUTCH_FRAC[k] * C.STALL[k] * T / C.T_FIRM      # joint-level slip moment (servo-side slip x N_DRIVE)
            kc = tb / (0.3 * D)
            m.jnt_stiffness[self.cj_ids[i, k]] = kc
            phi = d.qpos[qa]
            if kc * abs(phi) > tb:
                d.qfrc_applied[self.cd[i, k]] += np.sign(phi) * (kc * abs(phi) - tb)

    def thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; w = d.qvel[3:6]   # body-frame ang vel
        com = d.subtree_com[self.torso].copy()
        # EDF schedule
        if t < T_TO: edf = 0.0
        elif t < T_TO + 0.6: edf = C.EDF_MAX * smooth((t - T_TO) / 0.6)
        elif t < T_END - 2.6: edf = C.EDF_MAX * (0.8 if T_LEG0[1] + 1.0 < t < T_LAND0 else 1.0)
        elif t < T_END - 1.6: edf = C.EDF_MAX * (1 - smooth((t - (T_END - 2.6)) / 1.0))
        else: edf = 0.0
        self.edf = edf
        airborne = T_HOP <= t < T_TOUCH
        ez = Rb[:, 2]
        roll_err = -ez[1]; pitch_err = ez[0]
        if airborne:
            z0 = H_STAND + 0.05
            if t < T_LEG0[1]: zr = H_STAND + 0.04 * smooth((t - T_HOP) / 0.4)
            elif t < T_DESC: zr = H_STAND + 0.04 + (z0 + Z_HOVER_UP - H_STAND - 0.04) * smooth((t - T_LEG0[1]) / 1.5)
            else: zr = z0 + Z_HOVER_UP - Z_HOVER_UP * smooth((t - T_DESC) / 2.2) - 0.06 * smooth((t - T_LAND4 - T_SW) / 0.9)
            if self.xy_ref is None:
                self.xy_ref = com[:2].copy()
            vz = d.qvel[2]
            exy = self.xy_ref - com[:2]; vxy = d.qvel[0:2]
            tx = np.clip(0.30 * exy[0] - 0.45 * vxy[0], -0.10, 0.10); ty = np.clip(0.30 * exy[1] - 0.45 * vxy[1], -0.10, 0.10)
            F_need = C.M_ROBOT * (C.G + 4.0 * (zr - com[2] + (com[2] - pb[2])) - 3.5 * vz) / max(ez[2], 0.7) - edf * ez[2]
        else:
            tx = ty = 0.0; F_need = None
        wW = Rb @ w
        # on two feet the robot is an inverted pendulum about the foot line (m g h ~ 40 N m/rad): stiffer gains there
        kp, kd = (14.0, 4.5) if airborne else (60.0, 8.0)
        Mx = -kp * (roll_err + ty) - kd * wW[0]
        My = -kp * (pitch_err - tx) - kd * wW[1]
        # EDF moment about the COM (it acts along the body axis through the torso origin)
        M_edf = np.cross(pb - com, edf * ez)
        on = [i for i in range(8) if d.xmat[self.pod[i]].reshape(3, 3)[2, 0] > np.cos(25 * D)]
        cmd = np.zeros(8)
        if on:
            n = len(on)
            if F_need is None:
                light = (C.W_ROBOT - edf - 10.0)                       # feet keep ~10 N
                if T_UNLOAD <= t < T_HOP:
                    F_need = n * FAN_BASE + (light - n * FAN_BASE) * smooth((t - T_UNLOAD) / 0.6)
                elif T_TOUCH <= t < T_LAND_PAIRS + 3 * T_SW:
                    F_need = light + (n * FAN_BASE - light) * smooth((t - T_TOUCH) / (T_LAND_PAIRS - T_TOUCH))
                else:
                    F_need = n * FAN_BASE
            A = np.zeros((3, n))
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax)
                A[:, j] = [ax[2], mo[0], mo[1]]
            b = np.array([F_need, Mx - M_edf[0], My - M_edf[1]])
            free = np.ones(n, bool); F = np.full(n, F_need / n)
            for _ in range(4):
                Af = A[:, free]; fixed = A[:, ~free] @ F[~free]
                F0 = np.full(free.sum(), (F_need - fixed[0]) / max(free.sum(), 1))
                F[free] = F0 + np.linalg.pinv(Af) @ (b - fixed - Af @ F0)
                over = (F > F_MAX) | (F < 0)
                if not over.any(): break
                F = np.clip(F, 0, F_MAX); free &= ~over
                if not free.any(): break
            F = np.clip(F, 0, F_MAX)
            for j, i in enumerate(on):
                cmd[i] = F[j]
        self.fan += (cmd - self.fan) * min(1.0, DT / 0.04)
        self.sat = int(np.sum(cmd >= F_MAX - 1e-6))
        d.xfrc_applied[:] = 0
        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3)
                f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
        d.xfrc_applied[self.torso, :3] += edf * ez
        d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)

    def impactor(self, t):
        d = self.d
        for (th, arm, kind) in ((T_HIT1, 2, "side"), (T_HIT2, 6, "radial")):
            if t >= th - 0.03 and not any(h[0] == th for h in self.hits):
                pc = d.geom(f"prop{arm}").xpos.copy()
                az = np.radians(22.5 + 45 * arm)
                rad = np.array([np.cos(az), np.sin(az), 0]); tan = np.array([-np.sin(az), np.cos(az), 0])
                u = tan if kind == "side" else -rad
                start = pc - u * 0.18 + (rad * 0.6 * C.CLAW_R if kind == "side" else 0)
                d.qpos[self.imp_q:self.imp_q + 3] = start
                d.qpos[self.imp_q + 3:self.imp_q + 7] = [1, 0, 0, 0]
                d.qvel[self.imp_d:self.imp_d + 3] = u * 6.0; d.qvel[self.imp_d + 3:self.imp_d + 6] = 0
                self.hits.append((th, arm, kind))
        if self.hits and t > self.hits[-1][0] + 0.25:
            d.qpos[self.imp_q:self.imp_q + 3] = [0, 0, -5]; d.qvel[self.imp_d:self.imp_d + 6] = 0
        d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G          # impactor flies straight (gravity compensated)

    def run(self, fps=30):
        m, d = self.m, self.d
        frames = []
        nxt = 0.0
        segs = [("stand", 0, T_WALK0), ("walk", T_WALK0, T_WALK1), ("take-off", T_TO, T_LEG0[1] + 0.5),
                ("hover", T_LEG0[1] + 0.5, T_HIT1 - 0.05), ("hit side", T_HIT1 - 0.05, T_HIT1 + 1.0),
                ("hit radial", T_HIT2 - 0.05, T_HIT2 + 1.0), ("landing", T_DESC, T_END)]
        segp = {s[0]: dict(servo=np.zeros((8, C.NJ)), gear=np.zeros((8, C.NJ)), slip=np.zeros((8, C.NJ)), tilt=0.0, sat=0, fan=0.0,
                         rms=np.zeros((8, C.NJ)), n=0)
                for s in segs}
        t0 = time.time()
        dq_prev = d.qvel.copy()
        while d.time < T_END:
            t = d.time
            d.ctrl[:] = self.plan.at(t).ravel()
            d.qfrc_applied[:] = 0
            self.clutch(winch(t))
            self.thrust(t)
            self.impactor(t)
            mujoco.mj_step(m, d)
            qacc = (d.qvel - dq_prev) / DT; dq_prev = d.qvel.copy()
            frc = np.abs(d.actuator_force.reshape(8, C.NJ))
            servo = frc / C.STALL
            gear = (frc + C.ARMATURE * np.abs(qacc[self.da]) * 2) / C.GEAR_LIMIT      # joint level; joint acc = 2 x pivot-A acc
            slip = np.zeros((8, C.NJ))
            for (i, k), qa in self.cq.items():
                slip[i, k] = abs(d.qpos[qa])
            Rb = d.xmat[self.torso].reshape(3, 3)
            tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
            for nm, a, b in segs:
                if a <= t < b:
                    sp = segp[nm]
                    np.maximum(sp["servo"], servo, out=sp["servo"]); np.maximum(sp["gear"], gear, out=sp["gear"])
                    np.maximum(sp["slip"], slip, out=sp["slip"]); sp["tilt"] = max(sp["tilt"], tilt)
                    sp["sat"] = max(sp["sat"], int(self.sat)); sp["fan"] = max(sp["fan"], float(self.fan.max()))
                    sp["rms"] += (frc / C.RATED) ** 2; sp["n"] += 1
            if t >= nxt - 1e-9:
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, T=winch(t),
                                   servo=servo.max(), gear=gear.max(), slip=np.degrees(slip.max()), tilt=tilt,
                                   servo_arm=servo.max(1), slip_arm=np.degrees(slip.max(1))))
                hit_near = any(abs(t - h) < 0.35 for h in (T_HIT1, T_HIT2))
                nxt += (1 / (fps * 10)) if hit_near else 1 / fps
        print(f"sim {T_END:.1f} s in {time.time() - t0:.0f} s wall")
        return frames, segp


def summarize(segp):
    rows = []
    for nm, sp in segp.items():
        i, k = np.unravel_index(sp["servo"].argmax(), sp["servo"].shape)
        ig, kg = np.unravel_index(sp["gear"].argmax(), sp["gear"].shape)
        rows.append(dict(seg=nm, servo=float(sp["servo"].max()), servo_at=f"arm{i} j{k}", gear=float(sp["gear"].max()),
                         gear_at=f"arm{ig} j{kg}", slip=float(np.degrees(sp["slip"].max())), tilt=float(sp["tilt"]),
                         fan_max=float(sp["fan"]), sat=int(sp["sat"]),
                         rms_rated=float(np.sqrt(sp["rms"] / max(sp["n"], 1)).max())))
    return rows


if __name__ == "__main__":
    S = Sim()
    print(f"model: nq {S.m.nq} nv {S.m.nv} nbody {S.m.nbody} | robot {C.M_ROBOT:.2f} kg | fan max {F_MAX:.1f} N ({CASE}) | "
          f"T_END {T_END:.1f} s")
    frames, segp = S.run()
    rows = summarize(segp)
    for r in rows:
        print(f"{r['seg']:11s} servo {r['servo']:.2f}x stall ({r['servo_at']}) | gear {r['gear']:.2f} ({r['gear_at']}) | "
              f"rms/rated {r['rms_rated']:.2f} | slip {r['slip']:.1f} deg | tilt {r['tilt']:.1f} | fan max {r['fan_max']:.1f} N | saturated fans {r['sat']}")
    np.savez_compressed(f"sim4_{CASE}.npz", t=np.array([f["t"] for f in frames]), qpos=np.array([f["qpos"] for f in frames]),
                        fan=np.array([f["fan"] for f in frames]), edf=np.array([f["edf"] for f in frames]),
                        T=np.array([f["T"] for f in frames]), servo=np.array([f["servo"] for f in frames]),
                        gear=np.array([f["gear"] for f in frames]), slip=np.array([f["slip"] for f in frames]),
                        tilt=np.array([f["tilt"] for f in frames]), servo_arm=np.array([f["servo_arm"] for f in frames]),
                        slip_arm=np.array([f["slip_arm"] for f in frames]))
    json.dump(dict(case=CASE, rows=rows, robot_kg=C.M_ROBOT, fan_max=F_MAX), open(f"sim4_{CASE}.json", "w"), indent=1)
