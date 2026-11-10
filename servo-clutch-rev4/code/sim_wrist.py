"""MuJoCo hit / yank test of one Cephalo-1 arm (rev 3 servo chain, flight pose) with two tip wrists:
  knuckle  - geared knuckle + roll (wrists2.Knuckle): roll STS3032, two pitch hinges tied 1:1 by printed m1 gears
             (equality constraint = gear mesh), pitch STS3032 on hinge 1.
  magnet   - magnetic Ball Drive (wrists2.BallMagnetic): free 50 mm hollow steel ball on 3 rubber omni-wheel contacts,
             pulled into them by an N52 magnet (force law below), Dyneema tether on a badge reel.
The mantle is held fixed (an 8.6 kg body barely moves during a 20 ms hit; the flight controller holds it after).
Servos: position loop with the stall torque as the force limit.  Fan thrust 5.5 N (hover) along the pod axis.

Estimates (to be replaced by measurements): link masses, servo stiffness, contact softness, magnet force law
F(g) = Fc / (1 + g/g0)^2 with g0 = 2 mm (Fc swept; the magnet's 143 N catalogue pull is for thick flat steel at
zero gap, a 0.8 mm hollow ball through the wheel stand-off gets far less).
"""
import numpy as np, mujoco
from robot import arm_base, J_POS, J_TYPE, SPIGOT, arm_fk
from choreo import FLIGHT as FLIGHT_WIDE
FLIGHT = __import__('numpy').radians([-40, 0, -35, 0, -15, 0, 0, 0, 0])   # flight v2 'tulip' (design state): tip points straight up

DEG = np.pi / 180
BODY_Z = 0.6
STALL = [4.9] * 3 + [2.9] * 2 + [0.44] * 4          # N m: STS3250, STS3215 (est.), STS3032 (4.5 kg cm)
SERVO_M = [0.076] * 3 + [0.055] * 2 + [0.020] * 4    # kg (STS3032 20 g; the others estimated)
BRACKET = 0.647 - sum(SERVO_M)                       # rest of the 647 g arm, spread along the links
THRUST = 5.5
R_BALL, R_W, BALL_OFF = 0.025, 0.009, 0.034
POD_M = 0.100
U_UP_PITCH = 0.0                                     # tulip: the arm already points the fan up


def q2s(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel()); return " ".join(f"{v:.6f}" for v in q)


def v2s(v):
    return " ".join(f"{x:.6f}" for x in v)


def arm_xml():
    R0, p0 = arm_base(0)
    xml = [f'<body name="a0" pos="{v2s(p0 + [0, 0, BODY_Z])}" quat="{q2s(R0)}">']
    prev = 0.0
    seg = np.diff(np.concatenate([J_POS, [SPIGOT]]))
    for k in range(9):
        L = seg[k]
        m = SERVO_M[k] + BRACKET * L / seg.sum()
        ax = "0 1 0" if J_TYPE[k] == "p" else "0 0 1"
        rng = "-90 90" if k > 1 else ("-41 41" if k == 0 else "-30 30")
        xml.append(f'<body name="l{k}" pos="{J_POS[k] - prev:.5f} 0 0">'
                   f'<joint name="j{k}" axis="{ax}" range="{rng}" damping="0.02" armature="0.002"/>'
                   f'<geom type="capsule" fromto="0 0 0 {L:.5f} 0 0" size="{0.016 - 0.007 * k / 9:.4f}" mass="{m:.4f}" contype="0" conaffinity="0"/>')
        prev = J_POS[k]
    return xml, prev


def build(variant, Fc=30.0, imp_m=0.2, imp_v=4.0):
    xml, prev = arm_xml()
    tip = f'<body name="tip" pos="{SPIGOT - prev:.5f} 0 0"><site name="collar" pos="0.006 0 0"/>'
    pod = (f'<geom name="motor" type="cylinder" fromto="0.004 0 0 0.026 0 0" size="0.0145" mass="{POD_M * 0.6:.3f}" contype="1" conaffinity="1"/>'
           f'<geom name="prop" type="cylinder" fromto="0.032 0 0 0.036 0 0" size="0.0635" mass="{POD_M * 0.4:.3f}" contype="1" conaffinity="1"/>'
           f'<site name="podc" pos="0.02 0 0"/>')
    eq, act, extra_world = [], [], []
    if variant == "knuckle":
        tip += ('<geom type="box" pos="0.016 0 0" size="0.016 0.012 0.0065" mass="0.024" contype="0" conaffinity="0"/>'
                '<body name="rollb" pos="0.036 0 0"><joint name="roll" axis="1 0 0" range="-180 180" damping="0.01" armature="0.0005"/>'
                '<geom type="cylinder" fromto="0 0 0 0.010 0 0" size="0.012" mass="0.008" contype="0" conaffinity="0"/>'
                '<body name="h1" pos="0.010 0 0"><joint name="h1" axis="0 -1 0" range="-88 88" damping="0.01" armature="0.0005"/>'
                '<geom type="box" pos="0.012 0 0" size="0.012 0.013 0.008" mass="0.030" contype="0" conaffinity="0"/>'
                '<body name="h2" pos="0.024 0 0"><joint name="h2" axis="0 -1 0" range="-88 88" damping="0.005"/>'
                '<geom type="cylinder" fromto="0 0 0 0.016 0 0" size="0.0065" mass="0.006" contype="0" conaffinity="0"/>'
                f'<body name="pod" pos="0.016 0 0">{pod}</body></body></body></body>')
        eq.append('<joint joint1="h2" joint2="h1" polycoef="0 1 0 0 0" solref="0.002 1"/>')
        act += [('roll', 0.44), ('h1', 0.44)]
    else:
        # wheel cradle: three rubber omni wheels behind the ball's equator, magnet behind them
        for k in range(3):
            az = (90 + 120 * k) * DEG; pol = 128 * DEG
            nrm = np.array([np.cos(pol), np.sin(pol) * np.cos(az), np.sin(pol) * np.sin(az)])
            c = np.array([BALL_OFF, 0, 0]) + nrm * (R_BALL + R_W)
            tip += f'<geom name="wh{k}" type="sphere" pos="{v2s(c)}" size="{R_W}" mass="0.012" friction="0.05 0.005 0.0005" contype="1" conaffinity="1"/>'
        tip += f'<site name="magnet" pos="{BALL_OFF - R_BALL - 0.007:.4f} 0 0"/>'
    tip += "</body>"
    xml.append(tip)
    xml += ["</body>"] * 10
    world = "\n".join(xml)
    # world pose of the tip in the flight pose (for placing free bodies)
    _, (Rt, pt) = arm_fk(0, FLIGHT, np.array([0, 0, BODY_Z]))
    up = Rt @ np.array([np.cos(U_UP_PITCH), 0, np.sin(U_UP_PITCH)])
    if variant == "magnet":
        c0 = pt + Rt[:, 0] * BALL_OFF
        Rb = np.stack([up, np.cross([0, 1, 0], up) if abs(up[1]) < 0.9 else Rt[:, 1], np.zeros(3)], 1)
        y = Rt[:, 1]; x = up; z = np.cross(x, y); y = np.cross(z, x)
        Rb = np.stack([x, y, z], 1)
        world += (f'<body name="ball" pos="{v2s(c0 + up * 0.0002)}" quat="{q2s(Rb)}"><freejoint name="ballj"/>'
                  f'<geom name="ball" type="sphere" size="{R_BALL}" mass="0.050" friction="0.05 0.005 0.0005" contype="1" conaffinity="1"/>'
                  f'<site name="ballc"/>'
                  f'<body name="pod" pos="{R_BALL + 0.012:.4f} 0 0"><geom type="cylinder" fromto="{-0.012:.3f} 0 0 0 0 0" size="0.0055" mass="0.004" contype="0" conaffinity="0"/>{pod}</body></body>')
        tendon = ('<tendon><spatial name="tether" limited="true" range="0 0.13" width="0.001" solreflimit="0.004 1">'
                  '<site site="collar"/><site site="ballc"/></spatial></tendon>')
    else:
        tendon = ""
    # impactor: 0.2 kg sphere (a branch / thrown object), gravity-compensated so it flies straight
    a0 = 22.5 * DEG; dh = np.array([-np.sin(a0), np.cos(a0), 0])
    tgt = pt + up * 0.07 + Rt[:, 0] * (BALL_OFF if variant == "magnet" else 0.07)
    start = tgt - dh * 0.12
    world += (f'<body name="imp" pos="{v2s(start)}" gravcomp="1"><freejoint name="impj"/>'
              f'<geom name="imp" type="sphere" size="0.02" mass="{imp_m}" contype="1" conaffinity="1"/></body>')
    acts = "".join(f'<position name="s{k}" joint="j{k}" kp="{STALL[k] / (2 * DEG):.2f}" kv="{STALL[k] / (2 * DEG) * 0.02:.3f}" '
                   f'forcerange="-{STALL[k]} {STALL[k]}" forcelimited="true"/>' for k in range(9))
    acts += "".join(f'<position name="w_{j}" joint="{j}" kp="{s / (2 * DEG):.2f}" kv="{s / (2 * DEG) * 0.02:.3f}" '
                    f'forcerange="-{s} {s}" forcelimited="true"/>' for j, s in act)
    full = f"""<mujoco model="cephalo_wrist_{variant}">
<option timestep="0.0002" gravity="0 0 -9.81" integrator="implicitfast"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001" friction="0.5 0.01 0.001"/></default>
<worldbody>{world}</worldbody>
{tendon}
<equality>{''.join(eq)}</equality>
<actuator>{acts}</actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(full)
    d = mujoco.MjData(m)
    for k in range(9):
        d.joint(f"j{k}").qpos = FLIGHT[k]
    if variant == "knuckle":
        d.joint("h1").qpos = U_UP_PITCH / 2; d.joint("h2").qpos = U_UP_PITCH / 2
    ctrl = np.concatenate([FLIGHT, [0.0, U_UP_PITCH / 2] if variant == "knuckle" else []])
    d.ctrl[:] = ctrl
    d.joint("impj").qvel[:3] = 0
    return m, d, dict(dh=dh, imp_v=imp_v, Fc=Fc, variant=variant, start=start)


class Sim:
    def __init__(self, variant, Fc=30.0, imp_m=0.2, imp_v=4.0, t_hit=0.5, yank=None, g0=0.002, reel=1.5):
        self.m, self.d, self.info = build(variant, Fc, imp_m, imp_v)
        self.variant, self.Fc, self.g0, self.reel = variant, Fc, g0, reel
        self.t_hit, self.imp_v, self.yank = t_hit, imp_v, yank
        m = self.m
        self.b_pod = m.body("pod").id; self.b_imp = m.body("imp").id; self.b_tip = m.body("tip").id
        self.launched = False
        mujoco.mj_forward(m, self.d)
        self.info["start"] = self.d.geom("prop").xpos.copy() - self.info["dh"] * 0.12   # aim at the prop disc
        if variant == "magnet":
            self.b_ball = m.body("ball").id
            self.s_mag = m.site("magnet").id; self.s_col = m.site("collar").id
        self.log = []
        self.detached_at = None; self.reseated_at = None

    def step_forces(self):
        m, d = self.m, self.d
        d.xfrc_applied[:] = 0
        t = d.time
        # park the impactor until launch
        if not self.launched:
            j = m.joint("impj").qposadr[0]; d.qpos[j:j + 3] = self.info["start"]; d.qvel[m.joint("impj").dofadr[0]:][:6] = 0
            if t >= self.t_hit - 0.12 / max(self.imp_v, 1e-6) and self.imp_v > 0:
                d.qvel[m.joint("impj").dofadr[0]:][:3] = self.info["dh"] * self.imp_v; self.launched = True
        if self.imp_v == 0 and not self.launched:
            j = m.joint("impj").qposadr[0]; d.qpos[j:j + 3] = [5, 5, -5]
        axis = d.xmat[self.b_pod].reshape(3, 3)[:, 0]
        on = True
        gap = 0.0
        if self.variant == "magnet":
            c = d.xpos[self.b_ball]; mg = d.site_xpos[self.s_mag]
            dist = np.linalg.norm(mg - c)
            gap = max(dist - (R_BALL + 0.007), 0.0)
            F = self.Fc / (1 + gap / self.g0) ** 2 if gap < 0.06 else 0.0
            d.xfrc_applied[self.b_ball, :3] += F * (mg - c) / dist
            d.xfrc_applied[self.b_tip, :3] -= F * (mg - c) / dist
            on = gap < 0.004                          # the tip node cuts the fan once the ball leaves the seat
            if on:
                # wheel contacts are near-frictionless (rollers); the three driven omni wheels steer the ball back to fan-up; torque limited by wheel traction
                # (mu 1.0 x magnet preload x ball radius) and by three N20 298:1 gearmotors (~0.25 N m each, est.)
                a = d.xmat[self.b_ball].reshape(3, 3)[:, 0]
                tdir = d.xmat[self.b_tip].reshape(3, 3)[:, 0]          # hold the pod in line with the tip, like a servo
                cx = np.cross(a, tdir); sn = np.linalg.norm(cx)
                err = cx / max(sn, 1e-9) * np.arctan2(sn, a @ tdir)
                w = d.cvel[self.b_ball, :3] - d.cvel[self.b_tip, :3]
                tau = 2.0 * err - 0.08 * w
                tmax = min(1.0 * F * R_BALL, 0.35)   # capped below the 0.44 N m stall of the STS3032s it reacts against
                n = np.linalg.norm(tau)
                if n > tmax:
                    tau *= tmax / n
                d.xfrc_applied[self.b_ball, 3:] += tau; d.xfrc_applied[self.b_tip, 3:] -= tau
            if gap > 0.004:
                cl = d.site_xpos[self.s_col]; v = cl - c
                d.xfrc_applied[self.b_ball, :3] += self.reel * v / np.linalg.norm(v)   # badge reel spring
            if gap > 0.004 and self.detached_at is None:
                self.detached_at = d.time
            if self.detached_at is not None and self.reseated_at is None and gap < 0.0005 and d.time > self.detached_at + 0.05:
                self.reseated_at = d.time
        # fail-safe in the tip node: cut the fan while the pod is knocked > 25 deg off vertical, so off-axis thrust
        # cannot back-drive the small arm servos (without it a 4 m/s hit folds j5 to its limit)
        if np.degrees(np.arccos(np.clip(axis[2], -1, 1))) > 25:        # pod IMU: more than 25 deg off vertical
            on = False
        self.fan_on = on
        if on:
            d.xfrc_applied[self.b_pod, :3] += THRUST * axis
        if self.yank is not None:
            t0, rate = self.yank
            if t >= t0:
                d.xfrc_applied[self.b_pod, :3] += np.array([0, 0, 1.0]) * rate * (t - t0)
        self.gap = gap

    def loads(self):
        """gear-mesh torque (knuckle), pin force at hinge 1, actuator torques, tether tension."""
        m, d = self.m, self.d
        out = {}
        mujoco.mj_rnePostConstraint(m, d)
        if self.variant == "knuckle":
            eqf = 0.0
            for i in range(d.nefc):
                if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY:
                    eqf = abs(d.efc_force[i])
            out["gear_Nm"] = eqf
            h1 = m.body("h1").id
            out["pin_N"] = np.linalg.norm(d.cfrc_int[h1, 3:])
            out["pitch_servo"] = abs(d.actuator("w_h1").force[0]); out["roll_servo"] = abs(d.actuator("w_roll").force[0])
        else:
            ten = 0.0
            for i in range(d.nefc):
                if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON:
                    ten = abs(d.efc_force[i])
            out["tether_N"] = ten
            out["gap_mm"] = self.gap * 1000
        out["fan_on"] = float(getattr(self, "fan_on", True))
        out["j8"] = abs(d.actuator("s8").force[0]); out["j0"] = abs(d.actuator("s0").force[0])
        imp_pod = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if self.b_imp in (b1, b2):
                f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f); imp_pod += abs(f[0])
        out["impact_N"] = imp_pod
        ax = d.xmat[self.b_pod].reshape(3, 3)[:, 0]
        out["tilt_deg"] = np.degrees(np.arccos(np.clip(ax[2], -1, 1)))
        return out

    def run(self, T, rec_dt=None, cb=None):
        mujoco.mj_forward(self.m, self.d)
        nxt = 0.0
        while self.d.time < T:
            self.step_forces()
            mujoco.mj_step(self.m, self.d)
            if rec_dt is not None and self.d.time >= nxt:
                self.log.append((self.d.time, self.loads(), self.snapshot()))
                nxt += rec_dt
        return self

    def snapshot(self):
        d, m = self.d, self.m
        s = dict(q=np.array([d.joint(f"j{k}").qpos[0] for k in range(9)]),
                 pod_p=d.xpos[self.b_pod].copy(), pod_R=d.xmat[self.b_pod].reshape(3, 3).copy(),
                 imp=d.xpos[self.b_imp].copy())
        if self.variant == "knuckle":
            s["roll"] = d.joint("roll").qpos[0]; s["h1"] = d.joint("h1").qpos[0]; s["h2"] = d.joint("h2").qpos[0]
        else:
            s["ball_p"] = d.xpos[self.b_ball].copy(); s["ball_R"] = d.xmat[self.b_ball].reshape(3, 3).copy()
        return s
