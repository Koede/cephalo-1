"""Tensioned breakaway seats on the Cephalo-1 arm (MuJoCo), knuckle + roll wrist at the tip.

Hardware per seat (see claude/breakaway-joint.md): printed PETG male/female 45 deg cone seat with a flat rim of radius r_i,
a central Ø4 mm bore for the tension line, and two 1 mm Dyneema limit straps that stop the seat at 25 deg / 8 mm open.
One 1.5 mm Dyneema tension line runs down the arm centre through every seat (and through hollow brass hinge pins),
anchored at the tip coupler, with a series spring (k_c) at the root and a self-locking JGY-370 worm-gear winch.

Seat model (per seat, 3 DOF in series with the arm): rocking about two axes + axial separation.
  closed : stiff spring (preload holds the rim shut), k = T r / 0.15 deg
  open   : restoring moment  M = T(theta, x) * r,  axial force  F = T(theta, x)
           T = T0 + k_c * (r*theta + x)    (opening stretches the line by about r*theta + x)
  stop   : joint range = the Dyneema limit straps.
"""
import numpy as np, mujoco
import sim_wrist as SW
from robot import arm_base, J_POS, J_TYPE, SPIGOT, arm_fk

DEG = np.pi / 180
# baseline arm = rev 3 with the recommended j5 upgrade (STS3215), from the wrist hit test
SW.STALL = [4.9] * 3 + [2.9] * 3 + [0.44] * 3
SW.SERVO_M = [0.076] * 3 + [0.055] * 3 + [0.020] * 3
SW.BRACKET = 0.647 - (3 * 0.076 + 2 * 0.055 + 4 * 0.020)
# seats: position along the arm (m from the wall face) and rim radius (m); root carries most, tip least
SEATS = [(0.012, 0.030), (0.120, 0.026), (0.238, 0.022), (0.332, 0.018), (0.402, 0.014)]
K_CORD = 4000.0            # N/m: Dyneema line + series spring (a printed TPU spring or 2 shock-cord loops), est.
THETA_MAX, X_MAX = 25 * DEG, 0.008
MODES = {"soft": 25.0, "fly": 60.0, "walk": 150.0}
# clutch variant: breakaway torque per joint = C_i x line tension; set to 0.7 x servo stall at the 60 N 'firm' tension
CLUTCH_MODES = {"soft": 25.0, "firm": 60.0}
# reflected rotor inertia of each servo (motor rotor x gear ratio^2, ~1:345), kg m^2 - rough estimates, not datasheet
ARMATURE = [0.02] * 3 + [0.012] * 3 + [0.003] * 3
GEAR_STRIP = [3 * s for s in ([4.9] * 3 + [2.9] * 3 + [0.44] * 3)]    # assumed gear-train limit ~3x stall (unknown)
C_CLUTCH = [0.7 * s / 60.0 for s in ([4.9] * 3 + [2.9] * 3 + [0.44] * 3)]


def arm_xml_seats(seats=True):
    R0, p0 = arm_base(0)
    xml = [f'<body name="a0" pos="{SW.v2s(p0 + [0, 0, SW.BODY_Z])}" quat="{SW.q2s(R0)}">']
    events = [(x, "j", k) for k, x in enumerate(J_POS)]
    if seats:
        events += [(x, "s", i) for i, (x, r) in enumerate(SEATS)]
    events.sort()
    prev, depth = 0.0, 1
    seg_total = SPIGOT
    marks = [e[0] for e in events] + [SPIGOT]
    for n, (x, kind, k) in enumerate(events):
        L = marks[n + 1] - x
        if kind == "j" and CLUTCH:
            m = SW.SERVO_M[k] + SW.BRACKET * L / seg_total
            ax = "0 1 0" if J_TYPE[k] == "p" else "0 0 1"
            rng = "-90 90" if k > 1 else ("-41 41" if k == 0 else "-30 30")
            sz = 0.016 - 0.007 * k / 9
            box = "0.020 0.017 0.015" if k < 5 else "0.012 0.013 0.012"
            xml.append(f'<body name="l{k}" pos="{x - prev:.5f} 0 0">'
                       f'<joint name="j{k}" axis="{ax}" range="{rng}" damping="0.02" armature="{ARMATURE[k]}"/>'
                       f'<geom type="box" size="{box}" mass="{SW.SERVO_M[k]:.4f}" contype="0" conaffinity="0" rgba="0.2 0.2 0.22 1"/>'
                       f'<body name="cb{k}"><joint name="c{k}" axis="{ax}" range="-{CLUTCH_RANGE} {CLUTCH_RANGE}" damping="0.01" armature="0.001"/>'
                       f'<geom type="cylinder" fromto="0 -0.004 0 0 0.004 0" size="0.012" mass="0.004" contype="0" conaffinity="0" rgba="0.95 0.55 0.15 1"/>'
                       f'<geom type="capsule" fromto="0 0 0 {L:.5f} 0 0" size="{sz:.4f}" mass="{SW.BRACKET * L / seg_total:.4f}" contype="0" conaffinity="0" rgba="0.9 0.9 0.88 1"/>')
            depth += 1
        elif kind == "j":
            m = SW.SERVO_M[k] + SW.BRACKET * L / seg_total
            ax = "0 1 0" if J_TYPE[k] == "p" else "0 0 1"
            rng = "-90 90" if k > 1 else ("-41 41" if k == 0 else "-30 30")
            sz = 0.016 - 0.007 * k / 9
            box = "0.020 0.017 0.015" if k < 5 else "0.012 0.013 0.012"
            xml.append(f'<body name="l{k}" pos="{x - prev:.5f} 0 0">'
                       f'<joint name="j{k}" axis="{ax}" range="{rng}" damping="0.02" armature="{ARMATURE[k]}"/>'
                       f'<geom type="box" size="{box}" mass="{SW.SERVO_M[k]:.4f}" contype="0" conaffinity="0" rgba="0.2 0.2 0.22 1"/>'
                       f'<geom type="capsule" fromto="0 0 0 {L:.5f} 0 0" size="{sz:.4f}" mass="{SW.BRACKET * L / seg_total:.4f}" contype="0" conaffinity="0" rgba="0.9 0.9 0.88 1"/>')
        else:
            r = SEATS[k][1]
            xml.append(f'<body name="seat{k}" pos="{x - prev:.5f} 0 0">'
                       f'<joint name="s{k}a" axis="0 1 0" range="-25 25" damping="0.01" armature="0.0005"/>'
                       f'<joint name="s{k}b" axis="0 0 1" range="-25 25" damping="0.01" armature="0.0005"/>'
                       f'<joint name="s{k}x" type="slide" axis="1 0 0" range="-0.0005 {X_MAX}" damping="0.5" armature="0.005"/>'
                       f'<geom type="cylinder" fromto="0 0 0 0.004 0 0" size="{r:.4f}" mass="0.006" contype="0" conaffinity="0" rgba="0.95 0.55 0.15 1"/>'
                       f'<geom type="capsule" fromto="0.004 0 0 {max(L, 0.005):.5f} 0 0" size="0.010" mass="0.001" contype="0" conaffinity="0" rgba="0.9 0.9 0.88 1"/>'
                       f'<site name="seat{k}c"/>')
        depth += 1
        prev = x
    return xml, prev, depth


CLUTCH = False
CLUTCH_RANGE = 45
TELE_PRELOAD = None      # N; None = use the line tension
TELE_K = 600.0          # N/m rate of the telescope return spring


def build(seats=True, imp_v=4.0, imp_m=0.2, clutch=False):
    global CLUTCH
    CLUTCH = clutch
    xml, prev, depth = arm_xml_seats(seats and not clutch)
    pod = (f'<geom name="motor" type="cylinder" fromto="0.004 0 0 0.026 0 0" size="0.0145" mass="{SW.POD_M * 0.6:.3f}" contype="1" conaffinity="1" rgba="0.55 0.57 0.6 1"/>'
           f'<geom name="prop" type="cylinder" fromto="0.032 0 0 0.035 0 0" size="0.0635" mass="{SW.POD_M * 0.4:.3f}" contype="1" conaffinity="1" rgba="0.25 0.27 0.3 1"/>')
    tip = (f'<body name="tip" pos="{SPIGOT - prev:.5f} 0 0"><site name="tipc"/>'
           '<geom type="box" pos="0.016 0 0" size="0.016 0.012 0.0065" mass="0.024" contype="0" conaffinity="0" rgba="0.62 0.64 0.67 1"/>'
           + ('<body name="tele"><joint name="tx" type="slide" axis="1 0 0" range="-0.0005 0.015" damping="0.5" armature="0.005"/>'
              '<geom type="cylinder" fromto="0.002 0 0 0.010 0 0" size="0.009" mass="0.004" contype="0" conaffinity="0" rgba="0.95 0.55 0.15 1"/>' if clutch else '') +
           '<body name="rollb" pos="0.036 0 0"><joint name="roll" axis="1 0 0" range="-180 180" damping="0.01" armature="0.0005"/>'
           '<geom type="cylinder" fromto="0 0 0 0.010 0 0" size="0.012" mass="0.008" contype="0" conaffinity="0" rgba="0.2 0.5 0.85 1"/>'
           '<body name="h1" pos="0.010 0 0"><joint name="h1" axis="0 -1 0" range="-88 88" damping="0.01" armature="0.0005"/>'
           '<geom type="box" pos="0.012 0 0" size="0.012 0.013 0.008" mass="0.030" contype="0" conaffinity="0" rgba="0.2 0.21 0.23 1"/>'
           '<body name="h2" pos="0.024 0 0"><joint name="h2" axis="0 -1 0" range="-88 88" damping="0.005"/>'
           '<geom type="cylinder" fromto="0 0 0 0.016 0 0" size="0.0065" mass="0.006" contype="0" conaffinity="0" rgba="0.2 0.21 0.23 1"/>'
           f'<body name="pod" pos="0.016 0 0">{pod}</body></body></body></body>' + ('</body>' if clutch else '') + '</body>')
    xml.append(tip)
    xml += ["</body>"] * (depth)
    world = "\n".join(xml)
    _, (Rt, pt) = arm_fk(0, SW.FLIGHT, np.array([0, 0, SW.BODY_Z]))
    a0 = 22.5 * DEG; dh = np.array([-np.sin(a0), np.cos(a0), 0])
    world += (f'<body name="imp" pos="5 5 -5" gravcomp="1"><freejoint name="impj"/>'
              f'<geom name="imp" type="sphere" size="0.02" mass="{imp_m}" contype="1" conaffinity="1" rgba="0.85 0.15 0.1 1"/></body>')
    acts = "".join(f'<position name="s{k}" joint="j{k}" kp="{SW.STALL[k] / (2 * DEG):.2f}" kv="{SW.STALL[k] / (2 * DEG) * 0.02:.3f}" '
                   f'forcerange="-{SW.STALL[k]} {SW.STALL[k]}" forcelimited="true"/>' for k in range(9))
    acts += "".join(f'<position name="w_{j}" joint="{j}" kp="{0.44 / (2 * DEG):.2f}" kv="{0.44 / (2 * DEG) * 0.02:.3f}" '
                    f'forcerange="-0.44 0.44" forcelimited="true"/>' for j in ("roll", "h1"))
    full = f"""<mujoco model="cephalo_seats">
<option timestep="0.0002" gravity="0 0 -9.81" integrator="implicitfast"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001" friction="0.5 0.01 0.001"/><joint limited="true"/></default>
<worldbody>{world}</worldbody>
<equality><joint joint1="h2" joint2="h1" polycoef="0 1 0 0 0" solref="0.002 1"/></equality>
<actuator>{acts}</actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(full)
    d = mujoco.MjData(m)
    for k in range(9):
        d.joint(f"j{k}").qpos = SW.FLIGHT[k]
    d.ctrl[:] = np.concatenate([SW.FLIGHT, [0.0, 0.0]])
    return m, d, dh


class SeatSim:
    def __init__(self, seats=True, T0=60.0, imp_v=0.0, t_hit=0.5, yank=None, side=None, clutch=False):
        self.clutch = clutch and seats
        self.m, self.d, self.dh = build(seats, imp_v, clutch=self.clutch)
        self.seats, self.T0, self.imp_v, self.t_hit, self.yank, self.side = seats, T0, imp_v, t_hit, yank, side
        m = self.m
        self.b_pod, self.b_imp = m.body("pod").id, m.body("imp").id
        mujoco.mj_forward(m, self.d)
        self.start = self.d.geom("prop").xpos.copy() - self.dh * 0.12
        self.launched = False
        if self.clutch:
            self.cj = [(m.joint(f"c{k}").dofadr[0], m.joint(f"c{k}").qposadr[0]) for k in range(9)]
            self.tj = (m.joint("tx").dofadr[0], m.joint("tx").qposadr[0])
        elif seats:
            self.sj = [(m.joint(f"s{i}a").dofadr[0], m.joint(f"s{i}b").dofadr[0], m.joint(f"s{i}x").dofadr[0],
                        m.joint(f"s{i}a").qposadr[0], m.joint(f"s{i}b").qposadr[0], m.joint(f"s{i}x").qposadr[0]) for i in range(len(SEATS))]
        self.log = []

    def seat_forces(self):
        d = self.d
        d.qfrc_applied[:] = 0
        if not self.seats:
            return 0.0
        if self.clutch:
            # detent clutch: V-notch roller pressed by the line; breakaway torque = C_i * T, follower lift 3 mm per 45 deg
            slip = sum(abs(d.qpos[q]) for _, q in self.cj)
            tx = max(d.qpos[self.tj[1]], 0)
            T = self.T0 + K_CORD * (0.003 / (45 * DEG) * slip + tx)
            self.T = T
            for k, (dd, q) in enumerate(self.cj):
                phi = d.qpos[q]; tb = C_CLUTCH[k] * T
                kc = C_CLUTCH[k] * self.T0 / (0.2 * DEG)
                d.qfrc_applied[dd] -= np.sign(phi) * min(kc * abs(phi), tb)
            # telescoping wrist: own preload spring (TELE_PRELOAD, holds full thrust), extends up to 15 mm, line pulls it back
            dd, q = self.tj; x = d.qpos[q]; P = TELE_PRELOAD if TELE_PRELOAD else T; k_ax = P / 0.0002
            d.qfrc_applied[dd] -= min(k_ax * x, P + TELE_K * x) if x > 0 else k_ax * x
            return T
        open_sum = 0.0
        for i, (da, db, dx, qa, qb, qx) in enumerate(self.sj):
            r = SEATS[i][1]
            a, b, x = d.qpos[qa], d.qpos[qb], d.qpos[qx]
            th = np.hypot(a, b)
            open_sum += r * th + max(x, 0)
        T = self.T0 + K_CORD * open_sum          # one line through every seat: same tension everywhere
        self.T = T
        for i, (da, db, dx, qa, qb, qx) in enumerate(self.sj):
            r = SEATS[i][1]
            a, b, x = d.qpos[qa], d.qpos[qb], d.qpos[qx]
            th = np.hypot(a, b)
            k_closed = self.T0 * r / (0.15 * DEG)
            M = min(k_closed * th, T * r) if th > 1e-9 else 0.0
            if th > 1e-9:
                d.qfrc_applied[da] -= M * a / th; d.qfrc_applied[db] -= M * b / th
            k_ax = self.T0 / 0.0002
            d.qfrc_applied[dx] -= min(k_ax * max(x, 0), T) if x > 0 else k_ax * x
        return T

    def step(self):
        m, d = self.m, self.d
        T = self.seat_forces()
        d.xfrc_applied[:] = 0
        j = m.joint("impj")
        if not self.launched:
            d.qpos[j.qposadr[0]:j.qposadr[0] + 3] = self.start if self.imp_v > 0 else [5, 5, -5]
            d.qvel[j.dofadr[0]:j.dofadr[0] + 6] = 0
            if self.imp_v > 0 and d.time >= self.t_hit - 0.12 / self.imp_v:
                d.qvel[j.dofadr[0]:j.dofadr[0] + 3] = self.dh * self.imp_v; self.launched = True
        ax = d.xmat[self.b_pod].reshape(3, 3)[:, 0]
        on = np.degrees(np.arccos(np.clip(ax[2], -1, 1))) < 25
        self.fan_on = on
        if on:
            d.xfrc_applied[self.b_pod, :3] += SW.THRUST * ax
        if self.yank is not None:                 # fan runs away / snag: extra pull along the pod axis
            t0, F, dur = self.yank
            if t0 <= d.time < t0 + dur:
                d.xfrc_applied[self.b_pod, :3] += F * ax * min(1.0, (d.time - t0) / 0.05)
        if self.side is not None:                 # steady side load for the stiffness check
            d.xfrc_applied[self.b_pod, :3] += self.side
        mujoco.mj_step(m, d)

    def measure(self):
        m, d = self.m, self.d
        out = {k: abs(d.actuator(f"s{k}").force[0]) / SW.STALL[k] for k in range(9)}
        # torque the servo's gear train carries = motor torque + torque to accelerate the reflected rotor inertia
        for k in range(9):
            dof = m.joint(f"j{k}").dofadr[0]
            out[f"g{k}"] = (abs(d.actuator(f"s{k}").force[0]) + ARMATURE[k] * abs(d.qacc[dof])) / GEAR_STRIP[k]
        out["T"] = getattr(self, "T", self.T0) if self.seats else 0.0
        if self.clutch:
            out["open_deg"] = max(np.degrees(abs(d.qpos[q])) for _, q in self.cj)
            out["ext_mm"] = 1000 * max(d.qpos[self.tj[1]], 0)
        elif self.seats:
            out["open_deg"] = max(np.degrees(np.hypot(d.qpos[a], d.qpos[b])) for _, _, _, a, b, _ in self.sj)
            out["ext_mm"] = 1000 * sum(max(d.qpos[x], 0) for *_, x in self.sj)
        else:
            out["open_deg"] = 0.0; out["ext_mm"] = 0.0
        imp = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if self.b_imp in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f); imp += abs(f[0])
        out["impact_N"] = imp
        ax = d.xmat[self.b_pod].reshape(3, 3)[:, 0]
        out["tilt"] = np.degrees(np.arccos(np.clip(ax[2], -1, 1)))
        out["qdev"] = float(np.degrees(np.max(np.abs(np.array([d.joint(f"j{k}").qpos[0] for k in range(9)]) - SW.FLIGHT))))
        out["fan"] = float(self.fan_on)
        out["podz"] = d.xpos[self.b_pod][2]
        return out

    def run(self, T, rec=0.001):
        nxt = 0.0
        while self.d.time < T:
            self.step()
            if self.d.time >= nxt:
                self.log.append((self.d.time, self.measure(), (self.d.xpos.copy(), self.d.xmat.copy())))
                nxt += rec
        return self
