"""Heavy falls and recovery: Cephalo-1 loses its thrust (or is knocked over) in flight, braces, absorbs the impact
in its clutches and bumpers, then gets back up.

Robot: body4 / flip4 model (full MuJoCo, battery sag, prop torque, clutches) with the XL claw-props, 1 pack and an
STS3250 at j1 (as for the flip).  For falls the arm links, pods and dome get floor collision (in the other sims only the
toes, EDF can and plate touch the floor); the EDF-can bumper and dome get a TPU-like soft contact.

Scenarios (python3 fall4.py <name>):
  cut      hover upright at 1.5 m, every motor cuts out (ESC brown-out)                 -> free fall upright
  hit      hover upright at 1.5 m, a side blow tips it (impulse ~11 N m s); past 45 deg of tilt the ESC
           failsafe stops every motor                                                   -> tumbling fall
  inv      hover upside down at 1.2 m (after the flip), every motor cuts out             -> falls dome first
  drift    flying sideways at 2.5 m/s, 1.0 m up, every motor cuts out                   -> lands sliding
Reflexes and recovery (the same controller for every case):
  * free fall is sensed (specific force < 3 m/s^2: the IMU reads ~0 g): every arm whose root is not on top of the body
    reaches its foot down and out under the body (brace), servo speed limit raised to the 12 V no-load line;
    winch soft, so the detent clutches slip (0.375 x stall j2-j7, 0.5 x stall j0) before any servo is back-driven
    (at 'firm' the j0 clutch, 1.2 x stall, is stronger than its servo);
  * impact is sensed (floor contact): the brace targets freeze; once settled the winch re-seats the clutches gently;
  * once settled the body's tilt decides the recovery:
      tilt < 50 deg   -> body4 GetUp (arms roll in, place the feet, rise on 8 feet), then stand;
      tilt >= 50 deg  -> thrust-assisted righting: the motors restart, the arms take the flip path pose for that tilt
                         (arm labels rotated to the nearest arm-pair axis, yaw chosen to fit the actual attitude), the
                         robot lifts off on its side / upside down, climbs to 1 m and rolls back upright along the
                         flip path in reverse, then hovers (claw-props pre-load before lift-off; if it rolls away
                         on the ground the motors stop and it tries again).
Hardware (FALL_HW): "reinforced" (default, 2026-11-11) = ride-over clutch cams (rideover_parts.py: a V every 90 deg,
60 deg at j0/j1; past the crest the roller drops into the next V instead of hitting a stop), a clutch at j1, TPU pads on
the link-to-link stops, a Hall switch per clutch (home notch). After the fall any skipped notch is re-indexed: the servo
drives toward the link's own stop on that side, the link arrives first and is held, the roller rides back into the
home V (Hall switch says when to stop). A notch watchdog interrupts the get-up / pre-lift if a roller skips again.
"rev4" = the rev 4 clutches (single V, hard cam stops, j1 rigid).
Model refinements (both hardware sets): joint limits act on the link angle (servo half + clutch + other half, a fixed
tendon = the link shells meeting), not on the servo pivot; clutch-joint armature 5e-5 (sim4's 5e-4 is numerical and
coupled ~10 N m of fake torque into the rotor at fast slips).
Variants: FALL_J1CLUTCH, FALL_RIDE, FALL_SHELL, FALL_PROG, FALL_STOPX, FALL_TLIM, FALL_LIFTTLIM, FALL_HITM, FALL_HITDIR, ...
Summary of a batch: fall_summary.py.
"""
import os, sys, json, time
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "1"); os.environ.setdefault("CEPH_J1", "STS3250")
import numpy as np, mujoco
from scipy.optimize import lsq_linear
import cephalo4 as C
import power4 as PW
if os.environ.get("FALL_XLGEOM", "1") == "1":       # simulate the XL claw's real size (toes, folded fingers, prop disc): the
    C.CLAW_R = PW.PROPS["xl"]["R"]                  # other sims keep the 6 in geometry with XL thrust and mass
import flip4 as F4
import sim4 as S4
import body4 as B4
import power4 as PW

D = C.DEG; mj = B4.mj; Z = np.array([0, 0, 1.0])
# Hardware set: "reinforced" (2026-11-11, default): ride-over cams at every clutch, a clutch at j1 too, TPU pads on the link
# stops; "rev4": the rev 4 clutches (hard cam stops, j1 rigid). Both use the refined joint model (link-angle shell stops,
# no numerical clutch armature).
HW = os.environ.get("FALL_HW", "reinforced")
os.environ.setdefault("FALL_SHELL", "1"); os.environ.setdefault("FALL_CARM", "0.00005")
if HW == "reinforced":
    os.environ.setdefault("FALL_RIDE", "1"); os.environ.setdefault("FALL_J1CLUTCH", "1"); os.environ.setdefault("FALL_LIFTTLIM", "0.6")
if os.environ.get("FALL_J1CLUTCH"):                  # design variant: a detent clutch at the root yaw j1 too (rigid in rev 4)
    C.CLUTCH_FRAC[1] = 0.9; C.CLUTCH_TRAVEL[1] = np.radians(25)   # same 6-notch cam as j0 (60 deg pitch)
if os.environ.get("FALL_TRAVEL"):                     # design variant: clutch travel j2-j7 (deg), default 40
    C.CLUTCH_TRAVEL[2:] = np.radians(float(os.environ["FALL_TRAVEL"]))
if os.environ.get("FALL_J0TRAVEL"):                  # design variant: root clutch travel (deg), default 25
    C.CLUTCH_TRAVEL[0] = np.radians(float(os.environ["FALL_J0TRAVEL"]))
# dome as a crown sphere + a ring of 8 spheres (floor contact only), soft (TPU pad / bumper ring)
_ring = "".join(f'<geom name="dome_r{j}" type="sphere" pos="{0.085 * np.cos(np.radians(45 * j)):.4f} {0.085 * np.sin(np.radians(45 * j)):.4f} 0.045" '
                f'size="0.040" mass="0" contype="0" conaffinity="8"/>' for j in range(8))
S4.TORSO_XML = '<geom name="dome" type="sphere" pos="0 0 0.066" size="0.090" mass="0" contype="0" conaffinity="8"/>' + _ring
SOFT = np.array([0.012, 0.9])                        # bumper / dome pad contact: ~12 ms time constant (TPU, est.)
TOE_SOFT = np.array([float(os.environ.get("FALL_TOE", 0.010)), 0.9])   # TPU fingertip caps (~10 ms, est.)
LINK_SOFT = np.array([0.006, 1.0])
LINK_R = float(os.environ.get("FALL_LINKR", 0.0315))   # link contact radius = drawn envelope (servo + clutch cam)
FALL_WINCH = os.environ.get("FALL_WINCH", "soft")    # winch mode for falls (clutch slip moment)
BRACE_R, BRACE_DZ = float(os.environ.get("FALL_R", 0.36)), float(os.environ.get("FALL_DZ", 0.30))   # brace foot: 0.36 m out, 0.30 m below the plate
V_EMERG, A_EMERG = 230 * D, 2500 * D                 # free-fall arm limits (0.85 x STS3215 no-load line, unloaded arm)
Z_RIGHT = 1.0
HIT_M = float(os.environ.get("FALL_HITM", 36.0))
HIT_DIR = np.radians(float(os.environ.get("FALL_HITDIR", 0.0)))   # direction of the blow (about z)
INV_TILT = 140.0                                     # above this tilt the righting starts fully inverted
PRELOAD_T = 1.5                                      # claw-props carry 45 -> 85 % of the weight before lift-off
T_REARM = float(os.environ.get("FALL_REARM", 0.0))   # arms move to the flip pose while the motors spin up (0: no separate stage)
I_ROLL = 0.5
# reinforcement options
PROG = float(os.environ.get("FALL_PROG", 0.0))      # progressive cam: slip torque x (1 + PROG (phi/travel)^2)
STOP_X = np.radians(float(os.environ.get("FALL_STOPX", 0.0)))   # TPU end-stop pad compression range beyond the travel (deg)
STOP_K = float(os.environ.get("FALL_STOPK", 6.0))   # pad stiffness, x stall per rad
STOP_C = float(os.environ.get("FALL_STOPC", 0.05))  # pad damping, x stall per rad/s (compressing only)
SHELL_STOP = os.environ.get("FALL_SHELL", "0") == "1"   # link-angle stops (model fix) instead of stops on each servo pivot half
SHELL_SOFT = float(os.environ.get("FALL_SHELLSOFT", 0.008))  # shell stop contact time constant (TPU stop pads, est.)
SHELL_SOFT0 = float(os.environ.get("FALL_SHELLSOFT0", SHELL_SOFT))   # root pitch j0 stop against the mantle
RIDE = os.environ.get("FALL_RIDE", "0") == "1"      # ride-over cam (torque fuse): no clutch stop, the roller skips to the next V
RIDE_GAP = np.radians(5.0)
LIFT_TLIM = float(os.environ.get("FALL_LIFTTLIM", 1.0))
# Kickstand take-off (2026-11-12): dome-down, KICK_N arms fold into struts: the root link swings down
# until the j2 housing stands on the floor ~0.23 m from the body axis, the rest of the arm folds up out of the way. Only
# j0 carries the load (lever ~7 cm: up to ~65 N per strut at stall). An IMU loop on j0 levels the body on dome + struts,
# the other arms rise root-first with props off, then all props spin up (the folded strut tips point up, so their
# props thrust too) and the robot lifts off; the struts unfold into the flight pose once airborne. 0 = off.
KICK_N = int(os.environ.get("FALL_KICK", 3))
KICK_TLIM = float(os.environ.get("FALL_KICKTLIM", 0.6))       # servo torque limit on the ground in a kickstand take-off (strut roots: full)
Q_STRUT = np.radians([-48.0, 0, 90, 0, 50, 0, 0, 0])          # j0 at floor contact (level body), distal arm folded up
J0_PRE = np.radians(-15.0)
KICK_KP, KICK_KI = float(os.environ.get("FALL_KICKKP", 1.2)), float(os.environ.get("FALL_KICKKI", 3.0))   # rad per rad of tilt, and per rad s
T_FOLD = 0.9; T_PLANT = 1.5; T_RISE = 1.5; KICK_PRELOAD = 0.8
# Wobble + one-leg kickstand (2026-11-12, the default): the organic version. The arms rise, the props run below weight,
# and the body is steered round in a circle on its dome (attitude reference coning at WOB_TILT, WOB_HZ turns per second).
# Then the arm on the low side pops out as a single strut (prop stopped), props the body up level, and the robot takes off.
KICK_STYLE = os.environ.get("FALL_STYLE", "wobble")           # "wobble" (1 leg) | "tripod" (3 struts, no wobble)
WOB_TILT = np.radians(float(os.environ.get("FALL_WOBTILT", 10.0)))
WOB_HZ = float(os.environ.get("FALL_WOBHZ", 0.5)); WOB_TURNS = float(os.environ.get("FALL_WOBTURNS", 1.25))
WOB_FZ = float(os.environ.get("FALL_WOBFZ", 0.60))            # props carry this share of the weight while circling
T_POP = float(os.environ.get("FALL_TPOP", 0.9))
POP_FOLD = 0.55                                              # s for the leg to swing out
POP_PAIR = os.environ.get("FALL_POPPAIR", "0") == "1"
HOP = 0.22                                                   # m: first rise on 7 props; the climb waits for the 8th
Q_POP = np.radians([-48.0, 0, 75, 0, 10, 0, 0, 0])            # one-leg kickstand: root down, the rest of the arm out to the side
TLIM = float(os.environ.get("FALL_TLIM", 1.0))       # impact reflex: servo torque limit (x stall) from free fall until settled
if os.environ.get("FALL_J0FRAC"): C.CLUTCH_FRAC[0] = float(os.environ["FALL_J0FRAC"])

SCEN = {
    "cut":   dict(z0=1.5, inv=False, t_fault=1.0, fault="cut", text="motor cut-out at 1.5 m, upright"),
    "hit":   dict(z0=1.5, inv=False, t_fault=1.0, fault="hit", text="side blow at 1.5 m + ESC failsafe"),
    "inv":   dict(z0=1.2, inv=True, t_fault=1.0, fault="cut", text="motor cut-out at 1.2 m, upside down"),
    "drift": dict(z0=1.0, inv=False, t_fault=1.6, fault="cut", text="motor cut-out flying sideways at 2.5 m/s, 1.0 m up", v=2.5),
}


def Rot(u, th):
    u = u / np.linalg.norm(u); K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def ang(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def slerpR(Ra, Rb, u):
    Rr = Ra.T @ Rb; th = np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1))
    if th < 1e-6: return Rb.copy()
    w = F4.vee(Rr - Rr.T) / (2 * np.sin(th))
    return Ra @ Rot(w, th * u)


_ss = np.linspace(0, 1, 2001); _th = np.pi * mj(_ss)
def s_of_theta(th):
    return float(np.interp(th, _th, _ss))


class Fall(F4.Flip):
    def __init__(self, name):
        sc = SCEN[name]; self.sc = sc; self.name = name
        F4.Z0 = sc["z0"]
        if SHELL_STOP:                                  # joint limits as link-to-link (shell) stops on the link angle a + c + b
            S4.EXTRA_XML = S4.EXTRA_XML + "</worldbody><tendon>" + "".join(
                f'<fixed name="lim{i}_{k}" limited="true" range="{C.LIM[k][0]:.4f} {C.LIM[k][1]:.4f}" solreflimit="{SHELL_SOFT0 if k == 0 else SHELL_SOFT} 1">'
                f'<joint joint="q{i}_{k}a" coef="1"/>' + (f'<joint joint="c{i}_{k}" coef="1"/>' if C.CLUTCH_FRAC[k] > 0 else "")
                + f'<joint joint="q{i}_{k}b" coef="1"/></fixed>' for i in range(8) for k in range(C.NJ)) + "</tendon><worldbody>"   # sim4 has no tendon hook
        super().__init__()
        if os.environ.get("FALL_CARM"):                 # clutch-joint armature (numerical in sim4; there is no geared inertia in a cam clutch)
            for (i, k), dd in self.cd.items(): self.m.dof_armature[dd] = float(os.environ["FALL_CARM"])
        if os.environ.get("FALL_EQSOFT"): self.m.eq_solref[:, 0] = float(os.environ["FALL_EQSOFT"])
        if SHELL_STOP:                                  # the servo pivot itself has no stop (the servo turns 360 deg)
            for i in range(8):
                for k in range(C.NJ):
                    for h in "ab": self.m.jnt_range[self.m.joint(f"q{i}_{k}{h}").id] = [-1.6, 1.6]
            self.ten_ids = np.array([[self.m.tendon(f"lim{i}_{k}").id for k in range(C.NJ)] for i in range(8)])
        m, d = self.m, self.d
        # floor collision for arm links, pods and the dome (bit 8); soft bumper/dome contact
        fl = m.geom("floor").id; m.geom_contype[fl] = 1 | 8
        # arm i's links: contype = its own bit, conaffinity = floor bit + the other arms' bits -> arms hit the floor and
        # each other, never themselves (neighbouring links overlap at the joints)
        allbits = sum(1 << (4 + i) for i in range(8))
        for g in range(m.ngeom):
            bn = m.body(m.geom_bodyid[g]).name; gn = m.geom(g).name or ""
            if (bn.startswith(("arm", "a", "b", "c", "pod")) and not bn.startswith("imp") and m.geom_size[g][0] > 0.0125
                    and not gn.startswith(("toe", "prop"))):
                i = int("".join(ch for ch in bn.split("_")[0] if ch.isdigit()))
                m.geom_contype[g] = 1 << (4 + i); m.geom_conaffinity[g] = 8 | (allbits & ~(1 << (4 + i)))
        # contact envelope of a link = what is drawn (servo 35 mm + clutch cam and rocker to 31.5 mm from the link axis),
        # not the slimmer mass capsule (17-22 mm): otherwise the cams sink up to 12 mm into the floor
        for g in range(m.ngeom):
            if (m.geom_conaffinity[g] & 8) and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE and m.geom_size[g][0] < LINK_R:
                m.geom_size[g][0] = LINK_R; hl = m.geom_size[g][1]
                m.geom_rbound[g] = hl + LINK_R; m.geom_aabb[g] = [0, 0, 0, LINK_R, LINK_R, hl + LINK_R]
        m.body_contype[:] = 0; m.body_conaffinity[:] = 0             # body-level filters are precompiled: rebuild them
        for g in range(m.ngeom):
            b = m.geom_bodyid[g]; m.body_contype[b] |= m.geom_contype[g]; m.body_conaffinity[b] |= m.geom_conaffinity[g]
        self.bump_ids = {m.geom(n).id for n in ["edf", "plate", "dome"] + [f"dome_r{j}" for j in range(8)]}
        for g in self.bump_ids: m.geom_solref[g] = SOFT
        for g in self.toe: m.geom_solref[g] = TOE_SOFT          # TPU fingertip caps
        for g in range(m.ngeom):
            if (m.geom_conaffinity[g] & 8) and g not in self.bump_ids: m.geom_solref[g] = LINK_SOFT   # printed link shells
        self.belly_ids = set(self.bump_ids)
        if sc["inv"]: self.set_inverted_hover(sc["z0"])
        if sc.get("v"):
            d.qvel[0] = sc["v"]
            p = d.subtree_com[self.torso][:2].copy()
            self.xyplan = B4.Quintic(0, 1e3, p, p + np.array([sc["v"] * 1e3, 0]), np.array([sc["v"], 0]))
        self.R_hold = d.xmat[self.torso].reshape(3, 3).copy(); self.R_ref = self.R_hold.copy()
        self.w_ref = np.zeros(3); self.alpha_ref = np.zeros(3)
        self.phase = "hover"; self.label = "HOVER: " + sc["text"]
        self.v_prev = d.qvel[:3].copy(); self.sf = C.G; self.inv_ground = False
        self.t_fault = sc["t_fault"]; self.t_ff = None; self.t_imp = None; self.v_imp = None; self.t_settle = None
        self.hit_until = -1.0; self.calm = 0.0; self.skill = None; self.k = 0
        self.Mx = dict(servo=0.0, gear=0.0, slip=0.0, stop=0, bump=0.0, link=0.0, decel=0.0)
        self.pin_t = np.full(8, -9.0); self.kick = []; self.pre_t = PRELOAD_T; self.wob = False; self.wob_stage = 0; self.fz_cap = None; self.q_strut = Q_STRUT
        self.qacc_prev = d.qvel.copy(); self.qnom_hover = np.tile(Q_STAND if sc["inv"] else F4.Q_FLY, (8, 1))

    # ---- clutch with reinforcement options (progressive cam, TPU end stop); defaults reproduce sim4.clutch
    def clutch(self, T):
        m, d = self.m, self.d
        if not hasattr(self, "_stop_set"):
            self._stop_set = True
            for (i, k) in self.cq:
                tr = C.CLUTCH_TRAVEL[k]
                m.jnt_range[self.cj_ids[i, k]] = [-3.1, 3.1] if RIDE else [-(tr + STOP_X), tr + STOP_X]
        for (i, k), qa in self.cq.items():
            tb = C.CLUTCH_FRAC[k] * C.STALL[k] * T / C.T_FIRM
            tr = C.CLUTCH_TRAVEL[k]; phi = d.qpos[qa]
            if RIDE:                                                # ride-over cam: a V every 2 (travel + 5 deg); past the crest the
                P = 2 * (tr + RIDE_GAP)                             # roller drops into the next V instead of hitting a stop
                c0 = P * np.round(phi / P); m.qpos_spring[qa] = c0; phi = phi - c0
            a = abs(phi)
            tbe = tb * (1 + PROG * min(a / tr, 1.0) ** 2)           # progressive cam: slope (and slip torque) rises with travel
            kc = tb / (0.3 * D)
            m.jnt_stiffness[self.cj_ids[i, k]] = kc
            if kc * a > tbe:
                d.qfrc_applied[self.cd[i, k]] += np.sign(phi) * (kc * a - tbe)
            if STOP_X > 0 and a > tr and not RIDE:                   # TPU end-stop pad: spring + damping, independent of the line
                v = d.qvel[self.cd[i, k]]; ks = STOP_K * C.STALL[k]
                f = ks * (a - tr) + STOP_C * C.STALL[k] * max(0.0, v * np.sign(phi))
                d.qfrc_applied[self.cd[i, k]] -= np.sign(phi) * f

    def set_inverted_hover(self, z):
        m, d = self.m, self.d
        R = F4.Rax(np.pi); q4 = np.zeros(4); mujoco.mju_mat2Quat(q4, R.ravel())
        d.qpos[:3] = [0, 0, z]; d.qpos[3:7] = q4
        for i in range(8):
            for k in range(8):
                d.qpos[self.ja[i, k]] = Q_STAND[k] / 2; d.qpos[self.jb[i, k]] = Q_STAND[k] / 2
        q = np.tile(Q_STAND, (8, 1)); self.q_cmd = q.copy(); self.goal = q.copy(); self.goal_prev = q.copy(); self.q_ik = q.copy()
        d.ctrl[:] = q.ravel(); mujoco.mj_forward(m, d)
        self.edf = self.edf_target = 0.0; self.fan[:] = C.W_ROBOT / 8
        self.zplan = B4.Quintic(0, 1e-3, z, z); p = d.subtree_com[self.torso][:2].copy(); self.xyplan = B4.Quintic(0, 1e-3, p, p)

    def ground_ik(self, arms):
        if self.inv_ground: return F4.Flip.ground_ik(self, arms)
        return B4.Body.ground_ik(self, arms)

    # ---------------------------------------------------------------- senses
    def floor_forces(self):
        m, d = self.m, self.d; out = np.zeros(6); bump = 0.0; link = 0.0; toe = 0.0
        fl = m.geom("floor").id
        for j in range(d.ncon):
            c = d.contact[j]
            if fl not in (c.geom1, c.geom2): continue
            g = c.geom2 if c.geom1 == fl else c.geom1
            mujoco.mj_contactForce(m, d, j, out)
            if g in self.bump_ids: bump += abs(out[0])
            elif g not in self.toe: link += abs(out[0])
            else: toe += abs(out[0])
        self.toe_f = toe
        return bump, link

    # ---------------------------------------------------------------- reflexes
    def brace(self):
        d = self.d
        p = d.xpos[self.torso].copy(); R = d.xmat[self.torso].reshape(3, 3)
        c = p + R @ np.array([0, 0, 0.03])
        for i in range(8):
            Rb0, r = C.arm_base(i, p, R)
            if r[2] > c[2] + 0.06: continue                      # root on top of the body: leave it
            h = (r - c)[:2]; n = np.linalg.norm(h)
            h = h / n if n > 1e-3 else (Rb0[:, 0][:2] / max(np.linalg.norm(Rb0[:, 0][:2]), 1e-6))
            tgt = p + np.array([BRACE_R * h[0], BRACE_R * h[1], -BRACE_DZ])
            q = self.goal[i].copy()
            for _ in range(3): q = B4.ik(i, tgt, B4.DOWN, q, F4.Q_STAND, p, R, iters=1, w_reg=0.05)
            self.goal[i] = q

    # ---------------------------------------------------------------- recovery: thrust-assisted righting
    def plan_righting(self, t):
        d = self.d; R = d.xmat[self.torso].reshape(3, 3)
        phi = float(np.arccos(np.clip(R[2, 2], -1, 1)))
        best = None
        for k in range(8):
            a = np.radians(22.5 + 45 * k); ub = np.array([np.cos(a), np.sin(a), 0])
            for psi in np.radians(np.arange(-180, 180, 1.0)):
                Ru = B4.Rz(psi); uw = Ru @ ub
                e = ang(Rot(uw, phi) @ Ru, R)
                if best is None or e < best[0]: best = (e, k, psi)
        e, k, psi = best
        if phi > np.radians(INV_TILT): phi = np.pi                 # mostly upside down: lift off fully inverted, all 8 claw-props
        self.k = k; self.R_up = B4.Rz(psi); a = np.radians(22.5 + 45 * k)
        self.u_w = self.R_up @ np.array([np.cos(a), np.sin(a), 0]); self.phi = phi
        self.s0 = s_of_theta(phi); self.R_fam0 = Rot(self.u_w, phi) @ self.R_up
        self.fly = [(i + k) % 8 for i in F4.FLYERS]; self.under = [(i + k) % 8 for i in F4.UNDER]
        self.under_up = self.s0 >= F4.S_REJOIN                      # arms 5-7 point up too at this tilt: they fly
        self.q_start = self.goal.copy(); self.t_r = t; self.z_lift0 = d.xpos[self.torso][2]
        self.R_lift = R.copy()
        print(f"  righting: tilt {np.degrees(phi):.0f} deg, axis arm {k}, fit error {e:.1f} deg, path s {self.s0:.2f}")
        self.kick = []; self.wk = 0.0; self.t_wk1 = None; self.wob = False; self.wob_stage = 0; self.fz_cap = None
        self.q_strut = Q_STRUT
        if KICK_N > 0 and phi == np.pi:
            if KICK_STYLE == "wobble":
                self.wob = True; self.q_strut = Q_POP; self.set_tlim(KICK_TLIM); self.Mx["wobble"] = 1
            else: self.plan_kick(t, R)

    def plan_kick(self, t, R):
        """Kickstand arms = the arm on the low side of the tilted body and the two 135 deg away from it."""
        v = -R[2, :2]                                        # body-frame direction of the lowest plate rim point
        a = self.k if np.linalg.norm(v) < 0.03 else int(np.round((np.degrees(np.arctan2(v[1], v[0])) - 22.5) / 45.0)) % 8
        self.kick = {1: [a], 2: [(a - 1) % 8, (a + 1) % 8], 3: [a, (a + 3) % 8, (a + 5) % 8]}[KICK_N]
        self.t_k = t; self.q_k0 = self.goal.copy(); self.kI = {i: 0.0 for i in self.kick}; self.level_t = 0.0
        self.set_tlim(KICK_TLIM)
        self.Mx["kick"] = [int(i) for i in self.kick]
        print(f"  kickstand: low side arm {a}, struts {self.kick}")

    def strut_pose(self, i, dt, level=True):
        """Strut joint angles; j0 follows the IMU: a strut on the low side pushes down further (PI on the rim drop)."""
        R = self.d.xmat[self.torso].reshape(3, 3); az = np.radians(22.5 + 45 * i)
        e = -(R[2, 0] * np.cos(az) + R[2, 1] * np.sin(az))   # > 0: this side of the plate is low (= sin tilt x cos rel. azimuth)
        if level: self.kI[i] = float(np.clip(self.kI[i] + e * dt, -0.05, 0.08))
        q = self.q_strut.copy()
        q[0] = np.clip(self.q_strut[0] - (KICK_KP * e + KICK_KI * self.kI[i]), C.LIM[0][0] + 0.01, np.radians(-30))
        return q

    def wobble(self, t, dt, el):
        """Stage 0: the attitude reference cones round the level inverted attitude, so the body rolls round in a circle
        on its dome. Stage 1: the arm on the low side pops out as a strut and props the body up level; thrust is released."""
        d = self.d; R = d.xmat[self.torso].reshape(3, 3)
        if self.wob_stage == 0:
            if not hasattr(self, "wob_phi0") or el < dt * 1.5:
                w = F4.vee((R @ self.R_fam0.T) - (R @ self.R_fam0.T).T)         # where the body leans now: start the circle there
                self.wob_phi0 = float(np.arctan2(w[1], w[0])) if np.linalg.norm(w[:2]) > 0.02 else 0.0
            th = WOB_TILT * mj(el / 0.6)
            turn = WOB_HZ * (el - 0.3 * (1 - np.exp(-el / 0.3)))                  # turns so far (eases in)
            ph = self.wob_phi0 + 2 * np.pi * turn
            Rn = Rot(np.array([np.cos(ph), np.sin(ph), 0.0]), th) @ self.R_fam0
            self.R_ref = Rn; self.w_ref = Z * (2 * np.pi * WOB_HZ) * np.sin(th) * 0  # slow precession: no feed-forward rate
            self.fz_cap = 0.45 + (WOB_FZ - 0.45) * mj(el / 0.8)
            self.label = f"WOBBLE: props on at {self.fz_cap * 100:.0f} % of the weight, the body circles on its dome ({turn:.1f} turns)"
            if el >= self.T_wob:
                v = -R[2, :2]
                a = self.k if np.linalg.norm(v) < 0.03 else int(np.round((np.degrees(np.arctan2(v[1], v[0])) - 22.5) / 45.0)) % 8
                self.kick = [a]; self.kI = {a: 0.0}; self.Mx["kick"] = [int(a)]; self.gate[a] = 0.0
                if POP_PAIR: self.gate[(a + 4) % 8] = 0.0       # stop the opposite prop too: the other six stay balanced
                self.wob_stage = 1; self.t_pop = t; self.q_pop0 = self.goal[a].copy(); self.R_pop0 = self.R_ref.copy()
                print(f"  wobble done at {t:.2f} s, leg {a} pops out (tilt {np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1))):.1f} deg off level)")
            return
        tp = t - self.t_pop; a = self.kick[0]
        self.R_ref = slerpR(self.R_pop0, self.R_fam0, mj(tp / 0.5))
        self.fz_cap = WOB_FZ + (1.0 - WOB_FZ) * mj(tp / T_POP)
        self.label = f"KICKSTAND: arm {a} pops out and props the body up"
        if tp >= T_POP:
            self.wob_stage = 2; self.fz_cap = None

    def path_k(self, s):
        qp = F4.path_at(s); return np.array([qp[(j - self.k) % 8] for j in range(8)])

    # ---------------------------------------------------------------- control tick
    def control(self, t, dt):
        d = self.d
        self.sense()
        self.bat.step(self.edf, self.fan, self.load, dt, k_claw=B4.PROP_P["K"])
        self.fmax_eff = PW.claw_max(B4.PROP, self.bat.V); self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())
        v = d.qvel[:3].copy(); a = (v - self.v_prev) / dt; self.v_prev = v
        self.sf = float(np.linalg.norm(a + np.array([0, 0, C.G])))       # IMU specific force
        bump, link = self.floor_forces(); self.bump_f, self.link_f = bump, link
        Rb = d.xmat[self.torso].reshape(3, 3); tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
        self.xy_accel(t)
        ph = self.phase
        if ph == "hover":
            self.R_ref = self.R_hold; self.w_ref[:] = 0; self.alpha_ref[:] = 0
            if not self.sc["inv"] and self.edf_target < 30: self.edf_target = 34.0
            self.gimbal(t, range(8), self.qnom_hover)
            if t >= self.t_fault:
                if self.sc["fault"] == "cut":
                    self.cut(t)
                else:
                    self.phase = "hit"; self.hit_until = t + 0.25; self.label = "SIDE BLOW"
        elif ph == "hit":
            self.gimbal(t, range(8), self.qnom_hover)
            if tilt > 45:
                self.cut(t); self.label = "ESC FAILSAFE: tilt past 45 deg, motors stop"
        elif ph == "fall":
            self.v_fall = max(getattr(self, "v_fall", 0.0), float(np.linalg.norm(v)))
            if self.t_ff is None and (self.sf < 3.0 or t - self.t_cut > 0.08):
                self.t_ff = t; B4.V_FREE, B4.A_FREE = V_EMERG, A_EMERG
                if TLIM < 1.0: self.set_tlim(TLIM)
            if self.t_ff is not None:
                self.brace(); self.label = "FREE FALL sensed: arms brace under the body"
            if bump + link + self.foot_f.sum() > 30:
                self.phase = "impact"; self.t_imp = t; self.v_imp = self.v_fall
                B4.V_FREE, B4.A_FREE = 180 * D, 1200 * D
                print(f"  impact at t {t:.2f} s, speed {self.v_imp:.2f} m/s, tilt {tilt:.0f} deg")
        elif ph == "impact":
            self.label = f"IMPACT {self.v_imp:.1f} m/s: clutches slip, bumpers squash"
            moving = np.linalg.norm(v) > 0.06 or np.linalg.norm(d.qvel[3:6]) > 0.4
            self.calm = 0.0 if moving else self.calm + dt
            if self.calm > 0.5 or t - self.t_imp > 3.0:
                self.phase = "assess"; self.t_settle = t
                self.set_tlim(1.0)
                self.check_index(t)
        elif ph == "reindex":
            e = t - self.t_ri; self.winch_target = C.T_LINE["soft"]
            for (i, k), dq in list(self.ri.items()):               # drive the servo to the link's own stop on the side the
                tgt = C.LIM[k][0] if dq < 0 else C.LIM[k][1]          # roller skipped to: the link arrives first and is held
                if abs(self.d.qpos[self.cq[i, k]]) < np.radians(15):  # Hall switch: roller back in the home V -> stop here
                    self.goal[i, k] = self.q_cmd[i, k]; del self.ri[i, k]; continue
                self.goal[i, k] = self.ri_q0[i, k] + (tgt - self.ri_q0[i, k]) * mj(e / 1.5)
            self.label = f"RE-INDEX: {self.n_ri} clutch(es) skipped a notch; servos walk them back into the home notch"
            if e > 1.9 or not self.ri:
                self.phase = "assess"; self.t_settle = t; self.check_index(t)
        elif ph == "assess":
            self.label = f"SETTLED, tilt {tilt:.0f} deg: clutches re-seat"
            Ts, Tf = C.T_LINE["soft"], C.T_LINE["firm"]
            self.winch_target = Ts + (Tf - Ts) * mj((t - self.t_settle) / 1.2)     # winch re-seats the clutches gently
            if t - self.t_settle > 1.4:
                if tilt < 50:
                    self.phase = "getup"; self.lying = True; self.inv_ground = False
                    self.skill = B4.GetUp(); self.skill.start(self, t)
                else:
                    self.phase = "rearm"; self.plan_righting(t)
                    if self.kick: self.phase = "plant"
        elif ph == "getup":
            self.skill.step(self, t, dt); self.label = "RECOVER: " + self.skill.label(self)
            if getattr(self, "n_wd", 0) >= 1: self.winch_target = C.T_LINE["tight"]   # a clutch skipped in the last get-up: hold tight this time
            if RIDE and self.notch_off(): 
                self.n_wd = getattr(self, "n_wd", 0) + 1
                self.phase = "impact"; self.t_imp = t; self.calm = 0.0; self.lying = True     # notch watchdog: settle, re-index, start again
                print(f"  notch watchdog at {t:.2f} s: a clutch skipped during the get-up")
                return self.finish_tick(dt)
            if self.skill.done:
                self.phase = "stand"; self.skill = B4.Idle(2.0); self.skill.start(self, t)
        elif ph == "stand":
            self.skill.step(self, t, dt); self.label = "RECOVERED: standing"
            if self.skill.done: self.phase = "done"
        elif ph in ("plant", "rearm", "lift", "roll", "upright"):
            self.righting(t, dt, tilt)
            if ph in ("plant", "rearm", "lift"):             # pinned-arm governor: an arm without thrust whose servos stall
                for i in range(8):                           # (trapped under the body) holds where it is instead of pushing
                    if self.fan[i] < 1.0 and self.load[i].max() > 0.85 and i not in self.kick:
                        self.pin_t[i] = t
                    if t - self.pin_t[i] < 0.4:                 # servo position read back: command = where it is
                        qm = 2 * self.d.qpos[self.ja[i]]
                        self.goal[i] = qm; self.q_cmd[i] = qm; self.qd_cmd[i] = 0
        if ph in ("getup", "stand"): self.reflexes(dt)
        self.finish_tick(dt)

    def finish_tick(self, dt):
        self.apply_tlim(dt)
        self.spine(dt)
        self.d.ctrl[:] = self.q_cmd.ravel()
        self.T_line += (self.winch_target - self.T_line) * min(1, dt / 0.15)

    def notch_off(self):
        """True when a Hall switch has reported a roller out of its home V for 0.3 s (a skipped notch)."""
        off = any(abs(self.d.qpos[qa]) > C.CLUTCH_TRAVEL[k] + RIDE_GAP for (i, k), qa in self.cq.items())
        self.t_off = (getattr(self, "t_off", 0.0) + B4.CTRL_DT) if off else 0.0
        return self.t_off > 0.3

    def check_index(self, t):
        """Ride-over cams: a Hall switch per clutch (magnet in the cam's home V) tells whether the roller sits in the home
        notch. If not, the servo drives toward the link's own shell stop on the side the roller skipped to: the link
        reaches its stop first and is held there, the servo carries on, the roller rides back over the crest and drops
        into the home V (servo and link then agree at the stop angle)."""
        if not RIDE: return
        self.ri = {}
        for (i, k), qa in self.cq.items():
            P = 2 * (C.CLUTCH_TRAVEL[k] + RIDE_GAP); n = np.round(self.d.qpos[qa] / P)
            if n != 0: self.ri[i, k] = float(n * P)
        if self.ri:
            self.Mx["reindex"] = self.Mx.get("reindex", 0) + len(self.ri)
            print(f"  re-index at {t:.2f} s: " + ", ".join(f"arm {i} j{k} {np.degrees(v):+.0f} deg" for (i, k), v in self.ri.items()))
            self.phase = "reindex"; self.t_ri = t; self.ri_q0 = self.goal.copy(); self.n_ri = len(self.ri)

    def set_tlim(self, f):
        """Feetech torque-limit register (RAM): target torque limit f x stall for every arm servo (ramped, 0.3 s)."""
        self.tlim_target = f

    def apply_tlim(self, dt):
        cur = getattr(self, "tlim", 1.0); tgt = getattr(self, "tlim_target", 1.0)
        full = tuple(self.kick) if self.phase in ("plant", "rearm", "lift") else ()      # strut roots keep full torque
        if cur == tgt and full == getattr(self, "tlim_full", ()): return
        cur = tgt if abs(tgt - cur) < 1e-3 else cur + np.clip(tgt - cur, -dt / 0.3, dt / 0.3); self.tlim = cur; self.tlim_full = full
        lim = np.tile(C.STALL, 8) * cur
        for i in full: lim[i * C.NJ] = C.STALL[0]
        self.m.actuator_forcerange[:len(lim)] = np.stack([-lim, lim], 1)

    def cut(self, t):
        self.phase = "fall"; self.t_cut = t; self.fans_on = False; self.edf_target = 0.0
        self.label = "MOTORS CUT"; self.winch_target = C.T_LINE[FALL_WINCH] if FALL_WINCH in C.T_LINE else float(FALL_WINCH)

    def righting(self, t, dt, tilt):
        d = self.d; e = t - self.t_r
        if self.phase == "plant":                      # kickstand: fold the distal arm up, swing the root link down, level
            ek = t - self.t_k
            for i in self.kick:
                q = self.q_k0[i] + (Q_STRUT - self.q_k0[i]) * mj(ek / T_FOLD)
                q[0] = self.q_k0[i][0] + (J0_PRE - self.q_k0[i][0]) * mj(ek / T_FOLD)
                if ek > T_FOLD:
                    q = self.strut_pose(i, dt, level=ek > T_PLANT)
                    if ek < T_PLANT: q[0] = J0_PRE + (q[0] - J0_PRE) * mj((ek - T_FOLD) / (T_PLANT - T_FOLD))
                self.goal[i] = q
            self.level_t = self.level_t + dt if tilt > 176.5 else 0.0
            self.label = (f"KICKSTAND: arms {', '.join(str(i) for i in self.kick)} fold into struts" if ek < T_PLANT else
                          f"KICKSTAND: struts level the body ({180 - tilt:.0f} deg to go)")
            if ek >= T_PLANT and (self.level_t > 0.3 or ek > T_PLANT + 2.5):
                self.phase = "rearm"; self.t_r = t; self.q_start = self.goal.copy()
            return
        if self.phase == "rearm":                      # arms to the flip-path pose for this tilt (props still off)
            self.label = ("KICKSTAND: the other arms rise to the flight pose, props still off" if self.kick else
                          "WOBBLE TAKE-OFF: arms rise to the flight pose, props still off" if self.wob else "RECOVER: arms take the flip pose for this tilt")
            qp = self.path_k(self.s0)
            w = mj(e / 1.2)
            for i in range(8):
                if i in self.under and not self.under_up: continue
                if i in self.kick:
                    self.goal[i] = self.strut_pose(i, dt); continue
                self.goal[i] = self.q_start[i] + (qp[i] - self.q_start[i]) * w
                if self.kick or self.wob:                  # root first: swing the whole arm up off the floor, then settle j0
                    up0 = np.radians(25.0); a0 = self.q_start[i][0]
                    self.goal[i][1:] = self.q_start[i][1:] + (qp[i][1:] - self.q_start[i][1:]) * mj((e - 0.3) / 0.8)
                    self.goal[i][0] = a0 + (up0 - a0) * mj(e / 0.5) if e < 0.9 else up0 + (qp[i][0] - up0) * mj((e - 0.9) / 0.6)
            if e >= (T_RISE + 0.1 if (self.kick or self.wob) else T_REARM):
                self.phase = "lift"; self.t_l = t; self.fans_on = True; self.prop_scale = 1.0; self.spin_tau = 0.08
                self.gate[:] = 1.0; self.t_in = [None] * 8; self.iatt[:] = 0
                z = d.xpos[self.torso][2]
                self.pre_t = KICK_PRELOAD if self.kick else PRELOAD_T
                if self.wob:
                    self.pre_t = 0.0; self.T_wob = WOB_TURNS / WOB_HZ + 0.6; self.fz_cap = 0.45
                self.zplan = (B4.Quintic(t + self.T_wob + T_POP, 1.3, z, z + HOP) if self.wob else B4.Quintic(t + self.pre_t, 2.6, z, Z_RIGHT))
                self.t_air = None
                p = d.subtree_com[self.torso][:2].copy(); self.xyplan = B4.Quintic(t, 1e-3, p, p)
                self.R_ref = d.xmat[self.torso].reshape(3, 3).copy(); self.R_lift = self.R_ref.copy()
            return
        if self.phase == "lift":                       # motors restart, lift off on the side / upside down, climb
            el = t - self.t_l
            self.R_ref = slerpR(self.R_lift, self.R_fam0, mj((el - 0.3) / 2.0)); self.w_ref[:] = 0; self.alpha_ref[:] = 0
            qp = self.path_k(self.s0)
            if self.wob and self.wob_stage < 2: self.wobble(t, dt, el)
            z = d.xpos[self.torso][2]; wu = float(np.clip((z - self.z_lift0 - 0.15) / 0.45, 0, 1))
            air = z > self.z_lift0 + (0.04 if self.wob else 0.08)   # winch firm on the ground (clutches give), tight once airborne
            self.winch_target = C.T_LINE["tight"] if (air or self.kick) else C.T_LINE["firm"]   # kickstand: fewer props, each pulls harder
            popped = self.wob and self.wob_stage >= 1      # 7 props pull harder (22+ N): the flying arms need full servo torque
            if LIFT_TLIM < 1.0: self.set_tlim(1.0 if (air or popped) else (KICK_TLIM if (self.kick or self.wob) else LIFT_TLIM))   # soft servos while the arms still touch the floor
            if RIDE and not air and self.notch_off():      # a roller skipped while still on the ground: abort, settle, re-index
                self.fans_on = False; self.phase = "impact"; self.t_imp = t; self.calm = 0.0; self.tries = getattr(self, "tries", 0) + 1
                self.spin_tau = 0.5; self.winch_target = C.T_LINE["soft"]
                self.label = "RECOVER: a clutch skipped, props wind down, re-index and try again"; print(f"  notch abort at {t:.2f} s")
                return
            self.gimbal(t, [i for i in self.fly if i not in self.kick], qp)
            if self.under_up:
                self.gimbal(t, [i for i in self.under if i not in self.kick], qp)
            else:
                for i in self.under:
                    self.goal[i] = self.q_start[i] + (qp[i] - self.q_start[i]) * mj(wu)
            if not self.wob: self.label = "RECOVER: motors restart, lift off and climb"
            if self.kick:                                  # kickstand legs stay planted until the body has left the floor,
                if self.wob:                               # one leg, 7 props: hop, fold the leg up over 0.8 s, then climb on 8
                    if air and self.t_air is None and self.wob_stage == 2: self.t_air = t
                    if self.t_air is not None: self.wk = max(self.wk, float(np.clip((t - self.t_air - 0.15) / 0.8, 0, 1)))
                else:
                    self.wk = max(self.wk, float(np.clip((z - self.z_lift0 - 0.06) / 0.20, 0, 1)))   # then fold up and join
                if self.wk < 1.0:
                    for i in self.kick:
                        qs = self.strut_pose(i, dt, level=self.wk == 0.0)
                        if self.wob: qs = self.q_pop0 + (qs - self.q_pop0) * mj((t - self.t_pop) / POP_FOLD)   # the pop
                        self.goal[i] = qs + (qp[i] - qs) * mj(self.wk)
                else:
                    if self.t_wk1 is None:
                        self.t_wk1 = t
                        if self.wob: self.zplan = B4.Quintic(t + 0.4, 2.3, z, Z_RIGHT)
                    self.gate[:] = 1.0
                    self.gimbal(t, self.kick, qp)
                nf = 8
                if self.wob and self.wob_stage == 2:
                    self.label = (f"TAKE-OFF: 7 props lift, arm {self.kick[0]} still propping" if not air else
                                  f"TAKE-OFF: airborne on 7 props, arm {self.kick[0]} hangs out" if self.wk == 0 else
                                  f"TAKE-OFF: airborne, arm {self.kick[0]} folds up and its prop joins" if self.t_wk1 is None or t - self.t_wk1 < 0.5 else
                                  "TAKE-OFF: all 8 props, climbing")
                elif not self.wob: self.label = (f"KICKSTAND TAKE-OFF: all {nf} props spin up (strut tips point up too), {len(self.kick)} struts hold the body level" if not air else
                              "KICKSTAND TAKE-OFF: airborne, the struts unfold into the flight pose" if self.t_wk1 is None or t - self.t_wk1 < 0.5 else
                              "KICKSTAND TAKE-OFF: all 8 props, climbing")
            if el > 1.2 and z < self.z_lift0 + 0.08 and self.att_err > 50:      # it rolled away on the ground: stop, settle, re-assess
                self.fans_on = False; self.phase = "impact"; self.t_imp = t; self.calm = 0.0; self.tries = getattr(self, "tries", 0) + 1
                self.spin_tau = 0.5; self.winch_target = C.T_LINE["soft"]
                self.label = "RECOVER: rolled over on the ground, props wind down, try again"; print(f"  retry at {t:.2f} s")
                return
            ok = z > Z_RIGHT - 0.12 and self.att_err < 12 and el > 2.5 and (not (self.kick or self.wob) or (self.t_wk1 is not None and t - self.t_wk1 > 0.6))
            if ok or el > 8.0:
                self.phase = "roll"; self.t_ro = t; self.T_ro = max(2.5, 4.0 * self.phi / np.pi); self.set_tlim(1.0)
            return
        if self.phase == "roll":                       # flip path in reverse: back upright
            u = float(np.clip((t - self.t_ro) / self.T_ro, 0, 1))
            th = self.phi * (1 - mj(u))
            thd = -self.phi * 30 * u * u * (1 - u) ** 2 / self.T_ro
            thdd = -self.phi * (60 * u - 180 * u * u + 120 * u ** 3) / self.T_ro ** 2 if 0 < u < 1 else 0.0
            self.R_ref = Rot(self.u_w, th) @ self.R_up; self.w_ref = self.u_w * thd; self.alpha_ref = self.u_w * thdd
            s = s_of_theta(th); qp = self.path_k(s)
            self.winch_target = C.T_LINE["tight"]
            for i in self.under: self.goal[i] = qp[i]
            self.gimbal(t, self.fly, qp)
            if s >= F4.S_REJOIN: self.gimbal(t, self.under, qp, wax=30.0 * min(1.0, (s - F4.S_REJOIN) / 0.15))
            elif s < 0.45: self.gimbal(t, self.under, qp, wax=30.0 * (1 - s / 0.45))
            self.label = f"RECOVER: roll back upright ({np.degrees(th):3.0f} deg to go)"
            if u >= 1.0:
                self.phase = "upright"; self.t_up = t; self.winch_target = C.T_LINE["firm"]
            return
        if self.phase == "upright":
            self.R_ref = self.R_up; self.w_ref[:] = 0; self.alpha_ref[:] = 0
            self.edf_target = 34.0 * mj((t - self.t_up) / 1.0)
            self.gimbal(t, range(8), np.tile(F4.Q_FLY, (8, 1)))
            self.label = "RECOVERED: hovering upright"
            if t - self.t_up > 2.5: self.phase = "done"

    # ---------------------------------------------------------------- thrust (every physics step)
    def fall_thrust(self, t):
        m, d = self.m, self.d
        Rb = d.xmat[self.torso].reshape(3, 3); pb = d.xpos[self.torso]; com = d.subtree_com[self.torso].copy()
        wW = Rb @ d.qvel[3:6]
        self.edf += (min(self.edf_target, self.edf_cap) - self.edf) * min(1, B4.DT / (0.08 if self.edf_target > self.edf else 0.25)); edf = self.edf
        d.xfrc_applied[:] = 0
        ez = Rb[:, 2]
        if edf > 1e-3:
            d.xfrc_applied[self.torso, :3] += edf * ez
            d.xfrc_applied[self.torso, 3:] += np.cross(pb - d.xipos[self.torso], edf * ez)
        if t < self.hit_until:                         # side blow: HIT_M about world x + 20 N sideways for 0.25 s
            ch, sh = np.cos(HIT_DIR), np.sin(HIT_DIR)
            d.xfrc_applied[self.torso, 3:] += HIT_M * np.array([ch, sh, 0]); d.xfrc_applied[self.torso, :3] += 20.0 * np.array([sh, -ch, 0])
        if not self.fans_on:
            self.fan *= np.exp(-B4.DT / getattr(self, "spin_tau", 0.08))   # rotors run down (cut: ~0.2 s; retry: braked gently)
            for i in range(8):
                if self.fan[i] > 1e-2:
                    R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                    d.xfrc_applied[self.pod[i], :3] = f
            self.sat = 0
            return
        Re = self.R_ref.T @ Rb; eR = 0.5 * F4.vee(Re - Re.T); eW = Rb @ eR
        self.att_err = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))))
        cz = np.array([d.xmat[self.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
        w_ax = np.clip((cz - np.cos(40 * D)) / (np.cos(25 * D) - np.cos(40 * D)), 0, 1)
        self.gate_s += (self.gate - self.gate_s) * min(1, B4.DT / 0.12)
        wgt = w_ax * self.gate_s
        for i in range(8):
            if wgt[i] > 1e-3:
                if self.t_in[i] is None: self.t_in[i] = t
                wgt[i] *= mj((t - self.t_in[i]) / 0.3)
            else:
                self.t_in[i] = None
        on = [i for i in range(8) if wgt[i] > 1e-3]
        cmd = np.zeros(8)
        if on:
            if self.sat == 0: self.iatt = np.clip(self.iatt + 6.0 * eW * B4.DT, -2.5, 2.5)
            Mw = -40.0 * eW - 7.0 * (wW - self.w_ref) - self.iatt + I_ROLL * self.alpha_ref
            self.Mz_dem = float(Mw[2])
            zr, vzr, azr = self.zplan.at(t)
            Fz = C.M_ROBOT * (C.G + azr + 6.0 * (zr - pb[2]) + 3.5 * (vzr - d.qvel[2])) - edf * ez[2]
            if self.phase == "lift" and getattr(self, "fz_cap", None) is not None:
                Fz = min(Fz, C.W_ROBOT * self.fz_cap)                       # wobble / pop: props carry a set share of the weight
            if self.phase == "lift" and t - self.t_l < self.pre_t:         # pre-load on the ground: props carry < weight while
                Fz = min(Fz, C.W_ROBOT * (0.45 + 0.4 * (t - self.t_l) / self.pre_t))   # the arms and attitude settle
            M_edf = np.cross(pb - com, edf * ez)
            A = np.zeros((4, len(on))); fmax = self.fmax_eff * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax) - B4.PROP_P["q_per_T"] * F4.SPIN[i] * ax
                A[:, j] = [ax[2], mo[0], mo[1], mo[2]]
            b = np.array([Fz, Mw[0] - M_edf[0], Mw[1] - M_edf[1], Mw[2]])
            Wr = np.array([1.0, 3.0, 3.0, 0.15])
            AA = np.vstack([Wr[:, None] * A, 0.02 * np.eye(len(on))]); bb = np.concatenate([Wr * b, np.zeros(len(on))])
            F = lsq_linear(AA, bb, bounds=(np.zeros(len(on)), np.maximum(fmax, 1e-6)), method="bvls").x
            for j, i in enumerate(on): cmd[i] = F[j]
        self.fan += (cmd - self.fan) * min(1.0, B4.DT / 0.04)
        self.sat = int(np.sum((cmd >= self.fmax_eff - 1e-6) & (wgt > 0.99)))
        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
                d.xfrc_applied[self.pod[i], 3:] -= B4.PROP_P["q_per_T"] * F4.SPIN[i] * self.fan[i] * R[:, 0]
        v = d.qvel[0:3]; drag = -0.5 * B4.RHO * B4.CDA * np.linalg.norm(v) * v
        d.xfrc_applied[self.torso, :3] += drag

    def run(self, t_end=30.0):
        m, d = self.m, self.d
        frames = []; nxt = 0.0; t_ctrl = -1.0; Mx = self.Mx; qv_prev = d.qvel.copy()
        while d.time < t_end:
            t = d.time
            if t - t_ctrl >= B4.CTRL_DT - 1e-9:
                self.control(t, B4.CTRL_DT); t_ctrl = t
                if self.phase == "done": break
            d.qfrc_applied[:] = 0
            self.clutch(self.T_line)
            self.fall_thrust(t)
            d.qfrc_applied[self.imp_d + 2] += 0.2 * C.G
            mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all():
                print("NaN at", t); break
            qacc = (d.qvel - qv_prev) / B4.DT; qv_prev = d.qvel.copy()
            frc = np.abs(d.actuator_force.reshape(8, C.NJ))
            # gear load: servo torque + rotor inertia x joint acceleration; the acceleration is averaged over 2 ms
            # (the idealised detent is far stiffer than a printed cam/rocker and chatters at sub-ms periods)
            self.qa_hist = (getattr(self, "qa_hist", []) + [qacc[self.da]])[-5:]
            qa_f = np.mean(self.qa_hist, axis=0)
            gear = (frc + C.ARMATURE * np.abs(qa_f) * 2) / C.GEAR_LIMIT
            Mx["gear_raw"] = max(Mx.get("gear_raw", 0.0), float(((frc + C.ARMATURE * np.abs(qacc[self.da]) * 2) / C.GEAR_LIMIT).max()))
            if gear.max() > Mx["gear"]:                  # where the worst gear load happens (diagnostics)
                i, k = np.unravel_index(int(gear.argmax()), gear.shape)
                cs = float(np.degrees(d.qpos[self.cq[i, k]])) if (i, k) in self.cq else None
                Mx["gear_at"] = dict(t=float(t), phase=self.phase, arm=int(i), joint=int(k), servo=float(frc[i, k] / C.GEAR_LIMIT[k]),
                                     inertia=float(C.ARMATURE[k] * abs(qa_f[i, k]) * 2 / C.GEAR_LIMIT[k]), clutch_deg=cs)
            Mx["gear"] = max(Mx["gear"], float(gear.max())); self.gear_acc = np.maximum(getattr(self, "gear_acc", gear * 0), gear)
            if SHELL_STOP:
                lt = mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON
                for r in np.where(d.efc_type[:d.nefc] == lt)[0]:
                    tq = abs(float(d.efc_force[r])); jj = int(d.efc_id[r]) % C.NJ
                    sj = Mx.setdefault("shell_j", [0.0] * C.NJ); sj[jj] = max(sj[jj], tq)
                    if tq > Mx.get("shell_stop", 0.0):
                        Mx["shell_stop"] = tq; tid = int(d.efc_id[r]); Mx["shell_at"] = dict(t=float(t), arm=tid // C.NJ, joint=tid % C.NJ, phase=self.phase)
            fs = (sum(self.floor_forces()) + self.toe_f) if self.phase in ("fall", "impact") else 0.0
            Mx["floor_step"] = max(Mx.get("floor_step", 0.0), fs)
            self.fbuf = (getattr(self, "fbuf", []) + [fs])[-25:]
            Mx["floor_10ms"] = max(Mx.get("floor_10ms", 0.0), float(np.mean(self.fbuf)))
            self.vbuf = (getattr(self, "vbuf", []) + [d.qvel[:3].copy()])[-26:]
            if self.phase in ("fall", "impact") and len(self.vbuf) == 26:     # body deceleration averaged over 10 ms
                Mx["decel"] = max(Mx["decel"], float(np.linalg.norm((self.vbuf[-1] - self.vbuf[0]) / 0.010 + np.array([0, 0, C.G]))) / C.G)
            if t >= nxt - 1e-9:
                self.sense(); bump, link = self.floor_forces()
                slip = np.zeros((8, C.NJ))
                for (i, k), qa in self.cq.items(): slip[i, k] = abs(np.degrees(d.qpos[qa]))
                stops = int(sum(slip[i, k] > np.degrees(C.CLUTCH_TRAVEL[k]) - 1.0 for (i, k) in self.cq))
                Mx["servo"] = max(Mx["servo"], float(self.load.max())); Mx["slip"] = max(Mx["slip"], float(slip.max()))
                Mx["stop"] = max(Mx["stop"], stops); Mx["bump"] = max(Mx["bump"], bump); Mx["link"] = max(Mx["link"], link)
                P = Mx.setdefault("phase", {}).setdefault(self.phase, dict(servo=0.0, gear=0.0, slip=0.0, stops=0))
                P["servo"] = max(P["servo"], float(self.load.max())); P["gear"] = max(P["gear"], float(self.gear_acc.max()))
                P["slip"] = max(P["slip"], float(slip.max())); P["stops"] = max(P["stops"], stops)
                Rb = d.xmat[self.torso].reshape(3, 3)
                frames.append(dict(t=t, qpos=d.qpos.copy(), fan=self.fan.copy(), edf=self.edf, servo_arm=self.load.max(1),
                                   slip_arm=slip.max(1), up=float(np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))), att=self.att_err,
                                   foot=self.foot_f.copy(), V=self.bat.V, I=self.bat.I, fmax=self.fmax_eff,
                                   com=d.subtree_com[self.torso].copy(), label=self.label, phase=self.phase, bump=bump, link=link,
                                   sf=self.sf, load=self.load.copy(), gear=float(self.gear_acc.max()), gearj=self.gear_acc.copy(), slipj=slip.copy(),
                                   speed=float(np.linalg.norm(d.qvel[:3]))))
                self.gear_acc = self.gear_acc * 0
                nxt += 1 / 30
        return frames


Q_STAND = F4.Q_STAND

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "cut"
    t0 = time.time(); F = Fall(name); fr = F.run(float(os.environ.get("FALL_T", 40)))
    Mx = F.Mx
    print(f"{name}: sim {F.d.time:.1f} s in {time.time() - t0:.0f} s wall | end phase {F.phase}")
    print(f"  peak floor force {Mx.get('floor_step',0):.0f} N, 10 ms average {Mx.get('floor_10ms',0):.0f} N = {Mx.get('floor_10ms',0)/C.W_ROBOT:.1f} x weight")
    print(f"  impact {F.v_imp if F.v_imp else 0:.2f} m/s | body decel (10 ms) {Mx['decel']:.0f} g | bumper/dome {Mx['bump']:.0f} N | arm-floor {Mx['link']:.0f} N | "
          f"servo {Mx['servo']:.2f} | gear {Mx['gear']:.2f} of limit | clutch slip {Mx['slip']:.0f} deg | clutches {'past a crest' if RIDE else 'at stop'} {Mx['stop']}")
    print("  worst gear at", Mx.get("gear_at"))
    if "shell_stop" in Mx: print(f"  worst link-shell stop torque {Mx['shell_stop']:.1f} N m at", Mx.get("shell_at"), "| per joint", " ".join(f"{v:.0f}" for v in Mx["shell_j"]))
    for ph, P in Mx.get("phase", {}).items():
        print(f"    {ph:8s} servo {P['servo']:.2f} | gear {P['gear']:.2f} | slip {P['slip']:4.0f} deg | clutches at stop {P['stops']}")
    keys = [k for k in fr[0] if k not in ("label", "phase")]
    tag = os.environ.get("FALL_TAG", "")
    np.savez_compressed(f"fall4_{name}{tag}.npz", **{k: np.array([f[k] for f in fr]) for k in keys}, label=np.array([f["label"] for f in fr]),
                        phase=np.array([f["phase"] for f in fr]))
    json.dump(dict(name=name, hw=HW, text=SCEN[name]["text"], v_imp=F.v_imp, t_imp=F.t_imp, end=F.phase, **Mx), open(f"fall4_{name}{tag}.json", "w"), indent=1)
