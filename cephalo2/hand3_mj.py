"""hand3 fingers as MuJoCo bodies with contact (the genuine grasp): 4 fingers x (lag + 3 knuckles) on a pod.
Geometry, masses and limits come from hand3/hand3_design.py and the CAD part masses (hand3_cad_parts.json).

Model of the finger drive (as designed, rev D): Dyneema cords pulled by bus servos on spools, each through a TPU link.
  K1 (base): its own cord over a 6 mm lever: a ONE-WAY position drive with series compliance,
      torque = clip(kp * (wound angle - joint angle), 0, stall),  kp = TPU stiffness x lever^2.
  K2 + K3 (middle + tip): ONE cord over both knuckles (levers 4.5 and 3.5 mm): a tendon whose length is
      4.5 mm x K2 + 3.5 mm x K3; the servo winds it, tension = clip(k_TPU * (wound - length), 0, stall).  The tip follows the
      middle joint and wraps round whatever stops the middle phalanx.
  Opening is done by the elastic extensor only.
Collision: per phalanx a thin box (blade + pad, pad side towards the heel) and a barrel cylinder at its knuckle; the TPU heel
and the hub plate.  Friction 0.8 (TPU pad on a wooden / coated handle, est.), torsional contact (condim 4).
The fingers collide only with tools (contype 4 against conaffinity 4), not with the robot or each other.

python3 hand3_mj.py  -> grasp capacity table of one hand on a bar (hand3_grasp_capacity.json)."""
import os, sys, json
import numpy as np
import mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.environ.get("H3_DIR", "hand3e")))
import hand3_design as H

MM = 1e-3
D = json.load(open(os.path.join(HERE, os.environ.get("H3_DIR", "hand3e"), "hand3_design.json")))
PM = json.load(open(os.path.join(HERE, os.environ.get("H3_DIR", "hand3e"), "hand3_cad_parts.json")))
ROLES = ["base", "middle", "tip"]
SZ = D["sizing"]
LEVER = [SZ[r]["lever"] * MM for r in ROLES]
TAU_STALL = [SZ[r]["F_stall"] * l for r, l in zip(ROLES, LEVER)]              # N m at the knuckle, servo at stall
TAU_RATED = [SZ[r]["F_rated"] * l for r, l in zip(ROLES, LEVER)]              # sustained
KP = [SZ[r]["tpu_k"] * 1e3 * l * l for r, l in zip(ROLES, LEVER)]             # N m / rad: TPU link in series
EXT_K3 = 0.02           # N m: pretension of the elastic extensor at K3, so the middle joint closes first and the tip wraps last (needed with one cord)
COUPLED = bool(D["blade"].get("coupled_tip", False))
F_STALL_C, F_RATED_C, K_CORD = SZ["middle"]["F_stall"], SZ["middle"]["F_rated"], SZ["middle"]["tpu_k"] * 1e3      # N, N, N/m: the K2 + K3 cord
L_MAX = LEVER[1] * np.radians(H.FOLD_MAX[1]) + LEVER[2] * np.radians(H.FOLD_MAX[2])
# Hub C (2026-11-14, hand3_hubc.py): two rings slide on the rotor.  BASE ring: four cords, each past its finger's lag pin (offset
# E_LAG: the first pull swings the finger into its pair) and over K1.  CURL ring: four cords over K2 + K3.  Each ring is pulled by
# one stationary STS3215; each cord has its own TPU link, so a finger sees ring force / 4 and its own series stiffness.
try: HC_ = json.load(open(os.path.join(HERE, os.environ.get("H3_DIR", "hand3e"), "hand3_hubc.json")))
except Exception: HC_ = None
HUB_C = HC_ is not None and os.environ.get("H3_HUB", "C") == "C"
if HUB_C:
    E_LAG = HC_["e_lag"] * MM; LEVER = [l * MM for l in HC_["levers"]]; L_MAX = LEVER[1] * np.radians(H.FOLD_MAX[1]) + LEVER[2] * np.radians(H.FOLD_MAX[2])
    FB_STALL, FB_RATED, KB_CORD = HC_["base"]["F_stall_finger"], HC_["base"]["F_rated_finger"], HC_["base"]["tpu_k"] * 1e3
    F_STALL_C, F_RATED_C, K_CORD = HC_["curl"]["F_stall_finger"], HC_["curl"]["F_rated_finger"], HC_["curl"]["tpu_k"] * 1e3
    LB_MAX = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(H.FOLD_MAX[0]); LAG_SPRING = HC_["lag_spring_Nm"]; EXT_K1 = HC_["ext_k1_Nm"]; EXT_K2 = float(os.environ.get("H3_K2", HC_.get("ext_k2_Nm", 0.004))); EXT_K3 = float(os.environ.get("H3_K3", HC_.get("ext_k3_Nm", 0.02)))
TAU_SPREAD = SZ["spread"]["torque_per_blade_Nm"]
STALL_A = [H.SERVOS[SZ[r]["servo"] or "STS3032"]["stall_A"] for r in ROLES]
M_GRIP = (PM["root_grip"]["mass_g"]) * 1e-3
M_PH = [(PM[f"phalanx{j + 1}"]["mass_g"] + PM[f"pad{j + 1}"]["mass_g"]) * 1e-3 for j in range(3)]
M_FINGERS = 4 * (M_GRIP + sum(M_PH))                                          # moving mass modelled as bodies (kg per hand)
ZB, ZHEEL = D["Z"]["blade"] * MM, D["Z"]["heel"] * MM
FOLD_MAX = H.FOLD_MAX; MU = 0.8
PRELOAD = float(os.environ.get("H3_PRELOAD", "0.7"))                          # hold preload as a fraction of the rated torque
# Heel shape (H3_HEEL): "flat" = Ø16 TPU post with a flat top at Z heel (rev D / hub C); "v:<alpha>:<lip top mm>:<half width x mm>:<half
# length y mm>" = the hand3 final V saddle across the heel along the grip axis (hand y).  Default: the final hand's V30 saddle.
HEEL = os.environ.get("H3_HEEL", "v:30")
if HEEL.startswith("v"):
    _hv = [float(x) for x in (HEEL.split(":")[1:] + [""] * 4)[:4] if x != ""]
    _hv += [30.0, D["Z"]["heel"], 8.0, 8.5][len(_hv):]
    V_ALPHA, V_TOP, V_W, V_LY = np.radians(_hv[0]), _hv[1] * MM, _hv[2] * MM, _hv[3] * MM
    V_APEX = V_TOP - V_W * np.tan(V_ALPHA)


def seat(r_bar):
    """Height (hand z, m) of the centre of a bar of radius r_bar lying in the heel along the grip axis."""
    if not HEEL.startswith("v"): return ZHEEL + r_bar
    if r_bar * np.sin(V_ALPHA) <= V_W: return V_APEX + r_bar / np.cos(V_ALPHA)
    return V_TOP + np.sqrt(max(r_bar ** 2 - V_W ** 2, 0.0))


def heel_xml(i):
    if not HEEL.startswith("v"):
        return f'<geom name="h3_{i}_heel" type="cylinder" fromto="0 0 {D["Z"]["hub_hi"] * MM:.4f} 0 0 {ZHEEL:.4f}" size="0.008" mass="0" {CT}/>'
    th = 0.0005; a = V_ALPHA; hl = V_W / (2 * np.cos(a))
    x = [f'<geom name="h3_{i}_heel" type="cylinder" fromto="0 0 {D["Z"]["hub_hi"] * MM:.4f} 0 0 {V_APEX - 0.001:.4f}" size="{V_W:.4f}" mass="0" {CT}/>']
    for sg in (1, -1):
        n = np.array([-sg * np.sin(a), 0, np.cos(a)]); c = np.array([sg * V_W / 2, 0, V_APEX + V_W / 2 * np.tan(a)]) - th * n
        x.append(f'<geom name="h3_{i}_heel{"p" if sg > 0 else "m"}" type="box" pos="{c[0]:.5f} 0 {c[2]:.5f}" axisangle="0 1 0 {-sg * np.degrees(a):.3f}" '
                 f'size="{hl:.5f} {V_LY:.5f} {th:.5f}" mass="0" {CT}/>')
    return "".join(x)


GRIP_AXIS = np.array([0.0, 0.0, 1.0])                                         # in the pod frame (pod x = rotor axis): hand y
CT = 'contype="4" conaffinity="0" condim="4" friction="%.2f 0.004 0.0001"' % MU


def _quat(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel()); return " ".join(f"{v:.6f}" for v in q)


def _rz(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rx(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def hand_xml(i, park=0.0):
    """Body subtree for the pod of arm i (pod x = rotor axis).  Joints h3_<i>_l<k> (spread, + = spread) and h3_<i>_f<k>_<j>."""
    Rph = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float) @ _rz(park)          # hand (z = axis) -> pod
    x = [f'<body name="h3_{i}" quat="{_quat(Rph)}">',
         heel_xml(i),
         f'<geom name="h3_{i}_hub" type="cylinder" fromto="0 0 {D["Z"]["hub_lo"] * MM:.4f} 0 0 {D["Z"]["hub_hi"] * MM:.4f}" size="0.0255" mass="0" {CT}/>']
    for k in range(4):
        sg = H.LAG_SIGN[k]
        x.append(f'<body name="h3_{i}_g{k}" pos="0 0 {ZB:.4f}" quat="{_quat(_rz(H.FINGER_AZ[k]))}">'
                 f'<joint name="h3_{i}_l{k}" axis="0 0 {sg}" pos="{H.R_LAG * MM:.4f} 0 0" range="{-1 if not HUB_C else 0} {H.SPREAD_MAX + (1 if not HUB_C else 0):.1f}" damping="0.004" armature="2e-5"'
                 + (f' stiffness="0.02" springref="{-np.degrees(LAG_SPRING / 0.02):.1f}"' if HUB_C else "") + "/>"
                 f'<geom type="box" pos="{(H.R_LAG + H.KN[0]) / 2 * MM:.4f} 0 0" size="{(H.KN[0] - H.R_LAG) / 2 * MM + 0.004:.4f} 0.008 0.003" mass="{M_GRIP:.5f}" contype="0" conaffinity="0"/>')
        for j in range(3):
            x0 = H.KN[j]; x1 = H.KN[j + 1] if j < 2 else H.R_TIP
            ch = H.chord((x0 + x1) / 2) * MM / 2; th = (H.thick((x0 + x1) / 2) + H.PAD_T) * MM / 2
            x.append(f'<body name="h3_{i}_p{k}_{j}"' + (f' quat="{_quat(_rx(H.PITCH))}"' if j == 0 else "") + ">"
                     f'<joint name="h3_{i}_f{k}_{j}" axis="0 -1 0" pos="{x0 * MM:.4f} 0 0" range="-2 {FOLD_MAX[j]:.0f}" damping="0.002" armature="2e-5" '
                     + ((f'stiffness="0.01" springref="{-np.degrees((EXT_K1 if j == 0 else EXT_K2) / 0.01):.1f}"/>' if HUB_C else 'stiffness="0.004"/>') if j < 2 else f'stiffness="0.01" springref="{-np.degrees(EXT_K3 / 0.01):.1f}"/>') +
                     f''
                     f'<geom name="h3_{i}_b{k}_{j}" type="box" pos="{(x0 + x1) / 2 * MM:.4f} 0 0" size="{(x1 - x0) / 2 * MM - 0.002:.4f} {ch:.4f} {th:.4f}" '
                     f'mass="{M_PH[j]:.5f}" {CT}/>'
                     f'<geom type="cylinder" fromto="{x0 * MM:.4f} {-H.W_KN[j] / 2 * MM:.4f} 0 {x0 * MM:.4f} {H.W_KN[j] / 2 * MM:.4f} 0" size="{H.BARREL[j] / 2 * MM:.4f}" '
                     f'mass="0" {CT}/>')
        x.append("</body>" * 4)
    x.append("</body>")
    return "".join(x)


def act_xml(arms):
    a = []
    for i in arms:
        for k in range(4):
            if HUB_C:
                a.append(f'<position name="h3_{i}_b{k}" tendon="h3_{i}_tb{k}" kp="{KB_CORD:.1f}" forcerange="0 {FB_STALL:.1f}" forcelimited="true" ctrlrange="-0.01 0.05"/>')
            else:
                a.append(f'<position name="h3_{i}_l{k}" joint="h3_{i}_l{k}" kp="2.0" kv="0.01" forcerange="{-TAU_SPREAD} {TAU_SPREAD}" forcelimited="true" ctrlrange="-0.1 1.0"/>')
                a.append(f'<position name="h3_{i}_f{k}_0" joint="h3_{i}_f{k}_0" kp="{KP[0]:.4f}" forcerange="0 {TAU_STALL[0]:.4f}" forcelimited="true" ctrlrange="-0.1 4.0"/>')
            a.append(f'<position name="h3_{i}_c{k}" tendon="h3_{i}_t{k}" kp="{K_CORD:.1f}" forcerange="0 {F_STALL_C:.1f}" forcelimited="true" ctrlrange="-0.01 0.05"/>')
    return "".join(a)


def tendon_xml(arms):
    return "<tendon>" + "".join(f'<fixed name="h3_{i}_t{k}"><joint joint="h3_{i}_f{k}_1" coef="{LEVER[1]}"/><joint joint="h3_{i}_f{k}_2" coef="{LEVER[2]}"/></fixed>'
                                for i in arms for k in range(4)) + ("".join(
        f'<fixed name="h3_{i}_tb{k}"><joint joint="h3_{i}_l{k}" coef="{E_LAG}"/><joint joint="h3_{i}_f{k}_0" coef="{LEVER[0]}"/></fixed>' for i in arms for k in range(4)) if HUB_C else "") + "</tendon>"


def install(arms):
    """Patch MjModel.from_xml_string so the full-robot model gets the finger actuators (the bodies come from arm_xml)."""
    orig = mujoco.MjModel.from_xml_string
    if getattr(mujoco.MjModel, "_h3", False): return
    def from_xml_string(xml, *a, **k):
        if "cephalo1_rev4" in xml and "h3_" in xml:
            xml = xml.replace("<actuator>", tendon_xml(arms) + "<actuator>").replace("</actuator>", act_xml(arms) + "</actuator>")
        return orig(xml, *a, **k)
    mujoco.MjModel.from_xml_string = staticmethod(from_xml_string); mujoco.MjModel._h3 = True


class Fingers:
    """Indices and the grasp controller for the hands of a model: open -> close under a force cap -> hold (wound position)."""
    def __init__(self, m, d, arms):
        self.m, self.d, self.arms = m, d, list(arms)
        self.qa = {i: np.array([[m.joint(f"h3_{i}_f{k}_{j}").qposadr[0] for j in range(3)] for k in range(4)]) for i in arms}
        self.la = {i: np.array([m.joint(f"h3_{i}_l{k}").qposadr[0] for k in range(4)]) for i in arms}
        self.ub = {i: np.array([m.actuator(f"h3_{i}_b{k}" if HUB_C else f"h3_{i}_f{k}_0").id for k in range(4)]) for i in arms}   # base cords (hub C) / base servos
        self.uc = {i: np.array([m.actuator(f"h3_{i}_c{k}").id for k in range(4)]) for i in arms}         # middle + tip cord servos
        self.tn = {i: np.array([m.tendon(f"h3_{i}_t{k}").id for k in range(4)]) for i in arms}
        self.ul = {i: (np.array([m.actuator(f"h3_{i}_l{k}").id for k in range(4)]) if not HUB_C else None) for i in arms}
        self.tb = {i: (np.array([m.tendon(f"h3_{i}_tb{k}").id for k in range(4)]) if HUB_C else None) for i in arms}
        self.held = {i: None for i in arms}; self.n = 0
        self.sum2 = {i: np.zeros((4, 2)) for i in arms}; self.peak = {i: np.zeros((4, 2)) for i in arms}; self.I_peak = {i: 0.0 for i in arms}
        self.lim = np.array([FB_STALL if HUB_C else TAU_STALL[0], F_STALL_C]); self.rated = np.array([FB_RATED if HUB_C else TAU_RATED[0], F_RATED_C])

    def set_open(self, i, fold=(35.0, 0.0, 0.0)):
        """Spread, fingers open enough to pass a bar onto the heel."""
        d = self.d
        d.qpos[self.la[i]] = np.radians(H.SPREAD_MAX); d.qpos[self.qa[i]] = np.radians(np.tile(fold, (4, 1))); d.ctrl[self.uc[i]] = 0.0
        if HUB_C: d.ctrl[self.ub[i]] = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(fold[0]) + 0.6 * FB_RATED / KB_CORD   # ring part-way down: spread, base half open
        else: d.ctrl[self.ul[i]] = np.radians(H.SPREAD_MAX); d.ctrl[self.ub[i]] = np.radians(fold[0])

    def close(self, i, u):
        """u 0 -> 1: wind every cord towards full fold with the force capped at the preload (the fingers wrap the object)."""
        m, d = self.m, self.d
        m.actuator_forcerange[self.ub[i], 1] = PRELOAD * (FB_RATED if HUB_C else TAU_RATED[0]); m.actuator_forcerange[self.uc[i], 1] = PRELOAD * F_RATED_C
        if HUB_C: d.ctrl[self.ub[i]] = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(35.0) + min(u / 0.5, 1.0) * (LEVER[0] * np.radians(FOLD_MAX[0] - 35.0) + 0.004)
        else: d.ctrl[self.ub[i]] = np.radians(35.0) + u * (np.radians(FOLD_MAX[0]) + 0.6 - np.radians(35.0))
        d.ctrl[self.uc[i]] = (np.clip((u - 0.45) / 0.55, 0, 1) if HUB_C else u) * (L_MAX + 0.004)     # hub C: the base ring goes first, then the curl ring

    def hold(self, i):
        """Freeze the servos where they are (wound = actual + preload / stiffness); they then resist up to stall."""
        m, d = self.m, self.d
        if HUB_C:        # a ring is one rigid part: all four cords are wound to the same place (the mean), each has its own TPU link
            d.ctrl[self.ub[i]] = d.ten_length[self.tb[i]].mean() + PRELOAD * FB_RATED / KB_CORD; m.actuator_forcerange[self.ub[i], 1] = FB_STALL
        else: d.ctrl[self.ub[i]] = d.qpos[self.qa[i][:, 0]] + PRELOAD * TAU_RATED[0] / KP[0]; m.actuator_forcerange[self.ub[i], 1] = TAU_STALL[0]
        d.ctrl[self.uc[i]] = (d.ten_length[self.tn[i]].mean() if HUB_C else d.ten_length[self.tn[i]]) + PRELOAD * F_RATED_C / K_CORD; m.actuator_forcerange[self.uc[i], 1] = F_STALL_C
        self.held[i] = True

    def loads(self, i):
        d = self.d; return np.stack([d.actuator_force[self.ub[i]], d.actuator_force[self.uc[i]]], 1)       # (4, 2): base N m, cord N

    def log(self):
        self.n += 1
        for i in self.arms:
            f = self.loads(i); self.sum2[i] += f ** 2; self.peak[i] = np.maximum(self.peak[i], f)
            self.I_peak[i] = max(self.I_peak[i], float((f / self.lim).sum() * 1.4))

    def fold_deg(self, i):
        return np.degrees(self.d.qpos[self.qa[i]]), np.degrees(self.d.qpos[self.la[i]]) * np.array(H.LAG_SIGN)

    def report(self, i):
        rms = np.sqrt(self.sum2[i] / max(self.n, 1))
        return dict(rms_over_rated=[float(x) for x in (rms / self.rated).max(0)], peak_over_stall=[float(x) for x in (self.peak[i] / self.lim).max(0)],
                    servo_A_peak=self.I_peak[i], channels=["base ring cord (spread + K1)", "curl ring cord (K2 + K3)"] if HUB_C else ["K1 base servo", "K2 + K3 cord servo"])


# ---------------------------------------------------------------------------------------------- standalone capacity test
def bench_model(r_bar, m_bar=0.2, L=0.30):
    xml = f"""<mujoco><option timestep="0.0004" gravity="0 0 0" integrator="implicitfast" cone="elliptic" impratio="10"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001" friction="0.8 0.01 0.001"/><joint limited="true"/></default>
<worldbody><body name="pod0"><geom type="sphere" size="0.005" contype="0" conaffinity="0"/>{hand_xml(0)}</body>
<body name="bar" pos="{seat(r_bar) + 0.0005:.5f} 0 0"><freejoint name="barj"/>
<geom name="bar" type="cylinder" fromto="0 0 {-L / 2} 0 0 {L / 2}" size="{r_bar}" mass="{m_bar}" contype="0" conaffinity="4" condim="4" friction="{MU} 0.004 0.0001"/></body>
</worldbody>{tendon_xml([0])}<actuator>{act_xml([0])}</actuator></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def capacity(r_bar, wrench_dir, rate, t_max=6.0, lim_mm=3.0, lim_deg=5.0):
    """Close on a bar of radius r_bar lying on the heel along the grip axis, then ramp a wrench (pod frame, on the bar centre)
    until the bar has moved lim_mm or turned lim_deg.  Returns the wrench magnitude there, and at the rated-torque point."""
    m = bench_model(r_bar); d = mujoco.MjData(m); F = Fingers(m, d, [0]); bid = m.body("bar").id
    F.set_open(0); mujoco.mj_forward(m, d)
    w = np.asarray(wrench_dir, float); p0 = R0 = None; out = dict(limit=None, rated=None)
    while d.time < 1.0 + t_max:
        t = d.time
        if t < 0.5: F.close(0, t / 0.5)
        elif not F.held[0]: F.hold(0)
        d.xfrc_applied[bid] = 0
        if t < 0.45: d.xfrc_applied[bid, 0] = -2.0                      # the bar is pressed on the heel while the fingers close
        if t >= 1.0:
            if p0 is None: p0 = d.xpos[bid].copy(); R0 = d.xmat[bid].reshape(3, 3).copy()
            mag = rate * (t - 1.0); d.xfrc_applied[bid] = w * mag
            dp = np.linalg.norm(d.xpos[bid] - p0) * 1e3
            Re = d.xmat[bid].reshape(3, 3) @ R0.T; ang = np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)))
            if out["rated"] is None and (F.loads(0) / F.rated).max() > 1.0: out["rated"] = float(mag)
            if dp > lim_mm or ang > lim_deg: out["limit"] = float(mag); break
        mujoco.mj_step(m, d)
    out["fold"] = [float(x) for x in F.fold_deg(0)[0].mean(0)]
    return out


if __name__ == "__main__":
    print("hub C" if HUB_C else "rev D drum", "| K1 torque (N m): stall", round(TAU_STALL[0], 3), "rated", round(TAU_RATED[0], 3), "| K2 + K3 cord (N): stall", F_STALL_C, "rated", F_RATED_C, "| moving mass g", round(M_FINGERS * 1e3, 1))
    tests = dict(pull_off_heel=([1, 0, 0, 0, 0, 0], 8.0, "N"), push_on_heel=([-1, 0, 0, 0, 0, 0], 20.0, "N"), across=([0, 1, 0, 0, 0, 0], 8.0, "N"),
                 slide_along_bar=([0, 0, 1, 0, 0, 0], 4.0, "N"), cantilever=([0, 0, 0, 0, 1, 0], 0.15, "N m"), roll_about_bar=([0, 0, 0, 0, 0, 1], 0.05, "N m"),
                 twist_about_axis=([0, 0, 0, 1, 0, 0], 0.15, "N m"))
    res = {}
    for r in (0.010, 0.014, 0.019):
        res[f"D{round(2000 * r)}"] = {}
        for n, (w, rate, unit) in tests.items():
            c = capacity(r, w, rate); res[f"D{round(2000 * r)}"][n] = dict(unit=unit, **c)
            print(f"bar D{2000 * r:.0f} mm  {n:18s} moves 3 mm / 5 deg at {c['limit'] if c['limit'] is not None else float('nan'):6.2f} {unit:3s}   servo above rated from "
                  f"{c['rated'] if c['rated'] is not None else float('nan'):6.2f}   fold {np.round(c['fold'])}")
    json.dump(res, open("hand3_grasp_capacity.json", "w"), indent=1)
