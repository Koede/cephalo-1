"""Root fixes for the 1 m, 13-joint main design (pin hinge, 60 deg helix, STS3250 in joints 0-4, 16 N claw fans).

The lift test starts with the arm straight out and level, holds the walking stance from 0.5 s, then lifts the foot 45 mm.
Straight out, gravity needs about 7.2 N.m at joint 0 and 4.4 N.m at joint 3 against the STS3250's 4.9 N.m stall.
Each variant changes the root drive, adds a spring, or lets the claw fan carry part of the swing leg, then reruns the
whole dynamics evaluation (hover sag, wave, step, lift).  Writes results/root_fix.csv.

  JOINT_SCENARIO=more_thrust python3 fix_root.py
"""
import os, sys, re, math
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np, pandas as pd, mujoco
from multiprocessing import Pool
import eval_main as E, rank as R, dynamics as DY, metrics as M, config as C

MAIN = ("STS3250 to 35 %, STS3215 beyond", 13, 1.0)
ARMS = 8
# mass added per joint, kg (estimates, flagged in the write-up)
GEAR_KG = 0.045      # m1 spur stage: 20T steel pinion on the servo horn, 40T 7075 gear on the joint pin (webbed), 6802 bearing; see DESIGN.md
DUAL_KG = 0.0745 + 0.010   # second STS3250 in place of the idler stub, plus its bracket
SPRING_KG = 0.015    # steel torsion spring and its printed seat
BIG = {  # maker pages read Nov 9 2026
    "STS3095": dict(stall=9.32, mass=0.1945, speed=31 * 2 * math.pi / 60),   # Feetech STS3095-C002, 95 kg.cm at 12 V
    "XM540_W270": dict(stall=10.6, mass=0.165, speed=30 * 2 * math.pi / 60),  # ROBOTIS XM540-W270, 12 V
}
STS3250 = dict(stall=4.90, mass=0.0745, speed=4.4)

# variant: name -> dict(torque mult per joint, stiffness mult per joint, spring per joint as fraction of straight-out
# gravity torque, fan N during the lift, start the lift test from the stance instead of straight out, added kg per arm,
# speed per joint in rad/s)
def V(name, gear=None, dual=(), big=None, spring=None, fan=0.0, fan_raise=None, from_stance=False, ramp=False, note=""):
    """fan: tip thrust N through the whole lift test; fan_raise: thrust N while the arm rises from straight out (to 0.8 s),
    easing to `fan` by 1.0 s, before the foot lift starts at 1.5 s"""
    return dict(name=name, gear=gear or {}, dual=tuple(dual), big=big or {}, spring=spring or {}, fan=fan, fan_raise=fan_raise,
                from_stance=from_stance, ramp=ramp, note=note)
VARIANTS = [
    V("As locked in (STS3250, no fix)"),
    V("As locked in, lift test started from the stance", from_stance=True, note="diagnostic: same arm, the test skips the raise from straight out"),
    V("2:1 gear on joint 0", gear={0: 2}),
    V("3:1 gear on joint 0", gear={0: 3}),
    V("2:1 gear on joints 0 and 3", gear={0: 2, 3: 2}),
    V("2:1 gear on joints 0 and 1", gear={0: 2, 1: 2}),
    V("2:1 gear on joints 0 to 4", gear={k: 2 for k in range(5)}),
    V("Second STS3250 on joint 0", dual=(0,)),
    V("Second STS3250 on joints 0 and 3", dual=(0, 3)),
    V("Feetech STS3095 in joint 0", big={0: "STS3095"}),
    V("ROBOTIS XM540-W270 in joint 0", big={0: "XM540_W270"}),
    V("Torsion spring on joint 0, half of straight-out load", spring={0: 0.5}),
    V("Torsion spring on joint 0, full straight-out load", spring={0: 1.0}),
    V("Claw fan carries 4 N while the leg swings", fan=4.0),
    V("Claw fan carries 7 N while the leg swings", fan=7.0),
    V("2:1 gear on joints 0 and 3, claw fan 3 N", gear={0: 2, 3: 2}, fan=3.0),
    V("3:1 gear on joints 0 and 3", gear={0: 3, 3: 3}),
    V("Claw fan lifts the arm off the ground, 8 N, then off", fan_raise=8.0),
    V("Claw fan lifts the arm off the ground, 10 N, then off", fan_raise=10.0),
    V("Claw fan lifts the arm off the ground, 12.8 N, then off", fan_raise=12.8),
    V("Claw fan lifts the arm 10 N, then 2 N while the leg swings", fan_raise=10.0, fan=2.0),
    V("2:1 gear on joints 0 and 3, claw fan lifts the arm 8 N, then off", gear={0: 2, 3: 2}, fan_raise=8.0),
    # ramp=True: the arm is raised from straight out to the stance no faster than 80 % of each servo's rated speed,
    # held 0.2 s, then the same 45 mm foot lift; the RMS window is the same 2 s of lifting as the plain test
    V("No hardware change, raised at servo speed", ramp=True),
    V("2:1 gear on joint 0, raised at servo speed", gear={0: 2}, ramp=True),
    V("2:1 gear on joints 0 and 3, raised at servo speed", gear={0: 2, 3: 2}, ramp=True),
    V("2:1 gear on joints 0, 3 and 6, raised at servo speed", gear={0: 2, 3: 2, 6: 2}, ramp=True),
    V("3:1 gear on joint 0, 2:1 on joint 3, raised at servo speed", gear={0: 3, 3: 2}, ramp=True),
    V("Second STS3250 on joints 0 and 3, raised at servo speed", dual=(0, 3), ramp=True),
    V("Feetech STS3095 in joint 0, raised at servo speed", big={0: "STS3095"}, ramp=True),
    V("Torsion spring on joint 0, raised at servo speed", spring={0: 1.0}, ramp=True),
    V("Claw fan 4 N while the leg swings, raised at servo speed", fan=4.0, ramp=True),
    V("2:1 gear on joints 0 and 3, claw fan 2 N, raised at servo speed", gear={0: 2, 3: 2}, fan=2.0, ramp=True),
    V("1.5:1 gear on joint 0, raised at servo speed", gear={0: 1.5}, ramp=True),
    V("Claw fan 4 N only while the arm rises, raised at servo speed", fan_raise=4.0, ramp=True),
    V("Claw fan 6 N only while the arm rises, raised at servo speed", fan_raise=6.0, ramp=True),
    V("Claw fan 8 N only while the arm rises, raised at servo speed", fan_raise=8.0, ramp=True),
]

def added_kg(v):
    kg = GEAR_KG * len(v["gear"]) + DUAL_KG * len(v["dual"]) + SPRING_KG * len(v["spring"])
    kg += sum(BIG[b]["mass"] - STS3250["mass"] for b in v["big"].values())
    return kg

def straight_gravity(ch, masses, kv, cap, fr, first):
    xml, first = _orig_to_mjcf(ch, masses, kv, cap, fr)
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    return np.array([d.qfrc_bias[m.joint(first[k][0]).dofadr[0]] * abs(first[k][1]) for k in range(ch.nv)])

_orig_to_mjcf = DY.to_mjcf; _orig_drive = DY.drive_arrays; _orig_sim = DY._sim; _orig_masses = M.node_masses

def sim_prof(model, data, ch, first, qfun, T, tip_ref_fun, prof):
    """dynamics._sim with a time-varying tip thrust and per-joint saturation time"""
    dt = model.opt.timestep; n = int(T / dt)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip"); b = model.site_bodyid[sid]
    tips = np.zeros((n, 3)); qerr = np.zeros(n); sat = 0; refs = np.zeros((n, 3)); umax = 0.0
    jid = [model.joint(first[v][0]).qposadr[0] for v in range(ch.nv)]
    c0 = np.array([first[v][1] for v in range(ch.nv)]); fr = model.actuator_forcerange[:, 1]; jsat = np.zeros(ch.nv)
    for s_ in range(n):
        t = s_ * dt; qd = qfun(t)
        data.xfrc_applied[:] = 0; data.xfrc_applied[b, 2] = prof(t)
        data.ctrl[:] = c0 * qd; mujoco.mj_step(model, data)
        tips[s_] = data.site_xpos[sid]; qerr[s_] = np.sqrt(np.mean((data.qpos[jid] / c0 - qd) ** 2))
        u = np.abs(data.actuator_force) / np.maximum(fr, 1e-9)
        umax = max(umax, float(u.max())); sat += int((u > 0.98).any())
        if s_ >= 1000: jsat += u > 0.98
        if tip_ref_fun is not None: refs[s_] = tip_ref_fun(qd)
    return tips, qerr, sat / n, refs, umax, jsat / max(n - 1000, 1)

def run(v):
    row = E.one(MAIN)
    def node_masses(ch):
        m = _orig_masses(ch).copy()
        for k in v["gear"]: m[k] += GEAR_KG
        for k in v["dual"]: m[k] += DUAL_KG
        for k in v["spring"]: m[k] += SPRING_KG
        for k, b in v["big"].items(): m[k] += BIG[b]["mass"] - STS3250["mass"]
        return m
    def drive_arrays(*a, **kw):
        kv, cap = _orig_drive(*a, **kw); kv = kv.copy(); cap = cap.copy()
        for k, r in v["gear"].items(): cap[k] *= r; kv[k] *= r * r
        for k in v["dual"]: cap[k] *= 2; kv[k] *= 2
        for k, b in v["big"].items():
            s = BIG[b]["stall"] / cap[k]; cap[k] *= s; kv[k] *= s
        return kv, cap
    G = {}
    def to_mjcf(ch, masses, kv, cap, fr):
        xml, first = _orig_to_mjcf(ch, masses, kv, cap, fr)
        if v["spring"]:
            tau = straight_gravity(ch, masses, kv, cap, fr, first); G["tau0"] = tau
            qa = M.ik_foot(ch, M.FEET[1], M._x0(ch))[0]
            for k, frac in v["spring"].items():
                # linear torsion spring: frac of the straight-out torque at q=0, falling to zero past the stance angle
                t0 = frac * tau[k]; zero = 1.7 * qa[k]                     # spring torque zero at 1.7x the stance angle
                kk = abs(t0 / zero); ref = zero
                G[f"spring{k}"] = (kk, ref)
                nm = first[k][0]
                xml = re.sub(rf'(<joint name="{re.escape(nm)}" [^>]*?)/>', rf'\1 stiffness="{kk:.4f}" springref="{ref:.5f}"/>', xml, count=1)
        return xml, first
    base_sp = [STS3250["speed"]] * 5 + [4.7] * 8   # STS3250 in joints 0-4, STS3215 beyond
    G["speed"] = [(BIG[v["big"][k]]["speed"] if k in v["big"] else base_sp[k]) / v["gear"].get(k, 1) for k in range(13)]
    lift_calls = {"n": 0}
    def sim(model, data, ch, first, qfun, T, tip_ref_fun=None, thrust=0.0):
        if thrust == 0.0:   # the lift test is the one call without hover thrust
            lift_calls["n"] += 1
            if v["from_stance"]:
                q = qfun(0.0)
                for k in range(ch.nv): data.qpos[model.joint(first[k][0]).qposadr[0]] = q[k] / first[k][1]
                mujoco.mj_forward(model, data)
            shift = 0.0
            if v["ramp"]:
                qf0 = qfun; qa0 = qf0(0.0); spd = np.array(G["speed"]) * 0.8
                Tr = max(0.3, float(np.max(np.abs(qa0) * math.pi / (2 * spd))))   # cosine ease: peak rate = amp*pi/(2*Tr)
                shift = Tr + 0.2; G["raise_s"] = Tr
                qfun = lambda t: qa0 * 0.5 * (1 - math.cos(math.pi * t / Tr)) if t < Tr else qf0(t - shift)
            fan = v["fan"]; fr_ = v["fan_raise"]
            prof0 = (lambda t: fan) if fr_ is None else (lambda t: fr_ if t < 0.8 else (fr_ + (fan - fr_) * min(1, (t - 0.8) / 0.2)))
            if v["ramp"] and fr_ is not None:
                prof = lambda t: fr_ if t < shift - 0.2 else fr_ + (fan - fr_) * min(1.0, (t - shift + 0.2) / 0.2)
            else:
                prof = prof0 if not v["ramp"] else (lambda t: prof0(max(0.0, t - shift)))
            res = sim_prof(model, data, ch, first, qfun, T + shift, tip_ref_fun, prof)
            k0 = int(round(shift / model.opt.timestep)); G["jsat"] = res[-1]
            tips, qerr, sat, refs, umax = res[:-1]
            return tips[k0:], qerr[k0:], sat, refs[k0:], umax
        return _orig_sim(model, data, ch, first, qfun, T, tip_ref_fun, thrust=thrust)
    DY.to_mjcf = to_mjcf; DY.drive_arrays = drive_arrays; DY._sim = sim; M.node_masses = node_masses
    try:
        d = R._dyn(dict(row))
    finally:
        DY.to_mjcf = _orig_to_mjcf; DY.drive_arrays = _orig_drive; DY._sim = _orig_sim; M.node_masses = _orig_masses
    dk = added_kg(v); m0 = row["total_mass_kg"]; tw0 = row["thrust_to_weight"]
    out = dict(variant=v["name"], added_g_per_arm=round(dk * 1e3, 1), robot_kg=m0 + ARMS * dk,
               thrust_to_weight=tw0 * m0 / (m0 + ARMS * dk), fan_N=v["fan"], note=v["note"])
    out.update({k: d.get(k) for k in ("lift_tip_rms_mm", "lift_umax", "sag_mm", "wave_tip_rms_mm", "wave_sat_frac", "step_settle_s", "step_final_mm", "dyn_error")})
    # slowest root joint after the change, rad/s (MuJoCo does not model servo speed)
    sp = []
    for k in range(5):
        s = BIG[v["big"][k]]["speed"] if k in v["big"] else STS3250["speed"]
        sp.append(s / v["gear"].get(k, 1))
    out["root_speed_min_rad_s"] = min(sp)
    if "raise_s" in G: out["raise_s"] = round(G["raise_s"], 2)
    if "jsat" in G: out["joints_saturated_after_1s"] = " ".join(str(k) for k in np.where(G["jsat"] > 0.05)[0])
    for k, val in G.items():
        if k.startswith("spring"): out[k + "_Nm_per_rad"] = round(val[0], 3)
    return out

if __name__ == "__main__":
    hz = os.environ.get("WAVE_HZ", "1.0")
    with Pool(int(os.environ.get("WORKERS", "4"))) as p:
        only = os.environ.get("ONLY")   # comma-separated row numbers, e.g. for slow-wave reruns
        vs = [VARIANTS[int(i)] for i in only.split(",")] if only else VARIANTS
        rows = p.map(run, vs)
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(HERE, "results", f"root_fix{os.environ.get('TAG', '')}.csv"), index=False)
    pd.set_option("display.width", 250)
    print(df.drop(columns=["note"]).round(3).to_string())
