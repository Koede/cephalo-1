"""monitor4: the ALWAYS-ON electrical + mechanical overload monitor of Cephalo-1 (merged 2026-11-12; per-arm hand ratings 2026-11-14).

It is switched on by cephalo4.py, which every robot script imports (sim4, body4, fall4, chores4, quad4, flip4, dock4 ...):
its last lines call monitor4.install().  Scripts that use their own cephalo4 copy (hop_core) run through
`python3 monitor4.py run <script.py> [args]`.  That wraps mujoco.mj_step; the first time a full-robot model
(a body "torso" and servo actuators "s<i>_<k>", i.e. everything built on sim4.build()) is stepped, a Recorder is attached to that MjData.  Nothing in the simulation is changed:
the monitor only reads.  At exit it prints an overload table and writes monitor_out/<script>_<n>.json (+ .npz and a
dashboard PNG for the longest run).  CEPH_MONITOR=0 switches it off.

Configuration (environment, all optional):
  CEPH_PACKS=1|2      packs in parallel (default 1)          CEPH_SOC=1.0      state of charge at start
  CEPH_PROP=auto|xl|std   claw-prop (auto: pod mass > 170 g -> xl)
  CEPH_BELLY=edf|trivet|scabbard   belly thrust unit (default edf)
  CEPH_HANDS=1,3      arms carrying a hand on the bayonet connector (switch / pogo-pin channels)
  CEPH_HAND_ARMS=2,6  arms that carry the standard hand3 hand-prop instead of a claw-prop (own thrust ceiling, power
                      constant and ESC); CEPH_HAND_T = its full-throttle thrust at 22.2 V (default 23.35 N, mid),
                      CEPH_HAND_K = W / N^1.5 (10.1), CEPH_HAND_ESC = controller rating (60 A, est.)
  CEPH_MON_EVERY=5    log every n steps          CEPH_MON_PLOT=0   no PNG
Extra actuators (anything not s<i>_<k>) are counted as servos by their force limit (>= 4.5 N m STS3250, >= 2 STS3215,
else STS3032): names with "eye" go on the 7.4 V rail, names with "hand"/"fh"/"winch"/"tent"/"claw" and an arm digit
go through that arm's hand connector, the rest on the dome bus.

Power architecture monitored (merged from electrical-and-stress-monitor.md; ratings marked (est.) are estimates):
  pack(s) 6S 5000 mAh -> one XT150 per pack -> distribution board -> belly ESC (EDF 130 A ESC on XT150 / rotor VESC
  60 A est.) + 8 claw-prop ESCs (45 A; hand arms 60 A) + four 12 V bucks (one per adjacent arm pair: arms 0+1, 2+3, 4+5, 6+7; 15 A peak,
  10 A over 1 s, est.) + 7.4 V rail (STS3032: pod fold, eyestalks, dock release; 5 A buck, est.) + 5 V logic
  (Teensy, 12 XIAO ESP32S3 Sense with cameras: 25 W, est.).
  Per arm: data daisy chain unchanged; power on an 18 AWG trunk (10 A, est.) with taps at j0+j1, j2+j3, j4+j5, j6+j7
  (JST VH 10 A); each servo pigtail 3 A (Molex 5264 class).  Hand: Pololu #2811 switch (2 A continuous, 7.2 A max),
  Mill-Max spring pins 5.2 A derated.
"""
import os, sys, json, time, atexit
import numpy as np

ON = os.environ.get("CEPH_MONITOR", "1") != "0"
EVERY = int(os.environ.get("CEPH_MON_EVERY", 5))
PACKS = int(os.environ.get("CEPH_PACKS", 1))
SOC0 = float(os.environ.get("CEPH_SOC", 1.0))
BELLY = os.environ.get("CEPH_BELLY", "edf")
HANDS = [int(x) for x in os.environ.get("CEPH_HANDS", "").split(",") if x.strip()]
HAND_ARMS = [int(x) for x in os.environ.get("CEPH_HAND_ARMS", "").split(",") if x.strip()]
HAND_P = dict(K=float(os.environ.get("CEPH_HAND_K", 10.1)), T_ref=float(os.environ.get("CEPH_HAND_T", 23.35)), V_ref=22.2, cap=99.0,
              esc=float(os.environ.get("CEPH_HAND_ESC", 60.0)))
OUT = os.environ.get("CEPH_MON_OUT", "monitor_out")

# ---------------------------------------------------------------- electrical models (est. unless a datasheet is named)
SOC_PTS = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
OCV_PTS = np.array([3.27, 3.65, 3.73, 3.77, 3.80, 3.84, 3.88, 3.95, 4.02, 4.10, 4.20])
R_PACK, CAP_AH = 0.019, 5.0
#               stall N m, stall A, no-load A, rated N m, bus V
SERVO_EL = {"STS3250": (4.90, 3.6, 0.30, 1.63, 12.0), "STS3215": (2.94, 2.7, 0.24, 0.981, 12.0),
            "STS3032": (0.441, 1.5, 0.15, 0.147, 7.4)}
ETA_G, I_LOGIC = 0.55, 0.05
P_LOGIC_5V = 25.0             # Teensy + 12 XIAO ESP32S3 Sense (8 tips, 4 eyes) with cameras, ToF, IMU (est.)
I_RAIL74_IDLE = 16 * 0.08     # 8 pod fold + 8 eyestalk STS3032 holding, when not modelled as actuators (est.)
PROP = dict(std=dict(K=10.75, T_ref=12.63, V_ref=25.2, cap=99.0), xl=dict(K=5.1, T_ref=26.5, V_ref=21.3, cap=28.0))
BELLY_M = dict(edf=dict(K=11.2, T_ref=42.4, V_ref=25.2, exp=1.51, esc=130.0, name="EDF"),
               trivet=dict(K=3.12, T_ref=58.6, V_ref=21.0, exp=0.0, esc=60.0, name="rotor (MN605-S)"),
               scabbard=dict(K=3.12, T_ref=58.6, V_ref=21.0, exp=0.0, esc=60.0, name="rotor (MN605-S)"))[BELLY]

# ---------------------------------------------------------------- ratings: channel -> (rating, unit, label)
RATING = {
    "pack_A":      (175.0, "A", "pack current per pack (35 C sustained, est.)"),
    "pack_conn_A": (150.0, "A", "pack connector XT150 (per pack)"),
    "cell_low":    (1.0, "", "cell voltage under load (3.3 V/cell -> 1.0)"),
    "belly_A":     (BELLY_M["esc"], "A", f"{BELLY_M['name']} controller"),
    "belly_lead_A": (150.0, "A", "belly lead XT150"),
    "belly_ceil":  (1.0, "", f"{BELLY_M['name']} thrust / its ceiling at the loaded pack voltage"),
    "esc_A":       (45.0, "A", "claw-prop ESC (hand arms scaled to their 60 A controller)"),
    "prop_ceil":   (1.0, "", "claw-prop / hand-prop thrust / its ceiling at the loaded pack voltage"),
    "pigtail":     (1.0, "", "servo lead current / rating (3 A Molex 5264 pigtail; STS3250 on its own 5 A lead)"),
    "tap_A":       (10.0, "A", "2-servo power tap (JST VH)"),
    "trunk_A":     (20.0, "A", "arm power trunk 18 AWG, short peaks (est.)"),
    "trunk_1s_A":  (10.0, "A", "arm power trunk 18 AWG, 1 s average (est.)"),
    "bus_A":       (15.0, "A", "12 V buck, peak (one per arm pair, est.)"),
    "bus_1s_A":    (10.0, "A", "12 V buck, 1 s average (est.)"),
    "bus_V":       (0.6, "V", "12 V bus overvoltage above 12 V (regeneration; limit 12.6 V)"),
    "rail74_A":    (5.0, "A", "7.4 V rail buck (est.)"),
    "hand_sw_A":   (2.0, "A", "hand 12 V switch, continuous (Pololu #2811; 7.2 A max)"),
    "pogo_A":      (5.2, "A", "hand connector spring pin (derated)"),
    "servo":       (1.0, "", "servo torque / stall"),
    "thermal":     (1.0, "", "servo rms torque over 10 s / rated (heating)"),
    "gear":        (1.0, "", "gear-train load / strip limit (3 x stall, est.)"),
    "slip":        (1.0, "", "clutch slip / travel (1 = hard stop)"),
    "joint_F":     (840.0, "N", "joint radial force (6800 C0; 623 C0 is 220 N)"),
    "link_MPa":    (20.0, "MPa", "link cradle bending stress (printed PETG, est.)"),
    "tip_Nm":      (5.0, "N m", "spigot-pod coupler bending (5 N m design target for the undrawn tip bay, est.)"),
}
Z_LINK = 2.77e-6
BUS_OF_ARM = [0, 0, 1, 1, 2, 2, 3, 3]


def _stype(stall):
    return "STS3250" if stall >= 4.5 else ("STS3215" if stall >= 2.0 else "STS3032")


class Recorder:
    @staticmethod
    def make(m, d):
        import mujoco
        try:
            if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso") < 0: return False
            if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "s0_0") < 0: return False
            return Recorder(m, d)
        except Exception as e:                     # never break a simulation because of the monitor
            print(f"[monitor4] not attached: {e}", file=sys.stderr); return False

    def __init__(self, m, d):
        import mujoco
        self.mj = mujoco; self.m, self.d = m, d
        nid = lambda t, s: mujoco.mj_name2id(m, t, s)
        A, B, J = mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT
        self.nj = 0
        while nid(A, f"s0_{self.nj}") >= 0: self.nj += 1
        nj = self.nj
        self.act = np.array([[nid(A, f"s{i}_{k}") for k in range(nj)] for i in range(8)])
        self.stall = np.abs(m.actuator_forcerange[self.act, 1])
        self.typ = np.vectorize(_stype)(self.stall)
        el = lambda j: np.vectorize(lambda t: SERVO_EL[t][j])(self.typ)
        self.Is, self.I0, self.rated = el(1), el(2), el(3)
        self.lead = np.where(self.typ == "STS3250", 5.0, 3.0)
        self.Kt = self.stall / (self.Is - self.I0); self.R = 12.0 / self.Is
        self.dof = np.array([[m.jnt_dofadr[m.actuator_trnid[a, 0]] for a in row] for row in self.act])
        self.arm_mat = np.array([[m.dof_armature[x] for x in row] for row in self.dof])
        # extra actuators
        ours = set(self.act.ravel().tolist()); self.extra = []
        for a in range(m.nu):
            if a in ours: continue
            nm = (mujoco.mj_id2name(m, A, a) or "").lower()
            if nm.startswith("h3_"): continue          # hand3 finger servos: own 7.4 V supply in the hand, logged by hand3_mj.Fingers
            st = abs(m.actuator_forcerange[a, 1]) if m.actuator_forcelimited[a] else 0.0
            if st <= 0: continue
            arm = next((int(c) for c in nm if c.isdigit()), None)
            where = "rail" if "eye" in nm else ("hand" if any(s in nm for s in ("hand", "fh", "winch", "tent", "claw")) and arm is not None
                                                else "dome")
            self.extra.append((a, _stype(st), where, arm if arm is not None and arm < 8 else 0))
        # clutches, joint load bodies, pods, torso
        self.cl = [(i, k, nid(J, f"c{i}_{k}")) for i in range(8) for k in range(nj) if nid(J, f"c{i}_{k}") >= 0]
        self.cl_q = np.array([m.jnt_qposadr[j] for *_, j in self.cl], int)
        self.cl_tr = np.array([max(abs(m.jnt_range[j, 0]), abs(m.jnt_range[j, 1]), 1e-3) for *_, j in self.cl])
        self.cl_lab = np.array([i * nj + k for i, k, _ in self.cl], int)
        bb = [[nid(B, f"b{i}_{k}") for k in range(nj)] for i in range(8)]
        jb = [[nid(J, f"q{i}_{k}b") for k in range(nj)] for i in range(8)]
        self.has_jl = min(min(r) for r in bb) >= 0 and min(min(r) for r in jb) >= 0
        self.jl_b = np.array(bb).ravel(); self.jl_j = np.array(jb).ravel()
        self.pod = np.array([nid(B, f"pod{i}") for i in range(8)])
        self.has_pod = bool((self.pod >= 0).all())
        self.torso = nid(B, "torso")
        prop = os.environ.get("CEPH_PROP", "auto")
        if prop == "auto":
            prop = "xl" if (self.has_pod and m.body_mass[self.pod].mean() > 0.17) else "std"
        self.prop = PROP[prop]; self.prop_name = prop
        # per-arm claw-prop / hand-prop constants
        self.K_arm = np.array([HAND_P["K"] if i in HAND_ARMS else self.prop["K"] for i in range(8)])
        self.Tref_arm = np.array([HAND_P["T_ref"] if i in HAND_ARMS else self.prop["T_ref"] for i in range(8)])
        self.Vref_arm = np.array([HAND_P["V_ref"] if i in HAND_ARMS else self.prop["V_ref"] for i in range(8)])
        self.cap_arm = np.array([HAND_P["cap"] if i in HAND_ARMS else self.prop["cap"] for i in range(8)])
        self.escscale = np.array([45.0 / HAND_P["esc"] if i in HAND_ARMS else 1.0 for i in range(8)])    # to the 45 A channel rating
        self.soc = SOC0; self.vbus = 12.0; self.dq_prev = d.qvel.copy(); self.n = 0
        self.dt = m.opt.timestep * EVERY
        self.rows = []; self.loads = []; self.labels = {}
        self.script = os.path.basename(sys.argv[0] or "python")

    # ---------------------------------------------------------------- per-sample
    def tick(self):
        self.n += 1
        if self.n % EVERY:
            return
        try:
            self._log()
        except Exception as e:
            print(f"[monitor4] logging stopped: {e}", file=sys.stderr); self.n = -10 ** 12

    def _log(self):
        m, d, mj = self.m, self.d, self.mj
        nj = self.nj
        tau = d.actuator_force[self.act]; w = d.actuator_velocity[self.act]
        frc = np.abs(tau); servo = frc / self.stall
        qacc = d.qacc
        gear = (frc + self.arm_mat / 4 * np.abs(qacc[self.dof]) * 2) / (3 * self.stall)
        Im = frc / self.Kt; pm = tau * w
        P = 12.0 * I_LOGIC + self.R * Im ** 2 + np.where(pm >= 0, pm / ETA_G, pm * ETA_G)
        I_srv = np.maximum(P, 0) / 12.0
        # extras
        bus_extra = np.zeros(4); hand_I = np.zeros(8); rail = I_RAIL74_IDLE; ex_max = 0.0
        for a, t, where, arm in self.extra:
            st, Is, I0, _, V = SERVO_EL[t]
            f = abs(d.actuator_force[a]); v = d.actuator_velocity[a]
            pe = 12.0 * I_LOGIC + (V / Is) * (f / (st / (Is - I0))) ** 2 + max(f * abs(v), 0) / ETA_G
            Ie = pe / V; ex_max = max(ex_max, Ie)
            if where == "rail": rail += Ie
            elif where == "hand": hand_I[arm] += Ie; bus_extra[BUS_OF_ARM[arm]] += Ie
            else: bus_extra[0] += Ie
        for h in HANDS: hand_I[h] += 0.05
        taps = I_srv.reshape(8, nj // 2, 2).sum(2) if nj % 2 == 0 else I_srv
        trunk = I_srv.sum(1) + hand_I
        buses = np.array([trunk[0] + trunk[1], trunk[2] + trunk[3], trunk[4] + trunk[5], trunk[6] + trunk[7]]) + bus_extra
        P_bus = 12.0 * buses.sum()
        E = max(0.5 * 2e-3 * self.vbus ** 2 - P.sum() * self.dt, 0.5 * 2e-3 * 144.0)   # 2 mF; buck cannot sink
        self.vbus = float(np.sqrt(E / 1e-3))
        # thrust from the applied wrenches (read only)
        Rb = d.xmat[self.torso].reshape(3, 3)
        fan = np.linalg.norm(d.xfrc_applied[self.pod, :3], axis=1) if self.has_pod else np.zeros(8)
        belly = max(float(d.xfrc_applied[self.torso, :3] @ Rb[:, 2]), 0.0)
        # pack
        Kb = BELLY_M["K"]
        P_tot = Kb * belly ** 1.5 + np.sum(self.K_arm * fan ** 1.5) + P_bus / 0.9 + rail * 7.4 / 0.9 + P_LOGIC_5V / 0.9
        Eo = 6 * float(np.interp(self.soc, SOC_PTS, OCV_PTS)); Rp = R_PACK / PACKS
        V = (Eo + np.sqrt(max(Eo * Eo - 4 * P_tot * Rp, 0.0))) / 2
        I = P_tot / V; self.soc -= I * self.dt / 3600 / (CAP_AH * PACKS)
        I_esc = self.K_arm * fan ** 1.5 / V * self.escscale; I_belly = Kb * belly ** 1.5 / V
        ceil_p = np.minimum(self.cap_arm, self.Tref_arm * (V / self.Vref_arm) ** 1.5)
        ceil_b = BELLY_M["T_ref"] * (V / BELLY_M["V_ref"]) ** BELLY_M["exp"]
        # clutches
        slip = np.zeros(8 * nj)
        if len(self.cl): slip[self.cl_lab] = np.abs(d.qpos[self.cl_q]) / self.cl_tr
        # joint loads (vectorised)
        F = np.zeros(8 * nj); Mb = np.zeros(8 * nj); tipM = np.zeros(8)
        if self.has_jl:
            mj.mj_rnePostConstraint(m, d)
            ref = d.subtree_com[self.torso]
            c = d.cfrc_int[self.jl_b]; f = c[:, 3:]; tq = c[:, :3]
            pa = d.xanchor[self.jl_j]; ax = d.xaxis[self.jl_j]
            Ma = tq + np.cross(ref - pa, f)
            Mb = np.linalg.norm(Ma - ax * (Ma * ax).sum(1)[:, None], axis=1)
            F = np.linalg.norm(f - ax * (f * ax).sum(1)[:, None], axis=1)
            if self.has_pod:
                cp = d.cfrc_int[self.pod]; xp = d.xpos[self.pod]; xa = d.xmat[self.pod].reshape(8, 3, 3)[:, :, 0]
                Mp = cp[:, :3] + np.cross(ref - xp, cp[:, 3:])
                tipM = np.linalg.norm(Mp - xa * (Mp * xa).sum(1)[:, None], axis=1)
        self.dq_prev = d.qvel.copy()
        r = dict(t=float(d.time), pack_A=I / PACKS, pack_conn_A=I / PACKS, cell_low=3.3 / (V / 6),
                 belly_A=I_belly, belly_lead_A=I_belly, belly_ceil=belly / max(ceil_b, 1e-6),
                 esc_A=I_esc.max(), prop_ceil=(fan / ceil_p).max(), pigtail=max((I_srv / self.lead).max(), ex_max / 3.0),
                 tap_A=taps.max(), trunk_A=trunk.max(), bus_A=buses.max(), bus_V=self.vbus - 12.0, rail74_A=rail,
                 hand_sw_A=hand_I.max(), pogo_A=hand_I.max(), servo=servo.max(), gear=gear.max(), slip=slip.max(),
                 joint_F=F.max(), link_MPa=Mb.max() / Z_LINK / 1e6, tip_Nm=tipM.max(),
                 V=V, I=I, flight=float(fan.sum() + belly > 1.0))
        self.labels_now = dict(pigtail=int((I_srv / self.lead).argmax()), tap_A=int(taps.argmax()), trunk_A=int(trunk.argmax()),
                               bus_A=int(buses.argmax()), esc_A=int(I_esc.argmax()), prop_ceil=int((fan / ceil_p).argmax()),
                               servo=int(servo.argmax()), gear=int(gear.argmax()), slip=int(slip.argmax()),
                               joint_F=int(F.argmax()), link_MPa=int(Mb.argmax()), tip_Nm=int(tipM.argmax()))
        r["lab"] = self.labels_now
        r["bus_all"] = buses; r["trunk_all"] = trunk
        self.rows.append(r)
        self.loads.append((frc / self.rated).astype(np.float32))

    # ---------------------------------------------------------------- summary
    def where(self, ch, idx):
        nj = self.nj
        if ch in ("pigtail", "servo", "gear", "slip", "joint_F", "link_MPa"): return f"arm{idx // nj} j{idx % nj}"
        if ch == "tap_A": return f"arm{idx // (nj // 2)} j{2 * (idx % (nj // 2))}+j{2 * (idx % (nj // 2)) + 1}"
        if ch in ("trunk_A", "esc_A", "prop_ceil", "tip_Nm"): return f"arm{idx}"
        if ch == "bus_A": return f"bus {idx} (arms {2 * idx}+{2 * idx + 1})"
        return ""

    def summary(self):
        if len(self.rows) < 5: return None
        t = np.array([r["t"] for r in self.rows]); out = {}
        series = {ch: np.array([r[ch] for r in self.rows]) for ch in RATING if ch in self.rows[0]}
        # derived: 1 s bus average, 10 s rms thermal
        W1 = max(1, int(round(1.0 / self.dt)))
        busall = np.array([r["bus_all"] for r in self.rows])
        if len(busall) >= W1:
            k = np.ones(W1) / W1
            series["bus_1s_A"] = np.max([np.convolve(busall[:, b], k, "same") for b in range(4)], 0)
            tr = np.array([r["trunk_all"] for r in self.rows])
            series["trunk_1s_A"] = np.max([np.convolve(tr[:, a], k, "same") for a in range(8)], 0)
        L = np.array(self.loads); W10 = max(1, int(round(10.0 / self.dt)))
        ms = np.cumsum(np.vstack([np.zeros((1,) + L.shape[1:]), L ** 2]), 0)
        n = np.full(len(L), W10)                      # samples before the start count as unloaded
        rms = np.sqrt((ms[1:] - ms[np.maximum(np.arange(1, len(L) + 1) - W10, 0)]) / n[:, None, None])
        series["thermal"] = rms.reshape(len(L), -1).max(1)
        th_idx = rms.reshape(len(L), -1).argmax(1)
        for ch, y in series.items():
            rat = RATING[ch][0]; ratio = y / rat
            i = int(ratio.argmax())
            lab = self.rows[i]["lab"].get(ch) if ch in self.rows[i]["lab"] else (int(th_idx[i]) if ch == "thermal" else None)
            out[ch] = dict(peak=float(y[i]), ratio=float(ratio[i]), t=float(t[i]), where=self.where(ch if ch != "thermal" else "servo", lab) if lab is not None else "",
                           over_s=float((ratio > 1.005).sum() * self.dt), unit=RATING[ch][1], label=RATING[ch][2])
        self.series = {"t": t, **{ch: series[ch] / RATING[ch][0] for ch in series}}
        return dict(script=self.script, argv=sys.argv[1:], prop=self.prop_name, belly=BELLY, packs=PACKS, soc0=SOC0, hands=HANDS, hand_arms=HAND_ARMS,
                    sim_s=float(t[-1] - t[0]), min_V=float(min(r["V"] for r in self.rows)), max_pack_A=float(max(r["I"] for r in self.rows)),
                    soc_end=float(self.soc), channels=out)


_RECS = []


def install(mujoco=None):
    if not ON: return
    if mujoco is None:
        import mujoco
    if getattr(mujoco, "_ceph_monitor", False): return
    orig = mujoco.mj_step
    seen = {}
    def mj_step(m, d, *a, **k):
        orig(m, d, *a, **k)
        key = id(d)
        e = seen.get(key)
        if e is None or e[0] is not d:
            r = Recorder.make(m, d)
            seen[key] = e = (d, r)
            if r: _RECS.append(r)
        if e[1]: e[1].tick()
    mujoco.mj_step = mj_step; mujoco._ceph_monitor = True
    atexit.register(report)


def report():
    rs = [s for s in (r.summary() for r in _RECS) if s]
    if not rs: return
    os.makedirs(OUT, exist_ok=True)
    tag = "".join(f"_{k[5:]}{v}" for k, v in sorted(os.environ.items())
                  if k.startswith("CEPH_") and not k.startswith("CEPH_MON") and k != "CEPH_MONITOR")
    base = os.path.splitext(os.path.basename(sys.argv[0] or "python"))[0] + "".join("_" + a for a in sys.argv[1:3] if a.replace(".", "").isalnum()) + tag
    base = "".join(c if (c.isalnum() or c in "._-") else "-" for c in base)[:120]
    worst = {}
    for s in rs:
        for ch, c in s["channels"].items():
            if ch not in worst or c["ratio"] > worst[ch]["ratio"]: worst[ch] = c
    json.dump(dict(runs=rs, worst=worst), open(f"{OUT}/{base}.json", "w"), indent=1)
    longest = max(range(len(rs)), key=lambda i: rs[i]["sim_s"])
    rec = [r for r in _RECS if hasattr(r, "series")][longest]
    np.savez_compressed(f"{OUT}/{base}.npz", **rec.series)
    over = [(ch, c) for ch, c in worst.items() if c["ratio"] > 1.005]
    print(f"\n[monitor4] {base}: {len(rs)} run(s), {sum(s['sim_s'] for s in rs):.1f} s simulated | packs {PACKS}, belly {BELLY}, "
          f"prop {rs[0]['prop']} | min pack {min(s['min_V'] for s in rs):.2f} V, max {max(s['max_pack_A'] for s in rs):.0f} A")
    for ch, c in sorted(worst.items(), key=lambda x: -x[1]["ratio"]):
        flag = "OVER" if c["ratio"] > 1.005 else ("LIM" if c["ratio"] > 0.995 else ("near" if c["ratio"] > 0.85 else "ok"))
        print(f"  {flag:4s} {c['ratio']:5.2f}  {ch:12s} {c['peak']:8.2f} {c['unit']:4s} at {c['t']:6.2f} s {c['where']:24s} "
              f"{'over ' + format(c['over_s'], '.2f') + ' s' if c['over_s'] > 0 else '':12s} {c['label']}")
    print(f"  -> {OUT}/{base}.json" + (f" | {len(over)} channel(s) over their rating" if over else " | nothing over its rating"))
    if os.environ.get("CEPH_MON_PLOT", "1") != "0":
        try:
            plot(f"{OUT}/{base}.npz", f"{OUT}/{base}.png", title=f"{base}: {rs[longest]['sim_s']:.1f} s")
        except Exception as e:
            print(f"[monitor4] plot skipped: {e}")


def plot(npz, png, title=""):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    z = np.load(npz); t = z["t"]
    chans = [c for c in RATING if c in z.files]
    n = len(chans); cols = 2; rows = (n + 1) // 2
    fig, ax = plt.subplots(rows, cols, figsize=(15, 1.35 * rows), sharex=True)
    for j, ch in enumerate(chans):
        a = ax.flat[j]; y = z[ch]
        a.plot(t, y, color="#2f5d9e", lw=1.0); a.axhline(1, color="#c4314b", lw=1, ls="--")
        if (y > 1).any(): a.fill_between(t, 1, y, where=y > 1, color="#c4314b", alpha=0.25, lw=0)
        a.set_ylim(0, max(1.2, float(y.max()) * 1.1))
        a.text(0.005, 0.95, f"{RATING[ch][2]}  (peak {y.max():.2f})", transform=a.transAxes, fontsize=7.5, va="top", color="#1f2328")
        for s in ("top", "right"): a.spines[s].set_visible(False)
        a.tick_params(labelsize=7, colors="#6e7781")
    for a in list(ax.flat)[n:]: a.axis("off")
    fig.suptitle(f"Cephalo-1 overload monitor: fraction of rating (red dashed = 1.0) | {title}", fontsize=10, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(png, dpi=100); plt.close(fig)


if __name__ == "__main__":
    # python3 monitor4.py run <script.py> [args]  -> run any script under the monitor (for copies such as hop_core that
    #                                               import their own cephalo4)
    # python3 monitor4.py monitor_out/<run>.npz   -> replot
    if sys.argv[1:2] == ["run"]:
        import runpy
        install(); sys.argv = sys.argv[2:]
        sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[0])))
        runpy.run_path(sys.argv[0], run_name="__main__"); sys.exit()
    for f in sys.argv[1:]:
        plot(f, f.replace(".npz", ".png")); print("wrote", f.replace(".npz", ".png"))
