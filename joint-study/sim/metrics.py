"""Kinematic, static and build metrics for one arm configuration.

Stage A (kin_eval): everything that depends only on geometry and masses in the arm:
  Monte Carlo workspace / dexterity / collision, shape fits (fluidity), walking IK postures,
  element torques for walking, flight and a unit foot force, tendon wrap angles, Jacobians.
Stage B (score): drive choice, mechanism variant and landing compliance applied analytically:
  torque capacity (servo or tendon with capstan losses), safety factors, landing peak force,
  tip precision (backlash + load deflection + tendon friction + coupling), mass and T/W,
  root channel and mantle-volume checks, cost and garage build effort, composite score.
Every assumption is in config.py or at the top of this file."""
import math
import numpy as np
from scipy.optimize import least_squares
import config as C
import chain as CH
from servos import SERVOS

# ---------- assumptions specific to scoring ----------
RHO_PETG = 1270.0
VERT_FILL = 0.22            # printed vertebra volume fraction of its bounding cylinder [A]
LOCAL_STRUCT = 0.029        # kg per servo node at 65 mm pitch and 57.1 mm diameter, from arm/pin_cad.py: 23 g vertebra + 2.5 g lid
                            # + 3.6 g of screws and inserts beyond the variant's own hardware (was a 3 g guess before the CAD)
LOCAL_STRUCT_D = 0.0571
HW_MASS = dict(bearing=0.004, m3_bolt=0.001, nut=0.0004, dowel=0.002, pin_printed=0.0003, strap=0.0005,
               link_plate=0.002, magnet=0.002, spring=0.002, rod=0.006)
HW_COST = dict(bearing=0.6, m3_bolt=0.05, nut=0.02, dowel=0.15, pin_printed=0.0, strap=0.05, link_plate=0.0,
               magnet=0.25, spring=0.3, rod=1.0)
HUB_R_COLL = 0.100          # mantle collision sphere [A]
VOXEL = 0.025
MC_SAMPLES = 2000
MANTLE_VOL_PER_ARM = 240e-6 # m^3 of base-box room per arm inside the mantle [A]
PACKING = 1.6
SERVO_K_PER_STALL = 1 / math.radians(2.0)   # position-loop stiffness ~ stall torque per 2 deg [A]
LAND_CAP_MULT = 1.5         # brief overload a geared servo survives in a landing [A]
TWIST_PLAY_BALL = math.radians(3.0)
ROUTING_WRAP = math.radians(5.0)            # PTFE channel misalignment per vertebra passed [A]
import os as _os
SCENARIO = _os.environ.get("JOINT_SCENARIO", "baseline")
COUPLING = dict(rolling=0.05, pin=0.25, ball=0.35, flex=0.15, disc=0.15, kresling=0.40, twist=0.10)
MU = dict(tendon=0.15, section=0.15, hybrid=0.15, pushpull=0.25)
TENSION_CAP = dict(tendon=C.TENDON_SWL, section=C.TENDON_SWL, hybrid=C.TENDON_SWL, pushpull=60.0)
if SCENARIO == "tendon_optimistic":   # best case for remote actuation, to test whether the tendon verdict is robust
    MU = dict(tendon=0.07, section=0.07, hybrid=0.07, pushpull=0.15)
    TENSION_CAP = dict(tendon=300.0, section=300.0, hybrid=300.0, pushpull=100.0)
    MANTLE_VOL_PER_ARM = 400e-6
    ROUTING_WRAP = math.radians(2.0)
if SCENARIO == "more_thrust":   # bigger claw props for a 1 m arm: what flies if each tip fan gives 16 N instead of 10.8 N
    C.TIP_THRUST_MAX = 16.0       # N [A] about +48 %; the 30 deg flight-stress case grows with it
    C.TIP_PAYLOAD_MASS = 0.130    # kg [A] heavier motor, prop and ESC at the tip

ICO = None
def _ico_dirs():
    p = (1 + 5 ** 0.5) / 2
    v = np.array([[-1, p, 0], [1, p, 0], [-1, -p, 0], [1, -p, 0], [0, -1, p], [0, 1, p], [0, -1, -p], [0, 1, -p],
                  [p, 0, -1], [p, 0, 1], [-p, 0, -1], [-p, 0, 1]], float)
    v /= np.linalg.norm(v, axis=1)[:, None]
    f = [(0,11,5),(0,5,1),(0,1,7),(0,7,10),(0,10,11),(1,5,9),(5,11,4),(11,10,2),(10,7,6),(7,1,8),
         (3,9,4),(3,4,2),(3,2,6),(3,6,8),(3,8,9),(4,9,5),(2,4,11),(6,2,10),(8,6,7),(9,8,1)]
    c = np.array([v[list(t)].mean(0) for t in f]); return c / np.linalg.norm(c, axis=1)[:, None]
ICO = _ico_dirs()

# ---------- node masses ----------
def node_masses(ch):
    cfg = ch.cfg; fam = ch.fam
    m = np.zeros(ch.N)
    for k in range(ch.N):
        if ch.node_act[k] == "local":
            nm = cfg.get("root_servo", "STS3250_12V") if cfg["actuation"] == "hybrid" else fam.get("servo_override", cfg.get("servo"))
            if ch.node_servo: nm = ch.node_servo[k]
            sp = CH.drive_spec(nm)
            spn = fam.get("servos_per_node", 2 if fam["bend"] == 2 else 1) * (2 if fam.get("diff") else 1)   # 2-axis nodes carry a servo per axis
            if fam.get("shell_struct"):   # enclosed servo shell, mass from arm_cad.py: (kg, D ref m, P ref m)
                ms, d0, p0 = fam["shell_struct"]; struct = ms * (ch.D[k] / d0) ** 2 * (ch.P[k] / p0)
            else:
                struct = LOCAL_STRUCT * (ch.D[k] / LOCAL_STRUCT_D) ** 2 * ch.P[k] / 0.065 * (1.6 if fam.get("diff") else 1.0)
            m[k] = spn * sp["mass"] + struct
        else:
            m[k] = RHO_PETG * math.pi / 4 * ch.D[k] ** 2 * ch.P[k] * VERT_FILL
    # roll modules with local actuation carry a small servo
    for v in range(ch.nv):
        if ch.var_axis_kind[v] == "twist" and ch.var_kind[v] == "local" and not fam.get("diff"):
            m[ch.var_node[v]] += SERVOS["STS3032"]["mass"]
    return m

# ---------- target shapes ----------
def _integrate(kfun, s_marks, ch, nstep=600):
    L = s_marks[-1]; ds = L / nstep
    R = ch.base_R.copy(); p = ch.base_pos.copy()
    out = [p.copy()]; j = 1; s = 0.0
    for i in range(nstep):
        kx, ky, kz = kfun(s + 0.5 * ds)
        w = np.array([kx, ky, kz]) * ds
        th = np.linalg.norm(w)
        if th > 0:
            a = w / th
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = R @ (np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K)
        p = p + R[:, 2] * ds; s += ds
        while j < len(s_marks) and s >= s_marks[j] - 1e-9:
            out.append(p.copy()); j += 1
    while len(out) < len(s_marks): out.append(p.copy())
    return np.array(out)

def target_shapes(ch):
    L = float(ch.P.sum()); s_marks = np.concatenate([[0], np.cumsum(ch.P)])
    a = 0.08; c = 2.5 * math.pi / math.log((L + a) / a)
    r, pitch = 0.040, 0.050; cc = pitch / (2 * math.pi)
    kh, th = r / (r * r + cc * cc), cc / (r * r + cc * cc)
    S = {
        "arc_curl":     lambda s: (0, -1.5 * math.pi / L, 0),
        "spiral_curl":  lambda s: (0, -c / (L + a - s), 0),
        "s_curve":      lambda s: (0, (-1 if s < L / 2 else 1) * (2 * math.pi / 3) / (L / 2), 0),
        "helix_coil":   lambda s: (0, 0, 0) if s < 0.15 * L / 0.585 else (kh, 0, th),
        "side_arc":     lambda s: (math.pi / L, 0, 0),
        "twisted_bend": lambda s: (0, -math.pi / L, 0) if s < L / 2 else (math.pi / L, 0, 0),
        "tip_hook":     lambda s: (0, 0, 0) if s < 0.7 * L else (0, -math.pi / (0.3 * L), 0),
    }
    return {k: _integrate(f, s_marks, ch) for k, f in S.items()}

def _x0(ch):
    return np.clip(np.zeros(ch.nv), ch.lo + 1e-6, ch.hi - 1e-6)

def fit_shape(ch, target, x0=None):
    nv = ch.nv; N = ch.N
    def fun(q):
        m, t, J, Jd, eo, ea = CH.marks_jac(q, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, N, nv)
        return np.concatenate([(m[1:] - target[1:]).ravel(), 1e-3 * q])
    def jac(q):
        m, t, J, Jd, eo, ea = CH.marks_jac(q, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, N, nv)
        return np.vstack([J.reshape(3 * N, nv), 1e-3 * np.eye(nv)])
    x0 = _x0(ch) if x0 is None else x0
    r = least_squares(fun, x0, jac=jac, bounds=(ch.lo, ch.hi), method="trf", max_nfev=150, xtol=1e-7, ftol=1e-7)
    m = CH.run_fk(ch, r.x)[0]
    err = np.linalg.norm(m[1:] - target[1:], axis=1)
    pen = CH.self_penetration(m, ch.seg_r, 0.9) + CH.hub_penetration(m, ch.seg_r, HUB_R_COLL)
    return r.x, float(np.sqrt(np.mean(err ** 2))), float(pen)

# ---------- walking IK ----------
FEET = [np.array([0.300, -0.045, -C.BODY_HEIGHT]), np.array([0.300, 0.0, -C.BODY_HEIGHT]),
        np.array([0.300, 0.045, -C.BODY_HEIGHT]), np.array([0.300 - 0.034, 0.0, -C.BODY_HEIGHT])]
DOWN = np.array([0.0, 0.0, -1.0])

def ik_foot(ch, foot, x0):
    nv = ch.nv; N = ch.N; wd = 0.05; wg = 5.0
    zg = foot[2] + 0.010
    def both(q):
        m, t, J, Jd, eo, ea = CH.marks_jac(q, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, N, nv)
        r = [m[N] - foot, wd * (t - DOWN)]
        Jr = [J[N - 1], wd * Jd]
        below = np.minimum(0.0, m[1:N, 2] - zg)          # interior marks must stay above ground
        r.append(wg * below); Jr.append(wg * J[:N - 1, 2, :] * (below < 0)[:, None])
        r.append(3e-3 * q); Jr.append(3e-3 * np.eye(nv))
        return np.concatenate(r), np.vstack(Jr)
    res = least_squares(lambda q: both(q)[0], x0, jac=lambda q: both(q)[1], bounds=(ch.lo, ch.hi),
                        method="trf", max_nfev=200, xtol=1e-8, ftol=1e-8)
    q = res.x
    m, t, J, Jd, eo, ea = CH.marks_jac(q, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, N, nv)
    perr = float(np.linalg.norm(m[N] - foot)); ang = math.degrees(math.acos(np.clip(t @ DOWN, -1, 1)))
    pen = CH.self_penetration(m, ch.seg_r, 0.9) + CH.hub_penetration(m, ch.seg_r, HUB_R_COLL)
    ok = perr < 0.005 and ang < math.degrees(C.CLAW_VERTICAL_TOL) and pen < 1e-3
    return q, ok, perr, ang, pen

# ---------- statics ----------
def element_moments(ch, q, masses, tip_mass, F_tip):
    """moment vector at every rotation element (world), N*m"""
    m, eo, ea, eang, R = CH.run_fk(ch, q)
    cen = 0.5 * (m[:-1] + m[1:])
    g = np.array([0, 0, -C.G])
    Mv = np.zeros((len(ch.et), 3))
    for e in np.where(ch.et == 0)[0]:
        k = ch.enode[e]; o = eo[e]
        M = np.cross(cen[k:] - o, masses[k:, None] * g).sum(0)
        M += np.cross(m[-1] - o, tip_mass * g + F_tip)
        Mv[e] = M
    return Mv, ea, eang

def var_torques(ch, Mv, ea):
    tau_e = np.einsum("ij,ij->i", Mv, ea)
    tv = np.zeros(ch.nv); tmax = np.zeros(ch.nv)
    for e in np.where((ch.et == 0) & (ch.ev >= 0))[0]:
        v = ch.ev[e]; tv[v] += ch.ec[e] * tau_e[e]; tmax[v] = max(tmax[v], abs(tau_e[e]))
    sec = np.array([k == "section" for k in ch.var_kind])
    signed = np.where(sec, tmax * np.sign(tv + 1e-15), tv)
    return np.where(sec, tmax, np.abs(tv)), tau_e, signed

def node_moment_mag(ch, Mv):
    out = np.zeros(ch.N)
    for e in np.where(ch.et == 0)[0]:
        out[ch.enode[e]] = max(out[ch.enode[e]], np.linalg.norm(Mv[e]))
    return out

def wrap_angles(ch, q):
    """for each var: total |bend| of joints from root up to and including that var's node (rad) + vertebrae passed"""
    _, _, _, eang, _ = CH.run_fk(ch, q)
    node_bend = np.zeros(ch.N)
    for e in np.where(ch.et == 0)[0]:
        node_bend[ch.enode[e]] += abs(eang[e])
    cum = np.cumsum(node_bend)
    return np.array([cum[ch.var_node[v]] + ROUTING_WRAP * (ch.var_node[v] + 1) for v in range(ch.nv)])

# ---------- stage A ----------
def kin_eval(cfg, seed=0):
    ch = CH.build_chain(cfg)
    out = dict(N=ch.N, nv=ch.nv, pitch_fit=ch.pitch_fit, L=float(ch.P.sum()))
    masses = node_masses(ch)
    out["arm_mass_struct"] = float(masses.sum() + C.TIP_PAYLOAD_MASS)
    rng = np.random.default_rng(seed)
    # neighbour arm, straight, 45 deg around the hub
    ang = math.radians(45)
    Rz = np.array([[math.cos(ang), -math.sin(ang), 0], [math.sin(ang), math.cos(ang), 0], [0, 0, 1]])
    m0 = CH.run_fk(ch, _x0(ch))[0]
    nbr = (Rz @ m0.T).T
    # Monte Carlo
    Q = rng.uniform(ch.lo, ch.hi, size=(MC_SAMPLES, ch.nv))
    tip, tdir, pen = CH.mc_batch(Q, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, ch.N, ch.seg_r,
                                 HUB_R_COLL, nbr, ch.seg_r)
    free = (pen[:, 0] < 1e-6) & (pen[:, 1] < 1e-6)
    out["free_frac"] = float(free.mean()); out["nbr_hit_frac"] = float((pen[:, 2] > 1e-6).mean())
    vox = np.floor(tip[free] / VOXEL).astype(int)
    db = np.argmax(tdir[free] @ ICO.T, axis=1)
    if len(vox):
        key = vox[:, 0] * 1000003 + vox[:, 1] * 1009 + vox[:, 2]
        uk, inv = np.unique(key, return_inverse=True)
        pairs = np.unique(inv * 32 + db)
        out["ws_vox"] = int(len(uk)); out["dexterity"] = float(len(pairs) / (20 * len(uk)))
        out["ws_vol_L"] = float(len(uk) * VOXEL ** 3 * 1e3)
    else:
        out["ws_vox"] = 0; out["dexterity"] = 0.0; out["ws_vol_L"] = 0.0
    # shape fits
    shapes = target_shapes(ch); miss = {}; spen = 0.0; wrap_shape = np.zeros(ch.nv)
    for name, tgt in shapes.items():
        qs, rms, pen_s = fit_shape(ch, tgt)
        miss[name] = rms * 1e3 + 1e3 * pen_s
        spen += pen_s
        wrap_shape = np.maximum(wrap_shape, wrap_angles(ch, qs))
    for k, v in miss.items(): out["miss_" + k] = v
    out["miss_mean_mm"] = float(np.mean(list(miss.values())))
    out["miss_planar_mm"] = float(np.mean([miss[k] for k in ("arc_curl", "spiral_curl", "s_curve", "tip_hook")]))
    out["miss_3d_mm"] = float(np.mean([miss[k] for k in ("helix_coil", "side_arc", "twisted_bend")]))
    # walking
    x0 = _x0(ch); oks = []; qs = []
    for f in FEET:
        best = None
        for start in (x0, qs[-1] if qs else x0):
            r = ik_foot(ch, f, start)
            if best is None or (r[1] and not best[1]) or (r[1] == best[1] and r[2] < best[2]): best = r
        qs.append(best[0]); oks.append(best[1])
        out.setdefault("walk_perr_mm", 0.0); out["walk_perr_mm"] = max(out["walk_perr_mm"], best[2] * 1e3)
        out.setdefault("walk_ang_deg", 0.0); out["walk_ang_deg"] = max(out["walk_ang_deg"], best[3])
    out["walk_ok"] = bool(all(oks)); out["walk_ok_frac"] = float(np.mean(oks))
    # torques
    tau_walk = np.zeros(ch.nv); Mn_walk = np.zeros(ch.N); tau_unit = np.zeros(ch.nv); tau_grav = np.zeros(ch.nv)
    wrap_walk = np.zeros(ch.nv); bend_walk = np.zeros(ch.N)
    Fv = np.array([0, 0, C.MASS_TOTAL_REF * C.G / C.FEET_ON_GROUND])
    for q in qs:
        Mv, ea, _ = element_moments(ch, q, masses, C.TIP_PAYLOAD_MASS, Fv)
        tv = var_torques(ch, Mv, ea)[0]; tau_walk = np.maximum(tau_walk, tv)
        Mn_walk = np.maximum(Mn_walk, node_moment_mag(ch, Mv))
        wrap_walk = np.maximum(wrap_walk, wrap_angles(ch, q))
        _, _, _, eang, _ = CH.run_fk(ch, q); nb_ = np.zeros(ch.N)
        for e in np.where(ch.et == 0)[0]: nb_[ch.enode[e]] += abs(eang[e])
        bend_walk = np.maximum(bend_walk, nb_)
    qm = qs[1]
    Mv, ea, _ = element_moments(ch, qm, np.zeros(ch.N), 0.0, np.array([0, 0, 1.0]))   # per newton of foot force
    tau_unit = var_torques(ch, Mv, ea)[2]          # signed: ground reaction pushes the foot up
    Mvg, eag, _ = element_moments(ch, qm, masses, C.TIP_PAYLOAD_MASS, np.zeros(3))
    tau_grav = var_torques(ch, Mvg, eag)[2]        # signed: gravity pulls down
    Mn_unit = node_moment_mag(ch, Mv)
    J = CH.marks_jac(qm, ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, ch.N, ch.nv)[2][ch.N - 1]
    # flight: straight, thrust tilted sideways
    flight = {}
    for nm, thrust, tilt in (("nom", C.TIP_THRUST_HOVER, C.FLIGHT_TILT_NOM), ("max", C.TIP_THRUST_MAX, C.FLIGHT_TILT_MAX)):
        F = thrust * np.array([0, math.sin(tilt), math.cos(tilt)])
        Mv, ea, _ = element_moments(ch, x0, masses, C.TIP_PAYLOAD_MASS, F)
        flight[nm] = (var_torques(ch, Mv, ea)[0], node_moment_mag(ch, Mv))
    out["_arr"] = dict(tau_walk=tau_walk, Mn_walk=Mn_walk, tau_unit=tau_unit, tau_grav=tau_grav, Mn_unit=Mn_unit,
                       J=J, wrap_walk=wrap_walk, bend_walk=bend_walk, wrap_shape=wrap_shape, flight_nom=flight["nom"], flight_max=flight["max"],
                       D=ch.D.copy(), P=ch.P.copy(), var_kind=list(ch.var_kind), var_node=ch.var_node.copy(),
                       var_axis=list(ch.var_axis_kind), node_act=list(ch.node_act), node_servo=ch.node_servo, masses=masses, rng=ch.rng)
    return out

# ---------- stage B ----------
def _channel_capacity(D, od):
    pitch = od + 1.2e-3
    return int(2 * math.pi * 0.38 * D / pitch) + int(2 * math.pi * 0.25 * D / pitch)

def score(kin, cfg, fam, var, comp_name, comp, drive):
    a = kin["_arr"]; nv = kin["nv"]; N = kin["N"]; act = cfg["actuation"]
    D = a["D"]; vk = a["var_kind"]; vn = a["var_node"]
    r = {}
    # --- capacity per variable ---
    cap = np.zeros(nv); k_v = np.zeros(nv); bl = np.zeros(nv); rate = np.zeros(nv); eta = np.ones(nv)
    cap_fly = np.zeros(nv)   # tendon capacity with the arm near straight (flight): only routing friction
    n_drive = 0; n_tendon_root = 0; box_vol = 0.0; box_mass = 0.0; drive_cost = 0.0
    local_name = fam.get("servo_override", cfg.get("servo"))
    for v in range(nv):
        kind = vk[v]; node = vn[v]
        if kind == "local":
            nm = cfg.get("root_servo", "STS3250_12V") if act == "hybrid" else local_name
            if a.get("node_servo"): nm = a["node_servo"][node]
            if a["var_axis"][v] == "twist" and not fam.get("diff"): nm = "STS3032"
            sp = CH.drive_spec(nm)
            mult = 2.0 if fam.get("diff") else 1.0           # differential: both servos share each axis
            cap[v] = sp["stall"] * mult * (0.85 if fam.get("diff") else 1.0); cap_fly[v] = cap[v]
            k_v[v] = sp["stall"] * mult * SERVO_K_PER_STALL
            bl[v] = math.radians(sp["backlash"]) * (1.5 if fam.get("diff") else 1.0)
            rate[v] = sp["speed"]
            if not fam.get("diff") or a["var_axis"][v] == "bend":
                n_drive += 2 if fam.get("diff") else 1; drive_cost += sp["price"] * (2 if fam.get("diff") else 1)
        else:
            if act == "pushpull": kind = "pushpull"
            sp = CH.drive_spec(drive)
            rp = min(max(sp["stall"] / TENSION_CAP.get(kind, C.TENDON_SWL), 0.004), 0.012)   # pulley sized to reach the tendon limit
            rm = 0.38 * D[node]
            mu = MU.get(kind, 0.15)
            wrap = max(a["wrap_walk"][v], 0.0)
            eta[v] = math.exp(-mu * wrap)
            T = min(sp["stall"] / rp, TENSION_CAP.get(kind, C.TENDON_SWL))
            cap[v] = T * rm * eta[v]
            cap_fly[v] = T * rm * math.exp(-mu * ROUTING_WRAP * (node + 1))
            Lt = 0.10 + float(a["P"][:node + 1].sum())
            k_t = C.TENDON_EA / Lt
            k_tendon = 2 * k_t * rm ** 2
            k_drive = sp["stall"] * SERVO_K_PER_STALL * (rm / rp) ** 2
            k_v[v] = 1 / (1 / k_tendon + 1 / k_drive)
            bl[v] = math.radians(sp["backlash"]) * rp / rm + math.radians(0.3)
            rate[v] = sp["speed"] * rp / rm
            n_drive += 1; drive_cost += sp["price"]
            n_tendon_root += 1 if kind == "pushpull" else 2
            box_vol += np.prod(sp["dims"]) * 1e-9 * PACKING; box_mass += sp["mass"] + 0.004
    if box_vol > 0: box_mass += 0.030
    # --- loads ---
    # elastic return of compliant joints adds to what the actuator must supply
    k_el = fam.get("k_el", 0.0) * var.get("k_el_mult", 1.0) * (D / 0.040) ** 3
    tau_el = np.array([k_el[vn[v]] * a["bend_walk"][vn[v]] / max(1, fam["bend"]) for v in range(nv)])
    need_w = a["tau_walk"] + tau_el; need_f = a["flight_max"][0]; need_fn = a["flight_nom"][0]
    with np.errstate(divide="ignore", invalid="ignore"):
        u_walk = np.nanmax(np.where(cap > 0, need_w / cap, 0)); u_fly = np.nanmax(np.where(cap_fly > 0, need_f / cap_fly, 0))
        u_flyn = np.nanmax(np.where(cap_fly > 0, need_fn / cap_fly, 0))
    sk = var["sk"]; Mcap = 20e6 * sk * D ** 3
    s_walk = float(np.max(a["Mn_walk"] / Mcap)); s_fly = float(np.max(a["flight_max"][1] / Mcap))
    r.update(util_walk=float(u_walk), util_flight_nom=float(u_flyn), util_flight_max=float(u_fly),
             struct_walk=s_walk, struct_flight=s_fly)
    # --- mass, thrust to weight ---
    hw = var["hw"]; hw_mass = sum(HW_MASS.get(k, 0.001) * n for k, n in hw.items()) * N
    hw_cost = sum(HW_COST.get(k, 0.1) * n for k, n in hw.items()) * N
    comp_mass = 0.002 * comp.get("parts", 0) * nv
    arm_mass = kin["arm_mass_struct"] + hw_mass + comp_mass
    total = C.MASS_NON_ARM + C.N_ARMS * (arm_mass + box_mass)
    tw = (C.N_ARMS * C.TIP_THRUST_MAX + C.EDF_THRUST) / (total * C.G)
    hover_fan = (total * C.G - C.EDF_THRUST) / C.N_ARMS
    r.update(arm_mass_g=arm_mass * 1e3, box_mass_g=box_mass * 1e3, total_mass_kg=total, thrust_to_weight=tw,
             hover_N_per_fan=hover_fan)
    # --- landing ---
    J = a["J"]; Jz = J[2]
    c_rel, c_stroke = comp.get("k_rel", 1.0), math.radians(comp.get("stroke_deg", 0.0))
    if comp_name == "rigid" and fam.get("landing"):
        c_rel, c_stroke = 0.3, math.radians(10)
    k_soft = 1 / (1 / k_v + 1 / (c_rel * k_v)) if c_stroke > 0 else k_v
    kr = 1 / max(float(np.sum(Jz ** 2 / k_v)), 1e-9); ks = 1 / max(float(np.sum(Jz ** 2 / k_soft)), 1e-9)
    x_c = float(np.sum(np.abs(Jz) * c_stroke)) if c_stroke > 0 else 0.0
    f_of = lambda x: ks * x if x <= x_c else ks * x_c + kr * (x - x_c)
    E_of = lambda x: 0.5 * ks * x * x if x <= x_c else 0.5 * ks * x_c ** 2 + ks * x_c * (x - x_c) + 0.5 * kr * (x - x_c) ** 2
    x_of = lambda F: F / ks if F <= ks * x_c else x_c + (F - ks * x_c) / kr
    m_share = total / C.LANDING_FEET; mg = m_share * C.G; h = C.DROP_HEIGHT
    lo_, hi_ = 0.0, 5.0
    for _ in range(60):                       # peak compression of an unpowered drop
        mid = 0.5 * (lo_ + hi_)
        if E_of(mid) < mg * (h + mid): lo_ = mid
        else: hi_ = mid
    F = f_of(hi_)
    cap_land = np.where(np.array(vk) == "local", cap * LAND_CAP_MULT, cap)
    tu = a["tau_unit"]; tg = a["tau_grav"]
    with np.errstate(divide="ignore", invalid="ignore"):
        tau_land = np.abs(F * tu + tg)
        u_land = float(np.nanmax(np.where(cap_land > 0, tau_land / cap_land, 0)))
        F_allow = float(np.nanmin(np.where(np.abs(tu) > 1e-9, (cap_land - np.sign(tu) * tg) / np.abs(tu), np.inf)))
        F_allow = min(F_allow, float(np.nanmin(np.where(a["Mn_unit"] > 1e-9, Mcap / a["Mn_unit"], np.inf))))
    F_allow = max(F_allow, 0.0)
    E_cap = E_of(x_of(F_allow)) - mg * x_of(F_allow) if F_allow > mg else 0.0
    v_safe_unpowered = math.sqrt(2 * max(E_cap, 0) / m_share)
    # powered touchdown: fans hold hover weight through contact, so only kinetic energy loads the feet
    v_safe = math.sqrt(2 * max(E_of(x_of(F_allow)), 0) / m_share)
    s_land = float(np.max(F * a["Mn_unit"] / Mcap))
    r.update(land_peak_N=F, land_g=F / mg, util_land=u_land, struct_land=s_land, land_stroke_mm=x_c * 1e3,
             foot_allow_N=F_allow, touchdown_v_safe=v_safe, touchdown_v_safe_unpowered=v_safe_unpowered, foot_k_N_per_mm=kr / 1e3)
    # --- precision at the foot (walking load, mid stance) ---
    soft_always = comp_name in ("tendon_spring", "tpu_coupler")   # detent and breakaway are preloaded above walking load
    defl = need_w / (k_soft if soft_always else k_v)
    fr = var["fr"]
    hyst = (1 - eta) * need_w / k_v + fr * need_w / np.maximum(k_v, 1e-9) * 0.5
    coup = 0.0 if act == "local" else COUPLING.get(fam["kin"], 0.2)
    coup_err = coup * 0.05 * a["wrap_shape"]
    dth = np.sqrt(bl ** 2 + (0.3 * defl) ** 2 + hyst ** 2 + coup_err ** 2)
    if comp_name in ("overtravel_detent", "magnetic_breakaway"): dth = np.sqrt(dth ** 2 + math.radians(0.3) ** 2)  # re-seat repeatability [A]
    tip_err = float(np.sqrt(np.sum((np.linalg.norm(J, axis=0) * dth) ** 2)))
    if fam.get("twist_uncontrolled"):
        tip_err = math.sqrt(tip_err ** 2 + (TWIST_PLAY_BALL * 0.04 * math.sqrt(N)) ** 2)
    r.update(tip_err_mm=tip_err * 1e3, eta_min=float(eta.min()), coupling=coup)
    # --- packaging ---
    chan = _channel_capacity(D[0] if act != "hybrid" else D[2] if N > 2 else D[0], 4.0e-3 if act == "pushpull" else C.TENDON_CHANNEL_OD)
    r.update(n_drives_arm=n_drive, n_drives_robot=n_drive * C.N_ARMS, tendons_root=n_tendon_root,
             channel_util=n_tendon_root / max(chan, 1), mantle_util=box_vol / MANTLE_VOL_PER_ARM,
             min_rate_rad_s=float(rate.min()) if nv else 0.0)
    # --- build effort (whole robot, 8 arms) ---
    printed_parts = C.N_ARMS * (N * fam["printed"] + (4 if box_vol > 0 else 0)) + comp.get("parts", 0) * nv * C.N_ARMS
    hw_parts = C.N_ARMS * N * sum(hw.values())
    terms = C.N_ARMS * n_tendon_root * 2
    mins = C.N_ARMS * (N * var["mins"] + n_tendon_root * 6 + n_drive * 8 + comp.get("parts", 0) * nv * 3)
    print_g = 1e3 * (C.N_ARMS * (float(a["masses"].sum()) if act != "local" else N * 0.010) + C.N_ARMS * box_vol * 300)
    cost = C.N_ARMS * (drive_cost + hw_cost) + print_g * 0.025
    r.update(printed_parts=printed_parts, hw_parts=hw_parts, tendon_terminations=terms, assembly_h=mins / 60,
             print_h=print_g / 12.0, cost_usd=cost, tol_crit=var["tol"], supports=fam["supports"], risk=fam["risk"])
    # --- sub-scores ---
    dof = nv
    S = {}
    S["dof"] = 1 - math.exp(-dof / 12)
    S["fluidity"] = math.exp(-kin["miss_mean_mm"] / 25)
    S["workspace"] = min(1.0, kin["ws_vol_L"] / 30.0) * (0.5 + 0.5 * kin["free_frac"])
    S["dexterity"] = min(1.0, kin["dexterity"] / 0.06)
    sf = lambda u: min(1.0, 1 / (C.SF_REQUIRED * max(u, 1e-6)))
    S["loads"] = (sf(r["util_walk"]) * sf(r["util_flight_max"]) * sf(max(s_walk, s_fly)) * max(0.05, min(1.0, v_safe / 0.5))) ** 0.25
    S["precision"] = math.exp(-tip_err * 1e3 / 15)
    S["flight_mass"] = float(np.clip((tw - 1.0) / 0.6, 0, 1.2))
    S["speed"] = min(1.0, r["min_rate_rad_s"] / 3.0)
    S["build"] = ((1 / (1 + mins / 60 / 40)) * (1 / (1 + cost / 1500)) * (1 - 0.5 * var["tol"]) * (1 - 0.6 * fam["risk"])
                  * (1 / (1 + printed_parts / 800)) * (0.85 if fam["supports"] else 1.0)) ** (1 / 4)
    W = dict(dof=1.0, fluidity=2.0, workspace=0.5, dexterity=0.3, loads=1.5, precision=1.0, flight_mass=1.0, speed=0.5, build=2.0)
    lg = sum(W[k] * math.log(max(S[k], 1e-4)) for k in W) / sum(W.values())
    gates = dict(walk=kin["walk_ok"], tw=tw >= 1.15, chan=r["channel_util"] <= 1.0, mantle=r["mantle_util"] <= 1.0,
                 fits=kin["pitch_fit"] >= 0.95, walk_load=r["util_walk"] <= 1.0, flight_load=r["util_flight_nom"] <= 1.0,
                 struct=max(s_walk, float(np.max(a["flight_nom"][1] / Mcap))) <= 1.0)
    r.update({"s_" + k: v for k, v in S.items()})
    r["feasible"] = all(gates.values()); r["fail"] = ",".join(k for k, v in gates.items() if not v)
    r["score"] = math.exp(lg) * (1.0 if r["feasible"] else 0.5)
    r["dof"] = dof
    return r
