"""Kinematic chain for one tentacle: build from an arm configuration, numba forward kinematics,
capsule self-collision. Base frame: arm root at (HUB_RADIUS,0,0); local z points radially out
(world +x), local x points world +z (up), local y = world +y (tangential)."""
import math
import numpy as np
import numba as nb
import config as C
from servos import SERVOS
from families import FAMILIES, COMPLIANCE

FAM = {f["id"]: f for f in FAMILIES}
ROT, TRANS, EXT, MARK = 0, 1, 2, 3

# local-drive envelopes for non-servo actuators (mass kg, envelope mm, torque Nm, backlash deg, price)
SPECIAL_DRIVES = {
    "CYCLO": dict(mass=0.045, dims=(40.0, 36.0, 36.0), stall=3.0, speed=3.0, price=14, backlash=0.3),   # N20 + printed 1:30 cycloid
    "STS3250x2": dict(mass=0.149, dims=(45.2, 49.4, 35.0), stall=9.8, speed=4.4, price=80, backlash=0.5),   # two STS3250 ganged on one root axis
    "TSA":   dict(mass=0.020, dims=(50.0, 14.0, 14.0), stall=1.2, speed=1.5, price=12, backlash=0.1),   # twisted string, N20
}

def drive_spec(name):
    return SERVOS[name] if name in SERVOS else SPECIAL_DRIVES[name]

SHELL_WALL = 0.0024   # m printed wall around the servo pocket [A]
SHELL_CLEAR = 0.0005  # m clearance servo -> pocket [A]
_ENV_CACHE = {}

CLEVIS_D_SCALE = 57.1 / 48.6   # from arm/pin_cad.py fit check (STS3215)

def servo_envelope(dims, cant_deg):
    """Smallest round vertebra (outer diameter, axial span, both m) that holds one servo whose output
    shaft lies on a hinge axis canted cant_deg from the transverse plane toward the arm axis.
    The servo body is centred on the arm axis (the hinge axis may sit off the centreline)."""
    key = (tuple(dims), round(cant_deg, 2))
    if key in _ENV_CACHE: return _ENV_CACHE[key]
    Ls, Ws, Hs = dims
    c = math.radians(cant_deg)
    a = np.array([0.0, math.cos(c), math.sin(c)]); t = np.array([0.0, -math.sin(c), math.cos(c)]); n = np.array([1.0, 0, 0])
    best = None
    for phi in np.radians(np.arange(0, 180, 2.5)):
        u = math.cos(phi) * t + math.sin(phi) * n; v = np.cross(a, u)
        P = np.array([x * u + y * v + z * a for x in (-Ls / 2, Ls / 2) for y in (-Ws / 2, Ws / 2) for z in (-Hs / 2, Hs / 2)])
        r = float(np.hypot(P[:, 0], P[:, 1]).max()); span = float(np.ptp(P[:, 2]))
        if best is None or r < best[0]: best = (r, span)
    out = (2 * (best[0] * 1e-3 + SHELL_CLEAR + SHELL_WALL), best[1] * 1e-3)
    _ENV_CACHE[key] = out
    return out

def _axis_xy(phi):
    return np.array([math.cos(phi), math.sin(phi), 0.0])

class Chain:
    pass

def build_chain(cfg):
    """cfg keys: family, variant, range_deg, arrangement, actuation, n_nodes, compliance, servo, root_servo"""
    fam = FAM[cfg["family"]]
    var = next(v for v in fam["variants"] if v["v"] == cfg["variant"])
    rng = math.radians(cfg["range_deg"] * var.get("rng_mult", 1.0))
    act = cfg["actuation"]
    L = C.ARM_LENGTH
    N = cfg["n_nodes"]
    servo = cfg.get("servo")
    n_local_root = 2 if act == "hybrid" else 0

    cant_deg = var.get("cant", fam.get("cant", 0)) if fam["bend"] == 1 else 0.0
    def _node_env(sp, spn, cdeg):
        if spn == 1 and fam["bend"] <= 1:
            D_, span_ = servo_envelope(sp["dims"], cdeg)
            if cdeg < 1e-6:
                # servo shaft on a transverse hinge axis: the horn ear, horn, cradle floor, idler ear and
                # screw head stack along the shaft and set the diameter (pin_cad.py: 57.1 mm vs 48.6 mm box)
                D_ *= CLEVIS_D_SCALE
            return D_, span_
        # two servos per node (differentials, 2-axis nodes): side by side, old box rule
        return (max(sp["dims"][1], sp["dims"][2]) * 1e-3 + 0.008) * (1.25 if spn > 1 else 1.0), sp["dims"][0] * 1e-3 * (1.0 if spn == 1 else 1.6)
    # ---------- node geometry ----------
    D = np.zeros(N); P = np.zeros(N); node_act = []; node_servo = None
    if act == "graded":
        # servo in every node, sized to the load: root servo for the first quarter of the arm,
        # STS3215 to 60 %, the small servo beyond; each node as short as its own servo allows
        spn = fam.get("servos_per_node", 2 if fam["bend"] == 2 else 1) * (2 if fam.get("diff") else 1)   # 2-axis nodes carry a servo per axis
        f1, f2 = {"short_tip": (0.25, 0.60), "long_mid": (0.35, 0.85)}[cfg.get("split", "short_tip")]
        bands = [(f1, cfg.get("root_servo", "STS3250_12V")), (f2, "STS3215_12V"), (1.01, cfg.get("servo", "STS3032"))]
        Ds, Ps, names = [], [], []; s = 0.0
        if cfg.get("n_nodes", 0) > 0:
            # fixed joint count (main-design evaluation): uniform pitch, servo band by node start
            n_fix = int(cfg["n_nodes"])
            for k in range(n_fix):
                nm = next(n for f, n in bands if (k * L / n_fix) / L < f); sp = drive_spec(nm)
                env, span = _node_env(sp, spn, cant_deg)
                Ds.append(env); Ps.append(L / n_fix); names.append(nm)
            s = L
        while s < L - 1e-6:
            nm = next(n for f, n in bands if s / L < f); sp = drive_spec(nm)
            env, span = _node_env(sp, spn, cant_deg)
            p = max(var.get("pitch_D", fam["pitch_D"]) * env, span + 0.014)
            if s + p > L and Ps: break           # remainder shorter than a node: stretch the others instead
            Ds.append(env); Ps.append(p); names.append(nm); s += p
        P = np.array(Ps) * L / sum(Ps); D = np.array(Ds); N = len(P); node_act = ["local"] * N; node_servo = names
    elif act == "local":
        sp = drive_spec(fam.get("servo_override", servo))
        spn = fam.get("servos_per_node", 2 if fam["bend"] == 2 else 1) * (2 if fam.get("diff") else 1)   # 2-axis nodes carry a servo per axis
        env, span = _node_env(sp, spn, cant_deg)
        pmin = max(var.get("pitch_D", fam["pitch_D"]) * env, span + 0.014)
        N = max(3, min(N, int(L // pmin)))
        D = np.full(N, env); P = np.full(N, L / N); node_act = ["local"] * N
    else:
        nodes = list(range(N))
        if n_local_root:
            sp = drive_spec(cfg.get("root_servo", "STS3250_12V"))
            env, span = _node_env(sp, 1, cant_deg)
            proot = span + 0.020
            rest = N - n_local_root
            Lr = L - n_local_root * proot
            for k in range(N):
                if k < n_local_root:
                    D[k] = env; P[k] = proot; node_act.append("local")
                else:
                    s = (k - n_local_root + 0.5) / rest
                    D[k] = 0.036 + (C.D_TIP - 0.036) * s; P[k] = Lr / rest; node_act.append("tendon")
        else:
            for k in range(N):
                s = (k + 0.5) / N
                D[k] = C.D_ROOT + (C.D_TIP - C.D_ROOT) * s
            if fam.get("log_taper"):
                P = D / D.sum() * L
            else:
                P[:] = L / N
            node_act = [act] * N
    pitch_fit = float(np.min(P / (var.get("pitch_D", fam["pitch_D"]) * D)))   # <1 means the mechanism does not fit its pitch

    # ---------- elements ----------
    et, ax, ev, ec, el, enode = [], [], [], [], [], []
    var_lo, var_hi, var_kind, var_node, var_axis_kind = [], [], [], [], []
    def new_var(lo, hi, kind, node, ak):
        var_lo.append(lo); var_hi.append(hi); var_kind.append(kind); var_node.append(node); var_axis_kind.append(ak)
        return len(var_lo) - 1
    def add(t, a=(0, 0, 0), v=-1, c=0.0, l=0.0, k=0):
        et.append(t); ax.append(np.asarray(a, float)); ev.append(v); ec.append(c); el.append(l); enode.append(k)

    # section grouping
    nsec = 3
    sec_of = lambda k: min(nsec - 1, k * nsec // N)
    section_vars = {}
    passive_twist = fam.get("twist_uncontrolled", False)
    cant = math.radians(var.get("cant", fam.get("cant", 0)))

    def bend_axes(k):
        arr = cfg["arrangement"]
        if fam["bend"] == 2:
            return [np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]
        if fam["bend"] == 0:
            return []
        if arr == "planar": phi = 0.0 if k == 0 else math.pi / 2   # root yaw, then vertical-plane bending
        elif arr.startswith("alt90"): phi = math.pi / 2 - (k % 2) * math.pi / 2
        elif arr == "helix60": phi = math.pi / 2 + k * math.pi / 3
        else: phi = 0.0
        a = _axis_xy(phi)
        if cant:
            a = np.array([a[0] * math.cos(cant), a[1] * math.cos(cant), math.sin(cant)])
        return [a]

    for k in range(N):
        kind = node_act[k]
        Dk, Pk = D[k], P[k]
        used = 0.0
        # twist (roll) modules
        want_roll = cfg["arrangement"].endswith("roll4") and (k % 4 == 3)
        if want_roll or (fam.get("diff") and kind == "local"):
            vr = new_var(-math.radians(150), math.radians(150), kind, k, "twist")
            add(ROT, (0, 0, 1), vr, 1.0, k=k)
        axes = bend_axes(k)
        for ai, a in enumerate(axes):
            if kind == "section":
                key = (sec_of(k), ai if fam["bend"] == 2 else int(abs(a[1]) > 0.5))
                if key not in section_vars:
                    section_vars[key] = new_var(0, 0, "section", k, "bend")
                v = section_vars[key]
                var_lo[v] -= rng; var_hi[v] += rng
                coef = None   # filled after counting
            else:
                v = new_var(-rng, rng, kind, k, "bend"); coef = 1.0
            if fam["kin"] in ("rolling", "flex", "disc", "kresling"):
                sep = (0.5 * Dk if fam["kin"] == "rolling" else 0.3 * Pk) / max(1, len(axes))
                add(ROT, a, v, 0.5 if coef else -0.5, k=k); add(TRANS, l=sep, k=k)
                add(ROT, a, v, 0.5 if coef else -0.5, k=k)
                used += sep
            else:
                add(ROT, a, v, 1.0 if coef else -1.0, k=k)
        if fam["ext"]:
            frac = 0.30 if fam["kin"] == "kresling" else 0.08
            ve = new_var(-frac * Pk, 0.0, kind, k, "ext")
            add(EXT, (0, 0, 1), ve, 1.0, k=k)
            if fam["kin"] == "kresling":   # contraction twists the module
                add(ROT, (0, 0, 1), ve, math.radians(40) / (frac * Pk) * -1.0, k=k)
        add(TRANS, l=max(Pk - used, 1e-4), k=k)
        add(MARK, k=k)

    # section coefficients: each element gets 1/m of the group variable (m = elements per group, halves counted)
    ec = np.array(ec, float); ev = np.array(ev, int); et_arr = np.array(et, int)
    for key, v in section_vars.items():
        idx = np.where((ev == v) & (et_arr == ROT))[0]
        sub = np.abs(ec[idx])          # 1.0 or 0.5 markers
        n_eff = sub.sum()
        ec[idx] = sub / n_eff
        var_lo[v] = -rng * n_eff / 1.0 * (1.0 / 1.0); var_hi[v] = -var_lo[v]
        var_lo[v] = -rng * n_eff; var_hi[v] = rng * n_eff
        # with coef sub/n_eff, element angle = q*sub/n_eff; max |q| = rng*n_eff gives each full joint +-rng
    ch = Chain()
    ch.node_servo = node_servo; ch.cfg = cfg; ch.fam = fam; ch.var = var; ch.N = N; ch.D = D; ch.P = P; ch.node_act = node_act
    ch.et = et_arr.astype(np.int64); ch.ax = np.array(ax); ch.ev = ev.astype(np.int64); ch.ec = ec
    ch.el = np.array(el, float); ch.enode = np.array(enode, int)
    ch.lo = np.array(var_lo); ch.hi = np.array(var_hi); ch.var_kind = var_kind; ch.var_node = np.array(var_node)
    ch.var_axis_kind = var_axis_kind
    ch.nv = len(var_lo); ch.pitch_fit = pitch_fit; ch.rng = rng
    ch.passive_twist = passive_twist
    ch.seg_r = 0.5 * D
    ch.base_pos = np.array([C.HUB_RADIUS, 0.0, 0.0])
    ch.base_R = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]).T  # columns: local x,y,z in world
    # columns of base_R: local x -> world z, local y -> world y, local z -> world x
    ch.base_R = np.column_stack([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])   # right-handed: x up, y = -world y, z radial
    return ch

@nb.njit(cache=True)
def _rodrigues(a, th):
    c = math.cos(th); s = math.sin(th); v = 1 - c
    x, y, z = a[0], a[1], a[2]
    return np.array([[c + x*x*v, x*y*v - z*s, x*z*v + y*s],
                     [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
                     [z*x*v - y*s, z*y*v + x*s, c + z*z*v]])

@nb.njit(cache=True)
def fk(q, et, ax, ev, ec, el, base_pos, base_R, n_marks):
    """returns marks (n_marks+1,3) incl. base, elem origin (E,3), elem world axis (E,3), elem angle (E), tip R"""
    E = et.shape[0]
    p = base_pos.copy(); R = base_R.copy()
    marks = np.zeros((n_marks + 1, 3)); marks[0] = p
    eo = np.zeros((E, 3)); ea = np.zeros((E, 3)); eang = np.zeros(E)
    m = 1
    for i in range(E):
        t = et[i]
        if t == 0:
            th = ec[i] * q[ev[i]] if ev[i] >= 0 else 0.0
            eo[i] = p; wa = R @ ax[i]; ea[i] = wa; eang[i] = th
            R = R @ _rodrigues(ax[i], th)
        elif t == 1:
            eo[i] = p
            p = p + R[:, 2] * el[i]
        elif t == 2:
            eo[i] = p; ea[i] = R[:, 2]
            d = ec[i] * q[ev[i]]
            eang[i] = d
            p = p + R[:, 2] * d
        else:
            marks[m] = p; m += 1
    return marks, eo, ea, eang, R

@nb.njit(cache=True)
def seg_dist(p1, q1, p2, q2):
    d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2
    a = d1 @ d1; e = d2 @ d2; f = d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return math.sqrt(r @ r)
    if a <= 1e-12:
        s = 0.0; t = min(max(f / e, 0.0), 1.0)
    else:
        c = d1 @ r
        if e <= 1e-12:
            t = 0.0; s = min(max(-c / a, 0.0), 1.0)
        else:
            b = d1 @ d2; den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0; s = min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t = 1.0; s = min(max((b - c) / a, 0.0), 1.0)
    dv = p1 + d1 * s - p2 - d2 * t
    return math.sqrt(dv @ dv)

@nb.njit(cache=True)
def self_penetration(marks, seg_r, shrink):
    """sum of penetration depths over capsule pairs that are far enough apart along the arm to be able
    to touch (arc gap between them > sum of radii); 0 = collision free"""
    n = marks.shape[0] - 1
    s = np.zeros(n + 1)
    for i in range(n):
        d = marks[i + 1] - marks[i]; s[i + 1] = s[i] + math.sqrt(d @ d)
    tot = 0.0
    for i in range(n):
        for j in range(i + 2, n):
            if s[j] - s[i + 1] < shrink * (seg_r[i] + seg_r[j]): continue
            d = seg_dist(marks[i], marks[i + 1], marks[j], marks[j + 1])
            pen = shrink * (seg_r[i] + seg_r[j]) - d
            if pen > 0: tot += pen
    return tot

@nb.njit(cache=True)
def hub_penetration(marks, seg_r, hub_r):
    n = marks.shape[0] - 1
    tot = 0.0
    for i in range(1, n):
        c = np.zeros(3)
        d = seg_dist(marks[i], marks[i + 1], c, c)
        pen = hub_r + seg_r[i] - d
        if pen > 0: tot += pen
    return tot

@nb.njit(cache=True)
def pair_penetration(mA, rA, mB, rB):
    tot = 0.0
    for i in range(mA.shape[0] - 1):
        for j in range(mB.shape[0] - 1):
            d = seg_dist(mA[i], mA[i + 1], mB[j], mB[j + 1])
            pen = 0.9 * (rA[i] + rB[j]) - d
            if pen > 0: tot += pen
    return tot

def run_fk(ch, q):
    return fk(np.asarray(q, float), ch.et, ch.ax, ch.ev, ch.ec, ch.el, ch.base_pos, ch.base_R, ch.N)


@nb.njit(cache=True)
def marks_jac(q, et, ax, ev, ec, el, base_pos, base_R, n_marks, nv):
    """marks (n_marks+1,3), tip dir (3), d marks/dq (n_marks,3,nv), d tipdir/dq (3,nv)"""
    marks, eo, ea, eang, R = fk(q, et, ax, ev, ec, el, base_pos, base_R, n_marks)
    E = et.shape[0]
    J = np.zeros((n_marks, 3, nv)); Jd = np.zeros((3, nv))
    t = R[:, 2]
    # mark index reached after each element
    m = 0
    first_mark_after = np.zeros(E, np.int64)
    for i in range(E):
        first_mark_after[i] = m
        if et[i] == 3: m += 1
    for i in range(E):
        v = ev[i]
        if v < 0: continue
        if et[i] == 0:
            a = ea[i]; o = eo[i]; c = ec[i]
            for j in range(first_mark_after[i], n_marks):
                d = marks[j + 1] - o
                J[j, 0, v] += c * (a[1]*d[2] - a[2]*d[1])
                J[j, 1, v] += c * (a[2]*d[0] - a[0]*d[2])
                J[j, 2, v] += c * (a[0]*d[1] - a[1]*d[0])
            Jd[0, v] += c * (a[1]*t[2] - a[2]*t[1])
            Jd[1, v] += c * (a[2]*t[0] - a[0]*t[2])
            Jd[2, v] += c * (a[0]*t[1] - a[1]*t[0])
        elif et[i] == 2:
            a = ea[i]; c = ec[i]
            for j in range(first_mark_after[i], n_marks):
                J[j, :, v] += c * a
    return marks, t, J, Jd, eo, ea

@nb.njit(cache=True)
def mc_batch(Q, et, ax, ev, ec, el, base_pos, base_R, n_marks, seg_r, hub_r, nbr_marks, nbr_r):
    """for each sample: tip pos, tip dir, self-pen, hub-pen, neighbour-pen"""
    S = Q.shape[0]
    tip = np.zeros((S, 3)); tdir = np.zeros((S, 3)); pen = np.zeros((S, 3))
    for s in range(S):
        marks, eo, ea, eang, R = fk(Q[s], et, ax, ev, ec, el, base_pos, base_R, n_marks)
        tip[s] = marks[n_marks]; tdir[s] = R[:, 2]
        pen[s, 0] = self_penetration(marks, seg_r, 0.9)
        pen[s, 1] = hub_penetration(marks, seg_r, hub_r)
        pen[s, 2] = pair_penetration(marks, seg_r, nbr_marks, nbr_r)
    return tip, tdir, pen
