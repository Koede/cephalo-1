"""hand3 hub v3, multi-digit control (decision D, 2026-11-20): every finger curls on its own, all four at once.
Each root winds its own flexor with a self-locking M2 lead screw in a leg under the grip; 8 static Pololu 30:1 gearmotors under
the hub (lifted together by one SG92R) turn the screws of the parked roots.  Roots still travel one at a time at the station,
now at az 150.  Timeline: cross -> spin -> move roots to 3 + 1 -> seat over the drivers -> fingers curl, each to its own amount
and at its own time -> open -> 2 + 2 -> the same -> cross.  Spin drawn at 1/600 of the real rpm.
Output: hand3_v3d_site/index.html + sim_data.js;  python3 hand3_v3d_sim.py [check]"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3, hand3_multidigit as MD
V3.STATION_AZ = MD.STATION_AZ
os.chdir(HERE)
import coupler_hand_sim as SIM

FPS = 15; ST = json.load(open("hand3_indep_fingers_study.json"))["common"]; HUB = json.load(open("hand3e/hand3_hubc.json"))
THR = json.load(open("hand3e/hand3_thrust_cad.json"))["low"]; RPM_HOVER = SIM.RPM_HOVER
M_F, R_COM = ST["finger_g"] * 1e-3, ST["finger_com_r"] * 1e-3; W2 = (12500 * 2 * np.pi / 60) ** 2
CURL_LIM = json.load(open("hand3_hub_v3_check.json"))["curl"]
F_STALL, F_RATED = HUB["base"]["F_stall_finger"] * 0.8, HUB["base"]["F_rated_finger"] * 0.8      # both servos, 2:1, eyelet 0.8 (est.)
CROSS, THUMB, PINCH = V3.POSES["flight cross"], V3.POSES["3 + 1 thumb"], V3.POSES["2 + 2 pinch"]


def ease(x): x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


# timeline: (t0, t1, kind, data)
SEG = []
def add(dur, kind, **kw):
    t0 = SEG[-1][1] if SEG else 0.0; SEG.append((t0, t0 + dur, kind, kw))
add(1.0, "park", az=CROSS)
add(2.0, "spinup", az=CROSS); add(3.0, "hover", az=CROSS); add(1.5, "spindown", az=CROSS)
MIN_SEP = 2 * V3.V["block_half"] + 2.0                                           # roots never closer than this (deg), also while moving


def sep_ok(cur, k, a0, a1):
    """True if root k can slide from a0 to a1 (the short way) with every other root at least MIN_SEP away all along."""
    for a in np.linspace(a0, a1, 31):
        for i, b in enumerate(cur):
            if i != k and abs((a - b + 180) % 360 - 180) < MIN_SEP - 1e-6: return False
    return True


def plan(a, b):
    """Order in which to move the roots from pose a to pose b so that no two blocks ever touch (depth-first search)."""
    def dfs(cur, left):
        if not left: return []
        for k in left:
            a1 = b[k]; a0 = cur[k]; a0u = a0 + ((a1 - a0 + 180) % 360 - 180) - ((a1 - a0 + 180) % 360 - 180)
            d = (a1 - a0 + 180) % 360 - 180
            if sep_ok(cur, k, a0, a0 + d):
                nxt = list(cur); nxt[k] = a0 + d; r = dfs(nxt, [x for x in left if x != k])
                if r is not None: return [k] + r
        return None
    order = dfs(list(a), [k for k in range(4) if (a[k] - b[k]) % 360])
    if order is None: raise RuntimeError(f"no collision-free order from {a} to {b}")
    return order


def moves(a, b, order=None):
    """Each root move at the station: align (rotor turns the root to the station), engage (blade lifts its pins), index (hub turns
    under the held root), release (pins drop), retract."""
    order = plan(a, b) if order is None else order
    cur = list(a)
    for k in order:
        d = (b[k] - cur[k] + 180) % 360 - 180
        for ph, dur in (("align", 1.2), ("engage", 0.6), ("index", 1.2), ("release", 0.5), ("retract", 0.4)):
            add(dur, ph, k=k, az=tuple(cur), a0=cur[k], a1=cur[k] + d)
        cur[k] = b[k]
    return tuple(cur)


def seat_psi(az):
    """Rotor angle that puts every root over a driver (world az in MD.DRIVER_AZ)."""
    for psi in range(0, 360, 30):
        if all(round((a + psi) % 360) in MD.DRIVER_AZ for a in az): return float(psi)
    raise RuntimeError(f"no driver set for {az}")


def curl_set(cur, pose, c1, win):
    """Fingers curl on their own: finger k goes 0 -> c1[k] inside its own window win[k] (fractions of the segment)."""
    add(1.0, "seat", az=cur, psi=seat_psi(cur)); add(0.5, "keys", az=cur, up=True)
    add(2.6, "curl", az=cur, c0=[0.0] * 4, c1=list(c1), win=win, pose=pose); add(1.0, "hold", az=cur, c=list(c1), pose=pose)
    add(2.2, "curl", az=cur, c0=list(c1), c1=[0.0] * 4, win=[(0.0, 0.8), (0.1, 0.9), (0.2, 1.0), (0.0, 0.6)], pose=pose)
    add(0.5, "keys", az=cur, up=False)


cur = moves(CROSS, THUMB)
curl_set(cur, "3 + 1 thumb", [0.75, 0.55, 0.75, 0.30], [(0.0, 0.85), (0.15, 1.0), (0.0, 0.6), (0.3, 0.9)])
cur = moves(cur, PINCH)
curl_set(cur, "2 + 2 pinch", [0.70, 0.75, 0.45, 0.75], [(0.0, 1.0), (0.2, 0.8), (0.0, 0.5), (0.1, 0.7)])
cur = moves(cur, CROSS)
curl_set(cur, "flight cross", [0.75, 0.20, 0.60, 0.40], [(0.0, 0.7), (0.0, 1.0), (0.3, 1.0), (0.1, 0.5)])
add(1.0, "park", az=CROSS)
T_END = SEG[-1][1]


def _psi_end():
    """Rotor angle (world) at the end of every segment, so align / index know where they start."""
    psi = 0.0; out = []
    for t0, t1, kind, d in SEG:
        if kind == "align": psi = psi + ((V3.STATION_AZ - (d["a0"] + psi)) + 180) % 360 - 180
        elif kind == "index": psi = psi - (d["a1"] - d["a0"])
        elif kind == "seat": psi = psi + ((d["psi"] - psi) + 180) % 360 - 180
        out.append(psi)
    return out
PSI_END = _psi_end()


def state(t):
    i = next((j for j, (t0, t1, _, _) in enumerate(SEG) if t < t1), len(SEG) - 1)
    t0, t1, kind, d = SEG[i]; psi0 = PSI_END[i - 1] if i > 0 else 0.0; psi1 = PSI_END[i]
    f = np.clip((t - t0) / (t1 - t0), 0, 1); g = ease(f); az = list(d["az"]); c = 0.0; w = 0.0; held = None; s = 0.0
    psi = psi0 + (psi1 - psi0) * g
    if kind == "spinup": w = g
    elif kind == "hover": w = 1.0
    elif kind == "spindown": w = 1 - g
    elif kind in ("align", "engage", "index", "release", "retract"):
        k = d["k"]; held = k; S1, S2 = V3.ST["roll_gap"] + 0.2, V3.ST["stroke"]
        s = {"align": S1 * min(1.0, 3 * f), "engage": S1 + (S2 - S1) * g, "index": S2, "release": S2 - (S2 - S1) * g, "retract": S1 * (1 - g)}[kind]
        if kind == "index": az[k] = V3.STATION_AZ - psi                              # the held root stays at the station
        elif kind in ("release", "retract"): az[k] = d["a1"]
    cs = [0.0] * 4; key = 0.0
    if kind == "curl":
        for k in range(4):
            a, b = d["win"][k]; gk = ease((f - a) / max(b - a, 1e-6)); cs[k] = d["c0"][k] + (d["c1"][k] - d["c0"][k]) * gk
    elif kind == "hold": cs = list(d["c"])
    if kind == "keys": key = g if d["up"] else 1 - g
    elif kind in ("curl", "hold"): key = 1.0
    turn = V3.crank_turn(s) if held is not None else 0.0
    lift = [0.0] * 4
    if held is not None:
        Lf = np.hypot(V3.CR["pivot"][0] - V3.CR["finger"][0], V3.CR["pivot"][1] - V3.CR["finger"][2])
        lift[held] = min(V3.V["lift"], max(0.0, Lf * np.sin(np.radians(turn)) - 0.05))
    cords = [c_ * (V3.K_STROKE["K1"] + V3.K_STROKE["K23"]) for c_ in cs]; pulls = [x / V3.TACKLE for x in cords]
    moving = [kind == "curl" and abs(d["c1"][k] - d["c0"][k]) > 1e-6 and d["win"][k][0] <= f <= d["win"][k][1] for k in range(4)]
    return dict(kind=kind, f=f, az=az, psi=psi, s=s, turn=turn, lift=lift, out=[0.0] * 4, cs=cs, c=max(cs), w=w, cords=cords, pulls=pulls,
                held=held, pose=d.get("pose"), key=key, moving=moving)


def spins():
    ts = np.arange(0, T_END, 1 / FPS); a = 0.0; out = []
    for t in ts:
        out.append(a); a += state(t)["w"] * RPM_HOVER / 600 * 360 / FPS
    return ts, np.array(out)


Tz, Rz = SIM.Tz, SIM.Rz
def Tx(x): A = np.eye(4); A[0, 3] = x; return A
def Sz(z0, L): A = np.eye(4); A[2, 2] = L; A[2, 3] = z0; return A


def seg(a, b):
    """4x4 taking the unit z cylinder (0..1) onto the segment a -> b."""
    d = np.asarray(b, float) - np.asarray(a, float); L = max(np.linalg.norm(d), 1e-4); z = d / L
    x = np.cross([0, 1.0, 0], z); x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-9 else np.array([1.0, 0, 0]); y = np.cross(z, x)
    A = np.eye(4); A[:3, 0] = x; A[:3, 1] = y; A[:3, 2] = z * L; A[:3, 3] = a; return A
M = lambda m: np.asarray(m, float)


def build():
    U = lambda L: mf.Manifold.batch_boolean(L, mf.OpType.Add)
    parts, poses = {}, {}
    def add_(name, m, color, group, pose, opacity=1.0, metal=False, tol=0.04):
        parts[name] = dict(SIM.mesh(m, tol), c=color, g=group, o=opacity, metal=metal); poses[name] = pose
    I4 = lambda s, sp: np.eye(4); SP = lambda sp: Rz(sp)
    add_("frame (driver pockets)", MD.frame_d(), "#e9e9e4", "drive", I4)
    add_("motor base", HC.hardware()["motor_base"], "#555a63", "drive", I4, metal=True)
    add_("8 driver gearmotors (Pololu 30:1)", MD.drivers(False), "#8a8f98", "drive", lambda s, sp: Tz(s["key"] * MD.KEY["lift"]), metal=True, tol=0.02)
    add_("driver lift ring (SG92R)", MD.lift_ring(False), "#f0a030", "drive", lambda s, sp: Tz(s["key"] * MD.KEY["lift"]))
    add_("cage", V3.cage_v3(), "#c9cfd6", "hub", lambda s, sp: SP(sp))
    SA = Rz(V3.STATION_AZ); STP = MD.station_d()
    cols = dict(blade="#f0a030", tyre="#222222", motor="#9aa1aa", servo="#1f4e8c", sensor="#2e8b57")
    for n, m in STP.items():
        add_(f"station {n}", m, cols[n], "station", (lambda s, sp, n=n: SA @ Tx(V3.station_pose(s["s"])[n])), metal=(n == "motor"))
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add_("motor bell", bell, "#3a3f47", "hub", lambda s, sp: SP(sp), metal=True)
    add_("base disc", V3.base_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), tol=0.02)
    add_("top disc", V3.top_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), opacity=0.55, tol=0.02)
    add_("heel nut", C.parts()["heel"], "#2f2f2f", "hub", lambda s, sp: SP(sp))
    COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; R = C.parts()
    strand = mf.Manifold.cylinder(1.0, V3.V["cord_r"], V3.V["cord_r"], 10)
    bandu = {j: mf.Manifold.cylinder(1.0, V3.B.picks()[j]["cs"] / 2 + 0.2, V3.B.picks()[j]["cs"] / 2 + 0.2, 12) for j in range(3)}
    for k in range(4):
        RA = lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k])
        add_(f"root {k} + winch leg (AlSi10Mg)", MD.root_d(0), COL[k], "roots", RA, metal=True, tol=0.02)
        add_(f"lead screw M2 {k}", MD.screw(), "#b8bcc4", "roots", RA, metal=True, tol=0.01)
        add_(f"nut slider {k}", MD.slider(0.0), "#111111", "roots", (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tz(-s["pulls"][k])), tol=0.01)
        for sd in (-1, 1):
            add_(f"latch pin {k}{'ab'[sd > 0]}", V3.latch_pin(0, sd, 0.0, 0.0), "#d62728", "roots",
                 (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tx(s["out"][k]) @ Tz(s["lift"][k])), metal=True, tol=0.01)
        for sd in (-1, 1):
            add_(f"crank {k}{'ab'[sd > 0]}", V3.crank(sd), "#7a3db8", "roots",
                 (lambda s, sp, k=k, sd=sd: SP(sp) @ Rz(s["az"][k]) @ V3.crank_pose(sd, s["turn"] if s["held"] == k else 0.0)), tol=0.01)
            add_(f"crank pin {k}{'ab'[sd > 0]}", V3.crank_pin(sd), "#b8bcc4", "roots", RA, metal=True, tol=0.01)
        for sd in (-1, 1):
            r = MD.SLIDER["eye_x"]; zt = V3.Z_GAP[0] - 1.0
            def cp(s, sp, k=k, sd=sd, r=r, zt=zt):
                zb = MD.SLIDER["z_top0"] - s["pulls"][k] - 1.0
                return SP(sp) @ Rz(s["az"][k]) @ Tx(r) @ M(C.T(t=[0, sd * 0.7, 0])) @ Sz(zb, zt - zb)
            add_(f"tackle cord {k}{'ab'[sd > 0]}", strand, "#f2f2f2", "roots", cp, tol=0.0)
        for j in range(3):
            fp = lambda s, sp, k=k, j=j: SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(C.finger_frames(0, 0.0, tuple(f * s["cs"][k] for f in H.FOLD_MAX))[j + 1])
            add_(f"finger {k} phalanx {j + 1}", V3.B.phalanx_v3(j), "#2b2b30", "fingers", fp)
            # return band j: dowels ride their members; the band is drawn as 5 straight pieces along its centreline (wrap included)
            def fpx(s, sp, k=k, j=j):
                fr = C.finger_frames(0, 0.0, tuple(f * s["cs"][k] for f in H.FOLD_MAX))
                return SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(fr[0] @ C.T(C.rot("x", H.PITCH)) if j == 0 else fr[j])
            add_(f"finger {k} dowel {j + 1}p", V3.B.dowel_prox(j), "#b8bcc4", "fingers", fpx, metal=True, tol=0.01)
            add_(f"finger {k} dowel {j + 1}d", V3.B.dowel_dist(j), "#b8bcc4", "fingers", fp, metal=True, tol=0.01)
            cs = V3.B.picks()[j]["cs"]
            for q in range(5):
                def bp(s, sp, k=k, j=j, q=q):
                    pts = V3.B.band_points(j, H.FOLD_MAX[j] * s["cs"][k]); a, b = pts[q], pts[q + 1]
                    return fpx(s, sp, k, j) @ seg(a, b)
                add_(f"band {k}.{j + 1}.{q}", bandu[j], "#3fae5a", "fingers", bp, tol=0.0)
            add_(f"finger {k} pad {j + 1}", R[f"pad{j + 1}"], "#c23b22", "fingers", fp)
    return parts, poses


def imbalance(az):
    v = sum(np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]) for a in az); return M_F * W2 * R_COM * np.linalg.norm(v)


def hud_rows(s, t):
    imb = imbalance(s["az"]); ok = imb < 5
    rpm = s["w"] * RPM_HOVER
    names = {"park": "parked", "spinup": "rotor spin-up (flight cross only)", "hover": "hover: fingers are the rotor blades", "spindown": "spin-down",
             "align": f"roller turns the rotor: root {s['held']} to the station", "engage": f"blade pushes root {s['held']}'s cranks: pins lift",
             "index": f"roller turns the hub under the held root {s['held']}", "release": "blade backs off: pins drop into the new holes", "retract": "station retracts",
             "seat": "roller turns the rotor: every root over a driver", "keys": "drivers rise / drop (one SG92R)",
             "curl": f"each finger winds on its own ({s['pose']})", "hold": f"hold ({s['pose']})"}
    ph = names[s["kind"]]
    pins = "all 8 down" if max(s["lift"]) < 0.05 else f"root {s['held']}: lifted {max(s['lift']):.1f} mm"
    rows = [["root azimuths", " / ".join(f"{a % 360:.0f}°" for a in s["az"])],
            ["latch pins", pins],
            ["station carriage (az 150)", f"{s['s']:.1f} / {V3.ST['stroke']:.1f} mm", s["s"], V3.ST["stroke"], "#f0a030"],
            ["drivers", "engaged" if s["key"] > 0.99 else ("down" if s["key"] < 0.01 else "moving")]]
    for k in range(4):
        rows.append([f"finger {k}: curl / slider", f"{100 * s['cs'][k]:.0f} % / {s['pulls'][k]:.1f} mm" + (" (winding)" if s["moving"][k] else ""), s["cs"][k], 1.0, ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"][k]])
    rows += [["imbalance if spun at 12,500 rpm", f"{imb:.0f} N", imb, 1500, "#e04b39" if not ok else "#39a0e0"],
             ["spin allowed (interlock)", "yes" if (ok and s["key"] < 0.01 and max(s["cs"]) < 0.01) else "NO"],
             ["rotor speed", f"{int(rpm):,} rpm", rpm, THR["rpm"], "#39a0e0"],
             ["cord force per finger (est.)", "55 N needed; 30:1 N20 on an M2 screw gives ~120 N at stall"]]
    return ph, rows


def main(check=False):
    parts, poses = build(); ts, sp = spins(); names = list(parts)
    frames = np.zeros((len(ts), len(names), 12), np.float32); hud = []
    for i, (t, a) in enumerate(zip(ts, sp)):
        s = state(t)
        for j, n in enumerate(names): frames[i, j] = poses[n](s, a + s["psi"])[:3, :4].reshape(-1)
        ph, rows = hud_rows(s, t); hud.append(dict(t=round(float(t), 2), ph=ph, rows=rows))
    jumps = []; nxt = {}
    for i, (t0, t1, kind, d) in enumerate(SEG):
        if kind == "spinup": jumps.append(["spin", round(t0, 2)])
        MV = ("align", "engage", "index", "release", "retract")
        if kind == "align" and (i == 0 or SEG[i - 1][2] not in MV):
            j = i
            while SEG[j][2] in MV: j += 1
            tgt = next((p for p, a in V3.POSES.items() if tuple(a) == tuple(SEG[j][3]["az"])), "pose")
            jumps.append([f"to {tgt}".replace("flight cross", "cross"), round(t0, 2)])
        if kind == "curl" and max(d["c1"]) > 0: jumps.append([f"curl {d['pose']}", round(t0, 2)])
    data = dict(names=names, parts=[parts[n] for n in names], nf=len(ts), fps=FPS, frames=base64.b64encode(frames.tobytes()).decode(), hud=hud,
                title="hand3 hub v3 D: every finger on its own", groups=[["drive", "frame + servos"], ["station", "station"], ["hub", "hub + cage"], ["roots", "roots + winches + pins"], ["fingers", "fingers"]],
                jumps=[["start", 0]] + jumps, view=dict(wide=[85, 75, 60], close=[34, 26, 92]),
                note="Real CAD solids from hand3e/hand3_multidigit.py and hand3_hub_v3.py. Each root has its own winch: a self-locking M2 lead screw in a leg under the grip moves a nut slider that pulls the finger's 2:1 tackle. Eight static gearmotors under the hub rise together and turn the screws of whatever roots are parked over them, so every finger curls on its own, all at once. Roots still move at the station (az 150). Spin shown at 1/600 of real rpm. Loads are estimates.")
    os.makedirs("hand3_v3d_site", exist_ok=True)
    open("hand3_v3d_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    open("hand3_v3d_site/index.html", "w").write(open("hand3_v3_sim_template.html").read().replace("<title>Cephalo-2 coupler + hand3 motion sim</title>", "<title>hand3 hub v3 D motion sim</title>"))
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("hand3_v3d_site/sim_data.js") / 1e6, 2), "MB")
    if check: overlap(parts, poses, ts, sp)


def overlap(parts, poses, ts, sp, step=10):
    """Occupancy-grid overlap over the whole timeline (0.5 mm cells, eroded one cell; walls under ~1.5 mm are not seen, the
    range check in hand3_hub_v3_check covers those)."""
    sys.argv = sys.argv[:1]
    import coupler_sim_overlap as OV
    names = list(parts); G = {n: OV.grid(n, parts[n]) for n in names if not n.startswith("tackle cord") and not n.startswith("band ")}
    hits = {}
    for fi in sorted(set(list(range(0, len(ts), step)) + [int(t0 * FPS) + 1 for t0, _, k, _ in SEG] + [int(t1 * FPS) - 1 for _, t1, _, _ in SEG])):
        fi = min(fi, len(ts) - 1); s = state(ts[fi]); A = {n: poses[n](s, sp[fi] + s["psi"]) for n in G}
        bx = {n: OV.world_box(G[n], A[n]) for n in G}; nn = list(G)
        for i, a in enumerate(nn):
            for b in nn[i + 1:]:
                la, ha = bx[a]; lb, hb = bx[b]
                if np.any(la > hb) or np.any(lb > ha): continue
                Mab = np.linalg.inv(A[b]) @ A[a]; p = G[a]["pts"] @ Mab[:3, :3].T + Mab[:3, 3]
                idx = np.floor((p - G[b]["lo"]) / OV.CELL).astype(int); okm = np.all((idx >= 0) & (idx < G[b]["n"]), 1)
                if not okm.any(): continue
                ii = idx[okm]; n_in = int(G[b]["er"][ii[:, 0], ii[:, 1], ii[:, 2]].sum())
                if n_in:
                    key = f"{a} x {b}"; v = n_in * OV.CELL ** 3
                    if key not in hits or v > hits[key]["vol_mm3"]: hits[key] = dict(a=a, b=b, t=round(float(ts[fi]), 2), vol_mm3=round(v, 2))
    out = sorted(hits.values(), key=lambda h: -h["vol_mm3"])
    json.dump(dict(cell_mm=OV.CELL, overlaps=out), open("hand3_v3d_sim_overlap.json", "w"), indent=1)
    print("overlaps:", len(out))
    for h in out[:40]: print(f"  {h['vol_mm3']:8.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main(check=len(sys.argv) > 1 and sys.argv[1] == "check")
