"""hand3 hub v3 motion sim (2026-11-19): travelling finger roots, axial latch pins, one base ring with a 2:1 tackle per finger.
Every rigid part comes from hand3e/hand3_hub_v3.py (and hub E's drive from hand3e/hand3_hubc.py); each frame stores only poses.
Timeline: parked in the flight cross -> spin-up, hover, spin-down -> roots move one at a time to the 3 + 1 thumb pose (fork lifts the
pins, the root slides round, pins drop) -> curl to the empty-hand limit -> open -> 2 + 2 pinch -> curl -> open -> back to the cross.
The roots are shown sliding round the hub; on the real hand the parked fork holds the root and a roller turns the hub under it
(fork and roller not modelled yet).  Spin is drawn at 1/600 of the real rpm.  Loads are estimates.
Output: hand3_v3_site/index.html + sim_data.js;  python3 hand3_v3_sim.py [check]  ('check' also runs the overlap check)"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3
os.chdir(HERE)
import coupler_hand_sim as SIM

FPS = 20; ST = json.load(open("hand3_indep_fingers_study.json"))["common"]; HUB = json.load(open("hand3e/hand3_hubc.json"))
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
    order = plan(a, b) if order is None else order
    cur = list(a)
    for k in order:
        d = (b[k] - cur[k] + 180) % 360 - 180
        add(2.0, "move", k=k, az=tuple(cur), a0=cur[k], a1=cur[k] + d); cur[k] = b[k]
    return tuple(cur)
cur = moves(CROSS, THUMB)
add(2.0, "curl", az=cur, c0=0.0, c1=CURL_LIM["3 + 1 thumb"]["max_clear_curl"], pose="3 + 1 thumb"); add(1.0, "hold", az=cur, c=CURL_LIM["3 + 1 thumb"]["max_clear_curl"], pose="3 + 1 thumb")
add(1.5, "curl", az=cur, c0=CURL_LIM["3 + 1 thumb"]["max_clear_curl"], c1=0.0, pose="3 + 1 thumb")
cur = moves(cur, PINCH)
add(2.0, "curl", az=cur, c0=0.0, c1=CURL_LIM["2 + 2 pinch"]["max_clear_curl"], pose="2 + 2 pinch"); add(1.0, "hold", az=cur, c=CURL_LIM["2 + 2 pinch"]["max_clear_curl"], pose="2 + 2 pinch")
add(1.5, "curl", az=cur, c0=CURL_LIM["2 + 2 pinch"]["max_clear_curl"], c1=0.0, pose="2 + 2 pinch")
cur = moves(cur, CROSS)
add(1.0, "park", az=CROSS)
T_END = SEG[-1][1]


def state(t):
    for t0, t1, kind, d in SEG:
        if t < t1 or (t0, t1) == SEG[-1][:2]:
            break
    f = np.clip((t - t0) / (t1 - t0), 0, 1); az = list(d["az"]); lift = [0.0] * 4; out = [V3.V["float_"]] * 4; c = 0.0; w = 0.0; held = None
    if kind == "spinup": w = ease(f)
    elif kind == "hover": w = 1.0
    elif kind == "spindown": w = 1 - ease(f)
    elif kind == "move":
        k = d["k"]; held = k
        if f < 0.25: g = ease(f / 0.25); out[k] = V3.V["float_"] * (1 - g); lift[k] = V3.V["lift"] * g
        elif f < 0.75: out[k] = 0.0; lift[k] = V3.V["lift"]; az[k] = d["a0"] + (d["a1"] - d["a0"]) * ease((f - 0.25) / 0.5)
        else: g = ease((f - 0.75) / 0.25); az[k] = d["a1"]; lift[k] = V3.V["lift"] * (1 - g); out[k] = V3.V["float_"] * g if g > 0.9 else 0.0
    elif kind == "curl": c = d["c0"] + (d["c1"] - d["c0"]) * ease(f)
    elif kind == "hold": c = d["c"]
    if kind in ("park", "move", "curl", "hold"): out = [0.0 if (kind == "move" and i == d.get("k")) else o for i, o in enumerate(out)]
    if w == 0 and kind != "move": out = [0.0] * 4                                    # at rest the pins sit at their inner position
    cord = c * (V3.K_STROKE["K1"] + V3.K_STROKE["K23"]); pull = cord / V3.TACKLE
    return dict(kind=kind, f=f, az=az, lift=lift, out=out, c=c, w=w, cord=cord, pull=pull, held=held, pose=d.get("pose"))


def spins():
    ts = np.arange(0, T_END, 1 / FPS); a = 0.0; out = []
    for t in ts: out.append(a); a += state(t)["w"] * RPM_HOVER / 600 * 360 / FPS
    return ts, np.array(out)


Tz, Rz = SIM.Tz, SIM.Rz
def Tx(x): A = np.eye(4); A[0, 3] = x; return A
def Sz(z0, L): A = np.eye(4); A[2, 2] = L; A[2, 3] = z0; return A
M = lambda m: np.asarray(m, float)


def build():
    U = lambda L: mf.Manifold.batch_boolean(L, mf.OpType.Add)
    parts, poses = {}, {}
    def add_(name, m, color, group, pose, opacity=1.0, metal=False, tol=0.04):
        parts[name] = dict(SIM.mesh(m, tol), c=color, g=group, o=opacity, metal=metal); poses[name] = pose
    I4 = lambda s, sp: np.eye(4); SP = lambda sp: Rz(sp)
    dr = V3.drive(0.0)
    add_("hub E frame", dr["frame"], "#e9e9e4", "drive", I4)
    add_("STS3215 servos (2)", dr["servo_base"] + dr["servo_curl"], "#1f4e8c", "drive", I4)
    add_("spools", dr["base_spool"] + dr["curl_spool"], "#9aa1aa", "drive", I4)
    add_("idlers MR63", U([m for n, m in dr.items() if n.startswith("idler")]), "#9aa1aa", "drive", I4, metal=True)
    add_("carbon rods dia 3", U([m for n, m in dr.items() if n.startswith("rod")]), "#2a2a2a", "drive", I4)
    add_("tie ring", dr["tie ring"], "#cfcfc8", "drive", I4)
    add_("motor base", HC.hardware()["motor_base"], "#555a63", "drive", I4, metal=True)
    for w in ("out", "in"):
        for sd in ("near", "far"):
            add_(f"yoke {w} {sd}", V3.yoke(w, sd, 0.0), "#f0a030", "ring", (lambda s, sp: Tz(-(s["pull"] + HC.GAP)) if s["pull"] > 0 else np.eye(4)))
    P = HC._parts()
    add_("base pull ring + rail", P["ring_out"][0] + V3.ring_rail(), "#e07b39", "ring", lambda s, sp: SP(sp) @ Tz(-s["pull"]))
    add_("cage", P["cage"][0], "#c9cfd6", "hub", lambda s, sp: SP(sp))
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add_("motor bell", bell, "#3a3f47", "hub", lambda s, sp: SP(sp), metal=True)
    add_("base disc", V3.base_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), tol=0.02)
    add_("top disc", V3.top_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), opacity=0.55, tol=0.02)
    add_("heel nut", C.parts()["heel"], "#2f2f2f", "hub", lambda s, sp: SP(sp))
    COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; R = C.parts()
    strand = mf.Manifold.cylinder(1.0, V3.V["cord_r"], V3.V["cord_r"], 10)
    for k in range(4):
        RA = lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k])
        add_(f"root block {k}", V3.root_block(0), COL[k], "roots", RA, tol=0.02)
        for sd in (-1, 1):
            add_(f"latch pin {k}{'ab'[sd > 0]}", V3.latch_pin(0, sd, 0.0, 0.0), "#d62728", "roots",
                 (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tx(s["out"][k]) @ Tz(s["lift"][k])), metal=True, tol=0.01)
        add_(f"bead {k}", V3.bead(0, 0.0), "#111111", "roots", (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tz(-s["pull"])), tol=0.01)
        add_(f"spring box {k}", V3.spring_box(0, k), "#e6c229", "roots", RA)
        for sd in (-1, 1):
            r = V3.V["bead"]["eye_r"]; zt = V3.Z_GAP[0] - 1.0
            def cp(s, sp, k=k, sd=sd, r=r, zt=zt):
                zb = V3.Z_RING_TOP - s["pull"] + V3.V["bead"]["dz"][1] - 0.3
                return SP(sp) @ Rz(s["az"][k]) @ Tx(r) @ M(C.T(t=[0, sd * 0.7, 0])) @ Sz(zb, zt - zb)
            add_(f"tackle cord {k}{'ab'[sd > 0]}", strand, "#f2f2f2", "roots", cp, tol=0.0)
        for j in range(3):
            fp = lambda s, sp, k=k, j=j: SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(C.finger_frames(0, 0.0, tuple(f * s["c"] for f in H.FOLD_MAX))[j + 1])
            add_(f"finger {k} phalanx {j + 1}", R[f"phalanx{j + 1}"], "#2b2b30", "fingers", fp)
            add_(f"finger {k} pad {j + 1}", R[f"pad{j + 1}"], "#c23b22", "fingers", fp)
    return parts, poses


def imbalance(az):
    v = sum(np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]) for a in az); return M_F * W2 * R_COM * np.linalg.norm(v)


def hud_rows(s, t):
    imb = imbalance(s["az"]); ok = imb < 5
    rpm = s["w"] * RPM_HOVER
    names = {"park": "parked", "spinup": "rotor spin-up (flight cross only)", "hover": "hover: fingers are the rotor blades", "spindown": "spin-down",
             "move": f"fork lifts root {s['held']}'s pins, root slides round, pins drop", "curl": f"curl ({s['pose']})", "hold": f"hold ({s['pose']})"}
    ph = names[s["kind"]]
    pins = "all 8 down" if s["held"] is None else f"root {s['held']}: lifted {max(s['lift']):.1f} mm"
    rows = [["root azimuths", " / ".join(f"{a:.0f}°" for a in s["az"])],
            ["latch pins", pins],
            ["imbalance if spun at 12,500 rpm", f"{imb:.0f} N", imb, 1500, "#e04b39" if not ok else "#39a0e0"],
            ["spin allowed (interlock)", "yes" if ok else "NO: roots not balanced"],
            ["rotor speed", f"{int(rpm):,} rpm", rpm, THR["rpm"], "#39a0e0"],
            ["curl (all knuckles together)", f"{100 * s['c']:.0f} % of full fold"],
            ["cord travel per finger", f"{s['cord']:.1f} / {V3.CORD_STROKE:.1f} mm"],
            ["ring stroke (2:1 tackle)", f"{s['pull']:.2f} / {V3.RING_STROKE:.1f} mm", s["pull"], V3.RING_STROKE, "#e07b39"],
            ["both STS3215 servos", "pulling" if s["kind"] == "curl" and s["c"] > 0 and s["f"] < 1 else ("holding" if s["c"] > 0.01 else "idle")],
            ["cord force per finger rated / stall (est.)", f"{F_RATED:.0f} / {F_STALL:.0f} N"]]
    return ph, rows


def main(check=False):
    parts, poses = build(); ts, sp = spins(); names = list(parts)
    frames = np.zeros((len(ts), len(names), 12), np.float32); hud = []
    for i, (t, a) in enumerate(zip(ts, sp)):
        s = state(t)
        for j, n in enumerate(names): frames[i, j] = poses[n](s, a)[:3, :4].reshape(-1)
        ph, rows = hud_rows(s, t); hud.append(dict(t=round(float(t), 2), ph=ph, rows=rows))
    jumps = []; nxt = {}
    for i, (t0, t1, kind, d) in enumerate(SEG):
        if kind == "spinup": jumps.append(["spin", round(t0, 2)])
        if kind == "move" and (i == 0 or SEG[i - 1][2] != "move"):
            j = i
            while SEG[j][2] == "move": j += 1
            tgt = next((p for p, a in V3.POSES.items() if tuple(a) == tuple(SEG[j][3]["az"])), "pose")
            jumps.append([f"to {tgt}".replace("flight cross", "cross"), round(t0, 2)])
        if kind == "curl" and d["c1"] > 0: jumps.append([f"curl {d['pose']}", round(t0, 2)])
    data = dict(names=names, parts=[parts[n] for n in names], nf=len(ts), fps=FPS, frames=base64.b64encode(frames.tobytes()).decode(), hud=hud,
                title="hand3 hub v3: travelling finger roots", groups=[["drive", "frame + servos"], ["ring", "pull ring + yokes"], ["hub", "hub + cage"], ["roots", "roots + pins + cords"], ["fingers", "fingers"]],
                jumps=[["start", 0]] + jumps, view=dict(wide=[85, 75, 60], close=[34, 26, 92]),
                note="Real CAD solids from hand3e/hand3_hub_v3.py and hub E's drive. Roots are shown sliding round the hub; on the real hand a parked fork holds the root and a roller turns the hub under it (not modelled yet). Spin shown at 1/600 of real rpm. Loads are estimates.")
    os.makedirs("hand3_v3_site", exist_ok=True)
    open("hand3_v3_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    open("hand3_v3_site/index.html", "w").write(open("hand3_v3_sim_template.html").read().replace("<title>Cephalo-2 coupler + hand3 motion sim</title>", "<title>hand3 hub v3 motion sim</title>"))
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("hand3_v3_site/sim_data.js") / 1e6, 2), "MB")
    if check: overlap(parts, poses, ts, sp)


def overlap(parts, poses, ts, sp, step=10):
    """Occupancy-grid overlap over the whole timeline (0.5 mm cells, eroded one cell; walls under ~1.5 mm are not seen, the
    range check in hand3_hub_v3_check covers those)."""
    sys.argv = sys.argv[:1]
    import coupler_sim_overlap as OV
    names = list(parts); G = {n: OV.grid(n, parts[n]) for n in names if not n.startswith("tackle cord")}
    hits = {}
    for fi in sorted(set(list(range(0, len(ts), step)) + [int(t0 * FPS) + 1 for t0, _, k, _ in SEG] + [int(t1 * FPS) - 1 for _, t1, _, _ in SEG])):
        fi = min(fi, len(ts) - 1); s = state(ts[fi]); A = {n: poses[n](s, sp[fi]) for n in G}
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
    json.dump(dict(cell_mm=OV.CELL, overlaps=out), open("hand3_v3_sim_overlap.json", "w"), indent=1)
    print("overlaps:", len(out))
    for h in out[:40]: print(f"  {h['vol_mm3']:8.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main(check=len(sys.argv) > 1 and sys.argv[1] == "check")
