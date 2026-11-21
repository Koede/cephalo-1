"""hand3 hub v3 F3 motion sim (2026-11-20): every finger ROTATES round the hub on its own, all four at once, and curls on its own.
Each root has a Pololu 2359 under it (pinion on the base disc's gear skirt); a static 5-track ring under the hub powers them while
parked (lift stage 1).  Curl: D's per-finger winches, wound by the 8 static drivers (lift stage 2).
Timeline: cross -> spin -> all four roots move at once to 3 + 1 -> curl each on its own -> all move at once to 2 + 2 -> curl ->
regrip: fingers stay half curled (screws self-lock) while the roots walk round to one side -> open -> cross.  Spin drawn at 1/600.
Output: hand3_v3f_site/index.html + sim_data.js;  python3 hand3_v3f_sim.py [check]"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3, hand3_multidigit as MD, hand3_rotation_f3 as F
os.chdir(HERE)
import coupler_hand_sim as SIM

FPS = 15; ST = json.load(open("hand3_indep_fingers_study.json"))["common"]
THR = json.load(open("hand3e/hand3_thrust_cad.json"))["low"]; RPM_HOVER = SIM.RPM_HOVER
M_F, R_COM = ST["finger_g"] * 1e-3, ST["finger_com_r"] * 1e-3; W2 = (12500 * 2 * np.pi / 60) ** 2
NUM = F.numbers(); DEG_S = NUM["root_deg_s"]
CROSS = (0, 90, 180, 270); THUMB = (0, 60, 120, 240); PINCH = (0, 120, 180, 300); SIDE = (0, 60, 120, 180)   # all on drivers
L1, L2 = F.STAGE1, MD.KEY["lift"]
TH_AX = np.degrees(np.arctan2(F.AX[1], F.AX[0]))
BETA0 = TH_AX + 180 - (0.5 - (TH_AX * F.SKIRT["z"] / 360) % 1) * 360 / F.PINION["z"]


def ease(x): x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


SEG = []
def add(dur, kind, **kw):
    t0 = SEG[-1][1] if SEG else 0.0; SEG.append((t0, t0 + dur, kind, kw))


def best_targets(a, b):
    """Keep the cyclic order; pick which target each root takes so the largest move is smallest."""
    best = None
    for sh in range(4):
        tb = [b[(i + sh) % 4] for i in range(4)]
        d = [((y - x + 180) % 360) - 180 for x, y in zip(a, tb)]
        m = max(abs(x) for x in d)
        if best is None or m < best[0]: best = (m, [x + dd for x, dd in zip(a, d)])
    return best


def rotate(a, b, c=(0.0,) * 4, label=""):
    m, tgt = best_targets(a, b)
    add(max(0.6, 1.5 * m / DEG_S), "rotate", az=tuple(a), a0=list(a), a1=tgt, c=list(c), label=label)   # eased: peak speed <= motor speed
    return tuple(tgt)


def lift(l0, l1, cur, c=(0.0,) * 4, dur=0.5):
    add(dur, "lift", az=cur, l0=l0, l1=l1, c=list(c))


def curl(cur, c0, c1, win, pose, dur):
    add(dur, "curl", az=cur, c0=list(c0), c1=list(c1), win=win, pose=pose)


add(1.0, "park", az=CROSS)
add(2.0, "spinup", az=CROSS); add(3.0, "hover", az=CROSS); add(1.5, "spindown", az=CROSS)
cur = CROSS
lift(0, L1, cur); cur = rotate(cur, THUMB, label="3 + 1"); lift(L1, L2, cur, dur=0.6)
c1 = [0.75, 0.55, 0.75, 0.30]
curl(cur, [0] * 4, c1, [(0.0, 0.85), (0.15, 1.0), (0.0, 0.6), (0.3, 0.9)], "3 + 1 thumb", 2.6); add(1.0, "hold", az=cur, c=c1, pose="3 + 1 thumb")
curl(cur, c1, [0] * 4, [(0.0, 0.8), (0.1, 0.9), (0.2, 1.0), (0.0, 0.6)], "3 + 1 thumb", 2.2)
lift(L2, L1, cur, dur=0.6); cur = rotate(cur, PINCH, label="2 + 2"); lift(L1, L2, cur, dur=0.6)
c2 = [0.70, 0.75, 0.45, 0.75]
curl(cur, [0] * 4, c2, [(0.0, 1.0), (0.2, 0.8), (0.0, 0.5), (0.1, 0.7)], "2 + 2 pinch", 2.6); add(1.0, "hold", az=cur, c=c2, pose="2 + 2 pinch")
c3 = [0.30] * 4
curl(cur, c2, c3, [(0.0, 0.7), (0.1, 0.8), (0.0, 0.5), (0.2, 1.0)], "2 + 2 pinch", 1.6)
lift(L2, L1, cur, c=c3, dur=0.6); cur = rotate(cur, SIDE, c=c3, label="regrip: one side, fingers still curled"); lift(L1, L2, cur, c=c3, dur=0.6)
curl(cur, c3, [0] * 4, [(0.0, 0.7), (0.1, 0.8), (0.2, 0.9), (0.3, 1.0)], "one side", 1.6)
lift(L2, L1, cur, dur=0.6); cur = rotate(cur, CROSS, label="cross"); lift(L1, 0, cur)
add(1.0, "park", az=cur)
T_END = SEG[-1][1]


def state(t):
    i = next((j for j, (t0, t1, _, _) in enumerate(SEG) if t < t1), len(SEG) - 1)
    t0, t1, kind, d = SEG[i]
    f = np.clip((t - t0) / (t1 - t0), 0, 1); g = ease(f); az = list(d["az"]); w = 0.0
    if kind == "spinup": w = g
    elif kind == "hover": w = 1.0
    elif kind == "spindown": w = 1 - g
    lf = 0.0; cs = [0.0] * 4; moving = [False] * 4; rot = [False] * 4
    prev = [s for s in SEG[:i] if s[2] == "lift"]
    lf = prev[-1][3]["l1"] if prev else 0.0
    if kind == "lift": lf = d["l0"] + (d["l1"] - d["l0"]) * g; cs = list(d["c"])
    if kind == "rotate":
        az = [a + (b - a) * g for a, b in zip(d["a0"], d["a1"])]; cs = list(d["c"]); rot = [abs(b - a) > 1e-6 and 0 < f < 1 for a, b in zip(d["a0"], d["a1"])]
    if kind == "curl":
        for k in range(4):
            a, b = d["win"][k]; gk = ease((f - a) / max(b - a, 1e-6)); cs[k] = d["c0"][k] + (d["c1"][k] - d["c0"][k]) * gk
            moving[k] = abs(d["c1"][k] - d["c0"][k]) > 1e-6 and a <= f <= b
    elif kind == "hold": cs = list(d["c"])
    cords = [c_ * (V3.K_STROKE["K1"] + V3.K_STROKE["K23"]) for c_ in cs]; pulls = [x / V3.TACKLE for x in cords]
    return dict(kind=kind, f=f, az=az, psi=0.0, lift=lf, cs=cs, w=w, cords=cords, pulls=pulls, moving=moving, rot=rot,
                pose=d.get("pose"), label=d.get("label", ""))


def spins():
    ts = np.arange(0, T_END, 1 / FPS); a = 0.0; out = []
    for t in ts:
        out.append(a); a += state(t)["w"] * RPM_HOVER / 600 * 360 / FPS
    out = np.array(out); tot = out[-1]
    if tot > 0: out = out * (np.round(tot / 360) * 360 / tot)                     # the rotor parks where it started (roots over drivers)
    return ts, out


Tz, Rz = SIM.Tz, SIM.Rz
def Tx(x): A = np.eye(4); A[0, 3] = x; return A
def Txy(x, y): A = np.eye(4); A[0, 3] = x; A[1, 3] = y; return A
def Sz(z0, L): A = np.eye(4); A[2, 2] = L; A[2, 3] = z0; return A


def seg(a, b):
    d = np.asarray(b, float) - np.asarray(a, float); L = max(np.linalg.norm(d), 1e-4); z = d / L
    x = np.cross([0, 1.0, 0], z); x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-9 else np.array([1.0, 0, 0]); y = np.cross(z, x)
    A = np.eye(4); A[:3, 0] = x; A[:3, 1] = y; A[:3, 2] = z * L; A[:3, 3] = a; return A
M = lambda m: np.asarray(m, float)


def build():
    parts, poses = {}, {}
    def add_(name, m, color, group, pose, opacity=1.0, metal=False, tol=0.04):
        parts[name] = dict(SIM.mesh(m, tol), c=color, g=group, o=opacity, metal=metal); poses[name] = pose
    I4 = lambda s, sp: np.eye(4); SP = lambda sp: Rz(sp)
    add_("frame (driver pockets, post holes)", F.frame_f3(), "#e9e9e4", "drive", I4)
    add_("motor base", HC.hardware()["motor_base"], "#555a63", "drive", I4, metal=True)
    add_("8 curl drivers (Pololu 30:1)", MD.drivers(False), "#8a8f98", "drive", lambda s, sp: Tz(s["lift"]), metal=True, tol=0.02)
    add_("lift ring (SG92R, 2 stages)", F.lift_ring_f3(), "#f0a030", "drive", lambda s, sp: Tz(s["lift"]))
    add_("track ring (static, on springs)", F.track_ring(), "#4a4a52", "drive", lambda s, sp: Tz(F.track_dz(s["lift"])))
    add_("5 copper tracks", F.tracks(), "#c87533", "drive", lambda s, sp: Tz(F.track_dz(s["lift"])), metal=True, tol=0.02)
    add_("track ring posts", F.posts(), "#b8bcc4", "drive", lambda s, sp: Tz(F.track_dz(s["lift"])), metal=True, tol=0.02)
    add_("cage", V3.cage_v3(), "#c9cfd6", "hub", lambda s, sp: SP(sp))
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add_("motor bell", bell, "#3a3f47", "hub", lambda s, sp: SP(sp), metal=True)
    add_("base disc + gear skirt z166", F.base_disc_f3(), "#b8bec6", "hub", lambda s, sp: SP(sp), tol=0.01)
    add_("top disc", V3.top_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), opacity=0.55, tol=0.02)
    add_("heel nut", C.parts()["heel"], "#2f2f2f", "hub", lambda s, sp: SP(sp))
    COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; R = C.parts()
    strand = mf.Manifold.cylinder(1.0, V3.V["cord_r"], V3.V["cord_r"], 10)
    bandu = {j: mf.Manifold.cylinder(1.0, V3.B.picks()[j]["cs"] / 2 + 0.2, V3.B.picks()[j]["cs"] / 2 + 0.2, 12) for j in range(3)}
    pin0 = F.pinion().translate([-F.AX[0], -F.AX[1], 0])
    for k in range(4):
        RA = lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k])
        add_(f"root {k} + winch leg (AlSi10Mg)", F.root_f3(0), COL[k], "roots", RA, metal=True, tol=0.02)
        add_(f"rotation motor {k} (Pololu 2359)", F.motor(), "#d4af37", "roots", RA, metal=True, tol=0.02)
        add_(f"pinion {k} (m0.3 z12)", pin0, "#b08d2b", "roots",
             (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Txy(*F.AX) @ Rz(BETA0 + s["az"][k] * F.RG / F.RP)), metal=True, tol=0.005)
        add_(f"wipers {k}", F.wipers(k), "#e0a050", "roots", RA, metal=True, tol=0.01)
        add_(f"lead screw M2 {k}", MD.screw(), "#b8bcc4", "roots", RA, metal=True, tol=0.01)
        add_(f"nut slider {k}", MD.slider(0.0), "#111111", "roots", (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tz(-s["pulls"][k])), tol=0.01)
        for sd in (-1, 1):
            r = MD.SLIDER["eye_x"]; zt = V3.Z_GAP[0] - 1.0
            def cp(s, sp, k=k, sd=sd, r=r, zt=zt):
                zb = MD.SLIDER["z_top0"] - s["pulls"][k] - 1.0
                return SP(sp) @ Rz(s["az"][k]) @ Tx(r) @ M(C.T(t=[0, sd * 0.7, 0])) @ Sz(zb, zt - zb)
            add_(f"tackle cord {k}{'ab'[sd > 0]}", strand, "#f2f2f2", "roots", cp, tol=0.0)
        for j in range(3):
            fp = lambda s, sp, k=k, j=j: SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(C.finger_frames(0, 0.0, tuple(f * s["cs"][k] for f in H.FOLD_MAX))[j + 1])
            add_(f"finger {k} phalanx {j + 1}", V3.B.phalanx_v3(j), "#2b2b30", "fingers", fp)
            def fpx(s, sp, k=k, j=j):
                fr = C.finger_frames(0, 0.0, tuple(f * s["cs"][k] for f in H.FOLD_MAX))
                return SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(fr[0] @ C.T(C.rot("x", H.PITCH)) if j == 0 else fr[j])
            add_(f"finger {k} dowel {j + 1}p", V3.B.dowel_prox(j), "#b8bcc4", "fingers", fpx, metal=True, tol=0.01)
            add_(f"finger {k} dowel {j + 1}d", V3.B.dowel_dist(j), "#b8bcc4", "fingers", fp, metal=True, tol=0.01)
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
    imb = imbalance(s["az"]); ok = imb < 5; rpm = s["w"] * RPM_HOVER; L = s["lift"]
    names = {"park": "parked", "spinup": "rotor spin-up (flight cross only)", "hover": "hover: fingers are the rotor blades", "spindown": "spin-down",
             "lift": "lift ring moving (one SG92R)", "rotate": f"all four roots turn at once, each its own way -> {s['label']}",
             "curl": f"each finger winds on its own ({s['pose']})", "hold": f"hold ({s['pose']})"}
    stage = "0: flight (nothing static touches)" if L < 0.05 else ("1: tracks touch, rotation motors live" if L <= L1 + 0.01 else "2: curl keys in the screws")
    rows = [["root azimuths", " / ".join(f"{a % 360:.0f}°" for a in s["az"])],
            ["lift stage", f"{stage}  ({L:.1f} mm)", L, L2, "#f0a030"]]
    for k in range(4):
        d = ((s["az"][k] - CROSS[k] + 180) % 360) - 180
        rows.append([f"root {k} motor", ("turning" if s["rot"][k] else "idle") + f", {s['az'][k] % 360:.0f}°", (s["az"][k] % 360) / 360, 1.0, ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"][k]])
    for k in range(4):
        rows.append([f"finger {k}: curl / slider", f"{100 * s['cs'][k]:.0f} % / {s['pulls'][k]:.1f} mm" + (" (winding)" if s["moving"][k] else ""), s["cs"][k], 1.0, ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"][k]])
    rows += [["imbalance if spun at 12,500 rpm", f"{imb:.0f} N", imb, 1500, "#e04b39" if not ok else "#39a0e0"],
             ["spin allowed (interlock)", "yes" if (ok and L < 0.01 and max(s["cs"]) < 0.01) else "NO"],
             ["rotor speed", f"{int(rpm):,} rpm", rpm, THR["rpm"], "#39a0e0"],
             ["root drive", f"{NUM['push_N']} N at the pinion (need ~3, est.), {DEG_S}°/s, current-limited to {NUM['current_limit_A']} A"]]
    return names[s["kind"]], rows


def main(check=False):
    parts, poses = build(); ts, sp = spins(); names = list(parts)
    frames = np.zeros((len(ts), len(names), 12), np.float32); hud = []
    for i, (t, a) in enumerate(zip(ts, sp)):
        s = state(t)
        for j, n in enumerate(names): frames[i, j] = poses[n](s, a + s["psi"])[:3, :4].reshape(-1)
        ph, rows = hud_rows(s, t); hud.append(dict(t=round(float(t), 2), ph=ph, rows=rows))
    jumps = []
    for i, (t0, t1, kind, d) in enumerate(SEG):
        if kind == "spinup": jumps.append(["spin", round(t0, 2)])
        if kind == "rotate": jumps.append([f"turn: {d['label']}", round(t0, 2)])
        if kind == "curl" and max(d["c1"]) > max(d["c0"]): jumps.append([f"curl {d['pose']}", round(t0, 2)])
    data = dict(names=names, parts=[parts[n] for n in names], nf=len(ts), fps=FPS, frames=base64.b64encode(frames.tobytes()).decode(), hud=hud,
                title="hand3 hub v3 F3: every finger turns and curls on its own", groups=[["drive", "frame + drivers + tracks"], ["hub", "hub + cage"], ["roots", "roots + motors + winches"], ["fingers", "fingers"]],
                jumps=[["start", 0]] + jumps, view=dict(wide=[85, 75, 60], close=[34, 26, 80]),
                note="Real CAD solids from hand3e/hand3_rotation_f3.py, hand3_multidigit.py and hand3_hub_v3.py. Each finger root has its own Pololu 2359 gearmotor under the hub; its pinion rolls on a gear skirt under the base disc, so all four roots turn round the hub at the same time, each by its own amount. A static track ring under the hub powers them only while parked. Each finger also curls on its own (self-locking winch, wound by the static drivers). Spin shown at 1/600 of real rpm. Loads are estimates.")
    os.makedirs("hand3_v3f_site", exist_ok=True)
    open("hand3_v3f_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    open("hand3_v3f_site/index.html", "w").write(open("hand3_v3_sim_template.html").read().replace("<title>Cephalo-2 coupler + hand3 motion sim</title>", "<title>hand3 hub v3 F3 motion sim</title>"))
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("hand3_v3f_site/sim_data.js") / 1e6, 2), "MB", flush=True)
    if check: overlap(parts, poses, ts, sp)


def overlap(parts, poses, ts, sp, step=8):
    """Occupancy-grid overlap over the whole timeline (0.5 mm cells, eroded one cell).  Pinion x base disc is the gear mesh: skipped."""
    sys.argv = sys.argv[:1]
    import coupler_sim_overlap as OV
    names = list(parts); G = {n: OV.grid(n, parts[n]) for n in names if not n.startswith("tackle cord") and not n.startswith("band ")}
    hits = {}
    for fi in sorted(set(list(range(0, len(ts), step)) + [int(t0 * FPS) + 1 for t0, _, k, _ in SEG] + [int(t1 * FPS) - 1 for _, t1, _, _ in SEG])):
        fi = min(fi, len(ts) - 1); s = state(ts[fi]); A = {n: poses[n](s, sp[fi] + s["psi"]) for n in G}
        bx = {n: OV.world_box(G[n], A[n]) for n in G}; nn = list(G)
        for i, a in enumerate(nn):
            for b in nn[i + 1:]:
                if ("pinion" in a and "base disc" in b) or ("pinion" in b and "base disc" in a): continue
                if ("wipers" in a and "copper" in b) or ("wipers" in b and "copper" in a): continue
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
    json.dump(dict(cell_mm=OV.CELL, overlaps=out, timeline_s=round(T_END, 1)), open("hand3_v3f_sim_overlap.json", "w"), indent=1)
    print("overlaps:", len(out))
    for h in out[:40]: print(f"  {h['vol_mm3']:8.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main(check=len(sys.argv) > 1 and sys.argv[1] == "check")
