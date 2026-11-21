"""hand3 hub v3 S3 motion sim (2026-11-20): one cam sleeve turns all four finger roots at once, each along its own path, between
four grip layouts (2 + 2, cross, 3 + 1, one side); each finger still curls on its own (D's winches, 8 static drivers).
The sleeve is moved by a self-locking M2 screw at az 30, turned by a 9th static driver at lift stage 1; curl keys go in at stage 2.
Timeline: cross -> spin -> to 3 + 1 -> curl each on its own -> to 2 + 2 (through cross) -> curl -> regrip: fingers stay 30 % curled
while the sleeve walks the roots to one side -> open -> cross.  Spin drawn at 1/600.
Output: hand3_v3s_site/index.html + sim_data.js;  python3 hand3_v3s_sim.py [check]"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3, hand3_multidigit as MD, hand3_cam_s3 as S
os.chdir(HERE)
import coupler_hand_sim as SIM

FPS = 15; ST = json.load(open("hand3_indep_fingers_study.json"))["common"]
THR = json.load(open("hand3e/hand3_thrust_cad.json"))["low"]; RPM_HOVER = SIM.RPM_HOVER
M_F, R_COM = ST["finger_g"] * 1e-3, ST["finger_com_r"] * 1e-3; W2 = (12500 * 2 * np.pi / 60) ** 2
NUM = S.numbers(); MM_S = NUM["sleeve_mm_s"]
NAMES = [l[0] for l in S.LAYOUTS]; U_OF = dict(zip(NAMES, S.PRE_U))
L1, L2 = S.STAGE1, MD.KEY["lift"]


def ease(x): x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


SEG = []
def add(dur, kind, **kw):
    t0 = SEG[-1][1] if SEG else 0.0; SEG.append((t0, t0 + dur, kind, kw))


def rotate(u0, name, c=(0.0,) * 4, label=""):
    """Sleeve moves from slot coordinate u0 to layout `name`; every root follows its own slot."""
    u1 = U_OF[name]
    add(max(0.5, 1.5 * abs(u1 - u0) / MM_S), "rotate", az=tuple(S.theta_at(u0)), u0=u0, u1=u1, c=list(c), label=label or name)
    return u1


def lift(l0, l1, u, c=(0.0,) * 4, dur=0.5):
    add(dur, "lift", az=tuple(S.theta_at(u)), u=u, l0=l0, l1=l1, c=list(c))


def curl(cur, c0, c1, win, pose, dur):
    add(dur, "curl", az=cur, c0=list(c0), c1=list(c1), win=win, pose=pose)


u = U_OF["cross"]; CROSS = tuple(S.theta_at(u))
add(1.0, "park", az=CROSS, u=u)
add(2.0, "spinup", az=CROSS, u=u); add(3.0, "hover", az=CROSS, u=u); add(1.5, "spindown", az=CROSS, u=u)
lift(0, L1, u); u = rotate(u, "3 + 1"); lift(L1, L2, u, dur=0.6)
cur = tuple(S.theta_at(u))
c1 = [0.75, 0.55, 0.75, 0.30]
curl(cur, [0] * 4, c1, [(0.0, 0.85), (0.15, 1.0), (0.0, 0.6), (0.3, 0.9)], "3 + 1 thumb", 2.6); add(1.0, "hold", az=cur, c=c1, pose="3 + 1 thumb")
curl(cur, c1, [0] * 4, [(0.0, 0.8), (0.1, 0.9), (0.2, 1.0), (0.0, 0.6)], "3 + 1 thumb", 2.2)
lift(L2, L1, u, dur=0.6); u = rotate(u, "2 + 2", label="2 + 2 (through cross)"); lift(L1, L2, u, dur=0.6)
cur = tuple(S.theta_at(u))
c2 = [0.70, 0.75, 0.45, 0.75]
curl(cur, [0] * 4, c2, [(0.0, 1.0), (0.2, 0.8), (0.0, 0.5), (0.1, 0.7)], "2 + 2 pinch", 2.6); add(1.0, "hold", az=cur, c=c2, pose="2 + 2 pinch")
c3 = [0.30] * 4
curl(cur, c2, c3, [(0.0, 0.7), (0.1, 0.8), (0.0, 0.5), (0.2, 1.0)], "2 + 2 pinch", 1.6)
lift(L2, L1, u, c=c3, dur=0.6); u = rotate(u, "one side", c=c3, label="regrip: one side, fingers still curled"); lift(L1, L2, u, c=c3, dur=0.6)
cur = tuple(S.theta_at(u))
curl(cur, c3, [0] * 4, [(0.0, 0.7), (0.1, 0.8), (0.2, 0.9), (0.3, 1.0)], "one side", 1.6)
lift(L2, L1, u, dur=0.6); u = rotate(u, "cross"); lift(L1, 0, u)
add(1.0, "park", az=tuple(S.theta_at(u)), u=u)
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
    u = d.get("u", None)
    if kind == "rotate":
        u = d["u0"] + (d["u1"] - d["u0"]) * g; az = S.theta_at(u); cs = list(d["c"])
        a0, a1 = S.theta_at(d["u0"]), S.theta_at(d["u1"]); rot = [abs(b - a) > 1e-6 and 0 < f < 1 for a, b in zip(a0, a1)]
    if u is None:
        prevu = [s_[3].get("u", s_[3].get("u1")) for s_ in SEG[:i] if ("u" in s_[3] or "u1" in s_[3])]
        u = prevu[-1] if prevu else S.PRE_U[1]
    if kind == "curl":
        for k in range(4):
            a, b = d["win"][k]; gk = ease((f - a) / max(b - a, 1e-6)); cs[k] = d["c0"][k] + (d["c1"][k] - d["c0"][k]) * gk
            moving[k] = abs(d["c1"][k] - d["c0"][k]) > 1e-6 and a <= f <= b
    elif kind == "hold": cs = list(d["c"])
    cords = [c_ * (V3.K_STROKE["K1"] + V3.K_STROKE["K23"]) for c_ in cs]; pulls = [x / V3.TACKLE for x in cords]
    return dict(kind=kind, f=f, az=az, psi=0.0, lift=lf, cs=cs, w=w, cords=cords, pulls=pulls, moving=moving, rot=rot, u=u,
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
    add_("frame (9 driver pockets)", S.frame_s3(), "#e9e9e4", "drive", I4)
    add_("motor base", HC.hardware()["motor_base"], "#555a63", "drive", I4, metal=True)
    add_("8 curl drivers (Pololu 30:1)", MD.drivers(False), "#8a8f98", "drive", lambda s, sp: Tz(s["lift"]), metal=True, tol=0.02)
    add_("sleeve driver (Pololu 30:1, az 30)", S.driver9(0.0), "#6d7480", "drive", lambda s, sp: Tz(S.driver9_dz(s["lift"])), metal=True, tol=0.02)
    add_("lift ring (SG92R, 2 stages)", S.lift_ring_s3(), "#f0a030", "drive", lambda s, sp: Tz(s["lift"]))
    add_("cage", V3.cage_v3(), "#c9cfd6", "hub", lambda s, sp: SP(sp))
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add_("motor bell", bell, "#3a3f47", "hub", lambda s, sp: SP(sp), metal=True)
    add_("base disc", S.base_disc_s3(), "#b8bec6", "hub", lambda s, sp: SP(sp), tol=0.01)
    add_("brass counterweight", S.counterweight(), "#c9a227", "hub", lambda s, sp: SP(sp), metal=True, tol=0.01)
    SLZ = lambda s, sp: SP(sp) @ Tz(S.sleeve_dz(s["u"]))
    add_("cam sleeve", S.sleeve(), "#2e9e6a", "hub", SLZ, tol=0.01)
    add_("sleeve screw bracket (AlSi10Mg)", S.bracket(), "#8a6f4e", "hub", lambda s, sp: SP(sp), metal=True, tol=0.01)
    add_("sleeve lead screw M2 + coupler", S.screw9(), "#b8bcc4", "hub", lambda s, sp: SP(sp), metal=True, tol=0.01)
    add_("sleeve nut M2", S.nut9(), "#c9a227", "hub", SLZ, metal=True, tol=0.01)
    add_("top disc", V3.top_disc(), "#b8bec6", "hub", lambda s, sp: SP(sp), opacity=0.55, tol=0.02)
    add_("heel nut", C.parts()["heel"], "#2f2f2f", "hub", lambda s, sp: SP(sp))
    COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; R = C.parts()
    strand = mf.Manifold.cylinder(1.0, V3.V["cord_r"], V3.V["cord_r"], 10)
    bandu = {j: mf.Manifold.cylinder(1.0, V3.B.picks()[j]["cs"] / 2 + 0.2, V3.B.picks()[j]["cs"] / 2 + 0.2, 12) for j in range(3)}
    for k in range(4):
        RA = lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k])
        add_(f"root {k} + winch leg (AlSi10Mg)", S.root_s3(0), COL[k], "roots", RA, metal=True, tol=0.02)
        add_(f"cam pin {k}", S.pin(), "#b8bcc4", "roots", RA, metal=True, tol=0.005)
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
             "lift": "lift ring moving (one SG92R)", "rotate": f"cam sleeve rises / drops: all four roots turn at once, each its own way -> {s['label']}",
             "curl": f"each finger winds on its own ({s['pose']})", "hold": f"hold ({s['pose']})"}
    stage = "0: flight (nothing static touches)" if L < 0.05 else ("1: sleeve key in, roots free to turn" if L <= L1 + 0.01 else "2: curl keys in the screws")
    lay = next((n for n, uu in U_OF.items() if abs(uu - s["u"]) < 0.05), "between layouts")
    rows = [["root azimuths", " / ".join(f"{a % 360:.0f}°" for a in s["az"])],
            ["layout", lay],
            ["cam sleeve rise", f"{S.sleeve_dz(s['u']):.1f} / {S.STROKE:.1f} mm", S.sleeve_dz(s["u"]), S.STROKE, "#2e9e6a"],
            ["lift stage", f"{stage}  ({L:.1f} mm)", L, L2, "#f0a030"]]
    for k in range(4):
        rows.append([f"finger {k}: curl / slider", f"{100 * s['cs'][k]:.0f} % / {s['pulls'][k]:.1f} mm" + (" (winding)" if s["moving"][k] else ""), s["cs"][k], 1.0, ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"][k]])
    rows += [["imbalance if spun at 12,500 rpm", f"{imb:.0f} N", imb, 1500, "#e04b39" if not ok else "#39a0e0"],
             ["spin allowed (interlock)", "yes" if (ok and L < 0.01 and max(s["cs"]) < 0.01) else "NO"],
             ["rotor speed", f"{int(rpm):,} rpm", rpm, THR["rpm"], "#39a0e0"],
             ["root drive", f"one screw: {NUM['axial_total_N']:.0f} N on the sleeve, torque margin {NUM['torque_margin']}x; at a layout the slot walls hold the roots"]]
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
                title="hand3 hub v3 S3: one cam sleeve turns all four fingers; each curls on its own", groups=[["drive", "frame + drivers"], ["hub", "hub + cage + cam sleeve"], ["roots", "roots + winches"], ["fingers", "fingers"]],
                jumps=[["start", 0]] + jumps, view=dict(wide=[85, 75, 60], close=[34, 26, 80]),
                note="Real CAD solids from hand3e/hand3_cam_s3.py, hand3_multidigit.py and hand3_hub_v3.py. A cam sleeve on the cage has one slot per finger root; a self-locking screw at az 30 lifts or lowers it, and every root follows its own slot, so all four turn at once between 2 + 2, cross, 3 + 1 and one side. At each layout the slot is straight (axial), so the roots are locked. Each finger also curls on its own (self-locking winch, wound by the static drivers). Spin shown at 1/600 of real rpm. Loads are estimates.")
    os.makedirs("hand3_v3s_site", exist_ok=True)
    open("hand3_v3s_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    open("hand3_v3s_site/index.html", "w").write(open("hand3_v3_sim_template.html").read().replace("<title>Cephalo-2 coupler + hand3 motion sim</title>", "<title>hand3 hub v3 S3 motion sim</title>"))
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("hand3_v3s_site/sim_data.js") / 1e6, 2), "MB", flush=True)
    if check: overlap(parts, poses, ts, sp)


def overlap(parts, poses, ts, sp, step=8):
    """Occupancy-grid overlap over the whole timeline (0.5 mm cells, eroded one cell)."""
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
    json.dump(dict(cell_mm=OV.CELL, overlaps=out, timeline_s=round(T_END, 1)), open("hand3_v3s_sim_overlap.json", "w"), indent=1)
    print("overlaps:", len(out))
    for h in out[:40]: print(f"  {h['vol_mm3']:8.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main(check=len(sys.argv) > 1 and sys.argv[1] == "check")
