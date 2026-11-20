"""hand3 hub v3, decision C motion sim (2026-11-20): every finger root drives itself with its own worm N20 on the crown ring, all
four at once.  Rigid parts from hand3e/hand3_root_drive.py, hand3_hub_v3.py and hub E's drive; each frame stores only poses.
Timeline: parked in the flight cross -> spin-up, hover, spin-down -> dock (carriage in: roller on, rotor turned so the cage pads
face the pogo dock) -> all roots move at once to the 3 + 1 thumb pose -> undock -> curl -> open -> 2 + 2 pinch -> curl -> open ->
all on one side -> back to the cross.  Pinions turn with the travel (crown 74 : pinion 22).  Spin drawn at 1/600 of the real rpm.
Output: hand3_v3c_site/index.html + sim_data.js;  python3 hand3_v3c_sim.py [check]"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3, hand3_root_drive as RD
os.chdir(HERE)
import coupler_hand_sim as SIM

FPS = 20; ST = json.load(open("hand3_indep_fingers_study.json"))["common"]; HUB = json.load(open("hand3e/hand3_hubc.json"))
THR = json.load(open("hand3e/hand3_thrust_cad.json"))["low"]; RPM_HOVER = SIM.RPM_HOVER
M_F, R_COM = ST["finger_g"] * 1e-3, ST["finger_com_r"] * 1e-3; W2 = (12500 * 2 * np.pi / 60) ** 2
CURL_LIM = json.load(open("hand3_root_drive_check.json"))["curl"] if os.path.exists("hand3_root_drive_check.json") else {k: dict(max_clear_curl=0.7) for k in V3.POSES}
F_STALL, F_RATED = HUB["base"]["F_stall_finger"] * 0.8, HUB["base"]["F_rated_finger"] * 0.8      # both servos, 2:1, eyelet 0.8 (est.)
CROSS, THUMB, PINCH, SIDE = V3.POSES["flight cross"], V3.POSES["3 + 1 thumb"], V3.POSES["2 + 2 pinch"], V3.POSES["all four on one side"]
SPEED = np.degrees(RD.WORM["rpm"] / 60 * 2 * np.pi * RD.PIN["r_p"] / 18.5)          # root speed deg/s (crown pitch r 18.5)
GEAR = 18.5 / RD.PIN["r_p"]


def ease(x): x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


# timeline: (t0, t1, kind, data)
SEG = []
def add(dur, kind, **kw):
    t0 = SEG[-1][1] if SEG else 0.0; SEG.append((t0, t0 + dur, kind, kw))
add(1.0, "park", az=CROSS)
add(2.0, "spinup", az=CROSS); add(3.0, "hover", az=CROSS); add(1.5, "spindown", az=CROSS)
MIN_SEP = 2 * V3.V["block_half"] + 2.0


def best_shift(a0, a1):
    """All roots move at once; the whole pattern may turn, so pick the turn that makes the largest single move smallest."""
    a0 = np.array(a0, float); a1 = np.array(a1, float); best = None
    for sh in np.arange(-180, 180, 1.0):
        d = (a1 + sh - a0 + 180) % 360 - 180; m = np.abs(d).max()
        if best is None or m < best[0]: best = (m, d)
    return best


def moves(a, b):
    m, d = best_shift(a, b); a0 = np.array(a, float)
    add(0.8, "dock", az=tuple(a)); add(max(0.8, m / SPEED), "move", az=tuple(a), d=tuple(d)); add(0.5, "undock", az=tuple(a0 + d))
    return tuple(a0 + d)


def curl(cur, pose):
    c = CURL_LIM[pose]["max_clear_curl"]
    add(2.0, "curl", az=cur, c0=0.0, c1=c, pose=pose); add(1.0, "hold", az=cur, c=c, pose=pose); add(1.5, "curl", az=cur, c0=c, c1=0.0, pose=pose)


cur = moves(CROSS, THUMB); curl(cur, "3 + 1 thumb")
cur = moves(cur, PINCH); curl(cur, "2 + 2 pinch")
cur = moves(cur, SIDE); curl(cur, "all four on one side")
cur = moves(cur, CROSS)
add(1.0, "park", az=cur)
T_END = SEG[-1][1]


def _psi_end():
    """Rotor angle (world) at the end of every segment: docking turns it so the cage pads face the dock (psi = STATION_AZ)."""
    psi = 0.0; out = []
    for t0, t1, kind, d in SEG:
        if kind == "dock": psi = psi + ((V3.STATION_AZ - psi) + 180) % 360 - 180
        out.append(psi)
    return out
PSI_END = _psi_end()


def state(t):
    i = next((j for j, (t0, t1, _, _) in enumerate(SEG) if t < t1), len(SEG) - 1)
    t0, t1, kind, d = SEG[i]; psi0 = PSI_END[i - 1] if i > 0 else 0.0; psi1 = PSI_END[i]
    f = np.clip((t - t0) / (t1 - t0), 0, 1); g = ease(f); az = list(d["az"]); c = 0.0; w = 0.0; s = 0.0; mv = [0.0] * 4
    psi = psi0 + (psi1 - psi0) * g
    if kind == "spinup": w = g
    elif kind == "hover": w = 1.0
    elif kind == "spindown": w = 1 - g
    elif kind == "dock": s = V3.ST["roll_gap"] * min(1.0, 2.5 * f)
    elif kind == "move":
        s = V3.ST["roll_gap"]; mv = [dd * g for dd in d["d"]]; az = [a + m for a, m in zip(az, mv)]
    elif kind == "undock": s = V3.ST["roll_gap"] * (1 - g)
    elif kind == "curl": c = d["c0"] + (d["c1"] - d["c0"]) * g
    elif kind == "hold": c = d["c"]
    # pinion angle: total travel of each root so far (rotor frame) x crown/pinion ratio
    trav = [0.0] * 4
    for j in range(i):
        if SEG[j][2] == "move": trav = [x + dd for x, dd in zip(trav, SEG[j][3]["d"])]
    trav = [x + m for x, m in zip(trav, mv)]
    cord = c * (V3.K_STROKE["K1"] + V3.K_STROKE["K23"]); pull = cord / V3.TACKLE
    return dict(kind=kind, f=f, az=az, psi=psi, s=s, c=c, w=w, cord=cord, pull=pull, pose=d.get("pose"), pin=[x * GEAR for x in trav],
                moving=(kind == "move"), docked=(s >= V3.ST["roll_gap"] - 1e-6))


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
    add_("base pull ring + rail (3 mm flange)", V3.ring_v3(), "#e07b39", "ring", lambda s, sp: SP(sp) @ Tz(-s["pull"]))
    add_("cage (key slots only)", V3.cage_v3(), "#c9cfd6", "hub", lambda s, sp: SP(sp))
    SA = Rz(V3.STATION_AZ); STP = RD.station_c()
    cols = dict(tyre="#222222", motor="#9aa1aa", servo="#1f4e8c", sensor="#2e8b57", dock="#d4a017")
    for n, m in STP.items():
        add_(f"station {n}", m, cols[n], "station", (lambda s, sp, n=n: SA @ Tx(-min(s["s"], V3.ST["roll_gap"]) if n in ("tyre", "motor", "dock") else 0.0)), metal=(n == "motor"))
    add_("cage pads (8, copper)", RD.pads(), "#c87533", "hub", lambda s, sp: SP(sp), metal=True, tol=0.01)
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add_("motor bell", bell, "#3a3f47", "hub", lambda s, sp: SP(sp), metal=True)
    add_("base disc", RD.base_disc_c(), "#b8bec6", "hub", lambda s, sp: SP(sp), tol=0.02)
    add_("top disc + crown ring", RD.top_disc_c(), "#b8bec6", "hub", lambda s, sp: SP(sp), opacity=0.8, tol=0.02)
    add_("copper tracks (8)", RD.tracks(), "#c87533", "hub", lambda s, sp: SP(sp), metal=True, tol=0.01)
    add_("heel nut", C.parts()["heel"], "#2f2f2f", "hub", lambda s, sp: SP(sp))
    COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; R = C.parts()
    strand = mf.Manifold.cylinder(1.0, V3.V["cord_r"], V3.V["cord_r"], 10)
    bandu = {j: mf.Manifold.cylinder(1.0, V3.B.picks()[j]["cs"] / 2 + 0.2, V3.B.picks()[j]["cs"] / 2 + 0.2, 12) for j in range(3)}
    for k in range(4):
        RA = lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k])
        add_(f"root {k} (AlSi10Mg)", RD.root_c(k), COL[k], "roots", RA, metal=True, tol=0.02)
        add_(f"worm N20 {k}", RD.gearbox_and_can(), "#8a8f98", "roots", RA, metal=True, tol=0.02)
        add_(f"pinion {k}", RD.pinion(), "#c9a227", "roots",
             (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ M(C.T(t=[0, 0, RD.Z_SHAFT])) @ M(C.T(C.rot("x", s["pin"][k]))) @ M(C.T(t=[0, 0, -RD.Z_SHAFT]))), metal=True, tol=0.01)
        add_(f"bead {k}", V3.bead(0, 0.0), "#111111", "roots", (lambda s, sp, k=k: SP(sp) @ Rz(s["az"][k]) @ Tz(-s["pull"])), tol=0.01)
        for sd in (-1, 1):
            r = V3.V["bead"]["eye_r"]; zt = V3.Z_GAP[0] - 1.0
            def cp(s, sp, k=k, sd=sd, r=r, zt=zt):
                zb = V3.Z_RING_TOP - s["pull"] + V3.V["bead"]["dz"][1] - 0.3
                return SP(sp) @ Rz(s["az"][k]) @ Tx(r) @ M(C.T(t=[0, sd * 0.7, 0])) @ Sz(zb, zt - zb)
            add_(f"tackle cord {k}{'ab'[sd > 0]}", strand, "#f2f2f2", "roots", cp, tol=0.0)
        for j in range(3):
            fp = lambda s, sp, k=k, j=j: SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(C.finger_frames(0, 0.0, tuple(f * s["c"] for f in H.FOLD_MAX))[j + 1])
            add_(f"finger {k} phalanx {j + 1}", V3.B.phalanx_v3(j), "#2b2b30", "fingers", fp)
            # return band j: dowels ride their members; the band is drawn as 5 straight pieces along its centreline (wrap included)
            def fpx(s, sp, k=k, j=j):
                fr = C.finger_frames(0, 0.0, tuple(f * s["c"] for f in H.FOLD_MAX))
                return SP(sp) @ Rz(s["az"][k] - C.FAZ[0]) @ M(fr[0] @ C.T(C.rot("x", H.PITCH)) if j == 0 else fr[j])
            add_(f"finger {k} dowel {j + 1}p", V3.B.dowel_prox(j), "#b8bcc4", "fingers", fpx, metal=True, tol=0.01)
            add_(f"finger {k} dowel {j + 1}d", V3.B.dowel_dist(j), "#b8bcc4", "fingers", fp, metal=True, tol=0.01)
            cs = V3.B.picks()[j]["cs"]
            for q in range(5):
                def bp(s, sp, k=k, j=j, q=q):
                    pts = V3.B.band_points(j, H.FOLD_MAX[j] * s["c"]); a, b = pts[q], pts[q + 1]
                    return fpx(s, sp, k, j) @ seg(a, b)
                add_(f"band {k}.{j + 1}.{q}", bandu[j], "#3fae5a", "fingers", bp, tol=0.0)
            add_(f"finger {k} pad {j + 1}", R[f"pad{j + 1}"], "#c23b22", "fingers", fp)
    return parts, poses


def imbalance(az):
    v = sum(np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]) for a in az); return M_F * W2 * R_COM * np.linalg.norm(v)


def hud_rows(s, t):
    imb = imbalance(s["az"]); ok = imb < 5
    rpm = s["w"] * RPM_HOVER
    names = {"park": "parked", "spinup": "rotor spin-up", "hover": "hover: fingers are the rotor blades", "spindown": "spin-down",
             "dock": "dock: roller turns the rotor so the cage pads meet the pogo dock", "move": "all four roots drive themselves at once",
             "undock": "dock and roller back off", "curl": f"curl ({s['pose']})", "hold": f"hold ({s['pose']})"}
    ph = names[s["kind"]]
    mot = "running (each its own speed)" if s["moving"] else ("powered, holding" if s["docked"] else "unpowered: worms self-lock")
    rows = [["root azimuths (rotor frame)", " / ".join(f"{a % 360:.0f}°" for a in s["az"])],
            ["4 worm N20s", mot],
            ["dock (8 pogo pins)", "contact" if s["docked"] else "open"],
            ["station carriage", f"{s['s']:.1f} / {V3.ST['roll_gap']:.1f} mm", s["s"], V3.ST["roll_gap"], "#f0a030"],
            ["root speed", f"{SPEED:.0f}°/s (pinion z22 on a 74-tooth crown)"],
            ["imbalance if spun at 12,500 rpm", f"{imb:.0f} N", imb, 1500, "#e04b39" if not ok else "#39a0e0"],
            ["spin allowed (interlock)", "yes" if (ok and not s["docked"]) else ("NO: docked" if s["docked"] else "NO: roots not balanced")],
            ["rotor speed", f"{int(rpm):,} rpm", rpm, THR["rpm"], "#39a0e0"],
            ["spin pull per root (motor 625 + root 644 + finger 826, est.)", "2,095 N" if rpm > 1000 else "-"],
            ["curl (all knuckles together)", f"{100 * s['c']:.0f} % of full fold"],
            ["ring stroke (2:1 tackle)", f"{s['pull']:.2f} / {V3.RING_STROKE:.1f} mm", s["pull"], V3.RING_STROKE, "#e07b39"],
            ["both STS3215 servos", "pulling" if s["kind"] == "curl" and s["c"] > 0 and s["f"] < 1 else ("holding" if s["c"] > 0.01 else "idle")]]
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
        if kind == "move":
            nxt_pose = next((SEG[j][3].get("pose") for j in range(i, len(SEG)) if SEG[j][2] == "curl"), None) or "cross"
            jumps.append([f"to {nxt_pose}".replace("flight cross", "cross"), round(SEG[i - 1][0], 2)])
        if kind == "curl" and d["c1"] > 0: jumps.append([f"curl {d['pose']}", round(t0, 2)])
    data = dict(names=names, parts=[parts[n] for n in names], nf=len(ts), fps=FPS, frames=base64.b64encode(frames.tobytes()).decode(), hud=hud,
                title="hand3 hub v3 C: roots that drive themselves", groups=[["drive", "frame + servos"], ["station", "station + dock"], ["ring", "pull ring + yokes"], ["hub", "hub + cage"], ["roots", "roots + worm motors"], ["fingers", "fingers"]],
                jumps=[["start", 0]] + jumps, view=dict(wide=[85, 75, 60], close=[34, 26, 92]),
                note="Real CAD solids from hand3e/hand3_root_drive.py, hand3_hub_v3.py and hub E's drive. Each root carries a self-locking worm N20 whose brass pinion walks a crown ring on the top disc, so all four roots move at once. Power only while parked: the station carriage brings the roller onto the cage and two pogo blocks onto 8 copper pads; tracks under the top disc feed each root's brushes. Spin shown at 1/600 of real rpm. Loads are estimates.")
    os.makedirs("hand3_v3c_site", exist_ok=True)
    open("hand3_v3c_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    open("hand3_v3c_site/index.html", "w").write(open("hand3_v3_sim_template.html").read().replace("<title>Cephalo-2 coupler + hand3 motion sim</title>", "<title>hand3 hub v3 C motion sim</title>"))
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("hand3_v3c_site/sim_data.js") / 1e6, 2), "MB")
    if check: overlap(parts, poses, ts, sp)


def overlap(parts, poses, ts, sp, step=10):
    """Occupancy-grid overlap over the whole timeline (0.5 mm cells, eroded one cell; walls under ~1.5 mm are not seen, the
    range check in hand3_hub_v3_check covers those)."""
    sys.argv = sys.argv[:1]
    import coupler_sim_overlap as OV
    names = list(parts); G = {n: OV.grid(n, parts[n]) for n in names if not n.startswith("tackle cord") and not n.startswith("band ")}
    SKIP = lambda a, b: ("pinion" in a and "top disc" in b) or ("pinion" in b and "top disc" in a)
    hits = {}
    for fi in sorted(set(list(range(0, len(ts), step)) + [int(t0 * FPS) + 1 for t0, _, k, _ in SEG] + [int(t1 * FPS) - 1 for _, t1, _, _ in SEG])):
        fi = min(fi, len(ts) - 1); s = state(ts[fi]); A = {n: poses[n](s, sp[fi] + s["psi"]) for n in G}
        bx = {n: OV.world_box(G[n], A[n]) for n in G}; nn = list(G)
        for i, a in enumerate(nn):
            for b in nn[i + 1:]:
                la, ha = bx[a]; lb, hb = bx[b]
                if SKIP(a, b) or np.any(la > hb) or np.any(lb > ha): continue
                Mab = np.linalg.inv(A[b]) @ A[a]; p = G[a]["pts"] @ Mab[:3, :3].T + Mab[:3, 3]
                idx = np.floor((p - G[b]["lo"]) / OV.CELL).astype(int); okm = np.all((idx >= 0) & (idx < G[b]["n"]), 1)
                if not okm.any(): continue
                ii = idx[okm]; n_in = int(G[b]["er"][ii[:, 0], ii[:, 1], ii[:, 2]].sum())
                if n_in:
                    key = f"{a} x {b}"; v = n_in * OV.CELL ** 3
                    if key not in hits or v > hits[key]["vol_mm3"]: hits[key] = dict(a=a, b=b, t=round(float(ts[fi]), 2), vol_mm3=round(v, 2))
    out = sorted(hits.values(), key=lambda h: -h["vol_mm3"])
    json.dump(dict(cell_mm=OV.CELL, overlaps=out), open("hand3_v3c_sim_overlap.json", "w"), indent=1)
    print("overlaps:", len(out))
    for h in out[:40]: print(f"  {h['vol_mm3']:8.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main(check=len(sys.argv) > 1 and sys.argv[1] == "check")
