"""Cephalo-2 coupler rev B.6 + hand3 (hub E) motion simulation, as an interactive three.js page (2026-11-19).

Every rigid part is exported once from the CAD (handmount_b.py, hand3e/hand3_hubc.py, hand3e/hand3_cad.py) and each frame
stores only its pose, computed with the same transforms as hand3_hubc.assembly(): rotor spin, finger lag (spread) and the
three knuckle folds (curl), the two pull rings, the collar turn and the hand's push-in.  Cords, TPU links and trim screws
change shape with the pull and are left out.  hand3 sits on the hand base turned end for end about y (a rotation).

Timeline (s): 0-3 approach, 3-5 push in (bullets engage), 5-7 collar 60 deg to locked, 8-10 spin-up, 10-14 hover spin,
14-15.5 spin-down, 15.5-18 spread, 18-21 curl, 21-23 hold, 23-26 open, 26-28 unlock, 28-30 pull out.
Loads per frame (est., from hand3_thrust_cad.json / hand3_hubc.json): rpm, motor phase current per bullet, rotor thrust
through the coupler, ring strokes, finger pull.  The picture spins at 1/600 of the real rpm so the fins can be seen.
Output: coupler_sim_site/index.html + sim_data.js"""
import os, sys, json, base64
import numpy as np
import manifold3d as mf
import handmount_b as HB
import hubE_leads as HL

HERE = os.path.dirname(os.path.abspath(__file__)); H3E = os.path.join(HERE, "hand3e")
sys.path.insert(0, H3E); _cwd = os.getcwd(); os.chdir(H3E)
import hand3_hubc as HC
import hand3_cad as C
import hand3_design as H
os.chdir(_cwd)

P = HB.P; FPS = 20; T_END = 30.0
THR = json.load(open(os.path.join(H3E, "hand3_thrust_cad.json")))["low"]
HUB = json.load(open(os.path.join(H3E, "hand3_hubc.json")))
I_HOVER = 36.0; T_HOVER = THR["T_max"] * (I_HOVER / THR["I_A"]) ** (2 / 3); RPM_HOVER = THR["rpm"] * np.sqrt(T_HOVER / THR["T_max"])


CURL_EMPTY = 0.70            # fraction of full curl an empty, fully spread hand reaches before opposite fingertips meet (tips cross from 0.72)


def ease(x):
    x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


def seg(t, a, b):
    return ease((t - a) / (b - a))


def state(t):
    dz = -40 + 30 * seg(t, 0, 3) + 10 * seg(t, 3, 5) - 10 * seg(t, 28, 29) - 30 * seg(t, 29, 30)
    th = -60 + 60 * seg(t, 5, 7) - 60 * seg(t, 26, 28)
    w = seg(t, 8, 10) * (1 - seg(t, 14, 15.5))                       # throttle fraction of hover rpm
    spread = H.SPREAD_MAX * (seg(t, 15.5, 18) - seg(t, 24.5, 26))
    curl = CURL_EMPTY * (seg(t, 18, 21) - seg(t, 23, 24.5))      # empty hand: the fingers stop where their tips meet
    fold = tuple(f * curl for f in H.FOLD_MAX)
    lag = tuple(s * spread for s in H.LAG_SIGN)
    pull_b = HC.E_LAG * np.radians(spread) + HC.LEVER[0] * np.radians(fold[0])
    pull_c = HC.LEVER[1] * np.radians(fold[1]) + HC.LEVER[2] * np.radians(fold[2])
    return dict(dz=dz, th=th, w=w, spread=spread, fold=fold, lag=lag, pull_b=pull_b, pull_c=pull_c)


def spin_angles():
    """Accumulated display spin (deg) per frame: real rpm / 600."""
    ts = np.arange(0, T_END, 1 / FPS); a = 0.0; out = []
    for t in ts:
        out.append(a); a += state(t)["w"] * RPM_HOVER / 600 * 360 / FPS
    return ts, np.array(out)


def M(m):
    return np.asarray(m, float)


def Tz(z):
    A = np.eye(4); A[2, 3] = z; return A


def Rz(deg):
    a = np.radians(deg); A = np.eye(4); A[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]; return A


ROT_Y180 = np.diag([-1.0, 1.0, -1.0, 1.0])                  # hand3 -> connector: end for end about y


def hand_to_conn(dz):
    return HL.hand3_to_conn_matrix(dz)                       # rev B.7: also clocked PSI about z (lead exit in a lug gap)


def ball_pose(th):
    """Detent ball: seated in a dimple at the two stops (theta 0 / -twist), pushed 0.8 back into its bore in between."""
    d = min(abs(th), abs(th + P["twist"])); back = 0.8 * min(d / 6.0, 1.0); a = np.radians(P["det_az"])
    A = np.eye(4); A[0, 3] = -back * np.cos(a); A[1, 3] = -back * np.sin(a); return A


def mesh(m, tol=0.04):
    try:
        m = m.simplify(tol)
    except Exception:
        pass
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3].astype(np.float32); t = np.asarray(me.tri_verts).astype(np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), t=base64.b64encode(t.tobytes()).decode(), nt=int(len(t)))


def capsules(pts, r):
    sp = [mf.Manifold.sphere(r, 14).translate([float(a) for a in p]) for p in pts]
    return mf.Manifold.batch_boolean([mf.Manifold.batch_hull([sp[i], sp[i + 1]]) for i in range(len(sp) - 1)], mf.OpType.Add)


def leads():
    """ESC leads on the arm side (the hand side comes from hubE_leads)."""
    rw = P["wire_od"] / 2; arm = []
    for az in P["az_bullet"]:
        u = np.array([np.cos(np.radians(az)), np.sin(np.radians(az))]); top = P["fem_recess"] + P["bul_len"]
        arm.append(capsules([(*(u * P["r_bullet"]), top + rw + 0.3), (*(u * P["r_bullet"]), top + 6.0), (*(u * 9.0), top + 14.0)], rw))
    return mf.Manifold.batch_boolean(arm, mf.OpType.Add)


def build():
    U = lambda L: mf.Manifold.batch_boolean(L, mf.OpType.Add)
    parts, poses = {}, {}          # name -> mesh dict + style; name -> function(frame state, spin) -> 4x4
    def add(name, m, color, group, pose, opacity=1.0, metal=False):
        parts[name] = dict(mesh(m), c=color, g=group, o=opacity, metal=metal); poses[name] = pose
    I4 = lambda s, sp: np.eye(4)
    # arm side
    tip = HB.cyl(20.0, P["z_back"], 46.0, 64) - HB.cyl(18.0, P["z_back"] + 2.5, 47, 64) - HB.cyl(P["pinboard_r"], P["z_back"] - 1, 47, 48)
    for az in P["az_bullet"]:
        tip -= HB.cyl(3.0, P["z_back"] - 1, P["z_back"] + 3.0, 24).translate([*HB.at(P["r_bullet"], az), 0])
    for r_, az in P["screws"]:                                   # insert bosses and bores for the 3 socket screws
        tip += HB.cyl(3.2, P["z_back"], P["z_back"] + 8.0, 24).translate([*HB.at(r_, az), 0]) ^ HB.cyl(20.0, P["z_back"], 46.0, 64)
        tip -= HB.cyl(1.6, P["z_back"] - 1, P["z_back"] + 9.0, 20).translate([*HB.at(r_, az), 0])
    al = leads()
    add("arm tip bay (sketch, ESC inside)", tip, "#6b7079", "arm", I4, 0.35)
    add("socket body", HB.arm_socket(), "#2b2d31", "arm", I4)
    add("nose ring", HB.nose_ring(), "#3d4046", "arm", I4)
    add("female bullets (Castle 4 mm)", U(HB.female_bullets()), "#d4a72c", "arm", I4, metal=True)
    add("ESC phase leads 14 AWG", al, "#b0302a", "arm", I4)
    add("3 x M3x20 DIN 7984", U(HB.screws()), "#b8bcc4", "arm", I4, metal=True)
    add("detent ball", HB.detent_ball(), "#e8e8f0", "arm", lambda s, sp: ball_pose(s["th"]), metal=True)
    add("O-ring 21 x 1.0", HB.oring(), "#1a7f37", "arm", I4)
    add("locking collar", HB.collar(0), "#d2542d", "collar", lambda s, sp: Rz(s["th"]))
    # hand side: coupler hand base + bullets + leads, moving with the push-in
    HP = lambda s, sp: Tz(s["dz"])
    add("hand base", HB.hand_base(), "#30343a", "hand", HP)
    add("male bullets", U(HB.male_bullets()), "#d4a72c", "hand", HP, metal=True)
    LM = HL.lead_meshes("conn")
    add("motor phase leads 14 AWG", U([LM[f"lead {k}"] for k in "BAC"]), "#b0302a", "hand", HP)
    add("solder joints (heat shrink)", U([LM[f"joint {k}"] for k in "BAC"]), "#111111", "hand", HP)
    add("motor stubs 18 AWG", U([LM[f"stub {k}"] for k in "BAC"]), "#d2452d", "hand", HP)
    add("signal wires (5 x 26 AWG)", LM["sig"], "#e6c229", "hand", HP)
    # hand3 hub E, posed like hand3_hubc.assembly()
    R = C.parts(); HPp = HC._parts(); FAZ = C.FAZ
    G = lambda s: hand_to_conn(s["dz"])
    S = lambda sp: M(C.T(C.rot("z", sp)))
    add("hub E frame", HPp["frame"][0], "#e9e9e4", "hand3", lambda s, sp: G(s))
    add("lead cover rail", HL.rail(), "#f08a24", "hand3", lambda s, sp: G(s), 0.55)
    add("rail screws 2 x M2x6", U(HL.rail_screws()), "#b8bcc4", "hand3", lambda s, sp: G(s), metal=True)
    add("hub E tie ring", HPp["tie_ring"][0], "#cfcfc8", "hand3", lambda s, sp: G(s))
    for w, sd in HC.YOKES:
        add(f"yoke {w} {sd}", HPp[f"yoke_{w}_{sd}"][0], "#a9b3bd", "hand3",
            (lambda w: lambda s, sp: G(s) @ Tz(-((s["pull_b"] if w == "out" else s["pull_c"]) + HC.GAP) if (s["pull_b"] if w == "out" else s["pull_c"]) > 0 else 0.0))(w))
    for n, m in HC.hardware().items():
        col = "#1f4e8c" if "servo" in n else ("#555a63" if "motor" in n else "#9aa1aa")
        add("hw " + n, m, col, "hand3", lambda s, sp: G(s), metal=("rod" in n or "idler" in n))
    add("cage", HPp["cage"][0], "#c9cfd6", "rotor", lambda s, sp: G(s) @ S(sp))
    add("pull ring (base: spread + K1)", HPp["ring_out"][0], "#e07b39", "rotor", lambda s, sp: G(s) @ S(sp) @ Tz(-s["pull_b"]))
    add("pull ring (curl: K2 + K3)", HPp["ring_in"][0], "#39a0e0", "rotor", lambda s, sp: G(s) @ S(sp) @ Tz(-s["pull_c"]))
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
    add("motor bell", bell, "#3a3f47", "rotor", lambda s, sp: G(s) @ S(sp), metal=True)
    for n in ("hub_lower", "hub_upper", "heel"):
        add(n.replace("_", " "), R[n], "#b8bec6" if n != "heel" else "#2f2f2f", "rotor", (lambda: lambda s, sp: G(s) @ S(sp))(), metal=(n != "heel"))
    for k in range(4):
        fr = lambda s, k=k: [M(F) for F in C.finger_frames(k, s["lag"][k], s["fold"])]
        add(f"root grip {k}", R["root_grip"], "#b8bec6", "rotor", (lambda k: lambda s, sp: G(s) @ S(sp) @ fr(s, k)[0])(k), metal=True)
        add(f"lag tube {k}", HC.lag_tube2(k), "#d0d0d0", "rotor", (lambda k: lambda s, sp: G(s) @ S(sp) @ M(C.T(t=[0, 0, C.ZB])) @ M(C.T(C.rot("z", FAZ[k]))))(k), metal=True)
        for j in range(3):
            add(f"finger {k} phalanx {j + 1}", R[f"phalanx{j + 1}"], "#2b2b30", "rotor", (lambda k, j: lambda s, sp: G(s) @ S(sp) @ fr(s, k)[j + 1])(k, j))
            add(f"finger {k} pad {j + 1}", R[f"pad{j + 1}"], "#c23b22", "rotor", (lambda k, j: lambda s, sp: G(s) @ S(sp) @ fr(s, k)[j + 1])(k, j))
    hands = {0: "R", 1: "L", 2: "R", 3: "L"}
    for k in range(4):
        hand = "R" if HC.LS[k] > 0 else "L"
        for nm in ("tray1", "tray2", "lid"):
            key = f"spring_{nm}_{hand}"
            if key in R:
                add(f"spring {nm} {k}", R[key], "#7a7f88", "rotor", (lambda k: lambda s, sp: G(s) @ S(sp) @ M(C.T(C.rot("z", FAZ[k]))))(k))
    return parts, poses


def main():
    parts, poses = build()
    ts, spins = spin_angles(); names = list(parts)
    frames = np.zeros((len(ts), len(names), 12), np.float32); hud = []
    for i, (t, sp) in enumerate(zip(ts, spins)):
        s = state(t)
        for j, n in enumerate(names):
            A = poses[n](s, sp); frames[i, j] = A[:3, :4].reshape(-1)
        rpm = s["w"] * RPM_HOVER; thrust = T_HOVER * s["w"] ** 2; amps = I_HOVER * s["w"] ** 3 if s["w"] > 0.02 else 0.0
        moving = abs(state(min(t + 0.05, T_END))["pull_b"] - s["pull_b"]) + abs(state(min(t + 0.05, T_END))["pull_c"] - s["pull_c"]) > 1e-3
        phase = ("approach" if t < 3 else "push in: bullets engage" if t < 5 else "collar turns to lock" if t < 7 else "locked" if t < 8 else
                 "rotor spin-up" if t < 10 else "hover (fingers are the rotor fins)" if t < 14 else "spin-down" if t < 15.5 else
                 "spread (lag)" if t < 18 else "curl (K1, K2, K3) until the tips meet" if t < 21 else "hold grip" if t < 23 else "open" if t < 26 else
                 "collar turns to unlock" if t < 28 else "pull out")
        hud.append(dict(t=round(float(t), 2), ph=phase, dz=round(s["dz"], 1), th=round(s["th"], 1), rpm=int(rpm), A=round(amps, 1),
                        T=round(thrust, 1), spread=round(s["spread"], 1), fold=[round(f, 1) for f in s["fold"]],
                        pb=round(s["pull_b"], 2), pc=round(s["pull_c"], 2), servo="moving" if moving else ("holding" if s["pull_b"] + s["pull_c"] > 0.1 else "idle")))
    data = dict(names=names, parts=[parts[n] for n in names], nf=len(ts), fps=FPS,
                frames=base64.b64encode(frames.tobytes()).decode(), hud=hud,
                meta=dict(rpm_hover=int(RPM_HOVER), T_hover=round(T_HOVER, 1), I_hover=I_HOVER, rpm_max=THR["rpm"], I_max=THR["I_A"], T_max=THR["T_max"],
                          travel_b=HUB["travel_b"], travel_c=HUB["travel_c"], F_rated=HUB["base"]["F_rated_finger"], F_stall=HUB["base"]["F_stall_finger"]))
    os.makedirs("coupler_sim_site", exist_ok=True)
    open("coupler_sim_site/sim_data.js", "w").write("window.SIM=" + json.dumps(data) + ";")
    tpl = open("coupler_sim_template.html").read()
    open("coupler_sim_site/index.html", "w").write(tpl)
    print(len(names), "parts,", len(ts), "frames,", round(os.path.getsize("coupler_sim_site/sim_data.js") / 1e6, 2), "MB;",
          "hover", int(RPM_HOVER), "rpm", round(T_HOVER, 1), "N")


if __name__ == "__main__":
    main()
