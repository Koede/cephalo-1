"""Cephalo-2 belly-rotor deployment video (kinematic; geometry from scabbard.py, turretcams.py, cephalo2_model.py).
The robot hovers in the flight pose on its eight hand3 rotors (hover not simulated).  The four blades: stowed upright
in the turret -> the elevator slides them down 194 mm through the trunnions -> heads seat, the rotor turns 6.5 deg and
the elevator fingers unhook -> the push rods let the blades swing out 90 deg on the trunnion hinges -> rods retract ->
the hub (one ring on the motor bell at the base of the turret) spins all four as the 22 in propeller -> and back.
Main view: full robot, slow orbit.  Inset: the turret cut open on the camera side (shell + cage), fixed camera.
python3 rotor_video.py still <t> | chunk <i> <n> | join <n>
"""
import os, sys, subprocess
os.environ.setdefault("C2_HANDS", "all")
import numpy as np
from PIL import Image, ImageDraw
import scabbard_drive as SD                          # patches scabbard.py (turret r 92, stow 185, drive features) before anything else
import cephalo2_model as M
import scabbard as S
import viz4 as V
import cephalo4 as C
from geom import Scene
from render import Camera, render
import manifold3d as m3

FPS = 30; W, H = 1920, 1080; IW, IH = 620, 560
P0 = np.array([0.0, 0.0, 0.80])                      # body origin while hovering (plate underside centre)
SPIN_SHOW = 3.0                                      # rev/s drawn at full speed (real: 72 rev/s = 4320 rpm)
# ---- timeline (s)
T = {}
T["stow"] = 1.5
T["slide"] = T["stow"] + 5.5
T["unhook"] = T["slide"] + 1.0
T["fold"] = T["unhook"] + 3.5
T["retract"] = T["fold"] + 0.8
T["spin"] = T["retract"] + 7.5
T["down"] = T["spin"] + 2.5
T["rods"] = T["down"] + 0.8
T["refold"] = T["rods"] + 3.5
T["rehook"] = T["refold"] + 1.0
T["lift"] = T["rehook"] + 5.5
T["end"] = T["lift"] + 1.5


def mj(u):
    u = float(np.clip(u, 0, 1)); return u * u * u * (10 - 15 * u + 6 * u * u)


def state(t):
    """Blade lift u (mm), droop beta (deg), push-rod droop (None = retracted), elevator lift, hub angle psi (deg), label."""
    st = dict(u=S.U_STOW, beta=S.BETA_H, rods=S.BETA_H, elev=S.U_STOW, psi=0.0, rpm=0.0)
    if t < T["stow"]:
        st["label"] = "Blades stowed upright in the turret"
    elif t < T["slide"]:
        a = mj((t - T["stow"]) / (T["slide"] - T["stow"]))
        st.update(u=S.U_STOW * (1 - a), elev=S.U_STOW * (1 - a), label=f"Elevator slides the blades down through the trunnions: {S.U_STOW * a:.0f} of {S.U_STOW:.0f} mm")
    elif t < T["unhook"]:
        a = mj((t - T["slide"]) / (T["unhook"] - T["slide"]))
        st.update(u=0.0, elev=14.0 * a, psi=6.5 * a, label="Heads seat in the trunnions; rotor turns 6.5 deg, elevator fingers unhook")
    elif t < T["fold"]:
        a = mj((t - T["unhook"]) / (T["fold"] - T["unhook"]))
        b = S.BETA_H + (S.BETA_F - S.BETA_H) * a
        st.update(u=0.0, elev=14.0, psi=6.5, beta=b, rods=b, label=f"Push rods rise, springs swing the blades out on the hinges: {S.BETA_H - b:.0f} of 89 deg")
    elif t < T["retract"]:
        st.update(u=0.0, elev=14.0, psi=6.5, beta=S.BETA_F, rods=None, label="Push rods retract: nothing stationary touches the rotor")
    elif t < T["spin"]:
        e = t - T["retract"]; ramp = min(e / 3.0, 1.0)
        st.update(u=0.0, elev=14.0, beta=S.BETA_F, rods=None, psi=6.5 + 360 * SPIN_SHOW * (0.5 * ramp ** 2 * 3.0 if e < 3.0 else 1.5 + (e - 3.0)),
                  rpm=4320 * ramp, label="The hub ring on the motor bell spins all four blades: 22 in belly propeller")
    elif t < T["down"]:
        e = t - T["spin"]; a = e / (T["down"] - T["spin"])
        psi_end = 6.5 + 360 * SPIN_SHOW * (1.5 + (T["spin"] - T["retract"] - 3.0))
        st.update(u=0.0, elev=14.0, beta=S.BETA_F, rods=None, psi=psi_end + 360 * SPIN_SHOW * (a - 0.5 * a * a) * (T["down"] - T["spin"]),
                  rpm=4320 * (1 - a), label="Spin down")
    else:
        psi_park = 6.5 + 360 * SPIN_SHOW * (1.5 + (T["spin"] - T["retract"] - 3.0) + 0.5 * (T["down"] - T["spin"]))
        psi_park = psi_park - (psi_park - 6.5) % 90.0 + 90.0 if (psi_park - 6.5) % 90 > 1e-6 else psi_park
        if t < T["rods"]:
            st.update(u=0.0, elev=14.0, beta=S.BETA_F, rods=S.BETA_F, psi=psi_park, label="Rotor parked on its index; push rods come down onto the cams")
        elif t < T["refold"]:
            a = mj((t - T["rods"]) / (T["refold"] - T["rods"]))
            b = S.BETA_F + (S.BETA_H - S.BETA_F) * a
            st.update(u=0.0, elev=14.0, beta=b, rods=b, psi=psi_park, label=f"Rods push the blades back down to hanging: {b:.0f} deg")
        elif t < T["rehook"]:
            a = mj((t - T["refold"]) / (T["rehook"] - T["refold"]))
            st.update(u=0.0, elev=14.0 * (1 - a), psi=psi_park - 6.5 * a, label="Elevator fingers drop into the head pockets; rotor turns back 6.5 deg")
        elif t < T["lift"]:
            a = mj((t - T["rehook"]) / (T["lift"] - T["rehook"]))
            st.update(u=S.U_STOW * a, elev=S.U_STOW * a, psi=psi_park - 6.5, label=f"Elevator lifts the blades back into the turret: {S.U_STOW * a:.0f} mm")
        else:
            st.update(psi=psi_park - 6.5, label="Stowed. Belly clean for walking, docking and the centaur neck")
    return st


# ---------------------------------------------------------------- geometry
def tm_prim(m, col, p=P0):
    t = M.tm(m) if not hasattr(m, "faces") else m
    Vv = np.asarray(t.vertices) + p
    return (Vv, np.asarray(t.faces), np.tile(np.asarray(col, float), (len(t.faces), 1)))


_STATIC = {}


def static_prims(cut):
    """Body parts that do not move.  cut: None (whole) or a manifold half-space to subtract from shell + cage (inset)."""
    key = cut is not None
    if key in _STATIC: return _STATIC[key]
    sc = Scene()
    parts = M.body_parts("flight")
    for name, (t, ck, grp) in parts.items():
        if name in ("shell", "cage") and cut is not None:
            man = M.TC.shell_with_windows() if name == "shell" else S.cage_solid()
            t = M.tm(man - cut)
        col = M.COL[ck]
        if name == "cam_disc": col = (0.75, 0.88, 0.95)
        if name == "portholes" and cut is not None: continue
        sc.add(tm_prim(t, col))
    DCOL = dict(servo=(0.22, 0.42, 0.78), belt=(0.12, 0.12, 0.14), pulley=(0.72, 0.74, 0.77), steel=(0.66, 0.68, 0.71), print=(0.30, 0.55, 0.85), brass=(0.80, 0.65, 0.30))
    kind = lambda k: "servo" if k.endswith("_servo") else "belt" if "belt" in k else "pulley" if "pulley" in k or k == "idlers" else "brass" if "miter" in k \
        else "print" if ("bracket" in k or "hanger" in k or "block" in k) else "steel"
    for k, man in SD.drive_parts().items():
        sc.add(tm_prim(man, DCOL[kind(k)]))
    _STATIC[key] = sc.prims
    return sc.prims


_ARMS = {}


def arm_prims(phase):
    """Arms in the flight pose with the hand rotors turning; cached per spin phase (4 phases over the 4-blade rotor's 90 deg period)."""
    k = int(round(phase / (np.pi / 2) * 4)) % 4
    if k in _ARMS: return _ARMS[k]
    import json
    q = np.radians(json.load(open("poses4.json"))["flight"])
    sc = Scene()
    for i in range(8):
        V.arm_mesh(sc, i, P0, np.eye(3), q, np.zeros(C.NJ), spin=k * (np.pi / 2) / 4 + 0.35 * i, fold=0.0, fan_on=True)
    qq = np.tile(q, (8, 1)); s_, b_ = M.bayonets(qq)
    sc.add(tm_prim(s_, M.COL["socket"])); sc.add(tm_prim(b_, M.COL["hbase"]))
    _ARMS[k] = sc.prims
    return sc.prims


def rotor_prims(st, inset=False):
    sc = Scene()
    rot = S.rotating(st["beta"], st["u"], psi=st["psi"])
    sc.add(tm_prim(rot.pop("hub"), M.COL["hub"]))
    sc.add(tm_prim(S.union([v for k, v in rot.items() if k.startswith("barrel")]), M.COL["hub"]))
    for k, v in rot.items():
        if k.startswith("blade"): sc.add(tm_prim(v, (0.24, 0.25, 0.28)), same=k != "blade0")
    rods, zt = S.rods_solid(st["rods"])
    sc.add(tm_prim(rods + S.fold_ring(zt), M.COL["rods"]))
    sc.add(tm_prim(S.elevator_solid(st["elev"]), M.COL["elev"]))
    sc.add(tm_prim(S.motor_solid(), M.COL["motor"]))
    if st["rpm"] > 100:   # swept disc, drawn as a thin ring
        from geom import torus
        sc.add(torus(P0 + [0, 0, S.Z_H * 1e-3], np.array([0, 0, 1.0]), S.R_TIP * 1e-3, 0.0015, [0.35, 0.37, 0.40], n=64, m=4))
    return sc.prims


CUT = m3.Manifold.cube([600, 630, 800]).translate([-300, -30, -300])         # removes y > -30: the near half of shell and cage (inset camera on +y), both servos stay in the far half


def frame(t):
    st = state(t)
    phase = (t * 2 * np.pi * 1.7) % (2 * np.pi)
    prims = static_prims(None) + arm_prims(phase) + rotor_prims(st)
    az = np.radians(-118 + 2.2 * t)
    eye = P0 + [1.95 * np.cos(az), 1.95 * np.sin(az), -0.10]
    img = render(prims, Camera(eye, P0 + [0, 0, 0.20], 40), W, H, ss=2)
    # inset: cutaway turret, fixed camera from -y, slightly above the plate
    ip = static_prims(CUT) + rotor_prims(st, True)
    ieye = P0 + [-0.09, 0.70, 0.08]
    iimg = render(ip, Camera(ieye, P0 + [0, 0, 0.0], 40), IW, IH, ss=2)
    return hud(Image.fromarray(img), Image.fromarray(iimg), t, st)


def hud(img, inset, t, st):
    d = ImageDraw.Draw(img)
    fb, fs, fm = V.font(30, True), V.font(22), V.font(26)
    d.rectangle([0, 0, W, 64], fill=(28, 30, 36))
    d.text((24, 16), "CEPHALO-2  |  belly rotor: stow, slide, swing out, spin  (scabbard.py; kinematic)", font=fb, fill=(240, 240, 236))
    # inset
    x0, y0 = W - IW - 24, 84
    img.paste(inset, (x0, y0)); d.rectangle([x0 - 1, y0 - 1, x0 + IW, y0 + IH], outline=(60, 62, 70), width=2)
    d.rectangle([x0, y0 + IH - 34, x0 + IW, y0 + IH], fill=(28, 30, 36))
    d.text((x0 + 10, y0 + IH - 29), "Cutaway: scabbards, elevator + belts, lead screws, rods, servos", font=V.font(19), fill=(230, 230, 226))
    # step box
    bx0, by0 = 24, 84
    d.rectangle([bx0, by0, bx0 + 840, by0 + 230], fill=(246, 247, 249), outline=(190, 194, 200))
    d.text((bx0 + 16, by0 + 12), st["label"], font=fm, fill=(20, 22, 26))
    rows = [("t", f"{t:5.1f} s"), ("blade lift through the trunnion", f"{st['u']:.0f} mm  (194 = stowed, 0 = seated)"),
            ("droop from the flight stop", f"{st['beta'] - S.BETA_F:.0f} deg  (89 = hanging)"),
            ("push rods", "retracted" if st["rods"] is None else f"on the cams at {st['rods']:.0f} deg"),
            ("rotor", f"{st['rpm']:.0f} rpm" + ("  (drawn at 1/24 speed)" if st["rpm"] > 100 else "")),
            ("hover", "8 x hand3 rotors, kinematic (not simulated)")]
    for k, (a, b) in enumerate(rows):
        d.text((bx0 + 16, by0 + 56 + 27 * k), a, font=fs, fill=(90, 94, 100)); d.text((bx0 + 400, by0 + 56 + 27 * k), b, font=fs, fill=(20, 22, 26))
    d.rectangle([0, H - 40, W, H], fill=(28, 30, 36))
    d.text((24, H - 33), "Blades PA-CF + carbon strip, 222 mm, r 278 (22 in).  Hub: one PA-CF ring on the MN605-S bell, trunnions on M6 shoulder bolts.  Elevator: STS3215 > jackshaft > 4 mm shaft > "
           "2 GT2 loops (12T).  Fold: STS3215 > jackshaft > GT2 loop round 3 T8x2 screws + 4 idlers > rod ring.  All parts fit-checked over the sequence (scabbard_drive.py).", font=V.font(17), fill=(210, 212, 216))
    return np.asarray(img)


if __name__ == "__main__":
    cmd = sys.argv[1]
    N = int(T["end"] * FPS)
    if cmd == "still":
        for t in map(float, sys.argv[2:]):
            Image.fromarray(frame(t)).save(f"rstill_{t}.png")
    elif cmd == "chunk":
        i, n = int(sys.argv[2]), int(sys.argv[3])
        a, b = N * i // n, N * (i + 1) // n
        out = f"rchunk_{i:02d}.part.mp4"
        p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
                              "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for j in range(a, b):
            p.stdin.write(frame(j / FPS).astype(np.uint8).tobytes())
        p.stdin.close()
        if p.wait() == 0: os.rename(out, f"rchunk_{i:02d}.mp4")
    elif cmd == "join":
        n = int(sys.argv[2])
        open("rchunks.txt", "w").write("".join(f"file 'rchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "rchunks.txt", "-c", "copy", "cephalo2_belly_rotor_deploy.mp4"], check=True)
