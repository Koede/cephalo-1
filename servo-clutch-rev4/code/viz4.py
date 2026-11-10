"""Rev 4 meshes + mission video renderer (frames drawn from the MuJoCo states in sim4_<case>.npz).

python3 viz4.py still <t>            -> still_<t>.png (check frame)
python3 viz4.py hero                 -> cephalo1_rev4_hero.png (stand / walk / take-off / hover frames, 2x2)
python3 viz4.py chunk <i> <n>        -> renders frame chunk i of n and encodes chunk_<i>.part.mp4 (renamed when done)
python3 viz4.py join <n>             -> cephalo1_rev4_mission.mp4
"""
import os, sys, json, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont
import cephalo4 as C
from geom import frustum, sphere, torus, box, merge, frame_z, rot, Scene
from render import Camera, render

WHITE = [0.92, 0.92, 0.90]; GREY = [0.55, 0.57, 0.60]; DARK = [0.20, 0.21, 0.23]; ORANGE = [0.95, 0.55, 0.15]
BLUE = [0.22, 0.42, 0.78]; STEEL = [0.70, 0.72, 0.75]; YEL = [0.95, 0.85, 0.35]; RED = [0.88, 0.15, 0.12]
BLADE = [0.25, 0.27, 0.30]
SCOL = {"STS3215": BLUE, "STS3250": [0.28, 0.29, 0.32]}
W_IMG, H_IMG = 1920, 1080
CASE = os.environ.get("CEPH_CASE", "mid")


# ------------------------------------------------------------------ meshes
def body_mesh(S, p, R, edf_ang):
    def T(v):
        return p + R @ np.asarray(v, float)
    S.add(frustum(T([0, 0, 0]), T([0, 0, 0.006]), 0.125, 0.125, GREY, n=48))
    V, F, Cc = sphere(np.zeros(3), 1.0, WHITE, n=40, m=20)
    V[:, 2] = np.maximum(V[:, 2], 0.0)
    V = (V * [0.125, 0.125, 0.150] + [0, 0, 0.006]) @ R.T + p
    S.add((V, F, Cc))
    S.add(torus(T([0, 0, 0.118]), R[:, 2], 0.058, 0.006, GREY, ref=R[:, 0]))
    for i in range(8):                                    # arm portholes (dark collars)
        a = np.radians(22.5 + 45 * i); d = np.array([np.cos(a), np.sin(a), 0])
        S.add(frustum(T(d * 0.100 + [0, 0, 0.039]), T(d * 0.110 + [0, 0, 0.039]), 0.031, 0.031, DARK, n=20), same=i > 0)
    S.add(frustum(T([0, 0, -0.004]), T([0, 0, -0.092]), 0.055, 0.047, DARK, n=32))
    S.add(torus(T([0, 0, -0.004]), R[:, 2], 0.051, 0.005, GREY, ref=R[:, 0]))
    for k in range(12):
        a = edf_ang + k * np.pi / 6
        d = R @ np.array([np.cos(a), np.sin(a), 0])
        S.add(box(T([0, 0, -0.088]) + d * 0.022, frame_z(R[:, 2], d), [0.04, 0.008, 0.002], STEEL), same=k > 0)


def arm_mesh(S, i, p, R, q, c, spin, fold, fan_on, warn=False):
    segs, (Rt, pt) = C.arm_fk(i, q, p, R, clutch=c)
    jf = C.joint_frames(segs)
    # spine: tapered white shell along the arm
    ss = np.linspace(0.0, C.SPIGOT, 34)
    P = [C.at_s(segs, s)[0] for s in ss]
    for k in range(len(P) - 1):
        r = float(np.interp(ss[k], [0, C.SPIGOT], [0.013, 0.008]))
        S.add(frustum(P[k], P[k + 1], r, r, WHITE, n=12), same=k > 0)
    for k in range(C.NJ):
        n = C.SERVO[k]
        L, Wd, H = np.array(C.SV[n][3:]) * 1e-3
        # servo k sits in link k (proximal of joint k), shaft on the joint axis 11.4 mm from its output end
        sc = C.S_J[k] - (L / 2 - 0.0114)
        cpos, Rl = C.at_s(segs, sc)
        ax = Rl[:, 1] if C.TYP[k] == "p" else Rl[:, 2]
        o = np.cross(ax, Rl[:, 0])
        S.add(box(cpos, np.stack([Rl[:, 0], o, ax], 1), [L, Wd, H * 0.92], SCOL[n]))
        # printed cradle: two white side cheeks along the servo (open shell)
        for sg in (-1, 1):
            S.add(box(cpos + o * sg * (Wd / 2 + 0.0025) - Rl[:, 0] * 0.004, np.stack([Rl[:, 0], o, ax], 1), [L + 0.006, 0.004, H * 0.80], WHITE), same=sg > 0)
        cj, axj, Rc = jf[k]
        hw = H / 2
        # horn side: clutch cam disc (orange) + rocker on the driven link; other side: idler boss (grey)
        if C.CLUTCH_FRAC[k] > 0:
            rc = 0.019 if n == "STS3215" else 0.025
            S.add(frustum(cj + axj * (hw + 0.001), cj + axj * (hw + 0.007), rc, rc, ORANGE, n=22, c2=[0.75, 0.40, 0.10], stripes=4, ref=Rc[:, 0]))
            pd, Rd = C.at_s(segs, C.S_J[k] + 0.026)
            S.add(box(pd + axj * (hw + 0.010), np.stack([Rd[:, 0], np.cross(axj, Rd[:, 0]), axj], 1), [0.050, 0.010, 0.006], [0.80, 0.45, 0.12]))
            S.add(frustum(pd + Rd[:, 0] * 0.022 + axj * (hw + 0.007), pd + Rd[:, 0] * 0.022 + axj * (hw + 0.014), 0.005, 0.005, STEEL, n=12))
        else:
            S.add(frustum(cj + axj * (hw + 0.001), cj + axj * (hw + 0.005), 0.012, 0.012, DARK, n=16))
        S.add(frustum(cj - axj * (hw + 0.006), cj - axj * (hw + 0.001), 0.011, 0.011, GREY, n=16))
        # driven link fork: two white plates from the joint axis forward
        pd2, Rd2 = C.at_s(segs, C.S_J[k] + 0.012)
        od = np.cross(axj, Rd2[:, 0])
        for sg in (-1, 1):
            S.add(box(pd2 + axj * sg * (hw + 0.0035 + (0.008 if sg > 0 and C.CLUTCH_FRAC[k] > 0 else 0)), np.stack([Rd2[:, 0], od, axj], 1),
                      [0.034, 0.020, 0.003], WHITE), same=sg > 0)
    # tip bay + pod
    u = Rt[:, 0]
    S.add(box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], [0.35, 0.36, 0.40]))
    S.add(frustum(pt, pt + u * 0.004, 0.018, 0.018, DARK, n=16))
    S.add(frustum(pt + u * 0.004, pt + u * 0.026, 0.0145, 0.0145, RED if warn else GREY, n=18))
    hub = pt + u * C.BLADE_OFF
    S.add(frustum(pt + u * 0.026, hub + u * 0.004, 0.006, 0.006, DARK, n=10))
    e1 = Rt[:, 1]; e2 = Rt[:, 2]
    for b in range(3):
        a = spin + b * 2 * np.pi / 3
        rd = np.cos(a) * e1 + np.sin(a) * e2
        bd = np.cos(fold) * rd + np.sin(fold) * u
        cb = hub + bd * C.CLAW_R / 2
        side = np.cross(u, rd)
        S.add(box(cb, np.stack([bd, side, np.cross(bd, side)], 1) @ rot([1, 0, 0], 0.35), [C.CLAW_R, 0.017, 0.0025], BLADE))
        S.add(sphere(hub + bd * C.CLAW_R, 0.004, DARK, n=8, m=6), same=True)
    if fan_on:
        S.add(torus(hub + u * 0.002, u, C.CLAW_R, 0.0012, [0.35, 0.37, 0.40], n=40, m=4))


def scene_from_state(st, t, spin, fold, fan, edf, imp=None):
    p, R, q, c = st[:4]
    S = Scene()
    body_mesh(S, p, R, edf_ang=t * 40.0 * (edf > 1))
    for i in range(8):
        arm_mesh(S, i, p, R, q[i], c[i], spin[i], fold[i], fan[i] > 1.0)
    if imp is not None and imp[2] > -1:
        S.add(sphere(imp, 0.02, RED, n=16, m=10))
    return S


# ------------------------------------------------------------------ HUD
def font(sz, bold=False):
    for f in (["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"] if bold else []) + ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        if os.path.exists(f):
            return ImageFont.truetype(f, sz)
    return ImageFont.load_default()


F_T, F_B, F_M, F_S = font(34, True), font(26, True), font(22), font(18)


def hud(img, info):
    im = Image.fromarray(img); d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, W_IMG, 64], fill=(20, 22, 26, 200))
    d.text((24, 12), "CEPHALO-1 REV 4  |  integrated design: servo-in-link arms + winch detent clutches + 6 in claw-props", font=F_B, fill=(240, 240, 240))
    d.rectangle([20, 80, 640, 470], fill=(255, 255, 255, 200), outline=(60, 60, 60, 255))
    y = 92
    d.text((36, y), info["mode"], font=F_T, fill=(20, 20, 20)); y += 50
    sl = info["slow"]
    d.text((36, y), f"t = {info['t']:5.2f} s" + ("      SLOW MOTION 1/10" if sl else ""), font=F_M, fill=(200, 30, 30) if sl else (40, 40, 40)); y += 36
    if info.get("sub"):
        d.rectangle([660, 80, 1420, 120], fill=(255, 255, 255, 200), outline=(60, 60, 60, 255))
        d.text((676, 88), info["sub"], font=F_M, fill=(180, 20, 20))
    rows = [("Robot mass (est.)", f"{C.M_ROBOT:.2f} kg"), ("Claw-prop max (est.)", f"{info['fmax']:.1f} N  (T/W {info['tw']:.2f})"),
            ("EDF thrust", f"{info['edf']:.0f} N"), ("Claw-props running", f"{info['nfan']}  (max {info['fanmax']:.1f} N)"),
            ("Winch line", f"{info['winch']}  {info['T']:.0f} N"), ("Worst servo load", f"{info['servo']:.2f} x stall"),
            ("Worst clutch slip", f"{info['slip']:.1f} deg"), ("Body tilt", f"{info['tilt']:.1f} deg")]
    for k, v in rows:
        d.text((36, y), k, font=F_M, fill=(70, 70, 70)); d.text((330, y), v, font=F_M, fill=(15, 15, 15)); y += 34
    # per-arm bars
    x0, y0 = W_IMG - 470, 80
    d.rectangle([x0, y0, W_IMG - 20, y0 + 300], fill=(255, 255, 255, 200), outline=(60, 60, 60, 255))
    d.text((x0 + 14, y0 + 8), "Per arm: servo load (x stall) / clutch slip", font=F_S, fill=(30, 30, 30))
    for i in range(8):
        yy = y0 + 40 + i * 31
        d.text((x0 + 14, yy), f"arm {i}", font=F_S, fill=(40, 40, 40))
        v = min(info["servo_arm"][i], 1.0)
        col = (40, 160, 70) if v < 0.7 else ((230, 160, 30) if v < 0.95 else (210, 40, 40))
        d.rectangle([x0 + 80, yy + 3, x0 + 80 + 220, yy + 21], outline=(120, 120, 120))
        d.rectangle([x0 + 80, yy + 3, x0 + 80 + int(220 * v), yy + 21], fill=col)
        s = min(info["slip_arm"][i] / 40.0, 1.0)
        d.rectangle([x0 + 312, yy + 3, x0 + 312 + 120, yy + 21], outline=(120, 120, 120))
        d.rectangle([x0 + 312, yy + 3, x0 + 312 + int(120 * s), yy + 21], fill=(240, 130, 30))
    d.rectangle([0, H_IMG - 40, W_IMG, H_IMG], fill=(20, 22, 26, 190))
    d.text((24, H_IMG - 32), "MuJoCo rigid-body simulation of the full robot. Masses, claw-prop thrust, rotor inertia, gear limits and clutch "
           "parameters are estimates; servos are stall-limited position loops.", font=F_S, fill=(230, 230, 230))
    return np.asarray(im)


# ------------------------------------------------------------------ mission video
def load(case=CASE):
    import sim4 as SM
    Z = np.load(f"sim4_{case}.npz")
    return SM, Z


def frame_info(SM, Z, j, states):
    t = Z["t"][j]
    fan = Z["fan"][j]
    slow = (j > 0 and Z["t"][j] - Z["t"][j - 1] < 0.02) or (j + 1 < len(Z["t"]) and Z["t"][j + 1] - t < 0.02)
    T = Z["T"][j]
    wm = "tight" if T > 100 else ("firm" if T > 40 else "soft")
    fmax = float(json.load(open(f"sim4_{CASE}.json"))["fan_max"])
    return dict(t=t, mode=SM.mode(t), slow=slow, edf=Z["edf"][j], nfan=int((fan > 1).sum()), fanmax=float(fan.max()),
                winch=wm, T=T, servo=float(Z["servo"][j]), slip=float(Z["slip"][j]), tilt=float(Z["tilt"][j]),
                servo_arm=Z["servo_arm"][j], slip_arm=Z["slip_arm"][j], fmax=fmax, tw=(C.EDF_MAX + 8 * fmax) / C.W_ROBOT)


def camera_for(SM, t, p, slow_target=None, slow_arm=None):
    # orbit around the body, wider in flight; zoom in for the hit slow-motion
    az = np.radians(205 + 4.0 * t)
    airborne = SM.T_HOP < t < SM.T_TOUCH + 0.5
    dist = 1.75 if not airborne else 2.05
    tgt = np.array([p[0], p[1], max(p[2] + 0.12, 0.35)])
    if slow_target is not None:                     # hit close-up: look at the struck claw-prop from outside
        a = np.radians(22.5 + 45 * slow_arm + 38)
        tgt = 0.35 * tgt + 0.65 * slow_target
        return Camera(tgt + np.array([np.cos(a) * 1.35, np.sin(a) * 1.35, 0.30]), tgt, fov_deg=40)
    eye = tgt + np.array([np.cos(az) * dist, np.sin(az) * dist, 0.80 if not airborne else 0.55])
    return Camera(eye, tgt, fov_deg=40)


def fold_spin(SM, Z, j):
    """Claw fold: 60 deg (feet) when the fan is idle, opens to 0 as thrust builds; spin integrates thrust."""
    fan = Z["fan"][:j + 1]
    fold = np.radians(60) * (1 - np.clip(fan[-1] / 2.5, 0, 1))
    dt = np.diff(Z["t"][:j + 1], prepend=0)
    spin = (np.sqrt(np.clip(fan, 0, None)) * dt[:, None]).sum(0) * 9.0
    return fold, spin


def render_frame(SM, Z, j):
    import states4 as ST
    st = ST.state(Z["qpos"][j])
    t = Z["t"][j]
    fold, spin = fold_spin(SM, Z, j)
    info = frame_info(SM, Z, j, st)
    slow_tgt = None
    if info["slow"]:
        arm = 2 if abs(t - SM.T_HIT1) < 0.5 else 6
        _, (Rt, pt) = C.arm_fk(arm, st[2][arm], st[0], st[1], clutch=st[3][arm])
        slow_tgt = pt
    S = scene_from_state(st, t, spin, fold, Z["fan"][j], Z["edf"][j], imp=st[4])
    cam = camera_for(SM, t, st[0], slow_tgt, arm if slow_tgt is not None else None)
    if slow_tgt is not None:
        info["sub"] = ("0.2 kg ball at 6 m/s into arm 2's claw-prop, side-on" if arm == 2 else
                       "0.2 kg ball at 6 m/s into arm 6's claw-prop, toward the body")
    img = render(S.prims, cam, W_IMG, H_IMG, ss=2, ground=(st[0][0], st[0][1]), fog=(4.0, 12.0))
    return hud(img, info)


def chunk(ci, n):
    SM, Z = load()
    N = len(Z["t"]); idx = np.array_split(np.arange(N), n)[ci]
    out = f"chunk_{ci:02d}.part.mp4"
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W_IMG}x{H_IMG}",
                           "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out],
                          stdin=subprocess.PIPE)
    for j in idx:
        ff.stdin.write(render_frame(SM, Z, j).tobytes())
    ff.stdin.close(); rc = ff.wait()
    if rc == 0:
        os.replace(out, f"chunk_{ci:02d}.mp4")
    print("chunk", ci, "rc", rc, len(idx), "frames")


def join(n):
    with open("chunks.txt", "w") as f:
        for i in range(n):
            f.write(f"file 'chunk_{i:02d}.mp4'\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "chunks.txt", "-c", "copy",
                    "cephalo1_rev4_mission.mp4"], check=True)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "still":
        SM, Z = load(); t = float(sys.argv[2]); j = int(np.argmin(np.abs(Z["t"] - t)))
        Image.fromarray(render_frame(SM, Z, j)).save(f"still_{t:.1f}.png"); print("saved", j, Z["t"][j])
    elif cmd == "chunk":
        chunk(int(sys.argv[2]), int(sys.argv[3]))
    elif cmd == "join":
        join(int(sys.argv[2]))
    elif cmd == "hero":
        SM, Z = load()
        tiles = []
        for t, label in ((1.0, "Stand"), (4.4, "Walk"), (11.5, "Take-off"), (20.0, "Hover")):
            j = int(np.argmin(np.abs(Z["t"] - t)))
            st = __import__("states4").state(Z["qpos"][j]); fold, spin = fold_spin(SM, Z, j)
            S = scene_from_state(st, Z["t"][j], spin, fold, Z["fan"][j], Z["edf"][j])
            cam = camera_for(SM, Z["t"][j], st[0])
            im = Image.fromarray(render(S.prims, cam, 960, 540, ss=2, ground=(st[0][0], st[0][1]), fog=(4.0, 12.0)))
            ImageDraw.Draw(im).text((18, 12), label, font=F_B, fill=(20, 20, 20)); tiles.append(im)
        out = Image.new("RGB", (1920, 1080))
        for k, im in enumerate(tiles):
            out.paste(im, ((k % 2) * 960, (k // 2) * 540))
        out.save("cephalo1_rev4_hero.png")
