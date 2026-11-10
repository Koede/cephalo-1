"""3-tile MuJoCo video: 0.2 kg object at 6 m/s into the prop.
  A rigid arm (j5 STS3215)   B rigid, j5 + j7 STS3215   C same + winch-tensioned detent clutches (60 N line)
Each tile draws the simulated arm straight from MuJoCo geoms, plus a live bar per joint for the gear-train load
(motor torque + reflected rotor inertia) as a fraction of an assumed 3x-stall gear limit.
usage: python3 make_clutch_video.py <start> <end> <out.mp4> | still <k> <out.png> | count"""
import sys, os, subprocess, numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
import sim_seats as SS, sim_wrist as SW
from tune_clutch import config
from geom import Scene, frustum, sphere, box, merge
from render import render, Camera
from robot import arm_fk, arm_mesh, body_mesh, pod_mesh

TW, TH = 640, 720
V = 6.0
F = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_t = ImageFont.truetype(FB, 21); f_s = ImageFont.truetype(F, 14); f_b = ImageFont.truetype(FB, 15); f_c = ImageFont.truetype(F, 19)
TILES = [("RIGID ARM", "j5 STS3215, rest as rev 3", False, False),
         ("RIGID, STRONGER TIP", "j5 + j7 STS3215 (+70 g)", True, False),
         ("CLUTCHED JOINTS", "j5 + j7 STS3215 + detent clutches, line 60 N", True, True)]
BODY = np.array([0, 0, SW.BODY_Z])


def run(j7, clutch):
    config(j7, 0.9, 60)
    s = SS.SeatSim(clutch, 60.0, imp_v=V, clutch=True).run(2.6, rec=0.001)
    geoms = (s.m.geom_type.copy(), s.m.geom_size.copy(), s.m.geom_rgba.copy(), s.m.geom_bodyid.copy())
    frames = []
    d2 = mujoco.MjData(s.m)
    return s, geoms


def timeline():
    ts, t = [], 0.30
    while t < 2.6:
        ts.append(t); t += (1 / 30) * (0.2 if 0.44 <= t < 0.75 else 1.0)
    return ts


def mj_meshes(m, xpos, xmat, gxpos, gxmat, skip_bodies):
    parts = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        if b in skip_bodies or b == 0:
            continue
        t = m.geom_type[g]; sz = m.geom_size[g]; col = m.geom_rgba[g][:3]
        p = gxpos[g]; R = gxmat[g].reshape(3, 3)
        if t == mujoco.mjtGeom.mjGEOM_CAPSULE or t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            a = R[:, 2] * sz[1]
            parts.append(frustum(p - a, p + a, sz[0], sz[0], col, n=14))
        elif t == mujoco.mjtGeom.mjGEOM_BOX:
            parts.append(box(p, R, 2 * sz[:3], col))
        elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
            parts.append(sphere(p, sz[0], col, n=14, m=10))
    return parts


def frame(runs, ts, k):
    t = ts[k]
    imgs, info = [], []
    for (title, sub, j7, cl), (s, _) in zip(TILES, runs):
        idx = min(int(round(t / 0.001)), len(s.log) - 1)
        tt, meas, (xpos, xmat) = s.log[idx]
        gx = s.glog[idx]
        Sc = Scene()
        body_mesh(Sc, BODY, edf_ang=t * 40)
        for i in range(1, 8):
            fr, (R, p) = arm_fk(i, SW.FLIGHT, BODY)
            arm_mesh(Sc, fr, (R, p), [0.95, 0.55, 0.15] if cl else [0.4, 0.4, 0.45])
            Sc.add(frustum(p, p + R[:, 0] * 0.07, 0.008, 0.007, [0.2, 0.21, 0.23]))
            pod_mesh(Sc, p + R[:, 0] * 0.07, R[:, 0], R[:, 2], t * 47 + i, 0.0, False)
        for part in mj_meshes(s.m, xpos, xmat, gx[0], gx[1], set()):
            Sc.add(part)
        _, (R0, p0) = arm_fk(0, SW.FLIGHT, BODY)
        tgt = 0.55 * p0 + 0.45 * (BODY + [0, 0, 0.08]) + np.array([0, 0, 0.06])
        az, el, dist = np.radians(22.5 + 15), 0.14, 1.25
        eye = tgt + dist * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
        imgs.append(render(Sc.prims, Camera(eye, tgt, fov_deg=42), TW, TH, ss=2, ground=(0, 0)))
        gear_now = [meas[f"g{j}"] for j in range(9)]
        gear_pk = np.max([[l[f"g{j}"] for j in range(9)] for _, l, _ in s.log[:idx + 1]], 0)
        info.append((title, sub, cl, gear_now, gear_pk, meas))
    im = Image.fromarray(np.concatenate(imgs, 1)).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0)); do = ImageDraw.Draw(ov)
    for j in range(3):
        do.rounded_rectangle([(j * TW + 8, 8), (j * TW + TW - 8, 70)], 9, fill=(255, 255, 255, 215))
        do.rounded_rectangle([(j * TW + 8, TH - 232), (j * TW + TW - 8, TH - 44)], 9, fill=(255, 255, 255, 215))
    im = Image.alpha_composite(im, ov).convert("RGB"); d = ImageDraw.Draw(im)
    for j, (title, sub, cl, gn, gp, meas) in enumerate(info):
        x0 = j * TW + 18
        d.text((x0, 14), title, font=f_t, fill=(200, 110, 20) if cl else (60, 60, 70))
        d.text((x0, 42), sub, font=f_s, fill=(40, 40, 40))
        y0 = TH - 226
        d.text((x0, y0), "gear-train load / est. limit   (bar = now, tick = peak)", font=f_s, fill=(40, 40, 40))
        for q in range(9):
            y = y0 + 22 + q * 14
            d.text((x0, y - 2), f"j{q}", font=f_s, fill=(60, 60, 60))
            bx0, bw = x0 + 30, TW - 120
            d.rectangle([(bx0, y), (bx0 + bw, y + 9)], outline=(150, 150, 150))
            lim_x = bx0 + bw * (1 / 1.5)
            v = min(gn[q] / 1.5, 1); pk = min(gp[q] / 1.5, 1)
            c = (200, 30, 20) if gn[q] > 1 else (60, 150, 70)
            d.rectangle([(bx0, y + 1), (bx0 + bw * v, y + 8)], fill=c)
            d.line([(bx0 + bw * pk, y - 1), (bx0 + bw * pk, y + 10)], fill=(200, 30, 20) if gp[q] > 1 else (30, 30, 30), width=2)
            d.line([(lim_x, y - 2), (lim_x, y + 11)], fill=(0, 0, 0), width=1)
            d.text((bx0 + bw + 6, y - 2), f"{gp[q]:.1f}", font=f_s, fill=(200, 30, 20) if gp[q] > 1 else (40, 40, 40))
        worst = max(gp)
        d.text((x0, y0 + 22 + 9 * 14 + 4), (f"peak {worst:.1f}x limit" + ("  - gears at risk" if worst > 1 else "  - within limit")) +
               (f"   clutch slip {meas['open_deg']:.0f}°" if cl else ""), font=f_b, fill=(200, 30, 20) if worst > 1 else (30, 120, 40))
    for c in (1, 2):
        d.line([(c * TW, 0), (c * TW, TH)], fill=(60, 60, 60), width=2)
    slow = 0.44 <= t < 0.75
    cap = f"MuJoCo: 0.2 kg object at {V:.0f} m/s into one fan    t = {t:4.2f} s" + ("    ▶ 1/5 speed" if slow else "")
    d.rectangle([(0, TH - 38), (3 * TW, TH)], fill=(25, 27, 30))
    tw = d.textlength(cap, font=f_c)
    d.text(((3 * TW - tw) / 2, TH - 30), cap, font=f_c, fill=(255, 220, 120) if slow else (240, 240, 240))
    return im


def run_all():
    runs = []
    for title, sub, j7, cl in TILES:
        config(j7, 0.9, 60)
        s = SS.SeatSim(cl, 60.0, imp_v=V, clutch=True)
        s.glog = []
        nxt = 0.0
        while s.d.time < 2.6:
            s.step()
            if s.d.time >= nxt:
                s.log.append((s.d.time, s.measure(), (s.d.xpos.copy(), s.d.xmat.copy())))
                s.glog.append((s.d.geom_xpos.copy(), s.d.geom_xmat.copy()))
                nxt += 0.001
        runs.append((s, None))
    return runs


def main():
    ts = timeline()
    if sys.argv[1] == "count":
        print(len(ts)); return
    runs = run_all()
    if sys.argv[1] == "still":
        frame(runs, ts, int(sys.argv[2])).save(sys.argv[3]); return
    a, b, out = int(sys.argv[1]), min(int(sys.argv[2]), len(ts)), sys.argv[3]
    tmp = out[:-4] + ".part.mp4"
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{3 * TW}x{TH}",
                          "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", tmp],
                         stdin=subprocess.PIPE)
    for k in range(a, b):
        p.stdin.write(np.asarray(frame(runs, ts, k)).tobytes())
    p.stdin.close()
    if p.wait() == 0:
        os.replace(tmp, out)


if __name__ == "__main__":
    main()
