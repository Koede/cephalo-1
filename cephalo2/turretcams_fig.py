"""Sheet for the turret cameras: cutaway of the Scabbard turret with the 5 camera bays, a to-scale section through one
side bay, and what the cameras see in the stand and flight poses.  python3 turretcams_fig.py -> cephalo1_turretcams.png"""
import json
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import scabbard as S
import turretcams as T

INK, DIM, MUTE = "#15171a", "#4a5058", "#9aa1aa"
COL = dict(shell=(0.80, 0.84, 0.88), cage=(0.62, 0.66, 0.70), blade=(0.25, 0.27, 0.30), elev=(0.85, 0.65, 0.25), plate=(0.70, 0.72, 0.74),
           cradle=(0.30, 0.55, 0.85), board=(0.10, 0.45, 0.25), module=(0.85, 0.20, 0.15), hs=(0.55, 0.56, 0.58), disc=(0.65, 0.85, 0.95),
           bridge=(0.30, 0.55, 0.85), bezel=(0.30, 0.55, 0.85), motor=(0.12, 0.30, 0.55))


def add(ax, man, col, alpha=1.0, L=np.array([0.35, -0.45, 0.82])):
    t = T.to_tm(man); tri = t.vertices[t.faces]; n = t.face_normals; L = L / np.linalg.norm(L)
    sh = 0.38 + 0.62 * np.clip(n @ L, 0, 1)
    fc = np.c_[np.outer(sh, col), np.full(len(sh), alpha)]
    ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, linewidths=0))


def cutaway(ax):
    A = T.assembly()
    keep = S.bx(-300, 300, -300, 0, -100, 400).rotate([0, 0, 20])          # half-space that is cut away (shell, cage)
    sh = T.shell_with_windows() - keep
    add(ax, S.plate_solid() - S.bx(-300, 300, -300, -60, -10, 20).rotate([0, 0, 20]), COL["plate"])
    add(ax, S.cage_solid() - keep, COL["cage"])
    bl = S.blade_solid(holes=False)
    for k in range(4):
        add(ax, S.place_blade(bl, k, S.BETA_H, S.U_STOW) - keep, COL["blade"])
    add(ax, S.elevator_solid(S.U_STOW), COL["elev"])
    add(ax, S.motor_solid(), COL["motor"])
    for k, v in A.items():
        c = COL["module"] if "module" in k else COL["board"] if "board" in k else COL["hs"] if "heatsink" in k else COL["disc"] if "disc" in k \
            else COL["bridge"] if "bridge" in k else COL["bezel"] if "bezel" in k else COL["cradle"]
        add(ax, v, c, 0.55 if "disc" in k else 1.0)
    add(ax, sh, COL["shell"], 0.22)
    for o, n in T.cam_frames():                                             # view-cone axes, 120 mm long
        o = o * 1e3; e = o + n * 120
        ax.plot(*np.c_[o, e], color=COL["module"], lw=1.6)
        a = np.radians(T.CONE); u = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0]); u /= np.linalg.norm(u); w = np.cross(n, u)
        ring = [o + 90 * (np.cos(a) * n + np.sin(a) * (np.cos(t) * u + np.sin(t) * w)) for t in np.linspace(0, 2 * np.pi, 40)]
        ring = np.array(ring); ax.plot(*ring.T, color=COL["module"], lw=0.6, alpha=0.6)
        for t in (0, np.pi / 2, np.pi, 1.5 * np.pi):
            p = o + 90 * (np.cos(a) * n + np.sin(a) * (np.cos(t) * u + np.sin(t) * w)); ax.plot(*np.c_[o, p], color=COL["module"], lw=0.5, alpha=0.5)
    lim = 150
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-30, 270); ax.set_box_aspect((1, 1, 1))
    ax.view_init(24, -62); ax.set_axis_off()


def section(ax):
    """To-scale section through a side bay (azimuth 45 deg plane: r out, z up)."""
    prof = S.mantle_profile(True)
    ax.plot(prof[:, 0], prof[:, 1], color=INK, lw=1.4); ax.plot(-prof[:, 0], prof[:, 1], color=INK, lw=1.4)
    pin = prof * [1, 1]; ax.plot(prof[:, 0] - T.SHELL_T, prof[:, 1] - 0.0, color=MUTE, lw=0.6)
    ax.add_patch(plt.Rectangle((-125, 0), 250, S.PLATE_T, fc="#d6d9dd", ec=INK, lw=0.8))
    gr = S.GUEST_R; ax.add_patch(plt.Rectangle((gr[0] - 2, S.PLATE_T), gr[1] - gr[0] + 4, S.GUEST_H - S.PLATE_T, fc="#c9cdd2", ec=INK, lw=0.8))
    ax.add_patch(plt.Rectangle((gr[0], S.PLATE_T), gr[1] - gr[0], S.GUEST_H - 2 - S.PLATE_T, fc="white", ec=DIM, lw=0.5))
    ax.text(gr[1] + 3, 45, "guest socket\n(dock / horse horns)", fontsize=7.5, color=DIM, va="center")
    ax.add_patch(plt.Rectangle((56, S.Z_CAGE_TOP), 26, 3, fc="#c9cdd2", ec=INK, lw=0.8)); ax.text(20, S.Z_CAGE_TOP + 1, "cage top ring", fontsize=7, color=DIM, va="center")
    for u, ls in ((S.U_STOW, "-"), (0.0, ":")):
        zh = S.Z_H - S.X_HEAD + u
        ax.add_patch(plt.Rectangle((44, zh + 10), 9, 8, fc=COL["elev"] if ls == "-" else "none", ec=COL["elev"], lw=0.8, ls=ls))
    ax.annotate("", (48.5, 25), (48.5, 195), arrowprops=dict(arrowstyle="<->", color=COL["elev"], lw=0.8))
    ax.text(30, 110, "elevator ring\nstroke", fontsize=7, color=COL["elev"], ha="center")
    ax.add_patch(plt.Rectangle((S.MOTOR["d"] / -2, -3), S.MOTOR["d"], S.MOTOR["h"] - 6, fc=COL["motor"], ec=INK, lw=0.6, alpha=0.6))
    # camera stack in the section: tilt the outline points
    t = np.radians(-T.TILT)
    def tp(x, z):
        x, z = x - T.R_LENS, z - T.Z_CAM
        return T.R_LENS + x * np.cos(t) + z * np.sin(t), T.Z_CAM - x * np.sin(t) + z * np.cos(t)
    def poly(xs, zs, **kw):
        P = np.array([tp(x, z) for x, z in zip(xs, zs)]); ax.add_patch(plt.Polygon(P, closed=True, **kw))
    xm = T.R_LENS - T.LENS_H; xb = xm - T.MOD + 2.0; x0 = xb - (T.XIAO["T"] - 6.5); xh = x0 - T.HS
    poly([x0, xb, xb, x0], [T.Z_CAM - 10.5, T.Z_CAM - 10.5, T.Z_CAM + 10.5, T.Z_CAM + 10.5], fc=COL["board"], ec=INK, lw=0.6)
    poly([xb, xm, xm, xb], [T.Z_CAM - 4.25, T.Z_CAM - 4.25, T.Z_CAM + 4.25, T.Z_CAM + 4.25], fc=COL["module"], ec=INK, lw=0.6)
    poly([xm, T.R_LENS, T.R_LENS, xm], [T.Z_CAM - 4, T.Z_CAM - 4, T.Z_CAM + 4, T.Z_CAM + 4], fc=COL["module"], ec=INK, lw=0.6)
    poly([xh, x0, x0, xh], [T.Z_CAM - 7, T.Z_CAM - 7, T.Z_CAM + 7, T.Z_CAM + 7], fc=COL["hs"], ec=INK, lw=0.6)
    poly([xh - 2, xh, xh, xh - 2], [T.Z_CAM - 7, T.Z_CAM - 7, T.Z_CAM + 14, T.Z_CAM + 14], fc=COL["cradle"], ec=INK, lw=0.5)
    ax.add_patch(plt.Polygon([tp(xh - 2, T.Z_CAM + 13), tp(xh, T.Z_CAM + 13), (61.0, T.Z_LUG[0]), (58.5, T.Z_LUG[0])], fc=COL["cradle"], ec=INK, lw=0.5))
    ax.add_patch(plt.Rectangle((58.5, T.Z_LUG[0]), 17.5, 5, fc=COL["cradle"], ec=INK, lw=0.5))
    d, th = T.WIN_SIDE["disc"]; zc = T.Z_CAM + T.WIN_SIDE["dz"]
    ax.add_patch(plt.Rectangle((T.R_DISC - th, zc - d / 2), th, d, fc=COL["disc"], ec=INK, lw=0.5))
    ax.plot([S.TURRET_R - 0.2] * 2, [zc - T.WIN_SIDE["d"] / 2, zc + T.WIN_SIDE["d"] / 2], color="white", lw=3)
    for a in (-T.CONE, 0, T.CONE):
        ang = np.radians(T.TILT + a); ax.plot([T.R_LENS, T.R_LENS + 70 * np.cos(ang)], [T.Z_CAM, T.Z_CAM + 70 * np.sin(ang)], color=COL["module"], lw=1.2 if a == 0 else 0.7, ls="-" if a == 0 else "--")
    # zenith (drawn in the same plane)
    zb = T.Z_ZEN - T.LENS_H
    ax.add_patch(plt.Rectangle((-10.5, zb - T.MOD + 2 - 8.5), 21, 8.5, fc=COL["board"], ec=INK, lw=0.6))
    ax.add_patch(plt.Rectangle((-4.25, zb - T.MOD + 2), 8.5, T.MOD - 2, fc=COL["module"], ec=INK, lw=0.6))
    ax.add_patch(plt.Rectangle((-4, zb), 8, T.LENS_H, fc=COL["module"], ec=INK, lw=0.6))
    ax.add_patch(plt.Rectangle((-56, S.Z_CAGE_TOP + 3), 112, 3, fc=COL["bridge"], ec=INK, lw=0.5)); ax.text(-30, S.Z_CAGE_TOP + 8.5, "bridge", fontsize=7, color=DIM)
    for a in (-T.CONE, 0, T.CONE):
        ang = np.radians(90 + a); ax.plot([0, 60 * np.cos(ang)], [T.Z_ZEN, T.Z_ZEN + 60 * np.sin(ang)], color=COL["module"], lw=1.2 if a == 0 else 0.7, ls="-" if a == 0 else "--")
    ax.annotate(f"lens r {T.R_LENS:.1f}, z {T.Z_CAM:.0f}\naxis {T.TILT:+.0f} deg, cone +-{T.CONE:.0f} deg", (T.R_LENS, T.Z_CAM), (100, 120), fontsize=7.5, color=INK,
                arrowprops=dict(arrowstyle="-", color=DIM, lw=0.6))
    ax.annotate(f"window dia {T.WIN_SIDE['d']:.0f} in the shell\nPC disc {d:.0f} x {th:.0f} in a bezel", (S.TURRET_R, zc - 12), (100, 70), fontsize=7.5, color=INK,
                arrowprops=dict(arrowstyle="-", color=DIM, lw=0.6))
    ax.text(-55, 120, "guest sector below the\ncameras free for packs\nor belts: z 82-177", fontsize=7.5, color=DIM)
    ax.set_xlim(-60, 175); ax.set_ylim(-10, 290); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("Section through a side bay (azimuth 45 deg), to scale, mm", fontsize=10, loc="left", color=INK)


def cmap_panel(ax, pose, title):
    D, tur, tip, per = T.masks(pose)
    az = np.degrees(np.arctan2(D[:, 1], D[:, 0])); el = np.degrees(np.arcsin(D[:, 2]))
    c = np.where(tur, 0, np.where(tip, 1, 2))
    cols = np.array(["#2f7fd8", "#f0a030", "#d7dadf"])
    ax.scatter(az, el, c=cols[c], s=5.5, marker="s", linewidths=0)
    ax.set_xlim(-180, 180); ax.set_ylim(-90, 90); ax.set_xticks([-180, -90, 0, 90, 180]); ax.set_yticks([-90, -30, 0, 30, 90])
    ax.tick_params(labelsize=7); ax.grid(color="white", lw=0.5); ax.set_facecolor("#f5f6f7")
    for a in (45, 135, -45, -135):
        ax.axvline(a, color="#2f7fd8", lw=0.4, ls=":")
    ax.set_title(f"{title}: turret {100*tur.mean():.0f} % of the sphere, + tip cameras {100*(tur|tip).mean():.0f} %", fontsize=9, loc="left")
    ax.set_xlabel("azimuth, deg (dotted: side cameras)", fontsize=7.5); ax.set_ylabel("elevation, deg", fontsize=7.5)


def main():
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    fig.text(0.02, 0.965, "Cephalo-1  Scabbard turret cameras", fontsize=17, fontweight="bold", color=INK)
    fig.text(0.02, 0.905, "4 side cameras in the guest-socket sectors + 1 zenith camera, all inside the blade-storage turret.\n"
             "Seeed XIAO ESP32S3 Sense + OV5640 with a 120 deg class lens (est.), behind clear PC windows.", fontsize=10, color=DIM)
    ax = fig.add_axes([0.0, 0.03, 0.47, 0.88], projection="3d"); cutaway(ax)
    ax2 = fig.add_axes([0.47, 0.37, 0.30, 0.52]); section(ax2)
    ax3 = fig.add_axes([0.52, 0.06, 0.22, 0.20]); cmap_panel(ax3, "stand", "Stand")
    ax4 = fig.add_axes([0.77, 0.06, 0.22, 0.20]); cmap_panel(ax4, "flight", "Flight (tulip)")
    fig.text(0.52, 0.335, "What they see (4000 directions, rays 1.5 m, arms + pods block, props see-through):", fontsize=9, color=INK)
    fig.text(0.52, 0.315, "blue = turret cameras    orange = only the tip cameras    grey = nobody", fontsize=8.5, color=DIM)
    ck = json.load(open("turretcams_check.json"))
    m = ck["mass_g"]
    lines = ["Fit checks on the solids (mm):",
             f"  elevator ring, whole 194 stroke  {ck['camera parts to elevator, whole stroke (worst, at u)'][0]:.1f}",
             f"  stowed blades  {ck['camera parts to stowed blades']:.1f}",
             f"  cage scabbards / sockets  {ck['side bays to cage']:.1f}",
             f"  turret shell  {ck['camera parts to turret shell']:.1f}",
             f"  lens to window disc  {ck['side lens front to window disc']:.1f} (roof {ck['zenith lens front to roof disc']:.1f})",
             f"  view cone to window edge  {ck['window check: side cone clear of the shell hole (worst margin mm)']['side']:.1f} (roof {ck['window check: side cone clear of the shell hole (worst margin mm)']['roof']:.1f})",
             f"  latch pins {ck['camera parts to latch pins']:.0f}, fold drive {ck['camera parts to fold ring + rods']:.0f}",
             "",
             f"Mass +{m['total']:.0f} g (est.): 5 cameras {m['cameras']:.0f}, cradles {m['cradles']:.0f},",
             f"  bridge {m['bridge']:.0f}, bezels + discs {m['bezels'] + m['discs']:.0f}, screws + wire {m['screws_wire']:.0f}",
             "Eyestalks (357 g) have no socket left under the turret.",
             "",
             "Mounting: each cradle hangs under the cage top ring",
             "on 2 x M2.5 + heat-set inserts; at 45 / 225 deg the",
             "same screws clamp the zenith bridge on top.",
             "Cradle prints flat on its back; snap the stack in",
             "from the front, USB-C exits through the floor."]
    fig.text(0.785, 0.90, "\n".join(lines), fontsize=8.2, color=INK, va="top", family="DejaVu Sans")
    fig.savefig("cephalo1_turretcams.png", dpi=120)


if __name__ == "__main__":
    main()
