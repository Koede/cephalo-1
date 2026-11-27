"""sk3d: tiny shared 3D sketch kit for the centaur design study (matplotlib only; metres everywhere).

All solids go into ONE depth-sorted polygon collection per axes (painter's algorithm), so overlaps look right.
    import sk3d
    fig = sk3d.figure(16, 9, "Sheet title")
    sc = sk3d.Scene(fig, 131, elev=16, azim=-62, title="Option A")      # or Scene(fig, (r, c, i)) / Scene(fig, [l, b, w, h])
    sc.box(c=(0, 0, 0.36), size=(0.56, 0.15, 0.10), fc=sk3d.COL["horse"])
    sc.cyl(p0, p1, r, fc)                # cylinder between two points
    sc.tube([p0, p1, p2], r, fc)         # polyline of cylinders with ball joints (links, arms, rods)
    sc.sphere(c, r, fc)   sc.dome(c, a, cz, fc)   sc.disc(c, r, normal, fc)   sc.poly(pts, fc)
    sc.octopus(base=(x, y, z_plate_underside), yaw=0, bars=0.194, arms="centaur")   # Cephalo-1 stand-in
    sc.line(p0, p1, color, lw)   sc.arrow(p, v, color, text)   sc.text(p, "label")   (drawn on top, not depth sorted)
    sc.done(lo=(x0, y0, z0), hi=(x1, y1, z1))     # equal aspect, axes off; call last
    sk3d.save(fig, "name.png")
Colours: sk3d.COL (octo, octo2, horse, frame, act, bar, latch, spring, tpu, pack, belt, ground, hi).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

COL = dict(octo="#e8833a", octo2="#c9661f", horse="#5b7fa6", frame="#9aa7b4", act="#2f3136", bar="#15171a", latch="#d43a2f",
           spring="#2e9e5b", tpu="#e2c044", pack="#7a5cc0", belt="#444a52", ground="#e9ecef", hi="#d43a2f", ghost="#c8ced6")
LIGHT = np.array([0.35, -0.45, 0.82]); LIGHT = LIGHT / np.linalg.norm(LIGHT)


def figure(w=16, h=9, title=None, dpi=120):
    fig = plt.figure(figsize=(w, h), dpi=dpi)
    fig.patch.set_facecolor("white")
    if title:
        fig.suptitle(title, fontsize=17, fontweight="bold", x=0.02, ha="left", y=0.985)
    return fig


def save(fig, path):
    fig.savefig(path, facecolor="white")
    plt.close(fig)
    print("wrote", path)


def rot(axis, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def _frame(d):
    d = np.asarray(d, float); d = d / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(d, a); u /= np.linalg.norm(u)
    return u, np.cross(d, u), d


class Scene:
    def __init__(self, fig, pos=111, elev=16, azim=-62, title=None, ortho=True):
        if isinstance(pos, (list,)) and len(pos) == 4:
            self.ax = fig.add_axes(pos, projection="3d")
        elif isinstance(pos, tuple):
            self.ax = fig.add_subplot(*pos, projection="3d")
        else:
            self.ax = fig.add_subplot(pos, projection="3d")
        if ortho:
            self.ax.set_proj_type("ortho")
        self.ax.view_init(elev=elev, azim=azim)
        self.ax.computed_zorder = False          # solids first (one sorted collection), annotations on top
        self.polys, self.cols, self.edges = [], [], []
        if title:
            self.ax.set_title(title, fontsize=13, fontweight="bold", loc="left", pad=0)

    # ---------------------------------------------------------------- primitives
    def poly(self, pts, fc="#888888", alpha=1.0, ec=None, shade=True):
        pts = np.asarray(pts, float)
        c = np.array(to_rgb(fc))
        if shade and len(pts) >= 3:
            n = np.cross(pts[1] - pts[0], pts[2] - pts[0]); ln = np.linalg.norm(n)
            if ln > 0:
                c = np.clip(c * (0.62 + 0.38 * abs(n @ LIGHT) / ln), 0, 1)
        self.polys.append(pts); self.cols.append((*c, alpha)); self.edges.append(ec if ec is not None else (0, 0, 0, 0.35 * alpha))

    def box(self, c, size, R=None, fc="#888888", alpha=1.0, sub=2):
        c = np.asarray(c, float); h = np.asarray(size, float) / 2
        R = np.eye(3) if R is None else np.asarray(R)
        for ax_ in range(3):
            for s in (-1, 1):
                u, v = [i for i in range(3) if i != ax_]
                for i in range(sub):
                    for j in range(sub):
                        q = []
                        for a, b in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)):
                            p = np.zeros(3); p[ax_] = s * h[ax_]; p[u] = -h[u] + 2 * h[u] * a / sub; p[v] = -h[v] + 2 * h[v] * b / sub
                            q.append(c + R @ p)
                        self.poly(q, fc, alpha)

    def cyl(self, p0, p1, r, fc="#888888", alpha=1.0, n=20, caps=True, r1=None):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        if np.linalg.norm(p1 - p0) < 1e-9:
            return
        u, v, d = _frame(p1 - p0); r1 = r if r1 is None else r1
        th = np.linspace(0, 2 * np.pi, n + 1)
        a = [p0 + r * (np.cos(t) * u + np.sin(t) * v) for t in th]; b = [p1 + r1 * (np.cos(t) * u + np.sin(t) * v) for t in th]
        for i in range(n):
            self.poly([a[i], a[i + 1], b[i + 1], b[i]], fc, alpha, ec=(0, 0, 0, 0.0))
        if caps:
            for ring, cen in ((a, p0), (b, p1)):
                for i in range(n):
                    self.poly([cen, ring[i], ring[i + 1]], fc, alpha, ec=(0, 0, 0, 0.0))

    def ring(self, c, r0, r1, z0, z1, fc="#888888", alpha=1.0, n=28, a0=0.0, a1=360.0):
        """Vertical-axis ring (or arc segment) from radius r0 to r1 between heights z0 and z1, centred at c (x, y)."""
        th = np.radians(np.linspace(a0, a1, n + 1)); cx, cy = c[0], c[1]
        P = lambda r, t, z: np.array([cx + r * np.cos(t), cy + r * np.sin(t), z])
        for i in range(n):
            t0, t1 = th[i], th[i + 1]
            self.poly([P(r1, t0, z0), P(r1, t1, z0), P(r1, t1, z1), P(r1, t0, z1)], fc, alpha, ec=(0, 0, 0, 0))
            self.poly([P(r0, t0, z0), P(r0, t1, z0), P(r0, t1, z1), P(r0, t0, z1)], fc, alpha, ec=(0, 0, 0, 0))
            self.poly([P(r0, t0, z1), P(r1, t0, z1), P(r1, t1, z1), P(r0, t1, z1)], fc, alpha, ec=(0, 0, 0, 0))
            self.poly([P(r0, t0, z0), P(r1, t0, z0), P(r1, t1, z0), P(r0, t1, z0)], fc, alpha, ec=(0, 0, 0, 0))

    def sphere(self, c, r, fc="#888888", alpha=1.0, n=10):
        self.dome(c, r, r, fc, alpha, n, full=True)

    def dome(self, c, a, cz, fc="#888888", alpha=1.0, n=12, full=False):
        c = np.asarray(c, float)
        ph = np.linspace(-np.pi / 2 if full else 0, np.pi / 2, n + 1); th = np.linspace(0, 2 * np.pi, 2 * n + 1)
        P = lambda p, t: c + np.array([a * np.cos(p) * np.cos(t), a * np.cos(p) * np.sin(t), cz * np.sin(p)])
        for i in range(n):
            for j in range(2 * n):
                self.poly([P(ph[i], th[j]), P(ph[i], th[j + 1]), P(ph[i + 1], th[j + 1]), P(ph[i + 1], th[j])], fc, alpha, ec=(0, 0, 0, 0))

    def disc(self, c, r, normal=(0, 0, 1), fc="#888888", alpha=0.35, n=28):
        c = np.asarray(c, float); u, v, _ = _frame(normal)
        th = np.linspace(0, 2 * np.pi, n + 1)
        self.poly([c + r * (np.cos(t) * u + np.sin(t) * v) for t in th], fc, alpha, shade=False)

    def tube(self, pts, r, fc="#888888", alpha=1.0, n=10, balls=True):
        pts = [np.asarray(p, float) for p in pts]
        for a, b in zip(pts[:-1], pts[1:]):
            self.cyl(a, b, r, fc, alpha, n, caps=False)
        if balls:
            for p in pts:
                self.sphere(p, r, fc, alpha, n=5)

    def ground(self, lo, hi, z=0.0, fc=None, n=6):
        fc = fc or COL["ground"]
        xs = np.linspace(lo[0], hi[0], n + 1); ys = np.linspace(lo[1], hi[1], n + 1)
        for i in range(n):
            for j in range(n):
                self.poly([(xs[i], ys[j], z), (xs[i + 1], ys[j], z), (xs[i + 1], ys[j + 1], z), (xs[i], ys[j + 1], z)], fc, 1.0,
                          ec=(0, 0, 0, 0.06), shade=False)

    # ---------------------------------------------------------------- annotations (always on top)
    def line(self, p0, p1, color="k", lw=1.2, ls="-"):
        self.ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color=color, lw=lw, ls=ls, zorder=1e6)

    def arrow(self, p, v, color="#d43a2f", text=None, lw=2.0, fs=10):
        p = np.asarray(p, float); v = np.asarray(v, float)
        self.ax.quiver(p[0], p[1], p[2], v[0], v[1], v[2], color=color, lw=lw, arrow_length_ratio=0.22, zorder=1e6)
        if text:
            q = p + v * 1.08
            self.ax.text(q[0], q[1], q[2], text, color=color, fontsize=fs, fontweight="bold", zorder=1e6)

    def text(self, p, s, color="k", fs=10, ha="left", weight="normal"):
        self.ax.text(p[0], p[1], p[2], s, color=color, fontsize=fs, ha=ha, fontweight=weight, zorder=1e6,
                     bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))

    # ---------------------------------------------------------------- Cephalo-1 stand-in (dims from scabbard.py / cephalo4)
    def octopus(self, base=(0, 0, 0.534), yaw=0.0, bars=None, arms="centaur", alpha=1.0, pods=True, fwd=0.0):
        """base = centre of the plate underside.  bars = stroke of the four Scabbard bars below their stowed position
        (None: stowed, 0.108 dock stroke, 0.194 full stroke).  yaw (deg) turns the octopus; the bars sit at yaw + 90 k.
        arms: 'centaur' (two arms nearest the heading `fwd` reach forward, six held up), 'tulip', 'low', 'none'."""
        b = np.asarray(base, float); Z = np.array([0, 0, 1.0])
        self.cyl(b, b + 0.006 * Z, 0.125, COL["frame"], alpha, n=32)                       # plate
        self.dome(b + 0.006 * Z, 0.125, 0.150, COL["octo"], alpha, n=9)                    # mantle
        self.cyl(b + 0.100 * Z, b + 0.224 * Z, 0.090, COL["octo2"], alpha, n=28)           # turret over the scabbard cage
        self.cyl(b - 0.034 * Z, b - 0.003 * Z, 0.057, COL["act"], alpha, n=24)             # rotor hub + pad
        for k in range(4):
            a = np.radians(yaw + 90 * k); er = np.array([np.cos(a), np.sin(a), 0]); et = np.array([-np.sin(a), np.cos(a), 0])
            self.cyl(b + 0.070 * er - 0.016 * Z - 0.026 * et, b + 0.070 * er - 0.016 * Z + 0.026 * et, 0.014, COL["act"], alpha, n=10)   # trunnion
            if bars is not None:
                tip = -(0.030 + bars)
                R = np.column_stack([er, et, Z])
                self.box(b + 0.070 * er + Z * (tip + 0.5 * min(0.222, -tip)), (0.013, 0.037, min(0.222, -tip)), R, COL["bar"], alpha, sub=1)
        if arms == "none":
            return
        for i in range(8):
            az = yaw + 22.5 + 45 * i; a = np.radians(az)
            er = np.array([np.cos(a), np.sin(a), 0])
            root = b + 0.125 * er + 0.020 * Z
            d = (az - fwd + 180) % 360 - 180
            if arms == "centaur" and abs(d) < 30:
                f = np.array([np.cos(np.radians(fwd)), np.sin(np.radians(fwd)), 0]); side = np.sign(d) * np.array([-f[1], f[0], 0])
                pts = [root, root + 0.16 * er + 0.02 * Z, b + 0.33 * f + 0.14 * side - 0.03 * Z, b + 0.47 * f + 0.10 * side - 0.12 * Z]
                self.tube(pts, 0.016, COL["octo2"], alpha)
                self.sphere(pts[-1], 0.03, COL["tpu"], alpha, n=5)
            elif arms == "low":
                self.tube([root, root + 0.18 * er - 0.02 * Z, root + 0.30 * er - 0.16 * Z, root + 0.33 * er - 0.30 * Z], 0.016, COL["octo2"], alpha)
            else:
                pts = [root, root + 0.15 * er + 0.03 * Z, root + 0.24 * er + 0.15 * Z, root + 0.24 * er + 0.28 * Z]
                self.tube(pts, 0.016, COL["octo2"], alpha)
                if pods:
                    self.disc(pts[-1] + 0.012 * Z, 0.095, Z, COL["octo"], 0.28 * alpha)

    # ---------------------------------------------------------------- finish
    def done(self, lo, hi):
        pc = Poly3DCollection(self.polys, facecolors=self.cols, edgecolors=self.edges, linewidths=0.3)
        pc.set_zorder(1)
        self.ax.add_collection3d(pc)
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        self.ax.set_xlim(lo[0], hi[0]); self.ax.set_ylim(lo[1], hi[1]); self.ax.set_zlim(lo[2], hi[2])
        self.ax.set_box_aspect(tuple(hi - lo), zoom=1.35)
        self.ax.set_axis_off()


def panel_text(fig, x, y, lines, fs=11, w=None, title=None, color="k"):
    """Text block in figure coordinates (top-left anchored).  lines: list of strings."""
    s = "\n".join(lines)
    if title:
        fig.text(x, y, title, fontsize=fs + 2, fontweight="bold", va="top", ha="left", color=color)
        y -= 0.03
    fig.text(x, y, s, fontsize=fs, va="top", ha="left", family="DejaVu Sans", linespacing=1.35)
