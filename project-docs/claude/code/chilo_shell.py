"""Chilo-1 body shell: lofted tergite (back plate) + sternite (belly pan) per segment, as (V, F) meshes in the segment
frame.  One function so the design study, the video renderer and the 3D preview all draw the same shell.

Segment frame: origin at the segment centre, x forward, z up.  The tergite is a superellipse-section loft from the
front edge to a rear lip that overlaps the next segment (roof-tile overlap, like a real centipede's tergites), with a
lateral flare (paratergite) that roofs the coxa servos.  The sternite is a shallow pan under the coxa servos.
"""
import numpy as np

# -------- dimensions (m)
X_FRONT, X_REAR = 0.064, -0.074          # tergite front edge / rear lip (segment pitch 0.120: 18 mm overlap of the next)
W_TOP = 0.052                             # half-width of the dorsal dome at the segment middle
W_FLARE = 0.094                           # half-width at the flare edge (covers the 24.7 mm coxa servos at y = 60 ± 23)
Z_TOP, Z_FLARE, Z_PAN = 0.040, -0.004, -0.032
PAN_W, PAN_X = 0.058, (0.058, -0.058)     # sternite half-width and x extent
NU, NV = 10, 13


def profile(u):
    """Cross-section half-outline at along-position u (0 front .. 1 rear): list of (y, z), from the crest to the flare lip."""
    taper = 1 - 0.18 * (2 * u - 1) ** 2 * 0 - 0.10 * max(0.0, 1 - u * 6) - 0.06 * max(0.0, (u - 0.75) * 4)   # narrower at the very front, slightly at the lip
    pts = []
    for k in range(NV):
        t = k / (NV - 1)
        a = t * np.pi / 2
        # superellipse (n = 2.6) dome from the crest to the shoulder, then the flare
        if t < 0.72:
            s = t / 0.72
            y = W_TOP * taper * np.sign(np.sin(s * np.pi / 2)) * abs(np.sin(s * np.pi / 2)) ** (2 / 2.6)
            z = Z_TOP * abs(np.cos(s * np.pi / 2)) ** (2 / 2.6)
        else:
            s = (t - 0.72) / 0.28
            y = (W_TOP + (W_FLARE - W_TOP) * s) * taper
            z = Z_FLARE * s + 0.004 * np.sin(s * np.pi)
        pts.append((y, z))
    return pts


def tergite(i=0):
    """(V, F) of one tergite; i only sets the head taper on segment 0."""
    V = []; F = []
    us = np.linspace(0, 1, NU + 1)
    for u in us:
        x = X_FRONT + (X_REAR - X_FRONT) * u
        zlip = -0.006 * max(0.0, (u - 0.85) / 0.15)              # the rear lip dips to sit over the next tergite
        pr = profile(u)
        row = []
        for y, z in reversed(pr[1:]):
            row.append((x, -y, z + zlip))
        for y, z in pr:
            row.append((x, y, z + zlip))
        V.append(row)
    V = np.array(V)                                                # (NU+1, 2NV-1, 3)
    n = V.shape[1]
    Vf = V.reshape(-1, 3)
    for a in range(NU):
        for b in range(n - 1):
            i0 = a * n + b; i1 = i0 + 1; i2 = i0 + n + 1; i3 = i0 + n
            F += [[i0, i2, i1], [i0, i3, i2]]
    # front wall (closes the shell at the front of segment 0; elsewhere it is hidden under the tergite ahead)
    c = len(Vf); Vf = np.vstack([Vf, [[X_FRONT, 0, 0.5 * (Z_TOP + Z_FLARE)]]])
    for b in range(n - 1):
        F.append([c, b, b + 1])
    return Vf, np.array(F)


def sternite():
    x0, x1 = PAN_X; w = PAN_W; z0, z1 = Z_PAN, Z_PAN + 0.014
    V = np.array([[x, y, z] for x in (x1, x0) for y in (-w, w) for z in (z0, z1)], float)
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def pack_box():
    """6S 1300 mAh pack envelope (75 x 35 x 30 mm, est.) standing on the sternite between the coxa servos."""
    return (0.075, 0.035, 0.030), (0.0, 0.0, Z_PAN + 0.014 + 0.015)


if __name__ == "__main__":
    V, F = tergite()
    print("tergite verts", len(V), "faces", len(F), "x %.0f..%.0f mm, y ±%.0f mm, z %.0f..%.0f mm" % (
        V[:, 0].min() * 1e3, V[:, 0].max() * 1e3, V[:, 1].max() * 1e3, V[:, 2].min() * 1e3, V[:, 2].max() * 1e3))
