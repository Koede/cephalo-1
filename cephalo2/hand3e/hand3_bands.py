"""hand3 hub v3 return springs, decision D (2026-11-20): one stock O-ring band across the back of each knuckle.

Each band is stretched between two steel dowels (dia 1 x 5, pressed into a printed clevis under the member's back), one on the
proximal member and one on the distal member, and runs in the knuckle's centre gap (|y| < 0.89) over a small back sector printed on
the distal member (radius r_w, distal-frame angles 180..275, the old saddle's place).  The O-ring's two strands lie one over the
other (dowels along y), so the band is cs wide and its mean radius on the sector is r_w + cs.  The sector keeps the arm at
r_w + cs whatever the fold; the band is never in front of the axis.
Replaces hub E's spring boxes, lids, the root post, the saddles, extensor cords and their bores.  The finger's flexor is one
Dyneema cord over K1, K2 and K3 (K1's anchor hole is now a pass-through into the axis passage; phalanx 1's unused tip-cord lane
is gone).
Angles as hand3_finger.wedge: p from +z towards +x (p 180 = back, -z).  Units mm, N, N m.  Estimates where marked."""
import os, json
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_finger as F
import hand3_springs as SP
from hand3_finger import cyl, box, app, T, rot

M_ = mf.Manifold
KN = H.KN
R_D = 0.5                                     # hardened steel dowel dia 1 x 5 (ISO 8734), press fit in PA-CF; bending <= 152 MPa
FIN_Y = (1.05, 2.45)                           # clevis fins either side of the band
FIN_X = 1.5                                    # fin half length along x (fin root bending 16 MPa at full fold + spin, est.)
W = 1309.0                                     # rad/s, 12,500 rpm
LIMIT = [0.40 * 31.9 * H.LEVER[j] * 1e-3 for j in range(3)]       # 40 % of the flexor's rated torque per knuckle
TGT = [0.020, 0.004, 0.018]
R_W = [2.3, 2.5, 2.2]                          # back sector radius (axis passage is r 1.7)
X_P0 = 30.6                                    # K1's proximal dowel: outboard of the crank tips (x <= 29.2) and tackle strands (26.3)
SPAN_MAX = [18.0, 20.0, 20.0]


# ---------------------------------------------------------------- band mechanics (centreline model, exact wrap)
def _rot(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])        # in (x, z): +th curls the distal member towards +z


def band_path(dp, dd, a, h, th):
    """Hook centres P = (-dp, -h) (proximal) and Q = R(th) (dd, -h) (distal), axis at 0; band centreline wraps a circle of radius a
    when the straight line would pass inside it.  Returns (length, arm, wrap_from, wrap_to) - wrap angles in distal-frame p deg."""
    P = np.array([-dp, -h]); Q = _rot(th) @ np.array([dd, -h])
    d = Q - P; t = np.clip(-(P @ d) / (d @ d), 0, 1); foot = P + t * d; dist = np.linalg.norm(foot)
    cr = P[0] * Q[1] - P[1] * Q[0]                                     # + when the straight line passes behind the axis
    if cr > 0 and dist >= a:
        return np.linalg.norm(d), cr / np.linalg.norm(d), None, None
    lp, lq = np.linalg.norm(P), np.linalg.norm(Q)
    pd = lambda v: np.degrees(np.arctan2(v[0], v[1])) % 360.0          # p angle (from +z towards +x)
    tq = pd(Q) + np.degrees(np.arccos(a / lq)); tp = pd(P) - np.degrees(np.arccos(a / lp))
    arc = (tp - tq) % 360.0                                            # round the back, from Q's tangent to P's
    L = np.sqrt(lp * lp - a * a) + np.sqrt(lq * lq - a * a) + a * np.radians(arc)
    thd = np.degrees(th)
    return L, a, (tq + thd) % 360, (tq + arc + thd) % 360               # wrap span in the distal frame


def band_eval(j, ID, cs, mat, dp, dd, n=31):
    a = R_W[j] + cs; h = a + R_D; EA = SP.E_MOD[mat] * np.pi * cs * cs / 4
    loop0 = np.pi * (ID + cs); rows = []
    for f in np.linspace(0, H.FOLD_MAX[j], n):
        L, arm, w0, w1 = band_path(dp, dd, a, h, np.radians(f))
        e = (2 * L + 2 * np.pi * (R_D + cs / 2)) / loop0 - 1.0
        Tn = max(e, 0) * EA
        rows.append(dict(fold=round(float(f), 1), e=round(float(e), 3), T=round(float(Tn), 2), arm=round(float(arm), 3),
                         tau=round(float(2 * Tn * arm * 1e-3), 5), wrap=None if w0 is None else (round(float(w0), 1), round(float(w1), 1))))
    m = np.pi ** 2 * (ID + cs) * cs * cs / 4 * 1.15e-3                  # silicone 1.15, NBR ~1.3 (est.)
    return dict(ID=ID, cs=cs, mat=mat, dp=round(dp, 2), dd=round(dd, 2), a=a, h=h, rows=rows, mass_g=round(m, 3),
                tau0=rows[0]["tau"], tau1=rows[-1]["tau"], e_max=max(r["e"] for r in rows), e0=rows[0]["e"])


def pick(j):
    """For every stock ring, solve the hook span for the target open torque, keep the ones under the 40 % limit and 100 % strain."""
    lo, hi = SP.WINDOW[f"K{j + 1}"]; best = None
    for ID, cs, mat in SP.STOCK:
        if cs > (1.5 if j == 0 else 1.78) or (j == 2 and cs > 2 * (H.GAP[2] / 2 - 0.1)): continue   # K1: keep the band above the station blade
        for span in np.arange(9.0, SPAN_MAX[j] + 1e-9, 0.25):
            dp = (X_P0 if j == 0 else None)
            if j == 0: dp = KN[0] - X_P0; dd = span - dp
            else: dp = dd = span / 2
            if dd < 6.0: continue
            r = band_eval(j, ID, cs, mat, dp, dd)
            if not (lo <= r["tau0"] <= hi) or r["tau1"] > LIMIT[j] or r["e_max"] > 1.0 or r["e0"] < 0.15: continue
            sc = abs(r["tau0"] - TGT[j]) * 1e3 + r["tau1"] / LIMIT[j] + 0.3 * (mat == "NBR") + 0.02 * span
            if best is None or sc < best[0]: best = (sc, r)
    return best[1] if best else None


_PK = {}


def picks():
    if not _PK:
        for j in range(3): _PK[j] = pick(j)
    return _PK


# ---------------------------------------------------------------- geometry (each member's own finger frame, before pitch)
def hooks(j):
    """(x_proximal, x_distal) of knuckle j's dowels."""
    p = picks()[j]; return KN[j] - p["dp"], KN[j] + p["dd"]


def clevis(x, j):
    p = picks()[j]; zc = -p["h"]; z0 = zc - R_D - 1.0; z1 = -1.2
    m = None
    for s in (-1, 1):
        a, b = sorted((s * FIN_Y[0], s * FIN_Y[1]))
        f = box([x - FIN_X, a, z0], [x + FIN_X, b, z1]); m = f if m is None else m + f
    return m


def dowel_hole(x, j, r=R_D - 0.02):
    return cyl(r, -FIN_Y[1] - 0.5, FIN_Y[1] + 0.5, "y", (x, 0, -picks()[j]["h"]), n=24)


def channel(j, x0, x1):
    """Room for the band under a member, |y| < cs/2 + 0.15, below the back sector's radius."""
    p = picks()[j]; w = p["cs"] / 2 + 0.15
    return box([min(x0, x1) - R_D - 0.4, -w, -p["h"] - R_D - p["cs"] - 1.0], [max(x0, x1) + R_D + 0.4, w, -R_W[j] + 0.05])


def back_sector(j):
    """On the distal member of knuckle j: r_w, distal-frame p 178..278, between the knuckle's lugs, axis passage kept."""
    g2 = H.GAP[j] / 2; c = (KN[j], 0, 0)
    s = F.wedge(cyl(R_W[j], -(g2 - 0.05), g2 - 0.05, "y", c, n=64), c, 178.0, 278.0)
    return s - cyl(1.7, -g2 - 1, g2 + 1, "y", c, n=32)


def phalanx_v3(j):
    """hand3_finger.phalanx without hand3_springs' patches; phalanx 1 keeps only the middle-cord lane (k=1); plus band parts."""
    x0 = KN[j]; x1 = KN[j + 1] if j < 2 else F.RT
    m = F.loft(np.linspace(x0 - 0.6, x1 + (0.6 if j < 2 else 0), 7), F.foil)
    m -= F.pad(j, lift=0.8)
    m = F.end_trim(m, x0, 1, H.FOLD_MAX[j])
    if j < 2: m = F.end_trim(m, x1, -1, H.FOLD_MAX[j + 1])
    m = F.distal_end(m, j)
    if j < 2:
        m = F.proximal_end(m, j + 1); m -= F.cord_slot(j)
    if j == 1 and getattr(H, "COUPLED_TIP", False):
        m -= box([KN[1] + 2.5, H.CORD_Y[2] - 0.5, -1.1], [KN[1] + 9.0, H.CORD_Y[1] + 0.5, 0.45])
    for k in range(j + 1, 3):                                              # lanes: j=0 -> k=1 only (to K2); j=1 -> k=2 (to K3)
        if j == 0 and k == 2: continue
        xa = KN[j]; xb = KN[j + 1] if k > j + 1 else KN[j + 1] - H.LEVER[j + 1] - 9.0
        m -= cyl(0.5, xa - 0.5, xb + (0.5 if k > j + 1 else 0), "x", (0, H.CORD_Y[k], 0), n=16)
    # band j (this member is distal): channel, sector, clevis;  band j+1 (this member is proximal): channel, clevis
    xd = hooks(j)[1]; m -= channel(j, KN[j], xd); m += back_sector(j) + clevis(xd, j); m -= dowel_hole(xd, j)
    if j < 2:
        xp = hooks(j + 1)[0]; m -= channel(j + 1, xp, KN[j + 1]); m += clevis(xp, j + 1); m -= dowel_hole(xp, j + 1)
    return m


def grip_band(g):
    """K1's proximal clevis and band channel on the (clean) root grip, grip frame before pitch."""
    xp = hooks(0)[0]
    return g - channel(0, xp, KN[0]) + clevis(xp, 0) - dowel_hole(xp, 0)


def band_solid(j, fold=0.0):
    """Band j for display/collision, in the PROXIMAL member's frame: centreline path swept with the cs radius (two strands merged
    into one tube of width cs and depth 2 cs + 2 R_D at the dowels)."""
    p = picks()[j]; a, h, cs = p["a"], p["h"], p["cs"]; th = np.radians(fold)
    P = np.array([-p["dp"], -h]); Q = _rot(th) @ np.array([p["dd"], -h])
    L, arm, w0, w1 = band_path(p["dp"], p["dd"], a, h, th)
    pts = [P]
    if w0 is not None:
        p0 = np.radians(w0) - th; p1 = np.radians(w1) - th
        for t in np.linspace(p0 + ((p1 - p0) % (2 * np.pi)), p0, 12): pts.append(a * np.array([np.sin(t), np.cos(t)]))   # P side first
    pts.append(Q)
    m = None
    for u, v in zip(pts[:-1], pts[1:]):
        seg = M_.batch_hull([cyl(cs / 2 + (R_D + cs / 2 if (k is P or k is Q) else cs / 2), -cs / 2, cs / 2, "y", (k[0] + KN[j], 0, k[1]), n=20)
                             for k in (u, v)])
        m = seg if m is None else m + seg
    return m


def dowels(j, fold=0.0):
    """Both dowels of band j in the proximal member's frame."""
    p = picks()[j]; th = np.radians(fold); Q = _rot(th) @ np.array([p["dd"], -p["h"]])
    d = lambda x, z: cyl(R_D, -2.5, 2.5, "y", (x, 0, z), n=20)
    return d(KN[j] - p["dp"], -p["h"]) + d(KN[j] + Q[0], Q[1])


def summary():
    out = {}
    for j in range(3):
        p = picks()[j]
        if p is None: out[f"K{j + 1}"] = None; continue
        out[f"K{j + 1}"] = dict(ring=f"{p['ID']:g} x {p['cs']:g} {p['mat']}", dowels_x=[round(x, 2) for x in hooks(j)], r_w=R_W[j],
                                arm=round(p["a"], 2), tau_open=p["tau0"], tau_full=p["tau1"], limit=round(LIMIT[j], 3), e0=p["e0"],
                                e_max=p["e_max"], T_open_N=p["rows"][0]["T"], T_full_N=p["rows"][-1]["T"], mass_g=p["mass_g"],
                                spin_pull_N=round(p["mass_g"] * 1e-3 * (KN[j] + p["dd"]) * 1e-3 * W ** 2, 1),
                                wrap_deg_full=p["rows"][-1]["wrap"])
    return out


if __name__ == "__main__":
    print(json.dumps(summary(), indent=1))


def band_points(j, fold=0.0, n=6):
    """n points along band j's centreline (proximal member's frame, x-z with y = 0): P, the wrap, Q.  Padded to exactly n."""
    p = picks()[j]; a, h = p["a"], p["h"]; th = np.radians(fold)
    P = np.array([-p["dp"], -h]); Q = _rot(th) @ np.array([p["dd"], -h])
    L, arm, w0, w1 = band_path(p["dp"], p["dd"], a, h, th)
    if w0 is None:
        mids = [P + (Q - P) * t for t in np.linspace(0, 1, n)[1:-1]]
    else:
        p0 = np.radians(w0) - th; p1 = p0 + np.radians((w1 - w0) % 360)
        mids = [a * np.array([np.sin(t), np.cos(t)]) for t in np.linspace(p1, p0, n - 2)]
    pts = [P] + mids + [Q]
    return [np.array([KN[j] + q[0], 0.0, q[1]]) for q in pts]


def dowel_prox(j):
    p = picks()[j]; return cyl(R_D, -2.5, 2.5, "y", (KN[j] - p["dp"], 0, -p["h"]), n=20)


def dowel_dist(j):
    """In the distal member's own frame."""
    p = picks()[j]; return cyl(R_D, -2.5, 2.5, "y", (KN[j] + p["dd"], 0, -p["h"]), n=20)
