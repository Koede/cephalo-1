"""XL claw-prop render meshes from the real Snap-Digit CAD (snapdigit.py, the adopted claw-prop), scaled 1.25x as in
xl-claw.md, on the T-Motor F90 2806.5 (xlclaw.py stator).  (Local copy of claude/code/xlclaw_cad.py.)

Frames match the MuJoCo bodies in xlclaw.claw_xml: prox = blade frame moved to the root pin, dist = blade frame moved to
the fitted knuckle pivot XL.PIV, rotor = pod frame, slider = skirt top.  Units: metres.
python3 xlclaw_cad.py -> xlclaw_cad.npz (cache)
"""
import os
import numpy as np
import xlclaw as XL

S = XL.SC * 1e-3                       # 6 in model mm -> XL metres
CACHE = "xlclaw_cad.npz"
_M = None


def _tris(m):
    me = m.to_mesh()
    V = np.asarray(me.vert_properties)[:, :3].astype(float); F = np.asarray(me.tri_verts)
    return V[F]


def _ribbon(P, nrm, w):
    """Flat strip through centre-line points P (k, 3) with face normal nrm (k, 3) and width w (k,)."""
    t = np.gradient(P, axis=0); t /= np.linalg.norm(t, axis=1, keepdims=True)
    b = np.cross(nrm, t); b /= np.linalg.norm(b, axis=1, keepdims=True)
    L = P - b * (w[:, None] / 2); R = P + b * (w[:, None] / 2)
    out = []
    for i in range(len(P) - 1):
        out += [[L[i], R[i], R[i + 1]], [L[i], R[i + 1], L[i + 1]]]
    return np.array(out)


def tape_width(r):
    import snapdigit as SD
    return np.where(r < SD.R_K0 - 1, 12.7, np.where((r > SD.R_K0 + 1.5) & (r < SD.R_K1 - 1.5), SD.TAPE_WAIST, SD.TAPE_W))


def build():
    import snapdigit as SD
    P = SD.build_parts()
    pin = np.array([SD.R_PIN, 0, SD.Z_PIN])
    piv = XL.PIV                                   # fitted single pivot of the tape fold (6 in mm, blade frame)
    prox = (_tris(P["proximal"]) - pin) * S
    dist = (_tris(P["distal"]) - piv) * S
    rotor = np.concatenate([_tris(P["hub"]), _tris(P["palm_cap"]),
                            _tris(SD.cyl([0, 0, SD.HUB_Z[1] + 2.0], [0, 0, SD.HUB_Z[1] + 6.5], 3.9, 6))]) * S      # M5 nyloc prop nut
    zr = -0.5                                      # bearing ring 0.5 mm under the skirt (slider frame)
    ring = SD.apply(SD.ring(), SD.tf(t=[0, 0, zr]))
    brg = SD.cyl([0, 0, zr - SD.BRG["w"]], [0, 0, zr], SD.BRG["ro"], 56) - SD.cyl([0, 0, -40], [0, 0, 10], SD.BRG["ri"] + 0.05, 56)
    rods = []
    for k in range(2):
        a = np.radians(90 + 180 * k); e = np.array([np.cos(a), np.sin(a), 0])
        rods.append(_tris(SD.cyl(e * SD.ROD_R + [0, 0, zr - SD.BRG["w"] - 0.5], e * SD.ROD_R + [0, 0, zr - 32.0], SD.ROD_D / 2, 10)))
    slider = np.concatenate([_tris(P["skirt"]), _tris(ring), _tris(brg)] + rods) * S
    tp = SD.tape_points(0.0, 60)
    r = tp[:, 0]; nrm = np.stack([SD.nrm_at(x) for x in r]); w = tape_width(r)
    lift = nrm * 0.15
    m0 = r <= SD.R_K0 + 0.01; m1 = r >= SD.R_K1 - 0.01
    tape_p = (_ribbon(tp[m0] + lift[m0], nrm[m0], w[m0]) - pin) * S
    tape_d = (_ribbon(tp[m1] + lift[m1], nrm[m1], w[m1]) - piv) * S
    st = XL.meshes()["stator"]
    out = dict(prox=prox, dist=dist, rotor=rotor, slider=slider, stator=st, tape_p=tape_p, tape_d=tape_d,
               k0=np.r_[(SD.P_K0 - pin) * S, [1, 0, 0], SD.N_K],
               k1=np.r_[((SD.knuckle_T(0.0) @ np.r_[SD.P_K0 + SD.LF * SD.T_K, 1])[:3] - piv) * S, [1, 0, 0], SD.N_K])
    np.savez_compressed(CACHE, **out)
    return out


def meshes():
    """Keys: prox, dist, rotor, slider, stator + the tape parts."""
    global _M
    if _M is None:
        _M = dict(np.load(CACHE)) if os.path.exists(CACHE) else build()
    return _M


def free_tape(Tp, Td, n=14):
    """World-frame ribbon of the free tape between the proximal (pose Tp = (R, p)) and the distal (Td)."""
    m = meshes()
    (Rp, pp), (Rd, pd) = Tp, Td
    a0, t0, n0 = Rp @ m["k0"][:3] + pp, Rp @ m["k0"][3:6], Rp @ m["k0"][6:]
    a1, t1, n1 = Rd @ m["k1"][:3] + pd, Rd @ m["k1"][3:6], Rd @ m["k1"][6:]
    L = np.linalg.norm(a1 - a0) / 3 + 1e-6
    s = np.linspace(0, 1, n)[:, None]
    c0, c1 = a0 + t0 * L, a1 - t1 * L
    P = (1 - s) ** 3 * a0 + 3 * (1 - s) ** 2 * s * c0 + 3 * (1 - s) * s ** 2 * c1 + s ** 3 * a1
    nrm = (1 - s) * n0 + s * n1
    return _ribbon(P + nrm * 0.15e-3, nrm, np.full(n, 6.0 * XL.SC * 1e-3))


TAPE_COL = (0.86, 0.72, 0.16)                      # painted tape-measure steel


def install():
    XL.meshes = meshes


if __name__ == "__main__":
    import time
    t0 = time.time(); m = build()
    print({k: v.shape for k, v in m.items()}, f"{time.time()-t0:.0f} s")
