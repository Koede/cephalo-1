"""Printable detent-cam clutch for one servo joint (sized for the STS3215 at j5/j7), plus per-joint parameter table.

Mechanism (per joint): the servo horn carries a CAM DISK. A ROCKER pivots on the link (M3 shoulder bolt). Its short
arm carries an MR63 roller that sits in the cam's V; its long arm carries an MR63 idler that the arm's central
tension line runs under, deflected by ALPHA. Line tension T pushes the idler -> rocker presses the roller into the V.
  roller force       P = 2 T sin(ALPHA/2) * (L_idler / L_roller)
  breakaway torque   M = P * K_CAM             (cam rise K_CAM = dr/dphi, constant over +-40 deg)
Past +-40 deg the cam meets a printed stop. When the load drops, P pushes the roller back down the V: it springs back.
Hardware: 1 x 624ZZ roller on M4x12, 1 x 623ZZ idler on M3x12, 1 x M3x16 shoulder bolt + nyloc (pivot),
6800ZZ (10x19x5) carrying the cam disk on the link so the roller force does not side-load the servo shaft,
4 x M2.5 or M3 screws into the stock round horn (measure your horn's bolt circle; HORN_BC below is a placeholder).
"""
import numpy as np, trimesh, manifold3d as m3d

ALPHA = np.radians(90)          # line deflection over the idler (wraps a quarter turn)
T_FIRM = 60.0                   # N, winch 'firm' setting
PHI_MAX = np.radians(40)        # cam travel each way
HORN_BC, HORN_HOLE, HORN_BORE = 14.0, 2.9, 8.5   # mm, placeholder: measure the STS3215 horn


def joint_params(stall, frac=0.9, k_cam_mm=None, l_roller=10.0):
    M = frac * stall                                   # N m breakaway at T_FIRM
    k = (k_cam_mm or (10.0 if stall > 2 else 4.0)) / 1000.0
    P = M / k
    Fl = 2 * T_FIRM * np.sin(ALPHA / 2)
    ratio = P / Fl
    return dict(M=M, k_cam_mm=k * 1000, P=P, ratio=ratio, l_roller=l_roller, l_idler=ratio * l_roller,
                rise_mm=k * 1000 * PHI_MAX)


SERVOS = {"STS3215 (j3-j5, j7)": 2.9, "STS3032 (j6, j8)": 0.44}     # root joints j0-j2 stay rigid (peaks <= 0.6 of limit)


def cam_disk(stall=2.9, R0=14.0, thick=6.0):
    p = joint_params(stall)
    k = p["k_cam_mm"]
    pts = []
    for a in np.linspace(-np.pi, np.pi, 361)[:-1]:
        ph = abs(a)
        if ph <= PHI_MAX:
            r = R0 + k * ph
        elif ph <= PHI_MAX + np.radians(12):           # stop lobe
            r = R0 + k * PHI_MAX + 4.0
        else:
            r = R0 + k * PHI_MAX + 4.0 - 2.0 * min(1, (ph - PHI_MAX - np.radians(12)) / np.radians(20))
        pts.append((r * np.cos(a), r * np.sin(a)))
    cs = m3d.CrossSection([pts])
    body = m3d.Manifold.extrude(cs, thick)
    body = body - m3d.Manifold.cylinder(thick + 2, HORN_BORE / 2, HORN_BORE / 2, 48).translate((0, 0, -1))
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        body = body - m3d.Manifold.cylinder(thick + 2, HORN_HOLE / 2 + 0.15, HORN_HOLE / 2 + 0.15, 24).translate(
            (HORN_BC / 2 * np.cos(a), HORN_BC / 2 * np.sin(a), -1))
    return body, p


def rocker(p, thick=8.0):
    """Roller: 624ZZ (4x13x5, C0 ~0.34 kN) on an M4 axle; pivot: M3 shoulder bolt; idler: 623ZZ (3x10x4) on M3."""
    lr, li = p["l_roller"], p["l_idler"]
    def disc(x, r):
        return m3d.CrossSection.circle(r, 48).translate((x, 0))
    cs = m3d.CrossSection.hull(m3d.CrossSection.batch_boolean([disc(-lr, 9.0), disc(0, 5.0), disc(li, 7.5)], m3d.OpType.Add))
    body = m3d.Manifold.extrude(cs, thick)
    for x, r in ((-lr, 2.15), (0.0, 1.65), (li, 1.65)):                 # M4 / M3 / M3 clearance
        body = body - m3d.Manifold.cylinder(thick + 2, r, r, 32).translate((x, 0, -1))
    body = body - m3d.Manifold.cylinder(5.2, 6.55, 6.55, 64).translate((-lr, 0, thick - 5.1))   # 624ZZ pocket
    body = body - m3d.Manifold.cylinder(4.2, 5.05, 5.05, 64).translate((li, 0, thick - 4.1))    # 623ZZ pocket
    return body


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


SERVOS = {"STS3215 (j3-j8)": 2.9}     # adopted set: all distal joints STS3215; root clutches use root_params()


# ---------------------------------------------------------------- adopted set (2026-11-08)
# Clutch slip torque at the 60 N 'firm' line tension, as a fraction of each joint's servo stall:
#   j0, j2 (STS3250 root pitch): 1.2 x  (never slips under steady loads, only on impact spikes), cam +-25 deg
#   j1 (root yaw): rigid
#   j3-j8 (all STS3215): 0.9 x, cam +-40 deg
ADOPTED = {"j0": ("STS3250", 4.9, 1.2), "j2": ("STS3250", 4.9, 1.2), "j3": ("STS3215", 2.9, 0.9), "j4": ("STS3215", 2.9, 0.9),
           "j5": ("STS3215", 2.9, 0.9), "j6": ("STS3215", 2.9, 0.9), "j7": ("STS3215", 2.9, 0.9), "j8": ("STS3215", 2.9, 0.9)}


def root_params():
    p = joint_params(4.9, frac=1.2, k_cam_mm=16.0, l_roller=10.0)
    p["rise_mm"] = 16.0 * np.radians(25)
    return p


def cam_disk_root(R0=18.0, thick=7.0):
    """STS3250 root cam: 16 mm/rad over +-25 deg, roller 626ZZ (6x19x6)."""
    k, pm = 16.0, np.radians(25)
    pts = []
    for a in np.linspace(-np.pi, np.pi, 361)[:-1]:
        ph = abs(a)
        r = R0 + k * min(ph, pm) + (4.0 if ph > pm else 0.0) - (2.0 if ph > pm + np.radians(15) else 0.0)
        pts.append((r * np.cos(a), r * np.sin(a)))
    body = m3d.Manifold.extrude(m3d.CrossSection([pts]), thick)
    body = body - m3d.Manifold.cylinder(thick + 2, 5.0, 5.0, 48).translate((0, 0, -1))
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        body = body - m3d.Manifold.cylinder(thick + 2, 1.6, 1.6, 24).translate((9 * np.cos(a), 9 * np.sin(a), -1))   # placeholder horn holes
    return body


def rocker_root(p, thick=9.0):
    lr, li = p["l_roller"], p["l_idler"]
    def disc(x, r):
        return m3d.CrossSection.circle(r, 48).translate((x, 0))
    cs = m3d.CrossSection.hull(m3d.CrossSection.batch_boolean([disc(-lr, 11.5), disc(0, 5.5), disc(li, 7.5)], m3d.OpType.Add))
    body = m3d.Manifold.extrude(cs, thick)
    for x, r in ((-lr, 3.15), (0.0, 1.65), (li, 1.65)):                 # M6 / M3 / M3
        body = body - m3d.Manifold.cylinder(thick + 2, r, r, 32).translate((x, 0, -1))
    body = body - m3d.Manifold.cylinder(6.2, 9.55, 9.55, 64).translate((-lr, 0, thick - 6.1))    # 626ZZ pocket
    body = body - m3d.Manifold.cylinder(4.2, 5.05, 5.05, 64).translate((li, 0, thick - 4.1))     # 623ZZ pocket
    # lightening slot in the long arm
    if li > 30:
        body = body - m3d.Manifold.cube((li - 22, 5, thick + 2)).translate((8, -2.5, -1))
    return body


def lighten_disk(body, R_in, R_out, thick, n=5, ang0=np.pi):
    """Cut n round pockets in the back half of a cam disk (away from the V and the stops)."""
    rm = 0.5 * (R_in + R_out); rr = 0.32 * (R_out - R_in)
    for i in range(n):
        a = ang0 + (i - (n - 1) / 2) * np.radians(32)
        body = body - m3d.Manifold.cylinder(thick + 2, rr, rr, 32).translate((rm * np.cos(a), rm * np.sin(a), -1))
    return body


def winch_parts():
    """Root winch: 1220WG-N20VA-class self-locking worm gearmotor (gearbox 18x12x10, 39 mm long, 3 mm D shaft;
    2.4 kg cm stall), Ø6 mm spool (r 3 mm), 4:1 Dyneema block (2 x 623ZZ moving, 1 x 623ZZ fixed).
    Firm 60 N line -> 15 N at the spool -> 0.45 kg cm; tight 150 N -> 1.1 kg cm (worm holds it unpowered)."""
    # motor cradle with 2 x M2 strap holes and a base flange with 2 x M3 to the mantle sector
    cradle = m3d.Manifold.cube((44, 16, 3)).translate((-2, -8, 0))
    walls = m3d.Manifold.cube((22, 2.5, 14)).translate((0, -8, 3)) + m3d.Manifold.cube((22, 2.5, 14)).translate((0, 5.5, 3))
    cradle = cradle + walls
    for x in (4, 18):
        cradle = cradle - m3d.Manifold.cylinder(20, 1.1, 1.1, 16).rotate((90, 0, 0)).translate((x, 10, 13))
    for x in (30, 38):
        cradle = cradle - m3d.Manifold.cylinder(5, 1.65, 1.65, 24).translate((x, 0, -1))
    spool = m3d.Manifold.cylinder(10, 3.0, 3.0, 48) + m3d.Manifold.cylinder(1.5, 7.0, 7.0, 48) + \
        m3d.Manifold.cylinder(1.5, 7.0, 7.0, 48).translate((0, 0, 8.5))
    dshaft = m3d.Manifold.cylinder(12, 1.55, 1.55, 32) - m3d.Manifold.cube((4, 4, 12)).translate((1.0, -2, 0))
    spool = spool - dshaft.translate((0, 0, -1))
    spool = spool - m3d.Manifold.cylinder(14, 0.9, 0.9, 16).rotate((90, 0, 0)).translate((0, 7, 5))     # line tie hole
    # moving pulley block: two 623ZZ side by side on M3, strap eye for the arm line
    blk = m3d.Manifold.cube((26, 12, 6)).translate((-13, -6, 0))
    for x in (-6.5, 6.5):
        blk = blk - m3d.Manifold.cylinder(8, 1.65, 1.65, 24).translate((x, 0, -1))
        blk = blk - m3d.Manifold.cylinder(4.2, 5.05, 5.05, 48).translate((x, 0, 6 - 4.1))
    blk = blk + (m3d.Manifold.cylinder(6, 5.0, 5.0, 32) - m3d.Manifold.cylinder(8, 2.0, 2.0, 24).translate((0, 0, -1))).translate((18, 0, 0))
    return cradle, spool, blk


def export_adopted(outdir="stl_clutch"):
    import os, json
    os.makedirs(outdir, exist_ok=True)
    PETG, EFF = 1.27, 0.65          # g/cm3, effective fill for 4 walls + 40 % gyroid (est.)
    parts = {}
    disk, p215 = cam_disk(2.9)
    disk = lighten_disk(disk, 10.0, 20.0, 6.0)
    parts["clutch_cam_STS3215"] = (disk, 6)
    parts["clutch_rocker_STS3215"] = (rocker(p215), 6)
    pr = root_params()
    parts["clutch_cam_STS3250_root"] = (lighten_disk(cam_disk_root(), 13.0, 25.0, 7.0), 2)
    parts["clutch_rocker_STS3250_root"] = (rocker_root(pr), 2)
    c, s, b = winch_parts()
    parts["winch_cradle_N20worm"] = (c, 1); parts["winch_spool_6mm"] = (s, 1); parts["winch_block_4to1"] = (b, 1)
    report = {}
    for nm, (man, qty) in parts.items():
        tm = to_trimesh(man)
        tm.export(f"{outdir}/{nm}.stl")
        report[nm] = dict(qty_per_arm=qty, watertight=bool(tm.is_watertight), extents_mm=np.round(tm.extents, 1).tolist(),
                          mass_g=round(tm.volume / 1000 * PETG * EFF, 1))
    report["_params"] = {"STS3215 joints (j3-j8)": {k: round(float(v), 2) for k, v in p215.items()},
                         "STS3250 root (j0, j2)": {k: round(float(v), 2) for k, v in pr.items()}}
    json.dump(report, open(f"{outdir}/adopted_parts.json", "w"), indent=1)
    return report


if __name__ == "__main__":
    import json
    print(json.dumps(export_adopted(), indent=1))
