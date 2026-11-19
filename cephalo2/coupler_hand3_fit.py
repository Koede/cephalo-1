"""hand3 on connector rev B.1: pack-frame base for the rev B hand base, screw clocking, motor-lead path.  (2026-11-18)
hand3 frame: z = 0 at the bottom of its pack frame, +z away from the arm.  Connector frame: z into the arm, the hand
base's bottom face at z = -9.  So connector z = -9 - hand3 z, and hand3 is turned by DELTA about the axis.
hand3 rev B.1 change: the pack frame's bottom hub disc grows from r 13 to r 17.5 and gets a r 20.3 x 4.5 boss
under it (it closes the hand base's lead grooves, lifts the pack plate 1 mm clear of the collar, and takes
4 x M3 x 8 DIN 7991 countersunk screws into the hand base's inserts); the old rev A interface goes away.
python3 coupler_hand3_fit.py -> coupler_hand3_fit.json"""
import os, sys, json
import numpy as np
import manifold3d as mf
import handmount_b as HB
H3 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3")
sys.path.insert(0, H3); _cwd = os.getcwd(); os.chdir(H3)
import hand3_cad as C3
os.chdir(_cwd)

R_DISC = 17.5            # pack-frame hub disc, rev B.1 (was 13)
BOSS = (20.3, 0.0)       # rev B.2: no boss.  The collar now ends 1.2 mm ABOVE the hand base's bottom face (lugs moved to the
                         # top of the flange), so hand3's pack plate bolts on flat.  (rev B.1 needed a 4.5 mm boss here.)


def to_conn(m, delta):
    """hand3 frame -> connector frame: mirror z (z -> -9 - z) then turn by delta."""
    return m.scale([1, 1, -1]).translate([0, 0, HB.P["z_flange"][0] - BOSS[1]]).rotate([0, 0, float(delta)])


def pack_frame_b():
    m = C3.pack_frame() + C3.cyl(R_DISC, 0, 2.5, n=96)
    if BOSS[1] > 0: m += C3.cyl(BOSS[0], -BOSS[1], 0.01, n=96)
    ch = HB.P["pf_chamfer"]                                   # rev B.6: 45 deg chamfer on the centre tube's top inner edge, room for the
    cone = mf.Manifold.cylinder(ch + 0.01, 7 + ch, 7, 96).translate([0, 0, -0.01])   # motor leads' 90 deg bend into the r 7 channel
    return m - C3.cyl(7, -BOSS[1] - 1, 60) - cone


def hand_screws_h3(delta):
    """M3 x 8 DIN 7991 countersunk from the pack side, flush with the disc top (hand3 z 2.5): the tier-1 servos stand
    0.5 mm above the disc and reach in to r 14 all round, so no screw head can stand proud there (a cap head scan found
    no clocking that clears).  Head cone dia 6.0 -> 3.0 over z 2.5 .. 1.0, shank through the disc, 5.0 into the hand base insert."""
    out = []
    for az in HB.P["hand_screw_az"]:
        x, y = HB.at(HB.P["hand_screw_r"], az - delta)                  # in hand3 coordinates
        cone = mf.Manifold.cylinder(1.5, 1.5, 3.0, 24).translate([x, y, 1.0])
        out.append(cone + C3.cyl(1.45, -BOSS[1] - 5.0, 1.0, c=(x, y, 0), n=16))
    return mf.Manifold.batch_boolean(out, mf.OpType.Add)


def holes(delta):
    cut = []
    for az in HB.P["hand_screw_az"]:
        x, y = HB.at(HB.P["hand_screw_r"], az - delta)
        cut.append(C3.cyl(1.7, -BOSS[1] - 1, 3.5, c=(x, y, 0), n=24) + mf.Manifold.cylinder(1.5, 1.6, 3.1, 24).translate([x, y, 1.0]) + C3.cyl(3.1, 2.5, 3.5, c=(x, y, 0), n=24))
    return mf.Manifold.batch_boolean(cut, mf.OpType.Add)


def fit(delta):
    hw = C3.hardware()
    fixed = [m for k, (m, rot, mat) in hw.items() if k.startswith("servo") or k.startswith("motor_base")]
    spools = [m for k, (m, rot, mat) in hw.items() if k.startswith("spool")]
    S = hand_screws_h3(delta)
    serv = mf.Manifold.batch_boolean(fixed, mf.OpType.Add); spl = mf.Manifold.batch_boolean(spools, mf.OpType.Add)
    pf = pack_frame_b() - holes(delta)
    return dict(screw_heads_x_servos=float((S ^ serv).volume()), screw_heads_x_spools=float((S ^ spl).volume()),
                screws_x_pack_frame=float((S ^ pf).volume())), pf


if __name__ == "__main__":
    res = {}
    scan = []
    for d in np.arange(0, 90, 1.0):
        r, _ = fit(d); scan.append((float(d), round(r["screw_heads_x_servos"] + r["screw_heads_x_spools"] + r["screws_x_pack_frame"], 2)))
    ok = [d for d, v in scan if v < 1e-3]
    res["clocking scan: hand3 turned by delta (deg) -> screw overlap with servos/spools/frame (mm3)"] = scan
    # pick the delta in the middle of the widest clear window
    best, run, cur = None, [], []
    for d, v in scan + [(999, 1)]:
        if v < 1e-3: cur.append(d)
        else:
            if len(cur) > len(run): run = cur
            cur = []
    DELTA = (0.0 if 0.0 in run else float(np.mean(run))) if run else None      # keep hand3's own clocking when the screws allow it
    res["clear window (deg)"] = [run[0], run[-1]] if run else None
    res["DELTA chosen (deg)"] = DELTA
    if DELTA is not None:
        r, pf = fit(DELTA); res["at DELTA"] = r
        Hb = HB.hand_base(); Mb = mf.Manifold.batch_boolean(HB.male_bullets(), mf.OpType.Add)
        PFc = to_conn(pf, DELTA)
        res["pack frame (rev B.1) x hand base / male bullets (mm3)"] = [float((PFc ^ Hb).volume()), float((PFc ^ Mb).volume())]
        hw = C3.hardware(); allhw = mf.Manifold.batch_boolean([m for (m, _, _) in hw.values()], mf.OpType.Add)
        HWc = to_conn(allhw, DELTA)
        Ccol = HB.collar(0); S = HB.arm_socket(); N = HB.nose_ring()
        res["hand3 hardware x hand base / collar / socket / nose ring (mm3)"] = [float((HWc ^ Hb).volume()), float((HWc ^ Ccol).volume()), float((HWc ^ S).volume()), float((HWc ^ N).volume())]
        res["hand3 pack frame x collar / socket / nose ring (mm3)"] = [float((PFc ^ Ccol).volume()), float((PFc ^ S).volume()), float((PFc ^ N).volume())]
        # lead path: grooves under the hand base end in the r 6.5 pocket, which lines up with the spine bore (r 7)
        res["gap: collar bottom to hand3 pack plate (mm)"] = round(HB.P["collar_z"][0] - (HB.P["z_flange"][0] - BOSS[1]), 2)
        Co = HB.collar(-HB.P["twist"]); res["collar OPEN x hand3 pack frame / hardware (mm3)"] = [float((PFc ^ Co).volume()), float((HWc ^ Co).volume())]
        res["boss radius vs collar lip inner radius (mm)"] = [BOSS[0], HB.P["lip_r"][0]]
        res["hand3 moves outboard along the tip axis vs. sitting on the hand base directly (mm)"] = BOSS[1]
        res["lead path: hand-base pocket r / hand3 spine bore r (mm)"] = [HB.P["r_wire_cb"], 7.0]
        res["pack frame rev B.1 mass (g, PA-CF 1.25)"] = round(pf.volume() / 1000 * 1.25, 1)
        res["pack frame mass added by the disc (g)"] = round((pf.volume() - (C3.pack_frame() - C3.cyl(7, -1, 60)).volume()) / 1000 * 1.25, 2)
        json.dump(res, open("coupler_hand3_fit.json", "w"), indent=1)
        import trimesh
        me = pf.to_mesh(); trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)).export("handmount_b_stl/hand3_pack_frame_revB1_PA-CF.stl")
    for k, v in res.items(): print(" ", k, v)
