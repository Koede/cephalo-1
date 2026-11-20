"""hand3 hub v3, decision C (2026-11-20): every finger root drives itself.

Each root carries a self-locking N20 worm gearmotor (E-S Motor 1220WG-N20VA, 6 V, 94 rpm, stall 2.4 kg cm; gearbox 18 x 12 x 10,
33 mm long; ~13 g) standing upright in a cradle on the root's outer end.  Its output shaft points at the hub axis and carries a
brass m0.5 z22 pinion that meshes a crown ring printed on the top disc's top face (74 teeth, pitch r 18.5).  So every root walks
round the hub on its own, all four at once, and the worm holds it in flight.  The latch pins, cranks, crank pins and the
station blade are gone; the discs lose their pin slots and pockets.
Power reaches the rotor only while it is parked: the station carriage (SG92R, as before) pushes two Adafruit 5358 4-contact pogo
halves onto 8 copper pads on the cage at the same time as the roller comes on.  From the pads 8 wires (est. routing) run to 8
copper-tape tracks set flush in the top disc's underside; each root has two pogo brushes in its top face on its own pair of
tracks.  2 x Adafruit DRV8833 on the static side drive the 4 motors.  The QTR-1A sensor stays (park index + interlock).
Root frame: x radial along the finger (az 0), z up.  Units mm."""
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_hub_v3 as V3
import hand3_hubc as HC
from hand3_finger import cyl, box, app, T, rot

M_ = mf.Manifold
V = V3.V
Z_GAP, Z_TOP = V3.Z_GAP, V3.Z_TOP                                          # 93..99, 99..101
CROWN = dict(m=0.5, n=74, r=(16.5, 20.5), z=(Z_TOP[1], Z_TOP[1] + 1.1))       # face-gear teeth on the top disc's top face
Z_PITCH = (CROWN["z"][0] + CROWN["z"][1]) / 2                                  # 101.55
PIN = dict(n=22, m=0.5, r_p=5.5, r_o=6.0, x=(16.3, 20.3), bore=1.5)            # brass m0.5 z22 pinion, 4 wide, on the 3 mm D shaft
Z_SHAFT = Z_PITCH + PIN["r_p"]                                                 # 107.05
WORM = dict(name="E-S Motor 1220WG-N20VA 6 V 94 rpm (self-locking worm)", m=13.0, gb=(10.0, 12.0, 18.0), can_r=6.0, can_L=15.0,
            shaft_h=5.0, shaft_L=8.0, rpm=94, stall_kgcm=2.4, stall_A=0.65)    # m: Chihai CHF-GW12T-N20VA 'approx. 13 g' (same body)
GB_X = (21.5, 21.5 + WORM["gb"][0])                                            # gearbox radial span (outer face 31.5)
GB_Z0 = Z_SHAFT - WORM["shaft_h"]                                              # 102.05: 1.05 above the top disc
CRADLE = dict(x=(20.5, 33.0), y=7.5, z1=GB_Z0 + 18.0, wall=1.0)                # printed tube round the gearbox only (the can sits above)
TRACK_R = (10.2, 11.4, 12.6, 16.4, 17.6, 18.8, 20.0, 21.2)                     # copper-tape tracks under the top disc (ribs at 13.5-15.5, 22.3-24)
TRACK_W = 0.8
PADS = dict(az_off=24.0, z=(61.2, 64.0), pitch_deg=360 * 2.54 / (2 * np.pi * HC.R_CAGE[1]))   # 2 rows of 4 pads, 0.1" pitch, on the cage


def gearbox_and_can():
    gx, gy, gz = WORM["gb"]
    gb = box([GB_X[0], -gy / 2, GB_Z0], [GB_X[1], gy / 2, GB_Z0 + gz])
    can = cyl(WORM["can_r"], GB_Z0 + gz, GB_Z0 + gz + WORM["can_L"], c=((GB_X[0] + GB_X[1]) / 2, 0, 0), n=40)
    shaft = cyl(1.5, GB_X[0] - WORM["shaft_L"], GB_X[0] + 0.01, "x", (0, 0, Z_SHAFT), n=20)
    return gb + can + shaft


def pinion():
    return cyl(PIN["r_o"], *PIN["x"], "x", (0, 0, Z_SHAFT), n=44) - cyl(PIN["bore"], PIN["x"][0] - 1, PIN["x"][1] + 1, "x", (0, 0, Z_SHAFT), n=16)


def crown_teeth():
    """Straight-sided radial teeth (half the angular pitch wide) - enough for fits and collisions; the real ring is cut with a
    face-gear profile for the z22 pinion."""
    p = 360.0 / CROWN["n"]; out = []
    for k in range(CROWN["n"]):
        a = k * p
        out.append(V3.wedge(CROWN["r"][0], CROWN["r"][1], CROWN["z"][0] - 0.01, CROWN["z"][1], a - p / 4, a + p / 4, n=8))
    return M_.batch_boolean(out, mf.OpType.Add)


def top_disc_c():
    m = V3.top_disc() + crown_teeth()
    for r in TRACK_R:                                                           # tape grooves (tape 0.1 mm sits flush)
        m -= cyl(r + TRACK_W / 2, Z_TOP[0] - 1, Z_TOP[0] + 0.12, n=128) - cyl(r - TRACK_W / 2, Z_TOP[0] - 2, Z_TOP[0] + 2, n=128)
    return m


def tracks():
    m = None
    for r in TRACK_R:
        t = cyl(r + TRACK_W / 2, Z_TOP[0] + 0.02, Z_TOP[0] + 0.1, n=128) - cyl(r - TRACK_W / 2, Z_TOP[0], Z_TOP[0] + 0.2, n=128)
        m = t if m is None else m + t
    return m


def base_disc_c():
    """v3 base disc without the latch-pin lip slots and pockets."""
    zt = V["z0"] + V["disc_t"]
    m = cyl(V["disc_r"], V["z0"], zt, n=128) + cyl(V["boss_r"], V["boss_z0"], Z_GAP[1], n=64)
    for r0, r1 in V["ribs"]:
        m -= cyl(r1 + V["groove_clr"], zt - V["rib_h"] - V["groove_clr"], zt + 1, n=128) - cyl(r0 - V["groove_clr"], zt - 2, zt + 2, n=128)
    m -= cyl(V["shaft_r"], V["boss_z0"] - 1, Z_GAP[1] + 1, n=32)
    for k in range(V["top_screw_n"]):
        a = np.radians(90 + 360 / V["top_screw_n"] * k); m -= cyl(0.8, Z_GAP[1] - 5, Z_GAP[1] + 1, c=(V["top_screw_r"] * np.cos(a), V["top_screw_r"] * np.sin(a), 0), n=16)
    return m


def cradle():
    """Printed tube round the gearbox, on the root's outer end.  Over the top disc (x < 25.8) it stops 0.3 above the disc; from
    25.8 out it runs down into the grip.  Inner wall has the shaft hole; the flexor channel is cut through it (see root_block_c)."""
    x0, x1 = CRADLE["x"]; y = CRADLE["y"]; z1 = CRADLE["z1"]; w = CRADLE["wall"]
    outer = box([x0, -y, Z_TOP[1] + 0.3], [x1, y, z1]) + box([25.8, -y, H.Z["blade"]], [x1, y, z1])
    inner = box([GB_X[0] - 0.2, -WORM["gb"][1] / 2 - 0.2, GB_Z0 - 0.01], [GB_X[1] + 0.2, WORM["gb"][1] / 2 + 0.2, z1 + 1])
    m = outer - inner - cyl(1.9, x0 - 1, GB_X[0] + 0.5, "x", (0, 0, Z_SHAFT), n=20)
    g2 = H.GAP[0] / 2                                                            # K1's clearance for the flexor sector (as the grip's)
    m -= app(cyl(H.LEVER[0] + 1.0, -(g2 + 0.8), g2 + 0.8, "y", (H.KN[0], 0, 0), n=48), T(rot("x", H.PITCH))).translate([0, 0, H.Z["blade"]])
    return m - box([x0 - 1, -y - 1, GB_Z0 + 6], [GB_X[0] - 0.1, y + 1, z1 + 1])    # inner wall only up to the shaft (pinion side open above)


_RB = {}


def root_block_c(az=0.0):
    """v3 root block without latch-pin bores, arm slots or crank cuts; plus the motor cradle and two brush bores."""
    if "m" not in _RB:
        zb = H.Z["blade"]
        grip = app(V3.grip_clean(), T(t=[0, 0, zb]))
        b = V3.wedge(V["block_r"][0], V["block_r"][1], *Z_GAP, -V["block_half"], V["block_half"])
        for r0, r1 in V["ribs"]:
            b += V3.wedge(r0, r1, Z_GAP[0] - V["rib_h"], Z_GAP[0] + 0.01, -V["block_half"], V["block_half"])
            b += V3.wedge(r0, r1, Z_GAP[1] - 0.01, Z_GAP[1] + V["rib_h"], -V["block_half"], V["block_half"])
        m = b + grip + cradle()
        for s in (-1, 1):                                                       # 2:1 tackle strands (as v3)
            m -= cyl(0.6, Z_GAP[0] - 3, zb + (1.0 if s > 0 else 6.0), c=(V["bead"]["eye_r"], s * 0.7, 0), n=16)
        m -= cyl(1.1, zb + 1.4, zb + 6.0, c=(V["bead"]["eye_r"], -0.7, 0), n=20)
        m -= V3.flexor_channel_world() if hasattr(V3, "flexor_channel_world") else _flexor_world()
        _RB["m"] = m
    return _RB["m"].rotate([0, 0, float(az)])


def _flexor_world():
    """The running strand's channel in root (world at az 0) coordinates, extended up through the cradle wall."""
    return app(V3.flexor_channel(), T(rot("x", H.PITCH))).translate([0, 0, H.Z["blade"]])


def brushes(k):
    """Root k's two pogo brushes (dia 1, in the block top) on tracks 2k, 2k+1."""
    m = None
    for r in TRACK_R[2 * k: 2 * k + 2]:
        b = cyl(0.5, Z_GAP[1] - 4.0, Z_TOP[0] + 0.02, c=(r, 0, 0), n=12); m = b if m is None else m + b
    return m


def brush_bores(k):
    m = None
    for r in TRACK_R[2 * k: 2 * k + 2]:
        b = cyl(0.65, Z_GAP[1] - 4.5, Z_GAP[1] + 0.5, c=(r, 0, 0), n=12); m = b if m is None else m + b
    return m


def root_c(k, az=0.0):
    """Root block of root k (brush bores at its own track radii)."""
    return (root_block_c(0.0) - brush_bores(k)).rotate([0, 0, float(az)])


def pads():
    """8 copper pads (2 rows of 4) on the cage, rotor frame, centred PADS['az_off'] deg from the station's line when parked."""
    m = None; r = HC.R_CAGE[1]
    for row, z in enumerate(PADS["z"]):
        for i in range(4):
            a = np.radians(PADS["az_off"] + (i - 1.5) * PADS["pitch_deg"])
            p = cyl(0.9, z - 0.8, z + 0.8, "x", (0, 0, 0), n=12) ^ box([r - 0.05, -2, z - 2], [r + 0.08, 2, z + 2])
            p = box([r - 0.02, -0.9, z - 0.8], [r + 0.08, 0.9, z + 0.8]).rotate([0, 0, float(np.degrees(a))])
            m = p if m is None else m + p
    return m


def dock_head():
    """Static: two Adafruit 5358 pogo halves on an arm of the station carriage, retracted 1.5 mm (the carriage's roll gap).
    In the station frame (x = radial at STATION_AZ)."""
    r = HC.R_CAGE[1]; a = np.radians(PADS["az_off"]); c, s = np.cos(a), np.sin(a)
    head = box([r + 0.6 + V3.ST["roll_gap"], -4.6, PADS["z"][0] - 1.6], [r + 6.0 + V3.ST["roll_gap"], 4.6, PADS["z"][1] + 1.6])    # 2 x (10.2 x 5) connector bodies (est.)
    head = head.rotate([0, 0, float(PADS["az_off"])])
    arm = box([r + 5.5 + V3.ST["roll_gap"], 2.0, 58.0], [40.0, 7.0, 60.0])    # under the tyre band to the carriage spine (est.)
    return head + arm


def station_c():
    """Station for decision C: roller (tyre + N20), SG92R carriage, QTR-1A, dock head.  Blade gone."""
    sp = V3.station_parts()
    return dict(tyre=sp["tyre"], motor=sp["motor"], servo=sp["servo"], sensor=sp["sensor"].translate([-10.0, 0, -22.0]), dock=dock_head())


def drive_numbers():
    v = WORM["rpm"] / 60 * 2 * np.pi * PIN["r_p"]                              # pinion pitch speed mm/s
    degs = np.degrees(v / (2 * np.pi * 18.5) * 2 * np.pi)                      # root angular speed (deg/s): pitch radius 18.5
    T_st = WORM["stall_kgcm"] * 9.81e-2; F_st = T_st / (PIN["r_p"] * 1e-3)
    W = 1309.0; m_tow = WORM["m"] + 2.6                                          # + pinion (1.1 g brass, est.) + cradle share (est.)
    r_cm = 26.5; F_c = m_tow * 1e-3 * r_cm * 1e-3 * W ** 2
    h = (GB_Z0 + 13.0) - H.Z["blade"]                                           # tower CoM above the grip mid-plane (est.)
    Mb = F_c * h * 1e-3
    d, b = CRADLE["x"][1] - 25.8, 2 * CRADLE["y"]; S = b * d * d / 6
    return dict(root_speed_deg_s=round(degs, 1), pinion_force_stall_N=round(F_st), hold_needed_N=6.0,
                tower_g=m_tow, tower_pull_N=round(F_c), root_radial_N=round(826 + F_c), cradle_base_moment_Nm=round(Mb, 2),
                cradle_base_section=f"{d:.1f} x {b:.0f} mm", cradle_base_stress_MPa=round(Mb * 1e3 / S, 1),
                cradle_SF_PAHT_CF_wet54=round(54 / (Mb * 1e3 / S), 2), cradle_SF_AlSi10Mg_200=round(200 / (Mb * 1e3 / S), 2),
                rib_bearing_MPa=round((826 + F_c) / 77.0, 1), motor_g_load=round(r_cm * 1e-3 * W ** 2 / 9.81))
