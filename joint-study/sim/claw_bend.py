"""Bent claw for the 16 in claw fan (Forest, Nov 9: "if it gets too big to grasp with, add a bend halfway through").

Layout (concept, not CAD yet): the fan is fixed on the claw axis, motor base on the arm's tip plate, prop outermost, so
its push runs along the arm's last link and adds no moment at the tip joints (a 16 in disc is too big for the old
pan-tilt wrist: at the 60 mm offset it could tilt only about 9 deg before the disc reaches the arm). The jaws cannot sit
past the spinning prop, so the claw goes round its edge: a carbon tube runs out sideways from the motor mount, 25 mm
behind the prop, to just past the disc edge (first half), bends 90 deg forward (the bend), and the two-finger jaws
reach past the prop plane (second half). Writes results/claw_bend.json.   python3 claw_bend.py
"""
import os, sys, json, math
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import claw_fans as CF

G = 9.81
R = 16 * 25.4 / 2               # mm, 16x5.4 prop radius
MARGIN = 30.0                   # mm, jaws' tube to the disc edge [A]
BEHIND = 25.0                   # mm, tube centre behind the prop plane [A]
REACH = 90.0                    # mm, fingertips past the prop plane [A]
TUBE_OD, TUBE_ID, CF_RHO = 16.0, 14.0, 1.55e-3   # mm, mm, g/mm^3 (pultruded carbon tube) [A]
r_b = R + MARGIN                # radius of the bend
L1 = r_b                        # first half: hub to the bend
L2 = BEHIND + REACH             # second half: bend to fingertips
tube_g = math.pi / 4 * (TUBE_OD ** 2 - TUBE_ID ** 2) * (L1 + L2) * CF_RHO
brackets_g = 12.0               # clamp at the motor mount + printed bend [A]
add_g = tube_g + brackets_g
# wake blockage of the first half: tube shadow from the motor's edge (29 mm) to the disc edge, share of disc area
block = TUBE_OD * (R - 29) / (math.pi * R ** 2)
# grip and foot moments about the arm axis at the tip joints (STS3215 2.94 N.m stall)
grip_kg = 0.5
M_grip = grip_kg * G * r_b / 1000
foot_N = 18.83 * G / 7
M_foot = foot_N * r_b / 1000
# disc vs ground with the jaws on the floor: fingertip at the origin, prop plane REACH above it, disc centre r_b to one side
tilt_disc_side = math.degrees(math.atan(REACH / (r_b + R)))
tilt_sideways = math.degrees(math.atan(REACH / R))
# hover with the bent claw on the recommended fan and pack
f = CF.FANS["MN5008 KV400 + 16x5.4 CF"]; full = CF.fan_full(f); rated = 0.75 * full
d_g = (f["motor"] - CF.TODAY["motor"]) + (f["prop"] - CF.TODAY["prop"]) + (f["esc"] - CF.TODAY["esc"]) + 5 + add_g
rows = {}
for pk in ("2 x DOGCOM 22 Ah 5C semi-solid", "3 x DOGCOM 22 Ah 5C semi-solid", "Tattu 22 Ah 25C LiPo"):
    kg, wh, amps = CF.PACKS[pk]; robot = CF.BASE_KG + 8 * d_g / 1000 + kg
    tw = (CF.CP.P17_MAX_N * (1 - 0.036) + 8 * rated * (1 - 1.5 * block)) / (robot * G)
    P, Tc, fan = CF.hover(robot, f, rated * (1 - 1.5 * block))
    P = P + 8 * CF.fan_P(f, fan) * 1.5 * block * 1.5          # thrust lost to the tube made up at P ~ T^1.5 [A]
    rows[pk] = dict(robot_kg=round(robot, 2), tw_rated=round(tw, 3), hover_min=round(wh * 0.8 / P * 60, 1), hover_W=round(P),
                    hover_A=round(P / 21), pack_A=amps, fan_hover_N=round(fan, 1))
out = dict(prop_R_mm=round(R, 1), bend_radius_mm=round(r_b, 1), first_half_mm=round(L1), second_half_mm=round(L2),
           fingertips_past_prop_mm=REACH, tube=f"{TUBE_OD:.0f} x {TUBE_ID:.0f} mm carbon", added_g_per_claw=round(add_g),
           tip_g=round(130 + d_g), wake_blockage=round(block, 3),
           grip_moment_Nm=dict(object_kg=grip_kg, M=round(M_grip, 2), of_STS3215_stall=round(M_grip / 2.94, 2)),
           foot_on_jaws=dict(foot_N=round(foot_N, 1), M=round(M_foot, 2), of_STS3215_stall=round(M_foot / 2.94, 2)),
           floor_grab_tilt_limit_deg=dict(toward_prop=round(tilt_disc_side, 1), sideways=round(tilt_sideways, 1), away_from_prop="none"),
           old_wrist_tilt_limit_deg=round(math.degrees(math.asin((60 - 28.5) / R)), 1),
           packs=rows)
json.dump(out, open(os.path.join(CF.RES, "claw_bend.json"), "w"), indent=1)
print(json.dumps(out, indent=1))
