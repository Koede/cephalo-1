#!/usr/bin/env python3
"""
horn_bearing.py  --  horn-side support bearing for joints 1 to 4 of the Cephalo-1 main arm (1 m, 13 joints)

The joint study (joint-study/sim/joint_loads.py) found the STS3250's own output shaft carrying 110 to 510 N sideways
about 9 mm out from the case, because the claw fan pushes across the hinge. This adds a second support on the horn
side, coaxial with the hinge axis, so the fork's horn ear turns in a ball bearing held by the servo lid:

  - bearing: 6705 / 61705 thin-section ball bearing, 25 x 32 x 4 mm, C0 85 kgf = 834 N, C 110 kgf
    (simplybearings.co.uk listing for the 61705; the 6705ZZ / 2RS is the same size). About 6 g [A].
  - lid (joints 1 to 4 only): a ring 1.8 mm thick round the bearing's outer race, standing on the lid top, with
    a 0.5 mm groove in the lid under the inner race so only the outer race touches the lid.
  - fork, horn ear (vertebrae 2 to 5, the forks of joints 1 to 4): a sleeve hangs from the ear into the bearing's
    bore and round the servo horn (0.3 mm clear of it), and the ear's boss flares from 10.5 to 12.5 mm radius at its
    root to carry the sleeve; the four horn screws and the access hole are cut again through the new material.

Nothing else moves: the stack along the hinge axis has 4.5 mm between the lid top and the horn ear, and the
bearing is 4 mm wide, with 0.5 mm running gap under the ear. The arm's 57.1 mm outer diameter is unchanged.

  python3 horn_bearing.py            # writes out_1m_13/horn_bearing/ (STLs, assembly, horn_bearing.json, report)
Frames as pin_cad.py: alpha along the hinge axis, beta across, zeta along the arm (joint-local).
"""
import json, math, os, sys
import numpy as np
from manifold3d import Manifold
import trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import pin_cad as C
import root_gear as RG
from pin_cad import box, cyl_x, joint_affine, to_trimesh

BRG_JOINTS = (1, 2, 3, 4)          # joints with the STS3250 outside the mantle (joint 0 is the geared root)
BRG = dict(name="6705 (61705) 25 x 32 x 4", d=25.0, D=32.0, B=4.0, C0_N=85 * 9.81, C_N=110 * 9.81, g=6.0)  # mass [A]
HOUSING_T = 1.8                    # ring wall round the outer race (3 perimeters)
GROOVE = 0.5                       # relief in the lid top under the inner race
RUN_GAP = 0.5                      # bearing top to the horn ear's underside
SLEEVE_CLR = 0.3                   # sleeve bore to the servo horn's rim
EAR_ROOT_R = 12.5                  # horn-ear boss radius at its root (was 10.5)
HW_G = 0.3                         # [A] a drop of retaining compound, nothing else


def ann_x(ro, ri, a0, a1):
    return cyl_x(ro, a0, a1) - cyl_x(ri, a0 - 1, a1 + 1)


def cone_x(r0, r1, a0, a1):
    return Manifold.cylinder(a1 - a0, r0, r1, C.SEG).rotate([0, 90, 0]).translate([a0, 0, 0])


class BearingArm(RG.GearedRootArm):
    def __init__(s, *a, **k):
        super().__init__(*a, **k)
        s.b_a0 = s.lid_hi                               # bearing sits on the lid top
        s.b_a1 = s.b_a0 + BRG["B"]
        assert s.b_a1 + RUN_GAP <= s.hear_lo + 1e-9, "no room for the bearing between lid and horn ear"
        s.h_ro = BRG["D"] / 2 + HOUSING_T
        s.sl_ri, s.sl_ro = C.HORN_R + SLEEVE_CLR, BRG["d"] / 2
        s.sl_a0 = s.b_a0 + GROOVE + 0.1                 # sleeve end, clear of the groove floor

    # ------------------------------------------------------------ joint-local pieces
    def bearing_lid(s):
        L = s.lid()
        ring = ann_x(s.h_ro, BRG["D"] / 2, s.b_a0 - 0.01, s.b_a1)
        cap = cyl_x(s.rho_cap, s.p_hi - 1, s.b_a1 + 1)          # keep inside the cradle cap the next ring clears
        L = L + (ring ^ cap)
        L = L - ann_x(BRG["d"] / 2 + 2.1, BRG["d"] / 2 - 0.3, s.b_a0 - GROOVE, s.b_a0 + 0.01)   # groove under the inner race
        return L

    def bearing_solid(s):
        return ann_x(BRG["D"] / 2, BRG["d"] / 2, s.b_a0, s.b_a1)

    def ear_add(s):
        """material added to the horn ear of a fork (joint-local of that joint): flared boss + sleeve, holes re-cut"""
        add = cone_x(EAR_ROOT_R, C.BOSS_R, s.hear_lo, s.hear_hi) + ann_x(s.sl_ro, s.sl_ri, s.sl_a0, s.hear_lo + 0.01)
        add = add - cyl_x(3.0, s.hear_lo - 1, s.hear_hi + 1)
        for k in range(4):
            t = math.radians(45 + 90 * k); y, z = C.HORN_PCD / 2 * math.cos(t), C.HORN_PCD / 2 * math.sin(t)
            add = add - cyl_x(1.7, s.hear_lo - 1, s.hear_hi + 1, y, z) - cyl_x(3.0, s.hear_hi - 1.8, s.hear_hi + 1, y, z)
        return add

    # ------------------------------------------------------------ overrides
    def part(s, i):
        p = super().part(i)
        if (i - 1) in BRG_JOINTS:
            p = p + s.ear_add().transform(joint_affine(i - 1, s.twist, s.pitch))
        return p

    def lid_global(s, j):
        L = s.bearing_lid() if j in BRG_JOINTS else s.lid()
        return L.transform(joint_affine(j, s.twist, s.pitch))

    def body(s, i):
        b = super().body(i)
        if i in BRG_JOINTS:
            b = b + s.bearing_solid().transform(joint_affine(i, s.twist, s.pitch))
        return b


def main():
    out = os.path.join(HERE, "out_1m_13", "horn_bearing"); os.makedirs(out, exist_ok=True)
    arm = BearingArm(1000.0, C.TWIST, 13)
    base = RG.GearedRootArm(1000.0, C.TWIST, 13)
    J, P, tw = arm.J, arm.pitch, arm.twist
    g = lambda m: m.volume() * C.PETG_RHO * C.EFF_FILL
    def local(m, j):
        M = np.eye(4); M[:3, :] = joint_affine(j, tw, P); return m.transform(np.linalg.inv(M)[:3, :])

    lid_b, lid_0 = arm.bearing_lid(), arm.lid()
    vb, v0 = arm.part(2), base.part(2)
    # radius check: every new feature against the arm's outer radius, about the arm axis (joint-local alpha, beta)
    def r_arm(m):
        v = np.asarray(m.to_mesh().vert_properties)[:, :3]; return float(np.hypot(v[:, 0], v[:, 1]).max())
    r_ring = r_arm(lid_b - lid_0) if not (lid_b - lid_0).is_empty() else 0.0
    r_ear = r_arm(arm.ear_add())
    # static interference and joint sweeps with the bearings in place (joints 1 and 2), and joint 1 + 2 together
    bodies = [arm.body(i) for i in range(J + 1)]
    stat = [float((bodies[i] ^ bodies[i + 1]).volume()) for i in range(J)]
    sweeps = {j: arm.joint_sweep(j, bodies) for j in (1, 2, 4)}
    pairs = {j: arm.pair_sweep(j, bodies) for j in (1, 3)}
    # bearing vs moving parts it must not touch: the fork's horn ear and sleeve of the next vertebra (no-sweep, coaxial)
    gap_top = arm.hear_lo - arm.b_a1
    # masses
    d_lid = g(lid_b) - g(lid_0)
    d_vert = g(vb) - g(v0)
    per_joint = d_lid + d_vert + BRG["g"] + HW_G
    per_arm = len(BRG_JOINTS) * per_joint
    robot0 = 14.93 + 1.988                               # central prop under the dome + Tattu 16 Ah pack (central_prop.py)
    # T/W: central prop max (download taken off) + 8 claw fans at 16 N rated, against the robot's weight
    T_rated = 49.9 * (1 - 0.036) + 8 * 16.0
    robot1 = robot0 + 8 * per_arm / 1000
    # loads from joint_loads.py (worst over joints 1-4, horn side, N) against the bearing's static rating
    loads = {"hover, any pose (13.2 N fans)": 111, "16 N rated, fan within 30 deg of up": 191, "16 N rated, within 60 deg": 244,
             "16 N rated, any direction": 409, "21.4 N full, any direction": 511}
    load_rows = {k: dict(N=v, of_C0=round(v / BRG["C0_N"], 2), twice_of_C0=round(2 * v / BRG["C0_N"], 2)) for k, v in loads.items()}
    # lid screws: four M3 through the 2 mm PETG lid carry the side load in bearing
    screw = {k: round(v / 4 / (3.0 * 2.0), 1) for k, v in loads.items()}   # MPa on the lid's hole walls (3 mm x 2 mm)

    stl = os.path.join(out, "stl"); os.makedirs(stl, exist_ok=True)
    to_trimesh(lid_b).export(os.path.join(stl, "servo_lid_bearing_x4.stl"))
    to_trimesh(local(vb, 1)).export(os.path.join(stl, "vertebra_bearing_x4.stl"))
    # one joint, assembled, joint-1 frame: cradle part (vertebra 1), lid, bearing, servo, horn, and the next vertebra's fork
    asm = [local(arm.part(1), 1), lid_b, arm.bearing_solid(), arm.servo_solid(), arm.horn_solid(), local(arm.part(2), 1)]
    trimesh.util.concatenate([to_trimesh(m) for m in asm]).export(os.path.join(out, "assembly_joint1_bearing.stl"))
    # a section through the hinge axis (beta = 0 plane) for a quick look
    sec = [m ^ box(-60, 60, -0.01, 0.01, -60, 120) for m in asm]

    res = dict(bearing=BRG, joints=list(BRG_JOINTS), stack_alpha=dict(lid_top=arm.lid_hi, bearing=[arm.b_a0, arm.b_a1], horn=[arm.horn_lo, arm.horn_hi],
               horn_ear=[arm.hear_lo, arm.hear_hi], gap_bearing_to_ear=round(gap_top, 2)),
               housing_r=arm.h_ro, sleeve_r=[arm.sl_ri, arm.sl_ro], ear_root_r=EAR_ROOT_R,
               arm_R=arm.R, max_r_new_ring=round(r_ring, 2), max_r_new_ear=round(r_ear, 2),
               static_interference_mm3=round(max(stat), 2), sweeps={str(k): v for k, v in sweeps.items()},
               pairs={str(k): v for k, v in pairs.items()},
               mass_g=dict(lid_extra=round(d_lid, 2), vertebra_extra=round(d_vert, 2), bearing=BRG["g"], per_joint=round(per_joint, 1),
                           per_arm=round(per_arm, 1), robot=round(8 * per_arm, 0)),
               robot_kg_before=round(robot0, 2), robot_kg_after=round(robot1, 2),
               tw_rated_before=round(T_rated / (robot0 * 9.81), 3), tw_rated_after=round(T_rated / (robot1 * 9.81), 3),
               loads=load_rows, lid_screw_hole_MPa=screw,
               note="load split between this bearing and the servo's own output bearing is set by the fits and is not modelled [A]")
    json.dump(res, open(os.path.join(out, "horn_bearing.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
