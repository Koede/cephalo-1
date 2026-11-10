#!/usr/bin/env python3
"""
mantle_notch.py  --  arm ports for the Cephalo-1 mantle, with the notch for the geared root socket

The geared root (root_gear.py) moves the joint-0 STS3250 30 mm down the arm and 13 mm along the hinge
axis, so its sleeve stands 6.8 mm proud of the 57.1 mm tube on one side. The mantle's arm openings need
a notch there. The mantle itself is not in this folder (it lives in Forest's Cephalo-1 CAD), so this
script writes the opening as a cutter to subtract from it, plus flat templates:

  out_1m_13/mantle_port_cutter_joint0_frame.stl   one port: opening + notch + 8 x M3 clearance holes,
                                                  from the flange face 30 mm into the mantle (joint-0 frame)
  out_1m_13/mantle_port_cutters_x8_r<R>.stl       all eight ports in the robot frame, for a mantle whose
                                                  arm joints sit on a circle of radius R (default 110 mm,
                                                  the study's hub radius [A]); z up, arm 0 along +x
  out_1m_13/mantle_port_template.dxf / .svg       the port outline and hole centres, flange-face plane
  out_1m_13/mantle_ports.json                     sizes, notch, and the smallest hub radius at which the
                                                  eight flanges stay clear of each other

  python3 mantle_notch.py --length 1000 --joints 13 [--hub-radius 110]
Frames as pin_cad.py: alpha along the joint-0 hinge axis (world up on the robot), beta across, zeta along the arm.
"""
import argparse, json, math, os, sys
import numpy as np
from manifold3d import Manifold, CrossSection
import trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pin_cad as C
import root_gear as RG
from pin_cad import box, cyl_z, to_trimesh

CLR = 0.5          # clearance round everything that passes through the mantle wall
DEPTH = 30.0       # cutter length into the mantle, from the flange face
HOLE_D = 3.4       # M3 clearance through the mantle wall

def port_outline(arm, length, joints):
    """2-D outline (alpha, beta) of everything inside the flange face, projected along the arm, + CLR; and the same
    for the ungeared socket pin_cad.py drew before the root fix (the opening the mantle was cut for)"""
    ec, bc = arm.socket_parts_local()
    cut = box(-200, 200, -200, 200, -200, arm.flange_z0 - 0.05)
    new = ((arm.socket_local() + ec + arm.servo0_local()) ^ cut).project().offset(CLR)
    old_arm = C.PinArm(length, C.TWIST, joints)
    oz0 = old_arm.cr_lo - 3.0
    inv = np.linalg.inv(np.vstack([C.joint_affine(0, C.TWIST, old_arm.pitch), [0, 0, 0, 1]]))[:3, :]
    old_body = (old_arm.part(0).transform(inv) + old_arm.servo_solid() + old_arm.lid())
    old = (old_body ^ box(-200, 200, -200, 200, -200, oz0 - 0.05)).project().offset(CLR)
    if old.is_empty():   # the ungeared socket sat wholly outside the flange face: only the flange's cable hole went through
        old = CrossSection.circle(old_arm.R - 8.0, 128)
    return new, old

def flange_holes(arm):
    """the 8 M3 holes of the root flange (alpha, beta), as root_gear.py draws them"""
    R, hw = arm.R, arm.hw
    rr = R + C.FLANGE_EXTRA / 2 - 0.5
    holes = [(rr * math.cos(math.radians(t)), rr * math.sin(math.radians(t))) for t in RG.FLANGE_HOLE_DEG]
    holes += [(arm.sfloor + 6.4, sy * (hw + 3.6)) for sy in (-1, 1)] + [(arm.sfloor - 3.0, sy * 9.0) for sy in (-1, 1)]
    return holes

def flange_cs(arm):
    R, hw = arm.R, arm.hw
    return CrossSection.circle(R + C.FLANGE_EXTRA, 128) + CrossSection.square([0.0 - (arm.sfloor - 6.0), 2 * (hw + 6.0)]).translate([arm.sfloor - 6.0, -(hw + 6.0)])

def to_robot(m, k, hub_r):
    """joint-0 frame of arm k -> robot frame (z up, arm 0 along +x), joint 0 on a circle of radius hub_r"""
    t = math.radians(45.0 * k)
    er = np.array([math.cos(t), math.sin(t), 0.0]); et = np.array([-math.sin(t), math.cos(t), 0.0])
    M = np.zeros((3, 4)); M[:, 0] = [0, 0, 1.0]; M[:, 1] = -et; M[:, 2] = er; M[:, 3] = hub_r * er
    return m.transform(M)

def write_dxf(path, polys, holes):
    L = ["0", "SECTION", "2", "ENTITIES"]
    for p in polys:
        for i in range(len(p)):
            (x0, y0), (x1, y1) = p[i], p[(i + 1) % len(p)]
            L += ["0", "LINE", "8", "PORT", "10", f"{x0:.3f}", "20", f"{y0:.3f}", "11", f"{x1:.3f}", "21", f"{y1:.3f}"]
    for (x, y) in holes:
        L += ["0", "CIRCLE", "8", "HOLES", "10", f"{x:.3f}", "20", f"{y:.3f}", "40", f"{HOLE_D/2:.3f}"]
    L += ["0", "ENDSEC", "0", "EOF"]
    open(path, "w").write("\n".join(L) + "\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=1000.0); ap.add_argument("--joints", type=int, default=13)
    ap.add_argument("--hub-radius", type=float, default=110.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_1m_13"))
    a = ap.parse_args()
    arm = RG.GearedRootArm(a.length, C.TWIST, a.joints)
    outline, tube = port_outline(arm, a.length, a.joints)
    holes = flange_holes(arm)
    notch = outline - tube
    nb = notch.bounds()
    z0 = arm.flange_z0                                     # flange face against the mantle (zeta)
    cut = Manifold.extrude(outline, DEPTH)                 # profile in x-y = (alpha, beta), extruded along +z
    for (x, y) in holes:
        cut = cut + Manifold.cylinder(DEPTH, HOLE_D / 2, HOLE_D / 2, 32).translate([x, y, 0])
    cut = cut.translate([0, 0, z0 - DEPTH + 0.01])          # flange face inward (toward the hub = -zeta)
    to_trimesh(cut).export(os.path.join(a.out, "mantle_port_cutter_joint0_frame.stl"))
    allc = [to_trimesh(to_robot(cut, k, a.hub_radius)) for k in range(8)]
    trimesh.util.concatenate(allc).export(os.path.join(a.out, f"mantle_port_cutters_x8_r{a.hub_radius:.0f}.stl"))

    # flange-to-flange clearance between neighbouring arms, against the hub radius
    fl = Manifold.extrude(flange_cs(arm), 3.0).translate([0, 0, arm.flange_z0])
    def overlap(r):
        A, B = to_robot(fl, 0, r), to_robot(fl, 1, r)
        return (A ^ B).volume()
    r_min = None
    for r in np.arange(80.0, 200.0, 0.5):
        if overlap(r) < 1e-6: r_min = float(r); break
    # templates
    polys = outline.to_polygons()
    write_dxf(os.path.join(a.out, "mantle_port_template.dxf"), polys, holes)
    ob = outline.bounds()
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{ob[0]-14:.1f} {ob[1]-14:.1f} {ob[2]-ob[0]+28:.1f} {ob[3]-ob[1]+28:.1f}" '
           f'width="{(ob[2]-ob[0]+28)*5:.0f}" height="{(ob[3]-ob[1]+28)*5:.0f}" font-family="sans-serif">',
           '<rect x="-1000" y="-1000" width="2000" height="2000" fill="#fff"/>',
           f'<circle cx="0" cy="0" r="{arm.R:.2f}" fill="none" stroke="#999" stroke-width="0.25" stroke-dasharray="1,1"/>']
    for p in tube.to_polygons():
        svg.append('<polygon fill="none" stroke="#06c" stroke-width="0.3" stroke-dasharray="1.5,0.8" points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in p) + '"/>')
    for p in notch.to_polygons():
        svg.append('<polygon fill="#f4c27a" stroke="none" points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in p) + '"/>')
    for p in polys:
        svg.append('<polygon fill="none" stroke="#000" stroke-width="0.35" points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in p) + '"/>')
    for (x, y) in holes:
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{HOLE_D/2}" fill="none" stroke="#c00" stroke-width="0.3"/>')
    svg.append(f'<text x="{nb[0]-1:.1f}" y="{ob[3]+8:.1f}" font-size="3">notch (orange): {nb[2]-nb[0]:.1f} x {nb[3]-nb[1]:.1f} mm on the -alpha side, '
               f'to {-ob[0]:.1f} mm from the hinge axis</text>')
    svg.append(f'<text x="{nb[0]-1:.1f}" y="{ob[3]+12:.1f}" font-size="3">alpha (joint-0 hinge axis, up on the robot) to the right, beta down; '
               f'flange-face plane; grey: {2*arm.R:.1f} mm tube; blue dashed: opening before the geared root; red: 8 x M3 clearance</text>')
    svg.append("</svg>")
    open(os.path.join(a.out, "mantle_port_template.svg"), "w").write("\n".join(svg))
    res = dict(outline_alpha=[round(ob[0], 2), round(ob[2], 2)], outline_beta=[round(ob[1], 2), round(ob[3], 2)],
               notch_alpha=[round(nb[0], 2), round(nb[2], 2)], notch_beta=[round(nb[1], 2), round(nb[3], 2)],
               old_opening_alpha=[round(tube.bounds()[0], 2), round(tube.bounds()[2], 2)], old_opening_beta=[round(tube.bounds()[1], 2), round(tube.bounds()[3], 2)],
               hole_edge_to_opening_mm=[round(float(min(np.hypot(*(np.array(h) - np.array(v))) for p in polys for v in p)) - HOLE_D / 2, 2) for h in holes],
               flange_face_zeta=round(z0, 2), cutter_depth=DEPTH, holes=[[round(x, 2), round(y, 2)] for x, y in holes],
               hub_radius_used=a.hub_radius, min_hub_radius_flanges_clear=r_min,
               flange_face_radius_at_min=None if r_min is None else round(r_min + z0, 1),
               flange_overlap_at_hub_radius_mm3=round(overlap(a.hub_radius), 1))
    json.dump(res, open(os.path.join(a.out, "mantle_ports.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))

if __name__ == "__main__":
    main()
