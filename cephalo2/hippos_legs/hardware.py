"""Stock fasteners and the holes they need, as manifold3d solids (rev D, 2026-11-27).

A fastened joint is declared once with `fasten(L, ...)`: it adds the screws (and inserts / nuts) as parts on the node of the
parts they join, and records the holes to cut.  `finish(L)` cuts every recorded hole from the named parts at the end of the
build, so a screw can never sit in solid material: the same-node overlap check (legs_cad_verify.py static) proves every hole
exists, and the moving-part sweep sees every screw head.

Dimensions (mm) are the standard ones:
  ISO 4762 socket head cap screw: head dia dk, head height k.     ISO 7380 button head: dk, k.
  Clearance holes ISO 273 medium; counterbores DIN 974-1 row for ISO 4762.
  Heat-set inserts: CNC Kitchen / Ruthex standard series (hole dia, length, knurl OD).
  Tapped holes in metal (motor housings, aluminium hubs) are drawn at the thread's nominal size + FIT, so the screw just fits.
"""
import numpy as np
from manifold3d import Manifold

SEG = 24
FIT = 0.05

SHCS = {2: (3.8, 2.0), 2.5: (4.5, 2.5), 3: (5.5, 3.0), 4: (7.0, 4.0), 5: (8.5, 5.0), 6: (10.0, 6.0), 8: (13.0, 8.0)}
BHCS = {3: (5.7, 1.65), 4: (7.6, 2.2), 5: (9.5, 2.75), 6: (10.5, 3.3)}
CLEAR = {2: 2.4, 2.5: 2.9, 3: 3.4, 4: 4.5, 5: 5.5, 6: 6.6, 8: 9.0}
CBORE = {2: 4.4, 2.5: 5.5, 3: 6.5, 4: 8.0, 5: 10.0, 6: 11.0, 8: 15.0}
INSERT = {2: (3.2, 4.0, 3.6), 2.5: (3.6, 5.7, 4.0), 3: (4.0, 5.7, 4.6), 4: (5.6, 8.1, 6.3), 5: (6.4, 9.5, 7.1), 6: (8.0, 12.7, 8.7)}
NUT = {3: (5.5, 2.4), 4: (7.0, 3.2), 5: (8.0, 4.7), 6: (10.0, 5.2), 8: (13.0, 6.8)}      # ISO 4032 across flats, height (nyloc: + ~1.5)
STEEL_RHO = 7.85e-6; BRASS_RHO = 8.5e-6


def _cyl(r, h, seg=SEG):
    return Manifold.cylinder(h, r, r, seg)


def _frame(p, u):
    """4x3 affine taking local z to unit vector u, origin to p."""
    u = np.asarray(u, float); u = u / np.linalg.norm(u)
    a = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    v = np.cross(u, a); v /= np.linalg.norm(v); w = np.cross(u, v)
    M = np.zeros((3, 4)); M[:, 0], M[:, 1], M[:, 2], M[:, 3] = v, w, u, p
    return M


def _along(m, p, u):
    return m.transform(_frame(np.asarray(p, float), u))


def screw(d, length, p, u, head="shcs"):
    """Screw with its head's bearing face at p, shank running along u (into the joint).  Shank drawn at d - 2 FIT so a hole
    drawn at d (a tapped hole or an insert bore) shows a running fit, not shared volume."""
    dk, k = (SHCS if head == "shcs" else BHCS)[d]
    m = _cyl(dk / 2, k).translate([0, 0, -k]) + _cyl(d / 2 - FIT, length)
    return _along(m, p, u)


class Joint:
    """One fastened joint: n screws of size d through the clamp parts (clearance hole, optional counterbore) into `base`
    (heat-set insert pocket, a hole already tapped in a bought part, or a nut).  Positions in the rest frame of the node.
    clamp: part-name key or list of keys (all get the clearance hole over the clamp stack t_clamp); base: key or None."""
    def __init__(s, name, node, d, length, pts, u, clamp, t_clamp, base, kind="insert", head="shcs", cbore=0.0, t_base=None):
        clamp = [] if clamp is None else ([clamp] if isinstance(clamp, str) else list(clamp))
        s.__dict__.update(dict(name=name, node=node, d=d, length=length, pts=[np.asarray(q, float) for q in pts], u=np.asarray(u, float) / np.linalg.norm(u),
                               clamp=clamp, t_clamp=t_clamp, base=base, kind=kind, head=head, cbore=cbore, t_base=t_base))

    def solids(s):
        """(screws, inserts or nuts or None, {part name: hole solid})"""
        d, u = s.d, s.u; sc = []; ins = []; holes = {}
        def cut(nm, m):
            if nm is None: return
            holes[nm] = holes[nm] + m if nm in holes else m
        dk, k = (SHCS if s.head == "shcs" else BHCS)[d]
        reach = s.length - (s.t_clamp - s.cbore)                           # thread length past the clamp stack
        if reach < 0.8 * d and s.kind != "nut":
            raise ValueError("%s: M%g x %g leaves %.1f mm of thread" % (s.name, d, s.length, reach))
        for q in s.pts:
            sc.append(screw(d, s.length, q + u * s.cbore, u, s.head))      # cbore > 0: head sunk that deep into the first clamp part
            c = _cyl(CLEAR[d] / 2, s.t_clamp + 0.2).translate([0, 0, -0.1])
            if s.cbore > 0:
                c = c + _cyl(max(CBORE[d], dk + 1.0) / 2, s.cbore + 0.1).translate([0, 0, -0.1])
            for nm in s.clamp:
                cut(nm, _along(c, q, u))
            if s.kind == "insert":
                hd, hl, od = INSERT[d]
                ins.append(_along(_cyl(hd / 2 - FIT, hl) - _cyl(d / 2, hl + 2).translate([0, 0, -1]), q + u * s.t_clamp, u))
                cut(s.base, _along(_cyl(hd / 2, max(hl, reach) + 1.0) + _cyl(d / 2 + 0.3, reach + 2.0), q + u * (s.t_clamp - 0.05), u))
            elif s.kind == "tapped":
                cut(s.base, _along(_cyl(d / 2, reach + 1.0), q + u * (s.t_clamp - 0.05), u))
            elif s.kind == "nut":
                af, h = NUT[d]; tb = s.t_base
                nut = Manifold.cylinder(h + 1.5, af / np.cos(np.pi / 6) / 2, af / np.cos(np.pi / 6) / 2, 6) - _cyl(d / 2, h + 4).translate([0, 0, -1])
                ins.append(_along(nut, q + u * (s.t_clamp + tb), u))
                cut(s.base, _along(_cyl(CLEAR[d] / 2, tb + 0.2), q + u * (s.t_clamp - 0.1), u))
        Ms = sc[0]
        for m in sc[1:]:
            Ms = Ms + m
        Mi = None
        for m in ins:
            Mi = m if Mi is None else Mi + m
        return Ms, Mi, holes


def screw_mass(d, length, n):
    dk, k = SHCS[d]
    return n * STEEL_RHO * np.pi * ((dk / 2) ** 2 * k + (d / 2) ** 2 * length)


def fasten(L, name, node, d, length, pts, u, clamp, t_clamp, base, kind="insert", head="shcs", cbore=0.0, t_base=None, group="hardware"):
    """Declare a fastened joint on leg L; the screws become one part '<name> screws n x Md x len'."""
    j = Joint(name, node, d, length, pts, u, clamp, t_clamp, base, kind, head, cbore, t_base)
    if not hasattr(L, "joints_hw"):
        L.joints_hw = []
    L.joints_hw.append(j)
    return j


def part_by(L, key):
    hits = [p for p in L.parts if key in p["name"]]
    if len(hits) != 1:
        raise KeyError("fastener target %r matches %d parts: %s" % (key, len(hits), [p["name"] for p in hits]))
    return hits[0]


def finish(L, COL):
    """Cut every declared hole and add the screws / inserts / nuts as parts."""
    for j in getattr(L, "joints_hw", []):
        Ms, Mi, holes = j.solids()
        for key, h in holes.items():
            p = part_by(L, key)
            p["solid"] = p["solid"] - h
        n = len(j.pts)
        L.parts.append(dict(name="%s screws %d x M%g x %g" % (j.name, n, j.d, j.length), node=j.node, group="hardware", col=COL["steel"], solid=Ms,
                            printed=False, mass=float(screw_mass(j.d, j.length, n))))
        if Mi is not None:
            what = {"insert": "heat-set inserts", "nut": "nyloc nuts"}[j.kind]
            L.parts.append(dict(name="%s %s %d x M%g" % (j.name, what, n, j.d), node=j.node, group="hardware", col=COL.get("brass", COL["steel"]), solid=Mi,
                                printed=False, mass=float(Mi.volume() * (BRASS_RHO if j.kind == "insert" else STEEL_RHO))))


def bolt_circle(n, pcd, centre, axis, phase=0.0):
    """n points on a circle of diameter pcd around centre, in the plane normal to axis ('x', 'y' or 'z')."""
    c = np.asarray(centre, float); r = pcd / 2; out = []
    for i in range(n):
        a = np.radians(phase) + 2 * np.pi * i / n
        e1, e2 = {"x": ((0, 1, 0), (0, 0, 1)), "y": ((1, 0, 0), (0, 0, 1)), "z": ((1, 0, 0), (0, 1, 0))}[axis]
        out.append(c + r * np.cos(a) * np.array(e1, float) + r * np.sin(a) * np.array(e2, float))
    return out


# ------------------------------------------------------------------ RobStride motors, from RobStride's own STEP models
# (github.com/RobStride/Product_Information, CAN2.0 folder, files dated 2026-10-30), measured with OCP (stepscan in the task notes).
# u = distance from the output's outer face into the motor (mm).  Profile: (r, u0, u1) stepped cylinders.
# Patterns: (n, pcd, phase deg, thread d, depth, face u, side) where side +1 = screw goes into the motor along +u from that face,
# -1 = from the back toward -u.  Phase is measured in the motor's own clocking; the assembly picks the clocking.
# Dowels: (n, pcd, phase, dia) at the output face.  Real bodies have ribs and a connector bump: the profile takes the largest radius
# over each band (an envelope that is never smaller than the motor), so clearances found against it hold for the real part.
MOTORS = {
    "RS02": dict(profile=[(14.3, 0.0, 0.5), (21.0, 0.5, 4.0), (39.25, 4.0, 32.0), (32.5, 32.0, 45.5)], length=45.5, out_r=21.0, out_face=0.5,
                 front_face=4.0, back_face=32.0,
                 out_pat=(6, 24.0, 43.9, 4, 6.0), dowels=(3, 24.0, 13.9, 4.0),
                 front_pat=(9, 73.0, 33.9, 3, 6.0), back_pat=(4, 52.0, 58.9, 4, 8.0), mass=0.405),
    "RS03": dict(profile=[(35.0, 0.0, 2.5), (49.3, 2.5, 42.0), (39.75, 42.0, 56.6)], length=56.6, out_r=35.0, out_face=0.0,
                 front_face=2.5, back_face=42.0,
                 out_pat=(6, 30.4, 0.0, 4, 9.0), dowels=(3, 30.4, -90.0, 4.0),
                 front_pat=(8, 98.0, 22.5, 4, 11.0), back_pat=(4, 98.0, 22.5, 4, 11.0, (22.5, 157.5, -157.5, -22.5)), mass=0.880),
    "RS04": dict(profile=[(14.6, 0.0, 3.0), (24.8, 3.0, 3.5), (60.0, 3.5, 42.5), (47.0, 42.5, 55.7)], length=55.7, out_r=24.8, out_face=3.0,
                 front_face=3.5, back_face=42.5,
                 out_pat=(9, 36.0, -165.0, 5, 8.0, (-165, -135, -75, -45, -15, 45, 75, 105, 165)), dowels=(3, 35.8, -105.0, 6.0),
                 front_pat=(10, 106.0, 21.0, 4, 7.0), back_pat=(10, 106.0, 21.0, 4, 6.0), mass=1.420),
    "RS06": dict(profile=[(26.0, 0.0, 1.5), (43.5, 1.5, 34.5), (35.0, 34.5, 50.5)], length=50.5, out_r=26.0, out_face=0.0,
                 front_face=1.5, back_face=34.5,
                 out_pat=(6, 24.0, 30.0, 4, 4.7), dowels=(3, 24.0, 0.0, 4.0),
                 front_pat=(8, 82.0, 22.5, 3, 8.0), back_pat=(4, 82.0, 67.5, 3, 8.0, (67.5, 112.5, -112.5, -67.5)), mass=0.621),
}


def motor_frame(p, u, clock):
    """Affine 4x3: motor local (x, y in the face plane, z = u into the motor) -> rest coords.  p = output face centre, u = unit
    vector into the motor, clock = a rest-frame vector giving the local x direction (projected onto the face plane)."""
    u = np.asarray(u, float); u = u / np.linalg.norm(u); c = np.asarray(clock, float); c = c - u * (c @ u); c = c / np.linalg.norm(c)
    w = np.cross(u, c); M = np.zeros((3, 4)); M[:, 0], M[:, 1], M[:, 2], M[:, 3] = c, w, u, np.asarray(p, float)
    return M


def motor_solid(name, M, seg=48, part="all"):
    """Motor body in rest coords, with its output and housing tapped holes cut (so screws drawn into them show a fit).
    part: "housing" (from the housing front face back), "output" (the spinning output disc in front of it, stopped 0.05 short)
    or "all".  0.1 mm running gap all round."""
    D = MOTORS[name]; m = None; ff = D["front_face"]
    for r, u0, u1 in D["profile"]:
        if part == "housing":
            u0 = max(u0, ff)
        elif part == "output":
            u1 = min(u1, ff - 0.1)
        if u1 - u0 < 1e-6:
            continue
        c = Manifold.cylinder(u1 - u0, r, r, seg).translate([0, 0, u0]); m = c if m is None else m + c
    # the spinning output flange runs on inside the housing envelope, past the front face, as deep as its tapped holes
    op = D["out_pat"]; r_in = op[1] / 2 + op[3] + 1.0; u_end = D["out_face"] + op[4] + 1.0
    if part == "housing" and u_end > ff:
        rb = (r_in + 0.1) / np.cos(np.pi / seg)                             # clears the turning output's polygon in any position
        m = m - Manifold.cylinder(u_end - ff + 0.2, rb, rb, seg).translate([0, 0, ff - 0.1])
    elif part == "output" and u_end > ff:
        m = m + Manifold.cylinder(u_end - ff + 0.1, r_in, r_in, seg).translate([0, 0, ff - 0.1])
    for key, face, side in (("out_pat", D["out_face"], 1), ("front_pat", D["front_face"], 1), ("back_pat", D["back_face"], -1)):
        if (part == "output" and key != "out_pat") or (part == "housing" and key == "out_pat"):

            continue
        for q in pattern_local(D, key):
            depth = D[key][4]
            h = _cyl(D[key][3] / 2, depth + 0.1)
            if key == "out_pat":                                            # output holes run through the pilot boss in front of the face
                h = _cyl(D[key][3] / 2, depth + face + 1.0).translate([0, 0, -face - 1.0])
            m = m - (h.translate([q[0], q[1], face - 0.05]) if side > 0 else h.translate([q[0], q[1], face - depth]))
    return m.transform(M)


def add_motor(L, label, name, M, stator, rotor, mass, act_col="act", out_frac=0.12):
    """Housing on the stator node, output disc on the rotor node (they turn against each other)."""
    L.add(label, stator, "actuators", act_col, motor_solid(name, M, part="housing"), mass=mass * (1 - out_frac))
    L.add(label.split(" (")[0] + " output rotor", rotor, "actuators", "steel", motor_solid(name, M, part="output"), mass=mass * out_frac)


def pattern_local(D, key):
    pat = D[key]; n, pcd, ph = pat[0], pat[1], pat[2]
    angs = pat[5] if len(pat) > 5 else [ph + 360.0 * i / n for i in range(n)]
    return [np.array([pcd / 2 * np.cos(np.radians(a)), pcd / 2 * np.sin(np.radians(a)), 0.0]) for a in angs]


def pattern(name, key, M):
    """Hole centres of a motor pattern on its face, in rest coords, and the face's u."""
    D = MOTORS[name]; face = {"out_pat": D["out_face"], "front_pat": D["front_face"], "back_pat": D["back_face"]}[key]
    return [M[:, :3] @ (q + np.array([0, 0, face])) + M[:, 3] for q in pattern_local(D, key)], face
