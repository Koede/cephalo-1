"""Checks for the Hippos-1 tail (tail.py) on the true solids.  python3 tail_checks.py -> tail_checks.json
1 parts at rest: no overlaps; 2 free range of every joint (distal group swept against the proximal group); 3 named poses
(kickstand crouched / reared, curled over the croup, hand to the side) against the rump socket and adapter; 4 masses;
5 servo loads and bending through the rump seat with the CAD masses; 6 current budget through the SFF pins; 7 print check."""
import json, numpy as np
import manifold3d as mf
import sff_core as H, tailmount as M, tail as TL
from sff_core import P

EPS = 0.02
RHO_EFF = 1.27e-3 * 0.70        # g/mm3: PETG, 4 walls + 40 % gyroid ~ 70 % of solid (est.)
G = 9.81
SEAT = np.array([0, 0, 3.8])

PARTS = TL.parts()
SERVOS = [TL.servo_env(k) for k in range(4)]
FIXED = {"rump_socket": M.rump_socket(), "tail_adapter": M.tail_adapter(), "hand_base": H.hand_body(), "root_cradle": PARTS["root_cradle"]}
LINKS = [PARTS[f"link{k}"] for k in range(4)]

def posed(q):
    """dict name -> posed manifold for joint angles q (deg)."""
    Ts = TL.pose_transforms(q)
    out = dict(FIXED); out["servo0"] = SERVOS[0]
    for k in range(4):
        out[f"link{k}"] = TL.apply(LINKS[k], Ts[k])
        if k < 3: out[f"servo{k + 1}"] = TL.apply(SERVOS[k + 1], Ts[k])
    out["tip_socket"] = TL.apply(PARTS["tip_socket"], Ts[3])
    return out

# designed contacts (bolted faces, servo cases in their cradles touch nothing by design)
DESIGNED = {("tail_adapter", "root_cradle"), ("tail_adapter", "hand_base"), ("link3", "tip_socket"), ("rump_socket", "hand_base"),
            ("rump_socket", "tail_adapter")}

def clashes(d, only=None):
    names = list(d); bad = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in DESIGNED or (b, a) in DESIGNED: continue
            if only and not (a in only or b in only): continue
            v = (d[a] ^ d[b]).volume()
            if v > EPS: bad.append((a, b, round(v, 2)))
    return bad

out = {}
out["rest_clashes"] = clashes(posed([0, 0, 0, 0]))

# ---- 2. free range per joint: sweep joint k, everything else at 0; distal group vs proximal group
def group_split(k, q):
    d = posed(q)
    dist = [f"link{j}" for j in range(k, 4)] + [f"servo{j}" for j in range(k + 1, 4)] + ["tip_socket"]
    prox = [n for n in d if n not in dist]
    A = mf.Manifold(); B = mf.Manifold()
    for n in dist: A += d[n]
    for n in prox: B += d[n]
    return A, B

import os
ranges = json.load(open('tail_ranges.json')) if os.path.exists('tail_ranges.json') else {}
for k in ([] if ranges else range(4)):
    lim = {}
    for sgn in (1, -1):
        free = 0
        for ang in range(2, 182, 2):
            q = [0, 0, 0, 0]; q[k] = sgn * ang
            A, B = group_split(k, q)
            if (A ^ B).volume() > EPS: break
            free = ang
        lim["+" if sgn > 0 else "-"] = free
    ranges[f"j{k} ({TL.JOINTS[k][0]}, {'pitch' if TL.JOINTS[k][1] == 'y' else 'yaw'})"] = lim
    print("range", k, lim)
json.dump(ranges, open("tail_ranges.json", "w")); out["free_range_deg"] = ranges

# ---- 3. named poses.  Pitch +: tail down (x_c = up).  Body pitch phi nose-up: gravity in the connector frame (-cos, 0, sin)
def gvec(phi_deg):
    """gravity direction in the connector frame (x_c = up, z_c = +x_h forward into the rump), body nose-up by phi"""
    p = np.radians(phi_deg); return np.array([-np.cos(p), 0.0, -np.sin(p)])

POSES = {
    "held out level, hand + 0.3 kg": dict(q=[0, 0, 0, 0], phi=0, pay=0.3, kick=None, crouch=0),
    "hand out to the side (j1 yaw 90), 0.3 kg": dict(q=[0, 90, 0, 0], phi=0, pay=0.3, kick=None, crouch=0),
    "curled up over the croup": dict(q=[-60, 0, -70, 0], phi=0, pay=0.0, kick=None, crouch=0),
    "kickstand crouched, column vertical": dict(q=[90, 0, 0, 0], phi=0, pay=0.0, kick=0.10, crouch=180),
    "kickstand crouched, raked 15 deg back": dict(q=[75, 0, 0, 0], phi=0, pay=0.0, kick=0.10, crouch=180),
    "kickstand reared 30 deg, column vertical (j0 60)": dict(q=[60, 0, 0, 0], phi=30, pay=0.0, kick=0.10, crouch=0),
    "kickstand reared 45 deg, column vertical (j0 45)": dict(q=[45, 0, 0, 0], phi=45, pay=0.0, kick=0.10, crouch=0),
}

SEAT_H = np.array([-310.0, 450.0]); HIP_H = np.array([-150.0, 378.0])        # horse frame x, z (mm), 0.75 scale
def world_z(p_c, phi, crouch):
    """connector-frame point -> height above the ground (mm): x_h = x_seat + z_c (the tail runs back along -z_c), z_h = z_seat + x_c,
    then nose-up phi about the rear hip"""
    xh, zh = SEAT_H[0] + (p_c[2] - SEAT[2]), SEAT_H[1] + p_c[0]
    a = np.radians(phi); r = np.array([xh, zh]) - HIP_H
    return float(HIP_H[1] + r[0] * np.sin(a) + r[1] * np.cos(a) - crouch)

def mass_props(m):
    t = H.to_trimesh(m)
    return t.volume * RHO_EFF / 1000.0, np.asarray(t.center_mass)          # kg, mm

LINK_MP = [mass_props(L) for L in LINKS]
TIP_MP = mass_props(PARTS["tip_socket"])
HAND = 0.060                                     # small SFF hand on the tip (screw picker class, est.)

def loads(spec):
    q = spec["q"]; g = gvec(spec["phi"]); Ts = TL.pose_transforms(q)
    pts = []                                     # (point mm, force N) for everything distal of j0
    for k in range(4):
        m, c = LINK_MP[k]; pts.append(((Ts[k] @ np.r_[c, 1])[:3], m * G * g))
        if k < 3:
            s = TL.SERVO[TL.JOINTS[k + 1][0]]; zc = TL.ZJ[k + 1] + (s[0] - 2 * s[3]) / 2
            pts.append(((Ts[k] @ np.r_[0, 0, zc, 1])[:3], s[5] * G * g))
    tip = (Ts[3] @ np.r_[0, 0, TL.Z_TIP - 15, 1])[:3]
    pts.append(((Ts[3] @ np.r_[TIP_MP[1], 1])[:3], TIP_MP[0] * G * g))
    pts.append((tip, (HAND + spec["pay"]) * G * g))
    if spec["kick"]:
        mouth = (Ts[3] @ np.r_[0, 0, TL.Z_TIP, 1])[:3]
        pts.append((mouth, -spec["kick"] * 30.8 * G * g))                # ground pushes up (world), 10 % of Centauri-1
    tq = []
    for k in range(4):
        Tk = Ts[k]; piv = (Tk @ np.r_[0, 0, TL.ZJ[k], 1])[:3]; ax = Tk[:3, :3] @ TL.joint_axis(k)
        sel = pts[2 * k:] if k > 0 else pts
        tq.append(abs(sum(np.cross(p - piv, f) @ ax for p, f in sel)))
    # bending through the seat: everything on the tail side of the SFF (incl. root cradle, j0 servo, adapter, base)
    s0 = TL.SERVO["STS3095"]; extra = [(np.array([0, 0, TL.ZJ[0] + (s0[0] - 2 * s0[3]) / 2]), s0[5] * G * g),
                                        (np.array([0, 0, -55.0]), (mass_props(PARTS["root_cradle"])[0] + 0.0331 + 0.0106 + 0.016) * G * g)]
    Mseat = np.linalg.norm(sum(np.cross(p - SEAT, f) for p, f in pts + extra))
    rated = [TL.SERVO[TL.JOINTS[k][0]][6] for k in range(4)]
    tq = [t / 1000.0 for t in tq]; Mseat /= 1000.0                                # N mm -> N m
    return dict(torque=[round(t, 3) for t in tq], frac_rated=[round(t / r, 3) for t, r in zip(tq, rated)], M_seat=round(Mseat, 3),
                lug_factor=round(94.8 * 5.8 / (2 * Mseat / (3 * 0.012)), 2) if Mseat > 0 else 99)

pose_res = {}
for name, spec in POSES.items():
    d = posed(spec["q"])
    cl = clashes(d)
    r = loads(spec)
    Ts = TL.pose_transforms(spec["q"]); mouth = (Ts[3] @ np.r_[0, 0, TL.Z_TIP, 1])[:3]
    r["tip_mouth_mm"] = mouth.round(1).tolist(); r["clashes"] = cl
    r["tip_height_above_ground_mm"] = round(world_z(mouth, spec["phi"], spec["crouch"]), 1)
    pose_res[name] = r
    print(name, r["frac_rated"], "seat", r["M_seat"], "lugs x", r["lug_factor"], "tip z", r["tip_height_above_ground_mm"], "clashes", cl)
out["poses"] = pose_res

# ---- 4. masses
mass = {n: round(mass_props(m)[0] * 1000, 1) for n, m in PARTS.items()}
mass["servos"] = round(sum(TL.SERVO[s][5] for s, _ in TL.JOINTS) * 1000, 1)
mass["bought (bearings 4 x 6800, screws, inserts, wire, pin set)"] = 4 * 5.5 + 40 + 25 + 8
mass["tail mount (adapter + base + bought)"] = 33.1 * 0.7 + 10.6 + 16.2
mass["total detachable tail"] = round(sum(v for k, v in mass.items() if k not in ("rump_pad",)), 1)
out["mass_g"] = mass
print("mass", mass)

# ---- 6. current through the SFF 12 V (Mill-Max 0906, 6.5 A, 5.2 A derated per pin)
cur = dict(stall_A={"STS3095": 9.8, "STS3250": 4.2, "STS3215": 2.7})
held = pose_res["held out level, hand + 0.3 kg"]
def amps(s, T):          # linear current-torque line from no-load current to stall (no-load est. 0.6 / 0.28 / 0.18 A)
    I0 = {"STS3095": 0.6, "STS3250": 0.28, "STS3215": 0.18}[s]; St = TL.SERVO[s][7]
    return I0 + (cur["stall_A"][s] - I0) * T / St
need = [amps(TL.JOINTS[k][0], held["torque"][k]) for k in range(4)]
cur["held_out_A"] = [round(a, 2) for a in need]; cur["held_out_total_A"] = round(sum(need), 2)
cur["limits_A"] = {"j0 STS3095": 2.8, "j1 STS3250": 0.8, "j2 STS3215": 1.0, "j3 STS3215": 0.6}      # sum = one pin derated
cur["limits_total_A"] = sum(cur["limits_A"].values())
lim = cur["limits_A"]
cur["torque_at_limit_Nm"] = {"j0": round((lim["j0 STS3095"] - 0.6) / (9.8 - 0.6) * 9.32, 2), "j1": round((lim["j1 STS3250"] - 0.28) / (4.2 - 0.28) * 4.90, 2),
                             "j2": round((lim["j2 STS3215"] - 0.18) / (2.7 - 0.18) * 2.94, 2), "j3": round((lim["j3 STS3215"] - 0.18) / (2.7 - 0.18) * 2.94, 2)}
cur["needed_held_out_Nm"] = held["torque"]
cur["one_pin_derated_A"] = 5.2; cur["two_pins_derated_A"] = 10.4
out["current"] = cur
print("current", cur)

# ---- 7. print: best of 6 axis orientations per part (down-facing area flatter than 45 deg, off the bed)
def overhang(m):
    t = H.to_trimesh(m); n = t.face_normals; c = t.triangles_center; ar = t.area_faces; best = None
    for up in ([0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]):
        u = np.array(up, float); h = c @ u; bed = h.min()
        sel = ((n @ u) < -0.72) & (h > bed + 0.05)
        a = float(ar[sel].sum())
        if best is None or a < best[1]: best = (up, a)
    return dict(up=best[0], overhang_mm2=round(best[1], 1))
out["print"] = {n: overhang(m) for n, m in PARTS.items() if n != "tip_socket"}
print("print", out["print"])
json.dump(out, open("tail_checks.json", "w"), indent=1, default=float)
