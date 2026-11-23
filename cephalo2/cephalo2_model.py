"""Cephalo-2 full-robot 3D model (GLB) for the rotatable viewer, built from the project's CAD.

Cephalo-2 = rev 4 arms (8 x 8 servo joints, clutches, winch, tip cameras) + hand3 final on arms 2 and 6 + XL claw-props
on the other six + bayonet hand connector on every tip (handmount.py, socket on the arm, shared hand base on the pod) +
Scabbard belly rotor in place of the EDF (scabbard.py) + turret with 5 cameras in place of the eyestalks (turretcams.py).
Two poses: stand (blades stowed in the turret) and flight (tulip, blades out, props as rotors).
python3 cephalo2_model.py -> cephalo2_stand.glb, cephalo2_flight.glb, cephalo2_parts.json
"""
import os, json
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "2")
import numpy as np, trimesh
import cephalo4 as C
import xlpod4; xlpod4.install()
import viz4 as V
HANDS = os.environ.get("C2_HANDS", "mixed")          # mixed: hand3 on arms 2 + 6, XL claw-props elsewhere; all: hand3 on all eight
if HANDS == "s3":                                     # hand3 hub v3 + D winches + S3 cam sleeve on all eight (2026-11-20)
    import hand3_draw_s3 as H3; H3.install(tuple(range(8)))
else:
    import hand3_draw as H3; H3.install(tuple(range(8)) if HANDS == "all" else (2, 6))
import scabbard as S
import turretcams as TC
import handmount as HM
from geom import Scene

MM = 1e-3
LEN_BASE = (HM.LEN_HAND_BASE + HM.P["gap"]) * MM          # the hand / pod sits this far out from the socket mouth
_pod = xlpod4.draw
def _pod_shift(S_, pt, Rt, *a, **k):
    return _pod(S_, pt + Rt[:, 0] * LEN_BASE, Rt, *a, **k)
xlpod4.draw = _pod_shift


def tm(man, scale=MM):
    me = man.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3] * scale, np.asarray(me.tri_verts), process=False)


def scene_groups(sc):
    """viz4 Scene prims (per-face colours) -> {rgb tuple: (V, F)} merged by colour."""
    out = {}
    for Vv, F, Cc, _ in sc.prims:
        cols = np.round(np.asarray(Cc, float), 2)
        for c in np.unique(cols, axis=0):
            m = np.all(cols == c, axis=1)
            Ff = F[m]
            used = np.unique(Ff)
            remap = -np.ones(len(Vv), int); remap[used] = np.arange(len(used))
            out.setdefault(tuple(c), []).append((Vv[used], remap[Ff]))
    return {c: trimesh.util.concatenate([trimesh.Trimesh(v, f, process=False) for v, f in lst]) for c, lst in out.items()}


COL = dict(shell=(0.93, 0.93, 0.91), plate=(0.55, 0.57, 0.60), cage=(0.78, 0.80, 0.83), blade=(0.16, 0.17, 0.19), hub=(0.28, 0.29, 0.31),
           motor=(0.12, 0.30, 0.55), elev=(0.85, 0.60, 0.20), rods=(0.70, 0.72, 0.75), cradle=(0.30, 0.55, 0.85), board=(0.08, 0.40, 0.22),
           lens=(0.85, 0.18, 0.12), hs=(0.62, 0.63, 0.66), glass=(0.55, 0.78, 0.95), socket=(0.95, 0.55, 0.15), hbase=(0.30, 0.31, 0.34),
           port=(0.20, 0.21, 0.23))


def body_parts(pose):
    """Body CAD in the body frame (m): name -> (trimesh, colour key, group)."""
    out = {}
    out["shell"] = (tm(TC.shell_with_windows()), "shell", "turret shell")
    out["plate"] = (tm(S.plate_solid()), "plate", "body")
    out["cage"] = (tm(S.cage_solid()), "cage", "scabbard cage")
    out["motor"] = (tm(S.motor_solid()), "motor", "belly rotor")
    out["pins"] = (tm(S.four(S.latch_pin(0.0), 45.0)), "rods", "scabbard cage")
    A = TC.assembly()
    grp = dict(cradle="cradle", bridge="cradle", bezel="cradle", board="board", module="lens", heatsink="hs", disc="glass")
    for key, ck in grp.items():
        ms = [v for k, v in A.items() if k.endswith("_" + key)]
        if ms: out["cam_" + key] = (tm(S.union(ms)), ck, "cameras" if ck != "glass" else "turret shell")
    # arm portholes (dark collars where the arms leave the mantle, as rev 4)
    sc = Scene()
    from geom import frustum
    for i in range(8):
        a = np.radians(22.5 + 45 * i); d = np.array([np.cos(a), np.sin(a), 0])
        sc.add(frustum(d * 0.100 + [0, 0, 0.039], d * 0.110 + [0, 0, 0.039], 0.031, 0.031, list(COL["port"]), n=24))
    for c, m in scene_groups(sc).items():
        out["portholes"] = (m, "port", "body")
    return out


# blade modes (scabbard.py): droop beta, lift u through the trunnion, push rods, elevator lift
BLADE_MODES = {"stowed": dict(beta=S.BETA_H, u=S.U_STOW, rods=S.BETA_H, elev=S.U_STOW),          # upright in the turret
               "bars": dict(beta=S.BETA_H, u=S.U_DOCK, rods=S.BETA_H, elev=S.U_DOCK),          # slid down 108 mm: dock / centaur bars
               "legs": dict(beta=S.BETA_L, u=0.0, rods=S.BETA_L, elev=14.0),                    # seated, folded to 60 deg: tripod
               "propeller": dict(beta=S.BETA_F, u=0.0, rods=None, elev=14.0)}                   # seated, flight stop: the belly rotor


def rotor_parts(mode):
    m = BLADE_MODES[mode]
    rot = S.rotating(m["beta"], m["u"], psi=0.0)
    rods, zt = S.rods_solid(m["rods"])
    return {"hub": (tm(rot.pop("hub")), "hub", "belly rotor"),
            "barrels": (tm(S.union([v for k, v in rot.items() if k.startswith("barrel")])), "hub", "belly rotor"),
            "blades": (tm(S.union([v for k, v in rot.items() if k.startswith("blade")])), "blade", "belly rotor"),
            "rods": (tm(rods + S.fold_ring(zt)), "rods", "scabbard cage"),
            "elevator": (tm(S.elevator_solid(m["elev"])), "elev", "scabbard cage")}


def build_rotor(mode):
    gl = trimesh.Scene()
    for name, (m, ck, grp) in rotor_parts(mode).items():
        gl.add_geometry(finish(m, COL[ck]), node_name=f"{grp}|{name}", geom_name=f"{grp}|{name}")
    gl.export(f"cephalo2_rotor_{mode}.glb")
    print("rotor", mode, round(os.path.getsize(f"cephalo2_rotor_{mode}.glb") / 1e6, 2), "MB")


def bayonets(q):
    """Arm socket + hand base on every arm tip, at the rev 4 spigot (the pod now starts LEN_BASE further out)."""
    sock, base = tm(HM.arm_socket()), tm(HM.hand_base())
    socks, bases = [], []
    for i in range(8):
        segs, (Rt, pt) = C.arm_fk(i, q[i])
        u = Rt[:, 0]
        Rc = np.stack([Rt[:, 1], np.cross(-u, Rt[:, 1]), -u], 1)        # connector x, y, z (z into the arm)
        T = np.eye(4); T[:3, :3] = Rc; T[:3, 3] = pt
        a = sock.copy(); a.apply_transform(T); socks.append(a)
        b = base.copy(); b.apply_transform(T); bases.append(b)
    return trimesh.util.concatenate(socks), trimesh.util.concatenate(bases)


def arms(pose):
    P4 = json.load(open("poses4.json"))
    q = np.tile(np.radians(P4[pose]), (8, 1))
    fan = pose == "flight"
    sc = Scene()
    for i in range(8):
        fold = 0.0 if fan else C.FOLD_STAND
        V.arm_mesh(sc, i, np.zeros(3), np.eye(3), q[i], np.zeros(C.NJ), spin=0.35 * i, fold=fold, fan_on=fan)
    return scene_groups(sc), q


def finish(m, rgb, alpha=1.0, budget=12000):
    """Merge vertices, decimate heavy meshes (quadric, keeps the shape), give it one PBR material (no per-face colours)."""
    m = m.copy(); m.merge_vertices()
    if len(m.faces) > budget:
        import fast_simplification as FS
        v, f = FS.simplify(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int32), target_reduction=1 - budget / len(m.faces))
        m = trimesh.Trimesh(v, f, process=False)
    mat = trimesh.visual.material.PBRMaterial(baseColorFactor=[*[float(x) for x in rgb], alpha], metallicFactor=0.1, roughnessFactor=0.6,
                                              alphaMode="BLEND" if alpha < 1 else "OPAQUE", doubleSided=True)
    m.visual = trimesh.visual.TextureVisuals(material=mat)
    return m


def build(pose):
    gl = trimesh.Scene()
    parts = {}
    for name, (m, ck, grp) in body_parts(pose).items():
        m = finish(m, COL[ck], 0.45 if ck == "glass" else 1.0)
        gl.add_geometry(m, node_name=f"{grp}|{name}", geom_name=f"{grp}|{name}")
        parts[name] = len(m.faces)
    groups, q = arms(pose)
    for k, (c, m) in enumerate(groups.items()):
        m = finish(m, c, budget=16000 if HANDS == "s3" else 5000)
        gl.add_geometry(m, node_name=f"arms and hands|c{k}", geom_name=f"arms and hands|c{k}")
    s, b = bayonets(q)
    for name, m, ck in (("sockets", s, "socket"), ("hand bases", b, "hbase")):
        m = finish(m, COL[ck], budget=7000)
        gl.add_geometry(m, node_name=f"bayonet connectors|{name}", geom_name=f"bayonet connectors|{name}")
    ntri = sum(len(g.faces) for g in gl.geometry.values())
    gl.export(f"cephalo2_{HANDS}_{pose}.glb")
    print(HANDS, pose, "triangles", ntri, "MB", round(os.path.getsize(f"cephalo2_{HANDS}_{pose}.glb") / 1e6, 2))
    return ntri


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "rotor":
        for mode in BLADE_MODES: build_rotor(mode)
        sys.exit()
    info = {p: build(p) for p in ("stand", "flight")}
    json.dump(info, open(f"cephalo2_parts_{HANDS}.json", "w"))
