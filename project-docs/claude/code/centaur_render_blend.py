"""Cycles render of scene.npz (from render_scene.py).  python3 render_blend.py scene.npz out_prefix [samples] [views...]
Views: hero (3/4 front-left), rear (3/4 rear-right, the tail), side (left side, long lens), tail (close-up of the tail mount)."""
import sys, math, numpy as np, bpy, bmesh
from mathutils import Vector

src, prefix = sys.argv[1], sys.argv[2]
SAMPLES = int(sys.argv[3]) if len(sys.argv) > 3 else 96
VIEWS = sys.argv[4:] or ["hero", "rear", "side", "tail"]
bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene
scn.render.engine = "CYCLES"; scn.cycles.device = "CPU"; scn.cycles.samples = SAMPLES
scn.cycles.use_adaptive_sampling = True; scn.cycles.adaptive_threshold = 0.02
scn.cycles.use_denoising = True
scn.cycles.max_bounces = 6; scn.cycles.transparent_max_bounces = 8
scn.render.resolution_x, scn.render.resolution_y = 1920, 1200
import os; scn.render.resolution_percentage = int(os.environ.get("RES_PCT", "100"))
scn.render.film_transparent = False
scn.view_settings.view_transform = "AgX"; scn.view_settings.look = "AgX - Medium High Contrast"

def hexrgb(h):
    h = h.lstrip("#"); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [((x + 0.055) / 1.055) ** 2.4 if x > 0.04045 else x / 12.92 for x in c] + [1.0]

# material families: (roughness, metallic, coat, colour override)
FAM = {"shell": (0.42, 0.0, 0.25, None), "trim": (0.5, 0.0, 0.0, None), "metal_dark": (0.32, 0.55, 0.0, None),
       "steel": (0.25, 1.0, 0.0, None), "alu": (0.3, 1.0, 0.0, None), "plastic_blue": (0.45, 0.0, 0.0, None),
       "rubber": (0.8, 0.0, 0.0, None), "rubber_tpu": (0.7, 0.0, 0.0, None), "carbon": (0.3, 0.1, 0.3, None),
       "octo": (0.33, 0.0, 0.35, None), "octo_arm": (0.38, 0.0, 0.25, None), "prop": (0.2, 0.0, 0.0, None),
       "pa_cf": (0.62, 0.0, 0.0, "#2c2f34"), "petg_light": (0.4, 0.0, 0.2, "#dfe3e8"), "petg_blue": (0.38, 0.0, 0.2, "#4d77a6"),
       "servo": (0.3, 0.2, 0.3, "#17181a"), "pcb": (0.4, 0.0, 0.3, "#1f6b3a"),
       "c2_shell": (0.38, 0.0, 0.35, None), "c2_plate": (0.35, 0.6, 0.0, None), "petg_orange": (0.4, 0.0, 0.2, None),
       "petg_dark": (0.45, 0.0, 0.1, None), "petg_blue2": (0.4, 0.0, 0.15, None), "metal_paint": (0.3, 0.4, 0.4, None),
       "lens": (0.08, 0.0, 0.8, None), "glass": (0.03, 0.0, 0.0, None), "tape": (0.3, 0.9, 0.0, None),
       "servo_blue": (0.35, 0.0, 0.3, None), "tpu_orange": (0.65, 0.0, 0.0, None)}
_mats = {}
def material(fam, fc, alpha):
    key = (fam, fc, alpha)
    if key in _mats: return _mats[key]
    r, m, coat, over = FAM.get(fam, (0.45, 0.0, 0.0, None))
    mat = bpy.data.materials.new(f"{fam}_{fc}_{alpha}"); mat.use_nodes = True
    b = mat.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = hexrgb(over or fc)
    b.inputs["Roughness"].default_value = r; b.inputs["Metallic"].default_value = m
    b.inputs["Coat Weight"].default_value = coat; b.inputs["Coat Roughness"].default_value = 0.15
    if fam == "glass":
        b.inputs["Transmission Weight"].default_value = 0.95; b.inputs["IOR"].default_value = 1.58
    elif alpha < 0.99:
        b.inputs["Alpha"].default_value = alpha
    _mats[key] = mat
    return mat

def make_mesh(name, verts, faces, mat, smooth_deg=32.0, merge=True):
    me = bpy.data.meshes.new(name); me.from_pydata(verts.tolist(), [], faces.tolist()); me.update()
    bm = bmesh.new(); bm.from_mesh(me)
    if merge: bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=2e-6)
    bm.to_mesh(me); bm.free()
    me.shade_smooth(); me.set_sharp_from_angle(angle=math.radians(smooth_deg))
    ob = bpy.data.objects.new(name, me); scn.collection.objects.link(ob); ob.data.materials.append(mat)
    return ob

D = np.load(src, allow_pickle=False)
for k in D.files:
    if k.startswith("g") and not k.endswith("_meta"):
        fam, fc, a = D[k + "_meta"]; tri = D[k]
        v = tri.reshape(-1, 3); f = np.arange(len(v)).reshape(-1, 3)
        make_mesh(k, v, f, material(str(fam), str(fc), float(a)), smooth_deg=20.0 if fam in ("shell",) else 35.0)
    elif k.startswith("o") and k.endswith("_v"):
        i = k[1:-2]; fam, fc, a = (str(x) for x in D[f"o{i}_meta"])
        make_mesh(f"cad{i}", D[k], D[f"o{i}_f"], material(fam, fc, float(a)), smooth_deg=30.0, merge=True)
    elif k.startswith("m") and k.endswith("_v"):
        i = k[1:-2]; fam = str(D[f"m{i}_meta"][0])
        make_mesh(f"tail{i}", D[k], D[f"m{i}_f"], material(fam, "#888888", 1.0), smooth_deg=30.0, merge=False)

# ---- floor (a sweep so the horizon disappears), lights, world
bpy.ops.mesh.primitive_plane_add(size=12, location=(0, 0, 0)); fl = bpy.context.object
fm = bpy.data.materials.new("floor"); fm.use_nodes = True
fb = fm.node_tree.nodes["Principled BSDF"]; fb.inputs["Base Color"].default_value = hexrgb("#b4b8be"); fb.inputs["Roughness"].default_value = 0.85
fl.data.materials.append(fm)
bpy.ops.mesh.primitive_plane_add(size=12, location=(-4.5, 0, 3), rotation=(0, math.radians(90), 0)); bk = bpy.context.object; bk.data.materials.append(fm)
w = bpy.data.worlds.new("w"); scn.world = w; w.use_nodes = True
bg = w.node_tree.nodes["Background"]; bg.inputs["Color"].default_value = hexrgb("#dfe4ea"); bg.inputs["Strength"].default_value = 0.45
def area(loc, size, power, rot):
    bpy.ops.object.light_add(type="AREA", location=loc, rotation=rot); L = bpy.context.object
    L.data.size = size; L.data.energy = power; return L
def aim(ob, target):
    d = Vector(target) - ob.location; ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
key = area((1.6, 1.8, 2.4), 2.2, 680, (0, 0, 0)); aim(key, (0, 0, 0.45))
fill = area((-1.2, -2.2, 1.4), 3.0, 260, (0, 0, 0)); aim(fill, (0, 0, 0.45))
rim = area((-2.2, 1.0, 1.8), 1.5, 450, (0, 0, 0)); aim(rim, (0, 0, 0.6))
bpy.ops.object.light_add(type="SUN", location=(0, 0, 5)); sun = bpy.context.object
sun.data.energy = 1.6; sun.data.angle = math.radians(8); sun.rotation_euler = (math.radians(35), math.radians(-20), math.radians(35))

# ---- cameras
CAMS = {"hero": ((2.05, 1.85, 1.05), (-0.05, 0.0, 0.62), 42), "rear": ((-2.15, -1.55, 0.95), (-0.15, 0.0, 0.58), 42),
        "side": ((-0.10, 3.30, 0.70), (-0.10, 0.0, 0.62), 50), "tail": ((-1.20, -0.80, 0.80), (-0.40, 0.0, 0.42), 55), "tailwide": ((-1.70, -1.30, 1.05), (-0.50, 0.0, 0.55), 42),
        "hands": ((1.22, 0.62, 0.78), (0.58, 0.0, 0.55), 50), "head": ((0.95, -0.85, 1.25), (0.22, 0.0, 0.86), 50)}
bpy.ops.object.camera_add(); cam = bpy.context.object; scn.camera = cam
cam.data.sensor_width = 36
for v in VIEWS:
    loc, tgt, lens = CAMS[v]
    cam.location = loc; aim(cam, tgt); cam.data.lens = lens
    cam.data.dof.use_dof = v in ("tail", "hands", "head"); cam.data.dof.focus_distance = (Vector(loc) - Vector(tgt)).length; cam.data.dof.aperture_fstop = 5.6
    scn.render.filepath = f"{prefix}_{v}.png"
    bpy.ops.render.render(write_still=True)
    print("rendered", scn.render.filepath, flush=True)
