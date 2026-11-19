"""Rotatable 3D viewer of the whole Hippos-1 tail on its SFF mount, with joint sliders and pose presets.
python3 tail_viewer.py -> tail_viewer.html  (template tail_viewer_template.html; numbers from tail_checks.json)."""
import json, base64, numpy as np
import sff_core as H, tailmount as M, tail as TL

def pack(m, col, name, level, ghost=False):
    t = H.to_trimesh(m); t.merge_vertices()
    v = np.asarray(t.vertices, np.float32)[:, [0, 2, 1]] * np.array([1, 1, -1], np.float32)     # (x, z, -y): a rotation
    f = np.asarray(t.faces, np.uint32)
    return dict(name=name, level=level, color=col, ghost=ghost,
                v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode())

P = TL.parts()
parts = [
    pack(TL.rump_pad(), "#9aa3ad", "rump pad", -1),
    pack(M.rump_socket(), "#e0a030", "rump socket + sleeve", -1),
    pack(H.pin_board(), "#2e7d32", "pin board", -1),
    pack(H.hand_base(), "#2f6db3", "SFF base", 0),
    pack(M.tail_adapter(), "#5aa0e6", "tail adapter", 0),
    pack(P["root_cradle"], "#4f6f93", "root cradle", 0),
    pack(TL.servo_env(0), "#2b2b2b", "STS3095 (j0 pitch)", 0),
    pack(P["link0"], "#7fa7d9", "link 0", 1), pack(TL.servo_env(1), "#2b2b2b", "STS3250 (j1 yaw)", 1),
    pack(P["link1"], "#7fa7d9", "link 1", 2), pack(TL.servo_env(2), "#2b2b2b", "STS3215 (j2 pitch)", 2),
    pack(P["link2"], "#7fa7d9", "link 2", 3), pack(TL.servo_env(3), "#2b2b2b", "STS3215 (j3 yaw)", 3),
    pack(P["link3"], "#7fa7d9", "link 3", 4), pack(P["tip_socket"], "#e0a030", "tip SFF socket", 4),
]
ck = json.load(open("tail_checks.json"))
joints = [dict(z=float(TL.ZJ[k]), axis=TL.JOINTS[k][1], servo=TL.JOINTS[k][0],
               lim=list(ck["free_range_deg"].values())[k]["+"]) for k in range(4)]
data = dict(parts=parts, joints=joints, ck=ck)
html = open("tail_viewer_template.html").read().replace("/*DATA*/", json.dumps(data, default=float))
open("tail_viewer.html", "w").write(html)
print("written", round(len(html) / 1e6, 2), "MB")
