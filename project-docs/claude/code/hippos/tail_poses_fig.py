"""Side-view sections (plane y = 0) of the tail in named poses -> tail_poses.png"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import sff_core as H, tailmount as M, tail as TL
P = TL.parts()
fixed = [(M.rump_socket(), "#e0a030"), (M.tail_adapter(), "#5aa0e6"), (H.hand_base(), "#2f6db3"), (P["root_cradle"], "#4f6f93"),
         (TL.servo_env(0), "#2b2b2b"), (TL.rump_pad(), "#9aa3ad")]
poses = [("rest, straight back", [0, 0, 0, 0]), ("curled over the croup", [-60, 0, -70, 0]),
         ("kickstand, crouched", [90, 0, 0, 0]), ("kickstand, reared (j0 at its 108° limit)", [108, 0, 27, 0])]
fig, axs = plt.subplots(1, 4, figsize=(18, 6.8), dpi=100)
for ax, (name, q) in zip(axs, poses):
    Ts = TL.pose_transforms(q); items = list(fixed)
    for k in range(4):
        items.append((TL.apply(P[f"link{k}"], Ts[k]), "#7fa7d9"))
        if k < 3: items.append((TL.apply(TL.servo_env(k + 1), Ts[k]), "#2b2b2b"))
    items.append((TL.apply(P["tip_socket"], Ts[3]), "#e0a030"))
    for m, col in items:
        sec = H.to_trimesh(m).section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
        if sec is None: continue
        for e in sec.entities:
            p = sec.vertices[e.points]
            ax.add_patch(Polygon(np.c_[-p[:, 2], p[:, 0]], closed=True, fc=col, ec="k", lw=0.3))   # horse view: x_h = -z_c (back = right), up = x_c
    ax.set_xlim(-40, 520); ax.set_ylim(-520, 300); ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_title(name, fontsize=12, fontweight="bold", loc="left"); ax.set_xlabel("mm behind the rump socket")
axs[0].set_ylabel("mm up from the socket axis")
fig.suptitle("Hippos-1 tail (pitch-yaw-pitch-yaw, 400 mm): section through the mid-plane", x=0.01, ha="left", fontsize=15, fontweight="bold")
fig.tight_layout(rect=[0, 0, 1, 0.94]); fig.savefig("tail_poses.png")
