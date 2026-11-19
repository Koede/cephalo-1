"""Section through the locked tail mount (plane y = 0 and the plane through the latch tab).  -> tailmount_section.png"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import sff_core as H, tailmount as M
from sff_core import P
parts = [("rump socket + sleeve", M.rump_socket(), "#e0a030"), ("pin board", H.pin_board(), "#2e7d32"),
         ("standard SFF base", H.hand_base(), "#2f6db3"), ("pad board", H.pad_board(), "#c9a227"), ("tail adapter", M.tail_adapter(), "#5aa0e6")]
fig, axs = plt.subplots(1, 2, figsize=(16, 8.5), dpi=110)
for ax, az, ttl in ((axs[0], 0.0, "Section through a lug (az 0°)"), (axs[1], 128.5, "Section through the latch tab (az 128.5°)")):
    a = np.radians(az); n = np.array([-np.sin(a), np.cos(a), 0.0]); u = np.array([np.cos(a), np.sin(a), 0.0])
    for name, m, col in parts:
        t = H.to_trimesh(m)
        sec = t.section(plane_origin=[0, 0, 0], plane_normal=n)
        if sec is None: continue
        for ent in sec.entities:
            pts = sec.vertices[ent.points]
            xy = np.c_[pts @ u, pts[:, 2]]
            ax.add_patch(Polygon(xy, closed=True, fc=col, ec="k", lw=0.4, alpha=0.9))
    ax.set_title(ttl, loc="left", fontsize=13, fontweight="bold")
    ax.set_xlim(-36, 36); ax.set_ylim(-38, 14); ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("mm across the axis"); ax.set_ylabel("mm along the axis (+ into the rump)")
    ax.axhline(0, color="#888", lw=0.6, ls=":")
axs[0].text(-35, 12, "rump bulkhead above (not drawn)", fontsize=10, color="#666")
axs[0].annotate("skirt in sleeve\n0.10 mm radial", (24.2, -20), (27, -33), fontsize=10, arrowprops=dict(arrowstyle="->"))
axs[0].annotate("cone seats (lugs)", (11.5, 3.8), (-5, -20), fontsize=10, arrowprops=dict(arrowstyle="->"))
axs[0].annotate("45° conical floor\n(prints unsupported)", (-15, -24), (-35, -34), fontsize=10, arrowprops=dict(arrowstyle="->"))
axs[1].annotate("release tab, reached\nthrough both windows", (20, -5), (22, 8), fontsize=10, arrowprops=dict(arrowstyle="->"))
handles = [plt.Rectangle((0, 0), 1, 1, fc=c) for _, _, c in parts]
fig.legend(handles, [n for n, _, _ in parts], loc="lower center", ncol=5, fontsize=11, frameon=False)
fig.suptitle("Hippos-1 tail mount, locked: standard SFF lock + skirt-in-sleeve", x=0.02, ha="left", fontsize=16, fontweight="bold")
fig.tight_layout(rect=[0, 0.05, 1, 0.95]); fig.savefig("tailmount_section.png")
