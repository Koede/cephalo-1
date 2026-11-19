"""LOCAL WORKING SUBSET of claude/code/xlclaw.py (XL claw-prop: 7.5 in Snap-Digit on a T-Motor F90 2806.5).
Constants, frames and the simplified render meshes only; the MuJoCo claw model, the Claw class and the load check are
in the project file, which stays the single source of truth."""
import numpy as np

SC = 1.25
MM = 1e-3 * SC                          # model lengths are the 6 in numbers in mm, times SC
R_TIP, R_PIN, Z_PIN = 76.2, 13.0, 1.5
R_K0, R_K1 = 44.0, 61.0
PRECONE, FOLD1_MAX, FOLD2_MAX = 2.0, 85.0, 80.0
HUB_R, HUB_Z = 18.5, (-3.0, 3.0)
POST_R, POST_Z = 7.9, 8.5
HOLE = (15.6, 6.0)
SKIRT = dict(ri=14.5, ro=17.5, h=6.0, travel=20.0)
SKIRT_TOP_OPEN = -7.0
ANCHOR_R, FL_H, AN_H = R_K1 + 3.5, 3.0, 2.8
PIV_T, PIV_N = 8.5, 1.21
MOTOR = dict(name="T-Motor F90 2806.5 1300KV", d=33.4, L=34.7, m=46.6, mount="19 x 19 mm, 4 x M3", shaft=4.0)
SERVO = dict(name="Feetech STS3032", stall=0.441, m=20.0, horn=6.0e-3)
TAPE = dict(name="2 plies of 1/2 in tape-measure blade, waist 7.5 mm", t=0.12, w=7.5, plies=2, R=12.0)
BELL_Z = (-38.7, -4.0)
# itemised pod mass (g), as in the project file's loads(): 156 g
POD_MASS_G = dict(motor=46.6, blades=3 * 2.58 * SC ** 3, hub_aluminium=12.0 * SC ** 3, printed=6.3 * SC ** 3, bearing_6708ZZ=21.0, servo=20.0,
                  rods_pins_tendons=7.0, mount=10.0)


def pitch(r):
    return np.arctan(101.6 / (2 * np.pi * max(r, 10.0)))


def chord(r):
    c = np.interp(r, [13.0, 30.0, 76.2], [17.0, 16.0, 10.0])
    tip = np.clip((R_TIP - r) / 4.0, 0, 1)
    return c * (0.45 + 0.55 * np.sqrt(tip))


def thick(r):
    return np.interp(r, [13.0, 30.0, 45.0, 76.2], [4.0, 2.8, 2.2, 1.4])


def top_z(r):
    return 0.5 * max(thick(r), 1.2) + 0.12 + 0.04 * chord(r)


def nrm_at(r):
    ph = pitch(r)
    return np.array([0, -np.sin(ph), np.cos(ph)])


def rotm(ax, a):
    ax = np.asarray(ax, float) / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


N_K = nrm_at(R_K0)
P_K0 = np.array([R_K0, 0, 0]) + N_K * top_z(R_K0)
PIV = P_K0 + PIV_T * np.array([1.0, 0, 0]) + PIV_N * N_K
AX_K = np.cross([1.0, 0, 0], N_K)


def meshes():
    """Motor body only in this subset ('stator'); the blades, hub and slider come from xlclaw_cad."""
    import mesh3d as M3
    st = [M3.tube(np.array([[0, 0, BELL_Z[0] * 1e-3], [0, 0, BELL_Z[1] * 1e-3]]), MOTOR["d"] / 2e3, 30), M3.disc(np.array([0, 0, BELL_Z[1] * 1e-3]), np.array([0, 0, 1.0]), MOTOR["d"] / 2e3, 30)]
    for a in np.radians([45, 135, 225, 315]):
        e = np.array([np.cos(a), np.sin(a), 0])
        st.append(M3.box(e * (MOTOR["d"] / 2e3 + 0.0004) + np.array([0, 0, -0.012]), np.c_[e, np.cross([0, 0, 1.0], e), [0, 0, 1.0]], np.array([0.0006, 0.004, 0.006])))
    return dict(stator=np.concatenate(st))
