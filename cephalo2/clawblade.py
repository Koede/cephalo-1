"""LOCAL WORKING SUBSET of claude/code/clawblade.py (planform + tape constants only; the project file is the full one)."""
import numpy as np

R_TIP = 0.0762
R_PIN = 0.013


def chord(r):
    return np.interp(r, [R_PIN, 0.030, R_TIP], [0.017, 0.016, 0.010])


def thick(r):
    return np.interp(r, [R_PIN, 0.030, 0.045, R_TIP], [0.0040, 0.0028, 0.0022, 0.0014])


def pitch_deg(r):
    return np.degrees(np.arctan(0.1016 / (2 * np.pi * np.maximum(r, 0.01))))


TAPE_T = 0.12e-3
TAPE_W = 12.7e-3
TAPE_WAIST = 6.0e-3
TAPE_R = 12.0e-3
