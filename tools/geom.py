"""Mesh-level pose checks on top of pin_cad's Manifold bodies (interference volumes in mm^3)."""
import math
import numpy as np
from common import add_paths

add_paths()

OVERLAP_MM3 = 2.0       # same contact threshold pin_cad.py uses for its sweeps


def pose_overlaps(arm, bodies, q, pairs=None, min_vol=None):
    """interference volume of link pairs at joint angles q (radians, length J).

    pairs: iterable of (i, k) link indices; default = all non-identical pairs within 2 links
    (neighbours i,i+1 and next-neighbours i,i+2). Returns {(i, k): volume}.
    """
    from pin_cad import rot_about_line  # noqa  (arm.pose_frames uses it)
    J = arm.J
    if pairs is None:
        pairs = [(i, i + d) for d in (1, 2) for i in range(J + 1 - d)]
    W = arm.pose_frames(list(q))
    out = {}
    cache = {}
    def placed(i):
        if i not in cache:
            cache[i] = bodies[i].transform(W[i][:3, :])
        return cache[i]
    for i, k in pairs:
        # skip pairs whose bounding boxes do not touch (cheap prefilter)
        a, b = placed(i), placed(k)
        ba, bb = a.bounding_box(), b.bounding_box()
        if any(ba[d] > bb[d + 3] or bb[d] > ba[d + 3] for d in range(3)):
            out[(i, k)] = 0.0
            continue
        out[(i, k)] = float((a ^ b).volume())
    return out


def worst(ov, thresh=OVERLAP_MM3):
    """(pair, volume) of the worst overlap above thresh, or (None, 0.0)"""
    bad = [(v, p) for p, v in ov.items() if v > thresh]
    if not bad:
        return None, 0.0
    v, p = max(bad)
    return p, v
