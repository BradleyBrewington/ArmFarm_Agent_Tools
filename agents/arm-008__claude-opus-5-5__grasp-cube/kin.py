"""SO101 kinematics with orientation: full FK transform and top-down IK.

The supplied ik.py is position-only; grasping needs the gripper pointing down.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

import fk as _fk

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
_TREE = ET.fromstring(_fk.URDF)
_BY_CHILD = {j.find("child").get("link"): j for j in _TREE.findall("joint")}
LIMITS = {}
for _j in _TREE.findall("joint"):
    if _j.get("type") == "revolute":
        lim = _j.find("limit")
        LIMITS[_j.get("name")] = (math.degrees(float(lim.get("lower"))), math.degrees(float(lim.get("upper"))))


def _rot(axis, a):
    axis = np.asarray(axis, float); axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _chain(link):
    chain = []
    while link != "base_link":
        j = _BY_CHILD[link]; chain.append(j); link = j.find("parent").get("link")
    out = []
    for j in reversed(chain):
        o = j.find("origin")
        xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(o.get("rpy", "0 0 0"), sep=" ")
        F = np.eye(4); F[:3, :3] = _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r); F[:3, 3] = xyz
        axis = None if j.get("type") == "fixed" else np.fromstring(j.find("axis").get("xyz"), sep=" ")
        out.append((j.get("name"), F, axis))
    return out


_TIP = _chain("gripper_frame_link")


def fk_T(joints, chain=_TIP):
    T = np.eye(4)
    for name, F, axis in chain:
        T = T @ F
        if axis is not None:
            M = np.eye(4); M[:3, :3] = _rot(axis, math.radians(float(joints.get(name, 0.0)))); T = T @ M
    return T


def ik_down(x, y, z, roll=0.0, tilt=0.0, seed=None):
    """Joints placing the tip at (x,y,z) with approach axis pointing down.

    tilt (deg) leans the approach away from vertical toward the base->target
    direction (useful far from the base). Returns (joints, residual_m).
    """
    target = np.array([x, y, z])
    base_dir = np.array([x - 0.0388353, y, 0.0]); base_dir /= max(np.linalg.norm(base_dir), 1e-9)
    t = math.radians(tilt)
    want = np.array([0, 0, -1.0]) * math.cos(t) + base_dir * math.sin(t)
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    lo = [LIMITS[n][0] for n in names]; hi = [LIMITS[n][1] for n in names]
    pan0 = math.degrees(-math.atan2(y, x - 0.0388353))
    seeds = [seed] if seed is not None else []
    seeds += [[pan0, -20, 60, 60], [pan0, 0, 30, 70], [pan0, -50, 90, 40], [pan0, 20, 0, 80]]
    best = None
    for s in seeds:
        s = np.clip(np.asarray(s, float), lo, hi)

        def res(q):
            J = dict(zip(names, q)); J["wrist_roll"] = roll
            T = fk_T(J)
            return np.concatenate([(T[:3, 3] - target) * 100.0, (T[:3, 2] - want) * 1.0])

        r = least_squares(res, s, bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
        J = dict(zip(names, r.x)); J["wrist_roll"] = roll
        err = float(np.linalg.norm(fk_T(J)[:3, 3] - target))
        ang = float(np.degrees(np.arccos(np.clip(fk_T(J)[:3, 2] @ want, -1, 1))))
        score = err * 1000 + ang * 0.2
        if best is None or score < best[0]:
            best = (score, J, err, ang)
    return best[1], best[2], best[3]


if __name__ == "__main__":
    import sys, json
    J, e, a = ik_down(*map(float, sys.argv[1:4]))
    print(json.dumps({k: round(v, 2) for k, v in J.items()}), f"err={e*1000:.1f}mm ang={a:.1f}deg")
