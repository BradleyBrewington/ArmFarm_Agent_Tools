"""Full-pose FK and a top-down grasp IK for the SO101 (robot venv; no placo needed).

fk_T(joints) -> 4x4 gripper_frame_link pose in base_link (joints in calibrated degrees).
The frame's +z axis points out along the fingers (approach direction).
ik_down(x, y, z, pitch=0) -> joint dict whose tip reaches xyz with the approach axis
tilted `pitch` degrees from straight down (toward the robot base is negative).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

from fk import URDF

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _rot(axis, a):
    axis = np.asarray(axis, float); axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _chain():
    tree = ET.fromstring(URDF)
    by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
    chain, link = [], "gripper_frame_link"
    while link != "base_link":
        j = by_child[link]; chain.append(j); link = j.find("parent").get("link")
    out = []
    for j in reversed(chain):
        o = j.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        r, p, y = [float(v) for v in o.get("rpy", "0 0 0").split()]
        F = np.eye(4); F[:3, :3] = _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r); F[:3, 3] = xyz
        axis = None if j.get("type") == "fixed" else np.array([float(v) for v in j.find("axis").get("xyz").split()])
        out.append((j.get("name"), F, axis))
    return out


CHAIN = _chain()


def fk_T(joints):
    T = np.eye(4)
    for name, F, axis in CHAIN:
        T = T @ F
        if axis is not None:
            M = np.eye(4); M[:3, :3] = _rot(axis, math.radians(float(joints.get(name, 0.0)))); T = T @ M
    return T


LIMITS = {"shoulder_pan": (-110, 110), "shoulder_lift": (-104, 104), "elbow_flex": (-97, 97), "wrist_flex": (-101, 101)}


def ik_down(x, y, z, pitch=0.0, roll=0.0, seed=None):
    """Tip at (x,y,z) with approach axis pointing down, tilted by `pitch` deg in the radial plane."""
    target = np.array([x, y, z], float)
    pan_dir = np.array([x - 0.0388, y]); pan_dir /= max(np.linalg.norm(pan_dir), 1e-9)
    p = math.radians(pitch)
    want = np.array([pan_dir[0] * math.sin(p), pan_dir[1] * math.sin(p), -math.cos(p)])
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    lo = [LIMITS[n][0] for n in names]; hi = [LIMITS[n][1] for n in names]

    def res(q):
        T = fk_T({**dict(zip(names, q)), "wrist_roll": roll})
        return np.concatenate([(T[:3, 3] - target) * 100.0, (T[:3, 2] - want) * 1.0])

    pan0 = math.degrees(-math.atan2(y, x - 0.0388))
    seeds = [seed] if seed else []
    seeds += [[pan0, 0, 0, 60], [pan0, -40, 40, 80], [pan0, 30, -30, 70], [pan0, -80, 80, 60]]
    best = None
    for s in seeds:
        s = np.clip(s, lo, hi)
        r = least_squares(res, s, bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
        if best is None or r.cost < best.cost:
            best = r
        if r.cost < 1e-8:
            break
    q = dict(zip(names, best.x))
    T = fk_T({**q, "wrist_roll": roll})
    pos_err = float(np.linalg.norm(T[:3, 3] - target))
    ang_err = math.degrees(math.acos(np.clip(T[:3, 2] @ want, -1, 1)))
    return q, pos_err, ang_err
