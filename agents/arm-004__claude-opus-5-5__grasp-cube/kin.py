"""Full-pose FK and top-down IK for the SO101, built on tools/fk.py URDF."""
import math, sys, os
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import fk as _fk

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _rot(axis, angle):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _chain():
    tree = ET.fromstring(_fk.URDF)
    by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
    chain, link = [], "gripper_frame_link"
    while link != "base_link":
        j = by_child[link]; chain.append(j); link = j.find("parent").get("link")
    out = []
    for j in reversed(chain):
        o = j.find("origin")
        xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(o.get("rpy", "0 0 0"), sep=" ")
        F = np.eye(4); F[:3, :3] = _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r); F[:3, 3] = xyz
        axis = None if j.get("type") == "fixed" else np.fromstring(j.find("axis").get("xyz"), sep=" ")
        out.append((j.get("name"), F, axis))
    return out

CHAIN = _chain()


def pose(joints):
    T = np.eye(4)
    for name, F, axis in CHAIN:
        T = T @ F
        if axis is not None:
            M = np.eye(4); M[:3, :3] = _rot(axis, math.radians(float(joints[name]))); T = T @ M
    return T


LIMITS = {"shoulder_pan": (-110, 110), "shoulder_lift": (-100, 100), "elbow_flex": (-96, 96),
          "wrist_flex": (-100, 100)}


def ik_down(x, y, z, pitch_deg=0.0, seed=None):
    """Solve pan/lift/elbow/wrist_flex so the tool point is at xyz and the tool
    approach axis points straight down (tilted by pitch_deg toward the base if needed)."""
    from scipy.optimize import least_squares
    target = np.array([x, y, z])
    names = ARM[:4]
    pan = -math.degrees(math.atan2(y, x - 0.0388353))   # pan is negative toward +y
    best = None
    seeds = [seed] if seed is not None else []
    seeds += [[pan, 0, 0, 60], [pan, -30, 40, 70], [pan, 30, -30, 80], [pan, -60, 60, 60]]
    lo = [LIMITS[n][0] for n in names]; hi = [LIMITS[n][1] for n in names]
    for s in seeds:
        s = np.clip(np.asarray(s, float)[:4], lo, hi)
        def res(q):
            J = dict(zip(names, q)); J["wrist_roll"] = 0.0
            T = pose(J)
            a = T[:3, :3] @ APPROACH
            want = np.array([0, 0, -1.0])
            if pitch_deg:
                radial = np.array([x, y, 0.0]); radial /= np.linalg.norm(radial)
                p = math.radians(pitch_deg)
                want = np.array([0, 0, -math.cos(p)]) + radial * math.sin(p)
            return np.concatenate([(T[:3, 3] - target) * 100.0, (a - want) * 1.0])
        r = least_squares(res, s, bounds=(lo, hi))
        err = np.linalg.norm(r.fun[:3]) / 100.0
        ang = np.linalg.norm(r.fun[3:])
        if best is None or err + ang * 0.01 < best[0]:
            best = (err + ang * 0.01, r.x, err, ang)
    _, q, err, ang = best
    if abs(q[0] - pan) > 30:
        err = max(err, 1.0)   # wrong-side solution: report as unreachable
    return dict(zip(names, map(float, q))), err, ang

APPROACH = np.array([0, 0, 1.0])  # placeholder, set after inspection
