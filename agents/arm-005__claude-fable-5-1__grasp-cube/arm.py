#!/usr/bin/env python3
"""Arm motion library for the grasp-cube task (robot venv: /opt/armfarm/venv/bin/python).

    from arm import Arm, fk_T, ik_topdown
    with Arm() as arm:
        arm.home()                 # four home joints only (wrist_roll/gripper untouched)
        arm.goto_xyz(x, y, z)      # top-down tool pose at base_link metres
        arm.gripper(100)           # open (percent)

Kinematics reuse the embedded URDF from fk.py. The IK here is a 4-joint numeric
solver (pan, lift, elbow, wrist_flex) that keeps the tool pointing straight down,
which is what a top-down cube grasp needs; wrist_roll is passed through.
"""
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calibrate_workspace as cw  # noqa: E402
import fk as _fk  # noqa: E402

JOINTS = cw.JOINTS
ARM_JOINTS = cw.ARM_JOINTS
HOME_JOINTS = cw.HOME_JOINTS
HOME = cw.load_home(HERE.parent / "home_pose.json")

# URDF joint limits (degrees) for the IK solver.
_LIMITS = {}
for _j in ET.fromstring(_fk.URDF).findall("joint"):
    if _j.get("name") in ARM_JOINTS:
        _l = _j.find("limit")
        _LIMITS[_j.get("name")] = (math.degrees(float(_l.get("lower"))), math.degrees(float(_l.get("upper"))))


def _rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


_TREE = ET.fromstring(_fk.URDF)
_BY_CHILD = {j.find("child").get("link"): j for j in _TREE.findall("joint")}
_CHAIN = []
_link = "gripper_frame_link"
while _link != "base_link":
    _joint = _BY_CHILD[_link]
    _CHAIN.append(_joint)
    _link = _joint.find("parent").get("link")
_CHAIN.reverse()
_STEPS = []
for _joint in _CHAIN:
    _o = _joint.find("origin")
    _xyz = np.fromstring(_o.get("xyz", "0 0 0"), sep=" ")
    _r, _p, _y = np.fromstring(_o.get("rpy", "0 0 0"), sep=" ")
    _fixed = np.eye(4)
    _fixed[:3, :3] = _rot([0, 0, 1], _y) @ _rot([0, 1, 0], _p) @ _rot([1, 0, 0], _r)
    _fixed[:3, 3] = _xyz
    _axis = None if _joint.get("type") == "fixed" else np.fromstring(_joint.find("axis").get("xyz"), sep=" ")
    _STEPS.append((_joint.get("name"), _fixed, _axis))


def fk_T(joints):
    """Full 4x4 transform of gripper_frame_link in base_link for joint degrees."""
    T = np.eye(4)
    for name, fixed, axis in _STEPS:
        T = T @ fixed
        if axis is not None:
            moving = np.eye(4)
            moving[:3, :3] = _rot(axis, math.radians(float(joints[name])))
            T = T @ moving
    return T


def fk_xyz(joints):
    return fk_T(joints)[:3, 3]


def tool_axis(joints):
    """Approach direction: the wrist-roll axis pointing from wrist toward fingertip.

    gripper_frame_link is gripper_link flipped by pi about y, and the fingertip sits
    at -z of gripper_link, so +z of gripper_frame_link points wrist -> tip.
    """
    return fk_T(joints)[:3, 2]


MARGIN_DEG = 1.0


def ik_reach(x, y, z, wrist_roll=0.0, seed=None, bounds=None, max_pitch=25.0, step=5.0):
    """ik_topdown with vertical approach, falling back to a tilt toward the base when needed."""
    last = None
    pitch = 0.0
    while pitch <= max_pitch + 1e-9:
        try:
            q = ik_topdown(x, y, z, wrist_roll=wrist_roll, pitch_deg=pitch, seed=seed, bounds=bounds)
            return q, pitch
        except ValueError as exc:
            last = exc
            pitch += step
    raise ValueError(str(last))


def ik_topdown(x, y, z, wrist_roll=0.0, pitch_deg=0.0, seed=None, bounds=None):
    """Joint degrees placing the fingertip at (x,y,z) with the tool pointing down.

    pitch_deg tilts the approach away from vertical (0 = straight down).
    bounds: optional {joint: (lo, hi)} in calibrated degrees (motor range).
    Raises ValueError when no solution is within ~2 mm / ~3 degrees.
    """
    from scipy.optimize import least_squares
    target = np.array([x, y, z], dtype=float)
    pan0 = math.degrees(math.atan2(-y, x - 0.0388353))
    # desired approach axis: straight down, tilted by pitch toward the base->target direction
    horiz = np.array([x - 0.0388353, y, 0.0])
    horiz = horiz / (np.linalg.norm(horiz) + 1e-9)
    want = np.array([0, 0, -1.0]) * math.cos(math.radians(pitch_deg)) + horiz * math.sin(math.radians(pitch_deg))
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    # Motor calibration bounds (less a margin) are authoritative; the URDF limits are nominal.
    lo = np.array([(bounds or _LIMITS)[j][0] + MARGIN_DEG for j in names])
    hi = np.array([(bounds or _LIMITS)[j][1] - MARGIN_DEG for j in names])

    def joints_of(q):
        d = dict(zip(names, q))
        d["wrist_roll"] = wrist_roll
        return d

    def resid(q):
        d = joints_of(q)
        pos = fk_xyz(d)
        ax = tool_axis(d)
        return np.concatenate([(pos - target) * 1000.0, (ax - want) * 60.0])

    seeds = []
    if seed is not None:
        seeds.append([seed[j] for j in names])
    for s in ([pan0, -30, 45, 70], [pan0, 0, 30, 60], [pan0, -60, 80, 60], [pan0, 20, 10, 60], [pan0, -45, 60, 80]):
        seeds.append(np.clip(s, lo, hi))
    best = None
    for s in seeds:
        r = least_squares(resid, np.clip(s, lo, hi), bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=400)
        err_mm = np.linalg.norm(r.fun[:3])
        err_ax = np.linalg.norm(r.fun[3:]) / 60.0
        if best is None or r.cost < best[0].cost:
            best = (r, err_mm, err_ax)
        if err_mm < 1.0 and err_ax < 0.03:
            break
    r, err_mm, err_ax = best
    if err_mm > 2.0 or err_ax > 0.06:
        raise ValueError(f"unreachable top-down pose ({x:.3f},{y:.3f},{z:.3f}): pos err {err_mm:.1f} mm, axis err {err_ax:.3f}")
    return joints_of(r.x)


class Arm:
    def __init__(self, port=None):
        self.port = port or os.environ.get("ARMFARM_SERIAL_PORT") or cw.PORT
        self._cm = None
        self.bus = None

    def __enter__(self):
        self._cm = cw.connected_bus(self.port)
        self.bus = self._cm.__enter__()
        self.bounds = {}
        for j in ARM_JOINTS:
            c = self.bus.calibration[j]
            half = (c.range_max - c.range_min) * 180 / 4095
            self.bounds[j] = (-half, half)
        return self

    def __exit__(self, *exc):
        return self._cm.__exit__(*exc)

    # ---- state
    def read(self):
        return self.bus.sync_read("Present_Position", list(JOINTS))

    def xyz(self):
        return fk_xyz(self.read())

    def torque_on(self, joints):
        cw.enable_at_current_position(self.bus, list(joints))

    # ---- motion
    def move(self, target, seconds, settle=0.3):
        """Smooth interpolated move (clamped to motor limits, stall-guarded)."""
        self.torque_on(list(target))
        cw.move(self.bus, target, seconds)
        if settle:
            time.sleep(settle)
        return self.read()

    def home(self, seconds=2.0, settle=0.5):
        """Return the four home joints only; wrist_roll and gripper untouched."""
        self.move(dict(HOME), seconds, settle)
        now = self.read()
        err = {j: abs(now[j] - HOME[j]) for j in HOME_JOINTS}
        return now, err

    def gripper(self, percent, seconds=0.6, settle=0.4):
        self.move({"gripper": float(percent)}, seconds, settle)
        return self.read()["gripper"]

    def goto_xyz(self, x, y, z, wrist_roll=None, seconds=1.5, pitch_deg=None, settle=0.3):
        now = self.read()
        roll = now["wrist_roll"] if wrist_roll is None else wrist_roll
        if pitch_deg is None:
            q, pitch_deg = ik_reach(x, y, z, wrist_roll=roll, seed=now, bounds=self.bounds)
        else:
            q = ik_topdown(x, y, z, wrist_roll=roll, pitch_deg=pitch_deg, seed=now, bounds=self.bounds)
        self.last_pitch = pitch_deg
        target = {j: q[j] for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")}
        if wrist_roll is not None:
            target["wrist_roll"] = roll
        return self.move(target, seconds, settle)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("read")
    sub.add_parser("home")
    g = sub.add_parser("grip"); g.add_argument("percent", type=float)
    m = sub.add_parser("xyz"); m.add_argument("x", type=float); m.add_argument("y", type=float); m.add_argument("z", type=float)
    m.add_argument("--roll", type=float); m.add_argument("--seconds", type=float, default=2.0); m.add_argument("--pitch", type=float, default=None)
    i = sub.add_parser("ik"); i.add_argument("x", type=float); i.add_argument("y", type=float); i.add_argument("z", type=float); i.add_argument("--pitch", type=float, default=0.0)
    j = sub.add_parser("joints"); j.add_argument("spec", help='JSON like {"wrist_roll": 90}'); j.add_argument("--seconds", type=float, default=1.5)
    a = p.parse_args()
    if a.cmd == "ik":
        print(json.dumps(ik_topdown(a.x, a.y, a.z, pitch_deg=a.pitch), indent=1)); sys.exit()
    with Arm() as arm:
        if a.cmd == "read":
            now = arm.read(); print(json.dumps(now, indent=1)); print("xyz", fk_xyz(now).round(4).tolist(), "axis", tool_axis(now).round(3).tolist())
        elif a.cmd == "home":
            print(json.dumps(arm.home(), indent=1))
        elif a.cmd == "grip":
            print(arm.gripper(a.percent))
        elif a.cmd == "xyz":
            print(json.dumps(arm.goto_xyz(a.x, a.y, a.z, wrist_roll=a.roll, seconds=a.seconds, pitch_deg=a.pitch), indent=1))
            print("xyz", arm.xyz().round(4).tolist())
        elif a.cmd == "joints":
            print(json.dumps(arm.move(json.loads(a.spec), a.seconds), indent=1))
