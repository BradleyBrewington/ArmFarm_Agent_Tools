"""Pick/place primitives for the black-cube task (robot venv).

Conventions found on arm-008 (2026-10-05):
  * top camera: robot +x is image down, robot +y is image right.
  * jaws close along robot +y at wrist_roll 0; closing axis angle = roll + 90 deg.
"""
import math
import time

import numpy as np

from kin import ik_down
import vision

HOVER_Z = 0.07
GRASP_Z = 0.014
OPEN = 80.0
CLOSED = 0.0
GRIP_EMPTY = 6.0       # gripper % at/below which the jaws closed on nothing
JAW_OFFSET = 0.026     # FK tip is on the fixed jaw; cube centre sits this far toward the moving jaw


def jaw_target(x, y, roll):
    """Tip XY that puts the cube centre (x, y) between the jaws."""
    r = math.radians(roll)
    return x - JAW_OFFSET * math.sin(r), y + JAW_OFFSET * math.cos(r)


def go_look(arm, seconds=1.2):
    """Home with wrist roll 0 and gripper open: the fixed pose the arm mask was captured in."""
    arm.move({**arm.home_pose, "wrist_roll": 0.0, "gripper": OPEN}, seconds)
    arm.wait(arm.home_pose, tol=4.0, timeout=1.5)


def plan(x, y, z, roll):
    """Top-down if reachable, else the smallest outward pitch that is."""
    for pitch in (0, 10, 20, 30, 40, 50):
        q, pe, ae = ik_down(x, y, z, pitch=pitch, roll=roll)
        if pe < 0.002 and ae < 1.0:
            return {**q, "wrist_roll": roll}
    raise ValueError(f"unreachable ({x:.3f}, {y:.3f}, {z:.3f})")


def roll_for(yaw_robot):
    """Wrist roll in [-45, 45) whose closing axis is parallel to a cube face normal."""
    r = (yaw_robot - 90.0) % 90.0
    return r - 90.0 if r >= 45 else r


def cube_pose(det, H=None):
    """Cube detection -> (x, y, yaw_deg) in base_link using the cube map."""
    H = vision.load_map() if H is None else H
    (cx, cy), (w, h), a = det["rect"]
    x, y = vision.px_to_robot(det["px"], H)
    d = (math.cos(math.radians(a)) * 20, math.sin(math.radians(a)) * 20)
    x2, y2 = vision.px_to_robot((det["px"][0] + d[0], det["px"][1] + d[1]), H)
    yaw = math.degrees(math.atan2(y2 - y, x2 - x))
    return x, y, yaw


def pick(arm, x, y, roll, fast=1.0):
    x, y = jaw_target(x, y, roll)
    above = plan(x, y, HOVER_Z, roll)
    down = plan(x, y, GRASP_Z, roll)
    arm.move({**above, "gripper": OPEN}, 1.3 * fast)
    arm.wait(above, tol=4, timeout=0.6)
    arm.move(down, 0.6 * fast)
    arm.wait(down, tol=3, timeout=0.6)
    arm.move({"gripper": CLOSED}, 0.35)
    time.sleep(0.3)
    arm.hold()
    arm.move(above, 0.6 * fast)
    return arm.gripper_pos()


def place(arm, x, y, roll, fast=1.0, z=None):
    x, y = jaw_target(x, y, roll)
    above = plan(x, y, HOVER_Z, roll)
    down = plan(x, y, GRASP_Z + 0.004 if z is None else z, roll)
    arm.move(above, 1.3 * fast)
    arm.wait(above, tol=4, timeout=0.6)
    arm.move(down, 0.6 * fast)
    arm.wait(down, tol=3, timeout=0.6)
    j = arm.joints()
    tip = arm.tip(j)
    arm.move({"gripper": OPEN}, 0.3)
    time.sleep(0.15)
    arm.move(above, 0.5 * fast)
    return tip
