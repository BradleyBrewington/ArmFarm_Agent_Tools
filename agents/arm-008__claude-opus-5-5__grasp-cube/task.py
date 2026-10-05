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
GRIP_EMPTY = 12.0      # gripper % at/below which the jaws closed on nothing
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
    for pitch in (0, 10, 20, 30, 40, 50, -15, -30):
        q, pe, ae = ik_down(x, y, z, pitch=pitch, roll=roll)
        if pe < 0.002 and ae < 1.0:
            return {**q, "wrist_roll": roll}
    raise ValueError(f"unreachable ({x:.3f}, {y:.3f}, {z:.3f})")


def roll_for(yaw_robot):
    """Wrist roll in [-45, 45) whose closing axis is parallel to a cube face normal."""
    r = (yaw_robot - 90.0) % 90.0
    return r - 90.0 if r >= 45 else r


def cube_pose(det):
    """Cube detection -> (x, y, yaw_deg) in base_link via the metric table model."""
    (cx, cy), (w, h), a = det["rect"]
    x, y = vision.cube_robot(det["px"])
    d = (math.cos(math.radians(a)) * 20, math.sin(math.radians(a)) * 20)
    x2, y2 = vision.cube_robot((det["px"][0] + d[0], det["px"][1] + d[1]))
    yaw = math.degrees(math.atan2(y2 - y, x2 - x))
    return x, y, yaw


def wrist_yaw_error():
    """Cube edge angle relative to the jaws in the wrist image (deg, folded), or None."""
    import wrist
    from camd_client import read_frame
    d = wrist.detect(read_frame("wrist")[0])
    return None if d is None else wrist.fold90(d["angle"])


def pick(arm, x, y, roll, fast=1.0, yaw_fix=True):
    """Open above the cube, square the jaws to it with the wrist camera, descend, close, lift.
    Returns (gripper % after lifting, roll used)."""
    for i in range(3 if yaw_fix else 1):
        tx, ty = jaw_target(x, y, roll)
        above = plan(tx, ty, HOVER_Z, roll)
        arm.move({**above, "gripper": OPEN}, (1.3 if i == 0 else 0.4) * fast)
        arm.wait(above, tol=2.5, timeout=0.8)
        if not yaw_fix:
            break
        time.sleep(0.12)
        err = wrist_yaw_error()
        if err is None or abs(err) < 5:
            break
        roll = max(-85.0, min(85.0, roll - 1.3 * err))
    down = plan(tx, ty, GRASP_Z, roll)
    arm.move(down, 0.6 * fast)
    arm.wait(down, tol=3, timeout=0.6)
    arm.hold()                       # torque-capped close; jaws need ~0.5 s to reach the cube
    t0 = time.monotonic()
    prev = arm.gripper_pos()
    while time.monotonic() - t0 < 1.0:
        time.sleep(0.08)
        g = arm.gripper_pos()
        if abs(g - prev) < 0.3 and time.monotonic() - t0 > 0.25:
            break
        prev = g
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


def _rot(v, roll):
    r = math.radians(roll)
    return np.array([math.cos(r) * v[0] - math.sin(r) * v[1], math.sin(r) * v[0] + math.cos(r) * v[1]])


def wrist_look(arm, x, y, roll, seconds=1.0):
    """Hover top-down over (x, y) and locate the cube with the wrist camera.
    Returns (cube_x, cube_y, cube_image_angle, detection) or None if not seen."""
    import wrist
    from camd_client import read_frame
    q = plan(x, y, wrist.HOVER_Z_SERVO, roll)
    arm.move({**q, "gripper": OPEN}, seconds)
    arm.wait(q, tol=1.5, timeout=1.0)
    time.sleep(0.15)
    tip = arm.tip()
    d = None
    for _ in range(3):
        d = wrist.detect(read_frame("wrist")[0])
        if d:
            break
        time.sleep(0.1)
    if not d:
        return None
    o = _rot(wrist.offset(d["px"]), roll)
    return tip["x"] + o[0], tip["y"] + o[1], d["angle"], d, tip


def servo(arm, x, y, roll, iters=3, tol=0.004):
    """Refine a cube estimate with the wrist camera. Returns (x, y, roll) or None."""
    import wrist
    ref = wrist.load().get("angle_ref", 0.0)
    for i in range(iters):
        r = wrist_look(arm, x, y, roll, seconds=1.0 if i == 0 else 0.5)
        if r is None:
            return None
        nx, ny, ang = r[0], r[1], r[2]
        moved = math.hypot(nx - x, ny - y)
        x, y = nx, ny
        droll = wrist.fold90(ang - ref)
        roll = max(-80.0, min(80.0, roll - droll * ROLL_SIGN))
        if moved < tol and abs(droll) < 6:
            break
    return x, y, roll


ROLL_SIGN = 1.0   # how a cube image-angle error maps to a wrist-roll correction (checked on arm)


CLEAR_XY = (0.24, 0.17)   # hover spot that keeps the arm off the near-base table area in the top view


def clear_view(arm, seconds=1.0):
    q = plan(CLEAR_XY[0], CLEAR_XY[1], 0.10, 0.0)
    arm.move({**q, "gripper": OPEN}, seconds)
    arm.wait(q, tol=4, timeout=0.8)


def detect_robot(use_mask, tries=4):
    from camd_client import read_frame
    for _ in range(tries):
        img, _ = read_frame("top")
        c = vision.candidates(img, use_mask)
        if c:
            d = max(c, key=lambda e: e["fill"] * min(e["area"], 7000))
            return d, cube_pose(d)
        time.sleep(0.15)
    return None, None


def push(arm, start, end, z=0.016, roll=0.0, step=0.01, speed=0.06):
    """Closed-gripper straight-line sweep along the table from start to end (for nudging the cube)."""
    q = plan(start[0], start[1], z + 0.03, roll)
    arm.move({**q, "gripper": CLOSED}, 1.2); arm.wait(q, tol=3, timeout=1)
    q = plan(start[0], start[1], z, roll)
    arm.move(q, 0.6); arm.wait(q, tol=3, timeout=1)
    n = max(1, int(math.hypot(end[0] - start[0], end[1] - start[1]) / step))
    for i in range(1, n + 1):
        f = i / n
        q = plan(start[0] + f * (end[0] - start[0]), start[1] + f * (end[1] - start[1]), z, roll)
        arm.move(q, step / speed)
    arm.wait(q, tol=3, timeout=1)
    q = plan(end[0], end[1], z + 0.03, roll)
    arm.move(q, 0.6)
