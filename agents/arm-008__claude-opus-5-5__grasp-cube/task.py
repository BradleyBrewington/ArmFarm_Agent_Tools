"""Pick/place primitives for the black-cube task (robot venv).

Conventions found on arm-008 (2026-10-05):
  * top camera: robot +x is image down, robot +y is image right (mirrored view).
  * the FK tip (gripper_frame_link) is on the fixed jaw. The jaws close along gripper_link y;
    the fixed jaw is on its +y side. Its horizontal angle is ~87 + roll - pan degrees, so roll
    for a given cube yaw is solved with FK (grasp_plan), never as a world angle.
  * the gripper Torque_Limit is capped (arm.GRIP_TORQUE) so a held cube never trips overload.
"""
import math
import time

import numpy as np

from kin import fk_T, ik_down
import vision

HOVER_Z = 0.07
GRASP_Z = 0.014
OPEN = 80.0
CLOSED = 0.0
GRIP_EMPTY = 12.0      # gripper % at/below which the jaws closed on nothing
JAW_OFFSET = 0.026     # FK tip is on the fixed jaw; cube centre sits this far toward the moving jaw


PITCH = 20.0           # fixed outward approach tilt: reaches x 0.14-0.32 at every height used


def plan(x, y, z, roll, pitch=None):
    """Tip at (x, y, z), approach tilted `pitch` from vertical (fixed PITCH by default)."""
    for p in ((PITCH, 0, 30, 40, -15) if pitch is None else (pitch,)):
        q, pe, ae = ik_down(x, y, z, pitch=p, roll=roll)
        if pe < 0.002 and ae < 1.0:
            return {**q, "wrist_roll": roll}
    raise ValueError(f"unreachable ({x:.3f}, {y:.3f}, {z:.3f})")


def closing_axis(q):
    """Horizontal unit vector from the moving jaw toward the fixed jaw (gripper_link +y)."""
    T = fk_T(q)
    v = np.array([T[0, 1], T[1, 1]])
    return v / np.linalg.norm(v)


def grasp_plan(cx, cy, z, yaw, roll_hint=None):
    """Joints that put the cube centre (cx, cy) between the jaws with the closing axis parallel
    to a cube face (cube yaw `yaw` deg, 90-deg symmetric). Returns (joints, roll)."""
    roll = 0.0 if roll_hint is None else roll_hint
    tx, ty = cx, cy
    for _ in range(4):
        q = plan(tx, ty, z, roll)
        u = closing_axis(q)
        ang = math.degrees(math.atan2(u[1], u[0]))
        if roll_hint is None:
            d = (yaw - ang) % 90.0
            d = d - 90.0 if d >= 45 else d
            roll = roll + d
            roll = roll - 90.0 if roll > 60 else roll + 90.0 if roll < -60 else roll
        tx, ty = cx + JAW_OFFSET * u[0], cy + JAW_OFFSET * u[1]
    q = plan(tx, ty, z, roll)
    return q, roll


def go_look(arm, seconds=1.2):
    """Home with wrist roll 0 and gripper open: the fixed pose the arm mask was captured in."""
    arm.move({**arm.home_pose, "wrist_roll": 0.0, "gripper": OPEN}, seconds)
    arm.wait(arm.home_pose, tol=4.0, timeout=1.5)


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


def pick(arm, x, y, yaw, fast=1.0, yaw_fix=True):
    """Open above the cube, square the jaws to it with the wrist camera, descend, close, lift.
    yaw: cube face angle in base_link degrees. Returns gripper % after lifting."""
    _, roll = grasp_plan(x, y, GRASP_Z, yaw)
    for i in range(3 if yaw_fix else 1):
        above, _ = grasp_plan(x, y, HOVER_Z, yaw, roll_hint=roll)
        arm.move({**above, "gripper": OPEN}, (1.3 if i == 0 else 0.4) * fast)
        arm.wait(above, tol=2.5, timeout=0.8)
        if not yaw_fix:
            break
        time.sleep(0.12)
        err = wrist_yaw_error()
        if err is None or abs(err) < 5:
            break
        roll = max(-90.0, min(90.0, roll - 1.3 * err))
    down, _ = grasp_plan(x, y, GRASP_Z, yaw, roll_hint=roll)
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


def place(arm, x, y, yaw=None, fast=1.0, z=None):
    """Lower the held cube so its centre lands at (x, y), open, retreat. Returns the cube
    centre implied by the measured tip (fixed jaw) and closing axis."""
    roll = arm.joints()["wrist_roll"] if yaw is None else None
    if yaw is None:
        q, _ = grasp_plan(x, y, GRASP_Z, 0.0, roll_hint=roll)
    else:
        q, roll = grasp_plan(x, y, GRASP_Z, yaw)
    above, _ = grasp_plan(x, y, HOVER_Z, 0.0, roll_hint=roll)
    down, _ = grasp_plan(x, y, GRASP_Z + 0.004 if z is None else z, 0.0, roll_hint=roll)
    arm.move(above, 1.3 * fast)
    arm.wait(above, tol=4, timeout=0.6)
    arm.move(down, 0.6 * fast)
    arm.wait(down, tol=3, timeout=0.6)
    j = arm.joints()
    t = fk_T(j)
    u = closing_axis(j)
    arm.move({"gripper": OPEN}, 0.3)
    time.sleep(0.2)
    arm.move(above, 0.5 * fast)
    return float(t[0, 3] - JAW_OFFSET * u[0]), float(t[1, 3] - JAW_OFFSET * u[1])


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


RETRY_OFFSETS = [(0.0, 0.0), (0.010, 0.0), (-0.008, 0.0), (0.018, 0.0), (0.0, 0.008), (0.0, -0.008)]


def pick_robust(arm, attempts=6, log=print):
    """Look, pick, verify by gripper opening; on a miss re-look and try a nudged estimate.
    Returns (ok, info dict)."""
    for i in range(attempts):
        go_look(arm, 1.0 if i == 0 else 0.8)
        time.sleep(0.25)
        d, p = detect_robot(True)
        if p is None:
            clear_view(arm); time.sleep(0.25)
            d, p = detect_robot(False)
        if p is None:
            log("cube not visible")
            return False, {"reason": "not_visible"}
        x, y, yaw = p
        ox, oy = RETRY_OFFSETS[i % len(RETRY_OFFSETS)]
        try:
            g = pick(arm, x + ox, y + oy, yaw)
        except ValueError as e:
            log(f"unreachable cube estimate {x:.3f},{y:.3f}: {e}")
            return False, {"reason": "unreachable", "x": x, "y": y}
        log(f"pick try {i}: est ({x:.3f},{y:.3f}) yaw {yaw:.0f} offset ({ox},{oy}) grip {g:.1f}")
        if g > GRIP_EMPTY:
            return True, {"x": x, "y": y, "px": d["px"], "attempts": i + 1, "offset": (ox, oy), "grip": g}
    return False, {"reason": "missed"}
