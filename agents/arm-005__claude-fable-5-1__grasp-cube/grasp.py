#!/usr/bin/env python3
"""Cube grasp/place primitives on top of arm.py and vision.py.

Geometry learned on arm-005 (2026-10-05):
  * The IK frame (gripper_frame_link) is the static jaw tip.
  * The moving jaw swings open radially outward from the shoulder-pan axis, so a
    grasp puts the static jaw just outside the cube's inner face and lets the moving
    jaw sweep the cube in when closing.
  * Table surface is at z ~ -0.008 in FK terms; the cube is ~40 mm.

    from grasp import Grasper
    with Grasper() as g:
        g.grasp(x, y)   -> bool (gripper did not close fully)
        g.place(x, y)
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import arm as A  # noqa: E402
import vision as V  # noqa: E402
from camd_client import read_frame  # noqa: E402

PAN_AXIS = np.array([0.0388353, 0.0])
CUBE = 0.040            # cube edge, metres
GRASP_BACK = 0.030      # static jaw this far inside (toward base) of the cube centre (1 cm margin to the near face)
HELD_FORWARD = 0.020    # held cube centre sits this far radially out from the frame
Z_TABLE = -0.008
Z_GRASP = Z_TABLE + 0.018
Z_HOVER = 0.075
Z_CARRY = 0.09
OPEN = 85.0             # wider opening keeps the moving jaw clear of the far face with the larger margin
CLOSED = 0.0
EDGE_MIN = 21.0         # a squarely held 40 mm cube reads ~23%; less means an edge/corner grab
HELD_MIN = 6.0          # gripper % above which something is between the jaws (empty closes to ~0.3)
HOLD_SQUEEZE = 6.0      # (fallback) goal this many % below contact when not torque-limited
GRIP_TORQUE_LIMIT = 220 # RAM register, 0..1000 = 0..100% of max torque
LOG = HERE / "grasp_log.jsonl"


def radial(x, y):
    v = np.array([x, y]) - PAN_AXIS
    return v / (np.linalg.norm(v) + 1e-9)


BASE_ROLL = 83.3        # wrist_roll at which the jaws open exactly along the radial direction
ROLL_MIN, ROLL_MAX = 45.0, 125.0


def roll_for_cube(cx, cy, rect_angle_deg):
    """Wrist roll that squares the jaws to a cube whose minAreaRect side angle is given.

    Image angles are measured from image-down (+x robot) toward image-right (+y robot):
    the jaw direction is radial_angle + (roll - BASE_ROLL) (verified 2026-10-05 by snapshots at
    roll 48/83/118), and a square's side direction is 90 - rect_angle. Wrap the needed
    offset into [-45, 45] since the cube has 4-fold symmetry.
    """
    if rect_angle_deg is None:
        return BASE_ROLL
    radial_angle = math.degrees(math.atan2(cy, cx - PAN_AXIS[0]))
    edge_angle = 90.0 - float(rect_angle_deg)
    delta = ((edge_angle - radial_angle + 45.0) % 90.0) - 45.0
    # wrist roll has undocumented physical stops; 48..118 was verified safe on arm-005
    return min(ROLL_MAX, max(ROLL_MIN, BASE_ROLL + delta))


def log(event, **kw):
    rec = {"t": time.time(), "event": event, **kw}
    with LOG.open("a") as f:
        f.write(json.dumps(rec) + "\n")
    print(json.dumps(rec), flush=True)


class Grasper:
    def __init__(self):
        self.arm = A.Arm()
        self.map = V.TableMap.load()

    def __enter__(self):
        self.arm.__enter__()
        # Cap gripper output below the servo's 25% overload threshold (Overload_Torque=25)
        # so a fully-closed goal can hold the cube indefinitely without tripping protection.
        A.cw.write_register(self.arm.bus, "Torque_Limit", "gripper", GRIP_TORQUE_LIMIT)
        return self

    def __exit__(self, *exc):
        return self.arm.__exit__(*exc)

    # ---- perception
    def see_cube(self, retries=3):
        for _ in range(retries):
            img, _ = read_frame("top")
            c = V.detect_cube(img)
            if c:
                return c, img
            time.sleep(0.1)
        return None, img

    def lookout_see(self):
        """Move the arm aside so the top camera can see the base area, then detect without the
        home-silhouette mask (used when the cube is hiding under the parked gripper)."""
        self.arm.goto_xyz(0.16, 0.20, 0.12, seconds=1.5, settle=0.4)
        img, _ = read_frame("top")
        c = V.detect_cube(img, use_arm_mask=False)
        return c, img

    def cube_xy(self, c):
        return self.map.pixel_to_robot(c["u"], c["v"])

    # ---- motion primitives
    def hover(self, x, y, z=Z_HOVER, seconds=1.2):
        return self.arm.goto_xyz(x, y, z, seconds=seconds)

    def grasp(self, cx, cy, snapshots=None, rect_angle=None):
        """Grasp a cube centred at robot (cx, cy). Returns (held: bool, gripper %).

        rect_angle (deg, from vision.detect_cube) squares the jaws to the cube via wrist roll.
        """
        r = radial(cx, cy)
        fx, fy = np.array([cx, cy]) - GRASP_BACK * r
        roll = roll_for_cube(cx, cy, rect_angle)
        self.arm.gripper(OPEN, seconds=0.5, settle=0.1)
        for zh in (Z_HOVER, 0.055, Z_GRASP + 0.02):   # far targets cannot hover high at full reach
            try:
                self.arm.goto_xyz(fx, fy, zh, wrist_roll=roll, seconds=1.2, settle=0.1)
                break
            except ValueError:
                if zh == Z_GRASP + 0.02:
                    raise
        self.arm.goto_xyz(fx, fy, Z_GRASP + 0.02, wrist_roll=roll, seconds=0.6, settle=0.1)
        self.arm.goto_xyz(fx, fy, Z_GRASP, wrist_roll=roll, seconds=0.5, settle=0.2)
        if snapshots:
            self._snap(snapshots + "_pre")
        self.arm.move({"gripper": CLOSED}, 0.8, settle=0.3)
        g0 = self.arm.read()["gripper"]
        if g0 < EDGE_MIN:
            # jaws closed further than a squarely-held cube allows: we caught an edge/corner.
            # Let go in place rather than lifting and flinging it; the caller retries with an offset.
            self.arm.gripper(OPEN, seconds=0.5, settle=0.1)
            self.arm.goto_xyz(fx, fy, Z_GRASP + 0.04, seconds=0.6, settle=0.1)
            log("grasp", target=[float(cx), float(cy)], frame=[float(fx), float(fy)], grip_closed=g0, grip_lifted=None, held=False, edge=True, roll=round(roll, 1))
            return False, g0
        self.arm.goto_xyz(fx, fy, Z_CARRY, seconds=1.0, settle=0.2)
        g1 = self.arm.read()["gripper"]
        held = g1 > HELD_MIN
        log("grasp", target=[float(cx), float(cy)], frame=[float(fx), float(fy)], grip_closed=g0, grip_lifted=g1, held=held, roll=round(roll, 1))
        return held, g1

    def hold(self, measured, squeeze=HOLD_SQUEEZE):
        """Relax the closing goal to a moderate squeeze so the servo never trips overload."""
        goal = max(CLOSED, measured - squeeze)
        self.arm.bus.sync_write("Goal_Position", {"gripper": goal}, normalize=True)
        time.sleep(0.1)
        A.cw.recover_overload(self.arm.bus, "gripper")
        return goal

    def place(self, cx, cy, release_z=Z_GRASP + 0.005):
        """Put the held cube down so that its centre lands near robot (cx, cy)."""
        r = radial(cx, cy)
        fx, fy = np.array([cx, cy]) - HELD_FORWARD * r
        self.arm.goto_xyz(fx, fy, Z_CARRY, wrist_roll=BASE_ROLL, seconds=1.3, settle=0.1)
        self.arm.goto_xyz(fx, fy, release_z, seconds=0.9, settle=0.2)
        self.arm.gripper(OPEN, seconds=0.5, settle=0.2)
        self.arm.goto_xyz(fx, fy, Z_HOVER, seconds=0.8, settle=0.1)
        log("place", target=[float(cx), float(cy)], frame=[float(fx), float(fy)])
        return fx, fy

    def home(self):
        return self.arm.home(seconds=1.6, settle=0.3)

    def _snap(self, tag):
        import cv2
        for cam in ("top", "wrist"):
            img, _ = read_frame(cam)
            cv2.imwrite(f"/tmp/{cam}_{tag}.jpg", img)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["see", "grasp", "place", "home"])
    p.add_argument("--x", type=float); p.add_argument("--y", type=float)
    a = p.parse_args()
    with Grasper() as g:
        if a.cmd == "see":
            c, _ = g.see_cube(); print(c, g.cube_xy(c) if c else None)
        elif a.cmd == "grasp":
            if a.x is None:
                c, _ = g.see_cube(); a.x, a.y = g.cube_xy(c)
            print(g.grasp(a.x, a.y, snapshots="cli"))
        elif a.cmd == "place":
            print(g.place(a.x, a.y))
        elif a.cmd == "home":
            print(g.home())
