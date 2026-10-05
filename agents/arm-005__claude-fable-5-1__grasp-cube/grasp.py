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
GRASP_BACK = 0.025      # static jaw this far inside (toward base) of the cube centre
HELD_FORWARD = 0.020    # held cube centre sits this far radially out from the frame
Z_TABLE = -0.008
Z_GRASP = Z_TABLE + 0.018
Z_HOVER = 0.075
Z_CARRY = 0.09
OPEN = 70.0
CLOSED = 0.0
HELD_MIN = 6.0          # gripper % above which something is between the jaws (empty closes to ~0.3)
LOG = HERE / "grasp_log.jsonl"


def radial(x, y):
    v = np.array([x, y]) - PAN_AXIS
    return v / (np.linalg.norm(v) + 1e-9)


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

    def cube_xy(self, c):
        return self.map.pixel_to_robot(c["u"], c["v"])

    # ---- motion primitives
    def hover(self, x, y, z=Z_HOVER, seconds=1.2):
        return self.arm.goto_xyz(x, y, z, seconds=seconds)

    def grasp(self, cx, cy, snapshots=None):
        """Grasp a cube centred at robot (cx, cy). Returns (held: bool, gripper %)."""
        r = radial(cx, cy)
        fx, fy = np.array([cx, cy]) - GRASP_BACK * r
        self.arm.gripper(OPEN, seconds=0.5, settle=0.1)
        self.arm.goto_xyz(fx, fy, Z_HOVER, seconds=1.2, settle=0.1)
        self.arm.goto_xyz(fx, fy, Z_GRASP + 0.02, seconds=0.6, settle=0.1)
        self.arm.goto_xyz(fx, fy, Z_GRASP, seconds=0.5, settle=0.2)
        if snapshots:
            self._snap(snapshots + "_pre")
        self.arm.move({"gripper": CLOSED}, 0.8, settle=0.3)
        g0 = self.arm.read()["gripper"]
        self.arm.goto_xyz(fx, fy, Z_CARRY, seconds=1.0, settle=0.2)
        g1 = self.arm.read()["gripper"]
        held = g1 > HELD_MIN
        log("grasp", target=[float(cx), float(cy)], frame=[float(fx), float(fy)], grip_closed=g0, grip_lifted=g1, held=held)
        return held, g1

    def place(self, cx, cy, release_z=Z_GRASP + 0.012):
        """Put the held cube down so that its centre lands near robot (cx, cy)."""
        r = radial(cx, cy)
        fx, fy = np.array([cx, cy]) - HELD_FORWARD * r
        self.arm.goto_xyz(fx, fy, Z_CARRY, seconds=1.3, settle=0.1)
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
