"""Wrist-camera visual servoing helpers for the cube grasp."""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

import cube_pick as cp

HERE = Path(__file__).resolve().parent
SERVO_FILE = HERE / "wrist_servo.json"


def detect_cube_wrist(img, dark_thresh=60, min_area=6000, max_area=400000, debug_path=None):
    """Largest dark compact blob in the wrist image that does not touch the bottom edge (jaws)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    dark = (gray < dark_thresh).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(dark, connectivity=8)
    h, w = gray.shape
    best = None
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if not (min_area <= area <= max_area):
            continue
        x, y, bw, bh = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
        if y + bh >= h - 3:      # touches bottom edge -> jaw
            continue
        if x <= 2 and bh > 250:  # left edge tall blob -> moving jaw
            continue
        fill = area / float(bw * bh)
        if fill < 0.4:
            continue
        cand = dict(px=(float(cents[i][0]), float(cents[i][1])), area=area, bbox=(x, y, bw, bh), fill=fill)
        if best is None or area > best["area"]:
            best = cand
    if debug_path:
        dbg = img.copy()
        if best:
            x, y, bw, bh = best["bbox"]
            cv2.rectangle(dbg, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(dbg, (int(best["px"][0]), int(best["px"][1])), 5, (0, 0, 255), -1)
        cv2.imwrite(str(debug_path), dbg)
    return best


class WristServo:
    def __init__(self):
        self.J = None           # 2x2: tool-frame displacement (m) -> pixel shift (du, dv)
        self.target = (600., 480.)
        self.z_servo = None
        if SERVO_FILE.exists():
            d = json.loads(SERVO_FILE.read_text())
            self.J = np.array(d["J"]) if d.get("J") else None
            self.target = tuple(d.get("target", self.target))
            self.z_servo = d.get("z_servo")

    def save(self):
        SERVO_FILE.write_text(json.dumps({"J": None if self.J is None else self.J.tolist(),
                                          "target": list(self.target), "z_servo": self.z_servo}, indent=1))

    @staticmethod
    def tool_axes_xy(joints):
        _, point, jaw = cp.tool_axes(joints)
        tx = np.array([jaw[0], jaw[1]])
        tx /= max(np.linalg.norm(tx), 1e-6)
        ty = np.array([-tx[1], tx[0]])
        return tx, ty

    def calibrate(self, arm, joints, z, roll, tilt, ik_reach, step=0.015, debug_dir=None):
        """Move +step along tool x then tool y, observe the cube shift in the wrist image."""
        pos0 = cp.fk_xyz(joints)
        tx, ty = self.tool_axes_xy(joints)
        time.sleep(0.5)
        c0 = detect_cube_wrist(cp.snap("wrist"), debug_path=debug_dir and f"{debug_dir}/jac0.jpg")
        if not c0:
            raise RuntimeError("cube not visible in wrist camera for Jacobian calibration")
        cols = []
        for axis in (tx, ty):
            x, y = pos0[0] + step * axis[0], pos0[1] + step * axis[1]
            j, _ = ik_reach(x, y, z, roll, tilt_start=tilt)
            arm.move_precise(j, speed_dps=30)
            time.sleep(0.5)
            c = detect_cube_wrist(cp.snap("wrist"), debug_path=debug_dir and f"{debug_dir}/jac{len(cols)+1}.jpg")
            if not c:
                raise RuntimeError("cube lost during Jacobian calibration")
            cols.append([(c["px"][0] - c0["px"][0]) / step, (c["px"][1] - c0["px"][1]) / step])
            arm.move_precise(joints, speed_dps=30)
        self.J = np.array(cols).T   # columns: tool x, tool y
        self.z_servo = z
        self.save()
        return self.J

    def step(self, joints, cube_px, gain=0.8, max_step=0.03):
        """Return (dx, dy) world displacement to move the cube toward the target spot."""
        err = np.array([self.target[0] - cube_px[0], self.target[1] - cube_px[1]])
        d_tool = np.linalg.solve(self.J, err) * gain
        norm = np.linalg.norm(d_tool)
        if norm > max_step:
            d_tool *= max_step / norm
        tx, ty = self.tool_axes_xy(joints)
        d_world = d_tool[0] * tx + d_tool[1] * ty
        return d_world, float(np.linalg.norm(err))
