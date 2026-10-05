#!/usr/bin/env python3
"""Fit table->robot (vision.RIGID_FILE) by touching the table with the closed fingertip.

With the approach tilted outward the fingertip is the arm silhouette's point farthest from
the robot base in the top image. Each touch gives (table XY at h=0, FK tip XY).
"""
import json
import math
import sys
import time

import cv2
import numpy as np

from arm import Arm
from camd_client import read_frame
import task
import vision

POINTS = [(x, y) for x in (0.17, 0.22, 0.27, 0.31) for y in (-0.15, -0.075, 0.0, 0.075, 0.15)]
BASE_PX = (700.0, -60.0)     # rough image position of the shoulder pan axis
OUT = vision.HERE / "tip_samples.jsonl"


def fingertip_px(img):
    m = vision.dark_mask(img)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    # the arm blob touches the top border; take the largest such component
    arm = [i for i in range(1, n) if st[i, 1] <= 2]
    if not arm:
        return None
    i = max(arm, key=lambda k: st[k, 4])
    ys, xs = np.nonzero(lab == i)
    d = np.hypot(xs - BASE_PX[0], ys - BASE_PX[1])
    far = d >= d.max() - 6            # average the outermost few pixels
    return float(xs[far].mean()), float(ys[far].mean())


def main():
    samples = []
    with Arm() as a:
        for x, y in POINTS:
            q = task.plan(x, y, 0.04, 0.0, pitch=35)
            a.move({**q, "gripper": 0.0}, 1.0); a.wait(q, tol=2, timeout=1)
            q = task.plan(x, y, 0.004, 0.0, pitch=35)
            a.move(q, 0.6); a.wait(q, tol=1.5, timeout=1); time.sleep(0.35)
            t = a.tip()
            px = fingertip_px(read_frame("top")[0])
            q = task.plan(x, y, 0.04, 0.0, pitch=35); a.move(q, 0.4)
            if px is None:
                continue
            tx, ty = vision.px_to_table(px, h=0.0)
            s = {"u": px[0], "v": px[1], "tx": tx, "ty": ty, "x": t["x"], "y": t["y"], "z": t["z"]}
            samples.append(s)
            print(json.dumps(s), flush=True)
        task.go_look(a)
    OUT.write_text("".join(json.dumps(s) + "\n" for s in samples))
    R, b, err = vision.fit_rigid([(s["tx"], s["ty"]) for s in samples], [(s["x"], s["y"]) for s in samples])
    print("rms mm", 1000 * np.sqrt((err ** 2).mean()), "max mm", 1000 * err.max())
    print("per point mm", np.round(1000 * err, 1).tolist())


if __name__ == "__main__":
    sys.exit(main())
