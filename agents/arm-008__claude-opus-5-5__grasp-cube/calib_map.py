#!/usr/bin/env python3
"""Fit the top-camera cube map by placing the held cube at known FK positions.

Start with the cube held in the gripper. For each grid point: place (roll 0),
go home, detect the cube, record (u, v, x, y), refit, then re-pick it.
Samples append to cube_map_samples.jsonl; the fit is written to cube_map.json.
"""
import json
import math
import sys
import time

import numpy as np

from arm import Arm
from camd_client import read_frame
import task
import vision

GRID = [(x, y) for x in (0.20, 0.25, 0.30, 0.34) for y in (-0.16, -0.08, 0.0, 0.08, 0.16)]


def load_samples():
    if not vision.SAMPLES_FILE.exists():
        return []
    return [json.loads(l) for l in vision.SAMPLES_FILE.read_text().splitlines() if l.strip()]


def detect(retries=5):
    for _ in range(retries):
        d = vision.detect_cube(read_frame("top")[0])
        if d:
            return d
        time.sleep(0.2)
    return None


def main():
    grid = GRID if len(sys.argv) < 2 else json.loads(sys.argv[1])
    samples = load_samples()
    with Arm() as a:
        for x, y in grid:
            if a.gripper_pos() < task.GRIP_EMPTY:
                print("not holding the cube; stopping"); return 1
            try:
                tip = task.place(a, x, y, 0.0)
            except ValueError as e:
                print("skip", x, y, e); continue
            # cube centre = measured fixed-jaw tip minus the jaw offset (roll 0 -> +y)
            cx, cy = tip["x"], tip["y"] - task.JAW_OFFSET
            a.home()
            time.sleep(0.3)
            d = detect()
            if d is None:
                print("cube not seen after placing at", x, y); return 1
            s = {"u": d["px"][0], "v": d["px"][1], "x": cx, "y": cy, "angle": d["angle"]}
            samples.append(s)
            with open(vision.SAMPLES_FILE, "a") as f:
                f.write(json.dumps(s) + "\n")
            pred = vision.px_to_robot(d["px"]) if vision.load_map() is not None else (math.nan, math.nan)
            fit = ""
            if len(samples) >= 4:   # until then keep the rough bootstrap map
                H, err = vision.fit_map([(s["u"], s["v"], s["x"], s["y"]) for s in samples])
                fit = f"; fit rms {1000*np.sqrt((err**2).mean()):.1f} max {1000*err.max():.1f} mm"
            print(f"placed ({cx:.3f},{cy:.3f}) px=({s['u']:.0f},{s['v']:.0f}) old-map err "
                  f"{1000*math.hypot(pred[0]-cx, pred[1]-cy):.1f} mm{fit}", flush=True)
            for attempt in range(3):
                d = detect()
                px, py, yaw = task.cube_pose(d)
                g = task.pick(a, px, py, task.roll_for(yaw))
                if g > task.GRIP_EMPTY:
                    break
                a.home()
                time.sleep(0.3)
            else:
                print("re-pick failed"); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
