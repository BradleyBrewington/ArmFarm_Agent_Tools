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

GRID = [(x, y) for x in (0.23, 0.27, 0.31) for y in (-0.16, -0.08, 0.0, 0.08, 0.16)]

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
            task.go_look(a)
            time.sleep(0.3)
            d = detect()
            if d is None:
                print("cube not seen after placing at", x, y); return 1
            tx, ty = vision.px_to_table(d["px"])
            s = {"u": d["px"][0], "v": d["px"][1], "tx": tx, "ty": ty, "x": cx, "y": cy, "angle": d["angle"]}
            samples.append(s)
            with open(vision.SAMPLES_FILE, "a") as f:
                f.write(json.dumps(s) + "\n")
            pred = vision.table_to_robot((tx, ty))
            fit = ""
            if len(samples) >= 3:   # until then keep the bootstrap rigid fit
                R, b, err = vision.fit_rigid([(s["tx"], s["ty"]) for s in samples], [(s["x"], s["y"]) for s in samples])
                fit = f"; fit rms {1000*np.sqrt((err**2).mean()):.1f} max {1000*err.max():.1f} mm"
            print(f"placed ({cx:.3f},{cy:.3f}) px=({s['u']:.0f},{s['v']:.0f}) old-map err "
                  f"{1000*math.hypot(pred[0]-cx, pred[1]-cy):.1f} mm{fit}", flush=True)
            ok, info = task.pick_robust(a)
            if not ok:
                print("re-pick failed", info); return 1
            if info["attempts"] > 1:
                with open(vision.HERE / "pick_misses.jsonl", "a") as f:
                    f.write(json.dumps(info) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
