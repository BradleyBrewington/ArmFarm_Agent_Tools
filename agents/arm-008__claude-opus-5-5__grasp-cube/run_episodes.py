#!/usr/bin/env python3
"""Grasp-cube episode loop.

Each cycle: start recording -> pick the black cube -> bring it to the home pose ->
confirm the grasp (gripper still closed on the cube after settling at home, and no cube
left on the table) -> stop recording (success flag) -> place the cube at a new position
chosen for workspace coverage -> back to the look pose.

python run_episodes.py [N] [--minutes M]
Logs: episodes_log.jsonl (one line per episode), coverage.json (successes per cell).
"""
import argparse
import json
import math
import random
import time
import traceback
from pathlib import Path

import numpy as np

from arm import Arm
from camd_client import read_frame
import recording
import task
import vision

HERE = vision.HERE
LOG = HERE / "episodes_log.jsonl"
COVERAGE = HERE / "coverage.json"
STOP_FILE = HERE / "STOP"
TASK = "Pick up the black cube and bring it to the home position"

# Placement workspace (cube centre, base_link metres): reachable at the fixed grasp pitch,
# inside the calibrated top-camera map and clear of the arm silhouette at the look pose.
X_RANGE = (0.18, 0.30)
Y_RANGE = (-0.14, 0.12)
CELL = 0.04
PICK_TRIES = 3


def cells():
    xs = np.arange(X_RANGE[0], X_RANGE[1] + 1e-9, CELL)
    ys = np.arange(Y_RANGE[0], Y_RANGE[1] + 1e-9, CELL)
    return [(round(float(x), 3), round(float(y), 3)) for x in xs for y in ys]


def cell_key(x, y):
    return f"{round((x - X_RANGE[0]) / CELL)},{round((y - Y_RANGE[0]) / CELL)}"


def load_cov():
    return json.loads(COVERAGE.read_text()) if COVERAGE.exists() else {}


def next_target(cov, last):
    """Least-covered cell (random tie-break, not the cell we just used), jittered inside it."""
    options = [c for c in cells() if cell_key(*c) != last]
    lo = min(cov.get(cell_key(*c), 0) for c in options)
    x, y = random.choice([c for c in options if cov.get(cell_key(*c), 0) == lo])
    x += random.uniform(-0.4, 0.4) * CELL
    y += random.uniform(-0.4, 0.4) * CELL
    x = min(max(x, X_RANGE[0]), X_RANGE[1])
    y = min(max(y, Y_RANGE[0]), Y_RANGE[1])
    return x, y, random.uniform(-45, 45)


def cube_on_table(arm):
    """A cube-like blob on the table away from the gripper (the held cube hangs at the tip)."""
    img, _ = read_frame("top")
    tip = arm.tip()
    for c in vision.candidates(img, use_arm_mask=False):
        x, y = vision.cube_robot(c["px"])
        if math.hypot(x - tip["x"], y - tip["y"]) > 0.07:
            return c
    return None


def confirm_grasp(arm):
    """At home: the jaws must still be held open by the cube, and the top camera must not see
    a cube on the table. Returns (ok, details)."""
    time.sleep(0.4)
    g1 = arm.gripper_pos()
    time.sleep(0.3)
    g2 = arm.gripper_pos()
    on_table = cube_on_table(arm)
    ok = g1 > task.GRIP_EMPTY and g2 > task.GRIP_EMPTY and abs(g1 - g2) < 3 and on_table is None
    return ok, {"grip": round(g2, 1), "cube_on_table": None if on_table is None else on_table["px"]}


def stop_episode(success, notes, tries=6):
    """stop_recording, retrying while the recorder's writer queue is momentarily full."""
    for i in range(tries):
        try:
            return recording.stop_recording(success=success, notes=notes)
        except RuntimeError as e:
            if "Full" not in str(e) or i == tries - 1:
                raise
            time.sleep(1.0 + i)


def quality_flags(stop):
    """Recorder quality flags of the finished episode (camera/queue problems), or None."""
    try:
        folder = stop.get("folder")
        return json.loads((Path(folder) / "episode.json").read_text()).get("quality_flags") if folder else None
    except (OSError, ValueError):
        return None


def log(entry):
    with open(LOG, "a") as f:
        f.write(json.dumps(entry, default=float) + "\n")


def episode(arm, n):
    t0 = time.monotonic()
    task.go_look(arm, 1.0)
    time.sleep(0.2)
    recording.start_recording(task=TASK)
    attempts = []
    ok, info = task.pick_robust(arm, attempts=PICK_TRIES, log=lambda m: attempts.append(m))
    if ok:
        arm.home()
        good, conf = confirm_grasp(arm)
    else:
        arm.home()
        good, conf = False, {"pick": info}
    notes = (f"grasp confirmed at home: gripper {conf.get('grip')}%, no cube on table"
             if good else f"failed: {json.dumps(conf, default=float)}")
    stop = stop_episode(bool(good), notes)
    entry = {"n": n, "t": time.time(), "success": bool(good), "pick": info, "confirm": conf,
             "attempt_log": attempts, "seconds": round(time.monotonic() - t0, 1),
             "folder": stop.get("folder"), "quality_flags": quality_flags(stop)}
    return good, info, entry


def reset(arm, cov, last_cell, holding):
    """Put the cube down at a new coverage target (or fetch it first if not held)."""
    if not holding:
        ok, _ = task.pick_robust(arm, attempts=4)
        if not ok:
            return None
    x, y, yaw = next_target(cov, last_cell)
    cx, cy = task.place(arm, x, y, yaw)
    task.go_look(arm, 1.0)
    return cx, cy


def main():
    p = argparse.ArgumentParser()
    p.add_argument("n", type=int, nargs="?", default=10)
    p.add_argument("--minutes", type=float, default=None)
    a = p.parse_args()
    cov = load_cov()
    start = time.monotonic()
    done = success = 0
    last_cell = None
    if recording.request("status").get("recording"):   # left open by a killed predecessor
        recording.stop_recording(success=False, notes="aborted: controlling process ended mid-episode; "
                                 "grasp not confirmed")
    with Arm() as arm:
        if arm.gripper_pos() > task.GRIP_EMPTY and arm.gripper_pos() < 50:   # still holding from before
            reset(arm, cov, None, holding=True)
        while done < a.n and (a.minutes is None or time.monotonic() - start < a.minutes * 60):
            if STOP_FILE.exists():       # touch tools/STOP to end cleanly between episodes
                STOP_FILE.unlink()
                print("stop file found; ending", flush=True)
                break
            try:
                _, p0 = task.detect_robot(True)
                good, info, entry = episode(arm, done)
                start_xy = (info.get("x"), info.get("y")) if info.get("x") is not None else (p0[0], p0[1]) if p0 else None
                entry["start_xy"] = start_xy
                done += 1
                if good:
                    success += 1
                    if start_xy:
                        k = cell_key(*start_xy)
                        cov[k] = cov.get(k, 0) + 1
                        COVERAGE.write_text(json.dumps(cov, indent=1, sort_keys=True))
                        last_cell = k
                log(entry)
                rate = success / max((time.monotonic() - start) / 3600, 1e-6)
                flags = entry.get("quality_flags")
                print(f"[{done}] {'OK ' if good else 'FAIL'} {entry['seconds']}s tries={info.get('attempts')} "
                      f"{'flags=' + ','.join(flags) + ' ' if flags else ''}"
                      f"start={start_xy and tuple(round(v, 3) for v in start_xy)} "
                      f"success {success}/{done} ~{rate:.0f}/h", flush=True)
                if flags:
                    time.sleep(3.0)          # let the recorder's writer catch up before the next episode
                placed = reset(arm, cov, last_cell, holding=good)
                for _ in range(2):           # a missed reset pick usually only nudged the cube
                    if placed is not None:
                        break
                    placed = reset(arm, cov, last_cell, holding=False)
                if placed is None:
                    print("reset failed: cube not recovered", flush=True)
                    break
            except Exception as e:
                traceback.print_exc()
                log({"n": done, "t": time.time(), "error": repr(e)})
                try:
                    if recording.request("status").get("recording"):
                        recording.stop_recording(success=False, notes=f"aborted: {e!r}")
                except Exception:
                    pass
                break
    print(f"done: {success}/{done} successful in {(time.monotonic() - start) / 60:.1f} min")


if __name__ == "__main__":
    main()
