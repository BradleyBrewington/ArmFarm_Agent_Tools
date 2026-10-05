#!/usr/bin/env python3
"""Autonomous grasp-cube episode loop (robot venv).

Per episode:
  start_recording -> detect cube from home -> grasp -> lift -> home (cube in hand)
  -> confirm grasp (gripper not closed) -> stop_recording(success)
  -> place cube at the next workspace-coverage position -> home -> repeat.

The pixel->robot table map is refined after every placement from where the cube
was actually seen versus where it was released (tools/table_map.json).

    /opt/armfarm/venv/bin/python tools/run_episodes.py --episodes 60 [--dry]
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import arm as A  # noqa: E402
import grasp as G  # noqa: E402
import recording  # noqa: E402
import vision as V  # noqa: E402

TASK = "Start episode, pickup the black cube, bring black cube to home position, confirm successful grasp, stop episode"
STATE = HERE / "episode_state.json"
STATS = HERE / "episode_stats.jsonl"
STOP_FILE = HERE / "STOP"      # touch to make the loop exit cleanly between episodes

# Top-image region where a placed cube is fully visible and detectable.
IMG_U = (200, 1100)
IMG_V = (270, 680)
MAP_OUTLIER_M = 0.03     # placement seen > 3 cm from prediction: cube tumbled, do not learn from it
MAP_MAX_POINTS = 80
# Polar coverage grid around the pan axis (metres, degrees).
RADII = (0.17, 0.20, 0.23, 0.26, 0.285)
ANGLES = tuple(range(-55, 56, 11))


def coverage_targets(tmap, bounds):
    """All grid cells that are IK-feasible and land inside the visible image region."""
    out = []
    for r in RADII:
        for a in ANGLES:
            x = G.PAN_AXIS[0] + r * math.cos(math.radians(a))
            y = r * math.sin(math.radians(a))
            try:
                fr = np.array([x, y]) - G.HELD_FORWARD * G.radial(x, y)
                A.ik_reach(fr[0], fr[1], G.Z_GRASP, wrist_roll=83.0, bounds=bounds)
                fg = np.array([x, y]) - G.GRASP_BACK * G.radial(x, y)
                A.ik_reach(fg[0], fg[1], G.Z_GRASP, wrist_roll=83.0, bounds=bounds)
            except ValueError:
                continue
            if tmap.H is not None and tmap.kind in ("homography", "similarity"):
                u, v = tmap.robot_to_pixel(x, y)
                if not (IMG_U[0] <= u <= IMG_U[1] and IMG_V[0] <= v <= IMG_V[1]):
                    continue
            out.append((round(x, 4), round(y, 4), r, a))
    return out


def start_episode():
    """Open a recording; a stale episode left open by a killed run is closed as failed first."""
    try:
        return recording.start_recording(TASK)
    except RuntimeError as exc:
        if "already open" not in str(exc):
            raise
        recording.stop_recording(False, json.dumps({"error": "stale episode closed on restart"}))
        return recording.start_recording(TASK)


def in_workspace(x, y):
    r = math.hypot(x - G.PAN_AXIS[0], y)
    return 0.12 <= r <= 0.32 and abs(math.degrees(math.atan2(y, x - G.PAN_AXIS[0]))) <= 62


def load_state():
    if STATE.exists():
        return json.loads(STATE.read_text())
    return {"visited": [], "episodes": 0, "successes": 0}


def save_state(s):
    STATE.write_text(json.dumps(s, indent=1))


def record_stats(**kw):
    with STATS.open("a") as f:
        f.write(json.dumps({"t": time.time(), **kw}) + "\n")


def next_target(targets, state, current_xy, rng):
    """Least-visited cell, preferring ones far from the cube's current spot."""
    counts = {}
    for v in state["visited"]:
        counts[(v[0], v[1])] = counts.get((v[0], v[1]), 0) + 1
    least = min(counts.get((t[0], t[1]), 0) for t in targets)
    pool = [t for t in targets if counts.get((t[0], t[1]), 0) == least]
    far = [t for t in pool if math.hypot(t[0] - current_xy[0], t[1] - current_xy[1]) > 0.06]
    return rng.choice(far or pool)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=1000)
    p.add_argument("--dry", action="store_true", help="no recording calls")
    p.add_argument("--seed", type=int, default=None)
    a = p.parse_args()
    rng = random.Random(a.seed)
    state = load_state()

    with G.Grasper() as g:
        targets = coverage_targets(g.map, g.arm.bounds)
        print(f"{len(targets)} coverage targets; map {g.map.kind} n={len(g.map.points)}", flush=True)
        if g.arm.read()["gripper"] > G.HELD_MIN:
            # a previous run was stopped while holding the cube: set it down safely first
            g.home()
            g.place(0.25, 0.0)
        g.arm.gripper(G.OPEN, seconds=0.6, settle=0.1)
        g.home()
        consecutive_failures = 0
        for ep in range(a.episodes):
            if STOP_FILE.exists():
                STOP_FILE.unlink()
                print("stop file seen; exiting at home", flush=True)
                break
            t0 = time.time()
            # ---- locate the cube from home (arm parked out of the detection region)
            cube, img = g.see_cube(retries=5)
            if cube is None:
                print("cube not visible; waiting", flush=True)
                record_stats(event="no_cube")
                time.sleep(3)
                consecutive_failures += 1
                if consecutive_failures > 20:
                    print("giving up: cube not visible", flush=True)
                    break
                continue
            cx, cy = g.cube_xy(cube)
            if not in_workspace(cx, cy):
                print(f"detection maps outside the workspace ({cx:.3f},{cy:.3f}); ignoring", flush=True)
                record_stats(event="bad_detection", px=[cube["u"], cube["v"]], xy=[cx, cy])
                time.sleep(2)
                consecutive_failures += 1
                if consecutive_failures > 20:
                    break
                continue
            # ---- episode
            notes = {}
            if not a.dry:
                start_episode()
            held = False
            grip = 0.0
            try:
                for attempt in range(3):
                    held, grip = g.grasp(cx, cy)
                    if held:
                        break
                    # failed: re-detect (cube may have been nudged) and try again
                    g.arm.gripper(G.OPEN, seconds=0.5, settle=0.1)
                    g.home()
                    cube, img = g.see_cube(retries=5)
                    if cube is None:
                        break
                    cx, cy = g.cube_xy(cube)
                    if not in_workspace(cx, cy):
                        break
                g.home()
                now = g.arm.read()
                grip_home = now["gripper"]
                # confirm: jaws still apart at home, and the cube is gone from where it was
                frac = V.cube_mask_dark_fraction(g.see_cube(retries=1)[1], cube["u"], cube["v"]) if cube else 1.0
                success = bool(held and grip_home > G.HELD_MIN and frac < 0.35)
                notes = {"grip_home": round(grip_home, 1), "spot_dark_fraction": round(frac, 2),
                         "cube_px": [round(cube["u"]), round(cube["v"])] if cube else None,
                         "cube_xy": [round(cx, 4), round(cy, 4)], "attempts": attempt + 1}
            except Exception as exc:  # keep the loop alive, mark episode failed
                success = False
                notes["error"] = repr(exc)
                print("episode error:", repr(exc), flush=True)
            if not a.dry:
                result = recording.stop_recording(success, json.dumps(notes))
                notes["episode"] = result.get("episode") or result.get("folder")
            state["episodes"] += 1
            state["successes"] += int(success)
            consecutive_failures = 0 if success else consecutive_failures + 1
            dt_ep = time.time() - t0
            # ---- reset: carry the cube to a new coverage position (or recover if dropped)
            if success:
                tx, ty, r, ang = next_target(targets, state, (cx, cy), rng)
                fx, fy = g.place(tx, ty)
                g.home()
                seen, _ = g.see_cube(retries=5)
                if seen is not None:
                    px, py = np.array([fx, fy]) + G.HELD_FORWARD * G.radial(tx, ty)
                    pred = g.map.pixel_to_robot(seen["u"], seen["v"])
                    err = math.hypot(pred[0] - px, pred[1] - py)
                    if len(g.map.points) < 6 or err < MAP_OUTLIER_M:
                        g.map.add(seen["u"], seen["v"], px, py)
                        g.map.points = g.map.points[-MAP_MAX_POINTS:]
                        g.map.save()
                    else:
                        record_stats(event="map_outlier", err=round(err, 4), px=[seen["u"], seen["v"]], xy=[px, py])
                    state["visited"].append([tx, ty, r, ang])
                    if len(g.map.points) in (4, 6, 10, 20) or len(g.map.points) % 25 == 0:
                        targets = coverage_targets(g.map, g.arm.bounds)
            else:
                g.arm.gripper(G.OPEN, seconds=0.5, settle=0.1)
                g.home()
            save_state(state)
            record_stats(event="episode", success=success, seconds=round(dt_ep, 1),
                         total=round(time.time() - t0, 1), notes=notes,
                         map_n=len(g.map.points), map_resid=round(float(np.mean(g.map.residuals() or [0])), 4))
            print(f"ep {state['episodes']} success={success} {dt_ep:.1f}s (+reset {time.time()-t0-dt_ep:.1f}s) "
                  f"rate={state['successes']}/{state['episodes']} notes={notes}", flush=True)
            if consecutive_failures >= 8:
                print("too many consecutive failures; stopping for inspection", flush=True)
                break


if __name__ == "__main__":
    main()
