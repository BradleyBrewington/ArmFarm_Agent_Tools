#!/usr/bin/env python3
"""Fit the wrist camera pose in the gripper frame from views of a stationary cube.

collect: perturb the arm around a base pose that sees the cube; record joints and the
cube's wrist-image centroid (wrist_views.jsonl).
fit: solve camera extrinsics (6), focal length (1) and the cube XY (2, per session)
by minimising reprojection error of the cube centre (z = CUBE_Z) -> wrist_cam.json.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from kin import fk_T

HERE = Path(__file__).resolve().parent
VIEWS = HERE / "wrist_views.jsonl"
CAM_FILE = HERE / "wrist_cam.json"
CUBE_Z = 0.015
W, H = 1280, 720


def rodrigues(r):
    th = np.linalg.norm(r)
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def project(cam, joints, P):
    """Project base_link point P into the wrist image. cam: dict(r, t, f, cx, cy)."""
    T = fk_T(joints)
    R_gc = rodrigues(np.array(cam["r"]))
    T_gc = np.eye(4); T_gc[:3, :3] = R_gc; T_gc[:3, 3] = cam["t"]
    T_bc = T @ T_gc
    pc = np.linalg.inv(T_bc) @ np.array([*P, 1.0])
    return np.array([cam["f"] * pc[0] / pc[2] + cam["cx"], cam["f"] * pc[1] / pc[2] + cam["cy"]]), pc[2]


def backproject(cam, joints, px, z=CUBE_Z):
    """Intersect the pixel ray with the plane base_z = z. Returns base_link XY."""
    T = fk_T(joints)
    T_gc = np.eye(4); T_gc[:3, :3] = rodrigues(np.array(cam["r"])); T_gc[:3, 3] = cam["t"]
    T_bc = T @ T_gc
    d = np.array([(px[0] - cam["cx"]) / cam["f"], (px[1] - cam["cy"]) / cam["f"], 1.0])
    d = T_bc[:3, :3] @ d
    o = T_bc[:3, 3]
    s = (z - o[2]) / d[2]
    p = o + s * d
    return float(p[0]), float(p[1])


def load_views():
    return [json.loads(l) for l in VIEWS.read_text().splitlines() if l.strip()]


def fit(views, init=None):
    sessions = sorted({v["session"] for v in views})
    known = {v["session"]: v["cube"] for v in views if v.get("cube")}
    free = [s for s in sessions if s not in known]
    x0 = list(init or [0, 0, 0, 0, 0.0, 0.0, 0.0, 900.0])
    for s in free:
        x0 += [0.2, 0.0]

    def unpack(x):
        cam = {"r": x[0:3], "t": x[3:6], "f": x[7], "cx": W / 2, "cy": H / 2}
        cubes = dict(known)
        for i, s in enumerate(free):
            cubes[s] = x[8 + 2 * i: 10 + 2 * i]
        return cam, cubes

    def res(x):
        cam, cubes = unpack(x)
        out = []
        for v in views:
            p, depth = project(cam, v["joints"], [*cubes[v["session"]], CUBE_Z])
            out += list((p - np.array(v["px"])) / 10.0) if depth > 0 else [100.0, 100.0]
        return out

    best = None
    # try a few camera orientations as starts (camera looks roughly along the fingers)
    for r0 in ([0, 0, 0], [0, 0, math.pi / 2], [0, 0, -math.pi / 2], [0, 0, math.pi],
               [math.pi, 0, 0], [0.3, 0, 0], [-0.3, 0, 0]):
        x0[0:3] = r0
        r = least_squares(res, np.array(x0, float), loss="soft_l1", f_scale=2.0)
        if best is None or r.cost < best.cost:
            best = r
    cam, cubes = unpack(best.x)
    resid = np.array(res(best.x)).reshape(-1, 2) * 10
    out = {"r": list(map(float, cam["r"])), "t": list(map(float, cam["t"])), "f": float(cam["f"]),
           "cx": W / 2, "cy": H / 2, "rms_px": float(np.sqrt((resid ** 2).sum(1).mean())),
           "n": len(views), "cubes": {s: list(map(float, c)) for s, c in cubes.items()}}
    return out, resid


def collect(session, base, n=16, seed=0):
    from arm import Arm
    from camd_client import read_frame
    import wrist
    rng = np.random.default_rng(seed)
    with Arm() as a:
        for i in range(n):
            d = {"shoulder_pan": 5, "shoulder_lift": 5, "elbow_flex": 5, "wrist_flex": 8, "wrist_roll": 30}
            q = {j: base[j] + (rng.uniform(-1, 1) * d[j] if i else 0.0) for j in d}
            a.move({**q, "gripper": 80.0}, 0.7)
            a.wait(q, tol=1.0, timeout=1.0)
            time.sleep(0.25)
            j = a.joints()
            det = wrist.detect(read_frame("wrist")[0])
            if det:
                with open(VIEWS, "a") as f:
                    f.write(json.dumps({"session": session, "joints": j, "px": det["px"], "angle": det["angle"]}) + "\n")
            print(i, det and [round(v) for v in det["px"]], flush=True)


if __name__ == "__main__":
    if sys.argv[1] == "collect":
        collect(sys.argv[2], json.loads(sys.argv[3]), int(sys.argv[4]) if len(sys.argv) > 4 else 16,
                seed=int(time.time()))
    else:
        cam, resid = fit(load_views())
        CAM_FILE.write_text(json.dumps(cam, indent=1))
        print(json.dumps(cam, indent=1))
        print("per-view px err", np.round(np.linalg.norm(resid, axis=1), 1).tolist())
