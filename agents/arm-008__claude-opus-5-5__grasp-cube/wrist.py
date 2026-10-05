"""Wrist-camera cube localisation for the final grasp alignment.

At a top-down hover (tip z = HOVER_Z_SERVO, gripper OPEN) the wrist camera's view of the
table is fixed in the gripper frame. wrist_map.json holds an affine map from the cube's
wrist-image centroid to the cube centre offset from the FK tip, in the gripper frame
(equal to base_link axes at wrist_roll 0), plus the cube image angle when aligned.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
MAP_FILE = HERE / "wrist_map.json"
SAMPLES_FILE = HERE / "wrist_samples.jsonl"
HOVER_Z_SERVO = 0.12
DARK = 60


def detect(img):
    """Cube blob in the wrist image (jaws at the bottom are excluded). Returns dict or None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 2] < DARK).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    h, w = m.shape
    best = None
    for i in range(1, n):
        x, y, bw, bh, area = st[i]
        if area < 4000 or area > 120000:
            continue
        if y + bh >= h - 3:            # touches the bottom: a jaw (or cube merged with one)
            continue
        if x <= 2 or x + bw >= w - 3 or y <= 2:
            continue
        cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        rect = cv2.minAreaRect(cnt)
        fill = area / max(rect[1][0] * rect[1][1], 1)
        if fill < 0.7:
            continue
        d = {"px": (float(cen[i][0]), float(cen[i][1])), "area": int(area), "angle": float(rect[2]), "fill": fill}
        if best is None or area > best["area"]:
            best = d
    return best


def fold90(a):
    a = a % 90.0
    return a - 90.0 if a >= 45 else a


def load():
    return json.loads(MAP_FILE.read_text())


def offset(px, wm=None):
    """Cube centre minus FK tip, gripper frame metres."""
    wm = wm or load()
    A = np.array(wm["A"])
    return A @ np.array([px[0], px[1], 1.0])


def fit(samples):
    """samples: list of dicts with u, v, ox, oy (and angle when aligned)."""
    s = np.array([[d["u"], d["v"], d["ox"], d["oy"]] for d in samples], float)
    X = np.hstack([s[:, :2], np.ones((len(s), 1))])
    A = np.linalg.lstsq(X, s[:, 2:], rcond=None)[0].T
    err = np.linalg.norm((X @ A.T) - s[:, 2:], axis=1)
    angles = [d["angle"] for d in samples if d.get("aligned")]
    ref = float(np.median([fold90(a) for a in angles])) if angles else 0.0
    wm = {"A": A.tolist(), "n": len(s), "rms_m": float(np.sqrt((err ** 2).mean())), "angle_ref": ref}
    MAP_FILE.write_text(json.dumps(wm, indent=1))
    return wm, err
