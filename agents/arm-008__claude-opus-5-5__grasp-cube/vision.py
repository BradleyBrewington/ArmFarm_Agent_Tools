"""Black-cube detection in the top camera and pixel -> robot table mapping.

detect_cube(img) -> dict(px=(u, v), area, angle, box) or None. (u, v) is the
centroid of the dark cube blob in raw 1280x720 pixels.
Mapping: top_plane_maps.json holds homographies from undistorted pixels to base_link XY
for horizontal planes at given heights, fitted with the fingertip as a marker (calib_tip.py).
"""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ARM_MASK_FILE = HERE / "arm_home_mask.png"   # dark pixels of the arm at the detection pose

DARK = 70
MIN_AREA, MAX_AREA = 1800, 14000


def grab(cam, tries=5):
    """camd_client.read_frame, retrying transient errors (a torn shared-memory read shows up as
    'JPEG decode failed'; a momentarily stale slot as CamdStale)."""
    from camd_client import CamdError, read_frame
    for i in range(tries):
        try:
            return read_frame(cam)
        except CamdError:
            if i == tries - 1:
                raise
            time.sleep(0.03)


def dark_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 2] < DARK)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return m


_ARM = None


def arm_mask():
    global _ARM
    if _ARM is None and ARM_MASK_FILE.exists():
        _ARM = cv2.imread(str(ARM_MASK_FILE), cv2.IMREAD_GRAYSCALE)
    return _ARM


def save_arm_mask(img, exclude=None):
    """Store the arm's dark silhouette at the detection pose (cube blob `exclude` removed)."""
    global _ARM
    m = dark_mask(img)
    if exclude is not None:
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
        u, v = map(int, exclude["px"])
        m[lab == lab[v, u]] = 0
    m = cv2.dilate(m, np.ones((15, 15), np.uint8))
    cv2.imwrite(str(ARM_MASK_FILE), m)
    _ARM = m


def candidates(img, use_arm_mask=True):
    m = dark_mask(img)
    am = arm_mask() if use_arm_mask else None
    if am is not None:
        m[am > 0] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    h, w = m.shape
    out = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if not MIN_AREA <= area <= MAX_AREA:
            continue
        if y <= 2 or x <= 2 or x + bw >= w - 2:   # touches border: arm or table edge
            continue
        aspect = bw / bh
        fill = area / float(bw * bh)
        if not 0.5 <= aspect <= 2.0:
            continue
        cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        rect = cv2.minAreaRect(cnt)
        rw, rh = rect[1]
        rfill = area / max(rw * rh, 1)
        if rfill < 0.75 or not 0.55 <= rw / max(rh, 1) <= 1.8:
            continue
        out.append({"px": (float(cent[i][0]), float(cent[i][1])), "area": int(area),
                    "angle": float(rect[2]), "rect": rect, "fill": rfill})
    return out


def detect_cube(img):
    c = candidates(img)
    if not c:
        return None
    return max(c, key=lambda d: d["fill"] * min(d["area"], 7000))


def cube_yaw(det):
    """Cube edge angle in image, folded to [-45, 45) degrees."""
    a = det["angle"] % 90.0
    return a - 90.0 if a >= 45 else a


_WS = HERE.parent / next(p for p in json.loads((HERE.parent / "calibration/ready.json").read_text())["files"]
                         if p.endswith("workspace_map.json"))
_WM = json.loads(_WS.read_text())
K = np.array(_WM["camera_matrix"]); DIST = np.array(_WM["dist_coeffs"])   # intrinsics are valid


# ---- pixel -> base_link maps fitted from fingertip touches (calib_tip.py) -------------
# The saved workspace_map homography is NOT the table plane (the board was held in the air
# during calibration), so robot coordinates come from these direct fits instead.
PLANE_FILE = HERE / "top_plane_maps.json"
CUBE_PLANE_Z = 0.02


def _undist(px):
    return cv2.undistortPoints(np.array([[px]], np.float64), K, DIST, P=K)[0, 0]


def fit_plane_map(samples, z):
    """samples (u, v, x, y) at tip height z -> homography (undistorted px -> XY), saved by z."""
    s = np.array(samples, float)
    src = cv2.undistortPoints(s[:, :2].reshape(-1, 1, 2), K, DIST, P=K).reshape(-1, 2)
    H, _ = cv2.findHomography(src, s[:, 2:], 0)
    pred = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(pred - s[:, 2:], axis=1)
    maps = json.loads(PLANE_FILE.read_text()) if PLANE_FILE.exists() else {}
    maps[f"{z:.3f}"] = {"H": H.tolist(), "n": len(s), "rms_m": float(np.sqrt((err ** 2).mean()))}
    PLANE_FILE.write_text(json.dumps(maps, indent=1))
    return H, err


def px_to_plane(px, z=CUBE_PLANE_Z):
    maps = json.loads(PLANE_FILE.read_text())
    key = min(maps, key=lambda k: abs(float(k) - z))
    H = np.array(maps[key]["H"])
    p = H @ np.array([*_undist(px), 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def cube_robot(px):
    return px_to_plane(px, CUBE_PLANE_Z)
