"""Black-cube detection in the top camera and pixel -> robot table mapping.

detect_cube(img) -> dict(px=(u, v), area, angle, box) or None. (u, v) is the
centroid of the dark cube blob in raw 1280x720 pixels.
Mapping: cube_map.json holds a homography from cube centroid pixels to the
base_link XY where the cube's centre sits on the table. It is fitted from
placements (place cube with the gripper at known FK XY, then detect it).
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
MAP_FILE = HERE / "cube_map.json"
ARM_MASK_FILE = HERE / "arm_home_mask.png"   # dark pixels of the arm at the detection pose
SAMPLES_FILE = HERE / "cube_map_samples.jsonl"

DARK = 70
MIN_AREA, MAX_AREA = 1800, 14000


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


def load_map():
    if not MAP_FILE.exists():
        return None
    return np.array(json.loads(MAP_FILE.read_text())["H"], float)


def px_to_robot(px, H=None):
    H = load_map() if H is None else H
    p = H @ np.array([px[0], px[1], 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def robot_to_px(xy, H=None):
    H = load_map() if H is None else H
    p = np.linalg.inv(H) @ np.array([xy[0], xy[1], 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def fit_map(samples, save=True):
    """samples: list of (u, v, x, y). Homography if >=6 samples, else affine."""
    s = np.array(samples, float)
    src, dst = np.ascontiguousarray(s[:, :2]), np.ascontiguousarray(s[:, 2:])
    if len(s) >= 6:
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 0.008)
    else:
        X = np.hstack([src, np.ones((len(s), 1))])
        A = np.linalg.lstsq(X, dst, rcond=None)[0].T
        H = np.vstack([A, [0, 0, 1]])
    pred = np.array([px_to_robot(p, H) for p in src])
    err = np.linalg.norm(pred - dst, axis=1)
    if save:
        MAP_FILE.write_text(json.dumps({"H": H.tolist(), "n": len(s), "rms_m": float(np.sqrt((err ** 2).mean())),
                                        "max_m": float(err.max())}, indent=1))
    return H, err


def cube_yaw(det):
    """Cube edge angle in image, folded to [-45, 45) degrees."""
    a = det["angle"] % 90.0
    return a - 90.0 if a >= 45 else a


# ---- metric table plane from the saved top-camera calibration -------------------------
CAL = json.loads((HERE.parent / "calibration/ready.json").read_text())
_WS = next(HERE.parent / p for p in CAL["files"] if p.endswith("workspace_map.json"))
_WM = json.loads(_WS.read_text())
K = np.array(_WM["camera_matrix"]); DIST = np.array(_WM["dist_coeffs"])
_Hi = np.linalg.inv(np.array(_WM["homography_undistorted_pixel_to_table_m"]))
_B = np.linalg.inv(K) @ _Hi
_s = 1 / np.linalg.norm(_B[:, 0])
_r1, _r2, _t = _B[:, 0] * _s, _B[:, 1] * _s, _B[:, 2] * _s
if _t[2] < 0:
    _r1, _r2, _t = -_r1, -_r2, -_t
R_CT = np.column_stack([_r1, _r2, np.cross(_r1, _r2)])   # table -> camera rotation
T_CT = _t
CAM_C = -R_CT.T @ T_CT                                     # camera centre, table frame (z < 0 is up)
CUBE_H = 0.015
RIGID_FILE = HERE / "table_to_robot.json"


def px_to_table(px, h=CUBE_H):
    """Raw top-camera pixel -> table-frame XY of the point at height h above the table."""
    n = cv2.undistortPoints(np.array([[px]], np.float64), K, DIST)[0, 0]
    d = R_CT.T @ np.array([n[0], n[1], 1.0])
    s = (-h - CAM_C[2]) / d[2]
    p = CAM_C + s * d
    return float(p[0]), float(p[1])


def fit_rigid(table_xy, robot_xy, save=True):
    """2D rotation(+reflection) + translation table -> robot (Procrustes); picks the better handedness."""
    P = np.asarray(table_xy, float); Q = np.asarray(robot_xy, float)
    pc, qc = P.mean(0), Q.mean(0)
    U, S, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    best = None
    for flip in (1, -1):
        D = np.diag([1, flip])
        R = (U @ D @ Vt).T
        b = qc - R @ pc
        err = np.linalg.norm((P @ R.T + b) - Q, axis=1)
        if best is None or err.mean() < best[2].mean():
            best = (R, b, err)
    R, b, err = best
    if save:
        RIGID_FILE.write_text(json.dumps({"R": R.tolist(), "b": b.tolist(), "n": len(P),
                                          "rms_m": float(np.sqrt((err ** 2).mean())), "max_m": float(err.max())}, indent=1))
    return R, b, err


def table_to_robot(xy):
    d = json.loads(RIGID_FILE.read_text())
    return tuple(np.array(d["R"]) @ np.asarray(xy) + np.array(d["b"]))


def cube_robot(px):
    return table_to_robot(px_to_table(px))
