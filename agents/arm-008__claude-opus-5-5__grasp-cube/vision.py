"""Top-camera geometry for grasp-cube: cube detection, pixel <-> table <-> robot base frame."""
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CAL = HERE.parent / "calibration" / "20261002T165755_996297676"
ROBOT_MAP = HERE / "robot_map.json"   # fitted table->robot transform (written by calibrate_robot_map)

_d = json.loads((CAL / "workspace_map.json").read_text())
K = np.array(_d["camera_matrix"]); D = np.array(_d["dist_coeffs"])
H_UND = np.array(_d["homography_undistorted_pixel_to_table_m"])
_G = np.linalg.inv(K) @ np.linalg.inv(H_UND)
_lam = 1 / np.linalg.norm(_G[:, 0])
_r1, _r2, _t = _G[:, 0] * _lam, _G[:, 1] * _lam, _G[:, 2] * _lam
if _t[2] < 0:
    _r1, _r2, _t = -_r1, -_r2, -_t
_R = np.column_stack([_r1, _r2, np.cross(_r1, _r2)]); _U, _, _Vt = np.linalg.svd(_R); R_CAM = _U @ _Vt
T_CAM = _t
CAM_CENTER = -R_CAM.T @ T_CAM          # in table frame; table "up" is -z here
CAM_HEIGHT = float(-CAM_CENTER[2])


def undistort_px(px):
    p = np.asarray(px, np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(p, K, D, P=K).reshape(-1, 2)


def px_to_table(px, height=0.0):
    """Raw pixel(s) -> table-frame XY (m) of a point `height` m above the table."""
    u = undistort_px(px)
    xy = cv2.perspectiveTransform(u.reshape(-1, 1, 2), H_UND).reshape(-1, 2)
    c = CAM_CENTER[:2]
    return xy + (c - xy) * (height / CAM_HEIGHT)


def table_to_px(xyz_table_up):
    """Table XY + height-above-table -> raw pixel."""
    p = np.asarray(xyz_table_up, float).reshape(-1, 3).copy()
    p[:, 2] = -p[:, 2]
    rv, _ = cv2.Rodrigues(R_CAM)
    out, _ = cv2.projectPoints(p.reshape(-1, 1, 3), rv, T_CAM, K, D)
    return out.reshape(-1, 2)


# ---------------------------------------------------------------- robot alignment
def load_map():
    return json.loads(ROBOT_MAP.read_text()) if ROBOT_MAP.exists() else None


def table_to_robot(xy, m=None):
    m = m or load_map()
    A = np.array(m["A"]); b = np.array(m["b"])
    return (np.asarray(xy, float).reshape(-1, 2) @ A.T) + b


def robot_to_table(xy, m=None):
    m = m or load_map()
    A = np.array(m["A"]); b = np.array(m["b"])
    return (np.asarray(xy, float).reshape(-1, 2) - b) @ np.linalg.inv(A).T


def fit_rigid(table_xy, robot_xy, reflect=None):
    """Least-squares robot = A @ table + b with A a rotation (optionally composed with a reflection)."""
    P = np.asarray(table_xy, float); Q = np.asarray(robot_xy, float)
    best = None
    for refl in ([reflect] if reflect is not None else [1, -1]):
        Pr = P * [1, refl]
        pc, qc = Pr.mean(0), Q.mean(0)
        Hm = (Pr - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(Hm)
        Rm = Vt.T @ U.T
        if np.linalg.det(Rm) < 0:
            Vt[-1] *= -1; Rm = Vt.T @ U.T
        A = Rm @ np.diag([1, refl]); b = qc - (A @ P.mean(0))
        res = np.linalg.norm(P @ A.T + b - Q, axis=1)
        if best is None or res.mean() < best[2].mean():
            best = (A, b, res, refl)
    A, b, res, refl = best
    return {"A": A.tolist(), "b": b.tolist(), "reflect": int(refl),
            "residual_mm": [float(r * 1000) for r in res]}


# ---------------------------------------------------------------- detection
CUBE_SIDE = 0.04  # nominal; refined from detected footprint size if needed


def dark_mask(img, thresh=70):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = (g < thresh).astype(np.uint8)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


def detect_cube(img, min_area=2500, max_area=20000):
    """Dark, compact blob not touching the image top (the arm enters from the top).

    Returns dict(px=(u,v) centroid, angle_deg, area, box) or None.
    """
    m = dark_mask(img)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    best = None
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a < min_area or a > max_area or y <= 2:
            continue
        if x <= 2 or x + w >= img.shape[1] - 2 or y + h >= img.shape[0] - 2:
            continue
        ys, xs = np.nonzero(lab == i)
        pts = np.column_stack([xs, ys]).astype(np.float32)
        rect = cv2.minAreaRect(pts)
        (cw, ch) = rect[1]
        if min(cw, ch) < 30 or max(cw, ch) / max(1, min(cw, ch)) > 1.8:
            continue
        fill = a / max(1.0, cw * ch)
        if fill < 0.7:
            continue
        score = fill
        if best is None or score > best["score"]:
            best = {"px": tuple(map(float, cent[i])), "angle_deg": float(rect[2]), "area": int(a),
                    "box": cv2.boxPoints(rect).tolist(), "score": float(score), "wh": (float(cw), float(ch))}
    return best


def arm_tip_px(img):
    """Lowest dark pixel of the blob touching the top border: the fingertip when the arm reaches forward."""
    m = dark_mask(img)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    ids = [i for i in range(1, n) if stats[i][1] <= 2 and stats[i][4] > 3000]
    if not ids:
        return None
    i = max(ids, key=lambda k: stats[k][4])
    ys, xs = np.nonzero(lab == i)
    ymax = ys.max()
    sel = ys >= ymax - 4
    return float(xs[sel].mean()), float(ymax)
