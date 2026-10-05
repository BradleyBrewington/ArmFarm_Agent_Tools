#!/usr/bin/env python3
"""Top-camera cube detection and pixel<->robot table mapping for the grasp-cube task.

    from vision import detect_cube, TableMap
    cube = detect_cube(img)            # {'u','v','angle','area','w','h'} or None
    tm = TableMap.load()               # tools/table_map.json (fitted from placements)
    x, y = tm.pixel_to_robot(u, v)

Detection: the cube is the only near-black, roughly square blob on the white table
outside the arm's parking region. Shadows are grey and are rejected by the threshold.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
MAP_PATH = HERE / "table_map.json"
CALIB = HERE.parent / "calibration" / "20261002T165916_799331938" / "workspace_map.json"
_cal = json.loads(CALIB.read_text())
K = np.array(_cal["camera_matrix"], dtype=np.float64)
D = np.array(_cal["dist_coeffs"], dtype=np.float64)

# Regions of the raw 1280x720 top image that can never hold the cube:
# the robot's own parking area at the top centre, the neighbouring station's arm at the
# top right, and the clamp at the bottom-left table edge.
EXCLUDE = [(430, 0, 900, 240), (1150, 0, 1280, 720), (0, 0, 110, 720)]
DARK = 70            # grey level below which a pixel counts as "black"
MIN_AREA = 1500      # px^2  (cube ~ 65x75 px)
MAX_AREA = 12000


def undistort_pts(pts):
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)


def detect_cube(img, exclude=EXCLUDE, debug=None):
    """Return the most cube-like dark blob in the raw top image, or None."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = (gray < DARK).astype(np.uint8) * 255
    for x0, y0, x1, y1 in exclude:
        mask[y0:y1, x0:x1] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if not MIN_AREA <= area <= MAX_AREA:
            continue
        fill = area / float(w * h)
        aspect = min(w, h) / float(max(w, h))
        if aspect < 0.5 or fill < 0.5:
            continue
        pts = np.column_stack(np.nonzero(labels == i))[:, ::-1].astype(np.float32)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        score = area * fill * aspect
        cand = {"u": float(cx), "v": float(cy), "angle": float(ang), "area": int(area),
                "w": int(w), "h": int(h), "rect_w": float(rw), "rect_h": float(rh), "score": float(score)}
        if best is None or score > best["score"]:
            best = cand
    if debug is not None:
        cv2.imwrite(str(debug), mask)
    return best


def cube_mask_dark_fraction(img, u, v, half=30):
    """Fraction of near-black pixels in a window: used to confirm the cube left its spot."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    u0, v0 = int(max(0, u - half)), int(max(0, v - half))
    win = gray[v0:int(v + half), u0:int(u + half)]
    return float((win < DARK).mean()) if win.size else 0.0


def _similarity(src, dst):
    """Least-squares similarity (Umeyama) src->dst as a 3x3 matrix; allows reflection."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, Dd = src - mu_s, dst - mu_d
    cov = Dd.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(cov)
    Sgn = np.eye(2)
    if np.linalg.det(U @ Vt) < 0:
        Sgn[1, 1] = -1
    R = U @ Sgn @ Vt
    var_s = (S ** 2).sum() / len(src)
    c = (sig * np.diag(Sgn)).sum() / var_s
    t = mu_d - c * R @ mu_s
    H = np.eye(3)
    H[:2, :2] = c * R
    H[:2, 2] = t
    return H


class TableMap:
    """Undistorted pixel -> robot (x, y) on the table plane.

    With >= 4 well-spread correspondences a homography is fitted; with fewer, a
    similarity transform (scale + rotation + translation) from the available points.
    """

    def __init__(self, points=None, z_table=-0.008):
        self.points = list(points or [])      # [{'u','v','x','y'}]
        self.z_table = z_table
        self.H = None
        self.kind = None
        self.fit()

    # ---- persistence
    @classmethod
    def load(cls, path=MAP_PATH):
        if Path(path).exists():
            d = json.loads(Path(path).read_text())
            return cls(d.get("points", []), d.get("z_table", -0.008))
        return cls()

    def save(self, path=MAP_PATH):
        Path(path).write_text(json.dumps({"points": self.points, "z_table": self.z_table,
                                          "kind": self.kind, "H": None if self.H is None else self.H.tolist()}, indent=1))

    # ---- fitting
    def add(self, u, v, x, y):
        self.points.append({"u": float(u), "v": float(v), "x": float(x), "y": float(y)})
        self.fit()

    def fit(self):
        n = len(self.points)
        if n == 0:
            self.H, self.kind = None, None
            return
        src = undistort_pts([[p["u"], p["v"]] for p in self.points])
        dst = np.array([[p["x"], p["y"]] for p in self.points], dtype=np.float64)
        if n >= 6:
            H, inl = cv2.findHomography(src, dst, cv2.RANSAC, 0.015)
            if H is not None and inl is not None and inl.sum() >= max(5, 0.7 * n):
                self.H, self.kind = H, "homography"
                return
        if n >= 2:
            self.H, self.kind = _similarity(src, dst), "similarity"
            return
        # one point: assume 0.42 mm/px, image-down = +x, image-right = +y
        s = 0.00042
        p = self.points[0]
        su, sv = src[0]
        self.H = np.array([[0, s, p["x"] - s * sv], [s, 0, p["y"] - s * su], [0, 0, 1]], dtype=np.float64)
        self.kind = "prior"

    def pixel_to_robot(self, u, v):
        if self.H is None:
            raise RuntimeError("table map has no points yet")
        su, sv = undistort_pts([[u, v]])[0]
        p = self.H @ np.array([su, sv, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def robot_to_pixel(self, x, y):
        """Approximate inverse (undistorted pixel), for sanity checks."""
        Hi = np.linalg.inv(self.H)
        p = Hi @ np.array([x, y, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def residuals(self):
        out = []
        for p in self.points:
            x, y = self.pixel_to_robot(p["u"], p["v"])
            out.append(math.hypot(x - p["x"], y - p["y"]))
        return out


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(HERE))
    from camd_client import read_frame
    img, _ = read_frame("top") if len(sys.argv) < 2 else (cv2.imread(sys.argv[1]), None)
    c = detect_cube(img, debug="/tmp/cube_mask.png")
    print(json.dumps(c, indent=1))
    tm = TableMap.load()
    if c and tm.H is not None:
        print("robot xy:", tm.pixel_to_robot(c["u"], c["v"]), "map:", tm.kind, "n=", len(tm.points))
