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
EXCLUDE = [(1080, 0, 1280, 720), (0, 0, 260, 720), (0, 0, 1280, 120)]  # outside the arm's reach / clutter at the table edges
# The parked arm hangs into the top-centre of the image; instead of a fixed box, any dark
# component that touches the top image border within this u-range is treated as the arm.
ARM_TOP_U = (380, 950)
DARK = 70            # grey level below which a pixel counts as "black"
MIN_AREA = 1200      # px^2  (cube ~ 65x75 px; partially masked by the arm still counts)
MAX_AREA = 12000


def undistort_pts(pts):
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)


ARM_MASK_PATH = HERE / "arm_home_mask.png"   # dilated silhouette of the arm parked at home
_arm_mask = cv2.imread(str(ARM_MASK_PATH), cv2.IMREAD_GRAYSCALE) if ARM_MASK_PATH.exists() else None


def detect_cube(img, exclude=EXCLUDE, debug=None, use_arm_mask=True):
    """Return the most cube-like dark blob in the raw top image, or None."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = (gray < DARK).astype(np.uint8) * 255
    for x0, y0, x1, y1 in exclude:
        mask[y0:y1, x0:x1] = 0
    if use_arm_mask and _arm_mask is not None and _arm_mask.shape == mask.shape:
        mask[_arm_mask > 0] = 0   # a cube touching the parked arm keeps its own pixels
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if y <= 2 and ARM_TOP_U[0] <= x + w / 2 <= ARM_TOP_U[1] + w:
            continue  # the robot arm itself (or whatever hangs in from the top edge)
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
    if win.size == 0:
        return 0.0
    valid = np.ones(win.shape, bool)
    if _arm_mask is not None and _arm_mask.shape == gray.shape:
        valid = _arm_mask[v0:int(v + half), u0:int(u + half)] == 0   # ignore the parked arm's own pixels
    if valid.sum() < 0.25 * win.size:
        return 0.0   # spot is mostly under the arm's silhouette: cannot judge, assume clear
    return float((win[valid] < DARK).mean())


def _similarity(src, dst):
    """Least-squares similarity (Umeyama) src->dst as a 3x3 matrix, reflection enforced."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, Dd = src - mu_s, dst - mu_d
    cov = Dd.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(cov)
    # The top camera looks down with image-down = robot +x and image-right = robot +y,
    # which is a reflection (det < 0); force that chirality so two points suffice.
    Sgn = np.eye(2)
    if np.linalg.det(U @ Vt) > 0:
        Sgn[1, 1] = -1
    R = U @ Sgn @ Vt
    var_s = (S ** 2).sum() / len(src)
    c = (sig * np.diag(Sgn)).sum() / var_s
    t = mu_d - c * R @ mu_s
    H = np.eye(3)
    H[:2, :2] = c * R
    H[:2, 2] = t
    return H


AFFINE_MIN_POINTS = 8
LOCAL_MIN_POINTS = 20
LOCAL_SIGMA_M = 0.03      # kernel width of the local residual correction
LOCAL_PRIOR_W = 0.5       # shrinks the correction toward zero where few neighbours exist
TRIM_M = 0.03


def _affine_trimmed(src, dst):
    """Least-squares affine (6 dof) with one pass of outlier trimming at TRIM_M."""
    def fit(S, Dd):
        Aeq = np.hstack([S, np.ones((len(S), 1))])
        M, *_ = np.linalg.lstsq(Aeq, Dd, rcond=None)   # 3x2
        H = np.eye(3)
        H[:2, :3] = M.T
        return H
    H = fit(src, dst)
    pred = (H @ np.hstack([src, np.ones((len(src), 1))]).T).T[:, :2]
    err = np.linalg.norm(pred - dst, axis=1)
    keep = err < TRIM_M
    if keep.sum() >= 6 and keep.sum() < len(src):
        H = fit(src[keep], dst[keep])
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
        # A homography from a handful of clustered points extrapolates wildly, so stay
        # with a similarity until there are enough well-spread points for a trimmed affine.
        if n >= AFFINE_MIN_POINTS:
            self.H, self.kind = _affine_trimmed(src, dst), "affine"
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

    def _global(self, u, v):
        su, sv = undistort_pts([[u, v]])[0]
        p = self.H @ np.array([su, sv, 1.0])
        return np.array([p[0] / p[2], p[1] / p[2]])

    def _residual_table(self):
        if getattr(self, "_res_cache_n", -1) != len(self.points):
            preds = np.array([self._global(p["u"], p["v"]) for p in self.points]) if self.points else np.zeros((0, 2))
            trues = np.array([[p["x"], p["y"]] for p in self.points]) if self.points else np.zeros((0, 2))
            self._res_preds, self._res_vecs = preds, trues - preds
            self._res_cache_n = len(self.points)
        return self._res_preds, self._res_vecs

    def pixel_to_robot(self, u, v, local=True):
        """Global fit plus a kernel-weighted local correction from nearby placement residuals.

        The arm has repeatable ~1.5 cm positioning bias in places (e.g. near the base) that
        no planar model captures; the residuals of past placements fix it locally.
        """
        if self.H is None:
            raise RuntimeError("table map has no points yet")
        g = self._global(u, v)
        if local and len(self.points) >= LOCAL_MIN_POINTS:
            preds, vecs = self._residual_table()
            d2 = ((preds - g) ** 2).sum(1)
            w = np.exp(-d2 / (2 * LOCAL_SIGMA_M ** 2))
            w[np.linalg.norm(vecs, axis=1) > TRIM_M] = 0.0     # ignore tumbled-cube outliers
            g = g + (w[:, None] * vecs).sum(0) / (w.sum() + LOCAL_PRIOR_W)
        return float(g[0]), float(g[1])

    def robot_to_pixel(self, x, y):
        """Approximate inverse (undistorted pixel), for sanity checks."""
        Hi = np.linalg.inv(self.H)
        p = Hi @ np.array([x, y, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def residuals(self):
        out = []
        for p in self.points:
            x, y = self.pixel_to_robot(p["u"], p["v"], local=False)
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
