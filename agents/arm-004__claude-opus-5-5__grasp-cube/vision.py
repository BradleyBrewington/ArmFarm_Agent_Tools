"""Top-camera cube detection and pixel->base mapping."""
import json, sys
from pathlib import Path
import numpy as np, cv2
HERE = Path(__file__).resolve().parent
WS = HERE.parent
MAP = json.loads((WS / 'calibration/20260923T105734_870476479/workspace_map.json').read_text())
K = np.array(MAP['camera_matrix']); D = np.array(MAP['dist_coeffs'])
H = np.array(MAP['homography_undistorted_pixel_to_table_m'])
CAL = HERE / 'top_to_base.json'


def pix_to_table(px):
    px = np.asarray(px, float).reshape(-1, 1, 2)
    und = cv2.undistortPoints(px, K, D, P=K).reshape(-1, 2)
    t = cv2.perspectiveTransform(und.reshape(-1, 1, 2), H).reshape(-1, 2)
    return t


def fit_similarity(src, dst):
    """Least-squares 2D similarity src->dst (Umeyama); returns 2x3."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(0), dst.mean(0)
    S, Dd = src - ms, dst - md
    U, s, Vt = np.linalg.svd(Dd.T @ S)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1, d]) @ Vt
    scale = (s * [1, d]).sum() / (S ** 2).sum()
    t = md - scale * R @ ms
    return np.hstack([scale * R, t[:, None]])


def fit_affine(src, dst):
    src = np.asarray(src, float); A = np.hstack([src, np.ones((len(src), 1))])
    M, *_ = np.linalg.lstsq(A, np.asarray(dst, float), rcond=None)
    return M.T


def load_cal():
    return np.array(json.loads(CAL.read_text())['M'])


CUBE_CAL = HERE / 'cube_cal.json'


def _cube_correction():
    """Correction learned from cubes placed at known positions: affine on table
    coords when >=4 points, else a mean translation bias on the tip map."""
    if not CUBE_CAL.exists():
        return None
    pts = json.loads(CUBE_CAL.read_text())
    if not pts:
        return None
    px = np.array([p['px'] for p in pts]); tg = np.array([p['target'] for p in pts])
    if len(pts) >= 4:
        return ('affine', fit_affine(pix_to_table(px), tg))
    M = load_cal(); t = pix_to_table(px)
    raw = (M[:, :2] @ t.T).T + M[:, 2]
    return ('bias', (tg - raw).mean(0))


def pix_to_base(px, M=None, cube=True):
    corr = _cube_correction() if (cube and M is None) else None
    t = pix_to_table(px)
    if corr and corr[0] == 'affine':
        A = corr[1]; return (A[:, :2] @ t.T).T + A[:, 2]
    M = load_cal() if M is None else M
    out = (M[:, :2] @ t.T).T + M[:, 2]
    if corr:
        out = out + corr[1]
    return out


def base_to_pix(xy, M=None):
    M = load_cal() if M is None else M
    A = np.vstack([M, [0, 0, 1]]); Ai = np.linalg.inv(A)
    xy = np.asarray(xy, float).reshape(-1, 2)
    t = (Ai[:2, :2] @ xy.T).T + Ai[:2, 2]
    Hi = np.linalg.inv(H)
    und = cv2.perspectiveTransform(t.reshape(-1, 1, 2), Hi).reshape(-1, 2)
    # re-distort
    n = cv2.undistortPoints(und.reshape(-1, 1, 2), K, None).reshape(-1, 2)
    pts3 = np.hstack([n, np.ones((len(n), 1))])
    raw, _ = cv2.projectPoints(pts3, np.zeros(3), np.zeros(3), K, D)
    return raw.reshape(-1, 2)


def wrist_cube(img, thresh=70, min_area=3000):
    """Find the cube in the wrist image: largest dark blob not touching the top
    (jaws) or left (stand/floor) border. Returns dict or None."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (5, 5), 0)
    m = (g < thresh).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    h, w = g.shape
    best = None
    for i in range(1, n):
        x, y, bw, bh, a = st[i]
        if a < min_area or y <= 2 or x <= 2:
            continue
        if best is not None and a <= best['area']:
            continue
        cnt, _ = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(max(cnt, key=cv2.contourArea))
        if a / max(1.0, rect[1][0] * rect[1][1]) < 0.75:
            continue
        if True:
            best = dict(cx=float(cen[i][0]), cy=float(cen[i][1]), area=int(a), bbox=(int(x), int(y), int(bw), int(bh)),
                        angle=edge_angle(rect), rect=rect, touches_bottom=bool(y + bh >= h - 2), touches_right=bool(x + bw >= w - 2))
    return best


def edge_angle(rect):
    """Cube edge angle in degrees normalised to [-45, 45)."""
    a = float(rect[2])
    return (a + 45.0) % 90.0 - 45.0


def top_cube(img, thresh=60, min_area=1500, max_area=15000, arm_mask_y=None):
    """Dark roughly-square blobs in the top image. Returns list of dicts sorted by area."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (5, 5), 0)
    m = (g < thresh).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, bw, bh, a = st[i]
        if not min_area <= a <= max_area or y <= 2:
            continue
        ar = bw / float(bh)
        if not 0.6 < ar < 1.7:
            continue
        cnt, _ = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(max(cnt, key=cv2.contourArea))
        if a / max(1.0, rect[1][0] * rect[1][1]) < 0.7:
            continue
        out.append(dict(cx=float(cen[i][0]), cy=float(cen[i][1]), area=int(a), bbox=(int(x), int(y), int(bw), int(bh))))
    return sorted(out, key=lambda d: -d['area'])


BASE_PX = (690.0, 0.0)


def arm_tip(img, ref, dark=90, diff=45):
    """Farthest dark changed pixel (from the base) of the arm blob in the top image."""
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(int)
    r = cv2.GaussianBlur(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(int)
    m = ((np.abs(g - r) > diff) & (g < dark)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    # arm blob = largest component whose top is near the image top
    cands = [i for i in range(1, n) if st[i][1] < 150 and st[i][4] > 3000]
    if not cands:
        cands = [int(np.argmax(st[1:, 4])) + 1]
    i = max(cands, key=lambda k: st[k][4])
    ys, xs = np.nonzero(lab == i)
    d = (xs - BASE_PX[0]) ** 2 + (ys - BASE_PX[1]) ** 2
    k = np.argsort(d)[-30:]
    return float(xs[k].mean()), float(ys[k].mean())


_JAW = {}


def jaw_mask(name='jaw_mask70.png'):
    if name not in _JAW:
        _JAW[name] = cv2.imread(str(HERE / name), cv2.IMREAD_GRAYSCALE) > 0
    return _JAW[name]


def wrist_cube_low(img, thresh=70, min_area=4000):
    """Cube at approach height with jaws masked out. Returns bottom-edge centre."""
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    m = ((g < thresh) & ~jaw_mask()).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    h, w = g.shape
    cands = [i for i in range(1, n) if st[i][4] >= min_area and st[i][0] > 2]
    if not cands:
        return None
    i = max(cands, key=lambda k: st[k][4])
    x, y, bw, bh, a = st[i]
    ys, xs = np.nonzero(lab == i)
    bottom = np.percentile(ys, 98)
    low = ys > bottom - 0.3 * bh
    cnt, _ = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rect = cv2.minAreaRect(max(cnt, key=cv2.contourArea))
    return dict(bx=float(xs[low].mean()), by=float(bottom), area=int(a), bbox=(int(x), int(y), int(bw), int(bh)),
                angle=edge_angle(rect), touches_bottom=bool(y + bh >= h - 2))


def top_cube_yaw(img, d):
    """Cube face yaw in base frame (deg, mod 90) from the top-image blob."""
    x, y, bw, bh = d['bbox']
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    sub = (g[y:y + bh, x:x + bw] < 60).astype(np.uint8)
    cnt, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    box = cv2.boxPoints(cv2.minAreaRect(max(cnt, key=cv2.contourArea))) + [x, y]
    b = pix_to_base(box)
    e = b[1] - b[0]
    return float(np.degrees(np.arctan2(e[1], e[0])) % 90.0)
