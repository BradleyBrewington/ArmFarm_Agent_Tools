#!/usr/bin/env python3
"""Cube pick-and-place library for arm-003 (SO101).

Run with /opt/armfarm/venv/bin/python.  Provides:
  - fk_T(joints)                 full 4x4 transform of gripper_frame_link (degrees in)
  - solve_ik(x, y, z, ...)       top-down (or tilted) IK for the 4 arm joints + roll
  - Arm                          motion wrapper (clamped goals, smooth interpolation)
  - detect_cube(img)             dark cube blob on the white table (top camera)
  - TableMap                     pixel -> table metres -> robot XY, fitted from data
"""
import json
import math
import os
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calibrate_workspace as cw  # noqa: E402
import camd_client  # noqa: E402
import fk as fkmod  # noqa: E402

WORKSPACE = HERE.parent
JOINTS = cw.JOINTS
ARM_JOINTS = cw.ARM_JOINTS
HOME_JOINTS = cw.HOME_JOINTS
PORT = os.environ.get("ARMFARM_SERIAL_PORT", cw.PORT)
MAP_FILE = HERE / "cube_map.json"
STATE_DIR = Path(os.environ.get("ARMFARM_STATE", "/tmp"))

# ---------------------------------------------------------------- kinematics

def _rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


def _chain():
    tree = ET.fromstring(fkmod.URDF)
    by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
    chain, link = [], "gripper_frame_link"
    while link != "base_link":
        joint = by_child[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    out = []
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        fixed = np.eye(4)
        fixed[:3, :3] = _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r)
        fixed[:3, 3] = xyz
        name = joint.get("name") if joint.get("type") != "fixed" else None
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ") if name else None
        out.append((fixed, name, axis))
    return out


CHAIN = _chain()
URDF_LIMITS = {}
for _j in ET.fromstring(fkmod.URDF).findall("joint"):
    if _j.get("name") in ARM_JOINTS:
        _l = _j.find("limit")
        URDF_LIMITS[_j.get("name")] = (math.degrees(float(_l.get("lower"))), math.degrees(float(_l.get("upper"))))


def fk_T(joints):
    """joints: dict of degrees for the five arm joints -> 4x4 transform of gripper_frame_link."""
    T = np.eye(4)
    for fixed, name, axis in CHAIN:
        T = T @ fixed
        if name is not None:
            moving = np.eye(4)
            moving[:3, :3] = _rot(axis, math.radians(float(joints[name])))
            T = T @ moving
    return T


def fk_xyz(joints):
    return fk_T(joints)[:3, 3]


def tool_axes(joints):
    """Returns (position, pointing direction (tool z), jaw closing direction (tool x)) in base_link."""
    T = fk_T(joints)
    return T[:3, 3], T[:3, 2], T[:3, 0]


# Live calibration limits in calibrated degrees (from config/motors/arm-003.json), symmetric about 0.
def _cal_limits():
    cfg = json.loads((WORKSPACE / "config/motors/arm-003.json").read_text())
    out = {}
    for j in ARM_JOINTS:
        half = (cfg[j]["range_max"] - cfg[j]["range_min"]) * 180 / 4095
        out[j] = (-half, half)
    return out


CAL_LIMITS = _cal_limits()
# Wrist roll: physical travel is narrower than calibration claims; stay conservative.
ROLL_LIMITS = (-120., 120.)


def joint_limits(j):
    lo = max(URDF_LIMITS[j][0], CAL_LIMITS[j][0])
    hi = min(URDF_LIMITS[j][1], CAL_LIMITS[j][1])
    if j == "wrist_roll":
        lo, hi = max(lo, ROLL_LIMITS[0]), min(hi, ROLL_LIMITS[1])
    return lo, hi


def solve_ik(x, y, z, roll=0., tilt_deg=0., seeds=None, axis_weight=0.08, tol_m=0.002):
    """Solve pan/lift/elbow/wrist_flex so gripper_frame_link is at (x,y,z) pointing down
    (tilted outward by tilt_deg away from the base).  wrist_roll is fixed to `roll`.
    Returns dict of degrees or raises ValueError.
    """
    from scipy.optimize import least_squares
    target = np.array([x, y, z], dtype=float)
    radial = np.array([x - 0.0388, y, 0.])
    radial = radial / max(np.linalg.norm(radial), 1e-6)
    t = math.radians(tilt_deg)
    want_axis = -math.cos(t) * np.array([0., 0., 1.]) + math.sin(t) * radial
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
    lo = np.array([joint_limits(j)[0] for j in names])
    hi = np.array([joint_limits(j)[1] for j in names])
    pan0 = math.degrees(-math.atan2(y, x - 0.0388353))
    if seeds is None:
        seeds = [[pan0, -20, 60, 60], [pan0, 0, 40, 50], [pan0, -45, 90, 45], [pan0, 20, 20, 50],
                 [pan0, -10, 80, 20], [pan0, 30, 0, 60]]

    def resid(q):
        joints = dict(zip(names, q))
        joints["wrist_roll"] = roll
        pos, axis, _ = tool_axes(joints)
        return np.concatenate([pos - target, axis_weight * (axis - want_axis)])

    best = None
    for s in seeds:
        q0 = np.clip(np.array(s, dtype=float), lo + 1e-3, hi - 1e-3)
        r = least_squares(resid, q0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=400)
        pos_err = float(np.linalg.norm(r.fun[:3]))
        ax_err = float(np.linalg.norm(r.fun[3:]) / axis_weight)
        score = pos_err + 0.02 * ax_err
        if best is None or score < best[0]:
            best = (score, pos_err, ax_err, r.x)
        if pos_err < tol_m and ax_err < 0.05:
            break
    score, pos_err, ax_err, q = best
    if pos_err > tol_m or ax_err > 0.25:
        raise ValueError(f"IK failed for {target.round(4).tolist()}: pos_err={pos_err*1000:.1f}mm axis_err={math.degrees(ax_err):.1f}deg")
    out = {j: float(v) for j, v in zip(names, q)}
    out["wrist_roll"] = float(roll)
    return out


def jaw_yaw(joints):
    """Yaw (degrees, base_link XY plane) of the jaw closing direction."""
    _, _, jaw = tool_axes(joints)
    return math.degrees(math.atan2(jaw[1], jaw[0]))


def roll_for_yaw(x, y, z, desired_yaw_deg, roll_guess=0., tilt_deg=0.):
    """Find wrist_roll so the jaw closing direction matches desired yaw (mod 90 for a cube).
    Returns (roll, joints)."""
    base = solve_ik(x, y, z, roll=roll_guess, tilt_deg=tilt_deg)
    yaw0 = jaw_yaw(base)
    # Empirically roll rotates the jaw yaw roughly 1:1 (sign determined below).
    probe = dict(base)
    probe["wrist_roll"] = roll_guess + 10.
    sign = 1. if ((jaw_yaw(probe) - yaw0 + 180) % 360 - 180) > 0 else -1.
    delta = (desired_yaw_deg - yaw0 + 45) % 90 - 45  # nearest equivalent mod 90
    roll = roll_guess + sign * delta
    lo, hi = joint_limits("wrist_roll")
    # try alternatives +-90 if out of range
    for cand in (roll, roll - sign * 90, roll + sign * 90):
        if lo <= cand <= hi:
            joints = solve_ik(x, y, z, roll=cand, tilt_deg=tilt_deg)
            return cand, joints
    return roll_guess, base


# ---------------------------------------------------------------- motion

class Arm:
    def __init__(self, port=PORT):
        self.port = port
        self._cm = None
        self.bus = None

    def __enter__(self):
        self._cm = cw.connected_bus(self.port)
        self.bus = self._cm.__enter__()
        cw.enable_at_current_position(self.bus)
        return self

    def __exit__(self, *exc):
        self._cm.__exit__(*exc)

    def read(self):
        return self.bus.sync_read("Present_Position", list(JOINTS))

    def read_raw(self, name, joints=None):
        return self.bus.sync_read(name, list(joints or JOINTS), normalize=False)

    def faults(self):
        return {j: cw.fault_bits(self.bus, j) for j in JOINTS}

    def clear_faults(self):
        for j in JOINTS:
            cw.recover_overload(self.bus, j)

    def command(self, target):
        goal = cw.clamp_target(target, self.bus.calibration)
        self.bus.sync_write("Goal_Position", goal, normalize=True)
        return goal

    def move(self, target, seconds=None, speed_dps=60., settle=0.3, tol=3.0, wait=True):
        """Smooth interpolation from the present position to target (dict of joint degrees / gripper %)."""
        target = cw.clamp_target(target, self.bus.calibration)
        start = self.bus.sync_read("Present_Position", list(target))
        start = cw.clamp_target(start, self.bus.calibration)
        if seconds is None:
            dist = max(abs(target[j] - start[j]) for j in target)
            seconds = max(0.4, dist / speed_dps)
        t0 = time.monotonic()
        while True:
            f = min((time.monotonic() - t0) / seconds, 1.)
            a = f * f * (3 - 2 * f)
            cmd = dict(target) if f >= 1 else {j: start[j] + a * (target[j] - start[j]) for j in target}
            self.command(cmd)
            if f >= 1:
                break
            time.sleep(0.02)
        if not wait:
            return None
        deadline = time.monotonic() + max(settle, 0.2) + 1.5
        while True:
            time.sleep(0.05)
            now = self.bus.sync_read("Present_Position", list(target))
            err = {j: abs(now[j] - target[j]) for j in target if j != "gripper"}
            if all(e < tol for e in err.values()) and time.monotonic() > t0 + seconds + settle:
                return now
            if time.monotonic() > deadline:
                return now

    def move_arm(self, joints, **kw):
        return self.move({j: joints[j] for j in ARM_JOINTS if j in joints}, **kw)

    def gripper(self, percent, seconds=0.6, settle=0.4):
        return self.move({"gripper": float(percent)}, seconds=seconds, settle=settle)

    def gripper_state(self):
        pos = self.bus.sync_read("Present_Position", ["gripper"])["gripper"]
        load = self.bus.sync_read("Present_Load", ["gripper"], normalize=False)["gripper"]
        return pos, load

    def home(self, seconds=None):
        home = cw.load_home(WORKSPACE / "home_pose.json")
        return self.move(home, seconds=seconds, speed_dps=50.)

    def home_error(self):
        home = cw.load_home(WORKSPACE / "home_pose.json")
        now = self.bus.sync_read("Present_Position", list(home))
        return {j: now[j] - home[j] for j in home}

    def xyz(self):
        pos = self.read()
        return fk_xyz(pos), pos


# ---------------------------------------------------------------- vision

def _load_calibration():
    ready = json.loads((WORKSPACE / "calibration/ready.json").read_text())
    for rel in ready["files"]:
        if rel.endswith("workspace_map.json"):
            data = json.loads((WORKSPACE / rel).read_text())
            return data
    raise FileNotFoundError("workspace_map.json not in calibration/ready.json")


CALIB = _load_calibration()
K = np.array(CALIB["camera_matrix"], dtype=float)
DIST = np.array(CALIB["dist_coeffs"], dtype=float)
H_TABLE = np.array(CALIB["homography_undistorted_pixel_to_table_m"], dtype=float)


def undistort_points(pts):
    pts = np.asarray(pts, dtype=float).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, DIST, P=K)
    return und.reshape(-1, 2)


def pixel_to_table(px):
    """Raw pixel(s) -> checkerboard-table metres."""
    und = undistort_points(px)
    homo = np.hstack([und, np.ones((len(und), 1))]) @ H_TABLE.T
    return homo[:, :2] / homo[:, 2:3]


def snap(cam="top", path=None):
    img, meta = camd_client.read_frame(cam)
    if path:
        cv2.imwrite(str(path), img)
    return img


def table_mask(gray):
    bright = (gray > 120).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(bright, connectivity=4)
    if n <= 1:
        return np.zeros_like(bright)
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = (labels == idx).astype(np.uint8)
    # fill holes (dark objects on the table) via flood fill from the border
    h, w = mask.shape
    ff = mask.copy()
    pad = np.zeros((h + 2, w + 2), np.uint8)
    inv = (1 - ff).astype(np.uint8)
    cv2.floodFill(inv, pad, (0, 0), 2)
    cv2.floodFill(inv, pad, (w - 1, 0), 2)
    cv2.floodFill(inv, pad, (0, h - 1), 2)
    cv2.floodFill(inv, pad, (w - 1, h - 1), 2)
    holes = (inv == 1).astype(np.uint8)
    filled = np.clip(mask + holes, 0, 1).astype(np.uint8)
    filled = cv2.erode(filled, np.ones((25, 25), np.uint8))
    return filled


def detect_cube(img, dark_thresh=90, min_area=700, max_area=12000, exclude=None, debug_path=None):
    """Find the cube as a compact dark blob fully inside the white table.
    Returns dict(px=(u,v), area, yaw_deg (image), rect) or None.
    exclude: optional binary mask (uint8) of pixels to ignore (e.g., the arm)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    tmask = table_mask(gray)
    dark = ((gray < dark_thresh) & (tmask > 0)).astype(np.uint8)
    if exclude is not None:
        dark[exclude > 0] = 0
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(dark, connectivity=8)
    best = None
    h, w = gray.shape
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if not (min_area <= area <= max_area):
            continue
        x, y, bw, bh = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        if y <= 2 or x <= 2 or x + bw >= w - 2 or y + bh >= h - 2:
            continue
        fill = area / float(bw * bh)
        aspect = max(bw, bh) / float(min(bw, bh))
        if fill < 0.45 or aspect > 2.2:
            continue
        comp = (labels == i).astype(np.uint8)
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
        cand = dict(px=(float(cents[i][0]), float(cents[i][1])), area=area, rect=rect,
                    yaw_img=float(rect[2]), bbox=(int(x), int(y), int(bw), int(bh)), fill=fill)
        score = area * fill
        if best is None or score > best[0]:
            best = (score, cand)
    if debug_path:
        dbg = img.copy()
        dbg[tmask == 0] = (dbg[tmask == 0] * 0.4).astype(np.uint8)
        if best:
            c = best[1]
            box = cv2.boxPoints(c["rect"]).astype(int)
            cv2.drawContours(dbg, [box], 0, (0, 255, 0), 2)
            cv2.circle(dbg, (int(c["px"][0]), int(c["px"][1])), 4, (0, 0, 255), -1)
        cv2.imwrite(str(debug_path), dbg)
    return best[1] if best else None


def draw_grid(img, step=100, path=None):
    out = img.copy()
    h, w = out.shape[:2]
    for x in range(0, w, step):
        cv2.line(out, (x, 0), (x, h), (0, 0, 255), 1)
        cv2.putText(out, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    for y in range(0, h, step):
        cv2.line(out, (0, y), (w, y), (255, 0, 0), 1)
        cv2.putText(out, str(y), (2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    if path:
        cv2.imwrite(str(path), out)
    return out


# ---------------------------------------------------------------- table -> robot mapping

class TableMap:
    """Affine map from checkerboard-table metres to robot base_link XY, fitted from
    (table_xy, robot_xy) correspondences.  Persisted in tools/cube_map.json."""

    def __init__(self, path=MAP_FILE):
        self.path = Path(path)
        self.pairs = []      # [ [tx, ty, rx, ry, note], ... ]
        self.A = None        # 2x3
        self.seed = None     # optional initial similarity params
        if self.path.exists():
            data = json.loads(self.path.read_text())
            self.pairs = data.get("pairs", [])
            self.A = np.array(data["A"]) if data.get("A") else None
        self.fit()

    def save(self):
        self.path.write_text(json.dumps({"pairs": self.pairs, "A": None if self.A is None else self.A.tolist(),
                                         "updated": time.time()}, indent=1))

    def add(self, table_xy, robot_xy, note=""):
        self.pairs.append([float(table_xy[0]), float(table_xy[1]), float(robot_xy[0]), float(robot_xy[1]), note])
        self.fit()
        self.save()

    def set_affine(self, A):
        self.A = np.array(A, dtype=float).reshape(2, 3)
        self.save()

    def fit(self):
        n = len(self.pairs)
        if n == 0:
            return self.A
        P = np.array([[p[0], p[1]] for p in self.pairs])
        R = np.array([[p[2], p[3]] for p in self.pairs])
        if n >= 4:
            X = np.hstack([P, np.ones((n, 1))])
            sol, *_ = np.linalg.lstsq(X, R, rcond=None)
            self.A = sol.T
        elif n >= 2:
            # similarity: [a -b tx; b a ty]
            rows, rhs = [], []
            for (px, py), (rx, ry) in zip(P, R):
                rows.append([px, -py, 1, 0]); rhs.append(rx)
                rows.append([py, px, 0, 1]); rhs.append(ry)
            a, b, tx, ty = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)[0]
            self.A = np.array([[a, -b, tx], [b, a, ty]])
        elif self.A is not None:
            # translation-only correction of the existing map
            pred = self.apply(P[0])
            self.A = self.A.copy()
            self.A[:, 2] += R[0] - pred
        return self.A

    def apply(self, table_xy):
        t = np.asarray(table_xy, dtype=float).reshape(2)
        return self.A @ np.array([t[0], t[1], 1.])

    def residuals(self):
        return [float(np.linalg.norm(self.apply(p[:2]) - np.array(p[2:4]))) for p in self.pairs]

    def pixel_to_robot(self, px):
        return self.apply(pixel_to_table([px])[0])

    def robot_to_table(self, robot_xy):
        M = np.vstack([self.A, [0, 0, 1]])
        inv = np.linalg.inv(M)
        r = np.array([robot_xy[0], robot_xy[1], 1.])
        return (inv @ r)[:2]

    def robot_to_pixel(self, robot_xy):
        """Approximate inverse for visualization (undistorted pixel)."""
        t = self.robot_to_table(robot_xy)
        Hinv = np.linalg.inv(H_TABLE)
        p = Hinv @ np.array([t[0], t[1], 1.])
        return p[:2] / p[2]


def log_event(kind, **data):
    rec = dict(t=time.time(), kind=kind, **data)
    with open(HERE / "cube_pick_log.jsonl", "a") as f:
        f.write(json.dumps(rec, default=float) + "\n")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["pos", "ik", "detect", "home", "goto", "grip", "snapgrid"])
    p.add_argument("args", nargs="*")
    a = p.parse_args()
    if a.cmd == "pos":
        with Arm() as arm:
            xyz, pos = arm.xyz()
            print(json.dumps({"joints": pos, "xyz": xyz.tolist(), "jaw_yaw": jaw_yaw(pos),
                              "axis": tool_axes(pos)[1].tolist()}, indent=1))
    elif a.cmd == "ik":
        x, y, z = map(float, a.args[:3])
        roll = float(a.args[3]) if len(a.args) > 3 else 0.
        tilt = float(a.args[4]) if len(a.args) > 4 else 0.
        j = solve_ik(x, y, z, roll=roll, tilt_deg=tilt)
        print(json.dumps(j, indent=1)); print("check", fk_xyz(j).round(4).tolist(), tool_axes(j)[1].round(3).tolist(), "jaw_yaw", jaw_yaw(j))
    elif a.cmd == "detect":
        img = snap("top", "/tmp/top.jpg")
        c = detect_cube(img, debug_path="/tmp/top_detect.jpg")
        print(json.dumps(c, default=str, indent=1))
        if c:
            print("table_xy", pixel_to_table([c["px"]])[0].tolist())
            tm = TableMap()
            if tm.A is not None:
                print("robot_xy", tm.pixel_to_robot(c["px"]).tolist())
    elif a.cmd == "home":
        with Arm() as arm:
            arm.home(); print(arm.home_error())
    elif a.cmd == "goto":
        x, y, z = map(float, a.args[:3])
        roll = float(a.args[3]) if len(a.args) > 3 else 0.
        tilt = float(a.args[4]) if len(a.args) > 4 else 0.
        j = solve_ik(x, y, z, roll=roll, tilt_deg=tilt)
        with Arm() as arm:
            now = arm.move_arm(j); print(now); print("fk", fk_xyz(now).round(4).tolist())
    elif a.cmd == "grip":
        with Arm() as arm:
            arm.gripper(float(a.args[0])); print(arm.gripper_state())
    elif a.cmd == "snapgrid":
        cam = a.args[0] if a.args else "top"
        img = snap(cam)
        draw_grid(img, path=f"/tmp/{cam}_grid.jpg"); print(f"/tmp/{cam}_grid.jpg")
