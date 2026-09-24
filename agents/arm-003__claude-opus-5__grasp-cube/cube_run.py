#!/usr/bin/env python3
"""Episode loop: detect black cube (top camera) -> grasp -> bring home -> confirm -> stop episode
-> place at a new position -> home -> repeat.  Self-calibrates the table->robot map from placements.

  /opt/armfarm/venv/bin/python tools/cube_run.py calib      # one unrecorded grasp+place to validate
  /opt/armfarm/venv/bin/python tools/cube_run.py run [N]    # N recorded episodes (default 1)
"""
import json
import math
import os
import random
import sys
import time
import traceback
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cube_pick as cp  # noqa: E402
import cube_servo as cs  # noqa: E402
import recording  # noqa: E402

TABLE_Z = -0.026          # contact probes (6 mm lag criterion) at 4 points: -0.024..-0.030
GRASP_Z = TABLE_Z + 0.012  # fingertip height while closing on the ~4 cm cube
HOVER_Z = 0.10
TRANSIT_Z = 0.15
PLACE_Z = GRASP_Z + 0.008
SERVO_Z = TABLE_Z + 0.088
LOW_GAP_PX = -25          # cube right edge minus fixed-jaw column at SERVO_LOW (negative: past the jaw column;
                          # biased so the mid check lands near MID_RIGHT first time; mid0 was ~+40 px left)
LOW_SCALE = 1.5           # wrist-camera Jacobian magnification at SERVO_LOW relative to SERVO_Z
MID_RIGHT = 770.          # desired cube right edge (px) at GRASP_Z+0.024, just above the cube top
YAW_ROLL_OFFSET = 30.     # empirical bias between top-camera yaw and jaw alignment
MAX_GRASP_R = 0.335       # max radius from the pan axis for a recorded grasp attempt
MID_V = 440.              # desired cube centroid row at GRASP_Z+0.024
MID_V_TOL = 30.           # 3/4 failures on 2026-09-23 had mid v < 412 with the old +-60 tolerance
LOW_V_PER_TILT = 2.5      # px of extra LOW_V per degree of approach tilt
LOW_V = 385.              # desired cube centroid row at SERVO_LOW (mid0 was ~+20 px too far along v)
SERVO_LOW = TABLE_Z + 0.050  # final servo height: tips just above the cube top   # fingertip height for wrist-camera servoing
ROLL_NEUTRAL = -30.       # wrist roll hard stop measured at +22 deg; free to at least -110
ROLL_RANGE = (-95., 12.)
GRIP_OPEN = 95.
GRIP_CLOSED = 0.
GRIP_HOLD_MIN = 6.        # gripper % above which something is between the jaws
# Servo gains.  Jacobians in wrist_servo.json were refitted from ~300 logged steps on 2026-09-23;
# with the old (2x too small along tool-x) Jacobian, gain 0.65 oscillated for 5-6 iterations.
SERVO_GAIN = 0.8
LOW_GAIN = 0.75
MID_GAIN = 0.7
FAST = 70.                # dps for transit moves
SERVO_SPEED = 45.         # dps for servo correction moves
IMG_DIR = Path("/tmp/cube_run")
IMG_DIR.mkdir(exist_ok=True)

PLACE_X = [0.13, 0.17, 0.21, 0.25, 0.29, 0.33]
PLACE_Y = [-0.16, -0.11, -0.06, -0.01, 0.04, 0.09, 0.14, 0.19]
CENTRAL = [(0.21, -0.03), (0.25, 0.06), (0.18, 0.08), (0.27, -0.06), (0.22, 0.12), (0.16, -0.05)]
STATE_FILE = HERE / "cube_run_state.json"


def log(msg, **data):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
    cp.log_event("log", msg=msg, **data)


def ik_reach(x, y, z, roll, tilt_start=0.):
    last = None
    for tilt in (t for t in (0., 10., 20., 30., 40., 50.) if t >= tilt_start):
        try:
            return cp.solve_ik(x, y, z, roll=roll, tilt_deg=tilt), tilt
        except ValueError as e:
            last = e
    raise ValueError(f"unreachable ({x:.3f},{y:.3f},{z:.3f}): {last}")


def hover_pose(x, y, roll, tilt):
    last = None
    for z in (HOVER_Z, 0.08, 0.06, 0.05):
        try:
            return ik_reach(x, y, z, roll, tilt_start=tilt)[0]
        except ValueError as e:
            last = e
    raise last


def choose_roll(x, y, z, yaw_deg, tilt):
    """Roll so the jaw closing direction matches the cube yaw (mod 90), within ROLL_RANGE."""
    base = cp.solve_ik(x, y, z, roll=ROLL_NEUTRAL, tilt_deg=tilt)
    yaw0 = cp.jaw_yaw(base)
    probe = dict(base); probe["wrist_roll"] = ROLL_NEUTRAL + 10.
    sign = 1. if ((cp.jaw_yaw(probe) - yaw0 + 180) % 360 - 180) > 0 else -1.
    delta = (yaw_deg - yaw0 + 45) % 90 - 45
    cands = [ROLL_NEUTRAL + sign * (delta + k) for k in (0, 90, -90)]
    cands = [c for c in cands if ROLL_RANGE[0] <= c <= ROLL_RANGE[1]]
    cands.sort(key=lambda c: abs(c - ROLL_NEUTRAL))
    return cands[0] if cands else ROLL_NEUTRAL


def path_min_z(a, b, n=25):
    zs = []
    for f in np.linspace(0, 1, n):
        j = {k: a[k] + f * (b[k] - a[k]) for k in cp.ARM_JOINTS}
        zs.append(cp.fk_xyz(j)[2])
    return min(zs)


def travel(arm, target, speed=FAST, precise=False):
    """Joint-space move, inserting a lifted waypoint if the straight path would dip near the table."""
    cur = arm.read()
    if path_min_z(cur, target) < TABLE_Z + 0.045:
        pos = cp.fk_xyz(target)
        try:
            wp, _ = ik_reach(pos[0], pos[1], TRANSIT_Z, target["wrist_roll"])
            log("travel via lifted waypoint")
            arm.move_arm(wp, speed_dps=speed)
        except ValueError:
            log("travel via home")
            arm.home()
    if precise:
        return arm.move_precise(target, speed_dps=speed)
    return arm.move_arm(target, speed_dps=speed)


def cube_pose(tmap, img, debug=None):
    c = cp.detect_cube(img, debug_path=debug)
    if not c:
        return None
    rxy = tmap.pixel_to_robot(c["px"])
    box = cv2.boxPoints(c["rect"])
    rb = np.array([tmap.pixel_to_robot(p) for p in box])
    e1, e2 = rb[1] - rb[0], rb[2] - rb[1]
    e = e1 if np.linalg.norm(e1) >= np.linalg.norm(e2) else e2
    yaw = math.degrees(math.atan2(e[1], e[0]))
    return dict(px=c["px"], area=c["area"], robot=(float(rxy[0]), float(rxy[1])), yaw=yaw,
                size=(float(np.linalg.norm(e1)), float(np.linalg.norm(e2))))


def wrist_dark_fraction(img):
    """Fraction of dark pixels in the between-jaws region of the wrist camera."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    roi = gray[300:600, 420:860]
    return float((roi < 70).mean())


class Runner:
    def __init__(self, arm):
        self.arm = arm
        self.tmap = cp.TableMap()
        self.servo = cs.WristServo()
        self.state = json.loads(STATE_FILE.read_text()) if STATE_FILE.exists() else {"episodes": [], "place_idx": 0, "order": []}

    def save_state(self):
        STATE_FILE.write_text(json.dumps(self.state, indent=1, default=float))

    # ------------------------------------------------------------ perception
    def observe(self, tag):
        time.sleep(0.4)
        img = cp.snap("top", IMG_DIR / f"{tag}_top.jpg")
        pose = cube_pose(self.tmap, img, debug=IMG_DIR / f"{tag}_detect.jpg")
        return pose, img

    # ------------------------------------------------------------ actions
    def pick(self, pose, notes):
        arm = self.arm
        x, y = pose["robot"]
        _, tilt = ik_reach(x, y, GRASP_Z, ROLL_NEUTRAL)
        roll = choose_roll(x, y, GRASP_Z, pose["yaw"], tilt)
        # the top-camera yaw estimate is systematically ~30 deg off (wrist-camera fix always adds ~+30)
        for cand in (roll + YAW_ROLL_OFFSET, roll + YAW_ROLL_OFFSET - 90., roll + YAW_ROLL_OFFSET + 90.):
            if ROLL_RANGE[0] <= cand <= ROLL_RANGE[1]:
                roll = cand
                break
        servo_j, tilt = ik_reach(x, y, SERVO_Z, roll, tilt_start=tilt)
        hover = hover_pose(x, y, roll, tilt)
        notes.append(f"target=({x:.3f},{y:.3f}) yaw={pose['yaw']:.0f} roll={roll:.0f} tilt={tilt:.0f}")
        arm.gripper(GRIP_OPEN, seconds=0.5)
        travel(arm, hover)
        arm.move_precise(servo_j, speed_dps=60)
        cx, cy = x, y
        converged = False
        for stage, z in enumerate((SERVO_Z,)):
            if stage > 0:
                servo_j, _ = ik_reach(cx, cy, z, roll, tilt_start=tilt)
                arm.move_precise(servo_j, speed_dps=30)
            if not self.servo.has(z):
                log(f"calibrating wrist-camera Jacobian at z={z:.3f}")
                J = self.servo.calibrate(arm, servo_j, z, roll, tilt, ik_reach, debug_dir=str(IMG_DIR))
                log(f"Jacobian px/m: {J.round(0).tolist()}")
            converged = False
            for it in range(6):
                time.sleep(0.35)
                img = cp.snap("wrist", IMG_DIR / f"servo{stage}_{it}.jpg")
                c = cs.detect_cube_wrist(img, debug_path=IMG_DIR / f"servo{stage}_{it}_det.jpg")
                if not c:
                    notes.append(f"servo{stage}.{it}: cube not in wrist view")
                    log("servo: cube not visible in wrist camera")
                    break
                d, err_px = self.servo.step(arm.read(), c["px"], z, gain=SERVO_GAIN)
                log(f"servo{stage}.{it}: cube px={np.round(c['px'])} err={err_px:.0f}px move=({d[0]*1000:.0f},{d[1]*1000:.0f})mm")
                if err_px < 20:
                    converged = True
                    break
                cx, cy = cx + float(d[0]), cy + float(d[1])
                if math.hypot(cx - x, cy - y) > 0.07 or not (0.10 <= cx <= 0.37 and -0.17 <= cy <= 0.27):
                    notes.append("servo: correction exceeded safety bounds; aborting servo")
                    log("servo: correction exceeded safety bounds; aborting")
                    converged = False
                    cx, cy = x, y
                    break
                try:
                    servo_j, _ = ik_reach(cx, cy, z, roll, tilt_start=tilt)
                except ValueError as e:
                    notes.append(f"servo: {e}")
                    break
                arm.move_precise(servo_j, speed_dps=SERVO_SPEED)
        notes.append(f"servo_final=({cx:.3f},{cy:.3f}) converged={converged}")
        # align jaw closing direction with the cube faces using the wrist-camera rectangle angle
        def img_delta():
            time.sleep(0.3)
            c = cs.detect_cube_wrist(cp.snap("wrist"), debug_path=IMG_DIR / "yaw_det.jpg")
            if not c:
                return None
            return ((c["angle"] + 45.) % 90.) - 45.
        d0 = img_delta()
        if d0 is not None and abs(d0) > 7.:
            sign = self.servo.roll_sign or 1.
            for trial in range(2):
                cand = roll - sign * d0
                if not (ROLL_RANGE[0] <= cand <= ROLL_RANGE[1]):
                    cand = roll - sign * (d0 - 90. if d0 > 0 else d0 + 90.)
                if not (ROLL_RANGE[0] <= cand <= ROLL_RANGE[1]):
                    break
                servo_j, _ = ik_reach(cx, cy, SERVO_Z, cand, tilt_start=tilt)
                arm.move_precise(servo_j, speed_dps=40)
                d1 = img_delta()
                log(f"yaw fix: image angle delta {d0:.0f} -> roll {roll:.0f}->{cand:.0f} -> delta {d1 if d1 is None else round(d1)}")
                if d1 is None:
                    roll = cand
                    break
                if abs(d1) < abs(d0) - 3.:
                    roll = cand
                    if not self.servo.roll_sign:
                        self.servo.roll_sign = sign
                        self.servo.save()
                    d0 = d1
                    if abs(d1) <= 7.:
                        break
                else:
                    # wrong sign: flip and retry from the original roll
                    sign = -sign
                    if not self.servo.roll_sign:
                        self.servo.roll_sign = sign
                        self.servo.save()
            notes.append(f"roll_after_yaw_fix={roll:.0f}")
            # re-centre after the roll change
            for it in range(2):
                time.sleep(0.3)
                c = cs.detect_cube_wrist(cp.snap("wrist"))
                if not c:
                    break
                d, err_px = self.servo.step(arm.read(), c["px"], SERVO_Z, gain=SERVO_GAIN)
                if err_px < 20:
                    break
                cx, cy = cx + float(d[0]), cy + float(d[1])
                servo_j, _ = ik_reach(cx, cy, SERVO_Z, roll, tilt_start=tilt)
                arm.move_precise(servo_j, speed_dps=SERVO_SPEED)
        low, _ = ik_reach(cx, cy, SERVO_LOW, roll, tilt_start=tilt)
        pre, _ = ik_reach(cx, cy, GRASP_Z + 0.03, roll, tilt_start=tilt)
        grasp, _ = ik_reach(cx, cy, GRASP_Z, roll, tilt_start=tilt)
        arm.move_precise(low, speed_dps=SERVO_SPEED)
        # final alignment: align at SERVO_LOW, then verify just above the cube top and retry if off
        if self.servo.J_low is not None:
            J_low = self.servo.J_low   # fitted from logged low-loop steps (2026-09-23)
        else:
            J_low = np.diag([1.0, 0.6]) @ (self.servo.J[self.servo.key(SERVO_Z)] * LOW_SCALE)
        low_v = LOW_V + LOW_V_PER_TILT * tilt
        z_mid = GRASP_Z + 0.024
        ok = False
        for attempt in range(3):
            for it in range(4 if attempt == 0 else 0):
                time.sleep(0.3)
                limg = cp.snap("wrist", IMG_DIR / f"low_wrist{attempt}_{it}.jpg")
                lc = cs.detect_cube_wrist(limg, debug_path=IMG_DIR / f"low_wrist{attempt}_{it}_det.jpg")
                if not lc:
                    notes.append("low: cube not in wrist view")
                    break
                bx, by, bw, bh = lc["bbox"]
                err = np.array([(cs.JAW_U - LOW_GAP_PX) - (bx + bw), low_v - lc["px"][1]])
                log(f"low{attempt}.{it}: cube right={bx+bw} v={lc['px'][1]:.0f} err=({err[0]:.0f},{err[1]:.0f})px")
                if abs(err[0]) < 12 and abs(err[1]) < 25:
                    break
                d_tool = np.linalg.solve(J_low, err) * LOW_GAIN
                n = np.linalg.norm(d_tool)
                if n > 0.025:
                    d_tool *= 0.025 / n
                tx, ty = self.servo.tool_axes_xy(arm.read())
                d = d_tool[0] * tx + d_tool[1] * ty
                cx, cy = cx + float(d[0]), cy + float(d[1])
                if math.hypot(cx - x, cy - y) > 0.08:
                    notes.append("low: correction exceeded bounds")
                    break
                low, _ = ik_reach(cx, cy, SERVO_LOW, roll, tilt_start=tilt)
                arm.move_precise(low, speed_dps=40)
            # verify just above the cube top
            jz, _ = ik_reach(cx, cy, z_mid, roll, tilt_start=tilt)
            arm.move_precise(jz, speed_dps=40)
            time.sleep(0.3)
            dimg = cp.snap("wrist", IMG_DIR / f"mid{attempt}.jpg")
            dc = cs.detect_cube_wrist(dimg, debug_path=IMG_DIR / f"mid{attempt}_det.jpg")
            if not dc:
                log(f"mid{attempt}: cube not detected; proceeding")
                notes.append(f"mid{attempt}: not detected")
                break
            right = dc["bbox"][0] + dc["bbox"][2]
            err = np.array([MID_RIGHT - right, MID_V - dc["px"][1]])
            log(f"mid{attempt}: right={right} v={dc['px'][1]:.0f} err=({err[0]:.0f},{err[1]:.0f})px")
            if abs(err[0]) <= 28 and abs(err[1]) <= MID_V_TOL:
                ok = True
                break
            d_tool = np.linalg.solve(J_low, err) * MID_GAIN
            n = np.linalg.norm(d_tool)
            if n > 0.02:
                d_tool *= 0.02 / n
            tx, ty = self.servo.tool_axes_xy(arm.read())
            d = d_tool[0] * tx + d_tool[1] * ty
            cx, cy = cx + float(d[0]), cy + float(d[1])
            if math.hypot(cx - x, cy - y) > 0.09:
                notes.append("mid: correction exceeded bounds")
                break
            # lift just above the cube top, shift, and re-verify (skip the low re-alignment)
            up, _ = ik_reach(cx, cy, z_mid + 0.02, roll, tilt_start=tilt)
            arm.move_precise(up, speed_dps=40)
        notes.append(f"mid_ok={ok}")
        grasp, _ = ik_reach(cx, cy, GRASP_Z, roll, tilt_start=tilt)
        now = arm.move_precise(grasp, speed_dps=35)
        fk = cp.fk_xyz(now)
        notes.append(f"grasp_fk=({fk[0]:.3f},{fk[1]:.3f},{fk[2]:.3f})")
        cp.snap("wrist", IMG_DIR / "grasp_wrist.jpg")
        cp.snap("top", IMG_DIR / "grasp_top.jpg")
        arm.gripper(GRIP_CLOSED, seconds=0.7, settle=0.5)
        time.sleep(0.3)
        gpos, gload = arm.gripper_state()
        notes.append(f"grip_pos={gpos:.1f} load={gload}")
        log(f"gripper after close: pos={gpos:.1f} load={gload}")
        held = gpos > GRIP_HOLD_MIN
        if held:
            # hold with a goal just inside the measured width instead of straining at full torque
            arm.command({"gripper": max(GRIP_CLOSED, gpos - 6.)})
            time.sleep(0.2)
        cp.snap("wrist", IMG_DIR / "closed_wrist.jpg")
        arm.move_arm(pre, speed_dps=40)
        arm.move_arm(hover, speed_dps=FAST)
        gpos2, _ = arm.gripper_state()
        notes.append(f"grip_pos_lifted={gpos2:.1f}")
        return held and gpos2 > GRIP_HOLD_MIN, hover

    def go_home_and_confirm(self, pose, notes):
        arm = self.arm
        arm.home()
        time.sleep(0.5)
        err = arm.home_error()
        gpos, gload = arm.gripper_state()
        wrist = cp.snap("wrist", IMG_DIR / "home_wrist.jpg")
        top = cp.snap("top", IMG_DIR / "home_top.jpg")
        frac = wrist_dark_fraction(wrist)
        seen = cp.detect_cube(top, debug_path=IMG_DIR / "home_detect.jpg")
        still_on_table = bool(seen and np.hypot(seen["px"][0] - pose["px"][0], seen["px"][1] - pose["px"][1]) < 120)
        held = gpos > GRIP_HOLD_MIN
        notes.append(f"home_err_max={max(abs(v) for v in err.values()):.1f} grip_home={gpos:.1f} wrist_dark={frac:.2f} cube_on_table={still_on_table}")
        log(f"home: grip={gpos:.1f} load={gload} wrist_dark={frac:.2f} still_on_table={still_on_table} home_err={ {k: round(v,1) for k,v in err.items()} }")
        return held and not still_on_table, dict(grip=gpos, wrist_dark=frac, still_on_table=still_on_table)

    def rake(self, pose, notes, variant=0):
        """Unrecorded recovery: drag a too-far cube toward the base with the closed gripper."""
        arm = self.arm
        x, y = pose["robot"]
        ang = math.atan2(y, x - 0.0388) + math.radians((0, 6, -6)[variant % 3])
        r = math.hypot(x - 0.0388, y)
        r_far = min(r + 0.035 + 0.02 * variant, 0.41)
        z_drag = TABLE_Z + 0.022 - 0.005 * variant
        roll = ROLL_NEUTRAL
        far = (0.0388 + r_far * math.cos(ang), r_far * math.sin(ang))
        near = (0.0388 + 0.24 * math.cos(ang), 0.24 * math.sin(ang))
        log(f"rake: cube r={r:.3f} -> drag from r={r_far:.3f} to 0.24 along {math.degrees(ang):.0f} deg")
        arm.gripper(GRIP_CLOSED, seconds=0.4)
        j_hi = tilt = None
        for dz in (0.07, 0.05, 0.035, 0.02, 0.01):
            try:
                j_hi, tilt = ik_reach(far[0], far[1], z_drag + dz, roll, tilt_start=30.)
                break
            except ValueError:
                continue
        if j_hi is None:
            raise RuntimeError("rake approach pose unreachable")
        travel(arm, j_hi)
        j_lo, _ = ik_reach(far[0], far[1], z_drag, roll, tilt_start=tilt)
        arm.move_precise(j_lo, speed_dps=25)
        steps = 6
        for i in range(1, steps + 1):
            f = i / steps
            px_, py_ = far[0] + f * (near[0] - far[0]), far[1] + f * (near[1] - far[1])
            j, _ = ik_reach(px_, py_, z_drag, roll)
            arm.move_arm(j, speed_dps=20)
        j_up, _ = ik_reach(near[0], near[1], z_drag + 0.08, roll)
        arm.move_arm(j_up, speed_dps=40)
        arm.home()
        notes.append("raked cube closer")

    def next_place(self):
        n_done = len(self.state["episodes"])
        if n_done < len(CENTRAL):
            return CENTRAL[n_done]
        if not self.state["order"]:
            grid = [(x, y) for x in PLACE_X for y in PLACE_Y]
            random.shuffle(grid)
            self.state["order"] = grid
        while self.state["order"]:
            x, y = self.state["order"].pop(0)
            try:
                # Keep placements a margin inside the graspable radius: a missed grasp out at r~0.32 can
                # shove the cube past the rake's reach (~0.41), where nothing on the arm can recover it.
                if math.hypot(x - 0.0388, y) > 0.305:
                    raise ValueError("too far")
                ik_reach(x, y, PLACE_Z, ROLL_NEUTRAL)
                px = self.tmap.robot_to_pixel((x, y))
                if 260 < px[0] < 1240 and 60 < px[1] < 690:
                    self.save_state()
                    return (x, y)
                log(f"skip place {x,y}: predicted pixel {px.round()} outside view")
            except ValueError:
                log(f"skip place {x,y}: unreachable")
        return random.choice(CENTRAL)

    def place(self, xy, notes):
        arm = self.arm
        x, y = xy
        roll = ROLL_NEUTRAL + random.uniform(-35, 35)
        joints, tilt = ik_reach(x, y, PLACE_Z, roll)
        pre, _ = ik_reach(x, y, PLACE_Z + 0.03, roll, tilt_start=tilt)
        hover = hover_pose(x, y, roll, tilt)
        travel(arm, hover)
        arm.move_arm(pre, speed_dps=60)
        now = arm.move_precise(joints, speed_dps=40)
        fk = cp.fk_xyz(now)
        arm.gripper(GRIP_OPEN * 0.8, seconds=0.6)
        arm.move_arm(pre, speed_dps=50)
        arm.move_arm(hover, speed_dps=FAST)
        arm.gripper(GRIP_CLOSED, seconds=0.4, settle=0.2)
        arm.home()
        notes.append(f"placed_at=({fk[0]:.3f},{fk[1]:.3f})")
        # learn: where did the cube land?
        pose, _ = self.observe("after_place")
        if pose:
            pred = self.tmap.robot_to_pixel((fk[0], fk[1]))
            dpx = float(np.hypot(pose["px"][0] - pred[0], pose["px"][1] - pred[1]))
            log(f"after place: cube px={np.round(pose['px'])} predicted={np.round(pred)} d={dpx:.0f}px")
            if dpx < 90:
                t = cp.pixel_to_table([pose["px"]])[0]
                self.tmap.add(t, (fk[0], fk[1]), note="place")
                res = self.tmap.residuals()
                log(f"map updated: {len(self.tmap.pairs)} pairs, mean residual {1000*np.mean(res):.1f} mm, A={self.tmap.A.round(4).tolist()}")
        return pose

    # ------------------------------------------------------------ episode
    def episode(self, record=True):
        global IMG_DIR
        IMG_DIR = Path("/tmp/cube_run") / time.strftime("%H%M%S")
        IMG_DIR.mkdir(parents=True, exist_ok=True)
        # /tmp is a 1 GB tmpfs: keep only the last few episodes' debug images (it filled up after ~440 episodes)
        old = sorted(p for p in IMG_DIR.parent.iterdir() if p.is_dir() and p.name.isdigit())
        for p in old[:-8]:
            for f in p.iterdir():
                f.unlink()
            p.rmdir()
        arm = self.arm
        notes = []
        t0 = time.time()
        arm.gripper(GRIP_CLOSED, seconds=0.4)
        arm.home()
        pose, _ = self.observe("start")
        if not pose:
            log("cube not detected at home; swinging arm aside to look, and opening gripper in case it is held")
            arm.gripper(GRIP_OPEN, seconds=0.5)
            park = dict(cp.cw.load_home(cp.WORKSPACE / "home_pose.json"))
            park["shoulder_pan"] = -70.
            arm.move(park, speed_dps=50)
            time.sleep(0.4)
            pose, _ = self.observe("start2")
            arm.gripper(GRIP_CLOSED, seconds=0.4)
            arm.home()
            if not pose:
                raise RuntimeError("cube not found on table")
        log(f"cube at px={np.round(pose['px'])} robot={np.round(pose['robot'],3)} yaw={pose['yaw']:.0f} size={np.round(pose['size'],3)}")
        for variant in range(3):
            r = math.hypot(pose["robot"][0] - 0.0388, pose["robot"][1])
            if r <= MAX_GRASP_R:
                break
            log(f"cube radius {r:.3f} beyond graspable {MAX_GRASP_R}; raking (unrecorded, variant {variant})")
            self.rake(pose, notes, variant=variant)
            pose, _ = self.observe("after_rake")
            if not pose:
                raise RuntimeError("cube lost after rake")
            log(f"after rake: cube robot={np.round(pose['robot'],3)}")
        if math.hypot(pose["robot"][0] - 0.0388, pose["robot"][1]) > MAX_GRASP_R:
            raise RuntimeError("cube still out of reach after raking")
        rec = None
        success = False
        info = {}
        if record:
            rec = recording.start_recording(task="grasp-cube: pick black cube, bring to home, confirm grasp")
            log(f"recording started: {rec}")
        try:
            held, hover = self.pick(pose, notes)
            log(f"grasp held={held}")
            if held:
                success, info = self.go_home_and_confirm(pose, notes)
            else:
                # An empty close leaves the cube on the table and the arm undamaged, so re-observe from
                # home (same state as the start of the episode) and retry the pick once before giving up.
                notes.append("gripper closed empty; retrying once")
                arm.home()
                pose2, _ = self.observe("retry")
                if pose2 and math.hypot(pose2["robot"][0] - 0.0388, pose2["robot"][1]) <= MAX_GRASP_R:
                    pose = pose2
                    log(f"retry: cube at px={np.round(pose['px'])} robot={np.round(pose['robot'],3)}")
                    held, hover = self.pick(pose, notes)
                    log(f"retry grasp held={held}")
                else:
                    notes.append("retry: cube not re-detected in reach")
                if held:
                    success, info = self.go_home_and_confirm(pose, notes)
                else:
                    notes.append("gripper closed empty; no grasp")
                    arm.home()
        except Exception as e:
            notes.append(f"error: {e!r}")
            log(f"episode error: {e!r}\n{traceback.format_exc()}")
            success = False
        result = None
        if record:
            result = recording.stop_recording(bool(success), "; ".join(notes))
            log(f"recording stopped success={success}: {json.dumps(result, default=str)[:400]}")
        # post-episode: place cube at a new position (or release if failed but holding)
        try:
            gpos, _ = arm.gripper_state()
            if success or (GRIP_HOLD_MIN < gpos < GRIP_OPEN - 15):
                xy = self.next_place()
                log(f"placing at {xy}")
                self.place(xy, notes)
            else:
                arm.gripper(GRIP_CLOSED, seconds=0.4)
                arm.home()
                # failed grasp: nudge map with the observed error if the cube is still there
        except Exception as e:
            log(f"place error: {e!r}\n{traceback.format_exc()}")
            arm.gripper(GRIP_OPEN * 0.6, seconds=0.5)
            arm.home()
            arm.gripper(GRIP_CLOSED, seconds=0.4)
        ep = dict(t=t0, dur=time.time() - t0, success=bool(success), notes=notes, recorded=record,
                  folder=(result or {}).get("folder") if isinstance(result, dict) else None, info=info)
        if record:
            self.state["episodes"].append(ep)
            self.save_state()
        n = len(self.state["episodes"]); s = sum(1 for e in self.state["episodes"] if e["success"])
        log(f"episode done success={success} dur={ep['dur']:.0f}s  totals: {s}/{n} successes")
        return success


def _on_term(signum, frame):
    try:
        st = recording.request("status")
        if st.get("recording"):
            recording.stop_recording(False, "aborted: process terminated mid-episode")
    finally:
        os._exit(1)


def main():
    import signal
    signal.signal(signal.SIGTERM, _on_term)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "run"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    with cp.Arm() as arm:
        r = Runner(arm)
        if cmd == "calib":
            r.episode(record=False)
        elif cmd == "run":
            for i in range(n):
                try:
                    r.episode(record=True)
                except Exception as e:
                    log(f"fatal episode error: {e!r}\n{traceback.format_exc()}")
                    try:
                        arm.home()
                    except Exception:
                        pass
                    break
        elif cmd == "place":
            xy = (float(sys.argv[2]), float(sys.argv[3]))
            r.place(xy, [])


if __name__ == "__main__":
    main()
