"""Wrist-camera visual servo at a fixed hover height, with wrist-roll alignment."""
import sys, time, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'tools'))
import arm, vision, kin
from camd_client import read_frame

HOVER_Z = 0.08
CFG = HERE / 'servo.json'
ROLL_GAIN = 0.83           # image degrees per wrist-roll degree
ROLL_CENTER = 83.4
ROLL_RANGE = (20.0, 150.0)

# Jacobian measured in base frame at pan=-18, roll=83.4; convert to tool frame
_J_BASE = np.array([[6250.0, 450.0], [-1050.0, -5300.0]])
_A0 = np.array([[0.215, -0.974], [-0.895, -0.168]])  # placeholder, recomputed below


def _tool_A(joints):
    R = kin.pose(joints)[:3, :3]
    A = np.array([R[:2, 0], R[:2, 1]])
    return A


def _init():
    q, _, _ = kin.ik_down(0.30, 0.083, 0.08)
    q['wrist_roll'] = 83.4
    return _J_BASE @ np.linalg.inv(_tool_A(q))

J_TOOL = _init()


def cfg():
    d = {'target_px': [863.0, 394.0]}
    if CFG.exists():
        d.update(json.loads(CFG.read_text()))
    return d


def detect(tries=3):
    img = None
    for _ in range(tries):
        img, _ = read_frame('wrist')
        d = vision.wrist_cube(img)
        if d:
            return d, img
        time.sleep(0.1)
    return None, img


def servo(b, x, y, roll=None, tol_px=10, iters=7, gain=0.9, log=print, align=True):
    T = np.array(cfg()['target_px'])
    roll = ROLL_CENTER if roll is None else roll
    d = None
    for i in range(iters):
        arm.goto(b, x, y, HOVER_Z, roll=roll, speed=50, correct=3)
        time.sleep(0.25)
        d, img = detect()
        if d is None:
            log(f'servo {i}: cube not visible at {x:.3f},{y:.3f}')
            return None
        e = T - np.array([d['cx'], d['cy']])
        edge = d['touches_bottom'] or d['touches_right']
        log(f'servo {i}: tool=({x:.4f},{y:.4f}) roll={roll:.1f} px=({d["cx"]:.0f},{d["cy"]:.0f}) '
            f'err={np.linalg.norm(e):.0f}px ang={d["angle"]:.1f} edge={edge}')
        ang_ok = abs(d['angle']) < 5 or not align
        if np.linalg.norm(e) < tol_px and not edge and ang_ok:
            return x, y, roll, d
        if align and abs(d['angle']) >= 3:
            nr = roll - d['angle'] / ROLL_GAIN
            if not ROLL_RANGE[0] <= nr <= ROLL_RANGE[1]:
                alt = [nr + 90 / ROLL_GAIN, nr - 90 / ROLL_GAIN]
                nr = min(alt, key=lambda r: abs(r - ROLL_CENTER))
            roll = float(np.clip(nr, *ROLL_RANGE))
        Jb = J_TOOL @ _tool_A(arm.joints(b))
        step = np.clip(gain * np.linalg.solve(Jb, e), -0.04, 0.04)
        x, y = x + step[0], y + step[1]
    return x, y, roll, d


LOW_Z = 0.045
LOW_SCALE = 1.8


def low_target():
    return np.array(cfg().get('low_target', [780.0, 415.0]))


def servo_low(b, x, y, roll, tol_px=15, iters=6, gain=0.7, log=print):
    T = low_target()
    d = None
    for i in range(iters):
        arm.goto(b, x, y, LOW_Z, roll=roll, speed=30, correct=3)
        time.sleep(0.25)
        img, _ = read_frame('wrist')
        d = vision.wrist_cube_low(img)
        if d is None:
            log(f'low {i}: cube not visible'); return None
        e = T - np.array([d['bx'], d['by']])
        log(f'low {i}: tool=({x:.4f},{y:.4f}) bottom=({d["bx"]:.0f},{d["by"]:.0f}) err={np.linalg.norm(e):.0f}px')
        if np.linalg.norm(e) < tol_px:
            return x, y, d
        Jb = LOW_SCALE * J_TOOL @ _tool_A(arm.joints(b))
        step = np.clip(gain * np.linalg.solve(Jb, e), -0.02, 0.02)
        x, y = x + step[0], y + step[1]
    return x, y, d
