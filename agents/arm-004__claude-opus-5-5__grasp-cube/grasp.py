"""Top-camera-guided grasp: tool point offset toward the fixed jaw."""
import sys, time, math
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'tools'))
import numpy as np
import arm, kin

OPEN = 85.0
GRASP_Z = -0.008   # table surface z~-0.021 (fingertip-shadow contact test); cube centre ~-0.006
JAW_OFFSET = -0.030    # tool point = cube centre + 0.02*tool_x (fixed jaw on the -x side)
EMPTY_BELOW = 4.0


def tool_x(x, y, roll):
    q, _, _ = kin.ik_down(x, y, 0.02); q['wrist_roll'] = roll
    return kin.pose(q)[:2, 0]


def choose_roll(x, y, yaw):
    """Roll so tool x is along a cube face normal; prefer near 83 deg."""
    best = None
    for k in range(-4, 5):
        for roll in np.arange(20, 150.1, 1.0):
            pass
    # tool-x yaw ~ roll + c(pan); solve numerically
    c = math.degrees(math.atan2(*tool_x(x, y, 0.0)[::-1]))
    opts = []
    for k in range(-4, 5):
        r = yaw + 90 * k - c
        r = (r + 180) % 360 - 180
        if 20 <= r <= 150:
            opts.append(r)
    return min(opts, key=lambda r: abs(r - 83.4))


def wrist_roll(b, cx, cy, roll0=83.4, log=print):
    """Hover above the cube and pick a roll that squares the jaws to its faces."""
    import servo
    roll = roll0
    for _ in range(2):
        arm.goto(b, cx, cy, 0.11, roll=roll, speed=45, correct=1)
        time.sleep(0.25)
        d, _ = servo.detect()
        if d is None:
            log('wrist: cube not seen for roll alignment'); return None
        if abs(d['angle']) < 3:
            break
        nr = roll - d['angle'] / servo.ROLL_GAIN
        opts = [nr + k * 90 / servo.ROLL_GAIN for k in (-2, -1, 0, 1, 2)]
        opts = [r for r in opts if 15 <= r <= 155] or [nr]
        roll = float(min(opts, key=lambda r: abs(r - 83.4)))
    return roll


# Jaw axis yaw measured relative to the arm heading: grasps with (jaw yaw - heading) in
# -120..-60 succeed ~95%; -180..-150 fail almost always (run10-19 logs, 2026-09-23).
# A fixed world yaw of -100 pushed high +y headings into the bad band.
PREFERRED_JAW_REL = -95.0


def prefer_roll(cx, cy, roll, off=None):
    """Among rolls equivalent for a square cube (90 deg apart), pick the one whose
    jaw axis yaw is closest to the verified-good direction, keeping the tool clear of the base."""
    off = JAW_OFFSET if off is None else off
    best = None
    want = math.degrees(math.atan2(cy, cx - 0.0388)) + PREFERRED_JAW_REL
    for k in (-2, -1, 0, 1, 2):
        r = roll + 90.0 * k
        if not 10.0 <= r <= 160.0:
            continue
        tx = tool_x(cx, cy, r)
        if math.hypot(cx - off * tx[0] - 0.0388, cy - off * tx[1]) < 0.12:
            continue
        yaw = math.degrees(math.atan2(tx[1], tx[0]))
        d = abs((yaw - want + 180) % 360 - 180)
        if best is None or d < best[0]:
            best = (d, r)
    return best[1] if best else roll


def grasp(b, cx, cy, yaw, log=print, off=JAW_OFFSET):
    arm.gripper(b, OPEN)
    roll = wrist_roll(b, cx, cy, log=log)
    if roll is None:
        roll = choose_roll(cx, cy, yaw)
    roll = prefer_roll(cx, cy, roll, off)
    tx = tool_x(cx, cy, roll)
    x, y = cx - off * tx[0], cy - off * tx[1]
    log(f'grasp cube=({cx:.3f},{cy:.3f}) yaw={yaw:.1f} roll={roll:.1f} tool=({x:.3f},{y:.3f})')
    arm.gripper(b, OPEN)
    arm.goto(b, x, y, 0.06, roll=roll, speed=35, correct=1)
    arm.goto(b, x, y, GRASP_Z, roll=roll, speed=22, correct=3)
    time.sleep(0.2)
    low = arm.tool_xyz(b)
    try:
        from camd_client import read_jpeg
        (HERE / 'diag').mkdir(exist_ok=True)
        (HERE / 'diag' / f'low_{int(time.time())}.jpg').write_bytes(read_jpeg('top')[0])
    except Exception:
        pass
    log(f'low at {low.round(4).tolist()}')
    for g in (45, 25, 12, 5, 0):
        pos = arm.gripper(b, g, 0.3)
        if pos > g + 6:
            break
    time.sleep(0.3)
    grip = arm.joints(b)['gripper']
    if grip > EMPTY_BELOW:
        arm.gripper(b, max(0.0, grip - 1), 0.2)
    arm.goto(b, x, y, 0.10, roll=roll, speed=30, correct=1)
    time.sleep(0.3)
    held = arm.joints(b)['gripper']
    return {'ok': held > EMPTY_BELOW, 'grip': grip, 'held': held, 'x': x, 'y': y, 'roll': roll}


def place(b, cx, cy, roll, log=print, z=-0.006, off=None):
    """Put the held cube down with its centre at (cx, cy)."""
    off = JAW_OFFSET if off is None else off
    tx = tool_x(cx, cy, roll)
    x, y = cx - off * tx[0], cy - off * tx[1]
    log(f'place cube=({cx:.3f},{cy:.3f}) tool=({x:.3f},{y:.3f}) roll={roll:.1f}')
    arm.goto(b, x, y, 0.08, roll=roll, speed=40, correct=1)
    arm.goto(b, x, y, z, roll=roll, speed=20, correct=2)
    arm.gripper(b, OPEN, 0.5)
    arm.goto(b, x, y, 0.09, roll=roll, speed=30, correct=0)
    return x, y


def pull_in(b, cx, cy, r_end=0.14, log=print):
    """Drag a cube lying beyond vertical reach back toward the shoulder with a tilted closed gripper."""
    SX = 0.0388
    ang = math.atan2(cy, cx - SX)
    r_c = math.hypot(cx - SX, cy)
    r0 = min(r_c + 0.05, 0.43)
    pitch = lambda r: float(np.clip((r - 0.26) / (0.43 - 0.26) * 68, 0, 70))
    xy = lambda r: (SX + r * math.cos(ang), r * math.sin(ang))
    log(f'pull_in from r={r_c:.3f} ang={math.degrees(ang):.1f}')
    arm.gripper(b, 2)
    x, y = xy(r0)
    arm.goto(b, x, y, 0.07, pitch=pitch(r0), speed=40, correct=2)
    arm.goto(b, x, y, -0.005, pitch=pitch(r0), speed=20, correct=3)
    for r in np.arange(r0, r_end - 1e-6, -0.01):
        x, y = xy(r); arm.goto(b, x, y, -0.012, pitch=pitch(r), speed=25, correct=1)
    arm.goto(b, x, y, 0.09, speed=30, correct=0)


DIP_Y = 380.0            # held-cube centre row in the wrist image
DIP_X = (680.0, 720.0)   # held cube centre column ~665; keep the cube against the fixed jaw
DIP_LEFT, DIP_TOP = 530.0, 100.0   # grasp-ready: cube top-left corner at the fixed-jaw tip (wrist px)
DIP_SCALE = 2.5          # wrist Jacobian scale at grasp height vs hover


def dip_cube(img):
    import cv2
    import vision
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    m = ((g < 70) & ~vision.jaw_mask('jaw_mask85.png')).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    c = [i for i in range(1, n) if st[i][4] > 15000]
    if not c:
        return None
    i = max(c, key=lambda k: st[k][4])
    ys, xs = np.nonzero(lab == i)
    # top-left edges of the cube: when grasp-ready, the cube's top-left corner sits at the fixed-jaw tip
    return float(np.percentile(xs, 5)), float(np.percentile(ys, 5)), int(st[i][4])


def dip_servo(b, x, y, roll, iters=6, tol=25.0, log=print):
    """Dip to grasp height, check the cube sits between the jaws in the wrist view,
    otherwise rise, shift and dip again. Ends at grasp height."""
    import servo
    from camd_client import read_frame
    Jb, prev = None, None
    for i in range(iters):
        arm.goto(b, x, y, GRASP_Z, roll=roll, speed=22, correct=3)
        time.sleep(0.25)
        img, _ = read_frame('wrist')
        d = dip_cube(img)
        if d is None:
            log(f'dip {i}: cube not in wrist view'); return x, y
        cx, cy, area = d
        e = np.array([DIP_LEFT - cx, DIP_TOP - cy])
        log(f'dip {i}: cube left/top=({cx:.0f},{cy:.0f}) area={area} err={np.linalg.norm(e):.0f}')
        if np.linalg.norm(e) < tol:
            return x, y
        p_now = np.array([cx, cy]); t_now = arm.tool_xyz(b)[:2]
        if Jb is None:
            Jb = DIP_SCALE * servo.J_TOOL @ servo._tool_A(arm.joints(b))
        elif prev is not None:
            dp, dt = p_now - prev[0], t_now - prev[1]
            if np.linalg.norm(dt) > 0.002:
                Jb = Jb + np.outer(dp - Jb @ dt, dt) / (dt @ dt)   # Broyden update
        prev = (p_now, t_now)
        dxy = np.clip(np.linalg.solve(Jb, e), -0.025, 0.025)
        arm.goto(b, x, y, GRASP_Z + 0.035, roll=roll, speed=30, correct=0)
        x, y = x + dxy[0], y + dxy[1]
        arm.goto(b, x, y, GRASP_Z + 0.035, roll=roll, speed=30, correct=1)
    arm.goto(b, x, y, GRASP_Z, roll=roll, speed=22, correct=3)
    return x, y


def sweep_arc(b, r, a0, a1, log=print, z=-0.012):
    """Sweep the closed gripper along an arc of radius r from angle a0 to a1 (deg), pushing the cube round."""
    SX = 0.0388
    xy = lambda a: (SX + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
    log(f'sweep r={r:.3f} {a0:.0f}->{a1:.0f}')
    arm.gripper(b, 2)
    x, y = xy(a0)
    arm.goto(b, x, y, 0.07, speed=40, correct=1)
    arm.goto(b, x, y, z, speed=20, correct=2)
    step = -4.0 if a1 < a0 else 4.0
    for a in np.arange(a0, a1 + step / 2, step):
        x, y = xy(a); arm.goto(b, x, y, z, speed=25, correct=0)
    arm.goto(b, x, y, 0.08, speed=30, correct=0)


def push_line(b, p0, p1, z=-0.012, step=0.01, log=print):
    """Push along a straight line with the closed gripper at table height."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    log(f'push {p0.round(3).tolist()} -> {p1.round(3).tolist()}')
    arm.gripper(b, 2)
    arm.goto(b, *p0, 0.05, speed=40, correct=1)
    arm.goto(b, *p0, z, speed=20, correct=2)
    n = max(1, int(np.linalg.norm(p1 - p0) / step))
    for k in range(1, n + 1):
        p = p0 + (p1 - p0) * k / n
        arm.goto(b, *p, z, speed=25, correct=0)
    arm.goto(b, *p1, 0.07, speed=30, correct=0)


PRE_Z = 0.021
PRE_CFG = HERE / 'pre_target.json'


def _pre():
    import json
    d = {'target': [627.0, 526.0]}
    if PRE_CFG.exists():
        d.update(json.loads(PRE_CFG.read_text()))
    J = np.array(json.loads((HERE / 'jpre.json').read_text())['J_tool_pre'])
    return np.array(d['target']), J


def _save_pre(T):
    import json
    PRE_CFG.write_text(json.dumps({'target': [float(T[0]), float(T[1])]}))


def pre_servo(b, x, y, roll, iters=6, tol=18.0, log=print):
    """Servo the cube to the learned pre-grasp pixel at PRE_Z, descend, check against
    the held-cube reference and learn the pre-grasp target from the residual."""
    import servo
    from camd_client import read_frame
    T, Jt = _pre()
    for i in range(iters):
        arm.goto(b, x, y, PRE_Z, roll=roll, speed=30, correct=3)
        time.sleep(0.25)
        img, _ = read_frame('wrist')
        d = dip_cube(img)
        if d is None:
            log(f'pre {i}: cube not in wrist view'); break
        e = T - np.array(d[:2])
        Jb = Jt @ servo._tool_A(arm.joints(b))
        log(f'pre {i}: px=({d[0]:.0f},{d[1]:.0f}) err={np.linalg.norm(e):.0f}')
        if np.linalg.norm(e) < tol:
            break
        dxy = np.clip(0.9 * np.linalg.solve(Jb, e), -0.02, 0.02)
        x, y = x + dxy[0], y + dxy[1]
    # descend and check against the held-cube reference
    arm.goto(b, x, y, GRASP_Z, roll=roll, speed=22, correct=3)
    time.sleep(0.25)
    img, _ = read_frame('wrist')
    d = dip_cube(img)
    if d is not None:
        cx, cy, _ = d
        ex = (DIP_X[0] - cx) if cx < DIP_X[0] else (DIP_X[1] - cx) if cx > DIP_X[1] else 0.0
        e = np.array([ex, DIP_Y - cy])
        log(f'dip check: px=({cx:.0f},{cy:.0f}) err={np.linalg.norm(e):.0f}')
        if np.linalg.norm(e) > 30:
            Jb = 2.0 * Jt @ servo._tool_A(arm.joints(b))
            dxy = np.clip(np.linalg.solve(Jb, e), -0.02, 0.02)
            # learn: the pre-grasp target should have been offset by the equivalent pre-height shift
            T_new = T - (Jt @ servo._tool_A(arm.joints(b))) @ dxy
            _save_pre(0.5 * T + 0.5 * T_new)
            log(f'dip correction {dxy.round(4).tolist()}; pre target -> {(0.5*T+0.5*T_new).round(0).tolist()}')
            arm.goto(b, x, y, PRE_Z, roll=roll, speed=30, correct=0)
            x, y = x + dxy[0], y + dxy[1]
            arm.goto(b, x, y, PRE_Z, roll=roll, speed=30, correct=1)
            arm.goto(b, x, y, GRASP_Z, roll=roll, speed=22, correct=3)
    return x, y
