"""Episode loop: pick cube -> home -> confirm -> stop episode -> place at new spot -> home."""
import sys, json, math, time, random, argparse
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'tools'))
import numpy as np, cv2
import arm, grasp, vision, recording
from camd_client import read_frame

SX = 0.0388
LOG = HERE / 'episodes.jsonl'
TASK = 'pick up the black cube and bring it to home'
# Placement grid covering the reachable top-down zone
# -y side (image left) is shared with the neighbouring arm: keep clear of it
GRID = [(a, r) for r in (0.155, 0.18) for a in (-35, -18, 0, 17)]
# Outer tier pulled to 0.195 (was 0.205): a place at 0.205 + ~2cm error landed cubes at r~0.22, in
# the dead zone (REACH<r<=~0.25) where pull_in can't recover them and the batch stalls (run44, run47,
# 2026-09-24). 0.195 keeps outer coverage but lands inside REACH even with placement error.
GRID += [(a, 0.195) for a in (-30, -18, 0, 17)]
# high +y headings: grasps miss beyond r~0.2 (failures at ang>=42), and a miss shoves the cube out to
# a far high-angle dead-zone spot that is very hard to recover, so keep these well inside.
GRID += [(a, r) for r in (0.14, 0.165, 0.18) for a in (34, 50)]
CORNER_ANG, CORNER_R = 38.0, 0.195   # cubes out here are pulled in before an episode
MAX_ANG = 55.0   # cubes beyond this heading are swept back round (grasps unreliable there)
MIN_R = 0.115    # cubes closer than this are pushed out before an episode
MIN_PX_X = 430   # top-image x; cubes left of this belong to the neighbour's area
REACH = 0.215


class Interrupted(BaseException):
    pass


def _on_term(signum, frame):
    import signal
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)   # let the cleanup finish
    raise Interrupted(f'signal {signum}')


def log(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


CLEAR_POSE = (0.10, 0.20, 0.20)   # arm parked high on the +y side, clear of the view


def find_cube(b=None):
    """Detect the cube from home; if the arm hides it, look again from a clear pose."""
    found = _find_cube()
    if found is None and b is not None:
        arm.goto(b, *CLEAR_POSE, speed=45, correct=0)
        time.sleep(0.4)
        found = _find_cube()
        arm.home(b)
    return found


def _find_cube():
    for _ in range(3):
        t, _ = read_frame('top')
        c = [d for d in vision.top_cube(t) if d['cx'] > MIN_PX_X]
        if c:
            c = c[0]
            m = vision.pix_to_base([(c['cx'], c['cy'])])[0]
            return float(m[0]), float(m[1]), vision.top_cube_yaw(t, c), c
        time.sleep(0.3)
    return None


def held_in_wrist():
    """Held cube shows as a large dark blob centred between the jaws."""
    img, _ = read_frame('wrist')
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    roi = g[150:370, 450:830]      # region the held cube fills between the jaws
    frac = float((roi < 60).mean())
    return frac > 0.6, frac, img


def lift_clear(b):
    p = arm.tool_xyz(b)
    if p[2] < 0.07:
        arm.goto(b, p[0], p[1], 0.09, speed=30, correct=0)


def next_target(done_counts, cube_xy):
    opts = []
    for a, r in GRID:
        x, y = SX + r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        if math.hypot(x - cube_xy[0], y - cube_xy[1]) < 0.06:
            continue
        opts.append((done_counts.get((a, r), 0), random.random(), a, r, x, y))
    opts.sort()
    return opts[0][2:]


def reachable_prep(b):
    """Make sure the cube is visible and within vertical grasp reach (reset action, outside episodes)."""
    for _ in range(4):
        found = find_cube(b)
        if found is None:
            return None
        cx, cy = found[0], found[1]
        r = math.hypot(cx - SX, cy)
        if r < MIN_R:
            # push outward along a line tilted +-40 deg off radial, starting 3.5 cm behind the cube
            ang = math.atan2(cy, cx - SX)
            for tilt in (-40, 40, -60, 60):
                a = ang + math.radians(tilt)
                d = np.array([math.cos(a), math.sin(a)])
                p0 = np.array([cx, cy]) - 0.045 * d
                if math.hypot(p0[0] - SX, p0[1]) >= 0.085:
                    break
            grasp.push_line(b, p0, np.array([cx, cy]) + 0.07 * d, log=log)
            arm.home(b); time.sleep(0.4)
            continue
        ang_d = math.degrees(math.atan2(cy, cx - SX))
        if ang_d > MAX_ANG and r <= REACH + 0.02:
            grasp.sweep_arc(b, r, ang_d + 30, ang_d - 35, log=log)
            arm.home(b); time.sleep(0.4)
            continue
        if ang_d > CORNER_ANG and r > CORNER_R:
            grasp.pull_in(b, cx, cy, r_end=0.16, log=log)
            arm.home(b); time.sleep(0.4)
            continue
        if r <= REACH:
            return found
        grasp.pull_in(b, cx, cy, log=log)
        arm.home(b); time.sleep(0.4)
    found = find_cube(b)
    if found and MIN_R <= math.hypot(found[0] - SX, found[1]) <= REACH:
        # the pull/sweep loop ran out: attempt anyway rather than stall the whole run
        return found
    return None   # could not make the cube graspable: stop rather than record a doomed episode


def record_cal(target):
    f = vision.CUBE_CAL
    found = find_cube()
    if found is None:
        return
    pts = json.loads(f.read_text()) if f.exists() else []
    c = found[3]
    pts.append(dict(target=list(map(float, target)), px=[c['cx'], c['cy']]))
    f.write_text(json.dumps(pts, indent=1))


def episode(b, n, stats, done_counts):
    found = reachable_prep(b)
    if found is None:
        log('cube not visible from home; skipping episode start')
        return None
    cx, cy, yaw, c = found
    log(f'episode {n}: cube at ({cx:.3f},{cy:.3f}) yaw={yaw:.0f}')
    t0 = time.time()
    rec = recording.start_recording(TASK)
    notes = []
    g = None
    ok = False
    try:
        for attempt in range(3):
            g = grasp.grasp(b, cx, cy, yaw, log=log)
            notes.append(f"attempt {attempt+1}: grip={g['held']:.1f}")
            if g['ok']:
                break
            lift_clear(b); arm.home(b); time.sleep(0.4)
            found = reachable_prep(b)
            if found is None:
                notes.append('cube lost'); break
            cx, cy, yaw, c = found
        if g and g['ok']:
            arm.home(b); time.sleep(0.5)
            grip = arm.joints(b)['gripper']
            vis, frac, img = held_in_wrist()
            ok = grip > grasp.EMPTY_BELOW and vis
            notes.append(f'at home gripper={grip:.1f} wrist_dark={frac:.2f}')
            cv2.imwrite(str(HERE / 'last_confirm.jpg'), img)
        else:
            lift_clear(b); arm.home(b)
    except Interrupted as e:
        notes.append(f'interrupted by {e}; episode aborted')
        recording.stop_recording(False, '; '.join(notes))
        log(f'episode {n}: interrupted, recording closed as failure')
        raise
    except Exception as e:
        notes.append(f'error: {e!r}')
        log(f'error during episode: {e!r}')
    res = recording.stop_recording(ok, '; '.join(notes))
    dt = time.time() - t0
    stats['succ' if ok else 'fail'] += 1
    rec_line = dict(n=n, t=time.time(), success=ok, cube=[cx, cy], duration=dt, notes=notes,
                    folder=res.get('folder') if isinstance(res, dict) else None)
    with open(LOG, 'a') as f:
        f.write(json.dumps(rec_line) + '\n')
    log(f'episode {n}: success={ok} ({dt:.0f}s) {notes}')
    # reset: place at a new position (also when a held cube failed confirmation)
    if g and (ok or arm.joints(b)['gripper'] > grasp.EMPTY_BELOW):
        a, r, tx, ty = next_target(done_counts, (cx, cy))
        done_counts[(a, r)] = done_counts.get((a, r), 0) + 1
        grasp.place(b, tx, ty, g['roll'], log=log)
        arm.home(b); time.sleep(0.4)
        record_cal((tx, ty))
    return ok


def main():
    import signal
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGHUP, _on_term)
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--budget', type=float, default=1e9, help='seconds; no new episode starts after budget-100s')
    a = ap.parse_args(); t_start = time.time()
    stats = {'succ': 0, 'fail': 0}
    done_counts = {}
    with arm.bus() as b:
        lift_clear(b); arm.home(b); time.sleep(0.4)
        streak = 0
        for i in range(a.n):
            if time.time() - t_start > a.budget - 100:
                log('time budget reached: stopping between episodes'); break
            r = episode(b, i, stats, done_counts)
            if r is None:
                break
            streak = 0 if r else streak + 1
            log(f'stats {stats}')
            if streak >= 3:
                log('3 consecutive failures: stopping for inspection'); break


if __name__ == '__main__':
    main()
