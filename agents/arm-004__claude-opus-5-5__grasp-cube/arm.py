"""Motion helpers: joint moves, Cartesian top-down moves, gripper, home."""
import json, math, os, sys, time
from contextlib import contextmanager
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'tools')); sys.path.insert(0, str(HERE))
import calibrate_workspace as cw
import kin

HOME = cw.load_home(HERE.parent / 'home_pose.json')
ARM4 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")


def clear_faults(ids=range(1, 7)):
    """Clear overload latches (status 32) that make the LeRobot handshake fail."""
    import scservo_sdk as scs
    ph = scs.PortHandler(os.environ['ARMFARM_SERIAL_PORT']); ph.openPort(); ph.setBaudRate(1000000)
    pk = scs.PacketHandler(0)
    cleared = []
    try:
        for mid in ids:
            _, c, e = pk.ping(ph, mid)
            if c == 0 and e:
                pos, _, _ = pk.read2ByteTxRx(ph, mid, 56)
                pk.write2ByteTxRx(ph, mid, 42, pos)
                pk.write1ByteTxRx(ph, mid, 40, 0); time.sleep(0.05)
                pk.write1ByteTxRx(ph, mid, 40, 1); time.sleep(0.05)
                cleared.append((mid, e))
    finally:
        ph.closePort()
    if cleared:
        print(f'[faults] cleared {cleared}', flush=True)
    return cleared


@contextmanager
def bus():
    clear_faults()
    with cw.connected_bus(os.environ['ARMFARM_SERIAL_PORT']) as b:
        cw.enable_at_current_position(b)
        yield b


def joints(b):
    return b.sync_read("Present_Position")


def move(b, target, speed=60.0, min_s=0.6, settle=0.3):
    """Joint-space smooth move; duration scaled by largest joint change (deg/s)."""
    cur = joints(b)
    d = max(abs(target[j] - cur[j]) for j in target if j != 'gripper') if any(j != 'gripper' for j in target) else 0
    cw.move(b, target, max(min_s, d / speed))
    time.sleep(settle)
    return joints(b)


def tool_xyz(b):
    T = kin.pose(joints(b)); return T[:3, 3]


def goto(b, x, y, z, roll=None, speed=60.0, pitch=0.0, correct=2, tol=0.003):
    """Top-down Cartesian move with closed-loop correction on measured FK."""
    import numpy as np
    if math.hypot(x - 0.0388, y) < 0.08 and z < 0.12:
        raise ValueError(f"refusing low target near the base: {(x, y, z)}")
    want = np.array([x, y, z]); aim = want.copy()
    for it in range(correct + 1):
        q, err, ang = kin.ik_down(*aim, pitch_deg=pitch)
        if err > 0.004:
            if it == 0:
                raise ValueError(f"IK miss {err*1000:.1f} mm at {tuple(aim)}")
            break
        if roll is not None:
            q['wrist_roll'] = roll
        move(b, q, speed if it == 0 else 30.0, min_s=0.6 if it == 0 else 0.3)
        e = want - tool_xyz(b)
        if np.linalg.norm(e) < tol:
            break
        aim = aim + e
    return joints(b), float(np.linalg.norm(e))


def gripper(b, value, seconds=0.6):
    cw.move(b, {'gripper': value}, seconds)
    time.sleep(0.2)
    return joints(b)['gripper']


def home(b, speed=60.0):
    return move(b, dict(HOME), speed)
