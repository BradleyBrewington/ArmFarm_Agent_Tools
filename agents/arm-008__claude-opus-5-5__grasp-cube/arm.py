"""Motion helpers for grasp-cube: connect, move joints, Cartesian top-down moves, gripper, home."""
import json
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import calibrate_workspace as cw  # noqa: E402
import kin  # noqa: E402

HOME = cw.load_home(HERE.parent / "home_pose.json")
GRIP_OPEN = 60.0
GRIP_CLOSED = 0.0


@contextmanager
def bus():
    with cw.connected_bus(os.environ["ARMFARM_SERIAL_PORT"]) as b:
        cw.enable_at_current_position(b, cw.JOINTS)
        yield b


def joints(b):
    return b.sync_read("Present_Position", list(cw.JOINTS))


def move(b, target, seconds=None, speed=120.0, min_s=0.25):
    """Smooth joint move; duration from largest joint change at `speed` deg/s."""
    cur = joints(b)
    if seconds is None:
        d = max(abs(target[j] - cur[j]) for j in target if j != "gripper") if any(j != "gripper" for j in target) else 0
        seconds = max(min_s, d / speed)
    cw.move(b, target, seconds)


def wait(b, target, tol=2.0, timeout=1.5):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        cur = joints(b)
        if all(abs(cur[j] - target[j]) < tol for j in target if j != "gripper"):
            return cur
        time.sleep(0.02)
    return joints(b)


def pose(x, y, z, roll=0.0, tilt=0.0):
    J, err, ang = kin.ik_down(x, y, z, roll=roll, tilt=tilt)
    if err > 0.004:
        raise ValueError(f"unreachable ({x:.3f},{y:.3f},{z:.3f}) err={err*1000:.1f}mm")
    return J


SAG = {}  # cached joint-space correction near recent targets: key -> (cmd - measured)


def goto(b, x, y, z, roll=0.0, speed=120.0, tilt=0.0, settle=True, correct=1):
    """Move tip to (x,y,z); `correct` closed-loop passes remove gravity sag measured by FK."""
    want = np.array([x, y, z])
    J = pose(x, y, z, roll, tilt)
    move(b, J, speed=speed)
    if not settle:
        return J
    cur = wait(b, J)
    aim = want.copy()
    for _ in range(correct):
        time.sleep(0.15)
        cur = joints(b)
        got = kin.fk_T(cur)[:3, 3]
        err = want - got
        if np.linalg.norm(err) < 0.0025:
            break
        aim = aim + err
        J = pose(*aim, roll, tilt)
        move(b, J, speed=speed, min_s=0.15)
        cur = wait(b, J, tol=1.0, timeout=0.6)
    return J


def tip(b):
    return kin.fk_T(joints(b))[:3, 3]


def gripper(b, value, seconds=0.35):
    cw.move(b, {"gripper": value}, seconds)


def home(b, speed=120.0):
    move(b, HOME, speed=speed)
    return wait(b, HOME, tol=4.0, timeout=2.0)


if __name__ == "__main__":
    with bus() as b:
        print(json.dumps(joints(b)))
