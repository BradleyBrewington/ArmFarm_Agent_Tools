"""pick(b, x, y): servo over the cube, descend, grasp, lift. Returns info dict."""
import sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'tools'))
import numpy as np
import arm, servo
from camd_client import read_jpeg

OPEN = 70.0
GRASP_Z = 0.014
EMPTY_BELOW = 4.0   # gripper reading when closed on nothing


def pick(b, x, y, log=print, save=None):
    arm.gripper(b, OPEN)
    r = servo.servo(b, x, y, log=log, tol_px=40)
    if r is None:
        return {'ok': False, 'reason': 'cube not visible'}
    x, y, roll, d = r
    r = servo.servo_low(b, x, y, roll, log=log)
    if r is None:
        return {'ok': False, 'reason': 'cube lost at low height'}
    x, y, _ = r
    arm.goto(b, x, y, GRASP_Z, roll=roll, speed=20, correct=3)
    time.sleep(0.2)
    if save: open(f'{save}_low.jpg', 'wb').write(read_jpeg('wrist')[0])
    for g in (45, 25, 12, 5, 0):
        pos = arm.gripper(b, g, 0.3)
        if pos > g + 6:
            break
    time.sleep(0.3)
    grip = arm.joints(b)['gripper']
    if grip > EMPTY_BELOW:
        arm.gripper(b, max(0.0, grip - 6), 0.2)
    arm.goto(b, x, y, 0.10, roll=roll, speed=30, correct=1)
    time.sleep(0.3)
    held = arm.joints(b)['gripper']
    if save: open(f'{save}_lift.jpg', 'wb').write(read_jpeg('wrist')[0])
    return {'ok': held > EMPTY_BELOW, 'grip_closed': grip, 'grip_lifted': held, 'x': x, 'y': y, 'roll': roll}
