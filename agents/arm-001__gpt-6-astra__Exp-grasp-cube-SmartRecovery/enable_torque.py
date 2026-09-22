"""Safely engage torque: pin Goal_Position to the current Present_Position for
every motor BEFORE enabling torque, so the arm holds its current pose instead of
snapping toward whatever stale goal is in the registers."""
import json
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import recording, fk
from episode_motion import JOINTS

b = FeetechMotorsBus(port=recording.serial_port(), motors={
    j: Motor(i + 1, 'sts3215', MotorNormMode.RANGE_0_100 if j == 'gripper' else MotorNormMode.DEGREES)
    for i, j in enumerate(JOINTS)})
b.connect()
try:
    b.calibration = b.read_calibration()
    before = b.sync_read('Torque_Enable', normalize=False)
    present = b.sync_read('Present_Position')
    if all(before.values()):
        print(json.dumps({'note': 'torque already enabled', 'torque': before, 'q': present}, indent=2))
    else:
        # Pin goal to present (no motion) while torque is still off, then engage.
        b.sync_write('Goal_Position', present)
        b.enable_torque()
        after_torque = b.sync_read('Torque_Enable', normalize=False)
        after_q = b.sync_read('Present_Position')
        after_goal = b.sync_read('Goal_Position')
        drift = {j: after_q[j] - present[j] for j in JOINTS}
        print(json.dumps({
            'torque': after_torque,
            'goal': after_goal,
            'q': after_q,
            'xyz': fk.forward(after_q),
            'drift_from_before_enable': drift,
        }, indent=2))
finally:
    b.disconnect(disable_torque=False)
