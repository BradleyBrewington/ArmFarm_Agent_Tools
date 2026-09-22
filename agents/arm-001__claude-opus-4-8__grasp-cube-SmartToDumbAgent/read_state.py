"""Read-only pose + camera snapshot. Does not touch torque or Goal_Position."""
import json
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import recording, camd_client, fk
from episode_motion import JOINTS

b = FeetechMotorsBus(port=recording.serial_port(), motors={
    j: Motor(i + 1, 'sts3215', MotorNormMode.RANGE_0_100 if j == 'gripper' else MotorNormMode.DEGREES)
    for i, j in enumerate(JOINTS)})
b.connect()
try:
    b.calibration = b.read_calibration()
    torque = b.sync_read('Torque_Enable', normalize=False)
    q = b.sync_read('Present_Position')
    out = {
        'torque_enable': torque,
        'q': q,
        'goal': b.sync_read('Goal_Position'),
        'xyz': fk.forward(q),
        'load': b.sync_read('Present_Load'),
        'temperature': b.sync_read('Present_Temperature'),
        'camera_alive': camd_client.alive(),
    }
    for role in ('top', 'wrist'):
        jpg, meta = camd_client.read_jpeg(role)
        open(f'/tmp/read_{role}.jpg', 'wb').write(jpg)
    print(json.dumps(out, indent=2))
finally:
    b.disconnect(disable_torque=False)
