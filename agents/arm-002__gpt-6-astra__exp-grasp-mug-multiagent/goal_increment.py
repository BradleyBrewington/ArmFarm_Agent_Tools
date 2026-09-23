"""Small shoulder increments from an already-enabled holding goal, with evidence."""
import json
import math
import time
from pathlib import Path
import numpy as np
from calibrate_workspace import JOINTS, connected_bus, clamp_target, preflight
from recording import serial_port
from camd_client import read_jpeg
from check_saved_alignment import arm_transform

ROOT = Path(__file__).resolve().parent
OFFSET = np.array([-.004200149, -.025801957, .003846939, 1])

def increment(delta, duration=4.):
    if not math.isfinite(delta) or not 0 < abs(delta) <= 2:
        raise ValueError('Shoulder increment must be nonzero and at most 2 degrees')
    if not math.isfinite(duration) or duration < 2:
        raise ValueError('Duration must be at least 2 seconds')
    joint = 'shoulder_lift'
    stamp = str(time.time_ns())
    prefix = ROOT / ('goal_increment_' + stamp)
    report = {'timestamp': time.time(), 'requested_increment_deg': delta, 'success': False,
              'note': 'Motion diagnostic only; not a grasp episode or calibrated fingertip measurement.'}
    def frames(label):
        for role in ('top', 'wrist'):
            data, _ = read_jpeg(role)
            Path(str(prefix) + '_' + label + '_' + role + '.jpg').write_bytes(data)
    def state(bus):
        return {'joints': bus.sync_read('Present_Position', list(JOINTS)),
                'goals': bus.sync_read('Goal_Position', list(JOINTS)),
                'raw_goals': bus.sync_read('Goal_Position', list(JOINTS), normalize=False),
                'torque': bus.sync_read('Torque_Enable', list(JOINTS), normalize=False),
                'faults': {j: bus.read('Status', j, normalize=False) for j in JOINTS}}
    frames('before')
    with connected_bus(serial_port()) as bus:
        before = state(bus)
        report['before'] = before
        # This separate path rejects disabled torque. Existing initialization remains available.
        if not all(before['torque'].values()) or any(before['faults'].values()):
            raise RuntimeError('Requires enabled torque and no faults before any write')
        old = before['goals'][joint]
        target = clamp_target({joint: old + delta}, bus.calibration)
        preflight(bus, [target])
        if abs(target[joint] - before['joints'][joint]) > 8:
            raise RuntimeError('Target exceeds 8-degree tracking bound')
        xyz0 = (arm_transform(before['joints']) @ OFFSET)[:3]
        predicted = dict(before['joints']); predicted[joint] += target[joint] - old
        if np.linalg.norm((arm_transform(predicted) @ OFFSET)[:3] - xyz0) > .02:
            raise RuntimeError('Predicted diagnostic displacement exceeds 2 cm')
        samples = []
        def check(command):
            for role in ('top', 'wrist'):
                read_jpeg(role)
            q = bus.sync_read('Present_Position', list(JOINTS))
            if not all(bus.sync_read('Torque_Enable', list(JOINTS), normalize=False).values()):
                raise RuntimeError('Torque lost')
            if any(bus.read('Status', j, normalize=False) for j in JOINTS):
                raise RuntimeError('Motor fault')
            if abs(q[joint] - command) > 8:
                raise RuntimeError('Tracking exceeds 8 degrees')
            displacement = (arm_transform(q) @ OFFSET)[:3] - xyz0
            if abs(q[joint] - before['joints'][joint]) > 3 or np.linalg.norm(displacement) > .02:
                raise RuntimeError('Measured motion exceeds 3 degrees or 2 cm')
            samples.append({'time':time.time(), 'command':command, 'measured':q[joint], 'delta_xyz':displacement.tolist()})
        try:
            # First write is exactly the old goal, never the measured position.
            check(old)
            bus.write('Goal_Position', joint, old)
            started = time.monotonic()
            while True:
                f = min(1., (time.monotonic() - started) / duration)
                alpha = f*f*(3-2*f)
                command = old + (target[joint]-old)*alpha
                check(command)
                bus.write('Goal_Position', joint, command)
                if f == 1:
                    break
                time.sleep(.05)
            for _ in range(5):
                time.sleep(.1)
                check(target[joint])
            after = state(bus)
            report['after'] = after
            report['other_raw_goals_unchanged'] = all(after['raw_goals'][j] == before['raw_goals'][j] for j in JOINTS if j != joint)
            if not report['other_raw_goals_unchanged']:
                raise RuntimeError('Unrequested goals changed')
            report['measured_delta_deg'] = after['joints'][joint] - before['joints'][joint]
            report['delta_xyz_m'] = ((arm_transform(after['joints']) @ OFFSET)[:3] - xyz0).tolist()
            report['success'] = True
        except BaseException as exc:
            report['error'] = repr(exc)
            raw = bus.read('Present_Position', joint, normalize=False)
            bus.write('Goal_Position', joint, raw, normalize=False)
            report['recovery'] = 'Held requested joint at measured raw position; no other goals written.'
            raise
        finally:
            report['samples'] = samples
            prefix.with_suffix('.json').write_text(json.dumps(report, indent=2))
    frames('after')
    print(json.dumps({'report': str(prefix.with_suffix('.json')), **{k:report[k] for k in ('success','measured_delta_deg','delta_xyz_m','other_raw_goals_unchanged')}}))
    return report

if __name__ == '__main__':
    import sys
    increment(float(sys.argv[1]))
