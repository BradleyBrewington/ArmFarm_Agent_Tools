"""Bounded joint motion with fresh cameras and feedback checks."""
import argparse
import json
import time

from calibrate_workspace import connected_bus, preflight, validate_target
from camd_client import read_jpeg
from recording import serial_port


def move_checked(target, duration):
    for role in ('top', 'wrist'):
        read_jpeg(role)
    with connected_bus(serial_port()) as bus:
        validate_target(target, bus.calibration)
        start = preflight(bus, [target])
        duration = max(duration, max(abs(target[j]-start[j]) for j in target)/8.)
        torque = bus.sync_read('Torque_Enable', list(target), normalize=False)
        if not all(torque.values()):
            raise RuntimeError('Requested motor has torque disabled; motion not started')
        started = time.monotonic()
        try:
            while True:
                fraction = min(1., (time.monotonic()-started)/duration)
                alpha = fraction*fraction*(3-2*fraction)
                command = {j:start[j]+alpha*(target[j]-start[j]) for j in target}
                for role in ('top', 'wrist'):
                    read_jpeg(role)
                measured = bus.sync_read('Present_Position', list(target))
                if any(abs(measured[j]-command[j])>12 for j in target if j!='gripper'):
                    raise RuntimeError('Tracking error exceeded 12 degrees')
                if any(bus.read('Status', j, normalize=False) for j in target):
                    raise RuntimeError('Motor reported a fault')
                bus.sync_write('Goal_Position', command)
                if fraction == 1:
                    break
                time.sleep(.05)
            time.sleep(.5)
            measured = bus.sync_read('Present_Position', list(target))
            print(json.dumps({'target':target,'measured':measured,'errors':{j:measured[j]-target[j] for j in target}}))
        except BaseException:
            measured = bus.sync_read('Present_Position', list(target), normalize=False)
            bus.sync_write('Goal_Position', measured, normalize=False)
            raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('target', type=json.loads)
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    move_checked(args.target, args.seconds)
