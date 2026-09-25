"""Shared station I/O, extracted from arm-003's ACT runtime."""
import json
from pathlib import Path
import socket
import struct
import time

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')


def call(op, timeout=2, **parameters):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(timeout);client.connect('/run/armfarm-recording-armfarm/control.sock')
        client.sendall((json.dumps({'op':op,**parameters})+'\n').encode())
        with client.makefile('rb') as stream:result=json.loads(stream.readline())
    if not result.get('ok'):raise RuntimeError(str(result.get('error','Recorder unavailable')))
    return result


def state(latest):
    out=[]
    for name,raw in zip(JOINTS,latest['state_raw']):
        cal=latest['calibration'][name];lo,hi=cal['range_min'],cal['range_max']
        if not 0<=lo<hi<=4095:raise ValueError('Invalid calibration')
        out.append((raw-(lo+hi)/2)*360/4095 if name!='gripper' else (min(hi,max(lo,raw))-lo)*100/(hi-lo))
    return out


def camera(role):
    data=(Path('/dev/shm/so101_camd')/(role+'.frame')).read_bytes()
    if data[:4]!=b'CMD1':raise RuntimeError('Invalid camera frame')
    n=struct.unpack('<I',data[4:8])[0];meta=json.loads(data[8:8+n])
    if time.monotonic()-meta['t_mono']>.25:raise RuntimeError('Stale camera: '+role)
    return data[8+n:]


def healthy(station, calibration=None, armed=False):
    if json.loads((station/'state/status.json').read_text()).get('phase')!='paused':
        raise RuntimeError('Station agent is no longer paused')
    s=call('status',timeout=.3)
    if s.get('pen_context'):raise RuntimeError('Station is owned by another controller')
    latest=s.get('latest')
    if not latest or time.monotonic()-latest['t_mono']>.2:raise RuntimeError('Stale motor feedback')
    if any(latest['fault_bits']) or any(latest['mode']):raise RuntimeError('Motor fault or incorrect operating mode')
    if calibration is not None and latest['calibration']!=calibration:raise RuntimeError('Calibration changed')
    if armed and latest['torque']!=[1]*6:raise RuntimeError('Motor torque changed')
    for role in ('top','wrist'):camera(role)
    return s,latest
