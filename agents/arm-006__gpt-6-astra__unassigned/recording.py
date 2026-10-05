"""Episode boundaries and optional independent serial connections. No motion code."""
import argparse
import json
import os
import socket
from pathlib import Path


def request(op, **parameters):
    path = os.environ.get('ARMFARM_RECORDING_SOCKET', '/run/armfarm-recording-'+os.environ.get('USER', 'protopi5')+'/control.sock')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(25); client.connect(path)
        client.sendall((json.dumps({'op': op, **parameters})+'\n').encode())
        with client.makefile('rb') as stream: response = json.loads(stream.readline())
    if not response.pop('ok'): raise RuntimeError(response['error'])
    return response


def start_recording(task=''):
    context = {}
    if os.environ.get('ARMFARM_RUN_DIR'):
        run = Path(os.environ['ARMFARM_RUN_DIR'])
        context = json.loads((run/'run.json').read_text())
        if (run/'session.json').exists():
            context['session_id'] = json.loads((run/'session.json').read_text())['id']
    return request('start_recording', task=task, context=context)


def stop_recording(success, notes):
    return request('stop_recording', success=success, notes=notes)


def serial_port():
    """Optional: separate serial endpoint for each concurrent motor client."""
    return request('port')['port']


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('operation', choices=['start', 'stop', 'status', 'port'])
    p.add_argument('--task', default=''); p.add_argument('--success', choices=['true', 'false'])
    p.add_argument('--notes', default=''); a = p.parse_args()
    if a.operation == 'stop' and a.success is None: p.error('stop requires --success true|false')
    result = (start_recording(a.task) if a.operation == 'start' else
              stop_recording(a.success == 'true', a.notes) if a.operation == 'stop' else request(a.operation))
    print(json.dumps(result, indent=2))
