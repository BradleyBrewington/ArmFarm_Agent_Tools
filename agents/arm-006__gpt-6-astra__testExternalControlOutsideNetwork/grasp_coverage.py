"""Read the station-owned grasp coverage snapshot. No network or motor access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time


def load(scope='current', layer='kinematic', root=None):
    root = Path(root) if root else Path(__file__).resolve().parent.parent/'grasp_coverage'
    manifest = json.loads((root/'latest.json').read_text())
    if manifest.get('status') != 'ready': return manifest
    for env, key in [('ARMFARM_TASK_ID', 'task_id'), ('ARMFARM_TASK_VERSION', 'task_version')]:
        if os.environ.get(env) and str(manifest.get(key)) != os.environ[env]:
            return {'status': 'stale', 'message': 'Task changed; waiting for coverage refresh'}
    motor = os.environ.get('ARMFARM_MOTOR_CALIBRATION')
    if motor and hashlib.sha256(Path(motor).read_bytes()).hexdigest() != manifest['motor_file_sha256']:
        return {'status': 'stale', 'message': 'Motor calibration changed; waiting for a new map'}
    path = (root/manifest['views'][scope+'-'+layer]).resolve()
    if not path.is_relative_to(root.resolve()): raise ValueError('Invalid coverage path')
    summary = json.loads(path.read_text())
    assets = {key: (path.parent/summary[key]).resolve() for key in ('image', 'cells', 'grid')}
    if any(not p.is_relative_to(path.parent) for p in assets.values()): raise ValueError('Invalid coverage asset path')
    image = assets['image']
    if summary['map_version'] != manifest['map_version'] or hashlib.sha256(image.read_bytes()).hexdigest() != summary['image_sha256']:
        raise ValueError('Coverage snapshot mismatch; retry')
    summary.update(**{key: str(p) for key, p in assets.items()},
                   snapshot_age_seconds=round(time.time()-manifest['updated_at'], 1))
    if summary['snapshot_age_seconds'] > 120:
        summary['status'] = 'stale'; summary['next_targets'] = []
    return summary


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scope', choices=['current', 'lifetime'], default='current')
    p.add_argument('--layer', choices=['kinematic', 'task'], default='kinematic')
    p.add_argument('--image', action='store_true', help='Print the PNG path for your image-view tool')
    a = p.parse_args()
    try:
        result = load(a.scope, a.layer)
    except (OSError, ValueError, KeyError) as exc:
        result = {'status': 'unavailable', 'message': str(exc)}
    print(result['image'] if a.image and result.get('status') == 'ready' else json.dumps(result, indent=2))
