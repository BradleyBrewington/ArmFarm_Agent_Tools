"""Identify the actual implementation, checkpoint and normalization for a run."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import socket

HERE=Path(__file__).resolve().parent


def sha256(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def select_policy(station, path=None, hostname=None):
    catalog=json.loads((HERE/'policies.json').read_text())
    if path is None:
        robot=hostname or socket.gethostname().split('.')[0]
        path=Path(station)/'policies'/catalog['defaults'][robot]/'checkpoint/pretrained_model'
    path=Path(path).resolve()
    digest=sha256(path/'model.safetensors')
    for name,model in catalog['models'].items():
        if model['sha256']==digest:return path,{'name':name,**model}
    raise ValueError('Checkpoint is not registered in benchmark/policies.json: '+digest)


def runtime_version(policy):
    release=HERE/'release.json'
    installed=json.loads(release.read_text()) if release.exists() else {}
    sources={p.name:sha256(p) for p in sorted(HERE.iterdir()) if p.suffix in ('.py','.sh') or p.name=='policies.json'}
    processors={p.name:sha256(p) for p in sorted(Path(policy).glob('policy_*')) if p.is_file()}
    return {'git_commit':installed.get('git_commit'),'bundle_sha256':installed.get('bundle_sha256'),
            'source_sha256':sources,'processor_sha256':processors,'policy_config_sha256':sha256(Path(policy)/'config.json'),
            'versions':{name:importlib.metadata.version(name) for name in ('lerobot','torch','torchvision')},
            'robot':socket.gethostname()}
