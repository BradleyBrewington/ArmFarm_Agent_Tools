"""Read-only consistency check of saved camera/arm poses; never commands motors."""
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from fk import URDF


def arm_transform(joints):
    tree = ET.fromstring(URDF)
    by_child = {j.find('child').get('link'): j for j in tree.findall('joint')}
    chain, link = [], 'gripper_frame_link'
    while link != 'base_link':
        joint = by_child[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find('origin')
        fixed = np.eye(4)
        fixed[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        fixed[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        result = result @ fixed
        if joint.get('type') != 'fixed':
            moving = np.eye(4)
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            moving[:3, :3] = Rotation.from_rotvec(axis * np.deg2rad(joints[joint.get('name')])).as_matrix()
            result = result @ moving
    return result


def unpack(v):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    t[:3, 3] = v[3:]
    return t


def pack(t):
    return np.r_[Rotation.from_matrix(t[:3, :3]).as_rotvec(), t[:3, 3]]


def main():
    root = Path(__file__).resolve().parents[1]
    source = root / 'calibration/20260921T152058_891355289'
    profile = json.loads((source / 'intrinsics.json').read_text())['cameras']['top']
    camera = np.array(profile['camera_matrix'])
    distortion = np.array(profile['dist_coeffs'])
    obj = np.zeros((63, 3))
    obj[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2) * .02
    arm, observations = [], []
    for index in (1, 2, 3):
        data = json.loads((source / f'captures/pose_{index}.json').read_text())['cameras']['top']
        arm.append(arm_transform(data['joints']))
        options = []
        for reverse in (False, True):
            corners = np.asarray(data['corners'], dtype=float).reshape(-1, 2)
            if reverse:
                corners = corners[::-1].copy()
            ok, r, t = cv2.solvePnP(obj, corners, camera, distortion)
            if not ok:
                raise RuntimeError('Saved checkerboard pose could not be solved')
            options.append(unpack(np.r_[r.ravel(), t.ravel()]))
        observations.append(options)
    candidates = []
    for flips in itertools.product((0, 1), repeat=2):
        board = [observations[i][f] for i, f in enumerate((0, *flips))]
        inverse = [np.linalg.inv(t) for t in arm]
        r, t = cv2.calibrateHandEye([a[:3, :3] for a in inverse], [a[:3, 3] for a in inverse],
                                   [b[:3, :3] for b in board], [b[:3, 3] for b in board], method=cv2.CALIB_HAND_EYE_PARK)
        if not np.isfinite(r).all() or np.linalg.det(r) < 0:
            continue
        base_camera = np.eye(4)
        base_camera[:3, :3], base_camera[:3, 3] = r, t.ravel()
        camera_base = np.linalg.inv(base_camera)
        gripper_board = np.linalg.inv(arm[0]) @ base_camera @ board[0]

        def residual(v):
            cb, gb = unpack(v[:6]), unpack(v[6:])
            parts = []
            for a, b in zip(arm, board):
                error = np.linalg.inv(b) @ cb @ a @ gb
                parts.extend(error[:3, 3])
                parts.extend(Rotation.from_matrix(error[:3, :3]).as_rotvec() * .1)
            return np.asarray(parts)

        fit = least_squares(residual, np.r_[pack(camera_base), pack(gripper_board)], max_nfev=2000)
        errors = residual(fit.x).reshape(3, 6)
        candidates.append({'corner_reversals': [0, *flips], 'cost': float(fit.cost),
                           'camera_from_base': unpack(fit.x[:6]).tolist(),
                           'gripper_from_board': unpack(fit.x[6:]).tolist(),
                           'translation_residual_mm': (np.linalg.norm(errors[:, :3], axis=1)*1000).tolist(),
                           'rotation_residual_degrees': np.rad2deg(np.linalg.norm(errors[:, 3:], axis=1)/.1).tolist()})
    report = {'source': str(source), 'motor_writes': False, 'poses': 3,
              'independent_validation': False, 'candidates': sorted(candidates, key=lambda x:x['cost']),
              'note': 'Diagnostic only. A fit to three saved views does not establish validated robot alignment.'}
    output = root / 'calibration_checks/saved_alignment_consistency.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
