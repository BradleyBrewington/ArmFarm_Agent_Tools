"""Read-only pixel-to-table query from the Sept 22 stationary checkerboard.

No robot imports, hardware access, calibration writes, or success credit.
Coordinates use the board's first detected inner corner and 20 mm squares.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def build(workspace):
    table_path = workspace / 'calibration_checks/live-table-20260922/table.json'
    table = json.loads(table_path.read_text())
    profile = json.loads((workspace / table['intrinsics_source']).read_text())['cameras']['top']
    camera = np.asarray(profile['camera_matrix'], dtype=float)
    distortion = np.asarray(profile['dist_coeffs'], dtype=float)
    metric = np.mgrid[0:9, 0:7].T.reshape(-1, 2).astype(float) * table['square_m']

    def undistort(points):
        return cv2.undistortPoints(np.asarray(points, dtype=float).reshape(-1, 1, 2),
                                   camera, distortion, P=camera).reshape(-1, 2)

    observed = [undistort(v['corners']) for v in table['views']]
    homography, _ = cv2.findHomography(metric, observed[0], method=0)
    if homography is None:
        raise ValueError('Checkerboard homography failed')
    inverse = np.linalg.inv(homography)
    projected = cv2.perspectiveTransform(metric.reshape(-1, 1, 2), homography).reshape(-1, 2)
    report = dict(source=str(table_path), frame='20260922_live_checkerboard',
                  square_m=table['square_m'], grid_cell_m=0.05,
                  fit_view=0, other_views='Temporal repeat checks of the same board, not full-workspace validation',
                  reprojection_rms_px=[float(np.sqrt(np.mean(np.sum((projected-v)**2, axis=1)))) for v in observed],
                  robot_alignment_used=False, accepted_calibration_modified=False,
                  caution='Table-plane points only; elevated features have parallax. Confirm current camera/table alignment. This does not define the usable workspace or credit any cell.')
    return inverse, undistort, report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path('/var/lib/armfarm/stations/armfarm/workspace'))
    parser.add_argument('--pixel', nargs=2, type=float, action='append', default=[])
    args = parser.parse_args()
    inverse, undistort, report = build(args.workspace)
    report['queries'] = []
    for pixel in args.pixel:
        xy = cv2.perspectiveTransform(undistort([pixel]).reshape(-1, 1, 2), inverse).reshape(2)
        report['queries'].append(dict(raw_pixel=pixel, table_xy_m=xy.tolist(),
                                      cell_candidate=np.floor(xy / .05).astype(int).tolist(),
                                      boundary_distance_m=float(np.min(np.minimum(np.mod(xy, .05), .05-np.mod(xy, .05))))))
    print(json.dumps(report, indent=2))
