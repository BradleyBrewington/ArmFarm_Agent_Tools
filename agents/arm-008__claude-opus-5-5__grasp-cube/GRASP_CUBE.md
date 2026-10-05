# Grasp-cube episode tooling (arm-008)

Run: `/opt/armfarm/venv/bin/python tools/run_episodes.py N [--minutes M]`
(look -> record -> pick -> home -> confirm -> stop -> place at a new coverage cell -> repeat).
Logs: `episodes_log.jsonl`, coverage per 4 cm cell: `coverage.json`, console: `run_log.txt`.

| File | Role |
|---|---|
| `arm.py` | Bus connection (exported calibration file, fault-tolerant connect, retrying sync I/O), gripper torque cap, moves via `calibrate_workspace.move`. |
| `kin.py` | Full FK (`fk_T`) and IK with a fixed approach tilt (`ik_down`). |
| `task.py` | `grasp_plan` (FK-solved wrist roll + jaw offset), `pick` (wrist-camera servo), `place`, `pick_robust`, `push`. |
| `vision.py` | Top-camera cube detection (arm mask at the look pose) and pixel -> base_link maps. |
| `wrist.py` | Wrist-camera cube detection with a fixed-jaw mask. |
| `calib_tip.py` | Fits `top_plane_maps.json` by touching the table with the fingertip (`python calib_tip.py 0.004` / `0.02`). |
| `wrist_servo.json` | Wrist-image Jacobian and target pixel for the pre-grasp hover. |

Notes
- The saved `workspace_map.json` homography is not the table plane (board was held in the air);
  only the camera intrinsics from it are used.
- Gripper `Torque_Limit` is set to 230 (RAM) so holding the cube never trips overload (25%).
- Wrist roll is not a world yaw (closing axis ~ 87 + roll - pan deg); always plan with FK.
- Grasp is confirmed at home by the gripper staying open on the cube and no cube on the table.
