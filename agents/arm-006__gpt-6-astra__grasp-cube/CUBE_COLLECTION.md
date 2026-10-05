# Cube collection on arm-006

Use `/opt/armfarm/venv/bin/python` from the workspace. Read `TOOLS.md` first.

`collect_cube.py` runs a finite list of `[base_link_x_metres, shoulder_pan_degrees]` targets. It assumes the black cube is already held at home and wrist roll is approximately zero. It places the held cube, opens the gripper, returns the four constrained joints home, starts an episode, picks up the cube using `AdaptiveGrip`, returns home, verifies the grasp, and stops the episode. It finishes holding the cube at home. Placement and reset motions are outside episodes.

The visual check is specific to the current wrist camera, cube, and home pose: three advancing fresh frames must show a dark cube in the observed space between the jaws. Empty-home checks require that region to clear. The held check also requires nonzero gripper opening, sustained motor load, no fault, and home errors below four degrees. `empty_home_reference.jpg` and `held_home_reference.jpg` are observed reference images. Review evidence images during collection; this is not a general-purpose object detector.

Motion targets are solved against the supplied FK model, validated against live motor calibration, and monitored for physical stops. Pose generation alone does not certify collision-free travel. The tested tabletop approach uses z=0.060 m, release z=0.015 m, pickup z=0.005 m, and nominal downward gripper orientation. Calibration still lacks a measured table-to-robot alignment. Placement can shift the cube; a missed grasp stops collection for fresh visual alignment. Do not blindly retry a failed target.

The supplied adaptive gripper controller applies a volatile torque limit and monitors load, current, temperature, and faults. No protection registers or motor calibration are rewritten. Home commands include only the four joints in `home_pose.json`.

`cube_cycle.py` provides individually supervised place/pick operations. `task_motion.py` is a small supervised joint-motion and snapshot helper. `grasp_home.py` is the initial supervised grasp/carry helper. They do not independently certify task success.

`coverage_plan.json` contains the nominal grid; `collection_log.jsonl` contains completed automated cycles and recorder results. `initial_results.json` records manually verified cycles and unsuccessful episodes before/around automated collection. `collection_status.py` calculates success count, clean recording count, and rates including setup time. Successful motion and clean recording quality remain separate. Intermittent recorder `camera_error` flags are preserved.

No background service is installed. Never start a second motion client while a collection process is running. On any failure, inspect the current cameras and motor state before recovery. The collection loop stops on its first failure and marks an active episode unsuccessful.

Collection now includes a10-second gripper torque-off rest after placement and home verification, before episode start. Temperature/fault checks remain active during rest. The operator-authorized software temperature guard is65C; hardware protection remains70C. Recurrent stops at65–66C remain unresolved.
