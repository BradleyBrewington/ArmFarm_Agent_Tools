# Cube run, 2026-10-05

Read AGENTS.md and TOOLS.md first. This run saved 17 successful episodes and 9 failures. Successful pickups sampled 16 distinct commanded locations including the manual initial pickup; full workspace coverage was not achieved. The best automated sequence ran about 21–23 seconds per cycle, but recovery failures prevented achievement of the overall hourly target.

Final state: arm at the four-joint home pose, all six motor fault values zero, cube resting on the table, recorder stopped. The target in cube_cycle_state.json is NOT a verified cube position. cube_recovery_required.json blocks blind restart. Re-localize and validate a pickup before clearing it.

Working central/far pickup: shoulder lift 15, elbow 20, wrist flex 30, wrist roll approximately -30 degrees; pan varies. Use live calibration and supplied move/settle; home must only command its four joints. Contact-stop gripper approach worked repeatedly near 24 percent; retain contact goal with modest closing bias. Mere contact is insufficient: verify retained cube at home with camera. Initial excessive squeezing triggered gripper overload; only the documented gripper recovery was used, never arm fault overrides.

The grid uses FK to vary elbow while retaining nominal height/pitch, but this is NOT a calibrated robot-to-camera/table transform. Some closer placements led to repeated missed or slipping grasps. Open-loop replay of placement coordinates is unreliable after a release rotates or shifts the cube. Do not repeatedly chase the cube using those stale coordinates. Camera robot alignment and table height remain uncalibrated in calibration/ready.json.

Episode receipts are in cube_cycles.jsonl and supplemental_episodes.json. Failed exception episodes are supplemented there so cube_status.py counts all outcomes. The recorder/upload service may remove completed episode files after upload. Source tools and recovery logs remain in tools/.
