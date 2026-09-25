# Basic simulation-to-real pickup test — 2026-09-24

The lip grasp lifted the cup in simulation and on physical arm001. User reassigned the physical test from arm002 because another controller was using arm002. No further arm002 commands were issued after the conflict was discovered.

Simulation used the supplied millimetre STL at 100% scale, estimated PLA mass 59.99 g, friction 0.4 and 0.98 Nm actuator cap. Windows trial `lip-test-003` retained the whole cup at least 29.49 mm above the table during a three-second hold. Replaying the same scene and commands on Pi4 retained the cup with 4.67 mm minimum clearance; both jaws contacted it, without table contacts. This difference limits confidence in quantitative contact predictions.

Physical episode: `20260924T133201-4b68126c`. This is a supervised basic pickup test, NOT a demonstration of workspace coverage, success ratio, throughput, or held-home completion.

Transfer used the grasp geometry, not the simulation joint coordinates. Arm001 live calibration, encoder feedback and paired cameras guided small movements. Early closures at diagnostic heights 120.6 and 91.6 mm closed empty (gripper 1.417%, load 96). Lowering and shifting laterally established wall contact at diagnostic height 62.8 mm: gripper 3.306%, load 208, visible mug displacement against the inner finger.

The mug tilted during the initial lift, then stabilized. Shoulder increments preserved the existing servo goals and grip. Final diagnostic tool height was 94.7 mm, a 31.9 mm tool rise from the contact pose. This is NOT a calibrated measurement of mug-bottom clearance. The top view and stable mug position relative to the moving wrist camera support physical pickup. A subsequent five-second stationary capture had 26 fresh frame pairs, unchanged six-motor encoder readings, and no fault bits. Gripper remained 1.957%, load 128. Motor force limits and calibration were not increased or changed.

Evidence:
- `arm001-lip-hold/hold.mp4`: side-by-side top/wrist five-second hold video.
- `arm001-lip-hold/025_top.jpg` and `025_wrist.jpg`: final hold photographs.
- `arm001-lip-hold/evidence.json`: timestamps, frame sequences and recorder feedback.
- `PHYSICAL_ACTIONS.jsonl`: exact physical commands and results, including arm002 interruption.
- `arm001-lip-actions/`: detailed arm001 action telemetry.
- Remote `tools/supervisor_lip_*.json` and matching JPEGs retain before/after evidence.

Key lesson: camera overlap is insufficient. Place fingers below the rim and across the wall, distinguish empty closure from contact, and verify the object follows actual arm motion. Servo tracking lag reached several degrees, so command changes alone do not prove a lift. Do not blindly replay these joint values on another arm or object location.

Final state: lowered the mug, opened jaws to approximately 9.2%, withdrew the fingers, and verified the mug remained on the table. Arm001 is holding its final pose; station agent remains paused. Recording closed successfully. The full episode has `feedback_gap` and `queue_overflow` quality flags and is NOT training-ready; the separate five-second hold evidence remains available. These recording-quality flags are distinct from observed pickup success.
