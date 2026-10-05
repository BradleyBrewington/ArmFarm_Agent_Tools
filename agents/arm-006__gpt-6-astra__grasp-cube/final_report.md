Collection stopped after another temperature guard trip.

38 successful episodes cumulatively, 31 without recording quality flags; 10 unsuccessful episodes. This resumed run added eight successes (seven clean recordings) and one unsuccessful episode. Full-workspace coverage remains incomplete. The earlier full-run rate was 34.4/hour; no sustained one-hour run at the target rate has been established.

The guard captured gripper temperature71C against a60C threshold during episode20261005T113703-f2c29c00. Twenty readings after release were50C with no faults. This discrepancy remains unresolved; safeguards were not relaxed. Detailed guard diagnostics were added to calibrate_workspace.py.

Cube placed on tabletop and visually confirmed. Arm home within1.275deg; gripper open and torque disabled for cooling. Recording stopped; all motor fault bits zero.

Coverage now includes grid indices0–28, with some corrected pickup positions and initial exploratory positions. Index29 failed; indices30–44 remain uncollected. Logs: initial_results.json, collection_log.jsonl, temperature_after_stop.json.
