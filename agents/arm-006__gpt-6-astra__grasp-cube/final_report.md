Collection paused for inconsistent temperature feedback.

51 cumulative successful episodes; 43 recordings without quality flags; 11 unsuccessful episodes. This run added 13 successes. Elapsed 13.5 minutes including alignment, recovery and temperature diagnosis; equivalent rate 58.0 successes/hour over this interval, not a sustained one-hour validation.

Coverage: planned grid indices0–40 completed, with some earlier corrected pickup positions. Indices41–44 remain. Entire-workspace coverage is not complete.

Temperature guard captured62C against the unchanged60C threshold. Subsequent idle trace with gripper torque disabled contained1783 readings ranging47–65C, with abrupt spikes and zero motor fault bits. Physical heating versus temperature feedback error remains unresolved. Further collection should wait for diagnosis. Trace: temperature_idle_trace.json.

Cube visually confirmed on table, arm home within1.10deg, gripper open with torque off. Recorder inactive; all motor fault bits zero.

The empty-hand camera check was updated to a higher image region after a cube on the table triggered the lower region; saved evidence distinguishes held images (dark fraction at least0.54) from released images (0.0). The check additionally requires open jaws. Held-object checks remain unchanged.

Latest read-only diagnostic: no new episodes. Reproduced single-register spikes to66C with gripper torque off; immediately following paired-register reads returned47C. See temperature_diagnosis.md and temperature_crosscheck_extended.json. Motion remains stopped pending diagnosis.
