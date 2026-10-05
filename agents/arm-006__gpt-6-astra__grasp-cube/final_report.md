Latest check: no new episodes. Two-minute torque-off cooldown observation ended at48C with zero motor faults. Added supervised-retry criterion is30seconds at or below43C; it has not been met. This is a cooldown criterion, not a change to the operator-authorized65C software guard or70C hardware protection. Evidence: cooldown_latest.json.

Latest continuation:2 additional successes,70 cumulative (61 clean recordings),13 unsuccessful episodes. The next grasp stopped at65C under the unchanged65C software guard and70C hardware protection. A10-second torque-off rest was added between cycles, outside recording, with temperature and fault checks; it did not prevent recurrence. No protections were relaxed.

Cube visually confirmed on table; arm home within1.275deg; gripper open and torque off; recorder inactive; all motor fault bits zero. Temperature after release49C.

All45 nominal grid locations have sampled successful coverage, with earlier position corrections. Repeat pass completed indices0–13;14 failed. Continuous entire-workspace coverage and sustained50/hour remain unverified.

Operator-authorized resumption completed17 successful episodes before the updated temperature guard tripped.

68 successful episodes cumulatively; 59 without recording quality flags; 12 unsuccessful episodes. This resumed run:17 successes,16 clean recordings,1 unsuccessful episode. Elapsed 13.5 minutes including setup and recovery; interval rate 75.5 successes/hour. The planned50-success resumed run was interrupted; this is not a sustained one-hour validation.

Coverage: all45 nominal grid locations have now had successful pickup coverage, with earlier position corrections. This is sampled coverage, not certification of every point in the physical workspace. Repeat pass completed indices0–11; index12 failed.

Operator instruction a4b8b931-c9fb-42cd-a0e8-88e8353a0eb8 authorized resumption under the updated65C software guard with70C hardware protection unchanged. Both were verified before motion. The guard tripped on66C during episode20261005T143258-860c6f8a. That episode was marked unsuccessful. No protections were relaxed further. Root cause remains unconfirmed.

A collection process also receivedSIGTERM between episodes after178.6seconds. Arm and cube state were inspected; recorder was inactive with no faults. The placed cube was recovered in a separately recorded successful episode; later commands used shorter batches.

Final state: cube visually confirmed on tabletop; arm home within1.275deg; gripper open with torque off; recording inactive; all motor fault bits zero. Post-release temperature49C, hardware protection70C.
