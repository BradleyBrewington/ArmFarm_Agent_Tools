# Shared ACT benchmark

Canonical source: `BradleyBrewington/ArmFarm_Agent_Tools`, `main`, `benchmark/`.
Both arm-003 and arm-004 execute this same implementation. The baseline is the
actual arm-003 `run_act_policy.py` retrieved on 2026-09-25, including the 24%
gripper output cap. Its station I/O and motor-bus functions are included here so
different station-local helpers cannot silently change benchmark behavior.

## Execution contract

- CPU, two Torch compute threads; saved checkpoint preprocessing/postprocessing.
- Live top and wrist RGB images resized to 640×360 and six calibrated positions.
- One 100×6 absolute-position chunk; execute all 100 actions in order at 30 Hz,
  then capture new observations and infer again. Hold during the refill pause.
- No temporal ensembling, added velocity ramp, automatic homing or task-success detection.
- Same calibrated-range clipping and wrist-roll clip of -95° through +12° on
  both arms. The latter is retained from the requested arm-003 baseline; it is
  not a newly measured physical limit for arm-004.
- Same gripper `Torque_Limit=240` applied at execution startup.
- Same existing phase, feedback, mode, torque and fault handling as arm-003.
  Local recovery for individual joint faults is not implemented in this baseline.
- Timed runs record episodes. Continuous runs keep bounded recent telemetry and
  do not record video. Ctrl+C or systemd stop holds the arm with torque enabled.

`policies.json` chooses each robot's default checkpoint. An explicit `--policy`
can select another installed checkpoint for cross-robot tests; its hash identifies
the model independently of the robot's name. No model weights are stored in Git.
Each robot uses its own live motor calibration and each policy its saved training
normalization. Those inputs are recorded rather than silently made identical.

## Run on either arm

```bash
station=/var/lib/armfarm/stations/armfarm

# Live-camera/model check: no motor connection or commands.
/opt/armfarm/venv/bin/python "$station/workspace/tools/run_act_policy.py" \
  --check --output "$station/workspace/tools/act_trials/check-$(date -u +%Y%m%dT%H%M%S)"

# Timed physical evaluation, using the same command on both arms.
/opt/armfarm/venv/bin/python "$station/workspace/tools/run_act_policy.py" \
  --execute --seconds 45 \
  --output "$station/workspace/tools/act_trials/eval-$(date -u +%Y%m%dT%H%M%S)"

# Continuous execution; append --policy PATH to override the model.
bash "$station/workspace/tools/run_act_loop.sh"
```

## Versioning and sync

`armfarm-benchmark-sync.timer` checks GitHub every minute, independent of the
laptop. Git downloads only the benchmark contents through a shallow filtered
cache. Identical bundle hashes mean identical benchmark files, including the
runner, helpers, policy catalog and launcher. Every report includes this bundle
hash, the Git snapshot commit, actual source hashes, checkpoint/processor/config
hashes, library versions and the robot's initial calibration and state.

Publish changes in this folder on the canonical repository's `main` branch.
Both robots then adopt them while idle. A policy lock defers updates during any
active timed/continuous run; restart a continuous run to adopt a pending update.
The launcher resolves an immutable release path, so a running process stays on
its original implementation. Sync never starts or restarts physical execution.
Offline robots keep their installed version; compare bundle hashes before
comparing results. This is periodic convergence, not an atomic two-robot rollout.

Local edits to managed benchmark files are repaired on the next idle sync. Make
changes in GitHub rather than editing the two Pis separately. Original replaced
entry points are backed up in `state/benchmark-backups/`. Existing calibration
scripts and historical trial helpers are not modified or imported by this runner.

```bash
cat /var/lib/armfarm/stations/armfarm/state/benchmark-sync.json
sudo systemctl start armfarm-benchmark-sync.service

# Pin or roll back: stop automatic following, then select an exact Git commit.
sudo systemctl stop armfarm-benchmark-sync.timer
/opt/armfarm/venv/bin/python \
  /var/lib/armfarm/stations/armfarm/workspace/tools/sync_act_benchmark.py \
  --revision COMMIT_SHA
```

With the same implementation, compare trials using controlled starting poses,
object placements, checkpoint identities and runtime versions. This deployment
does not retroactively make older trials comparable.

Tests: `python -m unittest discover -s benchmark -p 'test_*.py'` on Linux.
