#!/usr/bin/env bash
set -euo pipefail
station=/var/lib/armfarm/stations/armfarm
exec /opt/armfarm/venv/bin/python -u \
  "$station/workspace/tools/run_act_policy.py" \
  --execute --forever \
  --policy "$station/policies/arm004-act-100k/checkpoint/pretrained_model" \
  --output "$station/workspace/tools/act_trials/continuous-$(date -u +%Y%m%dT%H%M%S)-$$"
