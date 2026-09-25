#!/bin/bash
# Run one episode batch sized to finish before the agent session is restarted (the restart SIGTERMs
# running episodes). Restarts come ~55-60 min after each session starts (2026-09-23: 16:17 .. 20:17,
# 21:15, 22:10), so launch this right after a restart with the default 48 min budget.
# Usage: tools/hourly_batch.sh LOGNAME [MINUTES]
cd /var/lib/armfarm/stations/armfarm/workspace
budget=$(( ${2:-48} * 60 ))
echo "budget ${budget}s (ends by $(date -d @$(( $(date +%s) + budget )) +%T))"
exec timeout $(( budget + 60 )) /opt/armfarm/venv/bin/python tools/run_episodes.py --n 200 --budget "$budget" > "tools/${1:-batch}.log" 2>&1
