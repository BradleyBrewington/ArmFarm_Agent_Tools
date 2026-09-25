#!/bin/bash
cd /var/lib/armfarm/stations/armfarm/workspace
setsid nohup timeout 3600 /opt/armfarm/venv/bin/python tools/run_episodes.py --n "${1:-60}" > "tools/${2:-run2}.log" 2>&1 < /dev/null &
disown
