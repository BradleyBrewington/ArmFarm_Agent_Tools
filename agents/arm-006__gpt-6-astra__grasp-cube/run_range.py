"""Collect a bounded slice of the saved coverage grid; starts holding at home."""
import sys,json
from pathlib import Path
from collect_cube import run
points=json.loads((Path(__file__).parent/'coverage_plan.json').read_text())
run(points[int(sys.argv[1]):int(sys.argv[2])])
