import sys,json
from pathlib import Path
from collect_cube import run
points=json.loads((Path(__file__).parent/'coverage_plan.json').read_text())
run([points[int(sys.argv[1])]])
