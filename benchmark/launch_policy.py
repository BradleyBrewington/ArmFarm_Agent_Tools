"""Stable entry point installed as workspace/tools/run_act_policy.py."""
from pathlib import Path
import runpy
import sys

release=(Path(__file__).resolve().parent/'benchmark/current').resolve(strict=True)
sys.path.insert(0,str(release))
runpy.run_path(str(release/'run_act_policy.py'),run_name='__main__')
