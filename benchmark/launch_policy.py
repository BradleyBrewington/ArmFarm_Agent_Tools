"""Stable entry point installed as workspace/tools/run_act_policy.py."""
from pathlib import Path
import runpy
import sys

release=(Path(__file__).resolve().parent/'benchmark/current').resolve(strict=True)
sys.path.insert(0,str(release))
implementation=runpy.run_path(str(release/'run_act_policy.py'),run_name='act_policy_implementation')
# Keep read-only importers of the previous entry point working.
JOINTS=implementation['JOINTS']
ChunkPlayback=implementation['ChunkPlayback']
clamp_action=implementation['clamp_action']
GRIPPER_TORQUE_LIMIT=implementation['GRIPPER_TORQUE_LIMIT']
main=implementation['main']
if __name__=='__main__':main()
