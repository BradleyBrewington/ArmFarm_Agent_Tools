"""Report recorded outcomes separately from subsequent visual review."""
import json,time
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
rows=[json.loads(s) for s in Path('tools/cube_cycles.jsonl').read_text().splitlines()]
e={r['receipt']['episode']:dict(success=r['success'],time=r['time'],receipt=r['receipt']) for r in rows if r['event']=='episode'}
for r in json.loads(Path('tools/supplemental_episodes.json').read_text()):
 t=datetime.strptime(r['episode'][:15],'%Y%m%dT%H%M%S').replace(tzinfo=ZoneInfo('America/Chicago')).timestamp()
 e.setdefault(r['episode'],dict(success=r['success'],time=t,receipt={}))
review={r['episode']:r['actual_task_success'] for r in rows if r['event']=='manual_review'}
now=time.time();good=[r for r in e.values() if r['success']];poses=set()
for r in good:
 try:p=json.loads(r['receipt']['notes'])['source_pose']
 except (ValueError,KeyError,TypeError):continue
 poses.add(tuple(round(p[k],1) for k in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex')))
print(json.dumps(dict(recorded_successes=len(good),recorded_failures=len(e)-len(good),reviewed_task_successes=sum(review.get(k,r['success']) for k,r in e.items()),recorded_successes_last_hour=sum(r['time']>=now-3600 for r in good),distinct_successful_source_poses=len(poses),last_event=rows[-1],seconds_since_last_event=now-rows[-1]['time']),indent=2))
