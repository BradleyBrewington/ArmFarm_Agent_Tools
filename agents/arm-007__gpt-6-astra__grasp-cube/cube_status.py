import json,time
from pathlib import Path
rows=[json.loads(s) for s in Path('tools/cube_cycles.jsonl').read_text().splitlines()]
e=[r for r in rows if r['event']=='episode'];good=[r for r in e if r['success']]
poses=set()
for r in good:
 p=json.loads(r['receipt']['notes'])['source_pose'];poses.add((round(p['shoulder_pan'],1),round(p['elbow_flex'],1)))
print(json.dumps(dict(automated_successes=len(good),total_successes_including_manual=len(good)+1,failures=sum(not r['success'] for r in e),unique_automated_pickup_positions=len(poses),recent_successes_per_hour=(len(good[-10:])-1)*3600/(good[-1]['time']-good[-10:][0]['time']) if len(good)>1 else None,last_event=rows[-1],age_seconds=time.time()-rows[-1]['time']),indent=2))
