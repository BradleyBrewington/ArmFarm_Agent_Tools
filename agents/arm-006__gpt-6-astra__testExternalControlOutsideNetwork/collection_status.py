import json,time,os
from pathlib import Path
root=Path(__file__).resolve().parent
initial=json.loads((root/'initial_results.json').read_text())
rows=[json.loads(l) for l in (root/'collection_log.jsonl').read_text().splitlines()]
started=json.loads((Path(os.environ['ARMFARM_RUN_DIR'])/'run.json').read_text())['started']
n=len(initial['successful'])+len(rows)
clean=sum(not r['quality_flags'] for r in initial['successful'])+sum(r['result']['training_ready'] for r in rows)
result={'successful_episodes':n,'clean_successful_episodes':clean,'elapsed_minutes':(time.time()-started)/60,'overall_successes_per_hour':n*3600/(time.time()-started),'automated_cycle_seconds_mean':sum(r['seconds'] for r in rows)/len(rows),'automated_cycles':len(rows),'last_evidence':rows[-1]['evidence']}
print(json.dumps(result,indent=2));(root/'collection_status.json').write_text(json.dumps(result,indent=2))
