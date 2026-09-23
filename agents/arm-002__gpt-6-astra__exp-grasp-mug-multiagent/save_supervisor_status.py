"""Read-only episode/queue audit; writes derived evidence and coverage ledger in tools."""
import json,os,sqlite3,time,math
from pathlib import Path
import numpy as np
from check_saved_alignment import arm_transform
from mug_control import OFFSET
from recording import request
R=Path(__file__).resolve().parent
agent=os.environ['ARMFARM_AGENT_ID']
conn=sqlite3.connect('file:/var/lib/armfarm/stations/armfarm/state/operator-instructions.sqlite3?mode=ro',uri=True)
messages=[dict(zip(('id','text','created'),r)) for r in conn.execute('SELECT id,text,created FROM instructions WHERE agent_id=? ORDER BY created',(agent,))]
(R/'received_operator_guidance.json').write_text(json.dumps(messages,indent=2))
entries=[];evidence=R/'mug_evidence';evidence.mkdir(exist_ok=True)
for path in sorted(Path('/var/lib/armfarm/stations/armfarm/episodes/completed').glob('*/episode.json')):
 meta=json.loads(path.read_text())
 if meta.get('agent_id')!=agent:continue
 frames=[json.loads(s) for s in (path.parent/'frames.jsonl').open()]
 closures=[];last=None;active=False;end=None
 for f in frames:
  g=f['action'][5]
  if last is not None and g<last-.02:active=True;end=f
  elif active and end is not None and f['t_wall']-end['t_wall']>.7:
   if g<40:
    j=dict(zip(meta['joint_names'],f['observation.state']));idx=f['frame_index']
    item={'frame_index':idx,'t_wall':f['t_wall'],'measured_joints':j,'estimated_tool_xyz_base_m':(arm_transform(j)@OFFSET)[:3].tolist(),'verified_grasp':False,'note':'Closure candidate, not a verified grasp; diagnostic FK alignment.'}
    for role,im in f['images'].items():
     out=evidence/f"{meta['id']}_{idx}_{role}.jpg"
     with (path.parent/im['file']).open('rb') as src:src.seek(im['offset']);out.write_bytes(src.read(im['length']))
     item[role+'_evidence']=str(out)
    closures.append(item)
   active=False;end=None
  last=g
 entries.append({'episode_id':meta['id'],'started':meta['started'],'finished':meta.get('finished'),'success':meta.get('success'),'notes':meta.get('notes'),'quality_flags':meta.get('quality_flags'),'source':str(path.parent),'closure_attempts':closures,'verified_grasp_locations':[]})
(R/'mug_episode_ledger.json').write_text(json.dumps(entries,indent=2))
health=request('status');home=json.loads(Path('home_pose.json').read_text())['joints'];raw=health['latest']['state_raw'];cal=health['latest']['calibration'];names=list(cal)
joints={j:((raw[i]-(cal[j]['range_min']+cal[j]['range_max'])/2)*360/4095 if j!='gripper' else (raw[i]-cal[j]['range_min'])*100/(cal[j]['range_max']-cal[j]['range_min'])) for i,j in enumerate(names)}
cells=[{'id':f'{i}:{k}','bounds_x':[round(-.05+i*.05,3),round(i*.05,3)],'bounds_y':[round(-.45+k*.05,3),round(-.4+k*.05,3)],'usability':'pending_table_polygon_reach_and_obstacle_validation','verified_successes':0} for i in range(11) for k in range(17)]
now=time.time();s=sum(e['success'] is True for e in entries);fail=sum(e['success'] is False for e in entries)
status={'timestamp':now,'agent_id':agent,'received_operator_ids':[m['id'] for m in messages],'task_preserved':True,'recording':health['recording'],'home_joints_measured':{j:joints[j] for j in home},'home_errors_degrees':{j:joints[j]-v for j,v in home.items()},'torque':health['latest']['torque'],'motor_fault_bits':health['latest']['fault_bits'],'cumulative_current_assignment':{'successes':s,'unsuccessful':fail,'successes_trailing_hour':sum(e['success'] is True and e['finished']>=now-3600 for e in entries),'ratio_requirement_met':s>=2*fail and s>0},'episode_ledger':str(R/'mug_episode_ledger.json'),'grasp_location_caveat':'No successful grasp this run. Measured closure poses and paired images are in ledger. These are not asserted object centers; failed contacts displaced mug. No cells credited.','coverage':{'cell_size_m':.05,'coordinate_frame':'diagnostic base_link estimate','proposed_envelope_x_m':[-.05,.5],'proposed_envelope_y_m':[-.45,.4],'visible_table_polygon_xy_m':[[-.008,-.433],[.005,.367],[.470,.302],[.427,-.315]],'sources':['calibration/20260921T152058_891355289/intrinsics.json','calibration_checks/live-table-20260922/table.json','tools/current_top.jpg'],'bounds_rationale':'Outward-rounded 5 cm envelope of visible tabletop transformed using saved intrinsics and diagnostic live table alignment; not chosen from successful locations. Actual usable mask must intersect tabletop, object footprint clearance, kinematic reach, robot base and obstacles. Alignment and boundary reach still require validation.','usable_workspace_finalized':False,'coverage_complete':False,'candidate_cells':cells,'cells_without_verified_success':[c['id'] for c in cells],'untested_usability_cells':[c['id'] for c in cells]},'tool':'tools/mug_control.py','discoveries':['IK now uses live measured wrist roll instead of fixed +63 degrees.','Wrist -25 degrees makes jaw opening visible transversely in top view; no verified grasp follows from this alone.','Repeated closures reached near empty-jaw aperture and failed lift checks; some moved mug.','Home uses only four saved joints; wrist and gripper preserved.'],'next_hypothesis':'Inspect historical c_home_held evidence and successful handle approach source; compare freshly verified peer handle opposing-contact sequence. Retract and re-localize before insertion. Verify opposing contacts, 1-2 cm lift, stable hold, staged carry, then home; do not replay old joint poses blindly.','historical_success_reference':'03e5351425018a23b53da62aa87ca456482686359129db12f6a5690c8088777f:0','peer_success_episode':'20260923T150127-191efa22','stable_boundary':True}
(R/'supervisor_status.json').write_text(json.dumps(status,indent=2))
print(json.dumps({'episodes':len(entries),'successes':s,'failures':fail,'closure_candidates':sum(len(e['closure_attempts']) for e in entries),'home_errors':status['home_errors_degrees'],'recording':health['recording'],'torque':status['torque'],'grid_candidate_cells':len(cells)}))
