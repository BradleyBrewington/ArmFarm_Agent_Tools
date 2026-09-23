"""Record this supervised session without changing recorder outcome labels."""
import json,time,os,glob,sqlite3
from pathlib import Path
from datetime import datetime,timezone
root=Path('tools/mug_observations')
obs=[]
for p in root.glob('*.json'):
 if p.stem.isdigit():obs.append((int(p.stem)/1e9,str(p),json.loads(p.read_text())))
obs.sort()
ids=['20260923T145636-1f26e417','20260923T150127-191efa22','20260923T151738-2934b2ce']
ledger_path=Path('tools/mug_episode_ledger.json')
previous={e['id']:e for e in json.loads(ledger_path.read_text())} if ledger_path.exists() else {}
episodes=[]
for eid in ids:
 p=Path('../episodes/completed')/eid/'episode.json'
 if p.exists():
  d=json.loads(p.read_text());e={k:d.get(k) for k in ['id','started','finished','success','notes','quality_flags','training_ready']};e['recorder_metadata']=str(p.resolve())
 elif eid in previous:
  e=previous[eid]
 else:
  e={'id':eid,'started':1790193396.,'finished':1790193687.,'success':False,'notes':'Rim grasp slipped during home transfer. Outcome and completion confirmed by recording.stop tool response; local episode folder subsequently removed by external process.','quality_flags':[],'training_ready':True}
 e['observations']=[{'time':t,'state':p,'top':p[:-5]+'_top.jpg','wrist':p[:-5]+'_wrist.jpg'} for t,p,d in obs if e['started']<=t<=e['finished']]
 e['grasp_location_frame']='URDF base_link FK estimate, not calibrated object center'
 if eid==ids[0]:
  e['grasp_location_m']=[.3513112881,-.0068900436,.0627942258];e['location_type']='failed rim contact'
 elif eid==ids[1]:
  e['grasp_location_m']=[.3448741787,.008673585,.0202351365];e['location_type']='successful handle contact'
  e['measured_grasp_joints']={'shoulder_pan':-1.010989010989011,'shoulder_lift':61.62637362637363,'elbow_flex':-72.65934065934066,'wrist_flex':86.9010989010989,'wrist_roll':-24.65934065934066,'gripper':3.98110661268556}
  e['initial_grip_goal']=2.322537112010841;e['initial_grip_load']=112
  e['final_grip_goal']=.2771929824561402
  e['home_evidence']=['tools/mug_observations/1790194488249069867.json','tools/mug_observations/1790194488249069867_top.jpg','tools/mug_observations/1790194488249069867_wrist.jpg']
  e['pregrasp_evidence']=['tools/mug_observations/1790194356873441986.json','tools/mug_observations/1790194356873441986_top.jpg','tools/mug_observations/1790194356873441986_wrist.jpg']
  e['verification']='Both camera images inspected: mug held by handle at saved home; all four home joints within 2 degrees. Grip tightened only after measured relaxation during staged lifts.'
 else:
  e['grasp_location_m']=None;e['location_type']='no retained grasp'
  e['last_attempt_fk_m']=[.2771562316,.0418578383,.0139232392]
  e['initial_attempt_fk_m']=[.3098591339,.1285792431,.0143084088]
 episodes.append(e)
for e in episodes:
 if e['id'] in previous and previous[e['id']]['success'] != e['success']:raise RuntimeError('Outcome mismatch; do not overwrite')
 previous[e['id']]=e
episodes=sorted(previous.values(),key=lambda e:e['started'])
Path('tools/mug_episode_ledger.json').write_text(json.dumps(episodes,indent=2))
conn=sqlite3.connect('file:/var/lib/armfarm/stations/armfarm/state/operator-instructions.sqlite3?mode=ro',uri=True)
received=[{'id':r[0],'created':r[1]} for r in conn.execute('SELECT id,created FROM instructions WHERE agent_id=? ORDER BY created',(os.environ['ARMFARM_AGENT_ID'],))]
# Broad survey envelope, NOT an asserted usable mask or calibrated coverage.
cells=[{'id':f'x{i:02d}_y{j:02d}','x_bounds_m':[round(i*.05,2),round((i+1)*.05,2)],'y_bounds_m':[round(-.4+j*.05,2),round(-.4+(j+1)*.05,2)],'status':'unverified; usability and registration pending'} for i in range(9) for j in range(16)]
# Provisional geometry artifact retained only as history.
if not Path('tools/mug_grid.json').exists():Path('tools/mug_grid.json').write_text(json.dumps({'cell_size_m':.05,'frame':'nominal URDF base_link','proposed_survey_envelope_m':{'x':[0,.45],'y':[-.4,.4]},'accepted_usable_bounds':None,'cells':cells},indent=2))
if Path('tools/supervisor_status.json').exists():
 print('Episode ledger merged; existing supervisor status preserved.');raise SystemExit(0)
now=time.time();n=sum(e['success'] for e in episodes);f=len(episodes)-n
s={'updated_utc':datetime.now(timezone.utc).isoformat(),'agent_id':os.environ['ARMFARM_AGENT_ID'],'task':'exp-grasp-mug-multiagent','received_instruction_ids':received,'episode_ledger':'tools/mug_episode_ledger.json','grid_ledger':'tools/mug_grid.json','successes':n,'unsuccessful_episodes':f,'success_to_failure_ratio':n/f if f else None,'verified_successes_trailing_hour':sum(e['success'] and now-3600<=e['finished']<=now for e in episodes),'coverage_complete':False,'calibration':{'source':'calibration/ready.json','robot_alignment':'not_calibrated','height':'not_measured','consequence':'Cannot certify object centers in metric grid cells. FK is a tool-frame estimate; grasp contact and object center differ.','historical_local_contact_fk_extent_m':{'x':[.0589509645,.3166617889],'y':[-.202533442,.1983361962]},'current_session_observed_fk_contact_x_max_m':.3844547702},'coverage_proposal':{'survey_bounds_m':{'x':[0,.45],'y':[-.4,.4]},'cell_size_m':.05,'candidate_cells':144,'accepted_usable_bounds':None,'basis':'Broad envelope for surveying the visible tabletop and arm reach, encompassing historical local measured contact positions and current mug approach extent, with lateral extent expanded for the arm reach. NOT a calibrated usable rectangle. Must register table to robot and evaluate visibility/reachability across the envelope before adopting a usable-cell mask; expand if valid visible workspace lies beyond it. Do not discard cells for failed grasps.','untested_or_uncertified_cells':'All 144 candidate cells listed individually in tools/mug_grid.json. Successful contact nominally falls in x06_y08, but cell certification is withheld until camera/table registration and object-center measurement.','verified_cells':[]},'transferable_discovery':'A side-lying mug handle was retained through short lift, inward carry, intermediate joint pose and saved home with wrist_roll about -25 degrees. Initial measured contact at FK (.344874,.008674,.020235); exact local q and evidence in ledger. These joint targets are not portable to another arm.','blocker':'Repeat pickup is not yet reliable: fixed jaw contacts outer handle or body during descent and displaces/tilts mug. Several camera projections looked aligned but closure reached empty endpoint (~1.417 measured gripper) and no retained grasp. Large shoulder approach-direction error (up to ~6.5 degrees) makes commanded XYZ unreliable.','next_hypothesis':'Use the verified local handle geometry; obtain fresh table/robot registration and explicit opposing jaw contact. Clear mug vertically before lateral corrections. Test 1-2 cm retention and inspect relative motion before staged transfer. Preserve honest failed episodes.','tools':['tools/mug_motion.py','tools/mug_grip.py','tools/episode_motion.py'],'safety_note':'Motion enforces camera liveness, calibrated target clamp and 8-degree tracking error stop. Optional settle tolerance up to 7 degrees used on Cartesian moves due measured shoulder undershoot; home uses original 4-degree tolerance. Wrist roll observed safe from +56 through -25 only. No torque disabled.','turn_boundary':'Episode closed unsuccessful; returning to saved home, final state to be added after completion.'}
Path('tools/supervisor_status.json').write_text(json.dumps(s,indent=2));print(json.dumps({k:s[k] for k in ['successes','unsuccessful_episodes','success_to_failure_ratio','verified_successes_trailing_hour','coverage_complete']}))
