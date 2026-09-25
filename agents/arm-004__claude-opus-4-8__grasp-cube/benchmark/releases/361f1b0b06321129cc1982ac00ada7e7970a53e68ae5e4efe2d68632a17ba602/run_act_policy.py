"""ACT rollout: play complete 100-action chunks, timed or continuously."""
import argparse
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

from act_runtime import JOINTS,call,state,camera,healthy
from provenance import select_policy,runtime_version


GRIPPER_TORQUE_LIMIT = 240  # Feetech output scale: 0..1000.


def clamp_action(values,calibration):
    if len(values)!=6 or not all(math.isfinite(v) for v in values):raise ValueError('Non-finite or malformed action')
    result=[]
    for name,value in zip(JOINTS,values):
        c=calibration[name];span=(c['range_max']-c['range_min'])*180/4095
        lo,hi=(0.,100.) if name=='gripper' else (-span,span)
        if name=='wrist_roll':lo,hi=max(lo,-95.),min(hi,12.)
        result.append(min(hi,max(lo,value)))
    return result


class ChunkPlayback:
    """Consume each action exactly once; request inference only after the last slot."""
    def __init__(self,hz=30):
        self.period=1/hz;self.actions=[];self.index=0;self.next_due=0.

    def load(self,actions,now):
        if len(actions)!=100:raise ValueError('Expected 100 actions')
        self.actions=actions;self.index=0;self.next_due=now

    def ready(self,now):
        return self.index<len(self.actions) and now>=self.next_due

    def take(self,now):
        index=self.index;self.index+=1
        # Never skip actions or burst through overdue targets.
        self.next_due=now+self.period
        return index,self.actions[index]

    def exhausted(self,now):
        return bool(self.actions) and self.index==len(self.actions) and now>=self.next_due


def main():
    p=argparse.ArgumentParser(description=__doc__)
    execution=p.add_mutually_exclusive_group(required=True)
    execution.add_argument('--execute',action='store_true')
    execution.add_argument('--check',action='store_true',help='Validate live inference without motor commands')
    duration=p.add_mutually_exclusive_group()
    duration.add_argument('--seconds',type=float)
    duration.add_argument('--forever',action='store_true',help='Run continuously without video recording; retain bounded recent telemetry')
    p.add_argument('--policy',type=Path,help='Checkpoint directory; defaults to this robot\'s registered policy')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not a.check and not a.forever and (a.seconds is None or not 1<=a.seconds<=300):p.error('Duration must be between 1 and 300 seconds')
    station=Path('/var/lib/armfarm/stations/armfarm')
    lock=(station/'state/act-benchmark.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    a.policy,model=select_policy(station,a.policy)
    a.output.mkdir(parents=True,exist_ok=False)
    stop=threading.Event();request=threading.Event();shared_lock=threading.Lock()
    shared={'error':None,'chunk':None,'index':0}
    report={'status':'loading','duration_requested_s':a.seconds,'model_sha256':model['sha256'],
            'model':model,'benchmark':runtime_version(a.policy),
            'policy_mode':'full_chunk','temporal_coefficient':None,'threads':2,
            'chunk_size':100,'inference_pacing':'after_full_chunk','control_hz':30,
            'max_joint_units_per_second':None,'completed_chunks':0,
            'continuous':a.forever,'prediction_count':0,'command_count':0,
            'predictions':[],'commands':[],'task_success':None}
    def save():
        tmp=a.output/'report.tmp';tmp.write_text(json.dumps(report,indent=2));tmp.replace(a.output/'report.json')
    def interrupt(signum,frame):stop.set();raise RuntimeError('Rollout interrupted')
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt);save()
    import motor_bus as cw
    import cv2
    import numpy as np
    import torch
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor import PolicyProcessorPipeline
    from lerobot.processor.converters import batch_to_transition,transition_to_batch,policy_action_to_transition,transition_to_policy_action
    torch.set_num_threads(2);torch.set_num_interop_threads(1);cv2.setNumThreads(1)
    cfg=ACTConfig.from_pretrained(a.policy,local_files_only=True)
    cfg.device='cpu';cfg.pretrained_backbone_weights=None;cfg.n_action_steps=100;cfg.temporal_ensemble_coeff=None
    policy=ACTPolicy.from_pretrained(a.policy,config=cfg,local_files_only=True,strict=True).eval()
    pre=PolicyProcessorPipeline.from_pretrained(str(a.policy),config_filename='policy_preprocessor.json',overrides={'device_processor':{'device':'cpu'}},to_transition=batch_to_transition,to_output=transition_to_batch)
    post=PolicyProcessorPipeline.from_pretrained(str(a.policy),config_filename='policy_postprocessor.json',overrides={'device_processor':{'device':'cpu'}},to_transition=policy_action_to_transition,to_output=transition_to_policy_action)
    s,initial=healthy(station);calibration=initial['calibration'];report['initial']=initial
    if s.get('recording'):raise RuntimeError('Another recording is open')
    if not a.check and initial['torque'] not in ([0]*6,[1]*6):raise RuntimeError('Mixed initial torque state')
    if (station/'state/act-stop').exists():raise RuntimeError('Operator stop is set')

    def predict():
        started=time.monotonic();_,sample=healthy(station,calibration)
        obs={'observation.state':torch.tensor(state(sample),dtype=torch.float32)}
        for role in ('top','wrist'):
            img=cv2.imdecode(np.frombuffer(camera(role),np.uint8),cv2.IMREAD_COLOR)
            if img is None:raise RuntimeError('Camera decode failed')
            rgb=cv2.cvtColor(cv2.resize(img,(640,360),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2RGB)
            obs['observation.images.'+role]=torch.from_numpy(rgb).permute(2,0,1).float()/255.
        with torch.inference_mode():values=post(policy.predict_action_chunk(pre(obs)))[0].tolist()
        if len(values)!=100:raise ValueError('Checkpoint did not return 100 actions')
        return [clamp_action(v,calibration) for v in values],values,time.monotonic()-started

    first,raw,elapsed=predict()
    report['initial_prediction']={'raw':raw,'target':first,'seconds':elapsed}
    if a.check:
        report.update(status='validated',motor_commands_sent=0,action_shape=[100,6])
        save();print(json.dumps({'status':'validated','model':model['name'],'benchmark':report['benchmark'],
                               'inference_seconds':elapsed,'action_shape':[100,6],'motor_commands_sent':0}),flush=True)
        return
    for role in ('top','wrist'):(a.output/('before-'+role+'.jpg')).write_bytes(camera(role))
    recording=False;worker=None;armed=False
    def infer():
        try:
            while not stop.is_set():
                if not request.wait(.1):continue
                request.clear()
                if stop.is_set():break
                target,raw,elapsed=predict()
                if stop.is_set():break
                with shared_lock:
                    shared.update(chunk=target,index=shared['index']+1)
                    report['predictions'].append({'time':time.monotonic(),'seconds':elapsed,'raw':raw,'target':target})
                    report['prediction_count']+=1
                    if a.forever:del report['predictions'][:-5]
        except BaseException as error:
            with shared_lock:shared['error']=str(error)
            stop.set()
    try:
        with cw.connected_bus(call('port')['port']) as bus:
            cw.write_register(bus,'Torque_Limit','gripper',GRIPPER_TORQUE_LIMIT)
            report['gripper_torque_limit_raw']=cw.read_register(bus,'Torque_Limit','gripper')[0]
            _,now=healthy(station,calibration)
            for name in JOINTS:
                for key in ('range_min','range_max','homing_offset'):
                    if getattr(bus.calibration[name],key)!=calibration[name][key]:raise RuntimeError('Motor calibration differs')
            measured=state(now);live=bus.sync_read('Present_Position',list(JOINTS),normalize=True)
            if max(abs(live[j]-measured[i]) for i,j in enumerate(JOINTS))>1:raise RuntimeError('Normalization mismatch')
            settings=json.loads((station/'settings.json').read_text())
            context={'experiment_id':settings.get('experiment_id'),'agent_id':settings['agent_id'],
                     'provider':'lerobot','model':model['name'],'run_id':a.output.name}
            if not a.forever:
                report['recording_start']=call('start_recording',timeout=25,task=f'ACT {a.seconds:g}-second rollout, full 100-action chunks at 30 Hz',context=context);recording=True
            if now['torque']==[0]*6:
                bus.sync_write('Goal_Position',dict(zip(JOINTS,now['state_raw'])),normalize=False)
                bus.sync_write('Torque_Enable',{j:1 for j in JOINTS},normalize=False);time.sleep(.1)
            armed=True;_,now=healthy(station,calibration,armed=True)
            started=time.monotonic();deadline=math.inf if a.forever else started+a.seconds;report.update(status='running',started=time.time())
            playback=ChunkPlayback();playback.load(first,started);chunk_index=0;waiting=False
            worker=threading.Thread(target=infer,daemon=True);worker.start();save()
            next_health=started;actual=state(now)
            try:
                while time.monotonic()<deadline:
                    now_time=time.monotonic()
                    if stop.is_set():raise RuntimeError(shared['error'] or 'Stop requested')
                    if (station/'state/act-stop').exists():raise RuntimeError('Operator stop requested')
                    if now_time>=next_health:
                        status=call('status',timeout=.3);sample=status['latest']
                        if time.monotonic()-sample['t_mono']>.2:raise RuntimeError('Stale motor feedback')
                        if any(sample['fault_bits']) or any(sample['mode']):raise RuntimeError('Motor fault or incorrect operating mode')
                        if sample['torque']!=[1]*6:raise RuntimeError('Motor torque changed')
                        if status.get('pen_context') or json.loads((station/'state/status.json').read_text()).get('phase')!='paused':raise RuntimeError('Station controller changed')
                        actual=state(sample);next_health=time.monotonic()+.1
                    with shared_lock:
                        if shared['error']:raise RuntimeError(shared['error'])
                        if waiting and shared['chunk'] is not None:
                            playback.load(shared['chunk'],time.monotonic());chunk_index=shared['index']
                            shared['chunk']=None;waiting=False
                    now_time=time.monotonic()
                    if now_time>=deadline:break
                    if not waiting and playback.exhausted(now_time):
                        report['completed_chunks']+=1;waiting=True
                        if a.forever:
                            report['motion_seconds']=now_time-started;save()
                        request.set()
                    if not waiting and playback.ready(now_time):
                        action_index,command=playback.take(now_time)
                        bus.sync_write('Goal_Position',dict(zip(JOINTS,command)),normalize=True)
                        report['commands'].append({'time':time.monotonic(),'prediction':chunk_index,'action_index':action_index,'target':command,'measured':actual})
                        report['command_count']+=1
                        if a.forever:del report['commands'][:-300]
                    next_tick=min(next_health,deadline,time.monotonic()+.01 if waiting else playback.next_due)
                    stop.wait(max(0,next_tick-time.monotonic()))
                report.update(status='completed',motion_seconds=time.monotonic()-started)
            except BaseException:
                # Hold current arm positions on abort; preserve the gripper's holding target.
                raw_hold=bus.sync_read('Present_Position',list(JOINTS[:5]),normalize=False)
                bus.sync_write('Goal_Position',raw_hold,normalize=False)
                raise
            finally:
                stop.set()
                if worker:worker.join(timeout=6)
            _,report['final']=healthy(station,calibration,armed=True)
    except BaseException as error:
        report.update(status='aborted',error=str(error));raise
    finally:
        stop.set()
        if worker and worker.is_alive():worker.join(timeout=6)
        report['finished']=time.time();report['torque_left_enabled']=armed
        if recording:
            try:report['recording_stop']=call('stop_recording',timeout=25,success=False,notes='ACT rollout; task success not automatically verified. '+report['status'])
            except Exception as error:report['recording_error']=str(error)
        for role in ('top','wrist'):
            try:(a.output/('after-'+role+'.jpg')).write_bytes(camera(role))
            except Exception as error:report['camera_error']=str(error)
        if report.get('motion_seconds'):
            report['fresh_prediction_hz']=report['prediction_count']/report['motion_seconds']
            report['command_hz']=report['command_count']/report['motion_seconds']
            intervals=[b['time']-a['time'] for a,b in zip(report['commands'],report['commands'][1:]) if a['prediction']==b['prediction']]
            if intervals:report['chunk_playback_hz']=len(intervals)/sum(intervals)
        save();print(json.dumps({k:v for k,v in report.items() if k not in ('commands','predictions','initial','final')}),flush=True)


if __name__=='__main__':main()
