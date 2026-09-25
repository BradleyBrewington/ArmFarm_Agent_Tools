"""Read-only live-camera ACT validation; sends no motor commands."""
import argparse,hashlib,importlib.metadata,json,resource,sys,time
from pathlib import Path
station=Path('/var/lib/armfarm/stations/armfarm')
sys.path.insert(0,str(station/'workspace/tools'))
import act_trial as helper
from run_act_policy import clamp_action
import cv2,numpy as np,torch
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline
from lerobot.processor.converters import batch_to_transition,transition_to_batch,policy_action_to_transition,transition_to_policy_action
torch.set_num_threads(2);torch.set_num_interop_threads(1);cv2.setNumThreads(1)
folder=station/'policies/arm004-act-100k';path=folder/'checkpoint/pretrained_model'
with (path/'model.safetensors').open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
assert digest==helper.MODEL_SHA
config=ACTConfig.from_pretrained(path,local_files_only=True)
config.device='cpu';config.pretrained_backbone_weights=None;config.temporal_ensemble_coeff=None;config.n_action_steps=100
policy=ACTPolicy.from_pretrained(path,config=config,local_files_only=True,strict=True).eval()
pre=PolicyProcessorPipeline.from_pretrained(str(path),config_filename='policy_preprocessor.json',overrides={'device_processor':{'device':'cpu'}},to_transition=batch_to_transition,to_output=transition_to_batch)
post=PolicyProcessorPipeline.from_pretrained(str(path),config_filename='policy_postprocessor.json',overrides={'device_processor':{'device':'cpu'}},to_transition=policy_action_to_transition,to_output=transition_to_policy_action)
initial=helper.call('status')['latest']
timings=[]
for _ in range(3):
    started=time.monotonic();sample=helper.call('status')['latest']
    if time.monotonic()-sample['t_mono']>.2:raise RuntimeError('Feedback is stale')
    obs={'observation.state':torch.tensor(helper.state(sample),dtype=torch.float32)}
    for role in ('top','wrist'):
        frame=cv2.imdecode(np.frombuffer(helper.camera(role),np.uint8),cv2.IMREAD_COLOR)
        rgb=cv2.cvtColor(cv2.resize(frame,(640,360),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2RGB)
        obs['observation.images.'+role]=torch.from_numpy(rgb).permute(2,0,1).float()/255.
    with torch.inference_mode():actions=post(policy.predict_action_chunk(pre(obs)))
    assert tuple(actions.shape)==(1,100,6) and torch.isfinite(actions).all()
    for action in actions[0].tolist():clamp_action(action,sample['calibration'])
    timings.append(time.monotonic()-started)
final=helper.call('status')['latest']
report={'status':'validated','robot':'arm-004','model_sha256':digest,'action_shape':list(actions.shape),
        'inference_seconds':timings,'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        'motor_commands_sent':0,'torque_before':initial['torque'],'torque_after':final['torque'],
        'fault_bits':final['fault_bits'],'calibration':final['calibration'],
        'versions':{name:importlib.metadata.version(name) for name in ('lerobot','torch','torchvision')},
        'completed_at_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
(folder/'validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
