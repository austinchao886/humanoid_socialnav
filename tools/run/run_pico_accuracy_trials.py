"""Bounded simulation-only fresh-session comparison; run on the lab host."""
import argparse,json,subprocess,time
from pathlib import Path
ROOT=Path('/home/adcs-public-robot/Documents/socialnav_humanoid_ws');REPO=ROOT/'motion_pipeline';EXCHANGE=ROOT/'motion_exchange';RUNTIME=EXCHANGE/'.runtime'

def run(*cmd):
 r=subprocess.run(cmd,cwd=REPO,text=True,capture_output=True,timeout=180)
 if r.returncode:raise RuntimeError(' '.join(cmd)+': '+r.stderr[-1500:]+r.stdout[-1000:])
 return r.stdout

def read(path):
 try:return json.loads(path.read_text())
 except (FileNotFoundError,json.JSONDecodeError):return {}

def wait(predicate,seconds,label):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  value=predicate()
  if value:return value
  time.sleep(1)
 raise TimeoutError(label)

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--conditions',nargs='+',default=['pico-gmr-full-v1','pico-accuracy-half-v1','pico-accuracy-holds-v1']);p.add_argument('--repetitions',type=int,choices=range(1,4),default=3);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 # Fail closed before restarting anything: require isolated simulation DDS and expected torque-limited code.
 info=json.loads(run('docker','inspect','isaac-runner'))[0];env=dict(v.split('=',1) for v in info['Config']['Env'])
 assert env['DDS_DOMAIN']=='42' and env['DDS_INTERFACE']=='lo' and env['SIM_LOWCMD_TOPIC']=='rt/socialnav_sim/g1/lowcmd'
 assert all(env.get(k,'0')=='0' for k in ['ISAAC_RUNNER_VECTORIZED_IMPLICIT_WRITES','ISAAC_RUNNER_HOST_CRITICAL_METRICS','ISAAC_RUNNER_ZERO_GAIN_EFFORT_WRITES'])
 assert env.get('ISAAC_RUNNER_DEVICE','cuda:0')=='cuda:0'
 source=(ROOT/'unitree_sim_isaaclab/action_provider/action_provider_sonic_dds.py').read_text();assert 'self._motor_effort_upper' in source
 conditions=a.conditions
 assert all('/' not in motion and motion.startswith('pico-') for motion in conditions)
 for motion in conditions:assert read(EXCHANGE/motion/'validation.json').get('valid'),motion
 records=[]
 def save():
  temp=a.output/'trials.tmp';temp.write_text(json.dumps(records,indent=2)+'\n');temp.replace(a.output/'trials.json')
 for repetition in range(a.repetitions):
  for index in range(len(conditions)):
   motion=conditions[(index+repetition)%len(conditions)];record=dict(repetition=repetition+1,motion_id=motion,state='STARTING',started_epoch_s=time.time());records.append(record);save();print(json.dumps(record),flush=True)
   try:
    old=read(RUNTIME/'isaac_status.json').get('session_id')
    run('docker','stop','sonic-tracker');run('docker','restart','isaac-runner')
    def ready():
     d=read(RUNTIME/'isaac_status.json')
     return d if d.get('session_id') not in (None,old) and d.get('state')=='READY' and time.time()-d.get('updated_epoch_s',0)<5 else None
    status=wait(ready,120,'fresh Isaac READY');session=status['session_id'];record['session_id']=session;save()
    run('docker','start','sonic-tracker')
    def interactive():
     d=read(RUNTIME/'isaac_status.json')
     # Supervisor deliberately requests its own fresh bootstrap on startup.
     if d.get('session_id') in (None,old):return None
     if d.get('state') in ['UNSAFE','FAILED']:raise RuntimeError('unsafe initialization: '+str(d.get('reason')))
     return d if d.get('state')=='INTERACTIVE' and time.time()-d.get('updated_epoch_s',0)<5 else None
    status=wait(interactive,180,'interactive startup');session=status['session_id'];record['session_id']=session;save()
    record['command_epoch_s']=time.time();run('docker','exec','sonic-tracker','motion-cli','--domain','42','--interface','lo','control','approve_execute',motion,motion)
    record['state']='EXECUTING';save()
    def completed():
     for f in sorted((EXCHANGE/'executions').glob('*_0001_'+motion+'.json'),key=lambda f:f.stat().st_mtime,reverse=True):
      d=read(f)
      if d.get('session_id')==session:return f,d
     d=read(RUNTIME/'isaac_status.json')
     if d.get('session_id')!=session:raise RuntimeError('simulation restarted during playback')
     if d.get('state') in ['UNSAFE','FAILED','SAFE_STOP']:raise RuntimeError('terminal simulation state: '+str(d))
     return None
    report_path,report=wait(completed,600,'motion completion');record.update(state=report['result'],report=str(report_path),realtime_factor=report.get('performance',{}).get('unsupported_playback_realtime_factor'))
    trace=EXCHANGE/'executions'/Path(report['trace_path']).name
    output=a.output/(f'{len(records):02d}-'+motion+'.json')
    command=['docker','run','--rm','--network','none','-e','OPENBLAS_NUM_THREADS=1','-v',str(REPO)+':/workspace:ro','-v',str(EXCHANGE)+':/motion_exchange','--entrypoint','python','social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2','/workspace/tools/analysis/pico_accuracy.py','analyze','--artifact','/motion_exchange/'+motion,'--report','/motion_exchange/executions/'+report_path.name,'--trace','/motion_exchange/executions/'+trace.name,'--output','/motion_exchange/'+str(output.relative_to(EXCHANGE))]
    run(*command);record['analysis']=str(output)
   except Exception as exc:
    record.update(state='FAILED',error=str(exc));print('FAILED '+str(exc),flush=True)
   record['finished_epoch_s']=time.time();save();print(json.dumps(record),flush=True)
 print('STUDY_FINISHED',flush=True)
if __name__=='__main__':main()
