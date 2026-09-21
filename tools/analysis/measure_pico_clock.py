"""Bound MSI-to-lab clock offset using repeated samples over one SSH session."""
import json,subprocess,time
p=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','adcspublicrobot','python3','-u','-c',
 '"import sys,time; [(print(time.time_ns(),flush=True)) for line in sys.stdin]"'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
try:
 for i in range(10):
  t0=time.time_ns();p.stdin.write('sample\n');p.stdin.flush();remote=int(p.stdout.readline());t1=time.time_ns()
  rows.append(dict(offset_ns=remote-(t0+t1)/2,uncertainty_ns=(t1-t0)/2,rtt_ns=t1-t0));time.sleep(.02)
finally:p.stdin.close();p.wait(timeout=5)
best=min(rows,key=lambda r:r['rtt_ns'])
print(json.dumps(dict(schema_version=1,source='msi',target='lab',measured_epoch_s=time.time(),
 remote_minus_source_s=best['offset_ns']/1e9,uncertainty_s=best['uncertainty_ns']/1e9,
 samples=rows,scope='host_clock_offset_not_headset_capture_latency'),indent=2))
