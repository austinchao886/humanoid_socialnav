"""Run ten bounded simulator round trips, preserving one Isaac session.

Run from motion_pipeline on the simulator host. A PS4 client must be disconnected.
The report directory is printed at startup; individual runs retain full output.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

repo = Path(__file__).resolve().parents[2]
exchange = Path(os.environ.get('MOTION_EXCHANGE', str(repo.parent / 'motion_exchange')))
initial = json.loads((exchange / '.runtime/isaac_status.json').read_text())
assert initial['state'] == 'INTERACTIVE' and time.time()-initial['updated_epoch_s'] < 3
connections = subprocess.check_output(['ss','-tn'],text=True)
assert not any('16042' in line and 'ESTAB' in line for line in connections.splitlines()), 'Disconnect PS4 client first'
report = repo / '.build' / time.strftime('runtime-acceptance-%Y%m%dT%H%M%S')
report.mkdir(parents=True)
session = initial['session_id']
print('REPORT', report, 'SESSION',session,flush=True)
results = []
sources = [('phone-action1-r4-20260825','phone-action1-r4-20260825-a42efb29'),
           ('wave-official-e2e-005','wave-official-e2e-005-083aa901')]
env = dict(os.environ, PYTHONPATH=str(repo), MOTION_EXCHANGE=str(exchange),EXPECTED_SESSION=session)
for index in range(10):
    request,motion = sources[index%2]
    env['ACCEPTANCE_TURN'] = '0.15' if index in (3,7) else '0'
    print('START',index+1,motion,'turn',env['ACCEPTANCE_TURN'],flush=True)
    error = None
    with (report / f'round-{index+1:02}.log').open('w') as log:
        try:
            proc = subprocess.run([sys.executable,str(repo/'tools/acceptance/persistent_runtime_round_trip.py'),request,motion],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=150)
            passed = proc.returncode == 0
        except subprocess.TimeoutExpired:
            # Closing the test socket also expires the bridge deadman input.
            passed = False
            error = 'round exceeded 150s wall-clock limit'
    row = dict(round=index+1,motion_id=motion,turn=float(env['ACCEPTANCE_TURN']),passed=passed,error=error)
    results.append(row)
    (report/'summary.json').write_text(json.dumps(dict(session_id=session,rounds=results),indent=2)+'\n')
    print('RESULT',json.dumps(row),flush=True)
    if not passed:
        print((report/f'round-{index+1:02}.log').read_text()[-6000:],flush=True)
        raise SystemExit(1)
print('TEN_ROUNDS_PASS',flush=True)
