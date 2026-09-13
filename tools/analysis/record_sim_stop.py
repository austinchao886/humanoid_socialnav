"""Read-only DDS capture on hard-coded simulation topics; never publishes."""
import argparse
import json
from pathlib import Path
import queue
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=200)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if not 1 <= args.seconds <= 600:
    parser.error('seconds must be within [1, 600]')
ChannelFactoryInitialize(42, 'lo')
events = queue.Queue(maxsize=8192)
dropped = [0]

def enqueue(row):
    try:
        events.put_nowait(row)
    except queue.Full:
        dropped[0] += 1

def state(message):
    enqueue({'kind': 'state', 'epoch_s': time.time(), 'tick': int(message.tick),
             'remote': list(message.wireless_remote),
             'q': [float(m.q) for m in message.motor_state[:29]],
             'dq': [float(m.dq) for m in message.motor_state[:29]],
             'quat': list(message.imu_state.quaternion)})

def command(message):
    enqueue({'kind': 'command', 'epoch_s': time.time(),
             'q': [float(m.q) for m in message.motor_cmd[:29]],
             'dq': [float(m.dq) for m in message.motor_cmd[:29]],
             'kp': [float(m.kp) for m in message.motor_cmd[:29]],
             'kd': [float(m.kd) for m in message.motor_cmd[:29]]})

subscribers = [ChannelSubscriber('rt/socialnav_sim/g1/lowstate', LowState_),
               ChannelSubscriber('rt/socialnav_sim/g1/lowcmd', LowCmd_)]
subscribers[0].Init(state, 32)
subscribers[1].Init(command, 32)
deadline = time.monotonic()+args.seconds
counts = {'state': 0, 'command': 0}
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open('x') as stream:
    while time.monotonic() < deadline:
        try:
            row = events.get(timeout=.1)
        except queue.Empty:
            continue
        stream.write(json.dumps(row, separators=(',', ':'))+'\n')
        counts[row['kind']] += 1
print(json.dumps({'output': str(args.output), 'counts': counts, 'queue_drops': dropped[0]}), flush=True)
if dropped[0] or not all(counts.values()):
    raise SystemExit('incomplete capture')
