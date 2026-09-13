"""Synthetic DDS fixtures distinguish velocity bias from physical motion."""
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

import pytest


@pytest.mark.parametrize('kind', ['center', 'release'])
@pytest.mark.parametrize('reported_velocity', [0.0, 1.5])
def test_position_derivative_is_not_reported_velocity(tmp_path, kind, reported_velocity):
    capture = tmp_path / 'capture.jsonl'
    rows = []
    for i in range(1101):
        t = i*.02
        moving = t < 1
        buttons = 128 if moving or kind == 'center' else 0
        remote = list(struct.pack('<2sH5f16s', bytes(2), buttons, 0, 0, 0, 0,
                                  .5 if moving else 0, bytes(16)))
        q = [0.0]*29
        dq = [0.0]*29
        q[11] = .01*math.sin(2*math.pi*2*t)
        dq[11] = reported_velocity  # Deliberately independent of derivative of q.
        rows.append({'kind':'state','epoch_s':t,'tick':i*4,'remote':remote,'q':q,'dq':dq})
        rows.append({'kind':'command','epoch_s':t,'q':[0.0]*29})
    capture.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    script = Path(__file__).resolve().parents[2] / 'tools/analysis/analyze_sim_stop.py'
    result = subprocess.run([sys.executable, str(script), str(capture)],
                            check=True, capture_output=True, text=True)
    report = json.loads(result.stdout)
    assert len(report['stops']) == 1
    assert report['stops'][0]['kind'] == kind
    if reported_velocity > .9:
        assert report['stops'][0]['joint_speed_settled_after_wall_s'] is None
    else:
        assert report['stops'][0]['joint_speed_settled_after_wall_s'] == pytest.approx(3.0)
    tail = report['stops'][0]['windows'][-1]
    assert tail['wall_window_s'] == [10, 20]
    assert tail['max_dq_p95'] == pytest.approx(reported_velocity)
    assert tail['max_position_dq_rms'] == pytest.approx(.0886, abs=.001)
    assert tail['physical_peak_joint'] == 11
    assert tail['tick_gap_max'] == 4


def test_recorder_has_only_simulation_subscribers():
    source = (Path(__file__).resolve().parents[2] / 'tools/analysis/record_sim_stop.py').read_text()
    assert 'ChannelPublisher' not in source
    assert "ChannelFactoryInitialize(42, 'lo')" in source
    assert "ChannelSubscriber('rt/socialnav_sim/g1/lowstate'" in source
    assert "ChannelSubscriber('rt/socialnav_sim/g1/lowcmd'" in source
    assert 'args.seconds <= 600' in source
