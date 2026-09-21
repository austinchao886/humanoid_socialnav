"""Admission tests: invalid references must never reach the controller publisher."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from motion_contracts.validator import SONIC_OFFICIAL_NEUTRAL
from sonic_tracker import pico_live_session as live

class AdmissionTests(unittest.TestCase):
    def reference(self):
        return dict(session_id='source',produced_monotonic_s=100.,clock_verified=True,
                    host_sample_to_reference_ms=20.,clock_uncertainty_ms=12.,
                    joint_pos=SONIC_OFFICIAL_NEUTRAL.tolist(),body_quat=[1,0,0,0])

    def test_valid_reference(self):
        q,quat=live.validate_reference(self.reference(),100.01,'source')
        np.testing.assert_array_equal(q,SONIC_OFFICIAL_NEUTRAL)

    def test_reject_reference_before_publish(self):
        for field,value in [('session_id','reconnected'),('produced_monotonic_s',99.),
            ('clock_verified',False),('clock_uncertainty_ms',26.),
            ('host_sample_to_reference_ms',240.),('body_quat',[0,0,0,0]),
            ('joint_pos',[9.]*29),('joint_pos',[float('nan')]*29)]:
            with self.subTest(field=field,value=value):
                sample=self.reference();sample[field]=value
                with self.assertRaises(ValueError):live.validate_reference(sample,100.01,'source')

    def test_simulation_clock_uses_status_timestamps(self):
        before=dict(session_id='sim',updated_epoch_s=100.,simulation_time_s=20.)
        after=dict(session_id='sim',updated_epoch_s=102.,simulation_time_s=22.)
        self.assertEqual(live.realtime_factor(before,after),1.)
        after['session_id']='new'
        with self.assertRaises(ValueError):live.realtime_factor(before,after)
        after.update(session_id='sim',updated_epoch_s=100.)
        with self.assertRaises(ValueError):live.realtime_factor(before,after)

    def test_real_time_and_isolation_admission(self):
        env=dict(SONIC_DDS_DOMAIN='42',SONIC_INTERFACE='lo',
                 SONIC_LOWCMD_TOPIC='rt/socialnav_sim/g1/lowcmd',
                 SONIC_LOWSTATE_TOPIC='rt/socialnav_sim/g1/lowstate')
        state=dict(session_id='sim',state='INTERACTIVE')
        live.validate_simulation(env,state,'sim',1.)
        for rtf in [.474,.77,1.06,float('nan')]:
            with self.assertRaises(ValueError):live.validate_simulation(env,state,'sim',rtf)
        env['SONIC_DDS_DOMAIN']='0'
        with self.assertRaises(ValueError):live.validate_simulation(env,state,'sim',1.)

    def test_expired_arm_cannot_open_publisher(self):
        with tempfile.TemporaryDirectory() as directory:
            s=SimpleNamespace(runtime_dir=Path(directory))
            with patch.object(live,'Publisher') as publisher:
                live.execute(s,dict(action='arm',expires_epoch_s=time.time()-1))
                publisher.assert_not_called()
            report=json.loads((Path(directory)/'pico_live/session.json').read_text())
            self.assertEqual(report['state'],'REJECTED')

if __name__=='__main__':unittest.main()
