import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import pexpect
from sonic_tracker.supervisor import SonicSupervisor


class ControllerOutputTests(unittest.TestCase):
    def test_standing_wait_keeps_verbose_controller_running(self):
        # A real PTY writer cannot reach its heartbeat until >1 MB of control
        # telemetry has been consumed. Simulate the physics gate waiting on it.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            marker = root / 'heartbeat'
            supervisor = SonicSupervisor(root, root, object())
            source = (
                "import os,time; from pathlib import Path; "
                "[(os.write(1,b'telemetry '*400+b'\\n')) for _ in range(300)]; "
                f"Path({str(marker)!r}).touch(); time.sleep(10)"
            )
            child = pexpect.spawn(sys.executable, ['-c', source], encoding='utf-8')
            supervisor.child = child
            def status():
                return dict(session_id='session', state='READY_STANDING',
                            root_height_m=.78, root_tilt_rad=.02,
                            max_joint_velocity_rad_s=.2 if marker.exists() else 2)
            try:
                with patch.object(supervisor, '_read_isaac_status', side_effect=status):
                    # Reproduce the original bug with the drain disabled.
                    with patch.object(supervisor, '_service_controller_output'):
                        with self.assertRaisesRegex(RuntimeError, 'stable standing'):
                            supervisor._wait_for_stable_standing(
                                'session', stable_duration=0, timeout=.4)
                    self.assertFalse(marker.exists())
                    supervisor._wait_for_stable_standing(
                        'session', stable_duration=.1, timeout=5)
                    self.assertTrue(marker.exists())
            finally:
                child.close(force=True)

    def test_split_safety_message_is_not_discarded(self):
        class Child:
            def __init__(self):
                self.chunks = iter(['Safety check ', 'failed: test'])
            def read_nonblocking(self, **kwargs):
                try:
                    return next(self.chunks)
                except StopIteration:
                    raise pexpect.TIMEOUT('empty')
        with tempfile.TemporaryDirectory() as directory:
            supervisor = SonicSupervisor(Path(directory), Path(directory), object())
            supervisor.child = Child()
            with self.assertRaisesRegex(RuntimeError, 'SONIC safety error'):
                supervisor._service_controller_output()


if __name__ == '__main__':
    unittest.main()
