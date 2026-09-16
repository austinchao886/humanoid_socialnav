import fcntl
import io
import os
from pathlib import Path
import select
import sys
import unittest
import pexpect

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.gesture_transport import spawn_with_gesture_channel


class GestureTransportTests(unittest.TestCase):
    def test_private_channel_and_pty_output(self):
        log=io.StringIO()
        source='''import os,socket
s=socket.socket(fileno=int(os.environ["SONIC_GESTURE_FD"]))
print("receiver-ready",flush=True)
data=s.recv(1024)
s.send(data.upper())
print("receiver-done",flush=True)
'''
        child,channel=spawn_with_gesture_channel(sys.executable,["-c",source],
                                                encoding="utf-8",timeout=3)
        try:
            child.logfile=log
            child.expect_exact("receiver-ready")
            self.assertEqual(channel.send(b"private-message"),15)
            self.assertTrue(select.select([channel],[],[],3)[0])
            self.assertEqual(channel.recv(1024),b"PRIVATE-MESSAGE")
            child.expect_exact("receiver-done")
            child.expect(pexpect.EOF)
            self.assertIn("receiver-done",log.getvalue())
        finally:
            channel.close();child.close(force=True)

    def test_unrelated_descriptor_not_inherited(self):
        with open(os.devnull,"rb") as handle:
            sentinel=fcntl.fcntl(handle.fileno(),fcntl.F_DUPFD,100)
            os.set_inheritable(sentinel,True)
            try:
                code=f'''import os
try: os.fstat({sentinel}); print("leaked",flush=True)
except OSError: print("closed",flush=True)
'''
                child,channel=spawn_with_gesture_channel(sys.executable,["-c",code],
                                                        encoding="utf-8",timeout=3)
                try:
                    child.expect_exact("closed");child.expect(pexpect.EOF)
                finally:channel.close();child.close(force=True)
            finally:os.close(sentinel)

    def test_failed_spawn_closes_endpoints(self):
        before=set(os.listdir("/proc/self/fd"))
        with self.assertRaises(pexpect.ExceptionPexpect):
            spawn_with_gesture_channel("/no-such-sonic-test-executable",[])
        self.assertEqual(set(os.listdir("/proc/self/fd")),before)


if __name__=="__main__":unittest.main()
