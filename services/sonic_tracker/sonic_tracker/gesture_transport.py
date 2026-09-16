"""Supervisor-owned private IPC while retaining pexpect PTY logging/controls."""
import os
import socket
import pexpect


class DescriptorSpawn(pexpect.spawn):
    def __init__(self, *args, pass_fds=(), **kwargs):
        self._gesture_pass_fds = tuple(pass_fds)
        super().__init__(*args, **kwargs)

    def _spawnpty(self, args, **kwargs):
        kwargs["pass_fds"] = self._gesture_pass_fds
        return super()._spawnpty(args, **kwargs)


def spawn_with_gesture_channel(command, args, *, env=None, **kwargs):
    """Return (PTY child, supervisor socket); caller owns closing both.

    No authorization or feature enablement is implied. The private descriptor
    is a transport capability; Supervisor still must issue a scoped grant.
    Only the child's socket endpoint is inherited, not other open descriptors.
    """
    supervisor, child_socket = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    try:
        child_socket.set_inheritable(True)
        child_env = dict(os.environ if env is None else env)
        child_env["SONIC_GESTURE_FD"] = str(child_socket.fileno())
        child = DescriptorSpawn(command,args,env=child_env,
                                pass_fds=(child_socket.fileno(),),**kwargs)
        # Nonblocking writes let the sender detect backpressure, never stall
        # Supervisor's status/abort handling waiting for a full socket queue.
        supervisor.setblocking(False)
        return child, supervisor
    except BaseException:
        supervisor.close()
        raise
    finally:
        child_socket.close()
