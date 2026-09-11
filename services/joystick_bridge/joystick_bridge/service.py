"""Receive loopback TCP joystick frames and republish them on simulator DDS."""

from __future__ import annotations

import argparse
import json
import os
import socket
import time
import uuid

from motion_pipeline.joystick import MAX_FRAME_BYTES, decode_line
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


def _published_payload(command, bridge_session: str) -> str:
    payload = command.to_wire_dict()
    payload.update(
        bridge_session=bridge_session,
        bridge_received_monotonic_ns=time.monotonic_ns(),
    )
    return json.dumps(payload, separators=(",", ":"))


def _neutral_payload(sequence: int, bridge_session: str) -> str:
    return json.dumps(
        {
            "version": 1,
            "sequence": sequence,
            "deadman": False,
            "emergency_stop": False,
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
            "l2": 0.0,
            "r2": 0.0,
            "bridge_session": bridge_session,
            "bridge_received_monotonic_ns": time.monotonic_ns(),
        },
        separators=(",", ":"),
    )


def serve(host: str, port: int, topic: str) -> None:
    domain = int(os.getenv("DDS_DOMAIN", "42"))
    interface = os.getenv("DDS_INTERFACE") or "lo"
    ChannelFactoryInitialize(domain, interface)
    publisher = ChannelPublisher(topic, String_)
    publisher.Init()
    bridge_session = uuid.uuid4().hex

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        print(
            f"[joystick-bridge] listening on {host}:{port}; DDS {topic} "
            f"domain={domain} interface={interface}",
            flush=True,
        )
        while True:
            connection, address = server.accept()
            print(f"[joystick-bridge] client connected: {address}", flush=True)
            last_sequence = -1
            try:
                with connection, connection.makefile("rb") as stream:
                    for line in stream:
                        if len(line) > MAX_FRAME_BYTES:
                            raise ValueError("joystick frame exceeds size limit")
                        command = decode_line(line)
                        if command.sequence <= last_sequence:
                            continue
                        last_sequence = command.sequence
                        publisher.Write(
                            String_(data=_published_payload(command, bridge_session))
                        )
            except (ConnectionError, OSError, ValueError) as exc:
                print(f"[joystick-bridge] client ended: {exc}", flush=True)
            finally:
                publisher.Write(
                    String_(data=_neutral_payload(last_sequence + 1, bridge_session))
                )
                print("[joystick-bridge] published neutral command", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16042)
    parser.add_argument(
        "--topic",
        default=os.getenv("SIM_JOYSTICK_TOPIC", "rt/motion/joystick/cmd"),
    )
    args = parser.parse_args()
    if args.host not in {"127.0.0.1", "::1", "localhost"}:
        parser.error("the simulation bridge must bind to loopback")
    serve(args.host, args.port, args.topic)


if __name__ == "__main__":
    main()
