import json
import struct

import pytest

from motion_pipeline.joystick import (
    JoystickCommand,
    decode_line,
    encode_line,
    encode_unitree_remote,
)


def test_round_trip_and_unitree_mapping():
    command = JoystickCommand(7, True, False, 0.25, -0.75, -0.5, 0.4, 0.2, 0.8)
    assert decode_line(encode_line(command)) == command
    packet = encode_unitree_remote(command)
    assert len(packet) == 40
    _, buttons, lx, rx, ry, l2, ly, _ = struct.unpack("<2sH5f16s", packet)
    assert buttons & (1 << 7)
    assert buttons & (1 << 4)
    assert lx == pytest.approx(0.25)
    assert rx == pytest.approx(-0.5)
    assert ry == pytest.approx(-0.4)
    assert l2 == pytest.approx(0.2)
    assert ly == pytest.approx(0.75)


def test_invalid_axis_is_rejected():
    payload = JoystickCommand(1, False, False, 0, 0, 0, 0).to_wire_dict()
    payload["left_x"] = 2.0
    with pytest.raises(ValueError):
        decode_line(json.dumps(payload).encode())


def test_none_encodes_neutral_packet():
    assert encode_unitree_remote(None) == bytes(40)
