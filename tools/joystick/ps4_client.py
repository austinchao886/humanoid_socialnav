#!/usr/bin/env python3
"""Stream a Mac-connected DualShock 4 to the remote simulation bridge."""

from __future__ import annotations

import argparse
import socket
import sys
import time

from motion_pipeline.joystick import JoystickCommand, encode_line


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16042)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--deadzone", type=float, default=0.08)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--deadman-button", type=int, default=9, help="L1 on DS4")
    parser.add_argument("--estop-button", type=int, default=6, help="Options on DS4")
    args = parser.parse_args()

    try:
        import pygame
    except ImportError:
        raise SystemExit("pygame is required on the Mac: python3 -m pip install pygame")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.device_index:
        raise SystemExit("no PS4 controller found; pair it in macOS Bluetooth first")
    joystick = pygame.joystick.Joystick(args.device_index)
    joystick.init()
    if joystick.get_numaxes() < 4:
        raise SystemExit(f"controller exposes only {joystick.get_numaxes()} axes")
    print(f"[ps4-client] using {joystick.get_name()}", flush=True)

    period = 1.0 / args.rate
    sequence = 0
    with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
        connection.settimeout(None)
        print(f"[ps4-client] connected to {args.host}:{args.port}", flush=True)
        while True:
            started = time.monotonic()
            pygame.event.pump()

            def axis(index: int) -> float:
                value = _clamp(joystick.get_axis(index))
                return 0.0 if abs(value) < args.deadzone else value

            command = JoystickCommand(
                sequence=sequence,
                deadman=bool(joystick.get_button(args.deadman_button)),
                emergency_stop=bool(joystick.get_button(args.estop_button)),
                left_x=axis(0),
                left_y=axis(1),
                right_x=axis(2),
                right_y=axis(3),
            )
            connection.sendall(encode_line(command))
            if sequence % int(max(1.0, args.rate)) == 0:
                print(
                    f"[ps4-client] deadman={command.deadman} "
                    f"left=({command.left_x:+.2f},{command.left_y:+.2f})",
                    flush=True,
                )
            sequence += 1
            time.sleep(max(0.0, period - (time.monotonic() - started)))


if __name__ == "__main__":
    try:
        main()
    except (BrokenPipeError, ConnectionError, KeyboardInterrupt) as exc:
        print(f"[ps4-client] stopped: {exc}", file=sys.stderr)
