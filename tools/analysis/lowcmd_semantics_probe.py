#!/usr/bin/env python3
"""Probe Unitree LowCmd joint semantics on simulation-only DDS topics."""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from pathlib import Path

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

ARMATURE_5020 = 0.003609725
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925
ARMATURE_4010 = 0.00425
NATURAL_FREQ = 10.0 * 2.0 * math.pi
DAMPING_RATIO = 2.0


def gains(armature: float, multiplier: float = 1.0) -> tuple[float, float]:
    return (
        multiplier * armature * NATURAL_FREQ**2,
        multiplier * 2.0 * DAMPING_RATIO * armature * NATURAL_FREQ,
    )


def joint_gains(name: str) -> tuple[float, float]:
    if "ankle" in name:
        return gains(ARMATURE_5020, 2.0)
    if "hip_pitch" in name or "hip_roll" in name or "knee" in name:
        return gains(ARMATURE_7520_22)
    if "hip_yaw" in name or name == "waist_yaw_joint":
        return gains(ARMATURE_7520_14)
    if name in {"waist_roll_joint", "waist_pitch_joint"}:
        return gains(ARMATURE_5020, 2.0)
    if "wrist_pitch" in name or "wrist_yaw" in name:
        return gains(ARMATURE_4010)
    return gains(ARMATURE_5020)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--interface", default="wlp69s0")
    parser.add_argument("--lowstate-topic", default="rt/socialnav_sim/g1/lowstate")
    parser.add_argument("--lowcmd-topic", default="rt/socialnav_sim/g1/lowcmd")
    parser.add_argument("--joint", choices=JOINTS, default="right_wrist_roll_joint")
    parser.add_argument("--amplitude", type=float, default=0.05)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--step-duration", type=float, default=1.0)
    parser.add_argument("--recovery", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument(
        "--physics-dt",
        type=float,
        default=0.005,
        help="Seconds represented by one LowState tick",
    )
    parser.add_argument(
        "--clock",
        choices=("lowstate_tick", "wall"),
        default="lowstate_tick",
        help=(
            "Phase clock. Isaac publishes physics steps in LowState.tick; the "
            "official MuJoCo bridge uses a publisher counter and must use wall time."
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--isolated-container-network",
        action="store_true",
        help=(
            "Permit standard Unitree topic names only inside a Docker network "
            "with no host networking; never use this on a host or physical-LAN container"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.lowcmd_topic in {"rt/lowcmd", "rt/arm_sdk"}
        and not args.isolated_container_network
    ):
        raise RuntimeError("refusing to publish a diagnostic command on a physical G1 topic")
    if abs(args.amplitude) > 0.1:
        raise ValueError("probe amplitude must be <= 0.1 rad")

    ChannelFactoryInitialize(args.domain, args.interface)
    latest: LowState_ | None = None
    lock = threading.Lock()

    def receive(message: LowState_) -> None:
        nonlocal latest
        with lock:
            latest = message

    subscriber = ChannelSubscriber(args.lowstate_topic, LowState_)
    subscriber.Init(receive, 32)
    publisher = ChannelPublisher(args.lowcmd_topic, LowCmd_)
    publisher.Init()

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        with lock:
            state = latest
        if state is not None:
            break
        time.sleep(0.01)
    else:
        raise RuntimeError("timed out waiting for LowState")

    initial_q = [float(state.motor_state[index].q) for index in range(29)]
    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_pr = 0
    command.mode_machine = int(state.mode_machine)
    crc = CRC()
    for index, name in enumerate(JOINTS):
        kp, kd = joint_gains(name)
        motor = command.motor_cmd[index]
        motor.mode = 1
        motor.tau = 0.0
        motor.q = initial_q[index]
        motor.dq = 0.0
        motor.kp = kp
        motor.kd = kd

    args.output.parent.mkdir(parents=True, exist_ok=True)
    joint_index = JOINTS.index(args.joint)
    total = args.warmup + args.step_duration + args.recovery
    period = 1.0 / args.rate
    started = time.monotonic()
    start_tick = int(state.tick)
    next_deadline = started
    rows: list[list[float]] = []
    while True:
        elapsed_wall = time.monotonic() - started
        with lock:
            sample = latest
        if sample is None:
            elapsed_sim = 0.0
        else:
            elapsed_sim = ((int(sample.tick) - start_tick) & 0xFFFFFFFF) * args.physics_dt
        phase_time = elapsed_sim if args.clock == "lowstate_tick" else elapsed_wall
        if phase_time >= total:
            break
        if args.clock == "lowstate_tick" and elapsed_wall > max(30.0, total * 30.0):
            raise RuntimeError(
                f"timed out after {elapsed_wall:.1f}s with only {elapsed_sim:.3f}s simulation time"
            )
        stepped = args.warmup <= phase_time < args.warmup + args.step_duration
        target = initial_q[joint_index] + (args.amplitude if stepped else 0.0)
        command.motor_cmd[joint_index].q = target
        command.crc = crc.Crc(command)
        publisher.Write(command)
        if sample is not None:
            motor = sample.motor_state[joint_index]
            rows.append(
                [
                    elapsed_wall,
                    elapsed_sim,
                    float(sample.tick),
                    target,
                    float(motor.q),
                    float(motor.dq),
                    float(motor.tau_est),
                ]
            )
        next_deadline += period
        delay = next_deadline - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)

    with args.output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ("elapsed_wall_s", "elapsed_sim_s", "lowstate_tick", "q_des", "q", "dq", "tau_est")
        )
        writer.writerows(rows)
    print(f"wrote {len(rows)} samples to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
