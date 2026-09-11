# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
G1 robot DDS communication class
Handle the state publishing and command receiving of the G1 robot
"""

import numpy as np
import os
import time
from typing import Any, Dict, Optional
# from dds.dds_base import BaseDDSNode, node_manager
from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowState_, LowCmd_
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.utils.crc import CRC


class G1RobotDDS(DDSObject):
    """G1 robot DDS communication class - singleton pattern

    Features:
    - Publish simulated state on a simulation-only DDS topic
    - Receive simulated commands on a simulation-only DDS topic
    """

    def __init__(self,node_name:str="g1_robot"):
        """Initialize the G1 robot DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return

        super().__init__()
        self.node_name = node_name
        self.crc = CRC()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.torso_imu_state = unitree_hg_msg_dds__IMUState_()
        self.lowstate_topic = os.getenv(
            "SIM_LOWSTATE_TOPIC", "rt/socialnav_sim/g1/lowstate"
        )
        self.lowcmd_topic = os.getenv(
            "SIM_LOWCMD_TOPIC", "rt/socialnav_sim/g1/lowcmd"
        )
        self.secondary_imu_topic = os.getenv(
            "SIM_SECONDARY_IMU_TOPIC", "rt/socialnav_sim/g1/secondary_imu"
        )
        self._reported_command_write_failure = False
        # The DDS publisher runs on wall time while Isaac advances on physics
        # time.  A slow GUI/physics step must not be republished hundreds of
        # times with the same tick: SONIC treats every LowState sample as work,
        # and duplicate reliable samples can fill the reader queue and starve
        # the Isaac Python thread.  Publish each simulator state at most once.
        self._last_published_sim_step = None
        if self.lowcmd_topic == "rt/lowcmd":
            raise RuntimeError(
                "Refusing to use the physical G1 rt/lowcmd topic in simulator mode"
            )
        self._initialized = True

        # setup the shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_robot_state",  # read the state of the G1 robot from Isaac Lab
            # A full 29-DOF LowCmd contains five arrays (q, dq, tau, kp,
            # kd).  Once SONIC leaves its all-zero initialization command,
            # JSON serialization is commonly 4-7 KiB.  The old 3072-byte
            # segment silently retained the short initialization command and
            # made the action provider report a false DDS timeout.  Use a new
            # segment name so a stale 3072-byte POSIX segment from an older
            # simulator process cannot be reopened with the wrong capacity.
            output_shm_name="dds_robot_cmd_v2",  # output the command to Isaac Lab
            input_size=3072,
            output_size=16384  # full precision 29-DOF LowCmd JSON + headroom
        )

        print(f"[{self.node_name}] G1 robot DDS node initialized")

    def setup_publisher(self) -> bool:
        """Setup the publisher of the G1 robot"""
        try:
            self.publisher = ChannelPublisher(self.lowstate_topic, LowState_)
            self.publisher.Init()
            self.torso_imu_publisher = ChannelPublisher(
                self.secondary_imu_topic, IMUState_
            )
            self.torso_imu_publisher.Init()
            print(f"[{self.node_name}] State publisher initialized ({self.lowstate_topic})")
            print(
                f"[{self.node_name}] Torso IMU publisher initialized "
                f"({self.secondary_imu_topic})"
            )
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] State publisher initialization failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the G1 robot"""
        try:
            print(f"[{self.node_name}] Create ChannelSubscriber...")
            self.subscriber = ChannelSubscriber(self.lowcmd_topic, LowCmd_)
            self.subscriber.Init(lambda msg: self.dds_subscriber(msg, ""), 32)
            print(f"[{self.node_name}] Command subscriber initialized ({self.lowcmd_topic})")
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Command subscriber initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def dds_publisher(self) -> Any:
        """Convert Isaac Lab state to DDS message and publish."""
        try:
            data = self.input_shm.read_data()
            if data is None:
                return

            sim_step = data.get("sim_step")
            if sim_step is not None:
                sim_step = int(sim_step) & 0xFFFFFFFF
                if sim_step == self._last_published_sim_step:
                    return

            motor_state = self.low_state.motor_state
            imu_state = self.low_state.imu_state
            num_motors =len(motor_state)

            positions = data.get("joint_positions")
            velocities = data.get("joint_velocities")
            torques = data.get("joint_torques")

            if positions and velocities and torques:
                q_array = np.asarray(positions, dtype=np.float32)
                dq_array = np.asarray(velocities, dtype=np.float32)
                tau_array = np.asarray(torques, dtype=np.float32)
                for i in range(len(q_array)):
                    motor = motor_state[i]
                    motor.q = q_array[i]
                    motor.dq = dq_array[i]
                    motor.tau_est = tau_array[i]

            imu = data.get("imu_data")
            if imu and len(imu) >= 13:
                imu_array = np.asarray(imu, dtype=np.float32)

                # get_robot_imu_data() already returns Unitree/SONIC ordering:
                # [position xyz, quaternion wxyz, accel xyz, gyro xyz].
                imu_state.quaternion[:] = imu_array[3:7]

                imu_state.accelerometer[:] = imu_array[7:10]

                imu_state.gyroscope[:] = imu_array[10:13]

            # In simulator mode the LowState tick is the authoritative physics
            # step, not a publisher-thread counter.  The DDS publisher may run
            # many times while Isaac is still rendering one physics step; using
            # that wall-clock publish rate makes SONIC advance a 50 Hz reference
            # faster than simulation time.  Keep the legacy counter as a
            # fallback for callers that do not yet provide a simulation step.
            if sim_step is None:
                self.low_state.tick += 1
            else:
                self.low_state.tick = sim_step
            self.low_state.crc = self.crc.Crc(self.low_state)
            self.publisher.Write(self.low_state)

            torso_imu = data.get("torso_imu_data")
            if torso_imu and len(torso_imu) >= 13:
                torso_array = np.asarray(torso_imu, dtype=np.float32)
                self.torso_imu_state.quaternion[:] = torso_array[3:7]
                self.torso_imu_state.accelerometer[:] = torso_array[7:10]
                self.torso_imu_state.gyroscope[:] = torso_array[10:13]
                self.torso_imu_publisher.Write(self.torso_imu_state)

            if sim_step is not None:
                self._last_published_sim_step = sim_step

        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing publish data: {e}")


    def dds_subscriber(self, msg: LowCmd_,datatype:str=None) -> Dict[str, Any]:
        """Process the subscribe data: convert the DDS command to the Isaac Lab format

        Return data format:
        {
            "mode_pr": int,
            "mode_machine": int,
            "motor_cmd": {
                "positions": [29 joint position commands],
                "velocities": [29 joint velocity commands],
                "torques": [29 joint torque commands],
                "kp": [29 position gains],
                "kd": [29 speed gains]
            }
        }
        """
        try:
            # verify the CRC
            if self.crc.Crc(msg) != msg.crc:
                print(f"g1_robot_dds [{self.node_name}] Warning: CRC verification failed!")
                return {}

            # extract the command data
            num_cmd_motors = len(msg.motor_cmd)
            cmd_data = {
                "received_monotonic": time.monotonic(),
                "mode_pr": int(msg.mode_pr),
                "mode_machine": int(msg.mode_machine),
                "motor_cmd": {
                    "positions": [float(msg.motor_cmd[i].q) for i in range(num_cmd_motors)],
                    "velocities": [float(msg.motor_cmd[i].dq) for i in range(num_cmd_motors)],
                    "torques": [float(msg.motor_cmd[i].tau) for i in range(num_cmd_motors)],
                    "kp": [float(msg.motor_cmd[i].kp) for i in range(num_cmd_motors)],
                    "kd": [float(msg.motor_cmd[i].kd) for i in range(num_cmd_motors)]
                }
            }
            if not self.output_shm.write_data(cmd_data):
                if not self._reported_command_write_failure:
                    print(
                        f"g1_robot_dds [{self.node_name}] Error: LowCmd did not fit "
                        "in command shared memory; retaining the previous command"
                    )
                    self._reported_command_write_failure = True
            else:
                self._reported_command_write_failure = False

        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing subscribe data: {e}")
            return {}

    def get_robot_command(self) -> Optional[Dict[str, Any]]:
        """Get the robot control command

        Returns:
            Dict: the robot control command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None

    def write_robot_state(
        self,
        joint_positions,
        joint_velocities,
        joint_torques,
        imu_data,
        torso_imu_data=None,
        sim_step=None,
    ):
        """Write the robot state to the shared memory

        Args:
            joint_positions: the joint position list or torch.Tensor
            joint_velocities: the joint velocity list or torch.Tensor
            joint_torques: the joint torque list or torch.Tensor
            imu_data: the IMU data list or torch.Tensor
            torso_imu_data: independent torso IMU data for rt/secondary_imu
            sim_step: Isaac physics-step counter used to synchronize SONIC
        """
        if self.input_shm is None:
            return
        try:
            state_data = {
                "joint_positions": joint_positions.tolist() if hasattr(joint_positions, 'tolist') else joint_positions,
                "joint_velocities": joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else joint_velocities,
                "joint_torques": joint_torques.tolist() if hasattr(joint_torques, 'tolist') else joint_torques,
                "imu_data": imu_data.tolist() if hasattr(imu_data, 'tolist') else imu_data,
                "torso_imu_data": (
                    torso_imu_data.tolist()
                    if hasattr(torso_imu_data, "tolist")
                    else torso_imu_data
                ),
                "sim_step": None if sim_step is None else int(sim_step),
            }
            self.input_shm.write_data(state_data)
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error writing robot state: {e}")
