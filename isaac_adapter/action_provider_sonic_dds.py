"""Physics-driven 29-DOF DDS action provider for GEAR-SONIC."""

import time
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from dds.dds_master import dds_manager


# Unitree G1 motor order used by rt/lowcmd and rt/lowstate.
G1_MOTOR_JOINTS = [
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


class SonicDDSActionProvider(ActionProvider):
    """Apply the complete SONIC low-level PD command as joint torques.

    This matches the official MuJoCo bridge:
    tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq).
    The task clamps the result through the articulation actuator effort limits.
    If DDS commands disappear, command zero torque rather than replaying stale
    targets indefinitely.
    """

    def __init__(self, env, args_cli):
        super().__init__("SonicDDSActionProvider")
        self.env = env
        self.robot_dds = dds_manager.get_object("g129")
        if self.robot_dds is None:
            raise RuntimeError("g129 DDS object is not registered")

        names = env.scene["robot"].data.joint_names
        profile = getattr(args_cli, "asset_profile_contract", None) or {}
        joint_map = profile.get("contract_to_asset_joint") or {
            name: name for name in G1_MOTOR_JOINTS
        }
        axis_signs = profile.get("joint_axis_sign") or {
            name: 1 for name in G1_MOTOR_JOINTS
        }
        mapped_names = [joint_map[name] for name in G1_MOTOR_JOINTS]
        missing = [name for name in mapped_names if name not in names]
        if missing:
            raise ValueError(f"Isaac G1 is missing SONIC joints: {missing}")
        self._target_indices = torch.tensor(
            [names.index(name) for name in mapped_names],
            dtype=torch.long,
            device=env.device,
        )
        self._axis_signs = torch.tensor(
            [axis_signs[name] for name in G1_MOTOR_JOINTS],
            dtype=torch.float32,
            device=env.device,
        )
        self._torque = torch.zeros_like(env.scene["robot"].data.default_joint_pos[0])
        self._default_joint_pos = env.scene["robot"].data.default_joint_pos[0].clone()
        # Equivalent to the official simulator's gantry/bootstrap phase: hold
        # the spawn pose until SONIC publishes its first complete LowCmd.
        # Match gear_sonic_deploy/policy_parameters.hpp exactly.  These gains
        # are evaluated at the 200 Hz physics-servo rate by the official
        # baseline task; the 50 Hz reference/policy rate is a separate layer.
        natural_freq = 10.0 * 2.0 * torch.pi
        damping_ratio = 2.0
        armature_5020 = 0.003609725
        armature_7520_14 = 0.010177520
        armature_7520_22 = 0.025101925
        armature_4010 = 0.00425

        def gains(armature, multiplier=1.0):
            kp = multiplier * armature * natural_freq**2
            kd = multiplier * 2.0 * damping_ratio * armature * natural_freq
            return kp, kd

        hold_kp = []
        hold_kd = []
        for name in G1_MOTOR_JOINTS:
            if "ankle" in name:
                kp, kd = gains(armature_5020, 2.0)
            elif "hip_pitch" in name or "hip_roll" in name or "knee" in name:
                kp, kd = gains(armature_7520_22)
            elif "hip_yaw" in name or name == "waist_yaw_joint":
                kp, kd = gains(armature_7520_14)
            elif name in {"waist_roll_joint", "waist_pitch_joint"}:
                kp, kd = gains(armature_5020, 2.0)
            elif "wrist_pitch" in name or "wrist_yaw" in name:
                kp, kd = gains(armature_4010)
            else:
                kp, kd = gains(armature_5020)
            hold_kp.append(kp)
            hold_kd.append(kd)
        self._hold_kp = torch.tensor(hold_kp, dtype=torch.float32, device=env.device)
        self._hold_kd = torch.tensor(hold_kd, dtype=torch.float32, device=env.device)
        # SharedMemoryManager re-opens named segments left by an earlier Isaac
        # process.  Never replay a LowCmd cached by a previous simulator run;
        # only commands received after this provider instance was created are
        # eligible.  SONIC publishes continuously, so a live controller will
        # provide a fresh sample on its next cycle.
        self._accept_commands_after = time.monotonic()
        self._last_fresh_cmd = 0.0
        self._last_q_des: torch.Tensor | None = None
        self._last_dq_des: torch.Tensor | None = None
        self._stale_timeout_s = float(getattr(args_cli, "sonic_command_timeout", 0.25))
        self._reported_stale = False
        self._reported_pre_session_cmd = False

    def get_action(self, env) -> Optional[torch.Tensor]:
        cmd = self.robot_dds.get_robot_command()
        now = time.monotonic()
        if not self._last_fresh_cmd:
            robot = env.scene["robot"].data
            q = robot.joint_pos[0].index_select(0, self._target_indices)
            dq = robot.joint_vel[0].index_select(0, self._target_indices)
            q_default = self._default_joint_pos.index_select(0, self._target_indices)
            hold_torque = self._hold_kp * (q_default - q) - self._hold_kd * dq
            self._torque.zero_()
            self._torque.index_copy_(0, self._target_indices, hold_torque)
        if cmd and "motor_cmd" in cmd:
            motor_cmd = cmd["motor_cmd"]
            positions = motor_cmd.get("positions", [])
            velocities = motor_cmd.get("velocities", [])
            feedforward = motor_cmd.get("torques", [])
            kp = motor_cmd.get("kp", [])
            kd = motor_cmd.get("kd", [])
            received_at = float(cmd.get("received_monotonic", 0.0))
            fields = (positions, velocities, feedforward, kp, kd)
            if received_at <= self._accept_commands_after:
                if received_at and not self._reported_pre_session_cmd:
                    print(
                        "[SonicDDSActionProvider] ignoring LowCmd cached by a previous simulator session"
                    )
                    self._reported_pre_session_cmd = True
            elif all(len(field) >= 29 for field in fields) and received_at > self._last_fresh_cmd:
                q_des, dq_des, tau_ff, kp_t, kd_t = (
                    torch.as_tensor(field[:29], dtype=torch.float32, device=env.device)
                    for field in fields
                )
                if all(torch.isfinite(field).all() for field in (q_des, dq_des, tau_ff, kp_t, kd_t)):
                    robot = env.scene["robot"].data
                    q_asset = robot.joint_pos[0].index_select(0, self._target_indices)
                    dq_asset = robot.joint_vel[0].index_select(0, self._target_indices)
                    q = self._axis_signs * q_asset
                    dq = self._axis_signs * dq_asset
                    motor_torque_contract = (
                        tau_ff + kp_t * (q_des - q) + kd_t * (dq_des - dq)
                    )
                    motor_torque_asset = self._axis_signs * motor_torque_contract
                    self._torque.zero_()
                    self._torque.index_copy_(
                        0, self._target_indices, motor_torque_asset
                    )
                    self._last_fresh_cmd = received_at
                    self._last_q_des = q_des.clone()
                    self._last_dq_des = dq_des.clone()
                    self._reported_stale = False

        if self._last_fresh_cmd and now - self._last_fresh_cmd > self._stale_timeout_s:
            self._torque.zero_()
            if not self._reported_stale:
                print("[SonicDDSActionProvider] lowcmd timeout; commanding zero torque")
                self._reported_stale = True
        return self._torque.unsqueeze(0)

    @property
    def has_fresh_command(self) -> bool:
        """Whether this simulator session has received a valid live LowCmd."""
        return self._last_fresh_cmd > self._accept_commands_after

    @property
    def command_age_s(self) -> float:
        """Wall-clock age of the last valid command, or infinity before one arrives."""
        if not self.has_fresh_command:
            return float("inf")
        return max(0.0, time.monotonic() - self._last_fresh_cmd)

    @property
    def command_is_stale(self) -> bool:
        return self.has_fresh_command and self.command_age_s > self._stale_timeout_s

    @property
    def target_indices(self) -> torch.Tensor:
        return self._target_indices

    @property
    def axis_signs(self) -> torch.Tensor:
        return self._axis_signs

    def actual_joint_positions(self) -> torch.Tensor:
        """Current position transformed from the asset into Unitree semantics."""
        values = self.env.scene["robot"].data.joint_pos[0].index_select(
            0, self._target_indices
        )
        return self._axis_signs * values

    def actual_joint_velocities(self) -> torch.Tensor:
        """Current velocity transformed from the asset into Unitree semantics."""
        values = self.env.scene["robot"].data.joint_vel[0].index_select(
            0, self._target_indices
        )
        return self._axis_signs * values

    @property
    def desired_joint_positions(self) -> torch.Tensor | None:
        return self._last_q_des

    def cleanup(self):
        # DDS lifecycle is owned by dds_manager.
        pass
