"""Bootstrap-root support contract shared by the Isaac execution runner."""

from __future__ import annotations

import math


def elastic_support_scale(elapsed_s: float, duration_s: float) -> float:
    """Return a linear 1 -> 0 support fade with validated finite inputs."""

    elapsed_s = float(elapsed_s)
    duration_s = float(duration_s)
    if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError("support fade elapsed time must be finite and non-negative")
    if not math.isfinite(duration_s) or duration_s < 0.0:
        raise ValueError("support fade duration must be finite and non-negative")
    if duration_s == 0.0:
        return 0.0
    return max(0.0, min(1.0, 1.0 - elapsed_s / duration_s))


def bootstrap_controller_is_quiet(
    *,
    max_joint_velocity_rad_s: float,
    max_target_rate_rad_s: float,
    max_torque_ratio: float,
    handoff_progress: float,
    joint_velocity_limit_rad_s: float,
    target_rate_limit_rad_s: float,
    torque_ratio_limit: float,
) -> bool:
    """Return whether the whole-body controller is quiet for release counting.

    Absolute LowCmd position error is intentionally absent: SONIC uses an
    action-derived PD offset to create balancing torque.  Velocity, target
    change and effort utilization instead prove that the controller has
    stopped injecting transient energy, and the check must hold continuously
    in the runner's rolling release window.
    """

    values = (
        max_joint_velocity_rad_s,
        max_target_rate_rad_s,
        max_torque_ratio,
        handoff_progress,
        joint_velocity_limit_rad_s,
        target_rate_limit_rad_s,
        torque_ratio_limit,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    if min(
        max_joint_velocity_rad_s,
        max_target_rate_rad_s,
        max_torque_ratio,
        joint_velocity_limit_rad_s,
        target_rate_limit_rad_s,
        torque_ratio_limit,
    ) < 0.0:
        return False
    return bool(
        handoff_progress >= 1.0
        and max_joint_velocity_rad_s <= joint_velocity_limit_rad_s
        and max_target_rate_rad_s <= target_rate_limit_rad_s
        and max_torque_ratio <= torque_ratio_limit
    )


def resolve_elastic_target_height(
    explicit_height: float | None,
    spawn_height: float,
    reference_height: float | None = None,
) -> float:
    """Choose a safe elastic target without moving the robot before approval.

    Before an approved reference is known, an automatic target stays at the
    loaded asset's spawn height.  Once the reference contract is loaded, its
    first root-height sample becomes authoritative.  An explicit CLI override
    remains available for reproducing an upstream diagnostic.
    """

    candidates = (
        ("explicit", explicit_height),
        ("reference", reference_height),
        ("spawn", spawn_height),
    )
    for source, value in candidates:
        if value is None:
            continue
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{source} elastic target height must be finite and positive")
        return value
    raise ValueError("elastic target height has no valid source")


def bootstrap_root_is_stable(
    *,
    root_height_m: float,
    target_height_m: float,
    root_tilt_rad: float,
    root_vertical_velocity_m_s: float,
    max_root_height_error_m: float,
    max_root_tilt_rad: float,
    max_root_vertical_velocity_m_s: float,
) -> bool:
    """Return whether root state is stable enough to count the release window.

    A SONIC ``LowCmd.q`` is an action-derived PD target, not a reference
    tracking target. Its instantaneous error is deliberately excluded here.
    Joint velocity remains an instantaneous release condition in the runner;
    brief policy-action spikes do not erase an otherwise stable root window.
    """

    values = (
        root_height_m,
        target_height_m,
        root_tilt_rad,
        root_vertical_velocity_m_s,
        max_root_height_error_m,
        max_root_tilt_rad,
        max_root_vertical_velocity_m_s,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    if min(
        max_root_height_error_m,
        max_root_tilt_rad,
        max_root_vertical_velocity_m_s,
    ) < 0.0:
        return False
    return bool(
        abs(root_height_m - target_height_m) <= max_root_height_error_m
        and root_tilt_rad <= max_root_tilt_rad
        and abs(root_vertical_velocity_m_s)
        <= max_root_vertical_velocity_m_s
    )
