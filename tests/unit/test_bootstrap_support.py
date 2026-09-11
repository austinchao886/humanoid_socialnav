import pytest

from isaac_runtime.bootstrap_support import (
    bootstrap_controller_is_quiet,
    bootstrap_root_is_stable,
    elastic_support_scale,
    resolve_elastic_target_height,
)


def test_automatic_target_stays_at_spawn_until_reference_is_loaded():
    assert resolve_elastic_target_height(None, 0.76) == 0.76
    assert resolve_elastic_target_height(None, 0.76, 0.79) == 0.79


def test_explicit_target_has_priority_for_upstream_reproduction():
    assert resolve_elastic_target_height(1.0, 0.76, 0.79) == 1.0


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_explicit_target_is_rejected(value):
    with pytest.raises(ValueError, match="finite and positive"):
        resolve_elastic_target_height(value, 0.76, 0.79)


def test_bootstrap_root_stability_uses_physical_state():
    values = {
        "root_height_m": 0.755,
        "target_height_m": 0.76,
        "root_tilt_rad": 0.04,
        "root_vertical_velocity_m_s": 0.02,
        "max_root_height_error_m": 0.05,
        "max_root_tilt_rad": 0.15,
        "max_root_vertical_velocity_m_s": 0.15,
    }
    assert bootstrap_root_is_stable(**values)


def test_bootstrap_root_stability_rejects_each_unsafe_root_metric():
    baseline = {
        "root_height_m": 0.76,
        "target_height_m": 0.76,
        "root_tilt_rad": 0.0,
        "root_vertical_velocity_m_s": 0.0,
        "max_root_height_error_m": 0.05,
        "max_root_tilt_rad": 0.15,
        "max_root_vertical_velocity_m_s": 0.15,
    }
    unsafe = {
        "root_height_m": 0.60,
        "root_tilt_rad": 0.20,
        "root_vertical_velocity_m_s": 0.20,
    }
    for field, value in unsafe.items():
        assert not bootstrap_root_is_stable(**{**baseline, field: value}), field


def test_bootstrap_root_stability_rejects_non_finite_values():
    assert not bootstrap_root_is_stable(
        root_height_m=float("nan"),
        target_height_m=0.76,
        root_tilt_rad=0.0,
        root_vertical_velocity_m_s=0.0,
        max_root_height_error_m=0.05,
        max_root_tilt_rad=0.15,
        max_root_vertical_velocity_m_s=0.15,
    )


def test_elastic_support_scale_fades_linearly_and_clamps():
    assert elastic_support_scale(0.0, 1.0) == 1.0
    assert elastic_support_scale(0.25, 1.0) == 0.75
    assert elastic_support_scale(1.0, 1.0) == 0.0
    assert elastic_support_scale(2.0, 1.0) == 0.0
    assert elastic_support_scale(0.0, 0.0) == 0.0


@pytest.mark.parametrize(
    ("elapsed_s", "duration_s"),
    [(-0.1, 1.0), (float("nan"), 1.0), (0.0, -1.0)],
)
def test_elastic_support_scale_rejects_invalid_time(elapsed_s, duration_s):
    with pytest.raises(ValueError):
        elastic_support_scale(elapsed_s, duration_s)


def test_controller_quiet_requires_completed_handoff_and_all_metrics():
    baseline = {
        "max_joint_velocity_rad_s": 0.4,
        "max_target_rate_rad_s": 2.0,
        "max_torque_ratio": 0.6,
        "handoff_progress": 1.0,
        "joint_velocity_limit_rad_s": 1.0,
        "target_rate_limit_rad_s": 5.0,
        "torque_ratio_limit": 0.9,
    }
    assert bootstrap_controller_is_quiet(**baseline)
    unsafe = {
        "max_joint_velocity_rad_s": 1.1,
        "max_target_rate_rad_s": 5.1,
        "max_torque_ratio": 0.91,
        "handoff_progress": 0.99,
    }
    for field, value in unsafe.items():
        assert not bootstrap_controller_is_quiet(
            **{**baseline, field: value}
        ), field
