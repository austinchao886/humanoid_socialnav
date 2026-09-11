import json

import pytest

from motion_pipeline.asset_profile import (
    UNITREE_G1_MOTOR_JOINTS,
    load_asset_profile,
)


def _write_profile(path, *, qualification="qualified"):
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile_id": path.stem,
                "isaac_task": "Task",
                "source_asset": "asset.usd",
                "qualification": qualification,
                "physics_dt_s": 0.005,
                "allowed_extra_joints": ["hand"],
                "contract_to_asset_joint": {
                    name: name for name in UNITREE_G1_MOTOR_JOINTS
                },
                "joint_axis_sign": {
                    name: 1 for name in UNITREE_G1_MOTOR_JOINTS
                },
            }
        )
    )


def test_profile_checks_task_and_exact_extra_joint_contract(tmp_path):
    _write_profile(tmp_path / "test.json")
    profile = load_asset_profile("test", tmp_path)
    profile.assert_task("Task")
    profile.assert_articulation(list(UNITREE_G1_MOTOR_JOINTS) + ["hand"])
    assert profile.qualified
    assert set(profile.joint_effort_scale) == set(UNITREE_G1_MOTOR_JOINTS)
    assert set(profile.joint_effort_scale.values()) == {1.0}
    assert profile.body_mass_override_kg == {}
    assert profile.solver_position_iteration_count == 8
    assert profile.solver_velocity_iteration_count == 4
    with pytest.raises(ValueError, match="requires task"):
        profile.assert_task("Other")
    with pytest.raises(ValueError, match="extra-joint mismatch"):
        profile.assert_articulation(list(UNITREE_G1_MOTOR_JOINTS))


def test_profile_rejects_incomplete_sign_map(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["joint_axis_sign"].pop("left_knee_joint")
    (tmp_path / "test.json").write_text(json.dumps(value))
    with pytest.raises(ValueError, match="exactly the 29"):
        load_asset_profile("test", tmp_path)


def test_pending_profile_is_not_qualified(tmp_path):
    _write_profile(tmp_path / "test.json", qualification="pending_calibration")
    assert not load_asset_profile("test", tmp_path).qualified


def test_profile_loads_explicit_joint_effort_scale(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["joint_effort_scale"] = {
        name: (1.12 if name == "waist_pitch_joint" else 1.0)
        for name in UNITREE_G1_MOTOR_JOINTS
    }
    (tmp_path / "test.json").write_text(json.dumps(value))
    profile = load_asset_profile("test", tmp_path)
    assert profile.joint_effort_scale["waist_pitch_joint"] == pytest.approx(1.12)


def test_profile_rejects_unsafe_joint_effort_scale(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["joint_effort_scale"] = {
        name: (1.51 if name == "waist_pitch_joint" else 1.0)
        for name in UNITREE_G1_MOTOR_JOINTS
    }
    (tmp_path / "test.json").write_text(json.dumps(value))
    with pytest.raises(ValueError, match=r"within \[0.5, 1.5\]"):
        load_asset_profile("test", tmp_path)


def test_profile_loads_body_mass_overrides(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["body_mass_override_kg"] = {
        "imu_in_pelvis": 0.001,
        "left_hand_base_link": 0.536176,
    }
    (tmp_path / "test.json").write_text(json.dumps(value))
    profile = load_asset_profile("test", tmp_path)
    assert profile.body_mass_override_kg == {
        "imu_in_pelvis": pytest.approx(0.001),
        "left_hand_base_link": pytest.approx(0.536176),
    }


def test_profile_rejects_unsafe_body_mass_override(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["body_mass_override_kg"] = {"imu_in_pelvis": 0.0}
    (tmp_path / "test.json").write_text(json.dumps(value))
    with pytest.raises(ValueError, match=r"within \[0.0001, 20.0\]"):
        load_asset_profile("test", tmp_path)


def test_profile_loads_solver_iterations(tmp_path):
    _write_profile(tmp_path / "test.json")
    value = json.loads((tmp_path / "test.json").read_text())
    value["solver_position_iteration_count"] = 4
    value["solver_velocity_iteration_count"] = 1
    (tmp_path / "test.json").write_text(json.dumps(value))
    profile = load_asset_profile("test", tmp_path)
    assert profile.solver_position_iteration_count == 4
    assert profile.solver_velocity_iteration_count == 1


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("solver_position_iteration_count", 3),
        ("solver_position_iteration_count", 9),
        ("solver_velocity_iteration_count", 0),
        ("solver_velocity_iteration_count", 5),
    ],
)
def test_profile_rejects_unsafe_solver_iterations(tmp_path, field, value):
    _write_profile(tmp_path / "test.json")
    profile_value = json.loads((tmp_path / "test.json").read_text())
    profile_value[field] = value
    (tmp_path / "test.json").write_text(json.dumps(profile_value))
    with pytest.raises(ValueError, match=field):
        load_asset_profile("test", tmp_path)
