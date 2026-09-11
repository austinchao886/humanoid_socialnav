import json
from pathlib import Path

import numpy as np

from motion_contracts.artifact import (
    SONIC_OFFICIAL_NEUTRAL_ISAACLAB,
    convert_g1_qpos,
    convert_kimodo_qpos,
)
from motion_contracts.validator import validate_artifact


def make_qpos(path: Path, frames: int = 100):
    t = np.arange(frames) / 50.0
    qpos = np.zeros((frames, 36))
    qpos[:, 2] = 0.78
    qpos[:, 3] = 1.0
    qpos[:, 7:] = 0.05 * np.sin(t[:, None] * 2.0)
    np.savetxt(path, qpos, delimiter=",", header=",".join(str(i) for i in range(36)), comments="")


def test_convert_and_validate(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    manifest = convert_kimodo_qpos(source, artifact, request_id="r", motion_id="m", prompt="wave", seed=42, model="test", source_fps=50)
    result = validate_artifact(artifact)
    assert manifest["num_frames"] == 400
    assert manifest["execution_contract"]["asset"] == "sonic_official_g1"
    assert manifest["execution_contract"]["requires_bootstrap_root_support"] is True
    assert result.valid, result.errors


def test_nan_rejected(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    convert_kimodo_qpos(source, artifact, request_id="r", motion_id="m", prompt="wave", seed=42, model="test", source_fps=50)
    joint_pos = artifact / "joint_pos.csv"
    text = joint_pos.read_text().replace("0.0", "nan", 1)
    joint_pos.write_text(text)
    assert not validate_artifact(artifact).valid


def test_bad_quaternion_rejected(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    convert_kimodo_qpos(source, artifact, request_id="r", motion_id="m", prompt="wave", seed=42, model="test", source_fps=50)
    quat = np.zeros((400, 4))
    np.savetxt(artifact / "body_quat.csv", quat, delimiter=",", header="w,x,y,z", comments="")
    assert not validate_artifact(artifact).valid


def test_headerless_kimodo_csv_keeps_first_frame_and_resamples(tmp_path):
    source = tmp_path / "source.csv"
    qpos = np.zeros((4, 36))
    qpos[:, 2], qpos[:, 3] = 0.78, 1.0
    qpos[:, 7] = [0.1, 0.2, 0.3, 0.4]
    np.savetxt(source, qpos, delimiter=",")  # Official Kimodo output has no header.
    artifact = tmp_path / "motion"
    manifest = convert_kimodo_qpos(
        source, artifact, request_id="r", motion_id="m", prompt="wave", seed=42,
        model="test", source_fps=30, smoothing_sigma_frames=0,
    )
    converted = np.loadtxt(artifact / "joint_pos.csv", delimiter=",", skiprows=1)
    assert manifest["source_num_frames"] == 4
    assert np.allclose(converted[0], SONIC_OFFICIAL_NEUTRAL_ISAACLAB)
    assert np.isclose(converted[150, 0], 0.1)
    assert manifest["num_frames"] == 306


def test_video_contacts_are_resampled_and_neutral_padded(tmp_path):
    source = tmp_path / "source.csv"
    qpos = np.zeros((4, 36)); qpos[:, 2] = 0.76; qpos[:, 3] = 1.0
    np.savetxt(source, qpos, delimiter=",")
    contacts_source = tmp_path / "contacts.csv"
    np.savetxt(contacts_source, np.asarray([[1, 1], [1, 0], [0, 1], [1, 1]]),
               delimiter=",", fmt="%d", header="left_contact,right_contact", comments="")
    artifact = tmp_path / "video-motion"
    manifest = convert_g1_qpos(
        source, artifact, request_id="v", motion_id="vm", source="video", model="gem",
        source_fps=30, smoothing_sigma_frames=0, source_contacts_csv=contacts_source,
    )
    contacts = np.loadtxt(artifact / "foot_contacts.csv", delimiter=",", skiprows=1)
    assert contacts.shape == (manifest["num_frames"], 2)
    assert np.all(contacts[:50] == 1) and np.all(contacts[-50:] == 1)


def test_joint_specific_position_limit_is_rejected(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    convert_kimodo_qpos(
        source, artifact, request_id="r", motion_id="m", prompt="wave",
        seed=42, model="test", source_fps=50,
    )
    values = np.loadtxt(artifact / "joint_pos.csv", delimiter=",", skiprows=1)
    values[10, 4] = 0.8  # right_hip_roll upper limit is 0.5236 rad.
    np.savetxt(
        artifact / "joint_pos.csv", values, delimiter=",",
        header=",".join(f"joint_{i}" for i in range(29)), comments="",
    )
    result = validate_artifact(artifact)
    assert not result.valid
    assert any("right_hip_roll_joint" in error for error in result.errors)


def test_endpoint_reset_discontinuity_is_rejected(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    convert_kimodo_qpos(
        source, artifact, request_id="r", motion_id="m", prompt="wave",
        seed=42, model="test", source_fps=50,
    )
    values = np.loadtxt(artifact / "joint_pos.csv", delimiter=",", skiprows=1)
    values[-1, 0] = values[0, 0] + 0.5
    np.savetxt(
        artifact / "joint_pos.csv", values, delimiter=",",
        header=",".join(f"joint_{i}" for i in range(29)), comments="",
    )
    result = validate_artifact(artifact)
    assert not result.valid
    assert any("playback reset" in error for error in result.errors)


def test_non_neutral_frame_zero_is_rejected(tmp_path):
    source = tmp_path / "source.csv"
    make_qpos(source)
    artifact = tmp_path / "motion"
    convert_kimodo_qpos(
        source, artifact, request_id="r", motion_id="m", prompt="wave",
        seed=42, model="test", source_fps=50,
    )
    values = np.loadtxt(artifact / "joint_pos.csv", delimiter=",", skiprows=1)
    values[0, 9] = 0.1
    np.savetxt(
        artifact / "joint_pos.csv", values, delimiter=",",
        header=",".join(f"joint_{i}" for i in range(29)), comments="",
    )
    result = validate_artifact(artifact)
    assert not result.valid
    assert any("frame 0" in error and "left_knee_joint" in error for error in result.errors)
