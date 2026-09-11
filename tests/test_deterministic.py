import numpy as np

from motion_pipeline.artifact import SONIC_OFFICIAL_NEUTRAL_ISAACLAB
from motion_pipeline.deterministic import (
    FPS,
    KINDS,
    deterministic_trajectory,
)


def isaaclab_joints(qpos):
    from motion_pipeline.artifact import MUJOCO_TO_ISAACLAB

    return qpos[:, 7:][:, MUJOCO_TO_ISAACLAB]


def test_all_deterministic_references_begin_and_end_with_neutral_hold():
    for kind in KINDS:
        trajectory = deterministic_trajectory(kind)
        joints = isaaclab_joints(trajectory)
        hold = round(FPS)
        assert np.allclose(joints[:hold], SONIC_OFFICIAL_NEUTRAL_ISAACLAB)
        assert np.allclose(joints[-hold:], SONIC_OFFICIAL_NEUTRAL_ISAACLAB)
        assert np.allclose(trajectory[:, 3], 1.0)


def test_arm_raise_only_changes_left_arm():
    joints = isaaclab_joints(deterministic_trajectory("arm_raise"))
    changed = set(np.flatnonzero(np.max(np.abs(joints - joints[0]), axis=0) > 1e-6))
    assert changed == {11, 15, 21}


def test_shallow_squat_is_symmetric_and_returns_to_neutral():
    trajectory = deterministic_trajectory("shallow_squat")
    joints = isaaclab_joints(trajectory)
    lowest = int(np.argmin(trajectory[:, 2]))
    assert np.isclose(trajectory[lowest, 2], 0.70)
    assert np.isclose(joints[lowest, 0], joints[lowest, 1])
    assert np.isclose(joints[lowest, 9], joints[lowest, 10])
    assert np.isclose(joints[lowest, 13], joints[lowest, 14])
