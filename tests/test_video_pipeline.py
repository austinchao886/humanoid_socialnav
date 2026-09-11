import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from motion_pipeline.video_pipeline import OfflineVideoPipeline, VideoPipelineError, inspect_video


def _fake_run(command, **kwargs):
    if command[0] == "ffprobe":
        return SimpleNamespace(stdout=json.dumps({"streams": [{"codec_name": "h264", "width": 1920,
            "height": 1080, "avg_frame_rate": "30/1"}], "format": {"duration": "4.0"}}))
    if command[0] == "ffmpeg":
        Path(command[-1]).write_bytes(b"normalized")
        return SimpleNamespace(stdout="")
    values = dict(zip(command[1::2], command[2::2]))
    if command[0] == "model-adapter":
        frames = 120
        np.savez(values["--canonical"], global_orient=np.zeros((frames, 3)),
                 body_pose=np.zeros((frames, 63)), transl=np.zeros((frames, 3)),
                 betas=np.zeros(10), joints_world=np.zeros((frames, 24, 3)),
                 incam_global_orient=np.zeros((frames, 3)),
                 incam_body_pose=np.zeros((frames, 63)), incam_transl=np.zeros((frames, 3)),
                 camera_intrinsics=np.repeat(np.eye(3)[None], frames, axis=0), fps=np.asarray([30.0]))
        Path(values["--tracking"]).write_text(json.dumps({"person_count": 1,
            "single_person_frame_fraction": 1.0, "full_body_visible_fraction": 1.0,
            "feet_visible_fraction": 1.0, "longest_tracking_gap_s": 0.0}))
        Path(values["--overlay"]).write_bytes(b"overlay")
        Path(values["--world"]).write_bytes(b"world")
    elif command[0] == "retarget-adapter":
        qpos = np.zeros((120, 36)); qpos[:, 2] = 0.8; qpos[:, 3] = 1.0
        np.savez(values["--robot"], qpos=qpos, fps=np.asarray([30.0]),
                 foot_contacts=np.ones((120, 2), dtype=np.uint8),
                 contact_metrics=np.asarray([0.05, 0.0, 1.0, 1.0]))
        Path(values["--preview"]).write_bytes(b"preview")
    return SimpleNamespace(stdout="")


def test_offline_pipeline_exports_qpos_and_metadata(tmp_path, monkeypatch):
    source = tmp_path / "wave.mp4"; source.write_bytes(b"phone-video")
    monkeypatch.setattr("motion_pipeline.video_pipeline.subprocess.run", _fake_run)
    pipeline = OfflineVideoPipeline(
        {"gem": "model-adapter --canonical {canonical} --tracking {tracking} --overlay {overlay} --world {world}"},
        "retarget-adapter --canonical {canonical} --robot {robot} --preview {preview}")
    work = tmp_path / "work"; work.mkdir()
    qpos, metadata = pipeline.run(source, work, model="gem", static_camera=True)
    assert np.loadtxt(qpos, delimiter=",").shape == (120, 36)
    assert metadata["video"]["fps"] == 30.0
    assert metadata["capture_assumptions"]["feet_visible"] is True


def test_inspect_video_rejects_wrong_extension(tmp_path):
    source = tmp_path / "wave.avi"; source.write_bytes(b"video")
    with pytest.raises(VideoPipelineError, match="MP4 or MOV"):
        inspect_video(source)


def test_unconfigured_adapter_fails_closed(tmp_path, monkeypatch):
    source = tmp_path / "wave.mp4"; source.write_bytes(b"video")
    monkeypatch.setattr("motion_pipeline.video_pipeline.subprocess.run", _fake_run)
    work = tmp_path / "work"; work.mkdir()
    with pytest.raises(VideoPipelineError, match="not configured"):
        OfflineVideoPipeline({}, "").run(source, work, model="gem", static_camera=True)
