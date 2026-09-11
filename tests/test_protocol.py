import pytest

from motion_pipeline.protocol import ControlCommand, GenerateCommand, ProtocolError, VideoGenerateCommand, to_json


def test_generate_round_trip():
    original = GenerateCommand("r1", "wave hello", 42, 4.0)
    assert GenerateCommand.parse(to_json(original)) == original


@pytest.mark.parametrize(
    "raw",
    [
        "not-json",
        '{"schema_version":2,"request_id":"r","prompt":"x","seed":1,"duration_s":2}',
        '{"schema_version":1,"request_id":"r","prompt":"x","seed":1,"duration_s":99}',
        '{"schema_version":1,"request_id":"r","prompt":"x","seed":1,"duration_s":2,"extra":true}',
    ],
)
def test_invalid_generate(raw):
    with pytest.raises(ProtocolError):
        GenerateCommand.parse(raw)


def test_unknown_control_action():
    with pytest.raises(ProtocolError):
        ControlCommand.parse('{"schema_version":1,"request_id":"r","motion_id":"m","action":"go"}')


def test_video_generate_round_trip():
    original = VideoGenerateCommand("v1", "/inputs/wave.mov", "a" * 64, 30.0,
                                    "gem", "16bebf4", True)
    assert VideoGenerateCommand.parse(to_json(original)) == original


def test_video_generate_rejects_unsupported_model():
    with pytest.raises(ProtocolError, match="model"):
        VideoGenerateCommand.parse(
            '{"schema_version":1,"request_id":"v2","video_path":"/x.mp4",'
            '"video_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",'
            '"original_fps":30,"model":"wham","model_version":"x","static_camera":true}'
        )


def test_video_generate_rejects_invalid_checksum():
    with pytest.raises(ProtocolError, match="SHA-256"):
        VideoGenerateCommand.parse(
            '{"schema_version":1,"request_id":"v3","video_path":"/x.mp4",'
            '"video_sha256":"not-a-digest","original_fps":30,"model":"gem",'
            '"model_version":"x","static_camera":true}'
        )
