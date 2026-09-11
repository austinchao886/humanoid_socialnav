from pathlib import Path

import pytest

from video_generator import assets as video_assets


def test_asset_preflight_fails_closed_for_missing_file(tmp_path, monkeypatch):
    missing = tmp_path / "missing.ckpt"
    monkeypatch.setattr(video_assets, "ASSETS", {
        "gem": video_assets.AssetSpec("TEST_GEM_ASSET", str(missing)),
    })
    with pytest.raises(RuntimeError, match="missing or not a file"):
        video_assets.validate_video_assets()


def test_asset_preflight_accepts_expected_size_and_hash(tmp_path, monkeypatch):
    asset = tmp_path / "asset.bin"
    asset.write_bytes(b"pinned")
    monkeypatch.setattr(video_assets, "ASSETS", {
        "gem": video_assets.AssetSpec(
            "TEST_GEM_ASSET", str(asset), 6,
            "3fab5c181bd28a09b64397df76ae2bfaf1eac182979b5fdb7a342858004f36af",
        ),
    })
    assert video_assets.validate_video_assets()["gem"]["valid"] is True
