from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class AssetSpec:
    env: str
    default: str
    size: int | None = None
    sha256: str | None = None


ASSETS = {
    "gem": AssetSpec("GEM_CHECKPOINT", "/models/gem/gem_smpl.ckpt", 5518923941,
                     "1d15cbe2864d6de61a75e83fdbfe83bec3c7b183eee3d3dcdbd9107e4456454a"),
    "hmr2": AssetSpec("HMR2_CHECKPOINT", "/opt/GEM/inputs/checkpoints/hmr2/epoch=10-step=25000.ckpt",
                      2709494041, "2dcf79638109781d1ae5f5c44fee5f55bc83291c210653feead9b7f04fa6f20e"),
    "vitpose": AssetSpec("VITPOSE_CHECKPOINT", "/opt/GEM/inputs/checkpoints/vitpose/vitpose-h-multi-coco.pth",
                         2549075546, "50e33f4077ef2a6bcfd7110c58742b24c5859b7798fb0eedd6d2215e0a8980bc"),
    "yolo": AssetSpec("GEM_YOLO_MODEL", "/models/hf-gvhmr/yolo/yolov8x.pt", 136867539,
                      "c4d5a3f000d771762f03fc8b57ebd0aae324aeaefdd6e68492a9c4470f2d1e8b"),
    "smplx": AssetSpec("SMPLX_NEUTRAL", "/models/body_models/smplx/SMPLX_NEUTRAL.npz"),
}


def validate_video_assets(*, full_hash: bool = True) -> dict[str, dict[str, object]]:
    report: dict[str, dict[str, object]] = {}
    errors: list[str] = []
    for name, spec in ASSETS.items():
        path = Path(os.getenv(spec.env, spec.default))
        item: dict[str, object] = {"path": str(path), "valid": False}
        report[name] = item
        if not path.is_file():
            errors.append(f"{name} asset is missing or not a file: {path}")
            continue
        size = path.stat().st_size
        item["size"] = size
        if spec.size is not None and size != spec.size:
            errors.append(f"{name} asset size mismatch: {size} != {spec.size}")
            continue
        if full_hash and spec.sha256 is not None:
            digest = _sha256(path)
            item["sha256"] = digest
            if digest != spec.sha256:
                errors.append(f"{name} asset checksum mismatch")
                continue
        if name == "smplx":
            try:
                with np.load(path, allow_pickle=False) as body:
                    missing = {"v_template", "shapedirs"} - set(body.files)
                    if missing:
                        raise ValueError(f"missing arrays: {', '.join(sorted(missing))}")
            except (OSError, ValueError) as exc:
                errors.append(f"invalid licensed SMPL-X neutral model: {exc}")
                continue
        item["valid"] = True
    if errors:
        raise RuntimeError("video model asset preflight failed:\n- " + "\n- ".join(errors))
    return report


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Validate pinned video model assets")
    parser.add_argument("--size-only", action="store_true")
    args = parser.parse_args()
    print(json.dumps(validate_video_assets(full_hash=not args.size_only), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
