#!/usr/bin/env python3
"""Bridge the pinned official GEM-SMPL video demo to the canonical schema."""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tracking-report", type=Path, required=True)
    parser.add_argument("--overlay", type=Path, required=True)
    parser.add_argument("--world-view", type=Path, required=True)
    parser.add_argument("--static-camera", choices=["0", "1"], required=True)
    args = parser.parse_args()
    if args.static_camera != "1":
        raise SystemExit("v1 GEM adapter only accepts a fixed camera")

    import cv2
    import torch
    from ultralytics import YOLO

    gem_root = Path(os.environ.get("GEM_ROOT", "/opt/GEM"))
    demo = gem_root / "scripts/demo/demo_smpl_hpe.py"
    if not demo.is_file():
        raise FileNotFoundError(f"official GEM demo is missing: {demo}")
    report = audit_video(args.video, YOLO(os.environ.get("GEM_YOLO_MODEL", "yolov8x.pt")), cv2)
    if report["person_count"] != 1 or report["single_person_frame_fraction"] < 0.95:
        raise RuntimeError("GEM preflight requires exactly one person in at least 95% of frames")

    with tempfile.TemporaryDirectory(prefix="gem-offline-") as temp:
        output_root = Path(temp)
        command = [sys.executable, str(demo), "--video", str(args.video),
                   "--output_root", str(output_root), "--static_cam"]
        checkpoint = os.environ.get("GEM_CHECKPOINT")
        if checkpoint:
            command += ["--ckpt_path", checkpoint]
        subprocess.run(command, cwd=gem_root, check=True, timeout=1200)
        result_dir = output_root / args.video.stem
        kp2d = torch.load(result_dir / "preprocess/vitpose.pt", map_location="cpu",
                          weights_only=False)
        confidence = kp2d[..., 2].numpy()
        report["full_body_visible_fraction"] = float(np.mean(np.all(confidence >= 0.5, axis=1)))
        report["feet_visible_fraction"] = float(np.mean(np.all(confidence[:, [15, 16]] >= 0.5, axis=1)))
        args.tracking_report.parent.mkdir(parents=True, exist_ok=True)
        args.tracking_report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        params = torch.load(result_dir / "smpl_params.pt", map_location="cpu", weights_only=False)
        global_params, incam_params = params["body_params_global"], params["body_params_incam"]
        frames = len(global_params["global_orient"])
        betas = global_params.get("betas", torch.zeros(frames, 10)).detach().cpu()
        betas_sequence = betas if betas.ndim == 2 else betas[None].expand(frames, -1)
        betas_out = betas.mean(0) if betas.ndim == 2 else betas
        sys.path.insert(0, str(gem_root))
        from gem.utils.smplx_utils import make_smplx
        body_model = make_smplx("supermotion").cuda().eval()
        with torch.no_grad():
            body = body_model(body_pose=global_params["body_pose"].cuda(),
                              global_orient=global_params["global_orient"].cuda(),
                              transl=global_params["transl"].cuda(),
                              betas=betas_sequence.cuda())
        intrinsics = params["K_fullimg"].detach().cpu().numpy()
        if intrinsics.ndim == 2:
            intrinsics = np.repeat(intrinsics[None], frames, axis=0)
        np.savez_compressed(
            args.output,
            global_orient=global_params["global_orient"].detach().cpu().numpy(),
            body_pose=global_params["body_pose"].detach().cpu().numpy(),
            transl=global_params["transl"].detach().cpu().numpy(), betas=betas_out.numpy(),
            joints_world=body.joints[:, :24].cpu().numpy(),
            incam_global_orient=incam_params["global_orient"].detach().cpu().numpy(),
            incam_body_pose=incam_params["body_pose"].detach().cpu().numpy(),
            incam_transl=incam_params["transl"].detach().cpu().numpy(),
            camera_intrinsics=intrinsics, fps=np.asarray([30.0]),
        )
        shutil.copy2(result_dir / "1_incam.mp4", args.overlay)
        shutil.copy2(result_dir / "2_global.mp4", args.world_view)


def audit_video(path: Path, model, cv2) -> dict:
    capture = cv2.VideoCapture(str(path))
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 30.0)
    counts, missing, longest_missing = [], 0, 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        results = model(frame, classes=[0], verbose=False)
        count = sum(int((result.boxes.conf >= 0.5).sum().item()) for result in results)
        counts.append(count)
        missing = missing + 1 if count == 0 else 0
        longest_missing = max(longest_missing, missing)
    capture.release()
    if not counts:
        raise RuntimeError("video decoder produced no frames")
    return {"person_count": max(counts),
            "single_person_frame_fraction": float(np.mean(np.asarray(counts) == 1)),
            "full_body_visible_fraction": 1.0, "feet_visible_fraction": 1.0,
            "longest_tracking_gap_s": longest_missing / fps,
            "detector": "ultralytics-yolov8-person-conf-0.5"}


if __name__ == "__main__":
    main()
