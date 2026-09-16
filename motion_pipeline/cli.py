from __future__ import annotations

import argparse
import json
import os
import time
import uuid
from pathlib import Path

from motion_contracts.protocol import (
    CONTROL_TOPIC, GENERATE_TOPIC, STATUS_TOPIC, VIDEO_GENERATE_TOPIC,
    ControlCommand, GenerateCommand, VideoGenerateCommand, to_json,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish and observe motion pipeline DDS commands")
    parser.add_argument("--domain", type=int, default=int(os.getenv("DDS_DOMAIN", "1")))
    parser.add_argument("--interface")
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate")
    generate.add_argument("prompt")
    generate.add_argument("--request-id", default=None)
    generate.add_argument("--seed", type=int, default=42)
    generate.add_argument("--duration", type=float, default=4.0)
    generate_video = sub.add_parser("generate-video")
    generate_video.add_argument("video_path")
    generate_video.add_argument("--request-id", default=None)
    generate_video.add_argument("--model", choices=["gem", "gvhmr"], default="gem")
    generate_video.add_argument("--model-version", default=None)
    generate_video.add_argument("--static-camera", action=argparse.BooleanOptionalAction, default=True)
    control = sub.add_parser("control")
    control.add_argument("action", choices=["approve_execute", "reject", "abort", "reset", "cancel"])
    control.add_argument("request_id")
    control.add_argument("motion_id")
    control.add_argument("--prepared-plan-id", help="Approve this exact prepared composition plan; requires a capable supervisor")
    control.add_argument("--execution-token", help="Cancel only this active prepared execution; obtain token from EXECUTING status/report")
    sub.add_parser("listen")
    args = parser.parse_args()
    from .runtime.dds_transport import JsonDDS

    dds = JsonDDS(args.domain, args.interface)
    if args.command == "generate":
        cmd = GenerateCommand(args.request_id or uuid.uuid4().hex, args.prompt, args.seed, args.duration)
        dds.publish(GENERATE_TOPIC, to_json(cmd))
        print(to_json(cmd))
    elif args.command == "generate-video":
        from video_generator.pipeline import inspect_video

        video_path = os.path.abspath(args.video_path)
        info = inspect_video(Path(video_path))
        versions = {
            "gem": os.getenv("GEM_MODEL_VERSION", "16bebf402d8893184249ee206d957b8248cd8310"),
            "gvhmr": os.getenv("GVHMR_MODEL_VERSION", "unknown"),
        }
        cmd = VideoGenerateCommand(
            args.request_id or uuid.uuid4().hex, video_path, info.sha256, info.fps,
            args.model, args.model_version or versions[args.model], args.static_camera,
        )
        dds.publish(VIDEO_GENERATE_TOPIC, to_json(cmd))
        print(to_json(cmd))
    elif args.command == "control":
        cmd = ControlCommand(args.request_id, args.motion_id, args.action,
                             prepared_plan_id=args.prepared_plan_id,
                             execution_token=args.execution_token)
        dds.publish(CONTROL_TOPIC, to_json(cmd))
        print(to_json(cmd))
    else:
        dds.subscribe(STATUS_TOPIC, lambda payload: print(json.dumps(json.loads(payload), indent=2), flush=True))
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
