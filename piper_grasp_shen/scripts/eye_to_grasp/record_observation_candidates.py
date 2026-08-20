#!/usr/bin/env python3
"""Interactively record multiple read-only D435i pre-observation candidates."""

from __future__ import annotations

import argparse
from pathlib import Path
import select
import sys
import time

import cv2
import rclpy
import yaml

from record_dual_camera_observation import Recorder
from observation_candidate_controls import build_single_sample_candidate, terminal_action


PROJECT_DIR = Path(__file__).resolve().parents[2]


def save_record(output, candidates):
    record = {
        "schema_version": 2,
        "camera_global": "Orbbec Gemini 336L",
        "camera_local": "Intel RealSense D435i",
        "tag_family": "tag36h11",
        "tag_id": 0,
        "samples_per_candidate": 1,
        "candidates": candidates,
        "notes": "Manually taught D435i pre-observation poses; no motion commands issued.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(record, sort_keys=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_DIR / "runtime/eye_to_grasp/observation_candidates.yaml",
    )
    parser.add_argument("--candidates", type=int, default=5)
    parser.add_argument("--samples-per-candidate", type=int, default=1)
    parser.add_argument("--base-frame", default="piper_x/base_link")
    parser.add_argument("--flange-frame", default="piper_x/flange_link")
    parser.add_argument("--global-tag-frame", default="gemini_apriltag_0")
    parser.add_argument("--local-camera-frame", default="d435i_color_optical_frame")
    parser.add_argument("--local-tag-frame", default="apriltag_0")
    parser.add_argument("--joint-topic", default="/piper_x/feedback/joint_states")
    parser.add_argument("--global-image-topic", default="/gemini336l/color/image_raw")
    parser.add_argument("--local-image-topic", default="/sensors/d435i/color/image_raw")
    args = parser.parse_args()
    if args.candidates < 1:
        raise ValueError("Need at least one candidate")
    if args.samples_per_candidate != 1:
        print("--samples-per-candidate is ignored; one ENTER records one snapshot.")

    rclpy.init()
    node = Recorder(args)
    candidates = []
    capturing = False
    image_dir = args.output.parent / (args.output.stem + "_images")
    print("Headless recorder ready.")
    print("Press ENTER to capture one candidate; type q then ENTER to save and exit.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if readable:
                action = terminal_action(sys.stdin.readline())
                if action == "capture" and not capturing:
                    capturing = True
                    print(f"capturing candidate_{len(candidates) + 1:02d}", flush=True)
                elif action == "save" and not capturing:
                    if not candidates:
                        print("No candidate recorded; press ENTER first.", flush=True)
                    else:
                        break
                elif action == "eof":
                    raise RuntimeError("Terminal input closed")
                elif action == "invalid":
                    print("Unknown command. Press ENTER to capture or q then ENTER to save.")
                elif capturing:
                    print("Capture in progress; keep the arm and tags still.", flush=True)

            if not capturing:
                continue
            sample = node.sample(require_local_tag=False)
            if sample is None or node.global_image is None or node.local_image is None:
                continue

            candidate_id = f"candidate_{len(candidates) + 1:02d}"
            image_dir.mkdir(parents=True, exist_ok=True)
            global_path = image_dir / f"{candidate_id}_gemini336l.png"
            local_path = image_dir / f"{candidate_id}_d435i.png"
            cv2.imwrite(str(global_path), node.global_image)
            cv2.imwrite(str(local_path), node.local_image)
            candidates.append(
                build_single_sample_candidate(
                    candidate_id,
                    sample,
                    timestamp_s=time.time(),
                    global_image=str(global_path),
                    local_image=str(local_path),
                )
            )
            save_record(args.output, candidates)
            print(f"recorded {candidate_id}", flush=True)
            capturing = False
            if len(candidates) >= args.candidates:
                break

        save_record(args.output, candidates)
        print(f"saved: {args.output} ({len(candidates)} candidates)")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=__import__("sys").stderr)
        raise SystemExit(1)
