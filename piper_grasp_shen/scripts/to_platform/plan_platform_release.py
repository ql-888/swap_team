#!/usr/bin/env python3
"""Create a no-motion plan for opening the platform gripper by 10 mm."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import yaml


def partial_open_target(current_width_m: float, delta_m: float) -> float:
    if not math.isfinite(current_width_m) or not math.isfinite(delta_m):
        raise ValueError("Gripper widths must be finite")
    if current_width_m < 0.0 or delta_m <= 0.0:
        raise ValueError("Current width and opening delta must be positive")
    target = current_width_m + delta_m
    if target > 0.085:
        raise ValueError("Partial release target exceeds the 85 mm debug opening limit")
    return target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current-width-m", type=float, required=True)
    parser.add_argument("--delta-m", type=float, default=0.010)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    target = partial_open_target(args.current_width_m, args.delta_m)
    report = {
        "schema_version": 1,
        "created_at_unix_s": time.time(),
        "mode": "platform_partial_release_debug",
        "initial_gripper_feedback_m": args.current_width_m,
        "opening_delta_m": args.delta_m,
        "target_gripper_width_m": target,
        "arm_motion": False,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print(f"initial_gripper_feedback_m: {args.current_width_m:.4f}")
    print(f"target_gripper_width_m: {target:.4f}")
    print(f"opening_delta_mm: {args.delta_m * 1000.0:.1f}")
    print(f"report: {args.report}")
    print("Release planning only. No arm or gripper command was published.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
