#!/usr/bin/python3
"""Execute exactly one prevalidated platform stage while holding the drone."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


PROJECT_DIR = Path(__file__).resolve().parents[2]
CONFIRMATIONS = {
    "lift": "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY",
    "transport": "I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY",
    "align": "I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY",
    "descend": "I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY",
    "descend10": "I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY",
}


def load_stage_q(report_path: Path, stage: str) -> list[float]:
    with report_path.open(encoding="utf-8") as stream:
        report = yaml.safe_load(stream)
    expected_mode = (
        "platform_descent_debug"
        if stage in ("descend", "descend10")
        else "platform_preplacement_debug"
    )
    if report.get("mode") != expected_mode:
        raise ValueError(f"Report mode must be {expected_mode} for stage {stage}")
    values = report.get("stages", {}).get(stage, {}).get("q_deg")
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"Report stage {stage} must contain six joint values")
    q_deg = [float(value) for value in values]
    if not all(math.isfinite(value) for value in q_deg):
        raise ValueError(f"Report stage {stage} contains a non-finite joint value")
    return q_deg


def build_executor_command(selected_report: Path) -> list[str]:
    return [
        sys.executable,
        str(PROJECT_DIR / "scripts/execute_ros_pregrasp.py"),
        "--report",
        str(selected_report),
        "--confirmation",
        "I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52",
        "--mode",
        "transport_hold",
        "--gripper-open-m",
        "0.085",
        "--gripper-closed-m",
        "0.058",
        "--tolerance-deg",
        "0.5",
        "--timeout-s",
        "40",
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--stage",
        choices=("lift", "transport", "align", "descend", "descend10"),
        required=True,
    )
    parser.add_argument("--confirmation", required=True)
    args = parser.parse_args()

    required = CONFIRMATIONS[args.stage]
    if args.confirmation != required:
        raise ValueError(f"Incorrect confirmation; required: {required}")

    q_deg = load_stage_q(args.report, args.stage)
    with tempfile.TemporaryDirectory(prefix="piper_platform_stage_") as directory:
        selected_report = Path(directory) / "selected_stage.yaml"
        selected_report.write_text(
            yaml.safe_dump({"stages": {"transport": {"q_deg": q_deg}}}),
            encoding="utf-8",
        )
        result = subprocess.run(
            build_executor_command(selected_report), cwd=PROJECT_DIR, check=False
        )
    if result.returncode != 0:
        return result.returncode
    print(f"PLATFORM_STAGE_{args.stage.upper()}_COMPLETE")
    print("The 58 mm gripper hold command remains active; no release was commanded.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
