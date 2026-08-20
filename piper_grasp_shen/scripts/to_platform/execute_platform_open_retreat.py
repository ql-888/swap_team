#!/usr/bin/python3
"""Execute the planned 120 mm retreat while holding the gripper open."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml


CONFIRMATION = "I_CONFIRM_PLATFORM_OPEN_RETREAT_120MM_ONLY"


def load_report(path: Path):
    with path.open(encoding="utf-8") as stream:
        report = yaml.safe_load(stream)
    if report.get("mode") != "platform_open_retreat_debug":
        raise ValueError("Report is not an open-gripper platform retreat plan")
    age_s = time.time() - float(report["created_at_unix_s"])
    if not 0.0 <= age_s <= 300.0:
        raise ValueError(f"Open-retreat plan is stale: {age_s:.1f} s")
    inputs = report["inputs"]
    if abs(float(inputs["retreat_m"]) - 0.120) > 1e-6:
        raise ValueError("Open-retreat report must command exactly 120 mm")
    start_q = np.deg2rad(np.asarray(inputs["current_q_deg"], dtype=float))
    target_q = np.deg2rad(
        np.asarray(report["stages"]["retreat_open"]["q_deg"], dtype=float)
    )
    open_width = float(inputs["open_width_m"])
    if start_q.shape != (6,) or target_q.shape != (6,):
        raise ValueError("Open-retreat report must contain six start/target joints")
    return start_q, target_q, open_width


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise ValueError(f"Incorrect confirmation; required: {CONFIRMATION}")
    planned_start, target, open_width = load_report(args.report)

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import rclpy
    from execute_ros_pregrasp import RosPregraspExecutor

    rclpy.init()
    node = RosPregraspExecutor()
    gate_open = False
    try:
        node.spin_until_feedback()
        node.wait_for_control_subscribers()
        if float(np.max(np.abs(node.latest_q - planned_start))) > math.radians(0.75):
            raise RuntimeError("Robot joints changed since open-retreat planning")
        if node.latest_gripper_m is None or node.latest_gripper_m < 0.068:
            raise RuntimeError("Gripper feedback no longer indicates the released/open state")
        node.call_bool(node.gate_client, True, "/piper_x/control_enable")
        gate_open = True
        node.call_bool(node.enable_client, True, "/piper_x/enable_agx_arm")
        node.command_gripper(open_width)
        node.command_target(target)
        deadline = time.monotonic() + 40.0
        last_target = last_gripper = 0.0
        while time.monotonic() < deadline:
            node.spin_until_feedback(timeout_s=1.0)
            now = time.monotonic()
            error = float(np.max(np.abs(target - node.latest_q)))
            if error <= math.radians(0.5):
                hold_deadline = now + 1.0
                while time.monotonic() < hold_deadline:
                    node.command_gripper(open_width)
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(f"final_max_joint_error_deg: {math.degrees(error):.3f}")
                print(f"gripper_feedback_m: {node.latest_gripper_m:.4f}")
                print("PLATFORM_OPEN_RETREAT_120MM_COMPLETE")
                return 0
            if now - last_target >= 0.1:
                node.command_target(target)
                last_target = now
            if now - last_gripper >= 0.2:
                node.command_gripper(open_width)
                last_gripper = now
            time.sleep(0.05)
        raise RuntimeError("Open-gripper 120 mm retreat timed out")
    finally:
        if gate_open:
            try:
                node.call_bool(node.gate_client, False, "/piper_x/control_enable")
                print("ROS external control gate closed.")
            except Exception as exc:
                print(f"WARNING: failed to close ROS control gate: {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
