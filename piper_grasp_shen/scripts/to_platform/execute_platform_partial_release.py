#!/usr/bin/python3
"""Open the gripper by the separately planned 10 mm and move no arm joint."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import yaml


CONFIRMATION = "I_CONFIRM_PLATFORM_OPEN_GRIPPER_10MM_ONLY"


def load_report(path: Path) -> tuple[float, float, float]:
    with path.open(encoding="utf-8") as stream:
        report = yaml.safe_load(stream)
    if report.get("mode") != "platform_partial_release_debug":
        raise ValueError("Report is not a platform partial-release plan")
    age_s = time.time() - float(report["created_at_unix_s"])
    if not 0.0 <= age_s <= 300.0:
        raise ValueError(f"Partial-release plan is stale: {age_s:.1f} s")
    initial = float(report["initial_gripper_feedback_m"])
    delta = float(report["opening_delta_m"])
    target = float(report["target_gripper_width_m"])
    if abs((initial + delta) - target) > 1e-6 or abs(delta - 0.010) > 1e-6:
        raise ValueError("Partial-release report must command exactly +10 mm")
    return initial, delta, target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise ValueError(f"Incorrect confirmation; required: {CONFIRMATION}")
    planned_initial, _, target = load_report(args.report)

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import rclpy
    from execute_ros_pregrasp import RosPregraspExecutor

    rclpy.init()
    node = RosPregraspExecutor()
    gate_open = False
    try:
        node.spin_until_feedback()
        node.wait_for_control_subscribers()
        current = node.latest_gripper_m
        if current is None:
            raise RuntimeError("Fresh gripper feedback is unavailable")
        if abs(current - planned_initial) > 0.002:
            raise RuntimeError(
                f"Gripper changed since planning: planned {planned_initial:.4f} m, "
                f"current {current:.4f} m"
            )
        node.call_bool(node.gate_client, True, "/piper_x/control_enable")
        gate_open = True
        node.call_bool(node.enable_client, True, "/piper_x/enable_agx_arm")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            node.command_gripper(target)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        node.spin_until_feedback()
        final = node.latest_gripper_m
        if final is None:
            raise RuntimeError("Gripper feedback was lost after opening")
        if final < planned_initial + 0.006:
            raise RuntimeError(
                f"Gripper did not open sufficiently: {planned_initial:.4f} -> {final:.4f} m"
            )
        print(f"initial_gripper_feedback_m: {planned_initial:.4f}")
        print(f"target_gripper_width_m: {target:.4f}")
        print(f"final_gripper_feedback_m: {final:.4f}")
        print("PLATFORM_PARTIAL_RELEASE_COMPLETE")
        print("No arm joint command was published.")
        return 0
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
