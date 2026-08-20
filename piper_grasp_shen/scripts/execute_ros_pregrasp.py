#!/usr/bin/python3
"""Execute one prevalidated pregrasp target through agx_arm_ctrl only."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import yaml


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


class RosPregraspExecutor(Node):
    def __init__(self) -> None:
        super().__init__("piper_grasp_ros_pregrasp_executor")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.latest_q = None
        self.latest_gripper_m = None
        self.latest_stamp_ns = None
        self.latest_seen = 0.0
        self.create_subscription(
            JointState, "/piper_x/feedback/joint_states", self._feedback, qos
        )
        self.move_pub = self.create_publisher(
            JointState, "/piper_x/control/move_j", qos
        )
        self.joint_pub = self.create_publisher(
            JointState, "/piper_x/control/joint_states", qos
        )
        self.enable_client = self.create_client(SetBool, "/piper_x/enable_agx_arm")
        self.gate_client = self.create_client(SetBool, "/piper_x/control_enable")

    def _feedback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if "gripper" in positions and math.isfinite(float(positions["gripper"])):
            self.latest_gripper_m = float(positions["gripper"])
        if not all(name in positions for name in JOINT_NAMES):
            return
        q = np.asarray([positions[name] for name in JOINT_NAMES], dtype=float)
        if not np.all(np.isfinite(q)):
            return
        self.latest_q = q
        self.latest_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.latest_seen = time.monotonic()

    def spin_until_feedback(self, timeout_s: float = 3.0) -> None:
        deadline = time.monotonic() + timeout_s
        first_stamp = self.latest_stamp_ns
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_q is not None and self.latest_stamp_ns != first_stamp:
                return
        raise RuntimeError("Fresh /piper_x/feedback/joint_states was not received")

    def call_bool(self, client, value: bool, name: str) -> None:
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"ROS service is unavailable: {name}")
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"ROS service timed out: {name}")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"ROS service failed: {name}: {response.message}")

    def command_gripper(self, width_m: float) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["gripper"]
        message.position = [width_m]
        message.effort = [1.0]
        self.joint_pub.publish(message)

    def command_target(self, target_q) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [float(value) for value in target_q]
        self.move_pub.publish(message)

    def wait_for_control_subscribers(self, timeout_s: float = 3.0) -> None:
        """Wait for DDS discovery before publishing the first control message."""

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.move_pub.get_subscription_count() > 0
                and self.joint_pub.get_subscription_count() > 0
            ):
                return
        raise RuntimeError("ROS control publishers did not discover their subscribers")


def load_target(report_path: Path, stage: str = "pregrasp") -> np.ndarray:
    with report_path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    target_deg = np.asarray(data["stages"][stage]["q_deg"], dtype=float)
    if target_deg.shape != (6,) or not np.all(np.isfinite(target_deg)):
        raise ValueError(f"Report stage {stage} does not contain six finite target joints")
    return np.deg2rad(target_deg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--gripper-open-m", type=float, default=0.070)
    parser.add_argument("--gripper-closed-m", type=float, default=0.048)
    parser.add_argument(
        "--mode",
        choices=(
            "pregrasp",
            "approach50",
            "approach25",
            "grasp_close",
            "direct_grasp_close",
            "retreat_hold",
            "transport_hold",
        ),
        default="pregrasp",
    )
    args = parser.parse_args()
    if not (
        math.isfinite(args.gripper_open_m)
        and math.isfinite(args.gripper_closed_m)
        and 0.0 <= args.gripper_closed_m <= args.gripper_open_m <= 0.10
    ):
        raise ValueError(
            "Gripper widths must satisfy 0 <= closed <= open <= 0.10 m"
        )
    required_confirmation = {
        "pregrasp": "I_CONFIRM_ROS_PREGRASP_ONLY",
        "approach50": "I_CONFIRM_APPROACH_50MM_ONLY",
        "approach25": "I_CONFIRM_APPROACH_25MM_ONLY",
        "grasp_close": "I_CONFIRM_FINAL_25MM_AND_CLOSE",
        "direct_grasp_close": "I_CONFIRM_DIRECT_100MM_AND_CLOSE",
        "retreat_hold": "I_CONFIRM_FULL_RETREAT_WITH_GRIP",
        "transport_hold": "I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52",
    }[args.mode]
    if args.confirmation != required_confirmation:
        raise ValueError("Incorrect ROS pregrasp confirmation")
    target_stage = {
        "grasp_close": "grasp",
        "direct_grasp_close": "grasp",
        "retreat_hold": "retreat",
        "transport_hold": "transport",
    }.get(args.mode, "pregrasp")
    target = load_target(args.report, target_stage)
    tolerance = math.radians(args.tolerance_deg)
    rclpy.init()
    node = RosPregraspExecutor()
    gate_open = False
    try:
        node.spin_until_feedback()
        node.wait_for_control_subscribers()
        initial = node.latest_q.copy()
        print("initial_q_deg: " + " ".join(f"{x:.3f}" for x in np.rad2deg(initial)))
        print("target_q_deg: " + " ".join(f"{x:.3f}" for x in np.rad2deg(target)))
        node.call_bool(node.gate_client, True, "/piper_x/control_enable")
        gate_open = True
        node.call_bool(node.enable_client, True, "/piper_x/enable_agx_arm")
        if args.mode in ("retreat_hold", "transport_hold"):
            # Keep commanding the requested closed width: feedback remains wider when the object is
            # physically blocking the fingers, which preserves grasp force.
            node.command_gripper(args.gripper_closed_m)
            time.sleep(0.3)
        elif args.mode not in ("grasp_close", "direct_grasp_close"):
            node.command_gripper(args.gripper_open_m)
            time.sleep(0.5)
        node.command_target(target)

        deadline = time.monotonic() + args.timeout_s
        last_target_publish = time.monotonic()
        last_gripper_publish = time.monotonic()
        motion_started = False
        while time.monotonic() < deadline:
            node.spin_until_feedback(timeout_s=1.0)
            if time.monotonic() - node.latest_seen > 0.5:
                raise RuntimeError("ROS joint feedback became stale")
            error = float(np.max(np.abs(target - node.latest_q)))
            if float(np.max(np.abs(node.latest_q - initial))) >= math.radians(0.05):
                motion_started = True
            if error <= tolerance:
                print(f"final_max_joint_error_deg: {math.degrees(error):.3f}")
                if args.mode in ("retreat_hold", "transport_hold"):
                    hold_deadline = time.monotonic() + 1.0
                    while time.monotonic() < hold_deadline:
                        node.command_gripper(args.gripper_closed_m)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if node.latest_gripper_m is not None:
                        print(f"gripper_feedback_m: {node.latest_gripper_m:.4f}")
                    if args.mode == "transport_hold":
                        print("ROS_STANDARD52_TRANSPORT_COMPLETE")
                    else:
                        print("ROS_FULL_RETREAT_WITH_GRIP_COMPLETE")
                    print(
                        f"The {args.gripper_closed_m * 1000:.0f} mm close command "
                        "remains active; no release was commanded."
                    )
                elif args.mode in ("grasp_close", "direct_grasp_close"):
                    close_deadline = time.monotonic() + 2.0
                    while time.monotonic() < close_deadline:
                        node.command_gripper(args.gripper_closed_m)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if node.latest_gripper_m is not None:
                        print(f"gripper_feedback_m: {node.latest_gripper_m:.4f}")
                    if args.mode == "direct_grasp_close":
                        print("ROS_DIRECT_100MM_AND_CLOSE_COMPLETE")
                    else:
                        print("ROS_FINAL_25MM_AND_CLOSE_COMPLETE")
                    print(
                        f"The gripper was commanded to {args.gripper_closed_m * 1000:.0f} mm. No lift or retreat "
                        "was commanded."
                    )
                elif args.mode == "approach25":
                    print("ROS_APPROACH_25MM_ONLY_COMPLETE")
                    print(
                        "Gripper remains open; the final 25 mm approach, closing, "
                        "and retreat were not commanded."
                    )
                elif args.mode == "approach50":
                    print("ROS_APPROACH_50MM_ONLY_COMPLETE")
                    print(
                        "Gripper remains open; the final 50 mm approach, closing, "
                        "and retreat were not commanded."
                    )
                else:
                    print("ROS_PREGRASP_ONLY_COMPLETE")
                    print("Gripper remains open; no descent or closing command was published.")
                return 0
            if not motion_started and time.monotonic() - last_target_publish >= 0.1:
                node.command_target(target)
                last_target_publish = time.monotonic()
            if (
                args.mode in ("retreat_hold", "transport_hold")
                and time.monotonic() - last_gripper_publish >= 0.2
            ):
                node.command_gripper(args.gripper_closed_m)
                last_gripper_publish = time.monotonic()
            time.sleep(0.05)
        error = float(np.max(np.abs(target - node.latest_q)))
        raise RuntimeError(
            f"ROS pregrasp target timeout: maximum joint error {math.degrees(error):.3f} deg"
        )
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
