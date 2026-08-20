"""Safe boundary between Pink joint configurations and piper_sdk."""

from __future__ import annotations

import time
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from .config import DEFAULT_HARDWARE_SAFETY, HardwareSafety
from .units import protocol_to_radians, radians_to_protocol


@dataclass(frozen=True)
class JointFeedback:
    q_rad: tuple[float, ...]
    hz: float
    timestamp: float


class PiperSdkBridge:
    """Read and command six arm joints while enforcing local safety checks."""

    def __init__(
        self,
        can_name: str = "can0",
        safety: HardwareSafety = DEFAULT_HARDWARE_SAFETY,
    ) -> None:
        if not (Path("/sys/class/net") / can_name).exists():
            raise ConnectionError(
                f"CAN interface '{can_name}' does not exist; run scripts/setup_can.sh first"
            )
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise RuntimeError(
                "piper_sdk is missing from this environment. Run: "
                "python -m pip install piper-sdk==0.6.2"
            ) from exc
        if not 1 <= safety.speed_percent <= 100:
            raise ValueError("Piper speed_percent must be between 1 and 100")
        if safety.control_period_s <= 0.0 or safety.maximum_joint_step_rad <= 0.0:
            raise ValueError("Control period and maximum joint step must be positive")
        self.safety = safety
        self.arm = C_PiperInterface_V2(
            can_name=can_name,
            start_sdk_joint_limit=True,
            start_sdk_gripper_limit=True,
        )
        self.connected = False
        self.enabled = False
        self._last_command_q: tuple[float, ...] | None = None
        self._last_feedback_timestamp = 0.0
        self._last_feedback_seen_monotonic = 0.0

    def connect(self) -> None:
        self.arm.ConnectPort()
        self.connected = True
        self._last_command_q = None
        self._last_feedback_timestamp = 0.0
        self._last_feedback_seen_monotonic = time.monotonic()

    def close(self) -> None:
        disconnect = getattr(self.arm, "DisconnectPort", None)
        if callable(disconnect):
            disconnect()
        self.connected = False
        self.enabled = False

    def read_feedback(self) -> JointFeedback:
        if not self.connected:
            raise RuntimeError("Piper SDK bridge is not connected")
        message = self.arm.GetArmJointMsgs()
        state = message.joint_state
        raw = [getattr(state, f"joint_{index}") for index in range(1, 7)]
        q_rad = tuple(protocol_to_radians(raw))
        feedback = JointFeedback(
            q_rad=q_rad,
            hz=float(getattr(message, "Hz", 0.0)),
            timestamp=float(getattr(message, "time_stamp", 0.0)),
        )
        if not all(math.isfinite(value) for value in (*feedback.q_rad, feedback.hz, feedback.timestamp)):
            raise RuntimeError("Joint feedback contains NaN or infinity")
        now = time.monotonic()
        if feedback.timestamp > self._last_feedback_timestamp:
            self._last_feedback_timestamp = feedback.timestamp
            self._last_feedback_seen_monotonic = now
        elif now - self._last_feedback_seen_monotonic > self.safety.feedback_timeout_s:
            raise RuntimeError("Joint feedback timestamp stopped advancing")
        return feedback

    def wait_for_feedback(self) -> JointFeedback:
        deadline = time.monotonic() + self.safety.feedback_timeout_s
        last = None
        while time.monotonic() < deadline:
            last = self.read_feedback()
            if last.hz >= self.safety.minimum_feedback_hz:
                return last
            time.sleep(self.safety.control_period_s)
        hz = 0.0 if last is None else last.hz
        raise RuntimeError(
            f"Joint feedback is not healthy ({hz:.1f} Hz); no motion command was sent"
        )

    def enable(self) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.arm.EnablePiper():
                self.enabled = True
                return
            time.sleep(0.02)
        raise RuntimeError("Piper did not report enabled status; no target was sent")

    def command(self, q_rad: Sequence[float], model) -> None:
        if not self.connected or not self.enabled:
            raise RuntimeError("Refusing to command a disconnected or disabled Piper")
        model.validate_q(q_rad)
        feedback = self.read_feedback()
        if feedback.hz < self.safety.minimum_feedback_hz:
            raise RuntimeError("Joint feedback became stale; command stream stopped")

        reference = self._last_command_q or feedback.q_rad
        largest_step = max(abs(float(a) - float(b)) for a, b in zip(q_rad, reference))
        if largest_step > self.safety.maximum_joint_step_rad + 1.0e-9:
            raise RuntimeError(
                "Refusing a joint command step larger than "
                f"{self.safety.maximum_joint_step_rad:.4f} rad"
            )

        values = radians_to_protocol(q_rad)
        self.arm.MotionCtrl_2(0x01, 0x01, self.safety.speed_percent, 0x00)
        self.arm.JointCtrl(*values)
        self._last_command_q = tuple(float(value) for value in q_rad)

    def hold_current_position(self) -> None:
        """Keep position mode active; do not remove motor torque on shutdown."""

        if not self.connected or not self.enabled:
            return
        feedback = self.read_feedback()
        if feedback.hz < self.safety.minimum_feedback_hz:
            raise RuntimeError("Cannot hold position because joint feedback is stale")
        self.arm.MotionCtrl_2(0x01, 0x01, self.safety.speed_percent, 0x00)
        self.arm.JointCtrl(*radians_to_protocol(feedback.q_rad))

    def command_gripper(self, opening_m: float, effort: int = 1000) -> None:
        if not self.connected or not self.enabled:
            raise RuntimeError("Refusing to command a disconnected or disabled Piper")
        if not math.isfinite(opening_m) or not 0.0 <= opening_m <= 0.07:
            raise ValueError("Piper gripper opening must be between 0.0 and 0.07 m")
        if not 0 <= effort <= 5000:
            raise ValueError("Piper gripper effort must be between 0 and 5000")
        # GripperCtrl uses 0.001 mm, so metres are multiplied by 1e6.
        self.arm.GripperCtrl(round(opening_m * 1_000_000.0), effort, 0x01, 0)
