"""Trajectory generation and guarded execution for first hardware tests."""

from __future__ import annotations

import time

from .model import _robotics_stack


def interpolate_joint_path(start_q, goal_q, maximum_step_rad: float):
    np, _ = _robotics_stack()
    start = np.asarray(start_q, dtype=float)
    goal = np.asarray(goal_q, dtype=float)
    if start.shape != goal.shape or start.shape != (6,):
        raise ValueError("Joint path endpoints must both contain six values")
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
        raise ValueError("Joint path endpoints contain NaN or infinity")
    if not np.isfinite(maximum_step_rad) or maximum_step_rad <= 0.0:
        raise ValueError("maximum_step_rad must be finite and positive")
    largest_delta = float(np.max(np.abs(goal - start)))
    step_count = max(1, int(np.ceil(largest_delta / maximum_step_rad)))
    return [start + (goal - start) * (index / step_count) for index in range(1, step_count + 1)]


def execute_joint_path(bridge, model, path) -> None:
    """Execute a prevalidated path, stopping command generation on any fault."""

    deadline = time.monotonic() + bridge.safety.execution_timeout_s
    failed = False
    try:
        for q_target in path:
            if time.monotonic() >= deadline:
                raise RuntimeError("Motion execution timed out")
            started = time.monotonic()
            bridge.command(q_target, model)
            remaining = bridge.safety.control_period_s - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except BaseException:
        failed = True
        raise
    finally:
        try:
            bridge.hold_current_position()
        except Exception:
            if not failed:
                raise


def wait_for_joint_target(
    bridge, model, target_q, collision_checker=None,
    tolerance_rad: float = 0.01, timeout_s: float | None = None,
):
    """Repeat the final target until measured joints converge or time out."""

    np, _ = _robotics_stack()
    target = np.asarray(target_q, dtype=float)
    model.validate_q(target)
    if not np.isfinite(tolerance_rad) or tolerance_rad <= 0.0:
        raise ValueError("Joint target tolerance must be finite and positive")
    timeout = bridge.safety.execution_timeout_s if timeout_s is None else float(timeout_s)
    if not np.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("Joint target timeout must be finite and positive")
    deadline = time.monotonic() + timeout
    last_error = float("inf")
    while time.monotonic() < deadline:
        feedback = bridge.wait_for_feedback()
        measured = np.asarray(feedback.q_rad, dtype=float)
        model.validate_q(measured)
        if collision_checker is not None:
            collision_checker.validate_q(measured)
        last_error = float(np.max(np.abs(target - measured)))
        if last_error <= tolerance_rad:
            return feedback
        bridge.command(target, model)
        time.sleep(bridge.safety.control_period_s)
    raise RuntimeError(
        "Mechanical arm did not reach the commanded joint target: "
        f"maximum error {np.degrees(last_error):.3f} deg exceeds "
        f"{np.degrees(tolerance_rad):.3f} deg"
    )
