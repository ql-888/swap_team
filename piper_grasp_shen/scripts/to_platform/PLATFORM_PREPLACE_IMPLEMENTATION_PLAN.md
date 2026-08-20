# Platform Pre-placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a guarded online-debug workflow that locks Standard52h13 ID 0, plans a held drone through lift, transport-position, and final horizontal alignment stages, and executes only one explicitly confirmed stage at a time.

**Architecture:** Keep platform orchestration and planning in `scripts/to_platform/`, reuse the existing stable-pose capture and ROS joint executor, and represent each independently executable target in one YAML report. The measured marker supplies locked XY and yaw; Z is always `0.170 + 0.200 = 0.370 m`. No code path descends or releases the gripper.

**Tech Stack:** Python 3.10, NumPy, PyYAML, ROS 2 Humble, apriltag_ros, Pink IK, pytest, Bash.

**Spec:** `scripts/to_platform/PLATFORM_PREPLACE_DESIGN.md`

## Global Constraints

- Platform marker is `Standard52h13`, ID `0`, outer dashed-square edge `36.5 mm`, apriltag_ros pose size `0.0219 m`.
- Marker plane and platform support plane are the same horizontal plane at provisional base Z `0.170 m`.
- Marker centre is the provisional desired drone centre in XY.
- Pre-placement drone-marker Z is fixed to `0.370 m`; measured marker Z is ignored.
- Final drone heading equals platform marker heading; final drone bottom is horizontal.
- Transport orientation may differ from the final orientation.
- First hardware workflow stops after final pre-placement alignment; no descent and no gripper release exist.
- Each motion stage needs its own exact confirmation token and current gripper/joint feedback.
- Existing grasp scripts and state machine remain unchanged.
- This project has no Git metadata; replace commit steps with test and diff checkpoints.

---

### Task 1: Correct Standard52h13 metric configuration

**Files:**
- Modify: `config/apriltag_gemini336l_live_standard52h13.yaml`
- Modify: `scripts/capture_gemini_standard52_target.py`
- Create: `tests/test_platform_preplace.py`

**Interfaces:**
- Consumes: measured full outer edge `0.0365 m` and Standard52h13 ratio `6 / 10`.
- Produces: detector and captured pose metadata using `tag_size_m == 0.0219` and `measured_outer_edge_m == 0.0365`.

- [ ] **Step 1: Write failing metric-configuration tests**

```python
from pathlib import Path
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_platform_detector_uses_converted_standard52_size():
    config = yaml.safe_load(
        (PROJECT_ROOT / "config/apriltag_gemini336l_live_standard52h13.yaml")
        .read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert config["family"] == "Standard52h13"
    assert config["size"] == 0.0219
    assert config["tag"]["ids"] == [0]
    assert config["tag"]["sizes"] == [0.0219]


def test_platform_capture_records_measured_and_pose_sizes():
    source = (
        PROJECT_ROOT / "scripts/capture_gemini_standard52_target.py"
    ).read_text(encoding="utf-8")
    assert '"tag_size_m": 0.0219' in source
    assert '"measured_outer_edge_m": 0.0365' in source
```

- [ ] **Step 2: Run tests and verify the old 22.2/37 mm values fail**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py -v
```

Expected: both tests fail because the current files contain `0.0222` and `0.037`.

- [ ] **Step 3: Make the minimal metric corrections**

Change detector `size`, `tag.sizes`, and their comment to `0.0219 m` derived from `36.5 mm * 6 / 10`. Change capture metadata and its conversion string to:

```python
"tag_size_m": 0.0219,
"measured_outer_edge_m": 0.0365,
"size_conversion": "36.5 mm outer edge * width_at_border 6 / total_width 10",
```

- [ ] **Step 4: Run the focused tests**

Run the command from Step 2.

Expected: PASS.

- [ ] **Step 5: Record the checkpoint**

Run:

```bash
rg -n "0\.0222|0\.037|0\.0219|0\.0365" \
  config/apriltag_gemini336l_live_standard52h13.yaml \
  scripts/capture_gemini_standard52_target.py
```

Expected: active platform values are `0.0219` and `0.0365`; no stale active `0.0222` or `0.037` remains.

---

### Task 2: Generate three independently executable target stages

**Files:**
- Modify: `scripts/to_platform/plan_platform_above.py`
- Modify: `tests/test_platform_preplace.py`

**Interfaces:**
- Consumes: `base_from_tag: np.ndarray (4, 4)`, `current_tcp: np.ndarray (4, 4)`, `object_from_tcp: np.ndarray (4, 4)`, support height, marker clearance, and lift distance.
- Produces: `build_preplacement_targets(...) -> dict[str, np.ndarray]` with keys `lift`, `transport`, and `align`; planner report stages with matching keys and six-element `q_deg` arrays.

- [ ] **Step 1: Add failing pure-geometry tests**

Append:

```python
import importlib.util
import numpy as np


def load_platform_planner():
    path = PROJECT_ROOT / "scripts/to_platform/plan_platform_above.py"
    spec = importlib.util.spec_from_file_location("plan_platform_above", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_preplacement_targets_lock_xy_fixed_z_and_stage_orientation():
    planner = load_platform_planner()
    base_from_tag = np.eye(4)
    base_from_tag[:3, :3] = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    base_from_tag[:3, 3] = [0.31, -0.12, 0.99]
    current_tcp = np.eye(4)
    current_tcp[:3, 3] = [0.20, -0.20, 0.25]
    object_from_tcp = np.eye(4)
    object_from_tcp[2, 3] = -0.045

    targets = planner.build_preplacement_targets(
        base_from_tag,
        current_tcp,
        object_from_tcp,
        platform_height_m=0.170,
        object_height_above_tag_m=0.200,
        lift_waypoint_m=0.100,
    )

    assert set(targets) == {"lift", "transport", "align"}
    assert np.allclose(targets["lift"][:3, 3], [0.20, -0.20, 0.35])
    assert np.allclose(targets["transport"][:2, 3], [0.31, -0.12])
    assert targets["transport"][2, 3] == 0.370
    assert np.allclose(targets["transport"][:3, :3], np.eye(3))
    assert np.allclose(targets["align"][:3, 0], [0.0, 1.0, 0.0])
    assert np.allclose(targets["align"][:3, 2], [0.0, 0.0, 1.0])
    assert targets["align"][2, 3] == 0.370


def test_preplacement_targets_ignore_measured_tag_z():
    planner = load_platform_planner()
    current_tcp = np.eye(4)
    object_from_tcp = np.eye(4)
    first = np.eye(4)
    second = np.eye(4)
    first[:3, 3] = [0.2, 0.3, 0.05]
    second[:3, 3] = [0.2, 0.3, 0.80]
    first_targets = planner.build_preplacement_targets(
        first, current_tcp, object_from_tcp, 0.170, 0.200, 0.100
    )
    second_targets = planner.build_preplacement_targets(
        second, current_tcp, object_from_tcp, 0.170, 0.200, 0.100
    )
    assert np.allclose(first_targets["align"], second_targets["align"])
```

- [ ] **Step 2: Run the two geometry tests and verify failure**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py \
  -k "preplacement_targets" -v
```

Expected: FAIL because `build_preplacement_targets` does not exist.

- [ ] **Step 3: Implement pure target construction**

Add this public function above `main()`:

```python
def build_preplacement_targets(
    base_from_tag: np.ndarray,
    current_tcp: np.ndarray,
    object_from_tcp: np.ndarray,
    platform_height_m: float,
    object_height_above_tag_m: float,
    lift_waypoint_m: float,
) -> dict[str, np.ndarray]:
    target_z = platform_height_m + object_height_above_tag_m
    current_object = current_tcp @ np.linalg.inv(object_from_tcp)

    lift_tcp = current_tcp.copy()
    lift_tcp[2, 3] += lift_waypoint_m

    transport_object = current_object.copy()
    transport_object[:2, 3] = base_from_tag[:2, 3]
    transport_object[2, 3] = target_z

    align_object = np.eye(4)
    align_object[:3, :3], _ = tag_aligned_horizontal_rotation(
        base_from_tag[:3, :3], current_tcp[:3, :3]
    )
    align_object[:2, 3] = base_from_tag[:2, 3]
    align_object[2, 3] = target_z
    return {
        "lift": lift_tcp,
        "transport": transport_object @ object_from_tcp,
        "align": align_object @ object_from_tcp,
    }
```

The matrices returned are TCP targets. The tests use identity `object_from_tcp`, so their asserted object and TCP positions coincide.

- [ ] **Step 4: Refactor planner to solve and validate each stage**

Use default `--lift-waypoint-m 0.100`. Solve sequentially:

```python
targets = build_preplacement_targets(
    base_from_tag,
    current_tcp.homogeneous,
    recipe.object_from_tcp,
    args.platform_height_m,
    args.object_height_above_tag_m,
    args.lift_waypoint_m,
)
lift_result = solve_target(solver, model, model.pose_from_matrix(targets["lift"]), seeds)
transport_result = solve_target(
    solver, model, model.pose_from_matrix(targets["transport"]),
    (lift_result.q, current_q, model.neutral()[:6]),
)
align_result = solve_target(
    solver, model, model.pose_from_matrix(targets["align"]),
    (transport_result.q, lift_result.q, current_q, model.neutral()[:6]),
)
```

Build and collision-check three paths: current-to-lift, lift-to-transport, and transport-to-align. Store report metadata:

```yaml
mode: platform_preplacement_debug
inputs:
  platform_height_m: 0.170
  object_height_above_tag_m: 0.200
  fixed_object_z_m: 0.370
  measured_tag_z_ignored: true
  orientation_policy: transport preserves held orientation; align is horizontal and tag-aligned
stages:
  lift: {target: [...], q_deg: [...]}
  transport: {target: [...], q_deg: [...]}
  align: {target: [...], q_deg: [...]}
```

- [ ] **Step 5: Update capture validation from `0.0222` to `0.0219`**

Replace the exact expected capture size in `plan_platform_above.py` and update the error text to `21.9 mm converted pose size`.

- [ ] **Step 6: Run geometry and existing planner-related tests**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py tests/test_pose_recipe.py -v
```

Expected: PASS.

- [ ] **Step 7: Record the checkpoint**

Run:

```bash
python3 -m py_compile scripts/to_platform/plan_platform_above.py
rg -n '"lift"|"transport"|"align"|0\.370|measured_tag_z_ignored' \
  scripts/to_platform/plan_platform_above.py
```

Expected: syntax succeeds and all three report stages plus the fixed-Z policy are present.

---

### Task 3: Add one-stage-at-a-time execution gates

**Files:**
- Create: `scripts/to_platform/execute_platform_stage.py`
- Modify: `tests/test_platform_preplace.py`

**Interfaces:**
- Consumes: report path, stage in `{lift, transport, align}`, exact stage confirmation, `gripper_open_m=0.085`, `gripper_closed_m=0.058`.
- Produces: execution of exactly one report target through existing `scripts/execute_ros_pregrasp.py --mode transport_hold`; never commands gripper release.

- [ ] **Step 1: Add failing confirmation and report-validation tests**

Append:

```python
import subprocess


def write_stage_report(path):
    path.write_text(
        yaml.safe_dump(
            {
                "mode": "platform_preplacement_debug",
                "stages": {
                    "lift": {"q_deg": [1, 2, 3, 4, 5, 6]},
                    "transport": {"q_deg": [2, 3, 4, 5, 6, 7]},
                    "align": {"q_deg": [3, 4, 5, 6, 7, 8]},
                },
            }
        ),
        encoding="utf-8",
    )


def test_platform_stage_rejects_wrong_confirmation_before_ros(tmp_path):
    report = tmp_path / "report.yaml"
    write_stage_report(report)
    result = subprocess.run(
        [
            "/usr/bin/python3",
            str(PROJECT_ROOT / "scripts/to_platform/execute_platform_stage.py"),
            "--report", str(report),
            "--stage", "lift",
            "--confirmation", "WRONG",
        ],
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0
    assert "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY" in result.stderr


def test_platform_stage_source_has_no_release_mode():
    source = (
        PROJECT_ROOT / "scripts/to_platform/execute_platform_stage.py"
    ).read_text(encoding="utf-8")
    assert 'choices=("lift", "transport", "align")' in source
    assert "command_gripper" not in source
    assert "grasp_close" not in source
```

- [ ] **Step 2: Run tests and verify the missing entrypoint fails**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py \
  -k "platform_stage" -v
```

Expected: FAIL because `execute_platform_stage.py` does not exist.

- [ ] **Step 3: Implement the guarded stage wrapper**

The entrypoint must:

```python
CONFIRMATIONS = {
    "lift": "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY",
    "transport": "I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY",
    "align": "I_CONFIRM_PLATFORM_HORIZONTAL_ALIGN_ONLY",
}
```

Validate the exact token before importing or invoking ROS code. Load YAML and require `mode == "platform_preplacement_debug"` plus six finite joint values for the selected stage. Write a temporary report containing only:

```python
{"stages": {"transport": {"q_deg": selected_q_deg}}}
```

Then call:

```bash
scripts/execute_ros_pregrasp.py \
  --report <temporary-report> \
  --confirmation I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  --mode transport_hold \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 40
```

Print `PLATFORM_STAGE_<UPPERCASE_STAGE>_COMPLETE` only when the subprocess returns zero. Use `tempfile.TemporaryDirectory`; do not leave a stale executable report.

- [ ] **Step 4: Run focused tests and syntax check**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py \
  -k "platform_stage" -v
python3 -m py_compile scripts/to_platform/execute_platform_stage.py
```

Expected: PASS.

- [ ] **Step 5: Record the checkpoint**

Run:

```bash
rg -n "SAFE_LIFT_ONLY|TRANSPORT_POSITION_ONLY|HORIZONTAL_ALIGN_ONLY|release|grasp_close" \
  scripts/to_platform/execute_platform_stage.py
```

Expected: three exact confirmation strings appear; no release or close execution mode appears.

---

### Task 4: Add the operator-facing online-debug entrypoint

**Files:**
- Create: `scripts/to_platform/run_platform_preplace_debug.sh`
- Modify: `scripts/to_platform/README.md`
- Modify: `tests/test_platform_preplace.py`

**Interfaces:**
- Consumes: action `plan|lift|transport|align`; action-specific confirmation for motion actions.
- Produces: a fresh locked target and offline report for `plan`, or exactly one guarded motion stage for the other actions.

- [ ] **Step 1: Add failing launcher safety tests**

Append:

```python
def test_debug_launcher_exposes_only_plan_and_three_motion_stages():
    source = (
        PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"
    ).read_text(encoding="utf-8")
    assert "plan|lift|transport|align" in source
    assert "--platform-height-m 0.170" in source
    assert "--object-height-above-tag-m 0.200" in source
    assert "--lift-waypoint-m 0.100" in source
    assert "descend" not in source.lower()
    assert "release" not in source.lower()


def test_debug_launcher_checks_gripper_before_motion():
    source = (
        PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"
    ).read_text(encoding="utf-8")
    assert "read_ros_gripper_m.py" in source
    assert "0.058 <= value <= 0.085" in source
```

- [ ] **Step 2: Run launcher tests and verify failure**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py \
  -k "debug_launcher" -v
```

Expected: FAIL because the launcher does not exist.

- [ ] **Step 3: Implement `plan` action**

The shell script sources `scripts/source_ros_env.sh`, sets `ROS_DOMAIN_ID=1`, and uses:

```bash
RUN_DIR="${PROJECT_DIR}/runtime/to_platform"
TARGET_POSE="${RUN_DIR}/standard52h13_platform_target.yaml"
REPORT="${RUN_DIR}/platform_preplacement_debug_report.yaml"
```

For `plan`, require a publisher on `/perception/gemini336l_transport/standard52h13/detections`, call `capture_platform_target.py`, read six current joint values, and invoke:

```bash
.conda/bin/python scripts/to_platform/plan_platform_above.py \
  --target-pose "${TARGET_POSE}" \
  --current-q-deg "${Q[@]}" \
  --platform-height-m 0.170 \
  --object-height-above-tag-m 0.200 \
  --lift-waypoint-m 0.100 \
  --report "${REPORT}"
```

Finish with `PLAN_ONLY_COMPLETE` and explicitly print that no robot command was published.

- [ ] **Step 4: Implement the three motion actions**

For `lift|transport|align`, require the control node, existing report, gripper feedback within `[0.058, 0.085] m`, and the action-specific confirmation argument. Call `execute_platform_stage.py` for exactly that stage. Do not chain actions.

- [ ] **Step 5: Document the exact operator sequence**

Update README with:

```bash
scripts/run_apriltag_gemini336l_standard52h13.sh
scripts/to_platform/run_platform_preplace_debug.sh plan
scripts/to_platform/run_platform_preplace_debug.sh lift \
  I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY
scripts/to_platform/run_platform_preplace_debug.sh transport \
  I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY
scripts/to_platform/run_platform_preplace_debug.sh align \
  I_CONFIRM_PLATFORM_HORIZONTAL_ALIGN_ONLY
```

State after every motion command: stop, inspect the held drone, and continue only after the operator explicitly agrees. State that no descent or release exists.

- [ ] **Step 6: Run focused and full offline verification**

Run:

```bash
bash -n scripts/to_platform/run_platform_preplace_debug.sh
PYTHONDONTWRITEBYTECODE=1 .conda/bin/python -m pytest \
  -p no:cacheprovider tests/test_platform_preplace.py -v
./scripts/verify_offline.sh
```

Expected: shell syntax passes, platform tests pass, and the existing offline suite remains green.

- [ ] **Step 7: Record the implementation checkpoint**

Run:

```bash
find scripts/to_platform -maxdepth 1 -type f -printf '%f\n' | sort
rg -n "descend|release|command_gripper" scripts/to_platform \
  --glob '!PLATFORM_PREPLACE_DESIGN.md' \
  --glob '!PLATFORM_PREPLACE_IMPLEMENTATION_PLAN.md'
```

Expected: the new planner, guarded executor, launcher, design, plan, and README are visible; no new descent/release control path exists.

---

### Task 5: Live read-only validation before any motion

**Files:**
- Runtime output: `runtime/to_platform/standard52h13_platform_target.yaml`
- Runtime output: `runtime/to_platform/platform_preplacement_debug_report.yaml`
- Modify after milestone: `docs/CODEX_HANDOFF.md`

**Interfaces:**
- Consumes: live Gemini image/CameraInfo, Standard52h13 transform, robot TF, joint feedback, gripper feedback.
- Produces: verified locked target and offline three-stage report; no motion.

- [ ] **Step 1: Start or reuse only the platform detector used by this phase**

Run in a persistent terminal:

```bash
export ROS_DOMAIN_ID=1
scripts/run_apriltag_gemini336l_standard52h13.sh
```

Expected topic: `/perception/gemini336l_transport/standard52h13/detections` with ID `0`, `hamming=0`.

- [ ] **Step 2: Run the plan-only action**

Run:

```bash
scripts/to_platform/run_platform_preplace_debug.sh plan
```

Expected: 20 stable samples, fixed object Z `370 mm`, report stages `lift`, `transport`, `align`, and `PLAN_ONLY_COMPLETE`.

- [ ] **Step 3: Inspect the report before motion**

Check all stage joint vectors have six finite values, final align object Z is `0.370 m`, final object Z axis is vertical, final X heading matches the platform tag X heading, and all three interpolated paths passed collision checking.

- [ ] **Step 4: Update handoff with verified facts and pending hardware gates**

Record detector result, capture stability, planned joint targets, collision result, current gripper feedback, and the fact that no motion has yet occurred. Do not mark lift, transport, or alignment verified until each corresponding real stage completes.

- [ ] **Step 5: Stop and request the lift-stage go-ahead**

Show the operator the plan-only results. The next command is not run until the operator explicitly authorizes `I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY` while present at the robot with emergency stop ready.
