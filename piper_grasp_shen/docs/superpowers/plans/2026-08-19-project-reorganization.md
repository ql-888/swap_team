# Piper Grasp Shen Project Reorganization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize the project conservatively, internalize current camera calibrations, remove old-device path assumptions, and make offline planning runnable from the existing `piper_pink` Conda environment.

**Architecture:** Keep the public `assets/config/scripts/src/tests` boundaries and script names stable. Store source calibration reports under `calibration_data`, use package-relative robot mesh URIs, split Conda planning from system-Python ROS execution, and validate everything without enabling or moving the robot.

**Tech Stack:** Ubuntu 22.04, Bash, Python 3.10, Conda, ROS 2 Humble, Pinocchio, Pink, OpenCV, PyYAML, pytest.

**Spec:** `docs/superpowers/specs/2026-08-19-project-reorganization-design.md`

## Global Constraints

- Do not enable the arm, command joints, or operate the gripper during implementation or verification.
- Do not change TCP geometry, grasp recipes, safety confirmations, or speed limits.
- Preserve historical files under `runtime/`.
- Keep current public script filenames stable.
- Use the existing Conda environment `/home/guoyi/anaconda3/envs/piper_pink`.
- Use `/usr/bin/python3` for ROS scripts that import `rclpy`; use Conda Python for planning and tests.
- This directory is not a Git repository, so commit steps are recorded as unavailable rather than fabricated.

---

### Task 1: Calibration Data Ownership

**Files:**
- Create: `calibration_data/d435i_eye_in_hand.json`
- Create: `calibration_data/gemini336l_eye_to_hand.json`
- Create: `calibration_data/README.md`
- Modify: `config/handeye_d435i_wrist_end_pose.yaml`
- Modify: `config/handeye_orbbec_fixed.yaml`
- Modify: `tests/test_teammate_handeye_configs.py`

**Interfaces:**
- Consumes: the two user-provided JSON reports and existing normalized YAML transforms.
- Produces: project-relative `source.file` values and regression tests that compare JSON results to YAML matrices.

- [ ] **Step 1: Add failing locality and matrix-consistency tests**

Add tests that require `source.file` to start with `calibration_data/`, resolve it from project root, reconstruct the D435i matrix from `calibration_result.position/orientation`, and compare the Gemini `matrix` directly.

- [ ] **Step 2: Run the focused tests and verify failure**

Run:

```bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_teammate_handeye_configs.py
```

Expected: failure because current YAML sources are absolute and the project-local JSON files do not exist.

- [ ] **Step 3: Copy reports and update YAML metadata**

Copy without modifying JSON content, set the YAML paths to `calibration_data/d435i_eye_in_hand.json` and `calibration_data/gemini336l_eye_to_hand.json`, and document transform direction, camera identity, quality metrics, and the rule that runtime code consumes normalized YAML.

- [ ] **Step 4: Run focused tests and verify pass**

Run the command from Step 2. Expected: all tests in the file pass.

### Task 2: Portable Piper X Robot Model

**Files:**
- Modify: `assets/piper_x_description/urdf/piper_x_with_gripper_description.urdf`
- Modify: `tests/test_model_ik.py`
- Modify: `tests/test_collision_and_planning.py`

**Interfaces:**
- Consumes: existing STL files under `assets/piper_x_description/meshes` and `package_dirs=[assets]` in the loaders.
- Produces: a URDF containing only `package://piper_x_description/meshes/<name>.stl` mesh references.

- [ ] **Step 1: Add a failing URDF portability test**

The test reads the URDF text, asserts `"/home/" not in text`, asserts `".dae" not in text.lower()`, and checks that each `package://piper_x_description/meshes/*.stl` suffix exists beneath `assets`.

- [ ] **Step 2: Run the portability and collision tests and verify failure**

```bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_model_ik.py tests/test_collision_and_planning.py
```

Expected: failure on old `/home/shen_tao` URIs and collision mesh loading.

- [ ] **Step 3: Replace visual and collision mesh URIs**

Map each old DAE/STL reference for `base_link`, `link1` through `link6`, `flange`, `gripper_base`, `gripper_link1`, and `gripper_link2` to the corresponding existing STL package URI.

- [ ] **Step 4: Run the focused tests and verify pass**

Run the command from Step 2. Expected: model, IK, known-pose collision, and scene collision tests pass.

### Task 3: Standard52 Transport Shape Bug

**Files:**
- Modify: `scripts/plan_standard52_transport.py`
- Modify: `tests/test_pose_recipe.py`

**Interfaces:**
- Consumes: `object_from_tcp: np.ndarray` with shape `(4, 4)`.
- Produces: a three-element closing axis passed to `horizontalize_rotation(rotation: np.ndarray) -> np.ndarray`.

- [ ] **Step 1: Retain the existing failing regression test and add a shape assertion**

Confirm `test_standard52_transport_aligns_closing_axis_with_tag_x` fails because a four-element homogeneous row or column reaches `np.cross`.

- [ ] **Step 2: Run the single test and verify failure**

```bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_pose_recipe.py::test_standard52_transport_aligns_closing_axis_with_tag_x
```

Expected: `ValueError: incompatible dimensions for cross product`.

- [ ] **Step 3: Extract only the rotation portion**

Change the transport calculation so the closing direction is derived from `object_from_tcp[:3, :3]` and every axis passed to `np.cross` has shape `(3,)`.

- [ ] **Step 4: Run `tests/test_pose_recipe.py` and verify pass**

```bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_pose_recipe.py
```

Expected: all pose and recipe tests pass.

### Task 4: Conda and ROS Interpreter Boundaries

**Files:**
- Modify: `scripts/run.sh`
- Modify: `scripts/setup_env.sh`
- Modify: `scripts/execute_ros_pregrasp.py`
- Modify: `scripts/capture_d435i_apriltag_pose.py`
- Modify: `scripts/capture_gemini_standard52_target.py`
- Modify: `scripts/read_ros_gripper_m.py`
- Modify: `scripts/read_ros_joint_degrees.py`
- Modify: `tests/test_cli_safety.py`

**Interfaces:**
- Consumes: Conda executable at `/home/guoyi/anaconda3/envs/piper_pink/bin/python` and ROS Python at `/usr/bin/python3`.
- Produces: `PIPER_PINK_PYTHON` override, deterministic planning interpreter selection, and ROS scripts independent of an activated Conda `PATH`.

- [ ] **Step 1: Add failing script-boundary tests**

Assert ROS Python scripts start with `#!/usr/bin/python3`; assert `run.sh` contains `PIPER_PINK_PYTHON` and a `conda env list`/known-prefix fallback; assert `setup_env.sh` targets the named environment rather than `${PROJECT_DIR}/.conda`.

- [ ] **Step 2: Run CLI safety tests and verify failure**

```bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_cli_safety.py
```

Expected: interpreter-boundary assertions fail before changes.

- [ ] **Step 3: Implement deterministic interpreter selection**

Make `run.sh` prefer `$PIPER_PINK_PYTHON`, then `$CONDA_PREFIX/bin/python` only when that prefix is named `piper_pink`, then `/home/guoyi/anaconda3/envs/piper_pink/bin/python`; emit a clear error if imports fail. Make `setup_env.sh` use `conda run -n piper_pink` and editable installation. Set ROS script shebangs to `/usr/bin/python3`.

- [ ] **Step 4: Run CLI tests with ROS environment available**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider tests/test_cli_safety.py
```

Expected: all safety and interpreter tests pass without opening CAN.

### Task 5: Configure Existing `piper_pink` Environment

**Files:**
- Modify externally: `/home/guoyi/anaconda3/envs/piper_pink` package metadata through Conda/pip commands.
- Regenerate: `src/piper_grasp.egg-info` only as an installation artifact if pip requires it.

**Interfaces:**
- Consumes: `pyproject.toml` and `requirements.lock.txt`.
- Produces: imports for `numpy`, `pinocchio`, `pink`, `cv2`, `yaml`, `piper`, `can`, and `piper_pink` from the intended environment.

- [ ] **Step 1: Audit versions and `pip check` without modification**

```bash
conda run -n piper_pink python -c "import numpy,pinocchio,pink,cv2,yaml; print(numpy.__version__, pinocchio.__version__, pink.__version__, cv2.__version__)"
conda run -n piper_pink python -m pip check
```

- [ ] **Step 2: Install only missing locked dependencies**

Use Conda for native Pinocchio/OpenCV packages if absent, pip for `pin-pink==4.3.0`, `piper-sdk==0.6.2`, and `python-can==4.6.1`, preserving already compatible versions.

- [ ] **Step 3: Install this project editable without resolving dependencies**

```bash
conda run -n piper_pink python -m pip install --no-deps -e /home/guoyi/gy_ws/piper_grasp_shen
```

- [ ] **Step 4: Verify import origins and dependency consistency**

Print each module's `__file__`, require `piper_pink` to resolve under `/home/guoyi/gy_ws/piper_grasp_shen/src`, then run `python -m pip check`.

### Task 6: Project Guides and Safe Diagnostics

**Files:**
- Create: `docs/STRUCTURE.md`
- Create: `docs/DEVICE_SETUP.md`
- Modify: `README.md`
- Modify: `scripts/verify_offline.sh`

**Interfaces:**
- Consumes: stable scripts and verified environment behavior from Tasks 1–5.
- Produces: one structure guide, one device runbook, and a portable offline verification command.

- [ ] **Step 1: Update offline verification to use `scripts/run.sh` and disable cache writes**

The script must compile source without writing beside it, run pytest with `-p no:cacheprovider`, run `doctor`, and generate its report under `/tmp`.

- [ ] **Step 2: Write `STRUCTURE.md`**

Document each top-level directory, identify `runtime` as generated history, identify normalized YAML versus raw JSON calibration roles, and list stable user-facing scripts.

- [ ] **Step 3: Write `DEVICE_SETUP.md`**

Provide exact commands for environment activation, imports, ROS sourcing, USB/CAN read-only checks, TF publishers, AprilTag nodes, D435i pose capture, offline planning, and staged motion entrypoints. Mark every motion command clearly and do not execute it.

- [ ] **Step 4: Replace stale `/home/shen_tao` README commands**

Use `/home/guoyi/gy_ws/piper_grasp_shen` or project-relative commands and link the new guides.

### Task 7: Cleanup and End-to-End Verification

**Files:**
- Remove generated directories: all project `__pycache__` directories and obsolete `src/piper_grasp.egg-info` before editable reinstall if safe.
- Preserve: every file beneath `runtime/`.

**Interfaces:**
- Consumes: all earlier tasks.
- Produces: verified project status and an evidence-backed list of any hardware-only blockers.

- [ ] **Step 1: Remove only reproducible caches and confirm runtime count is unchanged**

Record `find runtime -type f | wc -l`, remove caches, and verify the same count afterward.

- [ ] **Step 2: Run the complete Python suite**

```bash
PYTHONPATH=src PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/guoyi/anaconda3/envs/piper_pink/bin/python -m pytest -q -p no:cacheprovider
```

Expected: zero failures.

- [ ] **Step 3: Run offline verification**

```bash
./scripts/verify_offline.sh
```

Expected: model load, collision geometry, doctor, and example grasp planning pass without CAN access.

- [ ] **Step 4: Run read-only device diagnostics**

Check `lsusb`, `ip -details link show can0`, ROS node/topic lists, required camera topics, `/piper_x/feedback/joint_states`, and TF availability. Do not invoke any script requiring an execution confirmation token.

- [ ] **Step 5: Report exact readiness**

State separately whether software tests pass, D435i is detected, Gemini 336L is detected, CAN exists, Piper feedback is live, and required TF chains resolve. Do not infer hardware readiness from software-only tests.
