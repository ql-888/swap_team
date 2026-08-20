# Multiple Observation Candidates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record and select multiple safe D435i pre-observation poses before using translation-only fallbacks.

**Architecture:** A ROS recorder stores grouped candidate samples. A pure Python converter averages each taught Tag-to-flange transform and writes planner-compatible recipes. The existing shell orchestrator tries exact recipes first and translation-only fallbacks second.

**Tech Stack:** ROS 2 Humble, Python, NumPy, SciPy, OpenCV, YAML, Bash.

**Spec:** `docs/superpowers/specs/2026-08-20-multi-observation-candidates-design.md`

## Global Constraints

- The recorder must not publish robot commands.
- Each candidate must include Gemini TF, D435i TF, flange TF, joints, and images.
- Exact taught poses run before positional fallbacks.
- No fallback may rotate around the approach/flange axis.
- Preserve the legacy single-candidate record when no new record exists.

---

### Task 1: Candidate transform conversion

**Files:**
- Create: `scripts/eye_to_grasp/make_observation_candidate_recipes.py`
- Create: `tests/test_observation_candidates.py`

**Interfaces:**
- Consumes: schema-version-2 candidate YAML.
- Produces: `build_candidate_recipes(record: dict) -> list[dict]`.

- [ ] Write tests for transform averaging, recorded order, and `[0.0]` axial angles.
- [ ] Run the focused test and verify the missing implementation fails.
- [ ] Implement the smallest converter that passes.
- [ ] Run the focused test and verify it passes.

### Task 2: Interactive candidate recorder

**Files:**
- Create: `scripts/eye_to_grasp/record_observation_candidates.py`

**Interfaces:**
- Consumes: ROS image, TF, and joint feedback topics.
- Produces: `runtime/eye_to_grasp/observation_candidates.yaml` schema version 2.

- [ ] Add a static safety test proving the recorder creates no publishers.
- [ ] Run it and verify failure before the recorder exists.
- [ ] Implement Space-to-capture and Q-to-save behavior with ten fresh samples per pose.
- [ ] Run syntax and safety tests.

### Task 3: Exact-first runtime selection

**Files:**
- Modify: `scripts/eye_to_grasp/run_relative_observation.sh`
- Modify: `tests/test_observation_candidates.py`

**Interfaces:**
- Consumes: generated exact candidate recipes.
- Produces: selected `relative_observation_recipe.yaml` and plan report.

- [ ] Add a shell-policy test for exact-before-offset ordering and absence of 90/180/270 rotations.
- [ ] Run the test and verify current behavior fails.
- [ ] Implement exact candidate planning followed by translation-only fallbacks.
- [ ] Run focused tests and `bash -n`.

### Task 4: Operator instructions

**Files:**
- Modify: `scripts/eye_to_grasp/README.md`

- [ ] Document camera/TF prerequisites and Space/Q controls.
- [ ] Document that manual positioning is operator-controlled and no motion command is sent.
- [ ] Run the full offline test subset and final syntax checks.
