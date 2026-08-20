#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/eye_to_grasp"
POSE_FILE="${RUN_DIR}/gemini336l_relative_locked_pose.yaml"
RECIPE_FILE="${RUN_DIR}/relative_observation_recipe.yaml"
REPORT_FILE="${RUN_DIR}/relative_observation_report.yaml"
CANDIDATE_DIR="${RUN_DIR}/observation_candidates"
MULTI_RECORD="${RUN_DIR}/observation_candidates.yaml"
LEGACY_RECORD="${RUN_DIR}/dual_camera_observation.yaml"
EXACT_DIR="${CANDIDATE_DIR}/exact"
FALLBACK_DIR="${CANDIDATE_DIR}/translation_fallbacks"

[[ "${1:-}" == "I_CONFIRM_EYE_TO_GRASP_RELATIVE" ]] || { echo "Required confirmation: I_CONFIRM_EYE_TO_GRASP_RELATIVE" >&2; exit 2; }
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || { echo "Required confirmation: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2; }
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || { echo "Required confirmation: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2; exit 2; }

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
mkdir -p "${RUN_DIR}"

echo "STAGE 1/3: capture current Gemini Tag pose"
python3 "${SCRIPT_DIR}/capture_global_apriltag_pose.py" \
  --base-frame piper_x/base_link --tag-frame gemini_apriltag_0 \
  --samples 10 --timeout-s 30 --output "${POSE_FILE}"

mapfile -t CURRENT_Q_DEG < <(python3 - "${POSE_FILE}" <<'PY'
import sys, yaml
for value in yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["current_q_deg"]:
    print(value)
PY
)

rm -rf -- "${EXACT_DIR}" "${FALLBACK_DIR}"
mkdir -p "${EXACT_DIR}" "${FALLBACK_DIR}"

echo "STAGE 2/3: EXACT TAUGHT CANDIDATES"
if [[ -s "${MULTI_RECORD}" ]]; then
  recipe_args=(--record "${MULTI_RECORD}" --output-dir "${EXACT_DIR}")
  if [[ -s "${LEGACY_RECORD}" ]]; then
    recipe_args+=(--legacy-record "${LEGACY_RECORD}")
  fi
  "${PROJECT_DIR}/.conda/bin/python" \
    "${SCRIPT_DIR}/make_observation_candidate_recipes.py" \
    "${recipe_args[@]}"
else
  echo "No multi-candidate record; using the legacy taught observation."
  python3 "${SCRIPT_DIR}/make_taught_observation_recipe.py" \
    --global-pose "${POSE_FILE}" \
    --record "${LEGACY_RECORD}" \
    --output "${EXACT_DIR}/candidate_01.yaml"
fi

planned=false
shopt -s nullglob
exact_candidates=("${EXACT_DIR}"/candidate_*.yaml)
for candidate in "${exact_candidates[@]}"; do
  echo "trying_exact_observation_candidate: $(basename "${candidate}")"
  if "${PROJECT_DIR}/scripts/run.sh" plan-grasp \
      --object-pose "${POSE_FILE}" --recipe "${candidate}" \
      --current-q-deg "${CURRENT_Q_DEG[@]}" --minimum-tcp-z-m 0.02 \
      --report "${REPORT_FILE}"; then
    cp "${candidate}" "${RECIPE_FILE}"
    planned=true
    break
  fi
done

if [[ "${planned}" != true ]]; then
  echo "All exact taught candidates failed; trying TRANSLATION FALLBACKS."
  "${PROJECT_DIR}/.conda/bin/python" - "${EXACT_DIR}" "${FALLBACK_DIR}" <<'PY'
import sys, yaml
from pathlib import Path
source_dir = Path(sys.argv[1])
out = Path(sys.argv[2])
offsets = [
    (30, 0, 0), (-30, 0, 0), (0, 30, 0), (0, -30, 0),
    (0, 0, 20), (0, 0, -20), (20, 20, 0), (-20, -20, 0),
]
output_index = 1
for source in sorted(source_dir.glob("candidate_*.yaml")):
    src = yaml.safe_load(source.read_text(encoding="utf-8"))
    for dx, dy, dz in offsets:
        item = dict(src)
        matrix = [row[:] for row in src["object_from_tcp"]]
        matrix[0][3] += dx / 1000.0
        matrix[1][3] += dy / 1000.0
        matrix[2][3] += dz / 1000.0
        item["object_from_tcp"] = matrix
        item["axial_angles_deg"] = [0.0]
        item["candidate_offset_tag_mm"] = [dx, dy, dz]
        item["fallback_source"] = source.name
        target = out / f"candidate_{output_index:03d}.yaml"
        target.write_text(yaml.safe_dump(item, sort_keys=False), encoding="utf-8")
        output_index += 1
PY

  fallback_candidates=("${FALLBACK_DIR}"/candidate_*.yaml)
  for candidate in "${fallback_candidates[@]}"; do
    echo "trying_translation_fallback: $(basename "${candidate}")"
    if "${PROJECT_DIR}/scripts/run.sh" plan-grasp \
        --object-pose "${POSE_FILE}" --recipe "${candidate}" \
        --current-q-deg "${CURRENT_Q_DEG[@]}" --minimum-tcp-z-m 0.02 \
        --report "${REPORT_FILE}"; then
      cp "${candidate}" "${RECIPE_FILE}"
      planned=true
      break
    fi
  done
fi
if [[ "${planned}" != true ]]; then
  echo "No exact or translation-only observation candidate passed planning." >&2
  exit 5
fi

echo "STAGE 3/3: move to Tag-relative pre-observation pose"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" "${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" --confirmation I_CONFIRM_ROS_PREGRASP_ONLY \
  --mode pregrasp --gripper-open-m 0.085 --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 --timeout-s 45
echo "RELATIVE_OBSERVATION_COMPLETE"
