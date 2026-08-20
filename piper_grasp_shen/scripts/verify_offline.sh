#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="${PROJECT_DIR}/.conda/bin/python"
REPORT=$(mktemp /tmp/piper_grasp_verify.XXXXXX.yaml)
trap 'rm -f -- "${REPORT}"' EXIT

cd "${PROJECT_DIR}"
env -u PYTHONPATH PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "${PYTHON}" -m pytest -q -p no:cacheprovider
./scripts/run.sh doctor --handeye config/handeye_orbbec_fixed.yaml
./scripts/run.sh plan-grasp \
    --object-pose config/object_pose.example.yaml \
    --recipe config/grasp_recipe.example.yaml \
    --report "${REPORT}"
test -s "${REPORT}"
echo "Offline verification passed. No hardware connection was opened."
