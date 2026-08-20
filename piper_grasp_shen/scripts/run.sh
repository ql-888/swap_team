#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="${PROJECT_DIR}/.conda/bin/python"

if [[ ! -x "${PYTHON}" ]]; then
    echo "Project environment is missing. Run scripts/setup_env.sh first." >&2
    exit 1
fi

cd "${PROJECT_DIR}"
exec env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PYTHON}" -m piper_pink "$@"

