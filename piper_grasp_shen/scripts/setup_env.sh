#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ENV_DIR="${PROJECT_DIR}/.conda"

if ! command -v conda >/dev/null 2>&1; then
    echo "conda was not found. Install Miniconda/Anaconda first." >&2
    exit 1
fi

if [[ ! -x "${ENV_DIR}/bin/python" ]]; then
    conda create -y -p "${ENV_DIR}" --override-channels -c conda-forge \
        --solver libmamba \
        python=3.10 pinocchio=4.1 qpsolvers=4.13 quadprog=0.1.13 \
        numpy=2.2 scipy=1.15 opencv=5 pyyaml pip pytest
fi

env PYTHONNOUSERSITE=1 "${ENV_DIR}/bin/python" -c \
    'import can, cv2, numpy, pinocchio, pink, piper_sdk, qpsolvers, scipy, yaml'
env PYTHONNOUSERSITE=1 "${ENV_DIR}/bin/python" -m pip install --no-deps -e "${PROJECT_DIR}"
env -u PYTHONPATH PYTHONNOUSERSITE=1 "${ENV_DIR}/bin/python" -m piper_pink info
env -u PYTHONPATH PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "${ENV_DIR}/bin/python" -m pytest -q "${PROJECT_DIR}/tests"

echo "Environment ready: ${ENV_DIR}"
echo "Run commands with: ${PROJECT_DIR}/scripts/run.sh info"
