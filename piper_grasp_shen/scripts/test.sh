#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "${PROJECT_DIR}"
exec env -u PYTHONPATH PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "${PROJECT_DIR}/.conda/bin/python" -m pytest -q

