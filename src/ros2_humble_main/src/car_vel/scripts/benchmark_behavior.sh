#!/usr/bin/env bash
set -euo pipefail
behavior_package="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
behavior_python="${BEHAVIOR_PYTHON:-${HOME}/.local/share/robot-behavior/venv/bin/python}"
cd -- "$behavior_package/person_behavior"
exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
  PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
  "$behavior_python" -s -m pphuman_runtime.benchmark "$@"
