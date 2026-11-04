#!/usr/bin/env bash
set -euo pipefail
behavior_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
behavior_python="${BEHAVIOR_PYTHON:-${HOME}/.local/share/robot-behavior/venv/bin/python}"
if [[ ! -x "$behavior_python" ]]; then
  echo "Behavior CPU environment missing. Run scripts/install_behavior_deps.sh explicitly." >&2
  exit 2
fi
cd -- "$behavior_root"
exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
  PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
  "$behavior_python" -s -m pphuman_runtime.worker --config config/inference.yaml "$@"
