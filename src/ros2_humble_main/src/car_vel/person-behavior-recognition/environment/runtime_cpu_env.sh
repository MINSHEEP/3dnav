# Source only in the model child; ROS stays in the system Python environment.
export PPHUMAN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$PPHUMAN_ROOT/config/local.env.sh" ]; then
  source "$PPHUMAN_ROOT/config/local.env.sh"
fi
if [ -z "${PPHUMAN_CONDA_PREFIX:-}" ] || [ ! -x "$PPHUMAN_CONDA_PREFIX/bin/python" ]; then
  echo 'Run bash scripts/setup_ubuntu_cpu.sh before starting this CPU package.' >&2
  exit 1
fi
export PPHUMAN_PYTHON="$PPHUMAN_CONDA_PREFIX/bin/python"
export LD_LIBRARY_PATH="$PPHUMAN_CONDA_PREFIX/lib:$PPHUMAN_CONDA_PREFIX/lib/python3.10/site-packages/paddle/libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1
unset PYTHONHOME PYTHONPATH
