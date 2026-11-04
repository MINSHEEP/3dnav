#!/usr/bin/env bash
# Explicit installation; the gateway never installs dependencies at startup.
set -euo pipefail
behavior_package="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
behavior_root="${behavior_package}/person-behavior-recognition"
behavior_home="${BEHAVIOR_HOME:-${HOME}/.local/share/robot-behavior}"
behavior_python="${BEHAVIOR_BOOTSTRAP_PYTHON:-/usr/bin/python3}"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
"$behavior_python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Use Python 3.10"'
"$behavior_python" "$behavior_root/scripts/verify_runtime_assets.py"
"$behavior_python" -m venv "$behavior_home/venv"
"$behavior_home/venv/bin/python" -m pip install -r "$behavior_package/requirements-behavior-cpu.txt"
"$behavior_home/venv/bin/python" -m pip check
echo "Installed OpenVINO INT8 + Paddle CPU runtime: $behavior_root"
echo "Requires system fonts-noto-cjk and python3-pil for the upstream preview."
echo "Bundled offline speech playback: sudo apt-get install ffmpeg"
echo "Optional offline speech fallback: sudo apt-get install espeak-ng"
