#!/usr/bin/env bash
# Explicit deployment operation. Never called by camera, launch, or startup.
set -euo pipefail
behavior_package="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
behavior_root="${behavior_package}/person_behavior"
behavior_home="${BEHAVIOR_HOME:-${HOME}/.local/share/robot-behavior}"
behavior_bootstrap="${BEHAVIOR_BOOTSTRAP_PYTHON:-/usr/bin/python3}"
behavior_venv="${behavior_home}/venv"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "This installer targets Ubuntu 22.04 x86_64 with Python 3.10 (ROS Humble host)." >&2
  exit 2
fi
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
"$behavior_bootstrap" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Use Python 3.10 for the pinned CPU wheels"'
cd -- "$behavior_root"
"$behavior_bootstrap" -s -m pphuman_runtime.assets
if [[ -e "$behavior_venv" && ! -f "$behavior_venv/pyvenv.cfg" ]]; then
  echo "Refusing to replace an existing non-venv path: $behavior_venv" >&2
  exit 2
fi
"$behavior_bootstrap" -m venv "$behavior_venv"
"$behavior_venv/bin/python" -m pip install --upgrade 'pip==24.3.1' 'setuptools==75.6.0' 'wheel==0.45.1'
"$behavior_venv/bin/python" -m pip install -r "$behavior_package/requirements-behavior-cpu.txt"
"$behavior_venv/bin/python" -m pip check
"$behavior_venv/bin/python" -s -m pphuman_runtime.assets --prepare-vendor "$behavior_home/vendor/PaddleDetection"
"$behavior_venv/bin/python" -s - <<'PY'
import cv2, numpy, onnxruntime, paddle
assert numpy.__version__ == '1.26.4'
assert cv2.__version__.startswith('4.6.')
assert paddle.__version__ == '2.6.2' and not paddle.is_compiled_with_cuda()
assert onnxruntime.__version__ == '1.18.0'
assert 'CPUExecutionProvider' in onnxruntime.get_available_providers()
assert 'CUDAExecutionProvider' not in onnxruntime.get_available_providers()
print('CPU imports verified. No model accuracy or throughput claim has been made.')
PY
"$behavior_venv/bin/python" -s -m pphuman_runtime.self_check --timeout 90
echo "Installed isolated CPU environment: $behavior_venv"
echo "Optional offline Chinese speech: sudo apt-get install espeak-ng"
echo "Optional cached MP3 playback: sudo apt-get install ffmpeg"
echo "Run scripts/benchmark_behavior.sh --video /absolute/path/video.mp4 --output /tmp/behavior-benchmark.json"
