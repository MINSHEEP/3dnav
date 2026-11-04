#!/usr/bin/env bash
# Install only into a NEW dedicated Conda prefix. Existing prefixes are read-only.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
task_reuse=0
task_conda="${CONDA_EXE:-}"
task_prefix="${PPHUMAN_CONDA_PREFIX:-}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --conda) task_conda="${2:?missing conda executable}"; shift 2 ;;
    --prefix) task_prefix="${2:?missing environment prefix}"; shift 2 ;;
    --reuse-env) task_reuse=1; shift ;;
    -h|--help)
      printf '%s\n' 'Usage: bash scripts/setup_ubuntu_cpu.sh [--conda /path/bin/conda] [--prefix /path/envs/pp_humanv2] [--reuse-env]' \
        'Creates pp_humanv2 if absent; --reuse-env never installs into an existing environment.'
      exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[ "$(uname -s)" = Linux ] && [ "$(uname -m)" = x86_64 ] || { echo 'Requires Linux x86_64.' >&2; exit 1; }
[ -f /opt/ros/humble/setup.bash ] || { echo 'Install ROS 2 Humble first; see README.' >&2; exit 1; }
/usr/bin/python3 -c 'import cv2, numpy, PIL, yaml, gi'
/usr/bin/python3 scripts/verify_runtime_assets.py
# Git clones restore the pinned source archive; standalone ZIPs already include it.
if [ ! -f vendor/PaddleDetection/deploy/pipeline/pphuman/video_action_infer.py ] || [ ! -f vendor/PaddleDetection/deploy/python/infer.py ]; then
  [ -f scripts/setup_vendor.py ] || { echo "Missing bundled PaddleDetection source; restore the deployment archive." >&2; exit 1; }
  /usr/bin/python3 scripts/setup_vendor.py
fi
if [ -z "$task_conda" ]; then task_conda="$(command -v conda || true)"; fi
if [ -z "$task_conda" ]; then
  for task_candidate in "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" "$HOME/miniforge3/bin/conda"; do
    if [ -x "$task_candidate" ]; then task_conda="$task_candidate"; break; fi
  done
fi
if [ -z "$task_prefix" ]; then
  [ -x "$task_conda" ] || { echo 'Conda not found. Pass --conda /path/bin/conda.' >&2; exit 1; }
  task_prefix="$("$task_conda" info --base)/envs/pp_humanv2"
fi
task_prefix="$(/usr/bin/python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).expanduser().resolve())' "$task_prefix")"
task_python="$task_prefix/bin/python"
if [ -e "$task_prefix" ]; then
  [ "$task_reuse" -eq 1 ] || { echo 'Environment exists; no changes made. Pass --reuse-env or choose a new --prefix.' >&2; exit 1; }
  [ -x "$task_python" ] || { echo 'Existing prefix has no Python.' >&2; exit 1; }
else
  [ "$task_reuse" -eq 0 ] || { echo '--reuse-env requires an existing environment.' >&2; exit 1; }
  [ -x "$task_conda" ] || { echo 'Conda not found.' >&2; exit 1; }
  env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 "$task_conda" create -y -p "$task_prefix" python=3.10 pip
  env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 "$task_python" -m pip install -r environment/requirements-cpu.lock.txt
fi
env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 "$task_python" -c \
  'import sys; assert sys.version_info[:2] == (3,10), "Requires Python 3.10"; import paddle, openvino, cv2, numpy, yaml'
env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 "$task_python" -m pip check
/usr/bin/python3 scripts/configure_local_environment.py --prefix "$task_prefix"
bash -c 'set -e; source scripts/start_common.sh; /usr/bin/python3 -c "import rclpy, sensor_msgs, std_msgs, pphuman_ros.recognition"'
printf '\nCPU environment configured. No camera or model inference was started.\n'
printf '%s\n' 'Start: bash scripts/start_all.sh'
