#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -lt 2 ]]; then
  echo "usage: run_thermal_camera.sh <python> <thermal_web.py> [args...]" >&2
  exit 2
fi

PYTHON_BIN="$1"
shift
DEPENDENCY_DIR="${USB_CAMERA_PYTHONPATH:-${HIKVISION_PYTHONPATH:-${HOME}/.local/share/hikvision-camera/python}}"

export PYTHONPATH="${DEPENDENCY_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${PYTHON_BIN}" "$@"
