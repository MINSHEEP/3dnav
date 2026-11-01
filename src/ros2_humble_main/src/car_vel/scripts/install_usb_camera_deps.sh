#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Reuse the established, ROS-safe dependency installer.  It installs a CPU
# PyTorch build into a private target directory instead of replacing ROS2's
# system Python packages.
export HIKVISION_PYTHONPATH="${USB_CAMERA_PYTHONPATH:-${HOME}/.local/share/hikvision-camera/python}"
exec /bin/bash "${SCRIPT_DIR}/install_hikvision_camera_deps.sh" "$@"
