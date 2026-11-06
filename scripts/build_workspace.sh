#!/usr/bin/env bash
set -euo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Humble's rosidl_adapter cannot read UTF-8 build paths with file(STRINGS).
build_args=()
if LC_ALL=C grep -q '[^ -~]' <<< "$workspace_root"; then
  workspace_key="$(printf '%s' "$workspace_root" | sha256sum | cut -c1-12)"
  build_args=(--build-base "/tmp/3dnav-build-${workspace_key}")
fi
# Rebuild setup files without recording another workspace as an underlay.
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH -u PKG_CONFIG_PATH \
  bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd -- "$1"
    shift
    colcon build --symlink-install "$@"
  ' bash "$workspace_root" "${build_args[@]}" "$@"
