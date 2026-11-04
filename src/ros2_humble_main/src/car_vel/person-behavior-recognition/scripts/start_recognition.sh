#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/start_common.sh"
_pphuman_params=(--params-file "$PPHUMAN_ROOT/config/ros.yaml")
if [ -f "$PPHUMAN_ROOT/config/ros.local.yaml" ]; then
  _pphuman_params+=(--params-file "$PPHUMAN_ROOT/config/ros.local.yaml")
fi
exec /usr/bin/python3 -m pphuman_ros.recognition --ros-args "${_pphuman_params[@]}" "$@"
