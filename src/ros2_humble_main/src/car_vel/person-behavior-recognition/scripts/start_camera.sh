#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/start_common.sh"
exec /usr/bin/python3 -m pphuman_ros.camera --ros-args --params-file "$PPHUMAN_ROOT/config/camera.yaml" "$@"
