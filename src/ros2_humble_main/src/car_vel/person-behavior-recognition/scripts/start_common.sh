#!/usr/bin/env bash
# Source only. System Python hosts ROS; the configured worker owns Conda/GPU.
set -e
PPHUMAN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PPHUMAN_ROOT
if [ -f "$PPHUMAN_ROOT/config/local.env.sh" ]; then
  source "$PPHUMAN_ROOT/config/local.env.sh"
fi
unset PYTHONHOME
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PPHUMAN_ROOT/ros2_ws/src/pphuman_ros:$PPHUMAN_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$PPHUMAN_ROOT/config/fastdds.xml}"
export ROS_LOG_DIR="$PPHUMAN_ROOT/run/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$PPHUMAN_ROOT"
