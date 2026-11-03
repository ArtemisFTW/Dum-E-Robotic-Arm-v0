#!/usr/bin/env bash
# Source this from the repository's executable helpers.
DUM_E_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$DUM_E_ROOT/ros2_ws/install/setup.bash" ]]; then
  printf 'Build the workspace with ./build_ros2.sh first.\n' >&2
  return 1
fi
source "$DUM_E_ROOT/ros2_ws/install/setup.bash"
export ROS_LOG_DIR="$DUM_E_ROOT/.ros/log"
export ROS_HOME="$DUM_E_ROOT/.ros"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
mkdir -p "$ROS_LOG_DIR"
