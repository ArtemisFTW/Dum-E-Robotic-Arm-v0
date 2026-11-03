#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/ros2_ws"
source /opt/ros/jazzy/setup.bash
# Copy install is intentional: it remains valid when the project folder moves.
colcon build --packages-up-to dum_e_control --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
