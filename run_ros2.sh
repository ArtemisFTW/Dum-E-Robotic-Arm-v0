#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/ros_env.sh"
exec ros2 launch dum_e_control arm.launch.py "$@"
