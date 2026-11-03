#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/ros_env.sh"
exec ros2 run dum_e_control arm "$@"
