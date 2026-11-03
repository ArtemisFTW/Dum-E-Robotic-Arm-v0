#!/usr/bin/env bash
set -eo pipefail
exec "$(dirname -- "${BASH_SOURCE[0]}")/run_ros2.sh" gui:=true "$@"
