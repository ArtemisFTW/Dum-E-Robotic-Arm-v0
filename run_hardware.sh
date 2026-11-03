#!/usr/bin/env bash
set -eo pipefail
DUM_E_PORT="${DUM_E_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
if [[ ! -r "$DUM_E_PORT" || ! -w "$DUM_E_PORT" ]]; then
  printf 'No serial read/write access to %s. See ROS2_GUIDE.md for device access.\n' "$DUM_E_PORT" >&2
  exit 1
fi
# Selecting this launcher leaves serial disabled until arm.sh enable.
exec "$(dirname -- "${BASH_SOURCE[0]}")/run_ros2.sh" dry_run:=false "port:=$DUM_E_PORT" "$@"
