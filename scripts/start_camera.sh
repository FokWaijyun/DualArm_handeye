#!/usr/bin/env bash
set -eo pipefail
SERIAL_NUMBER="${SERIAL_NUMBER:-}"
source /opt/ros/humble/setup.bash
if [[ -z "$SERIAL_NUMBER" ]]; then
  echo "请设置 SERIAL_NUMBER" >&2
  exit 2
fi
exec ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera_01 serial_number:="$SERIAL_NUMBER" enable_depth:=false enable_left_ir:=false enable_right_ir:=false
