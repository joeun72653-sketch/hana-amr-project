#!/usr/bin/env bash
# Supervised REAL CAN motor operation. Not a simulation.
set -eo pipefail
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
unset AMR_BENCH_BYPASS_DISARM || true

source /opt/ros/jazzy/setup.bash
source "$HOME/real_amr/install/setup.bash"

APP="$HOME/hana-amr-project-autonomous/amr_autonomous"
PORT="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG04BIQZ-if00-port0"

if [[ ! -e "$PORT" ]]; then
  echo "[BLOCKED] FT232R USB-CAN not present: $PORT" >&2; exit 1
fi
if [[ ! -r "$PORT" || ! -w "$PORT" ]]; then
  echo "[BLOCKED] USB-CAN access denied" >&2; exit 1
fi
if [[ ! -f "$HOME/amr_websocket/amr_openvino_camera_depth_integrated.py" ]]; then
  echo "[BLOCKED] Depth camera integrated script missing" >&2; exit 1
fi
if ss -ltn | grep -Eq ':(8000|8771)[[:space:]]'; then
  echo "[BLOCKED] Existing phone or map server uses :8000 or :8771. Stop old launch first." >&2
  ss -lntp | grep -E ':(8000|8771)[[:space:]]' || true
  exit 1
fi
if pgrep -af 'amr_ros_wheel_rpm_can.py|amr_ws_to_ros.py|amr_one_command_preview.launch.py' | grep -v 'run_live.sh' >/dev/null; then
  echo "[BLOCKED] Old CAN/manual/preview process detected. Review with pgrep -af first." >&2
  pgrep -af 'amr_ros_wheel_rpm_can.py|amr_ws_to_ros.py|amr_one_command_preview.launch.py' || true
  exit 1
fi

echo "[LIVE MOTOR] This enables CANopen motor drives and can move the AMR."
echo "Verify hardware E-stop, drive-side cable-loss stop, cleared path and supervision."
echo "30 RPM requested initially. Web operator may select up to 1000 RPM."
read -r -p "Type START to enable supervised CAN hardware (anything else cancels): " answer
if [[ "$answer" != "START" ]]; then
  echo "[CANCELED] No CAN process started."
  exit 1
fi

echo "[LIVE] Starting hardware CAN with guarded mux and Nav2 goal manager"
exec ros2 launch "$APP/amr_one_command_live.launch.py" "$@"
