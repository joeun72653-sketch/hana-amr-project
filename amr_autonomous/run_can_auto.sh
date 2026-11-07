#!/usr/bin/env bash
# Separate physical CAN stack for run_all.sh physical_auto:=true.
# Does NOT launch Nav2, RViz, LiDAR, map or web (no duplicate ROS nodes).
set -eo pipefail
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset AMR_BENCH_BYPASS_DISARM || true
source /opt/ros/jazzy/setup.bash
source "$HOME/real_amr/install/setup.bash"
APP="$HOME/hana-amr-project-autonomous/amr_autonomous"
REMOTE="$HOME/hana-amr-project-autonomous/amr_remote_client"
PORT="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG04BIQZ-if00-port0"
CAMERA="$HOME/amr_websocket/amr_openvino_camera_depth_integrated.py"
CAMERA_PY="$HOME/yolo_openvino_env/bin/python"
LIFT="$HOME/amr_lift/lift_websocket.py"
LIFT_PY="$HOME/amr_lift/.venv/bin/python"
for f in "$PORT" "$CAMERA" "$CAMERA_PY" "$LIFT" "$LIFT_PY" "$APP/scripts/guarded_motor_mux.py" "$REMOTE/amr_ros_wheel_rpm_can.py"; do
  [[ -e "$f" ]] || { echo "[BLOCKED] Missing: $f"; exit 1; }
done
[[ -r "$PORT" && -w "$PORT" ]] || { echo "[BLOCKED] CAN port access denied"; exit 1; }
if pgrep -af 'amr_ros_wheel_rpm_can.py|guarded_motor_mux.py|amr_openvino_camera_depth_integrated.py|lift_websocket.py' | grep -vE 'grep|run_can_auto.sh' >/dev/null; then
  echo "[BLOCKED] Existing motor/safety process; inspect duplicate owners:"
  pgrep -af 'amr_ros_wheel_rpm_can.py|guarded_motor_mux.py|amr_openvino_camera_depth_integrated.py|lift_websocket.py'
  exit 1
fi
if ! curl -fsS --max-time 3 http://127.0.0.1:8771/api/status | /usr/bin/python3 -c 'import json,sys; d=json.load(sys.stdin); assert d["physical_auto_allowed"] is True, "Web server is preview-only"'; then
  echo "[BLOCKED] Start run_all.sh physical_auto:=true first, then initialize AMCL/Nav2."
  exit 1
fi
for n in /controller_server /planner_server /bt_navigator; do
  state=$(timeout 5 ros2 lifecycle get "$n" 2>/dev/null || true)
  [[ "$state" == *"active [3]"* ]] || { echo "[BLOCKED] $n not active: $state"; exit 1; }
done
/usr/bin/python3 -m py_compile "$APP/scripts/guarded_motor_mux.py" "$REMOTE/amr_ros_wheel_rpm_can.py"
echo "[PHYSICAL AUTO] Separate CAN/depth/lift-state stack; 30 RPM HARD CAP."
echo "Requires verified E-stop, drive-side link-loss stop, lifted wheels and supervision."
read -r -p "Type START to allow real CAN motor output: " approval
[[ "$approval" == START ]] || { echo "[CANCELED]"; exit 1; }
pids=()
cleanup() {
  trap - EXIT INT TERM
  echo "[STOP] Stopping CAN stack; verify actual wheels stopped and use hardware E-stop if needed."
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
"$CAMERA_PY" -u "$CAMERA" & pids+=($!)
"$LIFT_PY" -u "$LIFT" & pids+=($!)
/usr/bin/python3 -u "$APP/scripts/guarded_motor_mux.py" --live-motor-output & pids+=($!)
sleep 3
/usr/bin/python3 -u "$REMOTE/amr_ros_wheel_rpm_can.py" --mux-input & pids+=($!)
echo "[CAN AUTO] Processes started; AUTO start requires fresh depth, lift, CAN feedback and web heartbeat."
echo "[CAN AUTO] Ctrl+C stops this stack. Keep run_all.sh running separately."
while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[FAULT] A required process exited ($pid); stopping all."
      exit 1
    fi
  done
  sleep 0.25
done
