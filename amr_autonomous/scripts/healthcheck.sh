#!/usr/bin/env bash
# Read-only status checks. No motor commands, ROS publications or Goal actions.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/source_ros_env.sh" || exit 1

run_check() {
  local label="$1"
  shift
  printf '\n=== %s ===\n' "$label"
  if timeout 8 "$@"; then
    return 0
  else
    printf '[INFO] Not available or timed out: %s\n' "$label"
    return 0
  fi
}

printf '\n[AMR_QR] READ ONLY: ROS graph may change as nodes start/stop.\n'
run_check "real_amr_lidar package" ros2 pkg executables real_amr_lidar
run_check "ROS nodes" ros2 node list
run_check "ROS topics/types" ros2 topic list -t
run_check "ROS actions/types" ros2 action list -t

for topic in /qr/raw /qr/status /qr/pose /scan_slam /lidar_icp/odom /map /amcl_pose /mapex/cmd_vel /mapex/wheel_rpm_left_requested /mapex/wheel_rpm_right_requested /amr_motor_state /amr_motion_authorized; do
  run_check "Topic: $topic" ros2 topic info "$topic" -v
done

for node in /map_server /amcl /planner_server /controller_server /bt_navigator /behavior_server; do
  run_check "Lifecycle: $node" ros2 lifecycle get "$node"
done

printf '\n[AMR_QR] For TF, use separate terminals:\n'
printf '  ros2 run tf2_ros tf2_echo map odom\n'
printf '  ros2 run tf2_ros tf2_echo odom base_link\n'
printf '  ros2 run tf2_ros tf2_echo map base_link\n'
printf '\n[AMR_QR] Healthcheck completed (NOT a safety clearance).\n'
