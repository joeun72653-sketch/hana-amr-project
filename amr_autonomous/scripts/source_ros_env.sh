#!/usr/bin/env bash
# Source this file in every ROS2 terminal:
#   source /path/to/hana-amr-project/amr_autonomous/scripts/source_ros_env.sh
# This does not start nodes or move hardware.

if [[ "$BASH_SOURCE" == "$0" ]]; then
  echo "[ERROR] Execute with 'source', not 'bash': source $0" >&2
  exit 2
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[ERROR] ROS2 Jazzy missing: /opt/ros/jazzy/setup.bash" >&2
  return 1
fi

source /opt/ros/jazzy/setup.bash || return 1
if [[ ! -f "$HOME/real_amr/install/setup.bash" ]]; then
  echo "[ERROR] Build/install real_amr overlay first: $HOME/real_amr/install/setup.bash" >&2
  return 1
fi
source "$HOME/real_amr/install/setup.bash" || return 1

export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

printf '[AMR_QR ENV] ROS_DISTRO=%s ROS_DOMAIN_ID=%s RMW=%s\n' \
  "$ROS_DISTRO" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
