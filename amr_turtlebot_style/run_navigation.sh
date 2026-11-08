#!/usr/bin/env bash
# TurtleBot-style baseline: existing map + AMCL + Nav2 + RViz.
# PREVIEW ONLY: no CAN owner, no motor authorization, no lift actuation.
set -eo pipefail
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
source /opt/ros/jazzy/setup.bash
source "$HOME/real_amr/install/setup.bash"
MAP="$HOME/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml"
[[ -f "$MAP" ]] || { echo "[ERROR] Existing map missing: $MAP" >&2; exit 1; }
# IPC-local ROS graph avoids broadcasting DDS sensor traffic to the office LAN.
# All cooperating ROS2 nodes must use the same DDS configuration.
if [[ "${AMR_LOCAL_DDS:-1}" == "1" ]]; then
  DDS="$HOME/.config/cyclonedds/amr_local.xml"
  if [[ ! -f "$DDS" ]]; then
    echo "[ERROR] Local DDS configuration missing: $DDS" >&2
    echo "Create it first or run with AMR_LOCAL_DDS=0 for your existing DDS settings." >&2
    exit 1
  fi
  export CYCLONEDDS_URI="file://$DDS"
fi
ROOT="$HOME/hana-amr-project-autonomous"
echo "[AMR] Existing map: $MAP"
echo "[AMR] TurtleBot-style baseline: map, localization, Nav2, RViz (NO MOTOR)"
exec bash "$ROOT/amr_autonomous/run_all.sh" "map_file:=$MAP" "physical_auto:=false" "$@"
