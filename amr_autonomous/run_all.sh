#!/usr/bin/env bash
# Start the existing real AMR mapping/localization/nav preview stack in ONE terminal.
# This is NON-ACTUATING: no CAN, ARM, Lift, or NavigateToPose requests.
# ROS 2 setup.bash references optional variables; nounset (-u) breaks Jazzy environment setup.
set -eo pipefail

# Explicit live mode shares the same saved-map/AMCL/Nav2/web stack,
# but delegates all hardware interlocks and CAN ownership to run_live.sh.
# Default invocation stays strictly non-actuating.
if [[ "${1:-}" == "--live-auto" ]]; then
  shift
  echo "[AMR] Explicit LIVE AUTO requested. Delegating to supervised CAN launch."
  exec bash "$HOME/hana-amr-project-autonomous/amr_autonomous/run_live.sh" "$@"
fi
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: bash amr_autonomous/run_all.sh [--live-auto]"
  echo "Default: read-only map and route preview, no motor commands."
  echo "--live-auto: guarded physical CAN with START confirmation and 30 RPM cap."
  exit 0
fi

export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

source /opt/ros/jazzy/setup.bash
source "$HOME/real_amr/install/setup.bash"

APP="$HOME/hana-amr-project-autonomous/amr_autonomous"
LAUNCH="$APP/amr_one_command_preview.launch.py"

if [[ ! -f "$LAUNCH" ]]; then
  echo "[ERROR] Launch not found: $LAUNCH" >&2
  exit 1
fi
if [[ ! -f "$HOME/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml" ]]; then
  echo "[ERROR] Map YAML not found. Check saved map path." >&2
  exit 1
fi
# Guard against duplicate phone/map server processes.
if ss -ltn | grep -Eq ':(8000|8771)[[:space:]]'; then
  echo "[ERROR] Port 8000 or 8771 already in use. Stop the OLD phone/map server first." >&2
  ss -ltnp | grep -E ':(8000|8771)[[:space:]]' || true
  exit 1
fi

echo "[AMR] Starting preview stack (NO CAN, NO MOTOR, NO LIFT)"
echo "[AMR] One Ctrl+C will stop the launched processes."
exec ros2 launch "$LAUNCH" "$@"
