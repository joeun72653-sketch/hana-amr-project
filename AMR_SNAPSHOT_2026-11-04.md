# AMR integrated snapshot — 2026-11-04

Branch: `amr-integrated-lift-target-accel-20261104`

## Confirmed on Ubuntu IPC (reported by operator)

- ROS 2 Jazzy integrated launch `amr_all_with_lift.launch.py` with `bench_motion:=true`.
- CAN motor control and 60 RPM/s software acceleration for driving and pivoting.
- OpenVINO + Depth + Web dashboard integrated execution.
- Lift target position from the Web dashboard: both lift target pulses equal; target arrival tolerance ±20 pulses.
- Smooth *button-release-to-zero* deceleration is **not** included in this snapshot; preserve existing stop behavior.

## Important

The newest working source files currently live on the Ubuntu IPC. This document does **not** certify that the files in GitHub are already synchronized. Upload the exact running IPC files using the companion snapshot upload instructions. Preserve the wheel CAN safety interlocks, stop verification, Depth gate, and lift motor-stop interlock. `bench_motion:=true` is not a substitute for a hardware emergency stop.

## Standard launch

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 launch ~/amr_remote_client/amr_all_with_lift.launch.py bench_motion:=true
```
