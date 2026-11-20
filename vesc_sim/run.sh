#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaac_install="${ISAAC_INSTALL:-/home/shin/isaac_sim_6_2}"
# The bundled CUDA runtime must precede CloudXR's older libcudart on this host.
cuda_runtime="$isaac_install/exts/isaacsim.pip.nv/pip_prebundle/nvidia/cuda_runtime/lib"
if [[ -d "$cuda_runtime" ]]; then
  export LD_LIBRARY_PATH="$cuda_runtime${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
vesc_ros_ws="${VESC_ROS_WS:-/home/shin/damvi_vesc_ros_ws}"
if [[ -f "$vesc_ros_ws/install/setup.bash" ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$vesc_ros_ws/install/setup.bash"
  set -u
fi
exec "$isaac_install/python.sh" "$task_dir/run_isaac.py" "$@"
