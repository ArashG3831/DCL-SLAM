#!/usr/bin/env bash
# Source this file on the Raspberry Pi/real-robot Wi-Fi network.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source $0" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
if [[ -n "${ROS_CYCLONEDDS_PREFIX:-}" && -d "${ROS_CYCLONEDDS_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${ROS_CYCLONEDDS_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${ROS_CYCLONEDDS_PREFIX}/lib:${ROS_CYCLONEDDS_PREFIX}/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_ws_root}/config/cyclonedds/real_robot_wifi.xml"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
printf 'ROS profile: real-robot CycloneDDS Wi-Fi (wlan0)\nRMW_IMPLEMENTATION=%s\nCYCLONEDDS_URI=%s\n' "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI"
