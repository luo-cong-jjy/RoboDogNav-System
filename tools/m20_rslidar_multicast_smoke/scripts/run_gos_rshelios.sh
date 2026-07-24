#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."

source install/setup.bash

exec ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:="$PWD/config/rslidar_multicast_rshelios_gos.yaml"
