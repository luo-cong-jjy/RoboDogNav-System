#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."

if [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: Cannot find /opt/ros/foxy/setup.bash or /opt/ros/humble/setup.bash" >&2
  exit 1
fi

colcon build --packages-up-to rslidar_sdk --symlink-install
