#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."

source install/setup.bash

echo "===== topics ====="
ros2 topic list | grep -Ei 'm20/.*/points|rslidar|points' || true

echo "===== front info ====="
ros2 topic info -v /m20/front/points || true

echo "===== rear info ====="
ros2 topic info -v /m20/rear/points || true

echo "===== front hz ====="
timeout 10 ros2 topic hz /m20/front/points || true

echo "===== rear hz ====="
timeout 10 ros2 topic hz /m20/rear/points || true

echo "===== front metadata ====="
timeout 5 ros2 topic echo /m20/front/points --no-arr || true

echo "===== rear metadata ====="
timeout 5 ros2 topic echo /m20/rear/points --no-arr || true
