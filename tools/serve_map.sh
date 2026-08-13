#!/bin/bash
# /map 토픽으로 지도 발행 (팀원 공유용, 도메인 0)
MAP=${1:-$HOME/fastlio_ws/results/loop_0810/grid005.yaml}

unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

echo "=== 지도: $MAP  /  DOMAIN=0 ==="
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" &
PID=$!
sleep 3
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
echo "=== /map 발행 중. Ctrl+C 로 종료 ==="
trap "kill $PID 2>/dev/null" EXIT
wait $PID
