#!/usr/bin/env bash
# run_slam.sh -- Point-LIO 파이프라인 위에 slam_toolbox 를 얹는다
#
#   ./tools/run_slam.sh
#
# 전제: 다른 터미널에서 ./tools/run_indoor.sh 가 이미 돌고 있어야 한다.
#       (/indoor/base_pose 와 TF 가 나와야 하기 때문)
#
# ─────────────────────────────────────────────────────────────
#  왜 필요한가
#
#  Point-LIO 는 오도메트리다. 지나간 길을 다시 지나도 "여기 와봤다"를 모르므로
#  벽이 두 겹으로 찍힌다. slam_toolbox 가 루프 클로저를 해서 이를 합친다.
#
#  /utlidar/cloud ─[pointcloud_to_laserscan]─> /scan ─┐
#                                                      ├─> slam_toolbox ─> /map
#  lio_tf 의 TF (odom -> base_link) ───────────────────┘
#
#  주의: run_indoor.sh 는 TF 를 indoor_map -> base_link 로 낸다.
#        slam_toolbox 는 odom -> base_link 를 기대하므로 이름을 이어준다.
#        (indoor_map 을 odom 으로 쓰는 셈. LIO 출력이 곧 오도메트리다)
# ─────────────────────────────────────────────────────────────

set -u
WS=~/fastlio_ws
PIDS=(); NAMES=()

cleanup() {
  echo; echo "── 종료 ─────────────────────────────────────────────────"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -INT "${PIDS[$i]}" 2>/dev/null && echo "   ${NAMES[$i]}"
  done
  for i in {1..20}; do
    alive=0
    for p in "${PIDS[@]:-}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && break
    sleep 0.5
  done
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
  echo "─────────────────────────────────────────────────────────"
}
trap 'cleanup; exit 130' INT TERM

start() {
  local name=$1 log=$2; shift 2
  printf '  %-22s ' "$name"
  "$@" >"$log" 2>&1 &
  local pid=$!
  PIDS+=($pid); NAMES+=("$name")
  sleep 0.5
  if kill -0 $pid 2>/dev/null; then echo "OK"
  else echo "실패"; tail -8 "$log"; cleanup; exit 1; fi
}

set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
source ~/setup_go2.sh
set -u

mkdir -p /tmp/go2_slam

echo "── 전제 확인 ────────────────────────────────────────────"
printf '  %-22s ' "/indoor/base_pose"
timeout 5 ros2 topic hz /indoor/base_pose 2>/dev/null | grep -m1 'average rate' \
  || { echo "미수신 ✗"; echo; echo "  먼저 ./tools/run_indoor.sh 를 실행하십시오."; exit 1; }
printf '  %-22s ' "/utlidar/cloud"
timeout 5 ros2 topic hz /utlidar/cloud 2>/dev/null | grep -m1 'average rate' \
  || { echo "미수신 ✗"; exit 1; }

echo
echo "── 노드 기동 ────────────────────────────────────────────"

# indoor_map -> odom 이름만 이어준다 (같은 원점)
start "odom 프레임" /tmp/go2_slam/odomframe.log \
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom indoor_map

# base_link -> utlidar_lidar
#   /utlidar/cloud 의 frame_id 는 'utlidar_lidar' 인데 이 프레임이 TF 트리에
#   없으면 pointcloud_to_laserscan 이 변환을 못 해 전부 버린다
#   ("Message Filter dropping message ... queue is full").
#   go2_calib 의 R_LB, LEVER 로 계산한 값이다. 라이다는 164.9° 기울어 있다.
start "라이다 프레임" /tmp/go2_slam/lidarframe.log \
  ros2 run tf2_ros static_transform_publisher \
    0.322 0.005 0.050  0.870692 -0.473557 0.119288 -0.058395 \
    base_link utlidar_lidar

# 3D 점군을 2D 스캔으로. 로봇 몸통 높이대만 잘라 쓴다.
start "점군→스캔" /tmp/go2_slam/p2l.log \
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:=/utlidar/cloud -r scan:=/scan \
    -p target_frame:=base_link \
    -p transform_tolerance:=0.5 \
    -p min_height:=-0.15 -p max_height:=0.60 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0087 \
    -p scan_time:=0.0667 \
    -p range_min:=0.35 -p range_max:=12.0 \
    -p use_inf:=true

start "slam_toolbox" /tmp/go2_slam/slam.log \
  ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$WS/config/slam_toolbox_go2.yaml"

sleep 3
cat <<'EOF'

── 확인 ─────────────────────────────────────────────────
  ros2 topic hz /scan          15 Hz 근처여야 함
  ros2 topic hz /map           1 Hz 근처
  ros2 run tf2_ros tf2_echo map base_link

── 지도 저장 (주행 후) ──────────────────────────────────
  ros2 run nav2_map_server map_saver_cli -f ~/fastlio_ws/results/slam_map

── 실행 중.  Ctrl+C 로 종료 ─────────────────────────────
  ※ 리모컨을 손에 들고 계십시오.
     같은 길을 두 번 지나가야 루프 클로저가 동작합니다.

EOF

while true; do sleep 10; done
