#!/usr/bin/env bash
# run_indoor.sh -- Go2 실내 자율주행 파이프라인 (LIO 기반 위치추정 + 지도)
#
#   ./run_indoor.sh [live | bag <경로> [속도]]
#
# ═══════════════════════════════════════════════════════════════════
#  왜 "실내" 전용인가
#
#  실외는 GPS 가 주 위치 소스이고, LIO 는 단기 자세·z 보조에 그친다
#  (2026-07-31 실측: 실외 306 m 에서 LIO 오차 110 m ~ 27 km).
#  실내는 GPS 가 없으므로 LIO 가 유일한 절대 위치원이고, 지도도 LIO 로 만든다.
#  두 파이프라인이 같은 로봇에서 번갈아 돌아가므로 토픽 이름을 분리한다.
#
#      실내 (이 스크립트)          실외 (GPS)
#      /indoor/base_pose           /gps/fix          gnss_bridge.py
#      /indoor/health              /gps/path         gnss_path.py
#      /indoor/health_info
#      TF indoor_map -> base_link
# ═══════════════════════════════════════════════════════════════════
#
#  로봇 기존 토픽                         이 파이프라인
#
#  /utlidar/cloud      15 Hz  ─┐
#  /utlidar/imu       250 Hz  ─┼─> l1_imu_fix ─> /l1_imu_fixed
#  /lowstate          500 Hz  ─┘                        │
#                                                       v
#                                                   Point-LIO
#                                                       │
#                                    /aft_mapped_to_init, /cloud_registered
#                                                       │
#                                          robot_pose (LEVER 보정)
#                                                       │
#                                            /indoor/base_pose  ──> 팀원A
#                                                       │
#  /utlidar/robot_odom 150 Hz ─> lio_health ─> /indoor/health  ──> 팀원A
#                                                       │
#                                                    lio_tf ──> /tf
#                                             indoor_map -> base_link
#
# ─── 팀원 인터페이스 ────────────────────────────────────────────────
#   /indoor/base_pose    nav_msgs/Odometry    몸통 위치·자세 (LEVER 보정됨)
#   /indoor/health       std_msgs/Bool        false = 위치 신뢰 불가, 즉시 정지
#   /indoor/health_info  std_msgs/String      사유 (JSON)
#   /tf                  indoor_map -> base_link  (health=false 면 발행 중단)
#   /cloud_registered    sensor_msgs/PointCloud2  실시간 정합 점군 (팀원B)
#
# ─── 쓰지 않는 것 ──────────────────────────────────────────────────
#   go2_odom_tf.py                  다리 오도메트리 TF. 표류(356 m 에 20 m)하고
#                                   LIO 와 프레임이 충돌한다. lio_tf.py 로 대체
#   go2_lowstate_to_jointstates.py  RViz 로봇 모델 시각화 전용. 충돌은 없으니
#                                   필요할 때만 따로 실행할 것
#
# ─── 지도 ──────────────────────────────────────────────────────────
#   Ctrl+C 로 정상 종료하면 PCD 가 저장된다(강제 종료하면 저장되지 않는다).
#     ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd
#   A* 용 2D 격자로 변환:
#     python3 tools/pcd_to_grid.py <scans.pcd> results/indoor_map 0.10
#   해상도 10 cm 가 이 데이터에 맞다(5 cm 는 벽이 끊겨 보인다).

set -u
WS=~/fastlio_ws
MODE=${1:-live}
NS=/indoor
MAP_FRAME=indoor_map
BASE_FRAME=base_link

PIDS=(); NAMES=()
cleanup() {
  echo; echo "── 종료 (역순) ──────────────────────────────────────────"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -INT "${PIDS[$i]}" 2>/dev/null && echo "   ${NAMES[$i]}"
  done
  for i in {1..24}; do
    alive=0
    for p in "${PIDS[@]:-}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && break
    sleep 0.5
  done
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
  local pcd=~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd
  [[ -f $pcd ]] && echo "   지도 저장됨: $pcd ($(du -h "$pcd" | cut -f1))"
  echo "─────────────────────────────────────────────────────────"
}
trap 'cleanup; exit 130' INT TERM

start() {   # start <이름> <로그> <명령...>
  local name=$1 log=$2; shift 2
  printf '  %-14s ' "$name"
  "$@" >"$log" 2>&1 &
  local pid=$!
  PIDS+=($pid); NAMES+=("$name")
  sleep 0.4
  if kill -0 $pid 2>/dev/null; then echo "OK"
  else echo "실패"; tail -8 "$log"; cleanup; exit 1; fi
}

# ─── 환경 ──────────────────────────────────────────────────────────
set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
set -u

if [[ $MODE == bag ]]; then
  BAG=${2:?bag 경로}; RATE=${3:-0.5}
  [[ $BAG != /* ]] && BAG=$WS/$BAG
  [[ -d $BAG ]] || { echo "✗ bag 없음: $BAG"; exit 1; }
  unset CYCLONEDDS_URI          # 오프라인 재생: 로봇과 분리 (실기 오작동 방지)
  echo "모드: bag 재생  $(basename "$BAG")  -r $RATE"
else
  source ~/setup_go2.sh || { echo "✗ setup_go2.sh 실패 — 로봇 연결 확인"; exit 1; }
  echo "모드: 실시간"
  echo
  echo "── 입력 토픽 확인 ───────────────────────────────────────"
  for t in /utlidar/cloud /lowstate /utlidar/imu /utlidar/robot_odom; do
    printf '  %-22s ' "$t"
    timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' \
      || { echo "미수신 ✗"; echo; echo "로봇 연결을 확인하고 다시 실행하십시오."; exit 1; }
  done
fi

# 이전 회차 PCD 를 지운다. Point-LIO 는 기존 파일에 덧붙이므로
# 지우지 않으면 실패한 회차의 점군이 섞여 지도가 층층이 어긋난다.
rm -f ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd

mkdir -p /tmp/go2_indoor
echo
echo "── 노드 기동 ────────────────────────────────────────────"
start "브리지"     /tmp/go2_indoor/bridge.log python3 "$WS/tools/l1_imu_fix.py"
sleep 2
start "Point-LIO"  /tmp/go2_indoor/lio.log \
      ros2 launch point_lio mapping_unilidar_l1.launch.py
sleep 4
# LIO 의 camera_init 과 indoor_map 을 같은 원점으로 묶는다.
# 이렇게 해야 /cloud_registered(camera_init) 와 TF(indoor_map) 가 함께 보인다.
start "map 프레임" /tmp/go2_indoor/staticframe.log \
      ros2 run tf2_ros static_transform_publisher \
      0 0 0 0 0 0 "$MAP_FRAME" camera_init
start "robot_pose" /tmp/go2_indoor/pose.log \
      python3 "$WS/tools/robot_pose.py" --ros-args -p out_topic:=$NS/base_pose
start "lio_health" /tmp/go2_indoor/health.log \
      python3 "$WS/tools/lio_health.py" --ros-args \
      -p lio_topic:=$NS/base_pose -p out_topic:=$NS/health \
      -p out_info_topic:=$NS/health_info
start "lio_tf"     /tmp/go2_indoor/tf.log \
      python3 "$WS/tools/lio_tf.py" --ros-args \
      -p in_topic:=$NS/base_pose -p health_topic:=$NS/health \
      -p parent_frame:=$MAP_FRAME -p child_frame:=$BASE_FRAME
sleep 2

cat <<EOF

── 인터페이스 ───────────────────────────────────────────
  $NS/base_pose        몸통 위치·자세          (팀원A)
  $NS/health           false = 즉시 정지       (팀원A)
  $NS/health_info      사유 JSON
  /tf                  $MAP_FRAME -> $BASE_FRAME
  /cloud_registered    실시간 정합 점군        (팀원B)

  상태 : tail -f /tmp/go2_indoor/health.log
  지도 : Ctrl+C 로 종료하면 PCD 저장 (강제 종료 금지)

EOF

if [[ $MODE == bag ]]; then
  echo "── 재생 ─────────────────────────────────────────────────"
  ros2 bag play "$BAG" -r "$RATE" 2>/dev/null
  echo "재생 완료. 5초 후 종료합니다."
  sleep 5
  cleanup
else
  echo "── 실행 중.  Ctrl+C 로 종료 ─────────────────────────────"
  echo
  echo "  ※ 주행 테스트 중에는 반드시 리모컨을 들고 계십시오."
  echo "     소프트웨어가 어디서 고장 나도 물리적으로 멈출 수 있는 건 그것뿐입니다."
  echo
  warned=0
  while true; do
    sleep 5
    if grep -q 'health=False' /tmp/go2_indoor/health.log 2>/dev/null; then
      [[ $warned == 0 ]] && {
        echo "  [경고] LIO 신뢰 불가 — $(grep -m1 '신뢰 불가' /tmp/go2_indoor/health.log)"
        echo "         TF 발행이 멈췄습니다. 로봇을 정지시키고 재시작하십시오."
        warned=1
      }
    fi
  done
fi
