#!/usr/bin/env bash
# =============================================================================
# repro_run.sh — 재현성 실험 1회 자동 실행
#
#   사람 손 타이밍을 실험에서 제거하는 것이 목적이다.
#   T2(Point-LIO) 로그에 "Multi thread started" 가 뜬 뒤 고정 시간만 기다렸다가
#   T3(bag) 를 시작한다. 이 간격이 매 실행 동일해야 비교가 성립한다.
#
#   사용: ./repro_run.sh <실행이름> [T2→T3 간격초, 기본 3]
#   예:   ./repro_run.sh r1
# =============================================================================
set +u

RUN_NAME="${1:?사용법: ./repro_run.sh <실행이름> [간격초]}"
GAP="${2:-3}"

BAG="${3:-$HOME/data/bags/indoor/floor_0805_1720}"
# 3번째 인자로 bag 경로. /lowstate 가 없는 bag 은 ACC_TOPIC 을 지정한다.
ACC_TOPIC="${4:-/lowstate}"
# 필요한 토픽만 재생. 전체를 쏟으면 DDS 큐가 넘쳐 유실된다 (2026-08-10).
TOPICS="/utlidar/cloud /utlidar/imu $ACC_TOPIC"
PCD_SRC="$HOME/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd"
BRIDGE="$HOME/fastlio_ws/tools/l1_imu_fix.py"
MONITOR="$HOME/fastlio_ws/tools/repro_monitor.py"
OUT="$HOME/fastlio_ws/results/repro/$RUN_NAME"

DRAIN_S=20        # bag 종료 후 Point-LIO 가 밀린 큐를 비울 시간
SAVE_TIMEOUT=300  # PCD 저장 대기 상한

# ---------------------------------------------------------------- 0. 준비 ----
echo "=============================================="
echo " 실행: $RUN_NAME   (T2→T3 간격 ${GAP}s)"
echo " 시작: $(date '+%F %T')"
echo "=============================================="

mkdir -p "$OUT"

echo "[0] 이전 프로세스 정리"
pkill -f pointlio           2>/dev/null
pkill -f point_lio          2>/dev/null
pkill -f l1_imu_fix         2>/dev/null
pkill -f repro_monitor      2>/dev/null
pkill -f "ros2 bag"         2>/dev/null
pkill rviz2                 2>/dev/null
sleep 3
rm -f "$PCD_SRC"

# ------------------------------------------------------------- 1. 환경 ----
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
unset CYCLONEDDS_URI
source "$HOME/catkin_point_lio_unilidar/install/setup.bash"

echo "[1] 환경: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-기본}"
if [ "${ROS_DOMAIN_ID:-0}" = "0" ]; then
  echo "    ⚠ DOMAIN_ID=0 입니다. 로봇이 연결돼 있으면 즉시 중단하세요."
fi

# 실행 조건을 그대로 남긴다 (나중에 조건 차이를 의심할 때 필요)
{
  echo "run_name=$RUN_NAME"
  echo "gap_s=$GAP"
  echo "date=$(date -Is)"
  echo "bag=$BAG"
  echo "rate=0.5"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-0}"
  echo "uptime=$(uptime)"
  echo "--- config ---"
  cat "$HOME/catkin_point_lio_unilidar/install/point_lio/share/point_lio/config/"*.yaml 2>/dev/null \
    || echo "(config 경로 확인 필요)"
} > "$OUT/condition.txt" 2>&1

# ---------------------------------------------------- 2. 관측 노드 + 브리지 ----
echo "[2] 관측 노드 시작"
python3 "$MONITOR" "$OUT" > "$OUT/monitor.log" 2>&1 &
MON_PID=$!
sleep 2

echo "[3] T1 브리지(l1_imu_fix.py) 시작"
python3 "$BRIDGE" --ros-args -p acc_topic:="$ACC_TOPIC" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2

# ------------------------------------------------------------ 3. Point-LIO ----
echo "[4] T2 Point-LIO 시작"
ros2 launch point_lio mapping_unilidar_l1.launch.py rviz:=false > "$OUT/pointlio.log" 2>&1 &
LIO_PID=$!

echo "    'Multi thread started' 대기 중..."
READY=0
for i in $(seq 1 60); do
  if grep -aq "Multi thread started" "$OUT/pointlio.log" 2>/dev/null; then
    READY=1
    echo "    확인 (${i}s)"
    break
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "    ⚠ 60초 안에 안 떴습니다. 중단합니다."
  kill -INT "$LIO_PID" 2>/dev/null
  kill "$BRIDGE_PID" "$MON_PID" 2>/dev/null
  exit 1
fi

echo "    고정 대기 ${GAP}s"
sleep "$GAP"

# ------------------------------------------------------------- 4. bag 재생 ----
echo "[5] T3 bag 재생 (-r 0.5) — 약 15분"
T_START=$(date +%s)
ros2 bag play "$BAG" -r 0.5 --topics $TOPICS > "$OUT/bagplay.log" 2>&1
T_END=$(date +%s)
echo "    재생 완료: $((T_END - T_START))s"

echo "[6] 큐 배출 대기 ${DRAIN_S}s"
sleep "$DRAIN_S"

# ------------------------------------------------------- 5. 정상 종료 → PCD ----
# ★ SIGINT 여야 한다. SIGTERM(pkill 기본)이면 PCD 저장 코드가 안 돈다.
echo "[7] Point-LIO 에 SIGINT (PCD 저장)"
NODE_PID=$(pgrep -f pointlio_mapping | head -1)
echo "    노드 PID=${NODE_PID:-없음} 에 직접 SIGINT"
[ -n "${NODE_PID:-}" ] && kill -INT "$NODE_PID" 2>/dev/null
kill -INT "$LIO_PID" 2>/dev/null

for i in $(seq 1 "$SAVE_TIMEOUT"); do
  [ -f "$PCD_SRC" ] && printf "\r    저장 중 %s (%ds)" "$(du -h "$PCD_SRC" | cut -f1)" "$i"
  if ! kill -0 "$LIO_PID" 2>/dev/null; then
    echo
    echo "    종료 확인 (${i}s)"
    break
  fi
  sleep 1
done
kill -0 "$LIO_PID" 2>/dev/null && { echo "    ⚠ 종료 안 됨. 강제 종료"; kill -9 "$LIO_PID" 2>/dev/null; }

echo "[8] 관측/브리지 종료"
kill -INT "$MON_PID" 2>/dev/null
sleep 3
kill "$BRIDGE_PID" 2>/dev/null
sleep 1
pkill -f l1_imu_fix 2>/dev/null
pkill -f rviz2 2>/dev/null
sleep 1

# -------------------------------------------------------------- 6. 결과 수집 ----
if [ -f "$PCD_SRC" ]; then
  cp "$PCD_SRC" "$OUT/scans.pcd"
  echo "[9] PCD 저장됨: $(du -h "$OUT/scans.pcd" | cut -f1)"
else
  echo "[9] ⚠ PCD 없음. pointlio.log 확인 필요"
fi

# 로그에서 핵심 신호 추출 (널바이트 대비 grep -a)
{
  echo "### IMU 초기화"
  grep -a "IMU Initializing" "$OUT/pointlio.log" || echo "(없음)"
  echo
  echo "### Reset ImuProcess"
  grep -ac "Reset ImuProcess" "$OUT/pointlio.log"
  echo
  echo "### imu loop back 횟수"
  grep -ac "imu loop back" "$OUT/pointlio.log"
  echo
  echo "### No Effective Points 횟수"
  grep -ac "No Effective Points" "$OUT/pointlio.log"
} > "$OUT/signals.txt" 2>&1

echo "----------------------------------------------"
cat "$OUT/signals.txt"
echo "----------------------------------------------"
echo " 완료: $RUN_NAME → $OUT"
echo " 종료: $(date '+%F %T')"
echo "=============================================="
