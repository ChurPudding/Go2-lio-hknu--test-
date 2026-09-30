#!/usr/bin/env bash
# run_zupt_test.sh -- ZUPT(yaw hold) 실시간 검증 1회를 자동 수행한다.
#
#   ./run_zupt_test.sh <bag> <출력이름> <on|off> [재생속도]
#
#     bag      : 절대경로 또는 ~/data/bags 상대
#     출력이름 : exp/<이름> 으로 저장. 이미 있으면 중단
#     on|off   : yaw hold 켬/끔 (off = baseline, 위치만 hold)
#     재생속도 : 기본 0.5
#     오프셋   : bag 재생 시작 지점[s]. 기본 0. 스탠딩 스타트는 25 권장
#                (원인규명 때 66m 나온 조건 = --start-offset 25)
#     정지속도 : 정지판정 속도임계[m/s]. 비우면 0.05. 저속확장 실험은 0.12
#
# 예)
#   ./run_zupt_test.sh ~/data/bags/lio_test_bag_loop_run1 zupt_run1_ON  on
#   ./run_zupt_test.sh ~/data/bags/lio_test_bag_loop_run1 zupt_run1_OFF off
#   ./run_zupt_test.sh ~/data/bags/lio_test_bag_loop_run1 zupt_run1_ON on 0.5 25   # 스탠딩 스타트
#
# 파이프라인 (순서대로 준비 확인 후 다음 실행)
#   1) l1_imu_fix.py        -> /l1_imu_fixed         ("l1_imu_fix started")
#   2) Point-LIO            -> /aft_mapped_to_init   (노드 laserMapping)
#   3) robot_pose.py        -> /lio/base_pose        ("lio_base_pose ready")
#   4) zupt_filter_yaw.py   -> /lio/base_pose_zupt   (in:=/lio/base_pose)
#   5) 녹화 3토픽           /aft_mapped_to_init /lio/base_pose /lio/base_pose_zupt
#   6) bag 재생 -r RATE
#   7) 역순 종료
#
# 규약: record 를 play 보다 먼저. CYCLONEDDS_URI 해제. 재생마다 LIO 새로.

set -u
WS=~/fastlio_ws
BAG=${1:?bag 경로}
NAME=${2:?출력이름}
YAW=${3:?on 또는 off}
RATE=${4:-0.5}
OFFSET=${5:-0}          # bag 재생 시작 오프셋 [s]. 스탠딩 스타트용(예: 25)
STILLV=${6:-}           # 정지판정 속도임계 [m/s]. 비우면 노드 기본(0.05). 저속확장은 0.12

[[ $BAG != /* ]] && BAG=~/data/bags/$BAG
case $YAW in
  on)  HOLDYAW=true  ;;
  off) HOLDYAW=false ;;
  *)   echo "✗ 3번째 인자는 on 또는 off"; exit 1 ;;
esac

OUT=$WS/exp/$NAME
LIOLOG=$WS/exp/$NAME.liolog

[[ -d $BAG ]] || { echo "✗ bag 없음: $BAG"; exit 1; }
[[ -e $OUT ]] && { echo "✗ 이미 있음: $OUT"; exit 1; }
mkdir -p "$WS/exp"

# ---------------- 환경 ----------------
set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
set -u
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=99

echo "──────────────────────────────────────────────────────────"
echo " ZUPT 검증  |  $(basename "$BAG")  ->  exp/$NAME"
echo " yaw hold = $HOLDYAW   (-r $RATE, offset ${OFFSET}s, still_v ${STILLV:-0.05})"
echo "──────────────────────────────────────────────────────────"

PIDS=()
cleanup() {
  # 역순 종료. Point-LIO 는 SIGINT 로 정상 종료(PCD 저장).
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT "${PIDS[$i]}" 2>/dev/null; done
  for i in {1..20}; do
    alive=0
    for p in "${PIDS[@]:-}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && break
    sleep 0.5
  done
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
}
trap 'echo; echo "중단됨"; cleanup; exit 130' INT TERM

wait_log() {  # <파일> <문구> <횟수>
  for i in $(seq 1 "$3"); do grep -q "$2" "$1" 2>/dev/null && return 0; sleep 0.5; done
  return 1
}

# ---------------- 1) 브리지 ----------------
echo -n "1) 브리지 ... "
python3 "$WS/tools/l1_imu_fix.py" >/tmp/zt_bridge.log 2>&1 &
PIDS+=($!)
wait_log /tmp/zt_bridge.log "l1_imu_fix started" 20 \
  && echo "OK" || { echo "실패"; cat /tmp/zt_bridge.log; cleanup; exit 1; }

# ---------------- 2) Point-LIO ----------------
echo -n "2) Point-LIO ... "
ros2 launch point_lio mapping_unilidar_l1.launch.py >"$LIOLOG" 2>&1 &
PIDS+=($!)
for i in {1..40}; do
  ros2 node list 2>/dev/null | grep -qE "laser_mapping|laserMapping" && break; sleep 0.5
done
ros2 node list 2>/dev/null | grep -qE "laser_mapping|laserMapping" \
  && echo "OK" || { echo "실패"; tail -20 "$LIOLOG"; cleanup; exit 1; }

# ---------------- 3) robot_pose ----------------
echo -n "3) robot_pose ... "
python3 "$WS/tools/robot_pose.py" >/tmp/zt_pose.log 2>&1 &
PIDS+=($!)
wait_log /tmp/zt_pose.log "lio_base_pose ready" 20 \
  && echo "OK" || { echo "실패"; cat /tmp/zt_pose.log; cleanup; exit 1; }

# ---------------- 4) zupt ----------------
echo -n "4) zupt (yaw=$HOLDYAW, still_v=${STILLV:-기본0.05}) ... "
ZARGS="-p in_topic:=/lio/base_pose -p out_topic:=/lio/base_pose_zupt -p hold_yaw:=$HOLDYAW"
[[ -n "$STILLV" ]] && ZARGS="$ZARGS -p still_speed:=$STILLV"
python3 "$WS/tools/zupt_filter_yaw.py" --ros-args $ZARGS >/tmp/zt_zupt.log 2>&1 &
PIDS+=($!)
wait_log /tmp/zt_zupt.log "yaw hold" 20 \
  && echo "OK" || { echo "실패"; cat /tmp/zt_zupt.log; cleanup; exit 1; }

# ---------------- 5) 녹화 ----------------
echo -n "5) 녹화 (3토픽) ... "
ros2 bag record -o "$OUT" \
    /aft_mapped_to_init /lio/base_pose /lio/base_pose_zupt \
    >/tmp/zt_rec.log 2>&1 &
PIDS+=($!)
wait_log /tmp/zt_rec.log "All requested topics are subscribed" 30 \
  && echo "OK" || { echo "실패"; cat /tmp/zt_rec.log; cleanup; exit 1; }
sleep 1

# ---------------- 6) 재생 ----------------
DUR=$(python3 - "$BAG" <<'PY' 2>/dev/null || echo "?"
import sys,sqlite3,glob,os
d=glob.glob(os.path.join(sys.argv[1],'*.db3'))[0]
c=sqlite3.connect(d)
a,b=c.execute("SELECT MIN(timestamp),MAX(timestamp) FROM messages").fetchone()
print('%.0f'%((b-a)*1e-9))
PY
)
EST=$(python3 -c "print('%.0f'%($DUR/$RATE))" 2>/dev/null || echo "?")
PLAYOPT=""
[[ "$OFFSET" != "0" ]] && PLAYOPT="--start-offset $OFFSET"
echo "6) 재생 ... bag ${DUR}s, 예상 ${EST}s, 오프셋 ${OFFSET}s  (기다리는 중)"
ros2 bag play "$BAG" -r "$RATE" $PLAYOPT >/dev/null 2>&1
echo "   재생 완료"

# ---------------- 7) 종료 ----------------
sleep 2; cleanup; sleep 1

# ---------------- 8) 검증 ----------------
echo "──────────────────────────────────────────────────────────"
N=$(ros2 bag info "$OUT" 2>/dev/null | grep -oP 'Messages:\s+\K\d+')
HOLD=$(grep -oP '고정 \K\d+' /tmp/zt_zupt.log | tail -1)
printf ' 결과 : 녹화 Messages %s | zupt 마지막 고정프레임 %s\n' "${N:-0}" "${HOLD:-?}"
if [[ -z ${N:-} || $N -lt 100 ]]; then
  echo " ✗ 실패 — 메시지가 너무 적다. 이 회차는 버릴 것"; exit 1
fi
# PCD 저장본을 이름 붙여 백업 (덮어쓰기 방지)
PCD=~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd
if [[ -f $PCD ]]; then
  cp "$PCD" "$WS/exp/${NAME}.pcd" 2>/dev/null && echo " PCD 백업: exp/${NAME}.pcd"
fi
echo " ✓ 정상 — exp/$NAME"
echo "──────────────────────────────────────────────────────────"
echo " CSV 추출:"
echo "   python3 $WS/tools/traj_to_csv_v3.py $OUT ${NAME}_zupt.csv --topic /lio/base_pose_zupt --rate $RATE"
echo "   python3 $WS/tools/traj_to_csv_v3.py $OUT ${NAME}_base.csv --topic /lio/base_pose --rate $RATE"
