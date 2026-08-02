#!/usr/bin/env bash
# run_lio.sh -- LIO 재생 실험 1회를 자동으로 수행한다.
#
#   ./run_lio.sh <알고리즘> <bag> <출력이름> [재생속도]
#
#     알고리즘 : pl        Point-LIO
#                fl        FAST-LIO  (go2_l1.yaml)
#                flnf      FAST-LIO  (go2_l1_nofeat.yaml)
#                flbefore  FAST-LIO  (go2_l1_before.yaml, /utlidar/imu 대조군)
#     bag      : ~/fastlio_ws 기준 상대경로 또는 절대경로
#     출력이름 : exp/<이름> 으로 저장. 이미 있으면 중단
#     재생속도 : 기본 0.5
#
# 예)
#   ./run_lio.sh flnf go2_corridor_0731_1930 cor_flnf_run1
#   for i in 1 2 3; do ./run_lio.sh pl go2_corridor_all_0731_1931 corall_pl_run$i; done
#
# 하는 일
#   1. 환경을 알고리즘에 맞게 source (srcoff/srcfl 대신 직접)
#   2. 브리지 -> LIO -> 녹화 순으로 띄우고 각각 준비 확인
#   3. 재생이 끝나면 역순으로 종료
#   4. Count 를 검증하고 결과를 한 줄로 출력
#
# 규약(00_실험순서 문서)
#   - 재생마다 LIO 노드를 새로 띄운다 (중력 초기화가 한 번뿐)
#   - record 를 play 보다 먼저 시작한다
#   - CYCLONEDDS_URI 를 해제한다 (오프라인 재생)

set -u
WS=~/fastlio_ws
ALG=${1:?알고리즘: pl | fl | flnf | flbefore}
BAG=${2:?bag 경로}
NAME=${3:?출력이름}
RATE=${4:-0.5}

OUT=$WS/exp/$NAME
LOG=$WS/exp/$NAME.log
[[ $BAG != /* ]] && BAG=$WS/$BAG

# ---------------------------------------------------------------- 사전 점검
[[ -d $BAG ]]   || { echo "✗ bag 없음: $BAG"; exit 1; }
[[ -e $OUT ]]   && { echo "✗ 이미 있음: $OUT  (지우거나 다른 이름을 쓸 것)"; exit 1; }
mkdir -p "$WS/exp"

case $ALG in
  pl)       WSENV=~/catkin_point_lio_unilidar/install/setup.bash
            LAUNCH=(point_lio mapping_unilidar_l1.launch.py)
            TOPIC=/aft_mapped_to_init ;;
  fl)       WSENV=$WS/install/setup.bash
            LAUNCH=(fast_lio mapping.launch.py config_file:=go2_l1.yaml rviz:=false)
            TOPIC=/Odometry ;;
  flnf)     WSENV=$WS/install/setup.bash
            LAUNCH=(fast_lio mapping.launch.py config_file:=go2_l1_nofeat.yaml rviz:=false)
            TOPIC=/Odometry ;;
  flbefore) WSENV=$WS/install/setup.bash
            LAUNCH=(fast_lio mapping.launch.py config_file:=go2_l1_before.yaml rviz:=false)
            TOPIC=/Odometry ;;
  *) echo "✗ 알 수 없는 알고리즘: $ALG"; exit 1 ;;
esac

# ---------------------------------------------------------------- 환경
set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$WSENV"
set -u
unset CYCLONEDDS_URI                    # 오프라인 재생 필수

echo "──────────────────────────────────────────────────────────"
echo " $ALG  |  $(basename "$BAG")  ->  exp/$NAME   (-r $RATE)"
echo "──────────────────────────────────────────────────────────"

PIDS=()
cleanup() {
  # SIGINT 로 정상 종료를 유도한다 (Point-LIO 는 이때 PCD 를 저장한다)
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null; done
  for i in {1..20}; do
    alive=0
    for p in "${PIDS[@]:-}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && break
    sleep 0.5
  done
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
}
trap 'echo; echo "중단됨"; cleanup; exit 130' INT TERM

# ---------------------------------------------------------------- 1) 브리지
echo -n "1) 브리지 ... "
python3 "$WS/tools/l1_imu_fix.py" >/tmp/run_lio_bridge.log 2>&1 &
BRIDGE=$!; PIDS+=($BRIDGE)
for i in {1..20}; do
  grep -q "l1_imu_fix started" /tmp/run_lio_bridge.log && break
  sleep 0.5
done
grep -q "l1_imu_fix started" /tmp/run_lio_bridge.log \
  && echo "OK" || { echo "실패"; cat /tmp/run_lio_bridge.log; cleanup; exit 1; }

# ---------------------------------------------------------------- 2) LIO
echo -n "2) LIO 노드 ... "
ros2 launch "${LAUNCH[@]}" >"$LOG" 2>&1 &
LIO=$!; PIDS+=($LIO)
for i in {1..40}; do
  ros2 node list 2>/dev/null | grep -qE "laser_mapping|laserMapping" && break
  sleep 0.5
done
if ros2 node list 2>/dev/null | grep -qE "laser_mapping|laserMapping"; then
  echo "OK"
else
  echo "실패 (노드가 뜨지 않음)"; tail -20 "$LOG"; cleanup; exit 1
fi

# ---------------------------------------------------------------- 3) 녹화
echo -n "3) 녹화 ($TOPIC) ... "
ros2 bag record -o "$OUT" "$TOPIC" >/tmp/run_lio_rec.log 2>&1 &
REC=$!; PIDS+=($REC)
for i in {1..30}; do
  grep -q "Subscribed to topic" /tmp/run_lio_rec.log && break
  sleep 0.5
done
grep -q "Subscribed to topic" /tmp/run_lio_rec.log \
  && echo "OK" || { echo "실패 (구독 안 됨)"; cat /tmp/run_lio_rec.log; cleanup; exit 1; }
sleep 1

# ---------------------------------------------------------------- 4) 재생
DUR=$(python3 - "$BAG" <<'PY' 2>/dev/null || echo "?"
import sys,sqlite3,glob,os
d=glob.glob(os.path.join(sys.argv[1],'*.db3'))[0]
c=sqlite3.connect(d)
a,b=c.execute("SELECT MIN(timestamp),MAX(timestamp) FROM messages").fetchone()
print('%.0f'%((b-a)*1e-9))
PY
)
EST=$(python3 -c "print('%.0f'%($DUR/$RATE))" 2>/dev/null || echo "?")
echo "4) 재생 ... bag ${DUR}s, 예상 ${EST}s"
ros2 bag play "$BAG" -r "$RATE" >/dev/null 2>&1
echo "   재생 완료"

# ---------------------------------------------------------------- 5) 종료
sleep 2
cleanup
sleep 1

# ---------------------------------------------------------------- 6) 검증
echo "──────────────────────────────────────────────────────────"
N=$(ros2 bag info "$OUT" 2>/dev/null | grep -oP 'Messages:\s+\K\d+')
D=$(ros2 bag info "$OUT" 2>/dev/null | grep -oP 'Duration:\s+\K[\d.]+')
ERR=$(grep -c "No Effective Points" "$LOG" 2>/dev/null || echo 0)
MAP=$(grep -c "mapping ]" "$LOG" 2>/dev/null || echo 0)
LINES=$(wc -l < "$LOG")

printf ' 결과 : Messages %s | Duration %ss | 로그 %s행\n' "${N:-0}" "${D:-0}" "$LINES"
[[ $ALG == fl* ]] && printf '        mapping 출력 %s | No Effective Points %s\n' "$MAP" "$ERR"

if [[ -z ${N:-} || $N -lt 100 ]]; then
  echo " ✗ 실패 — 메시지가 너무 적다. 이 회차는 버릴 것"
  exit 1
fi
echo " ✓ 정상"
echo "──────────────────────────────────────────────────────────"
