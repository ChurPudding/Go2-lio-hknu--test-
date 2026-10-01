#!/usr/bin/env bash
# =============================================================================
# run_zupt_ab_batch.sh  —  in-filter A/B 캡처 (12런 무인 실행)
# -----------------------------------------------------------------------------
#   대상 : lio_test_bag_loop_run1 / run2  ×  {off, on}  ×  3회  = 12 런
#   방식 : zvd 는 두 조건 모두 ON, AB_KEY(zupt_ab_env.sh, 기본 leg_en) 하나만 토글 (단일변수 A/B).
#          값은 launch 인자(<키>:=true|false)로 넘기고 go2_fix.yaml 은 고치지 않는다.
#          time_sync = false (정식 조건). 재생 offset 25 · 0.5배속 (run1/run2 표준).
#   출력 : ~/data/bags/ab_<키>/plout_run{1,2}_{off,on}_r{1..3}
#   로그 : ~/data/zupt_ab_logs/<키>/*.log
#   집계 : 끝난 뒤 ./zupt_ab_summarize.sh 로 지표 평균±표준편차 산출 (같은 AB_KEY 로).
#
#   먼저 검증 : ./run_zupt_ab_batch.sh --test   (run1 off·on 각 1회, ~30분)
#              수동 결과와 PL 샘플 수·정지창 거동이 맞는지 확인 후 전체 실행.
#   재실행 안전 : 정상 종료된 bag(metadata.yaml 존재)은 건너뛰고, 중단으로
#                미완인 bag만 지우고 다시 찍는다.  전체 ≈ 12 × ~16분 ≈ 3.3시간.
# =============================================================================
set -u

# ---- 경로 (로봇 PC에 맞는지 확인) -------------------------------------------
CATKIN=~/catkin_point_lio_unilidar/src/point_lio_ros2
ZVD="$CATKIN/scripts/zvd_node.py"
IMUFIX=~/fastlio_ws/tools/l1_imu_fix.py
source "$(dirname "$(readlink -f "$0")")/zupt_ab_env.sh"   # AB_KEY · BAGDIR · OUTDIR · LOGDIR
mkdir -p "$OUTDIR" "$LOGDIR"

# ---- 실험 매트릭스 ----------------------------------------------------------
BAGS=("lio_test_bag_loop_run1" "lio_test_bag_loop_run2")
CONDS=("off" "on")
REPS="${REPS:-3}"
REQUIRED_TOPICS=(/utlidar/cloud /utlidar/imu /utlidar/robot_odom)
PLAY_OPTS="--start-offset 25 -r 0.5 --clock"   # offset 25 필수

# ---- 타이밍(초) -------------------------------------------------------------
INIT_WAIT=8       # 노드 구독 준비 (재생 전)
FLUSH_WAIT=5      # 재생 후 마지막 스캔 처리·record flush
TEARDOWN_WAIT=6   # DDS 정리 (런 사이)

# ---- --test : run1 off·on 각 1회 -------------------------------------------
if [ "${1:-}" = "--test" ]; then
  BAGS=("lio_test_bag_loop_run1"); REPS=1
  echo "### --test : run1 off·on 각 1회만 ###"
fi

# ---- 소싱 (비대화형 셸이라 .bashrc 안 읽힘 → 명시 source) --------------------
set +u   # ROS setup.bash가 미설정 변수(AMENT_TRACE_SETUP_FILES 등)를 참조 → nounset 잠시 해제
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
set -u
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=99

# ---- 종료 처리 : setsid 그룹킬(정밀) + pkill 안전망(잔여) --------------------
PIDS=()          # 노드 PID(=PGID)
REC_PID=""       # record PID(=PGID)
teardown() {
  [ -n "$REC_PID" ] && kill -INT "-$REC_PID" 2>/dev/null   # record 먼저(db3 안전)
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -INT "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]:-}"; do kill -9 "-$p" 2>/dev/null; done
  [ -n "$REC_PID" ] && kill -9 "-$REC_PID" 2>/dev/null
  # 잔여 프로세스 안전망 (정밀 패턴만)
  for pat in zvd_node.py l1_imu_fix.py mapping_go2_fix pointlio_mapping \
             "ros2 bag record" component_container; do
    pkill -f "$pat" 2>/dev/null
  done
  sleep "$TEARDOWN_WAIT"
  PIDS=(); REC_PID=""
}
trap 'echo; echo "[중단] 정리 중..."; teardown; exit 130' INT TERM

# ---- AB_KEY 값 : 조건 → launch 인자, 노드에 들어갔는지 확인 (실패 시 중단) --
cond_val() { [ "$1" = "on" ] && echo true || echo false; }   # $1 = on|off
check_param() {   # $1 = true|false
  local got; got=$(ros2 param get /laserMapping "$AB_KEY" 2>/dev/null)   # "Boolean value is: True"
  echo "  param: $AB_KEY = ${got:-없음}"
  case "${got,,}" in
    *"is: $1") : ;;
    *) echo "  X $AB_KEY 설정 실패 ($1 원했으나 '${got:-없음}') - 중단."; teardown; exit 1 ;;
  esac
}

# ---- 한 런 -----------------------------------------------------------------
run_one() {   # $1=short(run1/run2) $2=bag $3=cond $4=rep
  local short="$1" bag="$2" cond="$3" r="$4"
  local tag="${short}_${cond}_r${r}" val; val=$(cond_val "$cond")
  local out="$OUTDIR/plout_${tag}" log="$LOGDIR/pl_${tag}.log"

  if [ -f "$out/metadata.yaml" ]; then echo "  [skip] 완료본 존재: $out"; return 0; fi
  rm -rf "$out"                                # 미완/잔여 제거 후 재캡처

  PIDS=(); REC_PID=""
  setsid python3 "$ZVD"                                   >"$LOGDIR/zvd_${tag}.log" 2>&1 & PIDS+=($!)
  setsid python3 "$IMUFIX" --ros-args -p time_sync:=false >"$LOGDIR/imu_${tag}.log" 2>&1 & PIDS+=($!)
  setsid ros2 launch point_lio mapping_go2_fix.launch.py rviz:=false "${AB_KEY}:=${val}" >"$log" 2>&1 & PIDS+=($!)
  sleep "$INIT_WAIT"
  check_param "$val"

  setsid ros2 bag record -o "$out" \
      /aft_mapped_to_init /utlidar/robot_odom /zupt_active >"$LOGDIR/rec_${tag}.log" 2>&1 & REC_PID=$!
  sleep 1

  echo "  [재생] $bag  ($PLAY_OPTS)"
  ros2 bag play "$BAGDIR/$bag" $PLAY_OPTS >"$LOGDIR/play_${tag}.log" 2>&1
  sleep "$FLUSH_WAIT"
  teardown

  if grep -iE "nan|error|-inf|inf[^o]" "$log" | grep -qv "INFO"; then
    echo "  ! LIO 로그 이상 패턴 - 확인: $log"
  else
    echo "  OK 로그 정상"
  fi
}

# ---- 사전 점검 : 토글 키·bag·필수 토픽 --------------------------------------
echo "### 사전 점검 ###"
ros2 launch point_lio mapping_go2_fix.launch.py --show-args 2>/dev/null | grep -q "'$AB_KEY':" \
  || { echo "!! mapping_go2_fix.launch.py 에 $AB_KEY 인자 없음 (AB_KEY 확인)"; exit 1; }
echo "  OK: 토글 키 $AB_KEY"
for bag in "${BAGS[@]}"; do
  [ -e "$BAGDIR/$bag" ] || { echo "!! bag 없음: $BAGDIR/$bag"; exit 1; }
  info="$(ros2 bag info "$BAGDIR/$bag" 2>/dev/null)"
  for t in "${REQUIRED_TOPICS[@]}"; do
    echo "$info" | grep -q "$t" || { echo "!! $bag 에 $t 없음"; exit 1; }
  done
  echo "  OK: $bag"
done

# ---- 본 실행 ---------------------------------------------------------------
for pat in zvd_node.py l1_imu_fix.py pointlio_mapping; do pkill -f "$pat" 2>/dev/null; done
sleep 2
START=$(date +%s)
TOTAL=$(( ${#BAGS[@]} * ${#CONDS[@]} * REPS )); N=0
echo "총 $TOTAL 런 시작 ($(date '+%F %H:%M:%S'))"

for bag in "${BAGS[@]}"; do
  short=${bag#lio_test_bag_loop_}
  for cond in "${CONDS[@]}"; do
    for r in $(seq 1 "$REPS"); do
      N=$((N+1))
      echo "==================================================="
      echo " [$N/$TOTAL] ${short}_${cond}_r${r}   ($(date '+%H:%M:%S'))"
      echo "==================================================="
      run_one "$short" "$bag" "$cond" "$r"
    done
  done
done

MIN=$(( ($(date +%s)-START)/60 ))
echo "==================================================="
echo "완료: $TOTAL 런, ${MIN}분. 토글 키 $AB_KEY (go2_fix.yaml 은 그대로)."
echo "출력: $OUTDIR/plout_run{1,2}_{off,on}_r{1..3}"
echo "다음: AB_KEY=$AB_KEY ./zupt_ab_summarize.sh 로 지표 집계"
