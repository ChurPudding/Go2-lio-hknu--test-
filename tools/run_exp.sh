#!/bin/bash
# run_exp.sh -- Point-LIO 재생 실험 반복 자동화
#
# 사용법:
#   ./run_exp.sh <실험이름> <반복횟수> [재생속도]
# 예:
#   ./run_exp.sh r05 3 0.5      # -r 0.5 로 3회
#   ./run_exp.sh r10 3 1.0      # 실시간 3회
#
# 매 회차마다 브리지 노드와 LIO 노드를 새로 띄운다.
# (중력 초기화가 bag 초반 정지 구간에서 한 번만 이뤄지므로 필수)
#
# 결과: ~/fastlio_ws/exp/<이름>_runN/  bag
#       ~/fastlio_ws/exp/<이름>_runN.csv
#       ~/fastlio_ws/exp/<이름>_runN.log   브리지 노드 로그(정지검증 포함)

set -u

NAME=${1:?실험 이름을 지정하세요}
N=${2:-3}
RATE=${3:-1.0}

WS=~/fastlio_ws
BAG=$WS/go2_run_full
TOOLS=$WS/tools
OUT=$WS/exp
TOPIC=/aft_mapped_to_init

mkdir -p "$OUT"

if [ ! -d "$BAG" ]; then echo "bag 없음: $BAG"; exit 1; fi
if [ ! -f "$TOOLS/l1_imu_fix.py" ]; then echo "브리지 노드 없음"; exit 1; fi

# bag 길이 + 여유 (재생속도 반영)
DUR=$(python3 -c "print(int(112/$RATE)+15)")

cleanup() {
    echo "  정리 중..."
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    sleep 1
    for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
    pkill -f pointlio_mapping 2>/dev/null
    pkill -f l1_imu_fix.py 2>/dev/null
    pkill -f "bag record" 2>/dev/null
    sleep 2
}
trap 'cleanup; exit 130' INT TERM

echo "=========================================="
echo " 실험 $NAME  |  $N 회  |  재생속도 x$RATE"
echo " 예상 소요: 약 $(( DUR * N / 60 + 1 )) 분"
echo "=========================================="

for i in $(seq 1 "$N"); do
    TAG="${NAME}_run${i}"
    echo ""
    echo "--- [$i/$N] $TAG ---"
    rm -rf "$OUT/$TAG"
    PIDS=()

    # 1) 브리지 노드
    python3 "$TOOLS/l1_imu_fix.py" > "$OUT/$TAG.log" 2>&1 &
    PIDS+=($!)
    sleep 3

    # 2) LIO
    ros2 launch point_lio mapping_unilidar_l1.launch.py \
        > "$OUT/$TAG.lio.log" 2>&1 &
    PIDS+=($!)
    sleep 6

    # 3) 녹화
    ros2 bag record -o "$OUT/$TAG" "$TOPIC" > /dev/null 2>&1 &
    PIDS+=($!)
    sleep 3

    # 4) 재생 (동기)
    echo "  재생 중 (최대 ${DUR}s)..."
    timeout "$DUR" ros2 bag play "$BAG" -r "$RATE" > /dev/null 2>&1
    sleep 3

    cleanup
    unset PIDS

    # 5) CSV 추출
    DB=$(ls "$OUT/$TAG"/*.db3 2>/dev/null | head -1)
    if [ -z "$DB" ]; then
        echo "  !! bag 생성 실패"
        continue
    fi
    python3 "$WS/dump_odom.py" "$DB" "$OUT/$TAG.csv" || echo "  !! CSV 추출 실패"

    # 정지검증 결과 표시
    grep -h "정지검증" "$OUT/$TAG.log" | tail -4 | sed 's/^/  /'
done

echo ""
echo "=========================================="
echo " 평가"
echo "=========================================="
CSVS=$(ls "$OUT/${NAME}_run"*.csv 2>/dev/null)
if [ -z "$CSVS" ]; then echo "CSV 없음"; exit 1; fi
python3 "$TOOLS/eval_lio.py" "$BAG"/*.db3 $CSVS
