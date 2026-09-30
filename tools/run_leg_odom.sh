#!/usr/bin/env bash
# run_leg_odom.sh — leg_odom_refine.py 실행 (실외/실내 k, 실기/bag 겸용)
#
# [source 용 / ./ 용 겸용] source tools/run_leg_odom.sh 로도,
# ./tools/run_leg_odom.sh 로도 실행됩니다. 본문을 서브셸 ( ... ) 로 감싸
# 두어, source 로 실행해도 set -e 와 변수가 호출한 셸로 새지 않습니다.
# (run_outdoor_loc.sh 와 같은 이유·같은 방식.)
#
# leg_odom_refine.py 는 TF 를 전혀 발행하지 않으므로(소유권은
# localization_stub.py) 이 스크립트도 TF 리매핑을 하지 않는다.
#
# 사용
#     ./run_leg_odom.sh                    # outdoor, 실기 모드 (setup_go2.sh)
#     ./run_leg_odom.sh indoor             # indoor, 실기 모드
#     ./run_leg_odom.sh outdoor --bag      # outdoor, bag 재생 모드
#     ./run_leg_odom.sh --bag indoor       # 순서 무관
#
# indoor/outdoor 는 kx_a/ky_a 에 넣을 k 값만 고른다(go2_calib.K_INDOOR /
# go2_calib.K_OUTDOOR). 값은 여기 박아두지 않고 매번 python3 -c 로
# go2_calib.py 에서 직접 읽는다 — 두 곳에서 값이 어긋나는 것을 막기 위함.

(
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

NS="${NS:-/hknu}"
MODE="outdoor"
BAG=false

for arg in "$@"; do
    case "$arg" in
        indoor|outdoor) MODE="$arg" ;;
        --bag) BAG=true ;;
        *)
            echo "알 수 없는 인자: $arg" >&2
            echo "사용: $0 [indoor|outdoor] [--bag]" >&2
            exit 1
            ;;
    esac
done

# --- k 값: go2_calib.py 에서 직접 읽는다 (스크립트에 박지 않는다) ---------
K="$(MODE="$MODE" python3 -c "
import os, sys
sys.path.insert(0, '${HERE}')
import go2_calib
mode = os.environ['MODE']
print(go2_calib.K_OUTDOOR if mode == 'outdoor' else go2_calib.K_INDOOR)
")"

# --- 실행 환경: bag 재생이냐 실기냐 ---------------------------------------
if $BAG; then
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID=99
    ENV_DESC="bag 재생 (CYCLONEDDS_URI 해제, ROS_DOMAIN_ID=99)"
else
    source ~/unitree_ros2/setup_go2.sh
    ENV_DESC="실기 (setup_go2.sh)"
fi

# --- 이중 실행 방지: /hknu/leg_odom 을 이미 누가 발행 중인지 2초간 확인 ----
TOPIC="${NS}/leg_odom"
echo "이중 실행 확인 중: ${TOPIC} (2초)..."
COUNT=0
for i in $(seq 1 10); do
    COUNT="$(ros2 topic info "$TOPIC" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')"
    if [ -n "$COUNT" ] && [ "$COUNT" -gt 0 ]; then
        break
    fi
    COUNT=0
    sleep 0.2
done

if [ "$COUNT" -gt 0 ]; then
    echo "** ${TOPIC} 를 이미 다른 노드가 발행 중입니다 (publisher ${COUNT}개). **" >&2
    echo "   이중 실행을 막기 위해 여기서 종료합니다. 기존 프로세스를 먼저 내려 주세요." >&2
    exit 1
fi
echo "  발행 중인 노드 없음 — 계속 진행"

# --- 시작 정보 -------------------------------------------------------------
echo
echo "모드: ${MODE}   k(kx_a=ky_a) = ${K}   ${ENV_DESC}"
echo "네임스페이스 ${NS},  TF 는 건드리지 않음(leg_odom_refine.py 는 TF 미발행)"
echo

ARGS=(--ros-args
      -r "__ns:=${NS}"
      -r "__node:=leg_odom_${MODE}"
      -p "kx_a:=${K}"
      -p "ky_a:=${K}")

exec python3 "${HERE}/leg_odom_refine.py" "${ARGS[@]}"
)
