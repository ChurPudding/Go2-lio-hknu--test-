#!/usr/bin/env bash
# run_outdoor_loc.sh — 실외 위치 추정 스텁 실행
#
# [source 용 / ./ 용 겸용] source tools/run_outdoor_loc.sh 로도,
# ./tools/run_outdoor_loc.sh 로도 실행됩니다. 본문을 서브셸 ( ... ) 로
# 감싸 두어, source 로 실행해도 set -e 와 변수가 호출한 셸로 새지 않고
# 맨 끝의 exec 도 서브셸 프로세스만 localization_stub.py 로 바꿔치기할
# 뿐 source 한 터미널 자체를 대체하지 않습니다. (이 노드는 포그라운드에서
# 계속 떠 있는 것이 정상이므로, 끝내려면 Ctrl-C 하면 됩니다.)
#
# 실내는 run_indoor.sh (go2_nav_interface.py) 가 담당한다.
# 둘 다 map->odom 을 발행하므로 동시에 띄우지 말 것.
#
# TF 리매핑을 매번 손으로 치면 언젠가 빠뜨린다. 빠뜨리면 /hknu/tf 로 나가고
# RViz 와 nav2 가 TF 를 못 찾는데, 에러가 나지 않아 원인을 찾기 어렵다.
# 그래서 스크립트로 고정한다.
#
# 사용
#     ./run_outdoor_loc.sh                     # k = go2_calib.K_OUTDOOR
#     ./run_outdoor_loc.sh 1.2007              # k 지정 (검증 완료로 표시)
#     PUBLISH_ODOM_BASE=true ./run_outdoor_loc.sh
#         -> Go2 가 odom -> base_link 를 방송하지 않을 때만

(
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

K="${1:-}"
NS="${NS:-/hknu}"
PUBLISH_ODOM_BASE="${PUBLISH_ODOM_BASE:-false}"

ARGS=(--ros-args
      -r "__ns:=${NS}"
      -r "__node:=localization_outdoor"
      -r /tf:=/tf
      -r /tf_static:=/tf_static
      -p "publish_odom_base_tf:=${PUBLISH_ODOM_BASE}")

if [ -n "$K" ]; then
    ARGS+=(-p "k:=${K}" -p k_verified:=true)
    echo "k = ${K} (검증됨)"
else
    echo "k = go2_calib.K_OUTDOOR (5m 직진 미검증). odom_scale_check.py 로 재고 인자로 넘겨 주세요."
fi

echo "네임스페이스 ${NS},  TF 는 전역 유지"
echo

exec python3 "${HERE}/localization_stub.py" "${ARGS[@]}"
)
