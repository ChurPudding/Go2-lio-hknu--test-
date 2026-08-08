#!/usr/bin/env bash
# =============================================================================
# repro_all.sh — 동일 조건 N회 반복
#
#   기본 3회. 1회당 약 16분 + 쿨다운 2분 → 총 약 55분.
#   중간에 다른 프로그램을 켜지 마세요. 그 자체가 변수가 됩니다.
#
#   사용: ./repro_all.sh [반복횟수] [접두어]
#   예:   ./repro_all.sh 3 base
# =============================================================================
set -u

N="${1:-3}"
PREFIX="${2:-base}"
COOLDOWN=120

DIR="$(cd "$(dirname "$0")" && pwd)"

echo "##############################################"
echo " 재현성 실험 ${N}회  (접두어: $PREFIX)"
echo " 예상 소요: 약 $(( N * 16 + (N - 1) * COOLDOWN / 60 ))분"
echo "##############################################"

for i in $(seq 1 "$N"); do
  bash "$DIR/repro_run.sh" "${PREFIX}_${i}" 3
  if [ "$i" -lt "$N" ]; then
    echo ">>> 쿨다운 ${COOLDOWN}s (열/캐시 상태 정렬)"
    sleep "$COOLDOWN"
  fi
done

echo
echo ">>> 전체 완료. 리포트 생성"
python3 "$DIR/repro_report.py" "$HOME/fastlio_ws/results/repro" "$PREFIX"
