#!/usr/bin/env bash
# =============================================================================
# zupt_ab_summarize.sh  —  12개 출력 bag을 yaw_compare 로 집계
# -----------------------------------------------------------------------------
#   각 런에서 추출 : 전체 최대 |PL-다리| 편차, 루프A/B PL 고유편차  → CSV
#   마지막에 (bag,cond) 조합별 평균±표준편차 표 출력.
#   ※ 캡처와 분리 → 측정 다시 안 하고 언제든 재실행 가능.
#   yaw_compare.py 위치가 다르면 : YAWCMP=/경로/yaw_compare.py ./zupt_ab_summarize.sh
#   대상 폴더는 zupt_ab_env.sh 의 AB_KEY 로 정한다 (run_zupt_ab_batch.sh 와 같은 값으로).
# =============================================================================
set -u
source "$(dirname "$(readlink -f "$0")")/zupt_ab_env.sh"   # AB_KEY · OUTDIR · LOGDIR
YAWCMP="${YAWCMP:-$HOME/fastlio_ws/analysis/yaw_compare.py}"   # 없으면 아래에서 안내 후 중단
CSV="$LOGDIR/summary.csv"
mkdir -p "$LOGDIR"

[ -f "$YAWCMP" ] || { echo "!! yaw_compare.py 없음: $YAWCMP"; \
  echo "   YAWCMP=/실제/경로/yaw_compare.py 로 지정해 다시 실행하세요."; exit 1; }

set +u   # ROS setup.bash가 미설정 변수를 참조 → nounset 잠시 해제
source /opt/ros/humble/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
set -u
unset CYCLONEDDS_URI; export ROS_DOMAIN_ID=99

# 한 줄에서 마지막 각도값(고유편차/최대편차)만 추출. 유니코드 −도 -로 정규화.
last_deg() { grep -oE '[-+−0-9.]+°' | tail -1 | tr -d '°' | tr '−' '-'; }

echo "tag,max_div_deg,loopA_dev_deg,loopB_dev_deg" > "$CSV"
for out in "$OUTDIR"/plout_run{1,2}_{off,on}_r{1,2,3}; do
  [ -d "$out" ] || { echo "  (없음) $(basename "$out")"; continue; }
  tag=$(basename "$out" | sed 's/^plout_//')
  rep=$(python3 "$YAWCMP" "$out" --out "$LOGDIR/yaw_${tag}.png" 2>/dev/null)
  maxd=$(echo "$rep" | grep "전체 최대" | last_deg)   # 그 줄엔 °값 1개
  la=$(  echo "$rep" | grep "루프A"     | last_deg)   # 줄 끝 필드 = 고유편차
  lb=$(  echo "$rep" | grep "루프B"     | last_deg)
  printf '%s,%s,%s,%s\n' "$tag" "${maxd:-NA}" "${la:-NA}" "${lb:-NA}" | tee -a "$CSV"
done

echo "--------------------------------------------------"
echo "조합별 평균±표준편차:"
python3 - "$CSV" << 'PY'
import sys, csv, math, collections
rows=list(csv.DictReader(open(sys.argv[1])))
def stat(v):
    v=[float(x) for x in v if x not in ('NA','')]
    if not v: return "NA"
    m=sum(v)/len(v)
    s=math.sqrt(sum((x-m)**2 for x in v)/len(v)) if len(v)>1 else 0.0
    return f"{m:+.1f}\u00b1{s:.1f}"
g=collections.defaultdict(lambda: collections.defaultdict(list))
for r in rows:
    key="_".join(r["tag"].split("_")[:2])   # run1_off 등
    for k in ("max_div_deg","loopA_dev_deg","loopB_dev_deg"):
        g[key][k].append(r[k])
print(f'{"조합":10} {"최대편차":>14} {"루프A":>12} {"루프B":>12}')
for key in sorted(g):
    d=g[key]
    print(f'{key:10} {stat(d["max_div_deg"]):>14} {stat(d["loopA_dev_deg"]):>12} {stat(d["loopB_dev_deg"]):>12}')
PY
echo "CSV: $CSV"
