#!/usr/bin/env bash
# doctor.sh -- 파이프라인을 쓸 수 있는 상태인지 한 번에 점검한다
#
#   ./tools/doctor.sh          전체 점검 (로봇 없이)
#   ./tools/doctor.sh robot    로봇 연결까지 포함
#
# 무엇이 빠졌는지, 어떻게 고치는지까지 알려 준다.
# 설치 직후·저장소를 새로 받은 직후에 한 번 돌려 보십시오.

set -u
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:-basic}
PASS=0; FAIL=0; WARN=0

g() { printf '  \033[32m✓\033[0m %s\n' "$1"; PASS=$((PASS+1)); }
r() { printf '  \033[31m✗\033[0m %s\n' "$1"; [[ $# -gt 1 ]] && printf '      → %s\n' "$2"; FAIL=$((FAIL+1)); }
y() { printf '  \033[33m!\033[0m %s\n' "$1"; [[ $# -gt 1 ]] && printf '      → %s\n' "$2"; WARN=$((WARN+1)); }
sec() { printf '\n\033[1m%s\033[0m\n' "── $1 ────────────────────────────────────────"; }

# ═══════════════════════════════════════════════ 1 파일
sec "1. 파일"

need_tools=(
  l1_imu_fix.py robot_pose.py lio_health.py lio_tf.py go2_calib.py
  pcd_to_grid.py run_indoor.sh install_go2_lio.sh
)
miss=()
for f in "${need_tools[@]}"; do [[ -f $WS/tools/$f ]] || miss+=("$f"); done
if [[ ${#miss[@]} -eq 0 ]]; then g "tools/ 필수 파일 ${#need_tools[@]} 개"
else r "tools/ 누락: ${miss[*]}" "저장소를 다시 받으십시오 (git pull)"; fi

[[ -x $WS/tools/run_indoor.sh ]] && g "run_indoor.sh 실행 권한" \
  || r "run_indoor.sh 실행 권한 없음" "chmod +x $WS/tools/*.sh"

[[ -f $WS/docs/INTERFACE_indoor.md ]] && g "docs/INTERFACE_indoor.md" \
  || y "인터페이스 문서 없음" "사용법을 볼 수 없습니다"

[[ -f $WS/README.md ]] && g "README.md" || y "README.md 없음"

# ═══════════════════════════════════════════════ 2 지도
sec "2. 지도 (A* 입력)"

MAP=$WS/results/indoor_map_inflated
if [[ -f $MAP.pgm && -f $MAP.yaml ]]; then
  g "indoor_map_inflated.pgm / .yaml"

  res=$(grep -oP 'resolution:\s*\K[\d.]+' "$MAP.yaml" 2>/dev/null)
  org=$(grep -oP 'origin:\s*\K.*' "$MAP.yaml" 2>/dev/null)
  if [[ -n ${res:-} ]]; then
    g "resolution $res m,  origin $org"
    awk -v r="$res" 'BEGIN{ if (r+0 < 0.05 || r+0 > 0.5) exit 1 }' \
      || y "해상도가 이례적입니다" "보통 0.10 을 씁니다"
  else
    r "yaml 에서 resolution 을 읽지 못함" "파일이 손상됐을 수 있습니다"
  fi

  # pgm 헤더 확인
  hdr=$(head -c 2 "$MAP.pgm")
  [[ $hdr == P5 ]] && g "pgm 형식 (P5, $(stat -c%s "$MAP.pgm") B)" \
    || r "pgm 헤더가 P5 가 아님" "지도를 다시 만드십시오"
else
  r "지도 파일 없음" "python3 tools/pcd_to_grid.py <scans.pcd> results/indoor_map 0.10"
fi

if [[ -f $MAP.npy ]]; then
  out=$(python3 - "$MAP.npy" <<'PY' 2>&1
import sys, numpy as np
a = np.load(sys.argv[1])
occ = int(a.sum()); tot = a.size
print("%d x %d, 장애물 %d 칸 (%.1f%%)" % (a.shape[1], a.shape[0], occ, 100*occ/tot))
if occ == 0:       sys.exit(2)
if occ/tot > 0.5:  sys.exit(3)
PY
)
  case $? in
    0) g "npy $out" ;;
    2) r "지도에 장애물이 하나도 없음" "높이 필터를 조정해 다시 만드십시오" ;;
    3) r "지도의 절반 이상이 장애물" "지면이 제거되지 않았습니다" ;;
    *) r "npy 를 읽지 못함: $out" ;;
  esac
fi

# ═══════════════════════════════════════════════ 3 파이썬
sec "3. 파이썬"

for m in numpy scipy matplotlib; do
  python3 -c "import $m" 2>/dev/null && g "$m" \
    || y "$m 없음" "pip3 install $m --break-system-packages"
done

bad=(); n=0
for f in "$WS"/tools/*.py; do
  [[ -f $f ]] || continue
  n=$((n+1))
  python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$f" 2>/dev/null \
    || bad+=("$(basename "$f")")
done
if   [[ $n -eq 0 ]];          then r "tools/*.py 가 없음" "저장소를 다시 받으십시오"
elif [[ ${#bad[@]} -eq 0 ]];  then g "tools/*.py 문법 ($n 개)"
else r "문법 오류: ${bad[*]}"; fi

badsh=(); n=0
for f in "$WS"/tools/*.sh; do
  [[ -f $f ]] || continue
  n=$((n+1))
  bash -n "$f" 2>/dev/null || badsh+=("$(basename "$f")")
done
if   [[ $n -eq 0 ]];         then r "tools/*.sh 가 없음"
elif [[ ${#badsh[@]} -eq 0 ]]; then g "tools/*.sh 문법 ($n 개)"
else r "문법 오류: ${badsh[*]}"; fi

# ═══════════════════════════════════════════════ 4 ROS2
sec "4. ROS2 · 패키지"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  g "ROS2 Humble"
  set +u; source /opt/ros/humble/setup.bash; set -u
else
  r "ROS2 Humble 없음" "먼저 ROS2 를 설치하십시오"
fi

for wsp in ~/unitree_ros2/cyclonedds_ws ~/ws_livox ~/catkin_point_lio_unilidar; do
  if [[ -f $wsp/install/setup.bash ]]; then
    g "$(basename "$wsp")"
    set +u; source "$wsp/install/setup.bash"; set -u
  else
    r "$(basename "$wsp") 빌드 안 됨" "$WS/tools/install_go2_lio.sh"
  fi
done

for p in unitree_go livox_ros_driver2 point_lio; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && g "패키지 $p" \
    || r "패키지 $p 없음" "$WS/tools/install_go2_lio.sh"
done

# ═══════════════════════════════════════════════ 5 설정
sec "5. Point-LIO 설정"

CFG=~/catkin_point_lio_unilidar/src/point_lio_ros2/config/unilidar_l1.yaml
if [[ -f $CFG ]]; then
  grep -q 'lid_topic:.*"/utlidar/cloud"'  "$CFG" && g 'lid_topic = /utlidar/cloud' \
    || r "lid_topic 이 /utlidar/cloud 가 아님" "$WS/tools/install_go2_lio.sh 6"
  grep -q 'imu_topic:.*"/l1_imu_fixed"'   "$CFG" && g 'imu_topic = /l1_imu_fixed' \
    || r "imu_topic 이 /l1_imu_fixed 가 아님" "브리지 출력을 써야 합니다"
  grep -q 'pcd_save_en: *true'            "$CFG" && g 'pcd_save_en = true' \
    || y "pcd_save_en 이 false" "지도를 만들 수 없습니다"
else
  r "unilidar_l1.yaml 없음" "Point-LIO 가 설치되지 않았습니다"
fi

[[ -f ~/setup_go2.sh ]] && g "setup_go2.sh" \
  || r "setup_go2.sh 없음" "효신님 PC 에서 복사해 오십시오"

# ═══════════════════════════════════════════════ 6 로봇
if [[ $MODE == robot ]]; then
  sec "6. 로봇 연결"
  RFAIL0=$FAIL
  if ip -br addr 2>/dev/null | grep -q 192.168.123; then
    IF=$(ip -br addr | grep 192.168.123 | awk '{print $1}')
    g "인터페이스 $IF"
    [[ $IF == enx* || $IF == enp0s20* ]] && g "USB-이더넷" \
      || y "내장 랜포트로 보입니다" "연결이 끊긴 사례가 있습니다. USB-이더넷 권장"
  else
    r "192.168.123.x 인터페이스 없음" "로봇 케이블을 확인하십시오"
  fi

  ping -c1 -W2 192.168.123.161 >/dev/null 2>&1 && g "로봇 응답 (192.168.123.161)" \
    || r "로봇 무응답" "전원과 케이블을 확인하십시오"

  set +u; source ~/setup_go2.sh >/dev/null 2>&1; set -u
  declare -A want=( [/utlidar/cloud]=15 [/lowstate]=500 [/utlidar/imu]=250 [/utlidar/robot_odom]=150 )
  for t in "${!want[@]}"; do
    hz=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 -oP 'average rate: \K[\d.]+')
    if [[ -z ${hz:-} ]]; then
      r "$t 미수신" "로봇이 켜져 있는지, 케이블이 맞는지"
    else
      exp=${want[$t]}
      awk -v h="$hz" -v e="$exp" 'BEGIN{ exit !(h > e*0.7) }' \
        && g "$t  ${hz} Hz  (기대 ${exp})" \
        || r "$t  ${hz} Hz — 기대 ${exp} 에 크게 못 미침" "대역폭·연결 확인"
    fi
  done
else
  sec "6. 로봇 연결"
  printf '  · 건너뜀.  로봇까지 보시려면:  %s robot\n' "$0"
fi

# ═══════════════════════════════════════════════ 결과
printf '\n\033[1m── 결과 ────────────────────────────────────────\033[0m\n'
printf '  통과 %d   경고 %d   실패 %d\n\n' "$PASS" "$WARN" "$FAIL"

if [[ $FAIL -gt 0 ]]; then
  RFAIL=$(( FAIL - ${RFAIL0:-$FAIL} ))
  if [[ $RFAIL -eq $FAIL ]]; then
    echo "  설치는 정상입니다. 로봇 연결만 확인하십시오."
    echo "      · USB-이더넷 케이블이 꽂혀 있는지"
    echo "      · 로봇 전원이 켜져 있고 부팅이 끝났는지"
    echo "      · source ~/setup_go2.sh 후 ros2 topic list"
  else
    echo "  실패 항목을 먼저 해결하십시오. 대부분은 아래로 고쳐집니다."
    echo "      $WS/tools/install_go2_lio.sh"
  fi
  exit 1
elif [[ $WARN -gt 0 ]]; then
  echo "  쓸 수 있습니다. 경고는 기능에 따라 필요할 수 있습니다."
else
  echo "  모두 정상입니다."
fi

cat <<EOF

  다음:
    ./tools/run_indoor.sh bag <녹화본>    재생으로 시험
    ./tools/run_indoor.sh                 실시간 (로봇 연결 후)
    docs/INTERFACE_indoor.md              토픽 쓰는 법

EOF
exit 0
