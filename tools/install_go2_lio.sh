#!/usr/bin/env bash
# install_go2_lio.sh -- 새 PC 에 Go2 실내 LIO 파이프라인을 설치한다
#
#   ./install_go2_lio.sh              전체 설치
#   ./install_go2_lio.sh check        무엇이 이미 설치됐는지만 확인
#   ./install_go2_lio.sh <단계번호>    특정 단계만 다시
#
# 다시 실행해도 안전하다. 이미 끝난 단계는 건너뛴다.
#
# 단계
#   1  apt 패키지
#   2  unitree_ros2   (메시지 + CycloneDDS)   ← 오래 걸린다
#   3  Livox-SDK2     (sudo 필요)
#   4  livox_ros_driver2
#   5  Point-LIO
#   6  도구·설정 배치
#
# 전제
#   Ubuntu 22.04, ROS2 Humble 이 이미 설치돼 있을 것.
#   ROS2 가 없으면 먼저 https://docs.ros.org 의 Humble 설치를 따를 것.

set -u
STEP=${1:-all}
LOG=/tmp/go2_install.log
: > "$LOG"

ok()   { printf '  \033[32m✓\033[0m %s\n' "$1"; }
ng()   { printf '  \033[31m✗\033[0m %s\n' "$1"; }
info() { printf '  · %s\n' "$1"; }
die()  { echo; ng "$1"; echo "  로그: $LOG"; exit 1; }
head1() { echo; echo "━━ $1 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }

run() {  # run <설명> <명령...>
  local d=$1; shift
  printf '  %-42s' "$d"
  if "$@" >>"$LOG" 2>&1; then echo "OK"; else echo "실패"; return 1; fi
}

# ────────────────────────────────────────────────────── 상태 확인
have_apt()   { dpkg -l ros-humble-desktop >/dev/null 2>&1; }
have_uni()   { [[ -f ~/unitree_ros2/cyclonedds_ws/install/setup.bash ]]; }
have_sdk()   { [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]] \
               || [[ -f /usr/local/lib/liblivox_lidar_sdk_static.a ]]; }
have_livox() { [[ -f ~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/package.xml ]]; }
have_plio()  { [[ -f ~/catkin_point_lio_unilidar/install/point_lio/share/point_lio/package.xml ]]; }
have_tools() { [[ -x ~/fastlio_ws/tools/run_indoor.sh ]]; }

status() {
  head1 "설치 상태"
  have_apt   && ok "1  apt 패키지"        || ng "1  apt 패키지"
  have_uni   && ok "2  unitree_ros2"      || ng "2  unitree_ros2"
  have_sdk   && ok "3  Livox-SDK2"        || ng "3  Livox-SDK2"
  have_livox && ok "4  livox_ros_driver2" || ng "4  livox_ros_driver2"
  have_plio  && ok "5  Point-LIO"         || ng "5  Point-LIO"
  have_tools && ok "6  도구·설정"          || ng "6  도구·설정"
  echo
}

[[ $STEP == check ]] && { status; exit 0; }

if ! ls /opt/ros/humble/setup.bash >/dev/null 2>&1; then
  die "ROS2 Humble 이 없습니다. 먼저 설치하십시오."
fi

# ────────────────────────────────────────────────────── 1 apt
step1() {
  head1 "1  apt 패키지"
  have_apt && { info "이미 설치됨 — 건너뜀"; return 0; }
  sudo apt update >>"$LOG" 2>&1
  run "패키지 설치" sudo apt install -y \
      git cmake build-essential \
      ros-humble-rmw-cyclonedds-cpp ros-humble-pcl-ros ros-humble-tf2-ros \
      ros-humble-nav2-map-server \
      libeigen3-dev libpcl-dev python3-colcon-common-extensions \
      python3-numpy python3-scipy python3-matplotlib fonts-nanum \
    || die "apt 실패"
}

# ────────────────────────────────────────────────────── 2 unitree_ros2
step2() {
  head1 "2  unitree_ros2  (수 분 소요)"
  have_uni && { info "이미 설치됨 — 건너뜀"; return 0; }
  [[ -d ~/unitree_ros2 ]] \
    || run "clone" git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2 \
    || die "clone 실패"

  # CycloneDDS 소스가 없으면 받는다
  if [[ ! -d ~/unitree_ros2/cyclonedds_ws/src/cyclonedds ]]; then
    mkdir -p ~/unitree_ros2/cyclonedds_ws/src
    run "cyclonedds clone" git clone https://github.com/eclipse-cyclonedds/cyclonedds \
        -b releases/0.10.x ~/unitree_ros2/cyclonedds_ws/src/cyclonedds \
      || die "cyclonedds clone 실패"
  fi

  info "빌드 중... (10분 이상 걸릴 수 있습니다)"
  ( set +u; source /opt/ros/humble/setup.bash; set -u
    cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select cyclonedds \
    && colcon build ) >>"$LOG" 2>&1 || die "unitree_ros2 빌드 실패"
  have_uni && ok "완료" || die "빌드는 끝났으나 setup.bash 가 없습니다"
}

# ────────────────────────────────────────────────────── 3 Livox-SDK2
step3() {
  head1 "3  Livox-SDK2  (sudo 필요)"
  have_sdk && { info "이미 설치됨 — 건너뜀"; return 0; }
  [[ -d ~/Livox-SDK2 ]] \
    || run "clone" git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2 \
    || die "clone 실패"
  ( mkdir -p ~/Livox-SDK2/build && cd ~/Livox-SDK2/build \
    && cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig \
  ) >>"$LOG" 2>&1 || die "Livox-SDK2 빌드/설치 실패"
  have_sdk && ok "완료" || die "설치 확인 실패"
}

# ────────────────────────────────────────────────────── 4 livox_ros_driver2
step4() {
  head1 "4  livox_ros_driver2"
  have_livox && { info "이미 설치됨 — 건너뜀"; return 0; }
  mkdir -p ~/ws_livox/src
  [[ -d ~/ws_livox/src/livox_ros_driver2 ]] \
    || run "clone" git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
       ~/ws_livox/src/livox_ros_driver2 \
    || die "clone 실패"

  # colcon 이 아니라 전용 build.sh 를 소스 폴더 안에서 실행해야 한다.
  # build.sh 가 package.xml / CMakeLists 를 ROS2 용으로 바꿔치기 하기 때문이다.
  info "build.sh humble 실행 (colcon 아님)"
  ( set +u; source /opt/ros/humble/setup.bash; set -u
    cd ~/ws_livox/src/livox_ros_driver2 && ./build.sh humble ) >>"$LOG" 2>&1 \
    || die "livox_ros_driver2 빌드 실패"
  have_livox && ok "완료" || die "빌드 확인 실패"
}

# ────────────────────────────────────────────────────── 5 Point-LIO
step5() {
  head1 "5  Point-LIO"
  have_plio && { info "이미 설치됨 — 건너뜀"; return 0; }
  mkdir -p ~/catkin_point_lio_unilidar/src
  [[ -d ~/catkin_point_lio_unilidar/src/point_lio_ros2 ]] \
    || run "clone" git clone https://github.com/dfloreaa/point_lio_ros2.git \
       ~/catkin_point_lio_unilidar/src/point_lio_ros2 \
    || die "clone 실패"

  # livox 를 먼저 source 해야 빌드된다
  ( set +u
    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    set -u
    cd ~/catkin_point_lio_unilidar && colcon build --symlink-install ) >>"$LOG" 2>&1 \
    || die "Point-LIO 빌드 실패"
  have_plio && ok "완료" || die "빌드 확인 실패"
}

# ────────────────────────────────────────────────────── 6 도구·설정
step6() {
  head1 "6  도구·설정"
  local SRC
  SRC=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)   # 이 스크립트의 상위 = fastlio_ws

  mkdir -p ~/fastlio_ws
  if [[ $SRC != "$HOME/fastlio_ws" ]]; then
    run "tools 복사"   cp -r "$SRC/tools"   ~/fastlio_ws/ || die "복사 실패"
    run "docs 복사"    cp -r "$SRC/docs"    ~/fastlio_ws/ || true
    run "results 복사" cp -r "$SRC/results" ~/fastlio_ws/ || true
  else
    info "이미 제자리 — 복사 생략"
  fi
  chmod +x ~/fastlio_ws/tools/*.sh 2>/dev/null

  # Point-LIO 설정: Go2 L1 토픽으로 맞춘다
  local CFG=~/catkin_point_lio_unilidar/src/point_lio_ros2/config/unilidar_l1.yaml
  if [[ -f $CFG ]]; then
    sed -i 's|lid_topic:.*|lid_topic:  "/utlidar/cloud"|' "$CFG"
    sed -i 's|imu_topic:.*|imu_topic:  "/l1_imu_fixed"|' "$CFG"
    grep -q 'pcd_save_en: *true' "$CFG" \
      || sed -i 's|pcd_save_en:.*|pcd_save_en: true|' "$CFG"
    ok "unilidar_l1.yaml 설정 완료 (/utlidar/cloud, /l1_imu_fixed)"
  else
    ng "unilidar_l1.yaml 을 찾지 못했습니다 — 수동 확인 필요"
  fi

  # setup_go2.sh
  if [[ ! -f ~/setup_go2.sh ]]; then
    if [[ -f "$SRC/setup_go2.sh" ]]; then
      cp "$SRC/setup_go2.sh" ~/ && ok "setup_go2.sh 배치"
    else
      ng "setup_go2.sh 가 없습니다 — 효신님 PC 에서 복사해 오십시오"
    fi
  else
    info "setup_go2.sh 이미 있음"
  fi

  # 편의 별칭
  if ! grep -q 'alias srcoff' ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc <<'EOF'

# --- Go2 LIO ---------------------------------------------------------
# 오프라인(bag 재생·알고리즘 실행)용. CYCLONEDDS_URI 를 해제해 실기와 분리한다.
alias srcoff='source /opt/ros/humble/setup.bash \
 && source ~/unitree_ros2/cyclonedds_ws/install/setup.bash \
 && source ~/catkin_point_lio_unilidar/install/setup.bash \
 && unset CYCLONEDDS_URI'
EOF
    ok "srcoff 별칭 추가 (새 터미널부터 적용)"
  else
    info "srcoff 별칭 이미 있음"
  fi
}

# ────────────────────────────────────────────────────── 실행
case $STEP in
  1) step1 ;;
  2) step2 ;;
  3) step3 ;;
  4) step4 ;;
  5) step5 ;;
  6) step6 ;;
  all) step1; step2; step3; step4; step5; step6 ;;
  *) echo "사용: $0 [check|all|1..6]"; exit 1 ;;
esac

status
cat <<'EOF'
━━ 다음 할 일 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. 새 터미널을 열어 별칭을 적용합니다.

  2. 로봇 연결 확인
       source ~/setup_go2.sh
       ros2 topic hz /utlidar/cloud       # 15 Hz
       ros2 topic hz /lowstate            # 500 Hz

  3. 파이프라인 실행
       cd ~/fastlio_ws
       ./tools/run_indoor.sh              # 실시간
       ./tools/run_indoor.sh bag <경로>   # 녹화본

  4. 사용법은 docs/INTERFACE_indoor.md 를 보십시오.

  ※ 로봇을 이 PC 에 USB-이더넷으로 직접 연결해야 합니다.
     내장 랜포트는 연결이 끊긴 사례가 있습니다.

EOF
