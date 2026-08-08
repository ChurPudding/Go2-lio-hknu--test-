#!/usr/bin/env bash
# =============================================================
#  Go2 로봇 개발 환경 재설치 스크립트  (Ubuntu 22.04 / ROS2 Humble)
#  - 효신님 Go2 프로젝트 환경 복원용
#  - 각 단계는 독립적으로 실행 가능하도록 함수로 분리
#  - 실행 전 반드시 README 부분을 읽어주세요
# =============================================================
#
#  [사용법]
#    1) chmod +x setup_go2_env.sh
#    2) ./setup_go2_env.sh            # 전체 순서대로 실행
#    또는 개별 단계만:
#       ./setup_go2_env.sh base       # 기본 도구만
#       ./setup_go2_env.sh ros2       # ROS2만
#       ./setup_go2_env.sh unitree    # unitree_ros2 / sdk 만
#
#  [주의]
#    - sudo 비밀번호를 몇 번 물어봅니다.
#    - unitree(cyclonedds) 빌드는 ROS2가 source 안 된 새 터미널에서 해야
#      안전합니다. 이 스크립트는 그 부분을 자동 처리합니다.
# =============================================================

set -Eeuo pipefail   # 오류 발생 시 즉시 중단, 정의 안 된 변수 사용 금지

# ---- 공통 설정 ------------------------------------------------
ROS_DISTRO="humble"
WORKDIR="${HOME}"
LOG() { echo -e "\n\033[1;32m[SETUP]\033[0m $*"; }
WARN() { echo -e "\n\033[1;33m[주의]\033[0m $*"; }

# ---- 0. 시스템 업데이트 & 한글 폴더 → 영어 -------------------
step_base() {
  LOG "0-1. 시스템 패키지 업데이트"
  sudo apt update && sudo apt upgrade -y

  LOG "0-2. 기본 개발 도구 설치"
  sudo apt install -y \
    build-essential cmake git curl wget vim \
    software-properties-common apt-transport-https \
    gpg net-tools htop tree unzip \
    python3-pip python3-venv

  LOG "0-3. universe 저장소 활성화 (ROS2에 필요)"
  sudo add-apt-repository -y universe

  LOG "0-4. 홈 디렉토리 폴더 이름을 영어로 변경"
  # 시스템 언어는 한국어로 두고 폴더명만 영어로. 창이 뜨면 'Update' 선택.
  sudo apt install -y xdg-user-dirs-gtk
  LANG=C xdg-user-dirs-gtk-update || WARN "폴더명 변경 창이 안 뜨면 로그인 후 수동 실행하세요."
}

# ---- 1. Google Chrome ---------------------------------------
step_chrome() {
  LOG "1. Google Chrome 설치"
  local deb="/tmp/google-chrome.deb"
  wget -q -O "${deb}" \
    "https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb"
  sudo apt install -y "${deb}"
  rm -f "${deb}"
}

# ---- 2. VS Code (Microsoft 공식 apt 저장소) ------------------
step_vscode() {
  LOG "2. VS Code 설치 (Microsoft 공식 저장소)"
  # 2-1. Microsoft GPG 키 등록
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
    | gpg --dearmor > /tmp/packages.microsoft.gpg
  sudo install -D -o root -g root -m 644 \
    /tmp/packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
  rm -f /tmp/packages.microsoft.gpg

  # 2-2. 저장소 추가
  echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
    | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

  # 2-3. 설치
  sudo apt update
  sudo apt install -y code
}

# ---- 3. Node.js 22 + Claude Code ----------------------------
step_claude() {
  LOG "3-1. Node.js 22 설치 (Claude Code npm 방식은 Node 22+ 필요)"
  curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
  sudo apt install -y nodejs

  LOG "3-2. npm 전역 경로를 사용자 홈으로 설정 (sudo 없이 설치하기 위함)"
  mkdir -p "${HOME}/.npm-global"
  npm config set prefix "${HOME}/.npm-global"
  # .bashrc 에 PATH 추가 (중복 방지)
  if ! grep -q 'npm-global/bin' "${HOME}/.bashrc"; then
    echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> "${HOME}/.bashrc"
  fi
  export PATH="${HOME}/.npm-global/bin:$PATH"

  LOG "3-3. Claude Code 설치"
  npm install -g @anthropic-ai/claude-code
  claude --version || WARN "claude 명령을 못 찾으면 새 터미널을 열어 확인하세요."
}

# ---- 4. ROS2 Humble -----------------------------------------
step_ros2() {
  LOG "4-1. 로케일 UTF-8 설정"
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

  LOG "4-2. ROS2 apt 저장소 등록 (GPG 키 + 저장소 직접 등록 방식)"
  # 참고: 최신 ros-apt-source(.deb) 방식도 있으나 GitHub API rate limit 에
  #       걸리면 실패할 수 있어, 더 안정적인 전통 방식을 기본으로 사용합니다.
  sudo apt install -y curl gnupg lsb-release
  local codename
  codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"   # jammy
  # GPG 키 등록
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  # 저장소 등록
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  LOG "4-3. ROS2 Humble Desktop + 개발 도구 설치"
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-dev-tools
  sudo apt install -y \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

  LOG "4-4. rosdep 초기화"
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update

  LOG "4-5. .bashrc 에 ROS2 source 추가"
  if ! grep -q 'source /opt/ros/humble/setup.bash' "${HOME}/.bashrc"; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${HOME}/.bashrc"
  fi
  # colcon 자동완성
  if ! grep -q 'colcon_argcomplete' "${HOME}/.bashrc"; then
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> "${HOME}/.bashrc"
  fi
}

# ---- 5. unitree_sdk2 (C++ SDK) ------------------------------
step_sdk() {
  LOG "5. unitree_sdk2 클론 & 빌드"
  cd "${WORKDIR}"
  if [ ! -d "${WORKDIR}/unitree_sdk2" ]; then
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
  fi
  cd "${WORKDIR}/unitree_sdk2"
  mkdir -p build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
  make -j"$(nproc)"
  sudo make install
  LOG "unitree_sdk2 설치 완료 (/opt/unitree_robotics)"
}

# ---- 6. unitree_ros2 (CycloneDDS 기반 메시지 패키지) --------
step_unitree() {
  LOG "6. unitree_ros2 설치 (Go2 메시지 + CycloneDDS)"
  cd "${WORKDIR}"
  if [ ! -d "${WORKDIR}/unitree_ros2" ]; then
    git clone https://github.com/unitreerobotics/unitree_ros2.git
  fi

  LOG "6-1. CycloneDDS 의존 패키지 설치 (Humble은 apt 패키지로 충분)"
  # 공식 문서: Humble을 쓰면 cyclonedds 소스 빌드는 건너뛰어도 됨.
  sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    libyaml-cpp-dev

  LOG "6-2. cyclonedds_ws 빌드 (unitree_go / unitree_api 메시지 패키지)"
  # 핵심: 이 빌드는 ROS2가 source 된 환경에서 해야 합니다.
  #       현재 셸의 .bashrc 소싱과 무관하게, 서브셸에서 명시적으로 source 후 빌드.
  #       (Humble이므로 cyclonedds 자체를 소스로 받을 필요 없음 → colcon build 만)
  cd "${WORKDIR}/unitree_ros2/cyclonedds_ws"
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

  LOG "6-3. example 워크스페이스 빌드 (read_motion_state 등 예제)"
  cd "${WORKDIR}/unitree_ros2/example"
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
           source ${WORKDIR}/unitree_ros2/cyclonedds_ws/install/setup.bash && \
           colcon build"

  LOG "6-4. Go2 전용 환경 스크립트(setup_go2.sh) 생성"
  # NIC 이름은 실제 환경에 맞게 수정 필요 (효신님 이전 환경: enxc0eac369bf02)
  cat > "${WORKDIR}/unitree_ros2/setup_go2.sh" <<'EOSH'
#!/usr/bin/env bash
# Go2 전용 ROS2 환경 (CycloneDDS). 새 터미널마다 source 해서 사용.
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 아래 NetworkInterface name 을 실제 Go2 연결 NIC 로 바꿔주세요 (ip a 로 확인)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enxc0eac369bf02" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
echo "[Go2] CycloneDDS 환경 로드됨 (RMW=$RMW_IMPLEMENTATION)"
EOSH
  chmod +x "${WORKDIR}/unitree_ros2/setup_go2.sh"
  WARN "setup_go2.sh 안의 NIC 이름(enxc0eac369bf02)을 'ip a' 로 확인 후 수정하세요."
}

# ---- 7. 파이썬 인지 파이프라인 라이브러리 -------------------
step_python() {
  LOG "7. Open3D / YOLO / 기타 파이썬 라이브러리 설치"
  # 효신님 이전 환경의 버전 핀 반영 (numpy<2, opencv<4.10)
  pip3 install --upgrade pip
  pip3 install \
    "numpy<2" \
    "opencv-python<4.10" \
    open3d \
    ultralytics \
    matplotlib \
    scipy
  WARN "ROS2 와 numpy 충돌이 나면 가상환경(venv) 사용을 권장합니다."
}

# =============================================================
#  메인 실행부
# =============================================================
main() {
  local target="${1:-all}"
  case "${target}" in
    base)     step_base ;;
    chrome)   step_chrome ;;
    vscode)   step_vscode ;;
    claude)   step_claude ;;
    ros2)     step_ros2 ;;
    sdk)      step_sdk ;;
    unitree)  step_unitree ;;
    python)   step_python ;;
    all)
      step_base
      step_chrome
      step_vscode
      step_claude
      step_ros2
      step_sdk
      step_unitree
      step_python
      LOG "===== 전체 설치 완료! 새 터미널을 열어 확인하세요. ====="
      echo "  - ros2 --version"
      echo "  - code --version"
      echo "  - claude --version"
      echo "  - google-chrome --version"
      ;;
    *)
      echo "사용법: $0 [all|base|chrome|vscode|claude|ros2|sdk|unitree|python]"
      exit 1
      ;;
  esac
}

main "$@"
