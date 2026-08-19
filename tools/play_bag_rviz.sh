#!/usr/bin/env bash
# play_bag_rviz.sh — 녹화본을 RViz 로 본다 (점군 + GPS 궤적)
#
# [source 용] 아래 어느 쪽으로도 실행됩니다:
#   source tools/play_bag_rviz.sh <bag>     (권장 — 사용자 기본 습관)
#   ./tools/play_bag_rviz.sh <bag>
# 본문 전체를 서브셸 ( ... ) 로 감싸 두었기 때문에 source 로 실행해도
# set -o pipefail, 함수(has), PIDS, trap, CYCLONEDDS_URI unset 이 호출한
# 셸로 새어나가지 않고, bag 인자를 빠뜨렸을 때의 `exit 1` 도 서브셸만
# 끝낼 뿐 source 한 터미널을 닫지 않습니다.
#
# set -u 는 일부러 쓰지 않습니다. /opt/ros/humble/setup.bash 가
# $AMENT_TRACE_SETUP_FILES 를 기본값 없이 참조하기 때문에, set -u 상태에서
# ROS 환경을 source 하면 "unbound variable" 로 즉시 죽습니다 (이번에 겪은
# 증상의 원인). 대신 사용자 입력 변수는 아래처럼 ${1:-} 형태로 직접 방어합니다.
#
# 왜 스크립트로 만드나
#   매번 터미널 세 개에 source 를 치고 RViz 에서 Reliability 를 Best Effort 로
#   바꾸는 일을 반복하게 된다. Best Effort 를 빠뜨리면 토픽은 보이는데 점이
#   하나도 안 뜨고, 원인을 찾는 데 시간이 든다. 설정 파일로 고정해 둔다.
#
# 하는 일
#   1. bag 안에 어떤 토픽이 있는지 확인해 볼 것을 고른다
#   2. RViz 설정 파일을 그 bag 에 맞춰 생성한다
#   3. RViz 와 bag 재생을 함께 띄운다
#   4. Ctrl-C 하면 둘 다 정리한다
#
# 사용
#   source play_bag_rviz.sh ~/go2/bags/outdoor/0812/go2_loop1_0812_1449
#   source play_bag_rviz.sh <bag> 1.0          # 배속 (기본 0.5)
#   GPS=1 source play_bag_rviz.sh <bag>        # GPS 궤적도 함께 (프레임 별도)

(
set -o pipefail

BAG="${1:-}"
RATE="${2:-0.5}"
GPS="${GPS:-0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$BAG" ]; then
    echo "사용: $0 <bag 폴더> [배속]"
    echo "예:   $0 ~/go2/bags/outdoor/0812/go2_loop1_0812_1449"
    exit 1
fi

# .db3 를 넘겨도 폴더로 바꿔 받는다
[ -f "$BAG" ] && BAG="$(dirname "$BAG")"
BAG="$(cd "$BAG" 2>/dev/null && pwd)" || { echo "bag 폴더 없음: $1"; exit 1; }
[ -f "$BAG/metadata.yaml" ] || echo "[warn] metadata.yaml 이 없습니다. 재생이 안 되면 tools/fix_rosbag2_metadata.py 를 쓰세요."

# ── 환경 ──────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[ -f ~/unitree_ros2/cyclonedds_ws/install/setup.bash ] \
    && source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
unset CYCLONEDDS_URI          # 로컬 재생. srcoff 와 같은 역할

echo "bag   $(basename "$BAG")"
echo "배속  ${RATE}x"

# ── 1. 토픽 확인 ──────────────────────────────────────────────────────
INFO="$(ros2 bag info "$BAG" 2>/dev/null)"
has() { grep -q "$1" <<< "$INFO"; }

if has "/utlidar/cloud_base"; then
    CLOUD="/utlidar/cloud_base";  FRAME="base_link"
elif has "/utlidar/cloud "; then
    CLOUD="/utlidar/cloud";       FRAME="utlidar_lidar"
elif has "/utlidar/cloud_deskewed"; then
    CLOUD="/utlidar/cloud_deskewed"; FRAME="odom"
    echo "[warn] cloud_deskewed 만 있습니다. 중복 88.6% 라 무겁습니다."
else
    echo "점군 토픽을 찾지 못했습니다. bag 내용:"
    echo "$INFO"
    exit 1
fi

# bag 에 TF 가 없으면 센서 자기 프레임을 써야 한다
if ! has "/tf"; then
    if [ "$CLOUD" != "/utlidar/cloud" ] && has "/utlidar/cloud "; then
        echo "[info] bag 에 /tf 가 없어 /utlidar/cloud 로 바꿉니다."
        CLOUD="/utlidar/cloud"; FRAME="utlidar_lidar"
    fi
fi

echo "점군  $CLOUD   (fixed frame: $FRAME)"

# ── 2. RViz 설정 생성 ─────────────────────────────────────────────────
CFG="/tmp/play_bag_$$.rviz"
cat > "$CFG" <<EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: $FRAME
    Frame Rate: 30
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane Cell Count: 40
      Color: 100; 100; 100
    - Class: rviz_default_plugins/PointCloud2
      Name: LiDAR
      Enabled: true
      Topic:
        Value: $CLOUD
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Best Effort
        History Policy: Keep Last
      Style: Points
      Size (Pixels): 3
      Alpha: 1
      Decay Time: 0
      Color Transformer: AxisColor
      Axis: Z
      Use rainbow: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Marker Scale: 0.6
      Show Names: true
      Show Axes: true
      Show Arrows: false
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25
      Pitch: 0.6
      Yaw: 0.8
      Focal Point: {X: 0, Y: 0, Z: 0}
EOF

# ── 3. 실행 ───────────────────────────────────────────────────────────
PIDS=()
cleanup() {
    echo
    echo "정리 중..."
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
    wait 2>/dev/null
    rm -f "$CFG"
    echo "종료"
}
trap cleanup EXIT INT TERM

ros2 run rviz2 rviz2 -d "$CFG" >/dev/null 2>&1 &
PIDS+=($!)
sleep 2

if [ "$GPS" = "1" ]; then
    if has "/gnss"; then
        python3 "$HERE/gnss_bridge.py" >/dev/null 2>&1 &
        PIDS+=($!)
        python3 "$HERE/gnss_path.py" >/dev/null 2>&1 &
        PIDS+=($!)
        echo "GPS   /gps/path 발행 중 — RViz 에서 Fixed Frame 을 gps_local 로"
        echo "      바꾸고 Path 를 추가하세요. 점군과 프레임이 달라 동시에는"
        echo "      보이지 않습니다."
    else
        echo "[warn] bag 에 /gnss 가 없어 GPS 를 건너뜁니다."
    fi
fi

echo
echo "재생 시작. 멈추려면 Ctrl-C"
echo

ros2 bag play "$BAG" -r "$RATE" --loop --clock
)
