#!/usr/bin/env bash
GO2_SUBNET="192.168.123"
GO2_ROBOT_IP="192.168.123.161"


MODE="${1:-auto}"
if [ "$MODE" = "wifi" ]; then
    GO2_IFACE="$WIFI_IFACE"
else
    GO2_IFACE=$(ip -o -4 addr show \
        | awk -v net="$GO2_SUBNET" '$4 ~ ("^" net "\\.") {print $2; exit}')
fi

#GO2_IFACE="wlp45s0"
#MODE="WIFI"

if [ -z "$GO2_IFACE" ]; then
    echo "[setup_go2] ✗ 인터페이스를 찾지 못했습니다. (${GO2_SUBNET}.x IP 확인 필요)"
    return 1 2>/dev/null || exit 1
fi


source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${GO2_IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
export ROS_DOMAIN_ID=0
echo "[Go2] CycloneDDS 환경 로드됨 (RMW=$RMW_IMPLEMENTATION)"

if ping -c1 -W1 "$GO2_ROBOT_IP" >/dev/null 2>&1; then
    echo "[setup_go2] ✓ 로봇(${GO2_ROBOT_IP}) 응답 확인"
else
    echo "[setup_go2] · 로봇(${GO2_ROBOT_IP}) 무응답 (WiFi 모드면 정상일 수 있음)"
fi
echo "[setup_go2] ✓ 인터페이스=${GO2_IFACE}  DOMAIN_ID=${ROS_DOMAIN_ID}  MODE=${MODE}"
