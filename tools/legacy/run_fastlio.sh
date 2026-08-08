#!/bin/bash
# FAST-LIO 실행 환경을 한 번에 소싱한다.
# 사용: source run_fastlio.sh   (bash run_fastlio.sh 아님 — 소싱해야 환경이 남는다)

echo "[1/3] Go2 환경 (setup_go2.sh) 소싱..."
source ~/setup_go2.sh

echo "[2/3] livox 워크스페이스 소싱..."
source ~/ws_livox/install/setup.bash

echo "[3/3] fastlio 워크스페이스 소싱..."
source ~/fastlio_ws/install/setup.bash

echo ""
echo "=============================================="
echo " 환경 준비 완료. 아래로 검증 후 실행하세요."
echo "=============================================="
echo " 1) 토픽 확인 :"
echo "      ros2 topic list --no-daemon | grep utlidar"
echo " 2) IMU 판별  :"
echo "      python3 ~/check_imu.py"
echo " 3) 실행      :"
echo "      ros2 launch fast_lio mapping.launch.py config_file:=go2_l1.yaml"
echo "    (L1 내장 IMU면 config_file:=go2_l1_lidar_imu.yaml)"
echo "=============================================="

# DDS 설정이 살아있는지 즉석 점검
echo ""
echo "[점검] 현재 DDS 관련 환경변수:"
env | grep -iE "cyclone|rmw_impl|ros_domain" | sed 's/^/    /' || echo "    (없음 — setup_go2.sh 확인 필요)"
