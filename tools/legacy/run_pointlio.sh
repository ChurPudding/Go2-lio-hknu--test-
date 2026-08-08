#!/usr/bin/env bash
# Go2 + livox + Point-LIO 소싱 후 실행
source ~/setup_go2.sh
source ~/ws_livox/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
ros2 launch point_lio mapping_unilidar_l1.launch.py "$@"
