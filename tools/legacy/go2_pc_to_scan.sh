#!/usr/bin/env bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/utlidar/cloud_deskewed \
  -r scan:=/scan \
  -p target_frame:=base \
  -p transform_tolerance:=0.05 \
  -p min_height:=-0.10 \
  -p max_height:=0.50 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.0667 \
  -p range_min:=0.20 \
  -p range_max:=15.0 \
  -p use_inf:=true
