#!/usr/bin/env python3
# 평지 bag에서 z 드리프트 / 지도 기울기 / roll·pitch 진단
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
rows = []
while r.has_next():
    t, d, _ = r.read_next()
    if t != '/aft_mapped_to_init': continue
    m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)); pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    rows.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, p.x, p.y, p.z, np.degrees(roll), np.degrees(pitch)))
A = np.array(sorted(rows)); t = A[:, 0] - A[0, 0]
dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(A[:, 1]), np.diff(A[:, 2])))])   # 누적 수평 이동거리
slope = np.polyfit(dist, A[:, 3], 1)[0]                                                 # z vs 거리 기울기
print(f'z: 최소 {A[:,3].min():+.2f}  최대 {A[:,3].max():+.2f}  끝 {A[-1,3]:+.2f} m   (평지 기대 ≈ 0)')
print(f'z/거리 기울기 {slope*100:+.2f} %  ({np.degrees(np.arctan(slope)):+.2f}°)   ← 지도 기울어짐이면 여기 값이 큼')
print(f'roll  평균 {A[:,4].mean():+.2f}°  표준편차 {A[:,4].std():.2f}°  끝 {A[-1,4]:+.2f}°')
print(f'pitch 평균 {A[:,5].mean():+.2f}°  표준편차 {A[:,5].std():.2f}°  끝 {A[-1,5]:+.2f}°')
