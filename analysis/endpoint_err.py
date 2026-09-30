#!/usr/bin/env python3
# 왕복 bag의 출발점 복귀 오차: 앞/뒤 60s 정지구간 평균 위치의 xy 거리 + 경로길이
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

TOPICS = {'/aft_mapped_to_init': 'Point-LIO', '/utlidar/robot_odom': 'leg'}
WIN = 60.0

def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    out = {t: [] for t in TOPICS}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic in out:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            out[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y))
    return out

def mean_xy(rows):
    return (sum(r[1] for r in rows) / len(rows), sum(r[2] for r in rows) / len(rows))

def path_len(rows, step=0.5):
    L, last_t, last = 0.0, None, None
    for t, x, y in rows:
        if last_t is None or t - last_t >= step:
            if last is not None:
                L += math.hypot(x - last[0], y - last[1])
            last_t, last = t, (x, y)
    return L

bag = sys.argv[1]
d = read(bag)
print(f'bag: {bag}')
for topic, name in TOPICS.items():
    rows = sorted(d[topic])
    if not rows:
        print(f'{name:9s}: 샘플 없음'); continue
    t0, t1 = rows[0][0], rows[-1][0]
    s = mean_xy([r for r in rows if r[0] <= t0 + WIN])
    e = mean_xy([r for r in rows if r[0] >= t1 - WIN])
    err = math.hypot(e[0] - s[0], e[1] - s[1])
    L = path_len(rows)
    print(f'{name:9s}: 샘플 {len(rows):6d} | 시작 ({s[0]:7.2f},{s[1]:7.2f}) → 끝 ({e[0]:7.2f},{e[1]:7.2f})'
          f' | 복귀오차 {err:6.2f} m | 경로길이 {L:7.1f} m | {100 * err / L if L else 0:5.2f} %')
