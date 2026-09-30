#!/usr/bin/env python3
# speed_ratio.py — 10 s 창마다 GPS 이동거리 ÷ 다리(raw) 이동거리 = 창별 축척 k, 속도 구간별로 모아 보기
# 사용: python3 ~/data/bags/speed_ratio.py <bag> [bag ...]
import sys, os, json, math
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String

WIN = 10.0

def reader(path, topics):
    with open(os.path.join(path, 'metadata.yaml')) as f:
        m = yaml.safe_load(f)['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialCompressionReader() if m.get('compression_mode') else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=m.get('storage_identifier', 'sqlite3')),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r

pool = []
for path in (p.rstrip('/') for p in sys.argv[1:]):
    gps, leg = [], []
    r = reader(path, ['/gnss', '/utlidar/robot_odom'])
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        if topic == '/gnss':
            try:
                g = json.loads(deserialize_message(data, String).data)
                if str(g.get('fixed', 0)).lower() in ('1', '1.0', 'true') and 0 < float(g.get('hdop', 99)) < 90:
                    gps.append((t, float(g['latitude']), float(g['longitude'])))
            except Exception:
                pass
        else:
            p = deserialize_message(data, Odometry).pose.pose.position
            leg.append((t, p.x, p.y))
    if len(gps) < 10:
        print(f'{os.path.basename(path)}: GPS 고정 샘플 부족')
        continue
    gps, leg = np.array(gps), np.array(leg)
    tg = gps[:, 0]
    E = (gps[:, 2] - gps[0, 2]) * 111320 * math.cos(math.radians(gps[0, 1]))
    N = (gps[:, 1] - gps[0, 1]) * 111320
    rows = []
    for t0 in np.arange(tg[0], tg[-1] - WIN, WIN):
        t1 = t0 + WIN
        inside = tg[(tg >= t0) & (tg <= t1)]
        if len(inside) < 0.7 * WIN or np.diff(inside).max() > 3:
            continue
        dg = math.hypot(np.interp(t1, tg, E) - np.interp(t0, tg, E), np.interp(t1, tg, N) - np.interp(t0, tg, N))
        dl = math.hypot(np.interp(t1, leg[:, 0], leg[:, 1]) - np.interp(t0, leg[:, 0], leg[:, 1]),
                        np.interp(t1, leg[:, 0], leg[:, 2]) - np.interp(t0, leg[:, 0], leg[:, 2]))
        if dg > 5 and dl > 1 and dg / WIN < 5:
            rows.append((dg / WIN, dl / WIN, dg / dl))
    if not rows:
        print(f'{os.path.basename(path)}: 쓸 만한 이동 창 없음')
        continue
    rows = np.array(rows)
    q1, q2, q3 = np.percentile(rows[:, 2], [25, 50, 75])
    print(f'{os.path.basename(path)}: 창 {len(rows):3d}개 · k 중앙 {q2:.2f} (IQR {q1:.2f}–{q3:.2f}) · '
          f'GPS 속도 {np.median(rows[:, 0]):.2f} m/s · 다리 raw 속도 {np.median(rows[:, 1]):.2f} m/s')
    pool.append(rows)

A = np.vstack(pool)
print('\nGPS 속도 구간별 k (모든 bag 합산)')
for lo, hi in [(0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 5.0)]:
    s = A[(A[:, 0] >= lo) & (A[:, 0] < hi)]
    if len(s):
        print(f'  {lo:.1f}–{hi:.1f} m/s : 창 {len(s):3d}개 · k 중앙 {np.median(s[:, 2]):.2f}')
