#!/usr/bin/env python3
# =============================================================================
# bag_to_csv.py  (rev3 - time_sync 보정 추가)
# rosbag2 -> MATLAB CSV.  가속도 시각정렬 방식 두 가지 지원.
#   기본(held)        : 각 자이로에 '최신' 본체 가속도를 붙임 (l1_imu_fix baseline)
#   --time-sync       : 200Hz 표본화 + tick->L1 offset + 선형보간 (l1_imu_fix redesign)
# 회전(R_LB)·스케일은 여기서 하지 않고 MATLAB 에서 적용 (선형연산이라 순서 무관).
#
# ★ unitree_go 메시지가 있는 ws 를 source 후 실행 (/lowstate 읽기용).
#   python3 bag_to_csv.py ~/data/bags/ekf_test1 --start 0 --dur 90 --lidar-stride 3 --time-sync
# =============================================================================
import argparse, os, glob, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs_py import point_cloud2 as pc2

CALIB_MIN = 200      # offset 확정 최소 표본 (l1_imu_fix 와 동일)
OFF_Q     = 0.90     # offset 상단분위 (lag 편향 제거)
PREROLL   = 2.0      # 창 시작 전 몇 초를 accel 웜업으로 더 읽을지


def detect_storage(uri):
    if os.path.isdir(uri):
        if glob.glob(os.path.join(uri, '*.mcap')):  return 'mcap'
        if glob.glob(os.path.join(uri, '*.db3')):   return 'sqlite3'
    elif uri.endswith('.mcap'): return 'mcap'
    elif uri.endswith('.db3'):  return 'sqlite3'
    return 'sqlite3'


def open_reader(uri):
    so = rosbag2_py.StorageOptions(uri=uri, storage_id=detect_storage(uri))
    r = rosbag2_py.SequentialReader(); r.open(so, rosbag2_py.ConverterOptions('cdr','cdr'))
    return r


def stamp_s(msg, recv):
    s = msg.header.stamp
    return recv*1e-9 if (s.sec == 0 and s.nanosec == 0) else s.sec + s.nanosec*1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--imu-topic',   default='/utlidar/imu')
    ap.add_argument('--acc-topic',   default='/lowstate')
    ap.add_argument('--cloud-topic', default='/utlidar/cloud')
    ap.add_argument('--imu-out',     default='imu.csv')
    ap.add_argument('--lidar-out',   default='lidar.csv')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--dur',   type=float, default=None)
    ap.add_argument('--lidar-stride', type=int, default=1)
    ap.add_argument('--time-sync', action='store_true',
                    help='켜면 200Hz 표본화 + tick 정렬 + 선형보간 (기본: held)')
    args = ap.parse_args()

    reader = open_reader(args.bag)
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for tp in (args.imu_topic, args.acc_topic, args.cloud_topic):
        if tp not in typemap:
            print(f'[!] 토픽 없음: {tp}\n    {sorted(typemap)}'); sys.exit(1)
    ImuT = get_message(typemap[args.imu_topic])
    AccT = get_message(typemap[args.acc_topic])
    PcT  = get_message(typemap[args.cloud_topic])

    gyros = []            # (h_g_sec, wx,wy,wz)   출력 대상(창 안)
    lidar_frames = []     # (t_sec, Nx3)
    held_acc = None
    # time_sync 수집물
    fresh_t = []; fresh_a = []; last_fresh = None
    d_buf = []; ref_stamp = None
    # held 방식 저장용
    held_rows = []
    tbag0 = None

    t_end = None if args.dur is None else args.start + args.dur

    while reader.has_next():
        topic, data, recv = reader.read_next()
        if tbag0 is None: tbag0 = recv
        rel = (recv - tbag0) * 1e-9
        if t_end is not None and rel > t_end + 0.2:
            break

        if topic == args.acc_topic:
            m = deserialize_message(data, AccT)
            a3 = np.array([float(m.imu_state.accelerometer[i]) for i in range(3)])
            held_acc = a3
            if args.time_sync and rel >= args.start - PREROLL:
                tick_sec = int(m.tick) / 1000.0
                if last_fresh is None or np.any(a3 != last_fresh):   # (1) 표본화
                    last_fresh = a3
                    fresh_t.append(tick_sec); fresh_a.append(a3)
                if ref_stamp is not None:                            # (2) offset 표본
                    d_buf.append(ref_stamp - tick_sec)

        elif topic == args.imu_topic:
            m = deserialize_message(data, ImuT)
            hg = stamp_s(m, recv); ref_stamp = hg
            if rel < args.start or (t_end is not None and rel > t_end):
                continue
            av = m.angular_velocity
            gyros.append((hg, av.x, av.y, av.z))
            if not args.time_sync:
                if held_acc is None: gyros.pop(); continue
                held_rows.append(held_acc.copy())

        elif topic == args.cloud_topic:
            if rel < args.start or (t_end is not None and rel > t_end):
                continue
            m = deserialize_message(data, PcT); ts = stamp_s(m, recv)
            gen = pc2.read_points(m, field_names=('x','y','z'), skip_nans=True)
            arr = np.array([[p[0],p[1],p[2]] for p in gen], dtype=np.float32)
            if args.lidar_stride > 1 and arr.shape[0] > 0: arr = arr[::args.lidar_stride]
            lidar_frames.append((ts, arr))

    if not gyros:
        print('[!] 창 안에 자이로가 없습니다. --start/--dur 확인'); sys.exit(1)

    gy = np.array(gyros)                       # [hg, wx,wy,wz]
    hg = gy[:,0]

    # ---------- 가속도 결정 ----------
    if args.time_sync:
        if len(d_buf) < CALIB_MIN or len(fresh_t) < 2:
            print('[!] time_sync 표본 부족 -> held 로 대체'); accel = None
        else:
            offset = float(np.quantile(d_buf, OFF_Q))               # (2) offset 확정
            ft = np.array(fresh_t); fa = np.array(fresh_a)
            o = np.argsort(ft); ft = ft[o]; fa = fa[o]
            keep = np.concatenate([[True], np.diff(ft) > 0]); ft = ft[keep]; fa = fa[keep]
            q = hg - offset                                         # (3) 시각 환산 + 보간
            accel = np.column_stack([np.interp(q, ft, fa[:,0]),
                                     np.interp(q, ft, fa[:,1]),
                                     np.interp(q, ft, fa[:,2])])
            print('[time_sync] offset = %.6f  (d표본 %d, fresh %d)'
                  % (offset, len(d_buf), len(ft)))
    else:
        accel = np.array(held_rows)

    if args.time_sync and accel is None:       # fallback: held 재수집이 없으므로 경고만
        print('[!] time_sync 불가. --time-sync 빼고 다시 실행하세요.'); sys.exit(1)

    t0 = min(hg.min(), min(ts for ts,_ in lidar_frames) if lidar_frames else hg.min())

    with open(args.imu_out, 'w') as f:
        f.write('t,wx,wy,wz,ax_body,ay_body,az_body\n')
        for i in range(len(hg)):
            f.write('%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n'
                    % (hg[i]-t0, gy[i,1], gy[i,2], gy[i,3],
                       accel[i,0], accel[i,1], accel[i,2]))

    npts = 0
    with open(args.lidar_out, 'w') as f:
        f.write('t,x,y,z\n')
        for ts, arr in lidar_frames:
            if arr.shape[0] == 0: continue
            block = np.column_stack([np.full(arr.shape[0], ts-t0), arr.astype(np.float64)])
            np.savetxt(f, block, fmt=['%.9f','%.5f','%.5f','%.5f'], delimiter=',')
            npts += arr.shape[0]

    mode = 'time_sync(보간)' if args.time_sync else 'held(baseline)'
    print(f'완료 [{mode}]  IMU {len(hg)}행 -> {args.imu_out}')
    print(f'      LiDAR {len(lidar_frames)}프레임 / {npts}점 -> {args.lidar_out}')
    print('  ※ 가속도는 본체 원본. MATLAB 이 R_LB·스케일 적용.')


if __name__ == '__main__':
    main()
