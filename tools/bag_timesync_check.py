#!/usr/bin/env python3
# =============================================================================
# bag_timesync_check.py   (rev1 - 타임싱크 "측정" 전용)
# -----------------------------------------------------------------------------
# 목적 : rosbag2 에서 세 소스의 시계가 PC 수신시각 대비 어떻게 벌어지는지 측정.
#          L1 클럭  : /utlidar/imu (자이로), /utlidar/cloud (포인트 시각)
#          tick클럭 : /lowstate (본체 가속도)
#        -> 각 소스 offset(t) = 송신시각 - 수신시각
#        -> L1 vs tick 상대 오프셋 Δ(t) = offset_L1 - offset_tick
#        -> 드리프트(기울기, ppm) / 지터(std) / 주기 후보(FFT) 리포트.
#   ★ 이건 "고치는" 코드가 아니라 "재보는" 코드입니다. 성질부터 파악용.
#
# 실행 전 : unitree_go 메시지 있는 워크스페이스 source (/lowstate 읽기용)
#   source ~/fastlio_ws/install/setup.bash   # 예시
#   python3 bag_timesync_check.py ~/data/bags/xxx --start 0 --dur 90 --outdir ./ts_out
# =============================================================================
import argparse, os, glob, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ---------- bag 열기 ----------------------------------------------------------
def detect_storage(uri):
    if os.path.isdir(uri):
        if glob.glob(os.path.join(uri, '*.mcap')):  return 'mcap'
        if glob.glob(os.path.join(uri, '*.db3')):   return 'sqlite3'
    elif uri.endswith('.mcap'): return 'mcap'
    elif uri.endswith('.db3'):  return 'sqlite3'
    return 'sqlite3'


def open_reader(uri):
    so = rosbag2_py.StorageOptions(uri=uri, storage_id=detect_storage(uri))
    r = rosbag2_py.SequentialReader()
    r.open(so, rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return r


# ---------- 송신시각 뽑기 -----------------------------------------------------
def header_stamp_s(msg):
    """std_msgs/Header 있는 메시지: header.stamp 를 초로. 0이면 None."""
    s = msg.header.stamp
    if s.sec == 0 and s.nanosec == 0:
        return None
    return s.sec + s.nanosec * 1e-9


def tick_s(msg, tick_scale):
    """/lowstate: tick(기본 ms) 를 초로. tick 없으면 None."""
    if not hasattr(msg, 'tick'):
        return None
    return float(msg.tick) * tick_scale


# ---------- 통계 헬퍼 ---------------------------------------------------------
def line_fit(x, y):
    """y ~ slope*x + intercept. (slope[s/s], intercept, 잔차 std) 반환."""
    slope, intercept = np.polyfit(x, y, 1)
    resid = y - (slope * x + intercept)
    return slope, intercept, float(np.std(resid)), resid


def dominant_period(x, resid):
    """선형 제거한 잔차에서 FFT 최대 봉우리 -> (주기[s], 상대세기 0~1)."""
    n = len(resid)
    if n < 32:
        return None, None
    span = x[-1] - x[0]
    if span <= 0:
        return None, None
    fs = (n - 1) / span                     # 평균 샘플레이트
    xu = np.linspace(x[0], x[-1], n)        # 균일 리샘플
    ru = np.interp(xu, x, resid)
    ru = (ru - ru.mean()) * np.hanning(n)
    mag = np.abs(np.fft.rfft(ru))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    mag[0] = 0.0                            # DC 제거
    k = int(np.argmax(mag))
    if freqs[k] <= 0:
        return None, None
    strength = mag[k] / (mag.sum() + 1e-12)
    return 1.0 / freqs[k], float(strength)


def report_source(name, recv, sender):
    """한 소스 리포트 출력 + (recv0기준 x, offset) 반환."""
    recv = np.asarray(recv); sender = np.asarray(sender)
    x = recv - recv[0]
    offset = sender - recv
    slope, intercept, jit, resid = line_fit(x, offset)
    rate = (len(recv) - 1) / (x[-1] - x[0]) if x[-1] > x[0] else float('nan')
    per, str_ = dominant_period(x, resid)
    print(f'\n[{name}]  n={len(recv)}  rate≈{rate:6.1f} Hz')
    print(f'   평균 offset      : {np.mean(offset):+.6f} s')
    print(f'   드리프트(기울기) : {slope*1e6:+.2f} ppm   ({slope*1e3:+.4f} ms/s)')
    print(f'   지터(잔차 std)   : {jit*1e3:8.3f} ms')
    if per is not None and str_ > 0.02:
        print(f'   주기 후보        : {per:8.3f} s  (세기 {str_*100:4.1f}%)')
    else:
        print(f'   주기 후보        : 뚜렷한 주기 없음')
    return x, offset


# ---------- 메인 -------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--imu-topic',   default='/utlidar/imu')
    ap.add_argument('--acc-topic',   default='/lowstate')
    ap.add_argument('--cloud-topic', default='/utlidar/cloud')
    ap.add_argument('--start', type=float, default=0.0, help='bag 시작 후 몇 초부터')
    ap.add_argument('--dur',   type=float, default=None, help='몇 초 동안 (기본: 끝까지)')
    ap.add_argument('--tick-scale', type=float, default=1e-3,
                    help='/lowstate tick 단위->초 (기본 ms=1e-3)')
    ap.add_argument('--outdir', default='./ts_out', help='CSV/그림 저장 폴더')
    args = ap.parse_args()

    reader = open_reader(args.bag)
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {args.imu_topic, args.acc_topic, args.cloud_topic}
    missing = [t for t in want if t not in typemap]
    if missing:
        print('[!] bag 에 없는 토픽:', missing)
        print('    bag 안 토픽 목록:')
        for k, v in typemap.items():
            print(f'      {k}   ({v})')
        sys.exit(1)
    msgclass = {t: get_message(typemap[t]) for t in want}

    imu = {'recv': [], 'send': []}
    acc = {'recv': [], 'send': []}
    cld = {'recv': [], 'send': []}

    t0 = None
    win_lo = args.start
    win_hi = None if args.dur is None else args.start + args.dur

    while reader.has_next():
        topic, data, t_recv_ns = reader.read_next()
        if topic not in want:
            continue
        t_recv = t_recv_ns * 1e-9
        if t0 is None:
            t0 = t_recv
        rel = t_recv - t0
        if rel < win_lo:
            continue
        if win_hi is not None and rel > win_hi:
            if rel > win_hi + 5.0:      # 창 지나면 조기 종료
                break
            continue

        msg = deserialize_message(data, msgclass[topic])
        if topic == args.imu_topic:
            s = header_stamp_s(msg)
            if s is not None:
                imu['recv'].append(t_recv); imu['send'].append(s)
        elif topic == args.cloud_topic:
            s = header_stamp_s(msg)
            if s is not None:
                cld['recv'].append(t_recv); cld['send'].append(s)
        elif topic == args.acc_topic:
            s = tick_s(msg, args.tick_scale)
            if s is not None:
                acc['recv'].append(t_recv); acc['send'].append(s)

    for nm, d in (('imu/L1', imu), ('lowstate/tick', acc), ('cloud/L1', cld)):
        if len(d['recv']) < 10:
            print(f'[!] {nm}: 표본 부족(n={len(d["recv"])}). '
                  f'토픽 이름/타임스탬프(0?) 확인 필요.')

    print('=' * 60)
    print(' 타임싱크 측정 리포트')
    print('=' * 60)

    series = {}
    if len(imu['recv']) >= 10:
        series['imu'] = report_source('imu / L1클럭', imu['recv'], imu['send'])
    if len(acc['recv']) >= 10:
        series['acc'] = report_source('lowstate / tick클럭', acc['recv'], acc['send'])
    if len(cld['recv']) >= 10:
        series['cld'] = report_source('cloud / L1클럭', cld['recv'], cld['send'])

    # ---- 핵심: L1 vs tick 상대 오프셋 Δ(t) ----------------------------------
    delta = None
    if 'imu' in series and 'acc' in series:
        xi, oi = series['imu']
        xa, oa = series['acc']
        # tick offset 을 imu 시간축으로 보간 (공통 recv 기준이라 x끼리 정렬됨)
        oa_i = np.interp(xi, xa, oa)
        d = oi - oa_i
        d0 = d - d[0]                        # 첫값 0 기준(읽기 편하게)
        slope, intercept, jit, resid = line_fit(xi, d0)
        per, str_ = dominant_period(xi, resid)
        print('\n' + '-' * 60)
        print('[Δ(t) = L1 - tick  상대 오프셋]  ← 이게 진짜 보고 싶은 것')
        print('-' * 60)
        print(f'   상대 드리프트    : {slope*1e6:+.2f} ppm   ({slope*1e3:+.4f} ms/s)')
        print(f'   상대 지터        : {jit*1e3:8.3f} ms')
        if per is not None and str_ > 0.02:
            print(f'   주기 후보        : {per:8.3f} s  (세기 {str_*100:4.1f}%)')
        else:
            print(f'   주기 후보        : 뚜렷한 주기 없음')
        delta = (xi, d0)

    # ---- 저장 ----------------------------------------------------------------
    os.makedirs(args.outdir, exist_ok=True)
    for key, (x, off) in series.items():
        np.savetxt(os.path.join(args.outdir, f'offset_{key}.csv'),
                   np.column_stack([x, off]),
                   delimiter=',', header='t_rel_s,offset_s', comments='')
    if delta is not None:
        np.savetxt(os.path.join(args.outdir, 'delta_L1_minus_tick.csv'),
                   np.column_stack(delta),
                   delimiter=',', header='t_rel_s,delta_s', comments='')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for key, (x, off) in series.items():
            axs[0].plot(x, (off - off[0]) * 1e3, label=key, lw=0.8)
        axs[0].set_ylabel('offset - offset0  [ms]')
        axs[0].set_title('source offset vs recv (첫값 0 기준)')
        axs[0].legend(); axs[0].grid(alpha=0.3)
        if delta is not None:
            axs[1].plot(delta[0], delta[1] * 1e3, color='crimson', lw=0.8)
        axs[1].set_ylabel('Δ = L1 - tick  [ms]')
        axs[1].set_xlabel('t_rel [s]')
        axs[1].set_title('L1 vs tick 상대 오프셋')
        axs[1].grid(alpha=0.3)
        fig.tight_layout()
        p = os.path.join(args.outdir, 'timesync.png')
        fig.savefig(p, dpi=130)
        print(f'\n[그림] {p}')
    except Exception as e:
        print(f'\n[i] 그림 생략(matplotlib 없음/오류): {e}')

    print(f'\n[CSV] {args.outdir}/ 에 저장 완료.')
    print('\n해석 가이드:')
    print('  · 드리프트 ppm 이 0에 가까우면  -> 두 시계 속도 거의 같음(상수 오프셋).')
    print('  · ppm 이 크고 일정하면         -> 크리스탈 속도차. 선형보정으로 해결 가능.')
    print('  · 지터가 크면(수 ms↑)          -> 순간 정렬 흔들림. 동적구간 LIO 튐 원인.')
    print('  · 주기 후보가 뚜렷하면          -> 규칙적 흔들림(버퍼/재정렬 등) 의심.')


if __name__ == '__main__':
    main()
