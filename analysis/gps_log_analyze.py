#!/usr/bin/env python3
"""
gps_log_analyze.py — gnss_monitor.py가 남긴 CSV 로그 분석기 (표준 라이브러리만 사용)

목적:
  ① FIX vs NO-FIX 구간에서 위성 수(sat_total / sat_inuse)와 hdop이 어떻게 다른가
  ② FIX 구간 안에서 '정지' vs '이동'일 때 위성 수·hdop이 어떻게 다른가
  → "이동/하늘 차폐 때문에 fix가 안 됐는가"를 숫자로 가려내기 위함.

주의: 위경도는 fix일 때만 신뢰할 수 있으므로(NO-FIX면 좌표가 캐시값에 멈춤),
      속도(=이동 여부)는 두 점이 모두 FIX이고 시간 간격이 정상일 때만 계산한다.

사용:
    python3 gps_log_analyze.py my_log*.csv
    python3 gps_log_analyze.py my_log.csv my_log_1.csv --move-threshold 0.3 --out classified.csv
"""
import argparse
import csv
import glob
import math
import sys
from datetime import datetime
from statistics import mean, median


def summarize(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return dict(n=len(vals), mean=mean(vals), median=median(vals),
                mn=min(vals), mx=max(vals))


def fmt(s):
    if s is None:
        return "(해당 데이터 없음)"
    return (f"n={s['n']:<5d} 평균 {s['mean']:6.2f}  중앙 {s['median']:6.2f}  "
            f"범위 {s['mn']:.2f}~{s['mx']:.2f}")


def load(paths):
    files = []
    for p in paths:
        g = sorted(glob.glob(p))
        files.extend(g if g else [p])
    seen, rows = set(), []
    for f in files:
        if f in seen:
            continue
        seen.add(f)
        try:
            fh = open(f, newline='')
        except OSError as e:
            print(f"[경고] 파일 열기 실패: {f} ({e})")
            continue
        with fh:
            for d in csv.DictReader(fh):
                try:
                    t = datetime.fromisoformat(d['wall_time']).timestamp()
                    rows.append(dict(
                        t=t, file=f,
                        fixed=int(float(d['fixed'])),
                        total=int(float(d['sat_total'])),
                        inuse=int(float(d['sat_inuse'])),
                        hdop=float(d['hdop']),
                        lat=float(d['latitude']),
                        lon=float(d['longitude']),
                    ))
                except (KeyError, ValueError):
                    continue
    rows.sort(key=lambda r: r['t'])
    return sorted(seen), rows


def add_speed(rows, max_gap, max_speed):
    prev = None
    for r in rows:
        r['speed'] = None
        if prev is not None:
            dt = r['t'] - prev['t']
            if 0 < dt <= max_gap and r['fixed'] >= 1 and prev['fixed'] >= 1:
                dlat = (r['lat'] - prev['lat']) * 111320.0
                dlon = ((r['lon'] - prev['lon']) * 111320.0
                        * math.cos(math.radians(r['lat'])))
                sp = math.hypot(dlat, dlon) / dt
                # 로봇개엔 불가능한 값 = 좌표 튐(teleport) → 제외
                r['speed'] = sp if sp <= max_speed else None
        prev = r


def valid_hdop(h):
    # 0.0 = NO-FIX 표기값, >=90 = "쓸 기하 없음" 센티넬(예: 500.0) → 통계에서 제외
    return h is not None and 0.0 < h < 90.0


def hms(ts):
    return datetime.fromtimestamp(ts).strftime('%H:%M:%S')


def episodes(rows, max_gap):
    """연속된 같은 fix 상태 구간(에피소드)으로 묶는다. 시간 점프는 세션 경계로 취급."""
    eps, cur = [], None
    prev_t = None
    for r in rows:
        boundary = prev_t is not None and (r['t'] - prev_t) > max_gap
        state = 'FIX' if r['fixed'] >= 1 else 'NO-FIX'
        if cur is None or state != cur['state'] or boundary:
            if cur:
                eps.append(cur)
            cur = dict(state=state, t0=r['t'], t1=r['t'], n=1)
        else:
            cur['t1'] = r['t']
            cur['n'] += 1
        prev_t = r['t']
    if cur:
        eps.append(cur)
    return eps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+', help="CSV 파일 또는 글롭 (예: my_log*.csv)")
    ap.add_argument('--move-threshold', type=float, default=0.3,
                    help="이동으로 볼 속도 임계값 [m/s] (기본 0.3)")
    ap.add_argument('--max-gap', type=float, default=5.0,
                    help="이 간격[s]보다 크면 세션 경계로 보고 속도 미계산 (기본 5)")
    ap.add_argument('--max-speed', type=float, default=30.0,
                    help="이 속도[m/s] 초과는 좌표 튐으로 보고 제외 (기본 30)")
    ap.add_argument('--out', default=None, help="분류 결과를 저장할 CSV 경로(선택)")
    args = ap.parse_args()

    files, rows = load(args.paths)
    if not rows:
        print("읽을 수 있는 데이터가 없습니다. 경로/헤더를 확인하세요.")
        sys.exit(1)
    add_speed(rows, args.max_gap, args.max_speed)

    thr = args.move_threshold
    fix = [r for r in rows if r['fixed'] >= 1]
    nofix = [r for r in rows if r['fixed'] < 1]
    moving = [r for r in fix if r['speed'] is not None and r['speed'] > thr]
    stationary = [r for r in fix if r['speed'] is not None and r['speed'] <= thr]
    sentinel = [r for r in rows if r['hdop'] >= 90.0]

    span = rows[-1]['t'] - rows[0]['t']
    print("=" * 60)
    print(f"파일 {len(files)}개, 총 {len(rows)}행, "
          f"기록 구간 {hms(rows[0]['t'])} ~ {hms(rows[-1]['t'])} (약 {span/60:.1f}분)")
    print(f"FIX {len(fix)}행 ({100*len(fix)/len(rows):.0f}%) · "
          f"NO-FIX {len(nofix)}행 ({100*len(nofix)/len(rows):.0f}%)")
    print("=" * 60)

    print("\n[1] FIX vs NO-FIX  (하늘 차폐/기하 문제를 보는 핵심)")
    print("  ── NO-FIX 구간 ──")
    print(f"    위성 감지(total) : {fmt(summarize([r['total'] for r in nofix]))}")
    print(f"    위성 사용(inuse) : {fmt(summarize([r['inuse'] for r in nofix]))}")
    print("  ── FIX 구간 ──")
    print(f"    위성 감지(total) : {fmt(summarize([r['total'] for r in fix]))}")
    print(f"    위성 사용(inuse) : {fmt(summarize([r['inuse'] for r in fix]))}")
    print(f"    hdop(유효만)     : {fmt(summarize([r['hdop'] for r in fix if valid_hdop(r['hdop'])]))}")

    print("\n[2] FIX 구간 안에서  정지 vs 이동  (이동이 품질을 떨어뜨리는가)")
    print(f"    (임계값 {thr} m/s 기준 · 속도 계산된 FIX행 {len(moving)+len(stationary)}개)")
    print("  ── 정지(≤임계) ──")
    print(f"    위성 사용(inuse) : {fmt(summarize([r['inuse'] for r in stationary]))}")
    print(f"    위성 감지(total) : {fmt(summarize([r['total'] for r in stationary]))}")
    print(f"    hdop(유효만)     : {fmt(summarize([r['hdop'] for r in stationary if valid_hdop(r['hdop'])]))}")
    print("  ── 이동(>임계) ──")
    print(f"    위성 사용(inuse) : {fmt(summarize([r['inuse'] for r in moving]))}")
    print(f"    위성 감지(total) : {fmt(summarize([r['total'] for r in moving]))}")
    print(f"    hdop(유효만)     : {fmt(summarize([r['hdop'] for r in moving if valid_hdop(r['hdop'])]))}")
    print(f"    이동 속도        : {fmt(summarize([r['speed'] for r in moving]))}  [m/s]")

    print("\n[3] fix 상태 에피소드 (언제 얼마나 fix/무fix였나)")
    for e in episodes(rows, args.max_gap):
        print(f"    {e['state']:<6} {hms(e['t0'])}~{hms(e['t1'])} "
              f"({e['t1']-e['t0']:5.0f}s, {e['n']}행)")

    print("\n[4] 참고")
    n_hi = len([r for r in fix if valid_hdop(r['hdop']) and r['hdop'] >= 5.0])
    print(f"    fixed=1 이지만 hdop≥5 (품질 나쁨) : {n_hi}행")
    print(f"    hdop 센티넬(≥90, 예:500) 출현     : {len(sentinel)}행 "
          f"— fixed=1 이어도 품질 보장 아님")

    # 자동 관찰 한 줄
    ni_nf = summarize([r['inuse'] for r in nofix])
    ni_fx = summarize([r['inuse'] for r in fix])
    if ni_nf and ni_fx:
        print(f"\n  관찰: NO-FIX 구간 평균 inuse {ni_nf['mean']:.1f} vs "
              f"FIX 구간 평균 inuse {ni_fx['mean']:.1f} "
              f"→ inuse가 4~5 미만이면 fix가 안 잡히는 패턴인지 확인해 보세요.")

    if args.out:
        with open(args.out, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['wall_time', 'file', 'fixed', 'sat_total', 'sat_inuse',
                        'hdop', 'speed_mps', 'motion'])
            for r in rows:
                if r['fixed'] < 1:
                    motion = 'no-fix'
                elif r['speed'] is None:
                    motion = 'unknown'
                elif r['speed'] > thr:
                    motion = 'moving'
                else:
                    motion = 'stationary'
                sp = '' if r['speed'] is None else f"{r['speed']:.3f}"
                w.writerow([hms(r['t']), r['file'], r['fixed'], r['total'],
                            r['inuse'], f"{r['hdop']:.2f}", sp, motion])
        print(f"\n분류 결과 저장: {args.out}")


if __name__ == '__main__':
    main()
