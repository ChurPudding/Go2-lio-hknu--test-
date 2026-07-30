#!/usr/bin/env python3
"""
gnss_dropout_probe.py  --  /gnss 가 t=475.5 초에 멈춘 원인 추적

관찰된 사실
  /gnss       0 ~ 475.5 초  1 Hz 정상, 이후 468초 공백, t=944 에 무효 1개
  /gpt_state  3.4 ~ 466.6 초  (거의 같이 멈춤)
  /lowcmd, /frontvideostream, /lf/sportmodestate 는 944초까지 정상

가설
  (A) 신호 문제      -> 멈추기 전 위성 수·HDOP 가 서서히 나빠졌을 것
  (B) 프로세스 종료  -> 마지막까지 품질 정상, 뚝 끊김
  (C) DDS 발견 문제  -> 로봇은 계속 보냈는데 노트북이 못 받음 (bag 만으로는 구분 어려움)

사용
    python3 gnss_dropout_probe.py <bag_0.db3>
"""
import sys
import json
import struct
import sqlite3

WIN = (430.0, 520.0)      # 관심 구간 [초]


def topic_id(con, name):
    r = con.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    return r[0] if r else None


def parse_log(buf):
    """rcl_interfaces/msg/Log"""
    le = buf[1] == 1
    E = '<' if le else '>'
    p = [4]

    def al(n):
        o = (p[0] - 4) % n
        if o:
            p[0] += n - o

    def u32():
        al(4)
        v = struct.unpack_from(E + 'I', buf, p[0])[0]
        p[0] += 4
        return v

    def s():
        n = u32()
        v = buf[p[0]:p[0] + n - 1].decode('utf-8', 'replace')
        p[0] += n
        return v

    u32(); u32()                      # stamp
    lvl = buf[p[0]]; p[0] += 1        # level
    name = s()
    msg = s()
    return lvl, name, msg


def parse_string(buf):
    """std_msgs/msg/String"""
    n = struct.unpack_from('<I', buf, 4)[0]
    return buf[8:8 + n - 1].decode('utf-8', 'replace')


def main():
    db = sys.argv[1]
    con = sqlite3.connect(db)
    t0 = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    def T(ts):
        return (ts - t0) * 1e-9

    # ------------------------------------------------------------------
    print('=' * 66)
    print('1) /gnss 마지막 30개 — 품질이 서서히 나빠졌는가')
    print('=' * 66)
    tid = topic_id(con, '/gnss')
    rows = list(con.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)))
    print('%8s %6s %5s %5s   %s' % ('t[s]', 'hdop', '총위성', '사용', 'lat, lon'))
    for ts, blob in rows[-31:-1]:
        try:
            d = json.loads(blob[8:].decode('utf-8', 'replace').strip('\x00'))
        except Exception:
            continue
        print('%8.1f %6.2f %5d %5d   %.6f, %.6f'
              % (T(ts), d.get('hdop', -1), d.get('satellite_total', -1),
                 d.get('satellite_inuse', -1), d.get('latitude', 0), d.get('longitude', 0)))
    print()
    print('마지막(무효) 샘플: t=%.1f  %s'
          % (T(rows[-1][0]), rows[-1][1][8:].decode('utf-8', 'replace').strip('\x00')))

    # ------------------------------------------------------------------
    print()
    print('=' * 66)
    print('2) /rosout  %.0f ~ %.0f 초' % WIN)
    print('=' * 66)
    tid = topic_id(con, '/rosout')
    seen = 0
    for ts, blob in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        t = T(ts)
        if not (WIN[0] <= t <= WIN[1]):
            continue
        try:
            lvl, name, msg = parse_log(blob)
        except Exception:
            continue
        if 'topics discovery' in msg:     # 4700번 반복되는 잡음
            continue
        print('%8.1f  %-24s %s' % (t, name, msg[:80]))
        seen += 1
        if seen > 60:
            print('   ... 생략')
            break
    if seen == 0:
        print('  (해당 구간에 로그 없음 — discovery 에러 제외)')

    # ------------------------------------------------------------------
    print()
    print('=' * 66)
    print('3) 다른 String 토픽의 같은 구간 내용')
    print('=' * 66)
    for name in ['/multiplestate', '/lf/battery_alarm', '/gpt_state']:
        tid = topic_id(con, name)
        if tid is None:
            continue
        vals = []
        for ts, blob in con.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
            t = T(ts)
            if WIN[0] <= t <= WIN[1]:
                try:
                    vals.append((t, parse_string(blob)))
                except Exception:
                    pass
        print('--- %s  (구간 내 %d 개)' % (name, len(vals)))
        prev = None
        for t, v in vals:
            if v != prev:                 # 값이 바뀐 순간만
                print('   %8.1f  %s' % (t, v[:100]))
                prev = v
        if vals and prev == vals[0][1]:
            print('   (구간 내내 동일)')

    # ------------------------------------------------------------------
    print()
    print('=' * 66)
    print('4) 로봇 상태 — 그 무렵 멈췄거나 모드가 바뀌었는가')
    print('=' * 66)
    tid = topic_id(con, '/lf/sportmodestate')
    OFF_POS, OFF_VEL = 80, 96
    prev_mode = None
    n = 0
    for ts, blob in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        t = T(ts)
        if not (WIN[0] <= t <= WIN[1]):
            continue
        if len(blob) < 4 + 112:
            continue
        E = '<' if blob[1] == 1 else '>'
        mode = blob[4 + 65]
        gait = blob[4 + 72]
        vel = struct.unpack_from(E + '3f', blob, 4 + OFF_VEL)
        sp = (vel[0] ** 2 + vel[1] ** 2) ** 0.5
        if (mode, gait) != prev_mode:
            print('   %8.1f  mode=%d gait=%d  속력 %.2f m/s' % (t, mode, gait, sp))
            prev_mode = (mode, gait)
        n += 1
    print('   (구간 내 %d 개, mode/gait 변화만 표시)' % n)


if __name__ == '__main__':
    main()
