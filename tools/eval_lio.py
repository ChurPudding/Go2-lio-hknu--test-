#!/usr/bin/env python3
"""
eval_lio.py -- LIO 결과를 robot_odom 기준선과 비교

사용법:
    python3 eval_lio.py <원본bag.db3> <결과.csv> [결과2.csv ...]

원본 bag 에서 /utlidar/robot_odom 을 참값으로 읽고,
dump_odom.py 로 뽑은 결과 CSV 들을 비교한다.
"""
import sys
import csv
import sqlite3
import struct

import numpy as np


# ---------- CDR 파서 ----------
class CDR:
    def __init__(self, buf):
        self.b = buf
        self.le = buf[1] == 1
        self.p = 4

    def _al(self, n):
        o = (self.p - 4) % n
        if o:
            self.p += n - o

    def u32(self):
        self._al(4)
        v = struct.unpack_from('<I' if self.le else '>I', self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        self._al(4)
        v = struct.unpack_from('<i' if self.le else '>i', self.b, self.p)[0]
        self.p += 4
        return v

    def f64n(self, n):
        self._al(8)
        v = struct.unpack_from(('<' if self.le else '>') + str(n) + 'd',
                               self.b, self.p)
        self.p += 8 * n
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace')
        self.p += n
        return s


def parse_odom(buf):
    c = CDR(buf)
    sec = c.i32()
    nsec = c.u32()
    c.string()
    c.string()
    pos = c.f64n(3)
    quat = c.f64n(4)
    return sec + nsec * 1e-9, pos, quat


def load_ref(db):
    con = sqlite3.connect(db)
    row = con.execute("SELECT id FROM topics WHERE name='/utlidar/robot_odom'").fetchone()
    if row is None:
        sys.exit('원본 bag 에 /utlidar/robot_odom 이 없습니다.')
    t, P, Q = [], [], []
    for (blob,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
        tt, p, q = parse_odom(blob)
        t.append(tt)
        P.append(p)
        Q.append(q)
    con.close()
    return np.array(t), np.array(P), np.array(Q)


def load_csv(path):
    t, P = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r['t']))
            P.append([float(r['x']), float(r['y']), float(r['z'])])
    if not t:
        sys.exit('%s 가 비어 있습니다. 녹화가 실패했는지 확인하세요.' % path)
    t = np.array(t)
    P = np.array(P)
    o = np.argsort(t)
    return t[o], P[o]


def yaw_of(Q):
    x, y, z, w = Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3]
    return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def find_turns(t, yaw, thr=0.35, min_dur=0.8):
    """odom yaw 로부터 회전 구간 자동 검출"""
    r = np.gradient(yaw, t)
    k = max(3, int(0.5 / np.median(np.diff(t))))
    r = np.convolve(r, np.ones(k) / k, mode='same')
    idx = np.where(np.abs(r) > thr)[0]
    segs = []
    if len(idx):
        s = p = idx[0]
        for i in idx[1:]:
            if i - p > k:
                segs.append((s, p))
                s = i
            p = i
        segs.append((s, p))
    return [(t[a], t[b]) for a, b in segs if t[b] - t[a] > min_dur]


def path_len(P):
    return float(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum())


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    db, csvs = sys.argv[1], sys.argv[2:]

    rt, rP, rQ = load_ref(db)
    t0 = rt[0]
    rt = rt - t0
    ryaw = np.degrees(yaw_of(rQ))
    turns = find_turns(rt, np.radians(ryaw))

    # 공통 15 Hz 격자
    g = np.arange(rt[0] + 0.5, rt[-1] - 0.5, 1 / 15.0)
    G = np.stack([np.interp(g, rt, rP[:, k]) for k in range(3)], 1)

    print('=' * 74)
    print('기준선 robot_odom : %d msgs, %.1f s' % (len(rt), rt[-1] - rt[0]))
    print('  수평 총거리 %.2f m | 루프클로저 %.3f m | z p-p %.1f mm | 최종 z %+.1f mm'
          % (path_len(G), np.linalg.norm(G[-1, :2] - G[0, :2]),
             1000 * (G[:, 2].max() - G[:, 2].min()),
             1000 * (G[-1, 2] - G[0, 2])))
    print('  검출된 회전 구간 %d개' % len(turns))
    print('=' * 74)

    for path in csvs:
        ct, cP = load_csv(path)
        ct = ct - t0
        if ct[-1] < g[0] or ct[0] > g[-1]:
            print('\n[%s] 시간 범위 불일치 - 원본 bag 과 짝이 맞는지 확인' % path)
            continue
        F = np.stack([np.interp(g, ct, cP[:, k]) for k in range(3)], 1)

        gz = G[:, 2] - G[0, 2]
        fz = F[:, 2] - F[0, 2]
        e = fz - gz

        print('\n[%s]  %d rows, %.0f Hz' % (path, len(ct), len(ct) / (ct[-1] - ct[0])))
        print('  수평 총거리   %8.2f m   (참값 %.2f, %+.0f%%)'
              % (path_len(F), path_len(G), 100 * (path_len(F) / path_len(G) - 1)))
        print('  루프 클로저   %8.3f m   (참값 %.3f)'
              % (np.linalg.norm(F[-1, :2] - F[0, :2]),
                 np.linalg.norm(G[-1, :2] - G[0, :2])))
        print('  z RMSE        %8.1f mm' % (1000 * np.sqrt((e ** 2).mean())))
        print('  z 최대오차    %8.1f mm' % (1000 * np.abs(e).max()))
        print('  최종 z 오차   %+8.1f mm   (참값 %+.1f)'
              % (1000 * e[-1], 1000 * (G[-1, 2] - G[0, 2])))

        if turns:
            print('  회전 구간별 z 변화 (참값 / 결과 / 차이, mm)')
            for i, (a, b) in enumerate(turns):
                dg = (np.interp(b, rt, rP[:, 2]) - np.interp(a, rt, rP[:, 2])) * 1000
                df = (np.interp(b, ct, cP[:, 2]) - np.interp(a, ct, cP[:, 2])) * 1000
                flag = '  <<<' if abs(df - dg) > 30 else ''
                print('    T%-2d %6.2f~%6.2f s  %+7.1f / %+7.1f / %+7.1f%s'
                      % (i, a, b, dg, df, df - dg, flag))

        # 수평 궤적 정렬 오차
        gx, gy = G[:, 0] - G[0, 0], G[:, 1] - G[0, 1]
        px, py = F[:, 0] - F[0, 0], F[:, 1] - F[0, 1]
        a0 = np.arctan2((gx * py - gy * px).sum(), (gx * px + gy * py).sum())
        rx = np.cos(a0) * gx - np.sin(a0) * gy
        ry = np.sin(a0) * gx + np.cos(a0) * gy
        err = np.hypot(px - rx, py - ry)
        print('  수평 위치오차 (yaw %+.1f° 정렬 후) RMS %.3f m, 최대 %.3f m'
              % (np.degrees(a0), err.mean(), err.max()))


if __name__ == '__main__':
    main()
