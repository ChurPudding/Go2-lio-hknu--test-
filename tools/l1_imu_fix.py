#!/usr/bin/env python3
"""
l1_imu_fix.py  --  Go2 L1 IMU 보정 브리지  (2026-08-26 rev, time_sync 추가)

배경
----
/utlidar/imu 는 자이로와 가속도계가 서로 다른 프레임에 있다.
  - 자이로     : L1 포인트클라우드 좌표계와 평행 (검증: 중력방향 1.87deg 이내)
  - 가속도계   : 실제 로봇 가속도와 상관 <=0.19 (본체 IMU는 0.83) -> 사용 불가
이 노드는 자이로(L1) + 가속도(본체, /lowstate)를 합쳐 /l1_imu_fixed 를 만든다.

이번 개정(time_sync) — 왜 넣었는가
---------------------------------
실측(3 bag)으로 확인: 본체 /lowstate 는 500Hz 로 발행되지만 가속도계의
'실효 갱신율'은 약 200Hz 다 (자이로는 정상 500Hz). 즉 accel 값이 한 번
갱신되는 동안 메시지가 2~3개 나오고, 그 사이는 이전 값이 그대로 유지(held)된다.
  -> 최신 accel 을 그대로 붙이면 최대 ~5ms 만큼 낡은 값을 쓰게 된다.
  -> 동적 구간에서 이 차이가 baseline 대비 중앙 0.8~1.5, 90분위 4~6 m/s^2.

time_sync=True 는 이 문제를 고친다. 세 가지를 한다.
  (1) 200Hz 표본화 : accel 이 실제로 바뀐 순간(Δa≠0)만 골라 진짜 표본으로 쓴다.
                     (held 중복은 버린다. held 를 보간에 넣으면 계단이 되살아난다.)
  (2) tick -> L1 시계 정렬 : /lowstate 는 header 가 없지만 tick(1kHz ms 카운터)이
                     측정 시각을 담는다. offset 을 구해 L1 자이로 시계로 옮긴다.
                       t_L(tick) = tick/1000 + offset
                     offset 은 /utlidar/imu header 를 기준으로,
                       d = (최신 자이로 header) - tick/1000
                     의 '상단분위'로 잡는다. (최신 자이로는 항상 과거라 d 는
                     참값보다 낮게 깔린다 -> 중앙값은 ~2ms 편향. 상단분위로 제거.)
                     드리프트 방지를 위해 최근 창에서 계속 갱신한다.
  (3) gyro 시각 보간 : 자이로 header 시각 h_g 에 맞춰 200Hz 표본을 선형보간.
                       q = h_g - offset  (tick 축으로 환산)
                       w = (q - t0)/(t1 - t0)
                       a(h_g) = (1-w)*a0 + w*a1
                     자이로를 감쌀 '다음 표본'이 아직 안 왔으면 잠깐 큐에 물렸다가
                     표본이 도착하면 보간해 발행한다(발행이 최대 ~5ms 지연될 수 있음).

출력 형식/토픽/프레임/extrinsic 은 baseline 과 동일하다. accel 값만 바뀐다.
따라서 Point-LIO config 는 손대지 않는다 (extrinsic_R 단위행렬 유지).

사용
----
  baseline(현재):  python3 l1_imu_fix.py --ros-args -p time_sync:=false
  재설계:          python3 l1_imu_fix.py --ros-args -p time_sync:=true
"""
import os
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, HistoryPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState, SportModeState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import R_LB, ACC_SCALE_BODY, EXPECTED_REST_ACC, LEVER  # noqa: E402


class L1ImuFix(Node):
    # 정지 구간 자동 검증
    REST_CHECK_N = 750          # 250Hz * 3초
    REST_TOL = 0.60             # m/s^2

    # time_sync 튜닝 상수
    CALIB_MIN = 200             # offset 확정에 필요한 최소 표본 (약 0.4초)
    OFF_Q = 0.90                # offset 상단분위 (lag 편향 제거용)
    OFF_REFRESH = 100           # 몇 개마다 offset 재계산 (드리프트 추적)
    FRESH_MAXLEN = 400          # 200Hz 표본 버퍼 (약 2초)
    DBUF_MAXLEN = 1500          # offset 추정 표본 창 (약 3초)
    PENDING_CAP = 64            # 대기 자이로 상한 (초과 시 hold 로 강제 방출)

    def __init__(self):
        super().__init__('l1_imu_fix')
        self.declare_parameter('out_topic', '/l1_imu_fixed')
        self.declare_parameter('frame_id', 'utlidar_lidar')
        self.declare_parameter('acc_scale', float(ACC_SCALE_BODY))
        self.declare_parameter('rest_check', True)
        self.declare_parameter('acc_topic', '/lowstate')
        # 시각 동기: False=baseline(최신 held), True=200Hz 표본 보간
        self.declare_parameter('time_sync', False)

        self.frame_id = self.get_parameter('frame_id').value
        self.acc_scale = float(self.get_parameter('acc_scale').value)
        self.rest_check = bool(self.get_parameter('rest_check').value)
        self.time_sync = bool(self.get_parameter('time_sync').value)

        self.acc_body = None          # 최신 본체 accel (baseline + fallback)
        self.n_imu = 0
        self.n_low = 0
        self._rest_buf = []
        self._rest_done = False

        # --- 레버암 보정 상태 (원심 + 접선) ---
        self.lever_l = R_LB @ LEVER   # 몸체→라이다 프레임 (상수, 1회 계산)
        self.prev_omega = None        # 접선 항용: 직전 각속도
        self.prev_t = None            # 직전 시각 [s]
        self.alpha_filt = np.zeros(3) # 각가속도 EMA (미분 노이즈 억제)
        self.ALPHA_BETA = 0.75        # 각가속도 저역통과 계수 (클수록 부드러움)

        # --- time_sync 상태 ---
        self.ref_stamp = None                       # 최신 /utlidar/imu header [s]
        self.last_fresh_acc = None                  # 마지막 fresh accel (중복 판정)
        self.fresh = deque(maxlen=self.FRESH_MAXLEN)  # (tick_sec, accel[3])
        self.pending = deque()                      # (sec, nsec, (wx,wy,wz))
        self._dbuf = deque(maxlen=self.DBUF_MAXLEN)   # d = ref_stamp - tick_sec
        self.offset = None
        self._noff = 0

        pub_qos = QoSProfile(
            depth=200, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        out_topic = self.get_parameter('out_topic').value
        self.pub = self.create_publisher(Imu, out_topic, pub_qos)

        self.acc_topic = self.get_parameter('acc_topic').value
        acc_cls = SportModeState if 'sportmode' in self.acc_topic else LowState
        self.create_subscription(
            acc_cls, self.acc_topic, self.on_lowstate, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/utlidar/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        log = self.get_logger()
        log.info('l1_imu_fix started -> %s (RELIABLE, depth=200)' % out_topic)
        log.info('R_LB[0,0] = %+.6f   (기대값 +0.523029)' % R_LB[0, 0])
        log.info('acc_scale = %.5f' % self.acc_scale)
        log.info('가속도 출처 = %s' % self.acc_topic)
        log.info('time_sync = %s  (%s)' % (
            self.time_sync,
            '200Hz 표본 보간' if self.time_sync else 'baseline: 최신 held'))
        log.info('정지 시 기대 가속도 = (%+.2f, %+.2f, %+.2f)'
                 % tuple(EXPECTED_REST_ACC))
        if abs(R_LB[0, 0] - 0.523029) > 1e-6:
            log.error('R_LB 이 확정본과 다르다. go2_calib.py 를 확인할 것.')
        if self.time_sync and 'sportmode' in self.acc_topic:
            log.warn('time_sync 는 tick 이 있는 /lowstate 에서만 동작한다. '
                     'sportmodestate 로는 baseline 만 유효.')

    # -----------------------------------------------------------
    def on_lowstate(self, msg):
        a = np.array([msg.imu_state.accelerometer[0],
                      msg.imu_state.accelerometer[1],
                      msg.imu_state.accelerometer[2]], dtype=float)
        self.acc_body = a
        self.n_low += 1

        if not self.time_sync:
            return

        # (1) 200Hz 표본화: accel 이 바뀐 순간만 버퍼에 (held 중복 제거)
        tick_sec = int(msg.tick) / 1000.0
        if self.last_fresh_acc is None or np.any(a != self.last_fresh_acc):
            self.last_fresh_acc = a
            self.fresh.append((tick_sec, a))

        # (2) offset 추정: d = 최신 자이로 header - tick_sec, 상단분위로 잡는다
        if self.ref_stamp is not None:
            self._dbuf.append(self.ref_stamp - tick_sec)
            self._noff += 1
            if self.offset is None:
                if len(self._dbuf) >= self.CALIB_MIN:
                    self.offset = float(np.quantile(self._dbuf, self.OFF_Q))
                    self.get_logger().info(
                        '[time_sync] offset 확정 = %.6f  (표본 %d개)'
                        % (self.offset, len(self._dbuf)))
            elif self._noff % self.OFF_REFRESH == 0:
                self.offset = float(np.quantile(self._dbuf, self.OFF_Q))

        # (3) 새 표본이 생겼으니 대기 중인 자이로를 보간해 방출 시도
        self._flush()

    def on_imu(self, msg):
        if not self.time_sync:
            # ---- baseline (07-30 그대로): 최신 held accel 을 붙여 즉시 발행 ----
            if self.acc_body is None:
                return
            s = msg.header.stamp
            av = msg.angular_velocity
            self._publish(s.sec, s.nanosec, av.x, av.y, av.z, self.acc_body)
            return

        # ---- redesign: 기준 시각 갱신 + 자이로를 대기열에 넣고 보간 시도 ----
        self.ref_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        av = msg.angular_velocity
        self.pending.append((msg.header.stamp.sec, msg.header.stamp.nanosec,
                             (av.x, av.y, av.z)))
        self._flush()
        # 안전장치: accel 스트림이 멎어 대기열이 넘치면 hold 로 강제 방출
        if self.offset is not None and len(self.pending) > self.PENDING_CAP:
            sec, ns, w = self.pending.popleft()
            a = self.fresh[-1][1] if self.fresh else self.acc_body
            if a is not None:
                self._publish(sec, ns, w[0], w[1], w[2], a)

    # -----------------------------------------------------------
    def _flush(self):
        """대기 자이로 중 보간 가능한 것을 발행한다.

        pending 은 자이로 도착 순(≈ header 오름차순)이라, 맨 앞이 아직
        준비 안 됐으면 뒤도 마찬가지이므로 break 한다.
        """
        if self.offset is None or not self.fresh:
            return
        fr = list(self.fresh)          # O(1) 인덱싱용 스냅샷 (~수백 개)
        newest = fr[-1][0]
        while self.pending:
            sec, ns, w = self.pending[0]
            hg = sec + ns * 1e-9
            q = hg - self.offset       # 자이로 시각을 tick 축으로 환산
            if q > newest:
                break                  # 감쌀 다음 표본이 아직 없음 -> 대기
            self.pending.popleft()
            a = self._interp(fr, q)
            self._publish(sec, ns, w[0], w[1], w[2], a)

    @staticmethod
    def _interp(fr, q):
        """fr=[(t,accel)...] 오름차순에서 q 시각의 accel 을 선형보간."""
        if q <= fr[0][0]:
            return fr[0][1]
        if q >= fr[-1][0]:
            return fr[-1][1]
        # q 는 대개 최신 근처라 뒤에서부터 탐색 (몇 번이면 찾음)
        for i in range(len(fr) - 1, 0, -1):
            t1, a1 = fr[i]
            t0, a0 = fr[i - 1]
            if t0 <= q <= t1:
                if t1 == t0:
                    return a1
                w = (q - t0) / (t1 - t0)
                return a0 + (a1 - a0) * w
        return fr[-1][1]

    def _publish(self, sec, nsec, wx, wy, wz, acc_body):
        acc_lidar = R_LB @ (np.asarray(acc_body, dtype=float) * self.acc_scale)

        # 레버암 보정: 라이다가 몸체 회전축에서 self.lever_l 만큼 떨어져 있어
        # 몸통 가속도에 두 항이 더해진다.  a_lidar = a_body + ω×(ω×r) + α×r
        omega = np.array([wx, wy, wz], dtype=float)          # 라이다 프레임 각속도

        # (1) 원심 항 ω×(ω×r) : 일정한 회전에서 큼
        a_centri = np.cross(omega, np.cross(omega, self.lever_l))
        acc_lidar = acc_lidar + a_centri

        # (2) 접선 항 α×r : 각가속도 α = dω/dt, 급격한 회전 변화에서 큼.
        #     미분은 노이즈를 키우므로 EMA 저역통과로 완화한다.
        a_tangent = np.zeros(3)
        t_now = sec + nsec * 1e-9
        if self.prev_omega is not None and self.prev_t is not None:
            dt = t_now - self.prev_t
            if 1e-4 < dt < 0.05:                             # dt 이상치 방어
                alpha_raw = (omega - self.prev_omega) / dt
                self.alpha_filt = (self.ALPHA_BETA * self.alpha_filt
                                   + (1.0 - self.ALPHA_BETA) * alpha_raw)
                a_tangent = np.cross(self.alpha_filt, self.lever_l)
                acc_lidar = acc_lidar + a_tangent
        self.prev_omega = omega
        self.prev_t = t_now

        # (임시) 두 항 크기 확인 — 검증 끝나면 지워도 됨
        if self.n_imu % 200 == 0:
            self.get_logger().info('a_centri=%.3f  a_tangent=%.3f m/s^2'
                % (np.linalg.norm(a_centri), np.linalg.norm(a_tangent)))

        out = Imu()
        out.header.stamp.sec = int(sec)
        out.header.stamp.nanosec = int(nsec)
        out.header.frame_id = self.frame_id
        out.angular_velocity.x = float(wx)
        out.angular_velocity.y = float(wy)
        out.angular_velocity.z = float(wz)
        out.linear_acceleration.x = float(acc_lidar[0])
        out.linear_acceleration.y = float(acc_lidar[1])
        out.linear_acceleration.z = float(acc_lidar[2])
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)
        self.n_imu += 1
        if self.rest_check and not self._rest_done:
            self._rest_buf.append(acc_lidar)
            if len(self._rest_buf) >= self.REST_CHECK_N:
                self._verify_rest()

    # -----------------------------------------------------------
    def _verify_rest(self):
        """초반 정지 구간 가속도 평균을 기대값과 비교한다."""
        self._rest_done = True
        mean = np.mean(np.asarray(self._rest_buf), axis=0)
        err = mean - EXPECTED_REST_ACC
        log = self.get_logger()
        log.info('[정지검증] 실측 평균 = (%+.2f, %+.2f, %+.2f)' % tuple(mean))
        log.info('[정지검증] 기대값   = (%+.2f, %+.2f, %+.2f)'
                 % tuple(EXPECTED_REST_ACC))
        log.info('[정지검증] 차이     = (%+.2f, %+.2f, %+.2f)' % tuple(err))
        if mean[2] > 0:
            log.error('[정지검증] 실패: z 가 양수다. R_LB 회전이 적용되지 '
                      '않았거나 방향이 반대다. 실험을 중단할 것.')
        elif np.max(np.abs(err)) > self.REST_TOL:
            log.warn('[정지검증] 주의: 성분별 오차가 %.2f m/s^2 를 넘는다. '
                     '초반 3초가 정지 구간이 아니었을 수 있다.' % self.REST_TOL)
        else:
            log.info('[정지검증] 통과. 진행해도 좋다.')

    def report(self):
        log = self.get_logger()
        if self.time_sync:
            off = ('%.3f' % self.offset) if self.offset is not None else '대기중'
            log.info('imu_in(대기 %d) lowstate_in=%d published=%d  '
                     'fresh=%d offset=%s'
                     % (len(self.pending), self.n_low, self.n_imu,
                        len(self.fresh), off))
        else:
            log.info('imu_in lowstate_in=%d published=%d'
                     % (self.n_low, self.n_imu))
        if self.n_low == 0:
            log.warn('/lowstate 미수신 -> 발행 0. bag 재생 확인.')


def main():
    rclpy.init()
    node = L1ImuFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
