#!/usr/bin/env python3
"""
zupt_filter_yaw.py  --  정지 중 LIO 위치 + yaw(자세) 표류를 막는다 (ZUPT)

    python3 zupt_filter_yaw.py
    python3 zupt_filter_yaw.py --ros-args -p in_topic:=/indoor/base_pose \
                                          -p out_topic:=/indoor/base_pose_zupt
    # yaw hold 를 꺼서 원본(위치만)과 동일하게 — baseline 비교용
    python3 zupt_filter_yaw.py --ros-args -p hold_yaw:=false

원본 zupt_filter.py 에서 바뀐 곳은 세 군데뿐이다(+토글 파라미터 hold_yaw).
  [1] 정지 진입 시 : 위치와 함께 자세(quaternion) 도 저장
  [2] 발행 시      : 정지 중 자세도 진입값으로 고정 + 각속도 0
  [3] release/init : hold_quat 초기화
나머지 로직(정지 판정·재사용·안전 해제)은 원본과 동일하다.

─────────────────────────────────────────────────────────────────────
왜 자세도 붙잡나 (원본과 달라진 전제)
─────────────────────────────────────────────────────────────────────
원본은 "자세는 라이다가 잘 잡고, 회전은 정지 판정에서 걸러진다"는 전제로
위치만 고정했다. 그러나 원인 규명 결과, **정지 중 로봇은 안 도는데(몸통
자이로 정지 중 회전 −2.2~−2.8°) Point-LIO 만 yaw 를 −206° 흘리는** 현상이
run1·run2 두 bag 에서 확인됐다. 기하 퇴화 위치에서 yaw 가 관측 불가라
추정기가 자유낙하하는 것이다. 그래서 정지 중에는 자세도 붙잡아야 한다.

붙잡는 판정 자체는 여전히 독립 센서(다리 오도메트리 + /l1_imu_fixed 자이로)로
하므로, LIO 가 −8°/s 로 헛돌아도 자이로는 0 에 가까워 정지를 제대로 잡는다.

─────────────────────────────────────────────────────────────────────
왜 필요한가 (위치)
─────────────────────────────────────────────────────────────────────
L1 라이다는 164.9° 아래로 기울어 달려 있어 스캔의 **93~97% 가 지면**이다
(2026-08-03 실측, 복도 정지 구간). 평면 하나로는 그 위에서 미끄러지는
수평 위치와 방향을 제약할 수 없다.

그래서 로봇이 **서 있는 동안** 위치가 흐른다. 실측:

    정지 12초 동안 표류   1.171 m  (3회 편차 0.006 m — 계통 오차)
    한 바퀴 돌고 온 루프  0.133 m

표류가 누적되지는 않는다. 걷기 시작하면 라이다가 위치를 되찾는다.
**서 있는 동안만 문제**이므로 그때만 붙잡아 주면 된다.

─────────────────────────────────────────────────────────────────────
ZUPT 란
─────────────────────────────────────────────────────────────────────
Zero-velocity UPdaTe. **"안 움직인다는 사실도 정보다"** 라는 발상이다.
여기서는 Point-LIO 내부를 고칠 수 없으므로 **출력단에서 붙잡는** 방식으로
흉내낸다.

─────────────────────────────────────────────────────────────────────
정지를 어떻게 아는가
─────────────────────────────────────────────────────────────────────
`/utlidar/robot_odom`(다리 운동학)과 자이로를 함께 본다.

    /utlidar/robot_odom   2초에 0.01 m  (0.005 m/s)
    LIO                   2초에 0.20~0.32 m
    자이로                0.009 rad/s   (걸을 때 2.5 rad/s)

30 배, 280 배 차이라 헷갈릴 여지가 없다.

─────────────────────────────────────────────────────────────────────
안전 원칙 — 모르면 붙잡지 않는다
─────────────────────────────────────────────────────────────────────
    · robot_odom 이 timeout 초 이상 안 옴
    · 붙잡은 지 max_hold 초 초과
    · 붙잡은 뒤 LIO 가 max_lio_move 이상 움직였다고 보고
    · 자이로가 크다 (제자리 걸음·회전)

─────────────────────────────────────────────────────────────────────
토픽
─────────────────────────────────────────────────────────────────────
받는 것
    /indoor/base_pose      LIO 위치 (robot_pose.py)
    /utlidar/robot_odom    다리 오도메트리 — 정지 판정 기준
    /l1_imu_fixed          보정된 IMU — 자이로로 회전 판정

내는 것
    /indoor/base_pose_zupt   nav_msgs/Odometry   붙잡기가 적용된 위치+자세
    /indoor/zupt_info        std_msgs/String     상태 JSON

covariance 는 입력 것을 그대로 전달한다. 신뢰도 판정은 lio_health 몫이다.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ZuptFilter(Node):
    def __init__(self):
        super().__init__('zupt_filter')

        self.declare_parameter('in_topic', '/indoor/base_pose')
        self.declare_parameter('ref_topic', '/utlidar/robot_odom')
        self.declare_parameter('imu_topic', '/l1_imu_fixed')
        self.declare_parameter('out_topic', '/indoor/base_pose_zupt')

        # 정지 판정 — 실측의 5~15 배 여유를 뒀다
        self.declare_parameter('still_speed', 0.05)    # [m/s]  실측 0.005
        self.declare_parameter('still_gyro', 0.15)     # [rad/s] 실측 0.009
        self.declare_parameter('still_window', 1.0)    # 이 구간을 봐서 판정 [s]

        # 안전
        self.declare_parameter('timeout', 0.5)         # 기준 신호 끊김 [s]
        self.declare_parameter('max_hold', 60.0)       # 최대 붙잡는 시간 [s]
        self.declare_parameter('max_lio_move', 2.0)    # 붙잡은 뒤 LIO 가 이만큼 움직이면 해제 [m]

        # [yaw hold] 정지 중 자세(yaw)도 고정할지. 이 파일의 핵심 기능.
        self.declare_parameter('hold_yaw', True)       # false 면 원본(위치만)과 동일

        g = lambda n: self.get_parameter(n).value
        self.v_th = float(g('still_speed'))
        self.w_th = float(g('still_gyro'))
        self.win = float(g('still_window'))
        self.timeout = float(g('timeout'))
        self.max_hold = float(g('max_hold'))
        self.max_move = float(g('max_lio_move'))
        self.hold_yaw_en = bool(g('hold_yaw'))

        # 상태
        self.ref = []            # (t, x, y) 최근 구간
        self.gyro = 0.0
        self.t_ref = None
        self.t_imu = None
        self.hold = None         # 붙잡은 위치 (x, y, z) 또는 None
        self.hold_lio = None     # 붙잡을 때의 LIO 위치
        self.t_hold = None
        self.n_in = self.n_hold = 0
        self.saved = 0.0         # 붙잡아서 막은 누적 표류 [m]
        self.last_anchor = None  # 마지막 기준점 (짧은 해제 뒤 재사용)
        self.ref_at_anchor = None  # 그때의 robot_odom 위치
        self.hold_quat = None    # [yaw] 붙잡은 자세 (x, y, z, w) 또는 None
        self.last_quat = None    # [yaw] 마지막 자세 기준 (짧은 해제 뒤 재사용)

        self.pub = self.create_publisher(Odometry, g('out_topic'), 10)
        self.pub_info = self.create_publisher(String, '/indoor/zupt_info', 10)
        self.create_subscription(Odometry, g('in_topic'), self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, g('ref_topic'), self.on_ref,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu,
                                 qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        self.get_logger().info('zupt_filter_yaw: %s -> %s' % (g('in_topic'), g('out_topic')))
        self.get_logger().info('  정지 판정: 속도 < %.3f m/s  그리고  자이로 < %.3f rad/s'
                               % (self.v_th, self.w_th))
        self.get_logger().info('  yaw hold : %s'
                               % ('ON' if self.hold_yaw_en else 'OFF (원본과 동일 — 위치만)'))

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ref(self, m):
        t = self.now()
        self.t_ref = t
        p = m.pose.pose.position
        self.ref.append((t, p.x, p.y))
        while self.ref and t - self.ref[0][0] > self.win + 0.5:
            self.ref.pop(0)

    def on_imu(self, m):
        self.t_imu = self.now()
        w = m.angular_velocity
        self.gyro = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)

    # ------------------------------------------------------------------
    def is_still(self):
        """(정지인가, 사유)."""
        t = self.now()

        if self.t_ref is None or t - self.t_ref > self.timeout:
            return False, 'ref_timeout'          # 모르면 붙잡지 않는다
        if self.t_imu is None or t - self.t_imu > self.timeout:
            return False, 'imu_timeout'

        sel = [r for r in self.ref if r[0] >= t - self.win]
        if len(sel) < 5 or sel[-1][0] - sel[0][0] < self.win * 0.6:
            return False, 'ref_부족'

        dt = sel[-1][0] - sel[0][0]
        v = math.hypot(sel[-1][1] - sel[0][1], sel[-1][2] - sel[0][2]) / dt
        if v > self.v_th:
            return False, 'moving %.3f m/s' % v
        if self.gyro > self.w_th:
            return False, 'rotating %.3f rad/s' % self.gyro
        return True, 'still %.3f m/s, %.3f rad/s' % (v, self.gyro)

    def on_pose(self, m):
        self.n_in += 1
        p = m.pose.pose.position
        lio = (p.x, p.y, p.z)
        still, why = self.is_still()

        # ── 붙잡기 해제 조건 ─────────────────────────────────────────
        if self.hold is not None:
            t = self.now()
            if not still:
                if 'moving' in why:      # 실제로 이동 — 기준점을 버린다
                    self.last_anchor = None
                    self.ref_at_anchor = None
                    self.last_quat = None        # [yaw] 자세 기준도 버린다
                self.release('%s' % why)
            elif t - self.t_hold > self.max_hold:
                self.release('%.0f초 초과' % self.max_hold)
            else:
                # LIO 가 많이 움직였다고 해도 풀지 않는다.
                # 표류가 클 때가 바로 붙잡아야 할 때이기 때문이다.
                # 판정은 robot_odom 과 자이로만 믿는다.
                moved = math.dist(lio, self.hold_lio)
                if moved > self.max_move:
                    self.get_logger().warn(
                        'LIO 가 %.2f m 표류 — 계속 붙잡는다' % moved)
                    self.hold_lio = lio      # 경고 반복을 막는다

        # ── 붙잡기 시작 ──────────────────────────────────────────────
        if self.hold is None and still:
            # 짧게 풀렸다가 다시 잡는 경우, 로봇이 실제로 움직이지 않았다면
            # **이전 기준점을 그대로 쓴다.** 새로 잡으면 그 사이 표류한 위치가
            # 새 기준이 되어 표류가 계단식으로 그대로 통과한다.
            reuse = (self.last_anchor is not None
                     and self.ref_at_anchor is not None
                     and self.ref
                     and math.hypot(self.ref[-1][1] - self.ref_at_anchor[0],
                                    self.ref[-1][2] - self.ref_at_anchor[1]) < 0.10)
            self.hold = self.last_anchor if reuse else lio
            if not reuse:
                self.last_anchor = lio
                self.ref_at_anchor = (self.ref[-1][1], self.ref[-1][2]) if self.ref else None
            self.hold_lio = lio
            self.t_hold = self.now()

            # [yaw] 위치와 같은 방식으로 자세도 저장.
            # 재사용이면 이전 자세를 유지(흘러간 yaw 가 새 기준이 되는 것 방지).
            q = m.pose.pose.orientation
            qt = (q.x, q.y, q.z, q.w)
            self.hold_quat = self.last_quat if (reuse and self.last_quat is not None) else qt
            if not reuse or self.last_quat is None:
                self.last_quat = qt

            self.get_logger().info('정지 감지 — 위치%s 고정%s (%s)'
                                   % ('+자세' if self.hold_yaw_en else '',
                                      '  [이전 기준 재사용]' if reuse else '', why))

        # ── 발행 ─────────────────────────────────────────────────────
        out = Odometry()
        out.header = m.header                    # 시각·프레임 그대로
        out.child_frame_id = m.child_frame_id
        out.pose = m.pose
        out.twist = m.twist

        if self.hold is not None:
            self.n_hold += 1
            self.saved = max(self.saved, math.dist(lio, self.hold_lio))
            out.pose.pose.position.x = self.hold[0]
            out.pose.pose.position.y = self.hold[1]
            out.pose.pose.position.z = self.hold[2]

            # [yaw hold] 정지 중 자세도 진입 시점 값으로 고정.
            # quaternion 4개를 통으로 잡으므로 yaw 뿐 아니라 roll/pitch 도 안정,
            # 오일러 변환·언랩·짐벌락 문제가 없다.
            if self.hold_yaw_en and self.hold_quat is not None:
                out.pose.pose.orientation.x = self.hold_quat[0]
                out.pose.pose.orientation.y = self.hold_quat[1]
                out.pose.pose.orientation.z = self.hold_quat[2]
                out.pose.pose.orientation.w = self.hold_quat[3]
                # 자세를 고정했으므로 각속도도 0 으로 낸다 — 실제로 안 도므로
                out.twist.twist.angular.x = 0.0
                out.twist.twist.angular.y = 0.0
                out.twist.twist.angular.z = 0.0

            # 속도도 0 으로 낸다 — 실제로 안 움직이므로
            out.twist.twist.linear.x = 0.0
            out.twist.twist.linear.y = 0.0
            out.twist.twist.linear.z = 0.0

        self.pub.publish(out)
        self.pub_info.publish(String(data=json.dumps(
            {'holding': self.hold is not None, 'reason': why,
             'yaw_hold': self.hold_yaw_en,
             'drift_blocked': round(self.saved, 3)}, ensure_ascii=False)))

    def release(self, why):
        held = self.saved
        self.get_logger().info('고정 해제 — %s  (막은 표류 %.3f m)' % (why, held))
        self.hold = None
        self.hold_lio = None
        self.t_hold = None
        self.saved = 0.0
        self.hold_quat = None            # [yaw] 붙잡은 자세도 비운다

    def report(self):
        if self.n_in == 0:
            self.get_logger().warn('입력 미수신 — run_indoor.sh 가 떠 있는지 확인할 것')
            return
        s = '고정 중' if self.hold is not None else '통과'
        self.get_logger().info('%s   수신 %d, 고정 %d (%.0f%%)   자이로 %.3f rad/s'
                               % (s, self.n_in, self.n_hold,
                                  100 * self.n_hold / self.n_in, self.gyro))


def main():
    rclpy.init()
    n = ZuptFilter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
