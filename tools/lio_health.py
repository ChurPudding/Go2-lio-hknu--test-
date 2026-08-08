#!/usr/bin/env python3
"""
lio_health.py  --  LIO 위치 추정이 신뢰할 수 있는지 감시하고 플래그를 발행한다

왜 필요한가
----------
LIO 가 틀어져도 **아무도 모른다.** 경로계획은 넘겨받은 위치를 믿고,
장애물 인식은 센서 프레임만 본다. 위치가 18 m 어긋나도 로봇은 자신 있게
벽으로 간다. 이를 감지할 수 있는 곳은 LIO 파이프라인뿐이다.

실측 근거 (2026-08-01 복도 실험, `RESULTS_0802_corridor.md`)
  - 정지 중(로봇 속도 0.002 m/s)인데 LIO 위치가 초당 0.4 m 씩 흘렀다
  - 5초에 1.9 m, 10초에 4.0 m. 세 회차 모두 동일
  - z 는 최종 -3.8 ~ -5.1 m 로 내려갔다 (실제는 평지)
  - 정상 회차는 루프 0.24 m, z RMSE 54 mm

판정 항목
--------
  1. 수신 끊김          LIO 가 timeout 초 이상 조용하면 즉시 NG
  2. 정지 중 표류        로봇이 멈춰 있는데 LIO 위치가 움직인다
  3. 속도 불일치         LIO 속도와 로봇 자체 속도가 오래 어긋난다
  4. z 급변            평지 가정에서 수직 위치가 튄다

2번이 복도 실패의 실제 증상이고 가장 이른 시점에 잡힌다.

기준 신호로 `/utlidar/robot_odom` 을 쓴다. 다리 운동학은 **장기적으로는
표류하지만 단기 속도는 정확하다**(실외 356 m 에서 20 m 표류, 그러나 초 단위
속도는 신뢰 가능). 여기서는 속도만 쓰므로 문제되지 않는다.

출력
----
  /lio/health        std_msgs/Bool     true = 신뢰 가능
  /lio/health_info   std_msgs/String   JSON. 어느 항목이 걸렸는지

**한 번 NG 가 되면 자동으로 복구하지 않는다.** LIO 는 한번 틀어지면 스스로
돌아오지 않기 때문이다. 재시작하려면 `/lio/health_reset` 에 발행한다.

사용
----
    python3 lio_health.py
    python3 lio_health.py --ros-args -p still_drift_max:=0.30

    ros2 topic echo /lio/health
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Empty


class LioHealth(Node):
    def __init__(self):
        super().__init__('lio_health')

        self.declare_parameter('lio_topic', '/aft_mapped_to_init')
        self.declare_parameter('ref_topic', '/utlidar/robot_odom')
        self.declare_parameter('timeout', 0.5)            # LIO 수신 끊김 [s]
        self.declare_parameter('still_speed', 0.05)       # 이하이면 정지로 본다 [m/s]
        self.declare_parameter('still_window', 2.0)       # 정지 판정 구간 [s]
        self.declare_parameter('still_drift_max', 0.20)   # 정지 중 허용 이동 [m]
        self.declare_parameter('speed_window', 3.0)       # 속도 비교 구간 [s]
        self.declare_parameter('speed_err_max', 0.40)     # 허용 속도 차 [m/s]
        self.declare_parameter('z_rate_max', 0.50)        # 허용 z 변화율 [m/s]
        self.declare_parameter('out_topic', '/indoor/health')
        self.declare_parameter('out_info_topic', '/indoor/health_info')
        self.declare_parameter('persist', 3)     # 연속 N 회 걸려야 NG
        self.declare_parameter('yaw_rate_max', 0.15)      # 이상 회전하면 정지검사 건너뜀 [rad/s]
        self.declare_parameter('auto_recover', False)

        g = lambda n: self.get_parameter(n).value
        self.timeout = float(g('timeout'))
        self.still_speed = float(g('still_speed'))
        self.still_window = float(g('still_window'))
        self.still_drift_max = float(g('still_drift_max'))
        self.speed_window = float(g('speed_window'))
        self.speed_err_max = float(g('speed_err_max'))
        self.z_rate_max = float(g('z_rate_max'))
        self.persist = int(g('persist'))
        self.hits = 0
        self.yaw_rate_max = float(g('yaw_rate_max'))
        self.auto_recover = bool(g('auto_recover'))

        self.lio = deque()        # (t, x, y, z)
        self.ref = deque()        # (t, x, y)
        self.healthy = True
        self.reason = ''
        self.latched = False      # 한 번 NG 면 유지
        self.t_last_lio = None
        self.started = False

        self.pub = self.create_publisher(Bool, g('out_topic'), 10)
        self.pub_info = self.create_publisher(String, g('out_info_topic'), 10)
        self.create_subscription(Odometry, g('lio_topic'), self.on_lio, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('ref_topic'), self.on_ref, qos_profile_sensor_data)
        self.create_subscription(Empty, g('out_topic') + '_reset', self.on_reset, 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            'lio_health: %s vs %s -> /lio/health' % (g('lio_topic'), g('ref_topic')))

    # ------------------------------------------------------------------
    @staticmethod
    def _t(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_lio(self, m):
        t = self._t(m)
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.lio.append((t, p.x, p.y, p.z, yaw))
        self.t_last_lio = self.get_clock().now().nanoseconds * 1e-9
        self.started = True
        self._trim(self.lio, t)

    def on_ref(self, m):
        t = self._t(m)
        p = m.pose.pose.position
        self.ref.append((t, p.x, p.y))
        self._trim(self.ref, t)

    def _trim(self, dq, t):
        keep = max(self.still_window, self.speed_window) + 1.0
        while dq and t - dq[0][0] > keep:
            dq.popleft()

    @staticmethod
    def _span(dq, t_end, win):
        """[t_end-win, t_end] 구간의 (첫, 끝) 항목. 부족하면 None."""
        sel = [r for r in dq if r[0] >= t_end - win]
        if len(sel) < 5 or sel[-1][0] - sel[0][0] < win * 0.6:
            return None
        return sel[0], sel[-1]

    def on_reset(self, _):
        self.latched = False
        self.healthy = True
        self.reason = ''
        self.get_logger().warn('health 수동 리셋')

    # ------------------------------------------------------------------
    def check(self):
        """(정상여부, 사유) 를 돌려준다."""
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.started:
            return True, 'waiting'

        # 1) 수신 끊김
        if self.t_last_lio is not None and now - self.t_last_lio > self.timeout:
            return False, 'lio_timeout %.1fs' % (now - self.t_last_lio)

        if not self.lio or not self.ref:
            return True, 'waiting_ref'
        t = self.lio[-1][0]

        # 2) 정지 중 표류 — 복도 실패의 실제 증상
        a = self._span(self.ref, t, self.still_window)
        b = self._span(self.lio, t, self.still_window)
        if a and b:
            dt = a[1][0] - a[0][0]
            ref_d = math.hypot(a[1][1] - a[0][1], a[1][2] - a[0][2])
            dyaw = abs(math.atan2(math.sin(b[1][4] - b[0][4]),
                                  math.cos(b[1][4] - b[0][4])))
            rotating = dt > 0 and dyaw / dt > self.yaw_rate_max
            if dt > 0 and ref_d / dt < self.still_speed and not rotating:
                lio_d = math.hypot(b[1][1] - b[0][1], b[1][2] - b[0][2])
                if lio_d > self.still_drift_max:
                    return False, 'still_drift %.2fm/%.1fs (ref %.2fm)' % (lio_d, dt, ref_d)

        # 3) 속도 불일치
        a = self._span(self.ref, t, self.speed_window)
        b = self._span(self.lio, t, self.speed_window)
        if a and b:
            dt = a[1][0] - a[0][0]
            if dt > 0:
                vr = math.hypot(a[1][1] - a[0][1], a[1][2] - a[0][2]) / dt
                vl = math.hypot(b[1][1] - b[0][1], b[1][2] - b[0][2]) / dt
                if abs(vl - vr) > self.speed_err_max:
                    return False, 'speed_mismatch lio %.2f ref %.2f m/s' % (vl, vr)

        # 4) z 급변
        b = self._span(self.lio, t, 1.0)
        if b:
            dt = b[1][0] - b[0][0]
            if dt > 0 and abs(b[1][3] - b[0][3]) / dt > self.z_rate_max:
                return False, 'z_jump %.2fm/s' % ((b[1][3] - b[0][3]) / dt)

        return True, 'ok'

    def tick(self):
        ok, why = self.check()

        # 연속 persist 회 걸려야 NG. 단발 잡음(로봇 자세 조정, 지나가는 사람)을 거른다.
        self.hits = self.hits + 1 if not ok else 0
        if not ok and self.hits < self.persist:
            ok = True
        if not ok and not self.latched:
            self.latched = True
            self.healthy = False
            self.reason = why
            self.get_logger().error('LIO 신뢰 불가 — %s' % why)
        elif ok and self.latched and self.auto_recover:
            self.latched = False
            self.healthy = True
            self.reason = ''
            self.get_logger().warn('health 자동 복구')
        elif not self.latched:
            self.healthy = True
            self.reason = why

        self.pub.publish(Bool(data=self.healthy))
        self.pub_info.publish(String(data=json.dumps(
            {'healthy': self.healthy, 'reason': self.reason, 'latched': self.latched},
            ensure_ascii=False)))

    def report(self):
        if not self.started:
            self.get_logger().warn('LIO 미수신 — 노드가 떠 있는지 확인할 것')
            return
        self.get_logger().info('health=%s  %s  (lio %d, ref %d)'
                               % (self.healthy, self.reason, len(self.lio), len(self.ref)))


def main():
    rclpy.init()
    n = LioHealth()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
