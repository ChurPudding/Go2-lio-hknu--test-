#!/usr/bin/env python3
"""
l1_imu_fix.py  --  Go2 L1 IMU 보정 브리지  (2026-07-30 rev)

배경
----
/utlidar/imu 는 자이로와 가속도계가 서로 다른 프레임에 있다.
  - 자이로     : L1 포인트클라우드 좌표계와 평행 (검증: 중력방향 1.87deg 이내 일치)
  - 가속도계   : 실제 로봇 가속도와 상관 <=0.19 (본체 IMU는 0.83) -> 사용 불가

이 노드는 두 소스의 멀쩡한 부분만 합쳐 /l1_imu_fixed 를 만든다.
  - angular_velocity    <- /utlidar/imu 그대로 (이미 LiDAR 프레임)
  - linear_acceleration <- /lowstate 본체 가속도를 LiDAR 프레임으로 회전

결과적으로 IMU 프레임 == LiDAR 프레임이므로 extrinsic_R 은 단위행렬 그대로 둔다.

이번 개정 내용
-------------
1. 퍼블리셔 QoS 를 RELIABLE 로 변경.
   Point-LIO ROS2 의 IMU 구독자는 기본 프로파일(RELIABLE)이라
   BEST_EFFORT 퍼블리셔와는 호환되지 않아 메시지가 한 개도 전달되지 않는다.
   (증상: ros2 topic info 의 Subscription count 는 1인데 LIO 가 IMU 를 못 받음,
    또는 T2 에 "incompatible QoS ... RELIABILITY" 경고)
2. R_LB / ACC_SCALE 하드코딩 제거 -> go2_calib.py 에서 import.
   같은 상수를 세 노드에 복사해 두면 한 곳을 놓쳐 조용히 틀린 결과가 나온다.
   go2_calib.py 가 없으면 여기서 ImportError 로 즉시 실패한다 (fail-loud).
3. 시작 시 R_LB 값을 로그로 출력하고, 초반 정지 구간 가속도 평균을
   기대값과 자동 비교한다. runbook_01 §5 의 수동 echo 점검을 대체.

주의
----
R_LB 의 yaw 성분은 -127.9deg +-0.5deg 로 확정됐다 (자이로 Kabsch, 2Hz 저역통과,
설명력 98.0%, 독립 시간창 7개 일치). 중력 성분은 회전축이 연직이라 yaw 에
불변이므로 정확하고, yaw 잔여 오차는 수평 동적가속도에만 영향을 준다.
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, HistoryPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState

# ---------------------------------------------------------------
# 외부 파라미터는 go2_calib.py 한 곳에서만 관리한다.
# 이 파일과 같은 디렉터리에 go2_calib.py 를 두면 된다.
# ---------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import R_LB, ACC_SCALE_BODY, EXPECTED_REST_ACC  # noqa: E402


class L1ImuFix(Node):
    # 정지 구간 자동 검증 설정
    REST_CHECK_N = 750          # 250Hz * 3초
    REST_TOL = 0.60             # m/s^2, 성분별 허용 오차

    def __init__(self):
        super().__init__('l1_imu_fix')
        self.declare_parameter('out_topic', '/l1_imu_fixed')
        self.declare_parameter('frame_id', 'utlidar_lidar')
        self.declare_parameter('acc_scale', float(ACC_SCALE_BODY))
        self.declare_parameter('rest_check', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.acc_scale = float(self.get_parameter('acc_scale').value)
        self.rest_check = bool(self.get_parameter('rest_check').value)

        self.acc_body = None          # 최신 본체 가속도
        self.n_imu = 0
        self.n_low = 0
        self._rest_buf = []
        self._rest_done = False

        # --- 여기가 핵심 수정: RELIABLE 퍼블리셔 -------------------
        pub_qos = QoSProfile(
            depth=200,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        out_topic = self.get_parameter('out_topic').value
        self.pub = self.create_publisher(Imu, out_topic, pub_qos)

        # 구독은 센서 데이터 프로파일 유지 (bag 재생이 녹화 당시 QoS 를 제공)
        self.create_subscription(
            LowState, '/lowstate', self.on_lowstate, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/utlidar/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        log = self.get_logger()
        log.info('l1_imu_fix started -> %s (RELIABLE, depth=200)' % out_topic)
        log.info('R_LB[0,0] = %+.6f   (기대값 +0.523029)' % R_LB[0, 0])
        log.info('acc_scale = %.5f' % self.acc_scale)
        log.info('정지 시 기대 가속도 = (%+.2f, %+.2f, %+.2f)'
                 % tuple(EXPECTED_REST_ACC))
        if abs(R_LB[0, 0] - 0.523029) > 1e-6:
            log.error('R_LB 이 확정본과 다르다. go2_calib.py 를 확인할 것.')

    # -----------------------------------------------------------
    def on_lowstate(self, msg):
        a = msg.imu_state.accelerometer
        self.acc_body = np.array([a[0], a[1], a[2]], dtype=float)
        self.n_low += 1

    def on_imu(self, msg):
        if self.acc_body is None:
            return
        acc_lidar = R_LB @ (self.acc_body * self.acc_scale)

        out = Imu()
        out.header.stamp = msg.header.stamp        # L1 타임스탬프 유지
        out.header.frame_id = self.frame_id
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration.x = float(acc_lidar[0])
        out.linear_acceleration.y = float(acc_lidar[1])
        out.linear_acceleration.z = float(acc_lidar[2])
        # orientation 미사용 표시 (FAST-LIO / Point-LIO 모두 참조하지 않음)
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)
        self.n_imu += 1

        if self.rest_check and not self._rest_done:
            self._rest_buf.append(acc_lidar)
            if len(self._rest_buf) >= self.REST_CHECK_N:
                self._verify_rest()

    # -----------------------------------------------------------
    def _verify_rest(self):
        """초반 정지 구간 가속도 평균을 기대값과 비교한다.

        재생 시작 직후 약 3초는 로봇이 정지 상태이므로(bag go2_run_full 은
        6.6초까지 정지) 여기서 중력 벡터가 맞는지 바로 판정할 수 있다.
        """
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
                     '초반 3초가 정지 구간이 아니었을 수 있다.'
                     % self.REST_TOL)
        else:
            log.info('[정지검증] 통과. 진행해도 좋다.')

    def report(self):
        self.get_logger().info(
            'imu_in=%d lowstate_in=%d published=%d'
            % (self.n_imu, self.n_low, self.n_imu))
        if self.n_low == 0:
            self.get_logger().warn(
                '/lowstate 미수신 -> 발행 0. bag 재생이 시작됐는지 확인할 것.')


def main():
    rclpy.init()
    node = L1ImuFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
