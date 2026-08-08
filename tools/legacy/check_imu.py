#!/usr/bin/env python3
"""
/utlidar/imu 를 잠깐 구독해서 몸통 IMU인지 L1 내장 IMU인지 추정한다.
로봇을 '정지'시킨 상태에서 실행할 것. 움직이면 판별이 흐려진다.

사용:
    source ~/setup_go2.sh
    python3 check_imu.py
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

N = 100  # 평균 낼 샘플 수 (약 0.4초 분량, 249Hz 기준)

class ImuChecker(Node):
    def __init__(self):
        super().__init__('imu_checker')
        self.ax = []
        self.ay = []
        self.az = []
        self.sub = self.create_subscription(
            Imu, '/utlidar/imu', self.cb, 50)
        self.get_logger().info(
            f'/utlidar/imu 수집 시작 — 로봇을 정지시킨 채 {N}개 샘플 대기')

    def cb(self, msg):
        a = msg.linear_acceleration
        self.ax.append(a.x)
        self.ay.append(a.y)
        self.az.append(a.z)
        if len(self.az) >= N:
            self.report()
            rclpy.shutdown()

    def report(self):
        mx = sum(self.ax) / len(self.ax)
        my = sum(self.ay) / len(self.ay)
        mz = sum(self.az) / len(self.az)
        mag = math.sqrt(mx*mx + my*my + mz*mz)

        print('\n' + '=' * 46)
        print(f'  샘플 수      : {len(self.az)}')
        print(f'  평균 가속도  : x={mx:+.3f}  y={my:+.3f}  z={mz:+.3f}')
        print(f'  크기(중력)   : {mag:.3f}  (9.8 근처여야 정상)')
        print('=' * 46)

        if abs(mag - 9.8) > 2.5:
            print('  [주의] 크기가 9.8에서 많이 벗어남.')
            print('         로봇이 움직였거나 단위가 g일 수 있음. 재확인 필요.')
        elif mz > 7.0 and abs(mx) < 2.5:
            print('  >> 판정: 몸통 IMU로 보임 (z축이 위를 향함)')
            print('     현재 go2_l1.yaml 의 extrinsic 을 그대로 사용.')
        elif mz < -5.0 and abs(mx) > 1.5:
            print('  >> 판정: L1 내장 IMU로 보임 (뒤집힌 자세, z 음수 + x 성분)')
            print('     go2_l1_lidar_imu.yaml 로 교체할 것.')
        else:
            print('  >> 판정: 애매함. 아래 원칙으로 직접 판단.')
            print('     - z가 +9.8 부근, x≈0  -> 몸통 IMU')
            print('     - z가 음수 + x 성분   -> L1 내장 IMU')
        print()

def main():
    rclpy.init()
    node = ImuChecker()
    try:
        rclpy.spin(node)
    except Exception:
        pass

if __name__ == '__main__':
    main()
