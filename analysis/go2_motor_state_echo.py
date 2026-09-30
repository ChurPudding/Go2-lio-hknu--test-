#!/usr/bin/env python3
# Go2 모터값 진단 노드
# /lowstate 의 motor_state[0..11] 을 다리별로 라벨링해 0.5초마다 출력한다.
#
# 실행 전 (실기 연결 터미널 기준):
#   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
#   export ROS_DOMAIN_ID=99            # 효신님 환경 설정에 맞춤
#   # (실기: CYCLONEDDS_URI 가 로봇 NIC 를 가리켜야 함 / bag 재생: unset CYCLONEDDS_URI)
#   python3 go2_motor_state_echo.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState

# Unitree 네이티브 관절 순서 (URDF 순서와 다름)
JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",   # 0,1,2  오른앞
    "FL_hip", "FL_thigh", "FL_calf",   # 3,4,5  왼앞
    "RR_hip", "RR_thigh", "RR_calf",   # 6,7,8  오른뒤
    "RL_hip", "RL_thigh", "RL_calf",   # 9,10,11 왼뒤
]


class MotorEcho(Node):
    def __init__(self):
        super().__init__("go2_motor_state_echo")
        # Unitree 네이티브 토픽은 BEST_EFFORT 로 발행됨 -> 맞춰줘야 조용히 씹히지 않음
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(LowState, "lowstate", self.cb, qos)
        self.last_print = self.get_clock().now()
        self.period_ns = 0.5 * 1e9  # 0.5초마다 한 번만 출력
        self.get_logger().info("waiting for /lowstate ...")

    def cb(self, msg: LowState):
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds < self.period_ns:
            return
        self.last_print = now

        lines = ["", "idx  joint       q(rad)    dq(rad/s)   tau(N·m)   T(°C)"]
        for i, name in enumerate(JOINTS):
            m = msg.motor_state[i]
            lines.append(
                f"{i:>2}  {name:<9} {m.q:>9.4f} {m.dq:>10.4f} "
                f"{m.tau_est:>10.3f} {m.temperature:>6}"
            )
        self.get_logger().info("\n".join(lines))


def main():
    rclpy.init()
    node = MotorEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
