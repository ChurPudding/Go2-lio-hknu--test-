#!/usr/bin/env python3
"""
Go2 lowstate -> joint_states 변환 노드
=====================================
실제 로봇은 관절 각도를 /lowstate (unitree_go/msg/LowState) 의
motor_state 배열로 발행합니다. 하지만 RViz의 로봇 모델(robot_state_publisher)은
표준 sensor_msgs/msg/JointState 형식(/joint_states)만 알아듣습니다.

이 노드는 그 둘을 이어 줍니다.
  /lowstate 의 motor_state[i].q  ->  /joint_states 의 position[관절이름]

실행:
    python3 go2_lowstate_to_jointstates.py

종료: Ctrl+C

주의(관절 순서):
  URDF가 기대하는 관절 이름 순서와, lowstate 의 motor_state 배열 순서가
  서로 다를 수 있습니다. 아래 LOWSTATE_INDEX 매핑이 그 대응을 정의합니다.
  로봇이 화면에서 뒤틀려 보이면, 이 매핑을 실측으로 바로잡으면 됩니다.
  (검증법: 한 다리의 한 관절만 손으로 움직이며 화면에서 어느 관절이
   움직이는지 확인)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import LowState
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# URDF가 기대하는 관절 이름 순서 (display 시각화 때 /joint_states 에 나온 순서)
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

# ---------------------------------------------------------------------------
# 각 URDF 관절이 lowstate.motor_state 배열의 몇 번째 인덱스에 해당하는지.
# Unitree 공식 SDK 의 motor_state 순서는 보통 FR -> FL -> RR -> RL 이고,
# URDF(JOINT_NAMES)는 FL -> FR -> RL -> RR 순서라, 아래처럼 재배치합니다.
#
#   lowstate motor_state 인덱스 (공식 SDK 기준):
#     0 FR_hip   1 FR_thigh   2 FR_calf
#     3 FL_hip   4 FL_thigh   5 FL_calf
#     6 RR_hip   7 RR_thigh   8 RR_calf
#     9 RL_hip  10 RL_thigh  11 RL_calf
#
# JOINT_NAMES 순서(FL,FR,RL,RR)에 맞춰 lowstate 인덱스를 나열:
# ---------------------------------------------------------------------------
LOWSTATE_INDEX = [
    3, 4, 5,     # FL_hip, FL_thigh, FL_calf  -> motor_state[3,4,5]
    0, 1, 2,     # FR_hip, FR_thigh, FR_calf  -> motor_state[0,1,2]
    9, 10, 11,   # RL_hip, RL_thigh, RL_calf  -> motor_state[9,10,11]
    6, 7, 8,     # RR_hip, RR_thigh, RR_calf  -> motor_state[6,7,8]
]


class LowStateToJointState(Node):
    def __init__(self):
        super().__init__("go2_lowstate_to_jointstates")

        # lowstate 는 BEST_EFFORT 로 발행되므로 구독도 BEST_EFFORT
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            LowState, "/lowstate", self.callback, sub_qos
        )

        # joint_states 는 robot_state_publisher 가 안정적으로 받도록 RELIABLE
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self._count = 0
        self.get_logger().info(
            "lowstate -> joint_states 변환 시작. /lowstate 수신 대기 중..."
        )

    def callback(self, msg: LowState):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [float(msg.motor_state[idx].q) for idx in LOWSTATE_INDEX]
        # 속도도 함께 담아주면 유용합니다(선택).
        js.velocity = [float(msg.motor_state[idx].dq) for idx in LOWSTATE_INDEX]
        self.pub.publish(js)

        # 너무 자주 로그가 찍히지 않도록 약 1초에 한 번만 안내
        self._count += 1
        if self._count % 500 == 0:
            self.get_logger().info("joint_states 발행 중...")


def main():
    rclpy.init()
    node = LowStateToJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
