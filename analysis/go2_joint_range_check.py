#!/usr/bin/env python3
"""녹화본의 관절각 범위를 URDF 한계와 대조한다.

사용법: 이 노드를 켠 뒤 다른 터미널에서 `ros2 bag play <bag>` 재생.
        재생이 끝나면 Ctrl-C → 관절별 min/max 리포트 출력.

핵심 구분:
- 일반 주행 녹화본 = '쓴 범위'(락 아님). URDF 한계 이탈/부호만 검사 가능.
- 관절을 락까지 훑은 녹화본 = min/max 가 진짜 락 값 → 보정계수 a,b 산출 가능.
이 노드는 사용범위가 URDF 폭의 90% 이상이면 'LOCK?'으로 표시해 둘을 구분한다.

  export ROS_DOMAIN_ID=99
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  # bag 재생 터미널과 동일 환경. 로컬 재생이면 unset CYCLONEDDS_URI.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState

# Unitree 네이티브 motor_state 인덱스 순서
JOINTS = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]

# 인덱스별 URDF 한계 (lower, upper) rad — thigh 는 앞/뒤 다름
LIMIT = {
    0: (-1.0472, 1.0472),  1: (-1.5708, 3.4907),  2: (-2.7227, -0.83776),  # FR
    3: (-1.0472, 1.0472),  4: (-1.5708, 3.4907),  5: (-2.7227, -0.83776),  # FL
    6: (-1.0472, 1.0472),  7: (-0.5236, 4.5379),  8: (-2.7227, -0.83776),  # RR
    9: (-1.0472, 1.0472), 10: (-0.5236, 4.5379), 11: (-2.7227, -0.83776),  # RL
}
TOL = 0.02  # rad, 한계 이탈 판정 여유


class JointRangeCheck(Node):
    def __init__(self):
        super().__init__("go2_joint_range_check")
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LowState, "lowstate", self.cb, be)
        self.mn = np.full(12, np.inf)
        self.mx = np.full(12, -np.inf)
        self.sm = np.zeros(12)
        self.n = 0
        self.get_logger().info("녹화본 재생을 시작하세요. 끝나면 Ctrl-C.")

    def cb(self, msg: LowState):
        q = np.array([msg.motor_state[i].q for i in range(12)])
        self.mn = np.minimum(self.mn, q)
        self.mx = np.maximum(self.mx, q)
        self.sm += q
        self.n += 1

    def report(self):
        if self.n == 0:
            self.get_logger().warn("수신된 /lowstate 메시지가 없습니다.")
            return
        mean = self.sm / self.n
        print("\n" + "=" * 92)
        print(f"관절각 범위 리포트  (샘플 {self.n}개)")
        print("-" * 92)
        print(f"{'idx':>3} {'joint':<9} {'obs_min':>8} {'obs_max':>8} "
              f"{'mean':>7} | {'urdf_lo':>8} {'urdf_hi':>8} {'used%':>6}  flags")
        print("-" * 92)
        for i in range(12):
            lo, hi = LIMIT[i]
            used = (self.mx[i] - self.mn[i]) / (hi - lo) * 100.0
            flags = []
            if self.mn[i] < lo - TOL or self.mx[i] > hi + TOL:
                flags.append("OUT-OF-RANGE!")          # 규약/스케일 의심
            if "calf" in JOINTS[i] and self.mx[i] > 0:
                flags.append("SIGN?")                    # calf 는 음수여야
            if used >= 90:
                flags.append("LOCK?")                    # 락 도달 → 보정에 사용가능
            print(f"{i:>3} {JOINTS[i]:<9} {self.mn[i]:>8.3f} {self.mx[i]:>8.3f} "
                  f"{mean[i]:>7.3f} | {lo:>8.3f} {hi:>8.3f} {used:>5.0f}%  "
                  f"{' '.join(flags)}")
        print("=" * 92)
        print("해석:")
        print("  OUT-OF-RANGE! → raw q 가 URDF 규약과 안 맞음. 보정(a,b) 필요.")
        print("  SIGN?         → calf 부호 반대. 규약 점검.")
        print("  LOCK?         → 그 관절은 락까지 훑음 → min/max 를 보정 끝점으로 사용가능.")
        print("  flag 없음      → 사용범위가 URDF 안에 정상 포함. raw q 그대로 써도 됨.")


def main():
    rclpy.init()
    node = JointRangeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
