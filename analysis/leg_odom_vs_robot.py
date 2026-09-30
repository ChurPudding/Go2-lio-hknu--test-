#!/usr/bin/env python3
"""내 다리 오도메트리 식 = /utlidar/robot_odom 인지 대조하는 노드.

목표: '내 식 출력 = robot_odom' 을 만들고, 안 맞으면 어디서(어느 항) 얼만큼
벌어지는지 본다. 스케일을 통째 곱하지 않고, 내부에 숨은 k_odom 을 피팅으로 찾는다.

핵심 설계:
- 비교는 '순간 속도'로 한다. robot_odom.twist 를 정답지로 써서 적분드리프트를
  배제 → 식 자체의 정확도만 본다. (누적궤적은 CSV로 따로 남겨 오프라인 확인)
- k_odom 은 최소자승으로 온라인 추정: k = Σ(내식·odom)/Σ(내식·내식)
- use_gyro 스위치로 ω×p 항 기여를 켜고 끄며 편향의 출처를 분해.
- twist 가 body 프레임인지 world 인지 모호 → 두 경우 잔차를 다 재서 더 작은 쪽으로
  프레임 규약까지 역판정.

*** 실행 전 반드시 확인 (블랙박스라 가정이 들어감) ***
  ros2 topic type /utlidar/robot_odom     # nav_msgs/msg/Odometry 인지
  ros2 interface show unitree_go/msg/IMUState  # quaternion/gyroscope/rpy 필드명
가정이 틀리면 아래 표시된 ASSUMPTION 부분만 고치면 됩니다.

  export ROS_DOMAIN_ID=99
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  # bag 재생 시: ros2 bag play <bag> --clock  (그리고 use_sim_time:=true)
  python3 leg_odom_vs_robot.py --ros-args -p use_gyro:=true -p stance_margin:=0.02
"""

import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import LowState
from nav_msgs.msg import Odometry          # ASSUMPTION: robot_odom 타입

import go2_leg_kinematics as kin


def quat_to_R(w, x, y, z):
    """쿼터니언(w,x,y,z) -> 회전행렬. Unitree imu_state.quaternion 순서 가정."""
    n = np.sqrt(w*w + x*x + y*y + z*z) + 1e-12
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])


class LegOdomVsRobot(Node):
    def __init__(self):
        super().__init__("leg_odom_vs_robot")
        self.use_gyro = self.declare_parameter("use_gyro", True).value
        self.stance_mode = self.declare_parameter("stance_mode", "force").value  # force|height|all
        self.stance_margin = self.declare_parameter("stance_margin", 0.02).value
        self.force_thr = self.declare_parameter("force_thr", 20.0).value
        self.reject = self.declare_parameter("reject_outliers", True).value
        self.alpha = self.declare_parameter("alpha", 1.0).value   # 1.0=필터끔, 작을수록 강함
        self.csv_path = self.declare_parameter("csv_path", "leg_odom_cmp.csv").value

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LowState, "lowstate", self.on_lowstate, be)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self.on_odom, 10)

        self.latest = None          # (q_all, dq_all, tau_all, omega, R)
        self.v_ema = None           # EMA 저역통과 상태
        # 최소자승 누적: body/world 두 프레임 각각
        self.acc = {"body": self._new_acc(), "world": self._new_acc()}
        self.n = 0
        # 궤적 적분 (오프라인 플롯용)
        self.p_mine = np.zeros(3)
        self.t_prev = None
        self.csv = open(self.csv_path, "w", newline="")
        self.w = csv.writer(self.csv)
        self.w.writerow(["t", "vmx", "vmy", "vmz", "vox", "voy", "voz",
                         "pmx", "pmy", "omx", "omy", "stance", "use_gyro"])
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"use_gyro={self.use_gyro} stance_mode={self.stance_mode} "
            f"force_thr={self.force_thr} reject={self.reject} alpha={self.alpha} "
            f"→ {self.csv_path}")

    @staticmethod
    def _new_acc():
        # Σ(m·o), Σ(m·m), Σ|o-m|^2, Σ|o|^2  (축별)
        return {"mo": np.zeros(3), "mm": np.zeros(3),
                "res": np.zeros(3), "oo": np.zeros(3)}

    def on_lowstate(self, msg: LowState):
        q = np.array([m.q for m in msg.motor_state[:12]])
        dq = np.array([m.dq for m in msg.motor_state[:12]])
        tau = np.array([m.tau_est for m in msg.motor_state[:12]])
        q_all, dq_all = kin.unpack_motor(q, dq)
        _, tau_all = kin.unpack_motor(tau, tau)   # tau 를 다리별로 분해
        imu = msg.imu_state
        omega = np.array(imu.gyroscope)                 # ASSUMPTION: 필드명
        quat = imu.quaternion                           # ASSUMPTION: (w,x,y,z)
        R = quat_to_R(quat[0], quat[1], quat[2], quat[3])
        self.latest = (q_all, dq_all, tau_all, omega, R)

    def on_odom(self, msg: Odometry):
        if self.latest is None:
            return
        q_all, dq_all, tau_all, omega, R = self.latest

        if self.stance_mode == "all":
            stance = list(kin.LEGS)
        elif self.stance_mode == "height":
            stance = kin.stance_by_height(q_all, self.stance_margin)
        else:  # "force"
            stance = kin.stance_by_force(q_all, tau_all, self.force_thr)

        v_mine = kin.body_velocity(q_all, dq_all, omega, stance,
                                   use_gyro=self.use_gyro,
                                   reject_outliers=self.reject)
        if self.alpha < 1.0:                      # 선택적 저역통과
            self.v_ema = v_mine if self.v_ema is None \
                else self.alpha * v_mine + (1 - self.alpha) * self.v_ema
            v_mine = self.v_ema

        tw = msg.twist.twist.linear
        v_odom = np.array([tw.x, tw.y, tw.z])

        # body 프레임 비교 & world 프레임 비교 둘 다 누적
        self._accumulate("body", v_mine, v_odom)
        self._accumulate("world", R @ v_mine, v_odom)
        self.n += 1

        # 궤적 적분 (내 식) + robot_odom 위치
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_prev is not None:
            dt = t - self.t_prev
            if 0 < dt < 0.5:
                self.p_mine = self.p_mine + R @ v_mine * dt
        self.t_prev = t
        op = msg.pose.pose.position
        self.w.writerow([f"{t:.4f}", *[f"{x:.5f}" for x in v_mine],
                         *[f"{x:.5f}" for x in v_odom],
                         f"{self.p_mine[0]:.4f}", f"{self.p_mine[1]:.4f}",
                         f"{op.x:.4f}", f"{op.y:.4f}", len(stance),
                         int(self.use_gyro)])

    def _accumulate(self, frame, m, o):
        a = self.acc[frame]
        a["mo"] += m * o
        a["mm"] += m * m
        a["res"] += (o - m) ** 2
        a["oo"] += o * o

    def report(self):
        if self.n < 5:
            return
        for frame in ("body", "world"):
            a = self.acc[frame]
            # 축별 최소자승 스케일과, 전체 스칼라 스케일
            k_axis = a["mo"] / np.maximum(a["mm"], 1e-9)
            k_scalar = a["mo"].sum() / max(a["mm"].sum(), 1e-9)
            rms = np.sqrt(a["res"].sum() / self.n)
            # k 적용 후 잔차 (스칼라 k로): Σ|o - k m|^2 = Σo² -2kΣmo +k²Σmm
            res_k = (a["oo"].sum() - 2*k_scalar*a["mo"].sum()
                     + k_scalar**2 * a["mm"].sum())
            rms_k = np.sqrt(max(res_k, 0) / self.n)
            self.get_logger().info(
                f"[{frame}] n={self.n} k_scalar={k_scalar:+.3f} "
                f"k_axis=[{k_axis[0]:+.2f} {k_axis[1]:+.2f} {k_axis[2]:+.2f}] "
                f"RMS(k=1)={rms:.4f} RMS(k_fit)={rms_k:.4f} m/s")

    def destroy_node(self):
        try:
            self.csv.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = LegOdomVsRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
