#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[python3 로 실행 — source 대상 아님]

leg_odom_refine.py — 다리 오도메트리 증분 보정 (2단계 실험 노드)

왜 필요한가
-----------
`/utlidar/robot_odom` 은 발 미끄러짐 때문에 이동 거리를 약 20% 짧게 센다.
`localization_stub.py` 는 스칼라 k=1.23 을 절대 위치에 곱해 이를 보정하지만,
실측된 k 가 1.1910 ~ 1.2327 로 ±1.8% 흩어져 있고 이 흩어짐이 속도
의존성일 가능성이 있다(400 m 코스면 약 7 m 오차). 이 노드는 그 보정을
프레임 간 증분(Δ) 단위로 다시 짜서 ZUPT · 미끄럼 배제 · 속도의존 축척 ·
방위 보정을 단계별로 켜고 끌 수 있게 한다.

    0단  Δ 추출 (항상 켬)         절대 위치 -> body 프레임 증분
    1단  ZUPT                    정지 중 Δ=0
    2단  미끄럼 배제               발 힘/속도로 접지 신뢰도 가중
    3단  속도의존 축척 (본체)      k_x(v), k_y(v)
    4단  방위 보정                 외부 heading 으로 yaw 대체

무엇을 하지 않는가
------------------
  - TF 를 전혀 발행하지 않는다. tf2 도 import 하지 않는다. TF 소유권은
    `localization_stub.py` 에 있다(map -> odom). 이 노드가 같은 것을
    발행하면 두 값이 번갈아 들어와 위치가 튄다.
  - 절대 위치·방위 고정을 하지 않는다(localization_stub.py 와 동일한
    한계). map 원점/진북 정렬은 이 노드의 몫이 아니다.
  - z 축에는 축척을 곱하지 않는다. Go2 의 z 는 고도가 아니라 몸통
    높이이고, k 는 수평 보폭 추정에서 나온 값이라 z 에 적용할 근거가
    없다(localization_stub.py:314 와 동일한 규칙).
  - twist 에 축척을 두 번 곱하지 않는다. localization_stub.py:327-328 은
    위치와 twist 양쪽에 k 를 곱하는데, 이는 twist 에 대해 k^2 이 되는
    기존 버그로 보인다. 이 노드는 twist 를 "보정된 위치의 시간 미분"으로
    다시 계산해 채운다(3단에서 이미 스케일이 반영된 dp_body 를 dt 로
    나눌 뿐, 추가로 k 를 곱하지 않는다).

회귀 없음 보장 (기본 파라미터)
-------------------------------
기본 파라미터(b_x=b_y=0, ZUPT/슬립/헤딩 전부 꺼짐, kx_a=ky_a=K_OUTDOOR)로
돌리면 이 노드의 출력은 `localization_stub.py` 의 출력과 수치적으로
같다. 두 노드의 방식(절대위치×k vs 증분누적×k)이 다르므로, 이를 보장하려면
누적 시작점(anchor)을 첫 메시지 위치에 미리 축척계수를 곱해 잡아야 한다.

    p_corr(N)_xy = kx_a*x(1) + kx*(x(N)-x(1))     (b=0 이면 kx 는 상수)
                 = kx_a * x(N)                      <- stub 의 k*x(N) 과 일치

z 는 축척을 안 곱하므로 원시값을 그대로 anchor 로 잡으면 항상
p_corr_z(N) = z(N) (stub 의 z 패스스루와 일치)이 텔레스코핑으로 성립한다.
첫 메시지는 이 anchor 설정에만 쓰고 출력하지 않는다("이전 값이 없으므로
버리고 상태만 초기화").

4단(방위 보정)이 꺼졌을 때도 정확히 no-op 이 되어야 하므로, 0단에서
`dp_body = R(yaw_t)^T @ dp_world` 로 분해할 때 쓴 yaw_t(직전 메시지 시점의
yaw)를 4단에서 되돌릴 때도 **동일하게** 쓴다(새로 도착한 메시지의 yaw 가
아니다). heading 이 없으면 yaw_hat = yaw_t 이므로
R(yaw_hat) @ R(yaw_t)^T = I 로 왕복이 손실 없이 성립한다.

heading 입력 단위 주의
----------------------
`gps_heading.py` 는 **지금 deg** 를 내보낸다(그 파일 docstring:
"절대 방위 [deg]"). rad 로 오인하면 값이 약 57배(180/pi) 틀어진다. 이
노드는 파라미터 `heading_unit`('deg'|'rad', 기본 'deg')으로 단위를 명시
받아 구독 즉시 rad 로 변환하고, 노드 내부는 전부 rad 만 쓴다. 나중에
`gps_heading.py` 가 rad 로 바뀌어도 `-p heading_unit:=rad` 로 대응하면
되고 이 파일은 고치지 않는다.

heading 토픽 이름 주의
----------------------
`gps_heading.py:53` 은 `~/heading` 으로 **상대** 선언하므로 실제 토픽
이름은 그 노드를 어떤 이름/네임스페이스로 띄웠는지에 따라 달라진다.
이 노드는 heading 토픽 이름을 코드에 박지 않는다. 파라미터
`heading_topic` 으로 받고, 기본값은 빈 문자열(=4단 꺼짐)이다.

    ros2 topic list | grep -i heading

로 실제 이름을 먼저 확인한 뒤 `-p heading_topic:=/실제/이름` 으로 넣을 것.

토픽
----
    구독  /utlidar/robot_odom   nav_msgs/Odometry         150Hz  필수
          /sportmodestate       unitree_go/SportModeState ~50Hz  2단용, 선택
          /lowstate             unitree_go/LowState        ~500Hz 2단/1단용, 선택
          heading_topic         std_msgs/Float32                 4단용, 선택(기본 꺼짐)
    발행  leg_odom              nav_msgs/Odometry
          leg_odom_info         std_msgs/String (JSON 진단, info_rate 로 throttle)

토픽은 상대 이름으로 선언한다. 실행 시 `-r __ns:=/hknu` 를 씌운다
(localization_stub.py 와 동일한 방식). **TF 관련 리매핑은 필요 없다 —
이 노드는 /tf, /tf_static 을 전혀 건드리지 않는다.**

QoS
---
survey_topics.py 에서 겪은 QoS 이중 구독 버그(BEST_EFFORT 발행자에
RELIABLE 로 구독하면 콜백이 조용히 안 불림) 때문에, Go2 가 내는 세 토픽
(robot_odom, lowstate, sportmodestate) 은 모두 `qos_profile_sensor_data`
(BEST_EFFORT) 로 구독한다. heading 토픽은 `gps_heading.py` 가 기본(=
RELIABLE) QoS 로 발행하므로 RELIABLE 로 구독한다. 발행은 전부 RELIABLE
depth 10.

사용
----
    source ~/unitree_ros2/setup_go2.sh
    python3 tools/leg_odom_refine.py --ros-args -r __ns:=/hknu

    # 3단만 개선치로 켜보고 싶다면 (예시, 실측 후)
    python3 tools/leg_odom_refine.py --ros-args -r __ns:=/hknu \\
        -p kx_b:=0.02 -p ky_b:=0.02

검증 절차 (실기 필요 — 이 세션에서는 실행하지 않았다. python3 -m
py_compile 로 문법만 확인했다)
----------------------------------------------------------------------
    # 터미널 1
    source ~/unitree_ros2/setup_go2.sh
    unset CYCLONEDDS_URI
    python3 tools/leg_odom_refine.py --ros-args -r __ns:=/hknu

    # 터미널 2
    unset CYCLONEDDS_URI
    ros2 bag play ~/data/bags/outdoor/0812/go2_loop1_0812_1449 -r 0.5

    # 터미널 3
    ros2 topic echo /hknu/leg_odom_info

확인할 것:
    1. 기본 파라미터에서 dist_out / dist_raw 가 K_OUTDOOR(1.23) 에 수렴
    2. dropped_frames 가 거의 0
    3. enable_scale:=false 로 켜면 dist_out == dist_raw
"""

import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from unitree_go.msg import SportModeState, LowState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from go2_calib import KX_A, KX_B, KY_A, KY_B
except ImportError as e:
    # 축척 계수는 go2_calib.py 한 곳에서만 관리한다. 못 찾으면 기본값을
    # 지어내는 대신 바로 실패한다 (localization_stub.py 와 동일한 원칙).
    raise ImportError(
        "go2_calib.py 를 찾지 못했습니다. tools/go2_calib.py 가 이 파일과 "
        "같은 폴더에 있는지 확인하세요."
    ) from e


# ==========================================================================
# 쿼터니언 <-> yaw 헬퍼 (자체 구현. tf_transformations 는 쓰지 않는다)
#
# geometry_msgs/Quaternion 순서 (x, y, z, w) 를 따른다. robot_pose.py 의
# quat_to_R / R_to_quat 형태를 참고했지만 회전행렬 전체가 필요 없으므로
# 표준 RPY 분해식을 직접 쓴다.
# ==========================================================================

def quat_to_yaw(x, y, z, w):
    """(x,y,z,w) -> yaw [rad]. Z-Y-X(yaw-pitch-roll) 관례."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rpy(x, y, z, w):
    """(x,y,z,w) -> (roll, pitch, yaw) [rad]."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    yaw = quat_to_yaw(x, y, z, w)
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    """(roll, pitch, yaw) [rad] -> (x,y,z,w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quat_replace_yaw(x, y, z, w, new_yaw):
    """roll/pitch 는 보존하고 yaw 만 new_yaw 로 바꾼 쿼터니언."""
    roll, pitch, _ = quat_to_rpy(x, y, z, w)
    return rpy_to_quat(roll, pitch, new_yaw)


def rotz(yaw):
    """z 축(yaw) 회전 2x2 행렬. roll/pitch 는 쓰지 않는다."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class LegOdomRefine(Node):

    def __init__(self):
        super().__init__('leg_odom_refine')

        p = self.declare_parameter

        p('in_topic', '/utlidar/robot_odom')
        p('sms_topic', '/sportmodestate')
        p('lowstate_topic', '/lowstate')
        p('heading_topic', '')           # 빈 문자열이면 4단 꺼짐
        p('heading_unit', 'deg')         # 'deg' | 'rad' — gps_heading.py 는 지금 deg
        p('out_topic', 'leg_odom')
        p('out_info_topic', 'leg_odom_info')

        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        p('enable_zupt', False)
        p('enable_slip', False)          # foot_field_probe.py 결과 나오기 전까지 반드시 False
        p('enable_scale', True)
        p('enable_heading', False)

        p('still_speed', 0.05)           # [m/s]  zupt_filter.py 실측 검증값
        p('still_gyro', 0.15)            # [rad/s]
        p('still_window', 1.0)           # [s]

        p('input_timeout', 0.5)          # [s]  프레임 드롭 / 정지판정 즉시해제 공용

        p('force_field', 'foot_force')   # 'foot_force' | 'foot_force_est'
        p('force_th', 20.0)
        p('resid_th', 0.10)
        p('min_contact', 2)

        p('kx_a', float(KX_A))
        p('kx_b', float(KX_B))
        p('ky_a', float(KY_A))
        p('ky_b', float(KY_B))
        p('k_min', 1.0)
        p('k_max', 1.5)

        p('heading_timeout', 2.0)        # [s]  Float32 에는 stamp 가 없어 벽시계 기준

        p('pos_var', 0.05)
        p('yaw_var', 0.02)
        p('info_rate', 5.0)

        g = lambda n: self.get_parameter(n).value

        self.heading_topic = str(g('heading_topic'))
        self.heading_unit = str(g('heading_unit'))
        if self.heading_unit not in ('deg', 'rad'):
            raise ValueError(
                f"heading_unit 은 'deg' 또는 'rad' 만 허용합니다: "
                f"{self.heading_unit!r}")

        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))

        self.enable_zupt = bool(g('enable_zupt'))
        self.enable_slip = bool(g('enable_slip'))
        self.enable_scale = bool(g('enable_scale'))
        self.enable_heading = bool(g('enable_heading'))

        self.still_speed = float(g('still_speed'))
        self.still_gyro = float(g('still_gyro'))
        self.still_window = float(g('still_window'))

        self.input_timeout = float(g('input_timeout'))

        self.force_field = str(g('force_field'))
        if self.force_field not in ('foot_force', 'foot_force_est'):
            raise ValueError(
                f"force_field 는 'foot_force' 또는 'foot_force_est' 만 "
                f"허용합니다: {self.force_field!r}")
        self.force_th = float(g('force_th'))
        self.resid_th = float(g('resid_th'))
        self.min_contact = int(g('min_contact'))

        self.kx_a = float(g('kx_a'))
        self.kx_b = float(g('kx_b'))
        self.ky_a = float(g('ky_a'))
        self.ky_b = float(g('ky_b'))
        self.k_min = float(g('k_min'))
        self.k_max = float(g('k_max'))

        self.heading_timeout = float(g('heading_timeout'))

        self.pos_var = float(g('pos_var'))
        self.yaw_var = float(g('yaw_var'))
        self.info_rate = float(g('info_rate'))

        # ------------------------------------------------------------
        # 상태
        # ------------------------------------------------------------
        self.initialized = False
        self.prev_raw = None            # np.array3, 직전 메시지 원시 위치
        self.prev_yaw = None            # float, 직전 메시지 yaw [rad]
        self.prev_stamp = None          # float, 직전 메시지 stamp [s]
        self.p_corr = None              # np.array3, 누적 보정 위치

        self.still_since = None         # 정지가 시작된 stamp [s], None=이동중
        self.last_odom_wall = time.time()   # watchdog 용 벽시계

        self.last_sms = None            # SportModeState 최신값
        self.last_sms_wall = None
        self.last_lowstate = None       # LowState 최신값
        self.last_lowstate_wall = None
        self.gyro_available_ever = False

        self.heading_yaw_rad = None     # 최신 heading [rad], 없으면 None
        self.heading_wall = None
        self.heading_ever_received = False
        self.heading_was_fresh = True   # timeout 최초 발생 시 로그용

        self.dropped_frames = 0
        self.k_clamped = 0
        self.dist_raw = 0.0
        self.dist_out = 0.0
        self.dist_alt = 0.0

        self._last_info_wall = 0.0

        # ------------------------------------------------------------
        # 발행자
        # ------------------------------------------------------------
        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.pub_odom = self.create_publisher(Odometry, g('out_topic'), reliable)
        self.pub_info = self.create_publisher(String, g('out_info_topic'), reliable)

        # ------------------------------------------------------------
        # 구독자 — Go2 가 내는 것은 전부 BEST_EFFORT (survey_topics.py 에서
        # 겪은 QoS 이중 구독 버그 참고: RELIABLE 로 구독하면 콜백이 조용히
        # 안 불린다)
        # ------------------------------------------------------------
        self.create_subscription(Odometry, g('in_topic'), self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(SportModeState, g('sms_topic'), self.on_sms,
                                 qos_profile_sensor_data)
        self.create_subscription(LowState, g('lowstate_topic'), self.on_lowstate,
                                 qos_profile_sensor_data)

        if self.heading_topic:
            # gps_heading.py 는 기본(RELIABLE) QoS 로 발행하므로 맞춰 구독한다.
            self.create_subscription(Float32, self.heading_topic, self.on_heading, 10)
        else:
            self.get_logger().info(
                "heading_topic 이 비어 있음 — 4단(방위 보정) 꺼짐. "
                "켜려면 `ros2 topic list | grep -i heading` 으로 실제 이름을 "
                "확인한 뒤 -p heading_topic:=/실제/이름 으로 지정할 것.")

        self.create_timer(0.2, self.watchdog)

        self.get_logger().info(
            "leg_odom_refine  단: "
            f"zupt={'ON' if self.enable_zupt else 'off'}  "
            f"slip={'ON' if self.enable_slip else 'off'}  "
            f"scale={'ON' if self.enable_scale else 'off'}  "
            f"heading={'ON' if self.enable_heading else 'off'}")
        self.get_logger().info(
            f"  kx(v) = {self.kx_a:.4f} + {self.kx_b:.4f}*v   "
            f"ky(v) = {self.ky_a:.4f} + {self.ky_b:.4f}*v   "
            f"clamp=[{self.k_min:.2f}, {self.k_max:.2f}]")
        self.get_logger().info(f"  force_field = {self.force_field}")
        if self.heading_topic:
            conv = ("heading 입력 단위: deg -> rad 변환" if self.heading_unit == 'deg'
                    else "heading 입력 단위: rad (변환 없음)")
            self.get_logger().info(f"  heading_topic = {self.heading_topic}  {conv}")

    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ------------------------------------------------------------------
    def on_sms(self, msg):
        self.last_sms = msg
        self.last_sms_wall = time.time()

    def on_lowstate(self, msg):
        self.last_lowstate = msg
        self.last_lowstate_wall = time.time()
        self.gyro_available_ever = True

    def on_heading(self, msg):
        heading_val_in = float(msg.data)
        if self.heading_unit == 'deg':
            heading_deg_in = heading_val_in
            yaw_rad = math.radians(heading_deg_in)
        else:
            heading_rad_in = heading_val_in
            yaw_rad = heading_rad_in
        self.heading_yaw_rad = yaw_rad
        self.heading_wall = time.time()
        self.heading_ever_received = True

    # ------------------------------------------------------------------
    def watchdog(self):
        """입력이 완전히 끊겼을 때 정지 판정을 즉시 푼다.

        dt·still_window 등은 메시지 stamp 기준으로 재는데(bag 배속과
        무관하게 결정적이도록), 입력 자체가 끊기면 콜백이 아예 안 불려서
        메시지 시계로는 감지할 수 없다. 그래서 이 타이머만 벽시계를 쓴다.
        """
        if time.time() - self.last_odom_wall > self.input_timeout:
            if self.still_since is not None:
                self.get_logger().warn(
                    f"{self.get_parameter('in_topic').value} 입력이 "
                    f"{self.input_timeout:.1f}s 이상 끊김 — 정지 판정 해제")
            self.still_since = None

    # ------------------------------------------------------------------
    def _gyro_fresh(self, now_wall):
        return (self.last_lowstate is not None
                and self.last_lowstate_wall is not None
                and now_wall - self.last_lowstate_wall <= self.input_timeout)

    def _update_still(self, t_stamp, speed_odom, now_wall):
        """1단 ZUPT 판정 갱신. (zupt_active, gyro_used, gyro_norm) 반환."""
        gyro_fresh = self._gyro_fresh(now_wall)
        gyro_norm = None
        if gyro_fresh:
            g = self.last_lowstate.imu_state.gyroscope
            gyro_norm = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)

        ok = speed_odom < self.still_speed
        if gyro_fresh:
            ok = ok and (gyro_norm < self.still_gyro)

        if ok:
            if self.still_since is None:
                self.still_since = t_stamp
        else:
            self.still_since = None

        zupt_active = (self.still_since is not None
                       and (t_stamp - self.still_since) >= self.still_window)
        return zupt_active, gyro_fresh, gyro_norm

    # ------------------------------------------------------------------
    def _slip_stage(self, dp_body, dt, now_wall):
        """2단 미끄럼 배제. (dp_body, n_contact, s, w, dp_body_alt) 반환.

        sms/lowstate 가 신선하지 않거나 접지 발이 min_contact 미만이면
        건너뛴다(w=1.0, s=0.0, dp_body_alt=dp_body 로 그대로 진단값을 채운다).
        """
        have_sms = (self.last_sms is not None and self.last_sms_wall is not None
                    and now_wall - self.last_sms_wall <= self.input_timeout)
        have_low = self._gyro_fresh(now_wall)

        if not (have_sms and have_low):
            return dp_body, 0, 0.0, 1.0, dp_body.copy()

        force_arr = getattr(self.last_lowstate, self.force_field)
        omega = np.array(self.last_lowstate.imu_state.gyroscope, dtype=float)
        fpos = np.array(self.last_sms.foot_position_body, dtype=float)
        fspd = np.array(self.last_sms.foot_speed_body, dtype=float)

        contacts = []
        for i in range(4):
            if force_arr[i] > self.force_th:
                f_i = fpos[3 * i:3 * i + 3]
                fdot_i = fspd[3 * i:3 * i + 3]
                v_i = -(fdot_i + np.cross(omega, f_i))
                contacts.append(v_i)

        n_contact = len(contacts)
        if n_contact < self.min_contact:
            return dp_body, n_contact, 0.0, 1.0, dp_body.copy()

        V = np.array(contacts)                       # (n,3)
        v_bar = np.median(V, axis=0)                 # 성분별 중앙값
        s = float(np.median(np.linalg.norm(V - v_bar, axis=1)))
        w = min(1.0, self.resid_th / max(s, 1e-6))

        dp_body_alt = v_bar * dt                      # 설계 A (진단 전용)
        dp_body_out = dp_body * w                      # 설계 B (기본 출력)
        return dp_body_out, n_contact, s, w, dp_body_alt

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        now_wall = time.time()
        self.last_odom_wall = now_wall

        t = self._stamp_to_sec(msg.header.stamp)
        pin = msg.pose.pose.position
        raw = np.array([pin.x, pin.y, pin.z])
        qo = msg.pose.pose.orientation
        yaw = quat_to_yaw(qo.x, qo.y, qo.z, qo.w)

        # --- 초기화: 첫 메시지는 anchor 설정에만 쓰고 출력하지 않는다 ----
        if not self.initialized:
            kx0 = self._clamp_k(self.kx_a) if self.enable_scale else 1.0
            ky0 = self._clamp_k(self.ky_a) if self.enable_scale else 1.0
            self.p_corr = np.array([kx0 * raw[0], ky0 * raw[1], raw[2]])
            self.prev_raw = raw
            self.prev_yaw = yaw
            self.prev_stamp = t
            self.initialized = True
            self.get_logger().info(
                f"초기화: anchor=({self.p_corr[0]:.3f}, {self.p_corr[1]:.3f}, "
                f"{self.p_corr[2]:.3f})  kx0={kx0:.4f} ky0={ky0:.4f}")
            return

        dt = t - self.prev_stamp
        if dt <= 0.0 or dt > self.input_timeout:
            self.dropped_frames += 1
            self.prev_raw = raw
            self.prev_yaw = yaw
            self.prev_stamp = t
            return

        # --- 0단: Δ 추출, body 프레임으로 회전 (yaw 만, roll/pitch 미사용) ---
        dp_world_raw = raw - self.prev_raw
        yaw_t = self.prev_yaw                     # 이 델타 분해의 기준 yaw
        R = rotz(yaw_t)
        dp_body = np.array([*(R.T @ dp_world_raw[:2]), dp_world_raw[2]])

        speed_odom = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        zupt_active, gyro_used, gyro_norm = self._update_still(t, speed_odom, now_wall)

        # --- 1단: ZUPT ---------------------------------------------------
        if self.enable_zupt and zupt_active:
            dp_body[:] = 0.0

        # --- 2단: 미끄럼 배제 ---------------------------------------------
        n_contact = 0
        slip_s = 0.0
        slip_w = 1.0
        dp_body_alt = dp_body.copy()
        if self.enable_slip:
            dp_body, n_contact, slip_s, slip_w, dp_body_alt = self._slip_stage(
                dp_body, dt, now_wall)

        # --- 3단: 속도의존 축척 (z 는 절대 건드리지 않는다) -----------------
        speed = math.hypot(dp_body[0], dp_body[1]) / dt
        kx = self._clamp_k(self.kx_a + self.kx_b * speed)
        ky = self._clamp_k(self.ky_a + self.ky_b * speed)
        if self.enable_scale:
            dp_body[0] *= kx
            dp_body[1] *= ky

        # twist 는 3단까지 반영된 dp_body 의 시간미분. 여기서 다시 k 를
        # 곱하지 않는다 (localization_stub.py 의 k^2 버그를 재현하지 않음).
        twist_body = dp_body / dt

        # --- 4단: 방위 보정 -------------------------------------------------
        heading_src, yaw_hat = self._select_heading(now_wall, yaw_t)
        Rh = rotz(yaw_hat)
        dp_world_corr = np.array([*(Rh @ dp_body[:2]), dp_body[2]])
        self.p_corr = self.p_corr + dp_world_corr

        # --- 진단 누적 -------------------------------------------------------
        self.dist_raw += float(np.linalg.norm(dp_world_raw))
        self.dist_out += float(np.linalg.norm(dp_world_corr))
        if n_contact >= self.min_contact:
            dp_world_alt = np.array([*(Rh @ dp_body_alt[:2]), dp_body_alt[2]])
            self.dist_alt += float(np.linalg.norm(dp_world_alt))

        # --- 출력 orientation: heading 을 실제로 썼을 때만 재구성 -----------
        if heading_src == 'gps':
            ox, oy, oz, ow = quat_replace_yaw(qo.x, qo.y, qo.z, qo.w, yaw_hat)
        else:
            ox, oy, oz, ow = qo.x, qo.y, qo.z, qo.w

        # --- state 갱신 -------------------------------------------------------
        self.prev_raw = raw
        self.prev_yaw = yaw
        self.prev_stamp = t

        # --- Odometry 발행 -----------------------------------------------------
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame

        out.pose.pose.position.x = float(self.p_corr[0])
        out.pose.pose.position.y = float(self.p_corr[1])
        out.pose.pose.position.z = float(self.p_corr[2])
        out.pose.pose.orientation.x = ox
        out.pose.pose.orientation.y = oy
        out.pose.pose.orientation.z = oz
        out.pose.pose.orientation.w = ow

        cov = [0.0] * 36
        cov[0] = self.pos_var
        cov[7] = self.pos_var
        cov[14] = self.pos_var
        cov[21] = self.yaw_var
        cov[28] = self.yaw_var
        cov[35] = self.yaw_var
        out.pose.covariance = cov
        out.twist.covariance = cov

        out.twist.twist.linear.x = float(twist_body[0])
        out.twist.twist.linear.y = float(twist_body[1])
        out.twist.twist.linear.z = float(twist_body[2])
        out.twist.twist.angular = msg.twist.twist.angular

        self.pub_odom.publish(out)

        # --- info 발행 (throttle) ------------------------------------------
        if self.info_rate > 0 and now_wall - self._last_info_wall >= 1.0 / self.info_rate:
            self._last_info_wall = now_wall
            info = {
                "t": t,
                "stage": {
                    "zupt": self.enable_zupt,
                    "slip": self.enable_slip,
                    "scale": self.enable_scale,
                    "heading": self.enable_heading,
                },
                "zupt_active": bool(zupt_active),
                "n_contact": n_contact,
                "slip_s": slip_s,
                "slip_w": slip_w,
                "speed": speed,
                "kx": kx, "ky": ky,
                "k_clamped": self.k_clamped,
                "dist_raw": self.dist_raw,
                "dist_out": self.dist_out,
                "dist_alt": self.dist_alt,
                "heading_src": heading_src,
                "dropped_frames": self.dropped_frames,
            }
            if not gyro_used:
                info["gyro_note"] = "lowstate 미수신 - twist 만으로 정지판정"
            self.pub_info.publish(String(data=json.dumps(info, ensure_ascii=False)))

    # ------------------------------------------------------------------
    def _clamp_k(self, k):
        c = max(self.k_min, min(self.k_max, k))
        if c != k:
            self.k_clamped += 1
        return c

    # ------------------------------------------------------------------
    def _select_heading(self, now_wall, yaw_t):
        """4단 yaw_hat 결정. (heading_src, yaw_hat) 반환.

        heading 이 꺼져 있거나 값이 없거나 오래됐으면 yaw_t(0단에서 dp_body
        를 분해할 때 쓴 바로 그 yaw)를 그대로 돌려준다 — 그래야 4단이
        꺼졌을 때 회전 재구성이 정확한 항등변환이 된다. Float32 에는 stamp
        가 없으므로 신선도 판정은 부득이 수신 시각의 벽시계로 한다(이
        노드의 다른 모든 타이밍은 메시지 stamp 기준이지만 여기만 예외).
        """
        if not self.enable_heading or not self.heading_topic:
            return 'odom', yaw_t

        if self.heading_yaw_rad is None:
            return 'odom', yaw_t

        fresh = (self.heading_wall is not None
                 and now_wall - self.heading_wall <= self.heading_timeout)
        if fresh:
            self.heading_was_fresh = True
            return 'gps', self.heading_yaw_rad

        if self.heading_was_fresh:
            self.heading_was_fresh = False
            self.get_logger().warn(
                f"heading 이 {self.heading_timeout:.1f}s 이상 안 옴 — "
                "odom yaw 로 되돌림")
        return 'timeout', yaw_t


def main():
    rclpy.init()
    node = LegOdomRefine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
