# 런북 02 — 실시간 매핑 및 좌표 주행

Go2에서 Point-LIO를 실시간으로 돌려 지도를 만들고, 그 지도상의 목표 좌표까지 자율 주행한다.

전제: 런북 01의 `/l1_imu_fixed` 구성이 검증된 상태.

---

## 0. 안전 — 먼저 읽을 것

로봇이 **스스로 움직인다.** 아래는 선택이 아니라 필수다.

1. **리모컨을 항상 손에 들고 있을 것.** Go2 리모컨의 조작이 ROS 명령보다 우선한다. 이상하면 즉시 조이스틱을 건드리거나 `L2+A`(StandDown).
2. **첫 시험은 반드시 좁은 목표부터.** 목표 거리 1~2 m, 최대 속도 0.2 m/s로 시작한다. 처음부터 방 반대편을 찍지 않는다.
3. **주변 3 m 이상 비울 것.** 사람, 의자, 케이블, 계단·단차 없는 곳.
4. **`goto` 노드는 Ctrl+C 시 반드시 StopMove를 보낸다.** 아래 코드에 포함돼 있다. 임의로 삭제하지 말 것.
5. **혼자 하지 말 것.** 한 명은 노트북, 한 명은 리모컨.
6. 배터리 30% 이하면 시작하지 않는다. 저전압에서 보행 제어가 불안정해진다.

이 노드에는 Nav2 수준의 전역 경로계획이나 코스트맵이 없다. **직선으로 밀고 가다 앞이 막히면 서는** 수준이다. 장애물 회피가 필요하면 9절 참조.

---

## 1. 준비

### 1.1 네트워크

```bash
source ~/setup_go2.sh      # 인터페이스 자동 감지 + CycloneDDS 설정
```

USB-이더넷 NIC 이름이 세션마다 바뀌므로(`enxc0eac369bf02` ↔ `enx00e04c360e79`) 스크립트로 잡는다. 확인:

```bash
ros2 topic hz /lowstate        # 500Hz
ros2 topic hz /utlidar/cloud   # 15Hz
ros2 topic hz /utlidar/imu     # 250Hz
```

셋 다 나와야 진행한다. 안 나오면 로봇 전원·케이블·`ROS_DOMAIN_ID=0` 확인.

### 1.2 워크스페이스

모든 터미널에서:

```bash
srclio     # 런북 01의 별칭
```

### 1.3 로봇 자세

로봇을 평평한 바닥에 세우고 **StandUp** 상태로 둔다. Point-LIO 초기화가 초반 정지 구간에서 이뤄지므로, **실행 후 5초간 움직이지 않는다.**

---

## 2. 실시간 매핑

터미널 3개.

```bash
# [T1] IMU 브리지
python3 ~/fastlio_ws/tools/l1_imu_fix.py
```

`/l1_imu_fixed`가 250 Hz로 나오는지 확인:

```bash
ros2 topic hz /l1_imu_fixed
ros2 topic echo /l1_imu_fixed --field linear_acceleration --once
```

정지 상태에서 **(x ≈ +1.6, y ≈ −1.8, z ≈ −9.5)** 근처여야 한다. 다르면 진행하지 말 것.

```bash
# [T2] Point-LIO + RViz
ros2 launch point_lio mapping_unilidar_l1.launch.py
```

`incompatible QoS` 경고가 없어야 한다.

```bash
# [T3] 모니터
ros2 topic hz /aft_mapped_to_init
```

### 매핑 요령

- **처음 5초는 정지.** 중력 초기화 구간이다.
- 리모컨으로 천천히 걷는다. 0.3 m/s 이하 권장.
- **급회전을 피한다.** 제자리 회전은 나눠서 한다.
- 방 전체를 한 바퀴 돌고 **출발점으로 돌아온다.** 루프를 닫으면 품질을 자체 검증할 수 있다.
- RViz에서 벽이 이중으로 보이면(번짐) 그 지점에서 정합이 실패한 것이다. 잠시 멈췄다 천천히 진행한다.

### 품질 즉석 확인

출발점으로 돌아왔을 때:

```bash
ros2 topic echo /aft_mapped_to_init --field pose.pose.position --once
```

x, y가 0에 가까울수록 좋다. **0.5 m 이내면 양호**, 1 m를 넘으면 다시 매핑하는 편이 낫다.

---

## 3. 지도 저장

yaml에 `pcd_save_en: true`, `interval: -1`이면 **노드 종료 시** PCD가 저장된다.

```bash
# T2에서 Ctrl+C 후
ls -lh ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/
pcl_viewer ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd
```

날짜별로 보관:

```bash
mkdir -p ~/fastlio_ws/maps
cp ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd \
   ~/fastlio_ws/maps/$(date +%Y%m%d_%H%M)_lab.pcd
```

> **중요:** Point-LIO의 좌표 원점(`camera_init`)은 **노드를 실행한 순간의 로봇 위치**다. 노드를 재시작하면 원점이 바뀌므로, 저장한 웨이포인트는 그 세션 안에서만 유효하다. 매핑과 주행을 같은 세션에서 이어서 하는 것이 가장 안전하다.

---

## 4. 웨이포인트 기록

### 방법 A — 현재 위치를 찍기 (권장)

로봇을 목표 지점까지 리모컨으로 데려간 뒤, 그 좌표를 저장한다.

```bash
cat > ~/fastlio_ws/tools/mark_wp.sh << 'EOF'
#!/bin/bash
# 사용법: ./mark_wp.sh <이름>
P=$(ros2 topic echo /aft_mapped_to_init --field pose.pose.position --once \
    | grep -E "^[xyz]:" | awk '{printf "%s ", $2}')
echo "$1 $P" | tee -a ~/fastlio_ws/waypoints.txt
EOF
chmod +x ~/fastlio_ws/tools/mark_wp.sh

~/fastlio_ws/tools/mark_wp.sh 출입구
~/fastlio_ws/tools/mark_wp.sh 창가
```

`~/fastlio_ws/waypoints.txt` 형식: `이름 x y z`

### 방법 B — RViz에서 찍기

RViz 상단 **2D Goal Pose** 버튼 → 지도 위 클릭. `/goal_pose`로 발행된다. 아래 노드가 이걸 구독한다.

RViz의 Fixed Frame이 `camera_init`인지 확인할 것.

---

## 5. 주행 노드

`~/fastlio_ws/tools/goto_node.py`로 저장한다.

```python
#!/usr/bin/env python3
"""
goto_node.py -- Point-LIO 맵 좌표로 Go2 주행

입력:
  /aft_mapped_to_init  (nav_msgs/Odometry)  현재 위치, frame=camera_init
  /goal_pose           (geometry_msgs/PoseStamped)  RViz 2D Goal Pose
  /utlidar/cloud       (sensor_msgs/PointCloud2)  전방 장애물 정지용
출력:
  /api/sport/request   (unitree_api/Request)  Move(1008) / StopMove(1003)

주의: Point-LIO 오도메트리의 자세는 IMU(=LiDAR) 프레임 기준이다.
로봇 전방(base +x)을 얻으려면 R_LB 로 변환해야 한다. 안 하면 헤딩이 128도 틀어진다.
"""
import json
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from unitree_api.msg import Request

# 본체(base) -> L1/LiDAR 프레임 회전 (자이로 Kabsch, 설명력 98.0%)
R_LB = np.array([
    [+0.523029, -0.838576, +0.152420],
    [-0.810712, -0.544668, -0.214668],
    [+0.263034, -0.011292, -0.964721],
])
R_BL = R_LB.T                       # LiDAR -> 본체
FWD_L = R_LB @ np.array([1.0, 0.0, 0.0])   # 로봇 전방을 LiDAR 프레임으로

# base_link 원점 -> LiDAR 원점 (base_link 프레임). 보정 없으면 목표 판정이
# 0.32 m 어긋난다. x,y 는 창분할 최소자승 추정(잔차 40% 개선), z 는 근사값.
LEVER = np.array([0.322, 0.005, 0.050])

API_MOVE = 1008
API_STOPMOVE = 1003

# ---- 안전 한계 (처음에는 이 값으로 시작할 것) ----
V_MAX = 0.20          # m/s
W_MAX = 0.50          # rad/s
ARRIVE_TOL = 0.15     # m
TURN_FIRST = 0.60     # rad. 헤딩 오차가 이보다 크면 제자리 회전만
ODOM_TIMEOUT = 0.5    # s
# 전방 장애물 판정 박스 (본체 프레임, LiDAR 원점 기준)
OBS_X = (0.25, 0.90)
OBS_Y = 0.30
OBS_Z = (-0.25, 0.60)
OBS_MIN_PTS = 12


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GoTo(Node):
    def __init__(self):
        super().__init__('goto_node')
        self.declare_parameter('v_max', V_MAX)
        self.declare_parameter('w_max', W_MAX)
        self.declare_parameter('tol', ARRIVE_TOL)
        self.declare_parameter('enable_obstacle_stop', True)

        self.pos = None
        self.yaw = None
        self.t_odom = 0.0
        self.goal = None
        self.blocked = False
        self.arrived = False

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(Request, '/api/sport/request', pub_qos)

        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self.on_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud',
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)     # 20 Hz
        self.get_logger().info('goto_node ready. waiting for /goal_pose')

    # ---------- 콜백 ----------
    def on_odom(self, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        R_ML = quat_to_R((o.x, o.y, o.z, o.w))   # LiDAR -> map
        R_MB = R_ML @ R_LB                       # base_link -> map
        # LiDAR 원점 -> 몸통 원점 보정
        self.pos = np.array([p.x, p.y, p.z]) - R_MB @ LEVER
        f = R_MB @ np.array([1.0, 0.0, 0.0])     # 맵 프레임에서의 로봇 전방
        self.yaw = math.atan2(f[1], f[0])
        self.t_odom = self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, m):
        self.goal = np.array([m.pose.position.x, m.pose.position.y])
        self.arrived = False
        self.get_logger().info('new goal: (%.2f, %.2f)' % tuple(self.goal))

    def on_cloud(self, m):
        if not self.get_parameter('enable_obstacle_stop').value:
            self.blocked = False
            return
        n = m.width * m.height
        if n == 0:
            return
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(n, m.point_step)
        P = buf[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(float)
        Q = (R_BL @ P.T).T                  # 본체 방향 정렬
        m1 = (Q[:, 0] > OBS_X[0]) & (Q[:, 0] < OBS_X[1])
        m2 = np.abs(Q[:, 1]) < OBS_Y
        m3 = (Q[:, 2] > OBS_Z[0]) & (Q[:, 2] < OBS_Z[1])
        cnt = int((m1 & m2 & m3).sum())
        was = self.blocked
        self.blocked = cnt >= OBS_MIN_PTS
        if self.blocked and not was:
            self.get_logger().warn('obstacle ahead (%d pts) -> hold' % cnt)

    # ---------- 명령 ----------
    def send(self, api_id, param='{}'):
        r = Request()
        r.header.identity.api_id = api_id
        r.parameter = param
        self.pub.publish(r)

    def move(self, vx, vy, wz):
        self.send(API_MOVE, json.dumps({'x': float(vx),
                                        'y': float(vy),
                                        'z': float(wz)}))

    def stop(self):
        self.send(API_STOPMOVE)

    # ---------- 주기 제어 ----------
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pos is None or (now - self.t_odom) > ODOM_TIMEOUT:
            self.stop()
            return
        if self.goal is None or self.arrived:
            self.stop()
            return

        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        tol = float(self.get_parameter('tol').value)

        e = self.goal - self.pos[:2]
        dist = float(np.linalg.norm(e))
        if dist < tol:
            self.arrived = True
            self.stop()
            self.get_logger().info('arrived. residual %.3f m' % dist)
            return

        head_err = wrap(math.atan2(e[1], e[0]) - self.yaw)
        wz = max(-w_max, min(w_max, 1.2 * head_err))

        if abs(head_err) > TURN_FIRST:
            vx = 0.0                        # 먼저 제자리 회전
        elif self.blocked:
            vx = 0.0
            wz = 0.0
        else:
            vx = max(0.0, min(v_max, 0.6 * dist))
            vx *= max(0.0, math.cos(head_err))

        self.move(vx, 0.0, wz)


def main():
    rclpy.init()
    node = GoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):                 # 확실히 멈추도록 반복 전송
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. 실행 절차

### 6.0 위치·헤딩 검증 (주행 전 필수, 가장 확실한 방법)

`robot_pose.py`를 띄우고 **리모컨으로 제자리 한 바퀴**를 돌린다.

```bash
python3 ~/fastlio_ws/tools/robot_pose.py
```

세 가지를 동시에 확인한다.

| 확인 | 기대 | 틀리면 |
|---|---|---|
| `/aft_mapped_to_init` x·y | 반지름 **0.32 m 원호** | LiDAR 원점 궤적이 맞음(정상) |
| `/robot_pose` x·y | **거의 제자리** | `LEVER` 부호·크기 확인 |
| `heading` | 반시계 1회전에 **+360°** | `R_LB` 미적용. 아래 8절 |

이 세 개가 맞으면 회전과 lever arm이 모두 정상이다. 아래 6.1은 heading만 보는 간이 버전이다.

### 6.1 헤딩만 간이 확인

**로봇을 움직이지 않는 상태에서** 노드의 헤딩 계산이 맞는지 먼저 본다. 이게 틀리면 로봇이 엉뚱한 방향으로 간다.

```bash
python3 - << 'EOF'
import numpy as np, math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
R_LB=np.array([[+0.523029,-0.838576,+0.152420],
               [-0.810712,-0.544668,-0.214668],
               [+0.263034,-0.011292,-0.964721]])
F=R_LB@np.array([1.,0.,0.])
def q2R(q):
    x,y,z,w=q; n=math.sqrt(x*x+y*y+z*z+w*w); x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); n=Node('hchk')
def cb(m):
    o=m.pose.pose.orientation; p=m.pose.pose.position
    f=q2R((o.x,o.y,o.z,o.w))@F
    print('pos (%+.2f,%+.2f)  heading %+7.1f deg' % (p.x,p.y,math.degrees(math.atan2(f[1],f[0]))))
n.create_subscription(Odometry,'/aft_mapped_to_init',cb,10)
rclpy.spin(n)
EOF
```

**검증 방법:** 리모컨으로 로봇을 제자리에서 **반시계 방향 90°** 돌린다. 출력 heading이 **+90° 증가**해야 한다. 감소하거나 엉뚱한 값이면 진행하지 말고 아래 8절을 본다.

### 6.2 주행

터미널 4개. T1·T2는 2절과 동일하게 이미 떠 있어야 한다.

```bash
# [T4] 주행 노드 — 처음에는 저속으로
python3 ~/fastlio_ws/tools/goto_node.py --ros-args \
  -p v_max:=0.15 -p w_max:=0.4 -p tol:=0.20
```

RViz에서 **2D Goal Pose**로 **1~2 m 앞**을 찍는다. 로봇이 회전 후 전진해 목표에 도달하면 `arrived. residual ___ m` 로그가 뜬다.

문제없으면 점진적으로 올린다.

```bash
-p v_max:=0.3 -p w_max:=0.6 -p tol:=0.15
```

### 6.3 저장한 웨이포인트로 가기

RViz 없이 좌표를 직접 던진다.

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "camera_init"}, pose: {position: {x: 2.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}'
```

`~/fastlio_ws/waypoints.txt`에 기록해 둔 값을 그대로 넣으면 된다.

---

## 7. 튜닝

| 파라미터 | 기본 | 조정 기준 |
|---|---|---|
| `v_max` | 0.20 | 진동하면 낮춘다. 실내는 0.3이 상한 |
| `w_max` | 0.50 | 회전 중 매핑이 깨지면 낮춘다 |
| `tol` | 0.15 | 오도메트리 정확도보다 크게. 루프 오차가 0.5 m면 tol도 0.3 이상 |
| `TURN_FIRST` | 0.60 rad | 작게 하면 회전 후 직진, 크게 하면 곡선 주행 |
| `OBS_X` 상한 | 0.90 m | 속도가 빠르면 늘린다 (정지거리 확보) |
| `OBS_MIN_PTS` | 12 | 오탐이 잦으면 늘린다. 15 Hz × 4000점 기준 |

코드 내 상수는 파일을 직접 수정한다.

---

## 8. 실패 모드

| 증상 | 원인 | 조치 |
|---|---|---|
| 로봇이 목표 반대로 간다 | 헤딩 계산에 `R_LB` 미적용 | 6.1 검증부터 다시 |
| 헤딩이 반대로 증가 | 부호 규약 문제 | `FWD_L` 대신 `-FWD_L` 시도 후 6.1 재검증 |
| 목표 근처에서 진동 | `tol`이 오도메트리 정확도보다 작음 | `tol` 상향, `v_max` 하향 |
| 목표에서 항상 0.3 m 앞/뒤 정지 | `LEVER` 부호 반대 또는 미적용 | 6.0 검증으로 확인 |
| 계속 "obstacle ahead" | 지면을 장애물로 오인 | `OBS_Z` 하한을 −0.25 → −0.15로 |
| 로봇이 반응 없음 | 스포츠 모드 아님 / 리모컨 점유 | 리모컨으로 StandUp 후 조이스틱 중립 |
| 주행 중 맵이 깨짐 | 회전 속도 과다 | `w_max` 하향 |
| 노드는 도는데 안 움직임 | `/api/sport/request` QoS | `ros2 topic info /api/sport/request --verbose` 확인 |
| 오도메트리가 튄다 | LiDAR 특징 부족 (빈 복도 등) | 구조물 있는 경로로 |

### 긴급 정지

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
'{header: {identity: {api_id: 1003}}, parameter: "{}"}'
```

리모컨이 우선이므로 조이스틱을 건드리는 것이 가장 빠르다.

---

## 9. 다음 단계

이 노드는 **직선 추종 + 전방 정지**만 한다. 대회(순천 실외 GPS 웨이포인트)로 가려면 아래가 필요하다.

1. **Follow-the-Gap 회피 통합** — 이미 설계·시뮬레이션한 반응형 회피를 `tick()`의 `blocked` 분기 자리에 넣는다. 정지 대신 열린 방향으로 조향한다.
2. **다중 웨이포인트 순차 주행** — `waypoints.txt`를 읽어 도달 시 다음 목표로 넘어가는 루프. `self.arrived` 처리부에 인덱스 증가만 추가하면 된다.
3. **GPS 좌표계 연결** — 실외에서는 `camera_init` 대신 GPS 기반 전역 좌표가 기준이 된다. `/gnss`(std_msgs/String JSON)를 `sensor_msgs/NavSatFix`로 바꾸는 브리지 + `robot_localization` 이중 EKF 구조로 LIO와 융합한다. LIO는 국소 정확도, GPS는 전역 드리프트 억제를 담당한다.
4. **실외 재검증** — L1의 유효 사거리가 실내 기준 2.4~5.6 m로 짧게 관측됐다. 개활지에서 특징이 부족하면 LIO가 퇴화(degenerate)한다. 실외 bag을 따로 받아 같은 지표로 평가할 것.
