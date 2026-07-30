# Go2 상태 데이터 추출 설명서 — 오도메트리 & 관절 움직임

Go2의 **오도메트리(위치·속도·자세)**와 **관절 움직임 값(각도·각속도·토크)**을
저수준(low-level)과 고수준(high-level) 두 계층에서 뽑는 방법을 정리한 문서입니다.
함께 제공되는 두 개의 파이썬 노드(`go2_lowlevel_reader.py`, `go2_highlevel_reader.py`)의
사용법과, 각 데이터가 어디서 어떤 형태로 나오는지를 설명합니다.

> 두 노드 모두 **읽기 전용**입니다. `/lowcmd` 나 `/api/sport/request` 로 명령을
> 보내지 않으므로, 실행해도 로봇은 스스로 움직이지 않습니다.

---

## 1. 저수준 vs 고수준 — 무엇이 어디서 나오나

Go2의 데이터는 두 계층으로 나뉩니다. 핵심만 먼저 잡고 가겠습니다.

| 구분 | 토픽 | 메시지 타입 | 여기서 얻는 것 |
|------|------|-------------|----------------|
| **저수준** | `/lowstate` | `unitree_go/msg/LowState` | **12관절**의 q, dq, ddq, tau_est + 본체 IMU |
| **고수준** | `/sportmodestate` | `unitree_go/msg/SportModeState` | **오도메트리**(위치·속도·요), 발 위치, 운동모드 |
| (보조) | `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 표준 형식 오도메트리 (RViz/Nav2 호환) |

직관적으로 정리하면 이렇습니다.

- **관절 하나하나의 움직임**이 궁금하다 → **저수준 `/lowstate`**
  모터 12개의 실제 각도·속도·토크가 그대로 나옵니다. 강화학습 정책 배포,
  보행 분석, 관절 로그 등에 씁니다.
- **로봇이 공간에서 어디에 있고 어디로 가는가**가 궁금하다 → **고수준 `/sportmodestate`**
  내부 상태추정기가 다리 움직임과 IMU를 융합해 만든 위치·속도가 나옵니다.
  이것이 곧 오도메트리입니다.

즉 "관절 = 저수준", "오도메트리 = 고수준"이 기본 대응입니다.
(저수준에는 절대 위치 개념이 없고, 고수준에는 개별 관절 각도가 없습니다.
서로 보완 관계입니다.)

---

## 2. 저수준 `/lowstate` — 관절 움직임 값

`LowState` 메시지에서 우리가 쓰는 핵심은 `motor_state` 배열과 `imu_state` 입니다.

### 2.1 관절(모터) 값 — `motor_state[i]`

`motor_state` 는 길이 20짜리 배열이지만, **앞의 12개만이 다리 관절**입니다
(Go2는 12자유도). 각 원소의 필드는 다음과 같습니다.

| 필드 | 의미 | 단위 |
|------|------|------|
| `q` | 관절 각도 (position) | rad |
| `dq` | 관절 각속도 (velocity) | rad/s |
| `ddq` | 관절 각가속도 (acceleration) | rad/s² |
| `tau_est` | 추정 토크 (estimated torque) | N·m |
| `temperature` | 모터 온도 | ℃ |

### 2.2 12관절 순서 (매우 중요)

Unitree 공식 SDK 기준 순서는 다음과 같습니다. 다리는 **FR → FL → RR → RL**,
각 다리 안에서는 **hip(고관절) → thigh(허벅지) → calf(정강이)** 입니다.

```
0  FR_hip     1  FR_thigh    2  FR_calf     (오른쪽 앞다리)
3  FL_hip     4  FL_thigh    5  FL_calf     (왼쪽 앞다리)
6  RR_hip     7  RR_thigh    8  RR_calf     (오른쪽 뒷다리)
9  RL_hip    10  RL_thigh   11  RL_calf     (왼쪽 뒷다리)
```

> **반드시 실측 검증하세요.** 펌웨어 버전에 따라 순서가 다를 수 있습니다.
> 로봇을 damping(무기력) 상태로 둔 뒤 **한쪽 다리의 한 관절만 손으로 살짝
> 움직이면서**, 화면에서 어느 인덱스의 `q` 값이 변하는지 확인하면 순서를
> 확정할 수 있습니다. 이 검증 없이 인덱스를 신뢰하면 안 됩니다.

### 2.3 IMU — `imu_state`

| 필드 | 의미 |
|------|------|
| `quaternion` | 자세 쿼터니언 [w, x, y, z] |
| `gyroscope` | 각속도 [x, y, z] (rad/s) |
| `accelerometer` | 가속도 [x, y, z] (m/s²) |
| `rpy` | 오일러각 [roll, pitch, yaw] (rad) |

---

## 3. 고수준 `/sportmodestate` — 오도메트리

`SportModeState` 에서 오도메트리에 해당하는 핵심 필드입니다.

| 필드 | 의미 | 단위 |
|------|------|------|
| `position[3]` | 본체 위치 (x, y, z) | m |
| `velocity[3]` | 본체 속도 (vx, vy, vz) | m/s |
| `yaw_speed` | 요(yaw) 각속도 | rad/s |
| `body_height` | 몸통 높이 | m |
| `foot_position_body[12]` | 발 4개 위치 (몸통 기준, FR/FL/RR/RL × xyz) | m |
| `foot_speed_body[12]` | 발 4개 속도 (몸통 기준) | m/s |
| `mode` | 운동 모드 (0 idle, 1 balanceStand, 3 locomotion 등) | - |
| `gait_type` | 걸음새 (0 idle, 1 trot, 2 run, 3 climb stair 등) | - |

> `position` 은 내부 상태추정기가 만든 값이라, 시간이 지나면 오차가 누적(drift)될
> 수 있습니다. 짧은 구간 추적에는 충분하지만, 큰 공간을 정확히 매핑하려면
> LiDAR SLAM 같은 별도 보정이 필요합니다.

### 3.1 표준 오도메트리 — `/utlidar/robot_odom`

`/sportmodestate` 의 `position` 은 Unitree 커스텀 필드입니다. 반면
`/utlidar/robot_odom` 은 **표준 `nav_msgs/msg/Odometry`** 라서 RViz의 Odometry
디스플레이, Nav2, tf 등과 바로 연동됩니다. 주요 접근 경로는 다음과 같습니다.

- 위치: `msg.pose.pose.position.x / .y / .z`
- 자세: `msg.pose.pose.orientation.x / .y / .z / .w`
- 선속도: `msg.twist.twist.linear.x / .y / .z`
- 각속도: `msg.twist.twist.angular.z`

고수준 리더는 두 소스를 함께 구독하여, 화면과 CSV에 나란히 기록합니다.

---

## 4. 실행 방법

### 4.1 준비

새 터미널을 열면 (`.bashrc` 설정 덕분에) 자동으로 Go2 환경이 소싱됩니다.
아래처럼 뜨는지 확인하세요.

```
Setup unitree ros2 environment (Go2, Humble)
```

혹시 안 뜨면 수동으로:

```bash
source ~/unitree_ros2/setup_go2.sh
```

토픽이 살아있는지 확인:

```bash
ros2 topic hz /lowstate
ros2 topic hz /sportmodestate
```

### 4.2 저수준(관절) 리더 실행

```bash
python3 go2_lowlevel_reader.py            # 화면 출력만
python3 go2_lowlevel_reader.py --csv       # 화면 + CSV 저장
python3 go2_lowlevel_reader.py --csv --rate 10   # 10Hz로 제한
```

### 4.3 고수준(오도메트리) 리더 실행

```bash
python3 go2_highlevel_reader.py            # 화면 출력만
python3 go2_highlevel_reader.py --csv       # 화면 + CSV 저장
python3 go2_highlevel_reader.py --csv --rate 10
```

두 리더는 **동시에** 실행해도 됩니다. 터미널을 두 개 열어 각각 돌리면,
관절 값과 오도메트리를 나란히 관찰할 수 있습니다.

종료는 `Ctrl+C` 입니다.

### 4.4 옵션 설명

- `--csv` : 실행 폴더에 타임스탬프가 붙은 CSV 파일을 만듭니다.
  - 저수준: `go2_lowstate_MMDD_HHMMSS.csv`
  - 고수준: `go2_odom_MMDD_HHMMSS.csv`
- `--rate N` : 초당 최대 N번만 출력/기록합니다. Go2 상태 토픽은 수백 Hz로
  들어오므로, 로그가 너무 빠르면 `--rate 10` 정도로 줄이는 것을 권장합니다.
  `0`(기본값)이면 들어오는 대로 전부 처리합니다.

---

## 5. CSV 출력 형식

### 5.1 저수준 CSV (`go2_lowstate_*.csv`)

- `t` : 유닉스 시각(초)
- 관절별 4열 × 12관절 = 48열: `FR_hip_q, FR_hip_dq, FR_hip_ddq, FR_hip_tau, ...`
- IMU 13열: `quat_w/x/y/z, gyro_x/y/z, acc_x/y/z, roll, pitch, yaw`

### 5.2 고수준 CSV (`go2_odom_*.csv`)

- `t` : 유닉스 시각(초)
- `pos_x/y/z, vel_x/y/z, yaw_speed, body_height, mode, gait_type`
- 발 위치 12열: `FR_foot_x/y/z, FL_..., RR_..., RL_...`
- 표준 odom 7열: `odom_x/y/z, odom_qx/qy/qz/qw`

이 CSV는 예전에 하셨던 방식(pandas/matplotlib)으로 바로 불러 분석·시각화할 수
있습니다. 예를 들어 `pos_x`, `pos_y` 를 산점도로 그리면 로봇의 이동 궤적이,
관절 `q` 를 시계열로 그리면 보행 주기가 드러납니다.

---

## 6. 자주 겪는 문제

- **아무 값도 안 나온다** → 토픽 이름/소싱 확인. `ros2 topic hz /lowstate` 로
  데이터가 흐르는지부터 봅니다. 엉뚱한 토픽만 보이면
  `ros2 daemon stop && ros2 daemon start` 후 재시도합니다.
- **QoS 경고/미수신** → 두 노드는 BEST_EFFORT로 구독하도록 설정되어 있어,
  RELIABLE/BEST_EFFORT 어느 발행자와도 호환됩니다. 그래도 안 되면 발행 QoS를
  `ros2 topic info /lowstate --verbose` 로 확인하세요.
- **`unitree_go` 임포트 오류** → Go2 메시지 패키지가 소싱되지 않은 것입니다.
  `setup_go2.sh` 를 소싱한 터미널에서 실행해야 합니다.
- **관절 순서가 이상하다** → 2.2절의 실측 검증을 반드시 수행하세요.

---

## 7. 다음 단계 아이디어

- 관절 `q` 시계열에서 보행 주기(gait cycle)와 위상차를 분석
- `position` 오도메트리와 LiDAR SLAM 결과를 겹쳐 drift(누적 오차) 비교
- 저수준 관절값 + 고수준 오도메트리를 함께 로깅해, 강화학습 정책의
  관측(observation) 데이터셋으로 활용
