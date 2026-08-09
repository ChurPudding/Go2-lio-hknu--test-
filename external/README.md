# external — 외부 패키지의 내 수정본

`go2_ws` 와 `catkin_point_lio_unilidar` 는 git 저장소가 아니라 **백업이 전혀 없었다.**
소실 시 복구 불가였으므로 여기로 옮겼다. (2026-08-08)

원본 소스는 각 저장소에서 받고, 여기 파일을 덮어쓰면 환경이 재현된다.

---

## point_lio_config/

원본: https://github.com/dfloreaa/point_lio_ros2
설치 위치: `~/catkin_point_lio_unilidar/src/point_lio_ros2/`

`my_changes.diff` 가 원본 대비 변경 전문이고, `base_commit.txt` 가 분기 시점이다.
복원 시: 원본 clone → 해당 커밋 체크아웃 → diff 적용.

### 실제로 바꾼 것은 4줄뿐이다

| 항목 | 원본 | 변경 | 이유 |
|---|---|---|---|
| `lid_topic` | `/unilidar/cloud` | `/utlidar/cloud` | Go2 실제 토픽명 |
| **`imu_topic`** | `/unilidar/imu` | **`/l1_imu_fixed`** | **브리지 출력 연결** |
| `publish_odometry_without_downsample` | `true` | `false` | A* 주행 실패 원인이었음 |
| `odom_child_frame_id` | `base_link` | **`base`** | **URDF 루트 링크명이 `base`** |

**`imu_topic` 한 줄이 브리지와 Point-LIO 를 잇는 지점이다.**
`l1_imu_fix.py` 가 L1 자이로 + 몸통 가속도를 결합해 발행하는 토픽을 여기서 받는다.
이 줄이 원본 그대로면 Point-LIO 는 쓸 수 없는 L1 가속도계를 그대로 쓰게 되어
km 단위로 발산한다.

`publish_odometry_without_downsample: false` 는 8월 3일에 확정됐다.
`true` 이면 제자리 걷기 중 `lio_health.py` 가 `health=False` 를 띄워
A* 추종이 멈췄다.

### 바꾸지 않았지만 확인한 것

```yaml
timestamp_unit: 0        # 원본이 이미 0. float32 초 단위.
                         # 2(마이크로초)로 두면 디스큐가 완전히 틀어진다.

extrinsic_R: 단위행렬     # 원본이 이미 단위행렬. 그대로 두는 것이 맞다.
                         # l1_imu_fix.py 가 이미 R_LB 회전을 적용하므로
                         # 여기서 또 돌리면 이중 변환이 된다.
```

### ⚠ `gravity_align` — 주석과 값이 어긋나 있다

현재 파일:

```yaml
#   → 강제 정렬을 끄고 필터가 초기 정지구간에서 잡은 자세를 그대로 기준으로 사용.
gravity_align: true
```

**주석은 "끈다"고 하는데 값은 `true`(켜짐)다.** 원본과 같은 값이라 실질 변경이 없다.

`.bak` 파일을 대조하면 이력이 나온다:

| 파일 | 값 |
|---|---|
| `unilidar_l1.yaml.bak_accnorm981` | **`false`** ← 여기서만 시도 |
| 나머지 6개 (현재 포함) | `true` |

가속도 정규화 실험(`accnorm981`) 때 껐다가 이후 되돌렸고, **주석만 남았다.**

**이 항목은 열어둘 가치가 있다.** 주석에 쓴 증상이 2026-08-07 관측과 정확히 맞는다:

> 강제 정렬하면 지정 방향이 실제(기울어진) 중력과 어긋나
> **궤적이 서서히 아래로 처지는 드리프트**가 발생

| | Point-LIO | 다리 오도메트리 |
|---|---|---|
| z p1~p99 | **5.33 m** | — |
| 지면 추정 | **−0.76 m** | −0.06 m |
| z 범위 | — | **0.28~0.32 m (4 cm)** |

→ **Point-LIO 22 % 드리프트 원인 규명 시 1순위 후보.**
   `gravity_align: false` 로 두고 `floor_0805_1720` 을 재실행해 볼 것.

### `.bak` 파일 6개

파일명에 실험 이력이 남아 있어 그대로 보관한다.

| 파일 | 시점 |
|---|---|
| `.bak_before_imufix` | 브리지 도입 이전 |
| `.bak_accnorm981` | 가속도 정규화 9.81 실험 (`gravity_align: false`) |
| `.bak_expA` | A/B/C 조건 실험 |
| `.bak_cov` | 공분산 조정 |
| `.bak_video` | 시연 녹화용 |
| `.bak` | 미상 |

> ⚠ config 는 `install/` 경로에도 복사해야 적용된다. `src/` 만 고치면 안 먹는다.

---

## unitree_setup/

원본: https://github.com/unitreerobotics/unitree_ros2

| 파일 | 내용 |
|---|---|
| `setup_go2.sh` | **직접 작성.** 원본에 없음. NIC 자동 감지 + CycloneDDS 설정 |
| `setup.sh` | 원본 수정본 |

`setup_go2.sh` 는 매일 쓰는 파일이고 원본 저장소에 없으므로 소실 시 복구 불가였다.

동작:
- `192.168.123.x` 대역 인터페이스 자동 탐색
- `CYCLONEDDS_URI` 를 그 인터페이스로 설정
- 로봇(`192.168.123.161`) ping 확인

> 로봇이 연결돼 있지 않으면 인터페이스를 못 찾고 중단된다.
> bag 재생만 할 때는 `/opt/ros/humble` 과 `cyclonedds_ws` 를 직접 source 하고
> `unset CYCLONEDDS_URI` 할 것.

---

## go2_description/

원본: 유니트리 공식 `Go2_URDF.zip` (ROS1 catkin 패키지)
설치 위치: `~/go2_ws/src/`

2026-07-05 에 ROS2 ament_cmake 로 변환했다. **git 저장소가 아니어서 백업이 없었다.**

### 변환 내용

| 원본 (ROS1) | 변환본 (ROS2) |
|---|---|
| `go2_rviz.launch` (XML) | **`display.launch.py`** (Python, 새로 작성) |
| `check_joint.rviz` | **`go2.rviz`** (rviz2 용 새로 작성) |
| catkin `CMakeLists.txt` | ament_cmake |
| `package.xml` format 2 | format 3 |
| `$(find go2_description)` | `get_package_share_directory()` |
| `type=` | `executable=` |

`use_gui` / `use_rviz` 인자를 추가해 실로봇 연동 시 GUI 를 끄고
실제 `/joint_states` 를 쓰도록 전환 가능하게 했다.

### 여기서 확인한 사실 두 가지

**1. L1 라이다가 아래로 기울어 장착돼 있다**

| child | xyz (m) | rpy (rad) |
|---|---|---|
| `radar` | 0.28945, 0, −0.046825 | 0, **2.8782**, 0 |
| `imu` | −0.02557, 0, 0.04232 | 0, 0, 0 |

pitch 2.8782 rad ≈ **165°**. 점의 대부분이 지면을 향하는 원인이 이것이다.
(7월 측정: 전체 점의 93~97 % 가 지면)

**2. 루트 링크 이름이 `base_link` 가 아니라 `base` 다**

이 때문에 나중에 두 곳에서 문제가 생겼다:
- `slam_toolbox` 의 `base_frame` 기본값이 `base_footprint` → `Failed to compute odom pose`
- Point-LIO 의 `odom_child_frame_id` 기본값이 `base_link` → `base` 로 수정

### 사용

```bash
source ~/go2_ws/install/setup.bash
ros2 launch go2_description display.launch.py
```

이걸 띄우지 않으면 `/tf` 가 하나도 발행되지 않아 RViz 가 `base`, `radar` 프레임을
찾지 못한다.

실로봇 연동 시:

```bash
ros2 launch go2_description display.launch.py use_gui:=false
```

---

## 복원 절차

```bash
# 1. Point-LIO
git clone https://github.com/dfloreaa/point_lio_ros2 \
    ~/catkin_point_lio_unilidar/src/point_lio_ros2
cd ~/catkin_point_lio_unilidar/src/point_lio_ros2
git checkout $(cut -d' ' -f1 ~/fastlio_ws/external/point_lio_config/base_commit.txt)
cp -r ~/fastlio_ws/external/point_lio_config/config .
cp -r ~/fastlio_ws/external/point_lio_config/launch .
cd ~/catkin_point_lio_unilidar && colcon build

# 2. URDF
mkdir -p ~/go2_ws/src
cp -r ~/fastlio_ws/external/go2_description ~/go2_ws/src/
cd ~/go2_ws && colcon build --packages-select go2_description

# 3. 환경 설정
cp ~/fastlio_ws/external/unitree_setup/setup_go2.sh ~/unitree_ros2/
cp ~/fastlio_ws/external/unitree_setup/setup.sh ~/unitree_ros2/
```

---

## 백업하지 않은 것과 이유

| | 이유 |
|---|---|
| `liosam_ws` (198 MB) | LIO-SAM 적용 실패로 미사용. git clone 으로 재취득 가능 |
| `ws_livox` (3.8 MB) | 원본 그대로. 변경 없음 |
| `unilidar_sdk`, `LIO-SAM` | git 저장소이고 변경 없음 |
| `point_lio_ros2/image/*.gif` (236 MB) | README 용 데모 영상. 빌드·실행 무관 |
| `point_lio_ros2/PCD/scans.pcd` (132 MB) | 실험 산출물. bag 에서 재생성 |
