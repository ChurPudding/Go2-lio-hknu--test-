# 파일 의존 관계

각 파일이 무엇을 받고 무엇에 기대는지 정리했습니다.
갱신 2026-08-13(아침) · 담당 효신

> **2026-08-12 갱신**: 실외 경로가 새로 생겼습니다. 실외 bag 에는
> `cloud_deskewed` 가 없어 `odom_map_build_v3.py` 가 원시 `/utlidar/cloud` 를
> 받아 `R_BL`·`LEVER` 로 직접 변환합니다. `/gnss` 를 쓰는 GPS 보정 경로
> (`build_maps_0812.py`)도 추가됐습니다. 근거는
> `results/outdoor_0812/NOTES.md`.

> **2026-08-07 갱신**: 지도 생성 경로가 바뀌었습니다. Point-LIO 의
> `PCD/scans.pcd` 대신 `odom_map_build.py` → `loop_correct_v2.py` 가 만든
> PCD 를 `pcd_to_grid.py` 가 받습니다. Point-LIO 관련 노드·설정은 지우지
> 않고 **보류**로 남겼습니다(실시간 위치추정에는 그대로 씁니다). 근거는
> `docs/2026-08-07_실험기록.md`.

---

## 0. 통합 개요 — 실내·실외 한 장

세 갈래가 있습니다. **실시간 위치추정**(Point-LIO), **실내 지도**, **실외
지도**. 뒤의 둘은 입력 토픽 하나만 다르고 `pcd_to_grid.py` 부터 다시 합쳐집니다.

```mermaid
flowchart TD
    subgraph src["로봇이 내보내는 것 (공통)"]
        T4["/utlidar/imu + /lowstate<br/>자이로 · 가속도"]
        T1D["/utlidar/cloud_deskewed<br/>odom 좌표계 · 실내만 녹화"]
        T1["/utlidar/cloud<br/>라이다 프레임 · 원시"]
        T2["/utlidar/robot_odom<br/>/sportmodestate"]
        T3["/gnss<br/>1 Hz · JSON · 실외만"]
    end

    CAL["tools/go2_calib.py<br/>R_LB · R_BL · LEVER · k"]

    %% ---------- A. 실시간 ----------
    T4 --> BR["l1_imu_fix.py<br/>param acc_topic"]
    CAL -.R_LB.-> BR
    BR --> LIO["Point-LIO<br/>⚠ 실내 드리프트 21.75%"]
    T1 --> LIO
    LIO --> RT["robot_pose → lio_health → lio_tf<br/>실시간 위치추정 · 주행"]

    %% ---------- B. 실내 지도 ----------
    T1D --> IN["odom_map_build_v2.py<br/>좌표변환 불필요<br/>이미 odom 좌표계"]
    T2 --> IN
    IN --> LC["loop_correct_v2.py<br/>ICP fitness ≥ 0.9 일 때만"]

    %% ---------- C. 실외 지도 ----------
    T1 --> OUT["odom_map_build_v3.py<br/>lidar→body→odom 직접 변환"]
    T2 --> OUT
    CAL -.R_BL · LEVER.-> OUT
    OUT --> BM["build_maps_0812.py<br/>none / rigid / warp"]
    T3 --> BM

    %% ---------- 합류 ----------
    LC --> PCD["scans.pcd"]
    BM --> PCD
    PCD --> P2G["pcd_to_grid.py<br/>실내 0.05 · 실외 0.15 m"]
    P2G --> GRID["map.pgm + map.yaml"]
    GRID --> NAV["nav2_map_server<br/>go2_nav_interface.py<br/>→ 팀원A"]

    %% ---------- 축척 ----------
    KSRC["축척 k ≈ 1.20<br/>줄자 1.1995<br/>GPS 1.1910 / 1.2327"]
    KSRC -.위치에만 적용.-> IN
    KSRC -.위치에만 적용.-> OUT

    style LIO stroke-dasharray: 4 4
    style RT stroke-dasharray: 4 4
```

### 실내 vs 실외 — 무엇이 다른가

| | 실내 | 실외 |
|---|---|---|
| **입력 점군** | `/utlidar/cloud_deskewed` | `/utlidar/cloud` (원시) |
| **좌표 변환** | 불필요 (이미 odom) | `R_BL` + `LEVER` 2단계 |
| **모션 보정** | 되어 있음 | **안 됨** — 급회전 시 밀림 |
| **누적 도구** | `odom_map_build_v2.py` | `odom_map_build_v3.py` |
| **축척 출처** | 줄자 실측 1.1995 (고정) | GPS 정렬 (bag 마다) |
| **드리프트 보정** | `loop_correct_v2.py` (ICP) | GPS `rigid`/`warp` |
| **좌표계** | odom (부팅 기준) | **ENU** (북쪽 위, 웨이포인트와 동일) |
| **격자 해상도** | 0.05 m | 0.15 m |
| **점 밀도** | 벽이 많아 조밀 | 개활지라 희소 |
| **동적 물체** | 적음 | 사람 — 유령 궤적 남음 |

### 실측 성적

| | 실내 | 실외 |
|---|---|---|
| 최고 폐루프 드리프트 | **0.275%** (loop_0810_1) | **0.31%** (loop1_1449, 276.7 m) |
| 자유공간 연결성 | 92% | 미측정 |
| 기준 산출물 | `results/loop_0810/grid005` | `results/outdoor_0812/grid_15/loop1_0812_1449_warp` |
| Point-LIO 비교 | 21.75% (⚠ 미해결) | 미실행 |

### 세 갈래의 관계

- **A(실시간)** 는 지도 생성에 쓰지 않습니다. 주행 중 위치추정 전용입니다.
  Point-LIO 의 실내 드리프트 21.75% 는 원인 미규명 상태로 **보류**입니다.
- **B(실내)** 는 완료됐습니다. 파이프라인·파라미터 모두 확정.
- **C(실외)** 가 현행 주력입니다. 대회 코스가 실외 GPS 웨이포인트이므로
  여기에 GTSAM + GPS factor 를 얹는 것이 다음 단계입니다.

세 갈래 모두 `go2_calib.py` 의 외부 파라미터에 의존합니다. **라이다를 다시
장착하거나 재교정하면 세 갈래가 한꺼번에 영향을 받습니다.**

---

## 1. 전체 흐름 — 실내

```mermaid
flowchart TD
    subgraph robot["로봇이 내보내는 것"]
        C1["/utlidar/imu<br/>250 Hz · 자이로 (LiDAR 프레임)"]
        C2["/lowstate<br/>500 Hz · 가속도"]
        C2B["/sportmodestate<br/>289 Hz · 가속도·자세·position<br/>가속도계는 lowstate 와 동일 센서"]
        C3["/utlidar/cloud<br/>15 Hz · 단일 프레임 · time 필드 있음"]
        C3D["/utlidar/cloud_deskewed<br/>15 Hz · odom 좌표계 · 원본의 2.8배 누적"]
        C4["/utlidar/robot_odom<br/>150 Hz · 다리+몸통IMU 융합<br/>거리 17% 짧음 · 회전은 정확"]
    end

    CAL["tools/go2_calib.py<br/>R_LB · R_BL · LEVER"]

    %% ---------- 실시간 위치추정 · 주행 (run_indoor.sh) ----------
    C1 --> BR["tools/l1_imu_fix.py<br/>param acc_topic"]
    C2 --> BR
    C2B -.대체 입력.-> BR
    CAL -.import R_LB.-> BR
    BR --> IMU["/l1_imu_fixed"]
    C3 --> LIO["mapping_unilidar_l1.launch.py<br/>외부 패키지 point_lio"]
    IMU --> LIO
    CFG["config/unilidar_l1.yaml<br/>백업: external/point_lio_config/"] -.읽음.-> LIO
    LIO --> AFT["/aft_mapped_to_init"]
    LIO --> REG["/cloud_registered"]
    LIO -.보류.-> PCDOLD["PCD/scans.pcd (Point-LIO)<br/>⏸ 드리프트 21.75%"]
    AFT --> RP["tools/robot_pose.py"]
    RP --> BP["/indoor/base_pose"]
    C4 --> HL["tools/lio_health.py"]
    BP --> HL
    HL --> HE["/indoor/health"]
    HE -.covariance.-> RP
    BP --> TF["tools/lio_tf.py"]
    HE --> TF
    TF --> TFT["/tf"]

    %% ---------- 축척계수 측정 (2026-08-10) ----------
    subgraph calib["축척계수 측정 · 2026-08-10"]
        SB["scale_0810_run1~3 (+_back)<br/>robot_odom 만 녹화"]
        TAPE["타일 실측 51.83 m<br/>128칸 x 40cm + 63cm"]
        SB --> SCK["tools/scale_check.py<br/>정지구간 자동검출 · 직선변위"]
        TAPE --> SCK
        SCK --> KK["k = 1.1995<br/>6회 · 편차 0.33 m<br/>전진·후진 차 0.37%"]
    end

    %% ---------- 지도 생성 · bag 오프라인 ----------
    subgraph mapping["지도 생성 · bag 오프라인 (실내)"]
        BAG["녹화 bag (loop_0810_1)<br/>cloud_deskewed + robot_odom + cloud"]
        BAG --> LC2["tools/loop_correct_v2.py --k<br/>bag 을 직접 읽는다<br/>루프 클로저 + 축척 보정"]
        BAG -.루프보정 없이 · 진단용.-> OMB["tools/odom_map_build_v2.py<br/>오차가 그대로 보인다"]
        LC2 --> PCDCORR["scans.pcd (보정)<br/>fitness≥0.9 일 때만 적용"]
        OMB --> PCDNL["scans.pcd (무보정)"]
    end

    KK -.k 적용.-> LC2
    KK -.k 적용.-> OMB
    C3D -.기록.-> BAG
    C4 -.기록.-> BAG
    C3 -.기록 · 향후 디스큐용.-> BAG

    PCDCORR --> P2G["tools/pcd_to_grid.py<br/>인자: pcd 출력이름 해상도"]
    PCDNL -.진단.-> P2G
    PCDOLD -.보류.-> P2G
    P2G --> GRID["results/loop_0810/<br/>grid.pgm + .yaml + .npy<br/>grid005 = 0.05 m/cell (권장)"]

    %% ---------- 검증 (2026-08-10) ----------
    subgraph verify["검증 · 2026-08-10"]
        GRID --> MM["tools/map_measure.py<br/>두 점 클릭 거리 측정"]
        MM --> MMR["실측 대비 0.1~5%<br/>벽 두께 약 0.17 m"]
        BAG --> YC["tools/yaw_check.py<br/>다리 요 vs 자이로 적분 vs 내부융합"]
        YC --> YCR["회전 정확도 회당 0.17도<br/>루프 폐쇄 0.58 m / 211.2 m = 0.27%"]
    end
    CAL -.import R_BL.-> YC

    %% ---------- 배포 ----------
    GRID --> MS["nav2_map_server<br/>lifecycle configure → activate"]
    MS --> MT["/map · nav_msgs/OccupancyGrid<br/>QoS transient_local + reliable<br/>미관측 없음 — 건물 밖도 free"]
    MT --> A["팀원A · A* / Nav2"]
    GRID -.자체 발행 경로.-> MP["tools/map_publisher.py"]
    MP --> MT2["/indoor/map"]
    BP --> NAV["tools/go2_nav_interface.py"]
    C3 --> NAV
    GRID --> NAV
    NAV --> OUT["/map /odom /scan /tf"]
    OUT --> A
```

**읽는 법**: 위쪽(로봇 토픽 → `l1_imu_fix` → Point-LIO)은 실시간
위치추정·주행 경로로 현재도 그대로 씁니다. 가운데 `mapping` 상자가 지도를
만드는 실내 경로입니다 — LIO 를 거치지 않고 녹화된 bag 을 오프라인으로
직접 읽습니다. Point-LIO 가 저장하던 PCD 경로(점선, "보류")는 지우지
않았지만 지도 생성에는 더 이상 쓰지 않습니다.

---

## 1-B. 전체 흐름 — 실외 (2026-08-12 신설)

```mermaid
flowchart TD
    subgraph robotO["실외 녹화 토픽"]
        O1["/utlidar/cloud<br/>15 Hz · 라이다 프레임 · 원시"]
        O2["/utlidar/robot_odom<br/>150 Hz"]
        O3["/sportmodestate<br/>289 Hz"]
        O4["/gnss<br/>1 Hz · std_msgs/String (JSON)<br/>fixed hdop lat lon satellite_inuse"]
        O5["/lowstate · /utlidar/imu"]
    end

    NOTE["⚠ cloud_deskewed 미녹화<br/>실외 bag 에는 없다"]

    CALO["tools/go2_calib.py<br/>R_BL · LEVER"]

    O1 --> V3["tools/odom_map_build_v3.py<br/>lidar→body→odom 직접 변환<br/>+ 고정 축척 k"]
    O2 --> V3
    CALO -.import R_LB→R_BL, LEVER.-> V3
    NOTE -.이유.-> V3

    O1 --> BM["tools/build_maps_0812.py<br/>일괄 + GPS 보정 3방식"]
    O2 --> BM
    O4 --> BM
    V3 -.함수 재사용<br/>cloud_xyz get_extrinsic load_odom.-> BM

    BM --> PN["*_none.pcd<br/>고정 k · odom 프레임"]
    BM --> PR["*_rigid.pcd<br/>GPS 상사변환 1회 · ENU"]
    BM --> PW["*_warp.pcd<br/>rigid + 잔차 평활 σ=30s · ENU"]

    PN --> P2GO["tools/pcd_to_grid.py"]
    PR --> P2GO
    PW --> P2GO
    P2GO --> GO["results/outdoor_0812/grid_15/<br/>0.15 m/cell · 실외 권장"]

    %% ---------- 분석 ----------
    subgraph analysis["분석 · 검증"]
        O2 --> CK["tools/check_0812.py<br/>전원리셋 점프 · 폐루프 오차"]
        O4 --> CK
        O2 --> GA["tools/gps_align_0812.py<br/>GPS↔오도 상사변환 · 축척"]
        O4 --> GA
        O2 --> PT["tools/plot_traj.py<br/>궤적 + GPS 겹쳐 그리기"]
        O4 --> PT
        O4 --> GN["tools/gps_noise_split.py<br/>정지 bag · 빠른떨림/느린흐름 분리"]
    end

    GA --> KO["축척 1.1910 / 1.2327<br/>실내 줄자 1.1995 와 교차검증"]
    GN --> SIG["σ_fast 0.21 m<br/>σ_slow 5.77 m (계단형)<br/>→ GTSAM σ 5 m + Huber"]

    %% ---------- 시각화 ----------
    PN --> CP["tools/compare_pcd.py<br/>점군 한눈에 · top/front/side"]
    PR --> CP
    PW --> CP
    GO --> CM["tools/compare_maps.py<br/>격자 한눈에"]
    PW --> V3D["tools/view3d.py<br/>grid / single / overlay"]

    %% ---------- 미사용 ----------
    O4 -.현재 미사용.-> GB["tools/gnss_bridge.py<br/>JSON → sensor_msgs/NavSatFix<br/>GTSAM 단계에서 필요"]
```

**읽는 법**: 실내와 갈라지는 지점은 **입력 토픽 하나**입니다. 실내는
`cloud_deskewed`(이미 odom 좌표계)를 그대로 쌓지만, 실외 bag 에는 그 토픽이
없어 원시 `/utlidar/cloud`(라이다 프레임)를 `R_BL`·`LEVER` 로 두 번 변환해야
합니다. 그 차이만 `odom_map_build_v3.py` 가 흡수하고, 이후 `pcd_to_grid.py`
부터는 실내와 완전히 같습니다.

GPS 보정(`rigid`/`warp`)은 **폐루프 경로에만** 유효합니다. 편도·직선 구간은
상사변환의 가로 방향 구속이 없어 축척이 GPS 노이즈를 흡수합니다
(slope_back 실측 1.7931 — 발산).

---

## 2. 파일별 의존

### `tools/go2_calib.py` — 외부 파라미터 상수

다른 파일들이 가져다 쓰는 원본입니다. 노드가 아니라 상수 모음입니다.

| 상수 | 값 | 뜻 |
|---|---|---|
| `R_LB` | 3×3 행렬 | 몸통 → 라이다 회전 (164.9° 기울어짐) |
| `R_BL` | `R_LB.T` | 라이다 → 몸통 |
| `LEVER` | (0.322, 0.005, 0.050) | 라이다 위치 (몸통 프레임) [m] |
| `ACC_REST_BODY` | 9.465 | 정지 시 본체 IMU 가속도 크기 |
| `ACC_SCALE_BODY` | 1.03614 | 9.807 / 9.465 |
| `EXPECTED_REST_ACC` | (1.66, −1.90, −9.48) | 정지 시 라이다 프레임 기대 가속도 |

**이 값들을 바꾸면 아래 절의 중복 문제를 먼저 해결해야 합니다.**

### `tools/l1_imu_fix.py`

```python
from go2_calib import R_LB, ACC_SCALE_BODY, EXPECTED_REST_ACC
```

| 받는 것 | `/utlidar/imu` (자이로), `/lowstate` 또는 `/sportmodestate` (가속도) |
| 내는 것 | `/l1_imu_fixed` |
| 의존 | **`go2_calib.py`** |
| 파라미터 | `acc_topic` (기본 `/lowstate`), `frame_id`, `acc_scale`, `rest_check` |

L1 내부 IMU 는 164.9° 기울어 장착돼 있어 그대로 쓰면 중력 방향이 틀어집니다.
**모든 LIO 실험의 전제입니다.**

> **2026-08-12 갱신**: 가속도 출처를 `acc_topic` 파라미터로 뺐습니다.
> `/lowstate`(500 Hz)와 `/sportmodestate`(289 Hz)의 `imu_state.accelerometer`
> 는 **같은 센서**입니다(2026-08-10 확인: |a| 평균 9.4960 vs 9.4947, 축별
> 평균도 일치). bag 에 `/lowstate` 가 없을 때 대체용입니다. WiFi 대역폭이
> 빠듯해 `/lowstate` 를 빼고 녹화한 경우가 여기 해당합니다.

### `mapping_unilidar_l1.launch.py` — 외부 패키지

우리가 만든 것이 아닙니다.

| 위치 | `~/catkin_point_lio_unilidar/src/point_lio_ros2` |
| 노드 이름 | `laserMapping` |
| 소스 | `src/laserMapping.cpp` (C++) |
| 설정 | `config/unilidar_l1.yaml` |

우리가 건드린 것은 설정 세 줄뿐입니다.

```yaml
lid_topic:  "/utlidar/cloud"
imu_topic:  "/l1_imu_fixed"
pcd_save_en: true
```

내는 것: `/aft_mapped_to_init`, `/cloud_registered`, 종료 시 `PCD/scans.pcd`

> ⏸ **2026-08-07 갱신**: 종료 시 저장하는 `PCD/scans.pcd` 는 지도 생성에
> **더 이상 쓰지 않습니다** (같은 bag 기준 드리프트 21.75%, 원인 미규명 —
> 보류). `/aft_mapped_to_init` 을 쓰는 실시간 위치추정 경로는 그대로
> 유효합니다. 이 설정 파일의 diff·이력 전문은 `external/point_lio_config/`
> 에 백업돼 있습니다(`external/README.md`).

> 📌 **미검증 항목**: `extrinsic_T` 가 Unitree 예제값 `[0.0077, 0.0147,
> -0.0067]`(크기 0.018 m)로 남아 있습니다. 그러나 `/l1_imu_fixed` 의 가속도는
> **몸통 IMU 위치**의 값(브리지가 회전만 시키고 평행이동은 안 함)이므로 실제
> IMU→라이다 변위는 **0.327 m**, 약 18배 차이입니다. `extrinsic_R` 은 항등이
> 맞습니다(브리지에서 회전을 이미 맞춤). B 조건에서 `extrinsic_T` 를 바꿔본
> 적은 없습니다. 다만 회차마다 성공/실패가 갈리는 비재현성은 상수 오차로
> 설명되지 않아, 단독 원인일 가능성은 낮습니다.

### `tools/odom_map_build.py` / `odom_map_build_v2.py` — 점군 누적 (실내)

| 받는 것 | 녹화 bag 안의 `/utlidar/cloud_deskewed` (파일, 오프라인) |
| 내는 것 | `PCD/scans.pcd` (voxel 다운샘플, 기본 0.05 m) |
| 의존 | open3d, rosbag2_py, sensor_msgs_py |

LIO 를 거치지 않습니다. `cloud_deskewed` 가 이미 다리 오도메트리와 같은
odom 좌표계이므로 좌표 변환 없이 그대로 누적합니다. bag 재생이 필요 없고
파일을 직접 읽습니다. v2 는 축척 보정 `k` 를 추가한 것입니다.

축척 보정 원리: 오차는 로봇 위치 `t(t)` 에만 있고 회전 `R(t)` 와 스캔
`p_body` 는 참값이므로, `p'(t) = p_odom(t) + (k-1)·t(t)` 로 각 프레임을
밀면 궤적만 k 배가 되고 스캔 자체는 그대로 남습니다.

### `tools/odom_map_build_v3.py` — 점군 누적 (실외, 2026-08-12 신설)

```python
from go2_calib import R_LB, LEVER      # R_BL = R_LB.T
```

| 받는 것 | 녹화 bag 안의 **`/utlidar/cloud`**(원시), `/utlidar/robot_odom` |
| 내는 것 | `scans.pcd` |
| 의존 | **`go2_calib.py`**, open3d, sqlite3 직접 읽기 |

**v2 와의 차이는 입력 토픽 하나입니다.** v2 가 쓰는 `cloud_deskewed` 는 이미
odom 좌표계라 변환이 필요 없지만, `/utlidar/cloud` 는 **라이다 프레임**이라
두 단계를 직접 해야 합니다.

```
p_body = R_BL · p_lidar + LEVER
p_odom = R_odom(t) · p_body + t_odom(t)
```

축척 보정은 v2 와 동일합니다(위치에만 적용).

주요 인자: `voxel`(실외 권장 0.15), `--k`(기본 1.1995), `--min-range` 0.6,
`--max-range` 40, `--stride`, `--max-dt` 0.05, `--invert-extrinsic`.

> ⚠ 원시 `cloud` 는 **모션 보정(deskew)이 안 되어 있습니다.** 회전이 빠르면
> 스캔이 밀립니다. 0812 폐루프는 제자리 회전을 피하고 호를 그리며 돌아
> 영향이 작았습니다(지면 봉우리 단일 유지 확인).

> 📌 `sensor_msgs_py.read_points_numpy` 는 못 씁니다. L1 점군은 `x,y,z`
> (float32)와 `ring`(uint16)의 datatype 이 섞여 있어 assertion 에 걸립니다.
> 버퍼 오프셋에서 직접 뜯는 `cloud_xyz()` 를 씁니다.

### `tools/build_maps_0812.py` — 일괄 + GPS 보정 (2026-08-12 신설)

```python
from odom_map_build_v3 import cloud_xyz, get_extrinsic, load_odom, \
                              nearest_odom, open_bag, quat_to_R, topic_id
```

| 받는 것 | bag 폴더들의 `/utlidar/cloud`, `/utlidar/robot_odom`, `/gnss` |
| 내는 것 | `<bag>_<mode>.pcd` × 방식 수, 그리고 `pcd_to_grid.py` 를 호출해 격자 |
| 의존 | **`odom_map_build_v3.py`**, **`go2_calib.py`**, **`pcd_to_grid.py`**, open3d |

보정 방식 세 가지:

| 방식 | 하는 일 | 좌표계 | 축척 출처 |
|---|---|---|---|
| `none` | 고정 k 만 적용 | odom (부팅 기준) | `--k` 기본 1.1995 |
| `rigid` | GPS ENU 에 상사변환 1회 (yaw·축척·평행이동) | **ENU** | 그 bag 의 GPS |
| `warp` | rigid + 잔차를 σ초 가우시안으로 평활해 더함 | **ENU** | 그 bag 의 GPS |

`rigid` 는 전역 변환 하나뿐이라 GPS 순간 노이즈가 지도에 섞이지 않습니다.
`warp` 는 저주파 성분만 남겨 서서히 쌓인 드리프트를 잡습니다
(loop_correct_v2 의 증분 궤적 변형과 같은 발상).

> ⚠ **`rigid`/`warp` 는 폐루프 전용입니다.** 직선 궤적은 상사변환의 가로
> 방향 구속이 없어 축척이 GPS 노이즈를 흡수합니다. slope_back(편도) 실측
> 축척 1.7931, 잔차 RMS 13.17 m — 발산. 편도 구간은 `none` 을 쓰십시오.

> GPS fix 가 20개 미만이면 `rigid`/`warp` 는 자동으로 건너뜁니다
> (slope_out_1403 은 `fixed:0` 뿐이라 `none` 만 생성됨).

### `tools/gps_align_0812.py` — GPS↔오도메트리 정렬 분석 (2026-08-12)

| 받는 것 | bag 안의 `/sportmodestate`, `/gnss` |
| 내는 것 | 표준출력 — GPS 원시 폐루프 오차, 오도 폐루프 오차, 상사변환 축척·yaw·잔차 |
| 의존 | numpy |

**축척 교차검증의 출처입니다.** 실내 줄자 1.1995 와 독립적으로
1.1910(1440) / 1.2327(1449) 를 얻었습니다.

### `tools/gps_noise_split.py` — GPS 오차 성분 분리 (2026-08-12)

| 받는 것 | **정지** bag 의 `/gnss` (기본 `gps_static_0812_1415`) |
| 내는 것 | σ_fast, σ_slow, σ_total, 권장 GTSAM σ, `--plot` 시 PNG |
| 의존 | numpy, (선택) matplotlib |

정지 상태라 참값이 상수인 점을 이용합니다. 인접 fix 간 차이로 빠른 떨림을,
win 초 이동평균의 이동으로 느린 흐름을 잽니다.

0812 실측(콜드스타트 직후, 위성 4.5, hdop 2.11):

- σ_fast **0.21 m** — 예상보다 훨씬 정밀
- σ_slow **5.77 m** — 연속 표류가 아니라 **계단형 점프 2회**(해 갈아탐)
- → GTSAM GPS factor σ **5 m** 시작 + **Huber 로버스트** + **hdop 가중**

### `tools/check_0812.py` — bag 무결성·폐루프 점검 (2026-08-12)

| 받는 것 | bag 안의 `/sportmodestate`, `/gnss` |
| 내는 것 | 표준출력 — 위치 점프(전원 리셋 흔적), 경로장, 시종점차, 드리프트 %, yaw 차, fixed 분포 |
| 의존 | numpy |

> ⚠ 드리프트 % 는 **폐루프에서만** 의미가 있습니다. 편도 구간(slope_*)의
> 59%·62% 는 무시하십시오.

### `tools/plot_traj.py` — 궤적 시각화 (2026-08-12)

| 받는 것 | bag 안의 `/sportmodestate`, `/gnss` |
| 내는 것 | `<bagdir>/plots/traj_<name>.png`, `traj_all.png` |
| 의존 | numpy, matplotlib |

GPS(ENU)와 오도메트리를 정렬해 겹쳐 그립니다. 파랑이 매끄럽고 주황이 그
주위에서 떠는 형태가 정상입니다.

### `tools/compare_pcd.py` / `compare_maps.py` / `view3d.py` — 비교·조망 (2026-08-12)

| | 받는 것 | 내는 것 |
|---|---|---|
| `compare_pcd.py` | `results/**/*.pcd` | 점군 top/front/side 격자 배치 PNG |
| `compare_maps.py` | `grid*/map.pgm` + `map.yaml` | 격자 나란히 배치 PNG |
| `view3d.py` | `*.pcd` | Open3D 대화형 창 (`grid`/`single`/`overlay`) |

`view3d.py --mode overlay` 가 `none`/`rigid`/`warp` 정렬 비교에 가장
유용합니다.

### `tools/gnss_bridge.py` — JSON → NavSatFix (현재 미사용)

| 받는 것 | `/gnss` (`std_msgs/String`, JSON) |
| 내는 것 | `sensor_msgs/NavSatFix` |

**0812 실외 분석에서는 쓰지 않았습니다.** 위 스크립트들이 bag 에서 JSON 을
직접 파싱합니다. **GTSAM 단계에서 표준 메시지가 필요해지면** 이 브리지를
쓰게 됩니다.

`/gnss` JSON 필드: `fixed`(0/1), `hdop`, `latitude`, `longitude`,
`satellite_total`, `satellite_inuse`, `timestamp`(Unix).

> ⚠ **`fixed:1` 만으로 부족합니다.** 콜드 스타트 중에는 `fixed:0` 이면서
> 마지막으로 잡았던 좌표를 그대로 들고 있습니다(0812 실측: 9일 전 좌표와
> timestamp). `satellite_inuse ≥ 4` 와 **timestamp 갱신**을 함께 확인하십시오.

### `tools/loop_correct_v2.py` — 루프 클로저 (실내)

| 받는 것 | 녹화 bag 안의 `/utlidar/cloud_deskewed`, `/utlidar/robot_odom` |
| 내는 것 | 보정된 `scans.pcd` |
| 의존 | open3d, rosbag2_py, sensor_msgs_py |

증분 재적분 방식. 출발-도착 오차를 걸음 수에 비례해 나눠 배분해 국소
모양을 보존합니다. **ICP `fitness ≥ 0.9` 일 때만 적용** — 낮으면
`odom_map_build.py` 의 무보정 출력을 그대로 씁니다. v1(`loop_correct.py`,
전역 나선 변환)은 회전 중심에서 먼 점을 크게 밀어내는 구조적 결함으로
**폐기**됐습니다(참고용으로만 보관).

> 실외에는 아직 적용하지 않았습니다. 개활지는 정합에 쓸 구조물이 적어
> fitness 0.9 를 넘기 어렵고, 0812 폐루프는 이미 0.31% 라 보정 없이도
> 쓸 만합니다.

### `tools/robot_pose.py`

| 받는 것 | `/aft_mapped_to_init`, `/indoor/health` |
| 내는 것 | `/indoor/base_pose` |
| 의존 | **없음 — 상수를 자체 보유** ⚠ |

`/aft_mapped_to_init` 은 라이다 위치입니다. 몸통 중심은 32 cm 뒤라 회전 시
어긋납니다. `LEVER` 로 보정합니다.

`/indoor/health` 를 받아 `pose.covariance[0]` 에 신뢰도를 싣습니다
(정상 0.01, 이상 1e6).

### `tools/lio_health.py`

| 받는 것 | `/indoor/base_pose`, `/utlidar/robot_odom` |
| 내는 것 | `/indoor/health`, `/indoor/health_info` |
| 의존 | 없음 |

`/utlidar/robot_odom`(다리 오도메트리)을 **기준**으로 씁니다. 장기적으로는
표류하지만(356 m 에 20 m) 단기 속도는 정확하므로 "로봇은 멈춰 있는데 LIO 가
움직인다"를 판정할 수 있습니다.

### `tools/lio_tf.py`

| 받는 것 | `/indoor/base_pose`, `/indoor/health` |
| 내는 것 | `/tf` (`indoor_map → base_link`) |
| 의존 | 없음 |

신뢰도가 나빠지면 TF 발행을 멈춥니다.

### `tools/pcd_to_grid.py`

| 받는 것 | `scans.pcd` (파일) |
| 내는 것 | `<out>.npy`, `.pgm`, `.yaml`, `_preview.png` |
| 의존 | numpy, matplotlib |

토픽을 쓰지 않는 오프라인 도구입니다. 입력 PCD 의 출처가 무엇이든(Point-LIO,
`odom_map_build_v2`, `odom_map_build_v3`) 형식이 같아 도구 자체는 그대로입니다.
2026-08-07 에 지면 추정 버그(최빈 bin 왼쪽 끝 → 중앙)를 고쳤습니다.

> ⚠ **인자 순서 주의**: `pcd_to_grid.py <pcd> <출력이름> <해상도>` 입니다.
> 해상도를 두 번째에 넣으면 그것이 **파일 이름**이 되고 해상도는 기본
> 0.05 로 남습니다(2026-08-12 에 `0.15.pgm` 이 생겨 확인).

> 실외 권장 해상도는 **0.15 m** 입니다. 0.05 로 하면 726만 칸에 장애물이
> 1만 칸(0.1%)뿐이라 지나치게 성깁니다. 0.15 에서는 2.0% 로 올라갑니다.

> 📌 지면 밴드가 전역 상수입니다. 실외는 지면 자체에 기복이 있어
> **셀별 국소 밴드**로 바꾸는 것이 낫습니다(미적용).

### `tools/pcd_view.py`

| 받는 것 | `*.pcd` |
| 내는 것 | `pcd_view_top/front/side/hist.png` (현재 디렉터리) |

> ⚠ **"z 범위 → 발산 의심" 경고는 실내 천장 기준입니다.** 실외는 나무·펜스·
> 건물이 있어 6~7 m 가 정상입니다. 실제 발산은 z 가 수십 m 로 튀고 점이
> 방사형으로 퍼집니다. 지면 추정값이 0.0~0.2 m 로 얇게 잡히면 정상입니다.

### `tools/map_publisher.py`

| 받는 것 | `results/*.yaml` + `*.pgm` (파일) |
| 내는 것 | `/indoor/map` (latched) |
| 의존 | numpy |

### `tools/go2_nav_interface.py`

| 받는 것 | `/indoor/base_pose`, `/utlidar/cloud`, 지도 파일 |
| 내는 것 | `/map`, `/odom`, `/scan`, `/tf` |
| 의존 | **없음 — 상수를 자체 보유** ⚠ |

Nav2 규약 이름으로 내보내는 다리입니다. 정적 TF 두 개(`map→odom`,
`base_link→utlidar_lidar`)도 이 노드가 냅니다.

### `tools/proximity_guard.py`

| 받는 것 | `/utlidar/cloud` (원시) |
| 내는 것 | `/indoor/safe`, `/indoor/obstacle` |
| 의존 | `go2_calib.py` (있으면 씀, 없으면 내장값) |

**LIO 에 의존하지 않습니다.** 위치가 발산해도 정상 동작합니다.

### `tools/run_indoor.sh`

source 하는 것:

```
/opt/ros/humble/setup.bash
~/unitree_ros2/cyclonedds_ws/install/setup.bash
~/catkin_point_lio_unilidar/install/setup.bash
~/setup_go2.sh                    (실시간 모드일 때만)
```

띄우는 것:

```
tools/l1_imu_fix.py
ros2 launch point_lio mapping_unilidar_l1.launch.py
ros2 run tf2_ros static_transform_publisher  ×2
tools/robot_pose.py
tools/lio_health.py
tools/map_publisher.py
tools/lio_tf.py
```

**실시간 위치추정·주행용입니다.** 지도 생성에는 더 이상 쓰지 않습니다.

### `tools/repro_run.sh`

| 인자 | `<실행이름> [간격초] [bag경로] [가속도토픽]` |

> **2026-08-12 갱신**: bag 경로와 `acc_topic` 을 인자로 받도록 바꿨습니다.
> 재생 토픽도 필요한 것만 추립니다 — 전체를 쏟으면 DDS 큐가 넘쳐 유실됩니다
> (2026-08-10 확인).

---

## 3. ⚠ 상수 중복 — 고쳐야 할 것

`R_LB` 와 `LEVER` 가 **여러 곳**에 있습니다.

| 파일 | 방식 |
|---|---|
| `go2_calib.py` | 원본 |
| `l1_imu_fix.py` | `from go2_calib import` ✓ |
| `odom_map_build_v3.py` | `from go2_calib import` ✓ |
| `build_maps_0812.py` | v3 경유 ✓ |
| `proximity_guard.py` | `import go2_calib` (없으면 내장값) ✓ |
| **`robot_pose.py`** | **자체 상수** ✗ |
| **`go2_nav_interface.py`** | **자체 상수** ✗ |

**`go2_calib.py` 를 고쳐도 아래 두 파일에는 반영되지 않습니다.**

라이다를 다시 장착하거나 재교정하면 값이 바뀌는데, 그때 일부만 갱신되어
위치가 조금씩 어긋나기 시작합니다. 증상이 미묘해서 원인 찾기가 어렵습니다.

**해결**: 두 파일이 `go2_calib` 을 import 하도록 바꾸십시오.

```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import R_LB, LEVER
R_BL = R_LB.T
```

### 축척계수 `k` 도 중복입니다

| 출처 | 값 | 방법 |
|---|---|---|
| 실내 줄자 (2026-08-10) | **1.1995** | 타일 51.83 m, 6회, 편차 0.33 m |
| 실외 GPS loop1_1440 | **1.1910** | 상사변환 정렬 |
| 실외 GPS loop1_1449 | **1.2327** | 상사변환 정렬 |

세 방법이 독립적으로 1.19~1.23 을 가리키므로 **다리 오도메트리가 거리를
약 20% 짧게 세는 것은 확정**입니다. 다만 1.19 vs 1.23 의 3.5% 편차는
미해결이며, 폐루프를 더 확보해 수렴점을 찾아야 합니다.

현재 `odom_map_build_v2/v3`, `loop_correct_v2`, `build_maps_0812` 가 각자
기본값 1.1995 를 들고 있습니다. **`go2_calib.py` 로 옮기는 것이 낫습니다.**

> ⚠ **폐루프 시험으로는 축척 오차가 잡히지 않습니다.** 궤적 전체가 같은
> 비율로 줄어들 뿐 모양은 같아 출발점으로 그대로 돌아옵니다. 0812 의 폐루프
> 0.31% 와 축척 20% 오차는 모순이 아니라 서로 다른 것을 재고 있습니다.
> **GPS 웨이포인트 주행에는 축척 쪽이 직접 영향을 줍니다.**

---

## 4. ⚠ 다리 오도메트리 z 는 고도가 아닙니다

`/sportmodestate` 의 `position[2]` 는 **지면 위 몸통 높이**입니다. 걸을 때
일정하게 유지되므로 고도 변화를 반영하지 않습니다.

2026-08-12 실측: 경사로를 246 m 내려가고 184 m 올라왔는데 **네 bag 모두
`dz` 가 ±0.01 m** 였습니다.

- 경사 구간의 z축 오차 측정은 이 방식으로 **불가능**합니다
- 고도차가 있는 코스에서는 지도가 **평면으로** 나옵니다
- 고도가 필요하면 **GPS altitude** 또는 **IMU 적분**을 별도로 붙여야 합니다

---

## 5. 외부 의존

| 대상 | 쓰는 곳 |
|---|---|
| ROS2 Humble | 전부 |
| `unitree_go` 메시지 | `l1_imu_fix.py`(`/lowstate`, `/sportmodestate`), 0812 분석 도구 전부 |
| `point_lio` 패키지 | `run_indoor.sh` |
| `tf2_ros` | `lio_tf.py`, `run_indoor.sh`, `go2_nav_interface.py` |
| numpy | 대부분 |
| scipy | 부풀리기, 연결성 검사 |
| matplotlib | `pcd_to_grid.py` 미리보기, `plot_traj.py`, `compare_*.py`, `gps_noise_split.py --plot` |
| open3d | 점군 누적·루프클로저·`view3d.py`·`compare_pcd.py` (0.19.0, 시스템 python3 에서 동작) |
| rosbag2_py / sensor_msgs_py | bag 오프라인 읽기 — 실내 도구들 |
| **sqlite3 직접 읽기** | **0812 실외 도구 전부** — DDS 를 거치지 않아 `srcoff` 불필요, 로봇이 움직일 위험 없음 |
| `~/setup_go2.sh` | 로봇 연결 (CycloneDDS 설정) |

`setup_go2.sh` 는 저장소 밖(홈 디렉터리)에 있습니다. 새 PC 로 옮기실 때
잊기 쉬우므로 `install_go2_lio.sh` 가 함께 복사합니다.

**`point_lio_ros2`, `go2_description`, `unitree_ros2` 의 수정본은
`external/`(2026-08-08 신설)에 백업돼 있습니다.** diff 전문·복원 절차는
`external/README.md` 참고.

---

## 6. 실외 녹화 규약 (2026-08-12 확립)

```bash
ros2 bag record -o go2_outdoor_$(date +%m%d_%H%M) \
  /utlidar/cloud /utlidar/imu /utlidar/robot_odom \
  /lowstate /gnss /sportmodestate
```

| 항목 | 값 | 근거 |
|---|---|---|
| 연결 | 로봇 위 공유기 + 노트북 WiFi | 유선은 케이블 장력으로 단선(7/30) |
| 대역폭 검증 | 30초 녹화 후 `cloud` Count ≈ 450 | 0812 실측 449/450 = 99.8% |
| GPS 시작 조건 | `fixed:1` + `inuse ≥ 4` + **timestamp 갱신** | 콜드스타트 시 옛 좌표 유지 |
| 경로 | 폐루프, 시작=도착, 방위 일치, **호를 그리며 선회** | 제자리 회전 회피가 0.31% 의 원인 |
| 시작·종료 | 각 10초 정지 | 바이어스 추정 |
| 정지 bag | 매 세션 3분 + 그때의 위성수·hdop 기록 | σ 를 조건의 함수로 알기 위함 |

> ⚠ **전원을 끄면 `/sportmodestate` 의 position 원점이 리셋됩니다.** bag 을
> 이어붙여 대폭루프를 만들 계획이면 중간에 끄지 마십시오. 배터리 교체 시점은
> 반드시 메모하십시오.

> 📌 `cloud_deskewed` 를 녹화 목록에 추가하면 실내 파이프라인(v2)을 실외에도
> 그대로 쓸 수 있습니다. 다음 세션에 검토.

---

## 7. 다음 작업

1. **GTSAM + GPS factor** — σ 5 m, Huber 로버스트, hdop 기반 factor별 가중
2. **축척 재현성** — 폐루프를 더 확보해 1.19~1.23 의 수렴점 확인
3. **정지 3분 bag** 매 세션 + 위성수·hdop 동시 기록
4. **격자 지면 밴드** 전역 → 셀별 국소로 (실외 기복 대응)
5. **루프 안쪽 미관측** — 라이다 40 m 로 중앙 미도달, 경로 설계 또는 보간 필요
6. `k` 를 `go2_calib.py` 로 통합, `robot_pose.py`·`go2_nav_interface.py` import 전환
