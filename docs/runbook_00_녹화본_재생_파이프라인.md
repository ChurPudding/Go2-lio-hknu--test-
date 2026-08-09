# 런북 00 — 녹화본 재생 파이프라인

로봇에서 bag을 뜨고, 그 bag을 LIO에 통과시키고, 결과를 `robot_odom` 기준선과 정량 비교하는 전 과정.

**이 파이프라인의 핵심은 재현성이다.** 같은 bag을 쓰면 설정 하나만 바꿔가며 공정하게 비교할 수 있다. 로봇에서 매번 새로 걸으면 보행 경로가 달라져 비교가 성립하지 않는다.

---

## 0. 왜 `robot_odom`이 참값인가

`/utlidar/robot_odom`의 **z만** 참값으로 쓴다.

- **z는 적분이 아니다.** 발이 지면에 닿아 있을 때 관절각으로 몸통 높이를 직접 계산하므로 매 스텝마다 지면이 z를 재고정한다. 누적 오차가 생길 수 없다.
- **x·y·yaw는 적분이다.** 다리 운동학을 누적하므로 시간에 비례해 드리프트한다. 절대 위치의 참값으로 쓰면 안 된다.

`go2_run_full` 기준 실측:

| 항목 | 값 |
|---|---|
| z 표준편차 | 5.1 mm |
| z peak-to-peak | 31.4 mm (110초) |
| 시작 z → 끝 z | 차이 **2.0 mm** |
| roll / pitch 평균 | −0.76° / +0.44° (중력 정렬됨) |

따라서:

- **세로 정확도** → `robot_odom` z와 직접 비교 (허용 ±20 mm)
- **수평 정확도** → 루프 클로저로 판정. `go2_run_full`은 32.5 m를 걷고 출발점 0.56 m 이내로 복귀하는 닫힌 루프다

---

## 1. 로봇에서 bag 뜨기

새 데이터셋이 필요할 때만. 기존 `go2_run_full`을 계속 쓰는 것이 비교에는 유리하다.

```bash
source ~/setup_go2.sh
srclio
```

토픽 확인:

```bash
ros2 topic hz /utlidar/cloud   # 15Hz
ros2 topic hz /utlidar/imu     # 250Hz
ros2 topic hz /lowstate        # 500Hz
```

녹화:

```bash
cd ~/fastlio_ws
ros2 bag record -o go2_run_$(date +%m%d_%H%M) \
  /utlidar/cloud /utlidar/imu /utlidar/robot_odom \
  /lowstate /sportmodestate /lf/sportmodestate /multiplestate
```

`/lowstate`는 **반드시 포함한다.** 본체 IMU가 여기 들어 있고, `/l1_imu_fixed` 생성에 필수다.

### 좋은 bag의 조건

| 조건 | 이유 |
|---|---|
| **시작·끝 각 5초 정지** | 중력 초기화, 그리고 오프라인 검증 시 정지 구간 통계 |
| **출발점으로 복귀 (닫힌 루프)** | 루프 클로저로 수평 정확도 판정 가능 |
| **회전 구간 다수 포함** | 회전이 LIO의 약점. 진단 가치가 높다 |
| **평지, 구조물 있는 실내** | 지면 평면과 벽으로 좌표계 검증 가능 |
| **2분 이내** | 길면 재생·분석이 느려지고 드리프트가 원인 분리를 방해 |

`go2_run_full` 기준: 110초, 32.9 m, 누적 회전 2281°(6.3바퀴), 최대 각속도 165°/s. 회전 비중이 커서 진단용으로 우수하다.

정지 구간을 빠뜨렸다면 그 bag은 진단용으로 쓰기 어렵다. 다시 뜨는 편이 빠르다.

---

## 1.5 재생 실험의 필수 조건 (2026-07-30 추가)

**이 두 가지를 지키지 않으면 결과가 재현되지 않는다.** 실측으로 확인했다.

### (1) `CYCLONEDDS_URI` 를 해제할 것

`setup_go2.sh` 는 로봇 통신을 위해 USB-이더넷 인터페이스 하나로 DDS 를 묶는다.

```
<NetworkInterface name="enxc0eac369bf02" priority="default" multicast="default"/>
```

**로컬 bag 재생에는 이 설정이 해롭다.** 로봇이 연결돼 있지 않으면 프로세스 간
발견·전달이 제대로 되지 않아 메시지가 유실된다. 심하면 재생 프로세스는 살아 있는데
구독자가 아무것도 못 받는다(실제로 겪음: `/lowstate 미수신 -> 발행 0`).

오프라인 전용 별칭을 만들어 쓸 것.

```bash
echo "alias srcoff='source ~/unitree_ros2/cyclonedds_ws/install/setup.bash && source ~/catkin_point_lio_unilidar/install/setup.bash && unset CYCLONEDDS_URI'" >> ~/.bashrc
source ~/.bashrc
```

- 오프라인 재생 → `srcoff`
- 로봇 연결 → `source ~/setup_go2.sh`

확인:

```bash
echo "[$CYCLONEDDS_URI]"     # 빈 대괄호여야 함
```

### (2) 반속 재생 `-r 0.5`

실시간 재생에서는 Point-LIO 가 못 따라가는 구간이 생겨 입력이 유실된다.
반속으로 바꾸면 z 정확도와 재현성이 크게 개선된다.

| 지표 | 실시간 (5회) | 반속 (3회) |
|---|---|---|
| z RMSE | 25.1 ~ 32.3 mm | **17.0 ± 0.3 mm** |
| z 최대오차 | 73.3 ~ 81.1 mm | **50.4 ± 1.6 mm** |

### (3) 반복 실행 필수

설정당 **최소 3회**. 단일 실행 비교는 무의미하다.

같은 설정 반복 시 실제 관측된 변동:

| 지표 | 반속 3회 | 변동 | 판정 가능성 |
|---|---|---|---|
| **z RMSE** | 17.0 ± 0.3 mm | 2% | **매우 높음** |
| z 최대오차 | 50.4 ± 1.6 mm | 3% | 높음 |
| 최종 z 오차 | −31.3 ± 4.7 mm | 15% | 보통 |
| 루프 클로저 | 0.785 ± 0.327 m | **42%** | 낮음 |
| 총거리 | 48.46 ± 0.63 m | 1% | 해석 주의 |

**주 지표는 z RMSE 로 확정한다.** 루프 클로저는 참고용이며, 20% 미만의 차이는
구분할 수 없다.

### 왜 z 는 재현되고 수평은 안 되는가

관측성의 차이다.

| | 제약하는 것 | 결과 |
|---|---|---|
| z (수직) | 지면 평면이 매 프레임 수천 점으로 **강하게** 고정 | 변동 2% |
| 수평 (요) | 벽만 제약하며 **간헐적**. 평면 지면은 요를 전혀 제약하지 못함 | 변동 42% |

이 데이터는 최대 사거리 2.4~5.6 m 에 대부분 지면이다. 벽이 시야에서 사라지는
구간에서 요가 표류하고, 표류 시점이 입력 도착 타이밍에 따라 달라지면서 궤적
전체가 갈린다. 33 m 를 걸으며 작은 방향 오차가 증폭된다.

**이건 알고리즘 결함이 아니라 정상적인 SLAM 거동이다.** 관측 정보가 없는 자유도는
추정할 수 없고, FAST-LIO 든 Point-LIO 든 같은 한계를 가진다. 그래서 관측성이 좋은
z 를 비교 지표로 쓴다.

실외 개활지에서는 벽조차 없어 이 문제가 더 심해진다. **GPS 가 요와 수평 위치를
잡아줘야 하는 이유가 바로 이것이다.**

---

## 2. 재생 및 결과 녹화

터미널 4개. 각각 **`srcoff`** 먼저 (§1.5 참조).

```bash
# [T1] IMU 브리지
python3 ~/fastlio_ws/tools/l1_imu_fix.py
```

```bash
# [T2] LIO
ros2 launch point_lio mapping_unilidar_l1.launch.py
```

`incompatible QoS` 경고가 없어야 한다. 나오면 브리지 퍼블리셔가 BEST_EFFORT인 것이다.

```bash
# [T3] 결과 녹화 — "Recording..." 확인 후 잠깐 대기
ros2 bag record -o ~/fastlio_ws/out_<실험이름> /aft_mapped_to_init
```

```bash
# [T4] 재생 — 마지막에 시작
cd ~/fastlio_ws && ros2 bag play ~/data/bags/indoor/go2_run_full -r 0.5
```

**종료 순서: T4 → T3 → T2 → T1.** 녹화를 먼저 닫아야 bag이 온전히 마감된다.

### 반드시 지킬 것

1. **재생마다 LIO 노드를 새로 띄운다.** 중력 초기화가 초반 정지 구간에서 한 번만 이뤄지므로, 노드를 켠 채 bag만 다시 재생하면 안 된다.
2. **record를 재생보다 먼저 시작한다.** 순서가 바뀌면 메시지 0개로 녹화된다.
3. 녹화 직후 확인:

```bash
ros2 bag info ~/fastlio_ws/out_<실험이름>
```

`Count`가 0이면 실패다. 다시 한다.

### 재생 옵션

| 상황 | 옵션 |
|---|---|
| PC가 못 따라감 (메시지 드롭) | `ros2 bag play ~/data/bags/indoor/go2_run_full -r 0.5` |
| 특정 구간만 보기 | `--start-offset 15 --playback-duration 10` |
| 반복 | `--loop` (초기화 문제로 진단에는 부적합) |

---

## 3. CSV 추출

`~/fastlio_ws/dump_odom.py`:

```python
import sqlite3, struct, sys, csv

def parse(buf):
    le = buf[1] == 1
    p = [4]
    def align(n):
        o = (p[0]-4) % n
        if o: p[0] += n-o
    def u32():
        align(4); v = struct.unpack_from('<I' if le else '>I', buf, p[0])[0]; p[0]+=4; return v
    def i32():
        align(4); v = struct.unpack_from('<i' if le else '>i', buf, p[0])[0]; p[0]+=4; return v
    def f64n(n):
        align(8); v = struct.unpack_from(('<' if le else '>')+str(n)+'d', buf, p[0]); p[0]+=8*n; return v
    def s():
        n = u32(); v = buf[p[0]:p[0]+n-1].decode('utf-8','replace'); p[0]+=n; return v
    sec = i32(); nsec = u32(); s(); s()
    pos = f64n(3); quat = f64n(4)
    return sec + nsec*1e-9, pos, quat

db, out = sys.argv[1], sys.argv[2]
topic = sys.argv[3] if len(sys.argv) > 3 else '/aft_mapped_to_init'
con = sqlite3.connect(db)
row = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
if row is None:
    sys.exit('토픽 %s 없음. ros2 bag info 로 확인하세요.' % topic)
with open(out, 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t','x','y','z','qx','qy','qz','qw'])
    n = 0
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
        t, pos, q = parse(blob)
        w.writerow([f'{t:.6f}', *[f'{v:.6f}' for v in pos], *[f'{v:.6f}' for v in q]]); n += 1
print(out, n, 'rows')
```

실행:

```bash
python3 ~/fastlio_ws/dump_odom.py \
  ~/fastlio_ws/out_<실험이름>/out_<실험이름>_0.db3 \
  ~/fastlio_ws/<실험이름>.csv
```

FAST-LIO 등 토픽명이 다르면 세 번째 인자로 준다.

```bash
python3 ~/fastlio_ws/dump_odom.py <db3> <csv> /Odometry
```

> **왜 sqlite3 직접 파싱인가:** `.db3`는 그냥 SQLite다. `rosbag2_py`를 쓰면 ROS2 환경과 메시지 패키지가 필요하지만, `nav_msgs/Odometry`처럼 가변 필드가 문자열뿐인 메시지는 CDR을 손으로 푸는 편이 빠르고 의존성이 없다. CDR 규칙: 앞 4바이트 encapsulation 헤더, 이후 각 원시 타입은 자기 크기에 맞춰 정렬, 문자열은 `uint32 길이 + 바이트(널 포함)`.

---

## 4. 평가

`eval_lio.py`를 `~/fastlio_ws/tools/`에 둔다. 원본 bag에서 참값을 읽고 결과 CSV들을 한 번에 비교한다.

```bash
python3 ~/fastlio_ws/tools/eval_lio.py \
  ~/data/bags/indoor/go2_run_full/go2_run_full_0.db3 \
  ~/fastlio_ws/expA.csv \
  ~/fastlio_ws/expB.csv
```

회전 구간은 odom yaw에서 자동 검출하므로 새 bag에도 그대로 쓸 수 있다.

### 출력 예시

```
==========================================================================
기준선 robot_odom : 16471 msgs, 110.2 s
  수평 총거리 32.53 m | 루프클로저 0.556 m | z p-p 31.4 mm | 최종 z -2.0 mm
  검출된 회전 구간 15개
==========================================================================

[imufix2.csv]  936164 rows, 8523 Hz
  수평 총거리      46.86 m   (참값 32.53, +44%)
  루프 클로저      1.223 m   (참값 0.556)
  z RMSE            25.5 mm
  z 최대오차        73.3 mm
  최종 z 오차      -30.2 mm   (참값 -2.0)
  회전 구간별 z 변화 (참값 / 결과 / 차이, mm)
    T0    6.58~  8.03 s     +7.5 /   +12.6 /    +5.1
    ...
    T11  61.04~ 62.00 s     +8.3 /   -34.4 /   -42.7  <<<
  수평 위치오차 (yaw +168.7° 정렬 후) RMS 0.660 m, 최대 1.452 m
```

`<<<`는 회전 구간 z 오차가 30 mm를 넘은 곳이다. 여기가 개선 대상이다.

### 판정 기준

| 지표 | 정상 | 경계 | 실패 |
|---|---|---|---|
| z RMSE | < 30 mm | 30~100 mm | > 100 mm |
| 최종 z 오차 | < 30 mm | 30~100 mm | m 단위 이상 → **발산** |
| 루프 클로저 | < 0.8 m | 0.8~2 m | > 5 m |
| 수평 총거리 | 참값의 ±15% | ±15~50% | 배수 이상 |

**발산 판별:** 최종 z 오차가 m 단위를 넘으면 궤적이 터진 것이다. 이때는 세부 지표를 볼 필요 없이 원인부터 찾는다. 발산 시각을 확인하려면:

```bash
python3 -c "
import csv,sys,math
t0=None
for r in csv.DictReader(open(sys.argv[1])):
    t=float(r['t']); t0=t0 or t
    d=math.dist([float(r['x']),float(r['y']),float(r['z'])],[0,0,0])
    if d>1.0: print('|pos|>1m at t=%.2f s'%(t-t0)); break
" ~/fastlio_ws/<실험이름>.csv
```

정지 구간(보통 초반 5초) 이후 **첫 회전 시각과 일치하면 IMU 프레임/부호 문제**를 의심한다. 이번 진단에서 정확히 그랬다 (t=7.79 s = 첫 회전 R0 한가운데).

---

## 5. 참고 — 이 파이프라인으로 확인된 것

같은 bag에 설정만 바꿔가며 비교한 결과다.

| 설정 | 최종 z 오차 | 수평 총거리 | 루프 클로저 |
|---|---|---|---|
| `/utlidar/imu`, `acc_norm` 9.81 | −15,911,110 mm | 26,812 m | 26,668 m |
| `/utlidar/imu`, `acc_norm` 10.41 | −15,338,000 mm | 16,976 m | 16,334 m |
| **`/l1_imu_fixed`** | **−30 mm** | **46.9 m** | **1.22 m** |
| (참값) | −2.0 mm | 32.5 m | 0.556 m |

`acc_norm`을 6% 고쳐도 아무 변화가 없다가, IMU 소스를 바꾸자 오차가 킬로미터에서 밀리미터로 떨어졌다. **같은 bag으로 반복 비교했기 때문에 이 결론이 성립한다.** 매번 새로 걸었다면 "이번엔 좀 나았다" 수준에서 끝났을 것이다.

---

## 6. 체크리스트

재생 실험 하나당:

- [ ] `srclio` (모든 터미널)
- [ ] 브리지 노드 실행, `/l1_imu_fixed` 250 Hz 확인
- [ ] 정지 구간 가속도 (x≈+1.6, y≈−1.8, z≈−9.5) 확인
- [ ] LIO 노드 **새로** 실행, QoS 경고 없음 확인
- [ ] record 시작 → "Recording..." 확인
- [ ] bag play 시작
- [ ] 재생 완료 후 T4 → T3 → T2 → T1 순 종료
- [ ] `ros2 bag info` 로 Count ≠ 0 확인
- [ ] `dump_odom.py` → `wc -l` 로 행 수 확인
- [ ] `eval_lio.py` 실행, 결과 기록
- [ ] 바꾼 설정 한 줄로 메모 (**한 번에 하나만 바꿀 것**)
