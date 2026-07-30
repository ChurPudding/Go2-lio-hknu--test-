# 런북 03 — 시연 영상 제작

RViz 화면과 로봇 실물을 녹화해 발표·보고용 자료를 만든다.

---

## 0. 준비

```bash
sudo apt install ffmpeg wmctrl x11-utils
mkdir -p ~/fastlio_ws/videos
```

### 세션 확인 (중요)

```bash
echo $XDG_SESSION_TYPE
```

- `x11` → 아래 스크립트가 창을 지정해 녹화한다
- `wayland` → `x11grab`으로 특정 창을 잡을 수 없다. **로그아웃 후 로그인 화면 톱니바퀴에서 "Ubuntu on Xorg"** 선택. 또는 GNOME 내장 녹화(`Ctrl+Alt+Shift+R`, 화면 전체)로 대체

현재 launch 로그에 `Ignoring XDG_SESSION_TYPE=wayland on Gnome`이 뜨고 있으므로 웨일랜드일 가능성이 높다.

### 녹화 스크립트

```bash
cat > ~/fastlio_ws/tools/rec_rviz.sh << 'EOF'
#!/bin/bash
# 사용법: ./rec_rviz.sh <출력이름> [녹화초]
NAME=${1:-out}; DUR=${2:-120}
OUT=~/fastlio_ws/videos/${NAME}.mp4

WID=$(wmctrl -l | grep -i rviz | head -1 | awk '{print $1}')
if [ -z "$WID" ]; then echo "RViz 창을 못 찾음 (세션이 wayland인지 확인)"; exit 1; fi
GEO=$(xwininfo -id "$WID" | awk '
  /Absolute upper-left X/{x=$4} /Absolute upper-left Y/{y=$4}
  /Width:/{w=$2} /Height:/{h=$2}
  END{printf "%dx%d+%d+%d", w-(w%2), h-(h%2), x, y}')
echo "녹화 $GEO -> $OUT (${DUR}s)"

ffmpeg -y -f x11grab -framerate 30 -video_size ${GEO%%+*} \
  -i ${DISPLAY}+$(echo $GEO | cut -d+ -f2-) -t "$DUR" \
  -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p "$OUT"
echo "완료 $OUT"
EOF
chmod +x ~/fastlio_ws/tools/rec_rviz.sh
```

---

## 1. 공정 비교를 위한 RViz 통일

두 영상의 시점이 다르면 비교가 성립하지 않는다. **먼저 시점을 잡고 config를 저장한 뒤, 모든 실행에서 그 config를 쓴다.**

1. RViz에서 시점 조정 (측면 뷰가 z 처짐을 보기에 좋다)
2. `File → Save Config As` → `~/fastlio_ws/rviz/compare.rviz`
3. 이후 실행 시 그 config 지정

맞춰야 할 항목:

| 항목 | 값 |
|---|---|
| Fixed Frame | `camera_init` |
| 표시 | `Path`, `CloudMap`, `Odometry` (동일하게) |
| 카메라 시점 | 저장된 config 그대로. 실행 중 조작하지 않음 |
| 창 크기 | 동일 (스크립트가 창 크기를 그대로 따름) |

발산하는 실행에서는 궤적이 화면 밖으로 나간다. **이게 대비를 보여주는 장면이므로 시점을 따라가게 조작하지 말 것.**

---

## 2. 영상 A — 발산 순간 (핵심 자료)

`/utlidar/imu` 원본으로 첫 회전 구간만 잡는다. 발산은 **t = 7.79 s**, 첫 회전 R0(6.65~7.94 s) 한가운데에서 시작한다.

yaml을 원본으로 되돌린다.

```bash
SRC=~/catkin_point_lio_unilidar/src/point_lio_ros2/config/unilidar_l1.yaml
cp "$SRC" "$SRC.bak_video"
sed -i 's|imu_topic: "/l1_imu_fixed"|imu_topic: "/utlidar/imu"|' "$SRC"
```

```bash
# [T1] LIO + RViz (브리지 노드 불필요)
ros2 launch point_lio mapping_unilidar_l1.launch.py

# [T2] 녹화
~/fastlio_ws/tools/rec_rviz.sh divergence_raw 20

# [T3] 재생
cd ~/fastlio_ws && ros2 bag play go2_run_full
```

끝나면 yaml을 되돌린다.

```bash
sed -i 's|imu_topic: "/utlidar/imu"|imu_topic: "/l1_imu_fixed"|' "$SRC"
```

---

## 3. 영상 B — 해결본

```bash
# [T1] 브리지
python3 ~/fastlio_ws/tools/l1_imu_fix.py

# [T2] LIO + RViz
ros2 launch point_lio mapping_unilidar_l1.launch.py

# [T3] 녹화 (bag 110초 + 여유)
~/fastlio_ws/tools/rec_rviz.sh fixed_full 125

# [T4] 재생
cd ~/fastlio_ws && ros2 bag play go2_run_full
```

---

## 4. 영상 — 알고리즘 비교

**실험 3·4가 끝난 뒤에 만든다.** 둘 다 `/l1_imu_fixed` 기준이어야 한다.

```bash
~/fastlio_ws/tools/rec_rviz.sh fastlio_fixed 125     # FAST-LIO launch로
~/fastlio_ws/tools/rec_rviz.sh pointlio_fixed 125    # Point-LIO launch로
```

깨진 IMU 상태의 두 알고리즘을 나란히 놓는 것은 **의미가 없다.** 둘 다 완전 실패라 비교 대상이 아니다 (진단정리 §7 참조). 대신 각각의 **실패 방식**을 보여주려면 로그(FAST-LIO의 "No Effective Points")를 캡처하는 편이 낫다.

---

## 5. 영상 D — 실시간 주행

화면만 찍으면 로봇의 실제 움직임이 안 보인다. **두 개를 동시에 찍는다.**

- **화면** — `rec_rviz.sh`
- **실물** — 스마트폰 삼각대 고정 촬영. 로봇 전체 이동 범위가 프레임에 들어오게

```bash
# 브리지 + LIO + robot_pose + goto 노드를 모두 띄운 상태에서
~/fastlio_ws/tools/rec_rviz.sh realtime_goto 90
# 녹화 시작 후 RViz에서 2D Goal Pose 클릭
```

> **안전:** 녹화한다고 리모컨에서 손을 떼지 말 것. 촬영 담당은 반드시 별도로 둔다. 2인 이상.

시작 신호를 맞추려면 촬영 시작 시 손뼉을 치면 후처리에서 동기화하기 쉽다.

---

## 6. 후처리

### 좌우 비교

```bash
cd ~/fastlio_ws/videos
ffmpeg -i divergence_raw.mp4 -i fixed_full.mp4 \
  -filter_complex "[0:v]scale=960:-2,drawtext=text='/utlidar/imu (raw)':x=20:y=20:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.5[l];
                   [1:v]scale=960:-2,drawtext=text='/l1_imu_fixed':x=20:y=20:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.5[r];
                   [l][r]hstack" \
  -c:v libx264 -crf 22 -pix_fmt yuv420p compare.mp4
```

길이가 다르면 짧은 쪽 기준으로 잘린다. 같은 길이로 맞추려면 양쪽을 `-t 20`으로 자른 뒤 붙인다.

### 배속·축소

```bash
ffmpeg -i fixed_full.mp4 -vf "setpts=0.25*PTS,scale=1280:-2" -an fixed_4x.mp4
```

### 발표용 GIF (발산 구간만)

```bash
ffmpeg -i divergence_raw.mp4 -ss 5 -t 6 -vf "fps=12,scale=800:-1" divergence.gif
```

t = 5~11 s 구간에 첫 회전과 발산이 모두 들어간다. **6초 안에 원인을 보여주는 자료**가 된다.

### 자막·시각 표시

```bash
ffmpeg -i divergence_raw.mp4 \
  -vf "drawtext=text='t=%{pts\\:hms}':x=w-260:y=h-50:fontsize=28:fontcolor=yellow" \
  -c:v libx264 -crf 22 divergence_t.mp4
```

---

## 7. 시연 세트 권장 구성

| 영상 | 내용 | 보여주는 것 |
|---|---|---|
| **A** | `/utlidar/imu`, 첫 회전 6초 | 발산 순간 |
| **B** | `/l1_imu_fixed`, 전체 110초 | 해결 |
| **C** | A와 B 좌우 비교 | 대비 |
| **D** | 실시간 주행 (화면 + 실물) | 실제 동작 |

**A와 B를 나란히 놓는 것이 알고리즘 비교보다 임팩트가 크다.** 원인 규명과 해결이 한 화면에 담기기 때문이다. 알고리즘 비교 영상은 `/l1_imu_fixed` 기준으로 재실행한 뒤에 만든다.

발표 순서 제안: C(대비) → A 확대(원인) → 진단 근거 표 → B(해결) → D(실주행).
