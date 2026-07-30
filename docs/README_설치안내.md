# Go2 개발 환경 재설치 템플릿 사용 안내

우분투를 새로 설치한 뒤, Go2 로봇 프로젝트 환경을 한 번에 복원하기 위한 스크립트입니다.
`setup_go2_env.sh` 하나로 아래 항목들을 순서대로 설치합니다.

## 설치되는 것들

| 순서 | 항목 | 내용 |
|------|------|------|
| base | 기본 개발 도구 | build-essential, cmake, git, curl, python3-pip 등 + **한글 폴더명을 영어로 변경** |
| chrome | Google Chrome | 공식 .deb 패키지 |
| vscode | VS Code | Microsoft 공식 apt 저장소 |
| claude | Claude Code | Node.js 22 + npm 전역 설치(sudo 없이) |
| ros2 | ROS2 Humble | Desktop + 개발 도구 + rosdep + colcon |
| sdk | unitree_sdk2 | C++ SDK 빌드 및 /opt/unitree_robotics 에 설치 |
| unitree | unitree_ros2 | Go2 메시지 패키지 + CycloneDDS + setup_go2.sh 생성 |
| python | 인지 파이프라인 | Open3D, YOLO(ultralytics), numpy<2, opencv<4.10 |

## 사용법

### 1단계 — 스크립트 준비
새 우분투에서 터미널을 열고, 이 파일들을 홈 디렉토리에 두신 뒤:

```bash
chmod +x setup_go2_env.sh
```

### 2단계 — 실행

**전체를 한 번에:**
```bash
./setup_go2_env.sh
```

**특정 단계만 (예: ROS2만):**
```bash
./setup_go2_env.sh ros2
```
사용 가능한 인자: `all base chrome vscode claude ros2 sdk unitree python`

### 3단계 — 설치 후 확인
설치가 끝나면 **새 터미널을 열고** 아래로 확인하세요:
```bash
ros2 --version
code --version
claude --version
google-chrome --version
```

## 꼭 확인하실 부분 (수동 수정 필요)

### 1) Go2 네트워크 인터페이스(NIC) 이름
`~/unitree_ros2/setup_go2.sh` 안의 `NetworkInterface name` 값을 실제 환경에 맞게 바꿔야 합니다.
로봇을 이더넷으로 연결한 뒤 아래로 NIC 이름을 확인하세요:
```bash
ip a
```
이전 환경에서는 `enxc0eac369bf02` 였습니다. 새 환경에서는 다를 수 있습니다.

### 2) Go2 사용 시 터미널 관리 (중요)
이전에 겪으셨던 문제를 방지하기 위한 원칙입니다:
- **TurtleBot 환경과 Go2 환경을 같은 .bashrc 에 함께 넣지 마세요.** DDS 충돌이 납니다.
- Go2 작업 시에는 새 터미널에서 아래를 source 해서 쓰세요:
  ```bash
  source ~/unitree_ros2/setup_go2.sh
  ```
- 만약 DDS daemon 이 꼬이면:
  ```bash
  ros2 daemon stop && ros2 daemon start
  ```

### 3) 파이썬 라이브러리 충돌
ROS2 와 numpy 버전이 충돌할 수 있습니다. 인지 파이프라인(Open3D/YOLO)은
가능하면 별도 가상환경(venv)에서 쓰는 것을 권장합니다:
```bash
python3 -m venv ~/go2_venv
source ~/go2_venv/bin/activate
pip install "numpy<2" "opencv-python<4.10" open3d ultralytics
```

## 안전장치

- 이 스크립트는 **디스크를 포맷하거나 rm -rf 로 파일을 지우지 않습니다.**
- .bashrc 는 append(추가)만 하며, 중복 실행해도 같은 줄이 여러 번 안 들어갑니다.
- 이미 clone 된 저장소는 다시 받지 않습니다(중복 방지).
- 따라서 여러 번 실행해도 안전합니다.

## 검증 내역
이 스크립트는 작성 후 아래 검토를 거쳤습니다:
- bash 문법 검사 (`bash -n`) 통과
- shellcheck 정적 분석 통과
- 모든 다운로드 URL 실제 접근성 확인 (HTTP 200)
- unitree_ros2 공식 문서 기준으로 Humble 빌드 절차 정정
  (Humble은 cyclonedds 소스 빌드 불필요 → apt 패키지 사용)
- 위험 패턴(rm -rf, .bashrc 덮어쓰기) 없음 확인
