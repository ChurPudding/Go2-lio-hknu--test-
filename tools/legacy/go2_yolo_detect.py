#!/usr/bin/env python3
"""
Go2 전면 카메라 실시간 객체 인식 (갈래 1: GStreamer 직접 -> YOLO)

동작 원리:
  1) GStreamer가 Go2의 H.264 멀티캐스트 영상을 디코딩해 원시 BGR 프레임을 stdout으로 흘려보냄
  2) 파이썬이 그 파이프를 프레임 크기(W*H*3 바이트)만큼씩 읽어 numpy 배열로 재구성
  3) 재구성한 프레임을 YOLOv8n(CPU)에 넣어 객체 탐지
  4) 화면에 박스/라벨을 실시간 표시 + 터미널에 탐지 로그 출력

OpenCV가 GStreamer 지원 없이 빌드된 환경(GStreamer: NO)에서도 동작합니다.
종료: 영상 창에서 q 키 (또는 터미널에서 Ctrl+C)
"""

import subprocess
import sys
import numpy as np
import cv2
from ultralytics import YOLO

# ----------------------------------------------------------------------
# 설정 (필요 시 이 부분만 수정)
# ----------------------------------------------------------------------
IFACE = None                # None이면 자동 감지. 직접 지정하려면 "enxc0eac369bf02" 처럼 넣으세요.
MCAST_ADDR = "230.1.1.1"    # Go2 카메라 멀티캐스트 주소
MCAST_PORT = 1720           # Go2 카메라 멀티캐스트 포트
WIDTH, HEIGHT = 1280, 720   # Go2 전면 카메라 해상도
MODEL_PATH = "yolov8n.pt"   # YOLO 모델 (홈 디렉터리에 있으면 파일명만으로 OK)
CONF_THRES = 0.4            # 이 신뢰도 미만은 무시 (필요 시 조절)
# ----------------------------------------------------------------------

FRAME_BYTES = WIDTH * HEIGHT * 3  # BGR 한 프레임의 바이트 수


def detect_iface():
    """USB-이더넷 인터페이스(enx*)를 자동으로 찾아 반환. 없으면 None."""
    try:
        out = subprocess.check_output(["ip", "-o", "link", "show"], text=True)
    except Exception:
        return None
    for line in out.splitlines():
        # "2: enxc0eac369bf02: <BROADCAST,...>" 형태에서 이름만 추출
        parts = line.split(": ")
        if len(parts) >= 2:
            name = parts[1].split("@")[0].strip()
            if name.startswith("enx"):
                return name
    return None


def build_pipeline(iface):
    """GStreamer 파이프라인 명령을 조립.
       udpsrc(멀티캐스트 수신) -> RTP 해제 -> h264 파싱 -> 디코딩 -> BGR 변환 -> stdout(fdsink)"""
    return [
        "gst-launch-1.0", "-q",
        "udpsrc", f"address={MCAST_ADDR}", f"port={MCAST_PORT}",
        f"multicast-iface={iface}",
        "!", "application/x-rtp, media=video, encoding-name=H264",
        "!", "rtph264depay",
        "!", "h264parse",
        "!", "avdec_h264",
        "!", "videoconvert",
        "!", f"video/x-raw,format=BGR,width={WIDTH},height={HEIGHT}",
        "!", "fdsink", "fd=1",   # fd=1 = stdout 으로 원시 프레임 출력
    ]


def main():
    # 1) 인터페이스 결정 (수동 지정 우선, 없으면 자동 감지)
    iface = IFACE or detect_iface()
    if not iface:
        print("[ERROR] USB-이더넷 인터페이스(enx*)를 찾지 못했습니다.")
        print("        스크립트 상단의 IFACE에 직접 이름을 넣어 주세요.")
        print("        (터미널에서 'ip -o link show | grep enx' 로 확인 가능)")
        sys.exit(1)
    print(f"[INFO] 사용할 인터페이스: {iface}")

    # 2) YOLO 모델 로딩
    print("[INFO] YOLO 모델 로딩 중...")
    model = YOLO(MODEL_PATH)
    names = model.names  # 클래스 번호 -> 이름 매핑
    print("[INFO] 모델 로딩 완료. GStreamer 스트림을 시작합니다.")

    # 3) GStreamer를 하위 프로세스로 실행하고 stdout 파이프를 연결
    proc = subprocess.Popen(
        build_pipeline(iface),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=FRAME_BYTES,
    )

    frame_idx = 0
    try:
        while True:
            # 파이프에서 정확히 한 프레임 분량을 읽음 (부족하면 반복해서 채움)
            raw = b""
            while len(raw) < FRAME_BYTES:
                chunk = proc.stdout.read(FRAME_BYTES - len(raw))
                if not chunk:
                    break
                raw += chunk
            if len(raw) < FRAME_BYTES:
                print("[WARN] 프레임 수신 중단 또는 불완전. 스트림을 확인하세요.")
                break

            # 바이트열 -> numpy 배열 -> (H, W, 3) 이미지로 재구성 (그리기 위해 복사)
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
            frame_idx += 1

            # 4) YOLO 추론 (CPU)
            results = model(frame, conf=CONF_THRES, verbose=False)
            r = results[0]

            # 5) 박스/라벨 그리기 + 로그 수집
            detected = []
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected.append(f"{label}({conf:.2f})")

            # 6) 터미널 로그 (탐지된 게 있을 때만)
            if detected:
                print(f"[frame {frame_idx}] {len(detected)}개 탐지: " + ", ".join(detected))

            # 7) 화면 표시
            cv2.imshow("Go2 YOLO Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료(Ctrl+C).")
    finally:
        proc.terminate()
        cv2.destroyAllWindows()
        print("[INFO] 정리 완료. 종료합니다.")


if __name__ == "__main__":
    main()
