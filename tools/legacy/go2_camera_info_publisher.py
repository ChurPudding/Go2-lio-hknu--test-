#!/usr/bin/env python3
import argparse, math, subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

DEFAULT_IFACE = "enxc0eac369bf02"
MCAST_ADDR, MCAST_PORT = "230.1.1.1", 1720
WIDTH, HEIGHT = 1280, 720
FRAME_ID = "camera_link"
TOPIC_IMAGE = "/go2/camera/image_raw"
TOPIC_INFO = "/go2/camera/camera_info"
DEFAULT_HFOV_DEG = 100.0
FRAME_BYTES = WIDTH * HEIGHT * 3


def build_pipeline(iface):
    return ["gst-launch-1.0", "-q", "udpsrc", f"address={MCAST_ADDR}",
            f"port={MCAST_PORT}", f"multicast-iface={iface}",
            "!", "application/x-rtp, media=video, encoding-name=H264",
            "!", "rtph264depay", "!", "h264parse", "!", "avdec_h264",
            "!", "videoconvert",
            "!", f"video/x-raw,format=BGR,width={WIDTH},height={HEIGHT}",
            "!", "fdsink", "fd=1"]


def read_exact(stream, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def make_camera_info(hfov_deg):
    hfov = math.radians(hfov_deg)
    fx = (WIDTH / 2.0) / math.tan(hfov / 2.0)
    fy, cx, cy = fx, WIDTH / 2.0, HEIGHT / 2.0
    info = CameraInfo()
    info.width, info.height = WIDTH, HEIGHT
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info, fx


class Go2CameraInfoPublisher(Node):
    def __init__(self, iface, hfov_deg):
        super().__init__("go2_camera_info_publisher")
        self.pub_img = self.create_publisher(Image, TOPIC_IMAGE, 10)
        self.pub_info = self.create_publisher(CameraInfo, TOPIC_INFO, 10)
        self.bridge = CvBridge()
        self.info_msg, fx = make_camera_info(hfov_deg)
        self.get_logger().info(f"내부 파라미터: hfov={hfov_deg:.1f}도 -> fx=fy={fx:.1f}")
        self.get_logger().info(f"GStreamer 시작 (iface={iface})")
        self.proc = subprocess.Popen(build_pipeline(iface),
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL,
                                     bufsize=FRAME_BYTES)
        self.count = 0
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(f"발행 시작 -> {TOPIC_IMAGE}, {TOPIC_INFO}")

    def tick(self):
        raw = read_exact(self.proc.stdout, FRAME_BYTES)
        if raw is None:
            self.get_logger().warn("프레임 수신 중단.")
            return
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
        stamp = self.get_clock().now().to_msg()
        img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img.header.stamp = stamp
        img.header.frame_id = FRAME_ID
        info = self.info_msg
        info.header.stamp = stamp
        info.header.frame_id = FRAME_ID
        self.pub_img.publish(img)
        self.pub_info.publish(info)
        self.count += 1
        if self.count % 90 == 0:
            self.get_logger().info(f"영상+정보 발행 중... {self.count} 프레임")

    def destroy_node(self):
        try:
            self.proc.terminate(); self.proc.wait(timeout=2)
        except Exception:
            try: self.proc.kill()
            except Exception: pass
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default=DEFAULT_IFACE)
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG)
    args = p.parse_args()
    rclpy.init()
    node = Go2CameraInfoPublisher(iface=args.iface, hfov_deg=args.hfov)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
