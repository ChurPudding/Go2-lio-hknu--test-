#!/usr/bin/env python3
import argparse, subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEFAULT_IFACE = "enxc0eac369bf02"
MCAST_ADDR, MCAST_PORT = "230.1.1.1", 1720
WIDTH, HEIGHT = 1280, 720
FRAME_ID, TOPIC = "camera_link", "/go2/camera/image_raw"
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


class Go2CameraPublisher(Node):
    def __init__(self, iface):
        super().__init__("go2_camera_publisher")
        self.pub = self.create_publisher(Image, TOPIC, 10)
        self.bridge = CvBridge()
        self.get_logger().info(f"GStreamer 시작 (iface={iface})")
        self.proc = subprocess.Popen(build_pipeline(iface),
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL,
                                     bufsize=FRAME_BYTES)
        self.count = 0
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(f"발행 시작 -> {TOPIC}")

    def tick(self):
        raw = read_exact(self.proc.stdout, FRAME_BYTES)
        if raw is None:
            self.get_logger().warn("프레임 수신 중단.")
            return
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        self.pub.publish(msg)
        self.count += 1
        if self.count % 90 == 0:
            self.get_logger().info(f"영상 발행 중... {self.count} 프레임")

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
    args = p.parse_args()
    rclpy.init()
    node = Go2CameraPublisher(iface=args.iface)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
