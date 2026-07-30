#!/usr/bin/env python3
import os
import yaml
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class DualV4L2SyncPub(Node):
    """
    Read two V4L2 devices (/dev/video10 and /dev/video11), publish:
      - /stereo/left/image_raw
      - /stereo/right/image_raw
      - /stereo/left/camera_info
      - /stereo/right/camera_info

    For each left-right pair, all four messages share the EXACT same timestamp.
    """

    def __init__(self):
        super().__init__('dual_v4l2_sync_pub')

        # -------- Parameters --------
        self.declare_parameter('left_device', '/dev/video10')
        self.declare_parameter('right_device', '/dev/video11')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        # v4l2loopback 常见是 YUYV/YUY2；也可能是 MJPG（看你管线写进去的 caps）
        self.declare_parameter('fourcc', 'YUYV')  # 'YUYV' or 'MJPG'

        self.declare_parameter('left_frame_id', 'stereo_left')
        self.declare_parameter('right_frame_id', 'stereo_right')

        # 标定 YAML（camera_calibration 输出的 yaml）
        self.declare_parameter('left_calib_yaml', '')
        self.declare_parameter('right_calib_yaml', '')

        # 发布话题
        self.declare_parameter('left_image_topic', '/stereo/left/image_raw')
        self.declare_parameter('right_image_topic', '/stereo/right/image_raw')
        self.declare_parameter('left_info_topic', '/stereo/left/camera_info')
        self.declare_parameter('right_info_topic', '/stereo/right/camera_info')

        # 减少延迟：处理跟不上时先 grab 丢掉旧帧，尽量拿最新
        self.declare_parameter('drop_old_frames', True)
        self.declare_parameter('grab_warmup', 2)  # 每次 tick 前各 grab N 次

        # -------- Read Parameters --------
        self.left_dev = self.get_parameter('left_device').value
        self.right_dev = self.get_parameter('right_device').value
        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value)

        self.left_frame_id = str(self.get_parameter('left_frame_id').value)
        self.right_frame_id = str(self.get_parameter('right_frame_id').value)

        self.left_yaml = str(self.get_parameter('left_calib_yaml').value)
        self.right_yaml = str(self.get_parameter('right_calib_yaml').value)

        self.left_image_topic = str(self.get_parameter('left_image_topic').value)
        self.right_image_topic = str(self.get_parameter('right_image_topic').value)
        self.left_info_topic = str(self.get_parameter('left_info_topic').value)
        self.right_info_topic = str(self.get_parameter('right_info_topic').value)

        self.drop_old = bool(self.get_parameter('drop_old_frames').value)
        self.grab_warmup = int(self.get_parameter('grab_warmup').value)

        # -------- Publishers --------
        self.bridge = CvBridge()
        self.pub_l_img = self.create_publisher(Image, self.left_image_topic, 10)
        self.pub_r_img = self.create_publisher(Image, self.right_image_topic, 10)
        self.pub_l_info = self.create_publisher(CameraInfo, self.left_info_topic, 10)
        self.pub_r_info = self.create_publisher(CameraInfo, self.right_info_topic, 10)

        # -------- Load camera_info from YAML --------
        self.l_info = self._load_camera_info_yaml(self.left_yaml, self.left_frame_id, "left")
        self.r_info = self._load_camera_info_yaml(self.right_yaml, self.right_frame_id, "right")

        # -------- Open V4L2 devices --------
        self.cap_l = self._open_cap(self.left_dev)
        self.cap_r = self._open_cap(self.right_dev)

        # -------- Timer --------
        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"dual_v4l2_sync_pub started:\n"
            f"  left={self.left_dev} right={self.right_dev}\n"
            f"  size={self.W}x{self.H} fps={self.fps} fourcc={self.fourcc}\n"
            f"  topics: {self.left_image_topic}, {self.right_image_topic}"
        )

    def _open_cap(self, dev: str) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open {dev}")

        # 尝试设置（有些设备不一定完全遵守）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        if self.fourcc:
            try:
                fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception:
                pass

        return cap

    def _load_camera_info_yaml(self, yaml_path: str, frame_id: str, tag: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id

        if not yaml_path:
            self.get_logger().warn(f"[{tag}] left/right_calib_yaml not set -> publishing empty CameraInfo")
            return msg
        if not os.path.exists(yaml_path):
            self.get_logger().warn(f"[{tag}] calib yaml not found: {yaml_path} -> publishing empty CameraInfo")
            return msg

        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)

            msg.width = int(data.get("image_width", 0))
            msg.height = int(data.get("image_height", 0))
            msg.distortion_model = data.get("distortion_model", "")

            def _mat(key: str, n: int):
                m = data.get(key, {})
                vals = m.get("data", [])
                vals = [float(x) for x in vals]
                if len(vals) != n:
                    self.get_logger().warn(f"[{tag}] {key}.data len={len(vals)} != {n} (yaml={yaml_path})")
                    return [0.0] * n
                return vals

            msg.d = [float(x) for x in data.get("distortion_coefficients", {}).get("data", [])]
            msg.k = _mat("camera_matrix", 9)
            msg.r = _mat("rectification_matrix", 9)
            msg.p = _mat("projection_matrix", 12)

            return msg

        except Exception as e:
            self.get_logger().warn(f"[{tag}] Failed to parse yaml {yaml_path}: {e} -> publishing empty CameraInfo")
            return CameraInfo(header=msg.header)

    def _tick(self):
        # 尽量丢掉旧帧，保证低延迟 & 左右更接近“最新”
        if self.drop_old:
            for _ in range(max(self.grab_warmup, 0)):
                self.cap_l.grab()
                self.cap_r.grab()

        ret_l, frame_l = self.cap_l.read()
        ret_r, frame_r = self.cap_r.read()

        if not ret_l or frame_l is None:
            self.get_logger().warn("left read failed")
            return
        if not ret_r or frame_r is None:
            self.get_logger().warn("right read failed")
            return

        # 关键：统一一个 stamp
        stamp = self.get_clock().now().to_msg()

        # Image（OpenCV 默认输出通常是 BGR）
        msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
        msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

        msg_l.header.stamp = stamp
        msg_r.header.stamp = stamp
        msg_l.header.frame_id = self.left_frame_id
        msg_r.header.frame_id = self.right_frame_id

        self.pub_l_img.publish(msg_l)
        self.pub_r_img.publish(msg_r)

        # CameraInfo 同 stamp
        self.l_info.header.stamp = stamp
        self.r_info.header.stamp = stamp
        self.pub_l_info.publish(self.l_info)
        self.pub_r_info.publish(self.r_info)

    def destroy_node(self):
        try:
            if getattr(self, "cap_l", None):
                self.cap_l.release()
            if getattr(self, "cap_r", None):
                self.cap_r.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DualV4L2SyncPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

