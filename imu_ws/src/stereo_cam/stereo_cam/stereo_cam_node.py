#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本节点把它从中间切成左右两路发布,并解决 VIO 两大坑:
    1) 帧堆积(stale frames): 独立采集线程 + 最小缓冲,持续排空到最新
    2) 时间戳: 取到最新帧的瞬间打戳,预留固定延迟补偿参数

防堆积策略:
  - CAP_PROP_BUFFERSIZE=1  : 让底层缓冲队列尽量短
  - 独立线程死循环 read()  : 持续排空内核队列,不让旧帧积压
  - 只保留最新一帧          : VIO 只要最新画面,旧帧直接丢(图像可丢,IMU 不可丢)
  - SensorDataQoS(depth=1) : 防止在 ROS 话题层再次堆积
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


FOURCC = {
    "MJPG": cv2.VideoWriter_fourcc(*"MJPG"),
    "YUYV": cv2.VideoWriter_fourcc(*"YUYV"),
}


class StereoCamNode(Node):
    def __init__(self):
        super().__init__("stereo_cam_node")

        # ---- 参数 ----
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 1280)       # 拼接总宽,单目=640
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("encoding", "mono8")  # mono8省带宽利于VIO;要彩色用bgr8
        # 曝光->驱动打戳 的固定延迟(秒)。Kalibr标定后填,先留0。
        self.declare_parameter("time_offset", 0.0)

        self.device = self.get_parameter("device").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value
        self.pixfmt = self.get_parameter("pixel_format").value
        self.frame_id = self.get_parameter("frame_id").value
        self.encoding = self.get_parameter("encoding").value
        self.time_offset = self.get_parameter("time_offset").value

        self.half_w = self.width // 2   # 单目宽度,从中间切开

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, "stereo/left/image_raw", qos_profile_sensor_data)
        self.pub_right = self.create_publisher(Image, "stereo/right/image_raw", qos_profile_sensor_data)

        self._open_camera()

        # 共享变量: 采集线程写, 发布定时器读
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.new_flag = False

        self.running = True
        self.grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self.grab_thread.start()

        # 发布定时器: 比帧率略快, 保证不漏发最新帧
        self.timer = self.create_timer(1.0 / (self.fps * 2), self._publish_latest)

        self.frame_count = 0
        self.stat_timer = self.create_timer(5.0, self._report_hz)

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开相机 {self.device}")
        # 顺序: 先设FOURCC, 再设分辨率, 再帧率, 最后缓冲
        self.cap.set(cv2.CAP_PROP_FOURCC, FOURCC.get(self.pixfmt, FOURCC["MJPG"]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"相机已开: {self.device} {real_w}x{real_h}@{self.fps} {self.pixfmt} "
            f"-> 单目 {self.half_w}x{real_h}, encoding={self.encoding}, time_offset={self.time_offset}s")
        if real_w != self.width:
            self.get_logger().warn(f"实际宽 {real_w} != 期望 {self.width},切分位置可能不对")

    def _grab_loop(self):
        """独立采集线程: 死循环 read(),始终排空到最新帧,拿到即打戳。"""
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("read() 失败,重试中...")
                continue
            stamp = self.get_clock().now()
            if self.time_offset != 0.0:
                stamp = stamp - Duration(seconds=self.time_offset)
            with self.lock:
                self.latest_frame = frame
                self.latest_stamp = stamp.to_msg()
                self.new_flag = True

    def _publish_latest(self):
        """发布最新帧: 切左右目,分别发。没新帧就跳过。"""
        with self.lock:
            if not self.new_flag or self.latest_frame is None:
                return
            frame = self.latest_frame
            stamp = self.latest_stamp
            self.new_flag = False

        left = frame[:, :self.half_w]
        right = frame[:, self.half_w:]
        if self.encoding == "mono8":
            left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        left_msg = self.bridge.cv2_to_imgmsg(left, encoding=self.encoding)
        right_msg = self.bridge.cv2_to_imgmsg(right, encoding=self.encoding)
        for m in (left_msg, right_msg):
            m.header.stamp = stamp          # 左右目共用同一时间戳(硬件同步保证)
            m.header.frame_id = self.frame_id
        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        self.frame_count += 1

    def _report_hz(self):
        self.get_logger().info(f"发布帧率: {self.frame_count / 5.0:.1f} fps")
        self.frame_count = 0

    def destroy_node(self):
        self.running = False
        if self.grab_thread.is_alive():
            self.grab_thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = StereoCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
