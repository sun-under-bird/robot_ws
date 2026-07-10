import os
import yaml
import cv2
import numpy as np

import rclpy
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge


def load_yaml_to_camerainfo(filename: str) -> CameraInfo:
    with open(filename, 'r') as f:
        calib_data = yaml.safe_load(f)

    ci = CameraInfo()
    ci.width = int(calib_data['image_width'])
    ci.height = int(calib_data['image_height'])
    ci.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
    ci.d = calib_data['distortion_coefficients']['data']
    ci.k = calib_data['camera_matrix']['data']
    ci.r = calib_data['rectification_matrix']['data']
    ci.p = calib_data['projection_matrix']['data']
    return ci


class StereoDriverNode(Node):
    def __init__(self):
        super().__init__('stereo_driver_node')

        config_dir = os.path.join(
            get_package_share_directory('stereo_camera_pkg_py'), 'config'
        )

        # ===== Params =====
        self.declare_parameter('left_yaml_path', os.path.join(config_dir, 'left.yaml'))
        self.declare_parameter('right_yaml_path', os.path.join(config_dir, 'right.yaml'))

        # 输入图像话题：优先 compressed（为了解码在本节点做）
        self.declare_parameter('input_compressed_topic', '/image_raw/compressed')

        # 裁剪参数：如果你的源图是 1280x480，两路各 640x480
        self.declare_parameter('crop_left_offset_x', 0)
        self.declare_parameter('crop_right_offset_x', 640)
        self.declare_parameter('crop_offset_y', 0)
        self.declare_parameter('crop_width', 640)
        self.declare_parameter('crop_height', 480)

        # 输出话题
        self.declare_parameter('left_image_topic', '/stereo/left/camera/image_raw')
        self.declare_parameter('right_image_topic', '/stereo/right/camera/image_raw')
        self.declare_parameter('left_info_topic', '/stereo/left/camera/camera_info')
        self.declare_parameter('right_info_topic', '/stereo/right/camera/camera_info')

        # 输出 frame_id（和你 TF 里一致）
        self.declare_parameter('left_frame_id', 'camera_left_frame')
        self.declare_parameter('right_frame_id', 'camera_right_frame')

        left_yaml = self.get_parameter('left_yaml_path').value
        right_yaml = self.get_parameter('right_yaml_path').value

        self.input_compressed_topic = self.get_parameter('input_compressed_topic').value

        self.left_image_topic = self.get_parameter('left_image_topic').value
        self.right_image_topic = self.get_parameter('right_image_topic').value
        self.left_info_topic = self.get_parameter('left_info_topic').value
        self.right_info_topic = self.get_parameter('right_info_topic').value

        self.left_frame_id = self.get_parameter('left_frame_id').value
        self.right_frame_id = self.get_parameter('right_frame_id').value

        self.crop_left_offset_x = int(self.get_parameter('crop_left_offset_x').value)
        self.crop_right_offset_x = int(self.get_parameter('crop_right_offset_x').value)
        self.crop_offset_y = int(self.get_parameter('crop_offset_y').value)
        self.crop_width = int(self.get_parameter('crop_width').value)
        self.crop_height = int(self.get_parameter('crop_height').value)

        # ===== Load calibration =====
        if not os.path.exists(left_yaml) or not os.path.exists(right_yaml):
            self.get_logger().error(
                f"标定文件未找到! 请检查路径:\nL: {left_yaml}\nR: {right_yaml}"
            )

        self.left_info_msg = load_yaml_to_camerainfo(left_yaml)
        self.right_info_msg = load_yaml_to_camerainfo(right_yaml)

        # ⚠️ 关键：裁剪后分辨率变化时，CameraInfo 的 width/height 也要匹配
        self.left_info_msg.width = self.crop_width
        self.left_info_msg.height = self.crop_height
        self.right_info_msg.width = self.crop_width
        self.right_info_msg.height = self.crop_height

        # 如果你的标定 YAML 本来就是裁剪后的(640x480)，这里刚好一致；
        # 如果不是，你应该用匹配裁剪分辨率重新标定或提供对应 YAML。

        # ===== Tools =====
        self.bridge = CvBridge()

        # 图像 QoS：best_effort 更适合高频图像，避免 QoS 不兼容
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===== Subscriber (CompressedImage) =====
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            self.input_compressed_topic,
            self.compressed_callback,
            img_qos
        )

        # ===== Publishers =====
        self.pub_left_img = self.create_publisher(Image, self.left_image_topic, 10)
        self.pub_left_info = self.create_publisher(CameraInfo, self.left_info_topic, 10)

        self.pub_right_img = self.create_publisher(Image, self.right_image_topic, 10)
        self.pub_right_info = self.create_publisher(CameraInfo, self.right_info_topic, 10)

        self.get_logger().info(
            f"Stereo Driver Started (compressed -> decode -> crop -> publish)\n"
            f"  input:  {self.input_compressed_topic}\n"
            f"  left:   {self.left_image_topic}, {self.left_info_topic}\n"
            f"  right:  {self.right_image_topic}, {self.right_info_topic}\n"
            f"  crop: left_x={self.crop_left_offset_x}, right_x={self.crop_right_offset_x}, "
            f"y={self.crop_offset_y}, w={self.crop_width}, h={self.crop_height}"
        )

    def _decode_mjpeg(self, msg: CompressedImage):
        # msg.format 常见 "jpeg"/"mjpeg"，都按 jpeg 解
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR
        return img

    def compressed_callback(self, msg: CompressedImage):
        try:
            cv_image = self._decode_mjpeg(msg)
            if cv_image is None:
                self.get_logger().warn("MJPEG decode failed: got None image.")
                return

            H, W, _ = cv_image.shape

            # 边界检查
            def crop(x0, y0, w, h):
                x1 = x0 + w
                y1 = y0 + h
                if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
                    raise ValueError(f"Crop out of range: img={W}x{H}, crop=({x0},{y0},{w},{h})")
                return cv_image[y0:y1, x0:x1]

            left_frame = crop(self.crop_left_offset_x, self.crop_offset_y, self.crop_width, self.crop_height)
            right_frame = crop(self.crop_right_offset_x, self.crop_offset_y, self.crop_width, self.crop_height)

            # 转成 ROS Image（bgr8）
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")

            # 时间戳：用压缩图消息的 header（如果你的 CompressedImage header 正常）
            stamp = msg.header.stamp
            left_msg.header.stamp = stamp
            right_msg.header.stamp = stamp

            left_msg.header.frame_id = self.left_frame_id
            right_msg.header.frame_id = self.right_frame_id

            # CameraInfo 同步 header
            self.left_info_msg.header = left_msg.header
            self.right_info_msg.header = right_msg.header

            # 发布
            self.pub_left_img.publish(left_msg)
            self.pub_left_info.publish(self.left_info_msg)

            self.pub_right_img.publish(right_msg)
            self.pub_right_info.publish(self.right_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
