import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
import numpy as np

def load_yaml_to_camerainfo(filename):
    with open(filename, 'r') as f:
        calib_data = yaml.safe_load(f)
    ci = CameraInfo()
    ci.width = calib_data['image_width']
    ci.height = calib_data['image_height']
    ci.distortion_model = calib_data['distortion_model']
    ci.d = calib_data['distortion_coefficients']['data']
    ci.k = calib_data['camera_matrix']['data']
    ci.r = calib_data['rectification_matrix']['data']
    ci.p = calib_data['projection_matrix']['data']
    return ci, calib_data

class StereoDriverNode(Node):
    def __init__(self):
        super().__init__('stereo_driver_node')

        self.declare_parameter('left_yaml_path', '')
        self.declare_parameter('right_yaml_path', '')
        
        left_yaml = self.get_parameter('left_yaml_path').value
        right_yaml = self.get_parameter('right_yaml_path').value

        if not os.path.exists(left_yaml) or not os.path.exists(right_yaml):
            self.get_logger().error(f"Calibration files not found!\nL: {left_yaml}\nR: {right_yaml}")
            return
        
        self.left_info_msg, left_calib = load_yaml_to_camerainfo(left_yaml)
        self.right_info_msg, right_calib = load_yaml_to_camerainfo(right_yaml)
        
        self.bridge = CvBridge()
        
        self.left_map1, self.left_map2 = self._compute_rectify_map(left_calib)
        self.right_map1, self.right_map2 = self._compute_rectify_map(right_calib)

        self.sub_raw = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        self.pub_left_img = self.create_publisher(Image, '/stereo/left/camera/image_rect_color', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/stereo/left/camera/camera_info', 10)
        self.pub_right_img = self.create_publisher(Image, '/stereo/right/camera/image_rect_color', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/stereo/right/camera/camera_info', 10)

        self.get_logger().info("Stereo Driver Started with Rectification...")

    def _compute_rectify_map(self, calib):
        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        D = np.array(calib['distortion_coefficients']['data'])
        R = np.array(calib['rectification_matrix']['data']).reshape(3, 3)
        P = np.array(calib['projection_matrix']['data']).reshape(3, 4)
        w = calib['image_width']
        h = calib['image_height']
        
        map1, map2 = cv2.initUndistortRectifyMap(
            K, D, R, P[:3, :3], (w, h), cv2.CV_16SC2
        )
        return map1, map2

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape 
            mid_point = width // 2 

            left_frame = cv_image[0:height, 0:mid_point]
            right_frame = cv_image[0:height, mid_point:width]

            left_rect = cv2.remap(left_frame, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            right_rect = cv2.remap(right_frame, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

            left_msg = self.bridge.cv2_to_imgmsg(left_rect, encoding="bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_rect, encoding="bgr8")

            timestamp = msg.header.stamp
            
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = "camera_left_frame"
            self.left_info_msg.header = left_msg.header

            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = "camera_right_frame"
            self.right_info_msg.header = right_msg.header

            self.pub_left_img.publish(left_msg)
            self.pub_left_info.publish(self.left_info_msg)
            self.pub_right_img.publish(right_msg)
            self.pub_right_info.publish(self.right_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

