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
    return ci

def scale_camerainfo(ci, scale_factor):
    scaled_ci = CameraInfo()
    scaled_ci.width = int(ci.width * scale_factor)
    scaled_ci.height = int(ci.height * scale_factor)
    scaled_ci.distortion_model = ci.distortion_model
    scaled_ci.d = ci.d
    scaled_ci.r = ci.r
    
    k = np.array(ci.k).reshape(3, 3)
    k_scaled = k.copy()
    k_scaled[0, 0] *= scale_factor
    k_scaled[0, 2] *= scale_factor
    k_scaled[1, 1] *= scale_factor
    k_scaled[1, 2] *= scale_factor
    scaled_ci.k = k_scaled.flatten().tolist()
    
    p = np.array(ci.p).reshape(3, 4)
    p_scaled = p.copy()
    p_scaled[0, 0] *= scale_factor
    p_scaled[0, 2] *= scale_factor
    p_scaled[1, 1] *= scale_factor
    p_scaled[1, 2] *= scale_factor
    scaled_ci.p = p_scaled.flatten().tolist()
    
    return scaled_ci

class StereoDriverNode(Node):
    def __init__(self):
        super().__init__('stereo_driver_node')

        self.declare_parameter('left_yaml_path', '')
        self.declare_parameter('right_yaml_path', '')
        
        left_yaml = self.get_parameter('left_yaml_path').value
        right_yaml = self.get_parameter('right_yaml_path').value

        if not os.path.exists(left_yaml) or not os.path.exists(right_yaml):
            self.get_logger().error(f"标定文件未找到! 请检查路径:\nL: {left_yaml}\nR: {right_yaml}")
        
        self.left_info_msg = load_yaml_to_camerainfo(left_yaml)
        self.right_info_msg = load_yaml_to_camerainfo(right_yaml)
        
        self.target_width = self.left_info_msg.width
        self.target_height = self.left_info_msg.height
        self.get_logger().info(f"输出分辨率: {self.target_width}x{self.target_height} (来自标定文件)")
        
        self.bridge = self.bridge = CvBridge()

        self.sub_raw = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )

        self.pub_left_img = self.create_publisher(Image, '/stereo/left/camera/image_raw', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/stereo/left/camera/camera_info', 10)
        self.pub_right_img = self.create_publisher(Image, '/stereo/right/camera/image_raw', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/stereo/right/camera/camera_info', 10)

        self.get_logger().info(f"Stereo Driver Started: 1280x480 -> {self.target_width}x{self.target_height}x2")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            height, width, _ = cv_image.shape 
            mid_point = width // 2 

            left_frame = cv_image[0:height, 0:mid_point]
            right_frame = cv_image[0:height, mid_point:width]

            if left_frame.shape[1] != self.target_width or left_frame.shape[0] != self.target_height:
                left_frame = cv2.resize(left_frame, (self.target_width, self.target_height), interpolation=cv2.INTER_LINEAR)
                right_frame = cv2.resize(right_frame, (self.target_width, self.target_height), interpolation=cv2.INTER_LINEAR)

            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")

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
