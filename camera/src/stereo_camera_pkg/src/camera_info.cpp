#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono;

class StereoCameraNode : public rclcpp::Node {
public:
    StereoCameraNode() : Node("stereo_camera_node")
    {
        const std::string config_dir =
            ament_index_cpp::get_package_share_directory("stereo_camera_pkg") + "/config";

        // parameters
        this->declare_parameter<int>("left_cam_idx", 0);
        this->declare_parameter<int>("right_cam_idx", 2);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<std::string>("left_calib_path",
            config_dir + "/left.yaml");
        this->declare_parameter<std::string>("right_calib_path",
            config_dir + "/right.yaml");

        // get parameter
        int left_idx = this->get_parameter("left_cam_idx").as_int();
        int right_idx = this->get_parameter("right_cam_idx").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        std::string left_calib_path = this->get_parameter("left_calib_path").as_string();
        std::string right_calib_path = this->get_parameter("right_calib_path").as_string();

        // open cameras
        cap_left_.open(left_idx, cv::CAP_V4L2);
        cap_right_.open(right_idx, cv::CAP_V4L2);
        RCLCPP_INFO(this->get_logger(), "Opening cameras...");
        if (!cap_left_.isOpened() || !cap_right_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        // width and height
        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_left_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_right_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));

        // load camera info (不改你yaml里的K/R/P，原样读出来)
        left_camera_info_ = load_camera_info(left_calib_path, width_, height_);
        right_camera_info_ = load_camera_info(right_calib_path, width_, height_);

        if (left_camera_info_.width == 0 || right_camera_info_.width == 0) {
            RCLCPP_FATAL(this->get_logger(), "Load camera calibration file failed!");
            rclcpp::shutdown();
            return;
        }

        // Publishers - 原始图像
        pub_left_raw_  = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw", 10);
        pub_right_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw", 10);

        // Publishers - 相机信息（仍然发布到 /left_camera/camera_info /right_camera/camera_info）
        pub_left_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info_raw", 10);
        pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info_raw", 10);

        // 定时器回调（≈15fps，66ms一次）
        timer_ = this->create_wall_timer(milliseconds(66), std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started successfully!");
        RCLCPP_INFO(this->get_logger(), "Left camera: /dev/video%d, Right camera: /dev/video%d", left_idx, right_idx);
    }

private:
    // 加载相机标定信息（保持你原来的：K/D/R/P 都读出来）
    sensor_msgs::msg::CameraInfo load_camera_info(const std::string& yaml_path, int width, int height) {
        sensor_msgs::msg::CameraInfo info_msg;
        try {
            YAML::Node calib_node = YAML::LoadFile(yaml_path);

            info_msg.width = width;
            info_msg.height = height;
            info_msg.distortion_model = calib_node["distortion_model"].as<std::string>();

            // D
            std::vector<double> D_vec = calib_node["distortion_coefficients"]["data"].as<std::vector<double>>();
            info_msg.d.clear();
            for (size_t i = 0; i < D_vec.size() && i < 5; ++i) {
                info_msg.d.push_back(D_vec[i]);
            }
            while (info_msg.d.size() < 5) info_msg.d.push_back(0.0);

            // K
            std::vector<double> K_vec = calib_node["camera_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) {
                info_msg.k[i] = (i < K_vec.size()) ? K_vec[i] : 0.0;
            }

            // R
            if (calib_node["rectification_matrix"] && calib_node["rectification_matrix"]["data"]) {
                std::vector<double> R_vec = calib_node["rectification_matrix"]["data"].as<std::vector<double>>();
                for (size_t i = 0; i < 9; ++i) {
                    info_msg.r[i] = (i < R_vec.size()) ? R_vec[i] : ((i % 4 == 0) ? 1.0 : 0.0);
                }
            } else {
                info_msg.r = {1,0,0, 0,1,0, 0,0,1};
            }

            // P
            if (calib_node["projection_matrix"] && calib_node["projection_matrix"]["data"]) {
                std::vector<double> P_vec = calib_node["projection_matrix"]["data"].as<std::vector<double>>();
                for (size_t i = 0; i < 12; ++i) {
                    info_msg.p[i] = (i < P_vec.size()) ? P_vec[i] : 0.0;
                }
            } else {
                for (auto &x : info_msg.p) x = 0.0;
                info_msg.p[0]  = info_msg.k[0]; info_msg.p[2]  = info_msg.k[2];
                info_msg.p[5]  = info_msg.k[4]; info_msg.p[6]  = info_msg.k[5];
                info_msg.p[10] = 1.0;
            }

            // ROI
            info_msg.binning_x = 0;
            info_msg.binning_y = 0;
            info_msg.roi.x_offset = 0;
            info_msg.roi.y_offset = 0;
            info_msg.roi.width = width;
            info_msg.roi.height = height;
            // 这里按你原来写法保留（你如果觉得语义要 raw，可以改 false，但你说不需要改就不动）
            info_msg.roi.do_rectify = true;

            RCLCPP_INFO(this->get_logger(), "Load calibration file success: %s", yaml_path.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading camera info: %s", e.what());
            info_msg.width = 0;
            info_msg.height = 0;
        }
        return info_msg;
    }

    void stereo_callback() {
        cv::Mat frame_left, frame_right;

        // 非阻塞读取（避免帧堆积）
        cap_left_.grab();
        cap_right_.grab();
        bool left_ret = cap_left_.retrieve(frame_left);
        bool right_ret = cap_right_.retrieve(frame_right);

        if (!left_ret || !right_ret || frame_left.empty() || frame_right.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame from camera! (left: %d, right: %d)", left_ret, right_ret);
            return;
        }

        // 只发布原始图像
        auto msg_left_raw  = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left).toImageMsg();
        auto msg_right_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right).toImageMsg();

        // 时间戳
        rclcpp::Time current_time = this->get_clock()->now();
        msg_left_raw->header.stamp  = current_time;
        msg_right_raw->header.stamp = current_time;

        // frame_id
        msg_left_raw->header.frame_id  = "left_camera_optical";
        msg_right_raw->header.frame_id = "right_camera_optical";

        // camera_info header 对齐
        sensor_msgs::msg::CameraInfo left_info  = left_camera_info_;
        sensor_msgs::msg::CameraInfo right_info = right_camera_info_;
        left_info.header  = msg_left_raw->header;
        right_info.header = msg_right_raw->header;

        // publish
        pub_left_raw_->publish(*msg_left_raw);
        pub_right_raw_->publish(*msg_right_raw);
        pub_left_info_->publish(left_info);
        pub_right_info_->publish(right_info);
    }

private:
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_, height_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // camera info
    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
