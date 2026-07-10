#include <array>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

class D435iKalibrCameraInfoNode : public rclcpp::Node
{
public:
    D435iKalibrCameraInfoNode() : Node("d435i_kalibr_camera_info_node")
    {
        const std::string package_share = ament_index_cpp::get_package_share_directory("stereo_camera_pkg");
        const std::string default_left_path = package_share + "/config/d435i_infra1_kalibr_camera_info.yaml";
        const std::string default_right_path = package_share + "/config/d435i_infra2_kalibr_camera_info.yaml";

        left_image_topic_ = declare_parameter<std::string>(
            "left_image_topic", "/camera/camera/infra1/image_rect_raw");
        right_image_topic_ = declare_parameter<std::string>(
            "right_image_topic", "/camera/camera/infra2/image_rect_raw");
        left_info_topic_ = declare_parameter<std::string>(
            "left_info_topic", "/camera/camera/infra1/camera_info_kalibr");
        right_info_topic_ = declare_parameter<std::string>(
            "right_info_topic", "/camera/camera/infra2/camera_info_kalibr");
        left_frame_id_ = declare_parameter<std::string>("left_frame_id", "");
        right_frame_id_ = declare_parameter<std::string>("right_frame_id", "");
        left_info_path_ = declare_parameter<std::string>("left_info_path", default_left_path);
        right_info_path_ = declare_parameter<std::string>("right_info_path", default_right_path);

        left_info_ = GetCameraInfoValue(left_info_path_);
        right_info_ = GetCameraInfoValue(right_info_path_);

        const rclcpp::QoS qos = rclcpp::SensorDataQoS();
        left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic_, qos);
        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic_, qos);
        left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            left_image_topic_,
            qos,
            std::bind(&D435iKalibrCameraInfoNode::OnLeftImage, this, std::placeholders::_1));
        right_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            right_image_topic_,
            qos,
            std::bind(&D435iKalibrCameraInfoNode::OnRightImage, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "left image: %s -> info: %s", left_image_topic_.c_str(), left_info_topic_.c_str());
        RCLCPP_INFO(get_logger(), "right image: %s -> info: %s", right_image_topic_.c_str(), right_info_topic_.c_str());
        RCLCPP_INFO(get_logger(), "left info file: %s", left_info_path_.c_str());
        RCLCPP_INFO(get_logger(), "right info file: %s", right_info_path_.c_str());
    }

private:
    // 从 ROS camera calibration YAML 读取 CameraInfo，保持 K/D/R/P 原样进入消息。
    sensor_msgs::msg::CameraInfo GetCameraInfoValue(const std::string &yaml_path)
    {
        sensor_msgs::msg::CameraInfo info;
        YAML::Node root = YAML::LoadFile(yaml_path);

        info.width = root["image_width"].as<uint32_t>();
        info.height = root["image_height"].as<uint32_t>();
        info.distortion_model = root["distortion_model"].as<std::string>();
        info.d = root["distortion_coefficients"]["data"].as<std::vector<double>>();

        SetArrayValue<9>(root["camera_matrix"]["data"].as<std::vector<double>>(), info.k);
        SetArrayValue<9>(root["rectification_matrix"]["data"].as<std::vector<double>>(), info.r);
        SetArrayValue<12>(root["projection_matrix"]["data"].as<std::vector<double>>(), info.p);

        info.binning_x = 0;
        info.binning_y = 0;
        info.roi.x_offset = 0;
        info.roi.y_offset = 0;
        info.roi.height = 0;
        info.roi.width = 0;
        info.roi.do_rectify = false;
        return info;
    }

    template <size_t Size>
    void SetArrayValue(const std::vector<double> &source, std::array<double, Size> &target)
    {
        if (source.size() != Size)
        {
            throw std::runtime_error("camera info matrix size mismatch");
        }

        for (size_t index = 0; index < Size; ++index)
        {
            target[index] = source[index];
        }
    }

    void OnLeftImage(const sensor_msgs::msg::Image::SharedPtr image)
    {
        PublishCameraInfoValue(*image, left_frame_id_, left_info_, left_info_pub_);
    }

    void OnRightImage(const sensor_msgs::msg::Image::SharedPtr image)
    {
        PublishCameraInfoValue(*image, right_frame_id_, right_info_, right_info_pub_);
    }

    // 用图像时间戳发布 CameraInfo，保证 RTAB-Map 的近似同步能匹配上。
    void PublishCameraInfoValue(
        const sensor_msgs::msg::Image &image,
        const std::string &frame_id_override,
        const sensor_msgs::msg::CameraInfo &template_info,
        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr &publisher)
    {
        sensor_msgs::msg::CameraInfo info = template_info;
        info.header = image.header;
        if (!frame_id_override.empty())
        {
            info.header.frame_id = frame_id_override;
        }
        publisher->publish(info);
    }

    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string left_info_topic_;
    std::string right_info_topic_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    std::string left_info_path_;
    std::string right_info_path_;

    sensor_msgs::msg::CameraInfo left_info_;
    sensor_msgs::msg::CameraInfo right_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D435iKalibrCameraInfoNode>());
    rclcpp::shutdown();
    return 0;
}
