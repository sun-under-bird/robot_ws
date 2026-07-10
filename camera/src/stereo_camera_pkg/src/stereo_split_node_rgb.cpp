#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdint>
#include <string>


class StereoSplitNodeYuyv : public rclcpp::Node
{
public:
    StereoSplitNodeYuyv() : Node("stereo_split_node_yuyv")
    {
        // 如果只是相机图像流，更推荐 SensorDataQoS；
        // 你原来用 reliable 也可以，但高帧率下可能更容易堆积。
        // auto qos = rclcpp::SensorDataQoS();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(30)).reliable().durability_volatile();

        left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/left/camera/image_color", qos);

        right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/right/camera/image_color", qos);

        left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/left/camera/camera_info", qos);

        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/right/camera/camera_info", qos);

        std::string pkg_share = ament_index_cpp::get_package_share_directory("stereo_camera_pkg");

        left_info_ = loadCameraInfo(pkg_share + "/config/left_1.yaml");
        right_info_ = loadCameraInfo(pkg_share + "/config/right_1.yaml");

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            qos,
            std::bind(&StereoSplitNodeYuyv::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Stereo split node YUYV color version started");
    }

private:
    sensor_msgs::msg::CameraInfo loadCameraInfo(const std::string& yaml_path)
    {
        sensor_msgs::msg::CameraInfo info;
        YAML::Node config = YAML::LoadFile(yaml_path);

        info.width = config["image_width"].as<int>();
        info.height = config["image_height"].as<int>();
        info.distortion_model = config["distortion_model"].as<std::string>();
        info.header.frame_id = config["camera_name"].as<std::string>();

        YAML::Node camera_matrix = config["camera_matrix"]["data"];
        for (int i = 0; i < 9; ++i)
        {
            info.k[i] = camera_matrix[i].as<double>();
        }

        YAML::Node distortion = config["distortion_coefficients"]["data"];
        info.d.clear();
        for (std::size_t i = 0; i < distortion.size(); ++i)
        {
            info.d.push_back(distortion[i].as<double>());
        }

        YAML::Node rectification = config["rectification_matrix"]["data"];
        for (int i = 0; i < 9; ++i)
        {
            info.r[i] = rectification[i].as<double>();
        }

        YAML::Node projection = config["projection_matrix"]["data"];
        for (int i = 0; i < 12; ++i)
        {
            info.p[i] = projection[i].as<double>();
        }

        return info;
    }

    bool convertToBgr(const sensor_msgs::msg::Image::SharedPtr& msg, cv::Mat& frame_bgr)
    {
        try
        {
            if (msg->encoding == sensor_msgs::image_encodings::YUV422_YUY2)
            {
                cv::Mat yuy2(
                    msg->height,
                    msg->width,
                    CV_8UC2,
                    const_cast<unsigned char*>(msg->data.data()),
                    msg->step);

                // YUYV / YUY2 -> BGR 彩色图
                cv::cvtColor(yuy2, frame_bgr, cv::COLOR_YUV2BGR_YUY2);
                return true;
            }
            else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);

                // ROS RGB8 -> OpenCV BGR
                cv::cvtColor(cv_ptr->image, frame_bgr, cv::COLOR_RGB2BGR);
                return true;
            }
            else if (msg->encoding == sensor_msgs::image_encodings::BGR8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

                frame_bgr = cv_ptr->image.clone();
                return true;
            }
            else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

                // 灰度输入也统一转成 BGR，保证后面发布 BGR8 不变形
                cv::cvtColor(cv_ptr->image, frame_bgr, cv::COLOR_GRAY2BGR);
                return true;
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unsupported encoding: %s",
                    msg->encoding.c_str());
                return false;
            }
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return false;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 你原来是每 2 帧取 1 帧。
        // 如果想全帧发布，把下面这个 if 删掉即可。
        if ((frame_count_++ % 2) != 0)
        {
            return;
        }

        cv::Mat frame_bgr;
        if (!convertToBgr(msg, frame_bgr))
        {
            return;
        }

        if (frame_bgr.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Converted frame is empty");
            return;
        }

        if (frame_bgr.cols % 2 != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Input image width must be even, current width=%d",
                frame_bgr.cols);
            return;
        }

        int half_width = frame_bgr.cols / 2;

        // 保持你原来的左右关系：
        // 右半幅作为 left，左半幅作为 right
        cv::Mat right = frame_bgr(cv::Rect(0, 0, half_width, frame_bgr.rows)).clone();
        cv::Mat left  = frame_bgr(cv::Rect(half_width, 0, half_width, frame_bgr.rows)).clone();

        // 如果相机画面倒置，需要打开下面旋转逻辑
        /*
        cv::rotate(left,  left,  cv::ROTATE_180);
        cv::rotate(right, right, cv::ROTATE_180);
        */

        rclcpp::Time stamp = msg->header.stamp;

        std_msgs::msg::Header left_header;
        left_header.stamp = stamp;
        left_header.frame_id = "camera_left_frame";

        std_msgs::msg::Header right_header;
        right_header.stamp = stamp;
        right_header.frame_id = "camera_right_frame";

        // 注意：这里必须是 BGR8，不能再写 MONO8
        auto left_msg = cv_bridge::CvImage(
            left_header,
            sensor_msgs::image_encodings::BGR8,
            left).toImageMsg();

        auto right_msg = cv_bridge::CvImage(
            right_header,
            sensor_msgs::image_encodings::BGR8,
            right).toImageMsg();

        left_image_pub_->publish(*left_msg);
        right_image_pub_->publish(*right_msg);

        sensor_msgs::msg::CameraInfo left_info_msg = left_info_;
        sensor_msgs::msg::CameraInfo right_info_msg = right_info_;

        left_info_msg.header.stamp = stamp;
        left_info_msg.header.frame_id = "camera_left_frame";

        right_info_msg.header.stamp = stamp;
        right_info_msg.header.frame_id = "camera_right_frame";

        // 这里让 CameraInfo 的 width/height 至少和发布图像一致。
        // 但注意：真正用于 SLAM 时，yaml 里的内参必须对应这个分辨率。
        left_info_msg.width = left.cols;
        left_info_msg.height = left.rows;
        right_info_msg.width = right.cols;
        right_info_msg.height = right.rows;

        left_info_pub_->publish(left_info_msg);
        right_info_pub_->publish(right_info_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    sensor_msgs::msg::CameraInfo left_info_;
    sensor_msgs::msg::CameraInfo right_info_;

    std::uint64_t frame_count_ = 0;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoSplitNodeYuyv>());
    rclcpp::shutdown();
    return 0;
}
