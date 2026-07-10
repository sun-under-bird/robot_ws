#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cstdint>

class StereoSplitNodeYuyv : public rclcpp::Node
{
public:
    StereoSplitNodeYuyv() : Node("stereo_split_node_yuyv")
    {
    //    auto qos = rclcpp::SensorDataQoS();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(30)).reliable().durability_volatile();

        left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/left/camera/image_mono", qos);
        right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/right/camera/image_mono", qos);
        left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera/camera_info", qos);
        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera/camera_info", qos);

        std::string pkg_share = ament_index_cpp::get_package_share_directory("stereo_camera_pkg");
        left_info_ = loadCameraInfo(pkg_share + "/config/left_1.yaml");
        right_info_ = loadCameraInfo(pkg_share + "/config/right_1.yaml");

        // left_info_ = loadCameraInfo(pkg_share + "/config/left_dj.yaml");
        // right_info_ = loadCameraInfo(pkg_share + "/config/right_dj.yaml");

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            qos,
            std::bind(&StereoSplitNodeYuyv::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Stereo split node (YUYV) started");
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
            info.k[i] = camera_matrix[i].as<double>();

        YAML::Node distortion = config["distortion_coefficients"]["data"];
        for (int i = 0; i < 5; ++i)
            info.d.push_back(distortion[i].as<double>());

        YAML::Node rectification = config["rectification_matrix"]["data"];
        for (int i = 0; i < 9; ++i)
            info.r[i] = rectification[i].as<double>();

        YAML::Node projection = config["projection_matrix"]["data"];
        for (int i = 0; i < 12; ++i)
            info.p[i] = projection[i].as<double>();

        return info;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        if ((frame_count_++ % 2) != 0)
        {
            return;
        }

        cv::Mat frame;

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
                cv::cvtColor(yuy2, frame, cv::COLOR_YUV2GRAY_YUY2);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
                cv::cvtColor(cv_ptr->image, frame, cv::COLOR_RGB2GRAY);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::BGR8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2GRAY);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
            {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
                frame = cv_ptr->image.clone();
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unsupported encoding: %s",
                    msg->encoding.c_str());
                return;
            }
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return;
        }

        int half_width = frame.cols / 2;

        // 右半幅作为 left，左半幅作为 right。
        cv::Mat right= frame(cv::Rect(0, 0, half_width, frame.rows)).clone();
        cv::Mat left= frame(cv::Rect(half_width, 0, half_width, frame.rows)).clone();

        // 左话题图像：逆时针旋转 180 度
        // 右话题图像：顺时针旋转 180 度
        //cv::Mat right_raw = frame(cv::Rect(0, 0, half_width, frame.rows)).clone();
        //cv::Mat left_raw  = frame(cv::Rect(half_width, 0, half_width, frame.rows)).clone();
        //cv::Mat left;
        //cv::Mat right;
	//cv::rotate(left_raw,  left,  cv::ROTATE_180);
	//cv::rotate(right_raw, right, cv::ROTATE_180);


        
        rclcpp::Time now = msg->header.stamp;

        std_msgs::msg::Header left_header;
        left_header.stamp = now;
        left_header.frame_id = "camera_left_frame";

        std_msgs::msg::Header right_header;
        right_header.stamp = now;
        right_header.frame_id = "camera_right_frame";

        auto left_msg  = cv_bridge::CvImage(left_header,  sensor_msgs::image_encodings::MONO8, left).toImageMsg();
        auto right_msg = cv_bridge::CvImage(right_header, sensor_msgs::image_encodings::MONO8, right).toImageMsg();

        left_image_pub_->publish(*left_msg);
        right_image_pub_->publish(*right_msg);

        sensor_msgs::msg::CameraInfo left_info_msg = left_info_;
        sensor_msgs::msg::CameraInfo right_info_msg = right_info_;
        left_info_msg.header.stamp = now;
        left_info_msg.header.frame_id = "camera_left_frame";
        right_info_msg.header.stamp = now;
        right_info_msg.header.frame_id = "camera_right_frame";

        left_info_pub_->publish(left_info_msg);
        right_info_pub_->publish(right_info_msg);


    }

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
