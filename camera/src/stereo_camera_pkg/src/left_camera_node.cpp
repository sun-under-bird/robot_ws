#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <string>

using namespace std::chrono;

class StereoCameraNode : public rclcpp::Node {
public:
    StereoCameraNode() : Node("stereo_camera_node") {
        this->declare_parameter<int>("left_cam_idx", 0);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 15);

        int left_idx = this->get_parameter("left_cam_idx").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        cap_left_.open(left_idx, cv::CAP_V4L2);

        RCLCPP_INFO(this->get_logger(), "Opening camera /dev/video%d ...", left_idx);
        if (!cap_left_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_left_.set(cv::CAP_PROP_FPS, fps);

        // Force YUYV
        cap_left_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_left_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        pub_left_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw", 10);

        auto period_ms = static_cast<int>(1000.0 / std::max(1, fps));
        timer_ = this->create_wall_timer(milliseconds(period_ms),
                                         std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Mono camera node started. Requested %dx%d @%dfps", width, height, fps);
        RCLCPP_INFO(this->get_logger(), "Actual  %.0fx%.0f @%.1ffps  FOURCC=%s",
                    cap_left_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_left_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_left_.get(cv::CAP_PROP_FPS),
                    fourcc_to_string(cap_left_).c_str());
    }

private:
    static std::string fourcc_to_string(cv::VideoCapture &cap) {
        int fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
        char fourcc_str[] = {
            char(fourcc & 0xFF),
            char((fourcc >> 8) & 0xFF),
            char((fourcc >> 16) & 0xFF),
            char((fourcc >> 24) & 0xFF),
            0
        };
        return std::string(fourcc_str);
    }

    void stereo_callback() {
        cv::Mat frame;
        cap_left_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Empty frame from camera");
            return;
        }

        static bool printed = false;
        if (!printed) {
            RCLCPP_INFO(this->get_logger(), "Frame type=%d channels=%d (CV_8UC2=%d CV_8UC3=%d)",
                        frame.type(), frame.channels(), CV_8UC2, CV_8UC3);
            printed = true;
        }

        cv::Mat bgr;
        // If truly YUYV, OpenCV often gives CV_8UC2
        if (frame.type() == CV_8UC2) {
            cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_YUY2);  // YUYV -> BGR
        } else if (frame.type() == CV_8UC3) {
            bgr = frame; // already BGR
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unexpected frame type=%d, channels=%d", frame.type(), frame.channels());
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "left_camera";

        pub_left_->publish(*msg);
    }

    cv::VideoCapture cap_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
