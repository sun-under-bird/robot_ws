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
        this->declare_parameter<int>("right_cam_idx", 2);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 15);

        int right_idx = this->get_parameter("right_cam_idx").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        cap_right_.open(right_idx, cv::CAP_V4L2);

        RCLCPP_INFO(this->get_logger(), "Opening camera /dev/video%d ...", right_idx);
        if (!cap_right_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_right_.set(cv::CAP_PROP_FPS, fps);

        // Force YUYV
        cap_right_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_right_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        pub_right_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw", 10);

        auto period_ms = static_cast<int>(1000.0 / std::max(1, fps));
        timer_ = this->create_wall_timer(milliseconds(period_ms),
                                         std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Mono (right) camera node started.");
        RCLCPP_INFO(this->get_logger(), "Requested %dx%d @%dfps", width, height, fps);
        RCLCPP_INFO(this->get_logger(), "Actual  %.0fx%.0f @%.1ffps  FOURCC=%s",
                    cap_right_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_right_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_right_.get(cv::CAP_PROP_FPS),
                    fourcc_to_string(cap_right_).c_str());
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
        static int frame_count = 0;
        if (frame_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Callback triggered %d times (≈1 second)", frame_count);
        }
        frame_count++;

        cv::Mat frame_right;
        cap_right_ >> frame_right;

        if (frame_right.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty frame");
            return;
        }

        static bool printed = false;
        if (!printed) {
            RCLCPP_INFO(this->get_logger(),
                        "Right frame type=%d channels=%d (CV_8UC2=%d CV_8UC3=%d)",
                        frame_right.type(), frame_right.channels(), CV_8UC2, CV_8UC3);
            printed = true;
        }

        cv::Mat right_bgr;
        if (frame_right.type() == CV_8UC2) {
            cv::cvtColor(frame_right, right_bgr, cv::COLOR_YUV2BGR_YUY2);
        } else if (frame_right.type() == CV_8UC3) {
            right_bgr = frame_right;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unexpected frame type=%d channels=%d",
                                 frame_right.type(), frame_right.channels());
            return;
        }

        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_bgr).toImageMsg();
        msg_right->header.stamp = this->now();
        msg_right->header.frame_id = "right_camera";

        pub_right_->publish(*msg_right);
    }

    cv::VideoCapture cap_right_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
