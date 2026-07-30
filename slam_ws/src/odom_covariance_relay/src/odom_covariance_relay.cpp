#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomCovarianceRelay : public rclcpp::Node
{
public:
  /// 创建里程计协方差转发节点，并加载输入、输出及协方差参数。
  OdomCovarianceRelay() : Node("odom_covariance_relay")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/odom");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/new_odom");

    // 默认协方差用于二维移动机器人：弱化 z、roll、pitch 方向的约束。
    std::vector<double> default_pose_cov = {
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    };

    // 速度协方差与位姿协方差分开配置，便于适配不同里程计来源。
    std::vector<double> default_twist_cov = {
      0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0,  0.0, 0.0, 0.0, 0.0,
      0.0, 0.0,  1e6, 0.0, 0.0, 0.0,
      0.0, 0.0,  0.0, 1e6, 0.0, 0.0,
      0.0, 0.0,  0.0, 0.0, 1e6, 0.0,
      0.0, 0.0,  0.0, 0.0, 0.0, 0.1
    };

    auto pose_cov = this->declare_parameter<std::vector<double>>("pose_covariance", default_pose_cov);
    auto twist_cov = this->declare_parameter<std::vector<double>>("twist_covariance", default_twist_cov);

    if (pose_cov.size() != 36) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'pose_covariance' size is %zu, expected 36. Using default values.",
                  pose_cov.size());
      pose_cov = default_pose_cov;
    }

    if (twist_cov.size() != 36) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'twist_covariance' size is %zu, expected 36. Using default values.",
                  twist_cov.size());
      twist_cov = default_twist_cov;
    }

    for (size_t i = 0; i < 36; ++i) {
      pose_covariance_[i] = pose_cov[i];
      twist_covariance_[i] = twist_cov[i];
    }

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&OdomCovarianceRelay::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", output_topic_.c_str());
  }

private:
  /// 复制输入里程计消息，并使用配置值覆盖位姿和速度协方差。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out_msg = *msg;

    out_msg.pose.covariance = pose_covariance_;
    out_msg.twist.covariance = twist_covariance_;

    pub_->publish(out_msg);
  }

private:
  std::string input_topic_;
  std::string output_topic_;

  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

/// 初始化 ROS 2，运行协方差转发节点直至退出。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCovarianceRelay>());
  rclcpp::shutdown();
  return 0;
}
