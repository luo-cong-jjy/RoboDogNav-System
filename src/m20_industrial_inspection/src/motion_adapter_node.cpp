#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionAdapterNode : public rclcpp::Node {
public:
  MotionAdapterNode() : Node("motion_adapter_node") {
    backend_type_ = declare_parameter<std::string>("backend_type", "sdk_deploy_cmd_vel");
    input_topic_ = declare_parameter<std::string>("input_topic", "/m20_inspection/cmd_vel_safe");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel");
    timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.6);
    publish_zero_on_timeout_ = declare_parameter<bool>("publish_zero_on_timeout", true);

    const std::unordered_set<std::string> supported_backends{
      "sdk_deploy_cmd_vel",
      "ros_cmd_vel",
    };
    if (supported_backends.count(backend_type_) == 0) {
      RCLCPP_WARN(
        get_logger(),
        "未知运动后端 backend_type=%s，将退回 sdk_deploy_cmd_vel。",
        backend_type_.c_str());
      backend_type_ = "sdk_deploy_cmd_vel";
    }

    if (backend_type_ == "ros_cmd_vel") {
      RCLCPP_WARN(
        get_logger(),
        "backend_type=ros_cmd_vel 是旧名称；当前推荐使用 sdk_deploy_cmd_vel，"
        "语义是把项目安全速度转发给 m20_sdk_deploy/rl_deploy_cmdvel。");
    }

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = now();
        timeout_zero_sent_ = false;
        publisher_->publish(*msg);
      });

    timer_ = create_wall_timer(100ms, [this]() { checkTimeout(); });

    RCLCPP_INFO(
      get_logger(),
      "运动适配层已启动：%s -> %s，后端=%s。sdk_deploy 仍作为独立 ROS 包运行。",
      input_topic_.c_str(),
      output_topic_.c_str(),
      backend_type_.c_str());
  }

private:
  void checkTimeout() {
    if (!publish_zero_on_timeout_ || timeout_zero_sent_) {
      return;
    }

    if (last_cmd_time_.nanoseconds() == 0) {
      return;
    }

    const double age = (now() - last_cmd_time_).seconds();
    if (age < timeout_sec_) {
      return;
    }

    // 安全层实现前的兜底保护：上游停止发送后，主动给后端补一个零速度。
    geometry_msgs::msg::Twist stop_cmd;
    publisher_->publish(stop_cmd);
    timeout_zero_sent_ = true;
    RCLCPP_WARN(get_logger(), "速度命令超时 %.2fs，已发布零速度。", age);
  }

  std::string backend_type_;
  std::string input_topic_;
  std::string output_topic_;
  double timeout_sec_{0.6};
  bool publish_zero_on_timeout_{true};
  bool timeout_zero_sent_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
