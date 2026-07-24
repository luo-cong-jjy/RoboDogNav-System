#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PatrolManagerNode : public rclcpp::Node {
public:
  PatrolManagerNode() : Node("patrol_manager_node") {
    route_name_ = declare_parameter<std::string>("route_name", "demo_motion");
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/m20_inspection/cmd_vel_nav");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    start_immediately_ = declare_parameter<bool>("start_immediately", true);
    loop_route_ = declare_parameter<bool>("loop_route", false);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 1.0);

    step_names_ = declare_parameter<std::vector<std::string>>(
      "step_names", {"forward", "stop", "turn_left", "stop"});
    step_durations_sec_ = declare_parameter<std::vector<double>>(
      "step_durations_sec", {4.0, 1.0, 3.0, 1.0});
    step_linear_x_ = declare_parameter<std::vector<double>>(
      "step_linear_x", {0.15, 0.0, 0.0, 0.0});
    step_linear_y_ = declare_parameter<std::vector<double>>(
      "step_linear_y", {0.0, 0.0, 0.0, 0.0});
    step_angular_z_ = declare_parameter<std::vector<double>>(
      "step_angular_z", {0.0, 0.0, 0.25, 0.0});

    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    start_delay_sec_ = std::max(0.0, start_delay_sec_);
    route_valid_ = validateRoute();

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { onTimer(); });

    RCLCPP_INFO(
      get_logger(),
      "巡检任务节点已启动：route=%s output=%s steps=%zu",
      route_name_.c_str(),
      cmd_topic_.c_str(),
      step_names_.size());
  }

private:
  bool validateRoute() {
    const std::size_t step_count = step_names_.size();
    const bool same_size =
      step_durations_sec_.size() == step_count &&
      step_linear_x_.size() == step_count &&
      step_linear_y_.size() == step_count &&
      step_angular_z_.size() == step_count;

    if (step_count == 0 || !same_size) {
      RCLCPP_ERROR(
        get_logger(),
        "巡检路线配置无效：所有 step_* 数组长度必须一致且不能为空。");
      return false;
    }

    for (std::size_t i = 0; i < step_count; ++i) {
      if (step_durations_sec_[i] <= 0.0) {
        RCLCPP_ERROR(
          get_logger(),
          "巡检路线配置无效：step_durations_sec[%zu] 必须大于 0。",
          i);
        return false;
      }
    }

    return true;
  }

  void onTimer() {
    if (!route_valid_ || !start_immediately_) {
      publishStop();
      return;
    }

    const rclcpp::Time current_time = now();
    if (route_start_time_.nanoseconds() == 0) {
      route_start_time_ = current_time;
      step_start_time_ = current_time;
      RCLCPP_INFO(get_logger(), "巡检路线将在 %.2fs 后开始。", start_delay_sec_);
    }

    if ((current_time - route_start_time_).seconds() < start_delay_sec_) {
      publishStop();
      return;
    }

    if (route_finished_) {
      publishStop();
      return;
    }

    const double step_elapsed = (current_time - step_start_time_).seconds();
    if (step_elapsed >= step_durations_sec_[current_step_index_]) {
      advanceStep(current_time);
    }

    publishCurrentStep();
  }

  void advanceStep(const rclcpp::Time & current_time) {
    if (current_step_index_ + 1 < step_names_.size()) {
      ++current_step_index_;
      step_start_time_ = current_time;
      RCLCPP_INFO(
        get_logger(),
        "进入巡检步骤 %zu/%zu：%s",
        current_step_index_ + 1,
        step_names_.size(),
        step_names_[current_step_index_].c_str());
      return;
    }

    if (loop_route_) {
      current_step_index_ = 0;
      route_start_time_ = current_time;
      step_start_time_ = current_time;
      RCLCPP_INFO(get_logger(), "巡检路线循环重启：%s", route_name_.c_str());
      return;
    }

    route_finished_ = true;
    publishStop();
    RCLCPP_INFO(get_logger(), "巡检路线完成：%s", route_name_.c_str());
  }

  void publishCurrentStep() {
    geometry_msgs::msg::Twist command;
    command.linear.x = step_linear_x_[current_step_index_];
    command.linear.y = step_linear_y_[current_step_index_];
    command.angular.z = step_angular_z_[current_step_index_];
    publisher_->publish(command);
  }

  void publishStop() {
    publisher_->publish(geometry_msgs::msg::Twist{});
  }

  std::string route_name_;
  std::string cmd_topic_;
  double publish_rate_hz_{10.0};
  bool start_immediately_{true};
  bool loop_route_{false};
  double start_delay_sec_{1.0};
  bool route_valid_{false};
  bool route_finished_{false};
  std::size_t current_step_index_{0};

  std::vector<std::string> step_names_;
  std::vector<double> step_durations_sec_;
  std::vector<double> step_linear_x_;
  std::vector<double> step_linear_y_;
  std::vector<double> step_angular_z_;

  rclcpp::Time route_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time step_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolManagerNode>());
  rclcpp::shutdown();
  return 0;
}
