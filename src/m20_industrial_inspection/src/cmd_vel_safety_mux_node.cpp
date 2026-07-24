#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CmdVelSafetyMuxNode : public rclcpp::Node {
public:
  CmdVelSafetyMuxNode() : Node("cmd_vel_safety_mux") {
    nav_cmd_topic_ = declare_parameter<std::string>(
      "nav_cmd_topic", "/m20_inspection/cmd_vel_nav");
    manual_cmd_topic_ = declare_parameter<std::string>(
      "manual_cmd_topic", "/m20_inspection/cmd_vel_manual");
    e_stop_topic_ = declare_parameter<std::string>(
      "e_stop_topic", "/m20_inspection/e_stop");
    safe_cmd_topic_ = declare_parameter<std::string>(
      "safe_cmd_topic", "/m20_inspection/cmd_vel_safe");

    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.4);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.2);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.5);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", 0.8);
    max_angular_accel_ = declare_parameter<double>("max_angular_accel", 1.0);
    manual_priority_ = declare_parameter<bool>("manual_priority", true);

    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    command_timeout_sec_ = std::max(0.05, command_timeout_sec_);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, 10);

    nav_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_nav_cmd_ = *msg;
        last_nav_time_ = now();
        has_nav_cmd_ = true;
      });

    manual_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_manual_cmd_ = *msg;
        last_manual_time_ = now();
        has_manual_cmd_ = true;
      });

    e_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (e_stop_active_ != msg->data) {
          RCLCPP_WARN(
            get_logger(),
            "急停状态切换：%s",
            msg->data ? "触发" : "解除");
        }
        e_stop_active_ = msg->data;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishSafeCommand(); });

    RCLCPP_INFO(
      get_logger(),
      "速度安全仲裁已启动：nav=%s manual=%s e_stop=%s -> safe=%s",
      nav_cmd_topic_.c_str(),
      manual_cmd_topic_.c_str(),
      e_stop_topic_.c_str(),
      safe_cmd_topic_.c_str());
  }

private:
  enum class CommandSource {
    kNone,
    kNavigation,
    kManual,
    kEmergencyStop,
  };

  void publishSafeCommand() {
    const rclcpp::Time current_time = now();
    const double dt = getPublishDt(current_time);

    CommandSource source = CommandSource::kNone;
    geometry_msgs::msg::Twist command = selectCommand(current_time, source);
    command = clampCommand(command);
    command = applyAccelerationLimit(command, dt);

    publisher_->publish(command);
    last_output_cmd_ = command;
    last_publish_time_ = current_time;

    if (source != last_source_) {
      RCLCPP_INFO(get_logger(), "当前速度来源：%s", sourceName(source).c_str());
      last_source_ = source;
    }
  }

  geometry_msgs::msg::Twist selectCommand(
    const rclcpp::Time & current_time,
    CommandSource & source) const {
    if (e_stop_active_) {
      source = CommandSource::kEmergencyStop;
      return geometry_msgs::msg::Twist{};
    }

    const bool nav_fresh = isFresh(has_nav_cmd_, last_nav_time_, current_time);
    const bool manual_fresh = isFresh(has_manual_cmd_, last_manual_time_, current_time);

    if (manual_priority_) {
      if (manual_fresh) {
        source = CommandSource::kManual;
        return last_manual_cmd_;
      }
      if (nav_fresh) {
        source = CommandSource::kNavigation;
        return last_nav_cmd_;
      }
    } else {
      if (nav_fresh) {
        source = CommandSource::kNavigation;
        return last_nav_cmd_;
      }
      if (manual_fresh) {
        source = CommandSource::kManual;
        return last_manual_cmd_;
      }
    }

    source = CommandSource::kNone;
    return geometry_msgs::msg::Twist{};
  }

  bool isFresh(
    bool has_command,
    const rclcpp::Time & command_time,
    const rclcpp::Time & current_time) const {
    if (!has_command || command_time.nanoseconds() == 0) {
      return false;
    }
    return (current_time - command_time).seconds() <= command_timeout_sec_;
  }

  geometry_msgs::msg::Twist clampCommand(geometry_msgs::msg::Twist command) const {
    command.linear.x = clampSymmetric(command.linear.x, max_linear_x_);
    command.linear.y = clampSymmetric(command.linear.y, max_linear_y_);
    command.linear.z = 0.0;
    command.angular.x = 0.0;
    command.angular.y = 0.0;
    command.angular.z = clampSymmetric(command.angular.z, max_angular_z_);
    return command;
  }

  geometry_msgs::msg::Twist applyAccelerationLimit(
    geometry_msgs::msg::Twist target,
    double dt) const {
    if (dt <= 0.0) {
      return target;
    }

    if (max_linear_accel_ > 0.0) {
      const double max_delta = max_linear_accel_ * dt;
      target.linear.x = limitDelta(last_output_cmd_.linear.x, target.linear.x, max_delta);
      target.linear.y = limitDelta(last_output_cmd_.linear.y, target.linear.y, max_delta);
    }

    if (max_angular_accel_ > 0.0) {
      const double max_delta = max_angular_accel_ * dt;
      target.angular.z = limitDelta(last_output_cmd_.angular.z, target.angular.z, max_delta);
    }

    return target;
  }

  double getPublishDt(const rclcpp::Time & current_time) const {
    if (last_publish_time_.nanoseconds() == 0) {
      return 1.0 / publish_rate_hz_;
    }
    return std::max(0.0, (current_time - last_publish_time_).seconds());
  }

  static double clampSymmetric(double value, double limit) {
    const double abs_limit = std::fabs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
  }

  static double limitDelta(double current, double target, double max_delta) {
    const double delta = std::clamp(target - current, -max_delta, max_delta);
    return current + delta;
  }

  static std::string sourceName(CommandSource source) {
    switch (source) {
      case CommandSource::kNavigation:
        return "导航";
      case CommandSource::kManual:
        return "手动";
      case CommandSource::kEmergencyStop:
        return "急停";
      case CommandSource::kNone:
      default:
        return "无有效命令";
    }
  }

  std::string nav_cmd_topic_;
  std::string manual_cmd_topic_;
  std::string e_stop_topic_;
  std::string safe_cmd_topic_;

  double command_timeout_sec_{0.5};
  double publish_rate_hz_{20.0};
  double max_linear_x_{0.4};
  double max_linear_y_{0.2};
  double max_angular_z_{0.5};
  double max_linear_accel_{0.8};
  double max_angular_accel_{1.0};
  bool manual_priority_{true};
  bool e_stop_active_{false};
  bool has_nav_cmd_{false};
  bool has_manual_cmd_{false};

  geometry_msgs::msg::Twist last_nav_cmd_;
  geometry_msgs::msg::Twist last_manual_cmd_;
  geometry_msgs::msg::Twist last_output_cmd_;
  rclcpp::Time last_nav_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_manual_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  CommandSource last_source_{CommandSource::kNone};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyMuxNode>());
  rclcpp::shutdown();
  return 0;
}
