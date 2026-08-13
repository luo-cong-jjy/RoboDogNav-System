#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace scan_planner
{
class M20GaitPublisher : public rclcpp::Node
{
public:
  M20GaitPublisher() : Node("m20_gait_publisher")
  {
    const double rate = declare_parameter<double>("rate", 60.0);
    gait_frequency_ = declare_parameter<double>("gait_frequency", 2.2);
    min_walk_speed_ = declare_parameter<double>("min_walk_speed", 0.05);
    max_walk_speed_ = declare_parameter<double>("max_walk_speed", 1.0);
    always_trot_ = declare_parameter<bool>("always_trot", false);
    hip_swing_ = declare_parameter<double>("hip_swing", 0.08);
    thigh_swing_ = declare_parameter<double>("thigh_swing", 0.32);
    calf_swing_ = declare_parameter<double>("calf_swing", 0.42);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.09);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&M20GaitPublisher::odomCallback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
        std::bind(&M20GaitPublisher::timerCallback, this));

    joint_msg_.name = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double twist_speed = std::hypot(vx, vy);
    rclcpp::Time stamp(odom->header.stamp);
    if (stamp.nanoseconds() == 0) stamp = now();
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;
    double pose_speed = 0.0;
    if (has_prev_pose_)
    {
      const double dt = (stamp - last_odom_time_).seconds();
      if (dt > 1e-4) pose_speed = std::hypot(x - last_odom_x_, y - last_odom_y_) / dt;
    }
    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;
    forward_speed_ = std::abs(vx) > min_walk_speed_ ? vx : horizontal_speed_;
    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;
    has_odom_ = true;
  }

  void timerCallback()
  {
    const auto stamp = now();
    if (!has_odom_)
    {
      publishStance(stamp);
      return;
    }
    const double ratio = always_trot_ ? 0.45 : std::clamp(
        (horizontal_speed_ - min_walk_speed_) / std::max(1e-3, max_walk_speed_ - min_walk_speed_),
        0.0, 1.0);
    if (ratio <= 1e-3)
    {
      publishStance(stamp);
      return;
    }
    const double phase = 2.0 * kPi * gait_frequency_ * stamp.seconds();
    updateWheelMotion(stamp);
    fillLeg(0, phase, ratio, true);
    fillLeg(4, phase + kPi, ratio, false);
    fillLeg(8, phase + kPi, ratio, true);
    fillLeg(12, phase, ratio, false);
    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void publishStance(const rclcpp::Time &stamp)
  {
    const std::array<double, 16> stance = {
        0.05, 0.82, -1.58, wheel_position_[0],
        -0.05, 0.82, -1.58, wheel_position_[1],
        0.05, 0.95, -1.62, wheel_position_[2],
        -0.05, 0.95, -1.62, wheel_position_[3]};
    for (size_t i = 0; i < stance.size(); ++i)
    {
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }
    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void updateWheelMotion(const rclcpp::Time &stamp)
  {
    double dt = 0.0;
    if (has_last_publish_time_)
    {
      dt = (stamp - last_publish_time_).seconds();
      if (dt < 0.0 || dt > 0.2) dt = 0.0;
    }
    last_publish_time_ = stamp;
    has_last_publish_time_ = true;
    const double wheel_rate = forward_speed_ / std::max(0.01, wheel_radius_);
    for (double &position : wheel_position_)
    {
      position += wheel_rate * dt;
    }
    wheel_spin_rate_ = wheel_rate;
  }

  void fillLeg(size_t offset, double phase, double ratio, bool left_side)
  {
    const double s = std::sin(phase), c = std::cos(phase), swing = std::max(0.0, s);
    const double side = left_side ? 1.0 : -1.0;
    const bool rear = offset >= 8;
    const size_t leg_index = offset / 4;
    const double thigh_stance = rear ? 0.95 : 0.82;
    const double calf_stance = rear ? -1.62 : -1.58;
    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;
    joint_msg_.position[offset + 2] = calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);
    joint_msg_.position[offset + 3] = wheel_position_[leg_index];
    joint_msg_.position[offset] = std::clamp(joint_msg_.position[offset], -1.0, 1.0);
    joint_msg_.position[offset + 1] = std::clamp(joint_msg_.position[offset + 1], -1.2, 3.0);
    joint_msg_.position[offset + 2] = std::clamp(joint_msg_.position[offset + 2], -2.6, -0.9);
    const double omega = 2.0 * kPi * gait_frequency_;
    joint_msg_.velocity[offset] = side * hip_swing_ * ratio * 0.35 * c * omega;
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * ratio * s * omega;
    joint_msg_.velocity[offset + 2] = calf_swing_ * ratio * (s > 0.0 ? c : 0.0) * omega;
    joint_msg_.velocity[offset + 3] = wheel_spin_rate_;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_msg_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 4> wheel_position_{0.0, 0.0, 0.0, 0.0};
  double horizontal_speed_{0.0}, forward_speed_{0.0}, last_odom_x_{0.0}, last_odom_y_{0.0};
  double gait_frequency_{2.2}, min_walk_speed_{0.05}, max_walk_speed_{1.0};
  double hip_swing_{0.08}, thigh_swing_{0.32}, calf_swing_{0.42};
  double wheel_radius_{0.09}, wheel_spin_rate_{0.0};
  bool always_trot_{false}, has_odom_{false}, has_prev_pose_{false}, has_last_publish_time_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::M20GaitPublisher>());
  rclcpp::shutdown();
  return 0;
}
