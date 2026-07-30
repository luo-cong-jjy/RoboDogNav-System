#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    heading_error_resume_threshold_ =
        declare_parameter<double>("heading_error_resume_threshold", heading_error_threshold_);
    heading_slowdown_threshold_ =
        declare_parameter<double>("heading_slowdown_threshold", heading_error_threshold_);
    heading_alignment_min_hold_sec_ =
        declare_parameter<double>("heading_alignment_min_hold_sec", 0.0);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    const auto execution_hold_topic =
        declare_parameter<std::string>("execution_hold_topic", "/m20/control/execution_hold");
    require_external_execution_hold_ =
        declare_parameter<bool>("require_external_execution_hold", false);
    external_execution_hold_ = require_external_execution_hold_;
    heading_error_resume_threshold_ =
        std::clamp(heading_error_resume_threshold_, 0.0, heading_error_threshold_);
    heading_slowdown_threshold_ =
        std::clamp(heading_slowdown_threshold_, 0.0, heading_error_threshold_);
    heading_alignment_min_hold_sec_ = std::max(0.0, heading_alignment_min_hold_sec_);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    execution_hold_sub_ = create_subscription<std_msgs::msg::Bool>(
        execution_hold_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ClosedLoopController::executionHoldCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    heading_error_pub_ = create_publisher<std_msgs::msg::Float64>("heading_error", 20);
    heading_aligning_pub_ = create_publisher<std_msgs::msg::Bool>(
        "heading_aligning", rclcpp::QoS(1).reliable().transient_local());
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    heading_alignment_started_ = now();
    publishHeadingAligning();
    RCLCPP_INFO(
        get_logger(),
        "Closed-loop controller ready; external execution hold=%s, "
        "heading alignment enter=%.3frad exit=%.3frad slowdown=%.3frad hold=%.2fs",
        require_external_execution_hold_ ? "required" : "optional",
        heading_error_threshold_, heading_error_resume_threshold_,
        heading_slowdown_threshold_, heading_alignment_min_hold_sec_);
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void publishHeadingAligning()
  {
    std_msgs::msg::Bool msg;
    msg.data = heading_aligning_;
    heading_aligning_pub_->publish(msg);
  }

  void updateHeadingAlignment(
      double absolute_yaw_error, const rclcpp::Time &current_time)
  {
    if (!heading_aligning_ && absolute_yaw_error >= heading_error_threshold_)
    {
      heading_aligning_ = true;
      heading_alignment_started_ = current_time;
      publishHeadingAligning();
      RCLCPP_INFO(
          get_logger(), "Heading alignment entered at error %.3frad",
          absolute_yaw_error);
      return;
    }
    if (!heading_aligning_) return;

    const double held_sec = (current_time - heading_alignment_started_).seconds();
    if (held_sec >= heading_alignment_min_hold_sec_ &&
        absolute_yaw_error <= heading_error_resume_threshold_)
    {
      heading_aligning_ = false;
      publishHeadingAligning();
      RCLCPP_INFO(
          get_logger(), "Heading alignment released at error %.3frad after %.2fs",
          absolute_yaw_error, held_sec);
    }
  }

  double headingTranslationScale(double absolute_yaw_error) const
  {
    if (heading_error_threshold_ <= heading_slowdown_threshold_ + 1e-9)
      return absolute_yaw_error < heading_error_threshold_ ? 1.0 : 0.0;
    return std::clamp(
        (heading_error_threshold_ - absolute_yaw_error) /
            (heading_error_threshold_ - heading_slowdown_threshold_),
        0.0, 1.0);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void executionHoldCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (external_execution_hold_ != msg->data)
    {
      RCLCPP_INFO(
          get_logger(), "External trajectory hold %s",
          msg->data ? "asserted" : "released");
    }
    external_execution_hold_ = msg->data;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(external_execution_hold_);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    std_msgs::msg::Float64 heading_error_msg;
    heading_error_msg.data = yaw_error;
    heading_error_pub_->publish(heading_error_msg);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    const double absolute_yaw_error = std::abs(yaw_error);
    updateHeadingAlignment(absolute_yaw_error, current_time);
    if (heading_aligning_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    // A downstream safety hold blocks execution but must not erase the
    // upstream candidate command: collision prediction needs to keep checking
    // the motion that would execute after release. Freeze only the trajectory
    // clock here; the safety supervisor remains the sole zero-command gate.
    publishExecutionFrozen(external_execution_hold_);
    if (!external_execution_hold_)
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    // A wheel-legged M20 cannot realize SCAN's holonomic tangent change
    // instantaneously. Slow translation before the hard alignment gate so
    // the measured body heading catches the B-spline direction without
    // cutting toward an inflated obstacle.
    vel_world *= headingTranslationScale(absolute_yaw_error);
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heading_aligning_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_hold_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  bool require_external_execution_hold_{false};
  bool external_execution_hold_{false};
  bool heading_aligning_{false};
  rclcpp::Time heading_alignment_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, heading_error_resume_threshold_;
  double heading_slowdown_threshold_, heading_alignment_min_hold_sec_;
  double kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
