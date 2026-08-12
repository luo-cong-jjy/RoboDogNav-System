#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController()
  : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    bidirectional_tracking_enabled_ =
      declare_parameter<bool>("bidirectional_tracking_enabled", false);
    reverse_tracking_enter_angle_ =
      declare_parameter<double>("reverse_tracking_enter_angle", 2.10);
    reverse_tracking_exit_angle_ =
      declare_parameter<double>("reverse_tracking_exit_angle", 1.75);
    reverse_tracking_min_hold_sec_ =
      declare_parameter<double>("reverse_tracking_min_hold_sec", 0.80);
    reverse_tracking_entry_alignment_ =
      declare_parameter<double>("reverse_tracking_entry_alignment", 0.20);
    reverse_tracking_exit_alignment_ =
      declare_parameter<double>("reverse_tracking_exit_alignment", 0.35);
    if (!(0.0 <= reverse_tracking_exit_angle_ &&
      reverse_tracking_exit_angle_ < reverse_tracking_enter_angle_ &&
      reverse_tracking_enter_angle_ <= M_PI &&
      reverse_tracking_min_hold_sec_ >= 0.0 &&
      0.0 < reverse_tracking_entry_alignment_ &&
      reverse_tracking_entry_alignment_ < reverse_tracking_exit_alignment_ &&
      reverse_tracking_exit_alignment_ < M_PI_2))
    {
      throw std::invalid_argument("invalid bidirectional tracking hysteresis");
    }
    const auto execution_hold_topic =
      declare_parameter<std::string>("execution_hold_topic", "/m20/control/execution_hold");
    require_external_execution_hold_ =
      declare_parameter<bool>("require_external_execution_hold", false);
    external_execution_hold_ = require_external_execution_hold_;

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
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>(
      "planning/go2_execution_frozen",
      10);
    tracking_direction_pub_ =
      create_publisher<std_msgs::msg::String>("planning/tracking_direction", 10);
    cmd_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Closed-loop controller ready; external execution hold=%s; "
      "bidirectional M20 tracking=%s",
      require_external_execution_hold_ ? "required" : "optional",
      bidirectional_tracking_enabled_ ? "enabled" : "disabled");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d & value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d & pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4) {
      direction = traj_[1].evaluateDeBoorT(t_cur);
    }
    return direction.head<2>().squaredNorm() < 1e-4 ?
           odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  bool reversePathIsStraight() const
  {
    bool have_direction = false;
    const double horizon = std::min(traj_duration_, 2.0 * time_forward_);
    for (const double ratio : {0.0, 0.5, 1.0}) {
      const double t = ratio * horizon;
      const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(t);
      if (velocity.head<2>().squaredNorm() < 1e-4) {
        continue;
      }
      have_direction = true;
      const double path_yaw = std::atan2(velocity.y(), velocity.x());
      const double reverse_error = normalizeAngle(path_yaw + M_PI - odom_yaw_);
      if (std::abs(reverse_error) > reverse_tracking_exit_alignment_) {
        return false;
      }
    }
    return have_direction;
  }

  void updateTrackingDirection(double forward_yaw)
  {
    const double forward_error = normalizeAngle(forward_yaw - odom_yaw_);
    const double reverse_error = normalizeAngle(forward_yaw + M_PI - odom_yaw_);
    const bool reverse_path_is_straight = reversePathIsStraight();
    if (!bidirectional_tracking_enabled_) {
      reverse_tracking_ = false;
      reverse_tracking_hold_sec_ = 0.0;
    } else if (reverse_tracking_) {
      // Direction changes are evaluated once per B-spline, not at the 100 Hz
      // control rate. This rejects a genuinely curved replan without reacting
      // to a degenerate end tangent of an otherwise straight reverse path.
      if (!reverse_path_is_straight) {
        reverse_tracking_ = false;
        reverse_tracking_hold_sec_ = 0.0;
      } else if (std::abs(forward_error) <= reverse_tracking_exit_angle_ &&
        reverse_tracking_hold_sec_ >= reverse_tracking_min_hold_sec_)
      {
        reverse_tracking_ = false;
        reverse_tracking_hold_sec_ = 0.0;
      }
    } else if (reverse_path_is_straight &&
      std::abs(forward_error) >= reverse_tracking_enter_angle_ &&
      std::abs(reverse_error) <= reverse_tracking_entry_alignment_)
    {
      reverse_tracking_ = true;
      reverse_tracking_hold_sec_ = 0.0;
      reverse_tracking_yaw_ = odom_yaw_;
    }

    const std::string direction = reverse_tracking_ ? "REVERSE" : "FORWARD";
    if (direction != last_tracking_direction_) {
      std_msgs::msg::String message;
      message.data = direction;
      tracking_direction_pub_->publish(message);
      RCLCPP_INFO(get_logger(), "Trajectory tracking direction: %s", direction.c_str());
      last_tracking_direction_ = direction;
    }
  }

  double trackingYaw(double forward_yaw) const
  {
    // Validated reverse execution is straight rolling. Keep the body heading
    // captured on entry instead of following noisy/degenerate endpoint
    // tangents from a completed B-spline.
    return reverse_tracking_ ? reverse_tracking_yaw_ : forward_yaw;
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

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    }
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {knots(i) = msg->knots[i];}
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    const Eigen::Vector3d initial_position = traj_[0].evaluateDeBoorT(0.0);
    updateTrackingDirection(estimateDesiredYaw(0.0, initial_position));
    RCLCPP_INFO(
      get_logger(), "Received trajectory %lld, duration %.3fs",
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
    if (!require_external_execution_hold_) {
      external_execution_hold_ = false;
      return;
    }
    if (external_execution_hold_ != msg->data) {
      RCLCPP_INFO(
        get_logger(), "External trajectory hold %s",
        msg->data ? "asserted" : "released");
    }
    external_execution_hold_ = msg->data;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_) {
      publishExecutionFrozen(external_execution_hold_);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) {dt = 0.0;}
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double forward_yaw = estimateDesiredYaw(t_eval, pos_des);
    reverse_tracking_hold_sec_ +=
      reverse_tracking_ ? std::max(0.0, dt) : 0.0;
    const double tracking_yaw = trackingYaw(forward_yaw);
    const double yaw_error = normalizeAngle(tracking_yaw - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_) {
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
    if (!external_execution_hold_) {
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    }
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
      Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
      std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_) {
      command = geometry_msgs::msg::Twist();
    }
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_direction_pub_;
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
  bool bidirectional_tracking_enabled_{false};
  bool reverse_tracking_{false};
  double reverse_tracking_hold_sec_{0.0};
  double reverse_tracking_yaw_{0.0};
  std::string last_tracking_direction_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double reverse_tracking_enter_angle_, reverse_tracking_exit_angle_;
  double reverse_tracking_min_hold_sec_;
  double reverse_tracking_entry_alignment_, reverse_tracking_exit_alignment_;
};
}  // namespace scan_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
