#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OdomWaypointPatrolNode : public rclcpp::Node {
public:
  OdomWaypointPatrolNode() : Node("odom_waypoint_patrol_node") {
    route_name_ = declare_parameter<std::string>("route_name", "odom_demo_route");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/m20_inspection/cmd_vel_nav");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/m20_inspection/patrol_status");
    goal_topic_ = declare_parameter<std::string>(
      "goal_topic", "/m20_inspection/current_goal");
    goal_marker_topic_ = declare_parameter<std::string>(
      "goal_marker_topic", "/m20_inspection/current_goal_marker");
    route_marker_topic_ = declare_parameter<std::string>(
      "route_marker_topic", "/m20_inspection/route_marker");
    odom_trace_topic_ = declare_parameter<std::string>(
      "odom_trace_topic", "/m20_inspection/odom_trace");
    goal_input_topic_ = declare_parameter<std::string>("goal_input_topic", "/goal_pose");
    control_topic_ = declare_parameter<std::string>(
      "control_topic", "/m20_inspection/tracker_control");
    enable_runtime_goal_ = declare_parameter<bool>("enable_runtime_goal", true);
    wait_for_runtime_goal_ = declare_parameter<bool>("wait_for_runtime_goal", false);
    runtime_goal_requires_yaw_ = declare_parameter<bool>("runtime_goal_requires_yaw", true);
    marker_frame_id_ = declare_parameter<std::string>("marker_frame_id", "odom");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 1.0);
    loop_route_ = declare_parameter<bool>("loop_route", false);

    waypoint_x_ = declare_parameter<std::vector<double>>("waypoint_x", {0.0, 1.0, 1.0, 0.0});
    waypoint_y_ = declare_parameter<std::vector<double>>("waypoint_y", {0.0, 0.0, 1.0, 1.0});
    declare_parameter<std::vector<double>>("waypoint_yaw", std::vector<double>{});
    waypoint_yaw_ = get_parameter("waypoint_yaw").as_double_array();

    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.18);
    yaw_tolerance_rad_ = declare_parameter<double>("yaw_tolerance_rad", 0.25);
    dwell_time_sec_ = declare_parameter<double>("dwell_time_sec", 1.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.25);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.45);
    linear_gain_ = declare_parameter<double>("linear_gain", 0.7);
    angular_gain_ = declare_parameter<double>("angular_gain", 1.2);
    rotate_in_place_error_rad_ = declare_parameter<double>("rotate_in_place_error_rad", 0.65);
    tracking_mode_ = declare_parameter<std::string>("tracking_mode", "point_p");
    pure_pursuit_lookahead_distance_ = declare_parameter<double>(
      "pure_pursuit_lookahead_distance", 0.45);
    pure_pursuit_min_linear_speed_ = declare_parameter<double>(
      "pure_pursuit_min_linear_speed", 0.04);
    pure_pursuit_slowdown_distance_ = declare_parameter<double>(
      "pure_pursuit_slowdown_distance", 0.55);
    pure_pursuit_curvature_gain_ = declare_parameter<double>(
      "pure_pursuit_curvature_gain", 1.0);
    odom_trace_max_points_ = declare_parameter<int>("odom_trace_max_points", 600);
    odom_trace_min_interval_m_ = declare_parameter<double>("odom_trace_min_interval_m", 0.04);
    odom_trace_z_ = declare_parameter<double>("odom_trace_z", 0.04);
    goal_marker_scale_ = declare_parameter<double>("goal_marker_scale", 0.18);
    route_marker_scale_ = declare_parameter<double>("route_marker_scale", 0.08);

    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    start_delay_sec_ = std::max(0.0, start_delay_sec_);
    pure_pursuit_lookahead_distance_ = std::max(0.05, pure_pursuit_lookahead_distance_);
    pure_pursuit_min_linear_speed_ = std::max(0.0, pure_pursuit_min_linear_speed_);
    pure_pursuit_slowdown_distance_ = std::max(0.05, pure_pursuit_slowdown_distance_);
    odom_trace_max_points_ = std::max(10, odom_trace_max_points_);
    odom_trace_min_interval_m_ = std::max(0.0, odom_trace_min_interval_m_);
    if (tracking_mode_ != "point_p" && tracking_mode_ != "pure_pursuit") {
      RCLCPP_WARN(
        get_logger(),
        "未知 tracking_mode=%s，回退为 point_p。",
        tracking_mode_.c_str());
      tracking_mode_ = "point_p";
    }
    route_valid_ = validateRoute();
    has_active_goal_ = route_valid_ && !wait_for_runtime_goal_;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(goal_marker_topic_, 10);
    route_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(route_marker_topic_, 10);
    odom_trace_pub_ = create_publisher<nav_msgs::msg::Path>(odom_trace_topic_, 10);
    control_sub_ = create_subscription<std_msgs::msg::String>(
      control_topic_,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handleControlCommand(msg->data);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_frame_id_ = msg->header.frame_id.empty() ? marker_frame_id_ : msg->header.frame_id;
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
        last_odom_time_ = now();
        has_odom_ = true;
        appendOdomTrace(*msg);
      });

    if (enable_runtime_goal_) {
      goal_input_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_input_topic_,
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          handleRuntimeGoal(*msg);
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { onTimer(); });

    RCLCPP_INFO(
      get_logger(),
      "odom 坐标巡检节点已启动：route=%s odom=%s output=%s waypoints=%zu tracking=%s wait_for_runtime_goal=%s",
      route_name_.c_str(),
      odom_topic_.c_str(),
      cmd_topic_.c_str(),
      waypoint_x_.size(),
      tracking_mode_.c_str(),
      wait_for_runtime_goal_ ? "true" : "false");

    if (enable_runtime_goal_) {
      RCLCPP_INFO(
        get_logger(),
        "已启用 RViz 临时目标点输入：topic=%s",
        goal_input_topic_.c_str());
    }
  }

private:
  enum class State {
    kWaitStart,
    kGoToWaypoint,
    kDwell,
    kFinished,
  };

  bool validateRoute() {
    if (waypoint_x_.empty() || waypoint_x_.size() != waypoint_y_.size()) {
      RCLCPP_ERROR(get_logger(), "waypoint_x 和 waypoint_y 必须长度一致且不能为空。");
      return false;
    }

    if (!waypoint_yaw_.empty() && waypoint_yaw_.size() != waypoint_x_.size()) {
      RCLCPP_ERROR(get_logger(), "waypoint_yaw 为空或与 waypoint_x 长度一致。");
      return false;
    }

    return true;
  }

  void onTimer() {
    const rclcpp::Time current_time = now();

    if (!route_valid_) {
      publishStop();
      publishDebug("invalid_route", current_time, 0.0, 0.0);
      return;
    }

    if (paused_) {
      publishStop();
      publishDebug("paused", current_time, 0.0, 0.0);
      return;
    }

    if (!has_active_goal_) {
      publishStop();
      publishDebug("waiting_goal", current_time, 0.0, 0.0);
      return;
    }

    if (start_time_.nanoseconds() == 0) {
      start_time_ = current_time;
      RCLCPP_INFO(get_logger(), "odom 坐标巡检将在 %.2fs 后开始。", start_delay_sec_);
    }

    if ((current_time - start_time_).seconds() < start_delay_sec_) {
      publishStop();
      publishDebug("waiting_start", current_time, 0.0, 0.0);
      return;
    }

    if (!hasFreshOdom(current_time)) {
      publishStop();
      publishDebug("waiting_odom", current_time, 0.0, 0.0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待有效 /odom。");
      return;
    }

    double distance = 0.0;
    double heading_error = 0.0;
    switch (state_) {
      case State::kWaitStart:
        enterWaypoint(0);
        break;
      case State::kGoToWaypoint:
        driveToCurrentWaypoint(distance, heading_error);
        break;
      case State::kDwell:
        handleDwell(current_time);
        break;
      case State::kFinished:
        publishStop();
        break;
    }

    publishDebug(stateName(state_), current_time, distance, heading_error);
  }

  bool hasFreshOdom(const rclcpp::Time & current_time) const {
    if (!has_odom_ || last_odom_time_.nanoseconds() == 0) {
      return false;
    }
    return (current_time - last_odom_time_).seconds() <= odom_timeout_sec_;
  }

  void enterWaypoint(std::size_t index) {
    current_waypoint_index_ = index;
    state_ = State::kGoToWaypoint;
    RCLCPP_INFO(
      get_logger(),
      "前往 waypoint %zu/%zu: x=%.2f y=%.2f",
      current_waypoint_index_ + 1,
      waypoint_x_.size(),
      waypoint_x_[current_waypoint_index_],
      waypoint_y_[current_waypoint_index_]);
  }

  void driveToCurrentWaypoint(double & distance, double & heading_error) {
    const double target_x = waypoint_x_[current_waypoint_index_];
    const double target_y = waypoint_y_[current_waypoint_index_];
    const double dx = target_x - current_x_;
    const double dy = target_y - current_y_;
    distance = std::hypot(dx, dy);

    if (handleReachedWaypoint(distance)) {
      return;
    }

    if (tracking_mode_ == "pure_pursuit") {
      driveWithPurePursuit(distance, heading_error);
      return;
    }

    driveWithPointP(distance, heading_error);
  }

  bool handleReachedWaypoint(double distance) {
    if (distance <= goal_tolerance_m_) {
      if (!waypoint_yaw_.empty()) {
        const double yaw_error = normalizeAngle(waypoint_yaw_[current_waypoint_index_] - current_yaw_);
        if (std::fabs(yaw_error) > yaw_tolerance_rad_) {
          publishRotate(yaw_error);
          return true;
        }
      }

      state_ = State::kDwell;
      dwell_start_time_ = now();
      publishStop();
      RCLCPP_INFO(get_logger(), "到达 waypoint %zu，开始停留。", current_waypoint_index_ + 1);
      return true;
    }

    return false;
  }

  void driveWithPointP(double distance, double & heading_error) {
    const double target_x = waypoint_x_[current_waypoint_index_];
    const double target_y = waypoint_y_[current_waypoint_index_];
    const double dx = target_x - current_x_;
    const double dy = target_y - current_y_;
    const double target_heading = std::atan2(dy, dx);
    heading_error = normalizeAngle(target_heading - current_yaw_);

    geometry_msgs::msg::Twist command;
    command.angular.z = clamp(angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);

    if (std::fabs(heading_error) < rotate_in_place_error_rad_) {
      command.linear.x = clamp(linear_gain_ * distance, 0.0, max_linear_speed_);
    }

    cmd_pub_->publish(command);
  }

  void driveWithPurePursuit(double distance, double & heading_error) {
    double lookahead_x = waypoint_x_[current_waypoint_index_];
    double lookahead_y = waypoint_y_[current_waypoint_index_];
    computeLookaheadPoint(lookahead_x, lookahead_y);

    const double dx = lookahead_x - current_x_;
    const double dy = lookahead_y - current_y_;
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    const double robot_x = cos_yaw * dx + sin_yaw * dy;
    const double robot_y = -sin_yaw * dx + cos_yaw * dy;
    const double lookahead_distance = std::max(0.05, std::hypot(robot_x, robot_y));
    heading_error = std::atan2(robot_y, robot_x);

    if (std::fabs(heading_error) > rotate_in_place_error_rad_) {
      publishRotate(heading_error);
      return;
    }

    const double slowdown_scale = clamp(distance / pure_pursuit_slowdown_distance_, 0.0, 1.0);
    double linear_speed = max_linear_speed_ * slowdown_scale;
    if (distance > goal_tolerance_m_ * 1.5) {
      linear_speed = std::max(pure_pursuit_min_linear_speed_, linear_speed);
    }
    linear_speed = clamp(linear_speed, 0.0, max_linear_speed_);

    // 纯跟踪根据前视点在机器人坐标系下的横向偏差计算曲率。
    const double curvature = 2.0 * robot_y / (lookahead_distance * lookahead_distance);
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_speed;
    command.angular.z = clamp(
      pure_pursuit_curvature_gain_ * linear_speed * curvature,
      -max_angular_speed_,
      max_angular_speed_);
    cmd_pub_->publish(command);
  }

  void computeLookaheadPoint(double & lookahead_x, double & lookahead_y) const {
    double remaining = pure_pursuit_lookahead_distance_;
    double anchor_x = current_x_;
    double anchor_y = current_y_;

    for (std::size_t i = current_waypoint_index_; i < waypoint_x_.size(); ++i) {
      const double segment_dx = waypoint_x_[i] - anchor_x;
      const double segment_dy = waypoint_y_[i] - anchor_y;
      const double segment_length = std::hypot(segment_dx, segment_dy);

      if (segment_length < 1e-6) {
        anchor_x = waypoint_x_[i];
        anchor_y = waypoint_y_[i];
        continue;
      }

      if (segment_length >= remaining) {
        const double ratio = remaining / segment_length;
        lookahead_x = anchor_x + segment_dx * ratio;
        lookahead_y = anchor_y + segment_dy * ratio;
        return;
      }

      remaining -= segment_length;
      anchor_x = waypoint_x_[i];
      anchor_y = waypoint_y_[i];
    }

    lookahead_x = waypoint_x_[current_waypoint_index_];
    lookahead_y = waypoint_y_[current_waypoint_index_];
  }

  void handleDwell(const rclcpp::Time & current_time) {
    publishStop();
    if ((current_time - dwell_start_time_).seconds() < dwell_time_sec_) {
      return;
    }

    if (current_waypoint_index_ + 1 < waypoint_x_.size()) {
      enterWaypoint(current_waypoint_index_ + 1);
      return;
    }

    if (loop_route_) {
      enterWaypoint(0);
      return;
    }

    state_ = State::kFinished;
    RCLCPP_INFO(get_logger(), "odom 坐标巡检路线完成：%s", route_name_.c_str());
  }

  void publishRotate(double yaw_error) {
    geometry_msgs::msg::Twist command;
    command.angular.z = clamp(angular_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    cmd_pub_->publish(command);
  }

  void publishStop() {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publishDebug(
    const std::string & state_name,
    const rclcpp::Time & current_time,
    double distance,
    double heading_error) {
    publishStatus(state_name, distance, heading_error);
    if (!hasValidCurrentWaypoint()) {
      return;
    }
    publishCurrentGoal(current_time);
    publishGoalMarker(current_time);
    publishRouteMarkers(current_time);
  }

  void publishStatus(const std::string & state_name, double distance, double heading_error) {
    std_msgs::msg::String status_msg;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "route=" << route_name_
           << " state=" << state_name
           << " tracking_mode=" << tracking_mode_;

    if (!has_active_goal_) {
      stream << " waiting_goal=true";
    } else if (!hasValidCurrentWaypoint()) {
      stream << " invalid_route=true";
    } else {
      stream << " waypoint=" << (current_waypoint_index_ + 1) << "/" << waypoint_x_.size()
             << " target=(" << waypoint_x_[current_waypoint_index_] << ","
             << waypoint_y_[current_waypoint_index_] << ")";
    }

    stream << " current=(" << current_x_ << "," << current_y_ << ")"
           << " yaw=" << current_yaw_
           << " distance=" << distance
           << " heading_error=" << heading_error
           << " paused=" << (paused_ ? "true" : "false");
    status_msg.data = stream.str();
    status_pub_->publish(status_msg);
  }

  bool hasValidCurrentWaypoint() const {
    return has_active_goal_ && route_valid_ && current_waypoint_index_ < waypoint_x_.size() &&
           current_waypoint_index_ < waypoint_y_.size();
  }

  void publishCurrentGoal(const rclcpp::Time & current_time) {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = current_time;
    goal_msg.header.frame_id = currentFrame();
    goal_msg.pose.position.x = waypoint_x_[current_waypoint_index_];
    goal_msg.pose.position.y = waypoint_y_[current_waypoint_index_];
    goal_msg.pose.position.z = 0.0;

    const double yaw = waypoint_yaw_.empty() ? current_yaw_ : waypoint_yaw_[current_waypoint_index_];
    goal_msg.pose.orientation = quaternionFromYaw(yaw);
    goal_pub_->publish(goal_msg);
  }

  void publishGoalMarker(const rclcpp::Time & current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = currentFrame();
    marker.ns = "odom_waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = waypoint_x_[current_waypoint_index_];
    marker.pose.position.y = waypoint_y_[current_waypoint_index_];
    marker.pose.position.z = 0.18;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = goal_marker_scale_;
    marker.scale.y = goal_marker_scale_;
    marker.scale.z = goal_marker_scale_;
    marker.color.r = 0.1F;
    marker.color.g = 0.55F;
    marker.color.b = 1.0F;
    marker.color.a = 0.9F;
    goal_marker_pub_->publish(marker);
  }

  void publishRouteMarkers(const rclcpp::Time & current_time) {
    publishRouteLine(current_time);
    publishRoutePoints(current_time);
  }

  void publishRouteLine(const rclcpp::Time & current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = currentFrame();
    marker.ns = "odom_route";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.02, route_marker_scale_ * 0.35);
    marker.color.r = 0.0F;
    marker.color.g = 0.85F;
    marker.color.b = 0.35F;
    marker.color.a = 0.9F;

    for (std::size_t i = 0; i < waypoint_x_.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = waypoint_x_[i];
      point.y = waypoint_y_[i];
      point.z = 0.08;
      marker.points.push_back(point);
    }

    route_marker_pub_->publish(marker);
  }

  void publishRoutePoints(const rclcpp::Time & current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = currentFrame();
    marker.ns = "odom_route";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = route_marker_scale_;
    marker.scale.y = route_marker_scale_;
    marker.scale.z = route_marker_scale_;
    marker.color.r = 1.0F;
    marker.color.g = 0.7F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;

    for (std::size_t i = 0; i < waypoint_x_.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = waypoint_x_[i];
      point.y = waypoint_y_[i];
      point.z = 0.14;
      marker.points.push_back(point);
    }

    route_marker_pub_->publish(marker);
  }

  void handleRuntimeGoal(const geometry_msgs::msg::PoseStamped & msg) {
    const std::string frame_id = msg.header.frame_id.empty() ? currentFrame() : msg.header.frame_id;
    if (frame_id != currentFrame()) {
      RCLCPP_WARN(
        get_logger(),
        "收到临时目标点 frame=%s，当前按 %s 坐标直接使用，请确认 RViz Fixed Frame。",
        frame_id.c_str(),
        currentFrame().c_str());
    }

    waypoint_x_ = {msg.pose.position.x};
    waypoint_y_ = {msg.pose.position.y};
    const double goal_yaw = yawFromQuaternion(msg.pose.orientation);
    if (runtime_goal_requires_yaw_) {
      waypoint_yaw_ = {goal_yaw};
    } else {
      waypoint_yaw_.clear();
    }
    route_name_ = "runtime_goal";
    route_valid_ = validateRoute();
    if (!route_valid_) {
      has_active_goal_ = false;
      publishStop();
      return;
    }

    has_active_goal_ = true;
    current_waypoint_index_ = 0;
    state_ = State::kGoToWaypoint;
    start_time_ = now() - rclcpp::Duration::from_seconds(start_delay_sec_);
    dwell_start_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    RCLCPP_INFO(
      get_logger(),
      "收到运行时目标点：x=%.2f y=%.2f yaw=%.2f require_yaw=%s",
      waypoint_x_.front(),
      waypoint_y_.front(),
      goal_yaw,
      runtime_goal_requires_yaw_ ? "true" : "false");
  }

  void handleControlCommand(const std::string & command) {
    if (command == "pause") {
      paused_ = true;
      publishStop();
      RCLCPP_INFO(get_logger(), "目标跟踪器已暂停。");
      return;
    }

    if (command == "resume") {
      paused_ = false;
      RCLCPP_INFO(get_logger(), "目标跟踪器已继续。");
      return;
    }

    if (command == "stop") {
      paused_ = false;
      has_active_goal_ = false;
      current_waypoint_index_ = 0;
      state_ = State::kWaitStart;
      publishStop();
      RCLCPP_INFO(get_logger(), "目标跟踪器已停止并清空当前目标。");
      return;
    }

    if (command == "restart") {
      paused_ = false;
      if (has_active_goal_) {
        current_waypoint_index_ = 0;
        state_ = State::kGoToWaypoint;
      }
      RCLCPP_INFO(get_logger(), "目标跟踪器收到 restart。");
      return;
    }

    RCLCPP_WARN(get_logger(), "未知目标跟踪器控制命令：%s", command.c_str());
  }

  void appendOdomTrace(const nav_msgs::msg::Odometry & msg) {
    const double x = msg.pose.pose.position.x;
    const double y = msg.pose.pose.position.y;

    if (!odom_trace_.poses.empty()) {
      const auto & last_pose = odom_trace_.poses.back().pose.position;
      if (std::hypot(x - last_pose.x, y - last_pose.y) < odom_trace_min_interval_m_) {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.header.frame_id = msg.header.frame_id.empty() ? currentFrame() : msg.header.frame_id;
    pose.pose = msg.pose.pose;
    pose.pose.position.z = odom_trace_z_;

    odom_trace_.header.stamp = pose.header.stamp;
    odom_trace_.header.frame_id = pose.header.frame_id;
    odom_trace_.poses.push_back(pose);

    while (static_cast<int>(odom_trace_.poses.size()) > odom_trace_max_points_) {
      odom_trace_.poses.erase(odom_trace_.poses.begin());
    }

    odom_trace_pub_->publish(odom_trace_);
  }

  std::string currentFrame() const {
    return odom_frame_id_.empty() ? marker_frame_id_ : odom_frame_id_;
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
  }

  static std::string stateName(State state) {
    switch (state) {
      case State::kWaitStart:
        return "wait_start";
      case State::kGoToWaypoint:
        return "go_to_waypoint";
      case State::kDwell:
        return "dwell";
      case State::kFinished:
        return "finished";
    }
    return "unknown";
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double clamp(double value, double min_value, double max_value) {
    return std::min(std::max(value, min_value), max_value);
  }

  std::string route_name_;
  std::string odom_topic_;
  std::string cmd_topic_;
  std::string status_topic_;
  std::string goal_topic_;
  std::string goal_marker_topic_;
  std::string route_marker_topic_;
  std::string odom_trace_topic_;
  std::string goal_input_topic_;
  std::string control_topic_;
  std::string marker_frame_id_;
  std::string odom_frame_id_;
  std::string tracking_mode_;
  double publish_rate_hz_{10.0};
  double start_delay_sec_{1.0};
  bool loop_route_{false};
  bool route_valid_{false};
  bool enable_runtime_goal_{true};
  bool wait_for_runtime_goal_{false};
  bool runtime_goal_requires_yaw_{true};
  bool has_active_goal_{false};
  bool paused_{false};

  std::vector<double> waypoint_x_;
  std::vector<double> waypoint_y_;
  std::vector<double> waypoint_yaw_;

  double goal_tolerance_m_{0.18};
  double yaw_tolerance_rad_{0.25};
  double dwell_time_sec_{1.0};
  double odom_timeout_sec_{0.5};
  double max_linear_speed_{0.25};
  double max_angular_speed_{0.45};
  double linear_gain_{0.7};
  double angular_gain_{1.2};
  double rotate_in_place_error_rad_{0.65};
  double pure_pursuit_lookahead_distance_{0.45};
  double pure_pursuit_min_linear_speed_{0.04};
  double pure_pursuit_slowdown_distance_{0.55};
  double pure_pursuit_curvature_gain_{1.0};
  int odom_trace_max_points_{600};
  double odom_trace_min_interval_m_{0.04};
  double odom_trace_z_{0.04};
  double goal_marker_scale_{0.18};
  double route_marker_scale_{0.08};

  bool has_odom_{false};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  std::size_t current_waypoint_index_{0};
  State state_{State::kWaitStart};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dwell_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_trace_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_input_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path odom_trace_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomWaypointPatrolNode>());
  rclcpp::shutdown();
  return 0;
}
