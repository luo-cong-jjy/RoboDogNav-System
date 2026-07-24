#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class InspectionMissionNode : public rclcpp::Node {
public:
  InspectionMissionNode() : Node("inspection_mission_node") {
    mission_name_ = declare_parameter<std::string>("mission_name", "odom_inspection_mission");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    target_goal_topic_ = declare_parameter<std::string>(
      "target_goal_topic", "/m20_inspection/tracker_goal");
    rviz_goal_topic_ = declare_parameter<std::string>("rviz_goal_topic", "/goal_pose");
    mission_control_topic_ = declare_parameter<std::string>(
      "mission_control_topic", "/m20_inspection/mission_control");
    tracker_control_topic_ = declare_parameter<std::string>(
      "tracker_control_topic", "/m20_inspection/tracker_control");
    tracker_status_topic_ = declare_parameter<std::string>(
      "tracker_status_topic", "/m20_inspection/tracker_status");
    mission_status_topic_ = declare_parameter<std::string>(
      "mission_status_topic", "/m20_inspection/mission_status");
    mission_marker_topic_ = declare_parameter<std::string>(
      "mission_marker_topic", "/m20_inspection/mission_route_marker");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    start_immediately_ = declare_parameter<bool>("start_immediately", true);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 1.5);
    loop_route_ = declare_parameter<bool>("loop_route", false);
    goal_burst_count_ = declare_parameter<int>("goal_burst_count", 3);
    default_dwell_sec_ = declare_parameter<double>("default_dwell_sec", 1.0);
    marker_scale_ = declare_parameter<double>("marker_scale", 0.14);

    point_names_ = declare_parameter<std::vector<std::string>>(
      "point_names", {"front_door_goal"});
    point_x_ = declare_parameter<std::vector<double>>("point_x", {0.0});
    point_y_ = declare_parameter<std::vector<double>>("point_y", {0.8});
    declare_parameter<std::vector<double>>("point_yaw", std::vector<double>{});
    point_yaw_ = get_parameter("point_yaw").as_double_array();
    declare_parameter<std::vector<double>>("point_dwell_sec", std::vector<double>{});
    point_dwell_sec_ = get_parameter("point_dwell_sec").as_double_array();

    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    start_delay_sec_ = std::max(0.0, start_delay_sec_);
    goal_burst_count_ = std::max(1, goal_burst_count_);
    default_dwell_sec_ = std::max(0.0, default_dwell_sec_);
    marker_scale_ = std::max(0.05, marker_scale_);
    mission_valid_ = validateMission();

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_goal_topic_, 10);
    tracker_control_pub_ = create_publisher<std_msgs::msg::String>(tracker_control_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(mission_status_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(mission_marker_topic_, 10);

    tracker_status_sub_ = create_subscription<std_msgs::msg::String>(
      tracker_status_topic_,
      20,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        tracker_state_ = extractField(msg->data, "state=");
        last_tracker_status_time_ = now();
      });

    rviz_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      rviz_goal_topic_,
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handleRvizGoal(*msg);
      });

    mission_control_sub_ = create_subscription<std_msgs::msg::String>(
      mission_control_topic_,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handleMissionControl(msg->data);
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { onTimer(); });

    RCLCPP_INFO(
      get_logger(),
      "巡检任务管理器已启动：mission=%s points=%zu goal_out=%s tracker_status=%s",
      mission_name_.c_str(),
      point_x_.size(),
      target_goal_topic_.c_str(),
      tracker_status_topic_.c_str());
  }

private:
  enum class State {
    kWaitingStart,
    kDispatchGoal,
    kWaitingArrival,
    kDwell,
    kFinished,
  };

  bool validateMission() {
    if (point_x_.empty() || point_x_.size() != point_y_.size()) {
      RCLCPP_ERROR(get_logger(), "巡检点配置无效：point_x 和 point_y 必须长度一致且不能为空。");
      return false;
    }

    if (!point_yaw_.empty() && point_yaw_.size() != point_x_.size()) {
      RCLCPP_ERROR(get_logger(), "巡检点配置无效：point_yaw 为空或与 point_x 长度一致。");
      return false;
    }

    if (!point_dwell_sec_.empty() && point_dwell_sec_.size() != point_x_.size()) {
      RCLCPP_ERROR(get_logger(), "巡检点配置无效：point_dwell_sec 为空或与 point_x 长度一致。");
      return false;
    }

    if (point_names_.size() != point_x_.size()) {
      point_names_.resize(point_x_.size());
      for (std::size_t i = 0; i < point_names_.size(); ++i) {
        if (point_names_[i].empty()) {
          point_names_[i] = "point_" + std::to_string(i + 1);
        }
      }
    }

    return true;
  }

  void onTimer() {
    const rclcpp::Time current_time = now();
    publishMarkers(current_time);

    if (!mission_valid_) {
      publishStatus("invalid_mission");
      return;
    }

    if (start_time_.nanoseconds() == 0) {
      start_time_ = current_time;
    }

    if (stopped_) {
      publishStatus("stopped");
      return;
    }

    if (paused_) {
      publishStatus("paused");
      return;
    }

    if (state_ == State::kWaitingStart) {
      if (!start_immediately_) {
        publishStatus("waiting_manual_goal");
        return;
      }
      if ((current_time - start_time_).seconds() < start_delay_sec_) {
        publishStatus("waiting_start_delay");
        return;
      }
      dispatchConfiguredPoint(0, current_time);
    }

    switch (state_) {
      case State::kWaitingStart:
        break;
      case State::kDispatchGoal:
        publishGoalBurst(current_time);
        break;
      case State::kWaitingArrival:
        handleWaitingArrival(current_time);
        break;
      case State::kDwell:
        handleDwell(current_time);
        break;
      case State::kFinished:
        publishStatus("finished");
        break;
    }
  }

  void dispatchConfiguredPoint(std::size_t index, const rclcpp::Time & current_time) {
    publishTrackerControl("resume");
    current_point_index_ = index;
    manual_goal_active_ = false;
    stopped_ = false;
    paused_ = false;
    active_goal_ = makePose(point_x_[index], point_y_[index], pointYaw(index), current_time);
    state_ = State::kDispatchGoal;
    goal_send_count_ = 0;
    tracker_state_.clear();
    goal_dispatch_time_ = current_time;

    RCLCPP_INFO(
      get_logger(),
      "下发巡检点 %zu/%zu：%s x=%.2f y=%.2f",
      current_point_index_ + 1,
      point_x_.size(),
      point_names_[current_point_index_].c_str(),
      active_goal_.pose.position.x,
      active_goal_.pose.position.y);
  }

  void publishGoalBurst(const rclcpp::Time & current_time) {
    active_goal_.header.stamp = current_time;
    goal_pub_->publish(active_goal_);
    ++goal_send_count_;

    if (goal_send_count_ >= goal_burst_count_) {
      state_ = State::kWaitingArrival;
    }

    publishStatus("dispatch_goal");
  }

  void handleWaitingArrival(const rclcpp::Time & current_time) {
    publishStatus("waiting_arrival");
    if (tracker_state_ != "finished") {
      return;
    }

    if (last_tracker_status_time_.nanoseconds() <= goal_dispatch_time_.nanoseconds()) {
      return;
    }

    dwell_start_time_ = current_time;
    state_ = State::kDwell;
    RCLCPP_INFO(get_logger(), "到达当前巡检点，开始停留。");
  }

  void handleDwell(const rclcpp::Time & current_time) {
    publishStatus("dwell");

    if ((current_time - dwell_start_time_).seconds() < currentDwellSec()) {
      return;
    }

    if (manual_goal_active_) {
      state_ = State::kFinished;
      return;
    }

    if (current_point_index_ + 1 < point_x_.size()) {
      dispatchConfiguredPoint(current_point_index_ + 1, current_time);
      return;
    }

    if (loop_route_) {
      dispatchConfiguredPoint(0, current_time);
      return;
    }

    state_ = State::kFinished;
    RCLCPP_INFO(get_logger(), "巡检任务完成：%s", mission_name_.c_str());
  }

  void handleRvizGoal(const geometry_msgs::msg::PoseStamped & msg) {
    const rclcpp::Time current_time = now();
    active_goal_ = msg;
    active_goal_.header.frame_id = msg.header.frame_id.empty() ? frame_id_ : msg.header.frame_id;
    active_goal_.header.stamp = current_time;

    manual_goal_active_ = true;
    stopped_ = false;
    paused_ = false;
    publishTrackerControl("resume");
    state_ = State::kDispatchGoal;
    goal_send_count_ = 0;
    tracker_state_.clear();
    goal_dispatch_time_ = current_time;

    RCLCPP_INFO(
      get_logger(),
      "收到 RViz 临时目标：x=%.2f y=%.2f，当前巡检队列暂停。",
      active_goal_.pose.position.x,
      active_goal_.pose.position.y);
  }

  void handleMissionControl(const std::string & command) {
    const rclcpp::Time current_time = now();

    if (command == "pause") {
      if (stopped_) {
        RCLCPP_WARN(get_logger(), "任务已停止，pause 被忽略；如需重新开始请发送 restart。");
        return;
      }
      paused_ = true;
      publishTrackerControl("pause");
      publishStatus("paused");
      RCLCPP_INFO(get_logger(), "巡检任务已暂停。");
      return;
    }

    if (command == "resume") {
      if (stopped_) {
        RCLCPP_WARN(get_logger(), "任务已停止，resume 被忽略；如需重新开始请发送 restart。");
        return;
      }
      paused_ = false;
      publishTrackerControl("resume");
      publishStatus("running");
      RCLCPP_INFO(get_logger(), "巡检任务已继续。");
      return;
    }

    if (command == "stop") {
      paused_ = false;
      stopped_ = true;
      publishTrackerControl("stop");
      publishStatus("stopped");
      RCLCPP_INFO(get_logger(), "巡检任务已停止。");
      return;
    }

    if (command == "restart") {
      if (!mission_valid_) {
        RCLCPP_ERROR(get_logger(), "巡检任务配置无效，无法 restart。");
        return;
      }
      publishTrackerControl("stop");
      stopped_ = false;
      paused_ = false;
      manual_goal_active_ = false;
      tracker_state_.clear();
      dispatchConfiguredPoint(0, current_time);
      publishStatus("restart");
      RCLCPP_INFO(get_logger(), "巡检任务已从第一个点重新开始。");
      return;
    }

    RCLCPP_WARN(get_logger(), "未知巡检任务控制命令：%s", command.c_str());
  }

  void publishTrackerControl(const std::string & command) {
    std_msgs::msg::String msg;
    msg.data = command;
    tracker_control_pub_->publish(msg);
  }

  geometry_msgs::msg::PoseStamped makePose(
    double x,
    double y,
    double yaw,
    const rclcpp::Time & stamp) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    return pose;
  }

  double pointYaw(std::size_t index) const {
    if (point_yaw_.empty() || index >= point_yaw_.size()) {
      return 0.0;
    }
    return point_yaw_[index];
  }

  double currentDwellSec() const {
    if (manual_goal_active_ || point_dwell_sec_.empty() ||
        current_point_index_ >= point_dwell_sec_.size()) {
      return default_dwell_sec_;
    }
    return std::max(0.0, point_dwell_sec_[current_point_index_]);
  }

  void publishStatus(const std::string & state_name) {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "mission=" << mission_name_
           << " state=" << state_name;

    if (manual_goal_active_) {
      stream << " mode=rviz_goal";
    } else if (mission_valid_) {
      stream << " point=" << (current_point_index_ + 1) << "/" << point_x_.size()
             << " name=" << point_names_[current_point_index_];
    }

    stream << " target=(" << active_goal_.pose.position.x << ","
           << active_goal_.pose.position.y << ")"
           << " tracker_state=" << (tracker_state_.empty() ? "unknown" : tracker_state_)
           << " paused=" << (paused_ ? "true" : "false")
           << " stopped=" << (stopped_ ? "true" : "false");

    msg.data = stream.str();
    status_pub_->publish(msg);
  }

  void publishMarkers(const rclcpp::Time & current_time) {
    if (!mission_valid_) {
      return;
    }
    publishRouteLine(current_time);
    publishRoutePoints(current_time);
    publishPointLabels(current_time);
    publishActiveGoalMarker(current_time);
  }

  void publishRouteLine(const rclcpp::Time & current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = frame_id_;
    marker.ns = "inspection_mission";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.03, marker_scale_ * 0.25);
    marker.color.r = 0.05F;
    marker.color.g = 0.9F;
    marker.color.b = 0.55F;
    marker.color.a = 0.85F;

    for (std::size_t i = 0; i < point_x_.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = point_x_[i];
      point.y = point_y_[i];
      point.z = 0.10;
      marker.points.push_back(point);
    }
    marker_pub_->publish(marker);
  }

  void publishRoutePoints(const rclcpp::Time & current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = frame_id_;
    marker.ns = "inspection_mission";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color.r = 1.0F;
    marker.color.g = 0.72F;
    marker.color.b = 0.12F;
    marker.color.a = 0.95F;

    for (std::size_t i = 0; i < point_x_.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = point_x_[i];
      point.y = point_y_[i];
      point.z = 0.16;
      marker.points.push_back(point);
    }
    marker_pub_->publish(marker);
  }

  void publishPointLabels(const rclcpp::Time & current_time) {
    for (std::size_t i = 0; i < point_x_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = current_time;
      marker.header.frame_id = frame_id_;
      marker.ns = "inspection_mission_labels";
      marker.id = static_cast<int>(100 + i);
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = point_x_[i];
      marker.pose.position.y = point_y_[i];
      marker.pose.position.z = 0.45;
      marker.scale.z = marker_scale_ * 1.6;
      marker.color.r = 0.08F;
      marker.color.g = 0.12F;
      marker.color.b = 0.16F;
      marker.color.a = 0.95F;
      marker.text = point_names_[i];
      marker_pub_->publish(marker);
    }
  }

  void publishActiveGoalMarker(const rclcpp::Time & current_time) {
    if (state_ == State::kWaitingStart) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current_time;
    marker.header.frame_id = frame_id_;
    marker.ns = "inspection_mission";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = active_goal_.pose;
    marker.pose.position.z = 0.26;
    marker.scale.x = marker_scale_ * 1.15;
    marker.scale.y = marker_scale_ * 1.15;
    marker.scale.z = marker_scale_ * 1.15;
    marker.color.r = 0.15F;
    marker.color.g = 0.52F;
    marker.color.b = 1.0F;
    marker.color.a = 0.95F;
    marker_pub_->publish(marker);
  }

  static std::string extractField(const std::string & text, const std::string & key) {
    const std::size_t begin = text.find(key);
    if (begin == std::string::npos) {
      return "";
    }

    const std::size_t value_begin = begin + key.size();
    const std::size_t value_end = text.find(' ', value_begin);
    if (value_end == std::string::npos) {
      return text.substr(value_begin);
    }
    return text.substr(value_begin, value_end - value_begin);
  }

  std::string mission_name_;
  std::string frame_id_;
  std::string target_goal_topic_;
  std::string rviz_goal_topic_;
  std::string mission_control_topic_;
  std::string tracker_control_topic_;
  std::string tracker_status_topic_;
  std::string mission_status_topic_;
  std::string mission_marker_topic_;

  double publish_rate_hz_{5.0};
  bool start_immediately_{true};
  double start_delay_sec_{1.5};
  bool loop_route_{false};
  int goal_burst_count_{3};
  double default_dwell_sec_{1.0};
  double marker_scale_{0.14};
  bool mission_valid_{false};
  bool manual_goal_active_{false};
  bool paused_{false};
  bool stopped_{false};

  std::vector<std::string> point_names_;
  std::vector<double> point_x_;
  std::vector<double> point_y_;
  std::vector<double> point_yaw_;
  std::vector<double> point_dwell_sec_;

  std::size_t current_point_index_{0};
  int goal_send_count_{0};
  State state_{State::kWaitingStart};
  std::string tracker_state_;
  geometry_msgs::msg::PoseStamped active_goal_;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_dispatch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tracker_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dwell_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_control_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracker_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspectionMissionNode>());
  rclcpp::shutdown();
  return 0;
}
