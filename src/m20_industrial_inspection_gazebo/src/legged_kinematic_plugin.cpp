#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/pointer_cast.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/physics.hh"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace m20_industrial_inspection_gazebo {
namespace {

template <typename T>
T sdfValue(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback) {
  if (!sdf || !sdf->HasElement(name)) {
    return fallback;
  }
  return sdf->Get<T>(name);
}

std::string sanitizeName(std::string name) {
  for (char & c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      c = '_';
    }
  }
  return name;
}

}  // namespace

class M20LeggedKinematicPlugin : public gazebo::ModelPlugin {
public:
  M20LeggedKinematicPlugin() = default;

  ~M20LeggedKinematicPlugin() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();

    cmd_topic_ = sdfValue<std::string>(sdf, "cmd_topic", "/cmd_vel");
    odom_topic_ = sdfValue<std::string>(sdf, "odom_topic", "/odom");
    joint_states_topic_ = sdfValue<std::string>(sdf, "joint_states_topic", "/joint_states");
    odom_frame_id_ = sdfValue<std::string>(sdf, "odom_frame_id", "odom");
    base_frame_id_ = sdfValue<std::string>(sdf, "base_frame_id", "base_link");
    max_linear_speed_ = sdfValue<double>(sdf, "max_linear_speed", 0.45);
    max_lateral_speed_ = sdfValue<double>(sdf, "max_lateral_speed", 0.35);
    max_angular_speed_ = sdfValue<double>(sdf, "max_angular_speed", 1.05);
    command_timeout_sec_ = sdfValue<double>(sdf, "command_timeout_sec", 0.5);
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate_hz", 100.0);
    body_height_ = sdfValue<double>(sdf, "body_height", 0.59);
    terrain_following_ = sdfValue<bool>(sdf, "terrain_following", true);
    terrain_probe_start_above_ = sdfValue<double>(sdf, "terrain_probe_start_above", 1.15);
    terrain_probe_end_below_ = sdfValue<double>(sdf, "terrain_probe_end_below", 1.20);
    max_terrain_step_up_ = sdfValue<double>(sdf, "max_terrain_step_up", 0.35);
    max_terrain_step_down_ = sdfValue<double>(sdf, "max_terrain_step_down", 0.75);
    max_vertical_speed_ = sdfValue<double>(sdf, "max_vertical_speed", 0.80);
    disable_robot_collisions_ = sdfValue<bool>(sdf, "disable_robot_collisions", true);
    publish_odom_ = sdfValue<bool>(sdf, "publish_odom", true);
    publish_tf_ = sdfValue<bool>(sdf, "publish_tf", false);
    publish_joint_states_ = sdfValue<bool>(sdf, "publish_joint_states", true);
    animate_gait_ = sdfValue<bool>(sdf, "animate_gait", true);
    gait_frequency_ = sdfValue<double>(sdf, "gait_frequency", 2.2);
    min_walk_speed_ = sdfValue<double>(sdf, "min_walk_speed", 0.05);
    max_walk_speed_ = sdfValue<double>(sdf, "max_walk_speed", 1.0);
    hip_swing_ = sdfValue<double>(sdf, "hip_swing", 0.08);
    thigh_swing_ = sdfValue<double>(sdf, "thigh_swing", 0.32);
    calf_swing_ = sdfValue<double>(sdf, "calf_swing", 0.42);
    wheel_radius_ = sdfValue<double>(sdf, "wheel_radius", 0.09);

    if (disable_robot_collisions_) {
      model_->SetCollideMode("none");
    }
    model_->SetGravityMode(false);
    for (const auto & link : model_->GetLinks()) {
      link->SetGravityMode(false);
      link->SetKinematic(true);
      if (disable_robot_collisions_) {
        link->SetCollideMode("none");
      }
    }

    joint_msg_.name = {
      "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
      "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
      "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
      "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
    gazebo_joints_.reserve(joint_msg_.name.size());
    for (const auto & name : joint_msg_.name) {
      gazebo_joints_.push_back(model_->GetJoint(name));
      if (!gazebo_joints_.back()) {
        gzerr << "[M20LeggedKinematicPlugin] Cannot find joint: " << name << "\n";
      }
    }
    base_link_ = getLink(base_frame_id_);
    initializeLegChains();

    if (terrain_following_ && world_ && world_->Physics()) {
      auto shape = world_->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr());
      ray_shape_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(shape);
      if (!ray_shape_) {
        gzerr << "[M20LeggedKinematicPlugin] Terrain ray creation failed; z will stay kinematic.\n";
      }
    }

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    const std::string node_name = "m20_legged_kinematic_" + sanitizeName(model_->GetName());
    node_ = std::make_shared<rclcpp::Node>(node_name);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_,
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = *msg;
        last_cmd_wall_time_ = std::chrono::steady_clock::now();
        has_cmd_ = true;
      });

    if (publish_odom_) {
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 30);
    }
    if (publish_joint_states_) {
      joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&M20LeggedKinematicPlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[M20LeggedKinematicPlugin] Loaded for " << model_->GetName()
          << ", cmd_topic=" << cmd_topic_
          << ", odom_topic=" << odom_topic_
          << ", terrain_following=" << terrain_following_
          << ", body_height=" << body_height_
          << ", publish_tf=" << publish_tf_ << "\n";
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  struct LegChain {
    gazebo::physics::LinkPtr hipx;
    gazebo::physics::LinkPtr hipy;
    gazebo::physics::LinkPtr knee;
    gazebo::physics::LinkPtr wheel;
    ignition::math::Vector3d hipx_origin;
    ignition::math::Vector3d knee_origin;
    ignition::math::Vector3d wheel_origin;
  };

  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!node_) {
      return;
    }

    const double sim_time = info.simTime.Double();
    double dt = 0.0;
    if (last_update_sim_time_ > 0.0) {
      dt = sim_time - last_update_sim_time_;
      if (update_rate_hz_ > 0.0 && dt < 1.0 / update_rate_hz_) {
        return;
      }
    }
    last_update_sim_time_ = sim_time;
    if (dt <= 0.0 || dt > 0.25) {
      dt = 1.0 / std::max(1.0, update_rate_hz_);
    }

    const auto command = freshCommand();
    const auto pose = model_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double world_vx = cos_yaw * command.linear.x - sin_yaw * command.linear.y;
    const double world_vy = sin_yaw * command.linear.x + cos_yaw * command.linear.y;

    double next_x = pose.Pos().X() + world_vx * dt;
    double next_y = pose.Pos().Y() + world_vy * dt;
    double next_z = pose.Pos().Z();
    double vertical_velocity = 0.0;
    const double next_yaw = yaw + command.angular.z * dt;

    if (terrain_following_ && ray_shape_) {
      const double terrain = terrainHeightNear(next_x, next_y, pose.Pos().Z());
      const double target_z = terrain + body_height_;
      const double max_dz = std::max(0.0, max_vertical_speed_) * dt;
      const double dz = std::clamp(target_z - pose.Pos().Z(), -max_dz, max_dz);
      next_z = pose.Pos().Z() + dz;
      vertical_velocity = dz / std::max(1e-4, dt);
    }

    const ignition::math::Pose3d next_pose(next_x, next_y, next_z, 0.0, 0.0, next_yaw);
    model_->SetWorldPose(next_pose, true, true);
    if (base_link_) {
      base_link_->SetWorldPose(next_pose, true, true);
    }
    model_->SetLinearVel(ignition::math::Vector3d(world_vx, world_vy, vertical_velocity));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, command.angular.z));

    updateGait(info.simTime, command, dt, next_pose);
    publishOdometry(info.simTime, command, vertical_velocity);
  }

  geometry_msgs::msg::Twist freshCommand() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (!has_cmd_) {
      return geometry_msgs::msg::Twist{};
    }

    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_cmd_wall_time_).count();
    if (age > command_timeout_sec_) {
      return geometry_msgs::msg::Twist{};
    }

    geometry_msgs::msg::Twist command = last_cmd_;
    command.linear.x = std::clamp(command.linear.x, -max_linear_speed_, max_linear_speed_);
    command.linear.y = std::clamp(command.linear.y, -max_lateral_speed_, max_lateral_speed_);
    command.linear.z = 0.0;
    command.angular.x = 0.0;
    command.angular.y = 0.0;
    command.angular.z = std::clamp(command.angular.z, -max_angular_speed_, max_angular_speed_);
    return command;
  }

  double terrainHeightNear(double x, double y, double current_z) {
    const double current_terrain = current_z - body_height_;
    const ignition::math::Vector3d start(x, y, current_z + terrain_probe_start_above_);
    const ignition::math::Vector3d end(x, y, current_z - terrain_probe_end_below_);
    ray_shape_->SetPoints(start, end);

    double distance = 0.0;
    std::string entity;
    ray_shape_->GetIntersection(distance, entity);
    const double ray_length = (start - end).Length();
    if (!std::isfinite(distance) || distance < 0.0 || distance >= ray_length) {
      return current_terrain;
    }

    const double hit_z = start.Z() - distance;
    if (hit_z > current_terrain + max_terrain_step_up_) {
      return current_terrain;
    }
    if (hit_z < current_terrain - max_terrain_step_down_) {
      return current_terrain;
    }
    return hit_z;
  }

  void updateGait(
    const gazebo::common::Time & sim_time,
    const geometry_msgs::msg::Twist & command,
    double dt,
    const ignition::math::Pose3d & base_pose) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = sim_time.sec;
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

    const double horizontal_speed = std::hypot(command.linear.x, command.linear.y);
    const double ratio = animate_gait_
      ? std::clamp(
          (horizontal_speed - min_walk_speed_) /
          std::max(1e-3, max_walk_speed_ - min_walk_speed_),
          0.0, 1.0)
      : 0.0;

    updateWheelMotion(command.linear.x, dt);
    if (ratio <= 1e-3) {
      fillStance();
    } else {
      const double gait_direction = command.linear.x < -min_walk_speed_ ? -1.0 : 1.0;
      const double phase = gait_direction * 2.0 * kPi * gait_frequency_ * sim_time.Double();
      fillLeg(0, phase, ratio, true);
      fillLeg(4, phase + kPi, ratio, false);
      fillLeg(8, phase + kPi, ratio, true);
      fillLeg(12, phase, ratio, false);
    }

    joint_msg_.header.stamp = stamp;
    applyGazeboLinkPoses(base_pose);
    if (joint_pub_) {
      joint_pub_->publish(joint_msg_);
    }
  }

  void updateWheelMotion(double forward_speed, double dt) {
    const double wheel_rate = forward_speed / std::max(0.01, wheel_radius_);
    for (double & position : wheel_position_) {
      position += wheel_rate * dt;
    }
    wheel_spin_rate_ = wheel_rate;
  }

  void fillStance() {
    const std::array<double, 16> stance = {
      0.05, 0.82, -1.58, wheel_position_[0],
      -0.05, 0.82, -1.58, wheel_position_[1],
      0.05, 0.95, -1.62, wheel_position_[2],
      -0.05, 0.95, -1.62, wheel_position_[3]};
    for (size_t i = 0; i < stance.size(); ++i) {
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }
  }

  void fillLeg(size_t offset, double phase, double ratio, bool left_side) {
    const double s = std::sin(phase);
    const double c = std::cos(phase);
    const double swing = std::max(0.0, s);
    const double side = left_side ? 1.0 : -1.0;
    const bool rear = offset >= 8;
    const size_t leg_index = offset / 4;
    const double thigh_stance = rear ? 0.95 : 0.82;
    const double calf_stance = rear ? -1.62 : -1.58;

    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;
    joint_msg_.position[offset + 2] =
      calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);
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

  gazebo::physics::LinkPtr getLink(const std::string & name) const {
    auto link = model_->GetLink(name);
    if (!link) {
      gzerr << "[M20LeggedKinematicPlugin] Cannot find link: " << name << "\n";
    }
    return link;
  }

  void initializeLegChains() {
    configureLeg(
      legs_[0],
      "fl",
      ignition::math::Vector3d(0.3141, 0.0685, 0.0),
      ignition::math::Vector3d(0.0, 0.0984, -0.25),
      ignition::math::Vector3d(0.0, 0.059676, -0.25));
    configureLeg(
      legs_[1],
      "fr",
      ignition::math::Vector3d(0.3141, -0.0685, 0.0),
      ignition::math::Vector3d(0.0, -0.0984, -0.25),
      ignition::math::Vector3d(0.0, -0.059676, -0.25));
    configureLeg(
      legs_[2],
      "hl",
      ignition::math::Vector3d(-0.3141, 0.0685, 0.0),
      ignition::math::Vector3d(0.0, 0.0984, -0.25),
      ignition::math::Vector3d(0.0, 0.059676, -0.25));
    configureLeg(
      legs_[3],
      "hr",
      ignition::math::Vector3d(-0.3141, -0.0685, 0.0),
      ignition::math::Vector3d(0.0, -0.0984, -0.25),
      ignition::math::Vector3d(0.0, -0.059676, -0.25));
  }

  void configureLeg(
    LegChain & leg,
    const std::string & prefix,
    const ignition::math::Vector3d & hipx_origin,
    const ignition::math::Vector3d & knee_origin,
    const ignition::math::Vector3d & wheel_origin) {
    leg.hipx = getLink(prefix + "_hipx");
    leg.hipy = getLink(prefix + "_hipy");
    leg.knee = getLink(prefix + "_knee");
    leg.wheel = getLink(prefix + "_wheel");
    leg.hipx_origin = hipx_origin;
    leg.knee_origin = knee_origin;
    leg.wheel_origin = wheel_origin;
  }

  ignition::math::Pose3d jointPose(
    const ignition::math::Pose3d & parent_pose,
    const ignition::math::Vector3d & origin,
    const ignition::math::Vector3d & axis,
    double position) const {
    return parent_pose *
      ignition::math::Pose3d(origin, ignition::math::Quaterniond(axis, position));
  }

  void applyGazeboLinkPoses(const ignition::math::Pose3d & base_pose) const {
    static const ignition::math::Vector3d hipx_axis(-1.0, 0.0, 0.0);
    static const ignition::math::Vector3d pitch_axis(0.0, -1.0, 0.0);

    if (base_link_) {
      base_link_->SetWorldPose(base_pose, true, true);
    }

    for (size_t leg_index = 0; leg_index < legs_.size(); ++leg_index) {
      const auto & leg = legs_[leg_index];
      const size_t offset = leg_index * 4;
      const auto hipx_pose =
        jointPose(base_pose, leg.hipx_origin, hipx_axis, joint_msg_.position[offset]);
      const auto hipy_pose =
        jointPose(hipx_pose, ignition::math::Vector3d::Zero, pitch_axis, joint_msg_.position[offset + 1]);
      const auto knee_pose =
        jointPose(hipy_pose, leg.knee_origin, pitch_axis, joint_msg_.position[offset + 2]);
      const auto wheel_pose =
        jointPose(knee_pose, leg.wheel_origin, pitch_axis, joint_msg_.position[offset + 3]);

      if (leg.hipx) {
        leg.hipx->SetWorldPose(hipx_pose, true, true);
      }
      if (leg.hipy) {
        leg.hipy->SetWorldPose(hipy_pose, true, true);
      }
      if (leg.knee) {
        leg.knee->SetWorldPose(knee_pose, true, true);
      }
      if (leg.wheel) {
        leg.wheel->SetWorldPose(wheel_pose, true, true);
      }
    }
  }

  void publishOdometry(
    const gazebo::common::Time & sim_time,
    const geometry_msgs::msg::Twist & command,
    double vertical_velocity) {
    if (!publish_odom_ && !publish_tf_) {
      return;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = sim_time.sec;
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);
    const auto pose = model_->WorldPose();

    if (odom_pub_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_id_;
      odom.child_frame_id = base_frame_id_;
      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();
      odom.twist.twist.linear.x = command.linear.x;
      odom.twist.twist.linear.y = command.linear.y;
      odom.twist.twist.linear.z = vertical_velocity;
      odom.twist.twist.angular.z = command.angular.z;
      odom_pub_->publish(odom);
    }

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_id_;
      transform.child_frame_id = base_frame_id_;
      transform.transform.translation.x = pose.Pos().X();
      transform.transform.translation.y = pose.Pos().Y();
      transform.transform.translation.z = pose.Pos().Z();
      transform.transform.rotation.x = pose.Rot().X();
      transform.transform.rotation.y = pose.Rot().Y();
      transform.transform.rotation.z = pose.Rot().Z();
      transform.transform.rotation.w = pose.Rot().W();
      tf_broadcaster_->sendTransform(transform);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::physics::RayShapePtr ray_shape_;
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;
  gazebo::physics::LinkPtr base_link_;
  std::array<LegChain, 4> legs_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  mutable std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_wall_time_{std::chrono::steady_clock::now()};
  bool has_cmd_{false};

  sensor_msgs::msg::JointState joint_msg_;
  std::array<double, 4> wheel_position_{0.0, 0.0, 0.0, 0.0};
  double wheel_spin_rate_{0.0};

  std::string cmd_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  std::string joint_states_topic_{"/joint_states"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  double max_linear_speed_{0.45};
  double max_lateral_speed_{0.35};
  double max_angular_speed_{1.05};
  double command_timeout_sec_{0.5};
  double update_rate_hz_{100.0};
  double body_height_{0.59};
  double terrain_probe_start_above_{1.15};
  double terrain_probe_end_below_{1.20};
  double max_terrain_step_up_{0.35};
  double max_terrain_step_down_{0.75};
  double max_vertical_speed_{0.80};
  double gait_frequency_{2.2};
  double min_walk_speed_{0.05};
  double max_walk_speed_{1.0};
  double hip_swing_{0.08};
  double thigh_swing_{0.32};
  double calf_swing_{0.42};
  double wheel_radius_{0.09};
  double last_update_sim_time_{0.0};
  bool terrain_following_{true};
  bool disable_robot_collisions_{true};
  bool publish_odom_{true};
  bool publish_tf_{false};
  bool publish_joint_states_{true};
  bool animate_gait_{true};
};

GZ_REGISTER_MODEL_PLUGIN(M20LeggedKinematicPlugin)

}  // namespace m20_industrial_inspection_gazebo
