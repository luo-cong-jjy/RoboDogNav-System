#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace m20_nav2_gazebo_sandbox {
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

class M20FourWheelDrivePlugin : public gazebo::ModelPlugin {
public:
  M20FourWheelDrivePlugin() = default;

  ~M20FourWheelDrivePlugin() override {
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

    wheel_radius_ = sdfValue<double>(sdf, "wheel_radius", 0.09);
    wheel_separation_ = sdfValue<double>(sdf, "wheel_separation", 0.453);
    wheel_velocity_sign_ = sdfValue<double>(sdf, "wheel_velocity_sign", -1.0);
    max_wheel_torque_ = sdfValue<double>(sdf, "max_wheel_torque", 28.0);
    max_linear_speed_ = sdfValue<double>(sdf, "max_linear_speed", 0.45);
    max_angular_speed_ = sdfValue<double>(sdf, "max_angular_speed", 0.65);
    command_timeout_sec_ = sdfValue<double>(sdf, "command_timeout_sec", 0.5);
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate_hz", 100.0);
    use_planar_velocity_ = sdfValue<bool>(sdf, "use_planar_velocity", false);
    publish_odom_ = sdfValue<bool>(sdf, "publish_odom", true);
    publish_tf_ = sdfValue<bool>(sdf, "publish_tf", true);
    cmd_topic_ = sdfValue<std::string>(sdf, "cmd_topic", "/cmd_vel");
    odom_topic_ = sdfValue<std::string>(sdf, "odom_topic", "/odom");
    odom_frame_id_ = sdfValue<std::string>(sdf, "odom_frame_id", "odom");
    base_frame_id_ = sdfValue<std::string>(sdf, "base_frame_id", "base_link");

    front_left_joint_ = getJoint(sdfValue<std::string>(sdf, "front_left_joint", "fl_wheel_joint"));
    rear_left_joint_ = getJoint(sdfValue<std::string>(sdf, "rear_left_joint", "hl_wheel_joint"));
    front_right_joint_ = getJoint(sdfValue<std::string>(sdf, "front_right_joint", "fr_wheel_joint"));
    rear_right_joint_ = getJoint(sdfValue<std::string>(sdf, "rear_right_joint", "hr_wheel_joint"));

    if (!front_left_joint_ || !rear_left_joint_ || !front_right_joint_ || !rear_right_joint_) {
      gzerr << "[M20FourWheelDrivePlugin] Missing one or more wheel joints. Plugin disabled.\n";
      return;
    }

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    const std::string node_name = "m20_four_wheel_drive_" + sanitizeName(model_->GetName());
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
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&M20FourWheelDrivePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[M20FourWheelDrivePlugin] Loaded for " << model_->GetName()
          << ", cmd_topic=" << cmd_topic_
          << ", odom_topic=" << odom_topic_
          << ", wheel_radius=" << wheel_radius_
          << ", wheel_separation=" << wheel_separation_
          << ", wheel_velocity_sign=" << wheel_velocity_sign_ << "\n";
  }

private:
  gazebo::physics::JointPtr getJoint(const std::string & name) const {
    auto joint = model_->GetJoint(name);
    if (!joint) {
      gzerr << "[M20FourWheelDrivePlugin] Cannot find joint: " << name << "\n";
    }
    return joint;
  }

  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!node_) {
      return;
    }

    const double sim_time = info.simTime.Double();
    if (last_update_sim_time_ > 0.0 && update_rate_hz_ > 0.0) {
      const double min_period = 1.0 / update_rate_hz_;
      if (sim_time - last_update_sim_time_ < min_period) {
        return;
      }
    }
    last_update_sim_time_ = sim_time;

    const auto command = freshCommand();
    applyWheelVelocity(command.linear.x, command.angular.z);
    if (use_planar_velocity_) {
      applyPlanarVelocity(command.linear.x, command.angular.z);
    }
    publishOdometry(info.simTime);
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
    command.angular.z = std::clamp(command.angular.z, -max_angular_speed_, max_angular_speed_);
    return command;
  }

  void applyWheelVelocity(double linear_x, double angular_z) {
    const double left_velocity = wheel_velocity_sign_ *
      (linear_x - angular_z * wheel_separation_ * 0.5) / wheel_radius_;
    const double right_velocity = wheel_velocity_sign_ *
      (linear_x + angular_z * wheel_separation_ * 0.5) / wheel_radius_;

    setJointVelocity(front_left_joint_, left_velocity);
    setJointVelocity(rear_left_joint_, left_velocity);
    setJointVelocity(front_right_joint_, right_velocity);
    setJointVelocity(rear_right_joint_, right_velocity);
  }

  void setJointVelocity(const gazebo::physics::JointPtr & joint, double velocity) const {
    joint->SetParam("fmax", 0, max_wheel_torque_);
    joint->SetParam("vel", 0, velocity);
  }

  void applyPlanarVelocity(double linear_x, double angular_z) const {
    const auto pose = model_->WorldPose();
    const auto world_linear = pose.Rot().RotateVector(
      ignition::math::Vector3d(linear_x, 0.0, 0.0));
    const auto current_linear = model_->WorldLinearVel();
    model_->SetLinearVel(
      ignition::math::Vector3d(world_linear.X(), world_linear.Y(), current_linear.Z()));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, angular_z));
  }

  void publishOdometry(const gazebo::common::Time & sim_time) {
    if (!publish_odom_ && !publish_tf_) {
      return;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = sim_time.sec;
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);
    const auto pose = model_->WorldPose();
    const auto linear = model_->WorldLinearVel();
    const auto angular = model_->WorldAngularVel();
    const auto body_linear = pose.Rot().RotateVectorReverse(linear);
    const auto body_angular = pose.Rot().RotateVectorReverse(angular);

    if (publish_odom_) {
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
      odom.twist.twist.linear.x = body_linear.X();
      odom.twist.twist.linear.y = body_linear.Y();
      odom.twist.twist.linear.z = body_linear.Z();
      odom.twist.twist.angular.x = body_angular.X();
      odom.twist.twist.angular.y = body_angular.Y();
      odom.twist.twist.angular.z = body_angular.Z();
      odom_pub_->publish(odom);
    }

    if (publish_tf_) {
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
  gazebo::physics::JointPtr front_left_joint_;
  gazebo::physics::JointPtr rear_left_joint_;
  gazebo::physics::JointPtr front_right_joint_;
  gazebo::physics::JointPtr rear_right_joint_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  mutable std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_wall_time_{std::chrono::steady_clock::now()};
  bool has_cmd_{false};

  std::string cmd_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  double wheel_radius_{0.09};
  double wheel_separation_{0.453};
  double wheel_velocity_sign_{-1.0};
  double max_wheel_torque_{28.0};
  double max_linear_speed_{0.45};
  double max_angular_speed_{0.65};
  double command_timeout_sec_{0.5};
  double update_rate_hz_{100.0};
  double last_update_sim_time_{0.0};
  bool use_planar_velocity_{false};
  bool publish_odom_{true};
  bool publish_tf_{true};
};

GZ_REGISTER_MODEL_PLUGIN(M20FourWheelDrivePlugin)

}  // namespace m20_nav2_gazebo_sandbox
