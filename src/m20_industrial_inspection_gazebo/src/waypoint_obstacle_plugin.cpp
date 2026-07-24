#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace m20_industrial_inspection_gazebo {
namespace {

struct TimedPose {
  double time{0.0};
  ignition::math::Pose3d pose;
};

template <typename T>
T sdfValue(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback) {
  if (!sdf || !sdf->HasElement(name)) {
    return fallback;
  }
  return sdf->Get<T>(name);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::vector<TimedPose> parseTrajectory(const std::string & text) {
  std::vector<TimedPose> trajectory;
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ';')) {
    std::stringstream values(item);
    double time = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (values >> time >> x >> y >> z >> roll >> pitch >> yaw) {
      trajectory.push_back({time, ignition::math::Pose3d(x, y, z, roll, pitch, yaw)});
    }
  }

  std::sort(trajectory.begin(), trajectory.end(), [](const TimedPose & lhs, const TimedPose & rhs) {
    return lhs.time < rhs.time;
  });
  return trajectory;
}

ignition::math::Pose3d interpolatePose(const TimedPose & from, const TimedPose & to, double time) {
  const double duration = std::max(to.time - from.time, 1.0e-6);
  const double ratio = std::clamp((time - from.time) / duration, 0.0, 1.0);
  const auto from_pos = from.pose.Pos();
  const auto to_pos = to.pose.Pos();
  const auto from_rot = from.pose.Rot().Euler();
  const auto to_rot = to.pose.Rot().Euler();

  const double x = from_pos.X() + (to_pos.X() - from_pos.X()) * ratio;
  const double y = from_pos.Y() + (to_pos.Y() - from_pos.Y()) * ratio;
  const double z = from_pos.Z() + (to_pos.Z() - from_pos.Z()) * ratio;
  const double roll = from_rot.X() + normalizeAngle(to_rot.X() - from_rot.X()) * ratio;
  const double pitch = from_rot.Y() + normalizeAngle(to_rot.Y() - from_rot.Y()) * ratio;
  const double yaw = from_rot.Z() + normalizeAngle(to_rot.Z() - from_rot.Z()) * ratio;

  return ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
}

}  // namespace

class WaypointObstaclePlugin : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    loop_ = sdfValue<bool>(sdf, "loop", true);
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate", 30.0);
    trajectory_ = parseTrajectory(sdfValue<std::string>(sdf, "trajectory", ""));

    if (trajectory_.empty()) {
      trajectory_.push_back({0.0, model_->WorldPose()});
    }

    model_->SetGravityMode(false);
    model_->SetWorldPose(trajectory_.front().pose);
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&WaypointObstaclePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[m20_waypoint_obstacle] " << model_->GetName()
          << " waypoints=" << trajectory_.size()
          << " loop=" << loop_ << "\n";
  }

private:
  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!model_ || trajectory_.empty()) {
      return;
    }

    const double now = info.simTime.Double();
    if (update_rate_hz_ > 0.0 && (now - last_update_time_) < (1.0 / update_rate_hz_)) {
      return;
    }
    last_update_time_ = now;

    const auto pose = poseAtTime(now);
    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
  }

  ignition::math::Pose3d poseAtTime(double now) const {
    if (trajectory_.size() == 1) {
      return trajectory_.front().pose;
    }

    const double end_time = std::max(trajectory_.back().time, 1.0e-6);
    double time = now;
    if (loop_) {
      time = std::fmod(now, end_time);
      if (time < 0.0) {
        time += end_time;
      }
    } else if (time >= end_time) {
      return trajectory_.back().pose;
    }

    for (std::size_t i = 1; i < trajectory_.size(); ++i) {
      if (time <= trajectory_[i].time) {
        return interpolatePose(trajectory_[i - 1], trajectory_[i], time);
      }
    }
    return trajectory_.back().pose;
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  std::vector<TimedPose> trajectory_;
  bool loop_{true};
  double update_rate_hz_{30.0};
  double last_update_time_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(WaypointObstaclePlugin)

}  // namespace m20_industrial_inspection_gazebo
