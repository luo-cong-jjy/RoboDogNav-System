#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace m20_nav2_gazebo_sandbox
{
class SinusoidalObstaclePlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    initial_pose_ = model_->WorldPose();

    if (sdf->HasElement("axis")) {
      axis_ = sdf->Get<std::string>("axis");
    }
    if (sdf->HasElement("amplitude")) {
      amplitude_ = sdf->Get<double>("amplitude");
    }
    if (sdf->HasElement("period")) {
      period_ = sdf->Get<double>("period");
    }
    if (sdf->HasElement("phase")) {
      phase_ = sdf->Get<double>("phase");
    }
    if (sdf->HasElement("update_rate")) {
      update_rate_ = sdf->Get<double>("update_rate");
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SinusoidalObstaclePlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[m20_sinusoidal_obstacle] " << model_->GetName()
          << " axis=" << axis_
          << " amplitude=" << amplitude_
          << " period=" << period_ << "\n";
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (update_rate_ > 0.0 && (now - last_update_time_) < (1.0 / update_rate_)) {
      return;
    }
    last_update_time_ = now;

    const double safe_period = std::max(period_, 0.001);
    const double offset = amplitude_ * std::sin(2.0 * M_PI * now / safe_period + phase_);

    ignition::math::Pose3d pose = initial_pose_;
    if (axis_ == "x") {
      pose.Pos().X(initial_pose_.Pos().X() + offset);
    } else if (axis_ == "z") {
      pose.Pos().Z(initial_pose_.Pos().Z() + offset);
    } else {
      pose.Pos().Y(initial_pose_.Pos().Y() + offset);
    }

    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  ignition::math::Pose3d initial_pose_;
  std::string axis_{"y"};
  double amplitude_{1.0};
  double period_{12.0};
  double phase_{0.0};
  double update_rate_{30.0};
  double last_update_time_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(SinusoidalObstaclePlugin)
}  // namespace m20_nav2_gazebo_sandbox
