/**
 * @file sinusoidal_obstacle_plugin.cpp
 * @brief 正弦轨迹动态障碍物 Gazebo 模型插件（SinusoidalObstaclePlugin）
 *
 * 职责：
 *  - 让障碍物模型沿指定轴（x / y / z）按正弦规律往复运动，模拟动态障碍物；
 *  - 运动参数（轴、振幅、周期、相位、更新频率）全部由 SDF <plugin> 参数配置；
 *  - 每个仿真步直接把模型世界位姿设置为正弦偏移后的位姿（SetWorldPose），
 *    并清零线速度/角速度，属于运动学（kinematic）式驱动，不走物理动力学；
 *  - 通过 GZ_REGISTER_MODEL_PLUGIN 注册，用于在 Gazebo 世界中加载动态障碍物。
 */
#include <algorithm>    // 标准算法库：std::max（周期下限保护）
#include <cmath>        // 数学库：std::sin（正弦计算）、M_PI
#include <functional>   // 函数对象：std::bind / std::placeholders（绑定世界更新回调）
#include <string>       // 字符串：std::string（运动轴名）

#include <gazebo/common/Plugin.hh>    // Gazebo 插件基类声明（ModelPlugin）
#include <gazebo/common/Events.hh>    // Gazebo 事件：世界更新事件
#include <gazebo/gazebo.hh>           // Gazebo 主头文件：注册宏等
#include <gazebo/physics/physics.hh>  // Gazebo 物理：模型位姿/速度控制接口

namespace m20_nav2_system
{
// 正弦轨迹动态障碍物插件：继承 ModelPlugin 绑定到障碍物模型
class SinusoidalObstaclePlugin : public gazebo::ModelPlugin
{
public:
  // 插件加载入口：由 Gazebo 调用，解析 SDF 参数并挂接世界更新回调
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;                       // 保存绑定的模型指针
    initial_pose_ = model_->WorldPose();  // 记录初始位姿（正弦偏移的基准位置）

    if (sdf->HasElement("axis")) {        // 配置了运动轴
      axis_ = sdf->Get<std::string>("axis");   // 运动轴："x" / "y" / "z"
    }
    if (sdf->HasElement("amplitude")) {   // 配置了振幅
      amplitude_ = sdf->Get<double>("amplitude");   // 正弦振幅 [m]
    }
    if (sdf->HasElement("period")) {      // 配置了运动周期
      period_ = sdf->Get<double>("period");         // 正弦周期 [s]
    }
    if (sdf->HasElement("phase")) {       // 配置了初始相位
      phase_ = sdf->Get<double>("phase");           // 初始相位 [rad]
    }
    if (sdf->HasElement("update_rate")) { // 配置了更新频率
      update_rate_ = sdf->Get<double>("update_rate");   // 位姿更新频率 [Hz]
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(   // 挂接世界更新回调
      std::bind(&SinusoidalObstaclePlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[m20_sinusoidal_obstacle] " << model_->GetName()   // 打印加载信息
          << " axis=" << axis_
          << " amplitude=" << amplitude_
          << " period=" << period_ << "\n";
  }

private:
  // 世界更新回调：按正弦规律计算偏移量并设置模型位姿
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();   // 当前仿真时间 [s]
    if (update_rate_ > 0.0 && (now - last_update_time_) < (1.0 / update_rate_)) {   // 限频：距上次更新不足一个周期
      return;                                   // 跳过本次更新
    }
    last_update_time_ = now;                    // 记录本次更新时间

    const double safe_period = std::max(period_, 0.001);   // 周期下限保护，避免除零
    const double offset = amplitude_ * std::sin(2.0 * M_PI * now / safe_period + phase_);   // 计算正弦偏移量 [m]

    ignition::math::Pose3d pose = initial_pose_;  // 以初始位姿为基准
    if (axis_ == "x") {                           // 沿 x 轴运动
      pose.Pos().X(initial_pose_.Pos().X() + offset);   // x 叠加正弦偏移
    } else if (axis_ == "z") {                    // 沿 z 轴运动
      pose.Pos().Z(initial_pose_.Pos().Z() + offset);   // z 叠加正弦偏移
    } else {                                      // 默认沿 y 轴运动
      pose.Pos().Y(initial_pose_.Pos().Y() + offset);   // y 叠加正弦偏移
    }

    model_->SetWorldPose(pose);                   // 直接设置模型世界位姿（运动学驱动）
    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));   // 清零线速度
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));  // 清零角速度
  }

  gazebo::physics::ModelPtr model_;       // 绑定的模型指针
  gazebo::event::ConnectionPtr update_connection_;   // 世界更新事件连接
  ignition::math::Pose3d initial_pose_;   // 初始位姿（正弦运动的基准）
  std::string axis_{"y"};                 // 运动轴（默认 y）
  double amplitude_{1.0};                 // 正弦振幅 [m]
  double period_{12.0};                   // 运动周期 [s]
  double phase_{0.0};                     // 初始相位 [rad]
  double update_rate_{30.0};              // 更新频率 [Hz]
  double last_update_time_{0.0};          // 上次更新时的仿真时间
};

GZ_REGISTER_MODEL_PLUGIN(SinusoidalObstaclePlugin)   // 注册为 Gazebo 模型插件
}  // namespace m20_nav2_system
