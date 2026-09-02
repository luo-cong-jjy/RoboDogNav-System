/**
 * @file waypoint_obstacle_plugin.cpp
 * @brief 路点导航动态障碍物 Gazebo 模型插件（WaypointObstaclePlugin）
 *
 * 职责：
 *  - 让障碍物模型沿 SDF 中配置的路点轨迹运动，模拟沿固定路线行走的动态障碍物；
 *  - 轨迹由字符串 "t x y z roll pitch yaw; t x y z ..." 解析而来，支持循环（loop）与非循环两种模式；
 *  - 相邻路点之间按时刻做位置与姿态的线性插值（姿态角先归一化再做最短路径插值）；
 *  - 每帧直接设置模型世界位姿（SetWorldPose）并清零速度，属运动学驱动；
 *  - 通过 GZ_REGISTER_MODEL_PLUGIN 注册，用于在 Gazebo 世界中加载沿路点运动的障碍物。
 */
#include <algorithm>    // 标准算法库：std::max / std::clamp / std::sort
#include <cmath>        // 数学库：std::fmod（循环取模）、M_PI（角度归一化）
#include <sstream>      // 字符串流：std::stringstream（解析路点文本）
#include <string>       // 字符串：std::string
#include <vector>       // 容器：std::vector（路点列表）

#include "gazebo/common/Events.hh"    // Gazebo 事件：世界更新事件
#include "gazebo/gazebo.hh"           // Gazebo 主头文件：注册宏等
#include "gazebo/physics/physics.hh"  // Gazebo 物理：模型位姿控制接口

namespace m20_nav2_system {
namespace {

// 带时间戳的路点：记录到达该路点的时刻与对应位姿
struct TimedPose {
  double time{0.0};             // 到达该路点的仿真时刻 [s]
  ignition::math::Pose3d pose;  // 该路点的位姿
};

// 从 SDF 安全读取参数：SDF 为空或元素缺失时返回默认值
template <typename T>
T sdfValue(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback) {
  if (!sdf || !sdf->HasElement(name)) {   // 无 SDF 或参数缺失
    return fallback;                      // 返回默认值
  }
  return sdf->Get<T>(name);               // 解析参数并返回
}

// 将角度归一化到 [-PI, PI]，用于姿态插值时选择最短旋转方向
double normalizeAngle(double angle) {
  while (angle > M_PI) {        // 超过上界
    angle -= 2.0 * M_PI;        // 减去一整圈
  }
  while (angle < -M_PI) {       // 低于下界
    angle += 2.0 * M_PI;        // 加上一整圈
  }
  return angle;                 // 返回归一化后的角度
}

// 解析轨迹字符串："t x y z roll pitch yaw; t x y z ..."，返回按时间升序排序的路点列表
std::vector<TimedPose> parseTrajectory(const std::string & text) {
  std::vector<TimedPose> trajectory;    // 路点结果容器
  std::stringstream items(text);        // 按分号分割各路点
  std::string item;
  while (std::getline(items, item, ';')) {   // 逐个读取分号分隔的路点字符串
    std::stringstream values(item);     // 用空格继续拆出 7 个数值
    double time = 0.0;                  // 时刻
    double x = 0.0;                     // 位置 x
    double y = 0.0;                     // 位置 y
    double z = 0.0;                     // 位置 z
    double roll = 0.0;                  // 横滚角
    double pitch = 0.0;                 // 俯仰角
    double yaw = 0.0;                   // 偏航角
    if (values >> time >> x >> y >> z >> roll >> pitch >> yaw) {   // 成功解析出全部 7 个数值
      trajectory.push_back({time, ignition::math::Pose3d(x, y, z, roll, pitch, yaw)});   // 存入路点列表
    }
  }

  std::sort(trajectory.begin(), trajectory.end(), [](const TimedPose & lhs, const TimedPose & rhs) {   // 按时间升序排序
    return lhs.time < rhs.time;
  });
  return trajectory;    // 返回排序后的路点列表
}

// 在相邻两个路点之间按时刻 time 做线性插值，得到中间位姿
ignition::math::Pose3d interpolatePose(const TimedPose & from, const TimedPose & to, double time) {
  const double duration = std::max(to.time - from.time, 1.0e-6);   // 两路点的时间间隔（下限保护防除零）
  const double ratio = std::clamp((time - from.time) / duration, 0.0, 1.0);   // 插值比例，限制在 [0,1]
  const auto from_pos = from.pose.Pos();    // 起点位置
  const auto to_pos = to.pose.Pos();        // 终点位置
  const auto from_rot = from.pose.Rot().Euler();   // 起点欧拉角
  const auto to_rot = to.pose.Rot().Euler();       // 终点欧拉角

  const double x = from_pos.X() + (to_pos.X() - from_pos.X()) * ratio;   // 位置 x 线性插值
  const double y = from_pos.Y() + (to_pos.Y() - from_pos.Y()) * ratio;   // 位置 y 线性插值
  const double z = from_pos.Z() + (to_pos.Z() - from_pos.Z()) * ratio;   // 位置 z 线性插值
  const double roll = from_rot.X() + normalizeAngle(to_rot.X() - from_rot.X()) * ratio;   // 横滚角插值（最短路径）
  const double pitch = from_rot.Y() + normalizeAngle(to_rot.Y() - from_rot.Y()) * ratio;  // 俯仰角插值（最短路径）
  const double yaw = from_rot.Z() + normalizeAngle(to_rot.Z() - from_rot.Z()) * ratio;    // 偏航角插值（最短路径）

  return ignition::math::Pose3d(x, y, z, roll, pitch, yaw);   // 返回插值位姿
}

}  // namespace

// 路点导航动态障碍物插件：继承 ModelPlugin 绑定到障碍物模型
class WaypointObstaclePlugin : public gazebo::ModelPlugin {
public:
  // 插件加载入口：解析路点轨迹并挂接世界更新回调
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;                       // 保存绑定的模型指针
    loop_ = sdfValue<bool>(sdf, "loop", true);        // 是否循环运动
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate", 30.0);   // 更新频率 [Hz]
    trajectory_ = parseTrajectory(sdfValue<std::string>(sdf, "trajectory", ""));   // 解析路点轨迹

    if (trajectory_.empty()) {            // 未配置任何路点
      trajectory_.push_back({0.0, model_->WorldPose()});   // 使用当前位姿作为唯一路点（模型保持静止）
    }

    model_->SetGravityMode(false);        // 关闭重力：轨迹由位姿直接驱动，不受动力学影响
    model_->SetWorldPose(trajectory_.front().pose);   // 初始放置到第一个路点
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(   // 挂接世界更新回调
      std::bind(&WaypointObstaclePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[m20_waypoint_obstacle] " << model_->GetName()   // 打印加载信息
          << " waypoints=" << trajectory_.size()
          << " loop=" << loop_ << "\n";
  }

private:
  // 世界更新回调：按当前时刻查询插值位姿并设置模型
  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!model_ || trajectory_.empty()) {   // 模型无效或没有路点
      return;
    }

    const double now = info.simTime.Double();   // 当前仿真时间 [s]
    if (update_rate_hz_ > 0.0 && (now - last_update_time_) < (1.0 / update_rate_hz_)) {   // 限频：距上次更新不足一个周期
      return;                                   // 跳过本次更新
    }
    last_update_time_ = now;                    // 记录本次更新时间

    const auto pose = poseAtTime(now);          // 查询当前时刻的插值位姿
    model_->SetWorldPose(pose);                 // 设置模型世界位姿
    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));   // 清零线速度
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));  // 清零角速度
  }

  // 查询时刻 now 对应的位姿：循环模式对总时长取模折返，非循环模式到终点后保持静止
  ignition::math::Pose3d poseAtTime(double now) const {
    if (trajectory_.size() == 1) {        // 只有一个路点
      return trajectory_.front().pose;    // 始终返回该路点位姿
    }

    const double end_time = std::max(trajectory_.back().time, 1.0e-6);   // 轨迹总时长（下限保护）
    double time = now;
    if (loop_) {                          // 循环模式
      time = std::fmod(now, end_time);    // 对总时长取模，实现循环播放
      if (time < 0.0) {                   // 负数取模结果修正
        time += end_time;
      }
    } else if (time >= end_time) {        // 非循环模式且已到终点时刻
      return trajectory_.back().pose;     // 停在最后一个路点
    }

    for (std::size_t i = 1; i < trajectory_.size(); ++i) {   // 查找 now 所在的路点区间
      if (time <= trajectory_[i].time) {  // 落在区间 (i-1, i] 内
        return interpolatePose(trajectory_[i - 1], trajectory_[i], time);   // 区间内线性插值
      }
    }
    return trajectory_.back().pose;       // 兜底：返回最后一个路点
  }

  gazebo::physics::ModelPtr model_;                 // 绑定的模型指针
  gazebo::event::ConnectionPtr update_connection_;  // 世界更新事件连接
  std::vector<TimedPose> trajectory_;               // 路点轨迹列表
  bool loop_{true};                                 // 是否循环运动
  double update_rate_hz_{30.0};                     // 更新频率 [Hz]
  double last_update_time_{0.0};                    // 上次更新时的仿真时间
};

GZ_REGISTER_MODEL_PLUGIN(WaypointObstaclePlugin)   // 注册为 Gazebo 模型插件

}  // namespace m20_nav2_system
