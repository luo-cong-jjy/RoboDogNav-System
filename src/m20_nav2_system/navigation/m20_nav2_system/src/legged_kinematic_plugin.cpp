/**
 * @file legged_kinematic_plugin.cpp
 * @brief 四足腿部运动学 Gazebo 模型插件（M20LeggedKinematicPlugin）
 *
 * 职责：
 *  - 订阅速度指令话题（默认 /cmd_vel），接收线速度/角速度指令；
 *  - 以运动学（kinematic）方式直接驱动模型：按指令积分出下一时刻机体位姿，
 *    通过 SetWorldPose 移动机体，并同步设置各腿部连杆（hipx/hipy/knee/wheel）的世界位姿；
 *  - 可选地形跟随（terrain_following）：用射线（RayShape）向下探测地面高度，
 *    让机体高度贴合地形（带阶梯高度限幅与竖直速度限幅）；
 *  - 可选步态动画（animate_gait）：按前进速度插值出站立/行走两种姿态，
 *    腿关节（髋/膝）按正弦相位摆动模拟行走，轮子按前进速度自转；
 *  - 发布里程计（/odom）、TF（odom -> base_link）与关节状态（/joint_states）；
 *  - 指令带超时保护与速度限幅；所有运动均为运动学驱动，不依赖物理动力学。
 *
 * 通过 GZ_REGISTER_MODEL_PLUGIN 注册为 Gazebo 模型插件。
 */
#include <algorithm>    // 标准算法库：std::clamp / std::max / std::hypot
#include <array>        // 容器：std::array（固定大小数组，如 4 腿/16 关节）
#include <chrono>       // 时间库：std::chrono（指令超时计时）
#include <cmath>        // 数学库：std::cos / std::sin / std::hypot
#include <cctype>       // 字符分类函数：std::isalnum（模型名清理）
#include <functional>   // 函数对象：std::bind / std::placeholders（绑定世界更新回调）
#include <memory>       // 智能指针：std::shared_ptr / std::make_shared
#include <mutex>        // 互斥锁：std::mutex / std::lock_guard（保护指令共享数据）
#include <string>       // 字符串：std::string（话题名、坐标系名等）
#include <thread>       // 线程：std::thread（ROS2 执行器后台 spin 线程）
#include <vector>       // 容器：std::vector（关节指针列表）

#include <boost/pointer_cast.hpp>   // Boost 智能指针转换：dynamic_pointer_cast（射线形状转换）

#include "builtin_interfaces/msg/time.hpp"   // ROS2 时间消息：消息头时间戳
#include "gazebo/common/Events.hh"           // Gazebo 事件：世界更新事件（WorldUpdateBegin）
#include "gazebo/gazebo.hh"                  // Gazebo 主头文件：插件注册宏等
#include "gazebo/physics/RayShape.hh"        // 射线形状：RayShape（地形高度探测）
#include "gazebo/physics/physics.hh"         // Gazebo 物理：Model/Link/World 等物理实体
#include "geometry_msgs/msg/transform_stamped.hpp"   // TF 消息：TransformStamped（发布 TF 用）
#include "geometry_msgs/msg/twist.hpp"       // 速度消息：Twist（订阅 /cmd_vel 指令）
#include "nav_msgs/msg/odometry.hpp"         // 里程计消息：Odometry（发布 /odom）
#include "rclcpp/rclcpp.hpp"                 // ROS2 C++ 客户端库：Node/Publisher/Subscription
#include "sensor_msgs/msg/joint_state.hpp"   // 关节状态消息：JointState（发布 /joint_states）
#include "tf2_ros/transform_broadcaster.h"   // TF2 广播器：向 TF 树广播 odom->base_link

namespace m20_nav2_system {   // 项目命名空间：m20_nav2_system 下所有插件/节点共享
namespace {                    // 匿名命名空间：仅本翻译单元可见的辅助工具

// 从 SDF 参数中安全读取指定类型的值；SDF 为空或元素缺失时返回默认值 fallback
template <typename T>
T sdfValue(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback) {
  if (!sdf || !sdf->HasElement(name)) {   // SDF 为空或不存在该参数元素
    return fallback;                      // 返回调用者给定的默认值
  }
  return sdf->Get<T>(name);               // 解析 SDF 元素为类型 T 并返回
}

// 将模型名中的非字母数字字符替换为 '_'，用于生成合法的 ROS2 节点名
std::string sanitizeName(std::string name) {
  for (char & c : name) {                                           // 遍历名字中的每个字符
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') { // 既不是字母数字也不是下划线
      c = '_';                                                      // 替换为下划线
    }
  }
  return name;                                                      // 返回清理后的名字
}

}  // namespace

// 四足腿部运动学插件：继承 ModelPlugin 绑定到机器人模型，实现 cmd_vel -> 机体/腿部位姿 -> odom/joint_states 全链路
class M20LeggedKinematicPlugin : public gazebo::ModelPlugin {
public:
  M20LeggedKinematicPlugin() = default;   // 默认构造函数

  // 析构函数：退出前取消执行器并等待后台 spin 线程结束，避免悬挂线程
  ~M20LeggedKinematicPlugin() override {
    if (executor_) {              // 执行器存在
      executor_->cancel();        // 取消执行器，停止处理 ROS2 回调
    }
    if (spin_thread_.joinable()) {  // spin 线程可 join（仍在运行）
      spin_thread_.join();          // 等待线程结束
    }
  }

  // 插件加载入口：由 Gazebo 在模型创建时调用，完成参数解析、物理配置、ROS2 初始化与回调挂接
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;                    // 保存当前插件绑定的模型指针
    world_ = model_->GetWorld();       // 获取模型所在的世界指针

    // ---- 从 SDF 读取可配置参数（未配置时使用括号内的默认值） ----
    cmd_topic_ = sdfValue<std::string>(sdf, "cmd_topic", "/cmd_vel");         // 速度指令话题名
    odom_topic_ = sdfValue<std::string>(sdf, "odom_topic", "/odom");          // 里程计话题名
    joint_states_topic_ = sdfValue<std::string>(sdf, "joint_states_topic", "/joint_states");   // 关节状态话题名
    odom_frame_id_ = sdfValue<std::string>(sdf, "odom_frame_id", "odom");     // 里程计父坐标系
    base_frame_id_ = sdfValue<std::string>(sdf, "base_frame_id", "base_link");  // 机器人基座坐标系
    max_linear_speed_ = sdfValue<double>(sdf, "max_linear_speed", 0.45);      // 最大前进线速度 [m/s]
    max_lateral_speed_ = sdfValue<double>(sdf, "max_lateral_speed", 0.35);    // 最大侧向速度 [m/s]
    max_angular_speed_ = sdfValue<double>(sdf, "max_angular_speed", 1.05);    // 最大角速度 [rad/s]
    command_timeout_sec_ = sdfValue<double>(sdf, "command_timeout_sec", 0.5); // 指令超时时间 [s]
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate_hz", 100.0);         // 更新频率 [Hz]
    body_height_ = sdfValue<double>(sdf, "body_height", 0.59);                // 机体离地高度 [m]
    terrain_following_ = sdfValue<bool>(sdf, "terrain_following", true);      // 是否启用地形跟随
    terrain_probe_start_above_ = sdfValue<double>(sdf, "terrain_probe_start_above", 1.15);  // 地形探测起点（机体上方）[m]
    terrain_probe_end_below_ = sdfValue<double>(sdf, "terrain_probe_end_below", 1.20);      // 地形探测终点（机体下方）[m]
    max_terrain_step_up_ = sdfValue<double>(sdf, "max_terrain_step_up", 0.35);      // 允许的上台阶高度 [m]
    max_terrain_step_down_ = sdfValue<double>(sdf, "max_terrain_step_down", 0.75);    // 允许的下台阶高度 [m]
    max_vertical_speed_ = sdfValue<double>(sdf, "max_vertical_speed", 0.80);   // 最大竖直速度 [m/s]
    disable_robot_collisions_ = sdfValue<bool>(sdf, "disable_robot_collisions", true);   // 是否关闭机器人自身碰撞
    publish_odom_ = sdfValue<bool>(sdf, "publish_odom", true);                // 是否发布里程计
    publish_tf_ = sdfValue<bool>(sdf, "publish_tf", false);                   // 是否发布 TF
    publish_joint_states_ = sdfValue<bool>(sdf, "publish_joint_states", true);  // 是否发布关节状态
    animate_gait_ = sdfValue<bool>(sdf, "animate_gait", true);                // 是否启用步态动画
    gait_frequency_ = sdfValue<double>(sdf, "gait_frequency", 2.2);           // 步态频率 [Hz]
    min_walk_speed_ = sdfValue<double>(sdf, "min_walk_speed", 0.05);          // 开始行走的最小速度 [m/s]
    max_walk_speed_ = sdfValue<double>(sdf, "max_walk_speed", 1.0);           // 完全行走时的速度 [m/s]
    hip_swing_ = sdfValue<double>(sdf, "hip_swing", 0.08);                    // 髋关节摆动幅度 [rad]
    thigh_swing_ = sdfValue<double>(sdf, "thigh_swing", 0.32);                // 大腿摆动幅度 [rad]
    calf_swing_ = sdfValue<double>(sdf, "calf_swing", 0.42);                  // 小腿摆动幅度 [rad]
    wheel_radius_ = sdfValue<double>(sdf, "wheel_radius", 0.09);              // 轮子半径 [m]

    // ---- 物理模式配置：运动学驱动，关闭重力/碰撞，避免物理动力学干扰位姿控制 ----
    if (disable_robot_collisions_) {      // 允许关闭碰撞
      model_->SetCollideMode("none");     // 关闭模型整体碰撞
    }
    model_->SetGravityMode(false);        // 关闭模型重力
    for (const auto & link : model_->GetLinks()) {   // 遍历模型所有连杆
      link->SetGravityMode(false);        // 关闭每个连杆的重力
      link->SetKinematic(true);           // 设为运动学连杆（不受物理求解器支配）
      if (disable_robot_collisions_) {    // 允许关闭碰撞
        link->SetCollideMode("none");     // 关闭每个连杆的碰撞
      }
    }

    // ---- 初始化关节状态消息：16 个关节（4 腿 × 髋x/髋y/膝/轮） ----
    joint_msg_.name = {
      "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",   // 左前腿 4 关节
      "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",   // 右前腿 4 关节
      "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",   // 左后腿 4 关节
      "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};  // 右后腿 4 关节
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);   // 关节位置数组（初始全 0）
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);   // 关节速度数组（初始全 0）
    gazebo_joints_.reserve(joint_msg_.name.size());            // 预分配关节指针容器容量
    for (const auto & name : joint_msg_.name) {                // 按名字逐个查找关节
      gazebo_joints_.push_back(model_->GetJoint(name));        // 查找并存入关节指针列表
      if (!gazebo_joints_.back()) {                            // 未找到该关节
        gzerr << "[M20LeggedKinematicPlugin] Cannot find joint: " << name << "\n";   // 打印错误信息
      }
    }
    base_link_ = getLink(base_frame_id_);   // 获取基座连杆
    initializeLegChains();                  // 初始化四条腿的运动链结构（髋/膝/轮连杆及关节原点）

    // ---- 地形跟随：创建射线形状用于向下探测地面高度 ----
    if (terrain_following_ && world_ && world_->Physics()) {   // 启用地形跟随且物理引擎可用
      auto shape = world_->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr());   // 创建射线形状
      ray_shape_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(shape);   // 转换为 RayShape 类型
      if (!ray_shape_) {                    // 转换失败
        gzerr << "[M20LeggedKinematicPlugin] Terrain ray creation failed; z will stay kinematic.\n";   // 打印错误，z 将保持运动学默认
      }
    }

    if (!rclcpp::ok()) {                    // ROS2 尚未初始化（例如独立于 roslaunch 启动 Gazebo 时）
      int argc = 0;                         // 空参数个数
      char ** argv = nullptr;               // 空参数列表
      rclcpp::init(argc, argv);             // 初始化 ROS2 运行时
    }

    const std::string node_name = "m20_legged_kinematic_" + sanitizeName(model_->GetName());   // 以模型名生成唯一节点名
    node_ = std::make_shared<rclcpp::Node>(node_name);   // 创建 ROS2 节点
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(   // 订阅速度指令话题
      cmd_topic_,                        // 话题名（默认 /cmd_vel）
      rclcpp::QoS(10),                   // QoS：10 个消息的队列深度
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {   // 订阅回调：收到新速度指令时执行
        std::lock_guard<std::mutex> lock(cmd_mutex_);   // 加锁保护共享指令数据
        last_cmd_ = *msg;                // 保存最新指令
        last_cmd_wall_time_ = std::chrono::steady_clock::now();   // 记录收到指令的墙钟时刻（用于超时判断）
        has_cmd_ = true;                 // 标记已收到过指令
      });

    if (publish_odom_) {                 // 允许发布里程计
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 30);   // 创建里程计发布器（队列 30）
    }
    if (publish_joint_states_) {         // 允许发布关节状态
      joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);   // 创建关节状态发布器
    }
    if (publish_tf_) {                   // 允许发布 TF
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);   // 创建 TF 广播器
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();   // 单线程执行器：串行处理 ROS2 回调
    executor_->add_node(node_);          // 将节点加入执行器
    spin_thread_ = std::thread([this]() { executor_->spin(); });   // 后台线程持续 spin，处理订阅回调

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(   // 挂接世界更新回调（每个仿真步触发）
      std::bind(&M20LeggedKinematicPlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[M20LeggedKinematicPlugin] Loaded for " << model_->GetName()   // 打印加载信息
          << ", cmd_topic=" << cmd_topic_
          << ", odom_topic=" << odom_topic_
          << ", terrain_following=" << terrain_following_
          << ", body_height=" << body_height_
          << ", publish_tf=" << publish_tf_ << "\n";
  }

private:
  static constexpr double kPi = 3.14159265358979323846;   // 圆周率常量（π）

  // 单条腿的运动链结构：髋 x/y、膝、轮四段连杆及其在父系中的关节原点偏移
  struct LegChain {
    gazebo::physics::LinkPtr hipx;   // 髋部绕 x 轴连杆
    gazebo::physics::LinkPtr hipy;   // 髋部绕 y 轴（俯仰）连杆
    gazebo::physics::LinkPtr knee;   // 膝关节（大腿/小腿）连杆
    gazebo::physics::LinkPtr wheel;  // 轮子连杆
    ignition::math::Vector3d hipx_origin;    // hipx 关节相对机体的原点偏移
    ignition::math::Vector3d knee_origin;    // knee 关节相对 hipy 的原点偏移
    ignition::math::Vector3d wheel_origin;   // wheel 关节相对 knee 的原点偏移
  };

  // 世界更新回调：积分机体位姿 -> 地形跟随 -> 设置机体位姿/速度 -> 步态动画 -> 发布里程计
  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!node_) {              // 节点未初始化（提前退出场景）
      return;
    }

    const double sim_time = info.simTime.Double();   // 当前仿真时间 [s]
    double dt = 0.0;                       // 本次积分步长
    if (last_update_sim_time_ > 0.0) {     // 已运行过
      dt = sim_time - last_update_sim_time_;   // 计算实际步长
      if (update_rate_hz_ > 0.0 && dt < 1.0 / update_rate_hz_) {   // 限频：步长不足一个周期
        return;                            // 跳过本次更新
      }
    }
    last_update_sim_time_ = sim_time;      // 记录本次更新时间
    if (dt <= 0.0 || dt > 0.25) {          // 步长非法或过大（如刚启动/暂停恢复）
      dt = 1.0 / std::max(1.0, update_rate_hz_);   // 回退为固定标称步长
    }

    const auto command = freshCommand();   // 获取最新有效指令（带超时与限幅处理）
    const auto pose = model_->WorldPose(); // 当前机体世界位姿
    const double yaw = pose.Rot().Yaw();   // 当前偏航角
    const double cos_yaw = std::cos(yaw);  // 偏航角余弦
    const double sin_yaw = std::sin(yaw);  // 偏航角正弦
    const double world_vx = cos_yaw * command.linear.x - sin_yaw * command.linear.y;   // 坐标变换：机体系速度 -> 世界系 x 速度
    const double world_vy = sin_yaw * command.linear.x + cos_yaw * command.linear.y;   // 坐标变换：机体系速度 -> 世界系 y 速度

    double next_x = pose.Pos().X() + world_vx * dt;   // 积分出下一时刻 x
    double next_y = pose.Pos().Y() + world_vy * dt;   // 积分出下一时刻 y
    double next_z = pose.Pos().Z();                   // 默认保持当前 z
    double vertical_velocity = 0.0;                   // 竖直速度（默认 0）
    const double next_yaw = yaw + command.angular.z * dt;   // 积分出下一时刻偏航角

    if (terrain_following_ && ray_shape_) {   // 启用地形跟随且射线可用
      const double terrain = terrainHeightNear(next_x, next_y, pose.Pos().Z());   // 探测下一位置处的地面高度
      const double target_z = terrain + body_height_;   // 目标 z = 地面高度 + 机体离地高度
      const double max_dz = std::max(0.0, max_vertical_speed_) * dt;   // 单步最大竖直位移（竖直速度限幅）
      const double dz = std::clamp(target_z - pose.Pos().Z(), -max_dz, max_dz);   // 限制单步 z 变化量
      next_z = pose.Pos().Z() + dz;         // 更新下一时刻 z
      vertical_velocity = dz / std::max(1e-4, dt);   // 反算竖直速度
    }

    const ignition::math::Pose3d next_pose(next_x, next_y, next_z, 0.0, 0.0, next_yaw);   // 构造下一时刻机体位姿
    model_->SetWorldPose(next_pose, true, true);   // 设置模型世界位姿（运动学驱动）
    if (base_link_) {                      // 基座连杆存在
      base_link_->SetWorldPose(next_pose, true, true);   // 同步设置基座连杆位姿
    }
    model_->SetLinearVel(ignition::math::Vector3d(world_vx, world_vy, vertical_velocity));   // 设置模型线速度
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, command.angular.z));   // 设置模型角速度（仅偏航）

    updateGait(info.simTime, command, dt, next_pose);   // 更新步态动画（腿/轮关节姿态）
    publishOdometry(info.simTime, command, vertical_velocity);   // 发布里程计与 TF
  }

  // 获取最新指令：从未收到/已超时返回零速度；对线速度、侧向速度、角速度做限幅
  geometry_msgs::msg::Twist freshCommand() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);   // 加锁读取共享指令数据
    if (!has_cmd_) {              // 从未收到过指令
      return geometry_msgs::msg::Twist{};   // 返回全零指令（停车）
    }

    const double age = std::chrono::duration<double>(   // 计算指令年龄：距上次接收的秒数
      std::chrono::steady_clock::now() - last_cmd_wall_time_).count();
    if (age > command_timeout_sec_) {   // 超过超时时间：指令视为失效
      return geometry_msgs::msg::Twist{};   // 返回全零指令（自动停车）
    }

    geometry_msgs::msg::Twist command = last_cmd_;   // 拷贝最新指令
    command.linear.x = std::clamp(command.linear.x, -max_linear_speed_, max_linear_speed_);   // 前进线速度限幅
    command.linear.y = std::clamp(command.linear.y, -max_lateral_speed_, max_lateral_speed_); // 侧向速度限幅
    command.linear.z = 0.0;            // 竖直速度强制为 0（由地形跟随决定）
    command.angular.x = 0.0;           // 横滚角速度强制为 0
    command.angular.y = 0.0;           // 俯仰角速度强制为 0
    command.angular.z = std::clamp(command.angular.z, -max_angular_speed_, max_angular_speed_);   // 偏航角速度限幅
    return command;                // 返回处理后指令
  }

  // 地形探测：在 (x, y) 处从机体上方到下方发射射线，返回受限幅的地面高度
  double terrainHeightNear(double x, double y, double current_z) {
    const double current_terrain = current_z - body_height_;   // 当前估计的地面高度（机体 z 减去离地高度）
    const ignition::math::Vector3d start(x, y, current_z + terrain_probe_start_above_);   // 射线起点（机体上方）
    const ignition::math::Vector3d end(x, y, current_z - terrain_probe_end_below_);       // 射线终点（机体下方）
    ray_shape_->SetPoints(start, end);   // 设置射线起点与终点

    double distance = 0.0;           // 射线命中距离（输出参数）
    std::string entity;              // 命中的实体名（输出参数）
    ray_shape_->GetIntersection(distance, entity);   // 执行射线求交
    const double ray_length = (start - end).Length();   // 射线总长度
    if (!std::isfinite(distance) || distance < 0.0 || distance >= ray_length) {   // 未命中或距离非法
      return current_terrain;        // 返回当前估计地面高度
    }

    const double hit_z = start.Z() - distance;   // 由命中距离换算命中点高度
    if (hit_z > current_terrain + max_terrain_step_up_) {   // 地面比当前高太多（超出上台阶能力）
      return current_terrain;        // 保持当前高度（不强行上台阶）
    }
    if (hit_z < current_terrain - max_terrain_step_down_) {   // 地面比当前低太多（超出下台阶能力）
      return current_terrain;        // 保持当前高度（不强行下台阶）
    }
    return hit_z;                    // 返回受限幅后的地面高度
  }

  // 更新步态：根据前进速度计算站立/行走的混合比例，更新轮子自转与腿部关节姿态
  void updateGait(
    const gazebo::common::Time & sim_time,
    const geometry_msgs::msg::Twist & command,
    double dt,
    const ignition::math::Pose3d & base_pose) {
    builtin_interfaces::msg::Time stamp;    // 构建 ROS2 时间戳
    stamp.sec = sim_time.sec;               // 秒
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);   // 纳秒

    const double horizontal_speed = std::hypot(command.linear.x, command.linear.y);   // 水平合速度
    const double ratio = animate_gait_      // 行走姿态混合比例：在 min_walk_speed 与 max_walk_speed 之间线性插值
      ? std::clamp(
          (horizontal_speed - min_walk_speed_) /
          std::max(1e-3, max_walk_speed_ - min_walk_speed_),   // 归一化到 [0,1]（防除零）
          0.0, 1.0)
      : 0.0;                               // 未启用动画则恒为站立

    updateWheelMotion(command.linear.x, dt);   // 更新四个轮子的累计转角
    if (ratio <= 1e-3) {                   // 速度极低：处于站立姿态
      fillStance();                        // 填充固定的站立关节角
    } else {                               // 正在行走：按相位驱动四条腿摆动
      const double gait_direction = command.linear.x < -min_walk_speed_ ? -1.0 : 1.0;   // 后退时相位取反
      const double phase = gait_direction * 2.0 * kPi * gait_frequency_ * sim_time.Double();   // 步态相位（随时间线性增长）
      fillLeg(0, phase, ratio, true);        // 左前腿（FL）：基准相位
      fillLeg(4, phase + kPi, ratio, false); // 右前腿（FR）：反相（对角步态）
      fillLeg(8, phase + kPi, ratio, true);  // 左后腿（HL）：反相
      fillLeg(12, phase, ratio, false);      // 右后腿（HR）：基准相位
    }

    joint_msg_.header.stamp = stamp;         // 设置关节状态消息时间戳
    applyGazeboLinkPoses(base_pose);         // 按关节角把各腿部连杆设置到对应世界位姿
    if (joint_pub_) {                        // 关节状态发布器存在
      joint_pub_->publish(joint_msg_);       // 发布关节状态消息
    }
  }

  // 更新轮子运动：按前进速度累加四个轮子的转角（模拟轮子滚动）
  void updateWheelMotion(double forward_speed, double dt) {
    const double wheel_rate = forward_speed / std::max(0.01, wheel_radius_);   // 轮子角速度 = 线速度 / 轮半径
    for (double & position : wheel_position_) {   // 遍历四个轮子
      position += wheel_rate * dt;                // 累加转角
    }
    wheel_spin_rate_ = wheel_rate;                // 记录当前轮子角速度
  }

  // 站立姿态：填入四腿固定的站立关节角（髋x、髋y、膝、轮），速度为 0
  void fillStance() {
    const std::array<double, 16> stance = {       // 16 个关节的站立目标位置
      0.05, 0.82, -1.58, wheel_position_[0],      // 左前腿：髋x 0.05 / 髋y 0.82 / 膝 -1.58 / 轮
      -0.05, 0.82, -1.58, wheel_position_[1],     // 右前腿：髋x -0.05 / 其余同左前
      0.05, 0.95, -1.62, wheel_position_[2],      // 左后腿：髋y 0.95 / 膝 -1.62（后腿更蹲）
      -0.05, 0.95, -1.62, wheel_position_[3]};    // 右后腿
    for (size_t i = 0; i < stance.size(); ++i) {  // 遍历全部 16 个关节
      joint_msg_.position[i] = stance[i];         // 写入目标位置
      joint_msg_.velocity[i] = 0.0;               // 速度为 0
    }
  }

  // 行走步态：按相位正弦摆动填充一条腿的 4 个关节（髋x/髋y/膝/轮），并计算关节速度
  void fillLeg(size_t offset, double phase, double ratio, bool left_side) {
    const double s = std::sin(phase);         // 相位正弦（决定摆动方向）
    const double c = std::cos(phase);         // 相位余弦（决定跨步位置）
    const double swing = std::max(0.0, s);    // 只取正半周作为"迈步"阶段
    const double side = left_side ? 1.0 : -1.0;   // 左右腿髋x偏置方向
    const bool rear = offset >= 8;            // 是否为后腿（索引 8 起为后腿）
    const size_t leg_index = offset / 4;      // 腿序号（0..3）
    const double thigh_stance = rear ? 0.95 : 0.82;    // 站立基准：后腿大腿更前倾
    const double calf_stance = rear ? -1.62 : -1.58;   // 站立基准：后腿小腿更屈

    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);   // 髋x 外展摆动（随相位正弦）
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;     // 髋y（大腿）摆动
    joint_msg_.position[offset + 2] =       // 膝（小腿）摆动：迈步阶段伸展、支撑阶段微收
      calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);
    joint_msg_.position[offset + 3] = wheel_position_[leg_index];   // 轮子累计转角
    joint_msg_.position[offset] = std::clamp(joint_msg_.position[offset], -1.0, 1.0);       // 髋x 角度限幅
    joint_msg_.position[offset + 1] = std::clamp(joint_msg_.position[offset + 1], -1.2, 3.0);   // 大腿角度限幅
    joint_msg_.position[offset + 2] = std::clamp(joint_msg_.position[offset + 2], -2.6, -0.9);   // 小腿角度限幅

    const double omega = 2.0 * kPi * gait_frequency_;   // 步态角频率
    joint_msg_.velocity[offset] = side * hip_swing_ * ratio * 0.35 * c * omega;   // 髋x 角速度（位置导数的相位对应）
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * ratio * s * omega;          // 大腿角速度
    joint_msg_.velocity[offset + 2] = calf_swing_ * ratio * (s > 0.0 ? c : 0.0) * omega;   // 小腿角速度
    joint_msg_.velocity[offset + 3] = wheel_spin_rate_;   // 轮子角速度
  }

  // 按名字从模型中查找连杆；找不到时打印错误并返回空指针
  gazebo::physics::LinkPtr getLink(const std::string & name) const {
    auto link = model_->GetLink(name);   // 查询模型连杆
    if (!link) {                         // 未找到该连杆
      gzerr << "[M20LeggedKinematicPlugin] Cannot find link: " << name << "\n";   // 打印错误信息
    }
    return link;                         // 返回连杆（可能为空）
  }

  // 初始化四条腿的运动链：配置每腿的连杆指针与髋x/膝/轮关节原点偏移（前腿/后腿对称布置）
  void initializeLegChains() {
    configureLeg(
      legs_[0],                          // 左前腿（FL）
      "fl",
      ignition::math::Vector3d(0.3141, 0.0685, 0.0),       // hipx 关节原点偏移（机体前方偏左）
      ignition::math::Vector3d(0.0, 0.0984, -0.25),        // knee 关节原点偏移（髋部下方偏外）
      ignition::math::Vector3d(0.0, 0.059676, -0.25));     // wheel 关节原点偏移（膝部下方偏外）
    configureLeg(
      legs_[1],                          // 右前腿（FR）
      "fr",
      ignition::math::Vector3d(0.3141, -0.0685, 0.0),      // 机体前方偏右
      ignition::math::Vector3d(0.0, -0.0984, -0.25),       // 髋部下方偏内
      ignition::math::Vector3d(0.0, -0.059676, -0.25));    // 膝部下方偏内
    configureLeg(
      legs_[2],                          // 左后腿（HL）
      "hl",
      ignition::math::Vector3d(-0.3141, 0.0685, 0.0),      // 机体后方偏左
      ignition::math::Vector3d(0.0, 0.0984, -0.25),        // 髋部下方偏外
      ignition::math::Vector3d(0.0, 0.059676, -0.25));     // 膝部下方偏外
    configureLeg(
      legs_[3],                          // 右后腿（HR）
      "hr",
      ignition::math::Vector3d(-0.3141, -0.0685, 0.0),     // 机体后方偏右
      ignition::math::Vector3d(0.0, -0.0984, -0.25),       // 髋部下方偏内
      ignition::math::Vector3d(0.0, -0.059676, -0.25));    // 膝部下方偏内
  }

  // 配置单条腿：按前缀查找四个连杆，并保存髋x/膝/轮关节的原点偏移
  void configureLeg(
    LegChain & leg,
    const std::string & prefix,
    const ignition::math::Vector3d & hipx_origin,
    const ignition::math::Vector3d & knee_origin,
    const ignition::math::Vector3d & wheel_origin) {
    leg.hipx = getLink(prefix + "_hipx");     // 髋部绕 x 轴连杆
    leg.hipy = getLink(prefix + "_hipy");     // 髋部绕 y 轴（俯仰）连杆
    leg.knee = getLink(prefix + "_knee");     // 膝关节连杆
    leg.wheel = getLink(prefix + "_wheel");   // 轮子连杆
    leg.hipx_origin = hipx_origin;            // 保存 hipx 关节原点偏移
    leg.knee_origin = knee_origin;            // 保存 knee 关节原点偏移
    leg.wheel_origin = wheel_origin;          // 保存 wheel 关节原点偏移
  }

  // 运动学正解：由父级位姿、关节原点偏移、旋转轴与关节角，计算子关节的世界位姿
  ignition::math::Pose3d jointPose(
    const ignition::math::Pose3d & parent_pose,   // 父级连杆世界位姿
    const ignition::math::Vector3d & origin,      // 关节在父系中的原点偏移
    const ignition::math::Vector3d & axis,        // 关节旋转轴
    double position) const {                      // 关节角 [rad]
    return parent_pose *                          // 父位姿 左乘 相对变换
      ignition::math::Pose3d(origin, ignition::math::Quaterniond(axis, position));   // 平移 origin 并绕 axis 旋转 position
  }

  // 把 16 个关节角应用到 Gazebo 连杆：逐级串行计算髋x -> 髋y -> 膝 -> 轮的世界位姿并设置
  void applyGazeboLinkPoses(const ignition::math::Pose3d & base_pose) const {
    static const ignition::math::Vector3d hipx_axis(-1.0, 0.0, 0.0);   // 髋x 旋转轴：世界系 -x
    static const ignition::math::Vector3d pitch_axis(0.0, -1.0, 0.0);  // 髋y/膝/轮旋转轴：世界系 -y

    if (base_link_) {                      // 基座连杆存在
      base_link_->SetWorldPose(base_pose, true, true);   // 设置基座位姿
    }

    for (size_t leg_index = 0; leg_index < legs_.size(); ++leg_index) {   // 遍历四条腿
      const auto & leg = legs_[leg_index];       // 当前腿的运动链
      const size_t offset = leg_index * 4;       // 该腿 4 个关节在消息中的起始索引
      const auto hipx_pose =                    // 运动学正解：hipx 连杆位姿
        jointPose(base_pose, leg.hipx_origin, hipx_axis, joint_msg_.position[offset]);
      const auto hipy_pose =                    // 运动学正解：hipy 连杆位姿（在 hipx 基础上）
        jointPose(hipx_pose, ignition::math::Vector3d::Zero, pitch_axis, joint_msg_.position[offset + 1]);
      const auto knee_pose =                    // 运动学正解：knee 连杆位姿（在 hipy 基础上）
        jointPose(hipy_pose, leg.knee_origin, pitch_axis, joint_msg_.position[offset + 2]);
      const auto wheel_pose =                   // 运动学正解：wheel 连杆位姿（在 knee 基础上）
        jointPose(knee_pose, leg.wheel_origin, pitch_axis, joint_msg_.position[offset + 3]);

      if (leg.hipx) {                           // hipx 连杆存在
        leg.hipx->SetWorldPose(hipx_pose, true, true);   // 设置 hipx 世界位姿
      }
      if (leg.hipy) {                           // hipy 连杆存在
        leg.hipy->SetWorldPose(hipy_pose, true, true);   // 设置 hipy 世界位姿
      }
      if (leg.knee) {                           // knee 连杆存在
        leg.knee->SetWorldPose(knee_pose, true, true);   // 设置 knee 世界位姿
      }
      if (leg.wheel) {                          // wheel 连杆存在
        leg.wheel->SetWorldPose(wheel_pose, true, true);   // 设置 wheel 世界位姿
      }
    }
  }

  // 发布里程计消息（/odom）与 TF 变换（odom -> base_link）
  void publishOdometry(
    const gazebo::common::Time & sim_time,
    const geometry_msgs::msg::Twist & command,
    double vertical_velocity) {
    if (!publish_odom_ && !publish_tf_) {   // 两者都未启用
      return;
    }

    builtin_interfaces::msg::Time stamp;    // 构建 ROS2 时间戳
    stamp.sec = sim_time.sec;               // 秒
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);   // 纳秒
    const auto pose = model_->WorldPose();  // 模型当前世界位姿

    if (odom_pub_) {                        // 发布里程计消息
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;    // 消息时间戳
      odom.header.frame_id = odom_frame_id_;   // 父坐标系：odom
      odom.child_frame_id = base_frame_id_;    // 子坐标系：base_link
      odom.pose.pose.position.x = pose.Pos().X();   // 位置 x
      odom.pose.pose.position.y = pose.Pos().Y();   // 位置 y
      odom.pose.pose.position.z = pose.Pos().Z();   // 位置 z
      odom.pose.pose.orientation.x = pose.Rot().X();  // 姿态四元数 x
      odom.pose.pose.orientation.y = pose.Rot().Y();  // 姿态四元数 y
      odom.pose.pose.orientation.z = pose.Rot().Z();  // 姿态四元数 z
      odom.pose.pose.orientation.w = pose.Rot().W();  // 姿态四元数 w
      odom.twist.twist.linear.x = command.linear.x;   // 线速度 x（取指令值）
      odom.twist.twist.linear.y = command.linear.y;   // 线速度 y（取指令值）
      odom.twist.twist.linear.z = vertical_velocity;  // 线速度 z（地形跟随反算值）
      odom.twist.twist.angular.z = command.angular.z; // 偏航角速度（取指令值）
      odom_pub_->publish(odom);      // 发布里程计消息
    }

    if (tf_broadcaster_) {               // 发布 TF 变换（tf 广播）
      geometry_msgs::msg::TransformStamped transform;   // 构造 TF 消息
      transform.header.stamp = stamp;      // 时间戳
      transform.header.frame_id = odom_frame_id_;       // 父坐标系：odom
      transform.child_frame_id = base_frame_id_;        // 子坐标系：base_link
      transform.transform.translation.x = pose.Pos().X();  // 平移 x
      transform.transform.translation.y = pose.Pos().Y();  // 平移 y
      transform.transform.translation.z = pose.Pos().Z();  // 平移 z
      transform.transform.rotation.x = pose.Rot().X();     // 旋转四元数 x
      transform.transform.rotation.y = pose.Rot().Y();     // 旋转四元数 y
      transform.transform.rotation.z = pose.Rot().Z();     // 旋转四元数 z
      transform.transform.rotation.w = pose.Rot().W();     // 旋转四元数 w
      tf_broadcaster_->sendTransform(transform);    // 广播 TF 变换
    }
  }

  // ---- Gazebo 物理实体 ----
  gazebo::physics::ModelPtr model_;            // 绑定的 Gazebo 模型指针
  gazebo::physics::WorldPtr world_;            // 模型所在的世界指针
  gazebo::event::ConnectionPtr update_connection_;   // 世界更新事件连接（析构时自动断开）
  gazebo::physics::RayShapePtr ray_shape_;     // 地形探测射线形状
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;   // 模型关节指针列表（用于查找校验）
  gazebo::physics::LinkPtr base_link_;         // 基座连杆
  std::array<LegChain, 4> legs_;               // 四条腿的运动链结构（FL/FR/HL/HR）

  // ---- ROS2 通信对象 ----
  rclcpp::Node::SharedPtr node_;                       // ROS2 节点
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;   // /cmd_vel 速度指令订阅器
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;       // /odom 里程计发布器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_; // /joint_states 关节状态发布器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;        // TF 广播器
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // ROS2 单线程执行器
  std::thread spin_thread_;                          // 执行器后台 spin 线程

  // ---- 指令缓存（订阅回调线程与仿真主线程共享，需加锁） ----
  mutable std::mutex cmd_mutex_;                     // 保护指令数据的互斥锁
  geometry_msgs::msg::Twist last_cmd_;               // 最近一次收到的速度指令
  std::chrono::steady_clock::time_point last_cmd_wall_time_{std::chrono::steady_clock::now()};   // 最近一次收到指令的墙钟时刻
  bool has_cmd_{false};                              // 是否已收到过指令

  // ---- 关节状态与轮子运动状态 ----
  sensor_msgs::msg::JointState joint_msg_;           // 待发布的关节状态消息
  std::array<double, 4> wheel_position_{0.0, 0.0, 0.0, 0.0};   // 四个轮子的累计转角
  double wheel_spin_rate_{0.0};                      // 当前轮子角速度

  // ---- 话题名与坐标系名 ----
  std::string cmd_topic_{"/cmd_vel"};        // 速度指令话题名
  std::string odom_topic_{"/odom"};          // 里程计话题名
  std::string joint_states_topic_{"/joint_states"};   // 关节状态话题名
  std::string odom_frame_id_{"odom"};        // 里程计父坐标系
  std::string base_frame_id_{"base_link"};   // 机器人基座坐标系

  // ---- 运动学与限幅参数 ----
  double max_linear_speed_{0.45};            // 最大前进线速度 [m/s]
  double max_lateral_speed_{0.35};           // 最大侧向速度 [m/s]
  double max_angular_speed_{1.05};           // 最大角速度 [rad/s]
  double command_timeout_sec_{0.5};          // 指令超时时间 [s]
  double update_rate_hz_{100.0};             // 更新频率 [Hz]
  double body_height_{0.59};                 // 机体离地高度 [m]

  // ---- 地形跟随参数 ----
  double terrain_probe_start_above_{1.15};   // 地形探测起点（机体上方距离）[m]
  double terrain_probe_end_below_{1.20};     // 地形探测终点（机体下方距离）[m]
  double max_terrain_step_up_{0.35};         // 允许的上台阶高度 [m]
  double max_terrain_step_down_{0.75};       // 允许的下台阶高度 [m]
  double max_vertical_speed_{0.80};          // 最大竖直速度 [m/s]

  // ---- 步态动画参数 ----
  double gait_frequency_{2.2};               // 步态频率 [Hz]
  double min_walk_speed_{0.05};              // 开始行走的最小速度 [m/s]
  double max_walk_speed_{1.0};               // 完全行走时的速度 [m/s]
  double hip_swing_{0.08};                   // 髋关节摆动幅度 [rad]
  double thigh_swing_{0.32};                 // 大腿摆动幅度 [rad]
  double calf_swing_{0.42};                  // 小腿摆动幅度 [rad]
  double wheel_radius_{0.09};                // 轮子半径 [m]

  // ---- 状态与开关 ----
  double last_update_sim_time_{0.0};         // 上次执行更新的仿真时间
  bool terrain_following_{true};             // 是否启用地形跟随
  bool disable_robot_collisions_{true};      // 是否关闭机器人自身碰撞
  bool publish_odom_{true};                  // 是否发布里程计
  bool publish_tf_{false};                   // 是否发布 TF
  bool publish_joint_states_{true};          // 是否发布关节状态
  bool animate_gait_{true};                  // 是否启用步态动画
};

GZ_REGISTER_MODEL_PLUGIN(M20LeggedKinematicPlugin)   // 注册为 Gazebo 模型插件

}  // namespace m20_nav2_system
