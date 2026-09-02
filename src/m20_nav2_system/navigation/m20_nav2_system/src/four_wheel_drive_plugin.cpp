/**
 * @file four_wheel_drive_plugin.cpp
 * @brief 四轮差速驱动 Gazebo 模型插件（M20FourWheelDrivePlugin）
 *
 * 职责：
 *  - 订阅速度指令话题（默认 /cmd_vel），接收 geometry_msgs/Twist 的线速度/角速度指令；
 *  - 依据差速运动学模型，将线速度/角速度换算为左右两侧车轮的目标角速度，
 *    通过 Gazebo 关节接口（SetParam("vel"/"fmax")）驱动四个轮子关节；
 *  - 可选地通过直接设置模型速度（SetLinearVel/SetAngularVel）实现平面速度直控；
 *  - 可选地发布里程计话题（默认 /odom，nav_msgs/Odometry）与
 *    TF 变换（odom -> base_link，tf2_ros TransformBroadcaster）；
 *  - 指令带超时保护：超过 command_timeout_sec 未收到新指令则自动停车；
 *    速度指令还会被 max_linear_speed / max_angular_speed 限幅。
 *
 * 通过 GZ_REGISTER_MODEL_PLUGIN 注册为 Gazebo 模型插件，由模型 <plugin> 标签加载。
 */
#include <algorithm>    // 标准算法库：std::clamp（速度限幅）等
#include <cctype>       // 字符分类函数：std::isalnum（模型名清理）
#include <chrono>       // 时间库：std::chrono（指令超时计时）
#include <functional>   // 函数对象：std::bind / std::placeholders（绑定世界更新回调）
#include <memory>       // 智能指针：std::shared_ptr / std::make_shared
#include <mutex>        // 互斥锁：std::mutex / std::lock_guard（保护指令共享数据）
#include <string>       // 字符串：std::string（话题名、坐标系名等）
#include <thread>       // 线程：std::thread（ROS2 执行器后台 spin 线程）

#include "builtin_interfaces/msg/time.hpp"   // ROS2 时间消息：Odometry 头的时间戳
#include "gazebo/common/Events.hh"           // Gazebo 事件：世界更新事件（WorldUpdateBegin）
#include "gazebo/gazebo.hh"                  // Gazebo 主头文件：插件注册宏等
#include "gazebo/physics/physics.hh"         // Gazebo 物理：Model/Joint/World 等物理实体
#include "geometry_msgs/msg/transform_stamped.hpp"   // TF 消息：TransformStamped（发布 TF 用）
#include "geometry_msgs/msg/twist.hpp"       // 速度消息：Twist（订阅 /cmd_vel 指令）
#include "nav_msgs/msg/odometry.hpp"         // 里程计消息：Odometry（发布 /odom）
#include "rclcpp/rclcpp.hpp"                 // ROS2 C++ 客户端库：Node/Publisher/Subscription
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

// 四轮差速驱动插件：继承 ModelPlugin 绑定到机器人模型，实现 cmd_vel -> 轮速 -> odom/tf 全链路
class M20FourWheelDrivePlugin : public gazebo::ModelPlugin {
public:
  M20FourWheelDrivePlugin() = default;   // 默认构造函数

  // 析构函数：退出前取消执行器并等待后台 spin 线程结束，避免悬挂线程
  ~M20FourWheelDrivePlugin() override {
    if (executor_) {              // 执行器存在
      executor_->cancel();        // 取消执行器，停止处理 ROS2 回调
    }
    if (spin_thread_.joinable()) {  // spin 线程可 join（仍在运行）
      spin_thread_.join();          // 等待线程结束
    }
  }

  // 插件加载入口：由 Gazebo 在模型创建时调用，完成参数解析、关节获取、ROS2 初始化与回调挂接
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;                    // 保存当前插件绑定的模型指针
    world_ = model_->GetWorld();       // 获取模型所在的世界指针

    // ---- 从 SDF 读取可配置参数（未配置时使用括号内的默认值） ----
    wheel_radius_ = sdfValue<double>(sdf, "wheel_radius", 0.09);              // 车轮半径 [m]
    wheel_separation_ = sdfValue<double>(sdf, "wheel_separation", 0.453);     // 左右轮距 [m]
    wheel_velocity_sign_ = sdfValue<double>(sdf, "wheel_velocity_sign", -1.0);  // 轮速方向符号系数
    max_wheel_torque_ = sdfValue<double>(sdf, "max_wheel_torque", 28.0);      // 单轮最大输出扭矩 [Nm]
    max_linear_speed_ = sdfValue<double>(sdf, "max_linear_speed", 0.45);      // 最大线速度 [m/s]
    max_angular_speed_ = sdfValue<double>(sdf, "max_angular_speed", 0.65);    // 最大角速度 [rad/s]
    command_timeout_sec_ = sdfValue<double>(sdf, "command_timeout_sec", 0.5); // 指令超时时间 [s]
    update_rate_hz_ = sdfValue<double>(sdf, "update_rate_hz", 100.0);         // 更新频率 [Hz]
    use_planar_velocity_ = sdfValue<bool>(sdf, "use_planar_velocity", false); // 是否启用平面速度直控
    publish_odom_ = sdfValue<bool>(sdf, "publish_odom", true);                // 是否发布里程计
    publish_tf_ = sdfValue<bool>(sdf, "publish_tf", true);                    // 是否发布 TF
    cmd_topic_ = sdfValue<std::string>(sdf, "cmd_topic", "/cmd_vel");         // 速度指令话题名
    odom_topic_ = sdfValue<std::string>(sdf, "odom_topic", "/odom");          // 里程计话题名
    odom_frame_id_ = sdfValue<std::string>(sdf, "odom_frame_id", "odom");     // 里程计父坐标系
    base_frame_id_ = sdfValue<std::string>(sdf, "base_frame_id", "base_link");  // 机器人基座坐标系

    // ---- 获取四个轮子的驱动关节 ----
    front_left_joint_ = getJoint(sdfValue<std::string>(sdf, "front_left_joint", "fl_wheel_joint"));   // 左前轮关节
    rear_left_joint_ = getJoint(sdfValue<std::string>(sdf, "rear_left_joint", "hl_wheel_joint"));     // 左后轮关节
    front_right_joint_ = getJoint(sdfValue<std::string>(sdf, "front_right_joint", "fr_wheel_joint")); // 右前轮关节
    rear_right_joint_ = getJoint(sdfValue<std::string>(sdf, "rear_right_joint", "hr_wheel_joint"));   // 右后轮关节

    if (!front_left_joint_ || !rear_left_joint_ || !front_right_joint_ || !rear_right_joint_) {  // 任一车轮关节缺失
      gzerr << "[M20FourWheelDrivePlugin] Missing one or more wheel joints. Plugin disabled.\n";  // 打印错误并禁用插件
      return;                            // 提前退出，不再初始化
    }

    if (!rclcpp::ok()) {                 // ROS2 尚未初始化（例如独立于 roslaunch 启动 Gazebo 时）
      int argc = 0;                      // 空参数个数
      char ** argv = nullptr;            // 空参数列表
      rclcpp::init(argc, argv);          // 初始化 ROS2 运行时
    }

    const std::string node_name = "m20_four_wheel_drive_" + sanitizeName(model_->GetName());  // 以模型名生成唯一节点名
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
    if (publish_tf_) {                   // 允许发布 TF
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);   // 创建 TF 广播器
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();   // 单线程执行器：串行处理 ROS2 回调
    executor_->add_node(node_);          // 将节点加入执行器
    spin_thread_ = std::thread([this]() { executor_->spin(); });   // 后台线程持续 spin，处理订阅回调

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(   // 挂接世界更新回调（每个仿真步触发）
      std::bind(&M20FourWheelDrivePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[M20FourWheelDrivePlugin] Loaded for " << model_->GetName()   // 打印加载信息
          << ", cmd_topic=" << cmd_topic_
          << ", odom_topic=" << odom_topic_
          << ", wheel_radius=" << wheel_radius_
          << ", wheel_separation=" << wheel_separation_
          << ", wheel_velocity_sign=" << wheel_velocity_sign_ << "\n";
  }

private:
  // 按名字从模型中查找关节；找不到时打印错误并返回空指针
  gazebo::physics::JointPtr getJoint(const std::string & name) const {
    auto joint = model_->GetJoint(name);   // 查询模型关节
    if (!joint) {                          // 未找到该关节
      gzerr << "[M20FourWheelDrivePlugin] Cannot find joint: " << name << "\n";   // 打印错误信息
    }
    return joint;                          // 返回关节（可能为空）
  }

  // 世界更新回调：限频执行"取指令 -> 驱动轮子 -> 发布里程计/TF"的完整流程
  void onUpdate(const gazebo::common::UpdateInfo & info) {
    if (!node_) {              // 节点未初始化（关节缺失导致提前退出）
      return;
    }

    const double sim_time = info.simTime.Double();   // 当前仿真时间 [s]
    if (last_update_sim_time_ > 0.0 && update_rate_hz_ > 0.0) {   // 已运行过且限频开启
      const double min_period = 1.0 / update_rate_hz_;   // 最小更新周期 [s]
      if (sim_time - last_update_sim_time_ < min_period) {   // 距上次更新不足一个周期
        return;                      // 跳过本次更新，实现限频
      }
    }
    last_update_sim_time_ = sim_time;    // 记录本次更新时间

    const auto command = freshCommand();      // 获取最新有效指令（带超时与限幅处理）
    applyWheelVelocity(command.linear.x, command.angular.z);   // 差速运动学：换算左右轮速并驱动四轮
    if (use_planar_velocity_) {               // 启用平面速度直控模式
      applyPlanarVelocity(command.linear.x, command.angular.z);   // 直接设置模型线/角速度
    }
    publishOdometry(info.simTime);            // 发布里程计消息与 TF 变换
  }

  // 获取最新指令：从未收到/已超时返回零速度；对线速度与角速度做限幅
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
    command.linear.x = std::clamp(command.linear.x, -max_linear_speed_, max_linear_speed_);   // 线速度限幅
    command.angular.z = std::clamp(command.angular.z, -max_angular_speed_, max_angular_speed_);   // 角速度限幅
    return command;                // 返回处理后指令
  }

  // 差速运动学：将线速度 linear_x 与角速度 angular_z 换算为左右轮角速度，并驱动四个轮子关节
  void applyWheelVelocity(double linear_x, double angular_z) {
    const double left_velocity = wheel_velocity_sign_ *   // 左轮角速度 = 符号系数 * (线速度 - 角速度*半轮距) / 轮半径
      (linear_x - angular_z * wheel_separation_ * 0.5) / wheel_radius_;
    const double right_velocity = wheel_velocity_sign_ *  // 右轮角速度 = 符号系数 * (线速度 + 角速度*半轮距) / 轮半径
      (linear_x + angular_z * wheel_separation_ * 0.5) / wheel_radius_;

    setJointVelocity(front_left_joint_, left_velocity);    // 左前轮
    setJointVelocity(rear_left_joint_, left_velocity);     // 左后轮（与左前轮同速）
    setJointVelocity(front_right_joint_, right_velocity);  // 右前轮
    setJointVelocity(rear_right_joint_, right_velocity);   // 右后轮（与右前轮同速）
  }

  // 设置单个关节的目标角速度，并附带最大扭矩限制（Gazebo 物理交互接口）
  void setJointVelocity(const gazebo::physics::JointPtr & joint, double velocity) const {
    joint->SetParam("fmax", 0, max_wheel_torque_);   // 设置关节 0 轴的最大输出扭矩 [Nm]
    joint->SetParam("vel", 0, velocity);             // 设置关节 0 轴的目标角速度 [rad/s]
  }

  // 平面速度直控：绕过关节，直接把期望速度赋给模型（用于无轮/虚拟模型场景）
  void applyPlanarVelocity(double linear_x, double angular_z) const {
    const auto pose = model_->WorldPose();   // 当前模型世界位姿
    const auto world_linear = pose.Rot().RotateVector(   // 坐标变换：将机体系 x 向速度旋转到世界系
      ignition::math::Vector3d(linear_x, 0.0, 0.0));
    const auto current_linear = model_->WorldLinearVel();   // 当前世界系线速度
    model_->SetLinearVel(    // 设置模型线速度（保留竖直方向 z 分量）
      ignition::math::Vector3d(world_linear.X(), world_linear.Y(), current_linear.Z()));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, angular_z));   // 设置模型角速度（仅偏航）
  }

  // 发布里程计消息（/odom）与 TF 变换（odom -> base_link）
  void publishOdometry(const gazebo::common::Time & sim_time) {
    if (!publish_odom_ && !publish_tf_) {   // 两者都未启用
      return;
    }

    builtin_interfaces::msg::Time stamp;    // 构建 ROS2 时间戳
    stamp.sec = sim_time.sec;               // 秒
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);   // 纳秒
    const auto pose = model_->WorldPose();        // 模型世界系位姿
    const auto linear = model_->WorldLinearVel(); // 模型世界系线速度
    const auto angular = model_->WorldAngularVel();   // 模型世界系角速度
    const auto body_linear = pose.Rot().RotateVectorReverse(linear);   // 坐标变换：世界系速度旋转到机体坐标系
    const auto body_angular = pose.Rot().RotateVectorReverse(angular); // 坐标变换：世界系角速度旋转到机体坐标系

    if (publish_odom_) {            // 发布里程计消息
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
      odom.twist.twist.linear.x = body_linear.X();    // 机体系线速度 x
      odom.twist.twist.linear.y = body_linear.Y();    // 机体系线速度 y
      odom.twist.twist.linear.z = body_linear.Z();    // 机体系线速度 z
      odom.twist.twist.angular.x = body_angular.X();  // 机体系角速度 x
      odom.twist.twist.angular.y = body_angular.Y();  // 机体系角速度 y
      odom.twist.twist.angular.z = body_angular.Z();  // 机体系角速度 z
      odom_pub_->publish(odom);      // 发布里程计消息
    }

    if (publish_tf_) {               // 发布 TF 变换（tf 广播）
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
  gazebo::physics::JointPtr front_left_joint_;   // 左前轮关节
  gazebo::physics::JointPtr rear_left_joint_;    // 左后轮关节
  gazebo::physics::JointPtr front_right_joint_;  // 右前轮关节
  gazebo::physics::JointPtr rear_right_joint_;   // 右后轮关节

  // ---- ROS2 通信对象 ----
  rclcpp::Node::SharedPtr node_;                       // ROS2 节点
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;   // /cmd_vel 速度指令订阅器
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;       // /odom 里程计发布器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;        // TF 广播器
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // ROS2 单线程执行器
  std::thread spin_thread_;                          // 执行器后台 spin 线程

  // ---- 指令缓存（订阅回调线程与仿真主线程共享，需加锁） ----
  mutable std::mutex cmd_mutex_;                     // 保护指令数据的互斥锁
  geometry_msgs::msg::Twist last_cmd_;               // 最近一次收到的速度指令
  std::chrono::steady_clock::time_point last_cmd_wall_time_{std::chrono::steady_clock::now()};   // 最近一次收到指令的墙钟时刻
  bool has_cmd_{false};                              // 是否已收到过指令

  // ---- 话题名与坐标系名 ----
  std::string cmd_topic_{"/cmd_vel"};        // 速度指令话题名
  std::string odom_topic_{"/odom"};          // 里程计话题名
  std::string odom_frame_id_{"odom"};        // 里程计父坐标系
  std::string base_frame_id_{"base_link"};   // 机器人基座坐标系

  // ---- 运动学与限幅参数 ----
  double wheel_radius_{0.09};                // 车轮半径 [m]
  double wheel_separation_{0.453};           // 左右轮距 [m]
  double wheel_velocity_sign_{-1.0};         // 轮速方向符号系数
  double max_wheel_torque_{28.0};            // 单轮最大输出扭矩 [Nm]
  double max_linear_speed_{0.45};            // 最大线速度 [m/s]
  double max_angular_speed_{0.65};           // 最大角速度 [rad/s]
  double command_timeout_sec_{0.5};          // 指令超时时间 [s]
  double update_rate_hz_{100.0};             // 更新频率 [Hz]
  double last_update_sim_time_{0.0};         // 上次执行更新的仿真时间
  bool use_planar_velocity_{false};          // 是否启用平面速度直控
  bool publish_odom_{true};                  // 是否发布里程计
  bool publish_tf_{true};                    // 是否发布 TF
};

GZ_REGISTER_MODEL_PLUGIN(M20FourWheelDrivePlugin)   // 注册为 Gazebo 模型插件

}  // namespace m20_nav2_system
