/**
 * @file m20_gait_publisher.cpp
 * @brief M20 步态关节状态发布节点（M20GaitPublisher）
 *
 * 职责：
 *  - 订阅机体里程计（body_pose），从 twist 速度/位置差分估计水平与前进速度；
 *  - 按速度大小在"站立姿态"与"行走（对角小跑）姿态"之间插值，
 *    生成 16 个关节（4 腿 × 髋x/髋y/膝/轮）的目标位置与速度；
 *  - 以固定频率（默认 60 Hz）发布关节状态（joint_states，JointState），
 *    供下游运动控制/仿真使用；
 *  - 轮子按前进速度累计转角，模拟轮腿复合机器人的轮子滚动。
 */
#include <algorithm>    // 标准算法库：std::clamp / std::max
#include <array>        // 容器：std::array（固定大小数组）
#include <chrono>       // 时间库：std::chrono（定时器周期换算）
#include <cmath>        // 数学库：std::sin / std::cos / std::hypot
#include <memory>       // 智能指针：std::make_shared

#include <nav_msgs/msg/odometry.hpp>       // 里程计消息：Odometry（订阅 body_pose）
#include <rclcpp/rclcpp.hpp>               // ROS2 C++ 客户端库：Node/Publisher/Timer
#include <sensor_msgs/msg/joint_state.hpp> // 关节状态消息：JointState（发布 joint_states）

namespace scan_planner    // 扫描规划器命名空间
{
// M20 步态发布器：根据速度生成站立/行走关节状态并发布
class M20GaitPublisher : public rclcpp::Node
{
public:
  // 构造函数：声明参数、创建订阅/发布/定时器并初始化关节名
  M20GaitPublisher() : Node("m20_gait_publisher")
  {
    const double rate = declare_parameter<double>("rate", 60.0);          // 发布频率 [Hz]
    gait_frequency_ = declare_parameter<double>("gait_frequency", 2.2);   // 步态频率 [Hz]
    min_walk_speed_ = declare_parameter<double>("min_walk_speed", 0.05);  // 开始行走的最小速度 [m/s]
    max_walk_speed_ = declare_parameter<double>("max_walk_speed", 1.0);   // 完全行走时的速度 [m/s]
    always_trot_ = declare_parameter<bool>("always_trot", false);         // 是否恒定小跑（忽略速度插值）
    hip_swing_ = declare_parameter<double>("hip_swing", 0.08);            // 髋关节摆动幅度 [rad]
    thigh_swing_ = declare_parameter<double>("thigh_swing", 0.32);        // 大腿摆动幅度 [rad]
    calf_swing_ = declare_parameter<double>("calf_swing", 0.42);          // 小腿摆动幅度 [rad]
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.09);      // 轮子半径 [m]

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(   // 订阅机体里程计（传感器 QoS）
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&M20GaitPublisher::odomCallback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);   // 发布关节状态
    timer_ = create_wall_timer(          // 周期定时器：按发布频率驱动
        std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
        std::bind(&M20GaitPublisher::timerCallback, this));

    joint_msg_.name = {                  // 16 个关节名（4 腿 × 4 关节）
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",   // 左前腿
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",   // 右前腿
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",   // 左后腿
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};  // 右后腿
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);   // 位置数组（初始全 0）
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);   // 速度数组（初始全 0）
  }

private:
  static constexpr double kPi = 3.14159265358979323846;   // 圆周率常量（π）

  // 里程计回调：从 twist 速度/位置差分估计水平与前进速度
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const double vx = odom->twist.twist.linear.x;   // 机体系前进速度
    const double vy = odom->twist.twist.linear.y;   // 机体系侧向速度
    const double twist_speed = std::hypot(vx, vy);  // twist 合速度
    rclcpp::Time stamp(odom->header.stamp);         // 消息时间戳
    if (stamp.nanoseconds() == 0) stamp = now();    // 时间戳为 0 则用当前时刻
    const double x = odom->pose.pose.position.x;    // 位置 x
    const double y = odom->pose.pose.position.y;    // 位置 y
    double pose_speed = 0.0;                        // 位置差分速度（兜底）
    if (has_prev_pose_)                             // 有上一次位置
    {
      const double dt = (stamp - last_odom_time_).seconds();   // 时间差
      if (dt > 1e-4) pose_speed = std::hypot(x - last_odom_x_, y - last_odom_y_) / dt;   // 位置差分速度
    }
    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;   // 水平速度：优先 twist，否则差分
    forward_speed_ = std::abs(vx) > min_walk_speed_ ? vx : horizontal_speed_;       // 前进速度（驱动轮子滚动）
    last_odom_x_ = x;                    // 缓存位置与时间
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;               // 标记已有上一次位置
    has_odom_ = true;                    // 标记已收到里程计
  }

  // 定时器回调：根据速度计算行走比例，发布站立或行走关节状态
  void timerCallback()
  {
    const auto stamp = now();            // 当前时刻
    if (!has_odom_)                      // 尚未收到里程计
    {
      publishStance(stamp);              // 发布站立姿态
      return;
    }
    const double ratio = always_trot_ ? 0.45 : std::clamp(   // 行走混合比例：恒定小跑取 0.45，否则按速度线性插值
        (horizontal_speed_ - min_walk_speed_) / std::max(1e-3, max_walk_speed_ - min_walk_speed_),
        0.0, 1.0);
    if (ratio <= 1e-3)                   // 速度极低：站立
    {
      publishStance(stamp);              // 发布站立姿态
      return;
    }
    const double phase = 2.0 * kPi * gait_frequency_ * stamp.seconds();   // 步态相位（随时间线性增长）
    updateWheelMotion(stamp);            // 更新轮子累计转角
    fillLeg(0, phase, ratio, true);      // 左前腿（FL）：基准相位
    fillLeg(4, phase + kPi, ratio, false);   // 右前腿（FR）：反相（对角步态）
    fillLeg(8, phase + kPi, ratio, true);    // 左后腿（HL）：反相
    fillLeg(12, phase, ratio, false);        // 右后腿（HR）：基准相位
    joint_msg_.header.stamp = stamp;     // 设置消息时间戳
    joint_pub_->publish(joint_msg_);     // 发布关节状态
  }

  // 发布站立姿态：四腿固定站立关节角，速度 0
  void publishStance(const rclcpp::Time &stamp)
  {
    const std::array<double, 16> stance = {      // 16 个关节的站立目标位置
        0.05, 0.82, -1.58, wheel_position_[0],   // 左前腿：髋x 0.05 / 髋y 0.82 / 膝 -1.58 / 轮
        -0.05, 0.82, -1.58, wheel_position_[1],  // 右前腿
        0.05, 0.95, -1.62, wheel_position_[2],   // 左后腿：髋y 0.95 / 膝 -1.62（后腿更蹲）
        -0.05, 0.95, -1.62, wheel_position_[3]}; // 右后腿
    for (size_t i = 0; i < stance.size(); ++i)
    {
      joint_msg_.position[i] = stance[i];        // 写入目标位置
      joint_msg_.velocity[i] = 0.0;              // 速度为 0
    }
    joint_msg_.header.stamp = stamp;     // 设置消息时间戳
    joint_pub_->publish(joint_msg_);     // 发布关节状态
  }

  // 更新轮子运动：按前进速度累加四个轮子的转角
  void updateWheelMotion(const rclcpp::Time &stamp)
  {
    double dt = 0.0;                     // 时间差
    if (has_last_publish_time_)          // 有上次发布时间
    {
      dt = (stamp - last_publish_time_).seconds();   // 计算时间差
      if (dt < 0.0 || dt > 0.2) dt = 0.0;            // 非法/过大则置 0
    }
    last_publish_time_ = stamp;          // 更新上次发布时间
    has_last_publish_time_ = true;       // 标记已有上次发布时间
    const double wheel_rate = forward_speed_ / std::max(0.01, wheel_radius_);   // 轮子角速度 = 前进速度 / 轮半径
    for (double &position : wheel_position_)   // 遍历四个轮子
    {
      position += wheel_rate * dt;       // 累加转角
    }
    wheel_spin_rate_ = wheel_rate;       // 记录当前轮子角速度
  }

  // 行走步态：按相位正弦摆动填充一条腿的 4 个关节，并计算关节速度
  void fillLeg(size_t offset, double phase, double ratio, bool left_side)
  {
    const double s = std::sin(phase), c = std::cos(phase), swing = std::max(0.0, s);   // 相位正/余弦，迈步阶段取正半周
    const double side = left_side ? 1.0 : -1.0;   // 左右腿髋x偏置方向
    const bool rear = offset >= 8;      // 是否为后腿（索引 8 起为后腿）
    const size_t leg_index = offset / 4; // 腿序号（0..3）
    const double thigh_stance = rear ? 0.95 : 0.82;    // 站立基准：后腿大腿更前倾
    const double calf_stance = rear ? -1.62 : -1.58;   // 站立基准：后腿小腿更屈
    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);   // 髋x 外展摆动（随相位正弦）
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;     // 髋y（大腿）摆动
    joint_msg_.position[offset + 2] = calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);   // 膝（小腿）摆动
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;   // body_pose 订阅器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;   // joint_states 发布器
  rclcpp::TimerBase::SharedPtr timer_;   // 发布定时器
  sensor_msgs::msg::JointState joint_msg_;   // 待发布的关节状态消息
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};   // 上次里程计时间
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};   // 上次发布时间
  std::array<double, 4> wheel_position_{0.0, 0.0, 0.0, 0.0};   // 四个轮子累计转角
  double horizontal_speed_{0.0}, forward_speed_{0.0}, last_odom_x_{0.0}, last_odom_y_{0.0};   // 速度与上次位置
  double gait_frequency_{2.2}, min_walk_speed_{0.05}, max_walk_speed_{1.0};   // 步态参数
  double hip_swing_{0.08}, thigh_swing_{0.32}, calf_swing_{0.42};   // 摆动幅度参数
  double wheel_radius_{0.09}, wheel_spin_rate_{0.0};   // 轮半径与轮子角速度
  bool always_trot_{false}, has_odom_{false}, has_prev_pose_{false}, has_last_publish_time_{false};   // 状态标志
};
}  // namespace scan_planner

int main(int argc, char **argv)   // 程序入口
{
  rclcpp::init(argc, argv);       // 初始化 ROS2 运行时
  rclcpp::spin(std::make_shared<scan_planner::M20GaitPublisher>());   // 创建节点并阻塞式 spin
  rclcpp::shutdown();             // 关闭 ROS2 运行时
  return 0;                       // 正常退出
}
