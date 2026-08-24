/**
 * @file open_loop_controller.cpp
 * @brief 开环轨迹控制器节点（OpenLoopController）
 *
 * 职责：
 *  - 订阅规划器发布的 B 样条轨迹（planning/bspline），
 *    按"轨迹绝对时间"开环解析当前位置/速度/加速度（不反馈里程计）；
 *  - 以固定频率（默认 100 Hz）发布机体位姿（body_pose，Odometry 消息），
 *    模拟"理想跟踪规划轨迹"的运动学结果，供下游里程计/控制器使用；
 *  - 根据速度方向估计偏航角与偏航角速度（由 v×a 叉积公式计算 yaw_rate）；
 *  - 可选保持最终位置：轨迹结束后继续停留在终点（hold_final_position）。
 *
 * 用于仿真/调试：验证规划器输出轨迹本身是否平滑可行。
 */
#include <algorithm>    // 标准算法库：std::clamp / std::max
#include <cmath>        // 数学库：std::hypot / std::atan2
#include <cstdint>      // 定宽整数类型：std::int64_t（轨迹 ID）
#include <memory>       // 智能指针：std::make_shared
#include <string>       // 字符串：std::string（坐标系名）
#include <vector>       // 容器：std::vector（轨迹及其导数）

#include <Eigen/Eigen>              // Eigen 线性代数库：向量/矩阵
#include <nav_msgs/msg/odometry.hpp>    // 里程计消息：Odometry（发布 body_pose）
#include <rclcpp/rclcpp.hpp>            // ROS2 C++ 客户端库：Node/Publisher/Timer
#include <scan_planner_msgs/msg/bspline.hpp>   // 自定义消息：Bspline（订阅规划轨迹）
#include <tf2/LinearMath/Quaternion.h>  // TF2 四元数：由 RPY 构造姿态
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // TF2 与 geometry_msgs 转换（新版本路径）
#else
// ROS 2 Foxy installs this compatibility header with the legacy suffix.
// ROS 2 Foxy 安装的是带旧后缀的兼容头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "m20_trajectory/uniform_bspline.h"   // M20 自有均匀 B 样条轨迹求值

namespace scan_planner    // 扫描规划器命名空间
{
// 开环轨迹控制器：按规划轨迹时间轴开环发布机体位姿
class OpenLoopController : public rclcpp::Node
{
public:
  // 构造函数：声明参数、创建订阅/发布/定时器
  OpenLoopController() : Node("open_loop_controller")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");        // 里程计父坐标系
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "quadruped");   // 子坐标系（机体）
    const double publish_rate = declare_parameter<double>("publish_rate", 100.0);   // 位姿发布频率 [Hz]
    yaw_min_speed_ = declare_parameter<double>("yaw_min_speed", 0.05);      // 偏航估计的最小速度阈值 [m/s]
    hold_final_position_ = declare_parameter<bool>("hold_final_position", true);    // 轨迹结束后是否保持终点
    last_yaw_ = declare_parameter<double>("init_yaw", 0.0);                 // 初始偏航角 [rad]
    current_pos_ = Eigen::Vector3d(      // 初始位置（x/y/z）
        declare_parameter<double>("init_x", 0.0),
        declare_parameter<double>("init_y", 0.0),
        declare_parameter<double>("init_z", 0.3));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 20);    // 发布机体位姿
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(        // 订阅规划 B 样条轨迹
        "planning/bspline", 10,
        std::bind(&OpenLoopController::bsplineCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(          // 周期定时器：按发布频率驱动位姿发布
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate)),
        std::bind(&OpenLoopController::publishOdom, this));
    RCLCPP_INFO(get_logger(), "Open-loop controller ready");   // 打印就绪信息
  }

private:
  // 解析 B 样条消息：校验有效性，将控制点/节点向量装入 UniformBspline
  bool parseBspline(const scan_planner_msgs::msg::Bspline::ConstSharedPtr &msg,
                    UniformBspline &pos_traj)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)   // 消息不完整或阶数非法
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");   // 忽略无效轨迹
      return false;
    }
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());   // 位置控制点矩阵（3 × N）
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      pos_pts.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;   // 逐列拷贝控制点
    Eigen::VectorXd knots(msg->knots.size());          // 节点向量
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];   // 拷贝节点值
    pos_traj = UniformBspline(pos_pts, msg->order, 0.1);   // 构造均匀 B 样条（时间间隔 0.1）
    pos_traj.setKnot(knots);                           // 设置自定义节点向量
    return true;
  }

  // B 样条订阅回调：接收新轨迹，保存位置/速度/加速度样条与轨迹元信息
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    UniformBspline pos_traj;
    if (!parseBspline(msg, pos_traj)) return;          // 解析失败则忽略
    traj_ = {pos_traj, pos_traj.getDerivative()};      // 位置 + 速度样条
    traj_.push_back(traj_[1].getDerivative());         // 加速度样条
    start_time_ = rclcpp::Time(msg->start_time);       // 记录轨迹起始时刻
    traj_id_ = msg->traj_id;                           // 记录轨迹 ID
    traj_duration_ = traj_[0].getTimeSum();            // 记录轨迹总时长
    receive_traj_ = true;                              // 标记已收到轨迹
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",   // 打印接收信息
                static_cast<long long>(traj_id_), traj_duration_);
  }

  // 发布机体位姿（Odometry）：填充位置/姿态（yaw）/线速度/偏航角速度
  void publishState(const rclcpp::Time &stamp, const Eigen::Vector3d &pos,
                    const Eigen::Vector3d &vel, double yaw, double yaw_rate)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;             // 时间戳
    odom.header.frame_id = frame_id_;      // 父坐标系
    odom.child_frame_id = child_frame_id_; // 子坐标系
    odom.pose.pose.position.x = pos.x();   // 位置 x
    odom.pose.pose.position.y = pos.y();   // 位置 y
    odom.pose.pose.position.z = pos.z();   // 位置 z
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);      // 由 RPY 构造姿态（仅偏航）
    odom.pose.pose.orientation = tf2::toMsg(quaternion);   // 四元数转消息
    odom.twist.twist.linear.x = vel.x();   // 线速度 x
    odom.twist.twist.linear.y = vel.y();   // 线速度 y
    odom.twist.twist.linear.z = vel.z();   // 线速度 z
    odom.twist.twist.angular.z = yaw_rate; // 偏航角速度
    odom_pub_->publish(odom);              // 发布位姿消息
  }

  // 定时器回调：按轨迹时间轴开环计算并发布当前位置/速度/偏航
  void publishOdom()
  {
    const auto now = this->now();          // 当前时刻
    if (!receive_traj_)                    // 尚未收到轨迹
    {
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);   // 发布静止初始状态
      return;
    }
    const double elapsed = (now - start_time_).seconds();   // 相对轨迹起始的已流逝时间
    if (elapsed < 0.0)                     // 轨迹尚未开始（时间未到）
    {
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);   // 保持初始状态
      return;
    }
    if (!hold_final_position_ && elapsed > traj_duration_) return;   // 不保持终点且轨迹已结束：停止发布
    const double t = std::clamp(elapsed, 0.0, traj_duration_);   // 时间截断到轨迹范围内
    const Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t);     // 位置样条求值
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();               // 速度（默认 0）
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();               // 加速度（默认 0）
    if (elapsed <= traj_duration_)         // 轨迹未结束才取速度/加速度
    {
      vel = traj_[1].evaluateDeBoorT(t);   // 速度样条求值
      acc = traj_[2].evaluateDeBoorT(t);   // 加速度样条求值
    }
    const double speed = std::hypot(vel.x(), vel.y());           // 水平合速度
    if (speed > yaw_min_speed_) last_yaw_ = std::atan2(vel.y(), vel.x());   // 速度足够时用速度方向更新偏航角
    const double yaw_rate = speed > yaw_min_speed_              // 偏航角速度 = (vx*ay - vy*ax) / v^2
        ? (vel.x() * acc.y() - vel.y() * acc.x()) / std::max(speed * speed, 1e-6) : 0.0;
    current_pos_ = pos;                    // 缓存当前位置
    current_vel_ = vel;                    // 缓存当前速度
    publishState(now, pos, vel, last_yaw_, yaw_rate);   // 发布机体位姿
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;    // body_pose 发布器
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;   // 轨迹订阅器
  rclcpp::TimerBase::SharedPtr timer_;    // 发布定时器
  bool receive_traj_{false};              // 是否已收到轨迹
  bool hold_final_position_{true};        // 轨迹结束后是否保持终点
  std::vector<UniformBspline> traj_;      // 轨迹及其导数：[0]位置 [1]速度 [2]加速度
  double traj_duration_{0.0};             // 当前轨迹总时长 [s]
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};   // 当前轨迹起始时刻
  std::int64_t traj_id_{0};               // 当前轨迹 ID
  std::string frame_id_;                  // 里程计父坐标系
  std::string child_frame_id_;            // 机体子坐标系
  double last_yaw_{0.0};                  // 最近估计的偏航角 [rad]
  double yaw_min_speed_{0.05};            // 偏航估计的最小速度阈值 [m/s]
  Eigen::Vector3d current_pos_{Eigen::Vector3d::Zero()};   // 当前缓存位置
  Eigen::Vector3d current_vel_{Eigen::Vector3d::Zero()};   // 当前缓存速度
};
}  // namespace scan_planner

int main(int argc, char **argv)   // 程序入口
{
  rclcpp::init(argc, argv);       // 初始化 ROS2 运行时
  rclcpp::spin(std::make_shared<scan_planner::OpenLoopController>());   // 创建节点并阻塞式 spin
  rclcpp::shutdown();             // 关闭 ROS2 运行时
  return 0;                       // 正常退出
}
