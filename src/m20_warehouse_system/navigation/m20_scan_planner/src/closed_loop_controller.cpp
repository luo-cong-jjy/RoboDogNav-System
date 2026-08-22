/**
 * @file closed_loop_controller.cpp
 * @brief 闭环轨迹控制器节点（ClosedLoopController）
 *
 * 职责：
 *  - 订阅规划器发布的 B 样条轨迹（planning/bspline）与机体里程计（body_pose），
 *    以 100 Hz 闭环跟踪轨迹，输出底盘速度指令（cmd_vel，Twist）；
 *  - 位置控制：期望速度 = 轨迹速度 + kp_pos × 位置误差（世界系），
 *    经偏航角旋转到机体系并限幅（max_vx / max_vy）；
 *  - 偏航控制：kp_yaw × 偏航误差生成角速度指令，偏航误差过大时暂停执行（冻结）；
 *  - 轨迹时钟管理：exec_time_ 随时间推进，可选与机体位置投影同步
 *    （trajectory_progress_sync，避免漂移累积）；
 *  - 双向跟踪（可选 bidirectional_tracking_enabled）：检测到需要倒车跟踪的
 *    大角度偏航误差时，以进入时刻的姿态倒车直行，带迟滞切换与最短保持时间；
 *  - 外部执行保持（可选 require_external_execution_hold）：由下游安全监督
 *    通过话题控制轨迹时钟是否推进。
 */
#include <algorithm>    // 标准算法库：std::clamp / std::min / std::max
#include <cmath>        // 数学库：std::cos / std::sin / std::atan2 / M_PI
#include <cstdint>      // 定宽整数类型：std::int64_t（轨迹 ID）
#include <memory>       // 智能指针：std::make_shared
#include <limits>       // 数值极限：std::numeric_limits（投影初始化）
#include <stdexcept>    // 异常类型：std::invalid_argument（参数校验）
#include <string>       // 字符串：std::string
#include <vector>       // 容器：std::vector（轨迹及其导数）

#include <Eigen/Eigen>              // Eigen 线性代数库：向量/矩阵
#include <geometry_msgs/msg/twist.hpp>   // 速度消息：Twist（发布 cmd_vel）
#include <nav_msgs/msg/odometry.hpp>     // 里程计消息：Odometry（订阅 body_pose）
#include <rclcpp/rclcpp.hpp>             // ROS2 C++ 客户端库：Node/Publisher/Timer
#include <scan_planner_msgs/msg/bspline.hpp>   // 自定义消息：Bspline（订阅规划轨迹）
#include <std_msgs/msg/bool.hpp>         // 布尔消息：Bool（执行冻结标志）
#include <std_msgs/msg/string.hpp>       // 字符串消息：String（跟踪方向发布）
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // TF2 与 geometry_msgs 转换（新版本路径）
#else
// ROS 2 Foxy installs this compatibility header with the legacy suffix.
// ROS 2 Foxy 安装的是带旧后缀的兼容头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2/utils.hpp>)
#include <tf2/utils.hpp>                 // TF2 工具：tf2::getYaw（提取偏航角，新版本路径）
#else
// ROS 2 Foxy installs this compatibility header with the legacy suffix.
// ROS 2 Foxy 安装的是带旧后缀的兼容头文件
#include <tf2/utils.h>
#endif

#include "bspline_opt/uniform_bspline.h"   // 均匀 B 样条：UniformBspline（轨迹求值）

namespace scan_planner    // 扫描规划器命名空间
{
// 闭环轨迹控制器：基于里程计反馈闭环跟踪规划轨迹
class ClosedLoopController : public rclcpp::Node
{
public:
  // 构造函数：声明参数、校验双向跟踪迟滞参数、创建订阅/发布/定时器
  ClosedLoopController()
  : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);   // 前瞻时间 [s]（用于期望偏航方向）
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);   // 偏航误差阈值（超限暂停）[rad]
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);     // 位置比例增益
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);     // 偏航比例增益
    max_vx_ = declare_parameter<double>("max_vx", 0.75);    // 最大前进速度 [m/s]
    max_vy_ = declare_parameter<double>("max_vy", 0.35);    // 最大侧向速度 [m/s]
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);   // 最大偏航角速度（带硬上限）
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);   // 终点判定距离 [m]
    trajectory_progress_sync_ = declare_parameter<bool>("trajectory_progress_sync", true);   // 轨迹时钟是否与机体位置投影同步
    projection_samples_ = std::max(8, static_cast<int>(declare_parameter<int>("projection_samples", 60)));   // 位置投影采样点数
    max_time_ahead_ = std::max(0.0, declare_parameter<double>("max_time_ahead", 0.20));   // 投影时间超前上限 [s]
    bidirectional_tracking_enabled_ =
      declare_parameter<bool>("bidirectional_tracking_enabled", false);   // 是否启用双向（倒车）跟踪
    reverse_tracking_enter_angle_ =
      declare_parameter<double>("reverse_tracking_enter_angle", 2.10);    // 进入倒车跟踪的偏航误差角 [rad]
    reverse_tracking_exit_angle_ =
      declare_parameter<double>("reverse_tracking_exit_angle", 1.75);     // 退出倒车跟踪的偏航误差角 [rad]
    reverse_tracking_min_hold_sec_ =
      declare_parameter<double>("reverse_tracking_min_hold_sec", 0.80);   // 倒车跟踪最短保持时间 [s]
    reverse_tracking_entry_alignment_ =
      declare_parameter<double>("reverse_tracking_entry_alignment", 0.20);   // 进入倒车所需的路径对齐角 [rad]
    reverse_tracking_exit_alignment_ =
      declare_parameter<double>("reverse_tracking_exit_alignment", 0.35);    // 退出倒车所需的路径对齐角 [rad]
    if (!(0.0 <= reverse_tracking_exit_angle_ &&              // 参数合法性校验（迟滞顺序/范围）
      reverse_tracking_exit_angle_ < reverse_tracking_enter_angle_ &&
      reverse_tracking_enter_angle_ <= M_PI &&
      reverse_tracking_min_hold_sec_ >= 0.0 &&
      0.0 < reverse_tracking_entry_alignment_ &&
      reverse_tracking_entry_alignment_ < reverse_tracking_exit_alignment_ &&
      reverse_tracking_exit_alignment_ < M_PI_2))
    {
      throw std::invalid_argument("invalid bidirectional tracking hysteresis");   // 参数非法：抛出异常终止
    }
    const auto execution_hold_topic =
      declare_parameter<std::string>("execution_hold_topic", "/m20/control/execution_hold");   // 外部执行保持话题
    require_external_execution_hold_ =
      declare_parameter<bool>("require_external_execution_hold", false);   // 是否需要外部执行保持
    external_execution_hold_ = require_external_execution_hold_;   // 初始保持状态

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(   // 订阅规划轨迹
      "planning/bspline", 10,
      std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(   // 订阅机体里程计（传感器 QoS）
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    execution_hold_sub_ = create_subscription<std_msgs::msg::Bool>(   // 订阅外部执行保持（transient_local 保留最新值）
      execution_hold_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ClosedLoopController::executionHoldCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);   // 发布底盘速度指令
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>(   // 发布执行冻结标志（供上游安全监督）
      "planning/go2_execution_frozen",
      10);
    tracking_direction_pub_ =                // 发布当前跟踪方向（FORWARD/REVERSE）
      create_publisher<std_msgs::msg::String>("planning/tracking_direction", 10);
    cmd_timer_ = create_wall_timer(          // 控制定时器：100 Hz（10 ms）
      std::chrono::milliseconds(10),
      std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();               // 初始化控制时间戳
    RCLCPP_INFO(                             // 打印就绪信息
      get_logger(),
      "Closed-loop controller ready; external execution hold=%s; "
      "bidirectional M20 tracking=%s",
      require_external_execution_hold_ ? "required" : "optional",
      bidirectional_tracking_enabled_ ? "enabled" : "disabled");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;   // 偏航角速度硬上限 [rad/s]

  // 将角度归一化到 [-π, π]
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}    // 超过上界则减一整圈
    while (angle < -M_PI) {angle += 2.0 * M_PI;}   // 低于下界则加一整圈
    return angle;                                  // 返回归一化角度
  }

  // 向量范数限幅：若范数超过 max_norm 则等比缩放到 max_norm
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d & value, double max_norm)
  {
    const double norm = value.norm();              // 计算范数
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;   // 超限则缩放，过小直接返回
  }

  // 将机体当前位置投影到轨迹上，返回对应的轨迹时刻（均匀采样最近点）
  double projectTimeToCurrentPose() const
  {
    if (traj_.empty() || traj_duration_ <= 1e-6 || !have_odom_) return 0.0;   // 无轨迹/无里程计则返回 0
    double best_t = 0.0;                            // 最优轨迹时刻
    double best_dist = std::numeric_limits<double>::infinity();   // 最近距离（初始无穷大）
    for (int i = 0; i <= projection_samples_; ++i) {   // 沿轨迹均匀采样
      const double t = traj_duration_ * static_cast<double>(i) / projection_samples_;   // 采样时刻
      const Eigen::Vector3d p = traj_[0].evaluateDeBoorT(t);   // 轨迹位置
      const double d = (p.head<2>() - odom_pos_.head<2>()).squaredNorm();   // 与机体位置的平面距离平方
      if (d < best_dist) {                         // 更近则更新最优
        best_dist = d;
        best_t = t;
      }
    }
    return best_t;                                 // 返回最近点对应时刻
  }

  // 估计期望偏航角：取 t_cur 前瞻 time_forward_ 后的轨迹方向
  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d & pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);   // 前瞻时刻（不超出轨迹）
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;  // 前瞻点与当前点的方向
    if (direction.head<2>().squaredNorm() < 1e-4) {   // 方向太短（接近终点）则改用当前速度方向
      direction = traj_[1].evaluateDeBoorT(t_cur);
    }
    return direction.head<2>().squaredNorm() < 1e-4 ?      // 仍无方向则保持当前机体偏航
           odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  // 判断倒车路径是否近似直线（采样 0/0.5/1 比例处速度方向与机体反向对齐）
  bool reversePathIsStraight() const
  {
    bool have_direction = false;                     // 是否取到过有效方向
    const double horizon = std::min(traj_duration_, 2.0 * time_forward_);   // 检测视界
    for (const double ratio : {0.0, 0.5, 1.0}) {     // 三个采样比例
      const double t = ratio * horizon;              // 采样时刻
      const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(t);   // 轨迹速度
      if (velocity.head<2>().squaredNorm() < 1e-4) { // 速度过小则跳过
        continue;
      }
      have_direction = true;                         // 取到有效方向
      const double path_yaw = std::atan2(velocity.y(), velocity.x());   // 路径方向角
      const double reverse_error = normalizeAngle(path_yaw + M_PI - odom_yaw_);   // 路径反向与机体朝向的夹角
      if (std::abs(reverse_error) > reverse_tracking_exit_alignment_) {   // 夹角超限：不是直线倒车
        return false;
      }
    }
    return have_direction;                           // 有方向且全部对齐才判定为直线
  }

  // 更新跟踪方向（FORWARD/REVERSE）：基于偏航误差与迟滞逻辑切换
  void updateTrackingDirection(double forward_yaw)
  {
    const double forward_error = normalizeAngle(forward_yaw - odom_yaw_);   // 正向偏航误差
    const double reverse_error = normalizeAngle(forward_yaw + M_PI - odom_yaw_);   // 反向偏航误差
    const bool reverse_path_is_straight = reversePathIsStraight();   // 倒车路径是否直线
    if (!bidirectional_tracking_enabled_) {          // 未启用双向跟踪
      reverse_tracking_ = false;                     // 强制正向
      reverse_tracking_hold_sec_ = 0.0;
    } else if (reverse_tracking_) {                  // 当前处于倒车跟踪
      // Direction changes are evaluated once per B-spline, not at the 100 Hz
      // control rate. This rejects a genuinely curved replan without reacting
      // to a degenerate end tangent of an otherwise straight reverse path.
      // 方向切换按每条 B 样条评估一次（而非 100 Hz 控制率），从而拒绝真正弯曲的
      // 重规划轨迹，同时避免对直线倒车路径末端退化切线的误响应。
      if (!reverse_path_is_straight) {               // 路径不再直线：退出倒车
        reverse_tracking_ = false;
        reverse_tracking_hold_sec_ = 0.0;
      } else if (std::abs(forward_error) <= reverse_tracking_exit_angle_ &&   // 偏航误差回到退出角内且保持时间足够
        reverse_tracking_hold_sec_ >= reverse_tracking_min_hold_sec_)
      {
        reverse_tracking_ = false;                   // 退出倒车跟踪
        reverse_tracking_hold_sec_ = 0.0;
      }
    } else if (reverse_path_is_straight &&           // 未倒车：路径直线且误差大到进入角、反向对齐充分
      std::abs(forward_error) >= reverse_tracking_enter_angle_ &&
      std::abs(reverse_error) <= reverse_tracking_entry_alignment_)
    {
      reverse_tracking_ = true;                      // 进入倒车跟踪
      reverse_tracking_hold_sec_ = 0.0;
      reverse_tracking_yaw_ = odom_yaw_;             // 记录进入时刻的机体朝向
    }

    const std::string direction = reverse_tracking_ ? "REVERSE" : "FORWARD";   // 当前方向字符串
    if (direction != last_tracking_direction_) {     // 方向发生变化
      std_msgs::msg::String message;
      message.data = direction;                      // 填充方向消息
      tracking_direction_pub_->publish(message);     // 发布方向变化
      RCLCPP_INFO(get_logger(), "Trajectory tracking direction: %s", direction.c_str());   // 打印方向变化
      last_tracking_direction_ = direction;          // 记录上次方向
    }
  }

  // 计算实际跟踪偏航角：倒车时固定为进入时刻朝向，正向时用轨迹前瞻方向
  double trackingYaw(double forward_yaw) const
  {
    // Validated reverse execution is straight rolling. Keep the body heading
    // captured on entry instead of following noisy/degenerate endpoint
    // tangents from a completed B-spline.
    // 经过校验的倒车执行是直线滚动：保持进入倒车时捕获的机体朝向，
    // 而不是跟随已完成 B 样条的噪声/退化末端切线。
    return reverse_tracking_ ? reverse_tracking_yaw_ : forward_yaw;
  }

  // 发布停车指令：线速度置 0，角速度可选（默认 0）
  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);   // 角速度限幅
    cmd_vel_pub_->publish(cmd);              // 发布停车指令
  }

  // 发布执行冻结标志
  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;                       // 填充冻结状态
    execution_frozen_pub_->publish(msg);     // 发布标志
  }

  // B 样条订阅回调：解析新轨迹，初始化轨迹时钟（可选投影同步），评估跟踪方向
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0) {   // 消息不完整或阶数非法
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");   // 忽略无效轨迹
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());   // 位置控制点矩阵（3 × N）
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;   // 逐列拷贝控制点
    }
    Eigen::VectorXd knots(msg->knots.size());          // 节点向量
    for (size_t i = 0; i < msg->knots.size(); ++i) {knots(i) = msg->knots[i];}   // 拷贝节点值
    UniformBspline position(points, msg->order, 0.1);  // 构造均匀 B 样条
    position.setKnot(knots);                           // 设置自定义节点向量
    traj_ = {position, position.getDerivative()};      // 位置 + 速度样条
    traj_.push_back(traj_[1].getDerivative());         // 加速度样条
    traj_duration_ = traj_[0].getTimeSum();            // 轨迹总时长
    traj_id_ = msg->traj_id;                           // 轨迹 ID
    exec_time_ = trajectory_progress_sync_ ? projectTimeToCurrentPose() : 0.0;   // 初始化轨迹时钟（投影同步或从 0 开始）
    last_update_time_ = now();                         // 更新时间戳
    receive_traj_ = true;                              // 标记已收到轨迹
    const Eigen::Vector3d initial_position = traj_[0].evaluateDeBoorT(0.0);   // 轨迹起点
    updateTrackingDirection(estimateDesiredYaw(0.0, initial_position));   // 评估跟踪方向
    RCLCPP_INFO(                                       // 打印接收信息
      get_logger(), "Received trajectory %lld, duration %.3fs",
      static_cast<long long>(traj_id_), traj_duration_);
  }

  // 里程计回调：缓存位置与偏航角
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;   // 缓存位置
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);   // 提取偏航角
    have_odom_ = true;                       // 标记已收到里程计
  }

  // 外部执行保持回调：更新保持状态
  void executionHoldCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (!require_external_execution_hold_) { // 不要求外部保持：始终视为未保持
      external_execution_hold_ = false;
      return;
    }
    if (external_execution_hold_ != msg->data) {   // 状态变化时打印
      RCLCPP_INFO(
        get_logger(), "External trajectory hold %s",
        msg->data ? "asserted" : "released");
    }
    external_execution_hold_ = msg->data;    // 更新保持状态
  }

  // 控制定时器回调：闭环计算并发布速度指令
  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_) {     // 尚无轨迹或里程计
      publishExecutionFrozen(external_execution_hold_);   // 发布当前冻结状态
      publishStop();                         // 发布停车指令
      return;
    }
    const auto current_time = now();         // 当前时刻
    double dt = (current_time - last_update_time_).seconds();   // 控制周期
    if (dt < 0.0 || dt > 0.2) {dt = 0.0;}    // 周期非法/过大则置 0（暂停恢复保护）
    const double t_eval = std::min(exec_time_, traj_duration_);   // 轨迹求值时刻（截断）
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);   // 期望位置
    const double forward_yaw = estimateDesiredYaw(t_eval, pos_des);   // 期望偏航角
    reverse_tracking_hold_sec_ +=             // 倒车跟踪保持时间累加
      reverse_tracking_ ? std::max(0.0, dt) : 0.0;
    const double tracking_yaw = trackingYaw(forward_yaw);   // 实际跟踪偏航角（倒车/正向）
    const double yaw_error = normalizeAngle(tracking_yaw - odom_yaw_);   // 偏航误差
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);   // P 控制：偏航角速度指令
    if (std::abs(yaw_error) > heading_error_threshold_) {   // 偏航误差过大：暂停执行先转身
      publishExecutionFrozen(true);          // 发布冻结
      publishStop(yaw_command);              // 只发转角指令（不前进）
      last_update_time_ = current_time;
      return;
    }

    // A downstream safety hold blocks execution but must not erase the
    // upstream candidate command: collision prediction needs to keep checking
    // the motion that would execute after release. Freeze only the trajectory
    // clock here; the safety supervisor remains the sole zero-command gate.
    // 下游安全保持会阻止执行，但不能抹掉上游候选指令：碰撞预测需要持续检查
    // 释放后将要执行的运动。这里只冻结轨迹时钟；安全监督仍是唯一的下发零指令闸门。
    publishExecutionFrozen(external_execution_hold_);   // 发布冻结状态（供安全监督）
    if (!external_execution_hold_) {         // 未处于保持：推进轨迹时钟
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);   // 随时间推进
      if (trajectory_progress_sync_) {       // 启用位置投影同步
        const double projected_time = projectTimeToCurrentPose();   // 机体在轨迹上的投影时刻
        exec_time_ = std::min(exec_time_, projected_time + max_time_ahead_);   // 防止超前投影过多
      }
    }
    last_update_time_ = current_time;        // 更新时间戳
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);   // 期望位置（按轨迹时钟）
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);   // 期望速度
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());   // 平面位置误差
    const Eigen::Vector2d vel_world = clampNorm(       // 期望世界系速度（轨迹速度 + P×位置误差），限幅
      Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
      std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);    // 机体偏航角余弦
    const double s = std::sin(odom_yaw_);    // 机体偏航角正弦
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);   // 坐标变换：世界系 -> 机体系 x，限幅
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);   // 坐标变换：世界系 -> 机体系 y，限幅
    command.angular.z = yaw_command;         // 偏航角速度指令
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_) {   // 轨迹结束且已到终点
      command = geometry_msgs::msg::Twist(); // 下发全零指令（停车）
    }
    cmd_vel_pub_->publish(command);          // 发布速度指令
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;   // cmd_vel 发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;   // 执行冻结标志发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_direction_pub_;   // 跟踪方向发布器
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;   // 轨迹订阅器
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;   // 里程计订阅器
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_hold_sub_;   // 外部保持订阅器
  rclcpp::TimerBase::SharedPtr cmd_timer_;   // 控制定时器
  bool receive_traj_{false};                 // 是否已收到轨迹
  bool have_odom_{false};                    // 是否已收到里程计
  std::vector<UniformBspline> traj_;         // 轨迹及其导数：[0]位置 [1]速度 [2]加速度
  double traj_duration_{0.0};                // 当前轨迹总时长 [s]
  std::int64_t traj_id_{0};                  // 当前轨迹 ID
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};   // 机体位置（里程计）
  double odom_yaw_{0.0};                     // 机体偏航角（里程计）
  double exec_time_{0.0};                    // 轨迹时钟（当前执行时刻）[s]
  bool require_external_execution_hold_{false};   // 是否要求外部执行保持
  bool external_execution_hold_{false};      // 当前外部保持状态
  bool bidirectional_tracking_enabled_{false};    // 是否启用双向跟踪
  bool reverse_tracking_{false};             // 当前是否处于倒车跟踪
  double reverse_tracking_hold_sec_{0.0};    // 倒车跟踪已保持时长 [s]
  double reverse_tracking_yaw_{0.0};         // 进入倒车时刻的机体朝向 [rad]
  std::string last_tracking_direction_;      // 上次跟踪方向字符串
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};   // 上次控制时刻
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;   // 前瞻时间/偏航阈值/P 增益
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;   // 速度限幅与终点距离
  bool trajectory_progress_sync_{true};      // 轨迹时钟是否与位置投影同步
  int projection_samples_{60};               // 位置投影采样点数
  double max_time_ahead_{0.20};              // 投影时间超前上限 [s]
  double reverse_tracking_enter_angle_, reverse_tracking_exit_angle_;   // 倒车进入/退出角度 [rad]
  double reverse_tracking_min_hold_sec_;     // 倒车最短保持时间 [s]
  double reverse_tracking_entry_alignment_, reverse_tracking_exit_alignment_;   // 倒车进入/退出对齐角 [rad]
};
}  // namespace scan_planner

int main(int argc, char ** argv)   // 程序入口
{
  rclcpp::init(argc, argv);        // 初始化 ROS2 运行时
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());   // 创建节点并阻塞式 spin
  rclcpp::shutdown();              // 关闭 ROS2 运行时
  return 0;                        // 正常退出
}
